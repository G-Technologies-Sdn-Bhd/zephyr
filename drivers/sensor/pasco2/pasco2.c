/*  Infineon PAS CO2 sensor driver
 *	11.41
 * Copyright (c) 2023 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT  infineon_pasco2

#include <drivers/sensor.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <string.h>

#include "pasco2.h"
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static int pasco2_i2c_read_data(const struct device *dev, uint8_t reg_addr, uint8_t *value,
			       uint8_t len)
{
	const struct pasco2_config *cfg = dev->config;

	return i2c_burst_read_dt(&cfg->bus, reg_addr, value, len);
}

static int pasco2_i2c_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *value)
{
	const struct pasco2_config *cfg = dev->config;

	return i2c_reg_read_byte_dt(&cfg->bus, reg_addr, value);
}

static const struct pasco2_transfer_function pasco2_i2c_transfer_fn = {
	.read_data = pasco2_i2c_read_data,
	.read_reg = pasco2_i2c_read_reg,
	
};

int pasco2_i2c_init(const struct device *dev)
{
	struct pasco2_data *data = dev->data;
	const struct pasco2_config *cfg = dev->config;

	data->hw_tf = &pasco2_i2c_transfer_fn;

	if (!device_is_ready(cfg->bus.bus)) {
		return -ENODEV;
	}

	return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */

LOG_MODULE_REGISTER(PASCO2, CONFIG_SENSOR_LOG_LEVEL);

static int pasco2_write_command(const struct device *dev, uint8_t cmd)
{
	const struct pasco2_config *cfg = dev->config;
	uint8_t tx_buf[2] = { cmd };

	return i2c_write_dt(&cfg->bus, tx_buf, sizeof(tx_buf));
}
int pasco2_write_reg(const struct device *dev, uint16_t cmd, uint16_t val)
{
	const struct pasco2_config *config = dev->config;
	uint8_t tx_buf[6];

	sys_put_be16(cmd, &tx_buf[0]);
	// sys_put_be16(val, &tx_buf[2]);

	return i2c_write_dt(&config->bus, tx_buf, sizeof(tx_buf));
}

static int pasco2_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	// const struct pasco2_config *config = dev->config;
	struct pasco2_data *data = dev->data;
	uint8_t rx_buf[6];
	uint8_t co2ppm_sample;
	
	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);
	co2ppm_sample = sys_get_be16(&rx_buf[2]);
	data->co2ppm_sample = co2ppm_sample;
	// LOG_INF("CO2; %u",data->co2ppm_sample);
	return 0;
	
}

static int pasco2_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	// const struct pasco2_config *config = dev->config;
	struct pasco2_data *data = dev->data;
	uint8_t co2ppm;

	
	if (chan == SENSOR_CHAN_CO2) {
	
	co2ppm = (uint8_t)data->co2ppm_sample ;
	val->val1 = (uint8_t)(co2ppm) ;
	}
	else {
		return -ENOTSUP;
	}

	return 0;
}

static int pasco2_init(const struct device *dev)
{
	const struct pasco2_config *cfg = dev->config;

	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C bus %s is not ready!", cfg->bus.bus->name);
		return -EINVAL;
	}

	if (pasco2_write_command(dev, PASCO2_SENS_RST) < 0) {
		LOG_DBG("Failed to clear status register!");
		return -EIO;
	}

	k_sleep(K_MSEC(PASCO2_CLEAR_STATUS_WAIT_USEC));

	return 0;
}

static const struct sensor_driver_api pasco2_driver_api ={

    .sample_fetch = pasco2_sample_fetch,
	.channel_get = pasco2_channel_get,

};

#define  PASCO2_DEFINE(inst)											\
	struct  pasco2_data  pasco20_data_##inst;							\
	static const struct  pasco2_config  pasco20_cfg_##inst = {			\
		.bus = I2C_DT_SPEC_INST_GET(inst)								\
		};																\
	DEVICE_DT_INST_DEFINE(inst,  pasco2_init, NULL,						\
		&pasco20_data_##inst, &pasco20_cfg_##inst,						\
		POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,						\
		&pasco2_driver_api);

DT_INST_FOREACH_STATUS_OKAY( PASCO2_DEFINE)