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

#include "pasco2.h"

LOG_MODULE_REGISTER(PASCO2, CONFIG_SENSOR_LOG_LEVEL);

// static int pasco2_read_mode(const struct device *dev)
// {
// 	const struct pasco2_config *cfg = dev->config;
// 	uint8_t chip_id
// 	int ret;

// 	ret = data->hw_tf->read_reg(dev, PASCO2_PROD_ID, &chip_id);
// 		if (ret) {
// 			LOG_DBG("%s: Failed to read %s: %d", dev->name, "chip id", ret);
// 			return ret;
// 		}

// 		if (chip_id != PASCO2_PROD_ID) {
// 			LOG_DBG("%s: Invalid chip id 0x%x", dev->name, chip_id);
// 			return -EIO;
// 		}
// 		return ret;
// 	}

static int pasco2_write_command(const struct device *dev, uint8_t cmd)
{
	const struct pasco2_config *cfg = dev->config;
	uint8_t tx_buf[1] = { cmd };

	return i2c_write_dt(&cfg->bus, tx_buf, sizeof(tx_buf));
}
int pasco2_write_reg(const struct device *dev, uint16_t cmd, uint16_t val)
{
	const struct pasco2_config *config = dev->config;
	uint8_t tx_buf[5];

	sys_put_be16(cmd, &tx_buf[0]);
	sys_put_be16(val, &tx_buf[2]);

	return i2c_write_dt(&config->bus, tx_buf, sizeof(tx_buf));
}

// static int pasco2_read_sample(const struct device *dev,
// 		uint16_t *co2ppm_sample)
// {
// 	const struct pasco2_config *cfg = dev->config;
// 	uint8_t rx_buf[6];
// 	int rc;

// 	rc = i2c_read_dt(&cfg->bus, rx_buf, sizeof(rx_buf));
// 	if (rc < 0) {
// 		LOG_ERR("Failed to read data from device.");
// 		return rc;
// 	}
// 		*co2ppm_sample = sys_get_be16(rx_buf);
	
// 	return 0;
// }

static int pasco2_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	const struct pasco2_config *config = dev->config;
	struct pasco2_data *data = dev->data;
	uint8_t rx_buf[2];
	uint16_t co2ppm_sample;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	// if (i2c_read_dt(&config->bus, rx_buf, sizeof(rx_buf)) < 0) {
	// 	LOG_DBG("Failed to read data sample!");
	// 	return -EIO;
	// }
	// co2ppm_sample = sys_get_be16(&rx_buf[0]);
	data->co2ppm_sample = co2ppm_sample;

	// rc = pasco2_write_command(dev, measure_cmd[cfg->repeatability]);
	// if (rc < 0) {
	// 	LOG_ERR("Failed to start measurement.");
	// 	return rc;
	// }

	// k_sleep(K_USEC(measure_wait_us[cfg->repeatability]));

	// pasco2_read_sample(dev, &data->co2ppm_sample, &data->co2ppm_sample);
	// if (rc < 0) {
	// 	LOG_ERR("Failed to fetch data.");
	// 	return rc;
	// }

	return 0;
}

static int pasco2_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	const struct pasco2_data *data = dev->data;
	uint64_t co2ppm;

	
	if (chan == SENSOR_CHAN_CO2) {
	
	co2ppm = (uint64_t)data->co2ppm_sample * 1;
	val->val1 = (int32_t)(co2ppm * 1000) ;
	val->val2 = co2ppm/1000;
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

#define  PASCO2_DEFINE(inst)												\
	struct  pasco2_data  pasco20_data_##inst;							\
	static const struct  pasco2_config  pasco20_cfg_##inst = {			\
		.bus = I2C_DT_SPEC_INST_GET(inst)								\
		};																\
	DEVICE_DT_INST_DEFINE(inst,  pasco2_init, NULL,						\
		&pasco20_data_##inst, &pasco20_cfg_##inst,					\
		POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,						\
		&pasco2_driver_api);

DT_INST_FOREACH_STATUS_OKAY( PASCO2_DEFINE)