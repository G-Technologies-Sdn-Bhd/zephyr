/*  Infineon PAS CO2 sensor driver
 *
 * Copyright (c) 2023 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <drivers/sensor.h>
#include <drivers/sensor/pasco2.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <string.h>
#include <drivers/i2c.h>
#define DT_DRV_COMPAT infineon_pasco2

#include "pasco2.h"

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#error "PASCO2 driver enabled without any devices"
#endif

LOG_MODULE_REGISTER(PASCO2, CONFIG_SENSOR_LOG_LEVEL_ERR);

int pasco2_command(const struct device *dev, enum comman_type cmd, uint8_t reg_addr, uint8_t *val)
{
	struct pasco2_data *data = dev->data;
	int ret = -1;

	switch ((int)cmd) {
	case single_read:
		return data->hw_tf->read_reg(dev, reg_addr, val);
		break;
	case single_write:
		return data->hw_tf->write_reg(dev, reg_addr, val);
		break;
	default:
		LOG_ERR("Command Not Found");
		break;
	}
	return ret;
}

int pressure_reff_setup(const struct device *dev)
{
	int ret;
	const struct pasco2_config *cfg = dev->config;
	uint8_t tx_buff[2];
	sys_put_be16(cfg->pressure_ref, &tx_buff);

	ret = pasco2_command(dev, single_write, PASCO2_PRESS_REF_H, tx_buff[0]);

	ret = pasco2_command(dev, single_write, PASCO2_PRESS_REF_L, tx_buff[1]);
	return ret;
}

int calibration_setup(const struct device *dev)
{
	int ret;
	const struct pasco2_config *cfg = dev->config;
	uint8_t tx_buff[2];
	sys_put_be16(cfg->calibration_ref, &tx_buff);

	ret = pasco2_command(dev, single_write, PASCO2_CALIB_RED_H, tx_buff[0]);

	ret = pasco2_command(dev, single_write, PASCO2_CALIB_REF_L, tx_buff[1]);
	return ret;
}
int measurement_rate(const struct device *dev)
{
	int ret;
	const struct pasco2_config *cfg = dev->config;
	uint8_t tx_buff[2];
	sys_put_be16(cfg->sampling_rate, &tx_buff);
	ret = pasco2_command(dev, single_write, PASCO2_MEAS_RATE_H, tx_buff[0]);

	ret = pasco2_command(dev, single_write, PASCO2_MEAS_RATE_L, tx_buff[1]);
	return ret;
}
static int pasco2_get_co2(const struct device *dev)
{
	int ret;
	uint8_t rd_buff[2];
	struct pasco2_data *data = dev->data;

	ret = data->hw_tf->read_data(dev, PASCO2_CO2PPM_H, rd_buff, sizeof(rd_buff));
	int16_t result = sys_get_be16(&rd_buff);
	printk("CO2: %d ppm\r\n", result);
	data->co2ppm_sample = result;
	printk("Got new CO2 data: %d ppm \r\n", result);
}

static int pasco2_continuous_mode(const struct device *dev)
{
	int ret;
	const struct pasco2_config *config = dev->config;
	struct pasco2_data *data = dev->data;
	pasco2_measurement_config_t meas_config;
	uint8_t cfg = 0x06;
	uint8_t rd_buff[2];

#ifdef CONFIG_PASCO2_TRIGGER
	pasco2_get_co2(dev);
	return 0;
#else
	pasco2_command(dev, single_read, PASCO2_MEAS_STS, &meas_config.u);
	printk("meas_config %x\r\n", meas_config.u);
	if (meas_config.u == 0x10) {
		pasco2_get_co2(dev);
		return 0;
	} else {
		return -EACCES;
	}
#endif
}
int pasco2_single_shot_mode(const struct device *dev)
{
	int ret;
	const struct pasco2_config *config = dev->config;
	struct pasco2_data *data = dev->data;
	pasco2_measurement_config_t meas_config;
	uint8_t cfg = 0x05;
	uint8_t rd_buff[2] = { 0 };
	ret = pasco2_command(dev, single_write, PASCO2_MEAS_CFG, cfg);
	k_sleep(K_SECONDS(2));

	pasco2_get_co2(dev);
	return 0;
}

static int pasco2_standy_mode(const struct device *dev)
{
	return pasco2_command(dev, single_write, PASCO2_MEAS_CFG, PASCO2_STANDY_MODE);
}
static int pasco2_sample_rate_cfg(const struct device *dev, uint16_t val)
{
	int ret;
	uint8_t tx_buff[2];
	sys_put_be16(val, &tx_buff);
	ret = pasco2_command(dev, single_write, PASCO2_MEAS_RATE_H, tx_buff[0]);

	return pasco2_command(dev, single_write, PASCO2_MEAS_RATE_L, tx_buff[1]);
}

static int pasco2_calibration_cfg(const struct device *dev, uint16_t val)
{
	int ret;
	uint8_t tx_buff[2];
	sys_put_be16(val, &tx_buff);

	ret = pasco2_command(dev, single_write, PASCO2_CALIB_RED_H, tx_buff[0]);

	return pasco2_command(dev, single_write, PASCO2_CALIB_REF_L, tx_buff[1]);
}
static int pasco2_pressure_cfg(const struct device *dev, uint16_t val)
{
	int ret;
	uint8_t tx_buff[2];
	sys_put_be16(val, &tx_buff);

	ret = pasco2_command(dev, single_write, PASCO2_PRESS_REF_H, tx_buff[0]);

	return pasco2_command(dev, single_write, PASCO2_PRESS_REF_L, tx_buff[1]);
}

static int pasco2_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	int ret;
	if ((int)chan != SENSOR_CHAN_CO2) {
		LOG_ERR("Invalid Channel support only %s", "SENSOR_CHAN_CO2");
		return -EINVAL;
	}
	pasco2_standy_mode(dev);
	uint16_t dval = val->val1;

	switch ((int)attr) {
	case SENSOR_ATTR_PASCO2_SAMPLE_RATE:
		ret = pasco2_sample_rate_cfg(dev, dval);
		break;
	case SENSOR_ATTR_PASCO2_CALIB_REF:
		ret = pasco2_calibration_cfg(dev, dval);
		break;
	case SENSOR_ATTR_PASCO2_PRESSURE_REF:
		ret = pasco2_pressure_cfg(dev, dval);
		break;
	default:
		break;
	}
#if IS_ENABLED(CONFIG_PASCO2_PERIODIC_MODE)
	ret = pasco2_command(dev, single_write, PASCO2_MEAS_CFG, 0x06);
#endif
	ret = pasco2_command(dev, single_write, PASCO2_MEAS_CFG, 0x01);
	return ret;
}
static int pasco2_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct pasco2_data *data = dev->data;

	int rc;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_CO2) {
		return -ENOTSUP;
	}
#if IS_ENABLED(CONFIG_PASCO2_PERIODIC_MODE)
	rc = pasco2_continuous_mode(dev);
#else
	rc = pasco2_single_shot_mode(dev);
#endif

	if (rc < 0) {
		printk("No new data.\r\n");
		return 0;
	}

	return 0;
}
static int pasco2_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	const struct pasco2_data *data = dev->data;

	if (chan == SENSOR_CHAN_CO2) {
		val->val1 = (uint16_t)data->co2ppm_sample;
		return 0;
	}

	else {
		return -ENOTSUP;
	}
}
static int pasco2_init(const struct device *dev)
{
	const struct pasco2_config *cfg = dev->config;

	int ret;
	/*get the address of the i2c of the sensor 0x28*/
	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C bus %s is not ready!", cfg->bus.bus->name);
		return -EINVAL;
	} else {
		printk("\nI2C register is 0x%.2x \n", cfg->bus.addr);
	}

	ret = cfg->bus_init(dev);
	if (ret) {
		return ret;
	}
	k_busy_wait(200);
	uint8_t rx;
	pasco2_standy_mode(dev);

	pasco2_command(dev, single_write, PASCO2_SCRATCH_PAD, PASCO2_COMM_TEST_VAL);
	k_msleep(200);
	pasco2_command(dev, single_read, PASCO2_SCRATCH_PAD, &rx);

	pasco2_command(dev, single_write, PASCO2_SENS_RST, PASCO2_CMD_SOFT_RESET);

	pasco2_command(dev, single_read, PASCO2_CO2PPM_L, &rx);

	k_msleep(PASCO2_RESET_DELAY);
	pasco2_command(dev, single_read, PASCO2_MEAS_CFG, &rx);

	pasco2_command(dev, single_read, PASCO2_SENS_STS, &rx);

	k_msleep(1000);
#ifdef CONFIG_PASCO2_TRIGGER
	pasco2_trigger_init(dev);
#endif
	k_sleep(K_SECONDS(4));
	pressure_reff_setup(dev);

	calibration_setup(dev);

	measurement_rate(dev);

#if IS_ENABLED(CONFIG_PASCO2_PERIODIC_MODE)
	pasco2_command(dev, single_write, PASCO2_MEAS_CFG, 0x06);
#endif
	printk("Sensor status: %x\n", rx);

	if (rx & PASCO2_REG_SENS_STS_ICCER_MSK) {
		ret = PASCO2_ICCERR;
	} else if (rx & PASCO2_REG_SENS_STS_ORVS_MSK) {
		ret = PASCO2_ORVS;
	} else if (rx & PASCO2_REG_SENS_STS_ORTMP_MSK) {
		ret = PASCO2_ORTMP;
	} else if (!(rx & PASCO2_REG_SENS_STS_SEN_RDY_MSK)) {
		ret = PASCO2_ERR_NOT_READY;
	} else {
		ret = 0;
	}
	printk("Sensor status ready? =%d\r\n", ret);
	return ret;
}

static const struct sensor_driver_api pasco2_driver_api = {
	.attr_set = pasco2_attr_set,
#ifdef CONFIG_PASCO2_TRIGGER
	.trigger_set = pasco2_trigger_set,
#endif
	.sample_fetch = pasco2_sample_fetch,
	.channel_get = pasco2_channel_get,

};

#ifdef CONFIG_PASCO2_TRIGGER
#define PASCO2_CFG_IRQ(inst)                                                                       \
	.gpio_int = GPIO_DT_SPEC_INST_GET(inst, int_gpios), .int_pin = DT_INST_PROP(inst, int_pin),
#else
#define PASCO2_CFG_IRQ(inst)
#endif /* CONFIG_KX022_TRIGGER */

#define PASCO2_DEFINE(inst)                                                                        \
	static struct pasco2_data pasco20_data_##inst;                                             \
	static const struct pasco2_config pasco20_cfg_##inst = {                                   \
		.bus_init = pasco2_i2c_init,                                                       \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.sampling_rate = DT_INST_PROP(inst, sampling_rate),                                \
		.calibration_ref = DT_INST_PROP(inst, calibration_ref),                            \
		.pressure_ref = DT_INST_PROP(inst, pressure_ref),                                  \
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, int_gpios), (PASCO2_CFG_IRQ(inst)), ())    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, pasco2_init, NULL, &pasco20_data_##inst, &pasco20_cfg_##inst,  \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &pasco2_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PASCO2_DEFINE)
