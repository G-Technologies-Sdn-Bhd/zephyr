/*  Infineon PAS CO2 sensor driver
 *	
 * Copyright (c) 2023 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pasco2.h"

#define DT_DRV_COMPAT infineon_pasco2

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static int pasco2_i2c_read_data(const struct device *dev, uint8_t reg_addr, uint8_t *value,
			       uint8_t len)
{
	const struct pasco2_config *cfg = dev->config;

	return i2c_burst_read_dt(&cfg->bus, reg_addr, value, len);
}

static int pasco2_i2c_write_data(const struct device *dev, uint8_t reg_addr, uint8_t *value,
				uint8_t len)
{
	const struct pasco2_config *cfg = dev->config;

	return i2c_burst_write_dt(&cfg->bus, reg_addr, value, len);
}

static int pasco2_i2c_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *value)
{
	const struct pasco2_config *cfg = dev->config;

	return i2c_reg_read_byte_dt(&cfg->bus, reg_addr, value);
}

static int pasco2_i2c_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t value)
{
	const struct pasco2_config *cfg = dev->config;

	return i2c_reg_write_byte_dt(&cfg->bus, reg_addr, value);
}

static int pasco2_i2c_update_reg(const struct device *dev, uint8_t reg_addr, uint8_t mask,
				uint8_t value)
{
	const struct pasco2_config *cfg = dev->config;

	return i2c_reg_update_byte_dt(&cfg->bus, reg_addr, mask, value);
}

static const struct pasco2_transfer_function pasco2_i2c_transfer_fn = {
	.read_data = pasco2_i2c_read_data,
	.write_data = pasco2_i2c_write_data,
	.read_reg = pasco2_i2c_read_reg,
	.write_reg = pasco2_i2c_write_reg,
	.update_reg = pasco2_i2c_update_reg,
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