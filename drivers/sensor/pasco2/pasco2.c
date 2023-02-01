/*  Infineon PAS CO2 sensor driver
 *	
 * Copyright (c) 2023 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <drivers/sensor.h>
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

LOG_MODULE_REGISTER(PASCO2, CONFIG_SENSOR_LOG_LEVEL);

static int pasco2_get_reg(const struct device *dev)
{
	struct pasco2_data * data = dev->data;
	uint8_t chip_id;
	int ret;

	ret = data->hw_tf->read_reg(dev, PASCO2_SENS_RST, &chip_id);
	if (ret==0) 
	{
		printk("%s: Failed to read %s: %d", dev->name, "chip id", ret);
		return ret;
	}
	else{
		printk("%s: read %s: %d", dev->name, "chip id", ret);
	}
};

static int pasco2_write_command(const struct device *dev, uint8_t cmd)
{
	const struct pasco2_config *config  = dev->config;
	uint8_t tx_buf[2];//= { cmd };
	tx_buf[0] = PASCO2_SCRATCH_PAD;
	tx_buf[1] = cmd;
	return i2c_write_dt(&config->bus, tx_buf, sizeof(tx_buf));
}


static int pasco2_read_sample(const struct device *dev,
		uint8_t *co2ppm_sample)
{
	const struct pasco2_config *cfg = dev->config;
	uint8_t rx_buf[6];
	int rc;

	rc = i2c_read_dt(&cfg->bus, rx_buf, sizeof(rx_buf));

	if (rc < 0) {
		LOG_ERR("Failed to read data from device.");
		return rc;
	}

	*co2ppm_sample = sys_get_be16(rx_buf);
	
	return 0;
}

int pasco2_write_reg(const struct device *dev, uint8_t cmd, uint8_t val)
{
	const struct pasco2_config *config = dev->config;
	uint8_t tx_buf[2];
	uint8_t rx_buf;

	tx_buf[0] = PASCO2_SENS_RST;
	tx_buf[1] = val;
	sys_put_be16(cmd, &tx_buf);
	
	// if (i2c_write_read_dt(&config->bus, cmd, sizeof(cmd),
	// 		      rx_buf, sizeof(rx_buf)) < 0) 
	// 	{
	// 			printk("Failed to read data sample!\r\n");
	// 	}
    // else{
	// 	printk("read data sample : 0x%.2x\r\n",val);

	// }
return rx_buf;
}
	
static int pasco2_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	const struct pasco2_config *config = dev->config;
	struct pasco2_data *data = dev->data;
	int rc;

	if (chan != SENSOR_CHAN_ALL &&
		chan != SENSOR_CHAN_CO2) {
		return -ENOTSUP;
	}
		rc = pasco2_write_command(dev, PASCO2_SENS_RST);
	if (rc < 0) {
		printk("Failed to start measurement.");
		return rc;
	}
	
		rc = pasco2_read_sample(dev, &data->co2ppm_sample);
	if (rc < 0) {
			LOG_ERR("Failed to fetch data.");
			printk("Failed to fetch data.");
			return rc;
		}
	// uint8_t rx_buf[6];
	// uint8_t co2ppm_sample;

	// __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);
	// 	co2ppm_sample = sys_get_be16(&rx_buf[2]);
	// 	data->co2ppm_sample = co2ppm_sample;
	// 		return 0;

}
static int pasco2_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	const struct pasco2_data *data = dev->data;
	uint8_t co2ppm;
	if (chan == SENSOR_CHAN_CO2) 
	{
	co2ppm = (uint8_t)data->co2ppm_sample ;
	val->val1 = (uint8_t)(co2ppm/1) ;
	}

	else
	{
		return -ENOTSUP;
	}		return 0;

}


static int pasco2_init(const struct device *dev)
{
	const struct pasco2_config *cfg = dev->config;
	struct pasco2_data *data = dev->data;
	uint8_t chip_id;
	uint8_t val;
	int ret;

	int rc;

	ret = cfg->bus_init(dev);
	if (ret) {
		return ret;
	}

	// ret = data->hw_tf->write_reg(dev, PASCO2_SENS_RST, 0x23);
	ret = data->hw_tf->read_reg(dev, PASCO2_PROD_ID, &chip_id);
	// ret = data->hw_tf->read_reg(dev, PASCO2_PROD_ID, &chip_id);
	// 
	printk("\n%s: read: 0x%.2x", "chip id", chip_id);

	ret = data->hw_tf->write_reg(dev, PASCO2_SCRATCH_PAD, 0xA5);
	ret = data->hw_tf->read_reg(dev, PASCO2_SCRATCH_PAD, &chip_id);
	printk("\n%s: read: 0x%.2x", "chip id", chip_id);
	// if (ret) {
	// 	LOG_DBG("%s: Failed to read %s: %d", dev->name, "chip id", ret);
	// 	return ret;
	// }
	
	
	(void)k_msleep(PASCO2_CLEAR_STATUS_WAIT_USEC);



	// struct pasco2_data *data = dev->data;

		printk("\nPROD ID address is 0x%.2x \n",PASCO2_PROD_ID);

	if (!device_is_ready(cfg->bus.bus)) {
			LOG_ERR("I2C bus %s is not ready!", cfg->bus.bus->name);
			return -EINVAL;
		}
		
	/*get the address of the i2c of the sensor 0x28*/
	else {
		printk("\nI2C register is 0x%.2x \n", cfg->bus.addr);
		// printk("\nPROD ID address is 0x%.2x \n",PASCO2_PROD_ID);

		}
		
		/* clear status register */
	// rc = pasco2_write_command(dev, PASCO2_SENS_RST);


		if (rc < 0) {
			LOG_DBG("Failed to clear status register!");
			return rc;
		}
		else{
		// ret = data->hw_tf->read_reg(dev, PASCO2_SENS_RST, &chip_id);
			printk("Status Register = 0x%.2x\r\n",rc);
		}	
		
	k_busy_wait(PASCO2_CLEAR_STATUS_WAIT_USEC);
 	
	return 0;
}

static const struct sensor_driver_api pasco2_driver_api ={

    .sample_fetch = pasco2_sample_fetch,
	.channel_get = pasco2_channel_get,

};

#define  PASCO2_DEFINE(inst)											\
	static struct  pasco2_data  pasco20_data_##inst;					\
	static const struct  pasco2_config  pasco20_cfg_##inst = {			\
		.bus = I2C_DT_SPEC_INST_GET(inst),								\
		.bus_init = pasco2_i2c_init										\
		};																\
	DEVICE_DT_INST_DEFINE(inst,  pasco2_init, NULL,						\
		&pasco20_data_##inst, &pasco20_cfg_##inst,						\
		POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,						\
		&pasco2_driver_api);

DT_INST_FOREACH_STATUS_OKAY( PASCO2_DEFINE)