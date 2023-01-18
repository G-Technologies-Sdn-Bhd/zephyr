/* Sensirion PAS CO2 sensor driver
 *	
 * Copyright (c) 2023 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sensirion_pasco2

// #include <device.h>
// #include <drivers/i2c.h>
// #include <kernel.h>
// #include <drivers/sensor.h>
// #include <sys/__assert.h>
// #include <sys/byteorder.h>
// #include <logging/log.h>

#include "pasco2.h"



#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#error "PASCO2 driver enabled without any devices"
#endif

LOG_MODULE_REGISTER(PASCO2, CONFIG_SENSOR_LOG_LEVEL);


int pasco2_write_command(const struct device *dev, uint16_t cmd)
{
  
}

int pasco2_write_reg(const struct device *dev, uint16_t cmd, uint16_t val)
{
	
}

static int pasco2_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{

}

static int pasco2_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{

}

static const struct sensor_driver_api pasco2_driver_api = {

	.sample_fetch = pasco2_sample_fetch,
	.channel_get = pasco2_channel_get,
};

// #define PASCO2_DEFINE(inst)							            \
// 	struct pasco2_data pasco20_data_##inst;					    \
// 	static const struct pasco2_config pasco20_cfg_##inst = {	\
// 		.bus = I2C_DT_SPEC_INST_GET(inst),				        \
// 		PASCO2_TRIGGER_INIT(inst)					            \
// 	};									                        \
// 	DEVICE_DT_INST_DEFINE(inst, pasco2_init, NULL,				\
// 		&pasco20_data_##inst, &pasco20_cfg_##inst,			    \
// 		POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,			    \
// 		&pasco2_driver_api);

// DT_INST_FOREACH_STATUS_OKAY(PASCO2_DEFINE)
#define KX022_CFG_IRQ(inst)

#define PASCO2_INIT(inst)                                                                           \
	static struct pasco2_data dev_data_##inst;                                                  \
	static const struct pasco2_config dev_config_##inst = {                                     \
		.bus_init = pasco2_i2c_init,                                                       		\
		.bus_cfg = I2C_DT_SPEC_INST_GET(inst),                                             		\
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, int_gpios), (PASCO2_CFG_IRQ(inst)), ())     	\
	};                                                                                         	\
	DEVICE_DT_INST_DEFINE(inst, pasco2_init, NULL, &dev_data_##inst, &dev_config_##inst,        \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &pasco2_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(PASCO2_INIT)