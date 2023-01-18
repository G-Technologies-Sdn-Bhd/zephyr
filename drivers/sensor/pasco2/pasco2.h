

#ifndef ZEPHYR_DRIVER_SENSOR_PASCO2_H_
#define ZEPHYR_DRIVER_SENSOR_PASCO2_H_

#include <kernel.h>
#include <device.h>
#include <sys/util.h>
#include <stdint.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <string.h>
#include <drivers/i2c.h>

/*** Standard Commands ***/

#define PASCO2_PROD_ID          	0x00            // (reg status)
#define PASCO2_SENS_STS         	0x01            //sensor ready status
#define PASCO2_MEAS_RATE_H      	0x02            //reg
#define PASCO2_MEAS_RATE_L      	0x03
#define PASCO2_MEAS_CFG         	0x04

#define PASCO2_CO2PPM_H         	0x05
#define PASCO2_CO2PPM_L         	0x06

#define PASCO2_MEAS_STS         	0x07
#define PASCO2_INT_CFG          	0x08
#define PASCO2_ALARM_TH_H         0X09
#define PASCO2_ALARM_TH_L         0X0A

#define PASCO2_PRESS_REF_H        0X0B
#define PASCO2_PRESS_REF_L        0X0C

#define PASCO2_CALIB_RED_H        0X0D
#define PASCO2_CALIB_REF_L        0X0E
#define PASCO2_SENS_RST         	0x10        //soft reset (The user writes register SENS_RST to trigger a soft reset.)
#define PASCO2_SCRATCH_PAD			  0X0F        //scratchpad register (The user reads back register)

#define PASCO2_CLEAR_STATUS_WAIT_USEC   1000
#define PASCO2_OP_MODE              true
// #define PASCO2_STANDY_MODE       false

struct pasco2_config {
	// struct i2c_dt_spec i2c;
	struct i2c_dt_spec bus;

};

/////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////
// int pasco2_write_command(const struct device *dev, uint16_t cmd);

// int pasco2_write_reg(const struct device *dev, uint16_t cmd, uint16_t val);

// int pasco2_attr_set(const struct device *dev,
// 		    enum sensor_channel chan,
// 		    enum sensor_attribute attr,
// 		    const struct sensor_value *val);

// struct pasco2_transfer_function {
// 	int (*read_data)(const struct device *dev, uint8_t reg_addr, uint8_t *value, uint8_t len);
// 	int (*write_data)(const struct device *dev, uint8_t reg_addr, uint8_t *value, uint8_t len);
// 	int (*read_reg)(const struct device *dev, uint8_t reg_addr, uint8_t *value);
// 	int (*write_reg)(const struct device *dev, uint8_t reg_addr, uint8_t value);
// 	int (*update_reg)(const struct device *dev, uint8_t reg_addr, uint8_t mask, uint8_t value);
// };

struct pasco2_data
{
 	// const struct device *i2c;
  uint16_t co2ppm_sample;
//   const struct pasco2_transfer_function *hw_tf;
};

#endif  /* ZEPHYR_DRIVERS_SENSOR_PASCO2_H_ */
