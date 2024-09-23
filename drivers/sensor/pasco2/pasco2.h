/*  Infineon PAS CO2 sensor driver
 *
 * Copyright (c) 2023 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVER_SENSOR_PASCO2_H_
#define ZEPHYR_DRIVER_SENSOR_PASCO2_H_

#include <kernel.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <sys/util.h>
#include <stdint.h>
#include <drivers/sensor.h>
#include <string.h>
#include <sys/byteorder.h>
#include <logging/log.h>

/*** Standard Commands ***/
#define PASCO2_RESET_DELAY 2000
#define PASCO_COMM_DELAY_MS 5
#define PASCO2_PROD_ID 0x00 // (reg status)
#define PASCO2_SENS_STS 0x01 //sensor ready status
#define PASCO2_MEAS_RATE_H 0x02 //reg
#define PASCO2_MEAS_RATE_L 0x03
#define PASCO2_MEAS_CFG 0x04

#define PASCO2_CO2PPM_H 0x05
#define PASCO2_CO2PPM_L 0x06

#define PASCO2_MEAS_STS 0x07
#define PASCO2_OP_MODE_IDLE 0
#define PASCO2_OP_MODE_SINGLE 1
#define PASCO2_BOC_CFG_AUTOMATIC 1
#define PASCO2_INT_CFG 0x08
#define PASCO2_ALARM_TH_H 0X09
#define PASCO2_ALARM_TH_L 0X0A

#define PASCO2_PRESS_REF_H 0X0B
#define PASCO2_PRESS_REF_L 0X0C

#define PASCO2_CALIB_RED_H 0X0D
#define PASCO2_CALIB_REF_L 0X0E
#define PASCO2_SENS_RST 0x10
#define PASCO2_SCRATCH_PAD 0X0F //scratchpad register (The user reads back register)

#define PASCO2_CLEAR_STATUS_WAIT_USEC 1000
#define PASCO2_OPERATING_MODE true
#define PASCO2_COMM_TEST_VAL 0xA5
#define PASCO2_CMD_SOFT_RESET 0xA3

/** Result code indicating a successful operation */
#define PASCO2_OK (0)

/** Result code indicating a communication error */
#define PASCO2_ERR_COMM (1)

/** Result code indicating that an unexpectedly large I2C write was requested which is not supported */
#define PASCO2_ERR_WRITE_TOO_LARGE (2)

/** Result code indicating that the sensor is not yet ready after reset */
#define PASCO2_ERR_NOT_READY (3)

/** Result code indicating whether a non-valid command has been received by the serial communication interface */
#define PASCO2_ICCERR (4)

/** Result code indicating whether a condition where VDD12V has been outside the specified valid range has been detected */
#define PASCO2_ORVS (5)

/** Result code indicating whether a condition where the temperature has been outside the specified valid range has been detected */
#define PASCO2_ORTMP (6)

/** Result code indicating that a new CO2 value is not yet ready */
#define PASCO2_READ_NRDY (7)

#define PASCO2_REG_SENS_STS_ICCER_POS (3)
#define PASCO2_REG_SENS_STS_ICCER_MSK                                                              \
	(0x01 << PASCO2_REG_SENS_STS_ICCER_POS) /*!< SENS_STS: ICCER mask */

#define PASCO2_REG_SENS_STS_ORVS_POS (4)
#define PASCO2_REG_SENS_STS_ORVS_MSK                                                               \
	(0x01 << PASCO2_REG_SENS_STS_ORVS_POS) /*!< SENS_STS: ORVS mask */

#define PASCO2_REG_SENS_STS_ORTMP_POS (5)
#define PASCO2_REG_SENS_STS_ORTMP_MSK                                                              \
	(0x01 << PASCO2_REG_SENS_STS_ORTMP_POS) /*!< SENS_STS: ORTMP mask */

#define PASCO2_REG_SENS_STS_SEN_RDY_POS (7)
#define PASCO2_REG_SENS_STS_SEN_RDY_MSK                                                            \
	(0x01 << PASCO2_REG_SENS_STS_SEN_RDY_POS) /*!< SENS_STS: SEN_RDY mask */

#define PASCO2_STANDY_MODE 0x00
enum comman_type {
	single_read = 0,
	single_write,
	single_update,

};
typedef union {
	struct {
		uint32_t op_mode : 2; /*!< @ref xensiv_pasco2_op_mode_t */
		uint32_t boc_cfg : 2; /*!< @ref xensiv_pasco2_boc_cfg_t */
		uint32_t pwm_mode : 1; /*!< @ref xensiv_pasco2_pwm_mode_t */
		uint32_t pwm_outen : 1; /*!< PWM output software enable bit */
		uint32_t : 2;
	} b; /*!< Structure used for bit  access */
	uint8_t u; /*!< Type used for byte access */
} pasco2_measurement_config_t;

typedef union {
	struct {
		uint32_t alarm_typ : 1; /*!< @ref xensiv_pasco2_alarm_type_t */
		uint32_t int_func : 3; /*!< @ref xensiv_pasco2_interrupt_function_t */
		uint32_t int_typ : 1; /*!< @ref xensiv_pasco2_interrupt_type_t */
		uint32_t : 3;
	} b; /*!< Structure used for bit access */
	uint8_t u; /*!< Type used for byte access */
} pasco2_interrupt_config_t;

typedef union {
	struct {
		uint32_t : 3;
		uint32_t iccerr : 1; /*!< Communication error notification bit.
                                                             Indicates whether an invalid command has been received by the serial communication interface*/
		uint32_t orvs : 1; /*!< Out-of-range VDD12V error bit */
		uint32_t ortmp : 1; /*!< Out-of-range temperature error bit */
		uint32_t pwm_dis_st : 1; /*!< PWM_DIS pin status */
		uint32_t sen_rdy : 1; /*!< Sensor ready bit */
	} b; /*!< Structure used for bit  access */
	uint8_t u; /*!< Type used for byte access */
} pasco2_status_t;

typedef union {
	struct {
		uint32_t : 2;
		uint32_t alarm : 1; /*!< Set at the end of every measurement sequence if a threshold violation occurs */
		uint32_t int_sts : 1; /*!< Indicates whether the INT pin has been latched to active state (if alarm or data is ready) */
		uint32_t drdy : 1; /*!< Indicates whether new data is available */
		uint32_t : 3;
	} b; /*!< Structure used for bit  access */
	uint8_t u; /*!< Type used for byte access */
} pasco2_meas_status_t;
struct pasco2_config {
	int (*bus_init)(const struct device *dev);
	struct i2c_dt_spec bus;
	uint16_t sampling_rate;
	uint16_t calibration_ref;
	uint16_t pressure_ref;
#ifdef CONFIG_PASCO2_TRIGGER
	struct gpio_dt_spec gpio_int;
	uint8_t int_pin;
#endif
};
// struct pasco2_config {
// 	// // struct i2c_dt_spec i2c;
// 	// //->
// 	int (*bus_init)(const struct device *dev);
// 	// struct i2c_dt_spec bus_cfg;
// 	// //<-

// 	struct i2c_dt_spec bus;
// };
//->
struct pasco2_data;
//<-
struct pasco2_transfer_function {
	int (*read_data)(const struct device *dev, uint8_t reg_addr, uint8_t *value, uint8_t len);
	int (*read_reg)(const struct device *dev, uint8_t reg_addr, uint8_t *value);
	int (*write_data)(const struct device *dev, uint8_t reg_addr, uint8_t *value, uint8_t len);
	int (*write_reg)(const struct device *dev, uint8_t reg_addr, uint8_t value);
	int (*update_reg)(const struct device *dev, uint8_t reg_addr, uint8_t mask, uint8_t value);
};
struct pasco2_data {
	uint16_t co2ppm_sample;
	const struct pasco2_transfer_function *hw_tf;
#ifdef CONFIG_PASCO2_TRIGGER
	const struct device *gpio;
	struct gpio_callback gpio_cb;

	struct sensor_trigger drdy_trigger;
	sensor_trigger_handler_t drdy_handler;
	const struct device *dev;
	struct k_work work;
#endif
};

int pasco2_i2c_init(const struct device *dev);
int pasco2_command(const struct device *dev, enum comman_type cmd, uint8_t reg_addr, uint8_t *val);
int pasco2_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);
int pasco2_trigger_init(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_SENSOR_KX022_KX022_H_ */
