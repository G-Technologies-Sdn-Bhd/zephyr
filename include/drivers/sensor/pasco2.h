/*
 * Copyright (c) 2021-2022 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Extended public API for Kionix's KX022 sensors
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_PASCO2_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_PASCO2_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/sensor.h>
#include <device.h>

enum sensor_attribute_pasco2 {
	/* Configure the resolution of a sensor */
	SENSOR_ATTR_PASCO2_SAMPLE_RATE = SENSOR_ATTR_PRIV_START,

	/* Configure odr of a sensor */
	SENSOR_ATTR_PASCO2_CALIB_REF,

	/* motion detection timer */
	SENSOR_ATTR_PASCO2_PRESSURE_REF,
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_PASCO2_H_ */
