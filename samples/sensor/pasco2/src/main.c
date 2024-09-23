/*  Infineon PAS CO2 sensor driver
 *
 * Copyright (c) 2023 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <drivers/sensor/pasco2.h>
#include <stdio.h>
#ifdef CONFIG_PASCO2_TRIGGER
static void co2_handler(const struct device *dev, struct sensor_trigger *trig)
{
	struct sensor_value co2ppm;
	int rc;
	rc = sensor_sample_fetch(dev);
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_CO2, &co2ppm);
	} else {
		printf("PASCO2: failed: %d\n", rc);
		// break;
	}
	printf("CO2 value : %d ppm\n", co2ppm.val1);
}
#endif
void main(void)
{
	//   const struct i2c_device_id * id_table;

	const struct device *dev = device_get_binding("PASCO2");
	int rc;

	if (dev == NULL) {
		printf("Could not get PASCO2 device\n");
		return;
	}

	struct sensor_value attr;
	attr.val1 = 10;
	attr.val2 = 0;

	rc = sensor_attr_set(dev, SENSOR_CHAN_CO2, SENSOR_ATTR_PASCO2_SAMPLE_RATE, &attr);

#ifdef CONFIG_PASCO2_TRIGGER
	struct sensor_trigger trig = { .type = SENSOR_TRIG_DATA_READY };
	rc = sensor_trigger_set(dev, &trig, co2_handler);
#else
	while (true) {
		// #if !MODE
		struct sensor_value co2ppm;

		rc = sensor_sample_fetch(dev);
		if (rc == 0) {
			rc = sensor_channel_get(dev, SENSOR_CHAN_CO2, &co2ppm);
		} else {
			printf("PASCO2: failed: %d\n", rc);
			// break;
		}

		printf("CO2 value : %d ppm\n", co2ppm.val1);

		// #endif
		k_sleep(K_SECONDS(10));
	}
#endif
}
