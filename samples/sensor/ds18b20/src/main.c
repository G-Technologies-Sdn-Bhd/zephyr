/*
 * Copyright (c) 2021-2022 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>

static const char *now_str(void)
{
	static char buf[16]; /* ...HH:MM:SS.MMM */
	uint32_t now = k_uptime_get_32();
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= MSEC_PER_SEC;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
		 h, min, s, ms);
	return buf;
}

void main(void)
{
	const char *const label = DT_LABEL(DT_INST(0, maxim_ds18b20));
	const struct device *dht22 = device_get_binding(label);

	if (!dht22) {
		printf("Failed to find sensor %s\n", label);
		return;
	}

	while (true) {
		int rc = sensor_sample_fetch(dht22);

		if (rc != 0) {
			printf("Sensor fetch failed: %d\n", rc);
			break;
		}

		struct sensor_value temperature;

		rc = sensor_channel_get(dht22, SENSOR_CHAN_AMBIENT_TEMP,
					&temperature);
		
		if (rc != 0) {
			printf("get failed: %d\n", rc);
			break;
		}

		printf("[%s]: %.2f Cel\n",
		       now_str(),
		       sensor_value_to_double(&temperature)
			   );
		k_sleep(K_SECONDS(5));
	}
}
