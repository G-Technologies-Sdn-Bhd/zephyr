
#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>

void main(void)
{
	const struct device *dev = device_get_binding("HTU31D");
	int rc;

	if (dev == NULL) {
		printf("Could not get HTU31D device\n");
		return;
	}
	else{
		printf("Success Connected HTU31D...\n");
	}

	while (true) {
		struct sensor_value temp, hum;
		// struct sensor_value hum;

		rc = sensor_sample_fetch(dev);
		
		if (rc == 0) {
			rc = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP,
						&temp);
		}
		if (rc == 0) {
			rc = sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY,
						&hum);
		}
		if (rc != 0) {
			printf("HTU31D: failed: %d\n", rc);
			break;
		}

		printf(" HTU31D:  [ %.2f°C ] ; [ %0.2f %% RH ]\n",
		       sensor_value_to_double(&temp),
		       sensor_value_to_double(&hum));

		k_sleep(K_MSEC(2000));
	}
}
