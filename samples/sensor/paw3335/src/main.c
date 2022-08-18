/*
 * Copyright (c) 2022 G-Technologies Sdn. Bhd. - All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <drivers/sensor/paw3335.h>
#include <stdlib.h>
#include <math.h>
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define UNIT_PER_DISTANCE_MM	25.4 / 16000
#define M_PI        3.14159265358979323846


static double get_angle(double deltax, double deltay)
{
	double angle;

	angle = atan(deltay/deltax) * (180/M_PI);

	return angle;
}

void main(void)
{
	const struct device *dev;
	struct sensor_value val[3];

	int16_t xaxis = 0;
	int16_t yaxis = 0;
	int16_t xaxis_last, yaxis_last;
	uint8_t motion;
	double xdistance;
	double ydistance;
	double angle = 0.000;

	dev = device_get_binding(DT_LABEL(DT_INST(0, pixart_paw3335d)));
	if (dev == NULL) {
		printk("Could not obtain MPAW3335 device\n");
		return;
	}

	int rc;

	rc = sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_CFG_AXIS_SWAP_XY, NULL);
	if (rc < 0) {
		printk("Could not set attr\n");
		return;
	}
		printk("device connected %s",dev->name);

		while(1){

		int ret;
		ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_RAWDATA);
		if (ret < 0) {
			printk("Could not fetch x (%d)\n", ret);
			return;
		}

		ret = sensor_channel_get(dev, SENSOR_CHAN_ALL, val);
		if (ret < 0) {
			printk("Could not get x (%d)\n", ret);
			return;
		}

		motion = (uint8_t)val[0].val1 & 0x80;
		xaxis = xaxis + (int16_t)val[1].val1;
		yaxis = yaxis + (int16_t)val[2].val1;

		if (motion != 0x80){
			printk("NO MOTION DETECTED\n");
		}

		if (xaxis_last != xaxis && motion == 0x80){
			xaxis_last = xaxis;
			printk("X-AXIS changed\n");
		}

		if (yaxis_last != yaxis && motion == 0x80){
			yaxis_last = yaxis;
			printk("Y-AXIS changed\n");
		}

		xdistance = UNIT_PER_DISTANCE_MM * (xaxis_last);
		ydistance = UNIT_PER_DISTANCE_MM * (yaxis_last);

		angle = get_angle(xdistance, ydistance);

		printk("X_distance: %.3fmm\tY_distance: %.3fmm\tAngle: %.3f\n", xdistance, ydistance, angle);

		k_sleep(K_MSEC(1000));
		}
}
