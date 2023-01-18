/* Sensirion PAS CO2 sensor 
 *
 * Copyright (c) 2023 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>

void main(void){
    const struct device *dev = device_get_binding("PASCO2");
    int rc;

    if (dev== NULL){
        printf("Could not get PASCO2 device\n");
        return;
    }

    while(true){
        struct sensor_value co2ppm;

        rc= sensor_sample_fetch(dev);
        if (rc ==0){
            rc = sensor_channel_get(dev, SENSOR_CHAN_CO2, &co2ppm);
        }
        if (rc !=0){
            printf("PASCO2: failed: %d\n", rc);
            break;
        }
        printf("PASCO2; %.2f ppm\n",sensor_value_to_double(&co2ppm));
        k_sleep(K_MSEC(2000));
       
	}
    }
// }

// #include <zephyr.h>
// #include <device.h>
// #include <drivers/sensor.h>
// #include <sys/printk.h>
// #include <sys/util.h>

// struct channel_info {
// 	int chan;
// 	char *dev_name;
// };

// static struct channel_info info[] = {
// 	{ SENSOR_CHAN_CO2, "PASCO2" }
// };

// void main(void)
// {
// 	const struct device *dev[ARRAY_SIZE(info)];
// 	struct sensor_value val[ARRAY_SIZE(info)];
// 	unsigned int i;
// 	int rc;

// 	for (i = 0; i < ARRAY_SIZE(info); i++) {
// 		dev[i] = device_get_binding(info[i].dev_name);
// 		if (dev[i] == NULL) {
// 			printk("Failed to get \"%s\" device\n",
// 			       info[i].dev_name);
// 			return;
// 		}
// 	}

//     while (1) {
// 		/* fetch sensor samples */
// 		for (i = 0U; i < ARRAY_SIZE(info); i++) {
// 			rc = sensor_sample_fetch(dev[i]);
// 			if (rc) {
// 				printk("Failed to fetch sample for device %s (%d)\n",
// 				       info[i].dev_name, rc);
// 			}
// 		}

// 		for (i = 0U; i < ARRAY_SIZE(info); i++) {
// 			rc = sensor_channel_get(dev[i], info[i].chan, &val[i]);
// 			if (rc) {
// 				printk("Failed to get data for device %s (%d)\n",
// 				       info[i].dev_name, rc);
// 				continue;
// 			}
// 		}
//         k_sleep(K_MSEC(2000));
// 	}
// }