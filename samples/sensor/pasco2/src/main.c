/*  Infineon PAS CO2 sensor driver
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
//   const struct i2c_device_id * id_table;

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

        printf("CO2 : %.2f ppm\n",sensor_value_to_double(&co2ppm));
        // printf("PASCO2: %.2f ppm\n",co2);
        // readc= pasco2_read_mode(dev, );
        // printf("PASCO2 Prod ID: %#02x \n",readc);

        k_sleep(K_MSEC(2000));
       
	}
    }