/*
 * Copyright (c) 2019 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdlib.h>

void main(void)
{
    const struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, infineon_dps368)));

    if (dev == NULL) {
        printk("Failed to acquire DPS368 device\n");
        return;
    }

    printk("Device pointer: %p, Device name: %s\n", dev, dev->name);

    while (1) {
        struct sensor_value temp, press;

        sensor_sample_fetch(dev);
        sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);

        // printk("Temperature: %d.%06d°C, Pressure: %d.%06d kPa\n",
        //        temp.val1, abs(temp.val2), press.val1, press.val2);
        float temperature = (float)temp.val1 + (float)abs(temp.val2) / 1000000.0;
        float pressure = (float)press.val1 + (float)press.val2 / 1000000.0;
        float pressure_hpa =pressure*10 ;
        printk("[%.2f°C,  %.4f kPa,  %.3f hPa]\n", temperature, pressure,pressure_hpa);

         
        k_sleep(K_MSEC(1000));
    }
}
