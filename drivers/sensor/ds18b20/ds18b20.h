/*
 * Copyright (c) 2021-2022 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_DS18B20_DS18B20_H_
#define ZEPHYR_DRIVERS_SENSOR_DS18B20_DS18B20_H_

// #include <zephyr/device.h>
// #include <zephyr/devicetree.h>
// #include <zephyr/kernel.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/sys/util_macro.h>
#include <device.h>
#define DS_PRECISION 0x7f   //1f=9; 3f=10; 5f=11; 7f=12;
#define DS_AlarmTH  0x64
#define DS_AlarmTL  0x8a
#define DS_CONVERT_TICK 1000

#define SkipROM    0xCC  
#define SearchROM  0xF0  
#define ReadROM    0x33  
#define MatchROM   0x55  
#define AlarmROM   0xEC  

#define StartConvert    0x44  
#define ReadScratchpad  0xBE  
#define WriteScratchpad 0x4E  
#define CopyScratchpad  0x48  
#define RecallEEPROM    0xB8   
#define ReadPower       0xB4 

/* resolution is set using bit 5 and 6 of configuration register
 * macro only valid for values 9 to 12
 */

struct ds18b20_scratchpad {
	int16_t temp;
	uint8_t alarm_temp_high;
	uint8_t alarm_temp_low;
	uint8_t config;
	uint8_t res[3];
	uint8_t crc;
};

// struct ds18b20_config {
// 	const struct device *bus;
// 	uint8_t family;
// 	uint8_t resolution;
// };

// struct ds18b20_data {
// 	struct w1_slave_config config;
// 	struct ds18b20_scratchpad scratchpad;
// 	bool lazy_loaded;
// };

// static inline const struct device *ds18b20_bus(const struct device *dev)
// {
// 	const struct ds18b20_config *dcp = dev->config;

// 	return dcp->bus;
// }
struct ds18b20_data {
	const struct device *gpio;
	int temperature;
};

struct ds18b20_config {
	const char *ctrl;
	gpio_dt_flags_t flags;
	gpio_pin_t pin;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_DS18B20_DS18B20_H_ */
