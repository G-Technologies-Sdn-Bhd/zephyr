/*
 * Copyright (c) 2021-2022 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT maxim_ds18b20
#include <device.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <drivers/sensor.h>
#include <string.h>
#include <zephyr.h>
#include <logging/log.h>

#include "ds18b20.h"

LOG_MODULE_REGISTER(DS18b20, CONFIG_SENSOR_LOG_LEVEL);

/**
 * @brief Measure duration of signal send by sensor
 *
 * @param drv_data Pointer to the driver data structure
 * @param active Whether current signal is active
 *
 * @return duration in usec of signal being measured,
 *         -1 if duration exceeds DS18b20_SIGNAL_MAX_WAIT_DURATION
 */

static int8_t ds18b20_reset(const struct device *dev)
{
	int ret;
	struct ds18b20_data *drv_data = dev->data;
	const struct ds18b20_config *cfg = dev->config;

	ret = gpio_pin_configure(drv_data->gpio, cfg->pin,
			GPIO_OUTPUT);
	gpio_pin_set(drv_data->gpio, cfg->pin, true);
	k_busy_wait(50);
	gpio_pin_set(drv_data->gpio, cfg->pin, false);
	k_busy_wait(500);
	gpio_pin_set(drv_data->gpio, cfg->pin, true);
	k_busy_wait(40);

	ret = gpio_pin_configure(drv_data->gpio, cfg->pin,
			GPIO_INPUT);
	
	int t = 0;
	while(gpio_pin_get(drv_data->gpio, cfg->pin) &&t>60){
		t++;
		k_usleep(1);
	}
	if(t >60)
	{
		LOG_ERR("TIMEOUT");
		return 0;
	}else
	{
	k_busy_wait(500);
	ret = gpio_pin_configure(drv_data->gpio, cfg->pin,
			GPIO_OUTPUT);
	gpio_pin_set(drv_data->gpio, cfg->pin, true);
	
	}
	return 0;
}
static int8_t ds18b20_wr(const struct device *dev,uint8_t data){
	uint8_t i;
	int ret;
	struct ds18b20_data *drv_data = dev->data;
	const struct ds18b20_config *cfg = dev->config;

	ret = gpio_pin_configure(drv_data->gpio, cfg->pin,
			GPIO_OUTPUT);
	
	for(i=8;i>0;i--)
	{
		gpio_pin_set(drv_data->gpio, cfg->pin,0);
		k_busy_wait(5);

		if(data & 0x01)
		{
			gpio_pin_set(drv_data->gpio, cfg->pin,1);
		}
		else
		{
			gpio_pin_set(drv_data->gpio, cfg->pin,0);
		}
		k_busy_wait(65);
		gpio_pin_set(drv_data->gpio, cfg->pin,1);
		k_busy_wait(2);
		data >>=1;
	}
}
static uint8_t ds18b20_rd(const struct device *dev)
{
struct ds18b20_data *drv_data = dev->data;
const struct ds18b20_config *cfg = dev->config;
uint8_t i,data = 0;
int ret;

	ret = gpio_pin_configure(drv_data->gpio, cfg->pin,
			GPIO_OUTPUT);
	gpio_pin_set(drv_data->gpio, cfg->pin,1);
	k_busy_wait(5);
	
	for(i=8;i>0;i--)
	{
		data >>=1;
		ret = gpio_pin_configure(drv_data->gpio, cfg->pin,
			GPIO_OUTPUT);
		gpio_pin_set(drv_data->gpio, cfg->pin,0);
		k_busy_wait(2);

		ret = gpio_pin_configure(drv_data->gpio, cfg->pin,
			GPIO_INPUT);
		if(gpio_pin_get(drv_data->gpio, cfg->pin))
		{
			data |=0x80;

		}else
		{
			data &= DS_PRECISION;
		}
		k_busy_wait(65);
	}
	return data;
}

void conv_start(const struct device *dev){
	ds18b20_reset(dev);
	ds18b20_wr(dev,SkipROM);
	ds18b20_wr(dev,0x44);
}

// void temp_read_rom(uint8_t *addr)
// {
// 	uint8_t i;

// 	ds18b20_wr(0x33);
// 	for(i=8;i>0;i--)
//  	{
//   		*addr=temperature_read_byte();
//   		addr++;
//  	}
// }
static int ds18b20_sample_fetch(const struct device *dev,
			    enum sensor_channel chan)
{
	struct ds18b20_data *drv_data = dev->data;
	int ret = 0;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);
		uint8_t HH,HL,LL;
	int16_t temp_data;
	uint16_t data;
	float temp;
	
	conv_start(dev);
	k_busy_wait(800);
	ds18b20_reset(dev);
	k_busy_wait(1);
	ds18b20_wr(dev,SkipROM);
	ds18b20_wr(dev,ReadScratchpad);
	HL = ds18b20_rd(dev);
	HH = ds18b20_rd(dev);
	temp_data = ((HH<<8)|HL);
	temp = (float)(temp_data *0.0625);
	data = (uint16_t)((float)temp)*100;
	drv_data->temperature = (int)(temp*100);
	
	return ret;
}

static int ds18b20_channel_get(const struct device *dev,
			   enum sensor_channel chan,
			   struct sensor_value *val)
{
	struct ds18b20_data *drv_data = dev->data;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_AMBIENT_TEMP);
		
		val->val1 = drv_data->temperature/100;
		val->val2 = (drv_data->temperature % 100)*10000;
	// val = drv_data->temperature;
	return 0;
}

static const struct sensor_driver_api ds18b20_api = {
	.sample_fetch = &ds18b20_sample_fetch,
	.channel_get = &ds18b20_channel_get,
};

static int ds18b20_init(const struct device *dev)
{
	int rc = 0;
	struct ds18b20_data *drv_data = dev->data;
	const struct ds18b20_config *cfg = dev->config;

	drv_data->gpio = device_get_binding(cfg->ctrl);
	if (drv_data->gpio == NULL) {
		LOG_ERR("Failed to get GPIO device %s.", cfg->ctrl);
		return -EINVAL;
	}

	ds18b20_reset(dev);
	ds18b20_wr(dev,SkipROM);
	ds18b20_wr(dev,WriteScratchpad);
	ds18b20_wr(dev,DS_AlarmTH);
	ds18b20_wr(dev,DS_AlarmTL);
	ds18b20_wr(dev,DS_PRECISION);

	ds18b20_reset(dev);
	ds18b20_wr(dev,SkipROM);
	ds18b20_wr(dev,CopyScratchpad);
	int t =0;
	while(gpio_pin_get(drv_data->gpio, cfg->pin) && t> 60){
		t++;
		k_usleep(1);
		return 0;
	}
	conv_start(dev);

	return rc;
}

static struct ds18b20_data ds18b20_data;
static const struct ds18b20_config ds18b20_config = {
	.ctrl = DT_INST_GPIO_LABEL(0, dio_gpios),
	.flags = DT_INST_GPIO_FLAGS(0, dio_gpios),
	.pin = DT_INST_GPIO_PIN(0, dio_gpios),
};

DEVICE_DT_INST_DEFINE(0, &ds18b20_init, NULL,
		    &ds18b20_data, &ds18b20_config,
		    POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &ds18b20_api);
