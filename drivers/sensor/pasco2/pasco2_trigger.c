/*  Infineon PAS CO2 sensor driver
 *
 * Copyright (c) 2023 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <drivers/sensor.h>
#include "pasco2.h"
LOG_MODULE_REGISTER(PASCO2, CONFIG_SENSOR_LOG_LEVEL_ERR);

static void pasco2_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct pasco2_data *data = CONTAINER_OF(cb, struct pasco2_data, gpio_cb);

	// #if defined(CONFIG_KX022_TRIGGER_OWN_THREAD)
	// 	k_sem_give(&data->trig_sem);
	// #elif defined(CONFIG_KX022_TRIGGER_GLOBAL_THREAD)
	(void)k_work_submit(&data->work);
	// #endif
}
static void pasco2_handler_int(const struct device *dev)
{
	struct pasco2_data *data = dev->data;

	if (data->drdy_handler != NULL) {
		data->drdy_handler(dev, &data->drdy_trigger);
	}
}
static void pasco2_handle_int(const struct device *dev)
{
	struct pasco2_data *data = dev->data;
	int ret;

	(void)pasco2_handler_int(dev);
}

static void pasco2_work_cb(struct k_work *work)
{
	struct pasco2_data *data = CONTAINER_OF(work, struct pasco2_data, work);

	pasco2_handle_int(data->dev);
}
/**
 * @brief Initializes trigger function
 */
int pasco2_trigger_init(const struct device *dev)
{
	uint8_t val;
	struct pasco2_data *data = dev->data;
	const struct pasco2_config *cfg = dev->config;
	int ret;

	/* setup data ready gpio interrupt */

	if (!device_is_ready(cfg->gpio_int.port)) {
		if (cfg->gpio_int.port) {
			LOG_DBG("%s: device %s is not ready", dev->name, cfg->gpio_int.port->name);
			return -ENODEV;
		}
	}

	ret = gpio_pin_configure_dt(&cfg->gpio_int, GPIO_INPUT);
	if (ret) {
		LOG_DBG("%s: Failed to configure gpio %s: %d", dev->name, "pin", ret);
		return ret;
	}

	gpio_init_callback(&data->gpio_cb, pasco2_gpio_callback, BIT(cfg->gpio_int.pin));

	ret = gpio_add_callback(cfg->gpio_int.port, &data->gpio_cb);
	if (ret) {
		LOG_DBG("%s: Failed to configure gpio %s: %d", dev->name, "callback", ret);
		return ret;
	}

	data->dev = dev;

	data->work.handler = pasco2_work_cb;

	ret = gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_DISABLE);
	if (ret) {
		LOG_DBG("%s: Failed to configure gpio %s: %d", dev->name, "interrupt-DISABLE", ret);
	}

	return ret;
}

int pasco2_drdy_setup(const struct device *dev, sensor_trigger_handler_t handler)
{
	struct pasco2_data *data = dev->data;
	const struct pasco2_config *cfg = dev->config;
	int ret;
	data->drdy_handler = handler;
	uint8_t rx;
	uint8_t drdy_bit = 0x04;
	ret = pasco2_command(dev, single_write, PASCO2_INT_CFG, &rx);

	rx |= (drdy_bit << 1);

	ret = pasco2_command(dev, single_write, PASCO2_INT_CFG, rx);

	return ret;
}
int pasco2_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
	int ret;
	struct pasco2_data *data = dev->data;
	const struct pasco2_config *cfg = dev->config;

	if (handler == NULL) {
		LOG_WRN("%s: no handler", dev->name);
	}

	switch ((int)trig->type) {
	case SENSOR_TRIG_DATA_READY:
		ret = pasco2_drdy_setup(dev, handler);
		break;
	default:
		return -ENOTSUP;
	}

	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_DISABLE);
	if (ret) {
		LOG_DBG("%s: Failed to configure gpio %s: %d", dev->name, "interrupt-DISABLE", ret);
		return -EINVAL;
	}
	ret = gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_DBG("%s: Failed to configure gpio %s: %d", dev->name,
			"interrupt-EDGE_TO_ACTIVE", ret);
		return -EINVAL;
	}
}
