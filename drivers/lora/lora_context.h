/** @file
 * @brief Modem context header file.
 *
 * A lora_at context driver allowing application to handle all
 * aspects of received protocol data.
 */

/*
 * Copyright (c) 2019 Foundries.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_LORA_AT_LORA_AT_CONTEXT_H_
#define ZEPHYR_INCLUDE_DRIVERS_LORA_AT_LORA_AT_CONTEXT_H_

#include <kernel.h>
#include <net/buf.h>
#include <net/net_ip.h>
#include <sys/ring_buffer.h>
#include <drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LORA_AT_PIN(name_, pin_, flags_) { \
	.dev_name = name_, \
	.pin = pin_, \
	.init_flags = flags_ \
}

struct lora_at_iface {
	const struct device *dev;

	int (*read)(struct lora_at_iface *iface, uint8_t *buf, size_t size,
		    size_t *bytes_read);
	int (*write)(struct lora_at_iface *iface, const uint8_t *buf, size_t size);

	/* implementation data */
	void *iface_data;
};

struct lora_at_cmd_handler {
	void (*process)(struct lora_at_cmd_handler *cmd_handler,
			struct lora_at_iface *iface);

	/* implementation data */
	void *cmd_handler_data;
};

struct lora_at_pin {
	const struct device *gpio_port_dev;
	char *dev_name;
	gpio_pin_t pin;
	gpio_flags_t init_flags;
};

struct lora_at_context {
	/* lora_at data */
	char *data_manufacturer;
	char *data_model;
	char *data_revision;
	char *data_imei;
	int   *data_rssi;
	bool  is_automatic_oper;
	/* pin config */
	struct lora_at_pin *pins;
	size_t pins_len;

	/* interface config */
	struct lora_at_iface iface;

	/* command handler config */
	struct lora_at_cmd_handler cmd_handler;

	/* driver data */
	void *driver_data;
};

/**
 * @brief  Gets lora_at context by id.
 *
 * @param  id: lora_at context id.
 *
 * @retval lora_at context or NULL.
 */
struct lora_at_context *lora_at_context_from_id(int id);

/**
 * @brief  Finds lora_at context which owns the iface device.
 *
 * @param  *dev: device used by the lora_at iface.
 *
 * @retval Modem context or NULL.
 */
struct lora_at_context *lora_at_context_from_iface_dev(const struct device *dev);

/**
 * @brief  Registers lora_at context.
 *
 * @note   Prepares lora_at context to be used.
 *
 * @param  *ctx: lora_at context to register.
 *
 * @retval 0 if ok, < 0 if error.
 */
int lora_at_context_register(struct lora_at_context *ctx);

/* pin config functions */
int lora_at_pin_read(struct lora_at_context *ctx, uint32_t pin);
int lora_at_pin_write(struct lora_at_context *ctx, uint32_t pin, uint32_t value);
int lora_at_pin_config(struct lora_at_context *ctx, uint32_t pin, bool enable);
int lora_at_pin_init(struct lora_at_context *ctx);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_LORA_AT_LORA_AT_CONTEXT_H_ */
