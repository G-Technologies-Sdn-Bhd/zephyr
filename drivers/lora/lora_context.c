/** @file
 * @brief Modem context helper driver
 *
 * A lora_at context driver allowing application to handle all
 * aspects of received protocol data.
 */

/*
 * Copyright (c) 2019 Foundries.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// #include <logging/log.h>
// LOG_MODULE_REGISTER(lora_at_context, CONFIG_LORA_AT_LOG_LEVEL);

#include <kernel.h>

#include "lora_at_context.h"

static struct lora_at_context *contexts[CONFIG_LORA_AT_CONTEXT_MAX_NUM];

char *lora_at_context_sprint_ip_addr(const struct sockaddr *addr)
{
	static char buf[NET_IPV6_ADDR_LEN];

	if (addr->sa_family == AF_INET6) {
		return net_addr_ntop(AF_INET6, &net_sin6(addr)->sin6_addr,
				     buf, sizeof(buf));
	}

	if (addr->sa_family == AF_INET) {
		return net_addr_ntop(AF_INET, &net_sin(addr)->sin_addr,
				     buf, sizeof(buf));
	}

	LOG_ERR("Unknown IP address family:%d", addr->sa_family);
	strcpy(buf, "unk");
	return buf;
}

int lora_at_context_get_addr_port(const struct sockaddr *addr, uint16_t *port)
{
	if (!addr || !port) {
		return -EINVAL;
	}

	if (addr->sa_family == AF_INET6) {
		*port = ntohs(net_sin6(addr)->sin6_port);
		return 0;
	} else if (addr->sa_family == AF_INET) {
		*port = ntohs(net_sin(addr)->sin_port);
		return 0;
	}

	return -EPROTONOSUPPORT;
}

/**
 * @brief  Finds lora_at context which owns the iface device.
 *
 * @param  *dev: device used by the lora_at iface.
 *
 * @retval Modem context or NULL.
 */
struct lora_at_context *lora_at_context_from_iface_dev(const struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(contexts); i++) {
		if (contexts[i] && contexts[i]->iface.dev == dev) {
			return contexts[i];
		}
	}

	return NULL;
}

/**
 * @brief  Assign a lora_at context if there is free space.
 *
 * @note   Amount of stored lora_at contexts is determined by
 *         CONFIG_LORA_AT_CONTEXT_MAX_NUM.
 *
 * @param  *ctx: lora_at context to persist.
 *
 * @retval 0 if ok, < 0 if error.
 */
static int lora_at_context_get(struct lora_at_context *ctx)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(contexts); i++) {
		if (!contexts[i]) {
			contexts[i] = ctx;
			return 0;
		}
	}

	return -ENOMEM;
}

struct lora_at_context *lora_at_context_from_id(int id)
{
	if (id >= 0 && id < ARRAY_SIZE(contexts)) {
		return contexts[id];
	} else {
		return NULL;
	}
}

int lora_at_context_register(struct lora_at_context *ctx)
{
	int ret;

	if (!ctx) {
		return -EINVAL;
	}

	ret = lora_at_context_get(ctx);
	if (ret < 0) {
		return ret;
	}

	ret = lora_at_pin_init(ctx);
	if (ret < 0) {
		LOG_ERR("lora_at pin init error: %d", ret);
		return ret;
	}

	return 0;
}
