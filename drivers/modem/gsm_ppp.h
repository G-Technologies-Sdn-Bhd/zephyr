/*
 * Copyright (c) 2022 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MODEM_GSM_PPP_H
#define MODEM_GSM_PPP_H

#include <zephyr.h>
#include <device.h>
#include "modem_context.h"
#include <drivers/gpio.h>

#define HAS_PWR_SRC DT_INST_NODE_HAS_PROP(0, power_src_gpios)
#define HAS_PWR_KEY DT_INST_NODE_HAS_PROP(0, power_key_gpios)

#if HAS_PWR_SRC
static const struct gpio_dt_spec modem_power_src =
    GPIO_DT_SPEC_GET(DT_DRV_INST(0), power_src_gpios);
#endif
#if HAS_PWR_KEY
static const struct gpio_dt_spec modem_power_key =
    GPIO_DT_SPEC_GET(DT_DRV_INST(0), power_key_gpios);
#endif

/**
 * @brief  Disable power source of the modem.
 *
 * @param  ctx: modem_context struct
 *
 * @retval None.
 */
static inline void disable_power_source(void)
{
#if HAS_PWR_SRC
    if (modem_power_src.port) {
        gpio_pin_set_dt(&modem_power_src, 0);
    }
#endif
}

/**
 * @brief  Enable power source of the modem.
 *
 * @param  ctx: modem_context struct
 *
 * @retval None.
 */
static inline void enable_power_source(void)
{
#if HAS_PWR_SRC
    if (modem_power_src.port) {
        gpio_pin_set_dt(&modem_power_src, 1);
    }
#endif
}

#if HAS_PWR_KEY
static void press_power_key(k_timeout_t dur)
{
    if (modem_power_key.port) {
        gpio_pin_set_dt(&modem_power_key, 1);
        k_sleep(dur);
        gpio_pin_set_dt(&modem_power_key, 0);
    }
}
#endif

/**
 * @brief  Operations required to turn the modem on.
 *
 * @param  ctx: modem_context struct
 *
 * @retval None.
 */
static inline void power_on_ops(void)
{
#if DT_INST_NODE_HAS_PROP(0, power_key_on_ms)
    press_power_key(K_MSEC(DT_INST_PROP(0, power_key_on_ms)));
#endif
}

/**
 * @brief  Operations required to turn the modem off.
 *
 * @param  ctx: modem_context struct
 *
 * @retval None.
 */
static inline void power_off_ops(void)
{
#if DT_INST_NODE_HAS_PROP(0, power_key_off_ms)
    press_power_key(K_MSEC(DT_INST_PROP(0, power_key_off_ms)));
#endif
}

/**
 * @brief  Perform a soft reboot of the modem.
 *
 * This function powers the modem off using the power key, waits,
 * and then powers it back on.
 *
 * @retval None.
 */
static inline void modem_soft_reboot(void)
{
#if HAS_PWR_KEY
    LOG_WRN("Performing modem soft reboot...");
    power_off_ops();
    /* Wait for modem to power down completely */
    k_sleep(K_SECONDS(5));
    power_on_ops();
    LOG_INF("Modem soft reboot sequence complete.");
#else
    LOG_WRN("Soft reboot not supported, no power key defined.");
#endif
}

#endif /* MODEM_GSM_PPP_H */
