
#ifndef ZEPHYR_DRIVERS_SENSOR_GP2Y10_GP2Y10_H_
#define ZEPHYR_DRIVERS_SENSOR_GP2Y10_GP2Y10_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>

#define GP2Y10_BUF_LEN 7
#define GP2Y10_DATA_LEN 8
#define GP2Y10_START_BYTE 0xAA
#define GP2Y10_STOP_BYTE 0xFF
#define gp2y10_TIMEOUT 1000 *MSEC_PER_SEC
#define gp2y10_WAIT K_SECONDS(2)

struct gp2y10_data {

	const struct device *uart_dev;

	uint32_t data;

	uint8_t xfer_bytes;

	uint8_t rd_data[GP2Y10_DATA_LEN];
	uint8_t buffer[GP2Y10_BUF_LEN];

	bool data_valid;

	struct k_sem tx_sem;
	struct k_sem rx_sem;

};

struct gp2y10_cfg {
	const struct device *uart_dev;
	uart_irq_callback_user_data_t cb;
};

#endif
