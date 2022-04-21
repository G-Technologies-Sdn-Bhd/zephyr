
#ifndef ZEPHYR_DRIVERS_SENSOR_DYPA01_DYPA01_H_
#define ZEPHYR_DRIVERS_SENSOR_DYPA01_DYPA01_H_

#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/uart.h>


#define DYPA01_BUF_LEN 16
#define DYPA01_DATA_LEN 4

#define dypa01_TIMEOUT 1000 *MSEC_PER_SEC
#define dypa01_WAIT K_SECONDS(2)

struct dypa01_data {

	const struct device *uart_dev;

	uint16_t data;

	uint8_t xfer_bytes;

	uint8_t rd_data[DYPA01_DATA_LEN];
	uint8_t buffer[DYPA01_BUF_LEN];

	bool data_valid;

	struct k_sem tx_sem;
	struct k_sem rx_sem;

};

struct dypa01_cfg {
	const struct device *uart_dev;
	uart_irq_callback_user_data_t cb;
};

#endif
