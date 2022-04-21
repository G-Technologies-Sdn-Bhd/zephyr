
#define DT_DRV_COMPAT aiman_dypa01

#include <logging/log.h>

#include <pm/pm.h>
#include <drivers/sensor.h>
#include "dypa01.h"

LOG_MODULE_REGISTER(dypa01, CONFIG_SENSOR_LOG_LEVEL);


static void dypa01_uart_flush(const struct device *uart_dev)
{
	uint8_t c;

	while (uart_fifo_read(uart_dev, &c, 1) > 0) {
		continue;
	}
}

static int checksum(const uint8_t *data)
{
	return (data[0] + data[1] + data[2]) & 0xFF;
}

static void dypa01_uart_isr(const struct device *uart_dev, void *user_data)
{

	const struct device *dev = user_data;
	struct dypa01_data *d = dev->data;

	ARG_UNUSED(user_data);

	if (uart_dev == NULL) {
		return;
	}

	/* Verify uart_irq_update() */
	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (uart_irq_rx_ready(uart_dev)) {

		d->xfer_bytes += uart_fifo_read(uart_dev, &d->buffer[d->xfer_bytes],
					     DYPA01_BUF_LEN - d->xfer_bytes);

		if (d->xfer_bytes == DYPA01_BUF_LEN){

		//	LOG_HEXDUMP_INF( d->buffer,sizeof(d->buffer),"Rd data:");
			d->xfer_bytes = 0;
			uart_irq_rx_disable(uart_dev);
			k_sem_give(&d->rx_sem);
		}

	}
}

static inline int dypa01_poll_data(const struct device *dev)
{

	struct dypa01_data *d = dev->data;
	const struct dypa01_cfg *cfg = dev->config;

	int ret;
	int sum;
	int count = 0;
	int i = 0;
	int index;

	k_sem_reset(&d->rx_sem);
	uart_irq_rx_enable(cfg->uart_dev);
	ret = k_sem_take(&d->rx_sem, dypa01_WAIT);

	/* find the index of the received uart buffer since
		we could also have consecutive 0xFF */
	if (d->buffer[i] == 0xFF){
		while(d->buffer[i] == 0xFF){
			i++;
		}
		index = i - 1;
	}
	else if (d->buffer[i] != 0xFF){
		i++;
		while (d->buffer[i] != 0xFF){
			i++;
		}
		while(d->buffer[i] == 0xFF){
			i++;
		}
		index = i - 1;
	}

	while (count != 4){
		d->rd_data[count++] = d->buffer[index++];
	}
			LOG_HEXDUMP_INF( d->buffer,sizeof(d->buffer),"Rd data:");

	LOG_HEXDUMP_INF( d->rd_data,sizeof(d->rd_data),"Rd data 2:");


	sum = checksum(d->rd_data);
	if (sum != d->rd_data[3]){
		d->data_valid = false;
		return -EBADMSG;
	}

	d->data_valid = true;
	d->data = ((uint16_t)d->rd_data[1] << 8) + (d->rd_data[2]);

	return ret;
}

static int dypa01_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	pm_constraint_set(PM_STATE_SUSPEND_TO_IDLE);
	return dypa01_poll_data(dev);
	pm_constraint_release(PM_STATE_SUSPEND_TO_IDLE);
}


static int dypa01_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct dypa01_data *d = dev->data;

	if (chan != SENSOR_CHAN_DISTANCE){
		return -ENOTSUP;
	}

	if (!d->data_valid){
		LOG_DBG("Checksum mismatch");
		return -EINVAL;
	}

	val->val1 = (int16_t)d->data;
	val->val2 = 0;

	return 0;
}

static const struct sensor_driver_api dypa01_api_funcs = {
	.sample_fetch = dypa01_sample_fetch,
	.channel_get = dypa01_channel_get,
};

static int dypa01_init(const struct device *dev)
{
	struct dypa01_data *d = dev->data;
	const struct dypa01_cfg *cfg = dev->config;

	memset(d, 0, sizeof(struct dypa01_data));

	uart_irq_rx_disable(cfg->uart_dev);

	dypa01_uart_flush(cfg->uart_dev);

	uart_irq_callback_user_data_set(cfg->uart_dev, cfg->cb, (void *)dev);

	k_sem_init(&d->rx_sem, 0, 1);

	return 0;
}


#define DYPA01_INIT(inst)										\
													\
	static struct dypa01_data dypa01_data_##inst;							\
													\
	static const struct dypa01_cfg dypa01_cfg_##inst = {						\
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),						\
		.cb = dypa01_uart_isr,									\
	};												\
													\
	DEVICE_DT_INST_DEFINE(inst, dypa01_init, NULL, &dypa01_data_##inst, &dypa01_cfg_##inst,		\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &dypa01_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(DYPA01_INIT)
