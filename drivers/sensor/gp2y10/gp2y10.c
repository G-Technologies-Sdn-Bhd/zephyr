
#define DT_DRV_COMPAT sharp_gp2y10

#include <logging/log.h>

#include <pm/pm.h>
#include <drivers/sensor.h>
#include "gp2y10.h"

LOG_MODULE_REGISTER(gp2y10, CONFIG_SENSOR_LOG_LEVEL);


static void gp2y10_uart_flush(const struct device *uart_dev)
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

static void gp2y10_uart_isr(const struct device *uart_dev, void *user_data)
{

	const struct device *dev = user_data;
	struct gp2y10_data *d = dev->data;

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
					     GP2Y10_BUF_LEN - d->xfer_bytes);
        if(d->buffer[0]!= 0xAA){
            d->xfer_bytes = 0;
        }
        else{
		if (d->xfer_bytes == GP2Y10_BUF_LEN){

            //  LOG_HEXDUMP_WRN( d->buffer,sizeof(d->buffer),"Rd data:");
			d->xfer_bytes = 0;
			uart_irq_rx_disable(uart_dev);
			k_sem_give(&d->rx_sem);
		}
        }
	}
}

static inline int gp2y10_poll_data(const struct device *dev)
{

	struct gp2y10_data *d = dev->data;
	const struct gp2y10_cfg *cfg = dev->config;

	int ret;
	int sum;
	int count = 0;
	int i = 0;
	uint8_t retry = 0;
	int index;
	k_sem_reset(&d->rx_sem);
	uart_irq_rx_enable(cfg->uart_dev);
	ret = k_sem_take(&d->rx_sem, gp2y10_WAIT);
	if (ret) {
		return ret;
	}

	sum =d->buffer[1] +d->buffer[2] +d->buffer[3]+ d->buffer[4];
    if(sum !=d->buffer[5])
    {
		d->data_valid = false;
        LOG_ERR("Missmatch Checksum");

	}
    double value;
    value = (((uint16_t)d->buffer[1] << 8) | (d->buffer[2]))/1024.0;
    value *=5.0; 
    
    /*Dust coefficient need to recalibrate with dust meter */
    float a = 100/0.35;
    value = a *value;
	d->data_valid = true;
	d->data =(int)(value *100);
    uart_irq_rx_disable(cfg->uart_dev);

	return ret;
}

static int gp2y10_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	pm_constraint_set(PM_STATE_SUSPEND_TO_IDLE);
	return gp2y10_poll_data(dev);
	pm_constraint_release(PM_STATE_SUSPEND_TO_IDLE);
}


static int gp2y10_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct gp2y10_data *d = dev->data;

	if (chan != SENSOR_CHAN_PM_10){
		return -ENOTSUP;
	}

	if (!d->data_valid){
		LOG_DBG("Checksum mismatch");
		return -EINVAL;
	}

	val->val1 = (int16_t)d->data/100;
	val->val2 = (d->data%100)*10000;

	return 0;
}

static const struct sensor_driver_api gp2y10_api_funcs = {
	.sample_fetch = gp2y10_sample_fetch,
	.channel_get = gp2y10_channel_get,
};

static int gp2y10_init(const struct device *dev)
{
	struct gp2y10_data *d = dev->data;
	const struct gp2y10_cfg *cfg = dev->config;

	memset(d, 0, sizeof(struct gp2y10_data));

	uart_irq_tx_disable(cfg->uart_dev);
	uart_irq_rx_disable(cfg->uart_dev);

	gp2y10_uart_flush(cfg->uart_dev);

	uart_irq_callback_user_data_set(cfg->uart_dev, cfg->cb, (void *)dev);

	k_sem_init(&d->rx_sem, 0, 1);

	return 0;
}


#define GP2Y10_INIT(inst)										\
													\
	static struct gp2y10_data gp2y10_data_##inst;							\
													\
	static const struct gp2y10_cfg gp2y10_cfg_##inst = {						\
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),						\
		.cb = gp2y10_uart_isr,									\
	};												\
													\
	DEVICE_DT_INST_DEFINE(inst, gp2y10_init, NULL, &gp2y10_data_##inst, &gp2y10_cfg_##inst,		\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &gp2y10_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(GP2Y10_INIT)
