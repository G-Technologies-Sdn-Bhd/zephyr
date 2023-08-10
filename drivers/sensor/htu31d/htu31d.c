
#define DT_DRV_COMPAT te_htu31d

#include <device.h>
#include <drivers/i2c.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <sys/byteorder.h>
#include <sys/crc.h>
#include <drivers/i2c.h>


#include "htu31d.h"

LOG_MODULE_REGISTER(HTU31D, CONFIG_SENSOR_LOG_LEVEL);


#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#error "HTU31D driver enabled without any devices"
#endif

static uint8_t htu31d_compute_crc(uint16_t value)
{
        uint8_t buf[2];

        sys_put_be16(value, buf);

        return crc8(buf, 2, HTU31D_CRC_POLY, HTU31D_CRC_INIT, false );     
}


static int htu31d_write_command(const struct device *dev, uint8_t cmd)
{
    const struct htu31d_config *cfg = dev->config;
    
    uint8_t tx_buf[1] = {cmd};
   
    return i2c_write_dt(&cfg->bus, tx_buf, sizeof(tx_buf));
}

static int htu31d_sample_fetch(const struct device *dev,
								enum sensor_channel chan)
{
	
	const struct htu31d_config *cfg = dev->config;
	struct htu31d_data *data = dev->data;

	uint8_t tx_buf[2];	
    uint8_t rx_buf[8];
	uint8_t cvt=HTU31D_CONVERSION;

    int rc;

	uint16_t t_sample, rh_sample;
	rc = i2c_write_dt(&cfg->bus, &cvt, sizeof(cvt));
		
		if (rc < 0) 
		{
			LOG_ERR("Failed to send command to device.");
			return rc;
		}

	k_msleep(HTU31D_WAIT_MS);
   
	sys_put_be16(HTU31D_CMD_READ_T_RH, tx_buf);
	rc =i2c_write_read_dt(&cfg->bus, tx_buf, sizeof(tx_buf),
			      rx_buf, sizeof(rx_buf));

    if (rc < 0) {
        LOG_ERR("Failed to read data from device.");
        return rc;
    }
	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);
    
	// Verify CRC 
	t_sample = (rx_buf[0] << 8) | rx_buf[1];
	if (htu31d_compute_crc(t_sample) != rx_buf[2]) {
        LOG_ERR("Invalid CRC for T.");
        return -EIO;
    }

    rh_sample = (rx_buf[3] << 8) | rx_buf[4];
	if (htu31d_compute_crc(rh_sample) != rx_buf[5]) {
        LOG_ERR("Invalid CRC for RH.");
        return -EIO;
    }

	data->t_sample = t_sample;
	data->rh_sample = rh_sample;

    return 0;


};


static int htu31d_channel_get(const struct device *dev,
                              enum sensor_channel chan,
                              struct sensor_value *val)
{
    const struct htu31d_data *data = dev->data;

	if (chan == SENSOR_CHAN_AMBIENT_TEMP) 
	{
	/* val = -40 + 165 * sample / (2^16 -1) */

    double tmp = ((double)data->t_sample / 65535.0) * 165.0 - 40.0;

    val->val1 = (int32_t)tmp;
    val->val2 = (int32_t)((tmp - val->val1) * 1000000.0);
	} 
	
	else if (chan == SENSOR_CHAN_HUMIDITY) 
	{
	/* val = 100 * sample / (2^16 -1) */

    double tmp = ((double)data->rh_sample / 65535.0) * 100.0;

    val->val1 = (int32_t)tmp;
    val->val2 = (int32_t)((tmp - val->val1) * 15625.0 / 1024.0);
	}
	
	else 
	{
        return -ENOTSUP;
    }

    return 0;
}

static int htu31d_init(const struct device *dev)
{
	const struct htu31d_config *cfg = dev->config;

	int rc = 0;
	
	if (!device_is_ready(cfg->bus.bus)) 
	{
		LOG_ERR("I2C bus %s is not ready!\n\r", cfg->bus.bus->name);
		return -EINVAL;
	}

	rc = htu31d_write_command(dev, HTU31D_CMD_RESET);
	
	if (rc < 0) 
	{
		LOG_ERR("Failed to reset the device.");
		return rc;
	}

	k_sleep(K_MSEC(HTU31D_RESET_WAIT_MS));
	k_sleep(K_MSEC(1000));

	return 0;
}

static const struct sensor_driver_api htu31d_driver_api = {
	.sample_fetch = htu31d_sample_fetch,
	.channel_get = htu31d_channel_get,
};

#define HTU31D_INIT(inst)                               				\
    static struct htu31d_data htu31d_data_##inst;        				\
                                                        				\
    static const struct htu31d_config htu31d_config_##inst = {			\
	.bus = I2C_DT_SPEC_INST_GET(inst),									\
    };                                                  				\
                                                        				\
    DEVICE_DT_INST_DEFINE(inst, htu31d_init, NULL,              		\
                          &htu31d_data_##inst, &htu31d_config_##inst, 	\
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,    	\
                          &htu31d_driver_api);

DT_INST_FOREACH_STATUS_OKAY(HTU31D_INIT)