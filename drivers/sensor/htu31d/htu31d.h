/*



*/
#ifndef ZEPHYR_DRIVERS_SENSOR_HTU31D_HTU31D_H_
#define ZEPHYR_DRIVERS_SENSOR_HTU31D_HTU31D_H_

#include<device.h>

#define HTU31D_ADDR_1               0X40    //GND
#define HTU31D_ADDR_2               0X41    //VDD


#define HTU31D_CMD_READ_SERIAL             0x05
#define HTU31D_CMD_READ_RH                 0x04
#define HTU31D_CMD_RESET                   0x05
#define HTU31D_CMD_HEATER_ON               0x02
#define HTU31D_CMD_HEATER_OFF              0x01
#define HTU31D_CMD_READ_SN                 0x04
#define HTU31D_CMD_READ_DIAG               0x04

#define HTU31D_CMD_READ_T_RH               0x00
#define HTU31D_CONVERSION                   0x40


#define HTU31D_RESET_WAIT_MS	    5       //5 ms
#define HTU31D_WAIT_MS	    20       //5 ms


#define HTU31D_CRC_POLY		        0x31    // (x^8 + x^5 + x^4 +1)
#define HTU31D_CRC_INIT		        0x00    //initialization
struct htu31d_config
{
    struct i2c_dt_spec bus;
    const struct device *i2c_dev;
	// uint8_t i2c_addr;
};


struct htu31d_data{
    uint16_t t_sample;
    uint16_t rh_sample;
    uint16_t raw_temp; 
    uint16_t raw_humidity;
};
static const uint8_t measure_cmd[3] = {
	 0xE0, 0xF6, 0xFD
};
#endif /*ZEPHYR_DRIVERS_SENSOR_HTU31D_HTU31D_H*/