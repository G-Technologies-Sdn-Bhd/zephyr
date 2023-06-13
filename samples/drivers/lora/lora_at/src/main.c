/*
 * Copyright (c) 2019 Manivannan Sadhasivam
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/lora.h>
#include <errno.h>
#include <sys/util.h>
#include <zephyr.h>
#include <random/rand32.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include"drivers/lora_at.h"

#define BUFFER_SIZE 4
#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>

LOG_MODULE_REGISTER(lora_at);

void main(void)
{
	const char *const label = DT_LABEL(DT_INST(0, ebyte_e78));
	const struct device *lora_dev = device_get_binding(label);
	const struct device *dev = device_get_binding("SHT3XD");
	struct sensor_value temp, hum;
	int rc;

	if (!device_is_ready(lora_dev)) {
		LOG_ERR("%s Device not ready", label);
		return;
	}

	if (dev == NULL) {
		printf("Could not get SHT3XD device\n");
		return;
	}
	
	LOG_WRN("%s Device ready", label);


	uint32_t data;
	char str[20];
	uint8_t da[20];
	char d;
	char stt[9];
    uint32_t eui=1100001;

    snprintf(stt, sizeof(stt), "%09d", eui);
   
   lora_config(lora_dev,eui);

	while (1) {
		if(LORA_MGMT_RAISE_CONNECT_RESULT_EVENT()==0)
	{
		rc = sensor_sample_fetch(dev);

		if (rc == 0) {
				rc = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP,
							&temp);
			}
			if (rc == 0) {
				rc = sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY,
							&hum);
			}
			if (rc != 0) {
				printf("SHT3XD: failed: %d\n", rc);
				break;
			}
				printf("SHT3XD: %d Cel ; %d %%RH\n",
				(int)sensor_value_to_double(&temp)*100,
				(int)sensor_value_to_double(&hum)*100);
			
			uint16_t t,h;
			t =  (int)sensor_value_to_double(&temp)*100;
			h = (int)sensor_value_to_double(&hum)*100;

			char a ="ca1f35221765d6c4499edbcada383764";
			char b ="1";
			char c ="ec0eadd58c17f2a2";
			uint16_t buffer[BUFFER_SIZE];// {0x12, 0x22, 0x32};
			char hexString[(BUFFER_SIZE*2)+1];
		
			buffer[0]= t;
			buffer[1]=h;

			DECODE(buffer, sizeof(buffer), hexString, sizeof(hexString));

			printk("Merged buffer: %s \n", hexString);

			printf("data: %d Cel ; %d %%RH\n",
						t,
						h);
				snprintf(str, sizeof(str), "%d%d", t,h);

			lora_send(lora_dev,hexString,sizeof(hexString));
			k_msleep(10000);
		}
	}
}
