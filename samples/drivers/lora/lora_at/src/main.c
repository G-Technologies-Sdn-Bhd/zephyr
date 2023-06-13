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

// #define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
// BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
// 	     "No default LoRa radio specified in DT");

#define BUFFER_SIZE 255
#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(lora_at);

void decode(uint16_t* buffer, size_t buffer_size, char* merged_buffer, size_t merged_buffer_size) {
    size_t index = 0;

    for (size_t i = 0; i < buffer_size; i++) {
        /*Note: if want to send hex used 2x is want to send in decimal use 2d*/
		index += snprintf(merged_buffer + index, merged_buffer_size - index,
                                 "%02d", buffer[i]);

        if (index >= merged_buffer_size - 1) {
            // The merged buffer is full, exit the loop to avoid overflow
            break;
        }
    }

    // Null-terminate the merged buffer
    merged_buffer[index] = '\0';
}

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

	

	uint32_t data;//[10] ;//=112233;
	char str[20];
	uint8_t da[20];
	char d;
	while (1) {

	da[0]= 10;
	da[1] =20;
	da[2] =30;
	snprintf(str, sizeof(str), "%d", da[0],da[1],da[2]);
	LOG_INF(" data: %s",log_strdup(str));

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

// #define BUFFER_SIZE 4
  uint16_t buffer[BUFFER_SIZE];// {0x12, 0x22, 0x32};
    char hexString[(BUFFER_SIZE*2)+1];
	buffer[0]= t;///100;//27;
	buffer[1]=h;//(t%100);//00;
	// buffer[2]= h/100;;
	// buffer[3]= h%100;;
   decode(buffer, sizeof(buffer), hexString, sizeof(hexString));

 printk("Merged buffer: %s \n", hexString);

printf("data: %d Cel ; %d %%RH\n",
		      t,
		       h);
	// data =sys_rand32_get();
	// sys_rand_get(&data,12);
	// d =STRINGIFY(data);
	snprintf(str, sizeof(str), "%d%d", t,h);
		// LOG_INF("Received data: %s %d %d ", log_strdup(hexString),strlen(hexString),sizeof(hexString));
	// for(uint8_t i=0;i<3;i++)
	// {
		lora_send(lora_dev,hexString,sizeof(hexString));
	// }
	// printf("Sending\r\n");
	k_msleep(10000);
	}
}
