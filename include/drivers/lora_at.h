/*
 * Copyright (c) 2021-2022 G-Technologies Sdn. Bhd. - All Rights Reserved
 *
 * Unauthorized copying and distributing of this file, via any medium is
 * strictly prohibited.
 *
 * If received in error, please contact G-Technologies Sdn. Bhd. at
 * info@gtsb.com.my, quoting the name of the sender and the addressee, then
 * delete it from your system.
 *
 * Proprietary and confidential
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_LORA_AT_H_
#define ZEPHYR_INCLUDE_DRIVERS_LORA_AT_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "components/power.h"
#include "modules/cms.h"

#define DECODE(buffer, buffer_size, merged_buffer, merged_buffer_size) \
    size_t index = 0; \
    for (size_t i = 0; i < buffer_size; i++) { \
        /* Note: if you want to send hex, use "%02x"; if you want to send in decimal, use "%02d" */ \
        index += snprintf(merged_buffer + index, merged_buffer_size - index, "%06d", buffer[i]); \
        if (index >= merged_buffer_size - 1) { \
            /* The merged buffer is full, exit the loop to avoid overflow */ \
            break; \
        } \
    } \
    merged_buffer[index] = '\0'


int LORA_MGMT_RAISE_CONNECT_RESULT_EVENT(void);

#define MAX_ASCII_LEN 230

struct lora_data
{
	
   struct c_pwr_fields pwr;
   struct m_cms_fields cms;
    uint8_t msg_mode;

	/* Report and Event type */
	uint8_t rprt_evt_type;

	/* Event Date */
	uint8_t evt_day;
	uint8_t evt_month;
	uint8_t evt_year;

	/* Event Time */
	uint8_t evt_hr;
	uint8_t evt_min;
	uint8_t evt_sec;

	/* GSM connection status */
	uint8_t gsm_conn_stat;

	/* GSM RSSI */
	uint8_t gsm_rssi;

	/* GSM MCC */
	uint32_t gsm_mcc; /* MKMK ROTO 8 */

	/* GSM type */
	char gsm_type; /* HSLOK EC21E MQTT */

	/* Primary battery level */
	uint16_t vmain;

	/* Secondary battery level */
	uint16_t vbat;

	/* GPS status */
	uint8_t gps_stat;

	/* Latitude */
	double gps_lat;

	/* Longtitude */
	double gps_lon;

	/* GPS speed */
	uint16_t gps_spd;

	/* GPS course */
	uint16_t gps_crs;

	/* GPS odometer */
	double gps_odo;

	/* IGN status */
	uint8_t ign_status;

	uint8_t key_status;

	uint32_t seq;

	// uint32_t vsolar;
	// uint32_t vsolar_batt;
	uint32_t evt;
};
static struct lora_data LoRa;
#ifdef __cplusplus
}
#endif

#endif	/* ZEPHYR_INCLUDE_DRIVERS_LORA_AT_H_ */