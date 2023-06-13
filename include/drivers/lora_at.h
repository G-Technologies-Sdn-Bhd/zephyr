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

int LORA_MGMT_RAISE_CONNECT_RESULT_EVENT(void);

#define MAX_ASCII_LEN 256

struct sensorData
{
	int64_t timestamp;
	int vmain;
	int v_phase1;
	int v_phase2;
	int v_phase3;
	int i_phase1;
	int i_phase2;
	int i_phase3;
	int p_phase1;
	int p_phase2;
	int p_phase3;
	int swell_v1;
	int swell_v2;
	int swell_v3;
	int swell_percentage1;
	int swell_percentage2;
	int swell_percentage3;
	int current;
	int active_power;
	int active_total_energy;
	int pf1;
	int pf2;
	int pf3;
	uint64_t tm;
};

struct devInfo
{
	int64_t timestamp;
	int vmain;
	uint64_t tm;
};
static struct devInfo DevInfo;
static struct sensorData SensorData;
#ifdef __cplusplus
}
#endif

int LORA_MGMT_RAISE_CONNECT_RESULT_EVENT(void);
void lora_start(const struct device *dev);
void lora_stop(const struct device *dev);
// typedef void (*lora_at_callback_t)(uint8_t status);
// lora_at_callback_t temp
#endif	/* ZEPHYR_INCLUDE_DRIVERS_LORA_AT_H_ */