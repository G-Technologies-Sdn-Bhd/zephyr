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

#ifdef __cplusplus
}
#endif

int LORA_MGMT_RAISE_CONNECT_RESULT_EVENT(void);
void lora_start(const struct device *dev);
void lora_stop(const struct device *dev);
enum lora_evt{
	LORA_EVT_INIT =0,
	LORA_EVT_STATED,
	LORA_EVT_CONNECTED,
	LORA_EVT_STOP,
};
typedef void (*lora_cb)(int *evt);
typedef void (*lora_connect_cb)(int *evt);
// typedef enum lora_cb_state{
// 	LORA_CB_INIT =0,
// 	LORA_CB_STATED,
// 	LORA_CB_STOP,
// }lora_cb_state_t;
int lora_at_get_command(void);
void lora_set_origin_axis(void);
void lorawan_register_modem_status_callback(
lora_cb cb);
void lorawan_status_callback(
lora_connect_cb cb);
// typedef void (*lora_at_callback_t)(uint8_t status);
// lora_at_callback_t temp
#endif	/* ZEPHYR_INCLUDE_DRIVERS_LORA_AT_H_ */