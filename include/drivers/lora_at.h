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

#define DECODE(buffer, buffer_size, merged_buffer, merged_buffer_size) \
    size_t index = 0; \
    for (size_t i = 0; i < buffer_size; i++) { \
        /* Note: if you want to send hex, use "%02x"; if you want to send in decimal, use "%02d" */ \
        index += snprintf(merged_buffer + index, merged_buffer_size - index, "%02d", buffer[i]); \
        if (index >= merged_buffer_size - 1) { \
            /* The merged buffer is full, exit the loop to avoid overflow */ \
            break; \
        } \
    } \
    merged_buffer[index] = '\0'


int LORA_MGMT_RAISE_CONNECT_RESULT_EVENT(void);
#ifdef __cplusplus
}
#endif

#endif	/* ZEPHYR_INCLUDE_DRIVERS_LORA_AT_H_ */