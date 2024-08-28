/*
 * Copyright (c) 2023 Trackunit Corporation
 * Copyright (c) 2023 Bjarki Arge Andreasen
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file drivers/rtc.h
 * @brief Public real time clock driver API
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_RTC_H_
#define ZEPHYR_INCLUDE_DRIVERS_RTC_H_

/**
 * @brief RTC Interface
 * @defgroup rtc_interface RTC Interface
 * @ingroup io_interfaces
 * @{
 */

// #include <types.h>
#include <device.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Mask for alarm time fields to enable when setting alarm time
 * @name RTC Alarm Time Mask
 * @anchor RTC_ALARM_TIME_MASK
 * @{
 */
#define RTC_ALARM_TIME_MASK_SECOND	BIT(0)
#define RTC_ALARM_TIME_MASK_MINUTE	BIT(1)
#define RTC_ALARM_TIME_MASK_HOUR	BIT(2)
#define RTC_ALARM_TIME_MASK_MONTHDAY	BIT(3)
#define RTC_ALARM_TIME_MASK_MONTH	BIT(4)
#define RTC_ALARM_TIME_MASK_YEAR	BIT(5)
#define RTC_ALARM_TIME_MASK_WEEKDAY	BIT(6)
#define RTC_ALARM_TIME_MASK_YEARDAY	BIT(7)
#define RTC_ALARM_TIME_MASK_NSEC	BIT(8)
/**
 * @}
 */

/**
 * @brief Structure for storing date and time values with sub-second precision.
 *
 * @details The structure is 1-1 mapped to the struct tm for the members
 * \p tm_sec to \p tm_isdst making it compatible with the standard time library.
 *
 * @note Use \ref rtc_time_to_tm() to safely cast from a \ref rtc_time
 * pointer to a \ref tm pointer.
 */
struct rtc_time {
	int tm_sec;	/**< Seconds [0, 59] */
	int tm_min;	/**< Minutes [0, 59] */
	int tm_hour;	/**< Hours [0, 23] */
	int tm_mday;	/**< Day of the month [1, 31] */
	int tm_mon;	/**< Month [0, 11] */
	int tm_year;	/**< Year - 1900 */
	int tm_wday;	/**< Day of the week [0, 6] (Sunday = 0) (Unknown = -1) */
	int tm_yday;	/**< Day of the year [0, 365] (Unknown = -1) */
	int tm_isdst;	/**< Daylight saving time flag [-1] (Unknown = -1) */
	int tm_nsec;	/**< Nanoseconds [0, 999999999] (Unknown = 0) */
};

/**
 * @typedef rtc_update_callback
 * @brief RTC update event callback
 *
 * @param dev Device instance invoking the handler
 * @param user_data Optional user data provided when update irq callback is set
 */
typedef void (*rtc_update_callback)(const struct device *dev, void *user_data);

/**
 * @typedef rtc_alarm_callback
 * @brief RTC alarm triggered callback
 *
 * @param dev Device instance invoking the handler
 * @param id Alarm id
 * @param user_data Optional user data passed with the alarm configuration
 */
typedef void (*rtc_alarm_callback)(const struct device *dev, uint16_t id, void *user_data);

/**
 * @cond INTERNAL_HIDDEN
 *
 * For internal driver use only, skip these in public documentation.
 */

/**
 * @typedef rtc_api_set_time
 * @brief API for setting RTC time
 */
typedef int (*rtc_api_set_time)(const struct device *dev, const struct rtc_time *timeptr);

/**
 * @typedef rtc_api_get_time
 * @brief API for getting RTC time
 */
typedef int (*rtc_api_get_time)(const struct device *dev, struct rtc_time *timeptr);

/**
 * @typedef rtc_api_alarm_get_supported_fields
 * @brief API for getting the supported fields of the RTC alarm time
 */
typedef int (*rtc_api_alarm_get_supported_fields)(const struct device *dev, uint16_t id,
						  uint16_t *mask);

/**
 * @typedef rtc_api_alarm_set_time
 * @brief API for setting RTC alarm time
 */
typedef int (*rtc_api_alarm_set_time)(const struct device *dev, uint16_t id, uint16_t mask,
				      const struct rtc_time *timeptr);

/**
 * @typedef rtc_api_alarm_get_time
 * @brief API for getting RTC alarm time
 */
typedef int (*rtc_api_alarm_get_time)(const struct device *dev, uint16_t id, uint16_t *mask,
				      struct rtc_time *timeptr);

/**
 * @typedef rtc_api_alarm_is_pending
 * @brief API for testing if RTC alarm is pending
 */
typedef int (*rtc_api_alarm_is_pending)(const struct device *dev, uint16_t id);

/**
 * @typedef rtc_api_alarm_set_callback
 * @brief API for setting RTC alarm callback
 */
typedef int (*rtc_api_alarm_set_callback)(const struct device *dev, uint16_t id,
					  rtc_alarm_callback callback, void *user_data);

/**
 * @typedef rtc_api_update_set_callback
 * @brief API for setting RTC update callback
 */
typedef int (*rtc_api_update_set_callback)(const struct device *dev,
					   rtc_update_callback callback, void *user_data);

/**
 * @typedef rtc_api_set_calibration
 * @brief API for setting RTC calibration
 */
typedef int (*rtc_api_set_calibration)(const struct device *dev, int32_t calibration);

/**
 * @typedef rtc_api_get_calibration
 * @brief API for getting RTC calibration
 */
typedef int (*rtc_api_get_calibration)(const struct device *dev, int32_t *calibration);

/**
 * @brief RTC driver API
 */
__subsystem struct rtc_driver_api {
	rtc_api_set_time set_time;
	rtc_api_get_time get_time;
};

/** @endcond */

/**
 * @brief API for setting RTC time.
 *
 * @param dev Device instance
 * @param timeptr The time to set
 *
 * @return 0 if successful
 * @return -EINVAL if RTC time is invalid or exceeds hardware capabilities
 * @return -errno code if failure
 */
__syscall int rtc_set_time(const struct device *dev, const struct rtc_time *timeptr);

static inline int z_impl_rtc_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	const struct rtc_driver_api *api = (const struct rtc_driver_api *)dev->api;

	return api->set_time(dev, timeptr);
}

/**
 * @brief API for getting RTC time.
 *
 * @param dev Device instance
 * @param timeptr Destination for the time
 *
 * @return 0 if successful
 * @return -ENODATA if RTC time has not been set
 * @return -errno code if failure
 */
__syscall int rtc_get_time(const struct device *dev, struct rtc_time *timeptr);

static inline int z_impl_rtc_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	const struct rtc_driver_api *api = (const struct rtc_driver_api *)dev->api;

	return api->get_time(dev, timeptr);
}
/**
 * @}
 */

/**
 * @name RTC Interface Helpers
 * @{
 */

/**
 * @brief Forward declaration of struct tm for \ref rtc_time_to_tm().
 */
struct tm;

/**
 * @brief Convenience function for safely casting a \ref rtc_time pointer
 * to a \ref tm pointer.
 */
static inline struct tm *rtc_time_to_tm(struct rtc_time *timeptr)
{
	return (struct tm *)timeptr;
}

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#include <syscalls/rtc.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_RTC_H_ */
