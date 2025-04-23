/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/bluetooth/services/bas.h>

#include <hal/nrf_saadc.h>

#include <app_event_manager.h>
#include <caf/events/power_event.h>
#include "battery_event.h"

#define MODULE battery_meas
#include <caf/events/module_state_event.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_ISP_BATTERY_MEAS_LOG_LEVEL);

#define ADC_NODE DT_NODELABEL(adc)
#define ADC_RESOLUTION 12
#define ADC_OVERSAMPLING 4 /* 2^ADC_OVERSAMPLING samples are averaged */
#define ADC_MAX 4096
#define ADC_REFERENCE ADC_REF_INTERNAL
#if IS_ENABLED(CONFIG_SOC_NRF54L15_CPUAPP)
#define ADC_REF_INTERNAL_MV 900UL
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_VOLTAGE_GAIN 4
#elif IS_ENABLED(CONFIG_SOC_NRF52833)
#define ADC_REF_INTERNAL_MV 600UL
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_VOLTAGE_GAIN 6
#endif
#define ADC_CHANNEL_ID 0
/* ADC asynchronous read is scheduled on odd works. Value read happens during
 * even works. This is done to avoid creating a thread for battery monitor.
 */
#define BATTERY_WORK_INTERVAL (CONFIG_ISP_BATTERY_MEAS_POLL_INTERVAL_MS / 2)

#if IS_ENABLED(CONFIG_ISP_BATTERY_MEAS_HAS_VOLTAGE_DIVIDER)
#define BATTERY_VOLTAGE(sample) (sample * ADC_VOLTAGE_GAIN * ADC_REF_INTERNAL_MV * (CONFIG_ISP_BATTERY_MEAS_VOLTAGE_DIVIDER_UPPER + CONFIG_ISP_BATTERY_MEAS_VOLTAGE_DIVIDER_LOWER) / CONFIG_ISP_BATTERY_MEAS_VOLTAGE_DIVIDER_LOWER / ADC_MAX)
#else
#define BATTERY_VOLTAGE(sample) (sample * ADC_VOLTAGE_GAIN * ADC_REF_INTERNAL_MV / ADC_MAX)
#endif

static const struct device *const adc_dev = DEVICE_DT_GET(ADC_NODE);
static struct k_work_delayable battery_lvl_read;
static struct k_poll_signal async_sig = K_POLL_SIGNAL_INITIALIZER(async_sig);
static struct k_poll_event async_evt = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &async_sig);
static int16_t adc_buffer;
static bool adc_async_read_pending;
static atomic_t active;
static bool sampling;
static bool calibrated;

static const struct adc_sequence sequence = {
	.options = NULL,
	.channels = BIT(ADC_CHANNEL_ID),
	.buffer = &adc_buffer,
	.buffer_size = sizeof(adc_buffer),
	.resolution = ADC_RESOLUTION,
	.oversampling = ADC_OVERSAMPLING,
	.calibrate = false,
};
static const struct adc_sequence sequence_calibrate = {
	.options = NULL,
	.channels = BIT(ADC_CHANNEL_ID),
	.buffer = &adc_buffer,
	.buffer_size = sizeof(adc_buffer),
	.resolution = ADC_RESOLUTION,
	.oversampling = ADC_OVERSAMPLING,
	.calibrate = true,
};

static const struct adc_channel_cfg channel_cfg = ADC_CHANNEL_CFG_DT(DT_CHILD(DT_NODELABEL(adc), channel_0));

/** @brief Function for converting the input voltage (in milli volts) into percentage of 3.0 Volts.
 *
 *  @details The calculation is based on a linearized version of the battery's discharge
 *           curve. 3.0V returns 100% battery level. The limit for power failure is 2.1V and
 *           is considered to be the lower boundary.
 *
 *           The discharge curve for CR2032 is non-linear. In this model it is split into
 *           4 linear sections:
 *           - Section 1: 3.0V - 2.9V = 100% - 42% (58% drop on 100 mV)
 *           - Section 2: 2.9V - 2.74V = 42% - 18% (24% drop on 160 mV)
 *           - Section 3: 2.74V - 2.44V = 18% - 6% (12% drop on 300 mV)
 *           - Section 4: 2.44V - 2.1V = 6% - 0% (6% drop on 340 mV)
 *
 *           These numbers are by no means accurate. Temperature and
 *           load in the actual application is not accounted for!
 *
 *  @param[in] mvolts The voltage in mV
 *
 *  @return    Battery level in percent.
 */
static uint8_t battery_level_in_percent(const uint16_t mvolts)
{
	uint8_t battery_level;

	if (mvolts >= 3000)
	{
		battery_level = 100;
	}
	else if (mvolts > 2900)
	{
		battery_level = 100 - ((3000 - mvolts) * 58) / 100;
	}
	else if (mvolts > 2740)
	{
		battery_level = 42 - ((2900 - mvolts) * 24) / 160;
	}
	else if (mvolts > 2440)
	{
		battery_level = 18 - ((2740 - mvolts) * 12) / 300;
	}
	else if (mvolts > 2100)
	{
		battery_level = 6 - ((2440 - mvolts) * 6) / 340;
	}
	else
	{
		battery_level = 0;
	}

	return battery_level;
}

static int init_adc(void)
{
	if (!device_is_ready(adc_dev))
	{
		LOG_ERR("ADC device not ready");
		return -ENODEV;
	}

	int err = adc_channel_setup(adc_dev, &channel_cfg);
	if (err)
	{
		LOG_ERR("Setting up the ADC channel failed");
		return err;
	}

	return 0;
}

static void battery_monitor_start(void)
{
	sampling = true;
	k_work_reschedule(&battery_lvl_read, K_MSEC(BATTERY_WORK_INTERVAL));
}

static void battery_monitor_stop(void)
{
	/* Cancel cannot fail if executed from another work's context. */
	(void)k_work_cancel_delayable(&battery_lvl_read);
	sampling = false;

	module_set_state(MODULE_STATE_STANDBY);
}

static void battery_lvl_process(void)
{
	uint32_t voltage = BATTERY_VOLTAGE(adc_buffer);
	uint8_t level;

	if (voltage > CONFIG_ISP_BATTERY_MEAS_MAX_LEVEL)
	{
		level = 100;
	}
	else if (voltage < CONFIG_ISP_BATTERY_MEAS_MIN_LEVEL)
	{
		level = 0;
		LOG_WRN("Low battery");
	}
	else
	{
		level = battery_level_in_percent(voltage);
	}

	bt_bas_set_battery_level(level);

	struct battery_level_event *event = new_battery_level_event();
	event->level = level;
	APP_EVENT_SUBMIT(event);

	LOG_INF("Battery level: %u%% (%u mV)", level, voltage);
}

static void battery_lvl_read_fn(struct k_work *work)
{
	int err;

	if (!adc_async_read_pending)
	{
		if (likely(calibrated))
		{
			err = adc_read_async(adc_dev, &sequence, &async_sig);
		}
		else
		{
			err = adc_read_async(adc_dev, &sequence_calibrate, &async_sig);
			calibrated = true;
		}

		if (err)
		{
			LOG_WRN("Battery level async read failed");
		}
		else
		{
			adc_async_read_pending = true;
		}
	}
	else
	{
		err = k_poll(&async_evt, 1, K_NO_WAIT);
		if (err)
		{
			LOG_WRN("Battery level poll failed");
		}
		else
		{
			adc_async_read_pending = false;
			battery_lvl_process();
		}
	}

	if (atomic_get(&active) || adc_async_read_pending)
	{
		k_work_reschedule(&battery_lvl_read, K_MSEC(BATTERY_WORK_INTERVAL));
	}
	else
	{
		battery_monitor_stop();
	}
}

static int init_fn(void)
{
	int err = 0;

	err = init_adc();

	if (err)
	{
		return err;
	}

	k_work_init_delayable(&battery_lvl_read, battery_lvl_read_fn);
	battery_monitor_start();

	return err;
}

static bool app_event_handler(const struct app_event_header *aeh)
{
	if (is_module_state_event(aeh))
	{
		struct module_state_event *event = cast_module_state_event(aeh);

		if (check_state(event, MODULE_ID(main), MODULE_STATE_READY))
		{
			static bool initialized;

			__ASSERT_NO_MSG(!initialized);
			initialized = true;

			int err = init_fn();

			if (err)
			{
				module_set_state(MODULE_STATE_ERROR);
			}
			else
			{
				module_set_state(MODULE_STATE_READY);
				atomic_set(&active, true);
			}

			return false;
		}

		return false;
	}

	/*if (is_wake_up_event(aeh))
	{
		if (!atomic_get(&active))
		{
			atomic_set(&active, true);

			battery_monitor_start();
			module_set_state(MODULE_STATE_READY);
		}

		return false;
	}*/

	/*if (is_power_down_event(aeh))
	{
		if (atomic_get(&active))
		{
			atomic_set(&active, false);

			if (adc_async_read_pending)
			{
				__ASSERT_NO_MSG(sampling);
				// Poll ADC and postpone shutdown
				k_work_reschedule(&battery_lvl_read, K_MSEC(0));
			}
			else
			{
				// No ADC sample left to read, go to standby
				battery_monitor_stop();
			}
		}

		return sampling;
	}*/

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}
APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);
//APP_EVENT_SUBSCRIBE_EARLY(MODULE, power_down_event);
// APP_EVENT_SUBSCRIBE(MODULE, wake_up_event);
