/******************************************************************************
 * @file    motion.c
 * @author  Insight SiP
 *
 * @attention
 *	THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *	OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *	OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *	DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *	GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *	OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <app_event_manager.h>
#define MODULE motion
#include <caf/events/module_state_event.h>
#include <zephyr/logging/log.h>
#include <motion_event.h>

LOG_MODULE_REGISTER(MODULE);

#define MOTION_DEV DT_NODELABEL(accel0)

static bool initialized = false;
static const struct device *const motion_dev = DEVICE_DT_GET(MOTION_DEV);
static struct gpio_dt_spec gpio_supply = GPIO_DT_SPEC_GET(MOTION_DEV, supply_gpios);

static void fetch_and_display(const struct device *sensor)
{
	struct sensor_value accel[3];

	int rc = sensor_sample_fetch(sensor);
	if (rc < 0)
	{
		LOG_ERR("sensor_sample_fetch failed: %d", rc);
		return;
	}
	
	rc = sensor_channel_get(sensor,	SENSOR_CHAN_ACCEL_XYZ, accel);
	if (rc < 0)
	{
		LOG_ERR("sensor_channel_get failed: %d", rc);
		return;
	}

	// Submit event
	struct accel_event *accel_event = new_accel_event();
	accel_event->x = sensor_value_to_double(&accel[0]);
	accel_event->y = sensor_value_to_double(&accel[1]);
	accel_event->z = sensor_value_to_double(&accel[2]);
	APP_EVENT_SUBMIT(accel_event);
}

#ifdef CONFIG_LIS2DE12_TRIGGER
static void trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	fetch_and_display(dev);
}
#endif

static int motion_init(void)
{
	if (gpio_supply.port)
	{
		gpio_pin_configure_dt(&gpio_supply, GPIO_OUTPUT_ACTIVE);
		LOG_DBG("Supply on %s pin %d", gpio_supply.port->name, gpio_supply.pin);
	}
	else
	{
		LOG_ERR("Unable to set Lis2de12 power supply");
		return -ENODEV;
	}

	k_msleep(7);

	if (device_init(motion_dev) != 0)
	{
		LOG_ERR("Lis2de12 not initialized");
		return -ENODEV;
	}

	if (!device_is_ready(motion_dev))
	{
		LOG_ERR("Lis2de12 not ready");
		return -ENODEV;
	}

#if CONFIG_LIS2DE12_TRIGGER
	struct sensor_trigger trig;
	int rc;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	struct sensor_value odr = {
		.val1 = 1,
	};

	rc = sensor_attr_set(motion_dev, trig.chan,
						 SENSOR_ATTR_SAMPLING_FREQUENCY,
						 &odr);
	if (rc != 0)
	{
		LOG_ERR("Failed to set odr: %d", rc);
		return 0;
	}

	LOG_INF("Sampling at %u Hz", odr.val1);

	rc = sensor_trigger_set(motion_dev, &trig, trigger_handler);
	if (rc != 0)
	{
		LOG_ERR("Failed to set trigger: %d", rc);
		return 0;
	}
#endif

	return 0;
}

static bool app_event_handler(const struct app_event_header *aeh)
{
	if (is_module_state_event(aeh))
	{
		const struct module_state_event *event = cast_module_state_event(aeh);

		if (check_state(event, MODULE_ID(main), MODULE_STATE_READY))
		{
			__ASSERT_NO_MSG(!initialized);
			initialized = true;

			if (motion_init())
			{
				LOG_ERR("motion_init failed");
				module_set_state(MODULE_STATE_ERROR);
			}
			else
			{
				module_set_state(MODULE_STATE_READY);
			}
		}

		return false;
	}

	// If event is unhandled, unsubscribe.
	__ASSERT_NO_MSG(false);

	return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_EARLY(MODULE, module_state_event);