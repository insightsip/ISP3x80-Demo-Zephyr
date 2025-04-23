/******************************************************************************
 * @file    leds_ctrl.c
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
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/gpio.h>
#include <app_event_manager.h>
#define MODULE leds_ctrl
#include <caf/events/module_state_event.h>
#include <zephyr/logging/log.h>
#include "twr_event.h"

LOG_MODULE_REGISTER(MODULE);

#define GREEN_LEDS_DEV DT_ALIAS(greenled)
#define RED_LEDS_DEV DT_ALIAS(redled)

/** @brief LED identification. */
enum led_id_tag
{
	LED_ID_GREEN,
	LED_ID_RED,

	// Number of LEDs.
	LED_ID_COUNT
} ;


static const struct gpio_dt_spec green_led_dev = GPIO_DT_SPEC_GET(GREEN_LEDS_DEV, gpios);
static const struct gpio_dt_spec red_led_dev = GPIO_DT_SPEC_GET(RED_LEDS_DEV, gpios);
static bool initialized = false;
static struct  k_work_delayable delayed_leds_work;
static uint32_t led_in_use;

void my_expiry_function(struct k_timer *dummy)
{
	if (led_in_use == LED_ID_GREEN)
	{
		gpio_pin_set_dt(&green_led_dev, 0);
	}
	else if (led_in_use == LED_ID_RED)
	{
		gpio_pin_set_dt(&red_led_dev, 0);
	} 
}

K_TIMER_DEFINE(my_timer, my_expiry_function, NULL);

static void delayed_leds_handler(struct k_work *item)
{
	if (led_in_use == LED_ID_GREEN)
	{
		gpio_pin_set_dt(&green_led_dev, 0);
	}
	else if (led_in_use == LED_ID_GREEN)
	{
		gpio_pin_set_dt(&red_led_dev, 0);
	}
}


static int leds_init(void)
{
	if (!gpio_is_ready_dt(&green_led_dev))
	{
		LOG_ERR(" Green LEDs not found");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&red_led_dev))
	{
		LOG_ERR(" Red LEDs not found");
		return -ENODEV;
	}

	k_work_init_delayable(&delayed_leds_work, delayed_leds_handler);

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

			if (leds_init())
			{
				LOG_ERR("leds_init failed");
				module_set_state(MODULE_STATE_ERROR);
			}
			else
			{
				module_set_state(MODULE_STATE_READY);
			}
		}

		return false;
	}

	if (is_range_event(aeh))
	{
		gpio_pin_set_dt(&green_led_dev, 1);
		led_in_use = LED_ID_GREEN;
		//k_work_schedule(&delayed_leds_work, K_MSEC(100));
		k_timer_start(&my_timer, K_MSEC(20), K_NO_WAIT );
		return false;
	}

	if (is_twr_error_event(aeh))
	{
		gpio_pin_set_dt(&red_led_dev, 1);
		led_in_use = LED_ID_RED;
		//k_work_schedule(&delayed_leds_work,  K_MSEC(100));
		k_timer_start(&my_timer, K_MSEC(20), K_NO_WAIT );

		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, range_event);
APP_EVENT_SUBSCRIBE(MODULE, twr_error_event);