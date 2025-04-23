/*! ----------------------------------------------------------------------------
 * @file    port.c
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2016 - 2021 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "deca_device_api.h"
#include "deca_interface.h"
#include "dw_hw.h"
#include "dw_spi.h"

LOG_MODULE_REGISTER(dw);
#define DW_INST DT_INST(0, qorvo_dw3000)

static struct gpio_callback gpio_cb;
static struct gpio_dt_spec gpio_irq = GPIO_DT_SPEC_GET_OR(DW_INST, irq_gpios, {0});
static struct gpio_dt_spec gpio_reset = GPIO_DT_SPEC_GET_OR(DW_INST, reset_gpios, {0});
static struct gpio_dt_spec gpio_wakeup = GPIO_DT_SPEC_GET_OR(DW_INST, wakeup_gpios, {0});
static struct k_work dw_isr_work;


static void dw_isr_work_handler(struct k_work* item)
{
	while (gpio_pin_get(gpio_irq.port, gpio_irq.pin) != 0) 
	{
		dwt_isr();
	}
}

static void dw_isr_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
	k_work_submit(&dw_isr_work);
}

int dw_hw_init()
{
	/* Reset */
	if (gpio_reset.port) {
		gpio_pin_configure_dt(&gpio_reset, GPIO_INPUT);
		LOG_DBG("Reset on %s pin %d", gpio_reset.port->name, gpio_reset.pin);
	}

	/* Wakeup (optional) */
	if (gpio_wakeup.port) {
		gpio_pin_configure_dt(&gpio_wakeup, GPIO_OUTPUT_ACTIVE);
		LOG_DBG("Wakeup on %s pin %d", gpio_wakeup.port->name, gpio_wakeup.pin);
	}

	/* IRQ */
	if (gpio_irq.port) {
		k_work_init(&dw_isr_work, dw_isr_work_handler);

		gpio_pin_configure_dt(&gpio_irq, GPIO_INPUT);
		gpio_init_callback(&gpio_cb, dw_isr_cb, BIT(gpio_irq.pin));
		gpio_add_callback(gpio_irq.port, &gpio_cb);
		//gpio_pin_interrupt_configure_dt(&gpio_irq, GPIO_INT_EDGE_RISING);

		LOG_DBG("IRQ on %s pin %d", gpio_irq.port->name, gpio_irq.pin);
	} 
	else 
	{
		LOG_ERR("IRQ pin not configured");
	}

	return dw_spi_init();
}

void dw_hw_interrupt_enable(void)
{
	if (gpio_irq.port) {
		gpio_pin_interrupt_configure_dt(&gpio_irq, GPIO_INT_EDGE_RISING);
	}
}

void dw_hw_interrupt_disable(void)
{
	if (gpio_irq.port) {
		gpio_pin_interrupt_configure_dt(&gpio_irq, GPIO_INT_DISABLE);
	}
}


/****************************************************************************
 *
 *                          DW IC port section
 *
 *******************************************************************************/

/* @fn      reset_DW IC
 * @brief   DW_RESET pin on DW IC has 2 functions
 *          In general it is output, but it also can be used to reset the digital
 *          part of DW IC by driving this pin low.
 *          Note, the DW_RESET pin should not be driven high externally.
 * */
void dw_hw_reset(void)
{
	if (!gpio_reset.port) {
		LOG_ERR("No HW reset configured");
		return;
	}

	gpio_pin_configure_dt(&gpio_reset, GPIO_OUTPUT_ACTIVE);
	k_msleep(1); // 10 us?
	gpio_pin_configure_dt(&gpio_reset, GPIO_INPUT);
	k_msleep(2);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw_hw_wakeup()
 *
 * @brief This function wakes up the device by toggling io with a delay.
 *
 */
void dw_hw_wakeup(void)
{
	if (gpio_wakeup.port) 
	{
		/* Use WAKEUP pin if available */
		gpio_pin_set_dt(&gpio_wakeup, 1);
	} 
	else 
	{
		/* Use SPI CS pin */
		dw_spi_wakeup();
	}

	k_usleep(600); //k_usleep(200);

	if (gpio_wakeup.port) 
	{
		gpio_pin_set_dt(&gpio_wakeup, 0);
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn make_very_short_wakeup_io()
 *
 * @brief This will toggle the wakeup pin for a very short time. The device should not wakeup
 *
 */
void make_very_short_wakeup_io(void)
{
 /*  uint8_t cnt;

    SET_WAKEUP_PIN_IO_HIGH;
    for (cnt = 0; cnt < 10; cnt++)
        __NOP();
    SET_WAKEUP_PIN_IO_LOW;*/
}
