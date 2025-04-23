/*! ----------------------------------------------------------------------------
 * @file    dw_sleep.c
 * @brief   platform dependent sleep implementation
 *
 * @attention
 *
 * Copyright 2015 - 2021 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include <zephyr/kernel.h>
#include <deca_device_api.h>

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_sleep(unsigned int time_ms)
{
    k_msleep(time_ms);
}

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_usleep(unsigned long time_us)
{
    k_usleep(time_us);
}