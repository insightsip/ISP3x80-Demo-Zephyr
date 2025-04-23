/*! ----------------------------------------------------------------------------
 * @file    dw_hw.h
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2015 - 2021 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#ifndef DW_HW_H
#define DW_HW_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include "dw_spi.h"

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

int dw_hw_init(void);
void dw_hw_reset(void);
void dw_hw_wakeup(void);
void dw_hw_interrupt_enable(void);
void dw_hw_interrupt_disable(void);



void make_very_short_wakeup_io(void);


#endif /* DW_HW_H */
