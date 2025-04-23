/*! ----------------------------------------------------------------------------
 * @file    deca_probe_interface.c
 * @brief   Interface structure. Provides external dependencies required by the driver
 *
 * @attention
 *
 * Copyright 2015 - 2021 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include <zephyr/logging/log.h>

#include "deca_interface.h"
#include "dw_probe_interface.h"
#include "dw_spi.h"
#include "dw_hw.h"

LOG_MODULE_DECLARE(dw);

extern const struct dwt_driver_s dw3000_driver;
extern const struct dwt_driver_s dw3720_driver;
const struct dwt_driver_s* tmp_ptr[] = { &dw3000_driver, &dw3720_driver };

static const struct dwt_spi_s dw_spi_fct = {
    .readfromspi = dw_spi_read,
    .writetospi = dw_spi_write,
    .writetospiwithcrc = dw_spi_write_crc,
    .setslowrate = dw_spi_slowrate_set,
    .setfastrate = dw_spi_fastrate_set
};

const struct dwt_probe_s dw3000_probe_interf = 
{
    .dw = NULL,
    .spi = (void*)&dw_spi_fct,
    .wakeup_device_with_io = dw_hw_wakeup,
    .driver_list = (struct dwt_driver_s **)tmp_ptr,
    .dw_driver_num = 2,
};


int dw_probe_interface_init(void)
{
	int32_t ret;

	ret = dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);
	if (ret != DWT_SUCCESS)
	{
		LOG_ERR("dwt_probe failed");
		return -1;
	}

	return 0;
}