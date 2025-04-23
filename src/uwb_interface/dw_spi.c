/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   SPI access functions
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
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>
#include "dw_spi.h"


LOG_MODULE_DECLARE(dw);

#define DW_INST DT_INST(0, qorvo_dw3000)
#define DW_SPI	DT_PARENT(DT_INST(0, qorvo_dw3000))

static const struct device* spi;
static struct spi_cs_control cs_ctrl = SPI_CS_CONTROL_INIT(DW_INST, 0);
static struct spi_config spi_cfgs[2] = {0}; // configs for slow and fast
static struct spi_config* spi_cfg;


int dw_spi_init(void)
{
	/* set common SPI config */
	for (int i = 0; i < ARRAY_SIZE(spi_cfgs); i++) {
		spi_cfgs[i].cs = cs_ctrl;
		spi_cfgs[i].operation = SPI_WORD_SET(8);
	}

	spi_cfgs[0].frequency = 2000000;
	spi_cfgs[1].frequency = 32000000;

	/* Slow SPI clock speed: 2MHz */
	spi_cfg = &spi_cfgs[0];

	spi = DEVICE_DT_GET(DW_SPI);
	if (!spi) 
	{
		LOG_ERR("dw spi binding failed");
		return -1;
	} 
	else 
	{
		LOG_DBG("dw spi binding success (max %dMHz)", spi_cfgs[1].frequency / 1000000);	
	}

	return 0;
}

void dw_spi_slowrate_set(void)
{
   spi_cfg = &spi_cfgs[0];
}

void dw_spi_fastrate_set(void)
{
 	spi_cfg = &spi_cfgs[1];
}

void dw_spi_wakeup()
{
	gpio_pin_set_dt(&cs_ctrl.gpio, 1);
	k_sleep(K_USEC(500));
	gpio_pin_set_dt(&cs_ctrl.gpio, 0);
}

int32_t dw_spi_write_crc(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer, uint8_t crc8)
{
#ifdef DWT_ENABLE_CRC
	const struct spi_buf tx_buf[3] = 
	{
		{
			.buf = (void*)headerBuffer,
			.len = headerLength,
		},
		{
			.buf = (void*)bodyBuffer,
			.len = bodyLength,
		},
		{
			.buf = &crc8,
			.len = 1,
		},
	};

	const struct spi_buf_set tx = 
	{
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	return spi_transceive(spi, spi_cfg, &tx, NULL);
#endif //DWT_ENABLE_CRC
    return 0;
} 

int32_t dw_spi_write(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer)
{
  	const struct spi_buf tx_buf[2] = 
	{
		{
			.buf = (void*)headerBuffer,
			.len = headerLength,
		},
		{
			.buf = (void*)bodyBuffer,
			.len = bodyLength,
		},
	};
	const struct spi_buf_set tx = 
	{
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	return spi_transceive(spi, spi_cfg, &tx, NULL);
}

int32_t dw_spi_read(uint16_t headerLength, uint8_t *headerBuffer, uint16_t readLength, uint8_t *readBuffer)
{
 	const struct spi_buf tx_buf = {
		.buf = headerBuffer,
		.len = headerLength,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = headerLength,
		},
		{
			.buf = readBuffer,
			.len = readLength,
		},
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	return spi_transceive(spi, spi_cfg, &tx, &rx);
} 

