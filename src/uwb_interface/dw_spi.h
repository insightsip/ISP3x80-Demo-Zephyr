/*! ----------------------------------------------------------------------------
 * @file    deca_spi.h
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

#ifndef _DECA_SPI_H_
#define _DECA_SPI_H_

#include <errno.h>
#include <deca_types.h>

#define DECA_MAX_SPI_HEADER_LENGTH (3) // max number of bytes in header (for formating & sizing)

#define DATALEN1 200

typedef enum
{
	DW_HAL_NODE_UNLOCKED = 0,
	DW_HAL_NODE_LOCKED = -EBUSY
} dw_hal_lockTypeDef;

#define __HAL_LOCK(__HANDLE__)                        \
	do                                                \
	{                                                 \
		if ((__HANDLE__)->lock == DW_HAL_NODE_LOCKED) \
		{                                             \
			return -EBUSY;                            \
		}                                             \
		else                                          \
		{                                             \
			(__HANDLE__)->lock = DW_HAL_NODE_LOCKED;  \
		}                                             \
	} while (0U)

#define __HAL_UNLOCK(__HANDLE__)                   \
	do                                             \
	{                                              \
		(__HANDLE__)->lock = DW_HAL_NODE_UNLOCKED; \
	} while (0U)

/* description of spi interface to DW3000 chip */
typedef struct
{
	//nrf_drv_spi_t spi_inst;
	uint32_t frequency_slow;
	uint32_t frequency_fast;
	uint32_t csPin;
	//nrf_drv_spi_config_t spi_config;
	dw_hal_lockTypeDef lock;
} spi_handle_t;

/* description of connection to the DW3700 chip */
typedef struct
{
	uint16_t irqPin;
	uint16_t rstPin;
	uint16_t wkupPin;
	uint16_t csPin;
	spi_handle_t *pSpi;
} dw_t;

/* @fn    dw3000_spi_init
 * Initialise SPI
 * */
int dw_spi_init(void);

/* @fn      port_set_dw_ic_spi_slowrate
 * @brief   set 2MHz
 * */
void dw_spi_slowrate_set(void);

/* @fn      port_set_dw_ic_spi_fastrate
 * @brief   set 16MHz
 * */
void dw_spi_fastrate_set(void);

/* @fn      dw3000_spi_wakeup
 * @brief   wake up dw3000 with cs
 * */
void dw_spi_wakeup();

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospiwithcrc()
 *
 * Low level abstract function to write to the SPI when SPI CRC mode is used
 * Takes two separate byte buffers for write header and write data, and a CRC8 byte which is written last
 * returns 0 for success, or -1 for error
 */
int32_t dw_spi_write_crc(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer, uint8_t crc8);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw3000_spi_write()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int32_t dw_spi_write(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw3000_spi_read()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
// #pragma GCC optimize ("O3")
int32_t dw_spi_read(uint16_t headerLength, uint8_t *headerBuffer, uint16_t readLength, uint8_t *readBuffer);

#endif /* _DECA_SPI_H_ */
