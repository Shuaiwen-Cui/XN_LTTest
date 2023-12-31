/**
 * @file trx_access.h
 *
 * @brief HAL related APIs for externally plugged transceivers
 *
 * Copyright (C) 2014-2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 *
 */

/*
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* Prevent double inclusion */
#ifndef TRX_ACCESS_H
#define TRX_ACCESS_H

/**
 * \defgroup group_trx_access
 *  This module includes api's and defenitions required for Devices with
 * externally plugged transceivers(Non Soc's)
 * @{
 */
/* === Includes ============================================================ */

#include <stdint.h>
#include "conf_trx_access.h"

/* === Macros =============================================================== */

/**
 * Write access command of the transceiver
 */
#define WRITE_ACCESS_COMMAND            (0xC0)

/**
 * Read access command to the tranceiver
 */
#define READ_ACCESS_COMMAND             (0x80)

/**
 * Frame write command of transceiver
 */
#define TRX_CMD_FW                      (0x60)

/**
 * Frame read command of transceiver
 */
#define TRX_CMD_FR                      (0x20)

/**
 * SRAM write command of transceiver
 */
#define TRX_CMD_SW                      (0x40)

/**
 * SRAM read command of transceiver
 */
#define TRX_CMD_SR                      (0x00)

#define TRX_TRIG_DELAY()  { nop(); nop(); }

/**
 * @brief Clears the transceiver main interrupt
 *
 */
#define trx_irq_flag_clr()          CLEAR_TRX_IRQ()

/**
 * This macro is used for handling endianness among the different CPUs.
 */
#define U16_TO_TARGET(x) (x)

typedef void (*irq_handler_t)(void);

/* === Types =============================================================== */

/* This macro saves the global interrupt status */
#define ENTER_TRX_CRITICAL_REGION() { uint32_t int_status	\
							= __disable_irq();

/* This macro restores the global interrupt status */
#define LEAVE_TRX_CRITICAL_REGION()	if (!int_status) \
							__enable_irq(); }

/* === Externals ============================================================ */

/* === Prototypes =========================================================== */

void trx_irq_init(irq_handler_t trx_irq_cb);

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Macros for TRX GPIO pins access.
 */
/** Macro to set Reset pin to high */
#define TRX_RST_HIGH()                  RST_HIGH()
/** Macro to set Reset pin to low */
#define TRX_RST_LOW()                   RST_LOW()
/** Macro to set SLP_TR pin to high */
#define TRX_SLP_TR_HIGH()               SLP_TR_HIGH()
/** Macro to set SLP_TR pin to low */
#define TRX_SLP_TR_LOW()                SLP_TR_LOW()
/** Macro to get the transceiver's main IRQ status */
#define TRX_IRQ_HIGH()                  IRQ_PINGET()

/**
 * @brief Reads frame buffer of the transceiver
 *
 * This function reads the frame buffer of the transceiver.
 *
 * @param[out] data Pointer to the location to store frame
 * @param[in] length Number of bytes to be read from the frame
 * buffer.
 */
void trx_frame_read(uint8_t *data, uint8_t length);

/**
 * @brief Writes data into frame buffer of the transceiver
 *
 * This function writes data into the frame buffer of the transceiver
 *
 * @param[in] data Pointer to data to be written into frame buffer
 * @param[in] length Number of bytes to be written into frame buffer
 */
void trx_frame_write(uint8_t *data, uint8_t length);

/**
 * @brief Reads current value from a transceiver register
 *
 * This function reads the current value from a transceiver register.
 *
 * @param addr Specifies the address of the trx register
 * from which the data shall be read
 *
 * @return value of the register read
 */
uint8_t trx_reg_read(uint8_t addr);
uint8_t trx_reg_read_isr(uint8_t addr);

/**
 * @brief Writes data into a transceiver register
 *
 * This function writes a value into transceiver register.
 *
 * @param addr Address of the trx register
 * @param data Data to be written to trx register
 *
 */
void trx_reg_write(uint8_t addr, uint8_t data);
void trx_reg_write_isr(uint8_t addr, uint8_t data);

/**
 * @brief Subregister read
 *
 * @param   addr  offset of the register
 * @param   mask  bit mask of the subregister
 * @param   pos   bit position of the subregister
 *
 * @return  value of the read bit(s)
 */
uint8_t trx_bit_read(uint8_t addr, uint8_t mask, uint8_t pos);

/**
 * @brief Subregister write
 *
 * @param[in]   reg_addr  Offset of the register
 * @param[in]   mask  Bit mask of the subregister
 * @param[in]   pos   Bit position of the subregister
 * @param[out]  new_value  Data, which is muxed into the register
 */
void trx_bit_write(uint8_t reg_addr, uint8_t mask, uint8_t pos,
		uint8_t new_value);

/**
 * @brief Reads data from SRAM of the transceiver
 *
 * This function reads from the SRAM of the transceiver
 *
 * @param[in] addr Start address in SRAM for read operation
 * @param[out] data Pointer to the location where data stored
 * @param[in] length Number of bytes to be read from SRAM
 */
void trx_sram_read(uint8_t addr, uint8_t *data, uint8_t length);

/**
 * @brief Writes data into SRAM of the transceiver
 *
 * This function writes data into the SRAM of the transceiver
 *
 * @param addr Start address in the SRAM for the write operation
 * @param data Pointer to the data to be written into SRAM
 * @param length Number of bytes to be written into SRAM
 */
void trx_sram_write(uint8_t addr, uint8_t *data, uint8_t length);

/**
 * @brief Writes and reads data into/from SRAM of the transceiver
 *
 * This function writes data into the SRAM of the transceiver and
 * simultaneously reads the bytes.
 *
 * @param addr Start address in the SRAM for the write operation
 * @param idata Pointer to the data written/read into/from SRAM
 * @param length Number of bytes written/read into/from SRAM
 */
void trx_aes_wrrd(uint8_t addr, uint8_t *idata, uint8_t length);

/**
 * @brief Initializes the SPI interface for communication with the transceiver
 */
void trx_spi_init(void);

/* ! @} */
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  /* TRX_ACCESS_H */
/* EOF */
