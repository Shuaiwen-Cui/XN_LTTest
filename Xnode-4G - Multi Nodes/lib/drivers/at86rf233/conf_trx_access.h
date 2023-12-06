/**
 * \file *********************************************************************
 *
 * \brief Common TRX Access Configuration
 *
 * Copyright (c) 2013-2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
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
 */

#ifndef CONF_TRX_ACCESS_H_INCLUDED
#define CONF_TRX_ACCESS_H_INCLUDED

#include <FreeRTOS.h>
#include <lpc43xx.h>
#include <lpc43xx_scu.h>
#include <lpc43xx_gpio.h>
#include <lpc43xx_ssp.h>

//#define AT86RFX_SPI_BAUDRATE            (2250000UL)
#define AT86RFX_SPI_BAUDRATE            (4000000UL)

#define AT86RFX_SPI                     LPC_SSP0
#define AT86RFX_IRQn                    PIN_INT3_IRQn
#define AT86RFX_INT_PRIORITY            (configMAX_SYSCALL_INTERRUPT_PRIORITY)

/**
 * GPIO pins.
 */
#define CS_HIGH()                       GPIO_SetValue(7, 1 << 16)
#define CS_LOW()                        GPIO_ClearValue(7, 1 << 16)
#define RST_HIGH()                      GPIO_SetValue(3, 1 << 1)
#define RST_LOW()                       GPIO_ClearValue(3, 1 << 1)
#define SLP_TR_HIGH()                   GPIO_SetValue(5, 1 << 15)
#define SLP_TR_LOW()                    GPIO_ClearValue(5, 1 << 15)
#define IRQ_PINGET()                    (GPIO_ReadValue(3) & (1 << 0))

#define CSD_HIGH()                      GPIO_SetValue(5, 1 << 25)
#define CSD_LOW()                       GPIO_ClearValue(5, 1 << 25)
#define CPS_HIGH()                      GPIO_SetValue(5, 1 << 22)
#define CPS_LOW()                       GPIO_ClearValue(5, 1 << 22)
#define ASLECT_HIGH()                   GPIO_SetValue(3, 1 << 9)
#define ASLECT_LOW()                    GPIO_ClearValue(3, 1 << 9)

// Initialize interrupt P6_1 => GPIO3[0]
#define AT86RFX_INTC_INIT()          do { \
		NVIC_DisableIRQ(AT86RFX_IRQn); \
		LPC_SCU->PINTSEL0 &= (0x00 << 29); \
		LPC_SCU->PINTSEL0 |= 3 << 29; \
		LPC_GPIO_PIN_INT->ISEL = 0; \
		LPC_GPIO_PIN_INT->SIENR = 1 << 3; \
		LPC_GPIO_PIN_INT->IST = 1 << 3; \
		NVIC_SetPriority(PIN_INT3_IRQn, AT86RFX_INT_PRIORITY); \
	} while (0)

#define AT86RFX_ISR()               void GPIO3_IRQHandler(void)

/** Enables the transceiver main interrupt. */
#define ENABLE_TRX_IRQ()            NVIC_EnableIRQ(AT86RFX_IRQn)

/** Disables the transceiver main interrupt. */
#define DISABLE_TRX_IRQ()           NVIC_DisableIRQ(AT86RFX_IRQn)

/** Clears the transceiver main interrupt. */
#define CLEAR_TRX_IRQ()             NVIC_ClearPendingIRQ(AT86RFX_IRQn);

/*
 * This macro saves the trx interrupt status and disables the trx interrupt.
 */
#define ENTER_TRX_REGION()          DISABLE_TRX_IRQ()

/*
 *  This macro restores the transceiver interrupt status
 */
#define LEAVE_TRX_REGION()          ENABLE_TRX_IRQ()

#endif /* CONF_TRX_ACCESS_H_INCLUDED */
