/**********************************************************************
* $Id$		debug_frmwrk.c		2011-06-02
*//**
* @file		debug_frmwrk.c
* @brief	Contains some utilities that used for debugging through UART
* @version	1.0
* @date		02. June. 2011
* @author	NXP MCU SW Application Team
*
* Copyright(C) 2011, NXP Semiconductor
* All rights reserved.
*
***********************************************************************
* Software that is described herein is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
* NXP Semiconductors assumes no responsibility or liability for the
* use of the software, conveys no license or title under any patent,
* copyright, or mask work right to the product. NXP Semiconductors
* reserves the right to make changes in the software without
* notification. NXP Semiconductors also make no representation or
* warranty that such application will be suitable for the specified
* use without further testing or modification.
* Permission to use, copy, modify, and distribute this software and its
* documentation is hereby granted, under NXP Semiconductors’
* relevant copyright in the software, without fee, provided that it
* is used in conjunction with NXP Semiconductors microcontrollers.  This
* copyright, permission, and disclaimer notice must appear in all copies of
* this code.
**********************************************************************/

/* Peripheral group ----------------------------------------------------------- */
/** @addtogroup DEBUG_FRMWRK
 * @{
 */

#ifndef _DEBUG_FRMWRK_
#define _DEBUG_FRMWRK_

/* Includes ------------------------------------------------------------------- */
#include "debug_frmwrk.h"
#include "lpc43xx_scu.h"
#include <FreeRTOS.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "sdmalloc.h"
#include "vcom.h"

/* Debug framework */

void (*_db_msg)(const void *s);
void (*_db_msg_)(const void *s);
void (*_db_char)(uint8_t ch);
void (*_db_dec)(uint8_t decn);
void (*_db_dec_16)(uint16_t decn);
void (*_db_dec_32)(uint32_t decn);
void (*_db_hex)(uint8_t hexn);
void (*_db_hex_16)(uint16_t hexn);
void (*_db_hex_32)(uint32_t hexn);
uint8_t (*_db_get_char)(void);
uint8_t (*_db_get_char_nonblocking)(void);

#ifdef USB
#define BUFFER_SIZE 512
static char buffer[BUFFER_SIZE];
static bool busy = false;
#endif

/*********************************************************************//**
 * @brief		Puts a character to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	ch		Character to put
 * @return		None
 **********************************************************************/
void PutChar (uint8_t ch)
{
#ifdef USB
	VCOM_PutChar(ch);
#endif
}


/*********************************************************************//**
 * @brief		Get a character to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @return		character value that returned
 **********************************************************************/
uint8_t GetChar (void)
{
#ifdef USB
	return (uint8_t)VCOM_GetChar();
#else
	return 0;
#endif
}

/*********************************************************************//**
 * @brief		Get a character to UART port in non-blocking mode
 * @param[in]	     UARTx	Pointer to UART peripheral
  * @param[out]	c	character value that returned
 * @return		TRUE (there is a character for procesisng)/FALSE
 **********************************************************************/

uint8_t GetCharInNonBlock(void)
{
	return GetChar();
}
/*********************************************************************//**
 * @brief		Puts a string to UART port
 * @param[in]	UARTx 	Pointer to UART peripheral
 * @param[in]	str 	string to put
 * @return		None
 **********************************************************************/
void Puts(const void *str)
{
#ifdef USB
	VCOM_Write((char *)str, strlen((char *)str));
#endif
}


/*********************************************************************//**
 * @brief		Puts a string to UART port and print new line
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	str		String to put
 * @return		None
 **********************************************************************/
void Puts_(const void *str)
{
	Puts (str);
	Puts ("\n\r");
}


/*********************************************************************//**
 * @brief		Puts a decimal number to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	decnum	Decimal number (8-bit long)
 * @return		None
 **********************************************************************/
void PutDec(uint8_t decnum)
{
	uint8_t c1=decnum%10;
	uint8_t c2=(decnum/10)%10;
	uint8_t c3=(decnum/100)%10;
	PutChar('0'+c3);
	PutChar('0'+c2);
	PutChar('0'+c1);
}

/*********************************************************************//**
 * @brief		Puts a decimal number to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	decnum	Decimal number (8-bit long)
 * @return		None
 **********************************************************************/
void PutDec16(uint16_t decnum)
{
	uint8_t c1=decnum%10;
	uint8_t c2=(decnum/10)%10;
	uint8_t c3=(decnum/100)%10;
	uint8_t c4=(decnum/1000)%10;
	uint8_t c5=(decnum/10000)%10;
	PutChar('0'+c5);
	PutChar('0'+c4);
	PutChar('0'+c3);
	PutChar('0'+c2);
	PutChar('0'+c1);
}

/*********************************************************************//**
 * @brief		Puts a decimal number to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	decnum	Decimal number (8-bit long)
 * @return		None
 **********************************************************************/
void PutDec32(uint32_t decnum)
{
	uint8_t c1=decnum%10;
	uint8_t c2=(decnum/10)%10;
	uint8_t c3=(decnum/100)%10;
	uint8_t c4=(decnum/1000)%10;
	uint8_t c5=(decnum/10000)%10;
	uint8_t c6=(decnum/100000)%10;
	uint8_t c7=(decnum/1000000)%10;
	uint8_t c8=(decnum/10000000)%10;
	uint8_t c9=(decnum/100000000)%10;
	uint8_t c10=(decnum/1000000000)%10;
	PutChar('0'+c10);
	PutChar('0'+c9);
	PutChar('0'+c8);
	PutChar('0'+c7);
	PutChar('0'+c6);
	PutChar('0'+c5);
	PutChar('0'+c4);
	PutChar('0'+c3);
	PutChar('0'+c2);
	PutChar('0'+c1);
}

/*********************************************************************//**
 * @brief		Puts a hex number to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	hexnum	Hex number (8-bit long)
 * @return		None
 **********************************************************************/
void PutHex (uint8_t hexnum)
{
	uint8_t nibble, i;

	Puts("0x");
	i = 1;
	do {
		nibble = (hexnum >> (4*i)) & 0x0F;
		PutChar((nibble > 9) ? ('A' + nibble - 10) : ('0' + nibble));
	} while (i--);
}


/*********************************************************************//**
 * @brief		Puts a hex number to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	hexnum	Hex number (16-bit long)
 * @return		None
 **********************************************************************/
void PutHex16 (uint16_t hexnum)
{
	uint8_t nibble, i;

	Puts("0x");
	i = 3;
	do {
		nibble = (hexnum >> (4*i)) & 0x0F;
		PutChar((nibble > 9) ? ('A' + nibble - 10) : ('0' + nibble));
	} while (i--);
}

/*********************************************************************//**
 * @brief		Puts a hex number to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	hexnum	Hex number (32-bit long)
 * @return		None
 **********************************************************************/
void PutHex32 (uint32_t hexnum)
{
	uint8_t nibble, i;

	Puts("0x");
	i = 7;
	do {
		nibble = (hexnum >> (4*i)) & 0x0F;
		PutChar((nibble > 9) ? ('A' + nibble - 10) : ('0' + nibble));
	} while (i--);
}

/*********************************************************************//**
 * @brief		print function that supports format as same as printf()
 * 				function of <stdio.h> library
 * @param[in]	format formated string to be print
 * @return		None
 **********************************************************************/
void  lpc_printf (const  char *format, ...)
{
#ifdef USB
  va_list     vArgs;

	while (busy) { portYIELD(); }
	busy = true;
	
  va_start(vArgs, format);
  vsnprintf((char *)buffer, BUFFER_SIZE, (char const *)format, vArgs);
  va_end(vArgs);
  
	_DBG(buffer);
	
	busy = false;
#endif
}

/*********************************************************************//**
 * @brief		Initialize Debug frame work through initializing UART port
 * @param[in]	None
 * @return		None
 **********************************************************************/
void debug_frmwrk_init_clk(uint32_t Clock_Speed)
{
	_db_msg	= Puts;
	_db_msg_ = Puts_;
	_db_char = PutChar;
	_db_hex = PutHex;
	_db_hex_16 = PutHex16;
	_db_hex_32 = PutHex32;
	_db_dec = PutDec;
	_db_dec_16 = PutDec16;
	_db_dec_32 = PutDec32;
	_db_get_char = GetChar;
	_db_get_char_nonblocking = GetCharInNonBlock;
}

#endif /* _DEBUG_FRMWRK_ */

/**
 * @}
 */

