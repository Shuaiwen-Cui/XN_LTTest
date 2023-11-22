/**********************************************************************
* $Id$		debug_frmwrk.h			2011-06-02
*//**
* @file		debug_frmwrk.h
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
/** @defgroup DEBUG_FRMWRK DEBUG FRAMEWORK
 * @ingroup LPC4300CMSIS_FwLib_Drivers
 * @{
 */

#ifndef DEBUG_FRMWRK_H_
#define DEBUG_FRMWRK_H_

/* Includes ------------------------------------------------------------------- */
#include <stdint.h>

#define _DBG(x)	               _db_msg(x)
#define _DBG_(x)               _db_msg_(x)
#define _DBC(x)                _db_char(x)
#define _DBD(x)                _db_dec(x)
#define _DBD16(x)              _db_dec_16(x)
#define _DBD32(x)              _db_dec_32(x)
#define _DBH(x)                _db_hex(x)
#define _DBH16(x)              _db_hex_16(x)
#define _DBH32(x)              _db_hex_32(x)
#define _DG                    _db_get_char()
#define _DG_NONBLOCK           _db_get_char_nonblocking()
void  lpc_printf (const  char *format, ...);

extern void (*_db_msg)(const void *s);
extern void (*_db_msg_)(const void *s);
extern void (*_db_char)(uint8_t ch);
extern void (*_db_dec)(uint8_t decn);
extern void (*_db_dec_16)(uint16_t decn);
extern void (*_db_dec_32)(uint32_t decn);
extern void (*_db_hex)(uint8_t hexn);
extern void (*_db_hex_16)(uint16_t hexn);
extern void (*_db_hex_32)(uint32_t hexn);
extern uint8_t (*_db_get_char)(void);
extern uint8_t (*_db_get_char_nonblocking)(void);

void PutChar (uint8_t ch);
void Puts(const void *str);
void Puts_(const void *str);
void PutDec(uint8_t decnum);
void PutDec16(uint16_t decnum);
void PutDec32(uint32_t decnum);
void PutHex (uint8_t hexnum);
void PutHex16 (uint16_t hexnum);
void PutHex32 (uint32_t hexnum);
uint8_t GetChar (void);
uint8_t GetCharInNonBlock(void);
#define debug_frmwrk_init() debug_frmwrk_init_clk(0)
void debug_frmwrk_init_clk(uint32_t Clock_Speed);

#endif /* DEBUG_FRMWRK_H_ */

/**
 * @}
 */

