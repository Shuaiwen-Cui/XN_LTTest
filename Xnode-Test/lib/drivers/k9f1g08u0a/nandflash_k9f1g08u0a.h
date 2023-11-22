/**
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2013 Embest Tech. Co., Ltd.</center></h2>
  * @file    nandflash_k9f1g08u0a.h
  * @author  CMP Team
  * @version V1.0.0
  * @date    28-February-2013
  * @brief   The header file contains all macro definitions and function
  *          prototypes support for external Nand Flash SamSung K9F1G08U0A  
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, Embest SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
  * OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
  * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
  * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
  */
#ifndef __NANDFLASH_K9F1G08U0A_H__
#define __NANDFLASH_K9F1G08U0A_H__

/* Includes ------------------------------------------------------------------*/
#include "lpc_types.h"
#include "lpc43xx_scu.h"
#include "lpc43xx_cgu.h"
#include <xnode.h>


/** @defgroup Type
* @{
*/
/** NandFlash executive state*/        
typedef enum 
{
  NANDFLASH_STATUS_SUCCESS          = 0,     /*success*/
  NANDFLASH_STATUS_PAGE_OUT,                 /**/
  NANDFLASH_STATUS_BYTE_OUT,					
  NANDFLASH_STATUS_BLOCK_OUT,
  NANDFLASH_STATUS_PROG_ERROR,
  NANDFLASH_STATUS_ERASE_ERROR,
  NANDFLASH_STATUS_TIME_OUT
} NANDFLASH_STATUS_TypeDef;
/**
* @}
*/ 

/** @defgroup Macros
* @{
*/
/* nandflsh read for busy pin */
#define  NANDFLASH_RDY_PORT                  0xa
#define  NANDFLASH_RDY_PIN                   4
#define  NANDFLASH_RDY_GPIO_PORT             5
#define  NANDFLASH_RDY_GPIO_PIN              19

/* NANDFlash controller define */
#define EMC_CS0                              0
#define NANDFLASH_CS                         EMC_CS0

#if (NANDFLASH_CS == EMC_CS0)
#define K9F1G_CLE                            ((volatile uint8_t *)0x1C020000)
#define K9F1G_ALE                            ((volatile uint8_t *)0x1C010000)
#define K9F1G_DATA                           ((volatile uint8_t *)0x1C000000)
#endif /*#if (NANDFLASH_CS == EMC_CS0)*/

/* tmieout define */
#define NANDFLASH_TIME_OUT_READ              1200000    /* 12us  */
#define NANDFLASH_TIME_OUT_PROG              20000000   /* 200us */
#define NANDFLASH_TIME_OUT_ERASE             200000000  /* 2ms   */
#define NANDFLASH_TIME_OUT_RESET             500000     /* 500us */

// total 1024 blocks in a device
#define NANDFLASH_NUMOF_BLOCK                1024
#define NANDFLASH_INVALIDBLOCK_CHECK_COLUMM  2048		// Source: https://github.com/RT-Thread/realboard-lpc4088/blob/master/software/lpcware_lpc408x/BoardSupport/nandflash_k9f1g08u0a.h AND  https://www.dataman.com/media/datasheet/Samsung/K9FxG08xxA.pdf

// total pages in a block
#define NANDFLASH_PAGE_PER_BLOCK             64
#define NANDFLASH_RW_PAGE_SIZE               2048		
#define NANDFLASH_SPARE_SIZE                 64 
#define NANDFLASH_PAGE_FSIZE                 (NANDFLASH_RW_PAGE_SIZE + NANDFLASH_SPARE_SIZE)
#define NANDFLASH_BLOCK_RWSIZE               (NANDFLASH_RW_PAGE_SIZE * NANDFLASH_PAGE_PER_BLOCK)
#define NANDFLASH_BLOCK_FSIZE                (NANDFLASH_PAGE_FSIZE * NANDFLASH_PAGE_PER_BLOCK)
#define NANDFLASH_NUMOF_PAGE                 (NANDFLASH_PAGE_PER_BLOCK * NANDFLASH_NUMOF_BLOCK)


#define K9FXX_ID                             0xECF10000	/* Byte 3 and 2 only */

#define K9FXX_READ_1                         0x00
#define K9FXX_READ_2                         0x30
#define K9FXX_READ_ID                        0x90
#define K9FXX_RESET                          0xFF
#define K9FXX_BLOCK_PROGRAM_1                0x80
#define K9FXX_BLOCK_PROGRAM_2                0x10
#define K9FXX_BLOCK_ERASE_1                  0x60
#define K9FXX_BLOCK_ERASE_2                  0xD0
#define K9FXX_READ_STATUS                    0x70

#ifdef FRA	// each of the parameters are written in a different block (so that they can be updated separatedly)
	#define NANDFLASH_BLOCK_DATACFG     										1005 
	#define NANDFLASH_BLOCK_GATEWAY_DATATOSENDCOUNTCFG      1006 
	#define NANDFLASH_BLOCK_SENSOR_TOSENDCFG      					1007
	#define NANDFLASH_BLOCK_UNIXALARMTIME										1008
	#define NANDFLASH_BLOCK_NAND2SDATTEMP										1009
	
	#define NANDFLASH_BLOCK_DATALIMIT							    			900
	#define NANDFLASH_PAGE       										32
int write_tosend(bool istosend);
#endif

#define NANDFLASH_BLOCK_XNODECFG      1000 
#define NANDFLASH_BLOCK_TRIGGERSENSINGCFG      1001 
#define NANDFLASH_BLOCK_TRIGGERSENSINGDEFAULTCFG 1002
#define NANDFLASH_BLOCK_TRIGSENCFG      1003
#define NANDFLASH_BLOCK_DUTYCYCLEADXLCFG      1004
#define NANDFLASH_PAGE       32

/**
* @}
*/ 

/** @defgroup Declare function
* @{
*/


void NandFlash_Init(void);
NANDFLASH_STATUS_TypeDef NandFlash_ReadId(uint8_t *pBuffer);
NANDFLASH_STATUS_TypeDef NandFlash_Reset(void);
NANDFLASH_STATUS_TypeDef NandFlash_ReadStatus(uint32_t Cmd);
NANDFLASH_STATUS_TypeDef NandFlash_BlockErase(uint32_t blockNum);
NANDFLASH_STATUS_TypeDef NandFlash_PageProgram(uint8_t *pBuffer, uint32_t PageAddr, uint32_t ByteAddr, uint32_t NumToWrite);
NANDFLASH_STATUS_TypeDef NandFlash_PageRead(uint8_t *pBuffer, uint32_t PageAddr, uint32_t ByteAddr, uint32_t NumToRead);
Bool NandFlash_ValidBlockCheck( void );
NANDFLASH_STATUS_TypeDef NandFlash_WaitForReady(uint32_t timeout);
NANDFLASH_STATUS_TypeDef NandFlash_WaitForBusy(void);
NANDFLASH_STATUS_TypeDef NandFlash_WaitForBusy_READ(void);
int SDCard_Reset(void);
bool read_iswaitdeploy(void);
int read_tosend(void);
int write_tosend(bool istosend);
/**
* @}
*/ 
#endif /*__NANDFLASH_K9F1G08U0A_H__*/
/************** COPYRIGHT 2013 Embest Tech. Co., Ltd.*****END OF FILE****/
