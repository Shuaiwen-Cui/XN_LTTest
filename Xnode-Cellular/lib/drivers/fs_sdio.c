/**
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2013 Embest Tech. Co., Ltd.</center></h2>
  * @file    fs_sdio.c
  * @author  CMP Team
  * @version V1.0.0
  * @date    28-February-2013
  * @brief   Drivers for SD
  *         
  ******************************************************************************
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
#include <FreeRTOS.h>
#include "fs_sdio.h"
#include "lpc43xx_cgu.h"
#include "lpc43xx_scu.h"
#include "lpc43xx_timer.h"
#include "lpc_sdmmc.h"
#include "lpc43xx_sdif.h"
#include "lpc43xx_sdmmc.h"

static volatile  int32_t  sdio_wait_exit = 0;
struct   _mci_card_struct sdcardinfo;
static  uint32_t   sdmmc_irq            (uint32_t rinsts);
static  void       sdmmc_waitms         (uint32_t time);
static  uint32_t   sdmmc_irq_driven_wait(uint32_t bits);
static  void       sdmmc_setup_wakeup   (uint32_t bits);


DSTATUS disk_initialize (
	BYTE drv)		/* Physical drive number (0) */
{
  uint32_t sdio_clk;
  uint32_t timerout = 0x80000000;
	
  if (drv) {
    return RES_PARERR;
  }
  /* Setup muxing for SD interface */
  scu_pinmux(0xc ,8 , MD_DIS_PU | MD_EZI, FUNC7);  /* Pc.8 SDIO CD */        //checked
  scu_pinmux(0xc ,7 , SDFASTINOUTPIN, FUNC7);      /* Pc.7 SDIO D3 */        //checked
  scu_pinmux(0xc ,6 , SDFASTINOUTPIN, FUNC7);      /* Pc.6 SDIO D2 */        //checked
  scu_pinmux(0xc ,5 , SDFASTINOUTPIN, FUNC7);      /* Pc.5 SDIO D1 */        //checked
  scu_pinmux(0xc ,4 , SDFASTINOUTPIN, FUNC7);      /* Pc.4 SDIO D0 */        //checked
  scu_pinmux(0xc ,0 , MD_DIS_PU | MD_EHS, FUNC7);  /* Pc.0 SDIO clock */     //checked
  scu_pinmux(0xc ,10, SDFASTINOUTPIN, FUNC7);      /* Pc.10 SDIO command */  //checked
  CGU_EntityConnect(CGU_CLKSRC_PLL1, CGU_BASE_SDIO);

  /* The SDIO driver needs to know the SDIO clock rate */
  sdio_clk = CGU_GetPCLKFrequency(CGU_PERIPHERAL_SDIO);
 /* This init sdio with sdio_clk */
  sdif_init(sdio_clk, sdmmc_irq);
  /* Wait for a card to be inserted (note CD is not on the
     SDMMC power rail and can be polled without enabling
     SD slot power */
	// Kirill -- FIXME: add SDIO CD line to radio board
  //while (!(sdif_card_ndetect()) && (timerout--));
  if (!timerout) {
    return RES_ERROR;
  }
  sdif_power_onoff(0);
  /* Enumerate the card once detected. Note this function may
     block for a little while. */
  if (!sdmmc_acquire(sdmmc_setup_wakeup, sdmmc_irq_driven_wait, sdmmc_waitms, &sdcardinfo)) {
    return RES_ERROR;
  }
  

  return RES_OK;
}

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive number (0) */
	BYTE ctrl,		/* Control code */
	void *buff)		/* Buffer to send/receive control data */
{
  DRESULT res = RES_OK;
	
  if (drv) {
    return RES_PARERR;
  }

  switch (ctrl) {
    case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
      *(DWORD*)buff = sdcardinfo.device_size / 512;
      res = RES_OK;
      break;

    case GET_SECTOR_SIZE :	/* Get R/W sector size (WORD) */
      *(WORD*)buff = 512;	//512;
      res = RES_OK;
      break;

    case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
      *(DWORD*)buff = sdcardinfo.block_len;
      res = RES_OK;
      break;
  }
  
  return res;
}


DRESULT disk_read (
	BYTE drv,			/* Physical drive number (0) */
	BYTE *buff,			/* Pointer to the data buffer to store read data */
	DWORD sector,		/* Start sector number (LBA) */
	BYTE count)			/* Sector count (1..255) */
{
  if (drv || !count) return RES_PARERR;
  if (sdmmc_read_blocks((void*)buff, sector, sector + count - 1) == 0) {
    return RES_ERROR;
  }
  return RES_OK;
}
DSTATUS disk_status (
	BYTE drv)		/* Physical drive number (0) */
{
  return RES_OK;
}
#if _READONLY == 0
DRESULT disk_write (
	BYTE drv,			/* Physical drive number (0) */
	const BYTE *buff,	/* Pointer to the data to be written */
	DWORD sector,		/* Start sector number (LBA) */
	BYTE count)			/* Sector count (1..255) */
{
  if (drv || !count) return RES_PARERR;

  if (sdmmc_write_blocks((void*)buff, sector, sector + count - 1) == 0) {
    return RES_ERROR;
  }
  /* Wait for card program to finish */
  while (sdmmc_get_state() != SDMMC_TRAN_ST);
  return RES_OK;
}
#endif
/*-----------------------------------------------------------------------*/
/* Get current time                                                      */
/*-----------------------------------------------------------------------*/

DWORD get_fattime ()
{
	return	((2013UL-1980) << 25)	      // Year = 2013
			| (2UL << 21)	      // Month = Feb
			| (9UL << 16)	      // Day = 9
			| (22U << 11)	      // Hour = 22
			| (30U << 5)	      // Min = 30
			| (0U >> 1)	      // Sec = 0
			;
}

/*******************************************************************************
* @brief        SDIO interrupt handler callback
* @param[in]    rinsts Optional input parameter
* @return       Return value is 0, not currently used
*******************************************************************************/
static uint32_t sdmmc_irq(uint32_t rinsts)
{
  /* Set wait exit flag to tell wait function we are ready. In an RTOS,
    this would trigger wakeup of a thread waiting for the IRQ. */
  NVIC_DisableIRQ(SDIO_IRQn);
  sdio_wait_exit = 1;

  return 0;
}

/*********************************************************************//**
* @brief        Sets up the SD event driven wakeup
* @param[in]    bits Status bits to poll for command completion
* @return       None
**********************************************************************/
static void sdmmc_setup_wakeup(uint32_t bits)
{
  /* Wait for IRQ - for an RTOS, you would pend on an event here
    with a IRQ based wakeup. */
  NVIC_ClearPendingIRQ(SDIO_IRQn);
  sdio_wait_exit = 0;
  LPC_SDMMC->INTMASK = bits;
	//NVIC_SetPriority(SDIO_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(SDIO_IRQn);
}

/*********************************************************************//**
* @brief        A better wait callback for SDMMC driven by the IRQ flag
* @param[in]    bits Status bits to poll for command completion
* @return       0 on success, or failure condition (-1)
**********************************************************************/
static uint32_t sdmmc_irq_driven_wait(uint32_t bits)
{
  uint32_t status;

  /* Wait for event, would be nice to have a timeout, but keep it
     simple */
  while (sdio_wait_exit == 0);

  /* Get status and clear interrupts */
  status = LPC_SDMMC->RINTSTS;
  LPC_SDMMC->RINTSTS = status;
  LPC_SDMMC->INTMASK = 0;

  return status;
}

/*********************************************************************//**
* @brief        Delay callback for timed SDIF/SDMMC functions
* @param[in]    time Number of milliSeconds to wait
* @return       None
**********************************************************************/
void sdmmc_waitms(uint32_t time)
{
  /* In an RTOS, the thread would sleep allowing other threads to
     run. For standalone operation, we just spin on a timer */
  TIM_Waitus(time * 1000);
}
