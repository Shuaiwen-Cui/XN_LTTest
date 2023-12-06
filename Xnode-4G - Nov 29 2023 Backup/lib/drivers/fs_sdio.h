/**
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2013 Embest Tech. Co., Ltd.</center></h2>
  * @file    fs_sdio.h
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
#ifndef __FS_SDIO_H__
#define __FS_SDIO_H__

#include "lpc43xx.h"
#include "diskio.h"

/* I've found it's easier to understand the pullup/pulldown defines
   seperate instead of using the combo MD_* macros */
#define MD_ENAB_PD (1<<3) /* Enable pull down resistor at pad */
#define MD_DIS_PU (1<<4) /* Disable pullup resistor at pad */

/* SD clock/data pins have fast slew rate, not glitch filter,
   buffered input, and no pulldown or pullup. Note that the board
   already has pullups on the necessary SD signals */
#define SDFASTINOUTPIN (MD_DIS_PU | MD_EZI | MD_EHS | MD_ZI)



#endif /*__FS_SDIO_H__*/
