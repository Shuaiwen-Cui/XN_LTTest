/*
 * YAFFS: Yet Another Flash File System. A NAND-flash specific file system.
 *
 * Copyright (C) 2010-2011 Aleph One Ltd.
 *
 * Created by Charles Manning <charles@aleph1.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "yaffsHEADER.h"
#include "nanddrv.h"
#include "nand_chip.h"
#include "xnode_yaffs.h"


/** @defgroup Public variable
* @{
*/
/**
* @}
*/ 

#ifndef inline
#define inline __inline
#endif

int nanddrv_initialise(void)  //void NandFlash_Init(void)
{
  /*
    Init DATA Pin D0-D7 P1_7-----P1_14
  */                
  scu_pinmux(0x1,  7,  (MD_PLN_FAST), FUNC3);  /* P1_7: D0  */
  scu_pinmux(0x1,  8,  (MD_PLN_FAST), FUNC3);  /* P1_8: D1  */
  scu_pinmux(0x1,  9,  (MD_PLN_FAST), FUNC3);  /* P1_9: D2  */
  scu_pinmux(0x1,  10, (MD_PLN_FAST), FUNC3);  /* P1_10: D3 */
  scu_pinmux(0x1,  11, (MD_PLN_FAST), FUNC3);  /* P1_11: D4 */
  scu_pinmux(0x1,  12, (MD_PLN_FAST), FUNC3);  /* P1_12: D5 */
  scu_pinmux(0x1,  13, (MD_PLN_FAST), FUNC3);  /* P1_13: D6 */
  scu_pinmux(0x1,  14, (MD_PLN_FAST), FUNC3);  /* P1_14: D7 */
  /*
    Init ADDR Pin A16-A17  PD15-PD16
  */
  scu_pinmux(0xD,  16, (MD_PLN_FAST), FUNC2);  /* Pd_16: A16 */
  scu_pinmux(0xD,  15, (MD_PLN_FAST), FUNC2);  /* Pd_15: A17 */
                
  /*
    Init Control Pin
  */
  scu_pinmux(0x1, 3, MD_PLN_FAST, FUNC3); /* EMC_nOE */
  scu_pinmux(0x1, 6, MD_PLN_FAST, FUNC3); /* EMC_nWE */  
  scu_pinmux(0x1, 5, MD_PLN_FAST, FUNC3); /* EMC_nCS0*/

  /*
    Init RYD Pin
  */
  scu_pinmux(NANDFLASH_RDY_PORT, NANDFLASH_RDY_PIN, PUP_ENABLE | INBUF_ENABLE, FUNC4);

  /* Set up EMC Controller */
  /* 
   set
   ByteLane = 1; low level activate
   DataWidth = 8; DataWith = 8bit
   ExtendedWait = 0; no delay wait
   PageMode = 0;
  */
  LPC_EMC->STATICCONFIG0 = 0x80;
  
  /* set WAITWEN CCLK = 2 */
  LPC_EMC->STATICWAITWEN0 = 2;

  /* set WAITOEN CCLK = 2 */
  LPC_EMC->STATICWAITOEN0 = 2;

  LPC_EMC->STATICWAITWR0 = 0xf;  
  
  LPC_EMC->STATICWAITRD0 = 0xf;  
  
  LPC_EMC->STATICWAITTURN0 = 0xf;  
  
  LPC_EMC->STATICWAITPAG0 = 0xf;
  
  return 1;
}

static void nanddrv_send_addr(struct nand_chip *this, int page, int offset) 
{
	volatile uint8_t *pALE;
	pALE  = K9F1G_ALE;

	if(offset >= 0){
		*pALE =  (uint8_t)(offset & 0xFF);
		*pALE =  (uint8_t)((offset & 0xF00)>>8);
	}

	if(page >= 0){
		*pALE =  (uint8_t)(page & 0xFF);
		*pALE =  (uint8_t)((page & 0xFF00)>>8);	
	}
}

static void nanddrv_send_cmd(struct nand_chip *this, unsigned char cmd) // Changed to write value to *
{
  volatile uint8_t *pCLE;
  pCLE  = K9F1G_CLE;
  *pCLE = cmd;
}


static inline int nanddrv_status_pass(unsigned char status) // this seems right, no need to fix
{
	/* If bit 0 is zero then pass, if 1 then fail */
	return (status & (1 << 0)) == 0;
}

/*   // Comment out to reduce compiler warning
static inline int nanddrv_status_busy(unsigned char status) // Fix to bit 5 using true ready/busy
{
	uint32_t tmp;
	// If bit 5 is zero then busy, if 1 then ready  // Use true ready/busy at bit 5. But this stucks (driver also doesn't use this)
	tmp = GPIO_ReadValue(NANDFLASH_RDY_GPIO_PORT) & (1 << NANDFLASH_RDY_GPIO_PIN);

	return (tmp == 0);
}


static unsigned char nanddrv_get_status(struct nand_chip *this, // Change function id to K9FXX_READ_STATUS
					int wait_not_busy)
{
//	unsigned char status;

//	nanddrv_send_cmd(this, K9FXX_READ_STATUS);
//	status = this->read_cycle(this) & 0xff;
//	if(!wait_not_busy)
//		return status;
//	while (nanddrv_status_busy(status)) {
//		status = this->read_cycle(this) & 0xff;
//	}
//	return status;
	return 0; 	// must change this as it calls another function, not suitable to check right after sending a function and wait for reading
}
*/
int nanddrv_read_tr(struct nand_chip *this, int page, struct nanddrv_transfer *tr, int n_tr)
{
	unsigned char status;
	int ncycles;

	if(n_tr < 1)
		return 0;
	do
	{
	nanddrv_send_cmd(this, 0x00); 
	nanddrv_send_addr(this, page, tr->offset); 
	nanddrv_send_cmd(this, 0x30); 
	}while(NandFlash_WaitForBusy_READ());
//	NandFlash_WaitForBusy();
//	status = nanddrv_get_status(this, 1);	
//	if(!nanddrv_status_pass(status))
//		return -1;
//	nanddrv_send_cmd(this, 0x30); // why 2 0x30? (reading end)
	
	status = NandFlash_WaitForReady(NANDFLASH_TIME_OUT_READ);
	if (status == NANDFLASH_STATUS_SUCCESS) {	
	while (1) {
		if(this->bus_width_shift == 0) {
			unsigned char *buffer = tr->buffer;

			ncycles = tr->nbytes;

			while (ncycles> 0) {
				*buffer = (unsigned char) *K9F1G_DATA;
				ncycles--;
				buffer++;
			}
		} else {
			unsigned short *buffer = (unsigned short *)tr->buffer;

			ncycles = tr->nbytes >> 1;
			while (ncycles> 0) {
				*buffer = (unsigned short) *K9F1G_DATA;	
				ncycles--;
				buffer++;
			}
		}
		n_tr--;
		tr++;
		if(n_tr < 1)
			break;
//		nanddrv_send_cmd(this, 0x05);
//		nanddrv_send_addr(this, -1, tr->offset);
//		nanddrv_send_cmd(this, 0xE0);
	}
}
	return 0;
}

/*
int nanddrv_read_tr(struct nand_chip *this, int page, struct nanddrv_transfer *tr, int n_tr)
{
	int ncycles;
	unsigned char *pBuffer = tr->buffer;
  volatile uint8_t *pCLE;
  volatile uint8_t *pALE;
  volatile uint8_t *pDATA;
	uint32_t PageAddr;
	uint32_t ByteAddr;
	uint32_t NumToRead;
  NANDFLASH_STATUS_TypeDef status;
	
  pCLE  = K9F1G_CLE;
  pALE  = K9F1G_ALE;
  pDATA = K9F1G_DATA;  
 	
	PageAddr = page;
	ByteAddr = tr->offset;
	
  *pCLE = K9FXX_READ_1;

  *pALE =  (uint8_t)(ByteAddr & 0xFF);
  *pALE =  (uint8_t)((ByteAddr & 0xF00)>>8);

  *pALE =  (uint8_t)(PageAddr & 0xFF);
  *pALE =  (uint8_t)((PageAddr & 0xFF00)>>8);

  *pCLE = K9FXX_READ_2;
  NumToRead = tr->nbytes;
  NandFlash_WaitForBusy();

  status = NandFlash_WaitForReady(NANDFLASH_TIME_OUT_READ);
  if (status == NANDFLASH_STATUS_SUCCESS) {
    while (NumToRead--) {
      *pBuffer++ = *pDATA;
    }
  //  status = NandFlash_WaitForReady(NANDFLASH_TIME_OUT_READ);	
  }
  
  return status;

}
*/
/*
 * Program page
 * Cmd: 0x80, 5-byte address, data bytes,  Cmd: 0x10, wait not busy // K9FXX_BLOCK_PROGRAM_1 = 0x80
 */
int nanddrv_write_tr(struct nand_chip *this, int page,
		struct nanddrv_transfer *tr, int n_tr)
{
	unsigned char status;
	int ncycles;

	if (n_tr < 1)
		return 0;

	nanddrv_send_cmd(this, K9FXX_BLOCK_PROGRAM_1);
	nanddrv_send_addr(this, page, tr->offset);
	while (1) {
		if(this->bus_width_shift == 0) {
			unsigned char *buffer = tr->buffer;
			ncycles = tr->nbytes;
			while (ncycles> 0) {
				*K9F1G_DATA = *buffer;
				ncycles--;
				buffer++;
			}
		} else {
			unsigned short *buffer = (unsigned short *)tr->buffer;

			ncycles = tr->nbytes >> 1;
			while (ncycles> 0) {
				*K9F1G_DATA = *buffer;
				ncycles--;
				buffer++;
			}
		}
		n_tr--;
		tr++;
		if (n_tr < 1)
			break;
//		nanddrv_send_cmd(this, 0x85);		// random data input???
//		nanddrv_send_addr(this, -1, tr->offset);
	}

//	if(this->power_check && this->power_check(this) < 0)
//		return -1;

	nanddrv_send_cmd(this, K9FXX_BLOCK_PROGRAM_2);
	//status = nanddrv_get_status(this, 1);
	//NandFlash_WaitForBusy();
	status = NandFlash_WaitForReady(NANDFLASH_TIME_OUT_READ);
  if (status == NANDFLASH_STATUS_SUCCESS) {
    status = NandFlash_ReadStatus(K9FXX_BLOCK_PROGRAM_1);
  }
	if(nanddrv_status_pass(status))
		return 0;
	return -1;
}

/*
 * Block erase
 * Cmd: 0x60, 3-byte address, cmd: 0xD0. Wait not busy.
 */
int nanddrv_erase(struct nand_chip *this, int block)
{
	unsigned char status;

 	nanddrv_send_cmd(this, 0x60);
	nanddrv_send_addr(this, block * this->pages_per_block, -1);

	if(this->power_check && this->power_check(this) < 0)
		return -1;

	nanddrv_send_cmd(this, 0xD0);
	//status = nanddrv_get_status(this, 1);
	NandFlash_WaitForBusy();
	status = NandFlash_WaitForReady(NANDFLASH_TIME_OUT_READ);

	if(nanddrv_status_pass(status))
		return 0;
	return -1;
}



