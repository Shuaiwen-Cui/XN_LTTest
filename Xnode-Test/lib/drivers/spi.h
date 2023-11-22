/**********************************************************************
* @file			spi.h
* @brief		Contains macro definitions and function prototypes
*						support for SPI firmware on LPC4357
* @version	1.0
* @date			September 23, 2015
* @author		Martha Cuenca
**********************************************************************/

#ifndef _SPI_H 
#define _SPI_H

#include "lpc43xx_ssp.h"

/*****************************************************************************//**
 * @brief		Configure SPI communication between SPIx device and LPC4357
 * @return	SUCCESS when complete
 *******************************************************************************/
Status SPI_Init(LPC_SSPn_Type *SPIx, uint32_t spi_cpha,
                uint32_t spi_cpol, uint32_t spi_fclk,
								int32_t spi_databit, uint32_t spi_mode);
												
/*****************************************************************************//**
 * @brief		Deintialize SPI communication SPIx device and LPC4357
 * @param[in]	SPIx - peripheral selected
 * @return	SUCCESS when complete
 *******************************************************************************/
Status SPI_DeInit(LPC_SSPn_Type *SPIx);

 /*****************************************************************************//**
 * @brief		Send byte over SPI bus
 * @param[in]	SPIx - peripheral selected
 *						Byte - byte to be sent
 * @return	none
 *******************************************************************************/
void SPI_SendByte(LPC_SSPn_Type *SPIx, uint8_t Byte);

 /*****************************************************************************//**
 * @brief		Receive byte from SPI bus
 * @param[in]	SPIx  - peripheral selected
 *						pData - pointer to buffer where received byte will be stored
 * @return	none
 *******************************************************************************/
void SPI_ReceiveByte(LPC_SSPn_Type *SPIx, uint8_t *pData);

 /*****************************************************************************//**
 * @brief		Send and receive byte from SPI bus
 * @param[in]	SPIx  - peripheral selected
 *						Byte - byte to be sent
 *						pData - pointer to buffer where received byte will be stored
 * @return	none
 *******************************************************************************/
void SPI_TransferByte(LPC_SSPn_Type *SPIx, uint8_t Byte, uint8_t *pData);
#endif /* _SPI_H */
