/**********************************************************************
* @file			spi.c
* @brief		Contains functions support for SPI firmware on LPC4357
* @version	1.0
* @date			September 23, 2015
* @author		Martha Cuenca
**********************************************************************/

#include "spi.h"

/*******************************************************************************
 *	spi_cpha - clock phase, should be:
 *		 SSP_CPHA_FIRST: first clock edge
 *		 SSP_CPHA_SECOND: second clock edge
 *	spi_cpol - clock polarity, should be:
 *		 SSP_CPOL_HI: high level
 *		 SSP_CPOL_LO: low level
 * 	spi_fclk - clock rate in Hz
 *	spi_databit - databit number, should be:
 * 		 SSP_DATABIT_x, where x is in range from 4 - 16
 *	spi_mode - SPI mode, should be:
 *		 SSP_MASTER_MODE: Master mode
 *		 SSP_SLAVE_MODE: Slave mode
 *******************************************************************************/

/*****************************************************************************//**
 * @brief	Configures SPI communication between SPIx device and LPC4357
 * @return	SUCCESS when complete
 *******************************************************************************/
Status SPI_Init(LPC_SSPn_Type *SPIx, uint32_t spi_cpha, uint32_t spi_cpol,
	uint32_t spi_fclk, int32_t spi_databit, uint32_t spi_mode)
{
	
	SSP_CFG_Type SPI_ConfigStruct;
	
	// Initialize the SPIx peripheral
	SPI_ConfigStruct.CPHA = spi_cpha;
	SPI_ConfigStruct.CPOL = spi_cpol;
	SPI_ConfigStruct.ClockRate = spi_fclk;				
	SPI_ConfigStruct.Databit = spi_databit;
	SPI_ConfigStruct.Mode = spi_mode;
	SPI_ConfigStruct.FrameFormat = SSP_FRAME_SPI;
	SSP_Init(SPIx, &SPI_ConfigStruct);
	
	// TBD: Enable DMA function
	//SSP_DMACmd(SPIx, SSP_DMA_TX, ENABLE);
	//SSP_DMACmd(SPIx, SSP_DMA_RX, ENABLE);
	
	// Enable SPIx device operation
	SSP_Cmd(SPIx, ENABLE);
	
	return SUCCESS;
}

/*****************************************************************************//**
 * @brief		Deintializes SPI communication SPIx device and LPC4357
 * @param[in]	SPIx - peripheral selected
 * @return	SUCCESS when complete
 *******************************************************************************/
Status SPI_DeInit(LPC_SSPn_Type *SPIx)
{
	// Disable SPIx operation
	SSP_Cmd(SPIx, DISABLE);
	
	// De-initializes the SPIx peripheral registers to their default reset values
	SSP_DeInit(SPIx);
	
	return SUCCESS;
}

 /*****************************************************************************//**
 * @brief		Sends byte over SPI bus
 * @param[in]	SPIx - peripheral selected
 *				Byte - byte to be sent
 * @return	none
 *******************************************************************************/
void SPI_SendByte(LPC_SSPn_Type *SPIx, uint8_t Byte)
{
	char dummy;
	dummy = dummy;
	while (!(SPIx->SR & SSP_SR_TNF));	  // wait until TX FIFO is not full
	SPIx->DR = Byte;
	while (!(SPIx->SR & SSP_SR_TFE));	  // wait until TX FIFO is empty
	while (!(SPIx->SR & SSP_SR_RNE));	  // wait until RX FIFO is not empty
	dummy = SPIx->DR;
}

 /*****************************************************************************//**
 * @brief		Receives byte from SPI bus
 * @param[in]	SPIx  - peripheral selected
 *						pData - pointer to buffer where received byte will be stored
 * @return	none
 *******************************************************************************/
void SPI_ReceiveByte(LPC_SSPn_Type *SPIx, uint8_t *pData)
{
	while (!(SPIx->SR & SSP_SR_TNF));	  // wait until TX FIFO is not full
	SPIx->DR = 0x00;		 			  // Send out NOP to initiate SCLK
	while (!(SPIx->SR & SSP_SR_TFE));	  // wait until TX FIFO is empty
	while (!(SPIx->SR & SSP_SR_RNE));	  // wait until RX FIFO is not empty
	*pData = SPIx->DR;					  // Capture the receive buffer
}

 /*****************************************************************************//**
 * @brief		Send and receive byte from SPI bus
 * @param[in]	SPIx  - peripheral selected
 *						pData - pointer to buffer where received byte will be stored
 * @return	none
 *******************************************************************************/
void SPI_TransferByte(LPC_SSPn_Type *SPIx, uint8_t Byte, uint8_t *pData)
{
	while (!(SPIx->SR & SSP_SR_TNF));	  // wait until TX FIFO is not full
	SPIx->DR = Byte;
	while (!(SPIx->SR & SSP_SR_TFE));	  // wait until TX FIFO is empty
	while (!(SPIx->SR & SSP_SR_RNE));	  // wait until RX FIFO is not empty
	*pData = SPIx->DR;					  // Capture the receive buffer
}
