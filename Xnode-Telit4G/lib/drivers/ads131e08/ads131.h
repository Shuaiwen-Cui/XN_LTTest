/**********************************************************************
* @file			ads131.h
* @brief		Contains macro definitions and function prototypes
*						support for ADS131E08 ADC
* @version	1.0
* @date			September 23, 2015
* @author		Martha Cuenca
**********************************************************************/

#ifndef _ADS131_H 
#define _ADS131_H

#include "lpc_types.h"

#define ADS131_INT_PRIORITY	(configMAX_SYSCALL_INTERRUPT_PRIORITY)					// interrupt priority for ADC
#define ADS131_NCHANNELS	8
#define ADS131_VREF			((float) 2.4)					// reference voltage 2.4 V

extern volatile uint8_t ADS131_DRDY;
extern volatile uint32_t ADS131_DATA_COUNTER;				// for counting data

// Channel types
typedef enum {
	NORMAL, SHORTED, SUPPLY, TEMPERATURE, TEST, CH_OFF
} ADS131_chType;

/** @brief	ADC single channel configuration structure
*	  @param	channelAdd - channel register address
*				powerDown - 1 (power-down) or 0 (normal operation)
*				gain - 1, 2, 4, 8 or 12
*				type - 8 bits that correspond to channel type
*				*pData - points to current position in channel data buffer
**/
typedef struct {
	uint8_t channelAdd;
	uint8_t powerDown;
	uint8_t gain;
	uint8_t type;
	int32_t *pData;
} CHANNEL_CFG_Type;

/** @brief ADC global acquisition configuration structure 
*		      (global settings across all channels)
*	  @param	fsclk - SPI serial clock, should be in Hz
*				Nbits	- # bits of resolution (16 or 24)
*				DR - data rate, in kSPS (either 1, 2, 4, 8, 16, 32, or 64)
*    			sensing - 0: not sensing; 1: sensing
**/
typedef struct {
	uint32_t fsclk;			
	uint8_t  Nbits;				
	uint8_t	 DR;	
	uint8_t	 sensing;
} ADS131_CFG_Type;

/*****************************************************************************//**
 * @brief		initialize ADC
 * @return	SUCCESS when complete
 *******************************************************************************/
Status ads131_init(void);

/*****************************************************************************//**
 * @brief		de-initialize ADC
 * @return	SUCCESS when complete
 *******************************************************************************/
Status ads131_deinit(void);

/*****************************************************************************//**
 * @brief		reads from ADC registers
 * @return	none
 *******************************************************************************/
void ads131_readReg(uint8_t StartAddress, uint8_t NumRegs, uint8_t *pData);

/*****************************************************************************//**
 * @brief		writes to ADC registers
 * @return	none
 *******************************************************************************/
void ads131_writeReg(uint8_t StartAddress, uint8_t NumRegs, uint8_t *pData);

/*****************************************************************************//**
 * @brief		powers on ADC (see datasheet p.33)
 * @return		SUCCESS when complete
 *******************************************************************************/
Status ads131_on(void);

/*****************************************************************************//**
 * @brief		powers off ADC 
 * @return	none
 *******************************************************************************/
void ads131_off(void);

/*****************************************************************************//**
 * @brief		puts ADC into standby mode
 *	(do NOT send any other command other than wakeup after this command is called) 
 * @return	none
 *******************************************************************************/
void ads131_standby(void);

/*****************************************************************************//**
 * @brief		wakes up ADC from standby mode
 * @return	none
 *******************************************************************************/
void ads131_wakeup(void);

/*****************************************************************************//**
 * @brief		  Sets up global acquisition parameters
 * @param[in]	ADCset - points to ADS_CFG_Type of ADC setting parameters
 *						DataRate - in kSPS
 * @return		SUCCESS when complete
 *******************************************************************************/
Status ads131_setupDAQ(ADS131_CFG_Type *ADCset, uint8_t DataRate);

/*****************************************************************************//**
 * @brief		Sets up channel acquisition parameters
 * @param[in]	channelNum		channel number
 *						powerDownYes	0: do not power down; 1: power down
 *						gainValue		PGA: 1, 2, 4, 8, or 12
 *						channelType	channel input type (see ADS_chType enum)
 * @return		SUCCESS when complete
 *******************************************************************************/
Status ads131_setupCh(CHANNEL_CFG_Type *chPointer, uint8_t channelNum,
                      uint8_t powerDownYes, uint8_t gainValue,
					  ADS131_chType channelType);

/*****************************************************************************//**
 * @brief		ADS131 DRDY interrupt handler
 * @return		none
 *******************************************************************************/
void GPIO7_IRQHandler(void);

/*****************************************************************************//**
* @brief	start sensing command
* @param[in]	ADCset - points to ADS_CFG_Type of ADC setting parameters
* @return	SUCCESS when complete
*******************************************************************************/
void ads131_startSensing(ADS131_CFG_Type *ADCset);

/*****************************************************************************//**
 * @brief		reads one data point from all channels from ADC (24-bit mode)
 * 				and reads the status register
 * @return		none
 *******************************************************************************/
Status ads131_readDataPt(int32_t *pData, int32_t *stat);

/*****************************************************************************//**
* @brief		stop sensing command
* @param[in]	ADCset - points to ADS_CFG_Type of ADC setting parameters
* @return		SUCCESS when complete
*******************************************************************************/
void ads131_stopSensing(ADS131_CFG_Type *ADCset);

/*****************************************************************************//**
 * @brief		Removes offset for all channels
 * @return		SUCCESS when complete
 *******************************************************************************/
Status ads131_removeOffset(void);

/*****************************************************************************//**
* @brief		turns square wave test signal on or off
* @param[in]	sigOn - 1 (turn on signal); 0 (turn off signal)
* @return		SUCCESS when complete
*******************************************************************************/
Status ads131_testSignal(uint8_t sigOn);

#endif /* _ADS131_H */
