/**********************************************************************
* @file			ads131.c
* @brief		Contains functions support for ADS131E08 ADC
* @version		1.0
* @date			September 23, 2015
* @author		Martha Cuenca
**********************************************************************/

#include <xnode.h>
#undef SUCCESS
#include "ads131.h"						// important to include this file FIRST
#include "ads131_const.h"
#include "spi.h"
#include <lpc43xx.h>
#include <lpc43xx_scu.h>
#include <lpc43xx_cgu.h>
#include <lpc43xx_timer.h>

#include "FreeRTOS.h"
#include "task.h"

/**
* FSCLK must be 72,000,000/(multiple of 2) where 72e6 is the LPC4357 clk speed
*	Calculation of fsclk - datasheet p. 21
*	fsclk > (ADCsettings.Nbits*ADS131E08_NCHANNELS+24)/(1/ADCsettings.DR/1000-4/FCLK)
*	For 1 ksps & 24-bit DR, fsclk > 216423 Hz or 0.2 MHz
*
*	Note:
*		Because FCLK is 2.048 MHz, tSDECODE = 4*tCLK is 1.96 microsec. 
*		Since FSCLK is set to 2.25 MHz, one byte is transferred in 
*		8/FSCLK=3.56 microsec > tSDECODE. Processor can send subsequent bytes
*		without delay (see datasheet p.25). 
**/
#define ADS131_SPI	LPC_SSP0
#define FCLK 				2048000UL		// ADC clock
#define	TDELAYM			1UL					// time delay multiplier
#define FSCLK				2250000UL   // clock speed for SPI communication

volatile uint8_t ADS131_DRDY;		// data ready
volatile uint32_t ADS131_DATA_COUNTER;	// for counting data

/* Inits --------------------------------------------------------------------- */

/*****************************************************************************//**
 * @brief		configures SPI and other pins for ADC
 * @return	none
 *******************************************************************************/
void ads131_pin_init(void)			
{
	// Configure pins for SPI
	scu_pinmux(SPI_SCLK_CONF);	// SSP0_SCK PF_0
	scu_pinmux(SPI_MISO_CONF);	// SSP0_MISO PF_2
	scu_pinmux(SPI_MOSI_CONF);	// SSP0_MOSI PF_3
	scu_pinmux(ADS131_CS_CONF);   // Pin name P3_8 => GPIO 5[11]
	GPIO_SetDir(ADS131_CS_DIR);		// CS pin output
	scu_pinmux(RF233_CS_CONF);
	GPIO_SetDir(RF233_CS_DIR);
	
	// init 6G
	scu_pinmux(ADS131_ACC6G_CONF);
	GPIO_SetDir(ADS131_ACC6G_DIR);	
	
	// Configure other ADC pins
	scu_pinmux(ADS131_RESET_CONF);
	scu_pinmux(ADS131_PWDN_CONF);
	scu_pinmux(ADS131_START_CONF);
	scu_pinmux(ADS131_DRDY_CONF);
	GPIO_SetDir(ADS131_RESET_DIR);		
	GPIO_SetDir(ADS131_PWDN_DIR);		
	GPIO_SetDir(ADS131_START_DIR);		
	GPIO_SetDir(ADS131_DRDY_DIR);

	// Set pins low
	RF233_CS_HIGH();
	ADS131_CS_HIGH();
	ADS131_PWDN_LOW();
	ADS131_RESET_LOW();
	ADS131_START_LOW();
	ACC6G_HIGH();
	
	// Configure strain pins
	scu_pinmux(ADS131_U7ST_CONF);
	scu_pinmux(ADS131_U5N2_CONF);
	scu_pinmux(ADS131_U5N3_CONF);
	scu_pinmux(ADS131_U5N4_CONF);
	scu_pinmux(ADS131_U6N1_CONF);
	scu_pinmux(ADS131_U6N2_CONF);
	scu_pinmux(ADS131_U6N3_CONF);
	scu_pinmux(ADS131_U6N4_CONF);
	scu_pinmux(ADS131_U7N3_CONF);

	GPIO_SetDir(ADS131_U5N2_DIR);
	GPIO_SetDir(ADS131_U5N3_DIR);
	GPIO_SetDir(ADS131_U5N4_DIR);
	GPIO_SetDir(ADS131_U6N1_DIR);
	GPIO_SetDir(ADS131_U6N2_DIR);
	GPIO_SetDir(ADS131_U6N3_DIR);
	GPIO_SetDir(ADS131_U6N4_DIR);
	GPIO_SetDir(ADS131_U7N3_DIR);
  GPIO_SetDir(ADS131_U7ST_DIR);
	
	// Set strain pins low
	ADS131_U7ST_LOW();
	ADS131_U5N1_LOW();
	ADS131_U5N2_LOW();
	ADS131_U5N3_LOW();
	ADS131_U5N4_LOW();
	ADS131_U6N1_LOW();
	ADS131_U6N2_LOW();
	ADS131_U6N3_LOW();
	ADS131_U6N4_LOW();
	ADS131_U7N3_LOW();
}

/*****************************************************************************//**
 * @brief		initializes ADC DRDY interrupt
 * @return	none
 *******************************************************************************/
void ads131_int_init(void)
{
#define SCU_PINTSEL1_INTPIN7_Pos                              24                                                        /*!< SCU PINTSEL1: INTPIN7 Position      */
	// Initialize interrupt (DRDY pin PF_9 => GPIO7[23])
	LPC_SCU->PINTSEL1 &= (0x00 << SCU_PINTSEL1_INTPIN7_Pos);
	//LPC_SCU->PINTSEL1 |= (7 << SCU_PINTSEL1_PORTSEL7_Pos) | (23 << SCU_PINTSEL1_INTPIN7_Pos);
	LPC_SCU->PINTSEL1 |= 0xF7000000;
	
	LPC_GPIO_PIN_INT->ISEL = 0;    // sets pin interrupt mode to edge sensitive
	LPC_GPIO_PIN_INT->SIENF = (1 << 7);   // set pin interrupt enable falling edge
	
	// Set priority
	NVIC_SetPriority(PIN_INT7_IRQn, ADS131_INT_PRIORITY);
}

/* Interrupts ---------------------------------------------------------------- */

/*****************************************************************************//**
 * @brief		enables ADC DRDY interrupt
 * @return	none
 *******************************************************************************/
void ads131_int_enable(void)
{
	NVIC_EnableIRQ(PIN_INT7_IRQn);
}

/*****************************************************************************//**
 * @brief		disables ADC DRDY interrupt
 * @return	none
 *******************************************************************************/
void ads131_int_disable(void)
{
	NVIC_DisableIRQ(PIN_INT7_IRQn);				
}

/* Data communication -------------------------------------------------------- */

 /*****************************************************************************//**
 * @brief		sends command byte to ADC
 * @return	none
 *******************************************************************************/
void ads131_sendCommand(uint8_t ByteCmd)
{
	ADS131_CS_LOW();
	SPI_SendByte(ADS131_SPI, ByteCmd);
	TIM_Waitus((uint32_t)2*TDELAYM);				// 4/FCLK = 1.95 us
	ADS131_CS_HIGH();
}

//------------------------------------------------------------------------------
//         Public Functions
//------------------------------------------------------------------------------

/*****************************************************************************//**
 * @brief		initialize ADC
 * @return	SUCCESS when complete
 *******************************************************************************/
Status ads131_init(void)			
{
	ADS131_DRDY = 0;
	ADS131_DATA_COUNTER = 0;
	ads131_int_disable();
	ads131_pin_init();
	
	// Configure SPI driver and interrupt
	if (SPI_Init(ADS131_SPI, SSP_CPHA_SECOND, SSP_CPOL_HI, FSCLK, SSP_DATABIT_8,
	SSP_MASTER_MODE) != SUCCESS) {
		return ERROR;
	}
	
	ads131_int_init();
	ads131_int_enable();
	
	return SUCCESS;
}

/*****************************************************************************//**
 * @brief		de-initialize ADC
 * @return	SUCCESS when complete
 *******************************************************************************/
Status ads131_deinit(void)
{
	ADS131_CS_HIGH();
	return SPI_DeInit(ADS131_SPI);
}

/*****************************************************************************//**
 * @brief		reads from ADC registers
 * @return	none
 *******************************************************************************/
void ads131_readReg(uint8_t StartAddress, uint8_t NumRegs, uint8_t *pData)
{
	int i;
	ADS131_CS_LOW();
	SPI_SendByte(ADS131_SPI, RREG | StartAddress);
	SPI_SendByte(ADS131_SPI, NumRegs - 1);
	for (i = 0; i < NumRegs; i++) {
		SPI_ReceiveByte(ADS131_SPI, pData++);
	}
	ADS131_CS_HIGH();
}

/*****************************************************************************//**
 * @brief		writes to ADC registers
 * @return	none
 *******************************************************************************/
void ads131_writeReg(uint8_t StartAddress, uint8_t NumRegs, uint8_t *pData)
{
	int i;
	ADS131_CS_LOW();
	SPI_SendByte(ADS131_SPI, WREG | StartAddress);
	SPI_SendByte(ADS131_SPI, NumRegs - 1);
	for (i = 0; i < NumRegs; i++) {
		SPI_SendByte(ADS131_SPI, *pData++);
	}
	TIM_Waitus((uint32_t)2*TDELAYM);				// 4/FCLK = 1.95 us
	ADS131_CS_HIGH();
}

/*****************************************************************************//**
 * @brief		powers on ADC (see datasheet p.33)
 * @return		SUCCESS when complete
 *******************************************************************************/
Status ads131_on(void)
{
	uint8_t config3data, checkconfig3;
	
	// Power on and delay for power-on reset
	ADS131_PWDN_HIGH();
	ADS131_RESET_HIGH();
	vTaskDelay((uint32_t) 128*TDELAYM); // 2^18/FCLK = 128 ms
	
	// Reset
	ADS131_RESET_LOW();
	TIM_Waitus((uint32_t) 1*TDELAYM);	// 1/FCLK = 0.49 us
	ADS131_RESET_HIGH();
	TIM_Waitus((uint32_t) 9*TDELAYM);	// 18/FCLK = 8.79 us
	
	// Set chip select pin high
	ADS131_CS_HIGH();
	
	// Send SDATAC command to read or write to registers
	//		datasheet p.25
	ads131_sendCommand(SDATAC);
	
	// CONFIG3 (enable internal reference buffer)
	// TEST: Read Config 3 register
	config3data = SET_CONFIG3(1, 0, 1, 1);
	ads131_writeReg(CONFIG3, (uint8_t) 1, &config3data);
	ads131_readReg(CONFIG3, (uint8_t) 1, &checkconfig3);
	if (checkconfig3 != config3data) {
		return ERROR;
	}
	
	return SUCCESS;
}

/*****************************************************************************//**
 * @brief		powers off ADC 
 * @return	none
 *******************************************************************************/
void ads131_off(void)
{
	ADS131_CS_HIGH();
	ADS131_PWDN_LOW();
}

/*****************************************************************************//**
 * @brief		puts ADC into standby mode
 *	(do NOT send any other command other than wakeup after this command is called) 
 * @return	none
 *******************************************************************************/
void ads131_standby(void)
{
	ads131_sendCommand(STANDBY);
}

/*****************************************************************************//**
 * @brief		wakes up ADC from standby mode
 * @return	none
 *******************************************************************************/
void ads131_wakeup(void)
{
	ads131_sendCommand(WAKEUP);
}

/*****************************************************************************//**
 * @brief		  Sets up global acquisition parameters
 * @param[in]	ADCset - points to ADS_CFG_Type of ADC setting parameters
 *						DataRate - in kSPS
 * @return		SUCCESS when complete
 *******************************************************************************/
Status ads131_setupDAQ(ADS131_CFG_Type *ADCset, uint8_t DataRate)
{
	uint8_t config1data, checkconfig1, checkDaisyIn, checkClkEn, checkDR;
	uint8_t config2data, checkconfig2;
	
	// Not in sensing mode
	ADCset->sensing = 0;
	
	// Select corresponding Nbits of resolution
	if (DataRate < 32) {
		ADCset->Nbits = 24;
	} else {
		ADCset->Nbits = 16;
	}
	
	// Select correct data rate
	switch (DataRate) {
	case 64:
		ADCset->DR = 0x00;
		break;
	case 32:
		ADCset->DR = 0x01;
		break;
	case 16:
		ADCset->DR = 0x02;
		break;
	case 8:
		ADCset->DR = 0x03;
		break;
	case 4:
		ADCset->DR = 0x04;
		break;
	case 2:
		ADCset->DR = 0x05;
		break;
	case 1:
		ADCset->DR = 0x06;
		break;
	default:
		return ERROR;
	}
	
	// Write to CONFIG1 register
	//		DAISY_IN & CLK_EN settings set to 0
	config1data = SET_CONFIG1(0x00, 0x00, ADCset->DR);
	ads131_writeReg(CONFIG1, (uint8_t) 1, &config1data);
	// TEST: Read Config 1 register
	ads131_readReg(CONFIG1, (uint8_t) 1, &checkconfig1);
	checkDaisyIn = CONFIG1_DAISY_IN(checkconfig1);
	if (checkDaisyIn != (uint8_t) 0) {
		return ERROR;
	}
	checkClkEn = CONFIG1_CLK_EN(checkconfig1);
	if (checkClkEn != (uint8_t) 0) {
		return ERROR;
	}
	checkDR = CONFIG1_DR(checkconfig1);
	if (checkDR != ADCset->DR) {
		return ERROR;
	}
	
	// Write to CONFIG2 register (no test signal generation)
	config2data = CONFIG2_RESET;
	ads131_writeReg(CONFIG2, (uint8_t) 1, &config2data);
	
	// TEST: Read Config 2 register
	ads131_readReg(CONFIG2, (uint8_t) 1, &checkconfig2);
	if (checkconfig2 != 0xE0) {
		return ERROR;
	}
	
	return SUCCESS;
}

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
					  ADS131_chType channelType)
{
	uint8_t chnsetdata, checkchn, checkpdn, checkgain, checkmuxn;
	chPointer = chPointer + (channelNum-1);
	
	// Fill in channel config settings
	chPointer->powerDown = powerDownYes;
	
	switch (channelNum) {
	case 1:
		chPointer->channelAdd = CH1SET;
		break;
	case 2:
		chPointer->channelAdd = CH2SET;
		break;
	case 3:
		chPointer->channelAdd = CH3SET;
		break;
	case 4:
		chPointer->channelAdd = CH4SET;
		break;
	case 5:
		chPointer->channelAdd = CH5SET;
		break;
	case 6:
		chPointer->channelAdd = CH6SET;
		break;
	case 7:
		chPointer->channelAdd = CH7SET;
		break;
	case 8:
		chPointer->channelAdd = CH8SET;
		break;
	default:
		return ERROR;
	}
	
	switch (gainValue) {
	case 1:
		chPointer->gain = 0x01;
		break;
	case 2:
		chPointer->gain = 0x02;
		break;
	case 4:
		chPointer->gain = 0x04;
		break;
	case 8:
		chPointer->gain = 0x05;
		break;
	case 12:
		chPointer->gain = 0x06;
		break;
	default:
		return ERROR;
	}
	
	switch (channelType) {
	case NORMAL:
		chPointer->type = 0x00;
		break;
	case SHORTED:
		chPointer->type = 0x01;
		break;
	case SUPPLY:
		chPointer->type = 0x03;
		break;
	case TEMPERATURE:
		chPointer->type = 0x04;
		break;
	case TEST:
		chPointer->type = 0x05;
		break;
	case CH_OFF:								// default CH_OFF to shorted
		chPointer->type = 0x01;
		break;
	default:
		return ERROR;
	}
	
	// Write new settings to channelNum register
	chnsetdata = SET_CHnSET(chPointer->powerDown, chPointer->gain, chPointer->type);
	ads131_writeReg(chPointer->channelAdd, (uint8_t) 1, &chnsetdata);
	
	// TEST: Read CHSET register
	ads131_readReg(chPointer->channelAdd, (uint8_t) 1, &checkchn);
	checkpdn = CHnSET_PDn(checkchn);
	if (checkpdn != chPointer->powerDown) {
		return ERROR;
	}
	checkgain = CHnSET_GAINn(checkchn);
	if (checkgain != chPointer->gain) {
		return ERROR;
	}
	checkmuxn = CHnSET_MUXn(checkchn)	;
	if (checkmuxn != chPointer->type) {
		return ERROR;
	}
		
	return SUCCESS;
}

/*****************************************************************************//**
 * @brief		ADC DRDY interrupt handler
 * @return	none
 *******************************************************************************/
extern TaskHandle_t Stask; //local task for sensor
void GPIO7_IRQHandler(void)
{
	// Check interrupt status register
	//if (LPC_GPIO_PIN_INT->IST & (1 << 7))
	if (LPC_GPIO_PIN_INT->FALL & (1 << 7)) {
		// Clear interrupt
		//LPC_GPIO_PIN_INT->IST = (1 << 7);
		LPC_GPIO_PIN_INT->FALL = (1 << 7);
		// Sensor data is ready
		ADS131_DRDY = 1;
		ADS131_DATA_COUNTER += 1;
		/*
		if (Stask) {
			xTaskNotifyFromISR(Stask, 0, eNoAction, NULL);
		}
		*/
	}
}

/*****************************************************************************//**
* @brief	start sensing command
* @param[in]	ADCset - points to ADS_CFG_Type of ADC setting parameters
* @return	SUCCESS when complete
*******************************************************************************/
void ads131_startSensing(ADS131_CFG_Type *ADCset)
{
	//ACC6G_HIGH();
	ads131_sendCommand(START);
	ADS131_DRDY = 0;
	ADS131_DATA_COUNTER = 0;
	ads131_sendCommand(RDATAC);
	
	
	ADCset->sensing = 1;
}

/*****************************************************************************//**
 * @brief		reads one data point from all channels from ADC (24-bit mode)
 * 					and reads the status register
 * @return	none
 *******************************************************************************/
Status ads131_readDataPt(int32_t *pData, int32_t *stat)
{
	int32_t i, num;
	int32_t N = 27;       // N = ADCsettings.Nbits*ADS131E08_NCHANNELS + 24;
	uint8_t temp[27];
	
	
	ADS131_CS_LOW();
	// Receive data
	for (i = 0; i < N; i++)
	{
		SPI_ReceiveByte(ADS131_SPI, &temp[i]);
	}
	ADS131_CS_HIGH();
	
	// Parse data
	// 24-bit STATUS
	*stat = ((((temp[0] & 0x80) ? 0xff : 0) << 24) | ((int32_t) temp[0]) << 16) | (((int32_t) temp[1]) << 8) | ((int32_t) temp[2]);
	// channel data
	for (i = 1; i < (ADS131_NCHANNELS+1); i++) {
		num = i*3;
		*pData++ = (((temp[num] & 0x80) ? 0xff : 0) << 24) | (((int32_t) temp[num]) << 16) | (((int32_t) temp[num+1]) << 8) | ((int32_t) temp[num+2]);
	}
	
	return SUCCESS;
}

/*****************************************************************************//**
* @brief	stop sensing command
* @param[in]	ADCset - points to ADS_CFG_Type of ADC setting parameters
* @return	SUCCESS when complete
*******************************************************************************/
void ads131_stopSensing(ADS131_CFG_Type *ADCset)
{
	ADS131_CS_HIGH();
	
	ads131_sendCommand(SDATAC);
	ads131_sendCommand(STOP);
	
	ADCset->sensing = 0;
}

/*****************************************************************************//**
 * @brief		Removes offset for all channels
 * @return	SUCCESS when complete
 *******************************************************************************/
Status ads131_removeOffset(void)
{
	ads131_sendCommand(OFFSETCAL);
	vTaskDelay((uint32_t) 154*TDELAYM);				// 153 ms (datasheet p.24)
	return SUCCESS;
}

/*****************************************************************************//**
* @brief	turns square wave test signal on or off
* @param[in]	sigOn - 1 (turn on signal); 0 (turn off signal)
* @return	SUCCESS when complete
*******************************************************************************/
Status ads131_testSignal(uint8_t sigOn)
{
	uint8_t config2data, checkconfig2;
	
	if (sigOn) {
		// Create internal (1 mV / 2.4 V) square-wave test signal
		config2data = 0xF0;
		ads131_writeReg(CONFIG2, (uint8_t) 1, &config2data);
		// TEST: Read Config 2 register
		ads131_readReg(CONFIG2, (uint8_t) 1, &checkconfig2);
		if (checkconfig2 != 0xF0) {
			return ERROR;
		}
	} else {
		// Write to CONFIG2 register (no test signal generation)
		config2data = CONFIG2_RESET;
		ads131_writeReg(CONFIG2, (uint8_t) 1, &config2data);
		// TEST: Read Config 2 register
		ads131_readReg(CONFIG2, (uint8_t) 1, &checkconfig2);
		if (checkconfig2 != 0xE0) {
			return ERROR;
		}
	}
	
	return SUCCESS;
}
