/**********************************************************************
* @file			ads131e08_const.h
* @brief		Contains macro definitions and function prototypes
*						support for ADS131E08 ADC
* @version	1.0
* @date			September 23, 2015
* @author		Martha Cuenca
**********************************************************************/

#ifndef _ADS131_CONST_H 
#define _ADS131_CONST_H

#include "lpc_types.h"
#include "lpc43xx_gpio.h"

// Pins: port, pin number, pin mode, function mode (description)
#define SPI_SCLK_CONF		0xF, 0, SSP_IO, FUNC0	// SSP0_SCLK PF_0
#define SPI_MISO_CONF		0xF, 2, SSP_IO, FUNC2	// SSP0_MISO PF_2
#define SPI_MOSI_CONF		0xF, 3, SSP_IO, FUNC2	// SSP0_MOSI PF_3
#define RF233_CS_CONF  		0xF,  1, MD_PLN, FUNC4 // SSP0_SSEL PF_1 => GPIO7[16]
#define RF233_CS_DIR		7, 1 << 16, 1
#define ADS131_CS_CONF  	0x3,  8, MD_PDN, FUNC4	// SPIFI_CS P3_8 => GPIO5[11]
#define ADS131_CS_DIR		5, 1 << 11, 1
#define ADS131_RESET_CONF	0x4, 10, MD_PDN, FUNC4	// LCD_VD10 P4_10 => GPIO5[14]
#define ADS131_RESET_DIR	5, 1 << 14, 1
#define ADS131_PWDN_CONF	0x8,  4, MD_PDN, FUNC0	// LCD_VD7 P8_4 => GPIO4[4]
#define ADS131_PWDN_DIR		4, 1 << 4, 1
#define ADS131_START_CONF	0x8,  3, MD_PDN, FUNC0	// LCD_VD12 P8_3 => GPIO4[3]
#define ADS131_START_DIR	4, 1 << 3, 1
#define ADS131_DRDY_CONF	0xF,  9, SSP_IO, FUNC4	// PF_9 => GPIO7[23]
#define ADS131_DRDY_DIR		7, 1 << 23, 0

//Define Acceleration Control Switch
#define ADS131_ACC6G_CONF	  0x7,  4, MD_PUP, FUNC0
#define ADS131_ACC6G_DIR		3, 1 << 12, 1

// Pin control
#define RF233_CS_HIGH()    GPIO_SetValue(7, 1 << 16)
#define ADS131_CS_HIGH()    GPIO_SetValue(5, 1 << 11)
#define ADS131_CS_LOW()     GPIO_ClearValue(5, 1 << 11)
#define ADS131_RESET_HIGH()	GPIO_SetValue(5, 1 << 14)
#define ADS131_RESET_LOW()	GPIO_ClearValue(5, 1 << 14)
#define ADS131_PWDN_HIGH()	GPIO_SetValue(4, 1 << 4)
#define ADS131_PWDN_LOW()	GPIO_ClearValue(4, 1 << 4)
#define ADS131_START_HIGH()	GPIO_SetValue(4, 1 << 3)
#define ADS131_START_LOW()	GPIO_ClearValue(4, 1 << 3)

// Acceleration pin control
#define ACC6G_HIGH()	GPIO_SetValue(3, 1 << 12)
#define ACC6G_LOW()	GPIO_ClearValue(3, 1 << 12)

// strain sensing pins:
// all: U7ST (J1-16, PC_12)
// ch4: U5N2 (J1-45, P9_3), U5N3 (J1-73,  PE_4), U5N4 (J1-46, P9_4)
// ch5: U6N1 (J1-28, PD_1), U6N4 (J1-38, PD_11), U5N1 (J1-52, P3_0 - not GPIO?)
// ch6: U6N2 (J1-29, PD_2), U6N3 (J1-35,  PD_8), U7N3 (J1-11, P1_4)
#define ADS131_U7ST_CONF 0xC,  12, MD_PDN, FUNC4 // PC_12 => GPIO6[11]
#define ADS131_U7ST_DIR 6, 1 << 11, 1
#define ADS131_U5N1_CONF 0x9,   4, MD_PDN, FUNC0 // P9_4 => GPIO5[17]
#define ADS131_U5N1_DIR 5, 1 << 17, 1
#define ADS131_U5N2_CONF 0x9,   3, MD_PDN, FUNC0 // P9_3 => GPIO4[15]
#define ADS131_U5N2_DIR 4, 1 << 15, 1
#define ADS131_U5N3_CONF 0xE,   4, MD_PDN, FUNC4 // PE_4 => GPIO7[3]
#define ADS131_U5N3_DIR 7, 1 << 3, 1
#define ADS131_U5N4_CONF 0x9,   2, MD_PDN, FUNC4 // P9_2 => GPIO4[14]
#define ADS131_U5N4_DIR 4, 1 << 14, 1
#define ADS131_U6N1_CONF 0xD,   1, MD_PDN, FUNC4 // PD_1 => GPIO6[15]
#define ADS131_U6N1_DIR 6, 1 << 15, 1
#define ADS131_U6N2_CONF 0xD,   2, MD_PDN, FUNC4 // PD_2 => GPIO6[16]
#define ADS131_U6N2_DIR 6, 1 << 16, 1
#define ADS131_U6N3_CONF 0xD,   8, MD_PDN, FUNC4 // PD_8 => GPIO6[22]
#define ADS131_U6N3_DIR 6, 1 << 22, 1
#define ADS131_U6N4_CONF 0xD,  11, MD_PDN, FUNC4 // PD_11 => GPIO6[25]
#define ADS131_U6N4_DIR 6, 1 << 25, 1
#define ADS131_U7N3_CONF 0x1,   4, MD_PDN, FUNC0 // P1_4 => GPIO0[11]
#define ADS131_U7N3_DIR 0, 1 << 11, 1

// strain pin control
#define ADS131_U7ST_HIGH() GPIO_SetValue(6, 1 << 11)
#define ADS131_U7ST_LOW() GPIO_ClearValue(6, 1 << 11)
#define ADS131_U5N1_HIGH() GPIO_SetValue(5, 1 << 17)
#define ADS131_U5N1_LOW() GPIO_ClearValue(5, 1 << 17)
#define ADS131_U5N2_HIGH() GPIO_SetValue(4, 1 << 15)
#define ADS131_U5N2_LOW() GPIO_ClearValue(4, 1 << 15)
#define ADS131_U5N3_HIGH() GPIO_SetValue(7, 1 << 4)
#define ADS131_U5N3_LOW() GPIO_ClearValue(7, 1 << 4)
#define ADS131_U5N4_HIGH() GPIO_SetValue(4, 1 << 14)
#define ADS131_U5N4_LOW() GPIO_ClearValue(4, 1 << 14)
#define ADS131_U6N1_HIGH() GPIO_SetValue(6, 1 << 15)
#define ADS131_U6N1_LOW() GPIO_ClearValue(6, 1 << 15)
#define ADS131_U6N2_HIGH() GPIO_SetValue(6, 1 << 16)
#define ADS131_U6N2_LOW() GPIO_ClearValue(6, 1 << 16)
#define ADS131_U6N3_HIGH() GPIO_SetValue(6, 1 << 22)
#define ADS131_U6N3_LOW() GPIO_ClearValue(6, 1 << 22)
#define ADS131_U6N4_HIGH() GPIO_SetValue(6, 1 << 25)
#define ADS131_U6N4_LOW() GPIO_ClearValue(6, 1 << 25)
#define ADS131_U7N3_HIGH() GPIO_SetValue(0, 1 << 11)
#define ADS131_U7N3_LOW() GPIO_ClearValue(0, 1 << 11)

// -------------- REGISTER ADDRESSES ------------------------------------------
// ---------------- datasheet p.14 --------------------------------------------
//	Device settings (read-only)
#define ID					((uint8_t)(0x00))
//	Global settings across channels
#define CONFIG1				((uint8_t)(0x01))			// configures each ADC channel sample rate
#define CONFIG2				((uint8_t)(0x02))			// configures test signal generation
#define CONFIG3				((uint8_t)(0x03))			// configures multireference operation
#define	FAULT				((uint8_t)(0x04))			// configures fault detection operation
//	Channel specific settings: Configures power mode, PGA gain, and multiplexer settings channels
#define CH1SET				((uint8_t)(0x05))			
#define CH2SET				((uint8_t)(0x06))
#define CH3SET				((uint8_t)(0x07))
#define CH4SET				((uint8_t)(0x08))
#define CH5SET				((uint8_t)(0x09))
#define CH6SET				((uint8_t)(0x0A))
#define CH7SET				((uint8_t)(0x0B))
#define CH8SET				((uint8_t)(0x0C))
//	Fault detect status registers (read-only): Stores the status of whether the
// positive (P)/ negative (N) input on each channel has a fault or not
#define FAULT_STATP			((uint8_t)(0x12))			
#define FAULT_STATN			((uint8_t)(0x13))
//	GPIO register controls the action of the 3 GPIO pins
#define GPIO				((uint8_t)(0x14))

// ----------------- RESET VALUES FOR REGISTERS -------------------------------
#define CONFIG1_RESET			((uint8_t)(0x91))
#define CONFIG2_RESET			((uint8_t)(0xE0))
#define CONFIG3_RESET			((uint8_t)(0x40))
#define FAULT_RESET				((uint8_t)(0x00))	
#define CHnSET_RESET			((uint8_t)(0x10))		
#define GPIO_RESET				((uint8_t)(0x0F))

// ----------------- INIT VALUES FOR REGISTERS --------------------------------
#define CONFIG1_INIT			((uint8_t)(0x90))
#define CONFIG2_INIT			((uint8_t)(0xE0))
#define CONFIG3_INIT			((uint8_t)(0x40))
#define FAULT_INIT				((uint8_t)(0x00))	
#define CHnSET_INIT				((uint8_t)(0x00))		
#define GPIO_INIT				((uint8_t)(0x00))

// --------------------- BIT DEFINITIONS --------------------------------------
// -------------------- datasheet p.14-20 -------------------------------------
// Macro defines for ID control register
// --- Reading --- //
// Device family ID bitmask (Bits[7:5] = 110 for ADS131E08)
#define ID_REV_ID(data)			((uint8_t)((data & 0xE0) >> 5))
#define ID_REV_ADS131E08		((uint8_t)(0x06))
// Channel device ID bitmask (Bits[1:0] = 10 for 8-channel device
#define ID_NU_CH(data)			((uint8_t)(data & 0x03))
#define ID_8_CH					((uint8_t)(0x02))

// Macro defines for CONFIG1 register
// --- Reading --- //
// Select bit for either daisy-chain (0) or multiple read-back mode (1)
#define CONFIG1_DAISY_IN(data)		((uint8_t)((data >> 6) & 0x01))
// Oscillator clock output enable bit (can be neglected due to external
// clock being used and CLKSEL pin being grounded in hardware - datasheet p.36)
#define CONFIG1_CLK_EN(data)		((uint8_t)((data >> 5) & 0x01))
// Output data rate bitmask
#define CONFIG1_DR(data)			((uint8_t)(data & 0x07))
// --- Writing --- //
#define SET_CONFIG1(daisy_in, clk_en, dr)	\
			((uint8_t) (CONFIG1_INIT | (daisy_in << 6) | (clk_en << 5) | dr))

// Macro defines for CONFIG2 register
// --- Reading --- //
// Select bit for test signal source as either external (0) or internal (1)
#define CONFIG2_INT_TEST(data)		((uint8_t)((data >> 4) & 0x01))
// Select bit to determine the calibration signal amplitude
#define CONFIG2_TEST_AMP(data)		((uint8_t)((data >> 2) & 0x01))
// Bitmask to determine the calibration signal frequency
#define CONFIG2_TEST_FREQ(data)		((uint8_t)(data & 0x03))
// --- Writing --- //
#define SET_CONFIG2(int_test, test_amp, test_freq)	\
			((uint8_t) (CONFIG2_INIT | (int_test << 4) | (test_amp << 2) | test_freq))
 
// Macro defines for CONFIG3 register
// --- Reading --- //
// Select bit to determine the power-down reference buffer state
#define CONFIG3_PDB_REFBUF(data)		((uint8_t)((data >> 7) & 0x01))
// Select bit to determine reference voltage
#define CONFIG3_VREF_4V(data)			((uint8_t)((data >> 5) & 0x01))
// Select bit to determine op amp reference connection
#define CONFIG3_OPAMP_REF(data)			((uint8_t)((data >> 3) & 0x01))
// Select bit to determine op amp power-down reference buffer state
#define CONFIG3_PDB_OPAMP(data)			((uint8_t)((data >> 2) & 0x01))
// --- Writing --- //
#define SET_CONFIG3(pdb_refbuf, vref_4v, opamp_ref, pdb_opamp)	\
			((uint8_t) (CONFIG3_INIT | (pdb_refbuf << 7) | (vref_4v << 5) | (opamp_ref << 3) | (pdb_opamp << 2)))
 
// Macro defines for FAULT register
// --- Reading --- //
// Bitmask to determine fault detect comparator threshold
#define FAULT_COMP_TH(data)			((uint8_t)((data & 0xE0) >> 5))
// --- Writing --- //
#define SET_FAULT(comp_th)			((uint8_t) (FAULT_INIT | (comp_th << 5)))

// Macro defines for CHnSET registers
// --- Reading --- //
// Power-down enable bit
#define CHnSET_PDn(data)			((uint8_t)((data >> 7) & 0x01))
// Bitmask to determine PGA gain setting
#define CHnSET_GAINn(data)			((uint8_t)((data & 0x70) >> 4))
// Bitmask to determine channel input selection
#define CHnSET_MUXn(data)			((uint8_t)(data & 0x07))
// --- Writing --- //
#define SET_CHnSET(pdn, gainn, muxn)	\
			((uint8_t) (CHnSET_INIT | (pdn << 7) | (gainn << 4) | muxn))

// Macro defines for FAULT_STATP and FAULT_STATN registers
// --- Reading --- //
// Status bit corresponding to channel n
#define FAULT_STATP_INnP_FAULT(data, n)		((uint8_t)((data >> (n-1)) & 0x01))
#define FAULT_STATN_INnN_FAULT(data, n)		((uint8_t)((data >> (n-1)) & 0x01))

// Macro defines for GPIO register
// --- Reading --- //
// Bitmask to access bits used to read and write data to the GPIO ports
#define GPIO_GPIOD(data)					((uint8_t)((data & 0xF0) >> 4))
// Bitmask to access bits used to determine if the corresponding
// GPIOD pin is an input or output
#define GPIO_GPIOC(data)					((uint8_t)(data & 0x0F))
 // --- Writing --- //
#define SET_GPIO(gpiod, gpioc)				((uint8_t) (GPIO_INIT | (gpiod << 4) | gpioc))

// ----------------- SPI OPCODE COMMAND DEFINITIONS ----------------------------
// ------------------------- datasheet p.23 ------------------------------------
// System commands
#define WAKEUP					((uint8_t)(0x02))		// wake-up from standby mode
#define STANDBY					((uint8_t)(0x04))		// standby mode
#define RESET					((uint8_t)(0x06))		// reset device
#define START					((uint8_t)(0x08))		// start conversion
#define STOP					((uint8_t)(0x0A))		// stop conversion
#define OFFSETCAL				((uint8_t)(0x1A))		// channel offset calibration
// Data read commands
#define RDATAC					((uint8_t)(0x10))		// enable read data continuous mode
#define SDATAC					((uint8_t)(0x11))		// stop read data continuous mode
#define RDATA					((uint8_t)(0x12))		// read data by command
// Register read command bitmasks
#define RREG					((uint8_t)(0x20))		// read N registers starting at address R
#define WREG					((uint8_t)(0x40))		// write N registers starting at address R

#endif /* _ADS131_CONST_H */
