/*************************************************
*@File        adxl362.h
*@Brief       Headfile for driver of adxl362 wake-up sensor
*@Version     1.1
*@Date        01/02/17
*@Author      Yuguang Fu
**************************************************/

#ifndef _ADXL362_H 
#define _ADXL362_H

#include "lpc_types.h"
#include "lpc43xx_gpio.h"
#include "lpc43xx_timer.h"

/*------------- Global parameters -------------*/
// Pins: port, pin number, pin mode, function mode (description)
#define SPI_SCLK_CONF		    0xF, 0, SSP_IO, FUNC0	  // SSP0_SCLK  PF_0
#define SPI_MISO_CONF		    0xF, 2, SSP_IO, FUNC2	  // SSP0_MISO  PF_2
#define SPI_MOSI_CONF		    0xF, 3, SSP_IO, FUNC2	  // SSP0_MOSI  PF_3
#define ADXL362_CS_CONF  	  0xC, 9, MD_PDN, FUNC4	  // CS PC_9 => GPIO6[8]      //0x9, 1, MD_PDN, FUNC0	  // SPIFI_MOSI P9_1 => GPIO4[13]   	
#define ADXL362_CS_DIR		  6, 1 << 8,  1                                                //4, 1 << 13, 1
//#define ADXL362_ACTD_CONF	  0x3, 7, SSP_IO, FUNC4	  // P3_7 => GPIO5[10]                  //0xD, 10, SSP_IO, FUNC4	  // PD_10 => GPIO6[24], connected to INT1
//#define ADXL362_ACTD_DIR		5, 1 << 10, 0                                                //6, 1 << 24, 1  
//#define ADXL362_DRDY_CONF	  0xD, 12, SSP_IO, FUNC4	  // PD_12 => GPIO6[26], connected to INT2
//#define ADXL362_DRDY_DIR		6, 1 << 26, 1 
#define ADXL362_IACTD_CONF	  0xD, 9, SSP_IO, FUNC4	  // PD_9 => GPIO6[23] connected to INT2
#define ADXL362_IACTD_DIR		  6, 1 << 23,  0

// Pin control
#define ADXL362_CS_HIGH()     GPIO_SetValue(6, 1 << 8)
#define ADXL362_CS_LOW()      GPIO_ClearValue(6, 1 << 8)

// SPI parameters: SPI bus, clock speed
#define ADEXL362_SPI	                LPC_SSP0
#define ADEXL362_FSCLK				        6000000  // both 1000000 & 12000000 are good when just check id  // clock speed for SPI communication

// Priority and flag of ADXL362 interrupt
#define ADXL362_INT_PRIORITY           (uint32_t)0     
extern volatile uint8_t ADXL362_IACTD;

/* ------------- SPI Commands -----------------*/
#define ADXL362_REG_DEVID_AD          ((uint8_t)(0x00))         // Device ID 
#define ADXL362_REG_REVID             ((uint8_t)(0x03))         // Silicon revision ID
#define ADXL362_REG_XDATA             ((uint8_t)(0x08))         // X-axis data (8 MSB)
#define ADXL362_REG_YDATA             ((uint8_t)(0x09))         // Y-axis data (8 MSB)
#define ADXL362_REG_ZDATA             ((uint8_t)(0x0A))         // Z-axis data (8 MSB)
#define ADXL362_REG_STATUS            ((uint8_t)(0x0B))         // Status: ERR_USER_REGS, AWAKE, INACT, ACT, FIFO_OVERRUN, FIFO_WATERMARKT, FIFO_READY, DATA_READY
#define ADXL362_REG_XDATA_L           ((uint8_t)(0x0E))         // X-axis data (8 LSBs)
#define ADXL362_REG_XDATA_H           ((uint8_t)(0x0F))         // X-axis data (4 MSBs)
#define ADXL362_REG_YDATA_L           ((uint8_t)(0x10))         // Y-axis data (8 LSBs)
#define ADXL362_REG_YDATA_H           ((uint8_t)(0x11))         // Y-axis data (4 MSBs)
#define ADXL362_REG_ZDATA_L           ((uint8_t)(0x12))         // Z-axis data (8 LSBs)
#define ADXL362_REG_ZDATA_H           ((uint8_t)(0x13))         // Z-axis data (4 MSBs)
#define ADXL362_REG_TEMP_L            ((uint8_t)(0x14))         // Temperature data (8 LSBs)
#define ADXL362_REG_TEMP_H            ((uint8_t)(0x15))         // Temperature data (4 MSBs)
#define ADXL362_REG_SOFT_RESET        ((uint8_t)(0x1F))         // Soft reset
#define ADXL362_REG_THRESH_ACT_L      ((uint8_t)(0x20))         // Activity threshold (8 LSBs)
#define ADXL362_REG_THRESH_ACT_H      ((uint8_t)(0x21))         // Activity threshold (3 MSBs)
#define ADXL362_REG_TIME_ACT          ((uint8_t)(0x22))         // Activity time
#define ADXL362_REG_THRESH_INACT_L    ((uint8_t)(0x23))         // Inactivity threshold (8 LSBs)
#define ADXL362_REG_THRESH_INACT_H    ((uint8_t)(0x24))         // Inactivity threshold (3 MSBs)
#define ADXL362_REG_TIME_INACT_L      ((uint8_t)(0x25))         // Inactivity time (8 LSBs)
#define ADXL362_REG_TIME_INACT_H      ((uint8_t)(0x26))         // Inactivity time (8 MSBs)
#define ADXL362_REG_ACT_INACT_CTL     ((uint8_t)(0x27))         // Activity/inactivity control: LINK/LOOP, INACT_REF, INACT_EN, ACT_REF, ACT_EN 
#define ADXL362_REG_FIFO_CTL          ((uint8_t)(0x28))         // FIFO control: AH, FIFO_TEMP, FIFO_MODE
#define ADXL362_REG_FIFO_SPL          ((uint8_t)(0x29))         // FIFO samples
#define ADXL362_REG_INTMAP1           ((uint8_t)(0x2A))         // INT1 function map: INT_LOW, AWAKE, INACT, ACT, FIFO_OVERRUN, FIFO_WATERMARK, FIFO_READY, DATA_READY
#define ADXL362_REG_INTMAP2           ((uint8_t)(0x2B))         // INT2 function map: INT_LOW, AWAKE, INACT, ACT, FIFO_OVERRUN, FIFO_WATERMARK, FIFO_READY, DATA_READY
#define ADXL362_REG_FILTER_CTL        ((uint8_t)(0x2C))         // Filter control: RANGE, HALF_BW, EXT_SAMPLE, ODR
#define ADXL362_REG_POWER_CTL         ((uint8_t)(0x2D))         // Power control: EXT_CLK, LOW_NOISE, WAKEUP, AUTOSLEEP, MEASURE
#define ADXL362_REG_SELF_TST          ((uint8_t)(0x2E))         // Self test control

/* ------------- Register Commands  ------------- */
#define ADXL362_SOFTRESET_EN            0x52
#define ADXL362_ACT_INT_MODE            0x10
#define ADXL362_IACT_INT_MODE           0x20
#define ADXL362_ACT_INT_LOW             0x90
#define ADXL362_CLR_INT_MODE            0x00
#define ADXL362_DRDY_INT_MODE           0x01
#define ADXL362_SPI_WRITE               0x0A
#define ADXL362_SPI_READ                0x0B
#define ADXL362_FIFO_READ               0x0D

/* ------------- Functions -----------------*/

/* initialize ADXL362 */
Status adxl362_init(Bool reset);

/* clear ADXL362 interrupt status*/
void adxl362_clearirq(void);

/* disable ADXL362 interrupt pin*/
void adxl362_disableint(Bool int_pin);

/* read id */
void adxl362_readid(void);

/* start ADXL362 sensing */
void adxl362_startSensing(void);

/* stop ADXL362 sensing */
void adxl362_stopSensing(void);

/* read XYZ and temperature data */
Status adxl362_readDataXYZT(int16_t *pData);

/* ADXL362 self test */
Status adxl362_selfTest(uint8_t range);

/* set up activity detection */
void adxl362_act_int(uint16_t thres, uint16_t time, Bool abs);

/* set up inactiviey detection */
void adxl362_inact_int(uint16_t thres, uint16_t time, Bool abs);

/* set up FIFO configuration */
void adxl362_FIFO_config(Bool FIFO_ah, Bool FIFO_temp, uint8_t FIFO_mode, uint16_t FIFO_num);

/* set up interrupt pin function */
void adxl362_int_setup(Bool int_pin, Bool act_mode, int int_mode);

/* set up filter control */
void adxl362_filter_ctl(uint8_t range, Bool half_bw, uint16_t datarate);

/* read FIFO buffer */
void adxl362_FIFOread(uint16_t Sample_num, int16_t *Data);

/* test: SPI read, Device ID and Status */
void adxl362_get_idst(void);

/* initalize interrupt pin */
void adxl362_int_init(void);

/* enable interrupt pin */
void adxl362_int_enable(void);

/* initalize ADXL with interrupt function disabled*/
void adxl362_init_noINT(void);

/* check GPIO for data ready*/
uint32_t adxl362_get_drdy(void);

/* check status to confirm if an event is detected */
int adxl362_motiondetection(void);

#endif 

