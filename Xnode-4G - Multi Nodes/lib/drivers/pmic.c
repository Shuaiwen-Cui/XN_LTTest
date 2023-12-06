#include <LPC43xx.h>
#include <lpc43xx_scu.h>
#include <lpc43xx_i2c.h>
#include <stdio.h>
#include <string.h>
#include <vcom.h>
#include "pmic.h"

//register
# define STATUS 0x00
# define CONTROL 0x01
# define TO_READ 2 //read all four voltages (V1,V2,V3,V4)
# define V1MSB 0x06
# define V1LSB 0x07
# define V2MSB 0x08
# define V2LSB 0x09
# define V3MSB 0x0A
# define V3LSB 0x0B
# define V4MSB 0x0C
# define V4LSB 0x0D
# define LTC2990_single_ended_lsb 0.0003f

#define PMIC_READ_ADDR 	 0x98
#define PMIC_WRITE_ADDR  0x98

static uint8_t tempbuf[2];

void PMIC_Init(void)
{
  // Initializes the I2Cx peripheral with specified parameter
  I2C_Init(PMIC_I2C, PMIC_FREQ);
  // Enable I2C
  I2C_Cmd(PMIC_I2C, ENABLE);
}

Status PMIC_Write(uint8_t address, uint8_t val)
{
  I2C_M_SETUP_Type setup;
  Status status;

  tempbuf[0] = address;
	tempbuf[1] = val;

  setup.sl_addr7bit = PMIC_WRITE_ADDR >> 1;
  setup.tx_data     = tempbuf;
  setup.tx_length   = 2;
  setup.rx_data     = 0;
  setup.rx_length   = 0;
  setup.retransmissions_max = 3;

  status = I2C_MasterTransferData(PMIC_I2C, &setup, I2C_TRANSFER_POLLING);
  return status;
}

Status PMIC_Read(uint8_t address, uint8_t len, uint8_t *val)
{
  I2C_M_SETUP_Type setup;
  Status status;

  tempbuf[0] = address;

  setup.sl_addr7bit = PMIC_READ_ADDR >> 1;
  setup.tx_data     = tempbuf;
  setup.tx_length   = 1;
  setup.rx_data     = val;
  setup.rx_length   = len;
  setup.retransmissions_max = 3;

  status = I2C_MasterTransferData(PMIC_I2C, &setup, I2C_TRANSFER_POLLING);
  return status;
}

float VoltConv(uint16_t adc_code)
{
  float voltage;

  adc_code = adc_code & 0x7FFF;

  if (adc_code >> 14) {
    adc_code = 0;                 //! 1) Converts two's complement to binary
  }
	
  voltage = 0.00030518f * adc_code; //! 2) Convert code to Vcc Voltage from single-ended lsb
  return voltage;
}

float VoltConv_diff(uint16_t adc_code)
{
  float voltage;

  adc_code = adc_code & 0x7FFF;

  if (adc_code >> 14) {
    adc_code = 0;                 //! 1) Converts two's complement to binary
  }
	
  voltage = 0.00001942f * adc_code; //! 2) Convert code to Vcc Voltage from single-ended lsb
  return voltage;
}

Status PMIC_Status(chg_info *chg, pmic_info *pi)
{
  uint16_t temp = 0;
	uint8_t buff[2], stat[1];
	
	//Control Resister
	PMIC_Write(CONTROL, 0x1A); //control
  PMIC_Write(0x02, 0x00); //trigger conversion
	
	stat[0] = 0;

	//wait until the data is ready
	do {
		PMIC_Read(STATUS, 1, stat);
	}	while (stat[0] <= 63);

	PMIC_Read(V1MSB, 2, buff);
  temp=(uint16_t)(((buff[0])<<8)|buff[1]);
  pi->v1=VoltConv_diff(temp);

  PMIC_Read(V2MSB, 2, buff); 
  temp=((buff[0])<<8)| buff[1];
  pi->v2=VoltConv_diff(temp);

  PMIC_Read(V3MSB, 2, buff); 
  temp=((buff[0])<<8)| buff[1];
  pi->v3=VoltConv(temp);

  PMIC_Read(V4MSB, TO_READ, buff); 
  temp=((buff[0])<<8)| buff[1];
  pi->v4=VoltConv(temp);

  chg->Vcur = pi->v1 * 20000.f;
  chg->Vbat = pi->v3 * 2.f;

	return SUCCESS;
}
