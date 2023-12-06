#include <lpc43xx_i2c.h>
#include "poti2c.h"

//register
# define TPL_REG_A 0x00
# define TPL_REG_B 0x01



#define TPL_ADDR 0x50
static uint8_t tempbuf[2];

void Pot_Init(void)
{
	 // Initializes the I2Cx peripheral with specified parameter
  I2C_Init(TPL_I2C, TPL_FREQ);
	// Enable I2C
  I2C_Cmd(TPL_I2C, ENABLE);
}

Status Pot_Write(uint8_t address, uint8_t val)
{
  I2C_M_SETUP_Type config;
  Status result;

  tempbuf[0] = address;
	tempbuf[1] = val;

  config.sl_addr7bit = TPL_ADDR;
  config.tx_data     = tempbuf;
  config.tx_length   = 2;
  config.rx_data     = 0;
  config.rx_length   = 0;
  config.retransmissions_max = 3;

  result = I2C_MasterTransferData(TPL_I2C, &config, I2C_TRANSFER_POLLING);
  return result;
}
