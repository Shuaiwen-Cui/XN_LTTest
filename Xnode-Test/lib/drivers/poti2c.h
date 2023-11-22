
#include "lpc43xx_i2c.h"

#define TPL_I2C         LPC_I2C0
#define TPL_FREQ        100000

void Pot_Init(void);
Status Pot_Write(uint8_t address, uint8_t val);
