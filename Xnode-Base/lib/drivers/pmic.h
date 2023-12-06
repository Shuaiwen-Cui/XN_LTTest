#ifndef __PMIC_H__
#define __PMIC_H__

#include "lpc43xx_i2c.h"

#define PMIC_I2C         LPC_I2C0
#define PMIC_FREQ        400000

typedef struct {
  float v1, v2, v3, v4;
} pmic_info;

typedef struct {
  float Vcur, Vbat;
} chg_info;

void PMIC_Init(void);
Status PMIC_Status(chg_info *chg, pmic_info *pi);

#endif /* __PMIC_H__ */
