/*************************************************
*@File        TriggerSensing.h
*@Brief       Functions for wake-up sensor
*@Version     1.0
*@Date        08/27/17
*@Author      Yuguang Fu
**************************************************/

#ifndef _TRIGGERSENSING_H
#define _TRIGGERSENSING_H

#include <sensing.h>
#include "adxl362.h"

#define num 1000
#define SAMPLE_SET 170
#define overlap_size 400

int adxl362_setup(void);
int readadxl(int16_t *data[]);
int TriggerSensing (void);
void adxl_inact_setup(void);
void adxl_act_setup(void);
bool RTC_Reconfigure(void);
int Trig_Init(void);
int TrigSenSetup(void);
int TrigSenCfgSend(uint16_t *addrs, uint8_t addrlen);

#endif
