#ifndef _SNOOZEALARM_H
#define _SNOOZEALARM_H

#include <stdint.h>

extern float voltage;
extern float current;

#define MIN_VOLTAGE         (3.3f)

#define SNOOZE_ALARM_AWAKE  500 // milliseconds
#define SNOOZE_ALARM_PERIOD 1 // minutes

void SnoozeAlarm_Start(void);
void SnoozeAlarm_Stop(void);
void SnoozeAlarm_Sleep(void);
void SnoozeAlarm_Wake(uint16_t *addrs, uint8_t addrlen);
void SnoozeAlarm_Wake_Retrieve(uint16_t *addrs, uint8_t addrlen);
void SnoozeAlarm_Sleep_Retrieve(void) ;
void CheckBatteryVoltage(void);
#endif // _SNOOZEALARM_H
