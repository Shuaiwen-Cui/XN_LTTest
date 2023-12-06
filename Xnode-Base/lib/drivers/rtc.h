/**********************************************************************
* @file			rtc.h
* @date			July 3, 2017
* @author		Yuguang Fu
**********************************************************************/

#ifndef _RTC_H_
#define _RTC_H_

#include "lpc43xx_i2c.h"

#define RTC_I2C         LPC_I2C0
#define RTC_FREQ        100000
#define RTC_ADDR        ((uint8_t)(0x68))

extern volatile uint8_t RTC_INT;

/* ------------ registers  ---------------*/
#define RTC_REG_SEC   ((uint8_t)(0x00))         // second 
#define RTC_REG_MIN   ((uint8_t)(0x01))         // minute
#define RTC_REG_HOU   ((uint8_t)(0x02))         // hour
#define RTC_REG_DAY   ((uint8_t)(0x03))         // day
#define RTC_REG_DAT   ((uint8_t)(0x04))         // date
#define RTC_REG_MON   ((uint8_t)(0x05))         // month
#define RTC_REG_YER   ((uint8_t)(0x06))         // year1
#define RTC_REG_A1S   ((uint8_t)(0x07))         // seconds for Alarm 1
#define RTC_REG_A1M   ((uint8_t)(0x08))         // minute for Alarm 1
#define RTC_REG_A1H   ((uint8_t)(0x09))         // hour for Alarm 1
#define RTC_REG_A1D   ((uint8_t)(0x0A))         // day/date for Alarm 1
#define RTC_REG_A2M   ((uint8_t)(0x0B))         // minute for Alarm 2
#define RTC_REG_A2H   ((uint8_t)(0x0C))         // hour for Alarm 2
#define RTC_REG_A2D   ((uint8_t)(0x0D))         // day/date for Alarm 2
#define RTC_REG_CTL   ((uint8_t)(0x0E))         // control
#define RTC_REG_STU   ((uint8_t)(0x0F))         // control/status
#define RTC_REG_AGE   ((uint8_t)(0x10))         // aging offset
#define RTC_REG_TEMP_M   ((uint8_t)(0x11))         // MSB of Temp
#define RTC_REG_TEMP_L   ((uint8_t)(0x12))         // LSB of Temp


typedef struct {
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
  uint8_t day;
  uint8_t date;    
	uint8_t month;
	uint8_t year;
	Bool    clock24; // 1: 24,  0: 12
	Bool    AMPM;    // 1: am;  0: pm
}Clock_info;

typedef struct {
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t DoWM;    // day (DoW) or date (DoM)
	uint8_t mask;    // 
	Bool    dydt;    // 1: day, 0: date
	Bool    clock24; // 1: 24,  0: 12
	Bool    AMPM;    // 1: am;  0: pm
}Alarm1_info;

// for mask
// 4 = Alarm once per second
// 3 = Alarm when seconds match
// 2 = Alarm when minutes and seconds match
// 1 = Alarm when hours, minutes and seconds match
// NOTE the next two depend on A1DYDT
// 0 = If A1DYDT=NO then Alarm when date, hours, minutes and seconds match 
//     If A1DYDT=Yes then Alarm when day, hours, minutes and seconds match   

typedef struct {
	uint8_t minute;
	uint8_t hour;
	uint8_t DoWM;    // day (DoW) or date (DoM)
	uint8_t mask;    // 
	Bool    dydt;    // 1: day, 0: date
	Bool    clock24; // 1: 24,  0: 12
	Bool    AMPM;    // 1: am;  0: pm
}Alarm2_info;

// for mask
// 3 = Alarm once per minute (00 seconds of every minute)
// 2 = Alarm when minutes match
// 1 = Alarm when hours, minutes match
// NOTE the next two depend on A2DYDT
// 0 = If A2DYDT=NO then Alarm when date, hours and minutes match 
//     If A2DYDT=Yes then Alarm when day, hours and minutes match    

typedef struct{
	Bool EOSC;
	Bool nEOSC;
	Bool BBSQW;
	Bool nBBSQW;
	Bool RQET;   // request temperature conversion
	Bool INTC;   // 1: interupt
	Bool INTS;   // 1: sqw
	Bool A1E;    // enable Alarm1 interrupt
	Bool A2E;
	
	Bool COSF;   // clear OSF
	Bool E32K;
	Bool nE32K;
	Bool A1F;    // clear A1F
	Bool A2F;
}Control_info;

uint8_t BCD2Dec(uint8_t data);
Status BitRead(uint8_t bits, uint8_t bitPos);

void RTCS_Init(void);
Status RTC_SetClock(Clock_info *cin);
Status RTC_SetAlarm1(Alarm1_info *ain1);
Status RTC_SetAlarm2(Alarm2_info *ain2);
Status RTC_SetAging(int8_t ageb);
Status RTC_SetContrl(Control_info *ctn);

Status RTC_Read(Clock_info *cin);
Status rtc_read(uint8_t address, uint8_t len, uint8_t *val);
Status RTC_clearflag(void);

#endif
