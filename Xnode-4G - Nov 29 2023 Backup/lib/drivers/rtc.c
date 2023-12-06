/**********************************************************************
* @file			rtc.c
* @date			July 3, 2017
* @author		Yuguang Fu
**********************************************************************/

#include <LPC43xx.h>
#include <lpc43xx_scu.h>
#include <debug_frmwrk.h>
#include <lpc43xx_i2c.h>
#include <stdio.h>
#include <string.h>
#include <vcom.h>
#include <stdlib.h>
#include "rtc.h"
#include "led.h"
#include "lpc43xx_timer.h"
#include "lpc43xx_gpio.h"
#include "RemoteSensing.h"
#include "time.h"

#define _DEBUG_               0
#if _DEBUG_
#define PRINTF(...)           lpc_printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

char *Days[]={"null","Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
volatile uint8_t RTC_INT;		// data ready

/*------------- Private functions -------------- */
uint8_t Dec2BCD(uint8_t data)
{
	return (((data/10) * 16) + (data % 10));
}

uint8_t BCD2Dec(uint8_t data)
{
	return (((data/16) * 10) + (data % 16));
}

Status BitSet(uint8_t *bits, uint8_t bitPos)
{
	if(bitPos>=8)
		return ERROR;
		
	*bits|=(1<<bitPos);
	return SUCCESS;
}

Status BitClear(uint8_t *bits, uint8_t bitPos)
{
	if(bitPos>=8)
		return ERROR;
		
	*bits&=~(1<<bitPos);
	return SUCCESS;
}

Status BitRead(uint8_t bits, uint8_t bitPos)
{
	uint8_t bit=0;
	bit=bits&(1<<bitPos);
	
	if(bit==0)
		return ERROR;
	else
		return SUCCESS;
}

Status rtc_write(uint8_t address, uint8_t val)
{
	I2C_M_SETUP_Type data;
  Status status;
	static uint8_t tempbuf[2];

  tempbuf[0] = address;
	tempbuf[1] = val;

  data.sl_addr7bit = RTC_ADDR;
  data.tx_data     = tempbuf;
  data.tx_length   = 2;
  data.rx_data     = 0;
  data.rx_length   = 0;
  data.retransmissions_max = 3;

  status = I2C_MasterTransferData(RTC_I2C, &data, I2C_TRANSFER_POLLING);
  return status;
}

Status rtc_read(uint8_t address, uint8_t len, uint8_t *val)
{
	I2C_M_SETUP_Type data;
  Status status;
	static uint8_t tempbuf[2];

  tempbuf[0] = address;

  data.sl_addr7bit = RTC_ADDR;
  data.tx_data     = tempbuf;
  data.tx_length   = 1;
  data.rx_data     = val;
  data.rx_length   = len;
  data.retransmissions_max = 3;

  status = I2C_MasterTransferData(RTC_I2C, &data, I2C_TRANSFER_POLLING);

  return status;
}

/*------------- Public functions -------------- */
void RTCS_Init(void)
{
	// Initializes the I2Cx peripheral with specified parameter
	I2C_Init(RTC_I2C, RTC_FREQ);
	// Enable I2C
	I2C_Cmd(RTC_I2C, ENABLE);
	
	RTC_INT=0;
	//NVIC_DisableIRQ(PIN_INT7_IRQn);	
	//scu_pinmux(RTC_INT_CONF);
	//GPIO_SetDir(RTC_INT_DIR);
	//RTC_int_init();
	//NVIC_EnableIRQ(PIN_INT7_IRQn);
	
}

Status RTC_SetClock(Clock_info *cin)
{
	uint8_t hold;
	rtc_write(RTC_REG_SEC, Dec2BCD(cin->second));
	rtc_write(RTC_REG_MIN, Dec2BCD(cin->minute));
	
	hold =Dec2BCD(cin->hour);
	if(!cin->clock24)
	{
		BitSet(&hold, 6);
		if (!cin->AMPM)
			BitSet(&hold, 5);
		else
			BitClear(&hold, 5);
	}
	else
		BitClear(&hold, 6);
	rtc_write(RTC_REG_HOU, hold);	
		
	rtc_write(RTC_REG_DAY, Dec2BCD(cin->day));
	rtc_write(RTC_REG_DAT, Dec2BCD(cin->date));
	rtc_write(RTC_REG_MON, Dec2BCD(cin->month));	
	rtc_write(RTC_REG_YER, Dec2BCD(cin->year));	
	
	return SUCCESS;
}

Status RTC_SetAlarm1(Alarm1_info *ain1)
{
	uint8_t hold;
	
	hold=Dec2BCD(ain1->second);
	if(ain1->mask==4)
		BitSet(&hold,7);
	rtc_write(RTC_REG_A1S, hold);
	
	hold=Dec2BCD(ain1->minute);
	if((ain1->mask<5)&&(ain1->mask>2))
		BitSet(&hold, 7);
	rtc_write(RTC_REG_A1M, hold);
	
	hold=Dec2BCD(ain1->hour);
	if((ain1->mask<5)&&(ain1->mask>1))
		BitSet(&hold, 7);
	if(!ain1->clock24)
	{
		BitSet(&hold, 6);
		if(!ain1->AMPM)
			BitSet(&hold, 5);
		else
			BitClear(&hold, 5);
	}
	else
		BitClear(&hold, 6);
	rtc_write(RTC_REG_A1H, hold);
	
	hold=Dec2BCD(ain1->DoWM);
	if((ain1->mask<5)&&(ain1->mask>0))
		BitSet(&hold, 7);
	if(ain1->dydt)
		BitSet(&hold, 6);
	else
		BitClear(&hold, 6);
	rtc_write(RTC_REG_A1D, hold);
	return SUCCESS;
}

Status RTC_SetAlarm2(Alarm2_info *ain2)
{
	uint8_t hold;
	
	hold=Dec2BCD(ain2->minute);
	if(ain2->mask==3)
		BitSet(&hold, 7);
	rtc_write(RTC_REG_A2M, hold);
	
	hold=Dec2BCD(ain2->hour);
	if((ain2->mask<4)&&(ain2->mask>1))
		BitSet(&hold, 7);
	if(!ain2->clock24)
	{
		BitSet(&hold, 6);
		if(!ain2->AMPM)
			BitSet(&hold, 5);
		else
			BitClear(&hold, 5);
	}
	else
		BitClear(&hold, 6);
	rtc_write(RTC_REG_A2H, hold); 
	
	hold=Dec2BCD(ain2->DoWM);
	if((ain2->mask<4)&&(ain2->mask>0))
		BitSet(&hold, 7);
	if(ain2->dydt)
		BitSet(&hold, 6);
	else
		BitClear(&hold, 6);
	rtc_write(RTC_REG_A2D, hold); 
  return SUCCESS;	
}

Status RTC_SetAging(int8_t ageb)
{
	uint8_t hold;
	hold=abs(ageb);
	if(ageb<0)
	{
		hold=~hold;
		hold+=1;
	}
	rtc_write(RTC_REG_AGE, hold);
	return SUCCESS;
}

Status RTC_SetContrl(Control_info *ctn)
{
	uint8_t hold;

	//control register 
	rtc_read(RTC_REG_CTL, 1, &hold);  
	if(ctn->EOSC)
		BitClear(&hold, 7);	
	if(ctn->nEOSC)
		BitSet(&hold, 7);	
		
	if(ctn->BBSQW)
		BitSet(&hold, 6);	
	if(ctn->nBBSQW)
		BitClear(&hold, 6);
	
	if(ctn->RQET)
		BitSet(&hold, 5);
	
	BitClear(&hold ,3);// 1Hz square wave
	BitClear(&hold ,4);// 1Hz square wave
	if(ctn->INTC)
		BitSet(&hold, 2);
	if(ctn->INTS)
		BitClear(&hold ,2);
		
	if(ctn->A1E)
		BitSet(&hold, 0);
	else
		BitClear(&hold, 0);
	
	if(ctn->A2E)
		BitSet(&hold, 1);
	else
		BitClear(&hold, 1);	
	
	rtc_write(RTC_REG_CTL, hold);	
	
	//status register
	rtc_read(RTC_REG_STU, 1, &hold);
	
	if(ctn->COSF)       
		BitClear(&hold, 7);
	
	if(ctn->E32K)
		BitSet(&hold, 3);
	if(ctn->nE32K)
		BitClear(&hold, 3);
	
	if(ctn->A1F)
		BitClear(&hold, 0);
	
	if(ctn->A2F)
		BitClear(&hold, 1);
	
	return SUCCESS; 
}

Status RTC_Read(Clock_info *cin)
{
	uint8_t hold;
	int j;
	uint8_t A1Reg=0;
	uint8_t A2Reg=0;
	short degrees=0;
	uint8_t buf[19]={0};
	Alarm1_info ain1;
	Alarm2_info ain2;

	rtc_read(RTC_REG_SEC, 19, buf);
	// clock
	cin->second=BCD2Dec(buf[0]);
	cin->minute=BCD2Dec(buf[1]);
	
	hold=buf[2];
	if(hold & 0x40)
	{
		cin->clock24=FALSE;
		if(BitRead(hold, 5))
			cin->AMPM=FALSE;   // PM
		else
			cin->AMPM=TRUE;   // AM
		cin->hour=BCD2Dec(hold & 0x1F);
	}
	else
	{
		cin->clock24=TRUE;
		cin->hour=BCD2Dec(hold & 0x3F);
	}
	
	cin->day=BCD2Dec(buf[3]);
	cin->date=BCD2Dec(buf[4]);
	cin->month=BCD2Dec(buf[5]& 0x7F);
	cin->year=BCD2Dec(buf[6]);
	
	/* --------- print ----------- */
	lpc_printf("Clock: %02d:%02d:%02d", cin->hour, cin->minute, cin->second);
	if(!cin->clock24)
	{
		if(cin->AMPM)
			lpc_printf("(AM),");
		else
			lpc_printf("(PM),");
	}
	else
		lpc_printf("(24h),");	
	lpc_printf(", %2d/%2d/%2d%2d\r\n", cin->month, cin->date, 20, cin->year);
	
	// Alarm 1
	hold=buf[7];
	if(BitRead(hold, 7))
		BitSet(&A1Reg, 0);
	ain1.second=BCD2Dec(hold& 0x7F);
	
	hold=buf[8];
	if(BitRead(hold, 7))
		BitSet(&A1Reg, 1);
	ain1.minute=BCD2Dec(hold& 0x7F);
	
	hold=buf[9];
	if(BitRead(hold, 7))
		BitSet(&A1Reg, 2);
	if(BitRead(hold, 6))
	{
		ain1.clock24=FALSE;
		if(BitRead(hold, 5))
			ain1.AMPM=FALSE;
		else
			ain1.AMPM=TRUE;
		ain1.hour=BCD2Dec(hold & 0x1F);
	}
	else
	{
		ain1.clock24=TRUE;
		ain1.hour=BCD2Dec(hold & 0x3F);
	}
	
	hold=buf[10];
	if(BitRead(hold, 7))
		BitSet(&A1Reg, 3);
	if(BitRead(hold, 6))
	{
		ain1.dydt=TRUE;
		ain1.DoWM=BCD2Dec(hold & 0xF);
	}
	else
	{
		ain1.dydt=FALSE;
		ain1.DoWM=BCD2Dec(hold & 0x3F);
	}
	
	/* --------- print ----------- */
	PRINTF("Alarm #1: %02d:%02d:%02d", ain1.hour, ain1.minute, ain1.second);
	if(!ain1.clock24)
	{
		if(ain1.AMPM)
			PRINTF("(AM),");
		else
			PRINTF("(PM),");
	}
	else
		PRINTF("(24h),");	
	if(ain1.dydt)
		PRINTF("%s in every week,", Days[ain1.DoWM]);
	else
		PRINTF("%2d date in every month,", ain1.DoWM);
	switch(A1Reg)
	{
		case 15:
			PRINTF("Alarm mode: once per second\r\n");break;
		case 14:
			PRINTF("Alarm mode: when seconds match\r\n");break;
		case 12:
			PRINTF("Alarm mode: when minutes and seconds match\r\n");break;
		case 8:
			PRINTF("Alarm mode: when hours, minutes and seconds match\r\n");break;
		case 0:
			if(ain1.dydt)
				PRINTF("Alarm mode: when day, hours, minutes and seconds match\r\n");
			else
				PRINTF("Alarm mode: when date, hours, minutes and seconds match\r\n");
			break;
		default:
			PRINTF("configuration error\r\n");break;
	}
	
	// Alarm 2
	hold=buf[11];
	if(BitRead(hold, 7))
		BitSet(&A2Reg, 0);
	ain2.minute=BCD2Dec(hold & 0x7F);
	
	hold=buf[12];
	if(BitRead(hold, 7))
		BitSet(&A2Reg, 1);
	if(BitRead(hold, 6))
	{
		ain2.clock24=FALSE;
		if(BitRead(hold, 5))
			ain2.AMPM=FALSE;
		else
			ain2.AMPM=TRUE;
		ain2.hour=BCD2Dec(hold & 0x1F);
	}
	else
	{
		ain2.clock24=TRUE;
		ain2.hour=BCD2Dec(hold & 0x3F);
	}
	
	hold=buf[13];
	if(BitRead(hold, 7))
		BitSet(&A2Reg, 2);
	if(BitRead(hold, 6))
	{
		ain2.dydt=TRUE;
		ain2.DoWM=BCD2Dec(hold & 0xF);
	}
	else
	{
		ain2.dydt=FALSE;
		ain2.DoWM=BCD2Dec(hold & 0x3F);
	}
	
	/* --------- print ----------- */
	PRINTF("Alarm #2: %02d:%02d", ain2.hour, ain2.minute);
	if(!ain2.clock24)
	{
		if(ain2.AMPM)
			PRINTF("(AM),");
		else
			PRINTF("(PM),");
	}
	else
		PRINTF("(24h),");	
	if(ain2.dydt)
		PRINTF("%s in every week,", Days[ain2.DoWM]);
	else
		PRINTF("%2d date in every month,", ain2.DoWM);
	switch(A2Reg)
	{
		case 7:
			PRINTF("Alarm mode: once per second (00 seconds of every minute)\r\n");break;
		case 6:
			PRINTF("Alarm mode: when minutes match\r\n");break;
		case 4:
			PRINTF("Alarm mode: when hours and minutes match\r\n");break;
		case 0:
			if(ain2.dydt)
				PRINTF("Alarm mode: when day, hours and minutes match\r\n");
			else
				PRINTF("Alarm mode: when date, hours and minutes match\r\n");
			break;
		default:
			PRINTF("configuration error\r\n");break;
	}
	
	/*---- print ----*/
	PRINTF("Control byte:");
	for(j=7; j>=0; j--)
	{
		PRINTF("%d", BitRead(buf[14], j));
	}
	PRINTF("\r\n");
	
	PRINTF("Status byte:");
	for(j=7; j>=0; j--)
	{
		PRINTF("%d", BitRead(buf[15], j));
	}
	PRINTF("\r\n");
	// Temperature & Aging
	hold=buf[16];
	if(BitRead(hold, 7))
	{
		hold=~hold+1;
		PRINTF("Aging offset: %d\r\n",hold*(-1));	
	}
	else
	{
		PRINTF("Aging offset: %d\r\n",hold);	
	}
	
	hold=buf[17];
	degrees = hold <<8;
	degrees |= buf[18];
	degrees/=256;
	
	/*---- print ----*/
	PRINTF("Temperature: %d C\r\n", degrees);
	
	return SUCCESS;
}

void RTC_clearflag()
{
	uint8_t buf=0;	
	rtc_read(RTC_REG_STU, 1, &buf); 
	BitClear(&buf, 0);// Clear A1F
	BitClear(&buf, 1);// Clear A2F
	rtc_write(RTC_REG_STU, buf);
}
#define min(a, b) ((a < b) ? a : b)

void RTC_SetAin1(Alarm1_info *ain1, uint8_t task1T_local, uint8_t task3T_local, unsigned int *unixalarm1_Local)
{
	uint8_t cyear, cmonth, cdate, chour, cminute, csecond;
	struct tm tm, tm_alarm;
	uint8_t DL = 0;
	time_t epoch=0;	
#ifndef GATEWAY // be very careful here, while debugging, the USB can delay more and cause no data to be sent back
	DL = 2; // Sensor wakes up 2 sec before the gateway
#endif	

	gettime(&cyear, &cmonth, &cdate, &chour, &cminute, &csecond);

	// Convert to write to NAND
	tm.tm_sec = csecond;
	tm.tm_min = cminute;
	tm.tm_hour = chour;
	tm.tm_mday = cdate;
	tm.tm_mon = cmonth-1; // range 0 - 11
	tm.tm_year = cyear+2000-1900; // since 1900
	tm.tm_isdst = -1; // unknown
	epoch = mktime(&tm);
	*unixalarm1_Local = ((int) (epoch/(task1T_local*60)) + 1) * (task1T_local*60)-DL;
	// if too close to current time, postpone
	if ((int) *unixalarm1_Local - epoch < 15)
	{
		*unixalarm1_Local = ((int) (epoch/(task1T_local*60)) + 2) * (task1T_local*60)-DL;
	}
	tm_alarm = *localtime(unixalarm1_Local);

	// RTC
	ain1->second = tm_alarm.tm_sec;
	ain1->minute = tm_alarm.tm_min;  
	ain1->hour = tm_alarm.tm_hour;
	ain1->DoWM = tm_alarm.tm_mday;
	ain1->mask = 2; // Fire when all h:m:s match
	ain1->clock24 = TRUE;
	ain1->dydt = FALSE;
	ain1->AMPM = FALSE;	
}

void RTC_SetAin2(Alarm2_info *ain2,uint8_t task2T_local, unsigned int *unixalarm2_Local)
{
	uint8_t cyear, cmonth, cdate, chour, cminute, csecond;
	struct tm tm, tm_alarm;
	time_t epoch=0;
	
	gettime(&cyear, &cmonth, &cdate, &chour, &cminute, &csecond);

	// Convert to write to NAND
	tm.tm_sec = csecond;
	tm.tm_min = cminute;
	tm.tm_hour = chour;
	tm.tm_mday = cdate;
	tm.tm_mon = cmonth-1; // range 0 - 11
	tm.tm_year = cyear+2000-1900; // since 1900
	tm.tm_isdst = -1; // unknown
	epoch = mktime(&tm);

	*unixalarm2_Local = ((int)(epoch/3600) + 1) * task2T_local * 3600;
	// if too close to current time, postpone
	if ((int) *unixalarm2_Local - epoch < 15)
	{
		*unixalarm2_Local = ((int)(epoch/3600) + 2) * task2T_local * 3600;
	}
	tm_alarm = *localtime(unixalarm2_Local);
	
		// RTC
	ain2->minute=tm_alarm.tm_min;
	ain2->hour=tm_alarm.tm_hour;
	ain2->DoWM=tm_alarm.tm_mday;
	ain2->mask=1; // Fire when h:m match
	ain2->clock24=TRUE;
	ain2->dydt=FALSE;
	ain2->AMPM=FALSE;	
}

void RTC_AlarmControl(Control_info *ctn, Bool A1Enable, Bool A2Enable)
{
	ctn->EOSC = TRUE; // Enable Oscillator - Bit7-0xE
	ctn->nEOSC = FALSE; // Disable Oscillator
	ctn->BBSQW = FALSE; // Battery-Backed Square-Wave Enable - Bit6-0xE
	ctn->nBBSQW = TRUE; // Battery-Backed Square-Wave Disable
	ctn->RQET = FALSE; // Convert Temperature - Bit5-0xE
	ctn->INTC = TRUE; // Interrupt Control: 0 = output square wave, 1 = output square wave when alarms hit - Bit2-0xE
	ctn->INTS = FALSE; // !INTC
	ctn->A1E = A1Enable; // Alarm 1 Interrupt Enable - Bit0-0xE - For all cases except next case
	ctn->A1F = FALSE; // Alarm 1 Flag (A1F) - write to 0 to clear - Bit1-0xF
	ctn->A2E = A2Enable; // Alarm 2 Interrupt Enable - Bit1-0xE		
	ctn->A2F = FALSE; // Alarm 2 Flag (A1F) - write to 0 to clear - Bit0-0xF
	ctn->COSF = FALSE; // Oscillator Stop Flag - Bit7-0xF
	ctn->E32K = FALSE; // Enable 32kHz Output - Bit3-0xF
	ctn->nE32K = TRUE; // !E32K
}

void RTC_SetAin1_In(Alarm1_info *ain1, uint8_t Duration) // Set alarm 1 to fire in N minutes
{
	uint8_t cyear, cmonth, cdate, chour, cminute, csecond;
	gettime(&cyear, &cmonth, &cdate, &chour, &cminute, &csecond);

	ain1->second = csecond + 0;
	ain1->minute = cminute + Duration + ain1->second / 60;
	ain1->hour = chour + 0 + ain1->minute / 60;

	ain1->second = ain1->second % 60;
	ain1->minute = ain1->minute % 60;
	ain1->hour = ain1->hour % 24;

	ain1->DoWM = 1;
	ain1->mask = 2; // Fire when all h:m:s match
	ain1->clock24 = TRUE;
	ain1->dydt = FALSE;
	ain1->AMPM = FALSE;
}

void gettime(uint8_t *yy, uint8_t *momo, uint8_t *dd, uint8_t *hh, uint8_t *mm, uint8_t *ss)
{
	uint8_t buf[19]={0};
	uint8_t hold;

	RTCS_Init();
	rtc_read(RTC_REG_SEC, 19, buf);
	*ss=BCD2Dec(buf[0]);
	*mm=BCD2Dec(buf[1]);
	hold=buf[2];
	if(hold & 0x40)
	{
		*hh=BCD2Dec(hold & 0x1F);
	}
	else
	{
		*hh=BCD2Dec(hold & 0x3F);
	}
	*dd=BCD2Dec(buf[4]);
	*momo=BCD2Dec(buf[5]& 0x7F);
	*yy=BCD2Dec(buf[6]);
}
