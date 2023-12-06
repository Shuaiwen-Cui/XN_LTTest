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
	LED_RGB(1,0,0);
	if(!rtc_write(RTC_REG_SEC, Dec2BCD(cin->second))||
		!rtc_write(RTC_REG_MIN, Dec2BCD(cin->minute)))
		return ERROR;
	
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
	if(!rtc_write(RTC_REG_HOU, hold))
		return ERROR;
	if(!rtc_write(RTC_REG_DAY, Dec2BCD(cin->day))||
	!rtc_write(RTC_REG_DAT, Dec2BCD(cin->date))||
	!rtc_write(RTC_REG_MON, Dec2BCD(cin->month))||	
	!rtc_write(RTC_REG_YER, Dec2BCD(cin->year)))
		return ERROR;
	
	return SUCCESS;
}

Status RTC_SetAlarm1(Alarm1_info *ain1)
{
	uint8_t hold;
	
	hold=Dec2BCD(ain1->second);
	if(ain1->mask==4)
		BitSet(&hold,7);
	if(!rtc_write(RTC_REG_A1S, hold))
		return ERROR;
	
	hold=Dec2BCD(ain1->minute);
	if((ain1->mask<5)&&(ain1->mask>2))
		BitSet(&hold, 7);

	if(!rtc_write(RTC_REG_A1M, hold))
		return ERROR;
	
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

	if(!rtc_write(RTC_REG_A1H, hold))
		return ERROR;
	
	hold=Dec2BCD(ain1->DoWM);
	if((ain1->mask<5)&&(ain1->mask>0))
		BitSet(&hold, 7);
	if(ain1->dydt)
		BitSet(&hold, 6);
	else
		BitClear(&hold, 6);

	if(!rtc_write(RTC_REG_A1D, hold))
		return ERROR;

	return SUCCESS;
}

Status RTC_SetAlarm2(Alarm2_info *ain2)
{
	uint8_t hold;
	
	hold=Dec2BCD(ain2->minute);
	if(ain2->mask==3)
		BitSet(&hold, 7);
	if(!rtc_write(RTC_REG_A2M, hold))
		return ERROR;
	
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
	if(!rtc_write(RTC_REG_A2H, hold))
		return ERROR;
	
	hold=Dec2BCD(ain2->DoWM);
	if((ain2->mask<4)&&(ain2->mask>0))
		BitSet(&hold, 7);
	if(ain2->dydt)
		BitSet(&hold, 6);
	else
		BitClear(&hold, 6);
	if(!rtc_write(RTC_REG_A2D, hold))
		return ERROR;
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
	return (rtc_write(RTC_REG_AGE, hold));
}

Status RTC_SetContrl(Control_info *ctn)
{
	uint8_t hold;

	//control register 
	if(!rtc_read(RTC_REG_CTL, 1, &hold))
		return ERROR;

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

	if(!rtc_write(RTC_REG_CTL, hold))
			return ERROR;
	
	//status register
	if(!rtc_read(RTC_REG_STU, 1, &hold))
		return ERROR;
	
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
	uint8_t cntb=0;
	uint8_t statusb=0;
	short aging=0, degrees=0;
	uint8_t buf[19]={0};
	Alarm1_info ain1;
	Alarm2_info ain2;

	if(!rtc_read(RTC_REG_SEC, 19, buf))
		return ERROR;
	
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
	lpc_printf("%s, %2d/%2d/%2d%2d\r\n",Days[cin->day], cin->month, cin->date, 20, cin->year);
	
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
	lpc_printf("Alarm #1: %02d:%02d:%02d", ain1.hour, ain1.minute, ain1.second);
	if(!ain1.clock24)
	{
		if(ain1.AMPM)
			lpc_printf("(AM),");
		else
			lpc_printf("(PM),");
	}
	else
		lpc_printf("(24h),");	
	if(ain1.dydt)
		lpc_printf("%s in every week,", Days[ain1.DoWM]);
	else
		lpc_printf("%2d date in every month,", ain1.DoWM);
	switch(A1Reg)
	{
		case 15:
			lpc_printf("Alarm mode: once per second\r\n");break;
		case 14:
			lpc_printf("Alarm mode: when seconds match\r\n");break;
		case 12:
			lpc_printf("Alarm mode: when minutes and seconds match\r\n");break;
		case 8:
			lpc_printf("Alarm mode: when hours, minutes and seconds match\r\n");break;
		case 0:
			if(ain1.dydt)
				lpc_printf("Alarm mode: when day, hours, minutes and seconds match\r\n");
			else
				lpc_printf("Alarm mode: when date, hours, minutes and seconds match\r\n");
			break;
		default:
			lpc_printf("configuration error\r\n");break;
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
	lpc_printf("Alarm #2: %02d:%02d", ain2.hour, ain2.minute);
	if(!ain2.clock24)
	{
		if(ain2.AMPM)
			lpc_printf("(AM),");
		else
			lpc_printf("(PM),");
	}
	else
		lpc_printf("(24h),");	
	if(ain2.dydt)
		lpc_printf("%s in every week,", Days[ain2.DoWM]);
	else
		lpc_printf("%2d date in every month,", ain2.DoWM);
	switch(A2Reg)
	{
		case 7:
			lpc_printf("Alarm mode: once per second (00 seconds of every minute)\r\n");break;
		case 6:
			lpc_printf("Alarm mode: when minutes match\r\n");break;
		case 4:
			lpc_printf("Alarm mode: when hours and minutes match\r\n");break;
		case 0:
			if(ain2.dydt)
				lpc_printf("Alarm mode: when day, hours and minutes match\r\n");
			else
				lpc_printf("Alarm mode: when date, hours and minutes match\r\n");
			break;
		default:
			lpc_printf("configuration error\r\n");break;
	}
	
	// Control
	cntb=buf[14];
	statusb=buf[15];
	
	/*---- print ----*/
	lpc_printf("Control byte:");
	for(j=7; j>=0; j--)
	{
		lpc_printf("%d", BitRead(cntb, j));
	}
	lpc_printf("\r\n");
	
	lpc_printf("Status byte:");
	for(j=7; j>=0; j--)
	{
		lpc_printf("%d", BitRead(statusb, j));
	}
	lpc_printf("\r\n");
	// Temperature & Aging
	hold=buf[16];
	if(BitRead(hold, 7))
	{
		hold=~hold+1;
		aging=hold*(-1);
	}
	else
		aging=hold;
	
	hold=buf[17];
	degrees = hold <<8;
	degrees |= buf[18];
	degrees/=256;
	
	/*---- print ----*/
	lpc_printf("Temperature: %d C, Aging offset: %d\r\n", degrees, aging);
	
	return SUCCESS;
}

Status RTC_clearflag()
{
	uint8_t buf=0;	

	if(!rtc_read(RTC_REG_STU, 0, &buf)) // Clear A1F
		return ERROR;
	BitClear(&buf, 0);
	if(!rtc_write(RTC_REG_STU, buf))
		return ERROR;
	return SUCCESS;
}
