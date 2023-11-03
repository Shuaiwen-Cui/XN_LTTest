#include <xnode.h>
#include <GenericComm.h>
#include <pmic.h>
#include "SnoozeAlarm.h"
#include "rtc.h"
#include "NodeReset.h"
#include "TriggerSensing.h"
#include "sdcard.h"
#include "RemoteSensing.h"
#include "time.h"
#include "nandflash_k9f1g08u0a.h"

float voltage = 0.f;
float current = 0.f;

#define _DEBUG_               1
#if _DEBUG_
#define PRINTF(...)           lpc_printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifdef SNOOZEALARM
static bool sa_flag = false;

static void sacallback(uint16_t src, void *payload, uint8_t len)
{
	uint8_t i;
	uint16_t *addrs;

	// skip if already staying on
	if (sa_flag) {
		return;
	}

	// check for own address
	for (i = 0, addrs = (uint16_t *)payload; i < len / sizeof (uint16_t); ++i) {
		if (addrs[i] == LOCAL_ADDR) {
			sa_flag = true;
			break;
		}
	} 
}
#endif

bool lowVolt = false;
void CheckBatteryVoltage(void)
{
	pmic_info pi;
	chg_info chg;

	PMIC_Init();
	PMIC_Status(&chg, &pi);
	voltage = chg.Vbat;
	current = chg.Vcur;

#ifndef USB
if (voltage < MIN_VOLTAGE) {
	lowVolt = true;
	lpc_printf("ERROR: voltage too low (%.2fV < %.2fV), resetting...\r\n", voltage, MIN_VOLTAGE);
	die();
}
#endif
}

void SnoozeAlarm_Start(void)
{
#ifdef SNOOZEALARM
	uint32_t i;
#endif
	CheckBatteryVoltage();

#ifdef SNOOZEALARM
	sa_flag = false;

	// initialize GenericComm & register callback
	if (GenericComm_init() != SUCCESS || GenericComm_register(GC_APP_SNOOZEALARM, sacallback) != SUCCESS) {
		die();
	}

	// check for flag
	for (i = 0; i < SNOOZE_ALARM_AWAKE / 100; ++i) {
		vTaskDelay(100);
		if (sa_flag) {
			return;
		}
	}

	// no flag
	SnoozeAlarm_Sleep();
#endif
}

void SnoozeAlarm_Stop(void)
{
#ifdef SNOOZEALARM
	sa_flag = true;
#endif
}

#ifdef FRA 
#include <lpc43xx_scu.h>
#include "RemoteSensing.h"
#include "4GFTP.h"

extern bool iswaitdeploy, SwitchOn;
extern bool DataSendBackFTPDone;
uint8_t task1T, task2T, task3T; // very lazy definition, not even trying to save space - fixed
// J1-12
#define ADS131_TEMP_CONF 0xC,  2, MD_PDN, FUNC4 // PC_2 => GPIO6[1]
#define ADS131_TEMP_DIR 6, 1 << 1, 1
#define ADS131_TEMP_HIGH() GPIO_SetValue(6, 1 << 1)
#define ADS131_TEMP_LOW() GPIO_ClearValue(6, 1 << 1)

// J1-14
#define PC9_CONF 0xC,  9, MD_PDN, FUNC4 // PC_9 => GPIO6[8]
#define PC9_DIR 6, 1 << 8, 1
#define PC9_HIGH() GPIO_SetValue(6, 1 << 8)
#define PC9_LOW() GPIO_ClearValue(6, 1 << 8)

// J1-30
#define PD3_CONF 0xD,  3, MD_PLN, FUNC4 // PD_3 => GPIO6[17]
#define PD3_DIR 6, 1 << 17, 1
#define PD3_HIGH() GPIO_SetValue(6, 1 << 17)
#define PD3_LOW() GPIO_ClearValue(6, 1 << 17)

#define PD5_CONF  	  0xD, 5, MD_PDN, FUNC4	  // 
#define PD5_DIR		  6, 1 << 19,  1                        
#define PD5_HIGH()     GPIO_SetValue(6, 1 << 19)
#define PD5_LOW()      GPIO_ClearValue(6, 1 << 19)

#define PD7_CONF  	  0xD, 7, MD_PDN, FUNC4	  // 
#define PD7_DIR		  6, 1 << 21,  1                        
#define PD7_HIGH()     GPIO_SetValue(6, 1 << 21)
#define PD7_LOW()      GPIO_ClearValue(6, 1 << 21)

void SnoozeAlarm_Sleep(void) // Different in way of wake up time (so they are more synchronized, based on clock instead of based on current time). This function has NVIC_SystemReset at the end as it also needs to deal with turning off 4G modem. NodeReset wraps around this function
{
#ifdef GATEWAY
	uint32_t count;
#endif	
	uint8_t i;
	// Reschedule Clock
	uint8_t cyear, cmonth, cdate, chour, cminute, csecond;
	Alarm1_info ain1; 
	Alarm2_info ain2; 
	Control_info ctn;
	unsigned int unixalarm1 = 4294967295ULL, unixalarm2= 4294967295ULL;
	RTCS_Init();
	// Get time
	gettime(&cyear, &cmonth, &cdate, &chour, &cminute, &csecond);
#ifdef GATEWAY
	if (!DataSendBackFTPDone)
	{
		write_istherefiletosend(0);
	}
	count = read_isFTP();	
	if (count>DATACOUNTHRESHOLD) 
	{
		task1T = 1; // if enough data, reset right away// be VERY CAREFUL here, if something is wrong, node will stuck and waste energy in this loop forever - TUFIXDONE: changed from NVIC_NodeReset() to wake up WITHIN 1 minute, easier to control, and does not disrupt the main framework
	}	
	if (!lowVolt)
	{
		// Setup Alarm 1 based on current time - for waking up to send back data, gateway wakes up every N minutes
		RTC_SetAin1(&ain1, task1T, task3T, &unixalarm1); // use the same function as SN for now as GW can cause the unix check to fire. the bad thing with this is that for task3T, the GW will more often have to work harder since they sensor is not sync well after the GW takes 30 s to read clock
	} else
	{
		RTC_SetAin1(&ain1, 90, task3T, &unixalarm1); // sleep 90 more minutes if tired
	}
#else
	// Setup Alarm 1 based on current time - for waking up to send back data, sensor wakes up only when there's something to send
	if (!lowVolt)
	{
		if (SwitchOn)
		{
			RTC_SetAin1(&ain1, 1, task3T, &unixalarm1); //wake up 1 minute later if a switch is detected on the sensor node
		}else	if (read_tosend())
		{
			RTC_SetAin1(&ain1, task1T, task3T, &unixalarm1); 
		} 
	} else
	{
		RTC_SetAin1(&ain1, 90, task3T, &unixalarm1); // sleep 90 more minutes if tired
	}
#endif
	// Setup Alarm 2 based on current time - for waking up to check health, same for both sensor and gateway
	RTC_SetAin2(&ain2, task2T, &unixalarm2);
  RTC_AlarmControl(&ctn, TRUE , TRUE);
	
	NandFlash_BlockErase(NANDFLASH_BLOCK_UNIXALARMTIME);
	if (unixalarm1 > unixalarm2)
	{
		NandFlash_PageProgram((uint8_t *) &unixalarm2, NANDFLASH_BLOCK_UNIXALARMTIME * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,0, NANDFLASH_RW_PAGE_SIZE);
	} else
	{
		NandFlash_PageProgram((uint8_t *) &unixalarm1, NANDFLASH_BLOCK_UNIXALARMTIME * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,0, NANDFLASH_RW_PAGE_SIZE);
	}
	RTC_SetAlarm1(&ain1);
	RTC_SetAlarm2(&ain2);
	RTC_SetAging(1);
	RTC_SetContrl(&ctn);
	RTC_clearflag();

	PRINTF("CURRENT TIME: %02d:%02d:%02d\n\r",chour,cminute,csecond);
	PRINTF("ALARM 1     : %02d:%02d:%02d\n\r",ain1.hour,ain1.minute,ain1.second);
	PRINTF("ALARM 2     : %02d:%02d:00\n\r",ain2.hour,ain2.minute);
// just combine both snoozealarm_sleep and NodeReset
#ifdef GATEWAY
	adxl362_setup(); // TUFIXDONE: This should be done for GW only, as the setup is alreayd done  for SN.
	// Making sure 4g is off
	scu_pinmux(PD5_CONF);	
	GPIO_SetDir(PD5_DIR);
	PD5_HIGH();	
	TIM_Waitms(200);
	scu_pinmux(PD7_CONF);	
	GPIO_SetDir(PD7_DIR);
	PD7_LOW();	
	TIM_Waitms(200);
#endif
	PD3_HIGH();
	for (i = 0; i < 5; ++i) {
		PC9_HIGH();
		PC9_LOW();
	}
	NVIC_SystemReset(); 
}

#else

void SnoozeAlarm_Sleep(void)
{
	// Reschedule Clock
	Alarm1_info ain1;
	Control_info ctn;
	RTCS_Init();

	// Setup Alarm based on current time - either every N time or at N o'clock
	RTC_SetAin1_In(&ain1, SNOOZE_ALARM_PERIOD);
	RTC_AlarmControl(&ctn, TRUE, FALSE);                                     
	RTC_SetAlarm1(&ain1);
	RTC_SetAging(1);
	RTC_SetContrl(&ctn);
	RTC_clearflag();

	NodeReset();
}
#endif

void SnoozeAlarm_Sleep_noSleep(uint8_t Duration)
{
	// Reschedule Clock
	Alarm1_info ain1;
	Control_info ctn;
	RTCS_Init();

	// Setup Alarm based on current time - either every N time or at N o'clock
	RTC_SetAin1_In(&ain1, Duration);
	RTC_AlarmControl(&ctn, TRUE, FALSE);                                     
	RTC_SetAlarm1(&ain1);
	RTC_SetAging(1);
	RTC_SetContrl(&ctn);
	RTC_clearflag();
}

void SnoozeAlarm_Sleep_Retrieve(void) // same as normal, but changed 1 minute to 40 seconds
{
	// Reschedule Clock
	Clock_info cin_r;
	Alarm1_info ain1;
	Control_info ctn;
	uint8_t buf[19] = {0};
	uint8_t hold;

	RTCS_Init();

	// Read current time in explicit form
	rtc_read(RTC_REG_SEC, 19, buf);
	
	// clock
	cin_r.second = BCD2Dec(buf[0]);
	cin_r.minute = BCD2Dec(buf[1]);
	
	hold = buf[2];
	if (hold & 0x40) {
		cin_r.clock24 = FALSE;
		if (BitRead(hold, 5)) {
			cin_r.AMPM = FALSE;   // PM
		} else {
			cin_r.AMPM = TRUE;   // AM
		}
		cin_r.hour = BCD2Dec(hold & 0x1F);
	} else {
		cin_r.clock24 = TRUE;
		cin_r.hour = BCD2Dec(hold & 0x3F);
	}

	// Setup Alarm based on current time - either every N time or at N o'clock
	ain1.second = cin_r.second + 30;
	ain1.minute = cin_r.minute + ain1.second / 60;
	ain1.hour = cin_r.hour + 0 + ain1.minute / 60;

	ain1.second = ain1.second % 60;
	ain1.minute = ain1.minute % 60;
	ain1.hour = ain1.hour % 24;

	ain1.DoWM = 1;
	ain1.mask = 2; // Fire when all h:m:s match
	ain1.clock24 = TRUE;
	ain1.dydt = FALSE;
	ain1.AMPM = FALSE;

	ctn.EOSC = TRUE; // Enable Oscillator - Bit7-0xE
	ctn.nEOSC = FALSE; // Disable Oscillator
	ctn.BBSQW = FALSE; // Battery-Backed Square-Wave Enable - Bit6-0xE
	ctn.nBBSQW = TRUE; // Battery-Backed Square-Wave Disable
	ctn.RQET = FALSE; // Convert Temperature - Bit5-0xE
	ctn.INTC = TRUE; // Interrupt Control: 0 = output square wave, 1 = output square wave when alarms hit - Bit2-0xE
	ctn.INTS = FALSE; // !INTC
	ctn.A1E = TRUE; // Alarm 1 Interrupt Enable - Bit0-0xE
	ctn.A2E = FALSE; // Alarm 2 Interrupt Enable - Bit1-0xE
	ctn.COSF = FALSE; // Oscillator Stop Flag - Bit7-0xF
	ctn.E32K = FALSE; // Enable 32kHz Output - Bit3-0xF
	ctn.nE32K = TRUE; // !E32K
	ctn.A1F = FALSE; // Alarm 1 Flag (A1F) - write to 0 to clear - Bit1-0xF
	ctn.A2F = FALSE; // Alarm 2 Flag (A1F) - write to 0 to clear - Bit0-0xF

	RTC_SetAlarm1(&ain1);
	RTC_SetAging(1);
	RTC_SetContrl(&ctn);
	RTC_Read(&cin_r);
	RTC_clearflag();

	NodeReset();
}

void SnoozeAlarm_Wake(uint16_t *addrs, uint8_t addrlen)
{
#ifdef SNOOZEALARM
	uint32_t t0, t1;
	uint32_t diff = 0;

	// initialize GenericComm & register callback
	if (GenericComm_init() != SUCCESS) {
		die();
	}

	// broadcast message
	t0 = LPC_RITIMER->COUNTER;
	do {
		GenericComm_bcast(GC_APP_SNOOZEALARM, (uint8_t *)addrs, addrlen * sizeof (uint16_t));
		t1 = LPC_RITIMER->COUNTER;
		if (((t1 > t0) ? t1 - t0 : 0xFFFFFFFFUL - t0 + t1) >= 120000000UL) {
			diff++;
			t0 = LPC_RITIMER->COUNTER;
		}
	} while (diff < SNOOZE_ALARM_PERIOD * 60 + 5);
#endif
}

void SnoozeAlarm_Wake_Retrieve(uint16_t *addrs, uint8_t addrlen)
{
#ifdef SNOOZEALARM
	uint32_t t0, t1;
	uint32_t diff = 0;

	// initialize GenericComm & register callback
	if (GenericComm_init() != SUCCESS) {
		die();
	}

	// broadcast message
	t0 = LPC_RITIMER->COUNTER;
	do {
		GenericComm_bcast(GC_APP_SNOOZEALARM, (uint8_t *)addrs, addrlen * sizeof (uint16_t));
		t1 = LPC_RITIMER->COUNTER;
		if (((t1 > t0) ? t1 - t0 : 0xFFFFFFFFUL - t0 + t1) >= 120000000UL) {
			diff++;
			t0 = LPC_RITIMER->COUNTER;
		}
	} while (diff < 60 + 5);
#endif
}
