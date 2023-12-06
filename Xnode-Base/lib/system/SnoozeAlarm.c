#include <xnode.h>
#include <GenericComm.h>
#include <pmic.h>
#include "SnoozeAlarm.h"
#include "rtc.h"
#include "NodeReset.h"

float voltage = 0.f;
float current = 0.f;

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

void SnoozeAlarm_Sleep(void)
{
	// Reschedule Clock
	Clock_info cin_r;
	Alarm1_info ain1;
	Control_info ctn;
	uint8_t buf[19] = {0};
	uint8_t hold;

	RTCS_Init();

	// Read current time in explicit form
	
	if(!rtc_read(RTC_REG_SEC, 19, buf))
		die();
	
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
	ain1.second = cin_r.second + 0;
	ain1.minute = cin_r.minute + SNOOZE_ALARM_PERIOD + ain1.second / 60;
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
	if(!RTC_SetAlarm1(&ain1) || 
		!RTC_SetAging(1) || 
		!RTC_SetContrl(&ctn) || 
		!RTC_Read(&cin_r) ||
		!RTC_clearflag())
		die();
	
	NodeReset();
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
