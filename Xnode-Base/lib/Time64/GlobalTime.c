#include <stdio.h>
#include <stdlib.h>

#include "SysTime.h"
#include "LocalTime.h"
#include "GlobalTime.h"
#include "debug_frmwrk.h"

int64_t offset = 0; // determined by setClockOffset
int64_t drift = 0;  // determined by setClockDrift
uint64_t lastSyncTime = 0; // determined by setlastSyncTime.

_clock64_t GlobalTime_get()
{
	time_union_t time;
	uint64_t lt = LocalTime_get64();
	time.time64 = lt + drift * (lt - lastSyncTime) + offset;
	return time.ostime;
}

uint64_t GlobalTime_get64()
{
	uint64_t lt = LocalTime_get64();
	
	int64_t gt = (int64_t)lt + (int64_t)offset + drift * ((int64_t)lt - (int64_t)lastSyncTime)/1e9;  //scale by 1e9 and get ticks
	
	//lpc_printf("localtime is %lld, drift is %ld, lastSyncTime is %lld, offset is %lld\r\n", lt, drift, lastSyncTime, offset);
	
	return gt;
}

uint64_t GlobalTime_generate64( uint64_t lt)
{

	int64_t gt = (int64_t)lt + (int64_t)offset + drift * ((int64_t)lt - (int64_t)lastSyncTime)/1e9;  //scale by 1e9 and get ticks
	
	//lpc_printf("localtime is %lld, drift is %ld, lastSyncTime is %lld, offset is %lld\r\n", lt, drift, lastSyncTime, offset);
	
	return gt;
}

uint32_t GlobalTime_getHigh32()
{
	time_union_t time;
	uint64_t lt = LocalTime_get64();
	time.time64 = lt + drift * (lt - lastSyncTime) + offset;
	return time.ostime.high32;
}

uint32_t GlobalTime_getLow32()
{
	time_union_t time;
	uint64_t lt = LocalTime_get64();
	time.time64 = lt + drift * (lt - lastSyncTime) + offset;
	return time.ostime.low32;
}

uint32_t GlobalTime_getClockRate()
{
	return  LocalTime_getClockRate()+drift;
}

int GlobalTime_set(_clock64_t t)
{
	time_union_t time;
	time.ostime = t;
	offset = time.time64 - LocalTime_get64();
	return SUCCESS;
}

int GlobalTime_set64(uint64_t t)
{
	offset = t -  LocalTime_get64();
	return SUCCESS;
}

int GlobalTime_adjust(int32_t t)
{
	offset += t;
	return SUCCESS;
}

int GlobalTime_adjust64(int64_t t)
{
	offset += t;
	return SUCCESS;
}

// should only be called by the TimeSync service
int64_t ClockDrift_getClockDrift()
{
	return drift;
}

int64_t ClockDrift_getClockOffset()
{
	return offset;
}

int ClockDrift_setClockDrift(int64_t d)
{
	drift = d;    // scale of 1e9
	return SUCCESS;
}

int ClockDrift_setClockOffset(int64_t o)
{
	offset = o;
	return SUCCESS;
}

int ClockDrift_setLastSyncTime(uint64_t t)
{
	lastSyncTime = t;
	return SUCCESS;
}

// ConvertTime
_clock64_t ConvertTime_ticksToUs(_clock64_t ticks)
{
	time_union_t t;
	t.ostime = ticks;
	t.time64 = (uint64_t)((((long double)t.time64) * 1e6) / GlobalTime_getClockRate());
	return t.ostime;
}

uint64_t ConvertTime_ticksToUs64(uint64_t ticks)
{
	return (uint64_t)((((long double)ticks) * 1e6) / GlobalTime_getClockRate());
}

uint32_t ConvertTime_ticksToUs32(uint32_t ticks)
{
	return (uint32_t)((((uint64_t)ticks) * 1e6) / GlobalTime_getClockRate());
}

_clock64_t ConvertTime_usToTicks(_clock64_t us)
{
	time_union_t t;
	t.ostime = us;
	t.time64 = (uint64_t)((((long double)t.time64) * GlobalTime_getClockRate()) / 1e6);
	return t.ostime;
}

uint64_t ConvertTime_usToTicks64(uint64_t us)
{
	return (uint64_t)((((long double)us) * GlobalTime_getClockRate()) / 1e6);
}

uint32_t ConvertTime_usToTicks32(uint32_t us)
{
	return (uint32_t)((((uint64_t)us) * GlobalTime_getClockRate()) / 1e6);
}

/*
result_t GlobalTime_setAlarm(_clock64_t t)
{
	time_union_t time;
	time.ostime = t;
	time.time64 -= tpr.offset;
	return  SysTime_setAlarm(time.ostime);
}

result_t GlobalTime_setAlarm64(uint64_t t)
{
	t -= tpr.offset;
	return  SysTime_setAlarm64(t);
}

result_t GlobalTime_setAlarm32(uint32_t t)
{
	time_union_t time;
	time.ostime.high32 = LocalTime_getHigh32();
	time.ostime.low32 = t;
	time.time64 -= tpr.offset;
	return  SysTime_setAlarm32(time.ostime.low32);
}

result_t GlobalTime_alarmFired(uint32_t t)
{
	return SUCCESS;
}

void SerialIOSignal_signalReceived()
{
}

void SerialIOSignal_dataSignalReceived(char *data, uint32_t len)
{
	uint64_t time, tmp;
	struct timeval //
	{
		time_t tv_sec;
		int tv_usec;
	} tv;
	volatile char tstr[50];
	if (strncmp(data, "TSYNC", 5) != 0 || sscanf(data, "TSYNC %u %u\n", (unsigned int *)&tv.tv_sec, (unsigned int *)&tv.tv_usec) != 2) {
		return;
	}
	time = tv.tv_sec;
	time *=  LocalTime_getClockRate();
	tmp = tv.tv_usec;
	tmp *= LocalTime_getClockRate();
	tmp /= (uint64_t)1e6;
	time += tmp;
	LocalTime_set64(time);
	//strftime(tstr, 50, "%Y-%m-%d %H:%M:%S", localtime(&tv.tv_sec));
	trace("Synchronizing time with PC.  Local time now: %s.\r\n", tstr);
}
*/
