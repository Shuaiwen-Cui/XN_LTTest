#include <stdio.h>
#include <stdlib.h>

#include "SysTime.h"
#include "LocalTime.h"

int64_t offset_lc = 0; // determined by set64

_clock64_t LocalTime_get()
{
	time_union_t time;
	time.time64 = SysTime_get64() + offset_lc;
	return time.ostime;
}

uint64_t LocalTime_get64()
{
	return SysTime_get64() + offset_lc;
}

uint32_t LocalTime_getHigh32()
{
	time_union_t time;
	time.time64 = SysTime_get64() + offset_lc;
	return time.ostime.high32;
}

uint32_t LocalTime_getLow32()
{
	time_union_t time;
	time.time64 =  SysTime_get64() + offset_lc;
	return time.ostime.low32;
}

uint32_t LocalTime_getClockRate()
{
	return  SysTime_getClockRate();
}

int LocalTime_set(_clock64_t t)
{
	time_union_t time;
	time.ostime = t;
	offset_lc = time.time64 - SysTime_get64();
	return SUCCESS;
}

int LocalTime_set64(uint64_t t)
{
	offset_lc = t -  SysTime_get64();
	return SUCCESS;
}

int LocalTime_adjust(int32_t t)
{
	offset_lc += t;
	return SUCCESS;
}

int LocalTime_adjust64(int64_t t)
{
	offset_lc += t;
	return SUCCESS;
}

/*
result_t LocalTime_setAlarm(_clock64_t t)
{
	time_union_t time;
	time.ostime = t;
	time.time64 -= tpr.offset;
	return  SysTime_setAlarm(time.ostime);
}

result_t LocalTime_setAlarm64(uint64_t t)
{
	t -= tpr.offset;
	return  SysTime_setAlarm64(t);
}

result_t LocalTime_setAlarm32(uint32_t t)
{
	time_union_t time;
	time.ostime.high32 = LocalTime_getHigh32();
	time.ostime.low32 = t;
	time.time64 -= tpr.offset;
	return  SysTime_setAlarm32(time.ostime.low32);
}

result_t LocalTime_alarmFired(uint32_t t)
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
