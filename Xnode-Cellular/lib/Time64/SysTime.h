#ifndef _SYSTIME_H_
  #define _SYSTIME_H_
  #include "timeX.h"

	void RIT_IRQHandler(void); //Counter overflow interrupt

	void RealTimer_init(void);
	int RealTimer_Stop(void);
	_clock64_t SysTime_get(void);
	uint64_t   SysTime_get64(void);
	uint32_t   SysTime_getHigh32(void);
	uint32_t   SysTime_getLow32(void);
	uint32_t   SysTime_getClockRate(void);
	int   SysTime_set(_clock64_t t);
	int   SysTime_set64(uint64_t t);
	int   SysTime_adjust(int32_t t);
	int   SysTime_adjust64(int64_t t);
/*
  result_t   SysTime_setAlarm(_clock64_t t);
  result_t   SysTime_setAlarm64(uint64_t t);
  result_t   SysTime_setAlarm32(uint32_t t);
*/
	_clock64_t ConvertSysTime_ticksToUs(_clock64_t ticks);
	uint64_t   ConvertSysTime_ticksToUs64(uint64_t ticks);
	uint32_t   ConvertSysTime_ticksToUs32(uint32_t ticks);
	_clock64_t ConvertSysTime_usToTicks(_clock64_t us);
	uint64_t   ConvertSysTime_usToTicks64(uint64_t us);
	uint32_t   ConvertSysTime_usToTicks32(uint32_t us);

#endif
