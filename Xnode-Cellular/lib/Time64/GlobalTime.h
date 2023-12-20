#ifndef _GLOBALTIME_H_
  #define _GLOBALTIME_H_
  #include "timeX.h"
	
	_clock64_t GlobalTime_get(void);
	
	uint64_t GlobalTime_get64(void);
	
	uint64_t GlobalTime_generate64( uint64_t lt);

	uint32_t GlobalTime_getHigh32(void);
	
	uint32_t GlobalTime_getLow32(void);

	uint32_t GlobalTime_getClockRate(void);
	
	int GlobalTime_set(_clock64_t t);
	
	int GlobalTime_set64(uint64_t t);
	
	int GlobalTime_adjust(int32_t t);
	
	int GlobalTime_adjust64(int64_t t);
	
	int64_t ClockDrift_getClockDrift(void);
	
	int64_t ClockDrift_getClockOffset(void);
	
	int ClockDrift_setClockDrift(int64_t d);
	
	int ClockDrift_setClockOffset(int64_t o);
	
	int ClockDrift_setLastSyncTime(uint64_t t);
	
	_clock64_t ConvertTime_ticksToUs(_clock64_t ticks);
	
	uint64_t ConvertTime_ticksToUs64(uint64_t ticks);
	
	uint32_t ConvertTime_ticksToUs32(uint32_t ticks);
	
	_clock64_t ConvertTime_usToTicks(_clock64_t us);
	
	uint64_t ConvertTime_usToTicks64(uint64_t us);
	
	uint32_t ConvertTime_usToTicks32(uint32_t us);

#endif
