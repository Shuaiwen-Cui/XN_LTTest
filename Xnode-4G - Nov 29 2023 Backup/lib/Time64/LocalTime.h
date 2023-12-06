#ifndef _LOCALTIME_H_
  #define _LOCALTIME_H_
  #include "timeX.h"
	
	_clock64_t LocalTime_get(void);
	
	uint64_t LocalTime_get64(void);

	uint32_t LocalTime_getHigh32(void);
	
	uint32_t LocalTime_getLow32(void);

	uint32_t LocalTime_getClockRate(void);
	
	int LocalTime_set(_clock64_t t);
	
	int LocalTime_set64(uint64_t t);
	
	int LocalTime_adjust(int32_t t);
	
	int LocalTime_adjust64(int64_t t);

#endif
