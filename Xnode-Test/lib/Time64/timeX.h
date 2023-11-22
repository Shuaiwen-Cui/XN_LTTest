#ifndef _TIME_H_
#define _TIME_H_

#include <stdint.h>
#include <lpc_types.h>
#include <globalconstants.h>
//--Realtimer------->
typedef struct _RTimerParam //Parameters for Realtime Timer
{
	uint32_t th32;   //high part of the ticker
	uint32_t tl32;   // low part of the ticker
	uint32_t Frequency;
} _RTimerParam;

typedef struct {
	uint32_t  low32;
	uint32_t  high32;
} _clock64_t; /*old version is _clock64_t*/

typedef union {
	_clock64_t ostime; /*old is ostime */
	  uint64_t time64;
} time_union_t;

#endif /* _TIME_H_ */
