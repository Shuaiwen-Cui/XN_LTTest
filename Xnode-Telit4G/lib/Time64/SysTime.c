//Use to get system time
#include "SysTime.h"
#include <lpc43xx_rit.h>
#include "lpc43xx_timer.h"
#include "FreeRTOS.h"

  enum { CLK_RATE = configCPU_CLOCK_HZ }; //TickPerSec, here is 120MHz

  struct _RTimerParam RTP;//Parameters for Hardware Counter

	void RIT_IRQHandler(void) //Counter overflow interrupt
	{
		LPC_RITIMER->CTRL |= 1;
		RTP.th32++;
	}
	
	int RealTimer_Stop(void){ //stop the real-timer
	NVIC_DisableIRQ(RITIMER_IRQn);
	return SUCCESS;
	};

	__inline uint64_t GetTickCount(){
		RTP.tl32=LPC_RITIMER->COUNTER;
		return ((uint64_t)RTP.th32<<32)|RTP.tl32;
	}

	void RealTimer_init()
		{
		RTP.th32=0;
		RTP.tl32=0;
    //RIT_Init(LPC_RITIMER); 
    RTP.Frequency=configCPU_CLOCK_HZ;
		NVIC_SetPriority(RITIMER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
		//NVIC_EnableIRQ(RITIMER_IRQn);
	  }

_clock64_t SysTime_get()
{
	_clock64_t time;
	time.high32 = RTP.th32;
	time.low32=LPC_RITIMER->COUNTER;
	return time;
}

uint64_t SysTime_get64()
{
	uint64_t time;
	time = RTP.th32;
	time <<= 32;
	time += LPC_RITIMER->COUNTER;
	return time;
}

uint32_t SysTime_getHigh32()
{
	uint32_t time;
	time = RTP.th32;
	return time;
}

uint32_t SysTime_getLow32()
{
	uint32_t time;
	time =LPC_RITIMER->COUNTER;
	return time;
}

uint32_t SysTime_getClockRate()
{
	return CLK_RATE;
}

int SysTime_set(_clock64_t t)
{
	// this timer is read-only
	return FAIL;
}

int SysTime_set64(uint64_t t)
{
	// this timer is read-only
	return FAIL;
}

int SysTime_adjust(int32_t t)
{
	// this timer is read-only
	return FAIL;
}

int SysTime_adjust64(int64_t t)
{
	// this timer is read-only
	return FAIL;
}

_clock64_t ConvertSysTime_ticksToUs(_clock64_t ticks)
{
	time_union_t t;
	t.ostime = ticks;
	t.time64 = (uint64_t)((((long double)t.time64) * 1e6) / CLK_RATE);
	return t.ostime;
}

uint64_t ConvertSysTime_ticksToUs64(uint64_t ticks)
{
	return (uint64_t)((((long double)ticks) * 1e6) / CLK_RATE);
}

uint32_t ConvertSysTime_ticksToUs32(uint32_t ticks)
{
	return (uint32_t)((((uint64_t)ticks) * 1e6) / CLK_RATE);
}

_clock64_t ConvertSysTime_usToTicks(_clock64_t us)
{
	time_union_t t;
	t.ostime = us;
	t.time64 = (uint64_t)((((long double)t.time64) * CLK_RATE) / 1e6);
	return t.ostime;
}

uint64_t ConvertSysTime_usToTicks64(uint64_t us)
{
	return (uint64_t)((((long double)us) * CLK_RATE) / 1e6);
}

uint32_t ConvertSysTime_usToTicks32(uint32_t us)
{
	return (uint32_t)((((uint64_t)us) * CLK_RATE) / 1e6);
}
