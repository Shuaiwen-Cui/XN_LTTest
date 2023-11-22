#include <xnode.h>
#include <lpc43xx_gpio.h>
#include <lpc43xx_scu.h>
#include <lpc43xx_timer.h>
#include <triggersensing.h>
#include "NodeReset.h"
#include "SnoozeAlarm.h"

#define DEBUG_DIE 1

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

#ifdef FRA	
// GPIO pins to turn off 4G 
#define PD5_CONF  	  0xD, 5, MD_PDN, FUNC4	  // 
#define PD5_DIR		  6, 1 << 19,  1                        
#define PD5_HIGH()     GPIO_SetValue(6, 1 << 19)
#define PD5_LOW()      GPIO_ClearValue(6, 1 << 19)

#define PD7_CONF  	  0xD, 7, MD_PDN, FUNC4	  // 
#define PD7_DIR		  6, 1 << 21,  1                        
#define PD7_HIGH()     GPIO_SetValue(6, 1 << 21)
#define PD7_LOW()      GPIO_ClearValue(6, 1 << 21)
#endif
extern uint8_t TrigSenIdx;
void NodeReset_Init(void)
{
#ifdef GATEWAY
	#ifdef FRA	// to makesure 4G is off
	scu_pinmux(PD5_CONF);	
	GPIO_SetDir(PD5_DIR);
	PD5_HIGH();	
	TIM_Waitms(200);
	scu_pinmux(PD7_CONF);	
	GPIO_SetDir(PD7_DIR);
	PD7_LOW();	
	TIM_Waitms(200);
	#endif
#endif	
	
	// sensorboard power
	scu_pinmux(ADS131_TEMP_CONF);
	GPIO_SetDir(ADS131_TEMP_DIR);
	ADS131_TEMP_HIGH();
	// sleep timer
	scu_pinmux(PC9_CONF);
	scu_pinmux(PD3_CONF);
	GPIO_SetDir(PC9_DIR);
	GPIO_SetDir(PD3_DIR);
	PC9_HIGH();
	PD3_LOW();
}
#ifdef FRA	
void NodeReset(void)		// Gateway and Sensor are treated equally, both have enough sleep
{
	SnoozeAlarm_Sleep();
}

void die(void)
{
#ifndef GATEWAY
	uint8_t i=0;
#endif
	LED_RGB(1,1,1);
	TIM_Waitms(100);
#ifndef GATEWAY
	PD3_HIGH();
	for (i = 0; i < 5; ++i) {
		PC9_HIGH();
		PC9_LOW();
	}
#endif
	NodeReset();
}


#else
void NodeReset(void)
{
#ifndef GATEWAY
	uint8_t i;
	PD3_HIGH();
	for (i = 0; i < 5; ++i) {
		PC9_HIGH();
		PC9_LOW();
	}
#endif
	NVIC_SystemReset();
}

void die(void)
{
	LED_RGB(1,1,1);
#if DEBUG_DIE
	lpc_printf("!!!DEBUG DIE INSTEAD OF RESET!!!\r\n");
	for (;;) { __WFI(); }
#else
	TIM_Waitms(1000);
	NodeReset();
#endif
}
#endif
