#include <xnode.h>
#include <lpc43xx_gpio.h>
#include <lpc43xx_scu.h>
#include <lpc43xx_timer.h>
#include <triggersensing.h>
#include "NodeReset.h"

#define DEBUG_DIE 0

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
extern uint8_t TrigSenIdx;

void NodeReset_Init(void)
{
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

void NodeReset(void)
{
#ifndef GATEWAY
	uint8_t i;
	uint32_t j;
	PD3_HIGH();
	//TIM_Waitms(10);
	for (j = 0; j < 1000000; ++j) {}
	for (i = 0; i < 5; ++i) {
		PC9_HIGH();
		//TIM_Waitms(10);
		for (j = 0; j < 1000000; ++j) {}
		PC9_LOW();
		//TIM_Waitms(10);
		for (j = 0; j < 1000000; ++j) {}
	}
#endif
	NVIC_SystemReset();
}

void die(void)
{
	uint32_t j;
	LED_RGB(1,1,1);
#if DEBUG_DIE
	lpc_printf("!!!DEBUG DIE INSTEAD OF RESET!!!\r\n");
	for (;;) { __WFI(); }
#else
	//TIM_Waitms(1000);
	for (j = 0; j < 200000000UL; ++j) {}
	NodeReset();
#endif
}
