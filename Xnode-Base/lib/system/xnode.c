#include <xnode.h>
#include <lpc43xx_cgu.h>
#include <lpc43xx_evrt.h>
#include <lpc43xx_rit.h>
#include <lpc43xx_timer.h>
#include <lpc43xx_wwdt.h>
#include <stdio.h>
#include <timers.h>
#include <timesync.h>
#include <vcom.h>
#include <GenericComm.h>
#include <Radio.h>
#include <RemoteSensing.h>
#include <SnoozeAlarm.h>
#include <TriggerSensing.h>
#include <Utils.h>
#include <app.h>
#include "timesync.h"

#ifdef USB
static const char header1[] =
  "\r\n\r\n\r\n"
  "-------------------------------------------------------------------------------\r\n"
  "- Embedor Technologies Xnode Smart Sensor Ver. 2.6\r\n"
  "- * MCU: LPC4357\r\n"
  "- * Core: ARM Cortex-M4F\r\n"
  "- * OS: FreeRTOS 10.2.1\r\n"
  "- * Sensor: 24-bit acceleration\r\n"
  "- * Core freq: %u MHz\r\n"
  "-------------------------------------------------------------------------------\r\n"
  "\r\n";
#endif

void TIM_SemInit(void);
int SDCard_Init(void);
uint8_t TrigSenIdx = 0;
bool trig_active = false;

// Watchdog time out in 5 seconds
#define WDT_INTERRUPT_TIMEOUT 	5000000		// max value is (0xFFFFFF*4)/12000000 = 5.59s
// Watchdog warn in 1 second
#define WDT_WARNING_VALUE       1000000		// this value is said for generating interrupt but seems like WDT_INTERRUPT_TIMEOUT is the one that take care of this, so this variable is somewhat useless

static void Timer_Init(void)
{
	TIM_TIMERCFG_Type TIM_ConfigStruct;
	// Initialize Timer0, prescale count time of 1us
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;
	TIM_Init(LPC_TIMER0, TIM_TIMER_MODE, &TIM_ConfigStruct);
	// Initialize RITimer
	RIT_Init(LPC_RITIMER);
	RIT_Cmd(LPC_RITIMER, ENABLE);
}

static void wdtInit(void)
{
	st_Wdt_Config wdtCfg;
	// Initiate WWDT
  WWDT_Init();
  // Configure WWDT
  wdtCfg.wdtReset = ENABLE; // enable resetting
  wdtCfg.wdtProtect = DISABLE; 
  wdtCfg.wdtTmrConst = WDT_INTERRUPT_TIMEOUT;	// 
  wdtCfg.wdtWarningVal = WDT_WARNING_VALUE;	// Seems like this one is not used in WWDT_Configure
  wdtCfg.wdtWindowVal = WWDT_WINDOW_MAX;	// Seems like this one is not used in WWDT_Configure
  WWDT_Configure(wdtCfg);
	// Start watchdog
  WWDT_Start();
}

static void feedwdt(TimerHandle_t pxTimer)
{
	WWDT_Feed();
}

static void vAppMainTask(void *pvParameters)
{	
	TimerHandle_t tim_wdt;

	// The parameters are not used.
	(void)pvParameters;

	LED_RGB(0,1,0);

	tim_wdt = xTimerCreate("RSTim", pdMS_TO_TICKS(4000), pdTRUE, NULL, feedwdt);
	configASSERT(tim_wdt);
	xTimerStart(tim_wdt, portMAX_DELAY);
	
#ifdef USB
	VCOM_Init();
	debug_frmwrk_init();
	VCOM_Connect(true);
	vTaskDelay(1000);
	//sddebugenable();
	lpc_printf(header1, CGU_GetPCLKFrequency(CGU_PERIPHERAL_M4CORE)/1000000UL);
#endif

	if (SDCard_Init() != SUCCESS) { // if it fails in here, it means reading NAND also fails, should die
		lpc_printf("ERROR: SD card init failed; resetting...\r\n");
		die(); 
	}

#ifndef GATEWAY	
	
	lpc_printf("TrigSenIdx is %u\r\n", TrigSenIdx);

	if (TrigSenIdx==0) {
		lpc_printf("- no ADXL362 trigger\n\r");
		adxl362_init_noINT();
	} else {
	if (!RTC_Reconfigure()) { // it is turned on by the physical switch or waken up by ADXL
		adxl362_init(FALSE);
		if(!adxl362_motiondetection()) {        // it is the time when switch is turned on, and it is first time to configure ADXL,
			lpc_printf("- ADXL362 no trigger\n\r");
			adxl362_setup();
		} else { // it is the time when switch is turen on, and it is first time to configure ADXL,
			lpc_printf("- ADXL362 trigger\n\r");
			adxl_inact_setup();
			Timesync_Init();
			RefSelect_Init();
			trig_active=true;
			TriggerSensing();
		}			
	} else { // it is waken up by the clock using the schedule
		lpc_printf("- RTC wake-up\n\r");
		LED_RGB(0,1,0);
		adxl362_setup();
	}
}
#else
	adxl362_init_noINT();
#endif

	if (Util_Init() != SUCCESS) {
		lpc_printf("ERROR: Util init failed; resetting...\r\n");
		die();
	}

	if (Trig_Init()!= SUCCESS) {
		lpc_printf("ERROR: Trig init failed; resetting...\r\n");
		die();
	}

	if (XnodeConfig_Init()!= SUCCESS) {
		lpc_printf("ERROR: XnodeCOnfig init failed; resetting...\r\n");
		die();
	}
	LED_RGB(0,0,0);
	
	app_main();
	
	vTaskDelete(NULL);
}

/*********************************************************************//**
 * @brief       Entry point 
 * @param[in]   None
 * @return      None
 **********************************************************************/
int c_entry(void) 
{
	LED_Init();
	LED_RGB(1, 0, 0);

	// should be as early as possible!
	NodeReset_Init();

	SystemInit();
	CGU_Init();

	SDRAM_Init();
	Timer_Init();

	// Ensure all priority bits are assigned as preemption priority bits.
	NVIC_SetPriorityGrouping(0);
	wdtInit();
	if (xTaskCreate(vAppMainTask, "AppMain", 5 * configMINIMAL_STACK_SIZE, NULL,
	(tskIDLE_PRIORITY + 1UL), (TaskHandle_t *)NULL) != pdPASS) {
		// no printing yet
		die();
	}

	// Start the tasks
	vTaskStartScheduler();

	// Should not reach here
	return 0;
}

/* With ARM and GHS toolsets, the entry point is main() - this will
   allow the linker to generate wrapper code to setup stacks, allocate
   heap area, and initialize and copy code and data segments. For GNU
   toolsets, the entry point is through __start() in the crt0_gnu.asm
   file, and that startup code will setup stacks and data */
int main(void)
{
	return c_entry();
}

void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	lpc_printf("ERROR: malloc failed\r\n");
	die();
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	(void) pcTaskName;
	(void)pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	lpc_printf("ERROR: stack overflow in task '%s'\r\n", pcTaskName);
	die();
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/

#ifdef  DEBUG
/*******************************************************************************
* @brief        Reports the name of the source file and the source line number
*               where the CHECK_PARAM error has occurred.
* @param[in]    file Pointer to the source file name
* @param[in]    line assert_param error line source number
* @return       None
*******************************************************************************/
void check_failed(uint8_t *file, uint32_t line)
{
	lpc_printf("ERROR: %s line %u: wrong parameter value\n\r", file, line);
	die();
}
#endif
