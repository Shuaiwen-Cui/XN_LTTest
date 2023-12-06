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
#include <rtc.h>
#include <4GFTP.h>

// TUFIX: Add the sense + save
// TUFIX: USE LZ4
// TUFIXDONE: Parametrize phone #, FTP site, ... - TUADDED: phone #, ftp site, ftp usernames and password
// TUFIX: Allow to work without network
// TUFIX: Need NAND to SD dumping func

#ifdef USB
static const char header1[] =
  "\r\n\r\n\r\n"
  "-------------------------------------------------------------------------------\r\n"
  "- Embedor Technologies Xnode Smart Sensor Ver. 2.3\r\n"
  "- * MCU: LPC4357\r\n"
  "- * Core: ARM Cortex-M4F\r\n"
  "- * OS: FreeRTOS 9.0.0\r\n"
  "- * Sensor: 24-bit acceleration\r\n"
  "- * Core freq: %u MHz\r\n"
  "-------------------------------------------------------------------------------\r\n"
  "\r\n";
#endif

void TIM_SemInit(void);
int SDCard_Init(void);
uint8_t TrigSenIdx = 0;

#ifdef FRA
#include <nandflash_k9f1g08u0a.h>
bool issendonly = false; // false: nothing to be sent, or woken up by adxl, true: something to be sent so only send, no sensing
bool SwitchOn = false;
extern char ch, FRAch;	// To automatically choose which task to follow based on time (leverage existing Xnode code of gateway)
bool timetodeploy = false; // fasle: not time to auto-dpeloy yet; true: time to auto-dpeloy yet
bool iswaitdeploy = false; // fasle: not in deploy time frame; true: in the deploy time frame, increased frequency
extern uint8_t task1T, task2T, task3T;
#endif 

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

#ifdef FRA
static TimerHandle_t tim_total = NULL;
static void rstimer_tu(TimerHandle_t pxTimer)
{
	lpc_printf("rstimer_tu _ reset\n\r");
	NodeReset();	
}
#endif

static void feedwdt(TimerHandle_t pxTimer)
{
	WWDT_Feed();
}

static void vAppMainTask(void *pvParameters)
{	
	TimerHandle_t tim_wdt;
#ifdef FRA
	Clock_info *CurrentTime=sdcalloc(1,sizeof(Clock_info));  // FRA purpose: Use current time of gateway to determine which task to do
	configASSERT(CurrentTime);
#endif
	// The parameters are not used.
	(void)pvParameters;

	LED_RGB(0,1,0);
	
#ifdef FRA
	tim_total = xTimerCreate("RSTim", pdMS_TO_TICKS(8*60000), pdFALSE, NULL, rstimer_tu);
	configASSERT(tim_total);
	xTimerStart(tim_total, portMAX_DELAY);
#endif
	
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
	if (SDCard_Init() != SUCCESS) { // if it fails in here, it means reading NAND also fails (return value is of NAND)
		lpc_printf("ERROR: SD card init failed; resetting...\r\n");
		die(); 
	}
	 
#if 0 // 0 ; 1 reset 
	if (SDCard_Reset() != SUCCESS) {
		lpc_printf("ERROR: SD card reset failed; resetting...\r\n");
		die();
	}
	lpc_printf("SD card reset\r\n");
	for(;;) {}
#endif

#ifndef GATEWAY	
#ifdef FRA
	TrigSenIdx = 1; // For FRA project purpose, Sensor node is always in TriSendIdx = 1 mode - always waiting for triggering signal
#endif // TUFIXDONE: Need to rework this area, as waking up is late right now: Fix: move all the important stuffs (related to trigger) to be the earliest
	if (TrigSenIdx)
	{
	if (!RTC_Reconfigure()) { // it is turned on by the physical switch or woken up by ADXL
		adxl362_init(FALSE);
		if(adxl362_motiondetection()) {        // it is the time when switch is turned on, and it is first time to configure ADXL,
			lpc_printf("- ADXL362 trigger\n\r"); 
			adxl_inact_setup();
			TriggerSensing();
		} else 
		{
#ifdef FRA
			issendonly = read_tosend();
			RTC_clearflag();
			RTC_Read(CurrentTime);
			if (islongsleep(CurrentTime))  // last switch on was more than 10 seconds ago - Synchronize
			{
				SwitchOn = true;
				lpc_printf("- Switch on");
			}
#endif
			adxl362_setup();	// these 3 lines (adxl362_setup(); -  adxl362_int_init();  - adxl362_int_enable(); goes next to each other to make sure int2 is enabled for activity detection
			adxl362_int_init(); // init and enable activity int on INT2 
			adxl362_int_enable();		
		}
	} else { // it is woken up by the clock using the schedule
		issendonly = read_tosend();
		lpc_printf("- RTC wake-up\n\r");
		LED_RGB(0,1,0);
		adxl362_setup();	// these 3 lines (adxl362_setup(); -  adxl362_int_init();  - adxl362_int_enable(); goes next to each other to make sure int2 is enabled for activity detection
		adxl362_int_init(); // init and enable activity int on INT2 
		adxl362_int_enable();		
		if(!issendonly) // nothing to send
		{
			lpc_printf("\n\r");
		} else 
		{
			lpc_printf(" to send data back\n\r");
		}
	}
	} else {
		lpc_printf("- no ADXL362 trigger\n\r");
#ifndef FRA		
		adxl362_init_noINT();
#endif
	}
#else
#ifndef FRA		
	adxl362_init_noINT();
#endif

#ifdef FRA // TUFIX: Should put this to a function so it looks cleaner and easier to keep track
	// FRA: Turn off 4G (if it's on) --> Moved to NodeReset_init
	// FRA: Schedule
	// GW_SENSE: Let GW run TriggerSensing
#ifdef GW_SENSE // New tag: Gateway node also runs the task of sensing. By the design of sensing function in sensing.c, triggerSensing goes directly to sleeping mode, so the code won't come back after TriggerSensing
	TrigSenIdx = 1; // For FRA project purpose, Sensor node is always in TriSendIdx = 1 mode - always waiting for triggering signal. // GW_SENSE: INT2 is not setup for trigger during sensing so no need to care about locking the interrupt during saving function
	adxl362_init(FALSE); 
	if (adxl362_motiondetection())  // True = turned on by ADXL
	{        
		lpc_printf("- ADXL362 trigger\n\r"); 
		LED_RGB(1,0,1);
		adxl_inact_setup();
		TriggerSensing();
	} 
#endif
	// Check clock to determine if switch is flipped or not
	if (!RTC_Reconfigure())
	{
		lpc_printf("SYNCH APP\n\r");
		ch = '8';
		FRAch = '2'; // 1st time switch on - Synchronize
		LED_RGB(1,0,0); 
	} 
	else
	{
		// Set clocks in 15 minutes, so can flip switch again for synch app. THis is to prevent the case that the engineer flips switch on at say 12:29, alarm fires at 12:30. So when the 
		// engineer flips off at 12:29:59, then flip back on at 12:30:5, the alarm flag is on again - Use SnoozeAlarm_Sleep_noSleep(15);
		// However, it won't be much better than telling the engineer to flip on, see REG  = good, see BLUE = switch off and on until see RED - TUFIXDONE: ADD the readwaketime and compare
		RTC_clearflag();
		RTC_Read(CurrentTime);
		if (islongsleep(CurrentTime)) 
		{
			lpc_printf("SYNCH APP\n\r");
			ch = '8';
			FRAch = '2'; // last switch on was more than 10 seconds ago - Synchronize
			LED_RGB(1,0,0); 
		}
		else
		{
			LED_RGB(0,0,1); 
			/*
			// This is important, but not as important as the 1st time switch on, so adxl is allowed to trigger
			adxl362_setup(); // setup and get ready for being triggered by INT2
			adxl362_int_init(); // init and enable activity int on INT2
			adxl362_int_enable(); 
			*/// TUFIX: Disable this for now due to unstable when doing TCP connect, it triggers
			ch = '8';
			if (CurrentTime->minute == 0) // Check status
			{
				if ((CurrentTime->hour)%task2T == 0)
				{
					FRAch = '1'; // check and report volt
				}
			} 
			else if ((CurrentTime->minute == (60-task1T)) && (((CurrentTime->hour+1)%24)%task3T == 0))	// synch network
			{
				FRAch = '2'; // Synchronize
			}
			else if ((CurrentTime->minute == (60-4*task1T)) && (((CurrentTime->hour+1)%24)%task3T == 0))	// download new config
			{
				FRAch = '4';
			}
			else// woken up to upload data
			{
				FRAch = '3'; // so that during the hours that are not scheudled for anythingm gateway still asks for data
			}
		}
	}
#endif		
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
	
	if (xTaskCreate(vAppMainTask, "AppMain", 50 * configMINIMAL_STACK_SIZE, NULL,
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
	//lpc_printf("ERROR: malloc failed\r\n");
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
	//lpc_printf("ERROR: stack overflow in task '%s'\r\n", pcTaskName);
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
	//lpc_printf("ERROR: %s line %u: wrong parameter value\n\r", file, line);
	die();
}
#endif
