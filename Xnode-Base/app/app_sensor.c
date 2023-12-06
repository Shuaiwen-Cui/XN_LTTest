#ifndef GATEWAY
#include <xnode.h>
#include <timers.h>
#include <RemoteSensing.h>
#include <RetrieveData.h>
#include <SnoozeAlarm.h>

bool app_inactive = true;
extern bool trig_active;

static void appinacttimer(TimerHandle_t pxTimer)
{
	if (app_inactive) {
		lpc_printf("- node inactive for too long; resetting...\r\n");
		NodeReset();
	} else {
		xTimerDelete(pxTimer, portMAX_DELAY);
	}
}

int app_sensor(void)
{
	TimerHandle_t tim;

	LED_RGB(0,1,0);
	
	if(!trig_active)
		SnoozeAlarm_Start();

	if (RemoteSensing_init() != SUCCESS) {
		lpc_printf("ERROR: RemoteSensing init failed; resetting...\r\n");
		die();
	}

	if (RetrieveData_init() != SUCCESS) {
		lpc_printf("ERROR: DataRetrieving init failed; resetting...\r\n");
		die();
	}

	lpc_printf("- sensor node initialized.\r\n");

	// 6-min inactivity timer
	tim = xTimerCreate("RSTimer", pdMS_TO_TICKS(6*60000UL), pdFALSE, NULL, appinacttimer);
	configASSERT(tim);
	xTimerStart(tim, portMAX_DELAY);

	return 0;
}
#endif
