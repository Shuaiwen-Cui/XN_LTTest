#include <xnode.h>
#include <timers.h>
#include <lpc43xx_timer.h>
#include <ReliableComm.h>
#include <RemoteCommand.h>
#include <RemoteSensing.h>
#include <SnoozeAlarm.h>
#include "Utils.h"
#include <SnoozeAlarm.h>
#include "timesync.h"

static bool initialized = false, busy = false;
static uint16_t *uaddrs;
static uint8_t ulen, uidx, rcid = 0xff;
static TaskHandle_t xTaskToNotify = NULL;

#ifndef GATEWAY
extern bool app_inactive;
#endif

#ifdef FRA
float battvol[20],battcur[20];
#endif
static void utiltimer(TimerHandle_t pxTimer)
{
	lpc_printf("- node %03u: command timed out\r\n", uaddrs[uidx]);
	RemoteCommand_stop(rcid);
	if (xTaskToNotify) {
		xTaskNotifyGive(xTaskToNotify);
	}
}

static int voltfunc(void* arg, uint32_t len)
{
	float chg[2];
#ifndef GATEWAY
	app_inactive = false;
#endif
	CheckBatteryVoltage();
	chg[0] = voltage;
	chg[1] = current;
	RemoteCommand_done(RC_CMD_UTIL_VOLTAGE, SUCCESS, chg, 2 * sizeof (float));
	return SUCCESS;
}

static void voltsent(uint16_t *targets, uint8_t tcount)
{
	if (tcount == 0) {
		lpc_printf("- node %03u: offline\r\n", uaddrs[uidx]);
		if (xTaskToNotify) {
			xTaskNotifyGive(xTaskToNotify);
		}
	}
}

static void voltresp(int success)
{
	vTaskDelay(500);
	NodeReset();
}

static void voltexec(int success, void *retval, uint32_t len)
{
	float *chg = (float *)retval;
	if (len != (2 * sizeof (float)) || success != SUCCESS) {
		lpc_printf("- node %03u: offline\r\n", uaddrs[uidx]);
	} else {
		float pct = (chg[0] < 3.5f ? 3.5f : (chg[0] > 4.0f ? 4.0f : chg[0])) - 3.5f;
		pct = pct*200; // pct/0.5V*100
		lpc_printf("- node %03u: battery charge %3d%% (%.2fV), charging current %.2fmA\n\r", uaddrs[uidx], (int)pct, chg[0], (chg[1] < 120.f) ? 0.f : chg[1]);
#ifdef FRA
		battvol[uidx] = chg[0];
		battcur[uidx] = chg[1];
#endif		
	}
	if (xTaskToNotify) {
		xTaskNotifyGive(xTaskToNotify);
	}
}

static int resetfunc(void* arg, uint32_t len)
{
#ifndef GATEWAY
	app_inactive = false;
#endif
	RemoteCommand_done(RC_CMD_UTIL_RESET, SUCCESS, NULL, 0);
	return SUCCESS;
}

static void resetsent(uint16_t *targets, uint8_t tcount)
{
	if (tcount == 0) {
		lpc_printf("- all nodes unresponsive\r\n");
		if (xTaskToNotify) {
			xTaskNotifyGive(xTaskToNotify);
		}
	}
}

static void resetresp(int success)
{
	vTaskDelay(500);
	NodeReset();
}

static void resetexec(int success, void *retval, uint32_t len)
{
	if (xTaskToNotify) {
		xTaskNotifyGive(xTaskToNotify);
	}
}

int Util_Init(void)
{
	if (initialized) {
		return SUCCESS;
	}
	if (GenericComm_init() != SUCCESS
	|| ReliableComm_init() != SUCCESS
	|| RemoteCommand_init() != SUCCESS
	|| RemoteCommand_register(RC_CMD_UTIL_VOLTAGE, true, voltfunc, voltsent, voltresp, voltexec) != SUCCESS
	|| RemoteCommand_register(RC_CMD_UTIL_RESET, false, resetfunc, resetsent, resetresp, resetexec) != SUCCESS) {
		return FAIL;
	}
	initialized = true;
	return SUCCESS;
}

int Util_Wakeup(uint16_t *addrs, uint8_t *addrlen)
{
	uint8_t i;
	if (!initialized || busy || !addrs || !addrlen) {
		return FAIL;
	}

	lpc_printf("\r\n- waking up %u nodes on radio channel %u:", *addrlen, RADIO_CHANNEL);
	for (i = 0; i < *addrlen; ++i) {
		if ((i % 10) == 0) {
			lpc_printf("\r\n- %03u", addrs[i]);
		} else {
			lpc_printf(", %03u", addrs[i]);
		}
	}
	lpc_printf("\r\n- wait %u minute(s)...\r\n", SNOOZE_ALARM_PERIOD);
	SnoozeAlarm_Wake(addrs, *addrlen);
	lpc_printf("- woke up %u nodes.\r\n", *addrlen);

	return SUCCESS;
}

int Util_Wakeup_Retrieve(uint16_t *addrs, uint8_t *addrlen)
{
	if (!initialized || busy || !addrs || !addrlen) {
		return FAIL;
	}

	lpc_printf("\r\n- waking up %u nodes; wait %u seconds...\r\n", *addrlen, 20);
	SnoozeAlarm_Wake_Retrieve(addrs, *addrlen);
	lpc_printf("- woke up %u nodes.\r\n", *addrlen);

	return SUCCESS;
}

int Util_ReadVoltage(uint16_t *addrs, uint8_t addrlen)
{
	TimerHandle_t tim;
	if (!initialized || busy || !addrs || !addrlen) {
		return FAIL;
	}
	if (Util_Wakeup(rsnodes, &rsncnt) != SUCCESS) {
		lpc_printf("ERROR: failed to wake up nodes; resetting...\r\n");
		return FAIL;
	}
	busy = true;
	rcid = RC_CMD_UTIL_VOLTAGE;
	uaddrs = addrs;
	ulen = addrlen;
	uidx = 0;
	tim = xTimerCreate("RVTimer", pdMS_TO_TICKS(RC_INQWAIT + 500), pdFALSE, NULL, utiltimer);
	configASSERT(tim);
	xTaskToNotify = xTaskGetCurrentTaskHandle();
	lpc_printf("\r\n- checking status of %u sensor nodes...\r\n", addrlen);
	for (uidx = 0; uidx < ulen; ++uidx) {
		xTimerChangePeriod(tim, pdMS_TO_TICKS(RC_INQWAIT + 500), portMAX_DELAY);
		if (RemoteCommand_execute(RC_CMD_UTIL_VOLTAGE, uaddrs + uidx, 1, NULL, 0) != SUCCESS) {
			lpc_printf("- node %03u: offline\r\n", uaddrs[uidx]);
		} else {
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		}
		xTimerStop(tim, portMAX_DELAY);
	}
	lpc_printf("- finished status check.\r\n");
	xTimerDelete(tim, portMAX_DELAY);
	xTaskToNotify = NULL;
	vTaskDelay(500);
	rcid = 0xff;
	busy = false;
	return SUCCESS;
}

int Util_ResetNodes(uint16_t *addrs, uint8_t addrlen)
{
	TimerHandle_t tim;
	if (!initialized || busy || !addrs || !addrlen) {
		return FAIL;
	}
	busy = true;
	rcid = RC_CMD_UTIL_RESET;
	tim = xTimerCreate("RNTimer", pdMS_TO_TICKS(3000), pdFALSE, NULL, utiltimer);
	configASSERT(tim);
	lpc_printf("\r\n- resetting %u sensor nodes; wait %u seconds...\r\n", addrlen, 3);
	xTaskToNotify = xTaskGetCurrentTaskHandle();
	xTimerStart(tim, portMAX_DELAY);
	if (RemoteCommand_execute(RC_CMD_UTIL_RESET, addrs, addrlen, NULL, 0) == SUCCESS) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	}
	xTimerStop(tim, portMAX_DELAY);
	lpc_printf("- reset command has been sent to all nodes.\r\n");
	xTimerDelete(tim, portMAX_DELAY);
	xTaskToNotify = NULL;
	vTaskDelay(500);
	rcid = 0xff;
	busy = false;
	return SUCCESS;
}

#ifdef FRA  // Ask for current condition (voltage, current) and send text message
#include <4GFTP.h>
int FRAUtil_ReadVoltage(uint16_t *addrs, uint8_t addrlen); 

uint8_t ulen;

int FRAUtil_ReadVoltage(uint16_t *addrs, uint8_t addrlen)
{
  TimerHandle_t tim;
	if (!initialized || busy || !addrs || !addrlen) {
		return FAIL;
	}

	if (SyncClock_init() != SUCCESS) {
		lpc_printf("ERROR: RemoteSensing init failed; resetting...\r\n");
	} else if (SyncClock_start(false) != SUCCESS) {; // TUFIXDONE: same as format above (== success) looks better. Fix: Use status instead of int for return of SyncClock_start
		lpc_printf("ERROR: failed to read voltage\r\n");
	}	
	// keep trying even no node is detected (i.e. not checking # of nodes found)
	busy = true;
	rcid = RC_CMD_UTIL_VOLTAGE;
	uaddrs = addrs;
	ulen = addrlen;
	uidx = 0;
	tim = xTimerCreate("RVTimer", pdMS_TO_TICKS(2 * 2 * (RC_INQWAIT + 1000)), pdFALSE, NULL, utiltimer);
	configASSERT(tim);
	xTaskToNotify = xTaskGetCurrentTaskHandle();
	lpc_printf("\r\n- checking status of %u sensor node(s)...\r\n", addrlen);
	
	for (uidx = 0; uidx < ulen; ++uidx) {
		xTimerChangePeriod(tim, pdMS_TO_TICKS(2 * 2 * (RC_INQWAIT + 5000)), portMAX_DELAY);
		if (RemoteCommand_execute(RC_CMD_UTIL_VOLTAGE, uaddrs + uidx, 1, NULL, 0) != SUCCESS) {
			lpc_printf("- node %03u: offline\r\n", uaddrs[uidx]);
		} else {
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		}
		xTimerStop(tim, portMAX_DELAY);
	}
	lpc_printf("- finished status check.\r\n");
	Voltage_writedata_MQTT(battvol, battcur, uaddrs, uidx);
	xTimerDelete(tim, portMAX_DELAY);
	xTaskToNotify = NULL;
	vTaskDelay(500);
	rcid = 0xff;
	busy = false;
	return SUCCESS;
}
#endif
