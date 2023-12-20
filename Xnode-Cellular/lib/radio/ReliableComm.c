#include <xnode.h>
#include <string.h>
#include <timers.h>
#include "Radio.h"
#include "SendSMsg.h"
#include "SendLData.h"
#include "ReliableComm.h"
#include "lz4.h"
#include <inttypes.h>

#define _DEBUG_               0
#if _DEBUG_
#define PRINTF(...)           lpc_printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static TimerHandle_t tim;

static void rctimer(TimerHandle_t pxTimer)
{
	lpc_printf("ERROR: rctimer ReliableComm is hung; resetting\r\n");
	die();
}

typedef struct {
	rc_scallback_t scallback;
	rc_rcallback_t rcallback;
} app_info_t;
static app_info_t apps[RC_MAX_APPS];
static bool initialized = false;

typedef struct {
	uint8_t id;
	uint8_t addrlen;
	uint16_t addrs[MAX_NODES];
} __attribute((packed)) RC_SM_Msg;
static RC_SM_Msg smpkt;

typedef struct {
	uint16_t addrs[MAX_NODES];
	uint8_t addrlen;
	uint8_t id;
	uint16_t psrc;
	uint8_t plen;
	uint8_t ppayload[SMDDATASIZE];
	bool active;
} RC_SM_Info;
static RC_SM_Info sminfo;

static uint8_t *buf = NULL;
static SSMInfo ssmi;
static SLDInfo sldi;
static NodeInfo ni;
static uint8_t *dptr;

void SendSMsg_sendDone(SSMInfo *s, int success);
void SendSMsg_received(SSMInfo *s, int success);
void SendLData_sendDone(int success);
void SendLData_received(int success);
void SendLData_SLDIReq(void);
void smcallback(uint16_t src, void *payload, uint8_t len);

int ReliableComm_init(void)
{
	if (initialized) {
		return SUCCESS;
	}
	memset(apps, 0, sizeof (apps));
	dptr = NULL;
	buf = (uint8_t *)sdmalloc(MAX_REC_BUF_SIZE);
	configASSERT(buf);
	memset(&sminfo, 0, sizeof (RC_SM_Info));
	sminfo.active = false;
	if (GenericComm_init() != SUCCESS
	|| SendSMsg_init(SendSMsg_sendDone, SendSMsg_received) != SUCCESS
  || SendLData_init(SendLData_sendDone, SendLData_received, SendLData_SLDIReq) != SUCCESS
	) {
		PRINTF("ReliableComm: ERROR: failed to init components\r\n");
		return FAIL;
	}
	initialized = true;
	if (GenericComm_register(GC_APP_RC_SMSG_MCAST, smcallback) != SUCCESS) {
		return FAIL;
	}
	PRINTF("ReliableComm: init\r\n");
	return SUCCESS;
}
#ifdef FRA
bool resetdone; // TUFIX: hotfix: issue: MQTTTask created inside SyncClock_start, and once it is deleted (due to overtime), the 2nd round of RemoteCommand_execute happens right after the ReliableComm: Timer_start of TimerShort without waiting for it to fire, so most of the next steps fails. This is not an issue if the task finishes (no waiting inside) or there is no task at all.
#endif
int ReliableComm_reset(void)
{
	if (!initialized) {
		return FAIL;
	}
	dptr = NULL;
	sminfo.active = false;
	SendSMsg_reset();
	SendLData_reset();
	PRINTF("ReliableComm: reset\r\n");
#ifdef FRA	
	resetdone = true; // TUFIX: hotfix: issue: MQTTTask created inside SyncClock_start, and once it is deleted (due to overtime), the 2nd round of RemoteCommand_execute happens right after the ReliableComm: Timer_start of TimerShort without waiting for it to fire, so most of the next steps fails. This is not an issue if the task finishes (no waiting inside) or there is no task at all.
#endif
	return SUCCESS;
}

bool ReliableComm_isBusy(void)
{
	return initialized && (sminfo.active || SendSMsg_isBusy() || SendLData_isBusy());
}

int ReliableComm_register(uint8_t id, rc_scallback_t scallback, rc_rcallback_t rcallback)
{
	if (!initialized || !scallback || !rcallback || id >= RC_MAX_APPS) {
		PRINTF("ReliableComm: register app %u FAIL\r\n", id);
		return FAIL;
	}
	apps[id].scallback = scallback;
	apps[id].rcallback = rcallback;
	PRINTF("ReliableComm: register app %u\r\n", id);
	return SUCCESS;
}

int ReliableComm_unregister(uint8_t id)
{
	if (!initialized || id >= RC_MAX_APPS) {
		PRINTF("ReliableComm: unregister app %u FAIL\r\n", id);
		return FAIL;
	}
	apps[id].scallback = NULL;
	apps[id].rcallback = NULL;
	PRINTF("ReliableComm: unregister app %u\r\n", id);
	return SUCCESS;
}

int ReliableComm_send(uint8_t id, const uint16_t *addrs, uint8_t addrlen, uint8_t *data, uint32_t len)
{
	int res = FAIL;
#if 0 	
	uint32_t t0, t1, tdiff;
#endif
	if (!initialized || dptr != NULL || !addrs || !addrlen || addrlen > MAX_NODES || !data || len > MAX_REC_BUF_SIZE || SendSMsg_isBusy() || SendLData_isBusy()) {
		PRINTF("ReliableComm[%u]: send FAIL\r\n", id);
		return FAIL;
	}
	if (len <= SMDDATASIZE) {
		memset(&ssmi, 0, sizeof (SSMInfo));
		memset(&ni, 0, sizeof (NodeInfo));
		memcpy((uint16_t *)(ni.NodeID), addrs, addrlen * sizeof (uint16_t));
		ni.NodeNum = addrlen;
		ssmi.Nodes = &ni;
		memcpy(ssmi.Data, data, len);
		ssmi.Size = len;
		ssmi.id = id;
		if (addrlen > 1) {
			uint8_t i;
			if (sminfo.active) {
				PRINTF("ReliableComm[%u]: send FAIL\r\n", id);
				return FAIL;
			}
			sminfo.active = true;
			smpkt.id = id;
			smpkt.addrlen = addrlen;
			memcpy((void *)(smpkt.addrs), addrs, addrlen * sizeof (uint16_t));
			tim = xTimerCreate("RCTimer", pdMS_TO_TICKS(5*60000), pdFALSE, NULL, rctimer);
			configASSERT(tim);
			PRINTF("ReliableComm[%u]: send short broadcast\r\n", id);
			for (i = 0; i < 10; ++i) {
				if (GenericComm_bcast(GC_APP_RC_SMSG_MCAST, (uint8_t *)&smpkt, sizeof (RC_SM_Msg)) == SUCCESS) {
					res = SUCCESS;
				}
			}
			if (res != SUCCESS) {
				PRINTF("ReliableComm[%u]: send FAIL\r\n", id);
				sminfo.active = false;
				return FAIL;
			}
			for (i = 0; i < 20; ++i) {
				if (GenericComm_bcast(GC_APP_RC_SMSG_MCAST, data, len) == SUCCESS) {
					res = SUCCESS;
				}
			}
			xTimerStop(tim, portMAX_DELAY);
			xTimerDelete(tim, portMAX_DELAY);
			if (res != SUCCESS) {
				PRINTF("ReliableComm[%u]: send FAIL\r\n", id);
				sminfo.active = false;
				return FAIL;
			}
			configASSERT(apps[id].scallback);
			sminfo.active = false;
			PRINTF("ReliableComm: apps[id].scallback\r\n");
			apps[id].scallback((void *)smpkt.addrs, smpkt.addrlen, data, res);
			return SUCCESS;
			/*
			res = SendSMsg_bcast(&ssmi);
			*/
		} else {
			tim = xTimerCreate("RCTimer", pdMS_TO_TICKS(5*60000), pdFALSE, NULL, rctimer);
			configASSERT(tim);
			PRINTF("ReliableComm[%u]: send short unicast\r\n", id);
			res = SendSMsg_send(&ssmi);
		}
	} else {
		memset(&sldi, 0, sizeof (SLDInfo));
		memset(&ni, 0, sizeof (NodeInfo));
		memcpy((uint16_t *)(ni.NodeID), addrs, addrlen * 2);
		ni.NodeNum = addrlen;
		sldi.Nodes = &ni;
		sldi.Data = buf;
		sldi.id = id;
#if 0
		t0 = LPC_RITIMER->COUNTER;
		if (len < 1000 || (sldi.Length = LZ4_compress_default((const char *)data, (char *)sldi.Data, len, MAX_REC_BUF_SIZE)) == 0) {
			PRINTF("ReliableComm[%u]: no compression\r\n");
			memcpy(sldi.Data, data, len);
			sldi.Length = len; 
		} else {
			t1 = LPC_RITIMER->COUNTER;
			if (t0 > t1) {
				tdiff = 0xFFFFFFFFUL - t0 + t1;
			} else {
				tdiff = (t1 - t0) / 120000UL;
			}
			PRINTF("ReliableComm[%u]: compressed %u bytes to %u bytes in %ums\r\n", id, len, sldi.Length, tdiff);
		}
#else
		memcpy(sldi.Data, data, len);
		sldi.Length = len;
#endif
		tim = xTimerCreate("RCTimer", pdMS_TO_TICKS(5*60000), pdFALSE, NULL, rctimer);
		configASSERT(tim);
		if (addrlen > 1) {
			PRINTF("ReliableComm[%u]: send long broadcast\r\n", id);
			res = SendLData_bcast();
		}
		else {
			PRINTF("ReliableComm[%u]: send long unicast\r\n", id);
			res = SendLData_send();
		}
	}
	if (res == SUCCESS) {
		PRINTF("ReliableComm[%u]: DONE \r\n", id);
		dptr = data;
	} else {
		PRINTF("ReliableComm[%u]: send FAIL\r\n", id);
	}
	return res;
}

static void SendSMsg_sendDone(SSMInfo *s, int success)
{
	uint8_t *tmp = dptr;
	if (!initialized) {
		PRINTF("ReliableComm: SendSMsg_sendDone FAIL\r\n");
		return;
	}
	dptr = NULL;
	configASSERT(apps[s->id].scallback);
	xTimerStop(tim, portMAX_DELAY);
	xTimerDelete(tim, portMAX_DELAY);
	PRINTF("ReliableComm: SendSMsg_sendDone\r\n");
	apps[s->id].scallback((uint16_t *)(s->Nodes->NodeID), s->Nodes->NodeNum, tmp, success);
}

static void SendSMsg_received(SSMInfo *s, int success)
{
	if (!initialized) {
		PRINTF("ReliableComm: SendSMsg_received FAIL\r\n");
		return;
	}
	configASSERT(apps[s->id].rcallback);
	PRINTF("ReliableComm: SendSMsg_received\r\n");
	PRINTF("%d, %d, %d, %d, %d, %d, %d\r\n",s->id, s->DestAddr, (uint16_t *)(s->Nodes->NodeID), s->Nodes->NodeNum, s->Data, s->Size, success);

	apps[s->id].rcallback(s->DestAddr, (uint16_t *)(s->Nodes->NodeID), s->Nodes->NodeNum, s->Data, s->Size, success);
}

static void SendLData_sendDone(int success)
{
	uint8_t *tmp = dptr;
	if (!initialized) {
		PRINTF("ReliableComm: SendLData_sendDone FAIL\r\n");
		return;
	}
	dptr = NULL;
	configASSERT(apps[sldi.id].scallback);
	xTimerStop(tim, portMAX_DELAY);
	xTimerDelete(tim, portMAX_DELAY);
	PRINTF("ReliableComm: SendLData_sendDone\r\n");
	apps[sldi.id].scallback((uint16_t *)(sldi.Nodes->NodeID), sldi.Nodes->NodeNum, tmp, success);
}

static void SendLData_received(int success)
{
#if 0
	uint8_t *data;
	int32_t len = MAX_REC_BUF_SIZE;
	uint32_t t0, t1, tdiff;
	if (!initialized) {
		PRINTF("ReliableComm: SendLData_received FAIL\r\n");
		return;
	}
	if ((success == SUCCESS) && (data = (uint8_t *)sdmalloc(MAX_REC_BUF_SIZE)) != NULL) {
		t0 = LPC_RITIMER->COUNTER;
		if ((len = LZ4_decompress_safe((const char *)sldi.Data, (char *)data, sldi.Length, MAX_REC_BUF_SIZE)) < 0) {
			PRINTF("ReliableComm[%u]: no decompression\r\n", sldi.id);
		} else {
			t1 = LPC_RITIMER->COUNTER;
			if (t0 > t1) {
				tdiff = 0xFFFFFFFFUL - t0 + t1;
			} else {
				tdiff = (t1 - t0) / 120000UL;
			}
			PRINTF("ReliableComm[%u]: uncompressed %u bytes to %u bytes in %ums\r\n", sldi.id, sldi.Length, len, tdiff);
			memcpy(sldi.Data, data, len);
			sldi.Length = len;
		}
		sdfree(data);
	}
#else
	if (!initialized) {
		PRINTF("ReliableComm: SendLData_received FAIL\r\n");
		return;
	}
#endif
	configASSERT(apps[sldi.id].rcallback);
	PRINTF("ReliableComm: SendLData_received\r\n");
	apps[sldi.id].rcallback(sldi.DestAddr, (uint16_t *)(sldi.Nodes->NodeID), sldi.Nodes->NodeNum, sldi.Data, sldi.Length, success);
}

static void SendLData_SLDIReq(void)
{
	memset(&sldi, 0, sizeof (SLDInfo));
	memset(&ni, 0, sizeof (NodeInfo));
	sldi.Data = buf;
	sldi.Nodes = &ni;
	sldi.Length = MAX_REC_BUF_SIZE; 
	SendLData_SLDIAlloc(&sldi);
	PRINTF("ReliableComm: SendLData_SLDIReq\r\n");
}

void Timer_start(TimerHandle_t timer, uint32_t t)
{
	PRINTF("ReliableComm: Timer_start (%" PRIu64 " ms)\r\n", pdMS_TO_TICKS(t));
	if (xTimerChangePeriod(timer, pdMS_TO_TICKS(t), portMAX_DELAY) == pdPASS) {
		//PRINTF("ReliableComm: Timer_start (%u ms)\r\n", pdMS_TO_TICKS(t));
	}
}

void Timer_stop(TimerHandle_t timer)
{
	if (xTimerStop(timer, portMAX_DELAY) == pdPASS) {
		//PRINTF("ReliableComm: Timer_stop\r\n");
	}
}

void smcallback(uint16_t src, void *payload, uint8_t len)
{
	if (!initialized) {
		PRINTF("ReliableComm: smcallback FAIL\r\n");
		return;
	}
	// skip dupes
	if (sminfo.plen == len && sminfo.psrc == src && (memcmp(sminfo.ppayload, payload, len) == 0)) {
		PRINTF("ReliableComm: smcallback FAIL\r\n");
		return;
	}
	sminfo.plen = len;
	sminfo.psrc = src;
	memcpy(sminfo.ppayload, payload, len);
	if (!sminfo.active) {
		uint8_t i;
		RC_SM_Msg *m = (RC_SM_Msg *)payload;
		if (len != sizeof (RC_SM_Msg)) {
			PRINTF("ReliableComm: smcallback FAIL\r\n");
			return;
		}
		for (i = 0; i < m->addrlen; ++i) {
			if (m->addrs[i] == LOCAL_ADDR) {
				break;
			}
		}
		if (i == m->addrlen) {
			PRINTF("ReliableComm: smcallback FAIL\r\n");
			return;
		}
		sminfo.active = true;
		sminfo.id = m->id;
		sminfo.addrlen = m->addrlen;
		memcpy(sminfo.addrs, (void *)m->addrs, m->addrlen * sizeof (uint16_t));
	} else {
		if (sminfo.psrc != src) {
			PRINTF("ReliableComm: smcallback FAIL\r\n");
			return;
		}
		configASSERT(apps[sminfo.id].rcallback);
		sminfo.active = false;
		PRINTF("ReliableComm: SendSMsg_received\r\n");
		apps[sminfo.id].rcallback(src, sminfo.addrs, sminfo.addrlen, payload, len, SUCCESS);
	}
}
