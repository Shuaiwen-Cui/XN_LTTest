#include <xnode.h>
#include <string.h>
#include <queue.h>
#include "GenericComm.h"
#include "SendLData.h"
#include <Radio.h>
#ifdef FRA // for the timer issue
#include "SendSMsg.h"
#endif
#define _DEBUG_               0
#if _DEBUG_
#define PRINTF(...)           lpc_printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static GC_Msg msg, msg2;
static uint8_t flagCom, flagRF, sldState, rOrder, numIDPackets, msgIDLocal, inqRepeat, reqIndSeq, reqIndSize;
static uint16_t comment2Count, comment2CountRec;
static uint32_t nodeIndex, *reqIndex;
static uint8_t *recChk, *ackedNode, *replyNode, resendInd[2];
static NoticeMsg *finalAckMsg = (NoticeMsg *)msg2.data;
static NoticeMsg *sldMsg = (NoticeMsg *)msg.data;
static DataMsg *dataMsg = (DataMsg *)msg.data;
static RecentMsg rm;
static SLDInfo *sldi;
static bool initialized = false;
static ld_callback_t scallback, rcallback;
static ld_acallback_t acallback;
static QueueHandle_t ldQueueAck, ldQueueNotice, ldQueueData;
static TimerHandle_t timerNotice, timerTimeout, timerWait1, timerWait2;

void sldiReorder(SLDInfo *s);
void sendNoticeRandTask(void);
void sendAckRandTask(void);
void sendNoticeSubTask(void);
void sendDataSubTask(void);
void sendDataTask(void);
void beginTask(void);
void nodeInqTask(void);
void reqMissingDataTask(void);
void sendAckMsgTask(void);
void sendNoticeEndTask(void);
void sendNoticeSubTask2(void);
void comment3CheckTask(void);
void comment3CheckUnicastTask(void);
void sendDoneTask(void);
void receivedTask(void);
void informAckNodeTask(void);	
void timeoutResetTask(void);
void TimerNotice_fired(TimerHandle_t pxTimer);
void TimerTimeout_fired(TimerHandle_t pxTimer);
void TimerWait1_fired(TimerHandle_t pxTimer);
void TimerWait2_fired(TimerHandle_t pxTimer);
void ldcallback_ack(uint16_t src, void *payload, uint8_t len);
void ldcallback_notice(uint16_t src, void *payload, uint8_t len);
void ldcallback_data(uint16_t src, void *payload, uint8_t len);
void handleAck(GC_Msg *m);
void handleNotice(GC_Msg *m);
void handleData(GC_Msg *m);
void ldRecvTask(void *pvParameters);

static int staskres = FAIL, rtaskres = FAIL;

static void ldSTask(void *pvParameters)
{
	portYIELD();
	scallback(staskres);
	vTaskDelete(NULL);
}

static void startSTask(int res)
{
	PRINTF("ReliableComm: SendLData startSTask\r\n");
	staskres = res;
	configASSERT(xTaskCreate(ldSTask, "LDSTask", configMINIMAL_STACK_SIZE, NULL,
    (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL) == pdPASS);
}

static void ldRTask(void *pvParameters)
{
	portYIELD();
	rcallback(rtaskres);
	vTaskDelete(NULL);
}

static void startRTask(int res)
{
	PRINTF("ReliableComm: SendLData startRTask\r\n");
	rtaskres = res;
	configASSERT(xTaskCreate(ldRTask, "LDRTask", 2 * configMINIMAL_STACK_SIZE, NULL,
	  (tskIDLE_PRIORITY + 2UL), (TaskHandle_t *) NULL) == pdPASS);
}

static void ldTTask(void *pvParameters)
{
	portYIELD();
	sendDataTask();
	vTaskDelete(NULL);
}

static void startTTask(void)
{
	PRINTF("ReliableComm: SendLData startTTask\r\n");
  configASSERT(xTaskCreate(ldTTask, "LDTTask", configMINIMAL_STACK_SIZE, NULL,
	(tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL) == pdPASS);
}

int SendLData_init(ld_callback_t scb, ld_callback_t rcb, ld_acallback_t acb)
{
	if (initialized) {
		return SUCCESS;
	}
	if (GenericComm_init() != SUCCESS
	|| GenericComm_register(GC_APP_RC_LDATA_ACK, ldcallback_ack) != SUCCESS
	|| GenericComm_register(GC_APP_RC_LDATA_NOTICE, ldcallback_notice) != SUCCESS
	|| GenericComm_register(GC_APP_RC_LDATA_DATA, ldcallback_data) != SUCCESS) {
		return FAIL;
	}
	sldState = 0;
	flagCom = flagRF = inqRepeat = 0;
	scallback = scb;
	rcallback = rcb;
	acallback = acb;
	recChk = (uint8_t *)sdmalloc(RECCHKSIZE);
	reqIndex = (uint32_t *)sdmalloc(REQINDEXSIZE * sizeof (uint32_t));
	ackedNode = (uint8_t *)sdmalloc(ACKEDNODENUM);
	replyNode = (uint8_t *)sdmalloc(ACKEDNODENUM);
	configASSERT(recChk && reqIndex && ackedNode && replyNode);
	ldQueueAck = xQueueCreate(1, sizeof (GC_Msg *));
	ldQueueNotice = xQueueCreate(1, sizeof (GC_Msg *));
	ldQueueData = xQueueCreate(1, sizeof (GC_Msg *));
	timerNotice = xTimerCreate("LDNotice", 1000, pdFALSE, NULL, TimerNotice_fired);
	timerTimeout = xTimerCreate("LDTimeout", 1000, pdFALSE, NULL, TimerTimeout_fired);
	timerWait1 = xTimerCreate("LDWait1", 1000, pdFALSE, NULL, TimerWait1_fired);
	timerWait2 = xTimerCreate("LDWait2", 1000, pdFALSE, NULL, TimerWait2_fired);
	configASSERT(ldQueueAck && ldQueueNotice && ldQueueData && timerNotice && timerTimeout && timerWait1 && timerWait2);
	configASSERT(xTaskCreate(ldRecvTask, "LDRecvTask", 2 * configMINIMAL_STACK_SIZE, NULL,
	(tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL) == pdPASS);
	acallback();
	initialized = true;
	PRINTF("ReliableComm: SendLData_init\r\n");
	return SUCCESS;
}

int SendLData_reset(void)
{
	if (!initialized) {
		return FAIL;
	}
	radio_reset();
	flagCom = flagRF = 0;
	sldState = 0x0;
	sldMsg->Comment = 0xe;
	sldi->MsgID = 255;
	sldi->DestAddr = BCAST_ADDR;
	sldMsg->DestAddr = sldi->DestAddr;
	sldMsg->OrigAddr = LOCAL_ADDR;
	Timer_stop(timerNotice);
	Timer_stop(timerTimeout);
	Timer_stop(timerWait1);
	Timer_stop(timerWait2);
	sendNoticeSubTask();
	PRINTF("ReliableComm: SendLData_reset\r\n");
	return SUCCESS;
}

bool SendLData_isBusy(void)
{
		return (flagCom & BSYLD) !=0 || (flagRF & MSGLK) != 0;
}

int SendLData_SLDIAlloc(SLDInfo *s)
{
	sldi = s;
	PRINTF("ReliableComm: SendLData_SLDIAlloc\r\n");
	return SUCCESS;
}

int SendLData_send(void)
{
	if (!initialized || SendLData_isBusy()) {
		return FAIL;
	}
	flagCom = BSYLD | SNDER;
	flagRF |= MSGLK;
	sldState = 0x1;
	sldMsg->DestAddr = sldi->DestAddr = sldi->Nodes->NodeID[0];
	sldMsg->OrigAddr = LOCAL_ADDR;
	sldMsg->DataLength = sldi->Length;
	msgIDLocal = (msgIDLocal + 1) % 255;
	sldMsg->MsgID = sldi->MsgID = msgIDLocal;
	sldMsg->ReqInd[1] = (uint16_t)sldi->id;
	sldMsg->Comment = 0x1; // 0x1 Begin;
	memset(reqIndex, 0, REQINDEXSIZE * sizeof (uint16_t));
	memset(&rm, 0, sizeof (RecentMsg));
	PRINTF("ReliableComm: SendLData_send\r\n");
	Timer_start(timerTimeout, RC_INQWAIT);
	sendNoticeSubTask();
	return SUCCESS;
}

int SendLData_bcast(void)
{
	if (!initialized || sldi->Nodes->NodeNum == 0 || SendLData_isBusy()) {
		return FAIL;
	}
	flagCom = BSYLD | SNDER | BCAST;
	sldState = 0x21;
	nodeIndex = 0;
	sldi->DestAddr = sldMsg->DestAddr = BCAST_ADDR;
	sldMsg->OrigAddr = LOCAL_ADDR;
	msgIDLocal= (msgIDLocal + 1) % 255;
	sldMsg->MsgID = sldi->MsgID = msgIDLocal;
	sldMsg->Comment = 0xf; // 0xf Inquiry;
	sldMsg->Others = (uint8_t)(sldi->id);
	memset(sldi->Nodes->AckedNode, 0, ACKEDNODENUM);
	memset(ackedNode, 0, ACKEDNODENUM);
	memset(reqIndex, 0, REQINDEXSIZE * sizeof (uint32_t));
	memset(resendInd, 0, 2);
	memset(replyNode, 0, ACKEDNODENUM);
	memset(&rm, 0, sizeof (RecentMsg));
	PRINTF("ReliableComm: SendLData_bcast\r\n");
	Timer_start(timerTimeout, RC_BCINQWAIT);
	nodeInqTask();
	return SUCCESS;
}

static void TimerNotice_fired(TimerHandle_t pxTimer)
{
	if (!initialized) {
		return;
	}
	/* 
	Sender		1: Begin 2: End, 3: Done
	Receiver	1: Request Missing Packets 2: Done
	*/
	PRINTF("ReliableComm: SendLData TimerNotice_fired. state: %x Comment: %u\r\n", sldState, sldMsg->Comment);
	if (flagCom & BCAST) {
		if (flagCom & SNDER) { // Sender (Bcast)
			PRINTF("ReliableComm: SendLData TimerNotice_fired bcast sender\r\n");
			if (sldState == 0x21) {
				inqRepeat++;
				nodeInqTask();
			} else if (sldState == 0x23) {
				if ((flagCom & ACKED) && sldMsg->Comment == 1) {
					PRINTF("ReliableComm: SendLData Bcast Sender ACKED post sendDataTask();\r\n");
					sldState = 0x24;
					nodeIndex = 0;
					memset(ackedNode, 0, ACKEDNODENUM);
					reqIndSeq = 0;
					flagRF &= ~MSGLK;
					inqRepeat = 0;
					startTTask();
					//sendDataTask();
				} else if ((flagCom & TMOUT) && sldMsg->Comment == 1) {
					PRINTF("ReliableComm: SendLData TMOUT in TimerNotice_fired, 0x23\r\n");
					timeoutResetTask();
				} else if (sldMsg->Comment == 1) {
					PRINTF("ReliableComm: SendLData TimerNotice_fired Comment == 1, 0x23\r\n");
					flagRF &= ~MSGLK;
					informAckNodeTask();
				}
			} else if (sldState == 0x25) {
				if (sldMsg->Comment == 2) {
					if (!(flagCom & ACKED)) {
						PRINTF("ReliableComm: SendLData post informAckNodeTask 0x25\r\n");
						flagRF &= ~MSGLK;
						informAckNodeTask();
					} else {
						PRINTF("ReliableComm: SendLData post comment3CheckTask\r\n");
						sldState = 0x26;
						flagRF &= ~MSGLK;
						inqRepeat = 0;
						comment3CheckTask();
					}
				}
			} else if (sldState == 0x27 && sldMsg->Comment == 4) {
				if ((flagCom & ACKED) != ACKED) {
					flagRF &= ~MSGLK;
					inqRepeat++;
					informAckNodeTask();
				} else {
					Timer_stop(timerTimeout);
					sldiReorder(sldi);
					inqRepeat = 0;
					sendDoneTask();
				}
			}
		} else { // Receiver (Bcast)
			PRINTF("ReliableComm: SendLData TimerNotice_fired bcast receiver\r\n");
			if (sldState == 0x26) { // no need?
				reqMissingDataTask();
			}
		}
	} else if (flagCom & SNDER) { // Sender (Unicast)
		PRINTF("ReliableComm: SendLData TimerNotice_fired unicast sender\r\n");
		if (sldState == 0x1) {
			if (flagCom & TMOUT) {
				timeoutResetTask();
				return; 
			} else if ((flagCom & ACKED)!= ACKED) {
				sendNoticeSubTask();
			} else {
				Timer_stop(timerTimeout);
			}
		} else if (sldState == 0x3) {
			if (flagCom & TMOUT) {
				timeoutResetTask();
				return;
			}
			if ((flagCom & ACKED) != ACKED) {
				sendNoticeSubTask();
			}
		} else if (sldState == 0x5) {
			if (flagCom & TMOUT) {
				timeoutResetTask();
				return;
			}
			if ((flagCom & ACKED) != ACKED) {
				sendNoticeSubTask();
			} else {
				Timer_stop(timerTimeout);
				sendDoneTask();
			}
		}
	}
}

static void TimerTimeout_fired(TimerHandle_t pxTimer)
{
	if (!initialized) {
		return;
	}
	PRINTF("ReliableComm: SendLData TimerTimeout state: %x BCAST %u SNDER %u\r\n", sldState, flagCom & BCAST, flagCom & SNDER);
	if (!(flagCom & BCAST)) {
		if (flagCom & SNDER) {
			if (sldState == 0x1 || sldState == 0x2 || sldState == 0x3 || sldState == 0x4 || sldState == 0x5) {
				flagCom |= TMOUT;
				return;
			}
		} else {
			flagCom |= TMOUT;
			timeoutResetTask();
			return;
		}
		flagCom &= ~BSYLD;
		sldState = 0x0;
		//scallback(FAIL);
		startSTask(FAIL);
	} else if (flagCom & SNDER) {
		if (sldState == 0x21) {
			PRINTF("ReliableComm: SendLData TimerTimeout 0x21\r\n");
			flagCom |= TMOUT;
		} else if (sldState == 0x23 || sldState == 0x24 || sldState == 0x25 || sldState == 0x26) {
			timeoutResetTask();
		}
	} else {
		timeoutResetTask();
	}
}

static void TimerWait1_fired(TimerHandle_t pxTimer)
{
	PRINTF("ReliableComm: SendLData TimerWait1_fired\r\n");
	sendNoticeSubTask();
}

static void TimerWait2_fired(TimerHandle_t pxTimer)
{
	PRINTF("ReliableComm: SendLData TimerWait2_fired\r\n");
	sendNoticeSubTask2();
}

static void timeoutResetTask(void)
{
	PRINTF("ReliableComm: SendLData timeoutResetTask\r\n");
	Timer_stop(timerNotice);
	Timer_stop(timerTimeout);
	Timer_stop(timerWait1);
	Timer_stop(timerWait2);
	sldi->MsgID = 255;
	if (flagCom & SNDER) {
		//scallback(FAIL);
		startSTask(FAIL);
		flagCom = flagRF = inqRepeat = 0;
		sldi->DestAddr = LOCAL_ADDR;
		sldState = 0;
	} else {
		flagCom = flagRF = inqRepeat = 0;
		sldi->DestAddr = LOCAL_ADDR;
		sldState = 0;
	}
}

static void sldiReorder(SLDInfo *s)
{
	NodeInfo Node;
	uint8_t i, j = 0;
	for (i = 0; i < s->Nodes->NodeNum; i++) {
		if ((s->Nodes->AckedNode[i / 8] & (1 << (i % 8))) != 0) {
			Node.NodeID[j++] = s->Nodes->NodeID[i];
		}
	}
	Node.NodeNum = j;
	*(s->Nodes) = Node;
}

static void ldcallback_ack(uint16_t src, void *payload, uint8_t len)
{
	GC_Msg *pkt;
	if (!initialized) {
		return;
	}
	pkt = sdmalloc(sizeof (GC_Msg));
	if (pkt == NULL) {
		PRINTF("ERROR: sdmalloc failed in ldcallback_ack\r\n");
		die();
	}
	PRINTF("ReliableComm: SendLData ldcallback_ack\r\n");
	memcpy(pkt->data, payload, len);
	if (xQueueSend(ldQueueAck, &pkt, 0) != pdTRUE) {
		PRINTF("xQS ack FAIL\r\n");
		sdfree(pkt);
	}
}

static void handleAck(GC_Msg *m)
{
	uint8_t i;
	NoticeMsg *message = (NoticeMsg *)m->data;
	if (!initialized) {
		return;
	}
	if (message->DestAddr == LOCAL_ADDR) {
		// Receiver Unicast
		return;
	} else if (message->DestAddr == BCAST_ADDR && message->OrigAddr != LOCAL_ADDR) {
		// Receiver broadcast
		return;
	} else if (message->OrigAddr != LOCAL_ADDR || sldi->MsgID != message->MsgID || !(flagCom & BSYLD)) {
		return;
	}
	// Sender
	if (flagCom & BCAST) { // Broadcast Sender
		PRINTF("ReliableComm: SendLData handleAck broadcast sender\r\n");
		if (message->Comment == 0xf) {
			// Broadcast 
		} else if (message->Comment == 0x4) { // SendDone Broadcast Sender
			if (sldState == 0x27) { // ACKED signal for DONE notice.
				for (i = 0; i <= sldi->Nodes->NodeNum - 1; i++) {
					if (message->ReqInd[0] == sldi->Nodes->NodeID[i]) {
						ackedNode[i / 8] |= 1 << (i % 8); // 8 because of uint8_t 
						replyNode[i / 8] |= 1 << (i % 8);
					}
				}
				for (i = 0; i<= (sldi->Nodes->NodeNum - 1) / 8; i++) {
					if (ackedNode[i] != sldi->Nodes->AckedNode[i]) {
						break;
					}
					if (i == (sldi->Nodes->NodeNum - 1) / 8) {
						flagCom |= ACKED; // Need to modify in case some nodes die during SendDataMsg
					}
				}
			}
		}
	} else { // Unicast Sender
		PRINTF("ReliableComm: SendLData handleAck unicast sender\r\n");
		if (message->Comment == 1) { // Start sending data
			Timer_stop(timerTimeout);
			flagCom |= ACKED; // Acknowledged
		} else if (message->Comment == 3) { // Replying to ReqMissing packets
			flagCom |= ACKED;
			reqIndSize = message->DataLength;
			for (i = 0; i <= reqIndSize - 1; i++) {
				reqIndex[i] = message->ReqInd[i];
			}
		} else if (message->Comment == 4) { // Notify Done
			flagCom |= ACKED;
		}
	}
}

static void ldcallback_notice(uint16_t src, void *payload, uint8_t len)
{
	GC_Msg *pkt;
	if (!initialized) {
		return;
	}
	pkt = sdmalloc(sizeof (GC_Msg));
	if (pkt == NULL) {
		PRINTF("ERROR: sdmalloc failed in ldcallback_notice\r\n");
		die();
	}
	PRINTF("ReliableComm: SendLData ldcallback_notice\r\n");
	memcpy(pkt->data, payload, len);
	if (xQueueSend(ldQueueNotice, &pkt, 0) != pdTRUE) {
		PRINTF("xQS notice FAIL\r\n");
		sdfree(pkt);
	}
}
#ifdef FRA // TUFIX: very quick fix: GW sends out RC[RC_CMD_DR_RETRIEVE] and sometimes (~3%) if the reply message to confirm  is not sent back to the GW (for some reason), the GW moves on to receing the data with the TimerShort_fired regularly, and something crashes the node
extern uint8_t rcid_copy;
#endif
static void handleNotice(GC_Msg *m)
{
	uint8_t i, j, k, l, n;
	int16_t i16;
	uint16_t *p16;
	NoticeMsg *message = (NoticeMsg *)m->data;
	if (!initialized) {
		return;
	}
	PRINTF("ReliableComm: SendLData handleNotice Orig %u Dest %u Comment %u MsgID %u ReqInd[0] %u ReqInd[1] %u\r\n", message->OrigAddr, message->DestAddr, message->Comment, message->MsgID, message->ReqInd[0], message->ReqInd[1]);
	if (message->DestAddr == LOCAL_ADDR) { // Receiver Unicast
		PRINTF("ReliableComm: SendLData handleNotice receiver unicast\r\n");
		if (message->Comment == 1) { // Beginning of Data
			if ((flagCom & BSYLD) == 0) { // first time to receive "Begin"
				if ((flagRF & MSGLK) != MSGLK) {
					flagRF |= MSGLK;
					*sldMsg = *message;
					flagCom = BSYLD;
					sldState = 0x1;
					sldi->Length = message->DataLength;
					sldi->DestAddr = message->OrigAddr;
					sldi->MsgID = message->MsgID;
					sldi->id = (uint8_t)message->ReqInd[1];
					memset(recChk, 0, RECCHKSIZE);
					Timer_start(timerTimeout, RC_INQWAIT * 2 + RC_DWAIT * (sldi->Length / 1000 + 1) + RC_DWAITCONST);
				}
			}
			if (sldi->MsgID == message->MsgID && sldi->DestAddr == message->OrigAddr && sldState == 0x1) 
			{
				flagRF |= MSGLK;
#ifdef FRA // TUFIX: very quick fix: GW sends out RC[RC_CMD_DR_RETRIEVE] and sometimes (~3%) if the reply message to confirm  is not sent back to the GW (for some reason), the GW moves on to receing the data with the TimerShort_fired regularly, and something crashes the node
				PRINTF("ReliableComm: SendLData rcid_copy: %d\n\r",rcid_copy);
				if (rcid_copy == RC_CMD_DR_RETRIEVE)
				{
					TimerStop();
				}
#endif
				sendAckMsgTask();
			}
		} else if (message->Comment == 2) { // End of Data Receiver Unicast
			if (sldi->MsgID == message->MsgID && (flagCom & BSYLD) && sldi->DestAddr == message->OrigAddr) {
				sldState = 0x3;
				reqMissingDataTask();
			}
		} else if (message->Comment == 4) { // Done Receiver Unicast
			if (sldi->MsgID == message->MsgID && (flagCom & BSYLD) && sldi->DestAddr == message->OrigAddr && sldState == 0x3) {
				sldState = 0x4;
				flagCom &= ~(BSYLD | ACKED);
				flagRF &= ~MSGLK;
				flagRF |= MSGLK2;
				*finalAckMsg = *message;
				rm.MsgID[rm.ind] = sldi->MsgID;
				rm.DestAddr[rm.ind] = sldi->DestAddr;
				rm.ind = (rm.ind + 1) % RMSIZE;
				Timer_stop(timerTimeout);
				sendNoticeSubTask2();
			} else if ((flagRF & MSGLK2) != MSGLK2) {
				for (i = 0; i <= RMSIZE - 1; i++) {
					if (message->MsgID == rm.MsgID[i] && message->OrigAddr == rm.DestAddr[i]) {
						flagRF |= MSGLK2;
						message->ReqInd[0] = LOCAL_ADDR;
						*finalAckMsg = *message;
						sendNoticeSubTask2();
						break;
					}
				}
			}
		}
	} else if (message->DestAddr == BCAST_ADDR && message->OrigAddr != LOCAL_ADDR) { // broadcast receiver
		PRINTF("ReliableComm: SendLData handleNotice receiver broadcast\r\n");
		if (message->Comment == 0xe) {
			flagCom = 0;
			flagCom &= ~MSGLK;
			sldState = 0;
			sldi->MsgID = 255;
			sldi->DestAddr = LOCAL_ADDR;
			Timer_stop(timerNotice);
			Timer_stop(timerTimeout);
			Timer_stop(timerWait1);
			Timer_stop(timerWait2);
		} else if (message->Comment == 0xf && (sldState == 0x20 || sldState ==0x0 || sldState == 0x22)) { // Inquiry
			if ((flagCom & BSYLD) != BSYLD) { // first time to receive inquiry
				for (i = 0; i <= NTCDATASIZE - 1; i++) {
					if (message->ReqInd[i] == LOCAL_ADDR) {
						if ((flagRF & MSGLK) != MSGLK) { // This is not released until Comment 1 is received.
							flagCom = BCAST;
							flagRF |= MSGLK;
							sldState = 0x22;
							rOrder = i + NTCDATASIZE * ((message->DataLength & 0xff00) >> 8);
							numIDPackets = ((message->DataLength & 0xff) - 1) / NTCDATASIZE;
							sldi->MsgID = message->MsgID;
							sldi->DestAddr = message->OrigAddr;
							sldi->id = message->Others;
							message->ReqInd[0] = LOCAL_ADDR;
							*sldMsg = *message;
							Timer_start(timerTimeout, RC_BCINQWAITRX);
						}
						if (message->MsgID == sldi->MsgID && message->OrigAddr == sldi->DestAddr) {
							sendAckRandTask();
						}
						break;
					}
				}
			}
		} else if (message->Comment == 0x1 && sldi->DestAddr == message->OrigAddr && message->MsgID == sldi->MsgID) { // Beginning of Data (Broadcast receiver)
			if (sldState == 0x22) {
				sldState = 0x23;
				flagCom &= ~(RQMIS | ACKED | NORPL);
				flagCom |= BSYLD;
				flagRF |= MSGLK; // Not necessary. already locked.
				sldi->Length = message->DataLength;
				memset(recChk, 0, RECCHKSIZE);
				message->ReqInd[0] = LOCAL_ADDR;
				*sldMsg = *message;
				Timer_start(timerTimeout, RC_BCINQWAIT + RC_DWAIT * (sldi->Length / 1000 + 1) + RC_DWAITCONST);
			}
			if (!(flagCom & NORPL)) {
				sendAckRandTask();
			}
		} else if (message->Comment == 2 && sldi->DestAddr == message->OrigAddr && message->MsgID == sldi->MsgID) { // End of Data (Broadcast receiver)
			if (flagCom & BSYLD) {
				flagRF &= ~MSGLK;
				if (comment2CountRec != message->ReqInd[0]) {
					flagCom &= ~NORPL;
					comment2CountRec = message->ReqInd[0];
				}
				if (sldState == 0x23) {
					flagCom &= ~NORPL;
				}
				sldState = 0x25;
				if (!(flagCom & NORPL)) {
					reqMissingDataTask();
				}
			}
		} else if (message->Comment == 4 && (flagRF & MSGLK2) != MSGLK2) { // Done (Broadcast receiver)
			if (sldi->MsgID == message->MsgID && sldState == 0x25 && sldi->DestAddr == message->OrigAddr) {
				// First time to receive "End"
				sldState = 0x27;
				flagCom &= ~(BSYLD | ACKED);
				flagRF &= ~MSGLK;
				flagRF |= MSGLK2;
				message->ReqInd[0] = LOCAL_ADDR;
				*finalAckMsg = *message;
				rm.MsgID[rm.ind] = sldi->MsgID;
				rm.WaitTime[rm.ind] = rOrder;
				rm.DestAddr[rm.ind] = sldi->DestAddr;
				rm.ind = (rm.ind + 1) % RMSIZE;
				sendAckRandTask();
			} else {
				for (i = 0; i <= RMSIZE - 1; i++) {
					if (message->MsgID == rm.MsgID[i] && message->OrigAddr == rm.DestAddr[i]) {
						flagRF |= MSGLK2;
						message->ReqInd[0] = LOCAL_ADDR;
						*finalAckMsg = *message;
						xTimerChangePeriod(timerWait2, rm.WaitTime[i] * RC_REPLYINTERVAL + 1, portMAX_DELAY);
						//vTaskDelay(rm.WaitTime[i] * RC_REPLYINTERVAL + 1);
						//sendNoticeSubTask2();
						break;
					}
				}
			}
		} else if (message->Comment == 5 && sldi->DestAddr == message->OrigAddr && message->MsgID == sldi->MsgID) {
			p16 = (uint16_t *)(message->ReqInd);
			for (i = 1; i <= p16[0]; i++) {
				if (p16[i] != LOCAL_ADDR) {
					continue;
				}
				flagCom |= NORPL;
			}
		} else if (message->Comment == 6) {
			p16 = (uint16_t *)(message->ReqInd);
			for (i = 1; i <= p16[0]; i++) {
				if (p16[i] != LOCAL_ADDR) {
					continue;
				}
				for (j = 0; j < RMSIZE; j++) {
					if (message->MsgID != rm.MsgID[j] || message->OrigAddr != rm.DestAddr[j]) {
						continue;
					}
					rm.DestAddr[j] = LOCAL_ADDR;
				}
			}
		}
	} else if (message->OrigAddr == LOCAL_ADDR) { // Sender
		if (sldi->MsgID == message->MsgID && (flagCom & BSYLD)) {
			if (flagCom & BCAST) { // Broadcast Sender
				PRINTF("ReliableComm: SendLData handleNotice sender broadcast\r\n");
				if (message->Comment == 0xf) { // Broadcast 
					for (i = 0; sldi->Nodes->NodeNum - 1; i++) {
						if (message->ReqInd[0] == sldi->Nodes->NodeID[i]) {
							sldi->Nodes->AckedNode[i / 8] |= 1 << (i % 8);
							break;
						}
					}
				} else if (message->Comment == 1) { // Start sending data (Braodcast Sender)
					if (sldState == 0x23) {
						for (i = 0; i <= sldi->Nodes->NodeNum - 1; i++) {
							if (message->ReqInd[0] == sldi->Nodes->NodeID[i]) {
								ackedNode[i / 8] |= 1 << (i % 8);
								replyNode[i / 8] |= 1 << (i % 8);
							}
						}
						for (i = 0; i <= (sldi->Nodes->NodeNum - 1) / 8; i++) {
							if (ackedNode[i] != sldi->Nodes->AckedNode[i]) {
								break;
							}
							if (i == (sldi->Nodes->NodeNum - 1) / 8) {
								flagCom |= ACKED;
							}
						}
					}
				} else if (message->Comment == 3 && sldState == 0x25) { // Resending data (Braodcast Sender)
					if ((flagCom & ACKED) != ACKED) { // ACKED signal for END notice.
						for (i = 0; i <= sldi->Nodes->NodeNum - 1; i++) {
							if (message->Others == sldi->Nodes->NodeID[i]) {
								ackedNode[i / 8] |= 1 << (i % 8); // 8 because of uint8_t 
								replyNode[i / 8] |= 1 << (i % 8);
							}
						}
						for (i = 0; i <= (sldi->Nodes->NodeNum - 1) / 8; i++) {
							if (ackedNode[i] != sldi->Nodes->AckedNode[i]) {
								break;
							}
							if (i == (sldi->Nodes->NodeNum - 1) / 8) {
								flagCom |= ACKED;
							}
						}
					}
					//pack
					k = resendInd[0];
					l = resendInd[1];
					for (i16 = 0; i16 <= (int16_t)(message->DataLength) - 1; i16++) {
						if (resendInd[1] - resendInd[0] >= REQINDEXSIZE) {
							break;
						}
						n = 0;
						for(j = k; j <= l - 1; j++) {
							if (reqIndex[j % REQINDEXSIZE] == message->ReqInd[i16]) {
								n = 1;
								break;
							}
						}
						if (n == 0) {
							reqIndex[resendInd[1] % REQINDEXSIZE] = message->ReqInd[i16];
							if (resendInd[1] == 0xff) {
								resendInd[0] %= REQINDEXSIZE;
								resendInd[1] %= REQINDEXSIZE;
							}
							resendInd[1]++;
						}
					}
					if ((flagCom & ACKED) && sldState == 0x25) {
						flagRF &= ~MSGLK;
						flagCom |= ACKED;
					}
				}
			} else { // Unicast Sender
				PRINTF("ReliableComm: SendLData handleNotice sender unicast\r\n");
				if (message->Comment == 1) { // Start sending data
					if (sldState == 0x1) {
						Timer_stop(timerTimeout);
						Timer_stop(timerNotice);
						flagCom |= ACKED; // Acknowledged
						sldState = 0x2;
						nodeIndex = 0;
						flagRF &= ~MSGLK;
						Timer_start(timerTimeout, RC_DWAIT * (sldi->Length / 1000 + 1) + RC_DWAITCONST);
						startTTask();
						//sendDataTask();
					}
				} else if (message->Comment == 3) { // Replying to ReqMissing packets
					if ((flagCom & ACKED) == 0) {
						Timer_stop(timerNotice);
						flagRF &= ~MSGLK;
						flagCom |= ACKED;
						reqIndSize = message->DataLength;
						for(i = 0; i <= reqIndSize - 1;i++) {
							reqIndex[i] = message->ReqInd[i];
						}
						comment3CheckUnicastTask();
					}
				} else if (message->Comment == 4) { // Notify Done
					flagCom |= ACKED;
				}
			}
		}
	}
}

static void ldcallback_data(uint16_t src, void *payload, uint8_t len)
{
	GC_Msg *pkt;
	if (!initialized) {
		return;
	}
	pkt = sdmalloc(sizeof (GC_Msg));
	if (pkt == NULL) {
		PRINTF("ERROR: sdmalloc failed in ldcallback_data\r\n");
		die();
	}
	PRINTF("ReliableComm: SendLData ldcallback_data\r\n");
	memcpy(pkt->data, payload, len);
	if (xQueueSend(ldQueueData, &pkt, 0) != pdTRUE) {
			PRINTF("xQS data FAIL\r\n");
		sdfree(pkt);
	}
}


static void handleData(GC_Msg *m)
{
	uint8_t i;
	uint32_t idx;
	DataMsg *message = (DataMsg *)m->data;
	if (!initialized) {
		return;
	}
	PRINTF("ReliableComm: SendLData received index %u\r\n", message->Index);
	if (message->OrigAddr == sldi->DestAddr && (flagCom & BSYLD)) {
		idx = message->Index;
		if ((recChk[idx / 8] & (1 << (idx % 8))) == 0) {
			for (i = 0; i <= DATAPACKETSIZE - 1; i++) {
				if (idx * DATAPACKETSIZE + i<= sldi->Length - 1) {
					sldi->Data[idx * DATAPACKETSIZE + i] = message->Data[i];
				}
			}
			recChk[idx / 8] |= 1 << (idx % 8);
		}
		flagCom &= ~NORPL;
	}
}

static void sendNoticeRandTask(void)
{
	PRINTF("ReliableComm: SendLData sendNoticeRandTask\r\n");
	if (flagCom & BCAST) {
		if (sldState == 0x22) {
			xTimerChangePeriod(timerWait1, (uint32_t)rOrder * RC_REPLYINTERVAL + numIDPackets * RC_ONEPACKET + 1, portMAX_DELAY);
			//vTaskDelay((uint32_t)rOrder * RC_REPLYINTERVAL + numIDPackets * RC_ONEPACKET + 1);
			//sendNoticeSubTask();
		} else if (sldState == 0x23 || sldState == 0x25) {
			xTimerChangePeriod(timerWait1, (uint32_t)rOrder * RC_REPLYINTERVAL + 1, portMAX_DELAY);
			//vTaskDelay((uint32_t)rOrder * RC_REPLYINTERVAL + 1);
			//sendNoticeSubTask();
		} else if (sldState == 0x27) {
			xTimerChangePeriod(timerWait2, (uint32_t)rOrder * RC_REPLYINTERVAL + 1, portMAX_DELAY);
			//vTaskDelay((uint32_t)rOrder * RC_REPLYINTERVAL + 1);
			//sendNoticeSubTask2();
		}
	} else {
		sendNoticeSubTask();
	}
}

static void sendAckRandTask(void)
{
	PRINTF("ReliableComm: SendLData sendAckRandTask\r\n");
	flagCom |= ACKED;
	sendNoticeRandTask();
}

static void sendNoticeSubTask(void)
{
	int ret;
	PRINTF("ReliableComm: SendLData sendNoticeSubTask\r\n");
	do {
		if (!(flagRF & MSGLK)) {
			return;
		}
		ret = GenericComm_send(GC_APP_RC_LDATA_NOTICE, sldi->DestAddr, msg.data, sizeof (NoticeMsg), 0);
		if (ret != SUCCESS) {
			vTaskDelay(1);
		}
	} while (ret != SUCCESS);
	PRINTF("ReliableComm: SendLData sendNoticeSubTask state: %u commnent %u\r\n", sldState, sldMsg->Comment);
	if (flagCom & BCAST) {
		if (flagCom & SNDER) { // Sender (Bcast)
			if (sldState == 0x21) {
				flagRF &= ~MSGLK;
				if (nodeIndex == 0) {
					Timer_start(timerNotice, sldi->Nodes->NodeNum * RC_REPLYINTERVAL + RC_REPLYWAIT);
				} else {
					nodeInqTask();
				}
			} else if (sldState == 0x23 && sldMsg->Comment == 1) {
				Timer_start(timerNotice, sldi->Nodes->NodeNum * RC_REPLYINTERVAL + RC_REPLYWAIT);
			} else if (sldState == 0x23 && sldMsg->Comment == 5) {
				flagRF &= ~MSGLK;
				beginTask();
			} else if (sldState == 0x26) {
				sendNoticeEndTask();
			} else if (sldState ==0x25 && sldMsg->Comment == 2) {
				Timer_start(timerNotice, sldi->Nodes->NodeNum * RC_REPLYINTERVAL + RC_REPLYWAIT);
			} else if (sldState == 0x25 && sldMsg->Comment ==5) {
				flagRF &= ~MSGLK;
				sendNoticeEndTask();
			} else if (sldState == 0x27 && sldMsg->Comment == 4) {
				Timer_start(timerNotice, sldi->Nodes->NodeNum * RC_REPLYINTERVAL + RC_REPLYWAIT);
			} else if (sldState ==0x27 && sldMsg->Comment == 6) {
				sldMsg->Comment = 4;
				flagRF |= MSGLK;
				sendNoticeSubTask();
			}
		} else { // Receiver (Bcast)
			if (sldState == 0x22) {
				// Nothing ACKed
			} else if (sldState == 0x23) {
				// Nothing to do. Acked
			} else if (sldState ==0x25) {
				flagRF &= ~MSGLK; // Nothing to do. Acked
			}
		}
	} else {
		if (flagCom & SNDER) { // Sender (Unicast)
			if (sldState == 0x1 || sldState == 0x3 || sldState == 0x5) {
				Timer_start(timerNotice, RC_CONFWAIT);
			}
		} else { // Receiver (Unicast)
			if (sldState == 1) {
				flagRF &= ~MSGLK;	
				PRINTF("ReliableLComm Ack sent sldState == 1\r\n");
			} else if (sldState == 3) {
				flagRF &= ~MSGLK;
			}
		}
	}
}

static void sendNoticeSubTask2(void)
{
	int ret;
	PRINTF("ReliableComm: SendLData sendNoticeSubTask2\r\n");
	do {
		if (!(flagRF & MSGLK2)) {
			return;
		}
		ret = GenericComm_send(GC_APP_RC_LDATA_ACK, finalAckMsg->OrigAddr, msg2.data, sizeof (NoticeMsg), 0);
		if (ret != SUCCESS) {
			vTaskDelay(1);
		}
	} while (ret != SUCCESS);
	if (flagRF & MSGLK2) {
		flagRF &= ~MSGLK2;
		if (sldState == 0x27) {
			Timer_stop(timerTimeout);
			receivedTask();
		} else if (sldState == 0x4) {
			receivedTask();
		}
	}
}

static void sendDataSubTask(void)
{
	int ret;
	PRINTF("ReliableComm: SendLData sendDataSubTask\r\n");
	do {
		ret = GenericComm_send(GC_APP_RC_LDATA_DATA, sldi->DestAddr, msg.data, sizeof (DataMsg), 0);
		if (ret != SUCCESS) {
			vTaskDelay(1);
		}
	} while (ret != SUCCESS);
	flagRF &= ~MSGLK;
	if (flagCom & BCAST) {
		if (sldState == 0x26) {
			if (resendInd[1] > resendInd[0]) {
				startTTask();
				//sendDataTask();
			} else {
				comment2Count++;
				sendNoticeEndTask();
			}
		} else if (sldState == 0x24) { // 1st trial to send data
			if (nodeIndex * DATAPACKETSIZE <= sldi->Length - 1) {
				startTTask();
				//sendDataTask();
			} else { // The last data
				comment2Count++;
				sendNoticeEndTask();
			}
		}
	} else {
		if (sldState == 0x4) {
			if (reqIndSeq <= reqIndSize) {
				startTTask();
				//sendDataTask();
			} else {
				flagCom &= ~RQMIS; // Ready for the next request for missing packets
				sendNoticeEndTask();
			}
		} else if (sldState == 0x2) { // 1st trial to send data
			if (nodeIndex * DATAPACKETSIZE <= sldi->Length - 1) {
				startTTask();
				//sendDataTask();
			} else { // The last data
				sendNoticeEndTask();
			}
		}
	}
}

static void sendDataTask(void)
{
	uint16_t i;
	PRINTF("ReliableComm: SendLData sendDataTask\r\n");
	if ((flagRF & MSGLK) == MSGLK) {
		return;
	}
	flagRF |= MSGLK;
	dataMsg->OrigAddr = LOCAL_ADDR;
	dataMsg->Index = nodeIndex;
	for (i = 0; i <= DATAPACKETSIZE - 1; i++) {
		if (i + nodeIndex * DATAPACKETSIZE <= sldi->Length - 1) {
			dataMsg->Data[i] = sldi->Data[i + nodeIndex * DATAPACKETSIZE];
		}
	}
	if (!(flagCom & RQMIS)) { // 1st trial Both for Unicast and Multicast
		nodeIndex++;
	} else if ((flagCom & BCAST) == BCAST) { // Bcast, Sending Missing packets
		resendInd[0]++;
		if (resendInd[1] > resendInd[0]) {
			nodeIndex = reqIndex[resendInd[0] % REQINDEXSIZE]; 
		}
	} else { // Unicast, Sending Missing Packets	
		if (reqIndSeq++ <= reqIndSize) {
			nodeIndex = reqIndex[reqIndSeq];
		}
	}
	sendDataSubTask();
}

static void beginTask(void)
{
	PRINTF("ReliableLComm beginTask();\r\n");
	if ((flagRF & MSGLK) == MSGLK) {
		return;
	}
	flagRF |= MSGLK;
	sldMsg->DestAddr = BCAST_ADDR;
	sldMsg->OrigAddr = LOCAL_ADDR;
	sldMsg->MsgID = sldi->MsgID;
	sldMsg->DataLength = sldi->Length;
	sldMsg->Comment = 0x1;	//1 Begin;
	sendNoticeSubTask();
}

static void nodeInqTask(void)
{
	uint8_t i, j = 0, k = 0;
	if (flagRF & MSGLK) {
		PRINTF("ReliableComm: ERROR: SendLData nodeInqTask MSGLK\r\n");
		// need reset(); sendDone(FAIL);
		return;
	}
	PRINTF("ReliableComm: SendLData nodeInqTask inqRepeat %u acked? %x TMOUT %x\r\n", inqRepeat, flagCom & ACKED, flagCom & TMOUT);
	for (i = 0; i <= sldi->Nodes->NodeNum - 1; i++) {	
		if ((sldi->Nodes->AckedNode[i / 8] & (1 << (i % 8))) == 0) { //not ACKed Nodes
			break;
		}
	}
	if (i == sldi->Nodes->NodeNum && nodeIndex == 0) {
		flagRF &= ~MSGLK;
		sldState = 0x23;
		inqRepeat = 0;
		Timer_start(timerTimeout, RC_BCINQWAIT + RC_DWAIT * (sldi->Length / 1000 + 1) + RC_DWAITCONST);
		beginTask();
		return;
	} else if (flagCom & TMOUT) {
		for (i = 0; i <= (sldi->Nodes->NodeNum - 1) / 8; i++) {
			if (sldi->Nodes->AckedNode[i] != 0) {
				break;
			}
		}
		if (i == (sldi->Nodes->NodeNum - 1) / 8 + 1) {
			timeoutResetTask();
			return;
		}
		flagCom &= ~ACKED;
		flagCom &= ~TMOUT;
		flagRF  &= ~MSGLK;
		sldState = 0x23;
		inqRepeat = 0;
		Timer_start(timerTimeout, RC_BCINQWAIT + RC_DWAIT * (sldi->Length / 1000 + 1) + RC_DWAITCONST);
		beginTask();
		return;			
	}
	flagRF |= MSGLK;
	memset((uint16_t *)(sldMsg->ReqInd), 0, sizeof (uint16_t) * NTCDATASIZE);
	for (i = nodeIndex; i <= sldi->Nodes->NodeNum - 1; i++) {
		if ((sldi->Nodes->AckedNode[i / 8] & (1 << (i % 8))) == 0) { // not ACKed Nodes
			sldMsg->ReqInd[j++] = sldi->Nodes->NodeID[i]; // pack in a packet
			k++;
		} else {
			sldMsg->ReqInd[j++] = 0xffff;
		}
		if (j >= NTCDATASIZE || i == sldi->Nodes->NodeNum - 1) { // packet is full or all the nodeIDs are packed
			sldMsg->DataLength = (nodeIndex / NTCDATASIZE) << 8 | sldi->Nodes->NodeNum;
			nodeIndex = i + 1; // starting point for the next packet
			break;
		}
	}
	if (nodeIndex == sldi->Nodes->NodeNum) {
		nodeIndex = 0; // to repeat inquiry;
	}
	PRINTF("ReliableComm: SendLData nodeInqTask num of nnreplied nodes %u\r\n", k);	
	if (k == 0) {
		flagRF &= ~MSGLK;
		if (nodeIndex == 0) {
			Timer_start(timerNotice, sldi->Nodes->NodeNum * RC_REPLYINTERVAL + RC_REPLYWAIT);
		} else {
			nodeInqTask();
		}
	} else {
		sldMsg->Comment = 0xf;
		sendNoticeSubTask();
	}
}

static void reqMissingDataTask(void)
{
	uint32_t i;
	uint8_t j = 0;
	if (flagRF & MSGLK) {
		return;
	}
	PRINTF("ReliableComm: SendLData reqMissingDataTask\r\n");
	flagRF |= MSGLK;
	sldMsg->Comment = 3;
	memset((uint16_t *)(sldMsg->ReqInd), 0, NTCDATASIZE * sizeof (uint16_t));
	for (i = 0; i <= sldi->Length - 1; i += DATAPACKETSIZE) {
		if ((recChk[i / DATAPACKETSIZE / 8] & (1 << ((i / DATAPACKETSIZE) % 8))) == 0) {
			sldMsg->ReqInd[j] = i / DATAPACKETSIZE;
			j++;
			if (j >= NTCDATASIZE) {
				break;
			}
		}
	}
	sldMsg->DataLength = j;
	sldMsg->Others = LOCAL_ADDR;
	if (flagCom & BCAST) {
		sendNoticeRandTask();
	} else {
		sendNoticeSubTask();
	}
}

static void sendAckMsgTask(void)
{
	PRINTF("ReliableComm: SendLData sendAckMsgTask\r\n");
	if ((flagRF & MSGLK2) == MSGLK2) {
		sendNoticeSubTask2();
	} else {
		sendNoticeSubTask();
	}
}

static void comment3CheckTask(void )
{
	PRINTF("ReliableComm: SendLData comment3CheckTask\r\n");
	memset(ackedNode, 0, ACKEDNODENUM);
	if (resendInd[1] > resendInd[0]) {
		flagCom |= RQMIS;
		flagCom &= ~ACKED;
		nodeIndex = reqIndex[resendInd[0] % REQINDEXSIZE];
		startTTask();
		//sendDataTask();
	} else {
		sldState = 0x27;
		sldMsg->Comment = 4;
		flagCom &= ~RQMIS;
		flagCom &= ~ACKED;
		flagRF |= MSGLK;
		memset(replyNode, 0, ACKEDNODENUM);
		sendNoticeSubTask();
	}
}

static void comment3CheckUnicastTask(void)
{
	PRINTF("ReliableComm: SendLData comment3CheckUnicastTask\r\n");
	if (reqIndSize >= 1) {
		sldState = 0x4;
		flagCom |= RQMIS; // Sending Missing Packets
		reqIndSeq = 0;
		nodeIndex = reqIndex[reqIndSeq++];	
		startTTask();
		//sendDataTask();
	} else {
		sldState = 0x5;
		flagCom &= ~(RQMIS | ACKED);
		flagRF |= MSGLK;
		sldMsg->Comment = 4;
		Timer_start(timerTimeout, RC_INQWAIT);
		sendNoticeSubTask();
	}
}

static void sendNoticeEndTask(void)
{
	if (flagRF & MSGLK) {
		return;
	}
	PRINTF("ReliableComm: SendLData sendNoticeEndTask\r\n");
	flagRF |= MSGLK;
	if (sldState != 0x25) {
		flagCom &= ~ACKED;
	}
	if (flagCom & BCAST) {
		sldState = 0x25;
	} else {
		sldState = 0x3;
	}
	sldMsg->DestAddr = sldi->DestAddr;
	sldMsg->OrigAddr = LOCAL_ADDR;
	sldMsg->DataLength = sldi->Length;
	sldMsg->MsgID = sldi->MsgID;
	sldMsg->Comment = 2; //2 End;
	sldMsg->ReqInd[0] = comment2Count;
	memset(replyNode, 0, ACKEDNODENUM);
	sendNoticeSubTask();
}

static void sendDoneTask(void)
{
	PRINTF("ReliableComm: SendLData sendDoneTask\r\n");
	sldState = 0;
	flagRF &= ~MSGLK;
	flagCom &= ~(BSYLD | SNDER | ACKED | RQMIS | BCAST | TMOUT);
	inqRepeat = 0;
	//scallback(SUCCESS);
	startSTask(SUCCESS);
}

static void receivedTask(void)
{
	flagRF &= ~MSGLK2;
	flagCom &= ~(BSYLD | SNDER | ACKED | RQMIS | BCAST);
	sldState = 0;
	PRINTF("ReliableComm: SendLData receivedTask\r\n");
	//rcallback(SUCCESS);
	startRTask(SUCCESS);
}

static void informAckNodeTask(void)
{
	uint8_t i;
	uint16_t *p16;
	if (flagRF & MSGLK) {
		return;
	}
	PRINTF("ReliableComm: SendLData informAckNodeTask\r\n");
	flagRF |= MSGLK;
	p16 = (uint16_t *)(sldMsg->ReqInd);
	p16[0] = 0;
	for (i = 0; i < sldi->Nodes->NodeNum; i++) {
		if (replyNode[i / 8] & (1 << (i % 8))) {
			p16[++p16[0]] = sldi->Nodes->NodeID[i];
			if (p16[0] >= NTCDATASIZE - 1) {
				break;
			}
		}
	}
	if (sldState == 0x27) {
		sldMsg->Comment = 0x6;
	} else {
		sldMsg->Comment = 0x5;
	}
	memset(replyNode, 0, ACKEDNODENUM);
	sendNoticeSubTask();
}

static void ldRecvTask(void *pvParameters)
{
	GC_Msg *pkt;

	// The parameters are not used.
	( void ) pvParameters;

	while (1) {
		if(xQueueReceive(ldQueueAck, &pkt, 1) == pdTRUE) {
			//PRINTF("ReliableComm: SendLData xQR ack\r\n");
			handleAck(pkt);
		  sdfree(pkt);
		}
		if(xQueueReceive(ldQueueNotice, &pkt, 1) == pdTRUE) {
			//PRINTF("ReliableComm: SendLData xQR notice\r\n");
			handleNotice(pkt);
		  sdfree(pkt);
		}
		if(xQueueReceive(ldQueueData, &pkt, 1) == pdTRUE) {
			//PRINTF("ReliableComm: SendLData xQR data\r\n");
			handleData(pkt);
		  sdfree(pkt);
		}
	}
}
