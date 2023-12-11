#include <xnode.h>
#include <stdlib.h>
#include <string.h>
#include <queue.h>
#include "GenericComm.h"
#include "SendSMsg.h"

#define _DEBUG_               0 // 0 for non debugging mode
#if _DEBUG_
#define PRINTF(...)           lpc_printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static enum { STATE0, STATE1, STATE2, STATE3 } state = STATE0;
static GC_Msg gMsg1, gMsg2;
static uint8_t flagCom, flagRF, nodeIndex, rOrder, inqRepeat, numIDPackets; 
static uint16_t msgIDLocal, replyRepeat;
static uint8_t *ackedNode, *replyNode;
static SSMInfo ssmi, ssmiRx, tssmi;
static NodeInfo ni;
static ShortMsg *pGMsg1 = (ShortMsg *)gMsg1.data; // sender packet
static ShortMsg *pGMsg2 = (ShortMsg *)gMsg2.data; // the last reply packet involving rm check
static RecentMsg rm;
static bool initialized = false;
static sm_callback_t scallback, rcallback;
static QueueHandle_t smQueue = NULL;
static TimerHandle_t timerShort, timerTimeout;

void sendAckRandTask(void);
void sendAckRandTask2(void);
void sendShortSubTask(void);
void sendShortSubTask2(void);
void nodeInqTask(void);
void beginTask(void);
void sendDoneTask(void);
void receivedTask(void);
void informAckNodeTask(void);
void timeoutResetTask(void);
void ssmiReorder(SSMInfo *s);
void smcallback(uint16_t src, void *payload, uint8_t len);
void handleReceiveCommon(GC_Msg *m);
void TimerShort_fired(TimerHandle_t pxTimer);
void TimerTimeout_fired(TimerHandle_t pxTimer);
static void smRecvTask(void *pvParameters);
#ifdef FRA
TaskHandle_t SSMTask;
#endif
int SendSMsg_init(sm_callback_t scb, sm_callback_t rcb)
{
	if (initialized) {
		return SUCCESS;
	}
	if (GenericComm_init() != SUCCESS || GenericComm_register(GC_APP_RC_SMSG, smcallback) != SUCCESS) {
		PRINTF("ReliableComm: ERROR: SendSMsg failed to init GenericComm\r\n");
		return FAIL;
	}
	smQueue = xQueueCreate(1, sizeof (GC_Msg *));
	timerShort = xTimerCreate("SMShort", 1000, pdFALSE, NULL, TimerShort_fired);
	timerTimeout = xTimerCreate("SMTimeout", 1000, pdFALSE, NULL, TimerTimeout_fired);
	configASSERT(smQueue && timerShort && timerTimeout);
#ifndef FRA
	configASSERT(xTaskCreate(smRecvTask, "SMRecvTask", configMINIMAL_STACK_SIZE, NULL,
		(tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL) == pdPASS);
#else
	configASSERT(xTaskCreate(smRecvTask, "SMRecvTask", configMINIMAL_STACK_SIZE, NULL,
		(tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) SSMTask) == pdPASS);
#endif
	scallback = scb;
	rcallback = rcb;
	state = STATE0;
	flagCom = flagRF = inqRepeat = 0;
	memset(&rm, 0, sizeof (RecentMsg));
	memset(&ssmi, 0, sizeof (SSMInfo));
	memset(&ssmiRx, 0, sizeof (SSMInfo));
	memset(&ni, 0, sizeof (NodeInfo));
	ssmi.Nodes = ssmiRx.Nodes = &ni;
	ssmi.MsgID = ssmiRx.MsgID = 255;
	ackedNode = (uint8_t *)sdmalloc(ACKEDNODENUM);
	replyNode = (uint8_t *)sdmalloc(ACKEDNODENUM);
	configASSERT(ackedNode && replyNode);
	initialized = true;
	PRINTF("ReliableComm: SendSMsg_init\r\n");
	return SUCCESS;
}

int SendSMsg_reset(void)
{
	if (!initialized) {
		return FAIL;
	}
	flagCom = flagRF = inqRepeat = 0;
	state = STATE0;
	memset(&rm, 0, sizeof (RecentMsg));
	memset(&ssmi, 0, sizeof (SSMInfo));
	memset(&ssmiRx, 0, sizeof (SSMInfo));
	ssmi.Nodes = ssmiRx.Nodes = &ni;
	ssmi.MsgID = ssmiRx.MsgID = 255;
	ssmiRx.DestAddr = BCAST_ADDR;
	pGMsg1->DestAddr = ssmiRx.DestAddr;
	pGMsg1->OrigAddr = LOCAL_ADDR;
	pGMsg1->Comment = 0xe;
	sendShortSubTask();
	Timer_stop(timerShort);
	Timer_stop(timerTimeout);
	PRINTF("ReliableComm: SendSMsg_reset\r\n");
	return SUCCESS;
}

bool SendSMsg_isBusy(void)
{
	return (flagCom & BSYSM) || (flagRF & MSGLK);
}
extern bool TIM_busy;
int SendSMsg_send(SSMInfo *s)
{
	if (!initialized || SendSMsg_isBusy()) {
		PRINTF("ReliableComm: SendSMsg_send fail\r\n");
		return FAIL;
	}
	flagCom = BSYSM | SNDER;
	flagRF |= MSGLK;
	state = STATE1;
	ssmi = *(SSMInfo *)s;
	ssmi.DestAddr = pGMsg1->DestAddr = ssmi.Nodes->NodeID[0];
	pGMsg1->OrigAddr = LOCAL_ADDR;
	msgIDLocal = (msgIDLocal + 1) % 255;
	pGMsg1->MsgID = ssmi.MsgID = msgIDLocal;
	pGMsg1->Size = ssmi.Size;
	pGMsg1->Comment = 1;
	pGMsg1->id = ssmi.id;
	memcpy(pGMsg1->Data, ssmi.Data, SMDDATASIZE);
	memset(&rm, 0, sizeof (RecentMsg));
	memset(&ni, 0, sizeof (NodeInfo));
	inqRepeat = 0;
	PRINTF("ReliableComm: SendSMsg_send\r\n");
	Timer_start(timerTimeout, RC_INQWAIT);
	sendShortSubTask();
	return SUCCESS;
}

int SendSMsg_bcast(SSMInfo *s)
{
	if (!initialized || SendSMsg_isBusy()) {
		PRINTF("ReliableComm: SendSMsg_bcast fail\r\n");
		return FAIL;
	}
	flagCom = BSYSM | SNDER | BCAST; 
	state = STATE2;
	nodeIndex = 0;
	ssmi = *(SSMInfo *)s;
	ssmi.DestAddr = pGMsg1->DestAddr = BCAST_ADDR;
	pGMsg1->OrigAddr = LOCAL_ADDR;
	msgIDLocal = (msgIDLocal + 1) % 255;
	pGMsg1->MsgID = ssmi.MsgID = msgIDLocal;
	pGMsg1->Comment = 2;
	pGMsg1->id = ssmi.id;
	memset(ssmi.Nodes->AckedNode, 0, ACKEDNODENUM);
	memset(ackedNode, 0, ACKEDNODENUM);
	memset(replyNode, 0, ACKEDNODENUM);
	memset(&rm, 0, sizeof (RecentMsg));
	memset(&ni, 0, sizeof (NodeInfo));
	inqRepeat = 0;
	PRINTF("ReliableComm: SendSMsg_bcast\r\n");
	Timer_start(timerTimeout, RC_BCINQWAIT);
	nodeInqTask();
	return SUCCESS;
}

static void TimerShort_fired(TimerHandle_t pxTimer)
{
	uint8_t i;
	if (!initialized) {
		return;
	}
	PRINTF("ReliableComm: SendSMsg TimerShort_fired\r\n");
	if (flagCom & BCAST) {
		if (flagCom & SNDER) { // Bcast Sender
			if (state == STATE2) { 
				inqRepeat++;
				nodeInqTask();
			} else if (state == STATE3) {
				if (flagCom & ACKED) {
					memcpy(ssmi.Nodes->AckedNode, ackedNode, ACKEDNODENUM * sizeof(uint8_t));
					ssmiReorder(&ssmi);
					Timer_stop(timerTimeout);
					sendDoneTask();
				} else if (flagCom & TMOUT) {
					for (i = 0; i <= (ssmi.Nodes->NodeNum - 1) / 8; i++) {
						if (ackedNode[i] != 0) {
							break;
						}
					}
					if (i != (ssmi.Nodes->NodeNum - 1) / 8 + 1) {
						memcpy(ssmi.Nodes->AckedNode, ackedNode, ACKEDNODENUM * sizeof(uint8_t));
						ssmiReorder(&ssmi);
						sendDoneTask();
					} else {
						timeoutResetTask();
					}
				} else {
					inqRepeat++;
					flagRF &= ~MSGLK;
					informAckNodeTask();
				}
			}
		} else { // Bcast Receiver
			if (state != STATE3) {
				return;
			}
			PRINTF("ReliableComm: ERROR: SendSMsg TimerShort/BcastReceiver should not reach here.\r\n");
		}
	} else if (flagCom & SNDER) { // Unicast Sender
		if (state != STATE1) {
			return;
		}
		if (!(flagCom & ACKED)) {
			inqRepeat++;
			sendShortSubTask();
			return;
		}
	}
}

static void TimerTimeout_fired(TimerHandle_t pxTimer)
{
	if (!initialized) {
		return;
	}
	PRINTF("ReliableComm: SendSMsg TimerTimeout_fired\r\n");
	if (!(flagCom & BCAST)) { // Unicast
		flagCom |= TMOUT;
		timeoutResetTask();
	} else {
		if ((flagCom & SNDER) && state == STATE2) { // if nothing, time out. if some, continue with reduced number of targets.
			flagCom |= TMOUT;
		} else if ((flagCom & SNDER) && state == STATE3) {
			flagCom |= TMOUT;
		}
		if (!(flagCom & SNDER) && state == STATE2) { // Bcast Receiver
			timeoutResetTask();
		}
	}
}

static void timeoutResetTask(void)
{
	PRINTF("ReliableComm: SendSMsg timeoutResetTask\r\n");
	Timer_stop(timerShort);
	Timer_stop(timerTimeout);
	ssmi.MsgID = 255;
	/*
	if (flagCom & SNDER) {
		scallback(&ssmi, FAIL);
	}
	*/
	tssmi = ssmi;
	
	flagCom = flagRF = inqRepeat = 0;
	state = STATE0;
	memset(&ssmi, 0, sizeof (SSMInfo));
	memset(&ssmiRx, 0, sizeof (SSMInfo));
	ssmi.Nodes = &ni;
	ssmiRx.Nodes = &ni;
	ssmi.MsgID = 255;
	
	//if (flagCom & SNDER) {
		scallback(&tssmi, FAIL);
	//}
}

static void ssmiReorder(SSMInfo *s)
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

static void smcallback(uint16_t src, void *payload, uint8_t len)
{
	GC_Msg *pkt;
	if (!initialized) {
		return;
	}
	pkt = sdmalloc(sizeof (GC_Msg));
	configASSERT(pkt);
	PRINTF("ReliableComm: SendSMsg smcallback\r\n");
	memcpy(pkt->data, payload, len);
	if (xQueueSend(smQueue, &pkt, 0) != pdTRUE) {
		sdfree(pkt);
	}
}

static void handleReceiveCommon(GC_Msg *m)
{
	uint8_t i, j;
	uint16_t *p16;
	ShortMsg *message = (ShortMsg *)m->data;
	p16 = (uint16_t *)message->Data;
	if (!initialized) {
		return;
	}
	PRINTF("ReliableComm: SendSMsg handleReceiveCommon\r\n message->DestAddr = %d, LOCAL_ADDR = %d",message->DestAddr,LOCAL_ADDR);
	if (message->DestAddr == LOCAL_ADDR) { // Unicast Receiver
		PRINTF("ReliableComm: SendSMsg unicast receiver\r\n"); 
		if ((message->Comment != 1) || (flagRF & MSGLK2)) {
			return;
		}
		for (i = 0; i < RMSIZE; i++) {
			if (message->MsgID != rm.MsgID[i] || message->OrigAddr != rm.DestAddr[i]) {
				continue;
			}
			flagRF |= MSGLK2;
			flagRF |= ACKDN; // Tomo -> flag to indicate this is late ack/change from flagCom to flagRF
			p16 = (uint16_t *)message->Data;
			*p16 = LOCAL_ADDR;
			*pGMsg2 = *message;
			sendShortSubTask2();
			return;
		}
		if (flagCom & BSYSM) {
			return;
		}
		flagCom &= ~SNDER;
		flagCom |= BSYSM;
		flagRF |= MSGLK2;
		state = STATE1;
		*pGMsg2 = *message;			
		ssmiRx.DestAddr = message->OrigAddr;
		ssmiRx.MsgID = message->MsgID;
		ssmiRx.Size = message->Size;
		ssmiRx.id = message->id;
		memcpy(ssmiRx.Data, message->Data, SMDDATASIZE);
		rm.MsgID[rm.ind] = ssmiRx.MsgID;
		rm.DestAddr[rm.ind] = ssmiRx.DestAddr;
		rm.ind = (rm.ind + 1) % RMSIZE;
		sendShortSubTask2();
	} else if (message->DestAddr == BCAST_ADDR && message->OrigAddr != LOCAL_ADDR) { // Broadcast Receiver
		PRINTF("ReliableComm: SendSMsg broadcast receiver\r\n"); 
		if (message->Comment == 0xe) {
			flagCom = flagRF = inqRepeat = 0;
			state = STATE0;
			memset(&rm, 0, sizeof (RecentMsg));
			memset(&ssmi, 0, sizeof (SSMInfo));
			memset(&ssmiRx, 0, sizeof (SSMInfo));
			ssmi.Nodes = ssmiRx.Nodes = &ni;
			ssmi.MsgID = ssmiRx.MsgID = 255;
			Timer_stop(timerShort);
			Timer_stop(timerTimeout);
		} else if (message->Comment == 2 && !(flagCom & SNDER)) {
			// Inquiry, not working as a sender
			p16 = (uint16_t *)message->Data;
			for (i = 0; i < SMDDATASIZE / 2; i++) {
				if (p16[i] != LOCAL_ADDR) {
					continue;
				}
				if (!SendSMsg_isBusy()) { // received the inq packet for the first time
					flagCom = BCAST | BSYSM;
					flagCom &= ~SNDER;
					flagRF |= MSGLK;
					state = STATE2;
					rOrder = i + (SMDDATASIZE / 2) * (message->Others - 1);
					numIDPackets = (message->Size - 1) / (SMDDATASIZE / 2) + 1;
					ssmiRx.MsgID = message->MsgID;
					ssmiRx.DestAddr = message->OrigAddr;
					ssmiRx.id = message->id;
					p16[0] = LOCAL_ADDR;
					p16[1] = 0;
					replyRepeat = 1;
					*pGMsg1 = *message;
					Timer_start(timerTimeout, RC_BCINQWAITRX);
				}
				if (message->MsgID == ssmiRx.MsgID && message->OrigAddr == ssmiRx.DestAddr) {
					p16 = (uint16_t *)pGMsg1->Data;
					p16[1] = replyRepeat++;
					sendAckRandTask();
				}
				break;
			}
		} else if (message->Comment == 3 && !(flagRF & MSGLK2)) { // SMD
			if (ssmiRx.MsgID == message->MsgID && ssmiRx.DestAddr == message->OrigAddr && state == STATE2) {
				Timer_stop(timerTimeout);
				state = STATE3;
				flagRF &= ~MSGLK;	
				flagRF |= MSGLK2;
				ssmiRx.Size = message->Size;
				memcpy(ssmiRx.Data, message->Data, SMDDATASIZE);
				p16 = (uint16_t *)message->Data;
				p16[0] = LOCAL_ADDR;
				p16[1] = 0;
				replyRepeat = 1;
				*pGMsg2 = *message;
				rm.MsgID[rm.ind] = ssmiRx.MsgID;
				rm.WaitTime[rm.ind] = rOrder;
				rm.DestAddr[rm.ind] = ssmiRx.DestAddr;
				rm.ind = (rm.ind + 1) % RMSIZE;
				sendAckRandTask2();
			} else {
				p16 = (uint16_t *)message->Data;
				for (i = 0; i < RMSIZE; i++) {
					if (message->MsgID != rm.MsgID[i] || message->OrigAddr != rm.DestAddr[i]) {
						continue;
					}
					flagRF |= MSGLK2;
					flagRF |= ACKDN;
					p16[0] = LOCAL_ADDR;
					p16[1] = replyRepeat++;
					*pGMsg2 = *message;
					vTaskDelay(rm.WaitTime[i] * RC_REPLYINTERVAL + 1);
					sendShortSubTask2();
					break;
				}
			}
		} else if (message->Comment == 4) {
			p16 = (uint16_t *)message->Data;
			for (i = 1; i <= p16[0]; i++) {
				if (p16[i] != LOCAL_ADDR) {
					continue;
				}
				for (j = 0; j < RMSIZE; j++) {
					if (message->MsgID != rm.MsgID[j] || message->OrigAddr != rm.DestAddr[j]) {
						continue;
					}
					rm.DestAddr[j]=LOCAL_ADDR;
				}
			}
		}
	} else if (message->OrigAddr == LOCAL_ADDR && message->MsgID == ssmi.MsgID && (flagCom & BSYSM)) { // Sender 
		if (!(flagCom & BCAST)) {
			if (message->Comment == 1 && state == STATE1) { // Unicast Sender
				flagCom |= ACKED;
				Timer_stop(timerShort);
				Timer_stop(timerTimeout);
				sendDoneTask();
			}
		} else { // Bcast Sender
			if (message->Comment == 2 && state == STATE2) { // Bcast inquiry
				p16 = (uint16_t *)message->Data;
				for (i = 0; i < ssmi.Nodes->NodeNum; i++) {
					if (p16[0] == ssmi.Nodes->NodeID[i]) {
						ssmi.Nodes->AckedNode[i / 8] |= (1 << (i % 8));
						break;
					}
				}
			} else if (message->Comment == 3 && state == STATE3) { // Acked
				p16 = (uint16_t *)message->Data;
				for (i = 0; i < ssmi.Nodes->NodeNum; i++) {
					if (p16[0] == ssmi.Nodes->NodeID[i]) {
						ackedNode[i / 8] |= (1 << (i % 8));
						replyNode[i / 8] |= (1 << (i % 8));
					}
				}
				for (i = 0; i <= ((ssmi.Nodes->NodeNum - 1) / 8); i++) {
					if ((ackedNode[i] & ssmi.Nodes->AckedNode[i]) != ssmi.Nodes->AckedNode[i]) {
						return;
					}
				}
				memcpy(ssmi.Nodes->AckedNode, ackedNode, ACKEDNODENUM);
				flagCom |= ACKED;
			}
		}
	}
}

#ifdef FRA
void TimerStop(void)
{
	PRINTF("ReliableComm: SendSMsg TimerStop\r\n");
	Timer_stop(timerShort);
	Timer_stop(timerTimeout);
}
#endif // TUFIX: very quick fix: GW sends out RC[RC_CMD_DR_RETRIEVE] and sometimes (~3%) if the reply message to confirm  is not sent back to the GW (for some reason), the GW moves on to receing the data with the TimerShort_fired regularly, and something crashes the node
static void sendDoneTask(void)
{
	PRINTF("ReliableComm: SendSMsg sendDoneTask\r\n");
	/*
	scallback(&tssmi, SUCCESS);
	*/
	tssmi = ssmi;
	
	state = STATE0;
	flagRF &= ~MSGLK;
	flagCom &= ~(BSYSM | ACKED | SNDER | BCAST);
	inqRepeat = 0;
	SendSMsg_reset();
	
	scallback(&tssmi, SUCCESS);
}

static void receivedTask(void)
{
	PRINTF("ReliableComm: SendSMsg receivedTask\r\n");
	/*
	rcallback(&ssmiRx, SUCCESS);
	*/
	tssmi = ssmiRx;
	
	flagRF &= ~MSGLK2;
	flagCom &= ~BSYSM;
	state = STATE0;
	memset(&ssmiRx, 0, sizeof (SSMInfo));
	ssmiRx.Nodes = &ni;
	SendSMsg_reset();
	
	rcallback(&tssmi, SUCCESS);
}

static void sendShortSubTask(void)
{
	int ret;
	PRINTF("ReliableComm: SendSMsg sendShortSubTask\r\n");
	do {
		if (!(flagRF & MSGLK)) {
			PRINTF("!(flagRF & MSGLK)\n\r");		
			return;
		}
		if (flagCom & SNDER) {
			PRINTF("flagCom & SNDER\n\r");		
			ret = GenericComm_send(GC_APP_RC_SMSG, ssmi.DestAddr, gMsg1.data, sizeof (ShortMsg), 0);
		} else {
			ret = GenericComm_send(GC_APP_RC_SMSG, ssmiRx.DestAddr, gMsg1.data, sizeof (ShortMsg), 0);
		}
  } while (ret != SUCCESS);
	PRINTF("debug 3 bool TIM_busy = %d\r\n",TIM_busy);
	if (flagCom & BCAST) { // Bcast
		PRINTF("flagCom & BCAST\n\r");
		if (!(flagCom & SNDER)) {	
			return; // Bcast Receiver
		}	// Bcast Sender
		if (state == STATE2) {
			flagRF &= ~MSGLK;
			if (nodeIndex == 0) {
				Timer_start(timerShort, ssmi.Nodes->NodeNum * RC_REPLYINTERVAL + RC_REPLYWAIT);
			} else {
				nodeInqTask();
			}
		} else if (state == STATE3 && pGMsg1->Comment == 3) {
			Timer_start(timerShort, ssmi.Nodes->NodeNum * RC_REPLYINTERVAL + RC_REPLYWAIT);
		} else if (pGMsg1->Comment == 4) {
			beginTask();
		}
	} else { // Unicast
		if (flagCom & SNDER) { // Unicast Sender
			PRINTF("flagCom & SNDER\n\r");
			if (flagCom & TMOUT) {
				PRINTF("flagCom & TMOUT\n\r");
				timeoutResetTask();
				return;
			}
			if (state == STATE1) {
				PRINTF("state == STATE1\n\r");
				Timer_start(timerShort, RC_ACKWAIT + (rand() % 20));
			}
		} else { // Unicast Receiver
			// never reach here (should be OK)
		}
	}
}

static void sendShortSubTask2(void)
{
	int ret;
	PRINTF("ReliableComm: SendSMsg sendShortSubTask2\r\n");
	do {
		if (!(flagRF & MSGLK2)) {
			return;
		}
		ret = GenericComm_send(GC_APP_RC_SMSG, pGMsg2->OrigAddr, gMsg2.data, sizeof (ShortMsg), 0);
	} while (ret != SUCCESS);
	if (flagRF & MSGLK2) {
		if (flagRF & ACKDN) {
			flagRF &= ~ACKDN;
			flagRF &= ~MSGLK2;
		} else if (state == STATE3) {
			Timer_stop(timerTimeout);
			receivedTask();
		} else if (state == STATE1) {
			Timer_stop(timerTimeout);
			receivedTask();
		}
	}
}

static void sendAckRandTask(void)
{
	if (!(flagCom & BCAST)) {
		return;
	}
	PRINTF("ReliableComm: SendSMsg sendAckRandTask\r\n");
	vTaskDelay((uint32_t)rOrder * RC_REPLYINTERVAL + 1 + numIDPackets * RC_ONEPACKET);
	sendShortSubTask();
}

static void sendAckRandTask2(void)
{
	if (!(flagCom & BCAST)) {
		return;
	}
	PRINTF("ReliableComm: SendSMsg sendAckRandTask2\r\n");
	vTaskDelay((uint32_t)rOrder * RC_REPLYINTERVAL + 1);
	sendShortSubTask2();
}

static void nodeInqTask(void)
{		
	uint8_t i, j = 0, k = 0;
	uint16_t *p16;
	if (flagRF & MSGLK) {
		return;
	}
	PRINTF("ReliableComm: SendSMsg nodeInqTask\r\n");
	for (i = 0; i < ssmi.Nodes->NodeNum; i++) { // find not-acked nodes
		if ((ssmi.Nodes->AckedNode[i / 8] & (1 << (i % 8))) == 0) {
			// not ACKed Nodes
			break;
		}
	}
	if (i == ssmi.Nodes->NodeNum && nodeIndex == 0) { 
		// no not-acked node in the end inquiry packet, go to the next step.
		flagCom &= ~ACKED;
		state = STATE3;
		inqRepeat = 0;
		Timer_start(timerTimeout, RC_BCINQWAIT);
		return;
	} else if (flagCom & TMOUT) { // when timed out
		for (i = 0; i <= (ssmi.Nodes->NodeNum - 1) / 8; i++) {
			if (ssmi.Nodes->AckedNode[i] != 0) {
				break;
			}
		}
		if (i == (ssmi.Nodes->NodeNum - 1) / 8 + 1) {
			timeoutResetTask();
			return;
		}
		flagCom &= ~ACKED;
		flagCom &= ~TMOUT;
		state = STATE3;
		inqRepeat = 0;
		Timer_start(timerTimeout, RC_BCINQWAIT);
		beginTask();
		return;
	}
	// prepare for sending list of not-acked nodes
	flagRF |= MSGLK;
	memset(pGMsg1->Data, 0, SMDDATASIZE);
	p16 = (uint16_t *)pGMsg1->Data;
	for (i = nodeIndex; i < ssmi.Nodes->NodeNum; i++) { // pack the node lists
		if ((ssmi.Nodes->AckedNode[i / 8] & (1 << (i % 8))) == 0) { 
			// not ACKed Nodes -> pack in the packet
			p16[j++] = ssmi.Nodes->NodeID[i];
			k++;
		} else {
			p16[j++] = 0xffff;
		}
		if (j >= SMDDATASIZE/2 || i == ssmi.Nodes->NodeNum - 1) {
			// packet is full or all the nodeIDs are packed
			pGMsg1->Others = nodeIndex / (SMDDATASIZE / 2) + 1; 
			pGMsg1->Size = ssmi.Nodes->NodeNum;
			nodeIndex = i + 1;
			// the "Others"-th packet/"Size" packets
			// starting point for the next packet
			break;
		}
	}
	if (nodeIndex == ssmi.Nodes->NodeNum) { // list reached the end
		nodeIndex = 0;
	}
	if (k == 0) { // All the nodes in the packet is acked.
		flagRF &= ~MSGLK;
		if (nodeIndex == 0) { // wait for reply from previous inquiry packets
			Timer_start(timerShort, ssmi.Nodes->NodeNum * RC_REPLYINTERVAL + RC_REPLYWAIT);
		} else {
			nodeInqTask(); // go to the next in the node list.
		}
	} else {
		pGMsg1->Comment = 2;
		sendShortSubTask();
	}
}

static void beginTask(void)
{
	if (pGMsg1->Comment != 4) {		
		if (flagRF & MSGLK) {
			return;
		}
	}
	PRINTF("ReliableComm: SendSMsg beginTask\r\n");
	if (pGMsg1->Comment != 4) {
		flagCom &= ~ACKED;
	}
	flagRF |= MSGLK;
	pGMsg1->DestAddr = BCAST_ADDR;
	pGMsg1->OrigAddr = LOCAL_ADDR;
	pGMsg1->MsgID = ssmi.MsgID;
	pGMsg1->Size = ssmi.Size;
	pGMsg1->Comment = 3;
	memcpy(pGMsg1->Data, ssmi.Data, SMDDATASIZE);
	sendShortSubTask();
}

static void informAckNodeTask(void)
{
	uint8_t i;
	uint16_t *p16;
	if (flagRF & MSGLK) {
		return;
	}
	PRINTF("ReliableComm: SendSMsg informAckNodeTask\r\n");
	flagRF |= MSGLK;
	p16 = (uint16_t *)pGMsg1->Data;
	p16[0] = 0;
	pGMsg1->Comment = 4;
	for (i = 0; i < ssmi.Nodes->NodeNum; i++) {
		if (replyNode[i / 8] & (1 << (i % 8)) ) {
			p16[++p16[0]]= ssmi.Nodes->NodeID[i];
			if (p16[0] >= (SMDDATASIZE / 2 - 1)) {
				break;
			}
		}
	}
	memset(replyNode,0, ACKEDNODENUM);
	sendShortSubTask();
}

static void smRecvTask(void *pvParameters)
{
	GC_Msg *pkt;

	// The parameters are not used.
	( void ) pvParameters;

	while (1) {
		if(xQueueReceive(smQueue, &pkt, portMAX_DELAY) == pdTRUE) {
			handleReceiveCommon(pkt);
		  sdfree(pkt);
		}
	}
}
