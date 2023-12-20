#include <xnode.h>
#include <string.h>
#include "Radio.h"
#include "GenericComm.h"
#include "lpc43xx_timer.h"

#define _DEBUG_               1 // 0 for non debugging mode

#if _DEBUG_
#define PRINTF(...)           lpc_printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
#ifdef FRA
extern uint8_t tindex_ftp, tcount_ftp, targets2_ftp[MAX_NODES],counter_ftp, i;
bool detectoldid = false;
#endif
static gc_callback_t apps[GC_MAX_APPS];
static bool initialized = false;

static void gcRecvTask(void *pvParameters);

int GenericComm_init(void)
{
	if (initialized) {
		return SUCCESS;
	}
	if (radio_init() != SUCCESS) {
		PRINTF("GenericComm: ERROR: failed to init radio\r\n");
		return FAIL;
	}
	configASSERT(xTaskCreate(gcRecvTask, "GCRecvTask", configMINIMAL_STACK_SIZE, NULL,
	(tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL) == pdPASS);
	radio_set_short_addr(LOCAL_ADDR);
	if (RADIO_CHANNEL != RF_CHANNEL) {
		radio_set_channel(RADIO_CHANNEL);
	}
	memset(apps, 0, sizeof (apps));
	initialized = true;
	PRINTF("GenericComm: init\r\n");
	return SUCCESS;
}

int GenericComm_register(uint8_t id, gc_callback_t callback)
{
	if (!initialized || !callback || id >= GC_MAX_APPS) {
		PRINTF("GenericComm: register app %u FAIL\r\n", id);
		return FAIL;
	}
	apps[id] = callback;
	PRINTF("GenericComm: register app %u\r\n", id);
	return SUCCESS;
}

int GenericComm_unregister(uint8_t id)
{
	if (!initialized || id >= GC_MAX_APPS) {
		PRINTF("GenericComm: unregister app %u FAIL\r\n", id);
		return FAIL;
	}
	apps[id] = NULL;
	PRINTF("GenericComm: unregister app %u\r\n", id);
	return SUCCESS;
}
extern bool TIM_busy;
int GenericComm_send(uint8_t id, uint16_t dest, const uint8_t *payload, uint8_t len, uint8_t ack)
{
	uint8_t buf[MAX_PAYLOAD_SIZE];
	gc_header_t *hdr = (gc_header_t *)buf;
	PRINTF("GenericComm: GenericComm_send\r\n");
	if (dest == BCAST_ADDR) {
		PRINTF("dest == BCAST_ADDR\r\n");
		return GenericComm_bcast(id, payload, len);
	}

	if (!initialized || !payload || !len || len > GC_MAX_PAYLOAD_SIZE) {
		PRINTF("GenericComm[%u]: send FAIL\r\n", id);
		return FAIL;
	}

#if 0
	if (ack) {
		// ack requested
	}
#endif
	hdr->id = id;
	hdr->flag = ack;
	memcpy(buf + GC_HEADER_SIZE, payload, len);
	
	if (radio_send(dest, buf, len + GC_HEADER_SIZE) != 0) {
		PRINTF("GenericComm[%u]: send error\r\n", id);
		//vTaskDelay(2);
		return FAIL;
	}
	
	PRINTF("GenericComm[%u]: send\r\n", id);
	//portYIELD();
	return SUCCESS;
}

int GenericComm_bcast(uint8_t id, const uint8_t *payload, uint8_t len)
{
	uint8_t buf[MAX_PAYLOAD_SIZE];
	gc_header_t *hdr = (gc_header_t *)buf;
	if (!initialized || !payload || !len || len > GC_MAX_PAYLOAD_SIZE) {
		PRINTF("GenericComm[%u]: bcast FAIL\r\n", id);
		return FAIL;
	}
	hdr->id = id;
	hdr->flag = 0;
	memcpy(buf + GC_HEADER_SIZE, payload, len);
	if (radio_send(BCAST_ADDR, buf, len + GC_HEADER_SIZE) != 0) {
		PRINTF("GenericComm[%u]: bcast error\r\n", id);
		//vTaskDelay(2);
		return FAIL;
	}
	PRINTF("GenericComm[%u]: bcast\r\n", id);
	//portYIELD();
	return SUCCESS;
}

#if 0
static int GenericComm_ack(uint8_t id, uint16_t dest)
{
	gc_header_t hdr;
	if (!initialized || dest == BCAST_ADDR) {
		PRINTF("GenericComm[%u]: ack FAIL\r\n", id);
		return FAIL;
	}
	hdr.id = id;
	hdr.flag = 1;
	if (radio_send(dest, &hdr, GC_HEADER_SIZE) != 0) {
		PRINTF("GenericComm[%u]: ack error\r\n", id);
		//vTaskDelay(2);
		return FAIL;
	}
	PRINTF("GenericComm[%u]: ack\r\n", id);
	//portYIELD();
	return SUCCESS;
}
#endif

static void gcRecvTask(void *pvParameters)
{
	uint16_t src;
	uint8_t len;
	uint8_t buf[MAX_PAYLOAD_SIZE];
	gc_header_t *hdr = (gc_header_t *)buf;
	// The parameters are not used.
	( void ) pvParameters;
	while (1) {
		if (radio_recv(&src, buf, &len) != SUCCESS || len < GC_HEADER_SIZE || hdr->id >= GC_MAX_APPS || !apps[hdr->id]) {
			PRINTF("GenericComm: recv error\r\n");
			//vTaskDelay(2); // Why doesn't this work? FIXME
			TIM_Waitms(2);
			//portYIELD();
			continue;
		}
#if 0
		if (len == GC_HEADER_SIZE && hdr->flag) {
			// ack received
		} else if (hdr->flag) {
			// ack requested
		} else {
			PRINTF("GenericComm[%u]: recv s=%03u l=%u ack=%u\r\n", hdr->id, src, len - GC_HEADER_SIZE, hdr->flag);
			apps[hdr->id](src, buf + GC_HEADER_SIZE, len - GC_HEADER_SIZE);
		}
#else
		PRINTF("GenericComm[%u]: recv s=%03u l=%u\r\n", hdr->id, src, len - GC_HEADER_SIZE);
#ifdef FRA //TUFIX: This part is for keeping track of what node received the time from the GW. This used to be here, then moved to timesync_exec but I think due to short time limit (500 ms), the timer always fires before this could be done, so I move it back here, alittle messy but at least it works
		//Determine if the id received is new or old
		for (i = 0; i < counter_ftp; i++) {
			if (src == targets2_ftp[i]) 
				detectoldid = true;
		}
		if (!detectoldid) {
			targets2_ftp[counter_ftp] = src;
			counter_ftp++;
		}
#endif		
		apps[hdr->id](src, buf + GC_HEADER_SIZE, len - GC_HEADER_SIZE);
#endif
	}
}
