#include <xnode.h>
#include <stdlib.h>
#include <string.h>
#include "delay.h"
#include "rf233.h"
#include "rf233-config.h"
#include "rf233-const.h"
#include "Radio.h"
//#include "LocalTime.h"
//#include "GlobalTime.h"

#define FCF                    (0xAA61)
#define FRAME_SIZE             (128)
#define HEADER_SIZE            (9)
#define FOOTER_SIZE            (3)
#define PACKET_SIZE            (FRAME_SIZE - HEADER_SIZE - FOOTER_SIZE)
#define DEFAULT_ADDR           (0x0001)

uint16_t LOCAL_ADDR = DEFAULT_ADDR;
uint8_t RADIO_CHANNEL = 26;
uint8_t RADIO_POWER = TXP_4;
//extern uint64_t clock;
//extern int dosync;

static enum { OFF, BUSY, READY } state = OFF;

typedef struct {
  uint16_t fcf;
  uint8_t  seq;
  uint16_t pan;
  uint16_t dest;
  uint16_t src;
  char payload[];
} __attribute__((packed)) header_t;

static header_t *framehdr = NULL;
static uint8_t *framebuf = NULL;
//static TaskHandle_t xTaskToNotify = NULL;
static volatile uint8_t rx_flag = false;

// this is in interrupt context!
static void radio_callback(void)
{
	/*
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);
	xTaskToNotify = NULL;
	//portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	*/
	rx_flag = true;
}

int radio_init(void)
{
	if (rf233_init(radio_callback) != SUCCESS) {
		state = OFF;
		return FAIL;
	}
	framebuf = (uint8_t *)sdmalloc(FRAME_SIZE);
	configASSERT(framebuf);
	framehdr = (header_t *)framebuf;
	framehdr->fcf = FCF;
	framebuf[3] = IEEE802154_PANID & 0xFF;
	framebuf[4] = (IEEE802154_PANID >> 8) & 0xFF;
	if (radio_on() != SUCCESS) {
		state = OFF;
		return FAIL;
	}
	return SUCCESS;
}

int radio_reset(void)
{
	//return radio_init();
	return SUCCESS;
}
extern bool TIM_busy;
int radio_send(uint16_t dest, void *payload, uint8_t len)
{
	//uint8_t *test1; 
	//ts_msg *test2;
	int ret, r;
	if (len > PACKET_SIZE || rx_flag || state != READY) {
		return FAIL;
	}

	state = BUSY;
  // prepend MAC header
	framebuf[5] = dest & 0xFF;
	framebuf[6] = (dest >> 8) & 0xFF;
	framebuf[7] = LOCAL_ADDR & 0xFF;
	framebuf[8] = (LOCAL_ADDR >> 8) & 0xFF;
	memcpy(framehdr->payload, payload, len);

#if 0
	r = rand() & 0xFF + 10;
	while (!rf233_channel_clear()) {
	  delay_us(r);
		r *= 2;
		if (r > 2000) {
			state = READY;
			return FAIL;
		}
	}
#else
	for (r = 0; r < 7; ++r) {
		if (rf233_channel_clear()) {
			break;
		}
		delay_us(rand() & 0x7F + 10);
	}
#endif
	/*
	 if(dosync)
	 {
		 clock=GlobalTime_get64();
  	 memcpy((uint8_t*)(framehdr->payload)+2+12, &clock, sizeof(uint64_t));  // 2 is GC_HEADER_SIZE, 12 is the size of parameters before time64 in ts_msg.
	 }
	*/
	ret = rf233_send(framebuf, len + HEADER_SIZE);
	framehdr->seq++;
	state = READY;
	return ret;
}

int radio_recv(uint16_t *src, uint8_t *payload, uint8_t *len)
{
	uint16_t hfcf, hpan, hsrc, hdst;
  int framelen;
	if (state != READY || /*xTaskToNotify != NULL ||*/ !src || !payload || !len) {
		return FAIL;
	}
	/*
	xTaskToNotify = xTaskGetCurrentTaskHandle();
  if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) == 0) {
		xTaskToNotify = NULL;
		return FAIL;
	}
	*/
	while (!rx_flag) { portYIELD(); }
	state = BUSY;
	rx_flag = false;
	framelen = rf233_read(framebuf, FRAME_SIZE);
	if (framelen < HEADER_SIZE) {
		state = READY;
		return FAIL;
	}
	hfcf = framebuf[0] | (framebuf[1] << 8);
	hpan = framebuf[3] | (framebuf[4] << 8);
	hdst = framebuf[5] | (framebuf[6] << 8);
	hsrc = framebuf[7] | (framebuf[8] << 8);
	if (hfcf != FCF
	|| hpan != IEEE802154_PANID
	|| hsrc == 0 || hsrc == BCAST_ADDR
	|| (hdst != LOCAL_ADDR && hdst != BCAST_ADDR)) {
		state = READY;
		return FAIL;
	}
	*src = hsrc;
	*len = framelen - HEADER_SIZE;
	memcpy(payload, framebuf + HEADER_SIZE, *len);
	state = READY;
	/*
	if(dosync)
		clock = LocalTime_get64();
		//clock=TimeSync_GetSystemUs(); 
	*/
	return SUCCESS;
}

int radio_on(void)
{
	int ret = rf233_on();
	if (ret == SUCCESS) {
		state = READY;
	}
	return ret;
}

int radio_off(void)
{
	int ret = rf233_off();
	if (ret == SUCCESS) {
		state = OFF;
	}
	return ret;
}

int radio_sleep(void)
{
	int ret = rf233_sleep();
	if (ret == SUCCESS) {
		state = OFF;
	}
	return ret;
}

int radio_set_short_addr(uint16_t addr)
{
	if (!addr || addr == 0xffff) {
		return FAIL;
	}
	LOCAL_ADDR = addr;
	rf233_set_short_addr(addr);
	return SUCCESS;
}
int radio_get_channel(void)
{
	return rf233_get_channel();
}

int radio_set_channel(uint8_t ch)
{
	return rf233_set_channel(ch);
}

int radio_get_txp(void)
{
	return rf233_get_txp();
}

int radio_set_txp(uint8_t txp)
{
	return rf233_set_txp(txp);
}
