/*
 * Copyright (c) 2013, Thingsquare, http://www.thingsquare.com/.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
* Copyright (c) 2015 Atmel Corporation and 2012 – 2013, Thingsquare, http://www.thingsquare.com/. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of Atmel nor the name of Thingsquare nor the names of its contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* 4. This software may only be redistributed and used in connection with an
* Atmel microcontroller or Atmel wireless product.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
*
*/

#include <xnode.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "rf233.h"
#include "rf233-const.h"
#include "rf233-config.h"
#include "rf233-arch.h"
#include "trx_access.h"
#include "delay.h"
#include "LocalTime.h"
#include "GlobalTime.h"

#define rf233_hard_recovery()         /*die()*/rf233_init(rx_callback)
static int rf233_prepare(const void *payload, unsigned short payload_len);
static int rf233_transmit(unsigned short payload_len);
static void rf233_generate_random_seed(void);
static void rf233_flush_buffer(void);
static uint8_t rf233_status(void);
static void rf233_interrupt_poll(void);

#if NULLRDC_CONF_802154_AUTOACK_HW
static uint8_t ack_status = 0;
#endif
static uint8_t flag_transmit = 0;
static volatile int radio_is_on = 0;
static volatile int pending_frame = 0;
static volatile int sleep_on = 0;
static rf233_callback_t rx_callback = NULL;

extern uint64_t clock;
extern int dosync;

/*---------------------------------------------------------------------------*/
/* convenience macros */
#define rf233_command(c)                  trx_reg_write(RF233_REG_TRX_STATE, c)

/* each frame has a footer consisting of LQI, ED, RX_STATUS added by the radio */
#define FOOTER_LEN                        (3)   /* bytes */
#define MAX_PACKET_LEN                    (TRX_BUFFER_LENGTH - 1) /* bytes, excluding the length (first) byte */

#define RTIMER_SECOND 1000000
/* when transmitting, time to allow previous transmission to end before drop */
#define PREV_TX_TIMEOUT                   (10 * RTIMER_SECOND/1000)

/*
 * Energy level during reception, ranges from 0x00 to 0x53 (=83d) with a
 * resolution of 1dB and accuracy of +/- 5dB. 0xFF means invalid measurement.
 * 0x00 means <= RSSI(base_val), which is -91dBm (typ). See datasheet 12.7.
 * Ergo, real RSSI is (ed-91) dBm or less.
 */
#define RSSI_OFFSET                       (94)

#define _DEBUG_               0
#define DEBUG_PRINTDATA       0    /* print frames to/from the radio; requires DEBUG == 1 */
#if _DEBUG_
#define PRINTF(...)           lpc_printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
	do {                                                                  \
		uint32_t t0;                                                        \
		t0 = LPC_RITIMER->COUNTER;                                          \
		while(!(cond)) {                                                    \
			uint32_t t1 = LPC_RITIMER->COUNTER;                               \
			uint32_t diff = (t1 > t0) ? t1 - t0 : 0xFFFFFFFF - t0 + t1;       \
			diff /= 120;                                                      \
			if (diff > (max_time)) break;                                     \
		}                                                                   \
	} while(0)

/**
 * \brief      Get radio channel
 * \return     The radio channel
 */
int rf233_get_channel(void)
{
	uint8_t channel;
	channel = trx_reg_read(RF233_REG_PHY_CC_CCA) & PHY_CC_CCA_CHANNEL;
	return (int)channel;
}

/**
 * \brief      Set radio channel
 * \param ch   The radio channel
 * \retval -1  Fail: channel number out of bounds
 * \retval 0   Success
 */
int rf233_set_channel(uint8_t ch)
{
	uint8_t temp;
	PRINTF("RF233: setting channel %u\r\n", ch);
	if (ch > 26 || ch < 11) {
		return -1;
	}

	/* read-modify-write to conserve other settings */
	temp = trx_reg_read(RF233_REG_PHY_CC_CCA);
	temp &=~ PHY_CC_CCA_CHANNEL;
	temp |= ch;
	trx_reg_write(RF233_REG_PHY_CC_CCA, temp);
	return 0;
}

/**
 * \brief      Get transmission power
 * \return     The transmission power
 */
int rf233_get_txp(void)
{
	PRINTF("RF233: get txp\r\n");
	return trx_reg_read(RF233_REG_PHY_TX_PWR_CONF) & PHY_TX_PWR_TXP;
}

/**
 * \brief      Set transmission power
 * \param txp  The transmission power
 * \retval -1  Fail: transmission power out of bounds
 * \retval 0   Success
 */
int rf233_set_txp(uint8_t txp)
{
	PRINTF("RF233: setting txp %u\r\n", txp);
	if (txp > TXP_M17) {
		/* undefined */
		return -1;
	}

	trx_reg_write(RF233_REG_PHY_TX_PWR_CONF, txp);
	return 0;
}

// Section 9.8 of the RF233 manual suggests recalibrating filters at
// least every 5 minutes of operation. Transitioning out of sleep
// resets the filters automatically.
static void calibrate_filters(void)
{
  PRINTF("RF233: Calibrating filters.\r\n");
  trx_reg_write(RF233_REG_FTN_CTRL, 0x80);
	//while(trx_reg_read(RF233_REG_FTN_CTRL) & 0x80); 
	BUSYWAIT_UNTIL(!(trx_reg_read(RF233_REG_FTN_CTRL) & 0x80), RTIMER_SECOND/10);
}

/**
 * \brief      Init the radio
 * \return     Returns success/fail
 * \retval 0   Success
 */
int rf233_init(rf233_callback_t callback)
{
	uint8_t i;
	volatile uint8_t regtemp;
	volatile uint8_t radio_state;  /* don't optimize this away, it's important */
	PRINTF("RF233: init.\r\n");
	
	DISABLE_TRX_IRQ();

	/* init SPI and GPIOs, wake up from sleep/power up. */
	rf233_arch_init();
	trx_spi_init();

	/* reset will put us into TRX_OFF state */
	/* reset the radio core */
	rf233_arch_reset();

  // Read the PART_NUM register to verify that the radio is
  // working/responding. Could check in software, I just look at
  // the bus. If this is working, the first write should be 0x9C x00
  // and the return bytes should be 0x00 0x0B.
  regtemp = trx_reg_read(RF233_REG_PART_NUM);
  PRINTF("RF233: REG_PART_NUM = %0x\n\r", regtemp);
  regtemp = trx_reg_read(RF233_REG_VERSION_NUM);
  PRINTF("RF233: REG_VERSION_NUM = %0x\n\r", regtemp);

	/* before enabling interrupts, make sure we have cleared IRQ status */
	regtemp = trx_reg_read(RF233_REG_IRQ_STATUS);
	PRINTF("After wake from sleep\r\n");

	radio_state = rf233_status();
	PRINTF("After arch read reg: state 0x%04x\r\n", radio_state);

  calibrate_filters();

	if (radio_state == STATE_P_ON) {
		trx_reg_write(RF233_REG_TRX_STATE, TRXCMD_TRX_OFF);
	}

	/* Assign regtemp to regtemp to avoid compiler warnings */
	regtemp = regtemp;
	rx_callback = callback;
	trx_irq_init(rf233_interrupt_poll);
	ENABLE_TRX_IRQ();

	/* Configure the radio using the default values except these. */
	trx_reg_write(RF233_REG_TRX_CTRL_1,      RF233_REG_TRX_CTRL_1_CONF);
	trx_reg_write(RF233_REG_PHY_CC_CCA,      RF233_REG_PHY_CC_CCA_CONF);
	trx_reg_write(RF233_REG_PHY_TX_PWR,      RF233_REG_PHY_TX_PWR_CONF);
	trx_reg_write(RF233_REG_TRX_CTRL_2,      RF233_REG_TRX_CTRL_2_CONF);
	trx_reg_write(RF233_REG_IRQ_MASK,        RF233_REG_IRQ_MASK_CONF);
#if HW_CSMA_FRAME_RETRIES
	trx_bit_write(SR_MAX_FRAME_RETRIES, 3);
	trx_bit_write(SR_MAX_CSMA_RETRIES, 4);
#else
	trx_bit_write(SR_MAX_FRAME_RETRIES, 0);
	trx_bit_write(SR_MAX_CSMA_RETRIES, 7);
#endif
	rf233_set_pan_id(IEEE802154_PANID);

  // Configure the radio using the default values except these.
  //trx_reg_write(RF233_REG_XAH_CTRL_1,      0x02);
	//rf233_setchannel(26);
  {
    uint8_t addr[8];
    addr[0] = 0x22;
    addr[1] = 0x22;
    addr[2] = 0x22;
    addr[3] = 0x22;
    addr[4] = 0x22;
    addr[5] = 0x22;
    addr[6] = 0x22;
    addr[7] = 0x22;
    rf233_set_ieee_addr(addr);
    rf233_set_short_addr(LOCAL_ADDR);
  }
	
	rf233_generate_random_seed();

	for (i = 0; i < 8; i++) {
		regtemp = trx_reg_read(0x24 + i);
	}

	/* 11_09_rel */
	trx_reg_write(RF233_REG_TRX_RPC, 0xFF); /* Enable RPC feature by default */

	return 0;
}

/*
 * \brief Generates a 16-bit random number used as initial seed for srand()
 *
 */
static void rf233_generate_random_seed(void)
{
	uint8_t i;
	uint16_t seed = 0;
	uint8_t cur_random_val = 0;
	uint32_t j;

	/*
	 * We need to disable TRX IRQs while generating random values in RX_ON,
	 * we do not want to receive frames at this point of time at all.
	 */
	ENTER_TRX_REGION();

	j = 0;
	do {
		trx_reg_write(RF233_REG_TRX_STATE, TRXCMD_TRX_OFF);
		j++;
	} while (TRXCMD_TRX_OFF != rf233_status() && j < 10000);
	j = 0;
	do {
		/* Ensure that PLL has locked and receive mode is reached. */
		trx_reg_write(RF233_REG_TRX_STATE, TRXCMD_PLL_ON);
		j++;
	} while (TRXCMD_PLL_ON != rf233_status() && j < 10000);
	j = 0;
	do {
		trx_reg_write(RF233_REG_TRX_STATE, TRXCMD_RX_ON);
		j++;
	} while (TRXCMD_RX_ON != rf233_status() && j < 10000);

	/* Ensure that register bit RX_PDT_DIS is set to 0. */
	trx_bit_write(SR_RX_PDT_DIS, RX_ENABLE);

	/*
	 * The 16-bit random value is generated from various 2-bit random
	 * values.
	 */
	for (i = 0; i < 8; i++) {
		/* Now we can safely read the 2-bit random number. */
		cur_random_val = trx_bit_read(SR_RND_VALUE);
		seed = seed << 2;
		seed |= cur_random_val;
		delay_us(1); /* wait that the random value gets updated */
	}

	j = 0;
	do {
		/* Ensure that PLL has locked and receive mode is reached. */
		trx_reg_write(RF233_REG_TRX_STATE, TRXCMD_TRX_OFF);
		j++;
	} while (TRXCMD_TRX_OFF != rf233_status() && j < 10000);
	/*
	 * Now we need to clear potential pending TRX IRQs and
	 * enable the TRX IRQs again.
	 */
	trx_reg_read(RF233_REG_IRQ_STATUS);
	trx_irq_flag_clr();
	LEAVE_TRX_REGION();

	/* Set the seed for the random number generator. */
	srand(seed);
}

/**
 * \brief      prepare a frame and the radio for immediate transmission
 * \param payload         Pointer to data to copy/send
 * \param payload_len     length of data to copy
 * \return     Returns success/fail, refer to radio.h for explanation
 */
static int rf233_prepare(const void *payload, unsigned short payload_len)
{
	uint8_t templen;
	uint8_t radio_status;
	uint8_t data[TRX_BUFFER_LENGTH + 2];

	PRINTF("RF233: prepare %u\r\n", payload_len);
	if (payload_len > MAX_PACKET_LEN) {
		PRINTF("RF233: error, frame too large to tx\r\n");
		return RADIO_TX_ERR;
	}

#if USE_HW_FCS_CHECK
	/* Add length of the FCS (2 bytes) */
	templen = payload_len + 2;
#else   /* USE_HW_FCS_CHECK */
	/* FCS is assumed to already be included in the payload */
	templen = payload_len;
#endif  /* USE_HW_FCS_CHECK */
	data[0] = templen;
	memcpy(&data[1], payload, templen);
#if DEBUG_PRINTDATA
	{
		unsigned int i;
		uint8_t *pl = (uint8_t *)payload;
		PRINTF("RF233 prepare (%u/%u): 0x", payload_len, templen);
		for (i = 0; i < templen; i++) {
			PRINTF("%02x", *(pl + i));
		}
		PRINTF("\r\n");
	}
#endif  /* DEBUG_PRINTDATA */

	/* check that the FIFO is clear to access */
	radio_status = rf233_status();
#if NULLRDC_CONF_802154_AUTOACK_HW
	if (radio_status == STATE_BUSY_RX_AACK || radio_status == STATE_BUSY_TX_ARET) {
		PRINTF("RF233: TRX buffer unavailable: prep when %s\r\n", radio_status == STATE_BUSY_RX_AACK ? "rx" : "tx");
#else
	if (radio_status == STATE_BUSY_RX || radio_status == STATE_BUSY_TX) {
		PRINTF("RF233: TRX buffer unavailable: prep when %s\r\n", radio_status == STATE_BUSY_RX? "rx" : "tx");
#endif
		rf233_hard_recovery();
		return RADIO_TX_ERR;
	}

	/* Write packet to TX FIFO. */
	PRINTF("RF233 len = %u\r\n", payload_len);
	
	if(dosync)
	{
    clock=GlobalTime_get64();
		memcpy((uint8_t*)(&data[1]+23), &clock, sizeof(uint64_t));
	}
	
	trx_frame_write((uint8_t *)data, templen + 1);
	return RADIO_TX_OK;
}

/**
 * \brief      Transmit a frame already put in the radio with 'prepare'
 * \param payload_len    Length of the frame to send
 * \return     Returns success/fail, refer to radio.h for explanation
 */
static int rf233_transmit(unsigned short payload_len)
{
	static uint8_t status_now;
	PRINTF("RF233: tx %u\r\n", payload_len);

	/* prepare for TX */
	status_now = rf233_status();
#if NULLRDC_CONF_802154_AUTOACK_HW
	if (status_now == STATE_BUSY_RX_AACK || status_now == STATE_BUSY_TX_ARET) {
#else
	if (status_now == STATE_BUSY_RX || status_now == STATE_BUSY_TX) {
#endif
		PRINTF("RF233: collision, was receiving 0x%02X\r\n", status_now);
		/* NOTE: to avoid loops */
		//return RADIO_TX_COLLISION;
		return RADIO_TX_ERR;
	}
	if (status_now != STATE_PLL_ON) {
		uint32_t j = 0;
		/* prepare for going to state TX, should take max 80 us */
		trx_reg_write(RF233_REG_TRX_STATE, 0x09);
		do {
			status_now = trx_bit_read(0x01, 0x1F, 0);
			j++;
		} while (status_now == 0x1F && j < 100000);
	}

	if (rf233_status() != STATE_PLL_ON) {
		static uint8_t state;
		/* failed moving into PLL_ON state, gracefully try to recover */
		PRINTF("RF233: failed going to PLLON\r\n");
		rf233_command(TRXCMD_PLL_ON);   /* try again */
		state = rf233_status();
		if (state != STATE_PLL_ON) {
			/* give up and signal big fail (should perhaps reset radio core instead?) */
			PRINTF("RF233: graceful recovery (in tx) failed, giving up. State: 0x%02X\r\n", rf233_status());
			rf233_hard_recovery();
			return RADIO_TX_ERR;
		}
	}

	/* perform transmission */
#if NULLRDC_CONF_802154_AUTOACK_HW
	rf233_command(TRXCMD_TX_ARET_ON);
#endif
	rf233_command(TRXCMD_TX_START);
	flag_transmit = 1;

#if !NULLRDC_CONF_802154_AUTOACK_HW
	BUSYWAIT_UNTIL(rf233_status() == STATE_BUSY_TX, RTIMER_SECOND / 2000);
	BUSYWAIT_UNTIL(rf233_status() != STATE_BUSY_TX, PREV_TX_TIMEOUT);
#endif

#if !NULLRDC_CONF_802154_AUTOACK_HW
	if(rf233_status() != STATE_PLL_ON) {
		// something has failed
		PRINTF("RF233: radio fatal err after tx\r\n");
		rf233_hard_recovery();
		return RADIO_TX_ERR;
	}
	rf233_command(TRXCMD_RX_ON);
	PRINTF("RF233: tx ok\r\n");
	return RADIO_TX_OK;
#else
	BUSYWAIT_UNTIL(ack_status == 1, PREV_TX_TIMEOUT);
	if (ack_status) {
		ack_status = 0;
	  PRINTF("RF233: tx ok\r\n");
		return RADIO_TX_OK;
	} else {
		return RADIO_TX_NOACK;
	}
#endif
}

/**
 * \brief      Send data: first prepares, then transmits
 * \param payload         Pointer to data to copy/send
 * \param payload_len     length of data to copy
 * \return     Returns success/fail, refer to radio.h for explanation
 */
int rf233_send(const void *payload, unsigned short payload_len)
{
	PRINTF("RF233: send %u\r\n", payload_len);
	if (rf233_prepare(payload, payload_len) == RADIO_TX_ERR) {
		return RADIO_TX_ERR;
	}
	return rf233_transmit(payload_len);
}

/**
 * \brief      read a received frame out of the radio buffer
 * \param buf         pointer to where to copy received data
 * \param bufsize     Maximum size we can copy into bufsize
 * \return     Returns length of data read (> 0) if successful
 * \retval -1  Failed, was transmitting so FIFO is invalid
 * \retval -2  Failed, rx timed out (stuck in rx?)
 * \retval -3  Failed, too large frame for buffer
 * \retval -4  Failed, CRC/FCS failed (if USE_HW_FCS_CHECK is true)
 */
int rf233_read(void *buf, unsigned short bufsize)
{
	//uint8_t ed;       /* frame metadata */
	uint8_t frame_len = 0;
	uint8_t len = 0;
	//int rssi;
#if DEBUG_PRINTDATA
	uint8_t tempreadlen;
#endif  /* DEBUG_PRINTDATA */

	if (pending_frame == 0) {
		return 0;
	}
	DISABLE_TRX_IRQ();
	pending_frame = 0;

	/* get length of data in FIFO */
	trx_frame_read(&frame_len, 1);
#if DEBUG_PRINTDATA
	tempreadlen = frame_len;
#endif  /* DEBUG_PRINTDATA */

#if USE_HW_FCS_CHECK
	/* FCS has already been stripped */
	len = frame_len - 2;
	if (frame_len < 4) {
	  rf233_flush_buffer();
		ENABLE_TRX_IRQ();
		return 0;
	}
	if (!(trx_reg_read(RF233_REG_PHY_RSSI) & PHY_RSSI_CRC_VALID)) {
		rf233_flush_buffer();
		ENABLE_TRX_IRQ();
		return -4;
	}
#else
	len = frame_len;
	if (frame_len < 2) {
	  rf233_flush_buffer();
		ENABLE_TRX_IRQ();
		return 0;
	}
#endif  /* USE_HW_FCS_CHECK */

	if (len > bufsize) {
		/* too large frame for the buffer, drop */
		PRINTF("RF233: too large frame for buffer, dropping (%u > %u).\r\n", frame_len, bufsize);
		rf233_flush_buffer();
		ENABLE_TRX_IRQ();
		return -3;
	}
	PRINTF("RF233 read %u B\r\n", frame_len);

	/* read out the data into the buffer, disregarding the length and metadata bytes */
	trx_sram_read(1,(uint8_t *)buf, len);
#if DEBUG_PRINTDATA
	{
		int k;
		PRINTF("RF233: Read frame (%u/%u): ", tempreadlen, frame_len);
		for (k = 0; k < frame_len; k++) {
			PRINTF("%02x", *((uint8_t *)buf + k));
		}
		PRINTF("\r\n");
	}
#endif  /* DEBUG_PRINTDATA */

	//ed = trx_reg_read(RF233_REG_PHY_ED_LEVEL);
	//rssi = (int)ed - RSSI_OFFSET;
	//packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rssi);
	rf233_flush_buffer();
	ENABLE_TRX_IRQ();

	return len;
}

/**
 * \brief      perform a clear channel assessment
 * \retval >0  Channel is clear
 * \retval 0   Channel is not clear
 */
int rf233_channel_clear(void)
{
	uint8_t regsave;
	int was_off = 0;
	DISABLE_TRX_IRQ();
#if 0
	static uint8_t status_now;

	status_now = rf233_status();
#if NULLRDC_CONF_802154_AUTOACK_HW
	if (status_now == STATE_BUSY_RX_AACK || status_now == STATE_BUSY_TX_ARET) {
#else
	if (status_now == STATE_BUSY_RX || status_now == STATE_BUSY_TX) {
#endif
		ENABLE_TRX_IRQ();
		return 0;
	}
#endif

	if (rf233_status() != STATE_RX_ON) {
		/* CCA can only be performed in RX state */
		was_off = 1;
		rf233_command(TRXCMD_RX_ON);
	}
	delay_us(200);

	/* request a CCA, storing the channel number (set with the same reg) */
	regsave = trx_reg_read(RF233_REG_PHY_CC_CCA);
	regsave |= PHY_CC_CCA_DO_CCA | PHY_CC_CCA_MODE_CS_OR_ED;
	trx_reg_write(RF233_REG_PHY_CC_CCA, regsave);

	BUSYWAIT_UNTIL(trx_reg_read(RF233_REG_TRX_STATUS) & TRX_CCA_DONE,
		RTIMER_SECOND / 1000);
	regsave = trx_reg_read(RF233_REG_TRX_STATUS);
	/* return to previous state */
	if (was_off) {
		rf233_command(TRXCMD_TRX_OFF);
	}
#if NULLRDC_CONF_802154_AUTOACK_HW
	else {
		rf233_command(TRXCMD_RX_AACK_ON);
	}
#endif

	/* check CCA */
	if ((regsave & TRX_CCA_DONE) && (regsave & TRX_CCA_STATUS)) {
		PRINTF("RF233: CCA 1\r\n");
		ENABLE_TRX_IRQ();
		return 1;
	}
	PRINTF("RF233: CCA 0\r\n");
	ENABLE_TRX_IRQ();
	return 0;
}

/**
 * \brief      check whether we are currently receiving a frame
 * \retval >0  we are currently receiving a frame
 * \retval 0   we are not currently receiving a frame
 */
int rf233_receiving_packet(void)
{
	uint8_t trx_state;
	trx_state = rf233_status();

#if NULLRDC_CONF_802154_AUTOACK_HW
	if (trx_state == STATE_BUSY_RX_AACK) {
#else
	if (trx_state == STATE_BUSY_RX) {
#endif
		PRINTF("RF233: Receiving frame\r\n");
		return 1;
	}
	PRINTF("RF233: not Receiving frame\r\n");
	return 0;
}

/**
 * \brief      check whether we have a frame awaiting processing
 * \retval >0  we have a frame awaiting processing
 * \retval 0   we have not a frame awaiting processing
 */
int rf233_pending_packet(void)
{
	//PRINTF("RF233: Frame %spending\r\n", pending_frame ? "" : "not ");
	return pending_frame;
}

/**
 * \brief      switch the radio on to listen (rx) mode
 * \retval 0   Success
 */
int rf233_on(void)
{
	uint8_t state_now;
	PRINTF("RF233: on\r\n");

	/* Check whether radio is in sleep */
	if (sleep_on) {
		/* Wake the radio. It'll move to TRX_OFF state */
		rf233_arch_wake();
		delay_ms(1);
		sleep_on = 0;
	}
	state_now = rf233_status();
	if (state_now != STATE_PLL_ON && state_now != STATE_TRX_OFF
#if NULLRDC_CONF_802154_AUTOACK_HW
	&& state_now != STATE_TX_ARET_ON
#endif
	) {
		/* fail, we need the radio transceiver to be in either of those states */
		return -1;
	}

	/* go to RX_ON state */
#if NULLRDC_CONF_802154_AUTOACK_HW
	rf233_command(TRXCMD_RX_AACK_ON);
#else
	rf233_command(TRXCMD_RX_ON);
#endif
	radio_is_on = 1;
	return 0;
}

/**
 * \brief      switch the radio off
 * \retval 0   Success
 */
int rf233_off(void)
{
	PRINTF("RF233: off\r\n");
#if NULLRDC_CONF_802154_AUTOACK_HW
	if (rf233_status() != STATE_RX_AACK_ON ) {
#else
	if (rf233_status() != STATE_RX_ON) {
#endif
		/* fail, we need the radio transceiver to be in this state */
		return -1;
  }

	/* turn off the radio transceiver */
	rf233_command(TRXCMD_TRX_OFF);
	radio_is_on = 0;
	return 0;
}

void rf233_set_ieee_addr(uint8_t *ieee_addr)
{
	uint8_t *ptr_to_reg = ieee_addr;
	trx_reg_write((0x2b), *ptr_to_reg);
	ptr_to_reg++;
	trx_reg_write((0x2a), *ptr_to_reg);
	ptr_to_reg++;
	trx_reg_write((0x29), *ptr_to_reg);
	ptr_to_reg++;
	trx_reg_write((0x28), *ptr_to_reg);
	ptr_to_reg++;
	trx_reg_write((0x27), *ptr_to_reg);
	ptr_to_reg++;
	trx_reg_write((0x26), *ptr_to_reg);
	ptr_to_reg++;
	trx_reg_write((0x25), *ptr_to_reg);
	ptr_to_reg++;
	trx_reg_write((0x24), *ptr_to_reg);
	ptr_to_reg++;
}

void rf233_set_pan_id(uint16_t pan_id)
{
	uint8_t *d = (uint8_t *)&pan_id;

	trx_reg_write(0x22, d[0]);
	trx_reg_write(0x23, d[1]);
}

void rf233_set_short_addr(uint16_t addr)
{
	uint8_t *d = (uint8_t *)&addr;

	trx_reg_write(0x20, d[0]);
	trx_reg_write(0x21, d[1]);
	trx_reg_write(0x2d, d[0] + d[1]);
}

/* Put the Radio in sleep mode */
int rf233_sleep(void)
{
	int status;
	/* Check whether we're already sleeping */
	if (!sleep_on) {
		sleep_on = 1;
		/* Turn off the Radio */
		status = rf233_off();
		/* Set the SLP_PIN to high */
		if (status == 0) {
			rf233_arch_sleep();
		}
	}
	return 0;
}

/* used for indicating that the interrupt wasn't able to read SPI and must be serviced */
static volatile int interrupt_callback_wants_poll = 0;
/* used as a blocking semaphore to indicate that we are currently servicing an interrupt */
static volatile int interrupt_callback_in_progress = 0;

/**
 * \brief      Radio RF233 process, infinitely awaits a poll, then checks radio
 *             state and handles received data.
 */
//PROCESS_THREAD(rf233_radio_process, ev, data)
/*
void rf233_radio_process(void)
{
	int len;
	PRINTF("RF233: started.\r\n");

	while (1) {
		//PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
		__WFI();
		PRINTF("RF233: polled.\r\n");
		
		if (interrupt_callback_wants_poll) {
			rf233_interrupt_poll();
		}

		//packetbuf_clear();
		//packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);
		len = rf233_read(packetbuf_dataptr(), PACKETBUF_SIZE);
		if (len > 0) {
			packetbuf_set_datalen(len);
			//NETSTACK_RDC.input();
		} else {
			PRINTF("RF233: error while reading: %d\r\n", len);
		}
	}
}
*/

/**
 * \brief      RF233 radio process poll function, to be called from the
 *             interrupt handler to poll the radio process
 * \retval 0   success
 */
static void rf233_interrupt_poll(void)
{
	volatile uint8_t irq_source;
	/* handle IRQ source (for what IRQs are enabled, see rf233-config.h) */
	//irq_source = trx_reg_read_isr(RF233_REG_IRQ_STATUS);
	irq_source = trx_reg_read(RF233_REG_IRQ_STATUS);
	if (irq_source & IRQ_TRX_DONE) {
		if (flag_transmit == 1) {
			flag_transmit = 0;
			interrupt_callback_in_progress = 0;
#if NULLRDC_CONF_802154_AUTOACK_HW
			//if (!(trx_reg_read_isr(RF233_REG_TRX_STATE) & TRX_STATE_TRAC_STATUS)) {
			if (!(trx_reg_read(RF233_REG_TRX_STATE) & TRX_STATE_TRAC_STATUS)) {
				ack_status = 1;
			}
			//trx_reg_write_isr(RF233_REG_TRX_STATE, TRXCMD_RX_AACK_ON);
			trx_reg_write(RF233_REG_TRX_STATE, TRXCMD_RX_AACK_ON);
#endif
			return;
		}

		if (interrupt_callback_in_progress) {
			/* we cannot read out info from radio now, return here later (through a poll) */
			interrupt_callback_wants_poll = 1;
			//process_poll(&rf233_radio_process);
			//PRINTF("RF233: irq but busy, returns later.\r\n");
			return;
		}

		interrupt_callback_wants_poll = 0;
		interrupt_callback_in_progress = 1;

		DISABLE_TRX_IRQ();
		/* we have started receiving a frame, len can be read */
		pending_frame = 1;
		//process_poll(&rf233_radio_process);
		if (rx_callback) {
			
			if(dosync)
				clock = LocalTime_get64();
			
			rx_callback();
		}
		ENABLE_TRX_IRQ();
	}

#if 0
	/* Note, these are not currently in use but here for completeness. */
	if (irq_source & IRQ_TRX_DONE) {
		/* End of transmitted or received frame.  */
	}
	if (irq_source & IRQ_TRXBUF_ACCESS_VIOLATION) {
		/*
		 * Access violation on the shared TX/RX FIFO. Possible causes:
		 *  - buffer underrun while transmitting, ie not enough data in FIFO to tx
		 *  - reading too fast from FIFO during rx, ie not enough data received yet
		 *  - haven't read last rxed frame when next rx starts, but first is possible
		 *    to read out, with possible corruption - check FCS
		 *  - writing frames larger than 127 B to FIFO (len byte)
		 */
		//PRINTF("RF233-arch: access violation.\r\n");
	}
	if (irq_source & IRQ_BAT_LOW) {
		/* Battery low */
	}
	if (irq_source & IRQ_RX_ADDRESS_MATCH) {
		/* receiving frame address match */
	}
	if (irq_source & IRQ_CCA_ED_DONE) {
		/* CCA/ED done */
	}
	if (irq_source & IRQ_PLL_UNLOCK) {
		/* PLL unlock */
	}
	if (irq_source & IRQ_PLL_LOCK) {
		/* PLL lock */
	}
#endif

	interrupt_callback_in_progress = 0;
	return;
}

/*
 * Crude way of flushing the Tx/Rx FIFO: write the first byte as 0, indicating
 * a zero-length frame in the buffer. This is interpreted by the driver as an
 * empty buffer.
 */
static void rf233_flush_buffer(void)
{
	/* NB: tentative untested implementation */
	uint8_t temp = 0;
	trx_frame_write(&temp, 1);
}

static uint8_t rf233_status()
{
	return trx_reg_read(RF233_REG_TRX_STATUS) & TRX_STATUS;
}
