#ifdef USB
#include <xnode.h>
#include "lpc43xx_scu.h"
#include "usb.h"
#include "usbcfg.h"
#include "usbhw.h"
#include "usbcore.h"
#include "cdc.h"
#include "cdcuser.h"
#include "usbuser.h"
#include <string.h>

#ifdef __ICCARM__
#pragma data_alignment=4
#define __align(x)
#elif defined   (  __GNUC__  )
#define __align(x) __attribute__((aligned(x)))
#endif

volatile uint32_t systicks = 0;
LPC_USBDRV_INIT_T usb_cb;

extern DQH_T ep_QH[EP_NUM_MAX];
extern DTD_T ep_TD[EP_NUM_MAX];
extern uint32_t DevStatusFS2HS;

extern void USB_EndPoint0 (uint32_t event);

void VCOM_Init(void)
{
  scu_pinmux(0x9 ,2 , MD_PDN, FUNC2); 	// GPIO4_14: LD11
  scu_pinmux(0xA ,4 , MD_PDN, FUNC2); 	// GPIO4_11: LD10
  scu_pinmux(0x6 ,10 , MD_PUP, FUNC3);	// GPIO3_6: button 0
  scu_pinmux(0x4 ,0 , MD_PUP, FUNC0); 	// GPIO2_0: button 1

  LPC_GPIO_PORT->DIR[4] |= (1<<14);			// LD11 = output
  LPC_GPIO_PORT->DIR[4] |= (1<<11);			// LD11 = output
  LPC_GPIO_PORT->DIR[3] &= ~(1<<6);			// Button 0 = input
  LPC_GPIO_PORT->DIR[2] &= ~(1<<0);			// Button 1 = input
	
  /* initilize call back structures */
  memset((void*)&usb_cb, 0, sizeof(LPC_USBDRV_INIT_T));
  usb_cb.USB_Reset_Event = USB_Reset_Event;
  usb_cb.USB_P_EP[0] = USB_EndPoint0;
  usb_cb.USB_P_EP[1] = USB_EndPoint1;
  usb_cb.USB_P_EP[2] = USB_EndPoint2;
  usb_cb.ep0_maxp = USB_MAX_PACKET0;

  CDC_Init(0);                              // VCOM Initialization
  USB_Init(&usb_cb);                        // USB Initialization
}

void VCOM_Connect(bool blocking)
{
  USB_Connect(TRUE);                        // USB Connect
	if (blocking) {
    while (!USB_Configuration);             // wait until USB is configured	
	}
}

void VCOM_Disconnect(void)
{
	USB_Connect(FALSE);
}

/*----------------------------------------------------------------------------
  checks the serial state and initiates notification
 *---------------------------------------------------------------------------*/
void VCOM_CheckSerialState (void)
{
  unsigned short temp;
  static unsigned short serialState;

  temp = CDC_GetSerialState();
  if (serialState != temp) {
    serialState = temp;
    CDC_NotificationIn();                  // send SERIAL_STATE notification
  }
}


/*----------------------------------------------------------------------------
  Write buffer to USB
 *---------------------------------------------------------------------------*/
void VCOM_Write(char *buf, int size)
{
  static char __align(4) serBuf [USB_CDC_BUFSIZE];
  int len;
  for (len = 0; size > 0; buf += len, size -= len) {
		len = size < USB_CDC_BUFSIZE ? size : USB_CDC_BUFSIZE;
		memcpy(serBuf, buf, len);
		while (!CDC_DepInEmpty) { }
		CDC_DepInEmpty = 0;
		USB_WriteEP (CDC_DEP_IN, (uint8_t *)&serBuf[0], len);
  }
}


/*----------------------------------------------------------------------------
  Read from USB to buffer
 *---------------------------------------------------------------------------*/
void VCOM_Read(char *buf, int size, int *read)
{
  static char __align(4) serBuf [USB_CDC_BUFSIZE];
  int  numBytesToRead, numBytesRead = 0, numAvailByte;

  VCOM_CheckSerialState();
  CDC_OutBufAvailChar (&numAvailByte);
  if (numAvailByte > 0) {
    numBytesToRead = numAvailByte > USB_CDC_BUFSIZE ? USB_CDC_BUFSIZE : numAvailByte;
    numBytesRead = CDC_RdOutBuf (&serBuf[0], &numBytesToRead);
  }
	*read = numBytesRead < size ? numBytesRead : size;
	memcpy(buf, serBuf, *read);
}

void VCOM_PutChar(unsigned char x)
{
	static char __align(4) serBuf [USB_CDC_BUFSIZE];
	serBuf[0] = x;
	while (!CDC_DepInEmpty) { }
	CDC_DepInEmpty = 0;
	USB_WriteEP(CDC_DEP_IN, (uint8_t *)&serBuf[0], 1);
}

unsigned char VCOM_GetChar(void)
{
  static char __align(4) serBuf[USB_CDC_BUFSIZE];
  int numBytesRead, numAvailByte = 1;

  VCOM_CheckSerialState();
  CDC_OutBufAvailChar(&numAvailByte);
  if (numAvailByte > 0) {
    numBytesRead = CDC_RdOutBuf(&serBuf[0], &numAvailByte);
    if (numBytesRead > 0) {
      return (unsigned char)serBuf[0];
    }
  }
  return 0;
}
#endif
