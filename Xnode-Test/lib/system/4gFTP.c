#ifdef FRA // All 4G functions go into here
#include <xnode.h>
#include <lpc43xx_cgu.h>
#include <lpc43xx_scu.h>
#include <lpc43xx_sdif.h>
#include <lpc43xx_sdmmc.h>
#include <lpc_sdmmc.h>
#include <nandflash_k9f1g08u0a.h>
#include <stdio.h>
#include <string.h>
#include <ff.h>
#include <rtc.h>
#include <timers.h>
#include <lpc43xx_timer.h>
#include <math.h>
#include <queue.h>
#include <ads131.h>
#include <ads131_const.h>
#include <filter.h>
#include <SnoozeAlarm.h>
#include "Sensing.h"
#include "RemoteSensing.h"
#include "TriggerSensing.h"
#include <led.h>
#include <vcom.h>
#include <stdlib.h>
#include "lpc43xx_uart.h"
#include "4gFTP.h"
#include "yaffsHEADER.h"
#include "sdcard.h"
#include "MQTTPacket.h"
#include "time.h"

#define _DEBUG_               1
#if _DEBUG_
#define PRINTF(...)           lpc_printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
#define LINELEN               180

static void ATTimer(TimerHandle_t pxTimer);
static void TCP_Rev_Timer(TimerHandle_t pxTimer);
static void OffTimer(TimerHandle_t pxTimer);
static int checkATmessageReply(bool timerrepeat, uint8_t *message, uint8_t size, bool iswait, char *term, uint8_t termlen);
static int checkATmessageReply_GetNumber(bool timerrepeat, uint8_t *message, uint8_t size, bool iswait, char *term, uint8_t termlen,uint64_t* cellPhoneNumber);

extern float voltage;
extern float current;
extern uint8_t *data_8bittype; // for NAND

// UART Config
// UART Configuration structure variable
UART_CFG_Type UARTConfigStruct;
// UART FIFO configuration Struct variable
UART_FIFO_CFG_Type UARTFIFOConfigStruct;

bool sendingFTP = false;
extern uint16_t ftps1, ftps2, ftps3, ftps4;
extern uint64_t pnumber;
extern char ftpun[30], ftpp[30], ftpa[30];

volatile bool ok=false,DataSendBackFTPDone = true;
volatile bool notrestart;
extern uint32_t data_index;
static char line[180];
extern bool timetodeploy;
extern uint16_t *uaddrs;
extern uint8_t ulen, uidx;

extern uint8_t tindex_ftp, tcount_ftp, targets2_ftp[MAX_NODES], counter_ftp;

static TimerHandle_t timoff, timoffFTP;
extern FATFS Fatfs;
extern Clock_info *cin_r;
TimerHandle_t timAT, timoffReport;

int trem = 0, tlen = 0, tdone = 0;

uint32_t m;
static void ATTimer(TimerHandle_t pxTimer)
{
	notrestart = false;
//	lpc_printf("ATTimer fires\r\n");
	m = 0;
}

volatile bool timeout = false; // timeout for tcprev in tcpstart
static void TCP_Rev_Timer(TimerHandle_t pxTimer)
{
	timeout = true;
}

void UARTOn(int br)
{
	scu_pinmux(0xE ,11 , MD_PDN, FUNC2); 	// P3_4 : UART1_TXD
	scu_pinmux(0xE ,12 , MD_PLN|MD_EZI|MD_ZI, FUNC2); 	// P3_5 : UART01_RXD
	UART_ConfigStructInit(&UARTConfigStruct, br);
	UART_Init(TEST_UART, &UARTConfigStruct);
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
	UART_FIFOConfig(TEST_UART, &UARTFIFOConfigStruct);

	// Enable UART Transmit
	UART_TxCmd(TEST_UART, ENABLE);
}

void UARTBaudChange(int newbr)
{
	newbr = 115200;
	
  UART_DeInit(TEST_UART);
	UART_TxCmd(TEST_UART, DISABLE);
	UART_ConfigStructInit(&UARTConfigStruct, newbr);
  UART_Init(TEST_UART, &UARTConfigStruct);
  UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
  UART_FIFOConfig(TEST_UART, &UARTFIFOConfigStruct);
  UART_TxCmd(TEST_UART, ENABLE);	
}

void CellOn(void)
{
	// Turn on 4G module
	scu_pinmux(PD7_CONF);	
	GPIO_SetDir(PD7_DIR);
	PD7_HIGH();	
	TIM_Waitms(200);
	scu_pinmux(PD5_CONF);
	GPIO_SetDir(PD5_DIR);
	PD5_LOW();	
	TIM_Waitms(200);
	PD5_HIGH();	
}

void ATCommandCheck(void)
{
	uint8_t AT_ATCommandCheck[] = "AT\r\n";
	lpc_printf("Checking basic AT command...");	// buying sometime for starting up
	TIM_Waitms(100);
	while(!checkATmessageReply(1,AT_ATCommandCheck,sizeof(AT_ATCommandCheck),0,"OK",2)) {};	
	TIM_Waitms(100);
	lpc_printf("Done\n\r");
}

void ATHarwareFlowControl(void)
{
	uint8_t AT_ATHarwareFlowControl[] = "AT&K3\r\n";
	lpc_printf("Activating hardware flow control...");	
	TIM_Waitms(100);
	while(!checkATmessageReply(1,AT_ATHarwareFlowControl,sizeof(AT_ATHarwareFlowControl),0,"OK",2)) {};	
	TIM_Waitms(100);
	lpc_printf("Done\n\r");
}

void ATEchoCancel(void)
{
	uint8_t AT_ATEchoCancel[] = "ATE0\r\n";
	lpc_printf("Cancelling command echo...");
	TIM_Waitms(100);
	while(!checkATmessageReply(1,AT_ATEchoCancel,sizeof(AT_ATEchoCancel),0,"OK",2)) {};	
	TIM_Waitms(100);
	lpc_printf("Done\n\r");
}

void ATEOFCheck(void)
{
	lpc_printf("Waiting to finish...\n\r");
	while(!checkATmessageReply(0,NULL,0,0,"OK",2)){};
	lpc_printf("Done\n\r");
}

int ATTCPDataGet(void) // similar to checkATmessageReply
{
	uint32_t len, i;
	uint8_t buffer[1], counter=0;
	uint8_t term[] = "SRING: 1,"; 
	int termlen = 9;
	memset(buffer,0,1);
	timAT = xTimerCreate("RSTimer", pdMS_TO_TICKS(10000), pdFALSE, NULL, ATTimer);
	configASSERT(timAT);
	xTimerStart(timAT, portMAX_DELAY);
	notrestart = true;
	while(notrestart)
	{
		len = UART_Receive(TEST_UART, &buffer[0], 1, NONE_BLOCKING);// Use BLOCKING cause the stuck in exiting condition
		if (len > 0)
		{
			if (buffer[0] == 10) // NL, to prevent combining multiple lines to a word 
			{
				i = 0;
			}
			//lpc_printf("%d\t%c\n\r",buffer[0],buffer[0]);		//tthis way works better than strstr but notrestart = false; must be commented out
			if (i<termlen)
			{
				if (buffer[0]==term[i])
				{ 
					i = i+1;
				}
			} 
			else 
			{
				if (i == termlen)
				{
					if (buffer[0] != ',')
					{
						counter = counter*10 + ((int) (buffer[0] - '0'));// to convert to int and not ascii
					}
					else
					{
						return counter;
					}
				}
			}
		}
	}
	//lpc_printf("\n\r");
	xTimerStop(timAT, portMAX_DELAY);
	xTimerDelete(timAT, portMAX_DELAY);
	return 0;
}

void ATSMSSetup1(void)
{
}

void ATSMSSetup2(void)
{
}

void ATSMSSend_FTPACK(void)
{
}

void ATSwitchOff(void)
{
	uint8_t AT_ATSwitchOff[] = "AT#SHDN\r";
	lpc_printf("Switching off...");
	TIM_Waitms(100);
	while(!checkATmessageReply(1,AT_ATSwitchOff,sizeof(AT_ATSwitchOff),1,"OK",2)){};	
	TIM_Waitms(100);
	TIM_Waitms(1000); lpc_printf("Done\n\r");
}

void ATBaudChange(int newbr)
{
	uint8_t i=0;
	char AT_ATBaudChange[80];
	newbr = 115200;
	lpc_printf("Setting baud rate to %d...",newbr);
	snprintf(AT_ATBaudChange,80,"AT+IPR=%d,115200\r",newbr);
	TIM_Waitms(100);
	for (i=0;i<5;i=i+1){
		UART_Send(TEST_UART,(uint8_t *)  AT_ATBaudChange, sizeof(AT_ATBaudChange), BLOCKING, 0); 	TIM_Waitms(1000);
	}
	TIM_Waitms(100);
	lpc_printf("Done\n\r");
}

void ATConnConfig(void)
{
	uint8_t AT_ATConnConfig[80];
	uint8_t AT_ATActSocket[] = "AT#SGACT=1,1\r";
	uint8_t AT_ATConfSocket[] = "AT#SCFG=1,1,1500,0,600,50\r";	
	uint8_t AT_ATConfSocketExt[] = "AT#SCFGEXT=1,2,1,0\r";
	uint8_t AT_ATActUser[80];
	uint8_t AT_ATActPass[80];
	uint8_t AT_ATPDPAuth[80];

	snprintf((char *)AT_ATConnConfig,80,"AT+CGDCONT=1,\"IPV4V6\",\"%s\"\r",ftpa);
	snprintf((char *)AT_ATActUser,80,"AT#USERID=%s\r",ftpun);
	snprintf((char *)AT_ATActPass,80,"AT#PASSW=%s\r",ftpp);
	snprintf((char *)AT_ATPDPAuth,80,"AT#PDPAUTH=1,1,\"%s\",\"%s\"\r",ftpun,ftpp);

	/*
	lpc_printf("AT+GMI\r\n");
	while(!checkATmessageReply(0,"AT+GMI\r",7,0,"OK",2)){};	TIM_Waitms(100);
	
	lpc_printf("AT+GMM\r\n");
	while(!checkATmessageReply(0,"AT+GMM\r",7,0,"OK",2)){};	TIM_Waitms(100);

	lpc_printf("AT+GMR\r\n");
	while(!checkATmessageReply(0,"AT+GMR\r",7,0,"OK",2)){};	TIM_Waitms(100);
  */
	
	lpc_printf("AT+CMEE=2\r\n");
	while(!checkATmessageReply(0,"AT+CMEE=2\r",10,0,"OK",2)){};	TIM_Waitms(100);

	lpc_printf("AT+CGREG?\r\n");	
	while(!checkATmessageReply(0,"AT+CGREG?\r",10,0,"OK",2)){};	TIM_Waitms(100);
	
	lpc_printf("AT+CGDCONT?\r\n");
	while(!checkATmessageReply(0,"AT+CGDCONT?\r",12,0,"OK",2)){};	TIM_Waitms(100);

	lpc_printf("AT+CGATT?\r\n");
	while(!checkATmessageReply(0,"AT+CGATT?\r",10,0,"OK",2)){};	TIM_Waitms(100);

	lpc_printf("AT+CGACT?\r\n");
	while(!checkATmessageReply(0,"AT+CGACT?\r",10,0,"OK",2)){};	TIM_Waitms(100);
		
	lpc_printf("AT+CGCONTRDP=1\r\n");
	while(!checkATmessageReply(0,"AT+CGCONTRDP=1\r",15,0,"OK",2)){};	TIM_Waitms(100);
		
	lpc_printf("AT#SGACT=?\r\n");
	while(!checkATmessageReply(0,"AT#SGACT=?\r",11,0,"OK",2)){};	TIM_Waitms(100);

	lpc_printf("AT#CGPADDR=1\r\n");
	while(!checkATmessageReply(0,"AT#CGPADDR=1\r",13,0,"OK",2)){};	TIM_Waitms(100);

	/*
	lpc_printf("AT#PDPAUTH?\r\n");
	while(!checkATmessageReply(0,"AT#PDPAUTH?\r",12,0,"OK",2)){};	TIM_Waitms(100);
		
	lpc_printf((char *)AT_ATPDPAuth);
	while(!checkATmessageReply(0,AT_ATPDPAuth,sizeof(AT_ATPDPAuth),0,"OK",2)){};	TIM_Waitms(100);

	lpc_printf("AT+CGATT=0\r\n");
	while(!checkATmessageReply(0,"AT+CGATT=0\r",11,0,"OK",2)){};	TIM_Waitms(100);
  */

	lpc_printf("AT+CGATT=1\r\n");
	while(!checkATmessageReply(0,"AT+CGATT=1\r",11,0,"OK",2)){};	TIM_Waitms(100);

	lpc_printf("AT+CGCONTRDP=1\r\n");
	while(!checkATmessageReply(0,"AT+CGCONTRDP=1\r",15,0,"OK",2)){};	TIM_Waitms(100);

  /*
	lpc_printf("Configure user...");	
	TIM_Waitms(100);
	while(!checkATmessageReply(1,AT_ATActUser,sizeof(AT_ATActUser),0,"OK",2)) {};	
	TIM_Waitms(100);  lpc_printf("Done\n\r");
		
	lpc_printf("Configure password...");	
	TIM_Waitms(100);
	while(!checkATmessageReply(1,AT_ATActPass,sizeof(AT_ATActPass),0,"OK",2)) {};	
	TIM_Waitms(100);  lpc_printf("Done\n\r");	
  */

	lpc_printf("Configure Internet connection...");	
	TIM_Waitms(100);
	while(!checkATmessageReply(1,AT_ATConnConfig,sizeof(AT_ATConnConfig),0,"OK",2)) {};	
	TIM_Waitms(100);  lpc_printf("Done\n\r");

	lpc_printf("Configure socket...");	
	TIM_Waitms(100);
	while(!checkATmessageReply(1,AT_ATConfSocket,sizeof(AT_ATConfSocket),0,"OK",2)) {};	
	TIM_Waitms(100);  lpc_printf("Done\n\r");

	lpc_printf("Configure socket extended...");	
	TIM_Waitms(100);
	while(!checkATmessageReply(1,AT_ATConfSocketExt,sizeof(AT_ATConfSocketExt),0,"OK",2)) {};	
	TIM_Waitms(100);  lpc_printf("Done\n\r");

	lpc_printf("Activate socket...");	
	TIM_Waitms(100);
	while(!checkATmessageReply(1,AT_ATActSocket,sizeof(AT_ATActSocket),0,"#SGACT: ",8)) {};
	TIM_Waitms(100);  lpc_printf("Done\n\r");

	ATCommandCheck();
}


void ATFTPConfig(void)
{
}

void ATFTPSend(void)
{
}

void ATFTPReceive(void)
{
}

void ATFTPClose(void)
{
}
extern uint8_t channel_4G;

uint64_t ATNumGet(void)
{
//	//uint8_t AT_ATNumGet[] = "AT+CNUM\r";
//	uint64_t cellPhoneNumber = 0;
//	lpc_printf("Getting phone number: ");
//	cellPhoneNumber = /*channel_4G*/ /*kam*/ LOCAL_ADDR;
///*
//	TIM_Waitms(100);
//	while(!checkATmessageReply_GetNumber(1,AT_ATNumGet,sizeof(AT_ATNumGet),1,"OK",2,&cellPhoneNumber)) {};	
//	TIM_Waitms(100);
//*/
//		lpc_printf("%llu\n\r",cellPhoneNumber);
//	return cellPhoneNumber;
	

	uint64_t cellPhoneNumber = 0;
	lpc_printf("Getting phone number: ");
	cellPhoneNumber = /*channel_4G*/ /*kam*/ pnumber;

	lpc_printf("%llu\n\r",cellPhoneNumber);
	return cellPhoneNumber;
	
}

void ATTimeGet()
{
	uint8_t AT_ATTimeGet[] = "AT+CCLK?\r";	
  uint8_t AT_ATFTPSend[] = "AT+KFTPSND=1,,\"test\",\"TESTyymmddhhmmss.txt\",0,0\r"; 
	TimerHandle_t tim;
	uint32_t len, i, time;
	uint8_t buffer[1];
	uint32_t networktime[12];
	Clock_info *cin=sdcalloc(1,sizeof(Clock_info));
	configASSERT(cin);
	lpc_printf("Checking and update time...\r\n");
	TIM_Waitms(3000);
	for (i=0;i<100;i++) // add this part, seems to help with the wrong date issue
	{
		len = UART_Receive(TEST_UART, buffer, sizeof(buffer), NONE_BLOCKING);
	}
	tim = xTimerCreate("RSTimer", pdMS_TO_TICKS(5000), pdFALSE, NULL, ATTimer);
	configASSERT(tim);
	xTimerStart(tim, portMAX_DELAY);
	
	i = 0;
	UART_Send(TEST_UART, AT_ATTimeGet, sizeof(AT_ATTimeGet), BLOCKING, 0); 
	while (i<14)
	{
		len = 0;
		while(len==0){
			len = UART_Receive(TEST_UART, buffer, sizeof(buffer), NONE_BLOCKING);
			if (len > 0){		
				lpc_printf("%c",buffer[0]);
				if ((buffer[0]>47) & (buffer[0]<58)){
					if (i<12)
					{
						AT_ATFTPSend[i+26] = buffer[0];
						networktime[i] = (int) (buffer[0]-48);
					}
					i = i+1;
				}
			}
		}
		time = networktime[1]*1 + networktime[0]*10;
	}
	if (time > 18)	// instead of reset (and keep resetting) if wrong time due to no network, treat it the other way around, only set clock if time is correct, if time is not correct, use DS3231m time instead (which should be very close too)
	{
		//Adjust clock based on network time - TUFIXDONE -  NEED CHECKING FOR CLOCK. Nothing to fix here. 
		cin->second=(int) networktime[10]*10 + networktime[11];//	lpc_printf("cin->second = %d\n\r",cin->second);
		cin->minute=(int) networktime[8]*10 + networktime[9];//	lpc_printf("cin->minute = %d\n\r",cin->second);
		cin->hour=(int) networktime[6]*10 + networktime[7];//	lpc_printf("cin->hour = %d\n\r",cin->second);
		cin->AMPM=FALSE;
		cin->day=7;
		cin->date=(int) networktime[4]*10 + networktime[5];//	lpc_printf("cin->date = %d\n\r",cin->second);
		cin->month=(int) networktime[2]*10 + networktime[3];//	lpc_printf("cin->month = %d\n\r",cin->second);
		cin->year=(int) networktime[0]*10 + networktime[1];//	lpc_printf("cin->year = %d\n\r",cin->second);
		cin->clock24=TRUE;

		RTCS_Init();	
		RTC_SetClock(cin);
		xTimerStop(tim, portMAX_DELAY);
		xTimerDelete(tim, portMAX_DELAY);
	}
	lpc_printf("\n\r");
}

void ATTCPPacketSize(int ps)
{
	char AT_ATTCPPacketSize[80];
	lpc_printf("Setting up packet size for TCP (%d)...",ps);	
	snprintf(AT_ATTCPPacketSize,80,"AT#TCPMAXDAT=%d\r",ps);
	while(!checkATmessageReply(0,(uint8_t *) AT_ATTCPPacketSize,sizeof(AT_ATTCPPacketSize),0,"OK",2)){};	
	TIM_Waitms(1000); lpc_printf("Done\n\r");
}

bool flag = true;
void ATTCPConnect(void)
{
	char AT_ATTCPConnect[80];
	snprintf(AT_ATTCPConnect,80,"AT#SD=1,0,1883,\"%d.%d.%d.%d\",0,0,1\r",ftps1,ftps2,ftps3,ftps4);
	//snprintf(AT_ATTCPConnect,80,"AT#SD=1,0,7,\"%d.%d.%d.%d\",0,0,1\r",ftps1,ftps2,ftps3,ftps4);
	lpc_printf("Connecting to TCP server...");	TIM_Waitms(100);	
	while(!checkATmessageReply(0,(uint8_t *)AT_ATTCPConnect,strlen(AT_ATTCPConnect),0,"OK",2)){};	TIM_Waitms(100);	
	TIM_Waitms(1000); lpc_printf("Done\n\r");
}

void ATTCPSend(int len)
{
	char AT_ATTCPSend[80];
	snprintf(AT_ATTCPSend,80,"AT#SSENDEXT=1,%d\r",len);
	lpc_printf("Starting to send %d bytes...\r\n", len);	TIM_Waitms(100);
	while(!checkATmessageReply(0,(uint8_t *) AT_ATTCPSend, strlen(AT_ATTCPSend),0,"> ",2)){}; TIM_Waitms(100);
	lpc_printf("Done\n\r");
}

bool ATTCPReceive(int len, char* revMesg, int* revMesglen)
{
	TimerHandle_t tim;
	int i = 0;
	int tmp, tmplen = len;
  char buffer[3];
	*revMesglen = 0;
	if (len > 0)	// if len = 0, we only listen (for ktcpstart command), not sending anything
	{
		lpc_printf("Listening to incoming TCP messages (expecting %d bytes)...\n\r", len);
		for(i = 0; i < tmplen; ++i)
		{
			do {
				len = UART_Receive(TEST_UART, (uint8_t *)buffer, 1, NONE_BLOCKING);// Use BLOCKING cause the stuck in exiting condition
			} while (len < 1);
			do {
				len = UART_Receive(TEST_UART,((uint8_t *) buffer)+1, 1, NONE_BLOCKING);// Use BLOCKING cause the stuck in exiting condition
			} while (len < 1);
			buffer[2] = '\0';
			sscanf(buffer, "%x", &tmp);
			revMesg[*revMesglen] = (char)tmp; 
			*revMesglen += 1;
			//lpc_printf("%x - %c\n\r",tmp,(char)tmp);		//reading while receiving doesn't work all of the time, careful when enable this
		}
	} else  // ktcpstart has no eof pattern, use timeout instead
	{
		lpc_printf("Listening to incoming TCP messages...\n\r");
		timeout = false;
		tim = xTimerCreate("TCP_Rev_Timer", pdMS_TO_TICKS(1000), pdFALSE, NULL, TCP_Rev_Timer);
		configASSERT(tim);
		xTimerStart(tim, portMAX_DELAY);
		while(!timeout)
		{
			do {
				len = UART_Receive(TEST_UART, (uint8_t *)buffer, 1, NONE_BLOCKING);// Use BLOCKING cause the stuck in exiting condition
			} while (len < 1);
			do {
				len = UART_Receive(TEST_UART,((uint8_t *) buffer)+1, 1, NONE_BLOCKING);// Use BLOCKING cause the stuck in exiting condition
			} while (len < 1);
			buffer[2] = '\0';
			sscanf(buffer, "%x", &tmp);
			revMesg[*revMesglen] = (char)tmp; 
			*revMesglen += 1;
			//lpc_printf("%x - %c\n\r",tmp,(char)tmp);		//reading while receiving doesn't work all of the time, careful when enable this
		}	
	}
	return true; // found it
}

void ATTCPClose(void)
{
	uint8_t AT_ATTCPClose[] = "AT#SH=1\r";
	lpc_printf("Closing TCP connection...\n\r"); 
	while(!checkATmessageReply(1,AT_ATTCPClose,sizeof(AT_ATTCPClose),0,"OK",2)) {};	 TIM_Waitms(100);
	
}

void write_istherefiletosend(uint8_t input)
{
	NandFlash_BlockErase(NANDFLASH_BLOCK_GATEWAY_DATATOSENDCOUNTCFG) ;
	NandFlash_PageProgram(&input, NANDFLASH_BLOCK_GATEWAY_DATATOSENDCOUNTCFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,0, NANDFLASH_RW_PAGE_SIZE);
}

bool volatile istherefiletosend_manualFTP = false;
static void OffTimer(TimerHandle_t pxTimer)
{
	// toggle reset pin - to turn off by hardware
	lpc_printf("OffTimer - Physically turning off\n\r");
	scu_pinmux(PD7_CONF);	
	GPIO_SetDir(PD7_DIR);
	PD7_LOW();	
	TIM_Waitms(200);
//	DataSendBackFTPDone = false;
	// Update to number of data to send
	if (istherefiletosend_manualFTP &(DataSendBackFTPDone))
	{
		write_istherefiletosend(1);
	} else
	{
		write_istherefiletosend(0);	
	}
	NodeReset();
}

extern TaskHandle_t xTaskToNotifyMQTT;
extern TaskHandle_t MQTTTask;

static void ReportTimer(TimerHandle_t pxTimer)
{
//	lpc_printf("ReportTimer fired\n\r");
	vTaskDelete(MQTTTask);

	if (xTaskToNotifyMQTT) {
		xTaskNotifyGive(xTaskToNotifyMQTT);
	}
}

extern uint8_t rcid_copy;
extern bool synching4G;

static int checkATmessageReply(bool timerrepeat, uint8_t *message, uint8_t size, bool iswait, char *term, uint8_t termlen)
{
	uint32_t len, i;
	uint8_t buffer[1];
	bool returnbool = false;
	char tmpbuf[256];
	memcpy(tmpbuf, message, size);

	memset(buffer,0,1);
	if ((rcid_copy == RC_CMD_RS_SYNCCLOCK) && (synching4G))
	{
		if (size > 0)
		{
			UART_Send(TEST_UART, message, size, BLOCKING, iswait);
		}
		return true;
	}
	if (timerrepeat){
		timAT = xTimerCreate("RSTimer", pdMS_TO_TICKS(5000), pdTRUE, NULL, ATTimer);
	}else{
		timAT = xTimerCreate("RSTimer", pdMS_TO_TICKS(10000), pdFALSE, NULL, ATTimer);
	}
	configASSERT(timAT);
	xTimerStart(timAT, portMAX_DELAY);
	notrestart = true;
	if (size > 0)
	{
		UART_Send(TEST_UART, message, size, BLOCKING, iswait);
	}
	i = 0;
	while(notrestart)
	{
		len = 0;
		len = UART_Receive(TEST_UART, &buffer[0], 1, NONE_BLOCKING);// Use BLOCKING cause the stuck in exiting condition
		if (len > 0)
		{		
			if (buffer[0] == 10) // NL, to prevent combining multiple lines to a word 
			{
				i = 0;
			}
			lpc_printf("%c",buffer[0]);		//tthis way works better than strstr but notrestart = false; must be commented out
			if (buffer[0]==term[i])
			{ 
				i = i+1;
				//lpc_printf("Matched: %c, i = %d\n\r",buffer[0],i);
				if (i==termlen)
				{
					returnbool = true;
					notrestart = false;
				}
			}
		}
	}
	lpc_printf("\n\r"); // without this, the CR (without NL) may ruin the debug display
	xTimerStop(timAT, portMAX_DELAY);
	xTimerDelete(timAT, portMAX_DELAY);
	return (returnbool);
}

static uint64_t getnum(char * mes)
{
	uint64_t output=0;
	if (strlen(mes) < 12)
	{
		return 0;
	} else 
	{
		if (sscanf(mes, "\n+CNUM: \"\",\"%llu\",%*d\r",&output) != 1 || output < 10000000000 || output > 99999999999) // based on the i=0 condition, line starts with 0x0A
		{			
			return 0;
		} else
		{
			return output;
		}
	}
}

static int checkATmessageReply_GetNumber(bool timerrepeat, uint8_t *message, uint8_t size, bool iswait, char *term, uint8_t termlen,uint64_t* cellPhoneNumber)
{
	uint32_t len, i, j;
	uint8_t buffer[1];
	bool returnbool = false;
	char revmesg[100] = {'\0'};
	volatile bool foundNumber = false;
	memset(buffer,0,1);
	if (timerrepeat){
		timAT = xTimerCreate("RSTimer", pdMS_TO_TICKS(1000), pdTRUE, NULL, ATTimer);
	}else{
		timAT = xTimerCreate("RSTimer", pdMS_TO_TICKS(10000), pdFALSE, NULL, ATTimer);
	}
	configASSERT(timAT);
	xTimerStart(timAT, portMAX_DELAY);
	notrestart = true;
	UART_Send(TEST_UART, message, size, BLOCKING, iswait);

	i = 0; j = 0;
	while(notrestart){
	len = 0;
		len = UART_Receive(TEST_UART, &buffer[0], 1, NONE_BLOCKING);// Use BLOCKING cause the stuck in exiting condition
		if (len > 0){		
			if (buffer[0] == 10) // NL, to prevent combining multiple lines to a word 
			{
				if ((*cellPhoneNumber < 10000000000) || (*cellPhoneNumber > 99999999999))// once get a 11-digit number, stop looking
				{
					*cellPhoneNumber = getnum(revmesg);
					if ((*cellPhoneNumber > 10000000000) && (*cellPhoneNumber < 19999999999))
					{
						foundNumber = true;
					}
				}
				i = 0; j = 0;
				memset(revmesg,0, strlen(revmesg));
			}
			revmesg[j++] = buffer[0];
			if (buffer[0]==term[i])
			{ 
				i = i+1;
				if ((i==termlen) && (foundNumber)){
					returnbool = true;
    			notrestart = false;
				}		
			}
		}
	}
	xTimerStop(timAT, portMAX_DELAY);
	xTimerDelete(timAT, portMAX_DELAY);
	return (returnbool);
}

int reportSMS(bool isinitialized) 	// isinitialized = 1 then already initizliaed, skip the part dealing with this. return 1 if continue (no message received), 0 if stop (DEPLOYSTOP received)
{
	uint32_t i;
	bool output = false;
	if (!isinitialized){
		UARTOn(115200);
		CellOn();
		
		timoff = xTimerCreate("SMSTimer", pdMS_TO_TICKS(30000), pdFALSE, NULL, OffTimer);
		configASSERT(timoff);
		xTimerStart(timoff, portMAX_DELAY);
		
		TIM_Waitms(200); //  Time for modem to start up
		// NEED A TIMER HERE TO END THE WHOLE THING IF NO CONNECTION
		ATCommandCheck();
		ATEchoCancel(); 
		
		xTimerStop(timoff, portMAX_DELAY);
		
		ATTimeGet(); // Terrible idea, synch, then get the correct time, could cause out-of-sync immediately

	} else {
		timoff = xTimerCreate("SMSTimer", pdMS_TO_TICKS(30000), pdFALSE, NULL, OffTimer);
		configASSERT(timoff);
		xTimerStart(timoff, portMAX_DELAY);
			
		ATSMSSetup1();
		ATSMSSetup2();
		xTimerStop(timoff, portMAX_DELAY);

		lpc_printf("Sending text message...");	
		snprintf(line, LINELEN, "Deploy mode, found %u/%u node(s): ",counter_ftp, rsncnt);
		for (i=0;i<counter_ftp;i++){
			snprintf(line+strlen(line),LINELEN," %u", targets2_ftp[i]);
		}	
		lpc_printf("%s\n\r",line);
		snprintf(line+strlen(line),LINELEN,"\x1A\x1A\r\n");
		UART_Send(TEST_UART,(uint8_t *)  line, sizeof(line), BLOCKING, 0); 
		TIM_Waitms(2000);lpc_printf("Done\n\r");
		xTimerStop(timoff, portMAX_DELAY);
	}
	return (!output);
}

uint64_t CellNumberReport; // put this outside so it is kept over multiple runs
bool is4Gworking = false;
extern bool TIM_busy;

void reportMQTT(uint8_t isinitialized) 	// 0: new, so init 1: initialized, so just send data, 2: stopped, so shutting down
{
	uint8_t i;
	uint8_t *ReportMesg;
	char topic[] = "SynchData";
	is4Gworking = false;
	
	if (isinitialized != 1)
	{
		timoffReport = xTimerCreate("REPORTTimer", pdMS_TO_TICKS(45000), pdFALSE, NULL, ReportTimer); // 10 s timeout	
		configASSERT(timoffReport);
		xTimerStart(timoffReport, portMAX_DELAY); 
	}

	if (isinitialized == 0){
	  setTime();
		CellNumberReport = ATNumGet();
		//ATBaudChange(460800);
		//UARTBaudChange(460800);
		//ATCommandCheck();
		ATHarwareFlowControl();
		ATConnConfig();
		ATTCPConnect();
		MQTT_Connect(100,CellNumberReport);
	} else if (isinitialized == 1)
	{
		ReportMesg = (uint8_t*) sdcalloc(1, sizeof(uint64_t) + 2*sizeof(float) + rsncnt * sizeof(uint8_t) ); // number + voltage + current + (IDs of detected nodes)
		configASSERT(ReportMesg);
		memcpy(&ReportMesg[0],&CellNumberReport,sizeof(uint64_t)); 
		memcpy(&ReportMesg[sizeof(uint64_t)],&voltage,sizeof(float)); 
		memcpy(&ReportMesg[sizeof(uint64_t) + sizeof(float)],&current,sizeof(float)); 
		for (i=0;i<counter_ftp;i++){
				memcpy(&ReportMesg[sizeof(uint64_t) + 2*sizeof(float) + i],&targets2_ftp[i],sizeof(uint8_t));
		}	
		MQTT_Publish(ReportMesg, sizeof(uint64_t) + 2*sizeof(float) + rsncnt * sizeof(uint8_t), &topic[0]); 
		ATEOFCheck(); //TUFIXDONE: STILL show no carrier on the Xnode side, the MQTT broker shows disconnect ok - timing
	} else if (isinitialized == 2)
	{
		//MQTT_Disconnect();  // TUFIX: the right order is MQTT_Disconnect then EOF, but doing that seems to make KTCP_DATA: and KTCP_NOTIF: to be sent out at the same time, causing overflow and problem to the reading of result message. so go for eof then disconnect for now, it won't be nice in the broker side though (disconnect due to timeout)
		ATTCPClose();
		lpc_printf("Done TCP\n\r");
		ATSwitchOff();	
	}
	if (isinitialized != 1)
	{
		xTimerStop(timoffReport, portMAX_DELAY);		
		xTimerDelete(timoffReport, portMAX_DELAY);		
	}
	synching4G = true;
	is4Gworking = true; 
	if (isinitialized != 1)
	{
		vTaskDelete(NULL);		
	}
}

void SendFTP()
{
}

void ReceiveFTP()
{
}

int Voltage_writedata_sendsms(float* battvol, float* battcur, uint16_t *uaddrs, uint8_t uidx)
{
	return 0;
}

int read_isFTP_FAIL()
{
	FRESULT ret;              // Result code
  FIL file;                 // File object
	
	// mount sdcard
  memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
	f_mount(0, &Fatfs);
	
	ret = f_chdir("/Xnode");
	if (ret) {
		lpc_printf("Change directory error\n\r");
		return 0;
	}
	ret = f_open(&file, "DataToSendCount.cfg", FA_WRITE | FA_CREATE_ALWAYS);
	if (ret) {
		lpc_printf("Create a new file error\n");
		return 0;
	}

	snprintf(line, LINELEN, "COUNT = %u", 0);
	if (f_puts(line, &file) == 0) {
		lpc_printf("Error writing data\r\n");
		f_close(&file);
		return 0;
	}

	lpc_printf("WROTE %s \n\r",line);
	f_close(&file);
	return 1;
	// No need to write to NANDflash to this as this is only a part of a bigger function what already does writing to NAND flash
}

uint32_t read_isFTP()
{
  uint32_t i, count = 0;	// default 0;
  uint8_t *flashbuf;
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);

	// try NAND Flash config first
	PRINTF("Reading NAND Flash...\r\n");
  if (NandFlash_PageRead(flashbuf, NANDFLASH_BLOCK_GATEWAY_DATATOSENDCOUNTCFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
  0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
    PRINTF("NAND Flash read success\r\n");
		count = flashbuf[0];		for (i = 1;i<sizeof(count);i++) {count = count | flashbuf[i] <<(8*i);}
		
		if (count>50000) {count = 0;}	// prevent things go crazy (\)
	} else {
    PRINTF("NAND Flash read failed\r\n");
  }
#ifdef SDCARD	
	// mount sdcard
  memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
	f_mount(0, &Fatfs);

	// go to data dir	
	ret = f_chdir("/Xnode");
  if (ret) {
    PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
		read_isFTP_FAIL(); // if fails, write 0

	//	return 0; // keep moving forward
	} else {

		// data to send count
		ret = f_open(&file, "DataToSendCount.cfg", FA_READ);
		if (ret) {
			PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
			read_isFTP_FAIL(); // if fails, write 0
		//	return 0; // keep moving forward
		} else {	
			if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "COUNT = %u", &count) != 1) {
				PRINTF("Error reading number of data sets to be sent\r\n");
				read_isFTP_FAIL(); // if fails, write 0
				f_close(&file);
			//	return 0;
			}
			f_close(&file);
		}
	}
# endif
	
	// write to NAND flash
	if ( count != (uint32_t)flashbuf)
	{
		memcpy(flashbuf, &count, sizeof (count));
		if (NandFlash_BlockErase(NANDFLASH_BLOCK_GATEWAY_DATATOSENDCOUNTCFG) != NANDFLASH_STATUS_SUCCESS) {
			PRINTF("NAND Flash erase failed\r\n");
		} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_GATEWAY_DATATOSENDCOUNTCFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
		0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
			PRINTF("NAND Flash write success\r\n");
		} else {
			PRINTF("NAND Flash write failed\r\n");
		}
	}
	return count;
}

void write_isFTP(void)
{	
	uint32_t count;
  uint8_t *flashbuf;
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);
	
	count = read_isFTP();
	
	// write to NAND flash
	if (count >100) {
		count = 0;	// for cases when this value is overwritten in NAND and corrupted, if negative, it will take forever to reach >0 and then block the FTP send back
		lpc_printf("***************************count = %d\n\r",count);
		}
	count = count + 1;
	memcpy(flashbuf, &count, sizeof (count));
	if (NandFlash_BlockErase(NANDFLASH_BLOCK_GATEWAY_DATATOSENDCOUNTCFG) != NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash erase failed\r\n");
	} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_GATEWAY_DATATOSENDCOUNTCFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash write success\r\n");
	} else {
		PRINTF("NAND Flash write failed\r\n");
	}
	read_isFTP();
}

int tests = 0;
int failures = 0;
FILE* xml;
char output[3000];
char* cur_output = output;

bool MQTT_Connect(int AliveInt, uint64_t CellNumber)
{
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	int rc = 0, buflen = 100, DataCount;
	unsigned char *buf, *sessionPresent, *connack_rc, *revMesg, *clean_revMesg;
	int *revMesglen;
	char clientID[100];
	sprintf(clientID, "%llu", CellNumber);
	
	sessionPresent = (unsigned char*)sdcalloc(1,100);
	connack_rc = (unsigned char*)sdcalloc(1,100);
	revMesg = (unsigned char* )sdcalloc(1,1000 * sizeof (char));
	clean_revMesg = (unsigned char* )sdcalloc(1,1000 * sizeof (char));
	revMesglen = (int *)sdcalloc(1,sizeof (int));
	buf = (unsigned char*)sdcalloc(1,buflen * sizeof(char));     
	configASSERT(sessionPresent && connack_rc && revMesg && clean_revMesg && revMesglen && buf);

	failures = 0;
	data.clientID.cstring = clientID;
	data.keepAliveInterval = AliveInt;
	data.cleansession = 1;
	data.username.cstring = "testuser";
	data.password.cstring = "testpassword";
	data.willFlag = 1;
	data.will.message.cstring = "will message";
	data.will.qos = 1;
	data.will.retained = 0;
	data.will.topicName.cstring = "will topic";
	rc = MQTTSerialize_connect(buf, buflen, &data);
	trem = rc;
	tdone = 0;
	do {
		tlen = (trem <= 1420) ? trem : 1420;
		ATTCPSend(tlen);
		//lpc_printf("%s", buf+tdone);
		UART_Send(TEST_UART, buf+tdone, tlen, BLOCKING, 0);
		ATEOFCheck();
		tdone += tlen;
		trem -= tlen;
	} while (trem > 0);
	DataCount = ATTCPDataGet(); // read the LAST KNOWN GOOD MESSAGE from the broker
	ATTCPReceive(DataCount, (char *)revMesg, revMesglen);
	if (*revMesglen >= DataCount) // to be safe, only consider when recevied data length is sufficient, and get rid of extra things before (this WILL happen)
	{
		memcpy(&clean_revMesg[0], &revMesg[*revMesglen-DataCount], DataCount);
	}
	if( MQTTDeserialize_connack(sessionPresent, connack_rc, clean_revMesg, DataCount))
	{	
		lpc_printf("CONNACK Detected! Publishing data\n\r");
		return 1; // 
	} else
	{
		lpc_printf("Broker not ready, Reset\n\r");
		return 0; // failed
	}
}

bool MQTT_Disconnect(void)
{
	int rc = 0, buflen = 100;
	unsigned char* buf;
	buf = (unsigned char*)sdcalloc(1,buflen);     // TUFIXED: msising this one caused crash-duh
	configASSERT(buf);
	rc = MQTTSerialize_disconnect(buf, buflen);
	trem = rc;
	tdone = 0;
	do {
		tlen = (trem <= 1420) ? trem : 1420;
		ATTCPSend(tlen);
		//lpc_printf("%s", buf+tdone);
		UART_Send(TEST_UART, buf+tdone, tlen, BLOCKING, 0);
		ATEOFCheck();
		tdone += tlen;
		trem -= tlen;
	} while (trem > 0);
	return 1;
}

void MQTT_Publish(uint8_t *payload, int payloadlen, char *topic)
{
	unsigned char* buf;
	int buflen = payloadlen + 100, rc = 0, qos = 0;
	char dummytopic[100];
	unsigned char dup = 0, retained = 0, msgid = 1;
	MQTTString topicString = MQTTString_initializer;	
	buf = (unsigned char*)sdcalloc(1,buflen);     
	configASSERT(buf);
	strcpy(&dummytopic[0],topic);		
	topicString.cstring = dummytopic;	
	lpc_printf("topicString.cstring = %s\n\r",topicString.cstring);	
	rc = MQTTSerialize_publish(buf, buflen, dup, qos, retained, msgid, topicString,	payload, payloadlen);	
	trem = rc;
	tdone = 0;
	do {
		tlen = (trem <= 1420) ? trem : 1420;
		ATTCPSend(tlen);
		//lpc_printf("%s", buf+tdone);
		UART_Send(TEST_UART, buf+tdone, tlen, BLOCKING, 0);
		ATEOFCheck();
		tdone += tlen;
		trem -= tlen;
	} while (trem > 0);
}

int MQTT_Subscribe(char *topic)
{
	int rc = 0, DataCount, requestedQoSs = 0;
	char dummytopic[100];
	unsigned char dup = 0;
	unsigned short msgid = 1;
	MQTTString topicString = MQTTString_initializer;	
	unsigned short* return_packetid;
	int *return_count, *return_grantedQoSs, *revMesglen;
	unsigned char *revMesg, *clean_revMesg, *buf;
	
	revMesg = (unsigned char* )sdcalloc(1,1000 * sizeof (char));
	clean_revMesg = (unsigned char* )sdcalloc(1,1000 * sizeof (char));
	revMesglen = (int *)sdcalloc(1,sizeof (int));
	buf = (unsigned char*)sdcalloc(1,100);     
	return_packetid = (unsigned short*)sdcalloc(1,sizeof(unsigned short)); 
	return_count = (int*)sdcalloc(1,sizeof(int)); 
	return_grantedQoSs = (int*)sdcalloc(1,sizeof(int)); 
	configASSERT(revMesg && clean_revMesg && revMesglen && buf && return_packetid && return_count && return_grantedQoSs);
	strcpy(&dummytopic[0],topic);	
	topicString.cstring = dummytopic;
	lpc_printf("Sub topic = %s\n\r",topicString.cstring);
	rc = MQTTSerialize_subscribe(buf, 100, dup, msgid, 1, &topicString, &requestedQoSs);
	trem = rc;
	tdone = 0;
	do {
		tlen = (trem <= 1420) ? trem : 1420;
		ATTCPSend(tlen);
		//lpc_printf("%s", buf+tdone);
		UART_Send(TEST_UART, buf+tdone, tlen, BLOCKING, 0);
		ATEOFCheck();
		tdone += tlen;
		trem -= tlen;
	} while (trem > 0);

	ATEOFCheck();
	DataCount = ATTCPDataGet();
	ATTCPReceive(DataCount,(char *) revMesg, revMesglen);
	if (*revMesglen >= DataCount) // to be safe, only consider when recevied data length is sufficient, and get rid of extra things before (this WILL happen)
	{
		memcpy(&clean_revMesg[0], &revMesg[*revMesglen-DataCount], DataCount);
	}
	if (MQTTDeserialize_suback(return_packetid, 1, return_count, return_grantedQoSs, (unsigned char* ) clean_revMesg, DataCount))
	{
		lpc_printf("Subscription sucessful\n\r");
		return 1;
	} else
	{
		lpc_printf("Broker not ready, Reset\n\r");
		return 0; // failed
	}
}

void listNAND(struct list_head* list_head_num, struct yaffs_obj *dir)
{
	YCHAR buffer[YAFFS_MAX_NAME_LENGTH + 1];
	struct yaffs_obj *l;
	list_for_each(list_head_num, &dir->variant.dir_variant.children) 
	{
		l = list_entry(list_head_num, struct yaffs_obj, siblings);
		yaffs_get_obj_name(l, buffer, YAFFS_MAX_NAME_LENGTH + 1);
		PRINTF("----: %s \n\r", buffer);
	}
}

void setTime()
{
	UARTOn(115200);
	CellOn();
	TIM_Waitms(7000); //  Time for modem to start up
	sendingFTP = true;
	ATCommandCheck();
	ATEchoCancel();
	ATTimeGet(); 
}

void ManualFTP()
{
	int j=0, f, payloadlen = 500000;
	uint8_t *flashbuf, *payload;
	struct yaffs_stat *stat;
	struct yaffs_dev *dev;	
	struct yaffs_obj *dir, *l;
	struct list_head *list_head_num;
	char topic[] = "MonitoringData", path[120], pathnew[120];
	uint64_t CellNumber;
	YCHAR buffer[YAFFS_MAX_NAME_LENGTH + 1], buffernew[YAFFS_MAX_NAME_LENGTH + 1];
	YCHAR *restOfPath = (YCHAR*)sdmalloc(YAFFS_MAX_NAME_LENGTH + 1); // need this malloc here or str[i] = *restOfPath; causes trouble
	bool alreadysent = false;
	
	dev = (struct yaffs_dev*)sdmalloc(sizeof (struct yaffs_dev));
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf && dev && restOfPath);
	
	// mount sdcard
  memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
	f_mount(0, &Fatfs);

lpc_printf("ManualFTP\r\n");
	UARTOn(115200);
	CellOn();
	
	TIM_Waitms(7000); //  Time for modem to start up
	sendingFTP = true;
	timoffFTP = xTimerCreate("SMSTimer", pdMS_TO_TICKS(45000), pdFALSE, NULL, OffTimer); // 3 min timeout
	configASSERT(timoffFTP);
	xTimerStart(timoffFTP, portMAX_DELAY);
	ATCommandCheck();
	ATEchoCancel();
//	ATTimeGet(); 	// not getting time and changing the clock unless the clock info propagates to the sensors             
//	TIM_Waitms(5000);
	LED_RGB(0,1,1);
	CellNumber = ATNumGet();
	//ATBaudChange(460800);
	//UARTBaudChange(460800);
	//ATCommandCheck();
	ATHarwareFlowControl();
	ATTCPPacketSize(1420);
	ATConnConfig();
	ATTCPConnect();
	xTimerChangePeriod(timoffFTP,pdMS_TO_TICKS(180000), portMAX_DELAY);
	MQTT_Connect(20,CellNumber);
	j =0;
	// Get data and send
	if(DataSendBackFTPDone)
	{
	// get data from NAND Flash and send 
	// Read where to start sending data
		lpc_printf("Basestation - Preparing data\n\r");

		dev = (struct yaffs_dev*) yaffsfs_FindDevice("/nand/", &restOfPath);
		stat = (struct yaffs_stat*)sdmalloc(sizeof (struct yaffs_stat));
		configASSERT(dev && stat);
		dir = dev->root_dir;
		// List all data files available
		//lpc_printf("LIST:\n\r");
		//listNAND(list_head_num, dir);
		// Look up not-yet-sent files to send back
		list_for_each(list_head_num, &dir->variant.dir_variant.children) 
		{
			alreadysent = false;  // for every file, assume it's not sent yet
			l = list_entry(list_head_num, struct yaffs_obj, siblings);
			yaffs_get_obj_name(l, buffer, YAFFS_MAX_NAME_LENGTH + 1);
			lpc_printf("/nand/%s----", buffer);
			if ((buffer[strlen(buffer)-8] == 's') && (buffer[strlen(buffer)-7] == 'e') && (buffer[strlen(buffer)-6] == 'n') & (buffer[strlen(buffer)-5] == 't'))
			{
				alreadysent = true;
				lpc_printf("ALREADYSENT\n\r");
			} else if ((buffer[0] == 'l') && (buffer[1] == 'o') && (buffer[2] == 's') & (buffer[3] == 't'))
			{
				alreadysent = true;
				lpc_printf("ALREADYSENT\n\r");
			}
			if(!alreadysent)
			{
				j = j+1;
				if (j>4) { // send max 4 files back at once
					istherefiletosend_manualFTP = true; // this is for the next round, if false then will not wake up and wait for command. So essentially this only write 1 files, check the next file to be sent or not, then exit
					lpc_printf("istherefiletosend_manualFTP = true________________________\n\r");
					break;
				}
				lpc_printf("NOTSENT\n\r");  // buffer is file name, and it is not sent yet
				lpc_printf("buffer = %s\n\r",buffer);							
				snprintf(path, 256, "/nand/%s",buffer);
				lpc_printf("path = %s\n\r",path);

				f = yaffs_open(path, O_RDONLY, S_IREAD);	// we now know this file exists for sure
				yaffs_fstat(f, stat) ;	
				lpc_printf("f = %d, stat->st_size = %d\n\r",f, (unsigned int) stat->st_size);
				
				payload = (unsigned char*)sdcalloc(1,payloadlen);     
				data_8bittype = (uint8_t *)sdcalloc(1,(unsigned int) stat->st_size+sizeof(uint64_t)+2*sizeof(float)); // data + cellnum+ vol + curr
				configASSERT(payload && data_8bittype);
				
				memcpy(&data_8bittype[0],&CellNumber,sizeof(uint64_t)); 
				memcpy(&data_8bittype[sizeof(uint64_t)],&voltage,sizeof(float)); 
				memcpy(&data_8bittype[sizeof(uint64_t)+sizeof(float)],&current,sizeof(float)); 

				yaffs_read(f, &data_8bittype[sizeof(uint64_t)+2*sizeof(float)], (unsigned int) stat->st_size);
				MQTT_Publish(data_8bittype,(unsigned int) stat->st_size+sizeof(uint64_t)+2*sizeof(float), &topic[0]);
				yaffs_flush(f);
				yaffs_close(f); 
				sdfree(data_8bittype);
				sdfree(payload);
				
				strcpy(buffernew,buffer); 
				snprintf(buffernew+strlen(buffernew)-4,34,"_sent.txt");
				f = yaffs_open(path, O_RDONLY, S_IREAD); 
				snprintf(pathnew, 256, "/nand/%s",buffernew); 
				lpc_printf("pathnew = %s\n\r",pathnew);
				f = yaffs_rename(path,pathnew); 
				lpc_printf("f_rename = %d\n\r",f); 
				if (f==-1)
				{
					NAND2SD(); // always dump all files (900 blocks) to SD, regardless, before formatting
					yaffs_format("/nand",1,1,1);
					NodeReset();
				}
				yaffs_close(f); 
				TIM_Waitms(5000);
				ATEOFCheck(); //TUFIXDONE: STILL show no carrier on the Xnode side, the MQTT broker shows disconnect ok - timing
			}
		}
	}
	xTimerChangePeriod(timoffFTP,pdMS_TO_TICKS(30000), portMAX_DELAY);
	//MQTT_Disconnect();  // TUFIX: the right order is MQTT_Disconnect then EOF, but doing that seems to make KTCP_DATA: and KTCP_NOTIF: to be sent out at the same time, causing overflow and problem to the reading of result message. so go for eof then disconnect for now, it won't be nice in the broker side though (disconnect due to timeout)
	ATTCPClose();
	lpc_printf("Done TCP\n\r");
	ATSwitchOff();
	
	// Update to number of data to send
	if (istherefiletosend_manualFTP &(DataSendBackFTPDone))
	{
		write_istherefiletosend(1);
	} else
	{
		write_istherefiletosend(0);	
	}
	xTimerStop(timoffFTP, portMAX_DELAY);
}

typedef struct {
	int nodeid; // - TUFIXDONE: Used to be uint16_t but as the struct uses blocks of 4 bytes, u16 won't help and can cause confusion on the receiver/subsribver side
	float volt; //voltage
	float curr; //current
} StatusInfo;

int Voltage_writedata_MQTT(float* battvol, float* battcur, uint16_t *uaddrs, uint8_t uidx)
{
	uint64_t CellNumber;
	char topic[] = "StatusData";
	uint8_t *StatusMesg;
	StatusInfo statInfo[20];	// Maximum 20 items (including GW in a network) - should be 255+1 but we are not there yet
	
	//Gateway
	statInfo[0].nodeid = 0;
	statInfo[0].volt = voltage;
	statInfo[0].curr = current;
	//Sensor
	for (uidx = 0; uidx < ulen; ++uidx) {
		statInfo[uidx+1].nodeid = uaddrs[uidx];
		statInfo[uidx+1].volt = battvol[uidx];
		statInfo[uidx+1].curr = battcur[uidx];
	}
	
	UARTOn(115200);
  // Enable UART Transmit
  UART_TxCmd(TEST_UART, ENABLE);
	CellOn();
	timoff = xTimerCreate("SMSTimer", pdMS_TO_TICKS(60000), pdFALSE, NULL, OffTimer);
	configASSERT(timoff);
	xTimerStart(timoff, portMAX_DELAY);
	TIM_Waitms(7000); //  Time for modem to start up
	ATCommandCheck();
	ATEchoCancel();
	CellNumber = ATNumGet();
	// Add CellNumber to the package to identify the node
	StatusMesg = (uint8_t*) sdcalloc(1,(ulen+1) * sizeof(StatusInfo) + sizeof(uint64_t)); // add 8 for the cellnum
	configASSERT(StatusMesg);
	memcpy(&StatusMesg[0],&CellNumber,sizeof(uint64_t)); 
	memcpy(&StatusMesg[8],&statInfo[0],(ulen+1) * sizeof(StatusInfo)); 

	//ATBaudChange(460800);
	//UARTBaudChange(460800);
	//ATCommandCheck();
	ATHarwareFlowControl();
	ATConnConfig();
	ATTCPConnect();
	MQTT_Connect(20,CellNumber);
	MQTT_Publish(StatusMesg,(ulen+1) * sizeof(StatusInfo) + sizeof(uint64_t), &topic[0]);
	TIM_Waitms(5000);
	
	timoff = xTimerCreate("OffTimer", pdMS_TO_TICKS(50000), pdTRUE, NULL, OffTimer);
	configASSERT(timoff);
	xTimerStart(timoff, portMAX_DELAY);
	ATEOFCheck(); //TUFIXDONE: STILL show no carrier on the Xnode side, the MQTT broker shows disconnect ok - timing
	//MQTT_Disconnect();  // TUFIX: the right order is MQTT_Disconnect then EOF, but doing that seems to make KTCP_DATA: and KTCP_NOTIF: to be sent out at the same time, causing overflow and problem to the reading of result message. so go for eof then disconnect for now, it won't be nice in the broker side though (disconnect due to timeout)
	ATTCPClose();
	lpc_printf("Done TCP\n\r");
	ATSwitchOff();	
	xTimerStop(timoff, portMAX_DELAY);

	return SUCCESS;
}

void UpdateParamMQTT() // REQUIRE the publisher to have retained = 1, so that the last known good message is retained
{
	int j=0, payloadlen = 500000, DataCount = 0;
	uint64_t CellNumber;
	uint8_t *flashbuf, *payload;
	char topic[] = "MonitoringParameters";
	unsigned char *revMesg, *clean_revMesg, *return_dup, *return_payload, *return_retained;
	int *revMesglen, *return_qos, *return_payloadlen;
	YCHAR *restOfPath = (YCHAR*)sdmalloc(YAFFS_MAX_NAME_LENGTH + 1); // need this malloc here or str[i] = *restOfPath; causes trouble
	flashconfig_t rev_fc={0}; // so zero value means that the parameter is not assigned
	unsigned short* return_packetid;
	MQTTString* return_topicName;
	
	payload = (unsigned char*)sdcalloc(1,payloadlen);     
	revMesg = (unsigned char* )sdcalloc(1,1000 * sizeof (char));
	clean_revMesg = (unsigned char* )sdcalloc(1,1000 * sizeof (char));
	revMesglen = (int *)sdcalloc(1,sizeof (int));
	return_dup = (unsigned char*)sdcalloc(1,sizeof(unsigned char));
	return_qos = (int*)sdcalloc(1,sizeof(int));;
	return_packetid = (unsigned short*)sdcalloc(1,sizeof(unsigned short));;
	return_topicName = (MQTTString*)sdcalloc(1,sizeof(MQTTString)*100);;
  return_payload = (unsigned char*)sdcalloc(1,sizeof(unsigned char));;
	return_payloadlen = (int*)sdcalloc(1,sizeof(int));	
	return_retained = (unsigned char*)sdcalloc(1,sizeof(unsigned char));	
	
	for (j=0;j<payloadlen;j++)
	{
		payload[j] = j%255;
	}
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(restOfPath && flashbuf && payload && revMesg && clean_revMesg && revMesglen && return_dup && return_qos && return_packetid && return_topicName && return_payload && return_payloadlen && return_retained);
	
	// mount sdcard
  memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
	f_mount(0, &Fatfs);
	
lpc_printf("UpdateParamMQTT\r\n");
	UARTOn(115200);
	CellOn();
	
	TIM_Waitms(7000); //  Time for modem to start up
	sendingFTP = true;
	timoffFTP = xTimerCreate("SMSTimer", pdMS_TO_TICKS(30000), pdFALSE, NULL, OffTimer); // 3 min timeout
	configASSERT(timoffFTP);
	xTimerStart(timoffFTP, portMAX_DELAY);
	ATCommandCheck();
	ATEchoCancel();
	TIM_Waitms(5000);
	LED_RGB(0,1,1);
	CellNumber = ATNumGet();
	//ATBaudChange(460800);
	//UARTBaudChange(460800);
	//ATCommandCheck();
	ATHarwareFlowControl();
	ATTCPPacketSize(1420);
	ATConnConfig();
	ATTCPConnect();
	xTimerChangePeriod(timoffFTP,pdMS_TO_TICKS(30000), portMAX_DELAY);
	MQTT_Connect(20,CellNumber);
	MQTT_Subscribe(&topic[0]);
	DataCount = ATTCPDataGet(); // read the LAST KNOWN GOOD MESSAGE from the broker

	ATTCPReceive(DataCount,(char *) revMesg, revMesglen);
	if (*revMesglen >= DataCount) // to be safem, only consider when recevied data length is sufficient
	{
		memcpy(&clean_revMesg[0], &revMesg[*revMesglen-DataCount], DataCount);
		/*lpc_printf("revMesglen = %d\n\r",*revMesglen);
		lpc_printf("DataCount = %d\n\r",DataCount);
		for (j=0;j<*revMesglen;j++)
		{
			lpc_printf("clean_revMesg[%d]: %x\n\r", j, clean_revMesg[j]);
		}*/
		MQTTDeserialize_publish(return_dup, return_qos, return_retained, return_packetid, return_topicName,	&return_payload, return_payloadlen, clean_revMesg, DataCount);
	/*	lpc_printf("rc = %d\n\r",rc);
		lpc_printf("*return_payloadlen = %d\n\r",*return_payloadlen);
		for (j=0;j<*return_payloadlen;j++)
		{
			lpc_printf("return_payload[%d]: %x\n\r", j, return_payload[j]);
		}		*/
	}
	xTimerChangePeriod(timoffFTP,pdMS_TO_TICKS(30000), portMAX_DELAY);
	//MQTT_Disconnect();  // TUFIX: the right order is MQTT_Disconnect then EOF, but doing that seems to make KTCP_DATA: and KTCP_NOTIF: to be sent out at the same time, causing overflow and problem to the reading of result message. so go for eof then disconnect for now, it won't be nice in the broker side though (disconnect due to timeout)
	ATTCPClose();
	lpc_printf("Done TCP\n\r");
	ATSwitchOff();
	if (*return_payloadlen == 16) // TUFIX: Hard code this 16 bytes for now
	{
		rev_fc.task1time = (return_payload[1] << 8 | return_payload[0]); lpc_printf("rev_fc.task1time = %d\n\r",rev_fc.task1time);
		rev_fc.task2time = (return_payload[3] << 8 | return_payload[2]); lpc_printf("rev_fc.task2time = %d\n\r",rev_fc.task2time);
		rev_fc.adxlthresholdact = (return_payload[5] << 8 | return_payload[4]); lpc_printf("rev_fc.adxlthresholdact = %d\n\r",rev_fc.adxlthresholdact);
		rev_fc.adxltimeact = (return_payload[7] << 8 | return_payload[6]); lpc_printf("rev_fc.adxltimeact = %d\n\r",rev_fc.adxltimeact);
		rev_fc.adxlthresholdinact = (return_payload[9] << 8 | return_payload[8]); lpc_printf("rev_fc.adxlthresholdinact = %d\n\r",rev_fc.adxlthresholdinact);
		rev_fc.adxltimeinact = (return_payload[11] << 8 | return_payload[10]); lpc_printf("rev_fc.adxltimeinact = %d\n\r",rev_fc.adxltimeinact);
		rev_fc.senselowerlimit = (return_payload[13] << 8 | return_payload[12]); lpc_printf("rev_fc.senselowerlimit = %d\n\r",rev_fc.senselowerlimit);
		rev_fc.senseupperlimit = (return_payload[15] << 8 | return_payload[14]); lpc_printf("rev_fc.senseupperlimit = %d\n\r",rev_fc.senseupperlimit);
		// Change parameters in BS's sdcard and NAND, only change when parameters are read correctly
		SDCard_ReWriteCN(rev_fc);
	}
	xTimerStop(timoffFTP, portMAX_DELAY);
}

unsigned int readWakeTime(void) // read saved wakeup time fromt the NAND flash
{
  uint8_t i;
	unsigned int count = 0;	// default 0
  uint8_t *flashbuf;
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);

	// try NAND Flash config first
	PRINTF("Reading NAND Flash...\r\n");
  if (NandFlash_PageRead(flashbuf, NANDFLASH_BLOCK_UNIXALARMTIME * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
  0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
    PRINTF("NAND Flash read success\r\n");
		count = flashbuf[0];		for (i = 1;i<sizeof(count);i++) {count = count | flashbuf[i] <<(8*i);}
		lpc_printf("Saved Wakeup time: %u\n\r",count);
	} else {
    PRINTF("NAND Flash read failed\r\n");
  }
	return count;
}

bool islongsleep(Clock_info *CurrentTime) // compare the saved time witht the current time
{
	struct tm tm;
	time_t epoch=0;	
// Convert to write to NAND
	tm.tm_sec = CurrentTime->second	;
	tm.tm_min = CurrentTime->minute;
	tm.tm_hour = CurrentTime->hour;
	tm.tm_mday = CurrentTime->date;
	tm.tm_mon = CurrentTime->month-1; // range 0 - 11
	tm.tm_year = CurrentTime->year+2000-1900; // since 1900
	tm.tm_isdst = -1; // unknown
	epoch = mktime(&tm);
	lpc_printf("epoch = %u\n\r",epoch);
	if (epoch - readWakeTime() > LONGSLEEP_VAL)
	{
		return true;
	} else
	{
		return false;
	}
}

int yaffs_cleanOldFiles(void)
{
	int returnint = 0;
	struct yaffs_dev *dev;	
	struct yaffs_obj *dir, *l;
	struct list_head *list_head_num, *list_head_num_save;
	YCHAR buffer[YAFFS_MAX_NAME_LENGTH + 1];
	YCHAR *restOfPath = (YCHAR*)sdmalloc(YAFFS_MAX_NAME_LENGTH + 1); // need this malloc here or str[i] = *restOfPath; causes trouble
	char path[120];
	
	dev = (struct yaffs_dev*)sdmalloc(sizeof (struct yaffs_dev));
	lpc_printf("Basestation - Preparing data\n\r");

	dev = (struct yaffs_dev*) yaffsfs_FindDevice("/nand/", &restOfPath);
	list_head_num = (struct list_head*)sdmalloc(sizeof (struct list_head));
	list_head_num_save = (struct list_head*)sdmalloc(sizeof (struct list_head));
	configASSERT(restOfPath && dev && list_head_num && list_head_num_save);
	dir = dev->root_dir;
	// List all data files available
	//lpc_printf("LIST:\n\r");
	//listNAND(list_head_num, dir);
	// Look up not-yet-sent files to send back
	list_for_each_safe(list_head_num, list_head_num_save, &dir->variant.dir_variant.children) 
	{
		l = list_entry(list_head_num, struct yaffs_obj, siblings);
		yaffs_get_obj_name(l, buffer, YAFFS_MAX_NAME_LENGTH + 1);
		lpc_printf("/nand/%s: ", buffer);
		if ((buffer[strlen(buffer)-8] == 's') && (buffer[strlen(buffer)-7] == 'e') && (buffer[strlen(buffer)-6] == 'n') & (buffer[strlen(buffer)-5] == 't'))
		{
			snprintf(path, 256, "/nand/%s",buffer);
			lpc_printf("-delete\n\r");
			if (yaffs_unlink(path))
			{
				returnint = returnint+1;
			} else
			{
				break;
			}
		}
	
	}	
	return returnint;
}
#endif
