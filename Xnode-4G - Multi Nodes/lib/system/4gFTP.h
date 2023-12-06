/*************************************************
*@File        4gFTP.h
*@Brief       Headfile for 4G functions
*@Version     1.0
*@Date        04/19/18
*@Author      Tu Hoang
**************************************************/
#ifndef _4GFTP_H
#define _4GFTP_H
#include "yaffsHEADER.h"
#include "rtc.h"

void UARTOn(int br);
void UARTBaudChange(int newbr);
void CellOn(void);
void ATCommandCheck(void);
void ATEchoCancel(void);
void ATEOFCheck(void);
void ATSMSSetup1(void);
void ATSMSSetup2(void);
void ATSMSSend_FTPACK(void);
void ATSwitchOff(void);
void ATBaudChange(int newbr);
void ATConnConfig(void);
void ATFTPConfig(void);
void ATFTPSend(void);
void ATFTPReceive(void);
void ATFTPClose(void);
void ATTimeGet(void);
void ATTCPPacketSize(int ps);
void ATTCPConfig(void);
void ATTCPConnect(void);
void ATTCPSend(int len);
bool ATTCPReceive(int len, char* revMesg, int* revMesglen);
void ATCellInfo(void);
void ATNetworkInfo(void);
void ATTCPStart(void);
void ATTCPClose(void);
void write_istherefiletosend(uint8_t input);
int reportSMS(bool isinitialized); 	// isinitialized = 1 then already initizliaed, skip the part dealing with this. return 1 if continue (no message received), 0 if stop (DEPLOYSTOP received)
void SendFTP(void);
void ReceiveFTP(void);
int Voltage_writedata_sendsms(float* battvol, float* battcur, uint16_t *uaddrs, uint8_t uidx);
int read_isFTP_FAIL(void);
uint32_t read_isFTP(void);
void write_isFTP(void);
bool MQTT_Connect(int AliveInt, uint64_t CellNumber);
void MQTT_Publish(uint8_t *payload, int payloadlen, char *topic);
void listNAND(struct list_head* list_head_num, struct yaffs_obj *dir);
void ManualFTP(void);
int Voltage_writedata_MQTT(float* battvol, float* battcur, uint16_t *uaddrs, uint8_t uidx);
void UpdateParamMQTT(void);
unsigned int readWakeTime(void);
bool islongsleep(Clock_info *CurrentTime);
int yaffs_cleanOldFiles(void);
void setTime(void);
void reportMQTT(uint8_t isinitialized);

#define TEST_UART (LPC_USARTn_Type *)LPC_UART1

#define PD5_CONF  	  0xD, 5, MD_PDN, FUNC4	  // 
#define PD5_DIR		  6, 1 << 19,  1                        
#define PD5_HIGH()     GPIO_SetValue(6, 1 << 19)
#define PD5_LOW()      GPIO_ClearValue(6, 1 << 19)

#define PD7_CONF  	  0xD, 7, MD_PDN, FUNC4	  // 
#define PD7_DIR		  6, 1 << 21,  1                        
#define PD7_HIGH()     GPIO_SetValue(6, 1 << 21)
#define PD7_LOW()      GPIO_ClearValue(6, 1 << 21)

#endif
