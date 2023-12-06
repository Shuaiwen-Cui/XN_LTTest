#include <xnode.h>
#include <timers.h>
#include <lpc43xx_timer.h>
#include <math.h>
#include <string.h>
#include <queue.h>
#include <ads131.h>
#include <ads131_const.h>
#include <SnoozeAlarm.h>
#include "Sensing.h"
#include "RemoteSensing.h"
#include "TriggerSensing.h"
#include "GlobalTime.h"
#include "LocalTime.h"
#include <radio.h>

#if defined(GW_SENSE) || !defined(GATEWAY)
#include <filter.h>
#include <filter_o.h>
#include "ResampleUSF.h"
#include "EQRFilter280_280_100.h"
#include "arm_math.h"
#include "4GFTP.h"

#endif
//TUFIXDONE: Add a stop/disregard if sensing is short. Fix: If <= 6 seconds: Disregard
//TUFIXDONE: disregard long data sensing of the inactivity detection part. Fix: dslen shared between Sensing and writing functions allows this
//TUFIX: Make writing online too
//TUFIXDONE: Add online processing for trigger sensing. Fix: Normal processing, no detrend needed
//TUFIX: Add option to sync or not, to use ADXL data or not is already defined in TrigIndex = 0
//TUFIX: Make sure sensor can work with SD card but a corrupted one doesn't cause anythign wrong
#undef SUCCESS

#define _DEBUG_               0
#if _DEBUG_
#define PRINTF(...)           lpc_printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifdef FRA
#include <yaffsHEADER.h>
#include <nandflash_k9f1g08u0a.h>
extern uint16_t sllimit;	// Lower limit for sensing time
extern bool isformat;
extern bool DataSendBackFTPDone;

// Strain sensing
#ifdef STRAIN
bool autobalance = true;
bool shunt = true;
bool shunt2 = true;
uint8_t inc = 255;
uint8_t add = 255;
float vol4;
float vol6_A = 0;
float vol6_B;
float mean2;
float mean3;
float vref = 1.641;
float GF = 2.08;
float Vs = 3.3;
int i_Aend;
int i_Bend;
int i_Cend;
float sensitivity;
float input_strain = 1853.1 * 1e-6;
int i_sensingEnd;
#endif

uint16_t add_len = 3000; // TUADD: Added length to the combined data, so that the low pass filter doesn't omit the peak
#endif
// The task function declaration.
static void prvQueueSensingTask(void *tsensor);

// length of local queue of Sensor Data
#define mainQUEUE_LENGTH_DataProcess					1

#define DATA_RATE                             1
int BLOCKSIZE;
extern volatile uint8_t ADXL362_IACTD;
extern uint8_t TrigSenIdx;
uint64_t clock_s1=0, clock_s2=0;
uint64_t clock_g1=0, clock_g2=0;
#if defined(GW_SENSE) || !defined(GATEWAY)
	static bool isstart = false;
#endif

uint32_t dslen;

// Sensing configuration parameter
static ADS131_CFG_Type ADCsettings;
static CHANNEL_CFG_Type channelsSB[ADS131_NCHANNELS];

/* Sensor data structure */
typedef struct {
	SensorData *data; //data for sensors
	dataReady func; //callback function
	TaskHandle_t Stask; //local task for sensor
	TaskHandle_t Dtask; //local task for data
	QueueHandle_t queue; //local queue for data
} MySensor;
MySensor sensor;

TaskHandle_t Stask = NULL;
static TimerHandle_t tim;

int findmax(float *Data, int length)
{
	int i, index=0; float max=Data[0];
	for (i=0; i<length; i++)
	{
		if (max<Data[i])
		{
			max=Data[i];
			index=i;
		}
	}
	return index;
}

int findmin(float *Error, int length)
{
	int i, index=0; float min=Error[0];
	for (i=0; i<length; i++)
	{
		if (min>Error[i])
		{
			min=Error[i];
			index=i;
		}
	}
	return index;
}

int32_t RMS_data(int16_t * data, int length)
{
	int i;
	int32_t rms=0;
	for (i=0; i<length; i++)
		rms+=data[i]*data[i];
	rms=sqrt(rms/length);
	return rms;
}

static void sensingtimer(TimerHandle_t pxTimer)
{
	lpc_printf("ERROR: sensing timed out\r\n");
	die();
}

/* setup by default */
SensorData *setupAcquisition(uint16_t dataRate, uint32_t sampleTime, uint64_t clock)
{
	uint8_t i;
	SensorData *data = (SensorData*)sdcalloc(1, sizeof (SensorData));
	configASSERT(data);
	memset(data, 0, sizeof (SensorData));
	data->dataRate = dataRate;
	data->nodeID = LOCAL_ADDR;
	data->voltage = voltage;
	data->current = current;
	clock_g1=clock; // start sensing at clock_g (ticks)
	PRINTF("actual start sensing time is %lld\r\n", clock_g1);

	tim = xTimerCreate("SETimer", pdMS_TO_TICKS(3000), pdFALSE, NULL, sensingtimer);
	configASSERT(tim);
	xTimerStart(tim, portMAX_DELAY);

	if (ads131_on() != SUCCESS) {
		lpc_printf("ERROR: ads131_on() failed\n\r");
		die();
	}

	if (ads131_setupDAQ(&ADCsettings, DATA_RATE) != SUCCESS) {
		lpc_printf("ERROR: ads131_setupDAQ() failed\n\r");
		die();
	}

	xTimerStop(tim, portMAX_DELAY);
	xTimerDelete(tim, portMAX_DELAY);

	/* By default, use three channels for acceleration */
	for (i = 0; i < rschannels; i++) {
		data->channels[i].sampSize = sampleTime;
	}

	return data;
}

/* specify each channels */
Status setupChannel(SensorData *data, uint8_t channelNum, uint16_t samplingRate, chType sensorType)
{
	configASSERT(data);
	if (data->channels[channelNum].sampSize == 0) {
		if (ads131_setupCh(channelsSB, channelNum + 1, 0, 1, CH_OFF) != SUCCESS) {
			lpc_printf("ERROR: ads131_setupCh(%u) failed\n\r", channelNum);
			die();
			return ERROR;
		}
	}	else {
		data->channels[channelNum].sampSize *= samplingRate;
		data->channels[channelNum].samplingRate = samplingRate;
		data->channels[channelNum].type = sensorType;
#ifdef STRAIN
		if (ads131_setupCh(channelsSB, channelNum + 1 /*kam*/ + 5, 0, 1, NORMAL) != SUCCESS) {
#else
		if (ads131_setupCh(channelsSB, channelNum + 1, 0, 1, NORMAL) != SUCCESS) {
#endif
			lpc_printf("ERROR: ads131_setupCh(%u) failed\n\r", channelNum);
			die();
			return ERROR;
		}
	}
	return SUCCESS;
}

Status startAcquisition(SensorData *data, dataReady func)
{
	configASSERT(data && func);
	sensor.func = func;
	sensor.data = data;

	xTaskCreate(prvQueueSensingTask, "SensorTask", 3 * configMINIMAL_STACK_SIZE, &sensor, mainQUEUE_SENSING_TASK_PRIORITY, &sensor.Stask);
	configASSERT(sensor.Stask);
	Stask = sensor.Stask;

	return SUCCESS;
}

static void prvQueueSensingTask(void *tsensor)
{
#if defined(GW_SENSE) || !defined(GATEWAY)
	int i, j, testRate;
	int32_t stat;
	int32_t temp[ADS131_NCHANNELS + 1];
	float *sd[MAX_SENSOR_CHANNELS];
	int16_t *adxl[3];
	float mean_adxl[3]; double mean[3];
	uint32_t numSamples, numSamples_real;
	MySensor *tsen = (MySensor *)tsensor;
	configASSERT(tsen);

	// Prepare memory for data storage, BLOCKSIZE should be hardcoded and tested to make sure no spike occurs
	testRate = 1000 / tsen->data->channels[0].samplingRate;
	switch (testRate) {	// DO NOT CHANGE BLOCKSIZE (especially for case 2,5 and 10), if want to experiment with new values (for case 20, 50, 100), test first, but generally, can't go >30 for 8 channels, and 100 for 3 channels
#ifdef KHZ
		case 1:
			BLOCKSIZE = 10;
			numSamples = tsen->data->channels[0].sampSize * testRate;
			clock_g2 = clock_g1 + (numSamples)*1e3*120; // in ticks, at the end of sensing
			break;
#endif
		case 2:
			BLOCKSIZE = 10;
			numSamples = tsen->data->channels[0].sampSize * testRate + (size_2 - 1) + 2*BLOCKSIZE;
			clock_g2 = clock_g1 + (numSamples)*1e3*120; // in ticks, at the end of sensing
			if (clock_g1!=0)
				clock_g1 = clock_g1 - ((size_2 - 1) + 2*BLOCKSIZE)*1e3*120; // in ticks, at the beginning of sensing
			break;
		case 5:
			BLOCKSIZE = 10;
			numSamples = tsen->data->channels[0].sampSize * testRate + (size_5 - 1) + 2*BLOCKSIZE;
			clock_g2 = clock_g1 + (numSamples)*1e3*120; // in ticks, at the end of sensing
			if (clock_g1!=0)
				clock_g1 = clock_g1 - ((size_5 - 1) + 2*BLOCKSIZE)*1e3*120; // in ticks
			break;
		case 10:
			BLOCKSIZE = 10;
			numSamples = tsen->data->channels[0].sampSize * testRate + (size_10 - 1) + 2*BLOCKSIZE;
			clock_g2 = clock_g1 + (numSamples)*1e3*120; // in ticks, at the end of sensing
			if (clock_g1!=0)
				clock_g1 = clock_g1 - ((size_10 - 1) + 2*BLOCKSIZE)*1e3*120; // in ticks
			break;
		case 20:
			BLOCKSIZE = 20;
			numSamples = tsen->data->channels[0].sampSize * testRate + (size_10 - 1) + 10* (size_2 - 1) + 2*BLOCKSIZE;
			clock_g2 = clock_g1 + (numSamples)*1e3*120; // in ticks, at the end of sensing
			if (clock_g1!=0)
				clock_g1 = clock_g1 - ((size_10 - 1) + 10*(size_2 - 1) + 2*BLOCKSIZE)*1e3*120; // in ticks
			break;
		case 50:
			BLOCKSIZE = 50;
			numSamples = tsen->data->channels[0].sampSize*testRate + (size_10 - 1) + 10*(size_5 - 1) + 2*BLOCKSIZE;
			clock_g2 = clock_g1 + (numSamples)*1e3*120; // in ticks, at the end of sensing
			if (clock_g1!=0)
				clock_g1 = clock_g1 - ((size_10 - 1) + 10*(size_5 - 1) + 2*BLOCKSIZE)*1e3*120; // in ticks
			break;
		case 100:
			BLOCKSIZE = 100;
			numSamples = tsen->data->channels[0].sampSize * testRate + (size_10 - 1) + 10*(size_10 - 1) + 2*BLOCKSIZE;
			clock_g2 = clock_g1 + (numSamples)*1e3*120; // in ticks, at the end of sensing
			if (clock_g1!=0)
				clock_g1 = clock_g1 - ((size_10 - 1) + 10*(size_10 - 1) + 2*BLOCKSIZE)*1e3*120; // in ticks
			break;
		default:
			lpc_printf("Invalid decimation rate. %u\r\n", tsen->data->channels[0].samplingRate);
			break;
	}
	dslen = tsen->data->channels[0].sampSize; // This is the length of the final downsampled version to be saved. in FRA version, I cut down all the zero part so dslen usually < numSamples and it takes much less space
	memset(mean, 0, 3 * sizeof (double));
	memset(mean_adxl, 0 , 3*sizeof(float));
	for (i = 0; i < rschannels; ++i)	{
		if (tsen->data->channels[i].sampSize != 0) {
			sensor.data->channels[i].sampData = (float *)(sdcalloc(1, ((int) (numSamples/testRate)+1)*sizeof (float))); // add more space because sensing needs more than true sampsize and so sdfree works 
			sensor.data->channels[i].adxlData = (int16_t *)(sdcalloc(1,SAMPLE_SET*sizeof(int16_t)));
			sd[i] = (float *)sdcalloc(numSamples, sizeof (float));
			configASSERT(sensor.data->channels[i].sampData &&  sensor.data->channels[i].adxlData && sd[i]);	
		}
	}
#ifndef KHZ
	for (i = 0; i < rschannels; ++i)	{
		filtertest_SETUP(BLOCKSIZE, numSamples, testRate, i); // Setting up
	}
#endif

	tim = xTimerCreate("SETimer", pdMS_TO_TICKS(numSamples / testRate * 1000 + 90000), pdFALSE, NULL, sensingtimer);
	configASSERT(tim);
	xTimerStart(tim, portMAX_DELAY);
	lpc_printf("- sensing started...\r\n");

	ADXL362_IACTD=0;
	// Turn on sensing
	ads131_startSensing(&ADCsettings);
	numSamples_real = numSamples;
	// Main sensing loop
#ifdef STRAIN
	ADS131_DATA_COUNTER = 1;
#endif
	for (i = 0; i < numSamples;) {
		// get sample from ADC
		while (!ADS131_DRDY) { __WFI(); while (!ADS131_DRDY) { portYIELD(); } }
		i = ADS131_DATA_COUNTER - 1;
		ADS131_DRDY = 0;

		// for normal sensing, if clock_g1 ==0, it is trigger sensing.
		if (clock_g1 != 0 && (!isstart)) {
			if (clock_g1 > GlobalTime_get64()){
					i = 0; ADS131_DATA_COUNTER=0;
					continue;
			}
			else
				isstart = true;
		}

		if (i==0)
			clock_s1=LocalTime_get64();
		if (i==numSamples-1)
			clock_s2=LocalTime_get64();

#ifdef INACT
		if (ADXL362_IACTD) {
			PRINTF("inactivity detection\r\n");
			numSamples_real=i;			

// TUADD: Modify for CN case
#ifdef FRA // if too short, go to sleep instead of saving 
#ifdef STRAIN
			if (numSamples_real <= /*sllimit*/ /*kam*/2*1000) // 6 seconds - TUFIXDONE: Parametrize this number. Fix: Use sensing lower limit vlaue (s)
#else
			if (numSamples_real <= sllimit*1000) // 6 seconds - TUFIXDONE: Parametrize this number. Fix: Use sensing lower limit vlaue (s)
#endif
			{
				if ((i%testRate) == 1)	// make sure last round of downsampling is finished
				{
					PRINTF("Done, go to sleep\n\r");
					adxl362_setup();
					SnoozeAlarm_Sleep();
				}
			}
#endif		
			if ((i%testRate) == 1)	// make sure last round of downsampling is finished
			{
			//for simplicity, we just keep the original number, but all others are zeros.
				break;
			}
		}
#endif
		if (i >= numSamples) {
			break;
		}

		if (ads131_readDataPt(temp, &stat) == ERROR) {
			continue;
		}

		// So another downsampled version of the 'temp' data (1/10 size of original data) will be created and stored in filtertest function
		for (j = 0; j < rschannels; j++) {
#ifdef STRAIN
			sd[j][i] = (float) temp[j /* kam */ + 5];
#else
			sd[j][i] = (float) temp[j];
#endif
			sd[j][i] *= 2.4 / 0x7FFFFF; // V
#ifdef STRAIN
				if (j == 0) {
				sd[j][i] -= vref;
				} else {
					sd[j][i] = 0.f;
				}
#else
			if (j < 3) {
				sd[j][i] *= 1000. / 0.22; // mg  for RANGE 6G, 0.22; for RANGE 2G, 0.66
				mean[j]+=sd[j][i];
			}
#endif
#ifdef FRA //online filter for trigger sensing, no detrending
#ifdef KHZ
			 sensor.data->channels[j].sampData[i] = sd[j][i];
#else
			if (((i%10) == 0) & (i>0)) { // downsample every 10 points, and then every BLOCKSIZE points for 2nd filter
				filtertest(sd[j], sensor.data->channels[j].sampData, BLOCKSIZE, testRate,i,j);
			}
#endif
#else
			if (TrigSenIdx==0) {
				if (((i%10) == 0) & (i>0)) { // downsample every 10 points, and then every BLOCKSIZE points for 2nd filter
					filtertest(sd[j], sensor.data->channels[j].sampData, BLOCKSIZE, testRate,i,j);
				}
			}
#endif

		}
	}
	// Turn off ADC
	ads131_off();

#ifdef FRA //Run simple post-processing then exit, not doing post-sync for now. TUFIX: Make sure raw data is shown in SD CARD
	dslen = (numSamples_real-1)/testRate;

	// Write raw data as binary to SD Card
//	packSensorDataRAW(sd, 3, numSamples_real);	// CN doesn't need raw data, only SSTL needs 

	// Free raw data
	for (j=0;j<3;j++)	{sdfree(sd[j]);}
	////// Optional write original/RAW acceleration data in SD card of Leaf Node
	//write_acc(sd, numSamples);     

	// ---- read adxl data ----//
	lpc_printf("- read & write adxl data.\r\n");
	for (i=0; i<3; i++)
	{
		adxl[i]=(int16_t *)sdcalloc(SAMPLE_SET, sizeof (int16_t)); configASSERT(adxl[i]);
	}
	readadxl(adxl);
	for (j=0; j<3; j++)
	{
		memcpy(&sensor.data->channels[j].adxlData[j], adxl[j], SAMPLE_SET*sizeof(int16_t));
	}
	for (i=0; i<3; i++)
	{
		sdfree(adxl[i]);
	}

	lpc_printf("- post-processing completed.\r\n");
	LED_Off();

	//---------- End: post-sensing data fusion -----------//
	lpc_printf("- sensing completed.\r\n");
	LED_Off();
	
		// write time
	gettime(&sensor.data->year, &sensor.data->month, &sensor.data->date, &sensor.data->hour, &sensor.data->minute, &sensor.data->second);
	// pack and write data //TUFIXDONE: Make yaffs # of block variable, and make sure startup is not very early. Fix: Start up in write to NAND function
	
	if (PackAndSave(sensor.data) != SUCCESS) 
	{
		PRINTF("ERROR: failed to write data to SD card\r\n");
	}
	PRINTF("Done\r\n");	
	
	// Change a variable in sdcard to indicate there's something to send
	write_tosend(1);	// indicate that there's something to send

	// Done, go to sleep, set alarm in the next T minutes to wake up and send data
	lpc_printf("Done, go to sleep\n\r");
	adxl362_setup();
#if defined(GW_SENSE) && defined(GATEWAY)	
	write_istherefiletosend(1); 
	DataSendBackFTPDone = true; // technically not correct, but put this here to bypass the check in SnoozeAlarm_Sleep
#endif	
	NodeReset();
	
#else // FRA project doesn't require immediate data sending back
	sensor.func(sensor.data);
#endif
	xTimerStop(tim, portMAX_DELAY);
	xTimerDelete(tim, portMAX_DELAY);

	Stask = NULL;
	vTaskDelete(NULL);
#endif
}
