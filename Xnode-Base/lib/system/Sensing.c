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

#ifndef GATEWAY
#include <filter.h>
#include <filter_o.h>
#include "ResampleUSF.h"
#include "EQRFilter280_280_100.h"
#include "arm_math.h"
#endif

#undef SUCCESS

#define _DEBUG_               1
#if _DEBUG_
#define PRINTF(...)           lpc_printf(__VA_ARGS__)
#else
#define PRINTF(...)
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
#ifndef GATEWAY
	static bool isstart = false;
	extern bool node_gw;
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

		if (ads131_setupCh(channelsSB, channelNum + 1, 0, 1, NORMAL) != SUCCESS) {
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
#ifndef GATEWAY
	int i, j, testRate;
	int32_t stat;
	int32_t temp[ADS131_NCHANNELS + 1];
	float *sd[MAX_SENSOR_CHANNELS];
	int32_t *sdtempf, *sdtempg; double Idelay; int32_t scale_ts;
	float mean_adxl[3]; double mean[3];
	/*
	int k, m, fsrc, fdst, offset
	int lengthsrc, lengthdst, lengthdsto, lengthcomb, length=SAMPLE_SET, lengthdeci;
	int16_t *adxl[3], *adxlc[3], *adxlr[3], *adxlr2;
	int16_t *ac_delay; float *ac_error;
	float *data_comb[3], *data_deci[3], *tmpv, *correl, *diffvec, diffval;
	float rms[3]; 
	*/
	uint32_t numSamples, numSamples_real;
	int16_t *Filter_b_dyn;
	uint32_t timerLen;

	MySensor *tsen = (MySensor *)tsensor;
	configASSERT(tsen);

	// Prepare memory for data storage, BLOCKSIZE should be hardcoded and tested to make sure no spike occurs
	testRate = 1000 / tsen->data->channels[0].samplingRate;
	switch (testRate) {	// DO NOT CHANGE BLOCKSIZE (especially for case 2,5 and 10), if want to experiment with new values (for case 20, 50, 100), test first, but generally, can't go >30 for 8 channels, and 100 for 3 channels
		case 2:
			BLOCKSIZE = 10;
			numSamples = tsen->data->channels[0].sampSize * testRate + (size_2 - 1) + 2*BLOCKSIZE;
			if (!TrigSenIdx){
			clock_g2 = clock_g1 + (numSamples)*1e3*120; // in ticks, at the end of sensing
			clock_g1 = clock_g1 - ((size_2 - 1) + 2*BLOCKSIZE)*1e3*120; // in ticks, at the beginning of sensing
			}
			break;
		case 5:
			BLOCKSIZE = 10;
			numSamples = tsen->data->channels[0].sampSize * testRate + (size_5 - 1) + 2*BLOCKSIZE;
			if (!TrigSenIdx){
			clock_g2 = clock_g1 + (numSamples)*1e3*120; // in ticks, at the end of sensing
			clock_g1 = clock_g1 - ((size_5 - 1) + 2*BLOCKSIZE)*1e3*120; // in ticks
			}
			break;
		case 10:
			BLOCKSIZE = 10;
			numSamples = tsen->data->channels[0].sampSize * testRate + (size_10 - 1) + 2*BLOCKSIZE;
			if (!TrigSenIdx){
			clock_g2 = clock_g1 + (numSamples)*1e3*120; // in ticks, at the end of sensing
			clock_g1 = clock_g1 - ((size_10 - 1) + 2*BLOCKSIZE)*1e3*120; // in ticks
			}
			break;
		case 20:
			BLOCKSIZE = 20;
			numSamples = tsen->data->channels[0].sampSize * testRate + (size_10 - 1) + 10* (size_2 - 1) + 2*BLOCKSIZE;
		  if (!TrigSenIdx){
			clock_g2 = clock_g1 + (numSamples)*1e3*120; // in ticks, at the end of sensing
			clock_g1 = clock_g1 - ((size_10 - 1) + 10*(size_2 - 1) + 2*BLOCKSIZE)*1e3*120; // in ticks
			}
			break;
		case 50:
			BLOCKSIZE = 50;
			numSamples = tsen->data->channels[0].sampSize*testRate + (size_10 - 1) + 10*(size_5 - 1) + 2*BLOCKSIZE;
			if (!TrigSenIdx){
			clock_g2 = clock_g1 + (numSamples)*1e3*120; // in ticks, at the end of sensing
			clock_g1 = clock_g1 - ((size_10 - 1) + 10*(size_5 - 1) + 2*BLOCKSIZE)*1e3*120; // in ticks
			}
			break;
		case 100:
			BLOCKSIZE = 100;
			numSamples = tsen->data->channels[0].sampSize * testRate + (size_10 - 1) + 10*(size_10 - 1) + 2*BLOCKSIZE;
		  if (!TrigSenIdx){
			clock_g2 = clock_g1 + (numSamples)*1e3*120; // in ticks, at the end of sensing
			clock_g1 = clock_g1 - ((size_10 - 1) + 10*(size_10 - 1) + 2*BLOCKSIZE)*1e3*120; // in ticks
			}
			break;
		default:
			lpc_printf("Invalid decimation rate. %u\r\n", tsen->data->channels[0].samplingRate);
			break;
	}
	dslen = tsen->data->channels[0].sampSize; // This is the length of the final downsampled version to be saved. in FRA version, I cut down all the zero part so dslen usually < numSamples and it takes much less space

	PRINTF("time to start sensing is %lld, local time is %lld\r\n", clock_g1, GlobalTime_get64());

	memset(mean, 0, 3 * sizeof (double));
	memset(mean_adxl, 0 , 3*sizeof(float));

	for (i = 0; i < rschannels; ++i)	{
		if (tsen->data->channels[i].sampSize != 0) {
			sensor.data->channels[i].sampData = (float *)(sdcalloc(1, ((int) (numSamples/testRate)+1)*sizeof (float))); // add more space because sensing needs more than true sampsize and so sdfree works 
			sd[i] = (float *)sdcalloc(numSamples, sizeof (float));
			configASSERT(tsen->data->channels[i].sampData && sd[i]);
		}
	}

	for (i = 0; i < rschannels; ++i)	{
		filtertest_SETUP(BLOCKSIZE, numSamples, testRate, i); // Setting up
	}

	if(!TrigSenIdx) // normal sensing
		timerLen=numSamples / testRate * 1000 + 90000;
	else
		timerLen=numSamples / testRate * 1000 + 90000 + 60000;
	
	tim = xTimerCreate("SETimer", pdMS_TO_TICKS(timerLen), pdFALSE, NULL, sensingtimer);
	configASSERT(tim);
	xTimerStart(tim, portMAX_DELAY);
	lpc_printf("- sensing started...\r\n");

	ADXL362_IACTD=0;
	// Turn on sensing
	ads131_startSensing(&ADCsettings);
	LED_RGB(0,1,1);
	numSamples_real = numSamples;
	// Main sensing loop
	for (i = 0; i < numSamples;) {
		// get sample from ADC
		while (!ADS131_DRDY) { __WFI(); while (!ADS131_DRDY) { portYIELD(); } }
		i = ADS131_DATA_COUNTER - 1;
		ADS131_DRDY = 0;

		// for normal sensing, if clock_g1 ==0, it is trigger sensing.
		if ((!TrigSenIdx) && (!isstart)) {
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

		// for time sync test, we just comment it out. 
#ifdef INACT
		if (ADXL362_IACTD) {
			PRINTF("inactivity detection\r\n");
			numSamples_real=i;
			//for simplicity, we just keep the original number, but all others are zeros.
			break;
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
			sd[j][i] = (float) temp[j];
			sd[j][i] *= 2.4 / 0x7FFFFF; // V
			if (j < 3) {
				sd[j][i] *= 1000. / 0.22; // mg  for RANGE 6G, 0.22; for RANGE 2G, 0.66
				mean[j]+=sd[j][i];
			}
			/*
			else {
					sd[j][i] *= rscalibinfo[j-3].scale;
					sd[j][i] += rscalibinfo[j-3].offset;
			}
			*/
			//if (!TrigSenIdx)  // for time sync test, we just comment it out. 
			{
				if (((i%10) == 0) & (i>0)) { // downsample every 10 points, and then every BLOCKSIZE points for 2nd filter
					filtertest(sd[j], sensor.data->channels[j].sampData, BLOCKSIZE, testRate,i,j);
				}
			}
		}
	}
	// Turn off ADC
	ads131_off();
	LED_RGB(1,0,0);

	Filter_b_dyn = (int16_t *)sdmalloc(Filter_b_length * sizeof (int16_t));
	configASSERT(Filter_b_dyn);
	memcpy(Filter_b_dyn, Filter_b, Filter_b_length * sizeof (int16_t));

	ads131_deinit();
	radio_init();
	radio_set_short_addr(LOCAL_ADDR);
	vTaskPrioritySet(Stask, tskIDLE_PRIORITY + 1UL);
	
	LED_RGB(1,1,0);
	if (!TrigSenIdx) {
		PRINTF("time synchronization 2, wait for 5s\r\n");
		vTaskDelay(5000);
	}
	else{
		Post_event_Timesync(clock_s1);
		if(node_gw)
			clock_g1 = clock_s1;//+1298*120; 
		clock_g2 = clock_g1+(numSamples_real-1)*1e3*120;
		lpc_printf("Reference node: global time 1 is %lld, global time 2 is %lld", clock_g1, clock_g2);
	}

	PRINTF("localtime 1 is %lld, localtime 2 is %lld,\r\n", clock_s1, clock_s2);
	clock_s1 = GlobalTime_generate64(clock_s1);
	clock_s2 = GlobalTime_generate64(clock_s2);
	PRINTF("globaltime 1 is %lld, globaltime 2 is %lld\r\n", clock_s1, clock_s2);

	// data synchronization
	sdtempf=(int32_t*)sdcalloc(sensor.data->channels[0].sampSize, sizeof(int32_t)); configASSERT(sdtempf);
	sdtempg=(int32_t*)sdcalloc(sensor.data->channels[0].sampSize, sizeof(int32_t)); configASSERT(sdtempg);

	Idelay=((double)((int64_t)clock_s1+(int64_t)clock_s2-(int64_t)clock_g1-(int64_t)clock_g2))/2/120/1000000;
	PRINTF("delay is %lf\r\n", Idelay);
	for (j=0; j<rschannels; ++j) {
		if(j<3)
			scale_ts=1;
		else
			scale_ts=1000;
		
		for (i=0; i<sensor.data->channels[0].sampSize; i++)
			sdtempf[i]=(int32_t)(sensor.data->channels[j].sampData[i]*scale_ts*10000); // to keep the decimals in float

		filter_InitDelayLI_USF(tsen->data->channels[0].samplingRate, tsen->data->channels[0].samplingRate, sdtempf, sensor.data->channels[0].sampSize, Filter_b_dyn, Filter_b_length, sdtempg, sensor.data->channels[0].sampSize, Idelay, 100, Filter_scale_factor);
		for (i=0; i<sensor.data->channels[0].sampSize; i++)
			sensor.data->channels[j].sampData[i]=(float)sdtempg[i]/(scale_ts*10000);

		memset(sdtempf, 0, sensor.data->channels[0].sampSize*sizeof(int32_t));
		memset(sdtempg, 0, sensor.data->channels[0].sampSize*sizeof(int32_t));
	}
	sdfree(sdtempf);
	sdfree(sdtempg);

	sensor.data->timestamp = ((int64_t)clock_s1-(int64_t)clock_g1)/120; //us
	PRINTF("actual offset of start sensing time is %lld\r\n", sensor.data->timestamp); // ahead of actual sensing time

	//write_acc(sd, numSamples);     //// Optional write original/RAW acceleration data in SD card of Leaf Node

	/* // for time sync test, we just comment it out. 
	if(TrigSenIdx){
		for (j = 0; j < 3; ++j) {
			mean[j] /= numSamples_real;
			for(i=0; i<numSamples_real; i++)
				sd[j][i]-=mean[j];
		}

	// ---- read adxl data ----//
	PRINTF("- read & write adxl data.\r\n");
	for (i=0; i<3; i++)
		adxl[i]=(int16_t *)sdcalloc(length, sizeof (int16_t)); configASSERT(adxl[i]);
	readadxl(adxl);

//--------- Start: post-sensing data fusion ----------/
	LED_RGB(1,1,0);

	PRINTF("- post-sensing data fusion started.\r\n");
	ac_error = (float*)sdcalloc(201, sizeof(float));
	ac_delay = (int16_t*)sdcalloc(201, sizeof(int16_t));
	configASSERT(ac_delay && ac_error);

	// filter Xnode raw data with fc = 50Hz

	// upsample to 1000Hz
	lengthsrc = length;
	lengthdst = (length-1)*10+1;
	for (j=0; j<3; j++)
	{
		adxlr[j] = (int16_t*) sdcalloc(lengthdst, sizeof(int16_t)); configASSERT(adxlr[j]);
		filter_USF(100, 1000, adxl[j], lengthsrc, Filter_b_dyn, Filter_b_length, adxlr[j], lengthdst, 100, Filter_scale_factor);
	}
	PRINTF("upsample is completed\r\n");

	for (j=0; j<3; j++)
		rms[j] =	(float)(RMS_data(adxl[j], length));
	j = findmax(rms, 3);

	PRINTF("max rms is %u axis\r\n", j);

	lengthsrc = overlap_size; fdst=1000; fsrc = fdst - (201 - 100); lengthdsto = ceil (lengthsrc * fdst / fsrc);
	adxlr2 = (int16_t*)sdcalloc (lengthdsto, sizeof (int16_t));
	tmpv = (float*)sdcalloc(lengthdsto, sizeof(float));
	correl = (float*)sdcalloc(2*lengthdsto-1, sizeof(float));
	diffvec = (float*)sdcalloc(lengthdsto, sizeof(float));
	configASSERT(adxlr2 && tmpv && correl && diffvec);

	for (m=0; m<201; m++)
	{
		fsrc = fdst - (m-100);
		lengthdst = ceil (lengthsrc * fdst / fsrc);
		filter_USF(fsrc, fdst, &adxlr[j][(length-1)*10+1-lengthsrc], lengthsrc, Filter_b_dyn, Filter_b_length, adxlr2, lengthdst, 100, Filter_scale_factor);

		for (i=0; i<lengthdst; i++)
			mean_adxl[j]+=(float)adxlr2[i];
		mean_adxl[j]/=lengthdst;
		for (i=0; i<lengthdst; i++)
			tmpv[i] = (float)adxlr2[i]-mean_adxl[j];

		//cross correlation, check the axis
		arm_correlate_f32(tmpv, lengthdst, sd[j], lengthsrc, correl);
		k=findmax(correl, 2*lengthdsto-1);
		if(lengthsrc > lengthdst)
			ac_delay[m] = k - lengthsrc + 1;
		else
			ac_delay[m] = k - lengthdst + 1;
		PRINTF("max index of x axs is %d\r\n", ac_delay[m]); // z direction, which is the main direction of bridge vibration

		if (ac_delay[m]>=0)
		{
			if (ac_delay[m] ==0)
				ac_delay[m]=1;

			arm_sub_f32 (sd[j], &tmpv[ac_delay[m]-1], diffvec, lengthdst-ac_delay[m]+1);
			arm_power_f32 (diffvec, lengthdst-ac_delay[m]+1, &diffval);
			ac_error[m] = sqrt(diffval)/(lengthdst-ac_delay[m]+1);
		}
		else
		{
			arm_sub_f32 (tmpv, &sd[j][-ac_delay[m]-1], diffvec, lengthdst);
			arm_power_f32 (diffvec, lengthdst, &diffval);
			ac_error[m] = sqrt(diffval)/(lengthdst);
		}
		PRINTF("difference is %f\r\n", ac_error[m]);

		memset(adxlr2, 0, lengthdsto*sizeof(int16_t));
		memset(tmpv, 0, lengthdsto*sizeof(float));
	  memset(correl, 0, (2*lengthdsto-1)*sizeof(float));
		memset(diffvec,0, lengthdsto*sizeof(float));
		memset(mean_adxl, 0 , 3*sizeof(float));
	}

	LED_RGB(0,1,1);
	// ---- find best estimation ---- //
	lengthsrc = (length-1)*10+1;
	m = findmin(ac_error, 201); fdst = 1000; fsrc = fdst - (m-100); offset = ac_delay [m]; lengthdst = ceil (lengthsrc * fdst / fsrc); lengthdsto = ceil (overlap_size * fdst / fsrc);
	
	PRINTF("optimal m is %u, offset is %d, sampling rate is %d\r\n", m, offset, fsrc);
	
	for (j=0; j<3; j++)
	{
		adxlc[j]=(int16_t*) sdcalloc(lengthdst, sizeof(int16_t));
		filter_USF(fsrc, fdst, adxlr[j], lengthsrc, Filter_b_dyn, Filter_b_length, adxlc[j], lengthdst, 100, Filter_scale_factor);
	}
	
	lpc_printf("debug 1\r\n");

	// ---- combine data ---- //
	lengthcomb=lengthdst+numSamples-(lengthdsto - offset);
	for (j=0; j<3; j++)
	{
		data_comb[j]=(float*) sdcalloc(lengthcomb, sizeof(float)); configASSERT(data_comb[j]);
		for (i=0; i<lengthdst; i++)
			mean_adxl[j]+=(float)adxlc[j][i];
		mean_adxl[j]/=lengthdst;
		for (i=0; i<lengthdst; i++)
			data_comb[j][i]=(float)(adxlc[j][i])-mean_adxl[j];
		memcpy(&data_comb[j][lengthdst-(lengthdsto-offset)], sd[j], numSamples*sizeof(float));
	}
	
	lpc_printf("debug 2\r\n");

	// apply filter
	lengthdeci=(lengthcomb-(size_10 - 1))/10;
	lengthcomb=lengthdeci*10+(size_10 - 1);
	for (j=0; j<3; j++)
	{
		data_deci[j]=(float*) sdcalloc(lengthdeci, sizeof(float));
		filter(data_comb[j], data_deci[j], lengthcomb, 10);

		for (i = 0; i < sensor.data->channels[j].sampSize; ++i) {
			sensor.data->channels[j].sampData[i]=data_deci[j][i];
		}
		
		sdfree(data_deci[j]);
		sdfree(data_comb[j]);
	}
	

	for (i=0; i<3; i++){
		sdfree(adxl[i]);
	}

	PRINTF("- post-processing completed.\r\n");
	LED_Off();
	}
	*/
	//---------- End: post-sensing data fusion -----------//

	// Free memory
	for (j = 0; j < rschannels; ++j) {
	  sdfree(sd[j]);
	}
	sdfree(Filter_b_dyn);

	PRINTF("- sensing completed.\r\n\r\n");
	LED_Off();

	sensor.func(sensor.data);

	xTimerStop(tim, portMAX_DELAY);
	xTimerDelete(tim, portMAX_DELAY);
	
	vTaskDelay(5000);
	if(node_gw)
		Post_event_Notification(clock_s1);

	Stask = NULL;
	vTaskDelete(NULL);
#endif
}
