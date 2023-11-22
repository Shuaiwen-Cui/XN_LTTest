#include "timesync.h"
#include "SysTime.h"
#include "LocalTime.h"
#include "GlobalTime.h"
#include <ff.h>
#include <math.h> 
#include <stdio.h>
#include <stdlib.h>
#include <rtc.h>
#include <sdcard.h>
#include <task.h>

//--> type define
enum {
  TYPE_TIMING =   0,
  TYPE_FEEDBACK = 1,
};


#define _DEBUG_               0
#if _DEBUG_
#define PRINTF(...)           lpc_printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define LINELEN               80
static char line2[LINELEN];
extern FATFS Fatfs;
static bool priorityTask = false;

typedef struct _TS_Status //Flag for TS condition
{
	 uint8_t    isSynzd;   //timesync status
	uint16_t   preIndex;   //time index
	uint16_t   curIndex;   //time index
	uint16_t   Reserved;   //time index
	uint64_t  preGTime64;  //Reserved
	uint64_t  curGTime64;  //Reserved
	uint64_t  preLTime64;  //Reserved
	uint64_t  curLTime64;  //Reserved
} _TS_Status;
static _TS_Status condLocal;  //local status
#define MAXINDEX (0xFFFF);
extern bool app_inactive;

typedef struct ts_msg //timesync message
{
	uint16_t sender;   //leaf nodeID
	uint16_t receiver; //gateway ID
	uint16_t index;    //index count
	uint16_t dummy16;  //Reserved
	uint8_t  type;     //message type
	uint8_t  delay;    //total delay	
  uint8_t  unit;     //Reserved
	uint8_t  dummy;    //Reserved
  uint64_t time64;   //timing signal
} __attribute__((packed))ts_msg; //message used for timesynchr
ts_msg tsm;   //timesynchr message

typedef struct gw_config //gateway configuration
{
    char     ch;                  //serial input
    uint8_t  cnt;                 //number of leaf nodes.
    uint16_t gateway;             //gateway nodeid
    uint16_t nodeid[MAX_NODES];   //leaf nodeid

    TimerHandle_t ts;       //handle of source timer 
    uint8_t isTiming;       //flag of timing
    uint8_t isCounterOk;    //status of realtime counter
} gw_config; //message used for timesynchr

#ifdef GATEWAY
	static gw_config gwc; //gateway parameters
#endif
//--> parameters define
int i; //common loop variable
#define MSG_SIZE (sizeof(ts_msg))     //128bit message
//static  uint8_t msgbuf[MSG_SIZE];   //message buffer
static  uint8_t *tssbuf;  //data buffer

int64_t offsetSync[2]={0};//int64_t offsetSync[TSYNC_STNUM] = {0};
int64_t localtime_TS[2]={0};//int64_t localtime_TS[TSYNC_STNUM] = {0};
int64_t *tempoffset; // to store the data set of offsets;
int64_t *templocal; // to store the data set of offsets;
uint64_t clock = 0;
uint16_t index = 0;
uint16_t count = 0;
int dosync=0;
long double slope = 0;
long double intersect = 0;
long double correlation = 0;

//#ifdef GATEWAY
//volatile static  TaskHandle_t xTaskToNotify_TS = NULL; //task handle
//#endif

//<--------------------

//--> Private Function Declaration
void init_TimeSync_Buffer(void);
void init_NodeStatus(void);
void callback_tsc_cmd(uint16_t srcid, void *data, uint8_t len);//gencomm callback function

// void callback_timing_timer(TimerHandle_t pxTimer); //automatic timing
int Creat_TimingMessage(void);
void Update_LocalParameter(ts_msg *msg);
void Creat_FeedbackMessage(ts_msg *msg);

//-->Public function define
void TimeSync_GetConfig(void)
{
#ifdef GATEWAY
  gwc.cnt = 1; //number of leaf nodes.
  gwc.nodeid[0]=(uint16_t)0x2; //NodeID of leaf node
  gwc.gateway = (uint16_t)0x1; //NodeID of gateway
#endif
  tsm.index=0;
  tsm.dummy=0xff;
  tsm.delay=0;

  tsm.type=TYPE_TIMING;
#ifdef GATEWAY	
  gwc.isTiming=FALSE;
#endif
}

int TimeSync_Register(void) //please check "lib\system\GlobalConstants.h" for GC_APP_TIMESYNC define.
{
  init_TimeSync_Buffer();
  TimeSync_GetConfig();
  init_NodeStatus();
  GenericComm_register(GC_APP_TIMESYNC, callback_tsc_cmd);
	//dosync=1;
	return SUCCESS;
}

int Timesync_Init(void){
  RealTimer_init();
  NVIC_EnableIRQ(RITIMER_IRQn);
#ifdef GATEWAY	
  gwc.isCounterOk=TRUE;
#endif	
	TimeSync_Register();
  return SUCCESS;
};

void TimeSync_msg2dat(uint8_t *buff,ts_msg *msg,uint8_t len){
  memcpy(buff,msg,len);
}

void TimeSync_dat2msg(ts_msg *msg,uint8_t *buff,uint8_t len){
  memcpy(msg,buff,len);
}

void init_NodeStatus(void){
	condLocal.isSynzd=FALSE;   //timesync status
	condLocal.preIndex=0;     //time index
	condLocal.preGTime64=0;    //Reserved
	condLocal.preLTime64=0;    //Reserved
	condLocal.curLTime64=0;    //Reserved
};

static void init_TimeSync_Buffer(void) 
{
  tssbuf = (uint8_t *)sdmalloc(MSG_SIZE);
  configASSERT(tssbuf);
  for (i = 0; i < MSG_SIZE; ++i) {
    tssbuf[i] = (uint8_t)i;
  }
	//offsetSync = (int64_t *)sdcalloc(TSYNC_STNUM, sizeof(int64_t));
	//localtime_TS = (int64_t *)sdcalloc(TSYNC_STNUM, sizeof(int64_t));
	tempoffset = (int64_t *)sdcalloc(TSYNC_OTNUM, sizeof(int64_t));
	templocal = (int64_t *)sdcalloc(TSYNC_OTNUM, sizeof(int64_t));
	configASSERT(tempoffset && templocal);
}

//-->public functions
  int TimeSync_Start(void){
    //gwc.isTiming=TRUE;
		//gwc.ts = xTimerCreate("Timer_timing", pdMS_TO_TICKS(TIMING_PERIOD*1000), pdTRUE, NULL, callback_timing_timer); //gateway timing timer
    //configASSERT(gwc.ts); xTimerStart(gwc.ts, portMAX_DELAY);		
    int i;
    
		dosync=1;
    index = Creat_TimingMessage(); //creat timing signal
		
		/*
    if (index>TSYNC_STNUM)
				TimeSync_Stop();
		*/
		for (i=0; i<30; i++)
		{
    	while(GenericComm_bcast(GC_APP_TIMESYNC, tssbuf, MSG_SIZE) != SUCCESS) { //check sizeof(MSG_SIZE)
      	vTaskDelay(0); portYIELD();
      }
			vTaskDelay(4);
		}
		dosync=0;
		return SUCCESS;
  };

  void TimeSync_Stop(void){
		
	#ifdef GATEWAY
    gwc.isTiming=FALSE;
    //vTaskDelete(xTaskToNotify_TS);
    xTimerStop(gwc.ts, portMAX_DELAY);
    xTimerDelete(gwc.ts, portMAX_DELAY);
	#endif
		dosync=0;
		sdfree(offsetSync);
		sdfree(localtime_TS);
		sdfree(tempoffset);
		sdfree(templocal);
		//NodeReset();
  };

  //-->private callback functions
/*  // comment out to reduce warning
  static void callback_timing_timer(TimerHandle_t pxTimer) //callback of timing timer
  {
    int i;
    
    index = Creat_TimingMessage(); //creat timing signal
    if (index>TSYNC_STNUM)
				TimeSync_Stop();
		
		for (i=0; i<10; i++)
		{
    	while(GenericComm_bcast(GC_APP_TIMESYNC, tssbuf, MSG_SIZE) != SUCCESS) { //check sizeof(MSG_SIZE)
      	vTaskDelay(0); portYIELD();
      }
			vTaskDelay(4);
    }
  }
*/
	
int record_time(void)
{
	FIL file;                 // File object
	uint32_t i;

	// mount sdcard
	memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
	f_mount(0, &Fatfs);
	
	f_chdir("/Data");
	open_append(&file, "Timesync.txt");
	
	// last index
	for (i=0; i<TSYNC_STNUM; i++){
	snprintf(line2, LINELEN, "localtime_TS = %lld\n\r", localtime_TS[i]/120); 
	if (f_puts(line2, &file) == 0) {
			lpc_printf("Error writing data config\r\n");
	f_close(&file);
	return FAIL;
	}
	}

	for (i=0; i<TSYNC_STNUM; i++){
	snprintf(line2, LINELEN, "offset = %lld.\n\r", offsetSync[i]/120); 
	if (f_puts(line2, &file) == 0) {
			lpc_printf("Error writing data config\r\n");
	f_close(&file);
	return FAIL;
	}
	}
	
	f_close(&file);
	return SUCCESS;
}

int linreg(int n, int64_t *x, int64_t *y, long double* m, long double* b, long double* r){
	  // y = m*x+b
	  int i;
    long double   sumx = 0.;                      /* sum of x     */
    long double   sumx2 = 0.;                     /* sum of x**2  */
    long double   sumxy = 0.;                     /* sum of x * y */
    long double   sumy = 0.;                      /* sum of y     */
    long double   sumy2 = 0.;                     /* sum of y**2  */
	  long double   denom = 0.;

    for (i=0;i<n;i++){ 
        sumx  += (long double)x[i];       
        sumx2 += ((long double)x[i]) *((long double) x[i]);  
        sumxy += ((long double)x[i]) *((long double) y[i]);
        sumy  += (long double)y[i];      
        sumy2 += ((long double)y[i]) *((long double) y[i]); 
    } 

    denom = (n * sumx2 - sumx*sumx);
    if (denom == 0) {
        // singular matrix. can't solve the problem.
        *m = 0;
        *b = 0;
        if (r) *r = 0;
            return 1;
    }

    *m = (n * sumxy  -  sumx * sumy)*1e8 / denom; // scale by 1e8
    *b = (sumy * sumx2  -  sumx * sumxy) / denom;
    if (r!=NULL) {
        *r = (sumxy - sumx * sumy / n) /    /* compute correlation coeff */
              sqrt((sumx2 - sumx*sumx/n) *
              (sumy2 - sumy*sumy/n));
    }

    return 0; 
}

int compare (const void * a, const void * b)
{
  return *(int64_t*)a > *(int64_t*)b ? 1 : -1;
}

int64_t medium_select(int64_t* dataset, uint16_t num)
{
	qsort (dataset, num, sizeof(uint64_t), compare);
	return dataset[num/2];
}

int64_t mean_calculate(int64_t* dataset, uint16_t num)
{
	int i; 
	int64_t value=0;
	for (i=0; i<num; i++)
		value+=dataset[i];
	return (value/num);
}

void writeSyncTask(void *parameters)
{
	//lpc_printf("writing is started\r\n");
	record_time();
	lpc_printf("writing is finished\r\n");
	TimeSync_Stop();
	vTaskDelete(NULL);
}
	
void Compute_parameter(ts_msg *msg)
{
	if(msg->index==1 && count==TSYNC_OTNUM)
	{
		ClockDrift_setClockDrift(0); // in ticks
		ClockDrift_setClockOffset(offsetSync[0]);  // in ticks
		ClockDrift_setLastSyncTime(localtime_TS[0]);  // in ticks
		lpc_printf("local time is %lld, offset is %lld\r\n", localtime_TS[0], offsetSync[0]);
	}
	
	if (msg->index==TSYNC_STNUM && count==TSYNC_OTNUM)
	{
		slope=(offsetSync[1]-offsetSync[0])*1e9/(localtime_TS[1]-localtime_TS[0]);
		ClockDrift_setClockDrift(slope); // in ticks
		ClockDrift_setClockOffset(offsetSync[1]);  // in ticks
		ClockDrift_setLastSyncTime(localtime_TS[1]);  // in ticks
		lpc_printf("local time is %lld, offset is %lld\r\n", localtime_TS[1], offsetSync[1]);
		lpc_printf("slope is %Lf\r\n", slope);
		//count=1;
		/*
		linreg(TSYNC_STNUM, localtime_TS, offsetSync, &slope, &intersect, &correlation);
		
		lpc_printf("drift is %Lf\r\n", slope);
		lpc_printf("drift is %ld\r\n", (int32_t)slope);
		lpc_printf("offset is %lld\r\n", offsetSync[TSYNC_STNUM-1]); // in ticks
		lpc_printf("lastSync is %lld\r\n", localtime_TS[TSYNC_STNUM-1]); // in ticks
		
		ClockDrift_setClockDrift(slope); // in ticks
		ClockDrift_setClockOffset(offsetSync[TSYNC_STNUM-1]);  // in ticks
		ClockDrift_setLastSyncTime(localtime_TS[TSYNC_STNUM-1]);  // in ticks
		
		vTaskDelay(2000);
		if (xTaskCreate(writeSyncTask, "TSMain", 3 * configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *)NULL) != pdPASS) {
			die();
		}
		*/
		//lpc_printf("at No.%u packet, localtime_TS is %lld, offset is %lld\r\n", msg->index-1, localtime_TS[TSYNC_STNUM-1], offsetSync[TSYNC_STNUM-1]);
	}
}
	
static void callback_tsc_cmd(uint16_t src, void *data, uint8_t len) //executed by receiver
{		
  ts_msg msgs;
	//lpc_printf("received\r\n");
	if (!priorityTask){priorityTask = true; dosync=1;
	}
	
  TimeSync_dat2msg(&msgs,data,MSG_SIZE);
  Update_LocalParameter(&msgs);
	// record the timing pair for subsequent analysis...
	Compute_parameter(&msgs);
}

//-->private functions
 int Creat_TimingMessage(){ //Create Timing message
    tsm.sender=LOCAL_ADDR;
    tsm.receiver=BCAST_ADDR;
    tsm.index++;
    tsm.type=TYPE_TIMING;
    tsm.dummy=0xff;
    tsm.time64=clock; //** this is for gateway node, and we can use the system clock to synchronize.
	  //lpc_printf("index is %u, reference node time: %lld\r\n", tsm.index, tsm.time64);
	  //lpc_printf("index is %u\r\n", tsm.index);

    condLocal.preGTime64=condLocal.curGTime64;
    condLocal.curGTime64=tsm.time64;
    condLocal.curIndex=tsm.index;
    TimeSync_msg2dat(tssbuf,&tsm,MSG_SIZE);
    return tsm.index;
 };

void Update_LocalParameter(ts_msg *msg){ //update local time adjust parameter; note: send TSYNC_STNUM+1 round of packets
	
	int64_t time_offest=0;
	int64_t time_local=0;
	
	configASSERT(msg);
	
	//lpc_printf("gateway clock is %lld\r\n", (msg->time64)/120);
	//lpc_printf("%lld\r\n", clock/120);
	//lpc_printf("%lld\r\n", ((int64_t)clock - (int64_t)(msg->time64))/120);
	//lpc_printf("offset is %lld\r\n", ((msg->time64)-clock)/120);
	
	if (index != (msg->index))
	{
		/*
		if(count==0){//do nothing
		}
		else
		{
			time_offest = medium_select(tempoffset, count+1);
			time_local = mean_calculate(templocal, count+1);
			memset(tempoffset, 0, TSYNC_OTNUM*sizeof(int64_t));
			memset(templocal, 0, TSYNC_OTNUM*sizeof(int64_t));
			
			offsetSync[index-1]=time_offest;
			localtime_TS[index-1]=time_local;
			lpc_printf("at No.%u, offset is: %lld; localtime_TS is %lld\r\n", index, (offsetSync[index-1]), (localtime_TS[index-1]));
		}
		*/
		index = (msg->index);
		count=0;
		//templocal[count] = clock;
		//tempoffset[count] = (msg->time64) - clock;		
	}
	else
	{
		count++;
		if(count-1>=TSYNC_OTNUM){	
			priorityTask = false;
			dosync=0;
			lpc_printf("at No.%u packet, localtime_TS is %lld, offset is %lld\r\n", msg->index-1, localtime_TS[index-1], offsetSync[index-1]);
		}
		else
		{
			//lpc_printf("count is %u\r\n", count);
			
			templocal[count-1] = clock;
			tempoffset[count-1] =  (msg->time64) - clock;
			
			if (count==TSYNC_OTNUM)
			{
				time_offest = medium_select(tempoffset, count);
				time_local = mean_calculate(templocal, count);
				memset(tempoffset, 0, TSYNC_OTNUM*sizeof(int64_t));
				memset(templocal, 0, TSYNC_OTNUM*sizeof(int64_t));
			
				offsetSync[index-1]=time_offest;
				localtime_TS[index-1]=time_local;
				
				//lpc_printf("at No.%u, offset is: %lld; localtime_TS is %lld\r\n", index, (offsetSync[index-1]), (localtime_TS[index-1]));
				//count=0;
			}
		}
	}
};

void Creat_FeedbackMessage(ts_msg *msg){
  msg->receiver=msg->sender;
  msg->sender=LOCAL_ADDR;
  msg->index=condLocal.preIndex;
  msg->type=TYPE_FEEDBACK;
  msg->dummy=0x80;
  msg->time64=condLocal.curLTime64; //**
  TimeSync_msg2dat(tssbuf,msg,MSG_SIZE);
};

//---Bizarre---->
void TestFunction(void){ //test some function here
  uint16_t index=0xeff0;
  uint32_t k=0;
  lpc_printf("MSG_SIZE: %u\r\n", MSG_SIZE);

  for(k=0;k<0xff;k++){
    lpc_printf("index: %u\r\n", ++index);
    vTaskDelay(pdMS_TO_TICKS(500));
   }
  lpc_printf("test finished.\r\n");
};

//Code refer->

//1. GenericComm:
//GenericComm_bcast(GC_APP_TIMESYNC, tssbuf, MSG_SIZE, GC_NOTIMESTAMP); 
//GenericComm_send(GC_APP_TIMESYNC, BCAST_ADDR, tssbuf, MSG_SIZE, GC_NOACK, GC_NOTIMESTAMP); //=broadcast
//GenericComm_send(GC_APP_TIMESYNC, tsm.nodeid, tssbuf, MSG_SIZE, GC_NOACK, GC_NOTIMESTAMP); //p2p comm

//2. Task manage:
//	xTaskCreate(TimingTask, "TimingTask", configMINIMAL_STACK_SIZE, NULL,
//	        (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) &xTaskToNotify_TS);
//  vTaskDelete(xTaskToNotify_TS);

//3. Common codes
//#ifdef GATEWAY //#endif
//for (i = 0; i < MSG_SIZE; ++i){}
//lpc_printf("\r\n- This NodeID is %u,served as gateway.\r\n",  LOCAL_ADDR);


									/*More relax way of sync (RTC)*/
#ifdef FRA // sync clock (RTC DS3231m) functions
#include <SnoozeAlarm.h>
#include <4GFTP.h>
#include <RemoteSensing.h>
extern TaskHandle_t SSMTask;
uint8_t tindex_ftp, tcount_ftp, targets2_ftp[MAX_NODES],counter_ftp = 0;
static uint8_t rcid = 0xff;
#include <sdcard.h>
extern uint16_t adxl_range;
extern uint16_t thres_act; // set activity threshold detection as 200mg (if range is 2g, LSB/g is 1000)
extern uint16_t time_act; // set time for activity detection as 50ms (if ODR is 100, time is 5/100s = 50ms)
extern uint16_t thres_iact; // set activity threshold detection as 200mg (if range is 2g, LSB/g is 1000)
extern uint16_t time_iact; // set time for activity detection as 50ms (if ODR is 100, time is 5/100s = 50ms)
extern uint8_t task1T;
extern uint8_t task2T;
extern uint16_t thres_actOriginal, thres_iactOriginal;
typedef struct { // TUASK: does this need to be < 128?
	uint8_t use4G;
	uint8_t suyear;
	uint8_t sumonth;
	uint8_t sudate;
	uint8_t suhour;
	uint8_t suminute;
	uint8_t susecond;
	uint8_t sutask1time;	
	uint8_t surange;
	uint16_t suthresholdact;
	uint16_t sutimeact;
	uint16_t suthresholdinact;
	uint16_t sutimeinact;
} __attribute__((packed)) sutimetype;

extern uint16_t rsnodes[MAX_NODES];
extern uint8_t rsncnt;
extern uint32_t rsidx;
static sutimetype sutime;
static TaskHandle_t xTaskToNotify = NULL;
static bool syncclock_initialized=false;
static TimerHandle_t tim_sync;
static TimerHandle_t tim_sn = NULL;

static void sctimer(TimerHandle_t pxTimer)
{
	lpc_printf("- sctimer: Resetting\n\r");
#ifdef FRA // for FRA-autnomous, get rid of all the NVIC_SystemReset() and replace with setting alarm clock before sleeping
	NodeReset();
#else	
	NVIC_SystemReset();
#endif
}

static void sctimer_synch(TimerHandle_t pxTimer)
{
	RemoteCommand_stop(rcid);
	if (rcid == RC_CMD_RS_SYNCCLOCK) {
		lpc_printf("- timed out, try once more\r\n" );
	} else {
		lpc_printf("- command timed out\r\n");
	}
	if (xTaskToNotify) {
		xTaskNotifyGive(xTaskToNotify);
	}
}

static int syncclockfunc(void* arg, uint32_t len)
{
	flashconfig_t sensor_rev_fc={0};
	sutimetype *st = (sutimetype *)arg;
	Clock_info *cin=sdcalloc(1,sizeof(Clock_info));
	float chg[2];
	configASSERT(cin);
#ifndef GATEWAY
	app_inactive = false;
#endif
	
	if (len != sizeof (sutimetype)) 
	{
		lpc_printf("- fail to receive correct time data\n\r");
		RemoteCommand_done(RC_CMD_RS_SYNCCLOCK, FAIL, NULL, 0);
		NodeReset(); // Reset right away, no need to stay
		return FAIL;	
	}

	// Adjust clock of sensor node
	cin->second=(int) st->susecond;
	cin->minute=(int) st->suminute;	
	cin->hour=(int) st->suhour;	
	cin->AMPM=FALSE;
	cin->day=7;
	cin->date=(int) st->sudate;	
	cin->month=(int) st->sumonth;
	cin->year=(int) st->suyear;	
	cin->clock24=TRUE;

// RTCS_Init();	
	RTC_SetClock(cin);
	
// Set higher priority for sending task, so it's not interrupted (and will crash) by the incoming messages
	vTaskPrioritySet(SSMTask, tskIDLE_PRIORITY + 2UL);
	
	if (st->use4G == 0) // if no 4G is used, no need to update the other params, move on to the wake state
	{
		return SUCCESS;
	}
// Getting voltage to send back
	CheckBatteryVoltage();
	chg[0] = voltage;
	chg[1] = current;
	PRINTF("	chg[0] = voltage = %f\n\r", voltage);
	PRINTF("	chg[1] = current = %f\n\r", current);	

  sensor_rev_fc.task1time = (uint8_t) st->sutask1time; PRINTF("%d",st->sutask1time);
  sensor_rev_fc.adxlrange = (uint8_t) st->surange; PRINTF("%d",st->surange);
	sensor_rev_fc.adxlthresholdact = (uint16_t) st->suthresholdact; PRINTF("%d",st->suthresholdact);
	sensor_rev_fc.adxltimeact = (uint16_t) st->sutimeact; PRINTF("%d",st->sutimeact);
	sensor_rev_fc.adxlthresholdinact = (uint16_t) st->suthresholdinact; PRINTF("%d",st->suthresholdinact);
	sensor_rev_fc.adxltimeinact = (uint16_t) st->sutimeinact; PRINTF("%d",st->sutimeinact);
	SDCard_ReWriteCN(sensor_rev_fc);
	
	NodeReset(); // TUFIX: Right now, the program will crash in setting timer in SendSMsg, so this nodereset is to prevent that, and let the node sleep
	RemoteCommand_done(RC_CMD_RS_SYNCCLOCK, SUCCESS, chg, 2 * sizeof (float));
//	portYIELD(); // move on to the next step of resetting

	return SUCCESS;
}

static void syncclocksent(uint16_t *targets, uint8_t tcount)
{
	if (tcount == 0) {
		lpc_printf("- node %03u: no response\r\n", rsnodes[(int) rsidx%rsncnt]);
		if (xTaskToNotify) {
			xTaskNotifyGive(xTaskToNotify);
		}
	}
}

static void syncclockresp(int success)
{
	lpc_printf("ERROR: failed to syncasdasd \n\r");
	if (success != SUCCESS) {
		lpc_printf("ERROR: failed to synchronize clock\r\n");
		die();
	}
	NodeReset(); // rightaway
}

extern bool resetdone;

static void syncclockexec(int success, void *retval, uint32_t len)
{
	xTimerStop(tim_sync, portMAX_DELAY);	
	lpc_printf("- node %03u: synchronized\r\n", rsnodes[(int) rsidx%rsncnt]);
	RemoteCommand_stop(rcid);	// to stop the command, and to reset the resetdone, or will get stuck
	if (xTaskToNotify) {
		xTaskNotifyGive(xTaskToNotify);
	}
}

int SyncClock_init(void)
{
	if (syncclock_initialized) 
	{
		return SUCCESS;
	}
	if (RemoteCommand_init() != SUCCESS
	|| RemoteCommand_register(RC_CMD_RS_SYNCCLOCK, false, syncclockfunc, syncclocksent, syncclockresp, syncclockexec) != SUCCESS) // make the return value count // TUFIX: Since the Sensor has issue with setting clock then crash in SMessage, make retval = false (original plan was true)
	{
		return FAIL;
	}
	tim_sn = xTimerCreate("RSTimer", pdMS_TO_TICKS(7*60000), pdFALSE, NULL, sctimer);
	configASSERT(tim_sn);
	xTimerStart(tim_sn, portMAX_DELAY);
	syncclock_initialized = true;
	return SUCCESS;
}

TaskHandle_t MQTTTask = NULL;
TaskHandle_t xTaskToNotifyMQTT = NULL;
extern bool is4Gworking;// is4Gworking and synching4G are slightly similar, but is4Gworking is updated every TCP round, while synching4G is only updated once at the 1st time
extern bool TIM_busy;// synching4G is to bypass the while() loop for checking reply to improve the synch message density - disregard the result of the AT commands.
extern TimerHandle_t timoffReport;


void FreeRTOSWaitms(int waittime) // source: https://www.freertos.org/FreeRTOS_Support_Forum_Archive/May_2019/freertos_How_to_create_a_delay_in_a_task_without_blocking_that_task_f313a3c6e7j.html
{ // TUFIX: hotfix, use this to replace the TIM_Waitms as that one crashed in deleted tasks
	TickType_t currentTick = xTaskGetTickCount();
	while((xTaskGetTickCount() - currentTick) <  pdMS_TO_TICKS(waittime))	{	}
}
extern uint8_t rcid_copy;
bool synching4G = false;
Status SyncClock_start(bool use4G)
{
 	bool nextnode = false;
	int nodetrack = 0;
	rcid_copy = RC_CMD_RS_SYNCCLOCK;
	if (!syncclock_initialized) {
		return ERROR;
	}
	xTaskToNotifyMQTT = xTaskGetCurrentTaskHandle();
	// use task instead of function in here to be able to exit the task and decide if we should continue to use 4G or not ( a little bit different than other functions in 4Gftp.c because the timer does NOT shut down the node)
	if (use4G)
	{
		xTaskCreate((TaskFunction_t) reportMQTT, "MQTTTask", 10 * configMINIMAL_STACK_SIZE, (void *) 0, (tskIDLE_PRIORITY + 3UL), &MQTTTask); 
		if (!is4Gworking)
		{
			xTimerStop(timoffReport,portMAX_DELAY);
			xTimerDelete(timoffReport, portMAX_DELAY);
		}
	}

	// read Gateway clock
	tim_sync = xTimerCreate("RSTimer", pdMS_TO_TICKS(5000), pdTRUE, NULL, sctimer_synch);
	configASSERT(tim_sync);
	lpc_printf("\r\n- sending time to %u sensor nodes... is4Gworking = %d\r\n", rsncnt, is4Gworking);
	rcid = RC_CMD_RS_SYNCCLOCK;

	xTaskToNotify = xTaskGetCurrentTaskHandle();		
	memset(&sutime, 0, sizeof (sutimetype));
	sutime.use4G = (uint8_t) use4G;
	sutime.sutask1time = (uint8_t) task1T;	
	sutime.suthresholdact = (uint16_t) thres_actOriginal;
	sutime.sutimeact = (uint16_t) time_act;
	sutime.suthresholdinact = (uint16_t) thres_iactOriginal;
	sutime.sutimeinact = (uint16_t) time_iact;
	
	for (rsidx = 0; rsidx < 100000; ++rsidx) { //TUFIXDONE: Let the node sleep right after this. DO NOT MAKE IT STAY, stop once all nodes are heard, and do not send again to a node if found it. Fix: Move the part inside GenericComm.c to syncclockexec - cleaner
		nextnode = false;
		resetdone = false;
	
		xTimerChangePeriod(tim_sync, pdMS_TO_TICKS(20), portMAX_DELAY);
		for (i=0;i<counter_ftp;i++)
		{
			if (rsnodes[(int) rsidx%rsncnt] == targets2_ftp[i]) 
			{
				nextnode = true;
			}
		}
		if (nextnode)
		{
			continue; // not sending to same node again and again
		}

		lpc_printf("\r\n- Checking node %03u: Attempt #%d\n\r", rsnodes[(int) rsidx%rsncnt], rsidx);
		
		// Get (GW) non-time data before time data, and write (sensor ndoes) time data before non-time data
		gettime(&sutime.suyear, &sutime.sumonth, &sutime.sudate, &sutime.suhour, &sutime.suminute, &sutime.susecond);
		nodetrack = nodetrack + 1; // use for reporting mqtt, rsidx will be skipped often

		if (RemoteCommand_execute(RC_CMD_RS_SYNCCLOCK, rsnodes + (int) (rsidx%rsncnt), 1, &sutime, sizeof (sutimetype)) != SUCCESS) {
			lpc_printf("- Node %03u is missing.\r\n", rsnodes[(int) rsidx%rsncnt]);
			xTimerStop(tim_sync, portMAX_DELAY);
		} else {
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		}
		// repeatedly create and stop task
		if (((nodetrack%60)==0) && (nodetrack>1)) { // < 3s (need to multiply the timer period) interval since the network will timeout after about 3-5s - check KTCPCFG
			if (is4Gworking)
			{
				RemoteCommand_stop(rcid);
				xTimerStop(tim_sync,portMAX_DELAY);
				reportMQTT(1); // avoid creating and deleting task in here, something crashes the node. (so no timer needed either)
			}
			if (!use4G)
			{
				xTimerDelete(tim_sync, portMAX_DELAY);
				xTaskToNotify = NULL;
				rcid = 0xff;
				return SUCCESS;
			}
		}
		if ((counter_ftp >= rsncnt))  // found all nodes
		{
			if (is4Gworking)
			{
				RemoteCommand_stop(rcid);
				xTimerStop(tim_sync,portMAX_DELAY);
				reportMQTT(1); // avoid creating and deleting task in here, something crashes the node. (so no timer needed either)
				TIM_Waitms(1000); // wait a little bit before shutting down, so that the last messages get sent out
			}
			xTimerDelete(tim_sync, portMAX_DELAY);
			xTaskToNotify = NULL;
			rcid = 0xff;
			return SUCCESS;
		}
		while(!resetdone){} //wait until the radio is finished
		// TUFIX: hotfix: issue: MQTTTask created inside SyncClock_start, and once it is deleted (due to overtime), the 2nd round of RemoteCommand_execute happens right after the ReliableComm: Timer_start of TimerShort without waiting for it to fire, so most of the next steps fails. This is not an issue if the task finishes (no waiting inside) or there is no task at all.. 
	}
	if (is4Gworking)
	{
		xTaskCreate((TaskFunction_t) reportMQTT, "MQTTTask", 3 * configMINIMAL_STACK_SIZE, (void *) 2, (tskIDLE_PRIORITY + 3UL), (TaskHandle_t *)MQTTTask);
		xTimerStop(timoffReport,portMAX_DELAY);
	}
	xTimerDelete(tim_sync, portMAX_DELAY);
	xTaskToNotify = NULL;
	rcid = 0xff;
	rcid_copy = 0xff;
	return ERROR;
}
#endif
