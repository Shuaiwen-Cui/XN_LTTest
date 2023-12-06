#include "timesync.h"
#include "SysTime.h"
#include "LocalTime.h"
#include "GlobalTime.h"
#include <ff.h>
#include <math.h> 
#include <stdio.h>
#include <stdlib.h>

//--> type define
enum {
  TYPE_TIMING =   0,
  TYPE_FEEDBACK = 1,
};

#define LINELEN               80
static char line2[LINELEN];
extern FATFS Fatfs;
static int TSYNC_STNUM;
extern uint8_t TrigSenIdx;
static bool initialized = false;

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

typedef struct ts_msg //timesync message
{
	uint16_t sender;   //leaf nodeID
	uint16_t receiver; //gateway ID
	uint16_t index;    //index count
	uint16_t index_i;  //Reserved
	uint8_t  type;     //message type
	uint8_t  delay;    //total delay	
  uint16_t gatewayID;     //gateway ID (trigger sensing)
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

int64_t offsetSync[TSYNC_STNUM_max]={0};//int64_t offsetSync[TSYNC_STNUM] = {0};
int64_t localtime[TSYNC_STNUM_max]={0};//int64_t localtime[TSYNC_STNUM] = {0};
int64_t *tempoffset; // to store the data set of offsets;
int64_t *templocal; // to store the data set of offsets;
int64_t *tempglobal;
uint64_t clock = 0;
uint16_t index = 0, index_i=0, count=0;
int dosync=0;
long double slope = 0;
long double intersect = 0;
long double correlation = 0;
extern uint16_t gatewayID;

//#ifdef GATEWAY
//volatile static  TaskHandle_t xTaskToNotify_TS = NULL; //task handle
//#endif

//<--------------------

//--> Private Function Declaration
void init_TimeSync_Buffer(void);
void init_NodeStatus(void);
int Creat_TimingMessage(void);
void Creat_FeedbackMessage(ts_msg *msg);
#ifndef GATEWAY	
	static bool priorityTask = false;
	void Update_LocalParameter(ts_msg *msg);
	void callback_tsc_cmd(uint16_t srcid, void *data, uint8_t len);//gencomm callback function
#endif

//-->Public function define
void TimeSync_GetConfig(void)
{
#ifdef GATEWAY
  gwc.cnt = 1; //number of leaf nodes.
  gwc.nodeid[0]=(uint16_t)0x2; //NodeID of leaf node
  gwc.gateway = (uint16_t)0x1; //NodeID of gateway
#endif
  tsm.index=0;
  //tsm.dummy=0xff;
  tsm.delay=0;
  tsm.type=TYPE_TIMING;
#ifdef GATEWAY	
  gwc.isTiming=FALSE;
#endif
}

int TimeSync_Register(void) //please check "lib\system\GlobalConstants.h" for GC_APP_TIMESYNC define.
{
  LED_RGB(0,1,0);
  init_TimeSync_Buffer();
  TimeSync_GetConfig();
  init_NodeStatus();
#ifndef GATEWAY	
	GenericComm_init();
  GenericComm_register(GC_APP_TIMESYNC, callback_tsc_cmd);
#endif
	//dosync=1;
	return SUCCESS;
}

int Timesync_Init(void){

	if (initialized) {
		return SUCCESS;
	}
	RealTimer_init();
	NVIC_EnableIRQ(RITIMER_IRQn);
	
#ifdef GATEWAY	
  gwc.isCounterOk=TRUE;
	TSYNC_STNUM = 30;
#else	
	if(!TrigSenIdx) // normal sensing, set 2
		TSYNC_STNUM=2;
	else
		TSYNC_STNUM=30;	
	
	lpc_printf("TSYNC_STNUM is %u\r\n", TSYNC_STNUM);
	
#endif
	TimeSync_Register();	
	initialized = true;
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
	//localtime = (int64_t *)sdcalloc(TSYNC_STNUM, sizeof(int64_t));
	tempoffset = (int64_t *)sdcalloc(TSYNC_OTNUM, sizeof(int64_t));
	templocal = (int64_t *)sdcalloc(TSYNC_OTNUM, sizeof(int64_t));
	tempglobal = (int64_t *)sdcalloc(TSYNC_OTNUM+1, sizeof(int64_t));
	configASSERT(tempoffset && templocal && tempglobal);
}

//-->public functions
  int TimeSync_Start(void){
    //gwc.isTiming=TRUE;
		//gwc.ts = xTimerCreate("Timer_timing", pdMS_TO_TICKS(TIMING_PERIOD*1000), pdTRUE, NULL, callback_timing_timer); //gateway timing timer
    //configASSERT(gwc.ts); xTimerStart(gwc.ts, portMAX_DELAY);		
    uint16_t i;
    
		dosync=1;
    index = Creat_TimingMessage(); //creat timing signal
		
		/*
    if (index>TSYNC_STNUM)
				TimeSync_Stop();
		*/
		for (i=1; i<31; i++)
		{
			memcpy(tssbuf+12, &clock, sizeof(uint64_t));
			memcpy(tssbuf+6, &i, sizeof(uint16_t));
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
		//sdfree(offsetSync);
		//sdfree(localtime);
		sdfree(tempoffset);
		sdfree(templocal);
		sdfree(tempglobal);
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
	snprintf(line2, LINELEN, "localtime = %lld\n\r", localtime[i]/120); 
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

    *m = (n * sumxy  -  sumx * sumy)*1e9 / denom; // scale by 1e9
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
		ClockDrift_setLastSyncTime(localtime[0]);  // in ticks
		lpc_printf("local time is %lld, offset is %lld\r\n", localtime[0], offsetSync[0]);
	}
	
	if (msg->index==TSYNC_STNUM && count==TSYNC_OTNUM)
	{
		if(!TrigSenIdx){
			slope=(offsetSync[1]-offsetSync[0])*1e9/(localtime[1]-localtime[0]);
			ClockDrift_setClockDrift(slope); // in ticks
			ClockDrift_setClockOffset(offsetSync[1]);  // in ticks
			ClockDrift_setLastSyncTime(localtime[1]);  // in ticks
			lpc_printf("local time is %lld, offset is %lld\r\n", localtime[1], offsetSync[1]);
			lpc_printf("slope is %Lf\r\n", slope);
		}
		else{
			linreg(TSYNC_STNUM, localtime, offsetSync, &slope, &intersect, &correlation);
			//lpc_printf("drift is %Lf\r\n", slope);
			//lpc_printf("drift is %ld\r\n", (int32_t)slope);
			//lpc_printf("offset is %lld\r\n", offsetSync[TSYNC_STNUM-1]); // in ticks
			//lpc_printf("lastSync is %lld\r\n", localtime[TSYNC_STNUM-1]); // in ticks
			ClockDrift_setClockDrift(slope); // in ticks
			ClockDrift_setClockOffset(offsetSync[0]);  // in ticks
			ClockDrift_setLastSyncTime(localtime[0]);  // in ticks
		}
		/*
		vTaskDelay(2000);
		if (xTaskCreate(writeSyncTask, "TSMain", 3 * configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *)NULL) != pdPASS) {
			die();
		}
		*/
		//lpc_printf("at No.%u packet, localtime is %lld, offset is %lld\r\n", msg->index-1, localtime[TSYNC_STNUM-1], offsetSync[TSYNC_STNUM-1]);
	}
}

#ifndef GATEWAY	
static void callback_tsc_cmd(uint16_t src, void *data, uint8_t len) //executed by receiver
{		
  ts_msg msgs;
	//lpc_printf("received\r\n");
	
	TimeSync_dat2msg(&msgs,data,MSG_SIZE);
	lpc_printf("Sender is %u, Local addr is %u, msgs.gatewayID is %u, gatewayID is %u\r\n", msgs.sender, LOCAL_ADDR, msgs.gatewayID, gatewayID);
	if(TrigSenIdx && (msgs.gatewayID != gatewayID)){}
	else{
		lpc_printf("process TS message\r\n");
		LED_RGB(1,0,1);
		if (!priorityTask){priorityTask = true; dosync=1;
		}  
		Update_LocalParameter(&msgs);
		// record the timing pair for subsequent analysis...
		Compute_parameter(&msgs);
	}
}

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
			localtime[index-1]=time_local;
			lpc_printf("at No.%u, offset is: %lld; localtime is %lld\r\n", index, (offsetSync[index-1]), (localtime[index-1]));
		}
		*/
		index = (msg->index);
		count=0;
		//templocal[count] = clock;
		//tempoffset[count] = (msg->time64) - clock;		
	}
	else
	{	
		if (msg->index_i == index_i+1) {
		count++;
		if(count-1>=TSYNC_OTNUM){	
			priorityTask = false;
			dosync=0;
			lpc_printf("at No.%u packet, localtime is %lld, offset is %lld\r\n", msg->index-1, localtime[index-1], offsetSync[index-1]);
		}
		else
		{	
			//lpc_printf("count is %u\r\n", count);
			
			templocal[count-1] = clock;
			tempglobal[count-1] = (msg->time64);
			if(count>=2)
				tempoffset[count-2] =  tempglobal[count-1] - templocal[count-2];
			
			if (count==TSYNC_OTNUM)
			{
				time_offest = medium_select(tempoffset, count-1);
				time_local = mean_calculate(templocal, count-1);
				memset(tempoffset, 0, TSYNC_OTNUM*sizeof(int64_t));
				memset(templocal, 0, TSYNC_OTNUM*sizeof(int64_t));
			
				offsetSync[index-1]=time_offest;
				localtime[index-1]=time_local;
				
				//lpc_printf("at No.%u, offset is: %lld; localtime is %lld\r\n", index, (offsetSync[index-1]), (localtime[index-1]));
				//count=0;
			}
		}
		}
		else
			lpc_printf("missing\r\n");
	}
	index_i = msg->index_i;
};

#endif

//-->private functions
 int Creat_TimingMessage(){ //Create Timing message
    tsm.sender=LOCAL_ADDR;
    tsm.receiver=BCAST_ADDR;
    tsm.index++;
    tsm.type=TYPE_TIMING;
    if(TrigSenIdx){
			tsm.gatewayID = gatewayID; lpc_printf("gateway ID is %u\r\n",tsm.gatewayID);}
    tsm.time64=clock; //** this is for gateway node, and we can use the system clock to synchronize.
	  //lpc_printf("index is %u, reference node time: %lld\r\n", tsm.index, tsm.time64);
	  //lpc_printf("index is %u\r\n", tsm.index);

    condLocal.preGTime64=condLocal.curGTime64;
    condLocal.curGTime64=tsm.time64;
    condLocal.curIndex=tsm.index;
    TimeSync_msg2dat(tssbuf,&tsm,MSG_SIZE);
    return tsm.index;
};

void Creat_FeedbackMessage(ts_msg *msg){
  msg->receiver=msg->sender;
  msg->sender=LOCAL_ADDR;
  msg->index=condLocal.preIndex;
  msg->type=TYPE_FEEDBACK;
  //msg->dummy=0x80;
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

