/*========================================================================
Head:   timesync.h
source: timesync.C
========================================================================
*/

#ifndef _TIMESYNC_H
  #define _TIMESYNC_H
//**
	#include <xnode.h>
	#include <lpc43xx_rit.h>
	#include <stdio.h>
	#include <stdlib.h>
	#include <string.h>
	#include "lpc43xx_timer.h"
	#include "math.h"
	#include <assert.h>
	#include <stddef.h>
  #include "FreeRTOS.h"
  #include "timers.h"
//**
  #include <GenericComm.h>
  #include <ReliableComm.h>
  #include <RemoteCommand.h>
  #include <Utils.h> 
//**

//--> Global constant definition
#define TIMING_PERIOD 	 (1)   //synchornize period(unit:s)
#define TSYNC_OTNUM    (6)   // number of points for finding medium value of offset
#define TSYNC_STNUM  (2)   // number of points for secant regression

//--> Public function declaration
int Timesync_Init(void);
int TimeSync_Start(void);
void TimeSync_Stop(void);
int SyncClock_init(void);
Status SyncClock_start(bool use4G);

#endif // _TIMESYNC_H
