#ifndef GLOBALCONSTANTS_H
#define GLOBALCONSTANTS_H

#include <stdint.h>
#include <lpc_types.h> // include here to avoid the overlapping of definition


#ifndef SUCCESS
#define SUCCESS                 0
#endif
#ifndef FAIL
#define FAIL                    1
#endif
// TUFIX: COmment out, move to lpc_types.h

#define MAX_NODES               48

extern uint16_t LOCAL_ADDR;
extern uint8_t RADIO_CHANNEL;
extern uint8_t RADIO_POWER;

#ifdef FRA // add sync clock (DS3231m) information
// GenericComm app id's
enum {
	GC_APP_RC_SMSG = 0,
	GC_APP_RC_SMSG_MCAST = 1,
	GC_APP_RC_LDATA_ACK = 2,
	GC_APP_RC_LDATA_NOTICE = 3,
	GC_APP_RC_LDATA_DATA = 4,
	GC_APP_SNOOZEALARM = 5,
	GC_APP_TESTRADIO = 6,
	GC_APP_TIMESYNC =  7, //add by Jinbao
	GC_APP_SYNCCLOCK =  8,
	// add new apps above
	GC_MAX_APPS = 9
};
#else
// GenericComm app id's
enum {
	GC_APP_RC_SMSG = 0,
	GC_APP_RC_SMSG_MCAST = 1,
	GC_APP_RC_LDATA_ACK = 2,
	GC_APP_RC_LDATA_NOTICE = 3,
	GC_APP_RC_LDATA_DATA = 4,
	GC_APP_SNOOZEALARM = 5,
	GC_APP_TESTRADIO = 6,
	GC_APP_TIMESYNC=  7, //add by Jinbao
	// add new apps above
	GC_MAX_APPS = 8
};
#endif

// ReliableComm app id's
enum {
	RC_APP_REMOTECOMMAND = 0,
	RC_APP_LDATA_UCAST = 1,
	RC_APP_TESTRADIO = 2,
	// add new apps above
	RC_MAX_APPS = 3
};


// RemoteCommand command id's
enum {
	RC_CMD_UTIL_VOLTAGE = 0,
	RC_CMD_UTIL_RESET = 1,
	RC_CMD_RS_SETPARAMS = 2,
	RC_CMD_RS_GETDATA = 3,
	RC_CMD_TESTRADIO = 4,
	RC_CMD_TRIGSEN = 5,
	RC_CMD_DR_PRESET = 6,
	RC_CMD_DR_RETRIEVE = 7,
	RC_CMD_XNODECFG = 8,
	RC_CMD_RS_SYNCCLOCK = 9,

	// add new commands above
	RC_MAX_CMDS = 10
};

#endif /* GLOBALCONSTANTS_H */
