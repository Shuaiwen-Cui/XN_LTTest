#ifndef _RELIABLECOMMIMPL_H
#define _RELIABLECOMMIMPL_H

#include <xnode.h>
#include <timers.h>
#include "GenericComm.h"

// Common

#define RC_ACKWAIT               20
#define RC_INQWAIT               5000
#define RC_BCINQWAIT             5000
#define RC_BCINQWAITRX           8000
#define RC_DWAIT                 200
#define RC_DWAITCONST            15000
#define RC_REPLYINTERVAL         5
#define RC_REPLYWAIT             30
#define RC_ONEPACKET             10
#define RC_CONFWAIT              30

#define SMDDATASIZE              (GC_MAX_PAYLOAD_SIZE - 12)       // uint8_t 114-12=102
#define DATAPACKETSIZE           (GC_MAX_PAYLOAD_SIZE - 4)        // uint8_t 114-4=110
#define NTCDATASIZE              (SMDDATASIZE / 2 - 1)            // uint8_t (114-12)/2-1=51
#define RMSIZE                   16
#define ACKEDNODENUM             128                              // multiple of 4 (SLDI attribute packed)
#define RECCHKSIZE               (8192 / 2)                             // RECCHKSIZE*8*DATAPACKETSIZE: max data size
#define MAX_REC_BUF_SIZE         (RECCHKSIZE * 8 * DATAPACKETSIZE) // 7340032
#define REQINDEXSIZE             (SMDDATASIZE)

enum {
	// flagCom
	TMOUT = 0x1,	// Timeout
	ACKED = 0x2,	// Acknowledged
	SNDER = 0x4,	// Sender
	NORPL = 0x8,	// No need to reply
	RQMIS = 0x10,	// Requesting missing packets
	BSYLD = 0x20,	// Busy SendLData
	BSYSM = 0x20,	// Busy SendSMsg
	BCAST = 0x40,	// BroadCast

	// flagRF
	MSGLK  = 0x1,
	MSGLK2 = 0x2,
	ACKDN  = 0x80,	// Random wait before sending
};

struct NodeInfo {
	uint16_t NodeID[MAX_NODES];	// Node IDs
	uint8_t AckedNode[ACKEDNODENUM];
	uint8_t NodeNum;				// nodes in a community; NodeNum <= 30
} __attribute((packed));
typedef struct NodeInfo NodeInfo;

struct RecentMsg {
	uint8_t ind;
	uint8_t full;
	uint16_t DestAddr[RMSIZE];
	uint16_t MsgID[RMSIZE];
	uint16_t WaitTime[RMSIZE];
} __attribute((packed));
typedef struct RecentMsg RecentMsg;

struct SSMInfo {
	NodeInfo *Nodes;
	uint16_t DestAddr;
	uint8_t MsgID;
	uint8_t Size;
	uint8_t Data[SMDDATASIZE];
	uint8_t id;
} __attribute((packed));
typedef struct SSMInfo SSMInfo;

struct ShortMsg {
	uint16_t DestAddr;
	uint16_t OrigAddr;
	uint8_t Comment;
	uint8_t MsgID;
	uint8_t Others;
	uint8_t Size;
	int8_t Data[SMDDATASIZE];
	uint8_t id;
} __attribute((packed));
typedef struct ShortMsg ShortMsg;

// Long

// OrigAddr is stored in DestAddr; one of OrigAddr and DestAddr is equal to LOCAL_ADDR
struct SLDInfo {
	uint32_t Length;
	uint8_t *Data;
	NodeInfo *Nodes;
	uint16_t DestAddr;
	uint8_t MsgID;
	uint8_t Instruction;
	uint8_t id;
} __attribute((packed));
typedef struct SLDInfo SLDInfo;

struct DataMsg {
	uint16_t Index;
	uint16_t OrigAddr;				// MsgID
	uint8_t Data[DATAPACKETSIZE];
} __attribute((packed));
typedef struct DataMsg DataMsg;

struct NoticeMsg {
	uint32_t DataLength;
	uint16_t DestAddr;
	uint16_t OrigAddr;
	uint16_t Others;
	uint8_t MsgID;
	uint8_t Comment;				// 1: Beginning 2: End 3: Missing Packets
									// 4: Communication Done 5: Report
									// 0xf: Inquiry 0xe: reset
	uint16_t ReqInd[NTCDATASIZE];
} __attribute((packed));
typedef struct NoticeMsg NoticeMsg;

typedef struct {
	uint8_t data[GC_MAX_PAYLOAD_SIZE];
} GC_Msg;

void Timer_start(TimerHandle_t timer, uint32_t t);
void Timer_stop(TimerHandle_t timer);

#endif /* _RELIABLECOMMIMPL_H */
