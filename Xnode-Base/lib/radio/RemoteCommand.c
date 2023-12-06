#include <xnode.h>
#include <string.h>
#include "ReliableComm.h"
#include "RemoteCommand.h"

#define _DEBUG_               0
#if _DEBUG_
#define PRINTF(...)           lpc_printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

 struct cmdmsghdr {
	uint16_t adrs;
	uint8_t cmd;
	uint8_t res;
	uint8_t resp;
	uint8_t filler[3];
	uint32_t len;
} __attribute__((packed));
typedef struct cmdmsghdr cmdmsghdr;
	
struct cmdmsg {
	cmdmsghdr hdr;
	uint8_t data[1];
} __attribute__((packed));
typedef struct cmdmsg cmdmsg;

typedef struct {
	bool remote;
	bool inprogress;
	bool retval;
	rc_cmdfunc_t cmd;
	rc_cmdsentcb_t sent;
	rc_respsentcb_t resp;
	rc_cmdexeccb_t exec;
	cmdmsg *cmdm;
} cmdstruct;

static bool initialized = false;
static cmdstruct cmds[RC_MAX_CMDS];
static uint8_t tindex, tcount;
static uint16_t targets[MAX_NODES], targets2[MAX_NODES];

static void clearState(uint8_t id)
{
	cmds[id].remote = true;
	cmds[id].inprogress = false;
	if (cmds[id].cmdm) {
		sdfree(cmds[id].cmdm);
		cmds[id].cmdm = NULL;
	}
}

static int swapMsgs(uint8_t id, void *retval, uint32_t len)
{
	cmdmsg *tmpmsg = sdmalloc(sizeof (cmdmsghdr) + len);
	configASSERT(tmpmsg);
	memcpy(tmpmsg, cmds[id].cmdm, sizeof (cmdmsghdr));
	memcpy(tmpmsg->data, retval, len);
	tmpmsg->hdr.len = len;
	sdfree(cmds[id].cmdm);
	cmds[id].cmdm = tmpmsg;
	return SUCCESS;
}

static void executeCmd(uint8_t id)
{
	// execute command
	PRINTF("RemoteCommand[%u]: executing command...\r\n", id);
	cmds[id].inprogress = true;
	PRINTF("cmds[id].inprogress = %d\r\n", cmds[id].inprogress);

	if (cmds[id].cmd(cmds[id].cmdm->data, cmds[id].cmdm->hdr.len) != SUCCESS) {
		if (RemoteCommand_done(id, FAIL, NULL, 0) != SUCCESS) {
			clearState(id);
		}
	}
	PRINTF("cmds[id].inprogress = %d\r\n", cmds[id].inprogress);

}

static void executionDone(uint8_t id)
{
	// return data
	PRINTF("RemoteCommand[%u]: command %s.\r\n", id, cmds[id].cmdm->hdr.res == SUCCESS ? "executed" : "failed");
	if (cmds[id].exec) {
		cmds[id].exec(cmds[id].cmdm->hdr.res, cmds[id].cmdm->data, cmds[id].cmdm->hdr.len);
	}
	clearState(id);
}

static void rcSend(uint32_t id)
{
	uint8_t *rcdata = (uint8_t *)cmds[id].cmdm;
	uint32_t rclen = sizeof (cmdmsghdr) + cmds[id].cmdm->hdr.len;
	int i;

	if (cmds[id].remote) {
		targets[0] = cmds[id].cmdm->hdr.adrs;
		tcount = 1;
	}
	cmds[id].cmdm->hdr.adrs = LOCAL_ADDR;
#if 1
	for (i = 0; i < 20; ++i) {
		if (ReliableComm_send(RC_APP_REMOTECOMMAND, targets, tcount, rcdata, rclen) != SUCCESS){ portYIELD(); } else { break; }
	}
#else
	while (ReliableComm_send(RC_APP_REMOTECOMMAND, targets, tcount, rcdata, rclen) != SUCCESS) { portYIELD(); }
#endif
}

void ReliableComm_sendDone(uint16_t *addrs, uint8_t addrlen, uint8_t *data, int success)
{
	uint8_t i;
	uint8_t id = ((cmdmsg *)data)->hdr.cmd;
	if (!initialized) {
		return;
	}
	if (!cmds[id].inprogress) {
		PRINTF("RemoteCommand[%u]: command no longer in progress, ignoring sendDone.\r\n", id);
		return;
	}
	// remote: response sent
	if (cmds[id].remote) {
		PRINTF("RemoteCommand[%u]: %s.\r\n", id, success == SUCCESS ? "response sent" : "failed to send response");
		if (cmds[id].resp) {
			cmds[id].resp(SUCCESS);
		}
		clearState(id);
		return;
	}
	// local: request sent
	if (success == SUCCESS) {
		memcpy(targets2, addrs, addrlen * sizeof (uint16_t));
		tindex = addrlen;
	} else {
		tindex = 0;
	}
	if (tindex == 0) {
		PRINTF("RemoteCommand[%u]: send failed.\r\n", id);
	} else {
		PRINTF("RemoteCommand[%u]: successfully sent to %u of %u node(s): ", id, tindex, tcount);
		for (i = 0; i < tindex; i++) {
			PRINTF("%u ", targets2[i]);
		}
		PRINTF("\r\n");
	}
	if (cmds[id].sent) {
		cmds[id].sent(targets2, tindex);
	}
	if (tindex == 0) {
		clearState(id);
	} else if (!cmds[id].retval) {
		if (cmds[id].exec) {
			cmds[id].exec(SUCCESS, NULL, 0);
		}
		clearState(id);
	} else {
		memcpy(targets, targets2, tindex * sizeof (uint16_t));
		tcount = tindex;
	}
}

void ReliableComm_receive(uint16_t addr, uint16_t *addrs, uint8_t addrlen, uint8_t *data, uint32_t len, int success)
{
	cmdmsg *m = (cmdmsg *)data;
	uint8_t id;
	if (!initialized) {
		return;
	}
	if (success != SUCCESS) {
		PRINTF("RemoteCommand[%u]: receive failed.\r\n");
		return;
	}
	if (len < sizeof (cmdmsghdr) || m->hdr.cmd >= RC_MAX_CMDS || cmds[m->hdr.cmd].cmd == NULL || len != sizeof (cmdmsghdr) + m->hdr.len) {
		PRINTF("RemoteCommand[%u]: receive invalid message.\r\n");
		return;
	}
	id = m->hdr.cmd;
	if (cmds[id].remote) {
		// execute command
		if (cmds[id].cmd != NULL && cmds[id].inprogress) {
			PRINTF("RemoteCommand[%u]: ERROR: ReliableComm_receive execution already in progress\r\n", id);
			return;
		} else if (m->hdr.resp) {
			return;
		}
		cmds[id].cmdm = sdmalloc(len);
		configASSERT(cmds[id].cmdm);
		memcpy(cmds[id].cmdm, m, len);
		executeCmd(id);
	} else {
		// return data
		if (cmds[id].cmd == NULL || !cmds[id].inprogress || !m->hdr.resp) {
			return;
		}
		if (cmds[id].cmdm == NULL || cmds[id].cmdm->hdr.len < m->hdr.len) {
			if (cmds[id].cmdm != NULL) {
				sdfree(cmds[id].cmdm);
			}
			cmds[id].cmdm = sdmalloc(len);
			configASSERT(cmds[id].cmdm);
		}
		memcpy(cmds[id].cmdm, m, len);
		executionDone(id);
	}
}

int RemoteCommand_init(void)
{
	if (initialized) {
		return SUCCESS;
	}
	PRINTF("RemoteCommand: init\r\n");
	if (ReliableComm_init() != SUCCESS
	|| ReliableComm_register(RC_APP_REMOTECOMMAND, ReliableComm_sendDone, ReliableComm_receive) != SUCCESS) {
		return FAIL;
	}
	memset(cmds, 0, sizeof (cmds));
	initialized = true;
	return SUCCESS;
}

int RemoteCommand_register(uint8_t id, bool retval, rc_cmdfunc_t cmd, rc_cmdsentcb_t sent, rc_respsentcb_t resp, rc_cmdexeccb_t exec)
{
	PRINTF("RemoteCommand: register cmd %u\r\n", id);
	if (!initialized || !cmd || id >= RC_MAX_CMDS) {
		return FAIL;
	}
	cmds[id].remote = true;
	cmds[id].inprogress = false;
	cmds[id].retval = retval;
	cmds[id].cmd = cmd;
	cmds[id].sent = sent;
	cmds[id].resp = resp;
	cmds[id].exec = exec;
	return SUCCESS;
}

int RemoteCommand_unregister(uint8_t id)
{
	PRINTF("RemoteCommand: unregister cmd %u\r\n", id);
	if (!initialized || id >= RC_MAX_CMDS) {
		return FAIL;
	}
	if (cmds[id].inprogress) {
		PRINTF("RemoteCommand[%u]: ERROR: RemoteCommand_register execution already in progress\r\n", id);
		return FAIL;
	}
	cmds[id].cmd = NULL;
	return SUCCESS;
}

int RemoteCommand_stop(uint8_t id)
{
	PRINTF("RemoteCommand[%u]: stop\r\n", id);
	if (cmds[id].cmd != NULL && cmds[id].inprogress) {
		PRINTF("RemoteCommand[%u]: command stopped\r\n", id);
		ReliableComm_reset();
		clearState(id);
	}
	return SUCCESS;
}

// caller

int RemoteCommand_execute(uint8_t id, uint16_t *tgts, uint8_t tcnt, const void *args, uint32_t arglen)
{
	PRINTF("RemoteCommand[%u]: execute\r\n", id);
	if (!initialized || !tgts || !tcnt || !cmds[id].cmd) {
		return FAIL;
	}
	if (cmds[id].inprogress) {
		PRINTF("RemoteCommand[%u]: ERROR: execution already in progress\r\n", id);
		return FAIL;
	}
	cmds[id].cmdm = sdmalloc(sizeof (cmdmsghdr) + arglen);
	configASSERT(cmds[id].cmdm);
	if (tgts != targets && tgts != NULL) {
		memcpy(targets, tgts, tcnt * sizeof (uint16_t));
	}
	tcount = tcnt;
	cmds[id].cmdm->hdr.cmd = id;
	cmds[id].cmdm->hdr.len = arglen;
	cmds[id].cmdm->hdr.resp = false;
	memcpy(cmds[id].cmdm->data, args, arglen);
	cmds[id].inprogress = true;
	cmds[id].remote = false;
	tindex = 0;
	rcSend(id);
	return SUCCESS;
}

// callee

int RemoteCommand_done(uint8_t id, int res, void *retval, uint32_t len)
{
	if (!initialized || !cmds[id].cmd) {
		return FAIL;
	}
	PRINTF("RemoteCommand[%u]: done\r\n", id);
	if (!cmds[id].inprogress) {
		PRINTF("RemoteCommand[%u]: ERROR: command not in progress\r\n", id);
		return FAIL;
	}
	if (!cmds[id].retval) {
		if (cmds[id].resp) {
			cmds[id].resp(SUCCESS);
		}
		clearState(id);
		return SUCCESS;
	}
	cmds[id].cmdm->hdr.res = res;
	cmds[id].cmdm->hdr.resp = true;
	if (res != SUCCESS || retval == NULL || len == 0) {
		PRINTF("RemoteCommand[%u]: command failed.\r\n", id);
		cmds[id].cmdm->hdr.len = 0;
	} else {
		PRINTF("RemoteCommand[%u]: command executed.\r\n", id);
		if (swapMsgs(id, retval, len) != SUCCESS) {
			return FAIL;
		}
	}
	rcSend(id);
	return SUCCESS;
}

