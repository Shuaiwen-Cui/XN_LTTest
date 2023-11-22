/*******************************************************************************
 * Copyright (c) 2014 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial API and implementation and/or initial documentation
 *******************************************************************************/

// Source: http://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.html#_Toc398718033
#include "MQTTPacket.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <xnode.h>

#if !defined(_WINDOWS)
//	#include <sys/time.h>
//  	#include <sys/socket.h>
	#include <unistd.h>
  	#include <errno.h>
#else
#include <winsock2.h>
#include <ws2tcpip.h>
#define MAXHOSTNAMELEN 256
#define EAGAIN WSAEWOULDBLOCK
#define EINTR WSAEINTR
#define EINPROGRESS WSAEINPROGRESS
#define EWOULDBLOCK WSAEWOULDBLOCK
#define ENOTCONN WSAENOTCONN
#define ECONNRESET WSAECONNRESET
#endif

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

struct Options
{
	char* connection;         /**< connection to system under test. */
	char** haconnections;
	int hacount;
	int verbose;
	int test_no;
} options =
{
	"tcp://m2m.eclipse.org:1883",
	NULL,
	0,
	0,
	0,
};

void usage()
{

}

void getopts(int argc, char** argv)
{
	int count = 1;

	while (count < argc)
	{
		if (strcmp(argv[count], "--test_no") == 0)
		{
			if (++count < argc)
				options.test_no = atoi(argv[count]);
			else
				usage();
		}
		else if (strcmp(argv[count], "--connection") == 0)
		{
			if (++count < argc)
			{
				options.connection = argv[count];
				printf("\nSetting connection to %s\n", options.connection);
			}
			else
				usage();
		}
		else if (strcmp(argv[count], "--haconnections") == 0)
		{
			if (++count < argc)
			{
				char* tok = strtok(argv[count], " ");
				options.hacount = 0;
				options.haconnections = malloc(sizeof(char*) * 5);
				while (tok)
				{
					options.haconnections[options.hacount] = malloc(strlen(tok) + 1);
					strcpy(options.haconnections[options.hacount], tok);
					options.hacount++;
					tok = strtok(NULL, " ");
				}
			}
			else
				usage();
		}
		else if (strcmp(argv[count], "--verbose") == 0)
		{
			options.verbose = 1;
			printf("\nSetting verbose on\n");
		}
		count++;
	}
}


#define LOGA_DEBUG 0
#define LOGA_INFO 1
#include <stdarg.h>
#include <time.h>
//#include <sys/timeb.h>

#if defined(WIN32) || defined(_WINDOWS)
#define mqsleep(A) Sleep(1000*A)
#define START_TIME_TYPE DWORD
static DWORD start_time = 0;
START_TIME_TYPE start_clock(void)
{
	return GetTickCount();
}
#elif defined(AIX)
#define mqsleep sleep
#define START_TIME_TYPE struct timespec
START_TIME_TYPE start_clock(void)
{
	static struct timespec start;
	clock_gettime(CLOCK_REALTIME, &start);
	return start;
}
#else
#define mqsleep sleep
#define START_TIME_TYPE struct timeval
/* TODO - unused - remove? static struct timeval start_time; */

#endif




#define assert1(a, b, c, d, e) myassert(__FILE__, __LINE__, a, b, c, d, e)

int tests = 0;
int failures = 0;
FILE* xml;
char output[3000];
char* cur_output = output;

#define min(a, b) ((a < b) ? a : b)

int checkMQTTStrings(MQTTString a, MQTTString b)
{
	if (!a.lenstring.data)
	{
		a.lenstring.data = a.cstring;
		if (a.cstring)
			a.lenstring.len = strlen(a.cstring);
	}
	if (!b.lenstring.data)
	{
		b.lenstring.data = b.cstring;
		if (b.cstring)
			b.lenstring.len = strlen(b.cstring);
	}
	return memcmp(a.lenstring.data, b.lenstring.data, min(a.lenstring.len, b.lenstring.len)) == 0;
}


int checkConnectPackets(MQTTPacket_connectData* before, MQTTPacket_connectData* after)
{
	int rc = 0;
	int start_failures = failures;

	lpc_printf("struct_ids should be the same: ");
	(memcmp(before->struct_id, after->struct_id, 4) == 0) ? lpc_printf("PASS, struct_id = %.4s\n\r",after->struct_id) : lpc_printf("FAIL\n\r");

	lpc_printf("struct_versions should be the same: ");
	(before->struct_version == after->struct_version) ? lpc_printf("PASS, struct_versions = %d\n\r",after->struct_version):  lpc_printf("FAIL\n\r");

	lpc_printf("MQTT versions should be the same: ");
	(before->MQTTVersion == after->MQTTVersion) ? lpc_printf("PASS, MQTT versions = %d\n\r",after->MQTTVersion) : lpc_printf("FAIL\n\r");

	lpc_printf("ClientIDs should be the same: ");
	(checkMQTTStrings(before->clientID, after->clientID)) ? lpc_printf("PASS\n\r") : lpc_printf("FAIL\n\r");

	lpc_printf("keepAliveIntervals should be the same: ");
	(before->keepAliveInterval == after->keepAliveInterval) ? lpc_printf("PASS, keepAliveIntervals = %d\n\r",after->keepAliveInterval) : lpc_printf("FAIL\n\r"); 

	lpc_printf("cleansessions should be the same: ");
	(before->cleansession == after->cleansession) ? lpc_printf("PASS, cleansessions = %d\n\r",after->cleansession) : lpc_printf("FAIL\n\r"); 

	lpc_printf("willFlags should be the same: ");
	(before->willFlag == after->willFlag) ? lpc_printf("PASS, willFlag = %d\n\r",after->willFlag) : lpc_printf("FAIL\n\r"); 

	if (before->willFlag)
	{
		lpc_printf("will struct_ids should be the same: ");
		(memcmp(before->will.struct_id, after->will.struct_id, 4) == 0)? lpc_printf("PASS, struct_ids = %.4s\n\r",after->will.struct_id) : lpc_printf("FAIL\n\r");

		lpc_printf("will struct_versions should be the same: ");
		(before->will.struct_version == after->will.struct_version) ? lpc_printf("PASS, will struct_versions = %d\n\r",after->will.struct_version):  lpc_printf("FAIL\n\r");

		lpc_printf("topic names should be the same: ");
		(checkMQTTStrings(before->will.topicName, after->will.topicName)) ? lpc_printf("PASS\n\r") : lpc_printf("FAIL\n\r");

		lpc_printf("messages should be the same: ");
		(checkMQTTStrings(before->will.message, after->will.message))? lpc_printf("PASS\n\r") : lpc_printf("FAIL\n\r");

		lpc_printf("retained flags should be the same: ");
		(before->will.retained == after->will.retained)? lpc_printf("PASS, retained flags = %d\n\r",after->will.retained) : lpc_printf("FAIL\n\r"); 

		lpc_printf("will qos should be the same: ");
		(before->will.qos == after->will.qos) ? lpc_printf("PASS, will qos = %d\n\r",after->will.qos) : lpc_printf("FAIL\n\r"); 
	}

	lpc_printf("usernames should be the same: ");
	(checkMQTTStrings(before->clientID, after->clientID)) ? lpc_printf("PASS\n\r") : lpc_printf("FAIL\n\r");
	lpc_printf("passwords should be the same: ");
	(checkMQTTStrings(before->password, after->password)) ? lpc_printf("PASS\n\r") : lpc_printf("FAIL\n\r");
	return failures == start_failures;
}

int testMQTT(struct Options options) //1st simple test for MQTT, more to come if dev process shows some diaparity between the encode and decode process
{
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	MQTTPacket_connectData data_after = MQTTPacket_connectData_initializer;
	
	int rc = 0;
	unsigned char buf[100];
	int buflen = sizeof(buf);
	
	lpc_printf("testMQTT\r\n");
	failures = 0;

	data.clientID.cstring = "me";

	data.keepAliveInterval = 20;
	data.cleansession = 1;
	data.username.cstring = "testuser";
	data.password.cstring = "testpassword";

	data.willFlag = 1;
	data.will.message.cstring = "will message";
	data.will.qos = 1;
	data.will.retained = 0;
	data.will.topicName.cstring = "will topic";

	rc = MQTTSerialize_connect(buf, buflen, &data);
	if (rc > 0)
		lpc_printf("good rc from serialize connect, serialized length was %d\r\n", rc);
	
	rc = MQTTDeserialize_connect(&data_after, buf, buflen);
	if (rc == 1)
		lpc_printf("good rc from deserialize connect\r\n");

	/* data after should be the same as data before */
	rc = checkConnectPackets(&data, &data_after);
	if (rc == 1)
		lpc_printf("PASS\r\n", rc);
	
	return failures;
}

