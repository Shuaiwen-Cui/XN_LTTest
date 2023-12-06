#ifndef _SENSING_H 
#define _SENSING_H

#include <lpc_types.h>

#define MAX_SENSOR_CHANNELS			8
#define MAX_ACCEL_CHANNELS			3
#define RAW_DATA_RATE           1000

// Priorities at which the tasks are created.
#define mainQUEUE_SENSING_TASK_PRIORITY		    (tskIDLE_PRIORITY + 2)

static const uint8_t channtype[8] = {0,0,0,4,4,4,4,4};  // 1st 3 are acceleration (unchanged) and last 5 are Voltage

typedef enum {
	XNODEACCEL = 0,								/* onboard accel */
	STRAIN = 1,										/* attached to strain circuit */
	INTERNALTEMP = 2,							/* ADC internal temperature sensor */
	NOISEFLOOR = 3,								/* ADC channel noise floor */
	EXTERNAL = 4,									/* other external sensor */
	OFF = 5												/* channel turned off */
} chType;

static const char *SensorUnit[6] = {
	"mg",
	"uS",
	"C",
	"V",
	"V",
	"N/A"	
};

typedef struct {
	uint32_t sampSize;					/* number of elements in 'sampData' */
	float *sampData;					  /* resampled ADC data in 32-bit float format */
	uint16_t samplingRate;			/* sampling rate (Hz) */
	chType type;								/* channel/sensor type */
} ChannelData;

typedef struct {
	ChannelData channels[MAX_SENSOR_CHANNELS];
	uint16_t dataRate;					/* output data rate (sps) */
	uint16_t nodeID;
	float voltage;
	float current;
	int64_t timestamp;  				// timestampe of 1st data sample
	uint8_t year;
	uint8_t month;
	uint8_t date;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
} SensorData;

typedef void (*dataReady)(SensorData*);

SensorData* setupAcquisition(uint16_t dataRate, uint32_t sampleTime, uint64_t clock);
Status setupChannel(SensorData *data, uint8_t channelNum, uint16_t samplingRate, chType sensorType);
Status startAcquisition(SensorData *data, dataReady func);

#endif /* _SENSING_H */
