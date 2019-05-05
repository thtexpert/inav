#include "system_identification.h"

#if defined(USE_SYSTEM_IDENT)

#include "config/config_reset.h"
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "common/utils.h"
#include "common/memory.h"

#include "fc/config.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "drivers/logging.h"
#include "drivers/light_led.h"

#include "sensors/gyro.h"

PG_REGISTER_WITH_RESET_TEMPLATE(sysid_t, systemIdentification, PG_SYSTEM_IDENT, 0);

PG_RESET_TEMPLATE(sysid_t, systemIdentification,
		.axis =  0,  	//
		.order = 8,  	// max order 9
		.denum = 8,	// run every loop
		.level = 15	// level of stimulus (+/-) full throttle would be 500;
);

typedef enum SysIdState {
    SYSID_STATE_RESET = 0,
    SYSID_STATE_IDLE,
	SYSID_STATE_WAITFORSTART,
	SYSID_STATE_PRESAMPLES,
	SYSID_STATE_SAMPLES,
	SYSID_STATE_FINISHED
} SysIdState;

static SysIdState sysIdState = SYSID_STATE_RESET;

static int32_t sysidTimer = 0;
static int16_t numOfSamples = 0;
static uint16_t readAddressPointer = 0;

#define PRESAMPLES 64
#define MAXSAMPLES 511

typedef struct sysid_data_s {
    int32_t capture[MAXSAMPLES];
    uint16_t stimulus[MAXSAMPLES];
} sysid_data_t;

static sysid_data_t sysIdData;

void fillPrbsStimulus(uint8_t order);

int32_t encodeCaptureData(uint16_t samplecounter, float gyroRate)
{
	return ((int32_t) gyroRate) * 2 + sysIdData.stimulus[samplecounter];

}

float calculateErrorInject(uint16_t samplecounter){
	return (2.0 * (float)systemIdentification()->level) * ((float)sysIdData.stimulus[samplecounter] - 0.5);
}

void sysIdInitialize()
{
	// allocate memory for stimulus
	// fill stimulus data
	// allocate memory for capture
	// default state machine
	sysIdState = SYSID_STATE_RESET;
	sysidTimer = 0;
	// fill data and set numOfSamples
	fillPrbsStimulus(systemIdentification()->order);
	// fill capture with "ideal sample values"
	for(uint16_t i = 0; i < numOfSamples; i++)
	{
		sysIdData.capture[i] = encodeCaptureData(i, calculateErrorInject(i) );
	}
	readAddressPointer = 0;
}

float sysIdUpdate(float rateTarget, float gyroRate, flight_dynamics_index_t axis)
{
	static uint8_t denumerator = 0;
	static uint16_t samplecounter = 0;
	float errorRate = rateTarget - gyroRate;
	static float meas = 0;
	SysIdState nextsysIdState = sysIdState;
	if(feature(FEATURE_SYSTEM_IDENT) && axis == systemIdentification()->axis)
	{
			switch(sysIdState)
			{
			case SYSID_STATE_RESET:
				if(ARMING_FLAG(ARMED))
				{
					if(sysidTimer == 0)
					{
						nextsysIdState = SYSID_STATE_IDLE;
						sysidTimer = 0;
					}
				}
				else
				{
					sysidTimer = 100; // at least 100 cycles not enabled before getting out of reset
				}
				break;
			case SYSID_STATE_IDLE:
				if(IS_RC_MODE_ACTIVE(BOXSYSID))
				{
					nextsysIdState = SYSID_STATE_WAITFORSTART;
					sysidTimer = (int32_t)(500000.0 / (float)gyroConfig()->looptime / (float) systemIdentification()->denum);
				}
				break;
			case SYSID_STATE_WAITFORSTART:
				if(!IS_RC_MODE_ACTIVE(BOXSYSID))
				{
					// abort in case of disabled mode
					nextsysIdState = SYSID_STATE_IDLE;
					sysidTimer = 1;
				}
				if(sysidTimer == 0)
				{
					nextsysIdState = SYSID_STATE_PRESAMPLES;
					sysidTimer = PRESAMPLES;
				}
				break;
			case SYSID_STATE_PRESAMPLES:
				// inject error
				errorRate += calculateErrorInject(numOfSamples - sysidTimer - 1);
				if(sysidTimer == 0)
				{
					nextsysIdState = SYSID_STATE_SAMPLES;
					sysidTimer = numOfSamples;
					// turn on LED
					LED1_ON;
					samplecounter = 0;
				}
				break;
			case SYSID_STATE_SAMPLES:
				LED1_ON;
				if(!IS_RC_MODE_ACTIVE(BOXSYSID))
				{
					// abort in case of disabled mode
					nextsysIdState = SYSID_STATE_IDLE;
					sysidTimer = 1;
					LED1_OFF;
				}
				errorRate += calculateErrorInject(samplecounter);
				meas += gyroRate * 5;
				if(denumerator == 0)
				{
					sysIdData.capture[samplecounter] = encodeCaptureData(samplecounter, meas );
					samplecounter++;
				}
				if(sysidTimer == 0)
				{
					nextsysIdState = SYSID_STATE_FINISHED;
					sysidTimer = 1;
					LED1_OFF;
				}
				break;
			case SYSID_STATE_FINISHED:
				if(!IS_RC_MODE_ACTIVE(BOXSYSID))
				{
					nextsysIdState = SYSID_STATE_IDLE;
					sysidTimer = 1;
				}
				break;
			}
		denumerator++;
		if(denumerator == systemIdentification()->denum)
		{
			denumerator = 0;
			meas = 0;
			sysidTimer--;
		}
		if(sysidTimer < 0)
		{
			sysidTimer = 0;
		}
		sysIdState = nextsysIdState;
	}
	return errorRate;
}

void fillPrbsStimulus(uint8_t order)
{
	// based on https://blog.kurttomlinson.com/posts/prbs-pseudo-random-binary-sequence

	uint16_t taps = 0b10100;
	switch(order)
	{
	case 7:
		taps = 0b1100000;
		break;
	case 8:
		taps = 0b11000110;
		break;
	case 9:
		taps = 0b100010000;
		break;
/*	case 10:
		taps = 0b1000010000;
		break;
*/
	}
	uint16_t start_state = 0x1;  /* Any nonzero start state will work. */

	uint16_t lfsr = start_state;
	uint16_t period = 0;

	do
	{
      uint16_t lsb = lfsr & 1;  /* Get LSB (i.e., the output bit). */
	  //prbs = prbs + lsb;
  	  sysIdData.stimulus[period] = lsb;
	  lfsr >>= 1;          /* Shift register */
	  if (lsb == 1) {      /* Only apply toggle mask if output bit is 1. */
	    lfsr ^= taps;      /* Apply toggle mask, value has 1 at bits corresponding to taps, 0 elsewhere. */
	  }
	  ++period;
	} while (lfsr != start_state);
	numOfSamples = period;
	addBootlogEvent4(BOOT_EVENT_SYSTEM_IDENT_INIT_DONE, BOOT_EVENT_FLAGS_NONE, order, numOfSamples);

}

// set the address for reading capture data
void sysIdSetCaptureReadAddress(uint16_t address)
{
	readAddressPointer = address;
}

uint16_t sysIdGetCaptureReadAddress()
{
	return readAddressPointer;
}
// read numOfDataToRead samples from capture memory, address is auto incremented
// samples are filled with 0 in case of more samples are requested than available in memory
int32_t sysIdGetCaptureData(uint16_t index)
{
	int32_t data = 0;
	if(index < numOfSamples)
	{
		data = sysIdData.capture[index];
	}
	return data;
}

#endif /* USE_SYSTEM_IDENT */
