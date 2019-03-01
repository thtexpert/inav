/*
 * mixer_twin.c
 *
 *  Created on: 18.02.2019
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config_reset.h"
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/pwm_output.h"
#include "drivers/pwm_mapping.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/mixer_twin.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "navigation/navigation.h"

#include "rx/rx.h"

#include "sensors/battery.h"

// memory for real time swash plate conditions
swashPlate_t swashPlates[2];

PG_REGISTER_WITH_RESET_TEMPLATE(mixerflettner_t, mixerFlettner, PG_MIXER_FLETTNER, 0);

PG_RESET_TEMPLATE(mixerflettner_t, mixerFlettner,
		.pitchtravel = 710,			// scaling 10 = 1%
		.rolltravel = 450,			// scaling 10 = 1%
		.collectivetravel = 480,			// scaling 10 = 1%
		.cyclicring = 600,			// scaling 100 = 1degree
		.collectivemax = 1200,			// scaling 100 = 1degree
		.collectivemin = -400,			// scaling 100 = 1degree
		.cyclicmix = 580,			// scaling 10 = 1%
		.collectivemix = 180,		// scaling 10 = 1%
		.collectivemixthreshold = 100,	// scaling 100 = 1degree
		.collectivemixmax = 170,		// scaling 100 = 1degree
		.pitchff = 0,				// scaling 10 = 1%
		.centerall = 0,				// assume swashplates at 0,0,0 degree
		.platetype = 1,				// SwashPlateType H90 = 0, H120 = 1, custom = 2
		.rotationleft = 300,		// scaling 10 = 1degree
		.rotationright = 300,		// scaling 10 = 1degree
		.virtualrotleft = 170,		// scaling 10 = 1degree
		.virtualrotright = 170,		// scaling 10 = 1degree
		.cyclictravel = -430,		// scaling 10 = 1%
		.collectivtravel = -370,	// scaling 10 = 1%
		.collectivoffset = 400		// scaling 100 = 1degree;
);

PG_REGISTER_ARRAY_WITH_RESET_FN(servoSwash_t, MAX_FLETTNER_SWASH_SERVOS, flettnerSwashServos, PG_FLETTNER_SWASH_SERVOS, 0);

void pgResetFn_flettnerSwashServos(servoSwash_t *instance)
{
	RESET_CONFIG(servoSwash_t, &instance[0],
			.roll = 50,
			.pitch = 0,
			.collective = 0
	);
	RESET_CONFIG(servoSwash_t, &instance[1],
			.roll = 0,
			.pitch = 50,
			.collective = 0
	);
	RESET_CONFIG(servoSwash_t, &instance[2],
			.roll = 0,
			.pitch = 0,
			.collective = 50
	);
	RESET_CONFIG(servoSwash_t, &instance[3],
			.roll = 50,
			.pitch = 0,
			.collective = 0
	);
	RESET_CONFIG(servoSwash_t, &instance[4],
			.roll = 0,
			.pitch = 50,
			.collective = 0
	);
	RESET_CONFIG(servoSwash_t, &instance[5],
			.roll = 0,
			.pitch = 0,
			.collective = 50
	);

}

void flettnerMixer()
{
	int32_t 	pitch,roll,yaw; // +/-500 full scale
	int32_t 	tmp, collective;
	float 		gain;
    int32_t     lX1,lX2;
    int i;


    // receiver collective is mapped to THROTTLE
    collective = (rcData[THROTTLE] - PWM_RANGE_MIDDLE) * 2; // approx degree * 100 (480 -> 9.6 degree)

    if (FLIGHT_MODE(MANUAL_MODE)) {   // Direct passthru from RX
    	pitch = rcCommand[PITCH];
    	roll = rcCommand[ROLL];
    	yaw = rcCommand[YAW];
    } else {		// signals are taken from control loop
    	pitch = axisPID[PITCH];
    	roll = axisPID[ROLL];
    	yaw = axisPID[YAW];
    }


    /*
    	int16_t pitchtravel;  	// scaling 10 = 1%
    	int16_t rolltravel;  	// scaling 10 = 1%
    	int16_t collectivetravel;	// scaling 10 = 1%
    	int16_t cyclicring;		// scaling 100 = 1degree
    	int16_t collectivemax;		// scaling 100 = 1degree
    	int16_t collectivemin;		// scaling 100 = 1degree
    	int16_t cyclicmix;		  	// scaling 10 = 1%
    	int16_t collectivemix;	  	// scaling 10 = 1%
    	int16_t collectivemixthreshold;	// scaling 100 = 1degree
    	int16_t collectivemixmax;		// scaling 100 = 1degree
    	int16_t pitchff;	  	// scaling 100 = 1%
    	int16_t centerall;		// assume swashplates at 0,0,0 degree
    	int16_t collectivoffset;	// scaling 100 = 1degree;
    */

    // ROLL has no mixing
    lX1 = ( mixerFlettnerMutable()->rolltravel * roll);
    swashPlates[SwashLeft].roll = (int16_t) (lX1/512);
    swashPlates[SwashRight].roll = swashPlates[SwashLeft].roll;

    // NICK plus dma
    lX1 = ( mixerFlettnerMutable()->pitchtravel * pitch + (collective *  mixerFlettnerMutable()->pitchff) / 4);

    // cyclic differential YAW control
    swashPlates[SwashLeft].pitch = (int16_t)((lX1 - ( mixerFlettnerMutable()->cyclicmix * yaw))/512);
    swashPlates[SwashRight].pitch = (int16_t)((lX1 + ( mixerFlettnerMutable()->cyclicmix * yaw))/512);
    // scale down

    // CYCLICRING
    for (i = 0; i <= SwashRight; i++) {
		tmp = (swashPlates[i].roll * swashPlates[i].roll  + swashPlates[i].pitch * swashPlates[i].pitch) ;
		if( tmp > mixerFlettnerMutable()->cyclicring * mixerFlettnerMutable()->cyclicring )
		{
			// uups, out of cyclic ring limit
			// need to scale down the elongation
			gain =   mixerFlettnerMutable()->cyclicring / sqrt(tmp);
			swashPlates[i].roll *= gain;
			swashPlates[i].pitch *= gain;
		}
    }
    // PITCH
    lX1 = ( mixerFlettnerMutable()->collectivetravel * collective) + 512 *  mixerFlettnerMutable()->collectivoffset; // 10000 per degree


    //swashPlates[SWASH123].throttle = (cfg.swash_mix.collectivetravel * collective + cfg.swash_mix.pitchff * pitch)/64; // approx 16degree per 1000 LSB
    //swashPlates[SWASH456].throttle = swashPlates[SWASH123].throttle;

	lX2 = 0;
	if( mixerFlettnerMutable()->collectivemix > 0)
	{
		tmp = constrain((collective -  mixerFlettnerMutable()->collectivemixthreshold) *  mixerFlettnerMutable()->collectivemix, 0,  mixerFlettnerMutable()->collectivemixmax * 1000/2);
	}
	else
	{
		tmp = constrain((collective -  mixerFlettnerMutable()->collectivemixthreshold) *  mixerFlettnerMutable()->collectivemix, -  mixerFlettnerMutable()->collectivemixmax * 1000/2, 0);
	}

	lX2 = - yaw * tmp  /512;

    swashPlates[SwashLeft].collective =  (int16_t)((lX1 + lX2) / 512);
    swashPlates[SwashRight].collective =  (int16_t)((lX1 - lX2) / 512);


    // limit to pitch min and max

    swashPlates[SwashLeft].collective = constrain(swashPlates[SwashLeft].collective,  mixerFlettnerMutable()->collectivemin,  mixerFlettnerMutable()->collectivemax);
    swashPlates[SwashRight].collective = constrain(swashPlates[SwashRight].collective,  mixerFlettnerMutable()->collectivemin,  mixerFlettnerMutable()->collectivemax);

    if ( mixerFlettnerMutable()->centerall > 0)
	{
    	// center all in case of trimming
        swashPlates[SwashLeft].pitch = 0;
        swashPlates[SwashRight].pitch = 0;
        swashPlates[SwashLeft].roll = 0;
        swashPlates[SwashRight].roll = 0;
        swashPlates[SwashLeft].collective = 0;
        swashPlates[SwashRight].collective = 0;
	}

    // servo output mixing
    for(int p = SwashLeft; p <=SwashRight ; p++)
    {
    	for(i = 0 ; i < 3; i++)
    	{
    		int si = i + 3 * (p - SwashLeft); // servo index 0..2 on swash left, servo 3..5 on swash right
    		lX2 = swashPlates[p].collective * flettnerSwashServos(si)->collective/2 +
    			  swashPlates[p].pitch    * flettnerSwashServos(si)->pitch +
        		  swashPlates[p].roll     * flettnerSwashServos(si)->roll;
    		servo[si] = lX2 / 1000;
    	}
    }

    for (i = 0; i < MAX_FLETTNER_SWASH_SERVOS; i++) {
        /*
         * Add a servo midpoint to the calculation
         */
        servo[i] += servoParams(i)->middle;

        /*
         * Constrain servo position to min/max to prevent servo damage
         * If servo was saturated above min/max, that means that user most probably
         * allowed the situation when smix weight sum for an output was above 100
         */
        servo[i] = constrain(servo[i], servoParams(i)->min, servoParams(i)->max);
    }

}


bool isMixerUsingFlettner(void)
{
    return (mixerConfig()->platformType == PLATFORM_FLETTNER);
}
