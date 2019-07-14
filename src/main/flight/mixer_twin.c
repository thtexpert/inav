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

#include "fc/fc_core.h"
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
		.collectiveoffset = 400,		// scaling 100 = 1degree;
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
		.cyclicgain = 430,		// scaling 10 = 1%
		.collectivegain = -430		// scaling 10 = 1%
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


void writeFlettnerServos(int firstunusedservo)
{
    for (int i = 0; i < firstunusedservo; i++) {
            pwmWriteServo(i, servo[i]);
    }
}


void flettnerMixer()
{
	int32_t 	pitch,roll,yaw; // +/-500 full scale
	int32_t 	tmp, collective;
	float 		gain;
    int32_t     lX1,lX2;
    int i;


    // receiver collective is mapped to THROTTLE
    collective = (rxGetChannelValue(THROTTLE) - PWM_RANGE_MIDDLE) * 2; // approx degree * 100 (480 -> 9.6 degree)

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
    lX1 = ( mixerFlettnerMutable()->collectivetravel * collective) + 512 *  mixerFlettnerMutable()->collectiveoffset; // 10000 per degree


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



/////////////////////////////////////////////

swashPlate_t tiltPlates[2];
tiltlive_t tiltlive;

PG_REGISTER_WITH_RESET_TEMPLATE(mixertilt_t, mixerTilt, PG_MIXER_TILT, 0);

PG_RESET_TEMPLATE(mixertilt_t, mixerTilt,
		.nacellemax = 9000,					//90.00,	// heli mode nacelle position [deg]
		.nacellemin= 00,				//30.00,	// plane mode nacelle position [deg]
		.nacellespeed = 640,				// 7.00,	// nacelle turn rate [deg/sec]
		.cyclicring = 1500,				// 8.00,	// cyclic ring max deflection
		.collectivemaxheli =	1800,		//14.00,	// max pitch in heli mode [deg]
		.collectivemaxplane = 2600,			//26.00,	// max pitch in plane mode [deg]
		.collectiveminheli = -600,			//-4.00,	// min pitch in heli mode [deg]
		.collectiveminplane = 0,			// 2.00,	// min pitch in plane mode [deg]
		.gainpitchheli = 450,			//70.00,	// nick gain in heli mode [%]
		.gainpitchplane = 550,		//50.00,	// diffcoll gain in heli mode [%]
		.gaindiffcollheli = 450,		//45.00,	// diffcoll gain in heli mode [%]
		.gaindiffcollplane = 450,		//45.00,	// diffcoll gain in heli mode [%]
		.gaindiffpitchheli = 500,		//40.00,	// diffnick gain in heli mode [%]
		.gaindiffpitchplane = 800,			//0.00,	// diffnick gain in heli mode [%]
		.centerall = 0,					//0,
		.nacelletype = 2,				// 0, 1 , 2 for NONE, SINGLE, DUAL
		.spare2 = 0						//0.00
);

PG_REGISTER_ARRAY_WITH_RESET_FN(servoSwash_t, MAX_TILT_SWASH_SERVOS, tiltSwashServos, PG_TILT_SWASH_SERVOS, 0);

void pgResetFn_tiltSwashServos(servoSwash_t *instance)
{
	RESET_CONFIG(servoSwash_t, &instance[0],
			.roll = 0,
			.pitch = -750,
			.collective = -750
	);
	RESET_CONFIG(servoSwash_t, &instance[1],
			.roll = 0,
			.pitch = 750,
			.collective = -750
	);
	RESET_CONFIG(servoSwash_t, &instance[2],
			.roll = 0,
			.pitch = 750,
			.collective = 750
	);
	RESET_CONFIG(servoSwash_t, &instance[3],
			.roll = 0,
			.pitch = -750,
			.collective = 750
	);
	RESET_CONFIG(servoSwash_t, &instance[4],
			.roll = 0,
			.pitch = 1200,
			.collective = 1800
	);
	RESET_CONFIG(servoSwash_t, &instance[5],
			.roll = 0,
			.pitch = 1200,
			.collective = 1800
	);
}

static int32_t		nacelle_angle = 45000000; // default to heli position
static float nacelle_cos = 1.0; // 0 at heli, 1 at plane mode
static float nacelle_sin = 1.0; // 1 at heli, 0 at plane mode
static bool nacelleServoValid = false;

void presetNacelle(int16_t data)
{
	nacelle_angle = 90000000 - (data - 1050) * 100000;
}

void writeTiltrotorServos(int firstunusedservo)
{
	if(nacelleServoValid == false)
	{
		// nacelle servo control is enabled after calibration complete
		if( !isCalibrating() && failsafeIsReceivingRxData() ){
			presetNacelle(rxGetChannelValue(AUX2));
			nacelleServoValid = true;
		}
	}
    for (int i = 0; i < 4; i++) {
            pwmWriteServo(i, servo[i]);
    }
    for (int i = 4; i < firstunusedservo; i++) {
    	if(nacelleServoValid == true){
            pwmWriteServo(i, servo[i]);
		}
		else {
            pwmWriteServo(i, 0);
		}

    }
}

void nacelle_control(timeDelta_t looptime)
{
	// control the sweeping of the nacelle(s)
	// control is coming from AUX2
	// rc 1050-1950: 90..0 degree
	// 90 deg : Heli
	//  0 deg : plane mode
	//
	// max and min given by GUI
	// travel speed given by GUI
	// live data monitored in GUI by tiltlive.nacelle (1degree := 100)
	// live data stored inside with high resolution in nacelle_angle (1degree = 1000000)
	// also calculate the trigonometric components sin and cos

	int32_t nacelle_step_per_cycle_ppm = mixerTiltMutable()->nacellespeed * looptime;
	nacelle_step_per_cycle_ppm = nacelle_step_per_cycle_ppm / 100;
	if(nacelle_step_per_cycle_ppm < 1000)
	{
		nacelle_step_per_cycle_ppm = 1000;
	}

	int32_t		target_nacelle = 90000000 - (rxGetChannelValue(AUX2) - 1050) * 100000;

	if(target_nacelle > nacelle_angle)
	{
		nacelle_angle += nacelle_step_per_cycle_ppm;
	}
	if(target_nacelle < nacelle_angle)
	{
		nacelle_angle -= nacelle_step_per_cycle_ppm;
	}

	if(nacelle_angle > mixerTiltMutable()->nacellemax * 10000)
	{
		nacelle_angle = mixerTiltMutable()->nacellemax * 10000;
	}
	if(nacelle_angle < mixerTiltMutable()->nacellemin * 10000)
	{
		nacelle_angle = mixerTiltMutable()->nacellemin * 10000;
	}
	// update GUI interface
	tiltlive.nacelle = nacelle_angle / 10000;

	float 		gain;
	gain = (tiltlive.nacelle - 0) * (mixerTiltMutable()->gainpitchheli - mixerTiltMutable()->gainpitchplane) /9000 + mixerTiltMutable()->gainpitchplane;
	tiltlive.gainpitch = gain;

	gain = (tiltlive.nacelle - 0) * (mixerTiltMutable()->gaindiffcollheli - mixerTiltMutable()->gaindiffcollplane) /9000 + mixerTiltMutable()->gaindiffcollplane;
	tiltlive.gaindiffcoll = gain;

	gain = (tiltlive.nacelle - 0) * (mixerTiltMutable()->gaindiffpitchheli - mixerTiltMutable()->gaindiffpitchplane) /9000 + mixerTiltMutable()->gaindiffpitchplane;
	tiltlive.gaindiffpitch = gain;

	gain = (tiltlive.nacelle - 0) * (mixerTiltMutable()->collectiveminheli - mixerTiltMutable()->collectiveminplane) /9000 + mixerTiltMutable()->collectiveminplane;
	tiltlive.collectivemin = gain;

	gain = (tiltlive.nacelle - 0) * (mixerTiltMutable()->collectivemaxheli - mixerTiltMutable()->collectivemaxplane) /9000 + mixerTiltMutable()->collectivemaxplane;
	tiltlive.collectivemax = gain;

	float angle_rad = 2 * M_PI/ 36000.0 * tiltlive.nacelle ;
	nacelle_sin = sinf(angle_rad);
	nacelle_cos = cosf(angle_rad);

	// Nacelle Servo control  tiltSwashServos
	for(int i = 0; i< mixerTiltMutable()->nacelletype; i++)
	{
		gain = tiltSwashServos(4 + i)->pitch + (float)(tiltSwashServos(4 + i)->collective - tiltSwashServos(4 + i)->pitch)/ 9000. * (float)tiltlive.nacelle ;
		servo[4 + i] = (int16_t) gain;
	}
}

void tiltrotorMixer(void)
{
	float 	pitch,roll,yaw; // +/-500 full scale
	float		collective;
	float		diffcoll;
	float		diffpitch;
    int32_t     lX2;
    int i;

    if (FLIGHT_MODE(MANUAL_MODE)) {   // Direct passthru from RX
    	pitch = rcCommand[PITCH];
    	roll = rcCommand[ROLL];
    	yaw = rcCommand[YAW];
    } else {		// signals are taken from control loop
    	pitch = axisPID[PITCH];
    	roll = axisPID[ROLL];
    	yaw = axisPID[YAW];
    }
    // diff collective
    diffcoll = (float) ( ( -(float)roll * nacelle_sin + (float)yaw * nacelle_cos ) * (float)tiltlive.gaindiffcoll / 1000.0 / 1.0) ;
    diffpitch = (float) ( ( -(float)yaw * nacelle_sin - (float)roll * nacelle_cos ) * (float)tiltlive.gaindiffpitch / 1000.0 * 2.0) ;


    // receiver collective is mapped to THROTTLE
    collective = (float)((float)tiltlive.collectivemin + (float)(tiltlive.collectivemax - tiltlive.collectivemin) * (float)(rxGetChannelValue(THROTTLE) - motorConfig()->minthrottle)/(motorConfig()->maxthrottle - motorConfig()->minthrottle) );
    collective = constrain(collective, tiltlive.collectivemin, tiltlive.collectivemax);
    tiltlive.leftcollective = (int16_t)(collective - diffcoll);
    tiltlive.rightcollective = (int16_t)(collective + diffcoll);

    // pitch +- 330 maps to +-7 at gain 100.0 %
    pitch = (float)((float)pitch/330. * (float)(tiltlive.gainpitch)/1000. *10. * 100. );
    tiltlive.leftpitch = (int16_t) constrain(pitch + diffpitch,-mixerTiltMutable()->cyclicring,mixerTiltMutable()->cyclicring);
    tiltlive.rightpitch = (int16_t) constrain(pitch - diffpitch,-mixerTiltMutable()->cyclicring,mixerTiltMutable()->cyclicring);


    if (mixerTiltMutable()->centerall > 0)
	{
    	// center all in case of trimming
    	tiltlive.leftcollective = 0;
    	tiltlive.rightcollective = 0;
    	tiltlive.leftpitch = 0;
    	tiltlive.rightpitch = 0;
	}

	for(i = 0 ; i < 2; i++)
	{
		lX2 = tiltlive.leftcollective * tiltSwashServos(i)->collective -
			  tiltlive.leftpitch    * tiltSwashServos(i)->pitch  ;
		servo[i] = lX2 / 3000;
		servo[i] += servoParams(i)->middle;
        servo[i] = constrain(servo[i], servoParams(i)->min, servoParams(i)->max);
		lX2 = tiltlive.rightcollective * tiltSwashServos(i+2)->collective -
			  tiltlive.rightpitch    * tiltSwashServos(i+2)->pitch  ;
		servo[i+2] = lX2 / 3000;
		servo[i+2] += servoParams(i+2)->middle;
        servo[i+2] = constrain(servo[i+2], servoParams(i+2)->min, servoParams(i+2)->max);
	}


}

bool isMixerUsingTiltrotor(void)
{
    return (mixerConfig()->platformType == PLATFORM_TILTROTOR);
}


