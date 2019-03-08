/*
 * mixer_twin.h
 *
 *  Created on: 18.02.2019
 */

#ifndef SRC_MAIN_FLIGHT_MIXER_TWIN_H_
#define SRC_MAIN_FLIGHT_MIXER_TWIN_H_

#pragma once

#include "config/parameter_group.h"

enum {
    SwashLeft = 0,
	SwashRight = 1
};

// live data for flettner/tilt swash plates
typedef struct swashPlate_s {
	int16_t collective;	// 100 per degree
	int16_t roll;		// 100 per degree
	int16_t pitch;		// 100 per degree
} swashPlate_t;

extern swashPlate_t swashPlates[2];

typedef struct mixerflettner_s {
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
	int16_t pitchff;	  	// scaling 10 = 1%
	int16_t centerall;		// assume swashplates at 0,0,0 degree
	int16_t platetype;		// SwashPlateType H90 = 0, H120 = 1, custom = 2
	int16_t rotationleft;	// scaling 10 = 1degree
	int16_t rotationright;	// scaling 10 = 1degree
	int16_t virtualrotleft;	// scaling 10 = 1degree
	int16_t virtualrotright;	// scaling 10 = 1degree
	int16_t cyclictravel;	// scaling 10 = 1%
	int16_t collectiveoffset;	// scaling 100 = 1degree;
} mixerflettner_t;

PG_DECLARE(mixerflettner_t, mixerFlettner);

#define MAX_FLETTNER_SWASH_SERVOS (6)
// mix for servos in swashplate
typedef struct servoSwash_s {
	int16_t roll;	// scaling 10 = 1%
	int16_t pitch;	// scaling 10 = 1%, for tilt nacelle plane position
	int16_t collective;	// scaling 10 = 1%, for tilt nacelle heli position
} servoSwash_t;

PG_DECLARE_ARRAY(servoSwash_t, MAX_FLETTNER_SWASH_SERVOS, flettnerSwashServos);


// ------------ tilt rotor -------------


typedef struct mixertilt_s {
	int16_t nacellemax;					//90.00,	// heli mode nacelle position [deg]
	int16_t nacellemin;					//30.00,	// plane mode nacelle position [deg]
	int16_t nacellespeed;				// 7.00,	// nacelle turn rate [deg/sec]
	int16_t cyclicring; 				// 8.00,	// cyclic ring max deflection
	int16_t collectivemaxheli;			//14.00,	// max pitch in heli mode [deg]
	int16_t collectivemaxplane;			//26.00,	// max pitch in plane mode [deg]
	int16_t collectiveminheli;			//-4.00,	// min pitch in heli mode [deg]
	int16_t collectiveminplane;			// 2.00,	// min pitch in plane mode [deg]
	int16_t gainpitchheli;				//70.00,	// nick gain in heli mode [%]
	int16_t gainpitchplane;			//50.00,	// diffcoll gain in heli mode [%]
	int16_t gaindiffcollheli;			//45.00,	// diffcoll gain in heli mode [%]
	int16_t gaindiffcollplane;			//45.00,	// diffcoll gain in heli mode [%]
	int16_t gaindiffpitchheli;			//40.00,	// diffnick gain in heli mode [%]
	int16_t gaindiffpitchplane;			//0.00,	// diffnick gain in heli mode [%]
	int16_t centerall; 					//0,
	int16_t platetype;					// 0 (normal), 1 (custom)
	int16_t cyclictravel;				// scaling 10 = 1%
	int16_t collectivetravel;			// scaling 10 = 1%
	int16_t nacelletype;				// 0, 1 , 2 for NONE, SINGLE, DUAL
	int16_t spare2;						//0.00
} mixertilt_t;

PG_DECLARE(mixertilt_t, mixerTilt);

// index 4 and 5 store nacelle servo postions for heli and plane
#define MAX_TILT_SWASH_SERVOS (6)


PG_DECLARE_ARRAY(servoSwash_t, MAX_TILT_SWASH_SERVOS, tiltSwashServos);

typedef struct tiltlive_s {
	int16_t nacelle;					//90.00,
	int16_t leftpitch;					// 0.00,
	int16_t leftcollective;					//-2.00,
	int16_t rightpitch;					// 0.00,
	int16_t rightcollective;					//-2.00,
	int16_t gainpitch;					//70.00,
	int16_t gaindiffcoll;				//70.00,
	int16_t gaindiffpitch;				//70.00,
	int16_t collectivemin;					//-4.00,
	int16_t collectivemax;					//14.00,
	//int16_t pitchact;					//-2.00,
	//int16_t spare1;						//0.00,
	//int16_t spare2;						//0.00
} tiltlive_t;

extern tiltlive_t tiltlive; // live data from tilt rotor control


void writeFlettnerServos(int firstunusedservo);
bool isMixerUsingFlettner(void);
void flettnerMixer(void);

void writeTiltrotorServos(int firstunusedservo);
void nacelle_control(timeDelta_t looptime);
bool isMixerUsingTiltrotor(void);
void tiltrotorMixer(void);

#endif /* SRC_MAIN_FLIGHT_MIXER_TWIN_H_ */
