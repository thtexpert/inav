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
	int16_t nicktravel;  	// scaling 10 = 1%
	int16_t rolltravel;  	// scaling 10 = 1%
	int16_t pitchtravel;	// scaling 10 = 1%
	int16_t cyclicring;		// scaling 100 = 1degree
	int16_t pitchmax;		// scaling 100 = 1degree
	int16_t pitchmin;		// scaling 100 = 1degree
	int16_t cyclicmix;		  	// scaling 10 = 1%
	int16_t collectivemix;	  	// scaling 10 = 1%
	int16_t collectivemixthreshold;	// scaling 100 = 1degree
	int16_t collectivemixmax;		// scaling 100 = 1degree
	int16_t nickdma;	  	// scaling 10 = 1%
	int16_t centerall;		// assume swashplates at 0,0,0 degree
	int16_t platetype;		// SwashPlateType H90 = 0, H120 = 1, custom = 2
	int16_t rotationleft;	// scaling 10 = 1degree
	int16_t rotationright;	// scaling 10 = 1degree
	int16_t virtualrotleft;	// scaling 10 = 1degree
	int16_t virtualrotright;	// scaling 10 = 1degree
	int16_t cyclictravel;	// scaling 10 = 1%
	int16_t collectivtravel;	// scaling 10 = 1%
	int16_t collectivoffset;	// scaling 100 = 1degree;
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

typedef struct mixertilt_s {
//    int16_t min;                            // servo min, for tilt the heli position
//    int16_t max;                            // servo max , for tilt the plane position
//    int16_t middle;                         // servo middle
//    int16_t rate;                            // range [-100;+100] ; can be used to ajust a rate 0-100% and a direction
	int16_t nacellemax;					//90.00,	// heli mode nacelle position [deg]
	int16_t nacellemin;					//30.00,	// plane mode nacelle position [deg]
	int16_t nacellespeed;				// 7.00,	// nacelle turn rate [deg/sec]
	int16_t cyclicring; 				// 8.00,	// cyclic ring max deflection
	int16_t pitchmaxheli;				//14.00,	// max pitch in heli mode [deg]
	int16_t pitchmaxplane;				//26.00,	// max pitch in plane mode [deg]
	int16_t pitchminheli;				//-4.00,	// min pitch in heli mode [deg]
	int16_t pitchminplane;				// 2.00,	// min pitch in plane mode [deg]
	int16_t gainnickheli;				//70.00,	// nick gain in heli mode [%]
	int16_t gainnickplane;				//50.00,	// nick gain in heli mode [%]
	int16_t gaindiffcollheli;			//50.00,	// diffcoll gain in heli mode [%]
	int16_t gaindiffcollplane;			//45.00,	// diffcoll gain in heli mode [%]
	int16_t gaindiffnickheli;			//40.00,	// diffnick gain in heli mode [%]
	int16_t gaindiffnickplane;			//0.00,	// diffnick gain in heli mode [%]
	int16_t centerall; 					//0,
	int16_t spare1;						//0.00,
	int16_t spare2;						//0.00
} mixertilt_t;

PG_DECLARE(mixertilt_t, mixerTilt);

typedef struct tiltlive_s {
	int16_t nacelle;					//90.00,
	int16_t leftnick;					// 0.00,
	int16_t leftpitch;					//-2.00,
	int16_t rightnick;					// 0.00,
	int16_t rightpitch;					//-2.00,
	int16_t gainnick;					//70.00,
	int16_t gaindiffcoll;				//70.00,
	int16_t gaindiffnick;				//70.00,
	int16_t pitchmin;					//-4.00,
	int16_t pitchmax;					//14.00,
	int16_t pitchact;					//-2.00,
	int16_t spare1;						//0.00,
	int16_t spare2;						//0.00
} tiltlive_t;


bool isMixerUsingFlettner(void);
void flettnerMixer(void);

#endif /* SRC_MAIN_FLIGHT_MIXER_TWIN_H_ */
