/*
 * system_identification.h
 *
 *  Created on: 24.04.2019
 *      Author: anton
 */

#ifndef SRC_MAIN_FLIGHT_SYSTEM_IDENTIFICATION_H_
#define SRC_MAIN_FLIGHT_SYSTEM_IDENTIFICATION_H_
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#if defined(USE_SYSTEM_IDENT)

#include "config/parameter_group.h"
#include "common/axis.h"

typedef struct sysid_s {
	uint8_t axis;  	// select one of the three main axis
	uint8_t order;  // order of PRBS sequence (capture size is (2 ^ order - 1))
	uint8_t denum;	// number of main loops per sample
	uint8_t level;	// disturbance level +/-500 is full stick movement;
} sysid_t;

PG_DECLARE(sysid_t, systemIdentification);

void sysIdInitialize();
float sysIdUpdate(float rateTarget, float gyroRate, flight_dynamics_index_t axis);

// set the address for reading capture data
void sysIdSetCaptureReadAddress(uint16_t address);
uint16_t sysIdGetCaptureReadAddress();

// read numOfDataToRead samples from capture memory, address is auto incremented
// samples are filled with 0 in case of more samples are requested than available in memory
int32_t sysIdGetCaptureData(uint16_t index);

#endif /* USE_SYSTEM_IDENT */
#endif /* SRC_MAIN_FLIGHT_SYSTEM_IDENTIFICATION_H_ */
