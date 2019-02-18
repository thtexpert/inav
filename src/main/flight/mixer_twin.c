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

PG_REGISTER_WITH_RESET_TEMPLATE(mixerflettner_t, mixerFlettner, PG_MIXER_FLETTNER, 0);

PG_RESET_TEMPLATE(mixerflettner_t, mixerFlettner,
		.nicktravel = 710,			// scaling 10 = 1%
		.rolltravel = 450,			// scaling 10 = 1%
		.pitchtravel = 480,			// scaling 10 = 1%
		.cyclicring = 600,			// scaling 100 = 1degree
		.pitchmax = 1200,			// scaling 100 = 1degree
		.pitchmin = -400,			// scaling 100 = 1degree
		.cyclicmix = 580,			// scaling 10 = 1%
		.collectivemix = 180,		// scaling 10 = 1%
		.collectivemixthreshold = 100,	// scaling 100 = 1degree
		.collectivemixmax = 170,		// scaling 100 = 1degree
		.nickdma = 0,				// scaling 10 = 1%
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
