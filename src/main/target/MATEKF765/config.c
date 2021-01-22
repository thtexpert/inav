/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "platform.h"

#include "fc/fc_msp_box.h"

#include "io/piniobox.h"
#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "flight/mixer.h"

void targetConfiguration(void)
{
    //pinioBoxConfigMutable()->permanentId[0] = BOX_PERMANENT_ID_USER1;
    //pinioBoxConfigMutable()->permanentId[1] = BOX_PERMANENT_ID_USER2;
    mixerConfigMutable()->platformType = PLATFORM_AIRPLANE;   // default mixer to Airplane

    motorConfigMutable()->minthrottle = 1050;
    motorConfigMutable()->maxthrottle = 1950;


    ((controlRateConfig_t*)currentControlRateProfile)->stabilized.rcExpo8 = 0;
    ((controlRateConfig_t*)currentControlRateProfile)->stabilized.rcYawExpo8 = 0;
    ((controlRateConfig_t*)currentControlRateProfile)->manual.rcExpo8 = 0;
    ((controlRateConfig_t*)currentControlRateProfile)->manual.rcYawExpo8 = 0;
    ((controlRateConfig_t*)currentControlRateProfile)->throttle.rcMid8 = 0;
    ((controlRateConfig_t*)currentControlRateProfile)->throttle.rcExpo8 = 0;
}
