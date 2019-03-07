/*
 * This file is part of INAV
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#define MSP2_COMMON_TZ              0x1001  //out message       Gets the TZ offset for the local time (returns: minutes(i16))
#define MSP2_COMMON_SET_TZ          0x1002  //in message        Sets the TZ offset for the local time (args: minutes(i16))
#define MSP2_COMMON_SETTING         0x1003  //in/out message    Returns the value for a setting
#define MSP2_COMMON_SET_SETTING     0x1004  //in message        Sets the value for a setting

#define MSP2_COMMON_MOTOR_MIXER     0x1005
#define MSP2_COMMON_SET_MOTOR_MIXER 0x1006

#define MSP2_COMMON_SETTING_INFO    0x1007  //in/out message    Returns info about a setting (PG, type, flags, min/max, etc..).
#define MSP2_COMMON_PG_LIST         0x1008  //in/out message    Returns a list of the PG ids used by the settings

#define MSP2_COMMON_SERIAL_CONFIG       0x1009
#define MSP2_COMMON_SET_SERIAL_CONFIG   0x100A

#define MSP2_FLETTNER_SWASH_MIX       0x100B    // out message     return swash plate mix data
#define MSP2_FLETTNER_SET_SWASH_MIX   0x100C    // in message      set swash plate mix data
#define MSP2_FLETTNER_SERVO_MIX       0x100D    // out message     return swash plate servo mixing data
#define MSP2_FLETTNER_SET_SERVO_MIX   0x100E    // in message      set swash plate servo mixing data
#define MSP2_FLETTNER_SWASH           0x100F    // out message     return swash plate live data
#define MSP2_TILT_SETUP		          0x1010    // out message     return tilt setup data
#define MSP2_TILT_SET_SETUP		      0x1011    // in message      set tilt setup data
#define MSP2_TILT_SERVO_MIX		      0x1012    // out message     returns tilt servo mix data
#define MSP2_TILT_SET_SERVO_MIX		  0x1013    // in message      set tilt servo mix data
#define MSP2_TILT_LIVE   		      0x1014    // out message     returns tilt live data
#define MSP2_SERVO_PWM_OVERRIDE	      0x1015    // out message
#define MSP2_SET_SERVO_PWM_OVERRIDE	  0x1016    // in message

