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
 *
 * Authors:
 * Thomas Miric - marv-t
 *
 * Jeti EX Bus Communication Protocol:
 * http://www.jetimodel.com/en/show-file/642/
 *
 * JETI Telemetry Communication Protocol:
 * http://www.jetimodel.com/en/show-file/26/
 *
 * Following restrictions:
 * Communication speed: 125000 bps
 * Number of channels: 16
 *
 * Connect as follows:
 * Jeti EX Bus -> Serial RX (connect directly)
 * Serial TX -> Resistor(2k4) ->Serial RX
 * In jeti pdf it is different, but if the resistor breaks, the receiver continues to operate.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SERIALRX_JETIEXBUS

#include "build/build_config.h"
#include "build/debug.h"

#include "common/utils.h"
#include "common/bitarray.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/jetiexbus.h"

#ifdef USE_TELEMETRY

#include "config/feature.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer_twin.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "navigation/navigation.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "telemetry/telemetry.h"
#include "telemetry/jetiexbus.h"
#include "telemetry/frsky.h"
#endif // TELEMETRY


//
// Serial driver for Jeti EX Bus receiver
//
#define JETIEXBUS_BAUDRATE 125000                       // EX Bus 125000; EX Bus HS 250000 not supported
#define JETIEXBUS_OPTIONS (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED)
#define JETIEXBUS_MIN_FRAME_GAP     1000
#define JETIEXBUS_CHANNEL_COUNT     16                  // most Jeti TX transmit 16 channels

#define EXBUS_HEADER_LEN                6
#define EXBUS_CRC_LEN                   2
#define EXBUS_OVERHEAD                  (EXBUS_HEADER_LEN + EXBUS_CRC_LEN)
#define EXBUS_MAX_CHANNEL_FRAME_SIZE    (EXBUS_HEADER_LEN + JETIEXBUS_CHANNEL_COUNT*2 + EXBUS_CRC_LEN)
#define EXBUS_MAX_REQUEST_FRAME_SIZE    9

#define EXBUS_START_CHANNEL_FRAME       (0x3E)
#define EXBUS_START_REQUEST_FRAME       (0x3D)
#define EXBUS_EX_REQUEST                (0x3A)
#define EXBUS_JETIBOX_REQUEST           (0x3B)

#define EXBUS_CHANNELDATA               (0x3E03)        // Frame contains Channel Data
#define EXBUS_CHANNELDATA_DATA_REQUEST  (0x3E01)        // Frame contains Channel Data, but with a request for data
#define EXBUS_TELEMETRY_REQUEST         (0x3D01)        // Frame is a request Frame

enum {
    EXBUS_STATE_ZERO = 0,
    EXBUS_STATE_IN_PROGRESS,
    EXBUS_STATE_RECEIVED,
    EXBUS_STATE_PROCESSED
};

enum {
    EXBUS_TRANS_ZERO = 0,
    EXBUS_TRANS_RX_READY,
    EXBUS_TRANS_RX,
    EXBUS_TRANS_IS_TX_COMPLETED,
    EXBUS_TRANS_TX
};

enum exBusHeader_e {
    EXBUS_HEADER_SYNC = 0,
    EXBUS_HEADER_REQ,
    EXBUS_HEADER_MSG_LEN,
    EXBUS_HEADER_PACKET_ID,
    EXBUS_HEADER_DATA_ID,
    EXBUS_HEADER_SUBLEN,
    EXBUS_HEADER_DATA
};

#ifdef USE_TELEMETRY

#define EXTEL_DATA_MSG      (0x40)
#define EXTEL_UNMASK_TYPE   (0x3F)
#define EXTEL_SYNC_LEN      1
#define EXTEL_CRC_LEN       1
#define EXTEL_HEADER_LEN    6
#define EXTEL_MAX_LEN       29
#define EXTEL_OVERHEAD      (EXTEL_SYNC_LEN + EXTEL_HEADER_LEN + EXTEL_CRC_LEN)
#define EXTEL_MAX_PAYLOAD   (EXTEL_MAX_LEN - EXTEL_OVERHEAD)
#define EXBUS_MAX_REQUEST_BUFFER_SIZE   (EXBUS_OVERHEAD + EXTEL_MAX_LEN)

enum exTelHeader_e {
    EXTEL_HEADER_SYNC = 0,
    EXTEL_HEADER_TYPE_LEN,
    EXTEL_HEADER_USN_LB,
    EXTEL_HEADER_USN_HB,
    EXTEL_HEADER_LSN_LB,
    EXTEL_HEADER_LSN_HB,
    EXTEL_HEADER_RES,
    EXTEL_HEADER_ID,
    EXTEL_HEADER_DATA
};

enum exDataType_e {
    EX_TYPE_6b   = 0,                // int6_t  Data type 6b (-31 ¸31)
    EX_TYPE_14b  = 1,                // int14_t Data type 14b (-8191 ¸8191)
    EX_TYPE_22b  = 4,                // int22_t Data type 22b (-2097151 ¸2097151)
    EX_TYPE_DT   = 5,                // int22_t Special data type – time and date
    EX_TYPE_30b  = 8,                // int30_t Data type 30b (-536870911 ¸536870911)
    EX_TYPE_GPS  = 9,                // int30_t Special data type – GPS coordinates:  lo/hi minute - lo/hi degree.
    EX_TYPE_DES  = 255               // only for devicedescription
};

const uint8_t exDataTypeLen[]={
    [EX_TYPE_6b]  = 1,
    [EX_TYPE_14b] = 2,
    [EX_TYPE_22b] = 3,
    [EX_TYPE_DT]  = 3,
    [EX_TYPE_30b] = 4,
    [EX_TYPE_GPS] = 4
};

typedef struct exBusSensor_s{
    const char *label;
    const char *unit;
    const uint8_t exDataType;
    const uint8_t decimals;
} exBusSensor_t;

#define DECIMAL_MASK(decimals) (decimals << 5)

// list of telemetry messages
// after every 15 sensors a new header has to be inserted (e.g. "CF-Dev 1.12 S2")
exBusSensor_t jetiExSensors[] = {
	    {"TWINFLIGHT A",    "",         EX_TYPE_DES,   0              },     // device descripton
	    {"Voltage",         "V",        EX_TYPE_22b,   DECIMAL_MASK(1)},
	    {"Current",         "A",        EX_TYPE_22b,   DECIMAL_MASK(2)},
	    {"Altitude",        "m",        EX_TYPE_22b,   DECIMAL_MASK(2)},
	    {"Capacity",        "mAh",      EX_TYPE_22b,   DECIMAL_MASK(0)},
	    {"Power",           "W",        EX_TYPE_22b,   DECIMAL_MASK(1)},
	    {"Roll angle",      "\xB0",     EX_TYPE_22b,   DECIMAL_MASK(1)},
	    {"Pitch angle",     "\xB0",     EX_TYPE_22b,   DECIMAL_MASK(1)},
	    {"Nacelle angle",   "\xB0",     EX_TYPE_22b,   DECIMAL_MASK(1)},
	    {"Roll Integrator", "",         EX_TYPE_22b,   DECIMAL_MASK(0)},
	    {"Pitch Integrator","",         EX_TYPE_22b,   DECIMAL_MASK(0)},
	    {"Yaw Integrator",  "",         EX_TYPE_22b,   DECIMAL_MASK(0)},
	    {"Flight Mode",  	"",         EX_TYPE_22b,   DECIMAL_MASK(0)},
	    {"Heading",         "\xB0",     EX_TYPE_22b,   DECIMAL_MASK(1)},
	    {"Vario",           "m/s",      EX_TYPE_22b,   DECIMAL_MASK(2)},
	    {"TWINFLIGHT B",    "",         EX_TYPE_DES,   0              },     // device descripton
	    {"GPS Sats",        "",         EX_TYPE_22b,   DECIMAL_MASK(0)},
	    {"GPS Long",        "",         EX_TYPE_GPS,   DECIMAL_MASK(0)},
	    {"GPS Lat",         "",         EX_TYPE_GPS,   DECIMAL_MASK(0)},
	    {"GPS Speed",       "m/s",      EX_TYPE_22b,   DECIMAL_MASK(2)},
	    {"GPS H-Distance",  "m",        EX_TYPE_22b,   DECIMAL_MASK(0)},
	    {"GPS H-Direction", "\xB0",     EX_TYPE_22b,   DECIMAL_MASK(1)},
	    {"GPS Heading",     "\xB0",     EX_TYPE_22b,   DECIMAL_MASK(1)},
	    {"GPS Altitude",    "m",        EX_TYPE_22b,   DECIMAL_MASK(2)},
	    {"G-Force X",       "",         EX_TYPE_22b,   DECIMAL_MASK(3)},
	    {"G-Force Y",       "",         EX_TYPE_22b,   DECIMAL_MASK(3)},
	    {"G-Force Z",       "",         EX_TYPE_22b,   DECIMAL_MASK(3)}
};


// after every 15 sensors increment the step by 2 (e.g. ...EX_VAL15, EX_VAL16 = 17) to skip the device description
enum exSensors_e {
    EX_VOLTAGE = 1,
    EX_CURRENT,
    EX_ALTITUDE,
    EX_CAPACITY,
    EX_POWER,
    EX_ROLL_ANGLE,
    EX_PITCH_ANGLE,
	EX_NACELLE_ANGLE	  ,
	EX_INTEGRATOR_PITCH ,
	EX_INTEGRATOR_ROLL  ,
	EX_INTEGRATOR_YAW   ,
	EX_FLIGHT_MODE   ,
    EX_HEADING,
    EX_VARIO,
    EX_GPS_SATS = 16,
    EX_GPS_LONG,
    EX_GPS_LAT,
    EX_GPS_SPEED,
    EX_GPS_DISTANCE_TO_HOME,
    EX_GPS_DIRECTION_TO_HOME,
    EX_GPS_HEADING,
    EX_GPS_ALTITUDE,
    EX_GFORCE_X,
    EX_GFORCE_Y,
    EX_GFORCE_Z
};

union{
    int32_t vInt;
    uint16_t vWord[2];
    char    vBytes[4];
} exGps;

#define JETI_EX_SENSOR_COUNT (ARRAYLEN(jetiExSensors))
#endif //TELEMETRY

static serialPort_t *jetiExBusPort;

static uint32_t jetiTimeStampRequest = 0;

static uint8_t jetiExBusFramePosition;
static uint8_t jetiExBusFrameLength;

static uint8_t jetiExBusFrameState = EXBUS_STATE_ZERO;
static uint8_t jetiExBusRequestState = EXBUS_STATE_ZERO;

// Use max values for ram areas
static uint8_t jetiExBusChannelFrame[EXBUS_MAX_CHANNEL_FRAME_SIZE];
static uint8_t jetiExBusRequestFrame[EXBUS_MAX_REQUEST_FRAME_SIZE];

static uint16_t jetiExBusChannelData[JETIEXBUS_CHANNEL_COUNT];

#ifdef USE_TELEMETRY

static uint8_t jetiExBusTelemetryFrame[40];
static uint8_t jetiExBusTransceiveState = EXBUS_TRANS_RX;
static uint8_t firstActiveSensor = 0;
static uint32_t exSensorEnabled = 0;

static uint8_t sendJetiExBusTelemetry(uint8_t packetID, uint8_t item);
static uint8_t getNextActiveSensor(uint8_t currentSensor);

uint8_t calcCRC8(uint8_t *pt, uint8_t msgLen);

#endif //TELEMETRY


// Jeti Ex Bus CRC calculations for a frame
uint16_t calcCRC16(uint8_t *pt, uint8_t msgLen)
{
    uint16_t crc16_data = 0;
    uint8_t data=0;

    for (uint8_t mlen = 0; mlen < msgLen; mlen++){
        data = pt[mlen] ^ ((uint8_t)(crc16_data) & (uint8_t)(0xFF));
        data ^= data << 4;
        crc16_data = ((((uint16_t)data << 8) | ((crc16_data & 0xFF00) >> 8))
                      ^ (uint8_t)(data >> 4)
                      ^ ((uint16_t)data << 3));
    }
    return(crc16_data);
}

#ifdef USE_TELEMETRY


// Jeti Ex Telemetry CRC calculations for a frame
uint8_t calcCRC8(uint8_t *pt, uint8_t msgLen)
{
    uint8_t crc=0;

    for (uint8_t mlen = 0; mlen < msgLen; mlen++) {
        crc  ^= pt[mlen];
        crc = crc ^ (crc << 1) ^ (crc << 2) ^ (0x0e090700 >> ((crc >> 3) & 0x18));
    }
    return(crc);
}

#endif //TELEMETRY


void jetiExBusDecodeChannelFrame(uint8_t *exBusFrame)
{
    uint16_t value;
    uint8_t frameAddr;

    // Decode header
    switch (((uint16_t)exBusFrame[EXBUS_HEADER_SYNC] << 8) | ((uint16_t)exBusFrame[EXBUS_HEADER_REQ])){

    case EXBUS_CHANNELDATA_DATA_REQUEST:                   // not yet specified
    case EXBUS_CHANNELDATA:
        for (uint8_t i = 0; i < JETIEXBUS_CHANNEL_COUNT; i++) {
            frameAddr = EXBUS_HEADER_LEN + i * 2;
            value = ((uint16_t)exBusFrame[frameAddr + 1]) << 8;
            value += (uint16_t)exBusFrame[frameAddr];
            // Convert to internal format
            jetiExBusChannelData[i] = value >> 3;
        }
        break;
    }
}


void jetiExBusFrameReset(void)
{
    jetiExBusFramePosition = 0;
    jetiExBusFrameLength = EXBUS_MAX_CHANNEL_FRAME_SIZE;
}


/*
  supported:
  0x3E 0x01 LEN Packet_ID 0x31 SUB_LEN Data_array CRC16      // Channel Data with telemetry request (2nd byte 0x01)
  0x3E 0x03 LEN Packet_ID 0x31 SUB_LEN Data_array CRC16      // Channel Data forbids answering (2nd byte 0x03)
  0x3D 0x01 0x08 Packet_ID 0x3A 0x00 CRC16                   // Telemetry Request EX telemetry (5th byte 0x3A)

  other messages - not supported:
  0x3D 0x01 0x09 Packet_ID 0x3B 0x01 0xF0 CRC16              // Jetibox request (5th byte 0x3B)
  ...
*/

// Receive ISR callback
static void jetiExBusDataReceive(uint16_t c, void *rxCallbackData)
{
    UNUSED(rxCallbackData);

    timeUs_t now;
    static timeUs_t jetiExBusTimeLast = 0;
    static timeDelta_t jetiExBusTimeInterval;

    static uint8_t *jetiExBusFrame;

    // Check if we shall reset frame position due to time
    now = micros();

    jetiExBusTimeInterval = cmpTimeUs(now, jetiExBusTimeLast);
    jetiExBusTimeLast = now;

    if (jetiExBusTimeInterval > JETIEXBUS_MIN_FRAME_GAP) {
        jetiExBusFrameReset();
        jetiExBusFrameState = EXBUS_STATE_ZERO;
        jetiExBusRequestState = EXBUS_STATE_ZERO;
    }

    // Check if we shall start a frame?
    if (jetiExBusFramePosition == 0) {
        switch (c){
        case EXBUS_START_CHANNEL_FRAME:
            jetiExBusFrameState = EXBUS_STATE_IN_PROGRESS;
            jetiExBusFrame = jetiExBusChannelFrame;
            break;

        case EXBUS_START_REQUEST_FRAME:
            jetiExBusRequestState = EXBUS_STATE_IN_PROGRESS;
            jetiExBusFrame = jetiExBusRequestFrame;
            break;

        default:
            return;
        }
    }

    // Store in frame copy
    jetiExBusFrame[jetiExBusFramePosition] = (uint8_t)c;
    jetiExBusFramePosition++;

    // Check the header for the message length
    if (jetiExBusFramePosition == EXBUS_HEADER_LEN) {

        if ((jetiExBusFrameState == EXBUS_STATE_IN_PROGRESS) && (jetiExBusFrame[EXBUS_HEADER_MSG_LEN] <= EXBUS_MAX_CHANNEL_FRAME_SIZE)) {
            jetiExBusFrameLength = jetiExBusFrame[EXBUS_HEADER_MSG_LEN];
            return;
        }

        if ((jetiExBusRequestState == EXBUS_STATE_IN_PROGRESS) && (jetiExBusFrame[EXBUS_HEADER_MSG_LEN] <= EXBUS_MAX_REQUEST_FRAME_SIZE)) {
            jetiExBusFrameLength = jetiExBusFrame[EXBUS_HEADER_MSG_LEN];
            return;
        }

        jetiExBusFrameReset();                  // not a valid frame
        jetiExBusFrameState = EXBUS_STATE_ZERO;
        jetiExBusRequestState = EXBUS_STATE_ZERO;
        return;
    }

    // Done?
    if (jetiExBusFrameLength == jetiExBusFramePosition) {
        if (jetiExBusFrameState == EXBUS_STATE_IN_PROGRESS)
            jetiExBusFrameState = EXBUS_STATE_RECEIVED;
        if (jetiExBusRequestState == EXBUS_STATE_IN_PROGRESS) {
            jetiExBusRequestState = EXBUS_STATE_RECEIVED;
            jetiTimeStampRequest = micros();
        }

        jetiExBusFrameReset();
    }
}


// Check if it is time to read a frame from the data...
static uint8_t jetiExBusFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    if (jetiExBusFrameState != EXBUS_STATE_RECEIVED)
        return RX_FRAME_PENDING;

    if (calcCRC16(jetiExBusChannelFrame, jetiExBusChannelFrame[EXBUS_HEADER_MSG_LEN]) == 0) {
        jetiExBusDecodeChannelFrame(jetiExBusChannelFrame);
        jetiExBusFrameState = EXBUS_STATE_ZERO;
        return RX_FRAME_COMPLETE;
    } else {
        jetiExBusFrameState = EXBUS_STATE_ZERO;
        return RX_FRAME_PENDING;
    }
}


static uint16_t jetiExBusReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    if (chan >= rxRuntimeConfig->channelCount)
        return 0;

    return (jetiExBusChannelData[chan]);
}


#ifdef USE_TELEMETRY
void enableGpsTelemetry(bool enable)
{
    if (enable) {
        bitArraySet(&exSensorEnabled, EX_GPS_SATS);
        bitArraySet(&exSensorEnabled, EX_GPS_LONG);
        bitArraySet(&exSensorEnabled, EX_GPS_LAT);
        bitArraySet(&exSensorEnabled, EX_GPS_SPEED);
        bitArraySet(&exSensorEnabled, EX_GPS_DISTANCE_TO_HOME);
        bitArraySet(&exSensorEnabled, EX_GPS_DIRECTION_TO_HOME);
        bitArraySet(&exSensorEnabled, EX_GPS_HEADING);
        bitArraySet(&exSensorEnabled, EX_GPS_ALTITUDE);
    } else {
        bitArrayClr(&exSensorEnabled, EX_GPS_SATS);
        bitArrayClr(&exSensorEnabled, EX_GPS_LONG);
        bitArrayClr(&exSensorEnabled, EX_GPS_LAT);
        bitArrayClr(&exSensorEnabled, EX_GPS_SPEED);
        bitArrayClr(&exSensorEnabled, EX_GPS_DISTANCE_TO_HOME);
        bitArrayClr(&exSensorEnabled, EX_GPS_DIRECTION_TO_HOME);
        bitArrayClr(&exSensorEnabled, EX_GPS_HEADING);
        bitArrayClr(&exSensorEnabled, EX_GPS_ALTITUDE);
    }
}

/*
 * -----------------------------------------------
 *  Jeti Ex Bus Telemetry
 * -----------------------------------------------
 */
void initJetiExBusTelemetry(void)
{
    // Init Ex Bus Frame header
    jetiExBusTelemetryFrame[EXBUS_HEADER_SYNC] = 0x3B;       // Startbytes
    jetiExBusTelemetryFrame[EXBUS_HEADER_REQ] = 0x01;
    jetiExBusTelemetryFrame[EXBUS_HEADER_DATA_ID] = 0x3A;    // Ex Telemetry

    // Init Ex Telemetry header
    uint8_t *jetiExTelemetryFrame = &jetiExBusTelemetryFrame[EXBUS_HEADER_DATA];

    jetiExTelemetryFrame[EXTEL_HEADER_SYNC] = 0x9F;              // Startbyte
    jetiExTelemetryFrame[EXTEL_HEADER_USN_LB] = 0x1E;            // Serial Number 4 Byte
    jetiExTelemetryFrame[EXTEL_HEADER_USN_HB] = 0xA4;
    jetiExTelemetryFrame[EXTEL_HEADER_LSN_LB] = 0x00;            // increment by telemetry count (%16) > only 15 values per device possible
    jetiExTelemetryFrame[EXTEL_HEADER_LSN_HB] = 0x00;
    jetiExTelemetryFrame[EXTEL_HEADER_RES] = 0x00;               // reserved, by default 0x00

    bitArraySet(&exSensorEnabled, EX_VOLTAGE);

    bitArraySet(&exSensorEnabled, EX_CURRENT);

    bitArraySet(&exSensorEnabled, EX_POWER);
    bitArraySet(&exSensorEnabled, EX_CAPACITY);
    if (sensors(SENSOR_BARO)) {
        bitArraySet(&exSensorEnabled, EX_ALTITUDE);
#ifdef USE_VARIO
        bitArraySet(&exSensorEnabled, EX_VARIO);
#endif
    }
    if (sensors(SENSOR_ACC)) {
        bitArraySet(&exSensorEnabled, EX_ROLL_ANGLE);
        bitArraySet(&exSensorEnabled, EX_PITCH_ANGLE);
        bitArraySet(&exSensorEnabled, EX_GFORCE_X);
        bitArraySet(&exSensorEnabled, EX_GFORCE_Y);
        bitArraySet(&exSensorEnabled, EX_GFORCE_Z);
    }
    if (sensors(SENSOR_MAG)) {
        bitArraySet(&exSensorEnabled, EX_HEADING);
    }

    if (isMixerUsingTiltrotor()) {
        bitArraySet(&exSensorEnabled, EX_NACELLE_ANGLE);
    }

    enableGpsTelemetry(feature(FEATURE_GPS));

    firstActiveSensor = getNextActiveSensor(0);     // find the first active sensor
}

void createExTelemetryTextMessage(uint8_t *exMessage, uint8_t messageID, const exBusSensor_t *sensor)
{
    uint8_t labelLength = strlen(sensor->label);
    uint8_t unitLength = strlen(sensor->unit);

    exMessage[EXTEL_HEADER_TYPE_LEN] = EXTEL_OVERHEAD + labelLength + unitLength;
    exMessage[EXTEL_HEADER_LSN_LB] = messageID & 0xF0;                              // Device ID
    exMessage[EXTEL_HEADER_ID] = messageID & 0x0F;                                  // Sensor ID (%16)
    exMessage[EXTEL_HEADER_DATA] = (labelLength << 3) + unitLength;

    memcpy(&exMessage[EXTEL_HEADER_DATA + 1], sensor->label, labelLength);
    memcpy(&exMessage[EXTEL_HEADER_DATA + 1 + labelLength], sensor->unit, unitLength);

    exMessage[exMessage[EXTEL_HEADER_TYPE_LEN] + EXTEL_CRC_LEN] = calcCRC8(&exMessage[EXTEL_HEADER_TYPE_LEN], exMessage[EXTEL_HEADER_TYPE_LEN]);
}

uint32_t calcGpsDDMMmmm(int32_t value, bool isLong)
{
    uint32_t absValue = ABS(value);
    uint16_t deg16 = absValue / GPS_DEGREES_DIVIDER;
    uint16_t min16 = (absValue - deg16 * GPS_DEGREES_DIVIDER) * 6 / 1000;

    exGps.vInt = 0;
    exGps.vWord[0] = min16;
    exGps.vWord[1] = deg16;
    exGps.vWord[1] |= isLong ? 0x2000 : 0;
    exGps.vWord[1] |= (value < 0) ? 0x4000 : 0;

    return exGps.vInt;
}


int32_t getSensorValue(uint8_t sensor)
{
    switch (sensor) {
    case EX_VOLTAGE:
        return (getBatteryVoltage()+5)/10;
        break;

    case EX_CURRENT:
        return getAmperage();
        break;

    case EX_ALTITUDE:
        return baro.BaroAlt;
        break;

    case EX_CAPACITY:
        return getMAhDrawn();
        break;

    case EX_POWER:
        return (getBatteryVoltage() * getAmperage() / 1000);
        break;

    case EX_ROLL_ANGLE:
        return attitude.values.roll;
        break;

    case EX_PITCH_ANGLE:
        return attitude.values.pitch;
        break;

    case EX_HEADING:
        return attitude.values.yaw;
        break;

#ifdef USE_VARIO
    case EX_VARIO:
        return getEstimatedVario();
        break;
#endif

#ifdef USE_GPS
    case EX_GPS_SATS:
        return gpsSol.numSat;
    break;

    case EX_GPS_LONG:
        return calcGpsDDMMmmm(gpsSol.llh.lon, true);
    break;

    case EX_GPS_LAT:
        return calcGpsDDMMmmm(gpsSol.llh.lat, false);
    break;

    case EX_GPS_SPEED:
        return gpsSol.groundSpeed;
    break;

    case EX_GPS_DISTANCE_TO_HOME:
        return GPS_distanceToHome;
    break;

    case EX_GPS_DIRECTION_TO_HOME:
        return GPS_directionToHome;
    break;

    case EX_GPS_HEADING:
        return gpsSol.groundCourse;
    break;

    case EX_GPS_ALTITUDE:
        return gpsSol.llh.alt;
    break;
#endif

    case EX_GFORCE_X:
       return (int16_t)(((float)acc.accADCf[X]) * 100);
    break;

    case EX_GFORCE_Y:
       return (int16_t)(((float)acc.accADCf[Y]) * 100);
    break;

    case EX_GFORCE_Z:
        return (int16_t)(((float)acc.accADCf[Z]) * 100);
    break;

    case EX_NACELLE_ANGLE:
        if (isMixerUsingTiltrotor()) {
        	return (int16_t) tiltlive.nacelle/10; // given in 10*deg
        }
        else
        {
        	return 0;
        }

    break;

    case EX_INTEGRATOR_PITCH:
       	return (int16_t) lrintf(10 * axisPID_I[FD_PITCH] );
    break;

    case EX_INTEGRATOR_ROLL:
       	return (int16_t) lrintf(10 * axisPID_I[FD_ROLL] );
    break;

    case EX_INTEGRATOR_YAW:
      	return (int16_t) lrintf(10 * axisPID_I[FD_YAW] );
    break;

    case EX_FLIGHT_MODE:
       	return (int16_t) frskyGetFlightMode();
    break;

    default:
        return -1;
    }
}

uint8_t getNextActiveSensor(uint8_t currentSensor)
{
    while( ++currentSensor < JETI_EX_SENSOR_COUNT) {
        if (bitArrayGet(&exSensorEnabled, currentSensor)) {
            break;
        }
    }
    if (currentSensor == JETI_EX_SENSOR_COUNT ) {
        currentSensor = firstActiveSensor;
    }
    return currentSensor;
}

uint8_t createExTelemetryValueMessage(uint8_t *exMessage, uint8_t item)
{
    uint8_t startItem = item;
    uint8_t sensorItemMaxGroup = (item & 0xF0) + 0x10;
    uint8_t iCount;
    uint8_t messageSize;
    uint32_t sensorValue;

    exMessage[EXTEL_HEADER_LSN_LB] = item & 0xF0;                       // Device ID
    uint8_t *p = &exMessage[EXTEL_HEADER_ID];

    while (item < sensorItemMaxGroup) {
        *p++ = ((item & 0x0F) << 4) | jetiExSensors[item].exDataType;   // Sensor ID (%16) | EX Data Type

        sensorValue = getSensorValue(item);
        iCount = exDataTypeLen[jetiExSensors[item].exDataType];

        while (iCount > 1) {
            *p++ = sensorValue;
            sensorValue = sensorValue >> 8;
            iCount--;
        }
        if (jetiExSensors[item].exDataType != EX_TYPE_GPS) {
            *p++ = (sensorValue & 0x9F) | jetiExSensors[item].decimals;
        } else {
            *p++ = sensorValue;
        }

        item = getNextActiveSensor(item);

        if (startItem >= item) {
            break;
        }

        if ((p - &exMessage[EXTEL_HEADER_ID]) + exDataTypeLen[jetiExSensors[item].exDataType] + 1 >= EXTEL_MAX_PAYLOAD) {
            break;
        }
    }
    messageSize = (EXTEL_HEADER_LEN + (p-&exMessage[EXTEL_HEADER_ID]));
    exMessage[EXTEL_HEADER_TYPE_LEN] = EXTEL_DATA_MSG | messageSize;
    exMessage[messageSize + EXTEL_CRC_LEN] = calcCRC8(&exMessage[EXTEL_HEADER_TYPE_LEN], messageSize);

    return item;        // return the next item
}

void createExBusMessage(uint8_t *exBusMessage, uint8_t *exMessage, uint8_t packetID)
{
    uint16_t crc16;

    exBusMessage[EXBUS_HEADER_PACKET_ID] = packetID;
    exBusMessage[EXBUS_HEADER_SUBLEN] = (exMessage[EXTEL_HEADER_TYPE_LEN] & EXTEL_UNMASK_TYPE) + 2;    // +2: startbyte & CRC8
    exBusMessage[EXBUS_HEADER_MSG_LEN] = EXBUS_OVERHEAD + exBusMessage[EXBUS_HEADER_SUBLEN];

    crc16 = calcCRC16(exBusMessage, exBusMessage[EXBUS_HEADER_MSG_LEN] - EXBUS_CRC_LEN);
    exBusMessage[exBusMessage[EXBUS_HEADER_MSG_LEN] - 2] = crc16;
    exBusMessage[exBusMessage[EXBUS_HEADER_MSG_LEN] - 1] = crc16 >> 8;
}

void checkJetiExBusTelemetryState(void)
{
    return;
}

void handleJetiExBusTelemetry(void)
{
    static uint16_t framesLost = 0; // only for debug
    static uint8_t item = 0;
    uint32_t timeDiff;

    // Check if we shall reset frame position due to time
    if (jetiExBusRequestState == EXBUS_STATE_RECEIVED) {

        // to prevent timing issues from request to answer - max. 4ms
        timeDiff = micros() - jetiTimeStampRequest;

        if (timeDiff > 3000) {   // include reserved time
            jetiExBusRequestState = EXBUS_STATE_ZERO;
            framesLost++;
            return;
        }

        if ((jetiExBusRequestFrame[EXBUS_HEADER_DATA_ID] == EXBUS_EX_REQUEST) && (calcCRC16(jetiExBusRequestFrame, jetiExBusRequestFrame[EXBUS_HEADER_MSG_LEN]) == 0)) {
            if (serialRxBytesWaiting(jetiExBusPort) == 0) {
                jetiExBusTransceiveState = EXBUS_TRANS_TX;
                item = sendJetiExBusTelemetry(jetiExBusRequestFrame[EXBUS_HEADER_PACKET_ID], item);
                jetiExBusRequestState = EXBUS_STATE_PROCESSED;
                return;
            }
        } else {
            jetiExBusRequestState = EXBUS_STATE_ZERO;
            return;
        }
    }

    // check the state if transmit is ready
    if (jetiExBusTransceiveState == EXBUS_TRANS_IS_TX_COMPLETED) {
        if (isSerialTransmitBufferEmpty(jetiExBusPort)) {
            jetiExBusTransceiveState = EXBUS_TRANS_RX;
            jetiExBusRequestState = EXBUS_STATE_ZERO;
        }
    }
}

uint8_t sendJetiExBusTelemetry(uint8_t packetID, uint8_t item)
{
    static uint8_t sensorDescriptionCounter = 0xFF;
    static uint8_t requestLoop = 0xFF;
    static bool allSensorsActive = true;
    uint8_t *jetiExTelemetryFrame = &jetiExBusTelemetryFrame[EXBUS_HEADER_DATA];

    if (requestLoop) {
        while( ++sensorDescriptionCounter < JETI_EX_SENSOR_COUNT) {
            if (bitArrayGet(&exSensorEnabled, sensorDescriptionCounter) || (jetiExSensors[sensorDescriptionCounter].exDataType == EX_TYPE_DES)) {
                break;
            }
        }
        if (sensorDescriptionCounter == JETI_EX_SENSOR_COUNT ) {
            sensorDescriptionCounter = 0;
        }

        createExTelemetryTextMessage(jetiExTelemetryFrame, sensorDescriptionCounter, &jetiExSensors[sensorDescriptionCounter]);
        createExBusMessage(jetiExBusTelemetryFrame, jetiExTelemetryFrame, packetID);
        requestLoop--;
        if (requestLoop == 0) {
            item = firstActiveSensor;
            if (feature(FEATURE_GPS)) {
                enableGpsTelemetry(false);
                allSensorsActive = false;
            }
        }
    } else {
        item = createExTelemetryValueMessage(jetiExTelemetryFrame, item);
        createExBusMessage(jetiExBusTelemetryFrame, jetiExTelemetryFrame, packetID);

        if (!allSensorsActive) {
            if (sensors(SENSOR_GPS)) {
                enableGpsTelemetry(true);
                allSensorsActive = true;
            }
        }
    }

    serialWriteBuf(jetiExBusPort, jetiExBusTelemetryFrame, jetiExBusTelemetryFrame[EXBUS_HEADER_MSG_LEN]);
    jetiExBusTransceiveState = EXBUS_TRANS_IS_TX_COMPLETED;

    return item;
}
#endif // TELEMETRY

bool jetiExBusInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);

    rxRuntimeConfig->channelCount = JETIEXBUS_CHANNEL_COUNT;
    rxRuntimeConfig->rxRefreshRate = 5500;

    rxRuntimeConfig->rcReadRawFn = jetiExBusReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = jetiExBusFrameStatus;

    jetiExBusFrameReset();

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);

    if (!portConfig) {
        return false;
    }

    jetiExBusPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        jetiExBusDataReceive,
        NULL,
        JETIEXBUS_BAUDRATE,
        MODE_RXTX,
        JETIEXBUS_OPTIONS | (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)
        );

    serialSetMode(jetiExBusPort, MODE_RX);
    return jetiExBusPort != NULL;
}
#endif // USE_SERIALRX_JETIEXBUS
