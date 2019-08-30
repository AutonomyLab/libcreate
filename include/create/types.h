/**
Software License Agreement (BSD)

\file      types.h
\authors   Jacob Perron <jperron@sfu.ca>
\copyright Copyright (c) 2015, Autonomy Lab (Simon Fraser University), All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CREATE_TYPES_H
#define CREATE_TYPES_H

#include <vector>
#include <stdint.h>
#include <string>
#include <stdexcept>

namespace create {
  enum ProtocolVersion {
    V_1 = 1,
    V_2 = 2,
    V_3 = 4,
    V_ALL = 0xFFFFFFFF
  };

  class RobotModel {
    public:
      bool operator==(RobotModel& other) const;
      operator uint32_t() const;

      uint32_t getId() const;
      ProtocolVersion getVersion() const;
      float getAxleLength() const;
      unsigned int getBaud() const;
      float getMaxVelocity() const;
      float getWheelDiameter() const;

      /**
       * \brief Compatible with Roomba 400 series and earlier.
       */
      static RobotModel ROOMBA_400;

      /**
       * \brief Compatible with Create 1 or Roomba 500 series.
       */
      static RobotModel CREATE_1;

      /**
       * \brief Compatible with Create 2 or Roomba 600 series and greater.
       */
      static RobotModel CREATE_2;

    private:
      uint32_t id;
      ProtocolVersion version;
      float axleLength;
      unsigned int baud;
      float maxVelocity;
      float wheelDiameter;

      RobotModel(const ProtocolVersion version, const float axleLength, const unsigned int baud, const float maxVelocity = 0.5, const float wheelDiameter = 0.078);
      static uint32_t nextId;
  };

  enum SensorPacketID {
    ID_GROUP_0 = 0,
    ID_GROUP_1 = 1,
    ID_GROUP_2 = 2,
    ID_GROUP_3 = 3,
    ID_GROUP_4 = 4,
    ID_GROUP_5 = 5,
    ID_GROUP_6 = 6,
    ID_GROUP_100 = 100,
    ID_GROUP_101 = 101,
    ID_GROUP_106 = 106,
    ID_GROUP_107 = 107,
    ID_BUMP_WHEELDROP = 7,
    ID_WALL = 8,
    ID_CLIFF_LEFT = 9,
    ID_CLIFF_FRONT_LEFT = 10,
    ID_CLIFF_FRONT_RIGHT = 11,
    ID_CLIFF_RIGHT = 12,
    ID_VIRTUAL_WALL = 13,
    ID_OVERCURRENTS = 14,
    ID_DIRT_DETECT_LEFT = 15,
    ID_DIRT_DETECT_RIGHT = 16,
    ID_IR_OMNI = 17,
    ID_IR_LEFT = 52,
    ID_IR_RIGHT = 53,
    ID_BUTTONS = 18,
    ID_DISTANCE = 19,
    ID_ANGLE = 20,
    ID_CHARGE_STATE = 21,
    ID_VOLTAGE = 22,
    ID_CURRENT = 23,
    ID_TEMP = 24,
    ID_CHARGE = 25,
    ID_CAPACITY = 26,
    ID_WALL_SIGNAL = 27,
    ID_CLIFF_LEFT_SIGNAL = 28,
    ID_CLIFF_FRONT_LEFT_SIGNAL = 29,
    ID_CLIFF_FRONT_RIGHT_SIGNAL = 30,
    ID_CLIFF_RIGHT_SIGNAL = 31,
    ID_CARGO_BAY_DIGITAL_INPUTS = 32,
    ID_CARGO_BAY_ANALOG_SIGNAL = 33,
    ID_CHARGE_SOURCE = 34,
    ID_OI_MODE = 35,
    ID_SONG_NUM = 36,
    ID_PLAYING = 37,
    ID_NUM_STREAM_PACKETS = 38,
    ID_VEL = 39,
    ID_RADIUS = 40,
    ID_RIGHT_VEL = 41,
    ID_LEFT_VEL = 42,
    ID_LEFT_ENC = 43,
    ID_RIGHT_ENC = 44,
    ID_LIGHT = 45,
    ID_LIGHT_LEFT = 46,
    ID_LIGHT_FRONT_LEFT = 47,
    ID_LIGHT_CENTER_LEFT = 48,
    ID_LIGHT_CENTER_RIGHT = 49,
    ID_LIGHT_FRONT_RIGHT = 50,
    ID_LIGHT_RIGHT = 51,
    ID_LEFT_MOTOR_CURRENT = 54,
    ID_RIGHT_MOTOR_CURRENT = 55,
    ID_MAIN_BRUSH_CURRENT = 56,
    ID_SIDE_BRUSH_CURRENT = 57,
    ID_STASIS = 58,
    ID_NUM = 52
  };

  enum Opcode {
    OC_START = 128,
    OC_RESET = 7,
    OC_STOP = 173,
    OC_BAUD = 129,
    OC_CONTROL = 130,
    OC_SAFE = 131,
    OC_FULL = 132,
    OC_CLEAN = 135,
    OC_MAX = 136,
    OC_SPOT = 134,
    OC_DOCK = 143,
    OC_POWER = 133,
    OC_SCHEDULE = 167,
    OC_DATE = 168,
    OC_DRIVE = 137,
    OC_DRIVE_DIRECT = 145,
    OC_DRIVE_PWM = 146,
    OC_MOTORS = 138,
    OC_MOTORS_PWM = 144,
    OC_LEDS = 139,
    OC_SCHEDULING_LEDS = 162,
    OC_DIGIT_LEDS_RAW = 163,
    OC_BUTTONS = 165,
    OC_DIGIT_LEDS_ASCII = 164,
    OC_SONG = 140,
    OC_PLAY = 141,
    OC_SENSORS= 142,
    OC_QUERY_LIST=149,
    OC_STREAM = 148,
    OC_TOGGLE_STREAM = 150
  };

  enum BAUDCODE {
    BAUD_300 = 0,
    BAUD_600 = 1,
    BAUD_1200 = 2,
    BAUD_2400 = 3,
    BAUD_4800 = 4,
    BAUD_9600 = 5,
    BAUD_14400 = 6,
    BAUD_19200 = 7,
    BAUD_28800 = 8,
    BAUD_38400 = 9,
    BAUD_57600 = 10,
    BAUD_115200 = 11
  };

  enum CreateMode {
    MODE_OFF = 0,
    MODE_PASSIVE = 1,
    MODE_SAFE = 2,
    MODE_FULL = 3,
    MODE_UNAVAILABLE = -1
  };

  enum CleanMode {
    CLEAN_DEFAULT = OC_CLEAN,
    CLEAN_MAX = OC_MAX,
    CLEAN_SPOT = OC_SPOT
  };

  enum ChargingState {
    CHARGE_NONE = 0,
    CHARGE_RECONDITION = 1,
    CHARGE_FULL = 2,
    CHARGE_TRICKLE = 3,
    CHARGE_WAITING = 4,
    CHARGE_FAULT = 5
  };

  enum DayOfWeek {
    SUN = 0,
    MON = 1,
    TUE = 2,
    WED = 3,
    THU = 4,
    FRI = 5,
    SAT = 6
  };

  enum IRChars {
    IR_CHAR_NONE = 0,
    IR_CHAR_LEFT = 129,
    IR_CHAR_FORWARD = 130,
    IR_CHAR_RIGHT = 131,
    IR_CHAR_SPOT = 132,
    IR_CHAR_MAX = 133,
    IR_CHAR_SMALL = 134,
    IR_CHAR_MEDIUM = 135,
    IR_CHAR_LARGE = 136,
    IR_CHAR_CLEAN = 136,
    IR_CHAR_PAUSE = 137,
    IR_CHAR_POWER = 138,
    IR_CHAR_ARC_LEFT = 139,
    IR_CHAR_ARC_RIGHT = 140,
    IR_CHAR_STOP = 141,
    IR_CHAR_DOWNLOAD = 142,
    IR_CHAR_SEEK_DOCK = 143,
    IR_CHAR_DOCK_RESERVED = 240,
    IR_CHAR_RED_BUOY = 248,
    IR_CHAR_GREEN_BUOY = 244,
    IR_CHAR_FORCE_FIELD = 242,
    IR_CHAR_RED_GREEN_BUOY = 252,
    IR_CHAR_RED_FORCE_FIELD = 250,
    IR_CHAR_GREEN_FORCE_FIELD = 246,
    IR_CHAR_RED_GREEN_FORCE_FIELD = 254,
    IR_CHAR_600_DOCK_RESERVED = 160,
    IR_CHAR_600_FORCE_FIELD = 161,
    IR_CHAR_600_GREEN_BUOY = 164,
    IR_CHAR_600_GREEN_FORCE_FIELD = 165,
    IR_CHAR_600_RED_BUOY = 168,
    IR_CHAR_600_RED_FORCE_FIELD = 169,
    IR_CHAR_600_RED_GREEN_BUOY = 172,
    IR_CHAR_600_RED_GREEN_FORCE_FIELD = 173,
    IR_CHAR_VIRTUAL_WALL = 162
  };

  /**
   * \brief Represents a robot pose.
   */
  struct Pose {
    float x;
    float y;
    float yaw;

    /**
     * \brief 3x3 covariance matrix in row-major order.
     */
    std::vector<float> covariance;
  };

  typedef Pose Vel;
} // namespace create

#endif // CREATE_TYPES_H
