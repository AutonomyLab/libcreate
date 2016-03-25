/**
Software License Agreement (BSD)

\file      create.h
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

#ifndef CREATE_H
#define CREATE_H

#include <boost/shared_ptr.hpp>
#include <string>
#include <unistd.h>

#include "create/serial.h"
#include "create/data.h"
#include "create/types.h"
#include "create/util.h"

namespace create {
  class Create {
    private:
      enum CreateLED {
        LED_DEBRIS = 1,
        LED_SPOT = 2,
        LED_DOCK = 4,
        LED_CHECK = 8
      };

      RobotModel model;

      uint8_t mainMotorPower;
      uint8_t sideMotorPower;
      uint8_t vacuumMotorPower;

      // LEDs
      uint8_t debrisLED;
      uint8_t spotLED;
      uint8_t dockLED;
      uint8_t checkLED;
      uint8_t powerLED;
      uint8_t powerLEDIntensity;

      create::Pose pose;
      create::Vel vel;

      uint32_t prevTicksLeft;
      uint32_t prevTicksRight;
      float prevLeftVel;
      float prevRightVel;
      bool firstOnData;
      util::timestamp_t prevOnDataTime;

      void init();
      bool updateLEDs();
      void onData();

    protected:
      boost::shared_ptr<create::Data> data;
      boost::shared_ptr<create::Serial> serial;

    public:
      /* Default constructor.
       * Does not attempt to establish serial connection to Create.
       */
      Create(RobotModel = CREATE_2);

      /* Attempts to establish serial connection to Create.
       */
      Create(const std::string& port, const int& baud, RobotModel = CREATE_2);

      ~Create();

      /* Make a serial connection to Create.
       * This is the first thing that should be done after instantiated this class.
       * \return true if a successful connection is established, false otherwise.
       */
      bool connect(const std::string& port, const int& baud);

      inline bool connected() const { return serial->connected(); };

      /* Disconnect from serial.
       */
      void disconnect();

      /* Resets as if you have removed the battery.
       * Changes mode to MODE_PASSIVE.
       */
      // TODO
      //void reset();

      // TODO
      //void setBaud(int baudcode);

      /* Change Create mode.
       */
      bool setMode(const create::CreateMode& mode);

      /* Starts a cleaning mode.
       * Changes mode to MODE_PASSIVE.
       */
      bool clean(const create::CleanMode& mode = CLEAN_DEFAULT);

      /* Starts the docking behaviour.
       * Changes mode to MODE_PASSIVE.
       */
      bool dock() const;

      /* Sets the internal clock of Create.
       * \param day in range [0, 6]
       * \param hour in range [0, 23]
       * \param min in range [0, 59]
       */
      bool setDate(const create::DayOfWeek& day, const uint8_t& hour, const uint8_t& min) const;

      /* Set the average wheel velocity and turning radius of Create.
       * \param vel is in m/s
       * \param radius in meters.
       */
      //void driveRadius(const float& vel, const float& radius) const;

      /* Set the velocities for the left and right wheels (m/s).
       */
      bool driveWheels(const float& leftWheel, const float& rightWheel) const;

      /* Set the PWM for each wheel.
       */
      // TODO
      //void drivePWM(const int16_t& leftWheel, const int16_t& rightWheel) const;

      /* Set the forward and angular velocity of Create.
       * \param xVel in m/s
       * \param angularVel in rads/s
       */
      bool drive(const float& xVel, const float& angularVel) const;

      /* Set the power to the side brush motor.
       * \param power is in the range [-1, 1]
       */
      bool setSideMotor(const float& power);

      /* Set the power to the main brush motor.
       * \param power is in the range [-1, 1]
       */
      bool setMainMotor(const float& power);

      /* Set the power to the vacuum motor.
       * \param power is in the range [0, 1]
       */
      bool setVacuumMotor(const float& power);

      /* Set the power of all motors.
       * \param mainPower in the range [-1, 1]
       * \param sidePower in the range [-1, 1]
       * \param vacuumPower in the range [0, 1]
       */
      bool setAllMotors(const float& mainPower, const float& sidePower, const float& vacuumPower);

      /* Set the blue "debris" LED on/off.
       */
      bool enableDebrisLED(const bool& enable);

      /* Set the green "spot" LED on/off.
       */
      bool enableSpotLED(const bool& enable);

      /* Set the green "dock" LED on/off.
       */
      bool enableDockLED(const bool& enable);

      /* Set the orange "check Create" LED on/off.
       */
      bool enableCheckRobotLED(const bool& enable);

      /* Set the center power LED.
       * \param power in range [0, 255] where 0 = green and 255 = red
       * \param intensity in range [0, 255]
       */
      bool setPowerLED(const uint8_t& power, const uint8_t& intensity = 255);

      // TODO
      //void setScheduleLED(...);

      /* Set the four 7-segment display digits from left to right.
       */
      // TODO
      //void setDigits(uint8_t digit1, uint8_t digit2,
      //               uint8_t digit3, uint8_t digit4);

      // TODO
      // pushButton(...);

      /* Set the four 7-segment display digits from left to right with ASCII codes.
       * Any code out side the accepted ascii ranges results in blank display.
       *  \param digit1 is left most digit with ascii range [32, 126]
       *  \param digit2 is second to left digit with ascii range [32, 126]
       *  \param digit3 is second to right digit with ascii range [32, 126]
       *  \param digit4 is right most digit with ascii range [32, 126]
       */
      bool setDigitsASCII(const uint8_t& digit1, const uint8_t& digit2,
                          const uint8_t& digit3, const uint8_t& digit4) const;

      /* Defines a song from the provided notes and labels it with a song number.
       * \param songNumber can be one of four possible song slots, [0, 4]
       * \param songLength is the number of notes, maximum 16.
       *        length(notes) = length(durations) = songLength should be true.
       * \param notes is a sequence of notes. Each note is in the range [31, 127].
       *        Anything outside this range is considered a rest note.
       * \param durations for each note in fractions of a second from the range [0, 4)
       */
      bool defineSong(const uint8_t& songNumber,
                      const uint8_t& songLength,
                      const uint8_t* notes,
                      const float* durations) const;

      /* Play a previously created song.
       * This command will not work if a song was not already defined with the specified song number.
       */
      bool playSong(const uint8_t& songNumber) const;

      // TODO
      //void registerCallback(...);

      /* True if a left or right wheeldrop is detected.
       */
      bool isWheeldrop() const;

      /* Returns true if left bumper is pressed, false otherwise.
       */
      bool isLeftBumper() const;

      /* Returns true if right bumper is pressed, false otherwise.
       */
      bool isRightBumper() const;

      /* True if wall is seen to right of Create, false otherwise.
       */
      bool isWall() const;

      /* True if there are any cliff detections, false otherwise.
       */
      bool isCliff() const;

      //TODO
      //bool isVirtualWall() const;

      //TODO
      //bool isWheelOvercurrent() const;

      /* Get level of the dirt detect sensor.
       * \return value in range [0, 255]
       */
      uint8_t getDirtDetect() const;

      /* Get value of 8-bit IR character currently being received by omnidirectional sensor.
       * \return value in range [0, 255]
       */
      uint8_t getIROmni() const;

      /* Get value of 8-bit IR character currently being received by left sensor.
       * \return value in range [0, 255]
       */

      uint8_t getIRLeft() const;

      /* Get value of 8-bit IR character currently being received by right sensor.
       * \return value in range [0, 255]
       */
      uint8_t getIRRight() const;

      bool isCleanButtonPressed() const;
      bool isClockButtonPressed() const;
      bool isScheduleButtonPressed() const;
      bool isDayButtonPressed() const;
      bool isHourButtonPressed() const;
      bool isMinButtonPressed() const;
      bool isDockButtonPressed() const;
      bool isSpotButtonPressed() const;

      /* Get battery voltage.
       * \return value in millivolts
       */
      uint16_t getVoltage() const;

      /* Get current flowing in/out of battery.
       * A positive current implies Create is charging. 
       * \return value in milliamps
       */
      int16_t getCurrent() const;

      /* Get the temperature of battery.
       * \return value in Celsius
       */
      int8_t getTemperature() const;

      /* Get battery's remaining charge.
       * \return value in milliamp-hours
       */
      uint16_t getBatteryCharge() const;

      /* Get estimated battery charge capacity.
       * \return in milliamp-hours
       */
      uint16_t getBatteryCapacity() const;

      bool isIRDetectLeft() const;
      bool isIRDetectFrontLeft() const;
      bool isIRDetectCenterLeft() const;
      bool isIRDetectRight() const;
      bool isIRDetectFrontRight() const;
      bool isIRDetectCenterRight() const;

      uint16_t getDistLeft() const;
      uint16_t getDistFrontLeft() const;
      uint16_t getDistCenterLeft() const;
      uint16_t getDistRight() const;
      uint16_t getDistFrontRight() const;
      uint16_t getDistCenterRight() const;

      /* Return true if Create is moving forward.
       */
      bool isMovingForward() const;

      /* Get the current charging state.
       */
      create::ChargingState getChargingState() const;

      /* Get the current mode reported by Create.
       */
      create::CreateMode getMode() const;

      /* Get the estimated position of Create based on it's wheel encoders.
       */
      const create::Pose& getPose() const;

      /* Get the estimated velocity of Create based on wheel encoders.
       */
      const create::Vel& getVel() const;

      /* Get the number of corrupt serial packets since first connecting to Create.
       */
      uint64_t getNumCorruptPackets() const;

      /* Get the total number of serial packets (including corrupt packets) since first connecting to Create.
       */
      uint64_t getTotalPackets() const;
  };  // end Create class

}  // namespace create

#endif  // CREATE_DRIVER_H
