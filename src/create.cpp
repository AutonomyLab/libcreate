#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <iostream>
#include <cmath>
#include <ctime>
#include <assert.h>

#include "create/create.h"

#define GET_DATA(id) (data->getPacket(id)->getData())
#define BOUND(val,min,max) (val<min?val=min:(val>max?val=max:val=val))

namespace create {

  // TODO: Handle SIGINT to do clean disconnect

  void Create::init() {
    mainMotorPower = 0;
    sideMotorPower = 0;
    vacuumMotorPower = 0;
    debrisLED = 0;
    spotLED = 0;
    dockLED = 0;
    checkLED = 0;
    powerLED = 0;
    powerLEDIntensity = 0;
    prevTicksLeft = 0;
    prevTicksRight = 0;
    firstOnData = true;
    pose.x = 0;
    pose.y = 0;
    pose.yaw = 0;
    vel.x = 0;
    vel.y = 0;
    vel.yaw = 0;
    data = boost::shared_ptr<Data>(new Data());
    serial = boost::make_shared<Serial>(data);
  }

  Create::Create() {
    init();
  }

  Create::Create(const std::string& dev, const int& baud) {
    init();
    serial->connect(dev, baud);
  }

  Create::~Create() {
    disconnect();
  }

  void Create::onData() {
    if (firstOnData) {
      // Initialize tick counts
      prevTicksLeft = GET_DATA(ID_LEFT_ENC);
      prevTicksRight = GET_DATA(ID_RIGHT_ENC);
      prevOnDataTime = util::getTimestamp();
      firstOnData = false;
    }

    // Get current time
    util::timestamp_t curTime = util::getTimestamp();
    float dt = (curTime - prevOnDataTime) / 1000000.0;

    // Get cumulative ticks (wraps around at 65535)
    uint16_t totalTicksLeft = GET_DATA(ID_LEFT_ENC);
    uint16_t totalTicksRight = GET_DATA(ID_RIGHT_ENC);
    // Compute ticks since last update
    int ticksLeft = totalTicksLeft - prevTicksLeft;
    int ticksRight = totalTicksRight - prevTicksRight;
    prevTicksLeft = totalTicksLeft;
    prevTicksRight = totalTicksRight;

    // Handle wrap around
    if (fabs(ticksLeft) > 0.9 * util::CREATE_2_MAX_ENCODER_TICKS) {
      ticksLeft = (ticksLeft % util::CREATE_2_MAX_ENCODER_TICKS) + 1;
    }
    if (fabs(ticksRight) > 0.9 * util::CREATE_2_MAX_ENCODER_TICKS) {
      ticksRight = (ticksRight % util::CREATE_2_MAX_ENCODER_TICKS) + 1;
    }

    // Compute distance travelled by each wheel
    float leftWheelDist = (ticksLeft / util::CREATE_2_TICKS_PER_REV)
                           * util::CREATE_2_WHEEL_DIAMETER * util::PI;
    float rightWheelDist = (ticksRight / util::CREATE_2_TICKS_PER_REV)
                            * util::CREATE_2_WHEEL_DIAMETER * util::PI;

    float wheelDistDiff = rightWheelDist - leftWheelDist;
    float deltaDist = (rightWheelDist + leftWheelDist) / 2.0;

    // Moving straight
    float deltaX, deltaY;
    if (fabs(wheelDistDiff) < util::EPS) {
      deltaX = deltaDist * cos(pose.yaw);
      deltaY = deltaDist * sin(pose.yaw);
      vel.yaw = 0;
    }
    else {
      float turnRadius = (util::CREATE_2_AXLE_LENGTH / 2.0) * (leftWheelDist + rightWheelDist) / wheelDistDiff;
      float deltaYaw = (rightWheelDist - leftWheelDist) / util::CREATE_2_AXLE_LENGTH;
      deltaX = turnRadius * (sin(pose.yaw + deltaYaw) - sin(pose.yaw));
      deltaY = turnRadius * (cos(pose.yaw + deltaYaw) - cos(pose.yaw));
      pose.yaw = util::normalizeAngle(pose.yaw + deltaYaw);
      if (fabs(dt) > util::EPS) {
        vel.yaw = deltaYaw / dt;
      }
      else {
        vel.yaw = 0;
      }
    }

    if (fabs(dt) > util::EPS) {
      vel.x = deltaX / dt;
      vel.y = deltaY / dt;
    }
    else {
      vel.x = 0;
      vel.y = 0;
    }

    pose.x += deltaDist * cos(pose.yaw);
    pose.y += deltaDist * sin(pose.yaw);

    prevOnDataTime = curTime;
    // Make user registered callbacks, if any
    // TODO
  }

  bool Create::connect(const std::string& port, const int& baud) {
    bool timeout = false;
    time_t start, now;
    float maxWait = 30; // seconds
    float retryInterval = 5; //seconds
    time(&start);
    while (!serial->connect(port, baud, boost::bind(&Create::onData, this)) && !timeout) {
      time(&now);
      if (difftime(now, start) > maxWait) {
        timeout = true;
        CERR("[create::Create] ", "failed to connect over serial: timeout");
      }
      else {
        usleep(retryInterval * 1000000);
        COUT("[create::Create] ", "retrying to establish serial connection...");
      }
    }

    return !timeout;
  }

  void Create::disconnect() {
    serial->disconnect();
    firstOnData = true;
  }

  //void Create::reset() {
  //  serial->sendOpcode(OC_RESET);
  //  serial->reset(); // better
    // TODO : Should we request reading packets again?
  //}

  bool Create::setMode(const CreateMode& mode) {
    return serial->sendOpcode((Opcode) mode);
  }

  bool Create::clean(const CleanMode& mode) {
    return serial->sendOpcode((Opcode) mode);
  }

  bool Create::dock() const {
    return serial->sendOpcode(OC_DOCK);
  }

  bool Create::setDate(const DayOfWeek& day, const uint8_t& hour, const uint8_t& min) const {
    if (day < 0 || day > 6 ||
        hour < 0 || hour > 23 ||
        min < 0 || min > 59)
      return false;

    uint8_t cmd[4] = { OC_DATE, day, hour, min };
    return serial->send(cmd, 4);
  }

  /*void Create::driveRadius(const float& vel, const float& radius) const {
    // Expects each parameter as two bytes each and in millimeters
    int16_t vel_mm = roundf(vel * 1000);
    int16_t radius_mm = roundf(radius * 1000);
    BOUND(vel_mm, -500, 500);

    // Consider special cases for radius
    if (radius_mm != 32768 && radius_mm != 32767 &&
        radius_mm != -1 && radius_mm != 1) {
      BOUND(radius_mm, -2000, 2000);
    }

    uint8_t cmd[5] = { OC_DRIVE,
                       vel_mm >> 8,
                       vel_mm & 0xff,
                       radius_mm >> 8,
                       radius_mm & 0xff
                     };

    serial->send(cmd, 5);
  }
  */

  bool Create::driveWheels(const float& leftVel, const float& rightVel) const {
    int16_t leftCmd = roundf(leftVel * 1000);
    int16_t rightCmd = roundf(rightVel * 1000);
    BOUND(leftCmd, -util::CREATE_2_MAX_VEL * 1000, util::CREATE_2_MAX_VEL * 1000);
    BOUND(rightCmd, -util::CREATE_2_MAX_VEL * 1000, util::CREATE_2_MAX_VEL * 1000);

    uint8_t cmd[5] = { OC_DRIVE_DIRECT,
                       rightCmd >> 8,
                       rightCmd & 0xff,
                       leftCmd >> 8,
                       leftCmd & 0xff
                     };
    return serial->send(cmd, 5);
  }

  /*void Create::drivePWM(const int16_t& leftPWM, const int16_t& rightPWM) const {
    uint8_t cmd[5] = { OC_DRIVE_PWM,
                       rightPWM >> 8,
                       rightPWM & 0xff,
                       leftPWM >> 8,
                       leftPWM & 0xff
                     };
    serial->send(cmd, 5);
  }
  */

  bool Create::drive(const float& xVel, const float& angularVel) const {
    // Compute left and right wheel velocities
    float leftVel = xVel - ((util::CREATE_2_AXLE_LENGTH / 2.0) * angularVel);
    float rightVel = xVel + ((util::CREATE_2_AXLE_LENGTH / 2.0) * angularVel);
    return driveWheels(leftVel, rightVel);
  }

  bool Create::setAllMotors(const float& main, const float& side, const float& vacuum) {
    if (main < -1.0 || main > 1.0 ||
        side < -1.0 || side > 1.0 ||
        vacuum < -1.0 || vacuum > 1.0)
      return false;

    mainMotorPower = roundf(main * 127);
    sideMotorPower = roundf(side * 127);
    vacuumMotorPower = roundf(vacuum * 127);

    uint8_t cmd[4] = { OC_MOTORS_PWM,
                        mainMotorPower,
                        sideMotorPower,
                        vacuumMotorPower
                      };

    return serial->send(cmd, 4);
  }

  bool Create::setMainMotor(const float& main) {
    return setAllMotors(main, sideMotorPower, vacuumMotorPower);
  }

  bool Create::setSideMotor(const float& side) {
    return setAllMotors(mainMotorPower, side, vacuumMotorPower);
  }

  bool Create::setVacuumMotor(const float& vacuum) {
    return setAllMotors(mainMotorPower, sideMotorPower, vacuum);
  }

  bool Create::updateLEDs() {
    uint8_t LEDByte = debrisLED + spotLED + dockLED + checkLED;
    uint8_t cmd[4] = { OC_LEDS,
                        LEDByte,
                        powerLED,
                        powerLEDIntensity
                      };

    return serial->send(cmd, 4);
  }

  bool Create::enableDebrisLED(const bool& enable) {
    if (enable)
      debrisLED = LED_DEBRIS;
    else
      debrisLED = 0;
    return updateLEDs();
  }

  bool Create::enableSpotLED(const bool& enable) {
    if (enable)
      spotLED = LED_SPOT;
    else
      spotLED = 0;
    return updateLEDs();
  }

  bool Create::enableDockLED(const bool& enable) {
    if (enable)
      dockLED = LED_DOCK;
    else
      dockLED = 0;
    return updateLEDs();
  }

  bool Create::enableCheckRobotLED(const bool& enable) {
    if (enable)
      checkLED = LED_CHECK;
    else
      checkLED = 0;
    return updateLEDs();
  }

  bool Create::setPowerLED(const uint8_t& power, const uint8_t& intensity) {
    powerLED = power;
    powerLEDIntensity = intensity;
    return updateLEDs();
  }

  //void Create::setDigits(uint8_t digit1, uint8_t digit2,
  //                       uint8_t digit3, uint8_t digit4) {
  //}

  bool Create::setDigitsASCII(const uint8_t& digit1, const uint8_t& digit2,
                              const uint8_t& digit3, const uint8_t& digit4) const {
    if (digit1 < 32 || digit1 > 126 ||
        digit2 < 32 || digit2 > 126 ||
        digit3 < 32 || digit3 > 126 ||
        digit4 < 32 || digit4 > 126)
      return false;

    uint8_t cmd[5] = { OC_DIGIT_LEDS_ASCII,
                        digit1,
                        digit2,
                        digit3,
                        digit4
                      };

    return serial->send(cmd, 5);
  }

  bool Create::defineSong(const uint8_t& songNumber,
                          const uint8_t& songLength,
                          const uint8_t* notes,
                          const float* durations) const {
    int i, j;
    uint8_t duration;
    uint8_t cmd[2 * songLength + 3];
    cmd[0] = OC_SONG;
    cmd[1] = songNumber;
    cmd[2] = songLength;
    j = 0;
    for (i = 3; i < 2 * songLength + 3; i = i + 2) {
      if (durations[j] < 0 || durations[j] >= 4)
        return false;
      duration = durations[j] * 64;
      cmd[i] = notes[j];
      cmd[i + 1] = duration;
      j++;
    }

    return serial->send(cmd, 2 * songLength + 3);
  }

  bool Create::playSong(const uint8_t& songNumber) const {
    if (songNumber < 0 || songNumber > 4)
      return false;
    uint8_t cmd[2] = { OC_PLAY, songNumber };
    return serial->send(cmd, 2);
  }

  bool Create::isWheeldrop() const {
    return (GET_DATA(ID_BUMP_WHEELDROP) & 0x0C) != 0;
  }

  bool Create::isLeftBumper() const {
    return (GET_DATA(ID_BUMP_WHEELDROP) & 0x02) != 0;
  }

  bool Create::isRightBumper() const {
    return (GET_DATA(ID_BUMP_WHEELDROP) & 0x01) != 0;
  }

  bool Create::isWall() const {
    return GET_DATA(ID_WALL) == 1;
  }

  bool Create::isCliff() const {
    return GET_DATA(ID_CLIFF_LEFT) == 1 ||
           GET_DATA(ID_CLIFF_FRONT_LEFT) == 1 ||
           GET_DATA(ID_CLIFF_FRONT_RIGHT) == 1 ||
           GET_DATA(ID_CLIFF_RIGHT) == 1;
  }

  uint8_t Create::getDirtDetect() const {
    return GET_DATA(ID_DIRT_DETECT);
  }

  uint8_t Create::getIROmni() const {
    return GET_DATA(ID_IR_OMNI);
  }

  uint8_t Create::getIRLeft() const {
    return GET_DATA(ID_IR_LEFT);
  }

  uint8_t Create::getIRRight() const {
    return GET_DATA(ID_IR_RIGHT);
  }

  ChargingState Create::getChargingState() const {
    uint8_t chargeState = GET_DATA(ID_CHARGE_STATE);
    assert(chargeState >= 0);
    assert(chargeState <= 5);
    return (ChargingState) chargeState;
  }

  bool Create::isCleanButtonPressed() const {
    return (GET_DATA(ID_BUTTONS) & 0x01) != 0;
  }

  // Not working
  bool Create::isClockButtonPressed() const {
    return (GET_DATA(ID_BUTTONS) & 0x80) != 0;
  }

  // Not working
  bool Create::isScheduleButtonPressed() const {
    return (GET_DATA(ID_BUTTONS) & 0x40) != 0;
  }

  bool Create::isDayButtonPressed() const {
    return (GET_DATA(ID_BUTTONS) & 0x20) != 0;
  }

  bool Create::isHourButtonPressed() const {
    return (GET_DATA(ID_BUTTONS) & 0x10) != 0;
  }

  bool Create::isMinButtonPressed() const {
    return (GET_DATA(ID_BUTTONS) & 0x08) != 0;
  }

  bool Create::isDockButtonPressed() const {
    return (GET_DATA(ID_BUTTONS) & 0x04) != 0;
  }

  bool Create::isSpotButtonPressed() const {
    return (GET_DATA(ID_BUTTONS) & 0x02) != 0;
  }

  uint16_t Create::getVoltage() const {
    return GET_DATA(ID_VOLTAGE);
  }

  uint16_t Create::getCurrent() const {
    return GET_DATA(ID_CURRENT);
  }

  uint8_t Create::getTemperature() const {
    return GET_DATA(ID_TEMP);
  }

  uint16_t Create::getBatteryCharge() const {
    return GET_DATA(ID_CHARGE);
  }

  uint16_t Create::getBatteryCapacity() const {
    return GET_DATA(ID_CAPACITY);
  }

  bool Create::isIRDetectLeft() const {
    return (GET_DATA(ID_LIGHT) & 0x01) != 0;
  }

  bool Create::isIRDetectFrontLeft() const {
    return (GET_DATA(ID_LIGHT) & 0x02) != 0;
  }

  bool Create::isIRDetectCenterLeft() const {
    return (GET_DATA(ID_LIGHT) & 0x04) != 0;
  }

  bool Create::isIRDetectCenterRight() const {
    return (GET_DATA(ID_LIGHT) & 0x08) != 0;
  }

  bool Create::isIRDetectFrontRight() const {
    return (GET_DATA(ID_LIGHT) & 0x10) != 0;
  }

  bool Create::isIRDetectRight() const {
    return (GET_DATA(ID_LIGHT) & 0x20) != 0;
  }

  bool Create::isMovingForward() const {
    return GET_DATA(ID_STASIS) == 1;
  }

  const Pose& Create::getPose() const {
    return pose;
  }

  const Vel& Create::getVel() const {
    return vel;
  }

  uint64_t Create::getNumCorruptPackets() const {
    return serial->getNumCorruptPackets();
  }

  uint64_t Create::getTotalPackets() const {
    return serial->getTotalPackets();
  }

} // end namespace
