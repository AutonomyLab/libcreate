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

  namespace ublas = boost::numeric::ublas;

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
    totalLeftDist = 0.0;
    totalRightDist = 0.0;
    firstOnData = true;
    pose.x = 0;
    pose.y = 0;
    pose.yaw = 0;
    pose.covariance = std::vector<float>(9, 0.0);
    vel.x = 0;
    vel.y = 0;
    vel.yaw = 0;
    vel.covariance = std::vector<float>(9, 0.0);
    poseCovar = Matrix(3, 3, 0.0);
    data = boost::shared_ptr<Data>(new Data(model));
    serial = boost::make_shared<Serial>(data);
  }

  Create::Create(RobotModel m) : model(m) {
    init();
  }

  Create::Create(const std::string& dev, const int& baud, RobotModel m) : model(m) {
    init();
    serial->connect(dev, baud);
  }

  Create::~Create() {
    disconnect();
  }

  Create::Matrix Create::addMatrices(const Matrix &A, const Matrix &B) const {
    int rows = A.size1();
    int cols = A.size2();

    assert(rows == B.size1());
    assert(cols == B.size2());

    Matrix C(rows, cols);
    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        const float a = A(i, j);
        const float b = B(i, j);
        if (util::willFloatOverflow(a, b)) {
          // If overflow, set to float min or max depending on direction of overflow
          C(i, j) = (a < 0.0) ? std::numeric_limits<float>::min() : std::numeric_limits<float>::max();
        }
        else {
          C(i, j) = a + b;
        }
      }
    }
    return C;
  }

  void Create::onData() {
    if (firstOnData) {
      if (model == CREATE_2) {
        // Initialize tick counts
        prevTicksLeft = GET_DATA(ID_LEFT_ENC);
        prevTicksRight = GET_DATA(ID_RIGHT_ENC);
      }
      prevOnDataTime = util::getTimestamp();
      firstOnData = false;
    }

    // Get current time
    util::timestamp_t curTime = util::getTimestamp();
    float dt = (curTime - prevOnDataTime) / 1000000.0;
    float deltaDist, deltaX, deltaY, deltaYaw, leftWheelDist, rightWheelDist;
    if (model == CREATE_1) {
      /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
       * Angle returned is NOT correct if your robot is using older firmware:      *
       * http://answers.ros.org/question/31935/createroomba-odometry/              *
       * TODO: Consider using velocity command as substitute for pose estimation.  *
       * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
      deltaDist = ((int16_t) GET_DATA(ID_DISTANCE)) / 1000.0; //mm -> m
      deltaYaw = ((int16_t) GET_DATA(ID_ANGLE)) * (util::PI / 180.0); // D2R
      deltaX = deltaDist * cos( util::normalizeAngle(pose.yaw + deltaYaw) );
      deltaY = deltaDist * sin( util::normalizeAngle(pose.yaw + deltaYaw) );
      const float deltaYawWheelDist = (util::CREATE_1_AXLE_LENGTH / 2.0) * deltaYaw;
      leftWheelDist = deltaDist - deltaYawWheelDist;
      rightWheelDist = deltaDist + deltaYawWheelDist;
    }
    else if (model == CREATE_2) {
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
      leftWheelDist = (ticksLeft / util::CREATE_2_TICKS_PER_REV)
                       * util::CREATE_2_WHEEL_DIAMETER * util::PI;
      rightWheelDist = (ticksRight / util::CREATE_2_TICKS_PER_REV)
                        * util::CREATE_2_WHEEL_DIAMETER * util::PI;

      float wheelDistDiff = rightWheelDist - leftWheelDist;
      deltaDist = (rightWheelDist + leftWheelDist) / 2.0;

      // Moving straight
      if (fabs(wheelDistDiff) < util::EPS) {
        deltaX = deltaDist * cos(pose.yaw);
        deltaY = deltaDist * sin(pose.yaw);
        deltaYaw = 0.0;
      }
      else {
        float turnRadius = (util::CREATE_2_AXLE_LENGTH / 2.0) * (leftWheelDist + rightWheelDist) / wheelDistDiff;
        deltaYaw = wheelDistDiff / util::CREATE_2_AXLE_LENGTH;
        deltaX = turnRadius * (sin(pose.yaw + deltaYaw) - sin(pose.yaw));
        deltaY = -turnRadius * (cos(pose.yaw + deltaYaw) - cos(pose.yaw));
      }
    } // if CREATE_2

    totalLeftDist += leftWheelDist;
    totalRightDist += rightWheelDist;

    if (fabs(dt) > util::EPS) {
      vel.x = deltaDist / dt;
      vel.y = 0.0;
      vel.yaw = deltaYaw / dt;
    }
    else {
      vel.x = 0.0;
      vel.y = 0.0;
      vel.yaw = 0.0;
    }

    // Update covariances
    // Ref: "Introduction to Autonomous Mobile Robots" (Siegwart 2004, page 189)
    float kr = 1.0; // TODO: Perform experiments to find these nondeterministic parameters
    float kl = 1.0;
    float cosYawAndHalfDelta = cos(pose.yaw + (deltaYaw / 2.0)); // deltaX?
    float sinYawAndHalfDelta = sin(pose.yaw + (deltaYaw / 2.0)); // deltaY?
    float distOverTwoWB = deltaDist / (util::CREATE_2_AXLE_LENGTH * 2.0);

    Matrix invCovar(2, 2);
    invCovar(0, 0) = kr * fabs(rightWheelDist);
    invCovar(0, 1) = 0.0;
    invCovar(1, 0) = 0.0;
    invCovar(1, 1) = kl * fabs(leftWheelDist);

    Matrix Finc(3, 2);
    Finc(0, 0) = (cosYawAndHalfDelta / 2.0) - (distOverTwoWB * sinYawAndHalfDelta);
    Finc(0, 1) = (cosYawAndHalfDelta / 2.0) + (distOverTwoWB * sinYawAndHalfDelta);
    Finc(1, 0) = (sinYawAndHalfDelta / 2.0) + (distOverTwoWB * cosYawAndHalfDelta);
    Finc(1, 1) = (sinYawAndHalfDelta / 2.0) - (distOverTwoWB * cosYawAndHalfDelta);
    Finc(2, 0) = (1.0 / util::CREATE_2_AXLE_LENGTH);
    Finc(2, 1) = (-1.0 / util::CREATE_2_AXLE_LENGTH);
    Matrix FincT = boost::numeric::ublas::trans(Finc);

    Matrix Fp(3, 3);
    Fp(0, 0) = 1.0;
    Fp(0, 1) = 0.0;
    Fp(0, 2) = (-deltaDist) * sinYawAndHalfDelta;
    Fp(1, 0) = 0.0;
    Fp(1, 1) = 1.0;
    Fp(1, 2) = deltaDist * cosYawAndHalfDelta;
    Fp(2, 0) = 0.0;
    Fp(2, 1) = 0.0;
    Fp(2, 2) = 1.0;
    Matrix FpT = boost::numeric::ublas::trans(Fp);

    Matrix velCovar = ublas::prod(invCovar, FincT);
    velCovar = ublas::prod(Finc, velCovar);

    vel.covariance[0] = velCovar(0, 0);
    vel.covariance[1] = velCovar(0, 1);
    vel.covariance[2] = velCovar(0, 2);
    vel.covariance[3] = velCovar(1, 0);
    vel.covariance[4] = velCovar(1, 1);
    vel.covariance[5] = velCovar(1, 2);
    vel.covariance[6] = velCovar(2, 0);
    vel.covariance[7] = velCovar(2, 1);
    vel.covariance[8] = velCovar(2, 2);

    Matrix poseCovarTmp = ublas::prod(poseCovar, FpT);
    poseCovarTmp = ublas::prod(Fp, poseCovarTmp);
    poseCovar = addMatrices(poseCovarTmp, velCovar);

    pose.covariance[0] = poseCovar(0, 0);
    pose.covariance[1] = poseCovar(0, 1);
    pose.covariance[2] = poseCovar(0, 2);
    pose.covariance[3] = poseCovar(1, 0);
    pose.covariance[4] = poseCovar(1, 1);
    pose.covariance[5] = poseCovar(1, 2);
    pose.covariance[6] = poseCovar(2, 0);
    pose.covariance[7] = poseCovar(2, 1);
    pose.covariance[8] = poseCovar(2, 2);

    // Update pose
    pose.x += deltaX;
    pose.y += deltaY;
    pose.yaw = util::normalizeAngle(pose.yaw + deltaYaw);

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
    switch (mode) {
      case MODE_OFF:
        return serial->sendOpcode(OC_POWER);
        break;
      case MODE_PASSIVE:
        return serial->sendOpcode(OC_START);
        break;
      case MODE_SAFE:
        return serial->sendOpcode(OC_SAFE);
        break;
      case MODE_FULL:
        return serial->sendOpcode(OC_FULL);
        break;
      default:
        CERR("[create::Create] ", "cannot set robot to mode '" << mode << "'");
        break;
    }
    return false;
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

  bool Create::driveRadius(const float& vel, const float& radius) const {
    // Expects each parameter as two bytes each and in millimeters
    int16_t vel_mm = roundf(vel * 1000);
    int16_t radius_mm = roundf(radius * 1000);
    BOUND(vel_mm, -util::CREATE_2_MAX_VEL * 1000, util::CREATE_2_MAX_VEL * 1000);

    // Bound radius if not a special case
    if (radius_mm != 32768 && radius_mm != 32767 &&
        radius_mm != -1 && radius_mm != 1) {
      BOUND(radius_mm, -util::CREATE_2_MAX_RADIUS, util::CREATE_2_MAX_RADIUS);
    }

    uint8_t cmd[5] = { OC_DRIVE,
                       vel_mm >> 8,
                       vel_mm & 0xff,
                       radius_mm >> 8,
                       radius_mm & 0xff
                     };

    return serial->send(cmd, 5);
  }

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
    if (data->isValidPacketID(ID_BUMP_WHEELDROP)) {
      return (GET_DATA(ID_BUMP_WHEELDROP) & 0x0C) != 0;
    }
    else {
      CERR("[create::Create] ", "Wheeldrop sensor not supported!");
      return false;
    }
  }

  bool Create::isLeftBumper() const {
    if (data->isValidPacketID(ID_BUMP_WHEELDROP)) {
      return (GET_DATA(ID_BUMP_WHEELDROP) & 0x02) != 0;
    }
    else {
      CERR("[create::Create] ", "Left bumper not supported!");
      return false;
    }
  }

  bool Create::isRightBumper() const {
    if (data->isValidPacketID(ID_BUMP_WHEELDROP)) {
      return (GET_DATA(ID_BUMP_WHEELDROP) & 0x01) != 0;
    }
    else {
      CERR("[create::Create] ", "Right bumper not supported!");
      return false;
    }
  }

  bool Create::isWall() const {
    if (data->isValidPacketID(ID_WALL)) {
      return GET_DATA(ID_WALL) == 1;
    }
    else {
      CERR("[create::Create] ", "Wall sensor not supported!");
      return false;
    }
  }

  bool Create::isCliff() const {
    if (data->isValidPacketID(ID_CLIFF_LEFT) &&
        data->isValidPacketID(ID_CLIFF_FRONT_LEFT) &&
        data->isValidPacketID(ID_CLIFF_FRONT_RIGHT) &&
        data->isValidPacketID(ID_CLIFF_RIGHT)) {
      return GET_DATA(ID_CLIFF_LEFT) == 1 ||
             GET_DATA(ID_CLIFF_FRONT_LEFT) == 1 ||
             GET_DATA(ID_CLIFF_FRONT_RIGHT) == 1 ||
             GET_DATA(ID_CLIFF_RIGHT) == 1;
    }
    else {
      CERR("[create::Create] ", "Cliff sensors not supported!");
      return false;
    }
  }

  bool Create::isVirtualWall() const {
    if (data->isValidPacketID(ID_VIRTUAL_WALL)) {
      return GET_DATA(ID_VIRTUAL_WALL);
    }
    else {
      CERR("[create::Create] ", "Virtual Wall sensor not supported!");
      return false;
    }
  }

  uint8_t Create::getDirtDetect() const {
    if (data->isValidPacketID(ID_DIRT_DETECT)) {
      return GET_DATA(ID_DIRT_DETECT);
    }
    else {
      CERR("[create::Create] ", "Dirt detector not supported!");
      return -1;
    }
  }

  uint8_t Create::getIROmni() const {
    if (data->isValidPacketID(ID_IR_OMNI)) {
      return GET_DATA(ID_IR_OMNI);
    }
    else {
      CERR("[create::Create] ", "Omni IR sensor not supported!");
      return -1;
    }
  }

  uint8_t Create::getIRLeft() const {
    if (data->isValidPacketID(ID_IR_LEFT)) {
      return GET_DATA(ID_IR_LEFT);
    }
    else {
      CERR("[create::Create] ", "Left IR sensor not supported!");
      return -1;
    }
  }

  uint8_t Create::getIRRight() const {
    if (data->isValidPacketID(ID_IR_RIGHT)) {
      return GET_DATA(ID_IR_RIGHT);
    }
    else {
      CERR("[create::Create] ", "Right IR sensor not supported!");
      return -1;
    }
  }

  ChargingState Create::getChargingState() const {
    if (data->isValidPacketID(ID_CHARGE_STATE)) {
      uint8_t chargeState = GET_DATA(ID_CHARGE_STATE);
      assert(chargeState >= 0);
      assert(chargeState <= 5);
      return (ChargingState) chargeState;
    }
    else {
      CERR("[create::Create] ", "Charging state not supported!");
      return CHARGE_FAULT;
    }
  }

  bool Create::isCleanButtonPressed() const {
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x01) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  // Not supported by any 600 series firmware
  bool Create::isClockButtonPressed() const {
    CERR("[create::Create] ", "Clock button is not supported!");
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x80) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  // Not supported by any 600 series firmware
  bool Create::isScheduleButtonPressed() const {
    CERR("[create::Create] ", "Schedule button is not supported!");
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x40) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  bool Create::isDayButtonPressed() const {
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x20) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  bool Create::isHourButtonPressed() const {
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x10) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  bool Create::isMinButtonPressed() const {
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x08) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  bool Create::isDockButtonPressed() const {
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x04) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  bool Create::isSpotButtonPressed() const {
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x02) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  float Create::getVoltage() const {
    if (data->isValidPacketID(ID_VOLTAGE)) {
      return (GET_DATA(ID_VOLTAGE) / 1000.0);
    }
    else {
      CERR("[create::Create] ", "Voltage sensor not supported!");
      return 0;
    }
  }

  float Create::getCurrent() const {
    if (data->isValidPacketID(ID_VOLTAGE)) {
      return (((int16_t)GET_DATA(ID_CURRENT)) / 1000.0);
    }
    else {
      CERR("[create::Create] ", "Current sensor not supported!");
      return 0;
    }
  }

  int8_t Create::getTemperature() const {
    if (data->isValidPacketID(ID_TEMP)) {
      return (int8_t) GET_DATA(ID_TEMP);
    }
    else {
      CERR("[create::Create] ", "Temperature sensor not supported!");
      return 0;
    }
  }

  float Create::getBatteryCharge() const {
    if (data->isValidPacketID(ID_CHARGE)) {
      return (GET_DATA(ID_CHARGE) / 1000.0);
    }
    else {
      CERR("[create::Create] ", "Battery charge not supported!");
      return 0;
    }
  }

  float Create::getBatteryCapacity() const {
    if (data->isValidPacketID(ID_CAPACITY)) {
      return (GET_DATA(ID_CAPACITY) / 1000.0);
    }
    else {
      CERR("[create::Create] ", "Battery capacity not supported!");
      return 0;
    }
  }

  bool Create::isLightBumperLeft() const {
    if (data->isValidPacketID(ID_LIGHT)) {
      return (GET_DATA(ID_LIGHT) & 0x01) != 0;
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return false;
    }
  }

  bool Create::isLightBumperFrontLeft() const {
    if (data->isValidPacketID(ID_LIGHT)) {
      return (GET_DATA(ID_LIGHT) & 0x02) != 0;
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return false;
    }
  }

  bool Create::isLightBumperCenterLeft() const {
    if (data->isValidPacketID(ID_LIGHT)) {
      return (GET_DATA(ID_LIGHT) & 0x04) != 0;
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return false;
    }
  }

  bool Create::isLightBumperCenterRight() const {
    if (data->isValidPacketID(ID_LIGHT)) {
      return (GET_DATA(ID_LIGHT) & 0x08) != 0;
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return false;
    }
  }

  bool Create::isLightBumperFrontRight() const {
    if (data->isValidPacketID(ID_LIGHT)) {
      return (GET_DATA(ID_LIGHT) & 0x10) != 0;
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return false;
    }
  }

  bool Create::isLightBumperRight() const {
    if (data->isValidPacketID(ID_LIGHT)) {
      return (GET_DATA(ID_LIGHT) & 0x20) != 0;
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return false;
    }
  }

  uint16_t Create::getLightSignalLeft() const {
    if (data->isValidPacketID(ID_LIGHT_LEFT)) {
      return GET_DATA(ID_LIGHT_LEFT);
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return 0;
    }
  }

  uint16_t Create::getLightSignalFrontLeft() const {
    if (data->isValidPacketID(ID_LIGHT_FRONT_LEFT)) {
      return GET_DATA(ID_LIGHT_FRONT_LEFT);
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return 0;
    }
  }

  uint16_t Create::getLightSignalCenterLeft() const {
    if (data->isValidPacketID(ID_LIGHT_CENTER_LEFT)) {
      return GET_DATA(ID_LIGHT_CENTER_LEFT);
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return 0;
    }
  }

  uint16_t Create::getLightSignalRight() const {
    if (data->isValidPacketID(ID_LIGHT_RIGHT)) {
      return GET_DATA(ID_LIGHT_RIGHT);
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return 0;
    }
  }

  uint16_t Create::getLightSignalFrontRight() const {
    if (data->isValidPacketID(ID_LIGHT_FRONT_RIGHT)) {
      return GET_DATA(ID_LIGHT_FRONT_RIGHT);
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return 0;
    }
  }

  uint16_t Create::getLightSignalCenterRight() const {
    if (data->isValidPacketID(ID_LIGHT_CENTER_RIGHT)) {
      return GET_DATA(ID_LIGHT_CENTER_RIGHT);
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return 0;
    }
  }

  bool Create::isMovingForward() const {
    if (data->isValidPacketID(ID_STASIS)) {
      return GET_DATA(ID_STASIS) == 1;
    }
    else {
      CERR("[create::Create] ", "Stasis sensor not supported!");
      return false;
    }
  }

  float Create::getLeftWheelDistance() const {
      return totalLeftDist;
  }

  float Create::getRightWheelDistance() const {
      return totalRightDist;
  }

  float Create::getRequestedLeftWheelVel() const {
      if (data->isValidPacketID(ID_LEFT_VEL)) {
          uint16_t uvel = GET_DATA(ID_LEFT_VEL);
          int16_t vel;
          std::memcpy(&vel, &uvel, sizeof(vel));
          return (vel / 1000.0);
      }
      else {
        CERR("[create::Create] ", "Left wheel velocity not supported!");
        return 0;
      }
  }

  float Create::getRequestedRightWheelVel() const {
      if (data->isValidPacketID(ID_RIGHT_VEL)) {
        uint16_t uvel = GET_DATA(ID_RIGHT_VEL);
        int16_t vel;
        std::memcpy(&vel, &uvel, sizeof(vel));
        return (vel / 1000.0);
      }
      else {
        CERR("[create::Create] ", "Right wheel velocity not supported!");
        return 0;
      }
  }

  create::CreateMode Create::getMode() const {
    if (data->isValidPacketID(ID_OI_MODE)) {
      return (create::CreateMode) GET_DATA(ID_OI_MODE);
    }
    else {
      CERR("[create::Create] ", "Querying Mode not supported!");
      return create::MODE_UNAVAILABLE;
    }
  }

  Pose Create::getPose() const {
    return pose;
  }

  Vel Create::getVel() const {
    return vel;
  }

  uint64_t Create::getNumCorruptPackets() const {
    return serial->getNumCorruptPackets();
  }

  uint64_t Create::getTotalPackets() const {
    return serial->getTotalPackets();
  }

} // end namespace
