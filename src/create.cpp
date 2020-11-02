#include <iostream>
#include <cmath>
#include <ctime>
#include <memory>
#include <assert.h>

#include "create/create.h"

#define GET_DATA(id) (data->getPacket(id)->getData())
#define BOUND_CONST(val,min,max) (val<min?min:(val>max?max:val))
#define BOUND(val,min,max) (val = BOUND_CONST(val,min,max))

namespace create {

  namespace ublas = boost::numeric::ublas;

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
    mode = MODE_OFF;
    pose.x = 0;
    pose.y = 0;
    pose.yaw = 0;
    pose.covariance = std::vector<float>(9, 0.0);
    vel.x = 0;
    vel.y = 0;
    vel.yaw = 0;
    vel.covariance = std::vector<float>(9, 0.0);
    poseCovar = Matrix(3, 3, 0.0);
    requestedLeftVel = 0;
    requestedRightVel = 0;
    data = std::shared_ptr<Data>(new Data(model.getVersion()));
    if (model.getVersion() == V_1) {
      serial = std::make_shared<SerialQuery>(data);
    } else {
      serial = std::make_shared<SerialStream>(data);
    }
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
    size_t rows = A.size1();
    size_t cols = A.size2();

    assert(rows == B.size1());
    assert(cols == B.size2());

    Matrix C(rows, cols);
    for (size_t i = 0u; i < rows; i++) {
      for (size_t j = 0u; j < cols; j++) {
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
      if (model.getVersion() >= V_3) {
        // Initialize tick counts
        prevTicksLeft = GET_DATA(ID_LEFT_ENC);
        prevTicksRight = GET_DATA(ID_RIGHT_ENC);
      }
      prevOnDataTime = std::chrono::system_clock::now();
      firstOnData = false;
    }

    // Get current time
    auto curTime = std::chrono::system_clock::now();
    float dt = static_cast<std::chrono::duration<float>>(curTime - prevOnDataTime).count();
    float deltaDist, deltaX, deltaY, deltaYaw, leftWheelDist, rightWheelDist, wheelDistDiff;

    // Protocol versions 1 and 2 use distance and angle fields for odometry
    int16_t angleField;
    if (model.getVersion() <= V_2) {
      // This is a standards compliant way of doing unsigned to signed conversion
      uint16_t distanceRaw = GET_DATA(ID_DISTANCE);
      int16_t distance;
      std::memcpy(&distance, &distanceRaw, sizeof(distance));
      deltaDist = distance / 1000.0; // mm -> m

      // Angle is processed differently in versions 1 and 2
      uint16_t angleRaw = GET_DATA(ID_ANGLE);
      std::memcpy(&angleField, &angleRaw, sizeof(angleField));
    }

    if (model.getVersion() == V_1) {
      wheelDistDiff = 2.0 * angleField / 1000.0;
      leftWheelDist = deltaDist - (wheelDistDiff / 2.0);
      rightWheelDist = deltaDist + (wheelDistDiff / 2.0);
      deltaYaw = wheelDistDiff / model.getAxleLength();
    } else if (model.getVersion() == V_2) {
      /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
       * Certain older Creates have major problems with odometry                   *
       * http://answers.ros.org/question/31935/createroomba-odometry/              *
       *                                                                           *
       * All Creates have an issue with rounding of the angle field, which causes  *
       * major errors to accumulate in the odometry yaw.                           *
       * http://wiki.tekkotsu.org/index.php/Create_Odometry_Bug                    *
       * https://github.com/AutonomyLab/create_autonomy/issues/28                  *
       *                                                                           *
       * TODO: Consider using velocity command as substitute for pose estimation   *
       * to mitigate both of these problems.                                       *
       * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
      deltaYaw = angleField * (util::PI / 180.0); // D2R
      wheelDistDiff = model.getAxleLength() * deltaYaw;
      leftWheelDist = deltaDist - (wheelDistDiff / 2.0);
      rightWheelDist = deltaDist + (wheelDistDiff / 2.0);
    } else if (model.getVersion() >= V_3) {
      // Get cumulative ticks (wraps around at 65535)
      uint16_t totalTicksLeft = GET_DATA(ID_LEFT_ENC);
      uint16_t totalTicksRight = GET_DATA(ID_RIGHT_ENC);
      // Compute ticks since last update
      int ticksLeft = totalTicksLeft - prevTicksLeft;
      int ticksRight = totalTicksRight - prevTicksRight;
      prevTicksLeft = totalTicksLeft;
      prevTicksRight = totalTicksRight;

      // Handle wrap around
      if (fabs(ticksLeft) > 0.9 * util::V_3_MAX_ENCODER_TICKS) {
        ticksLeft = (ticksLeft % util::V_3_MAX_ENCODER_TICKS) + 1;
      }
      if (fabs(ticksRight) > 0.9 * util::V_3_MAX_ENCODER_TICKS) {
        ticksRight = (ticksRight % util::V_3_MAX_ENCODER_TICKS) + 1;
      }

      // Compute distance travelled by each wheel
      leftWheelDist = (ticksLeft / util::V_3_TICKS_PER_REV)
          * model.getWheelDiameter() * util::PI;
      rightWheelDist = (ticksRight / util::V_3_TICKS_PER_REV)
          * model.getWheelDiameter() * util::PI;
      deltaDist = (rightWheelDist + leftWheelDist) / 2.0;

      wheelDistDiff = rightWheelDist - leftWheelDist;
      deltaYaw = wheelDistDiff / model.getAxleLength();
    }

    measuredLeftVel = leftWheelDist / dt;
    measuredRightVel = rightWheelDist / dt;

    // Moving straight
    if (fabs(wheelDistDiff) < util::EPS) {
      deltaX = deltaDist * cos(pose.yaw);
      deltaY = deltaDist * sin(pose.yaw);
    } else {
      float turnRadius = (model.getAxleLength() / 2.0) * (leftWheelDist + rightWheelDist) / wheelDistDiff;
      deltaX = turnRadius * (sin(pose.yaw + deltaYaw) - sin(pose.yaw));
      deltaY = -turnRadius * (cos(pose.yaw + deltaYaw) - cos(pose.yaw));
    }

    totalLeftDist += leftWheelDist;
    totalRightDist += rightWheelDist;

    if (fabs(dt) > util::EPS) {
      vel.x = deltaDist / dt;
      vel.y = 0.0;
      vel.yaw = deltaYaw / dt;
    } else {
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
    float distOverTwoWB = deltaDist / (model.getAxleLength() * 2.0);

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
    Finc(2, 0) = (1.0 / model.getAxleLength());
    Finc(2, 1) = (-1.0 / model.getAxleLength());
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
    while (!serial->connect(port, baud, std::bind(&Create::onData, this)) && !timeout) {
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
    if (model.getVersion() == V_1){
      // Switch to safe mode (required for compatibility with V_1)
      if (!(serial->sendOpcode(OC_START) && serial->sendOpcode(OC_CONTROL))) return false;
    }
    bool ret;
    switch (mode) {
      case MODE_OFF:
        if (model.getVersion() == V_2) {
          CERR("[create::Create] ", "protocol version 2 does not support turning robot off");
          ret = false;
        } else {
          ret = serial->sendOpcode(OC_POWER);
        }
        break;
      case MODE_PASSIVE:
        ret = serial->sendOpcode(OC_START);
        break;
      case MODE_SAFE:
        if (model.getVersion() > V_1) {
          ret = serial->sendOpcode(OC_SAFE);
        }
        break;
      case MODE_FULL:
        ret = serial->sendOpcode(OC_FULL);
        break;
      default:
        CERR("[create::Create] ", "cannot set robot to mode '" << mode << "'");
        ret = false;
    }
    if (ret) {
      this->mode = mode;
    }
    return ret;
  }

  bool Create::clean(const CleanMode& mode) {
    return serial->sendOpcode((Opcode) mode);
  }

  bool Create::dock() const {
    return serial->sendOpcode(OC_DOCK);
  }

  bool Create::setDate(const DayOfWeek& day, const uint8_t& hour, const uint8_t& min) const {
    if (day < 0 || day > 6 ||
        hour > 23 ||
        min > 59)
      return false;

    uint8_t cmd[4] = { OC_DATE, day, hour, min };
    return serial->send(cmd, 4);
  }

  bool Create::driveRadius(const float& vel, const float& radius) {
    // Bound velocity
    float boundedVel = BOUND_CONST(vel, -model.getMaxVelocity(), model.getMaxVelocity());

    // Expects each parameter as two bytes each and in millimeters
    int16_t vel_mm = roundf(boundedVel * 1000);
    int16_t radius_mm = roundf(radius * 1000);

    // Bound radius if not a special case
    if (radius_mm != -32768 && radius_mm != 32767 &&
        radius_mm != -1 && radius_mm != 1) {
      BOUND(radius_mm, -util::MAX_RADIUS * 1000, util::MAX_RADIUS * 1000);
    }

    uint8_t cmd[5] = { OC_DRIVE,
                       static_cast<uint8_t>(vel_mm >> 8),
                       static_cast<uint8_t>(vel_mm & 0xff),
                       static_cast<uint8_t>(radius_mm >> 8),
                       static_cast<uint8_t>(radius_mm & 0xff)
                     };

    return serial->send(cmd, 5);
  }

  bool Create::driveWheels(const float& leftVel, const float& rightVel) {
    const float boundedLeftVel = BOUND_CONST(leftVel, -model.getMaxVelocity(), model.getMaxVelocity());
    const float boundedRightVel = BOUND_CONST(rightVel, -model.getMaxVelocity(), model.getMaxVelocity());
    requestedLeftVel = boundedLeftVel;
    requestedRightVel = boundedRightVel;
    if (model.getVersion() > V_1) {
      int16_t leftCmd = roundf(boundedLeftVel * 1000);
      int16_t rightCmd = roundf(boundedRightVel * 1000);

      uint8_t cmd[5] = { OC_DRIVE_DIRECT,
                         static_cast<uint8_t>(rightCmd >> 8),
                         static_cast<uint8_t>(rightCmd & 0xff),
                         static_cast<uint8_t>(leftCmd >> 8),
                         static_cast<uint8_t>(leftCmd & 0xff)
                       };
      return serial->send(cmd, 5);
    } else {
      float radius;
      // Prevent divide by zero when driving straight
      if (boundedLeftVel != boundedRightVel) {
        radius = -((model.getAxleLength() / 2.0) * (boundedLeftVel + boundedRightVel)) /
          (boundedLeftVel - boundedRightVel);
      } else {
        radius = util::STRAIGHT_RADIUS;
      }

      float vel;
      // Fix signs for spin in place
      if (boundedLeftVel == -boundedRightVel || std::abs(roundf(radius * 1000)) <= 1) {
        radius = util::IN_PLACE_RADIUS;
        vel = boundedRightVel;
      } else {
        vel = (std::abs(boundedLeftVel) + std::abs(boundedRightVel)) / 2.0 * ((boundedLeftVel + boundedRightVel) > 0 ? 1.0 : -1.0);
      }

      // Radius turns have a maximum radius of 2.0 meters
      // When the radius is > 2 but <= 10, use a 2 meter radius
      // When it is > 10, drive straight
      // TODO: alternate between these 2 meter radius and straight to
      // fake a larger radius
      if (radius > 10.0) {
        radius = util::STRAIGHT_RADIUS;
      }

      return driveRadius(vel, radius);
    }
  }

  bool Create::driveWheelsPwm(const float& leftWheel, const float& rightWheel)
  {
    static const int16_t PWM_COUNTS = 255;

    if (leftWheel < -1.0 || leftWheel > 1.0 ||
        rightWheel < -1.0 || rightWheel > 1.0)
      return false;

    int16_t leftPwm = roundf(leftWheel * PWM_COUNTS);
    int16_t rightPwm = roundf(rightWheel * PWM_COUNTS);

    uint8_t cmd[5] = { OC_DRIVE_PWM,
                       static_cast<uint8_t>(rightPwm >> 8),
                       static_cast<uint8_t>(rightPwm & 0xff),
                       static_cast<uint8_t>(leftPwm >> 8),
                       static_cast<uint8_t>(leftPwm & 0xff)
                     };

    return serial->send(cmd, 5);
  }

  bool Create::drive(const float& xVel, const float& angularVel) {
    // Compute left and right wheel velocities
    float leftVel = xVel - ((model.getAxleLength() / 2.0) * angularVel);
    float rightVel = xVel + ((model.getAxleLength() / 2.0) * angularVel);
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

    if (model.getVersion() == V_1) {
        uint8_t cmd[2] = { OC_MOTORS,
                           static_cast<uint8_t>((side != 0.0 ? 1 : 0) |
                                     (vacuum != 0.0 ? 2 : 0) |
                                     (main != 0.0 ? 4 : 0))
                         };
        return serial->send(cmd, 2);
    }

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
    std::vector<uint8_t> cmd(2 * songLength + 3);
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

    return serial->send(cmd.data(), cmd.size());
  }

  bool Create::playSong(const uint8_t& songNumber) const {
    if (songNumber > 4)
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

  bool Create::isLeftWheeldrop() const {
    if (data->isValidPacketID(ID_BUMP_WHEELDROP)) {
      return (GET_DATA(ID_BUMP_WHEELDROP) & 0x08) != 0;
    }
    else {
      CERR("[create::Create] ", "Wheeldrop sensor not supported!");
      return false;
    }
  }

  bool Create::isRightWheeldrop() const {
    if (data->isValidPacketID(ID_BUMP_WHEELDROP)) {
      return (GET_DATA(ID_BUMP_WHEELDROP) & 0x04) != 0;
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

  bool Create::isCliffLeft() const {
    if (data->isValidPacketID(ID_CLIFF_LEFT)) {
      return GET_DATA(ID_CLIFF_LEFT) == 1;
    }
    else {
      CERR("[create::Create] ", "Left cliff sensors not supported!");
      return false;
    }
  }

  bool Create::isCliffFrontLeft() const {
    if (data->isValidPacketID(ID_CLIFF_FRONT_LEFT)) {
      return GET_DATA(ID_CLIFF_FRONT_LEFT) == 1;
    }
    else {
      CERR("[create::Create] ", "Front left cliff sensors not supported!");
      return false;
    }
  }

  bool Create::isCliffRight() const {
    if (data->isValidPacketID(ID_CLIFF_RIGHT)) {
      return GET_DATA(ID_CLIFF_RIGHT) == 1;
    }
    else {
      CERR("[create::Create] ", "Rightt cliff sensors not supported!");
      return false;
    }
  }

  bool Create::isCliffFrontRight() const {
    if (data->isValidPacketID(ID_CLIFF_FRONT_RIGHT)) {
      return GET_DATA(ID_CLIFF_FRONT_RIGHT) == 1;
    }
    else {
      CERR("[create::Create] ", "Front right cliff sensors not supported!");
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
    if (data->isValidPacketID(ID_DIRT_DETECT_LEFT)) {
      return GET_DATA(ID_DIRT_DETECT_LEFT);
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

  bool Create::isSideBrushOvercurrent() const { 
    if (data->isValidPacketID(ID_OVERCURRENTS)) {
      return (GET_DATA(ID_OVERCURRENTS) & 0x01) != 0;
    }
    else {
      CERR("[create::Create] ", "Overcurrent sensor not supported!");
      return false;
    }
  }

  bool Create::isMainBrushOvercurrent() const { 
    if (data->isValidPacketID(ID_OVERCURRENTS)) {
      return (GET_DATA(ID_OVERCURRENTS) & 0x04) != 0;
    }
    else {
      CERR("[create::Create] ", "Overcurrent sensor not supported!");
      return false;
    }
  }
  
  bool Create::isWheelOvercurrent() const { 
    if (data->isValidPacketID(ID_OVERCURRENTS)) {
      return (GET_DATA(ID_OVERCURRENTS) & 0x18) != 0;
    }
    else {
      CERR("[create::Create] ", "Overcurrent sensor not supported!");
      return false;
    }
  }

  float Create::getLeftWheelDistance() const {
    return totalLeftDist;
  }

  float Create::getRightWheelDistance() const {
    return totalRightDist;
  }

  float Create::getMeasuredLeftWheelVel() const {
    return measuredLeftVel;
  }

  float Create::getMeasuredRightWheelVel() const {
    return measuredRightVel;
  }

  float Create::getRequestedLeftWheelVel() const {
    return requestedLeftVel;
  }

  float Create::getRequestedRightWheelVel() const {
    return requestedRightVel;
  }

  create::CreateMode Create::getMode() {
    if (data->isValidPacketID(ID_OI_MODE)) {
      mode = (create::CreateMode) GET_DATA(ID_OI_MODE);
    }
    return mode;
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
