#include "create/types.h"

namespace create {

  RobotModel::RobotModel(const ProtocolVersion version, const float axleLength, const unsigned int baud, const float maxVelocity, const float wheelDiameter):
    id(nextId),
    version(version),
    axleLength(axleLength),
    baud(baud),
    maxVelocity(maxVelocity),
    wheelDiameter(wheelDiameter) {
    nextId <<= 1;
  }

  bool RobotModel::operator ==(RobotModel& other) const {
    return id == other.id;
  }

  RobotModel::operator uint32_t() const {
    return id;
  }

  uint32_t RobotModel::getId() const {
    return id;
  }

  ProtocolVersion RobotModel::getVersion() const {
    return version;
  }

  float RobotModel::getAxleLength() const {
    return axleLength;
  }

  unsigned int RobotModel::getBaud() const {
    return baud;
  }

  float RobotModel::getMaxVelocity() const {
    return maxVelocity;
  }

  float RobotModel::getWheelDiameter() const {
    return wheelDiameter;
  }

  uint32_t RobotModel::nextId = 1;

  RobotModel RobotModel::ROOMBA_400(V_1, 0.258, 57600);
  RobotModel RobotModel::CREATE_1(V_2, 0.258, 57600);
  RobotModel RobotModel::CREATE_2(V_3, 0.235, 115200, 0.5, 0.072);
}
