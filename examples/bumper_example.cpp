#include "create/create.h"

int main(int argc, char** argv) {
  std::string port = "/dev/ttyUSB0";
  int baud = 115200;
  create::RobotModel model = create::RobotModel::CREATE_2;

  create::Create* robot = new create::Create(model);

  // Attempt to connect to Create
  if (robot->connect(port, baud))
    std::cout << "Successfully connected to Create" << std::endl;
  else {
    std::cout << "Failed to connect to Create on port " << port.c_str() << std::endl;
    return 1;
  }

  robot->setMode(create::MODE_FULL);

  uint16_t signals[6];
  bool contact_bumpers[2];
  bool light_bumpers[6];

  // Stop program when clean button is pressed
  while (!robot->isCleanButtonPressed()) {
    signals[0] = robot->getLightSignalLeft();
    signals[1] = robot->getLightSignalFrontLeft();
    signals[2] = robot->getLightSignalCenterLeft();
    signals[3] = robot->getLightSignalCenterRight();
    signals[4] = robot->getLightSignalFrontRight();
    signals[5] = robot->getLightSignalRight();

    contact_bumpers[0] = robot->isLeftBumper();
    contact_bumpers[1] = robot->isRightBumper();

    light_bumpers[0] = robot->isLightBumperLeft();
    light_bumpers[1] = robot->isLightBumperFrontLeft();
    light_bumpers[2] = robot->isLightBumperCenterLeft();
    light_bumpers[3] = robot->isLightBumperCenterRight();
    light_bumpers[4] = robot->isLightBumperFrontRight();
    light_bumpers[5] = robot->isLightBumperRight();

    // print signals from left to right
    std::cout << "[ " << signals[0] << " , "
                      << signals[1] << " , "
                      << signals[2] << " , "
                      << signals[3] << " , "
                      << signals[4] << " , "
                      << signals[5]
              << " ]" << std::endl;
    std::cout << "[ " << light_bumpers[0] << " , "
                      << light_bumpers[1] << " , "
                      << light_bumpers[2] << " , "
                      << light_bumpers[3] << " , "
                      << light_bumpers[4] << " , "
                      << light_bumpers[5]
              << " ]" << std::endl;
    std::cout << "[ " << contact_bumpers[0] << " , "
                      << contact_bumpers[1]
              << " ]" << std::endl;
    std::cout << std::endl;

    usleep(1000 * 100); //10hz
  }

  std::cout << "Stopping Create" << std::endl;

  // Disconnect from robot
  robot->disconnect();
  delete robot;

  return 0;
}
