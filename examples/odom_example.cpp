#include "create/create.h"

create::Create* robot;

int main(int argc, char** argv) {
  std::string port = "/dev/ttyUSB0";
  int baud = 115200;
  create::RobotModel model = create::CREATE_2;

  if (argc > 1 && std::string(argv[1]) == "create1") {
    model = create::CREATE_1;
    baud = 57600;
    std::cout << "1st generation Create selected" << std::endl;
  }

  robot = new create::Create(model);

  // Attempt to connect to Create
  if (robot->connect(port, baud))
    std::cout << "Successfully connected to Create" << std::endl;
  else {
    std::cout << "Failed to connect to Create on port " << port.c_str() << std::endl;
    return 1;
  }

  robot->setMode(create::MODE_FULL);

  usleep(1000000);

  // drive in a circle
  robot->drive(0.1, 0.5);

  // Quit when center "Clean" button pressed
  while (!robot->isCleanButtonPressed()) {
    create::Pose pose = robot->getPose();

    std::cout << "x: " << pose.x
              << "\ty: " << pose.y
              << "\tyaw: " << pose.yaw * 180.0/create::util::PI << std::endl;
  
    usleep(1000 * 100); //10hz
  }

  std::cout << "Stopping Create." << std::endl;

  robot->disconnect();
  delete robot;

  return 0;
}
