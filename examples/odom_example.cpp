#include "create/create.h"

create::Create* robot;

int main(int argc, char** argv) {
  std::string port = "/dev/ttyUSB0";
  int baud = 115200;
  create::RobotModel model = create::RobotModel::CREATE_2;

  if (argc > 1 && std::string(argv[1]) == "create1") {
    model = create::RobotModel::CREATE_1;
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

  // Drive in a circle
  robot->drive(0.1, 0.5);

  // Quit when center "Clean" button pressed
  while (!robot->isCleanButtonPressed()) {
    create::Pose pose = robot->getPose();
    create::Vel vel = robot->getVel();

    // Print pose
    std::cout << "x: " << pose.x
              << "\ty: " << pose.y
              << "\tyaw: " << pose.yaw * 180.0/create::util::PI << std::endl << std::endl;
  
    // Print velocity
    std::cout << "vx: " << vel.x
              << "\tvy: " << vel.y
              << "\tvyaw: " << vel.yaw * 180.0/create::util::PI << std::endl << std::endl;

    // Print covariances
    std::cout << "[ " << pose.covariance[0] << ", " << pose.covariance[1] << ", " << pose.covariance[2] << std::endl
              << "  " << pose.covariance[3] << ", " << pose.covariance[4] << ", " << pose.covariance[5] << std::endl
              << "  " << pose.covariance[6] << ", " << pose.covariance[7] << ", " << pose.covariance[8] << " ]" << std::endl << std::endl;;
    std::cout << "[ " << vel.covariance[0] << ", " << vel.covariance[1] << ", " << vel.covariance[2] << std::endl
              << "  " << vel.covariance[3] << ", " << vel.covariance[4] << ", " << vel.covariance[5] << std::endl
              << "  " << vel.covariance[6] << ", " << vel.covariance[7] << ", " << vel.covariance[8] << " ]" << std::endl << std::endl;;
    usleep(1000 * 100); //10hz
  }

  std::cout << "Stopping Create." << std::endl;

  robot->disconnect();
  delete robot;

  return 0;
}
