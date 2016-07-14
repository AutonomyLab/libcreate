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

  // Note: Some functionality does not work as expected in Safe mode
  robot->setMode(create::MODE_FULL);

  std::cout << "battery level: " <<
    robot->getBatteryCharge() / (float) robot->getBatteryCapacity() * 100.0 << "%" << std::endl;

  bool drive = false;

  // Make a song
  //uint8_t songLength = 16;
  //uint8_t notes[16] = { 67, 67, 66, 66, 65, 65, 66, 66,
  //                      67, 67, 66, 66, 65, 65, 66, 66 };
  //float durations[songLength];
  //for (int i = 0; i < songLength; i++) {
  //  durations[i] = 0.25;
  //}
  //robot->defineSong(0, songLength, notes, durations);
  //usleep(1000000);
  //robot->playSong(0);

  // Quit when center "Clean" button pressed
  while (!robot->isCleanButtonPressed()) {
    // Check for button presses
    if (robot->isDayButtonPressed())
      std::cout << "day button press" << std::endl;
    if (robot->isMinButtonPressed())
      std::cout << "min button press" << std::endl;
    if (robot->isDockButtonPressed()) {
      std::cout << "dock button press" << std::endl;
      robot->enableCheckRobotLED(false);
    }
    if (robot->isSpotButtonPressed()) {
      std::cout << "spot button press" << std::endl;
      robot->enableCheckRobotLED(true);
    }
    if (robot->isHourButtonPressed()) {
      std::cout << "hour button press" << std::endl;
      drive = !drive;
    }

    // Check for wheeldrop or cliffs
    if (robot->isWheeldrop() || robot->isCliff()) {
      drive = false;
      robot->setPowerLED(255);
    }

    // If everything is ok, drive forward using IR's to avoid obstacles
    if (drive) {
      robot->setPowerLED(0); // green
      if (robot->isLightBumperLeft() ||
          robot->isLightBumperFrontLeft() ||
          robot->isLightBumperCenterLeft()) {
        // turn right
        robot->drive(0.1, -1.0);
        robot->setDigitsASCII('-','-','-',']');
      }
      else if (robot->isLightBumperRight() ||
               robot->isLightBumperFrontRight() ||
               robot->isLightBumperCenterRight()) {
        // turn left
        robot->drive(0.1, 1.0);
        robot->setDigitsASCII('[','-','-','-');
      }
      else {
        // drive straight
        robot->drive(0.1, 0.0);
        robot->setDigitsASCII(' ','^','^',' ');
      }
    }
    else { // drive == false
      // stop moving
      robot->drive(0, 0);
      robot->setDigitsASCII('S','T','O','P');
    }

    // Turn on blue 'debris' light if moving forward
    if (robot->isMovingForward()) {
      robot->enableDebrisLED(true);
    }
    else {
      robot->enableDebrisLED(false);
    }

    // Check bumpers
    if (robot->isLeftBumper()) {
      std::cout << "left bump detected!" << std::endl;
    }
    if (robot->isRightBumper()) {
      std::cout << "right bump detected!" << std::endl;
    }

    usleep(1000 * 100); //10hz
  }

  std::cout << "Stopping Create." << std::endl;

  // Turn off lights
  robot->setPowerLED(0, 0);
  robot->enableDebrisLED(false);
  robot->enableCheckRobotLED(false);
  robot->setDigitsASCII(' ', ' ', ' ', ' ');

  // Make sure to disconnect to clean up
  robot->disconnect();
  delete robot;

  return 0;
}
