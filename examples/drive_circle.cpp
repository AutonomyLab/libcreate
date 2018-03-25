/**
Software License Agreement (BSD)

\file      drive_circle.cpp
\authors   Jacob Perron <jperron@sfu.ca>
\copyright Copyright (c) 2018, Autonomy Lab (Simon Fraser University), All rights reserved.

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
#include "create/create.h"

#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
  // Select robot. Assume Create 2 unless argument says otherwise
  create::RobotModel model = create::RobotModel::CREATE_2;
  std::string port = "/dev/ttyUSB0";
  int baud = 115200;
  if (argc > 1 && std::string(argv[1]) == "create1") {
    model = create::RobotModel::CREATE_1;
    baud = 57600;
    std::cout << "Running driver for Create 1" << std::endl;
  }
  else {
    std::cout << "Running driver for Create 2" << std::endl;
  }

  // Construct robot object
  create::Create* robot = new create::Create(model);

  // Connect to robot
  if (robot->connect(port, baud))
    std::cout << "Connected to robot" << std::endl;
  else {
    std::cout << "Failed to connect to robot on port " << port.c_str() << std::endl;
    return 1;
  }

  // Switch to Full mode
  robot->setMode(create::MODE_FULL);

  std::cout << std::endl << "Press center 'Clean' button to disconnect and end program" << std::endl;

  // There's a delay between switching modes and when the robot will accept drive commands
  usleep(100000);

  // Command robot to drive a radius of 0.15 metres at 0.2 m/s
  robot->driveRadius(0.2, 0.15);

  while (!robot->isCleanButtonPressed()) {
    // Get robot odometry and print
    const create::Pose pose = robot->getPose(); 

    std::cout << std::fixed << std::setprecision(2) << "\rOdometry (x, y, yaw): ("
              << pose.x << ", " << pose.y << ", " << pose.yaw << ")      ";

    usleep(10000);  // 10 Hz
  }

  std::cout << std::endl;

  // Call disconnect to avoid leaving robot in Full mode
  // Also, this consequently stops the robot from moving
  robot->disconnect();

  // Clean up
  delete robot;

  std::cout << "Bye!" << std::endl;
  return 0;
}
