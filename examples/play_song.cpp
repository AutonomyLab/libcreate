/**
Software License Agreement (BSD)

\file      play_song.cpp
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

  // Useful note defintions
  const uint8_t G = 55;
  const uint8_t AS = 58;
  const uint8_t DS = 51;
  const float half = 1.0f;
  const float quarter = 0.5f;
  const float dotted_eigth = 0.375f; 
  const float sixteenth = 0.125f;

  // Define a song
  const uint8_t song_len = 9;
  const uint8_t notes[song_len] = { G, G, G, DS, AS, G, DS, AS, G };
  const float durations[song_len] = { quarter, quarter, quarter, dotted_eigth, sixteenth, quarter, dotted_eigth, sixteenth, half };
  robot->defineSong(0, song_len, notes, durations);

  // Sleep to provide time for song to register
  usleep(1000000);

  std::cout << "Playing a song!" << std::endl;

  // Request to play the song we just defined
  robot->playSong(0); 

  // Expect the song to take about four seconds
  usleep(4500000);

  // Call disconnect to avoid leaving robot in Full mode
  robot->disconnect();

  // Clean up
  delete robot;

  std::cout << "Bye!" << std::endl;
  return 0;
}
