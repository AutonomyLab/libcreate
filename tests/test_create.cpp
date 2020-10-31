/**
Software License Agreement (BSD)

\file      test_create.cpp
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
#include "create/types.h"

#include "gtest/gtest.h"

TEST(CreateTest, ConstructorSingleParam)
{
  create::Create create_default;

  create::Create create_1(create::RobotModel::CREATE_1);

  create::Create create_2(create::RobotModel::CREATE_2);

  create::Create create_roomba_400(create::RobotModel::ROOMBA_400);
}

// TEST(CreateTest, ConstructorMultiParam)
// {
//   TODO(jacobperron): Document exception thrown and consider defining custom exception
//   create::Create create(std::string("/dev/ttyUSB0"), 11520);
// }

TEST(CreateTest, Connected)
{
  create::Create create;
  // Nothing to be connected to
  EXPECT_FALSE(create.connected());
}

TEST(CreateTest, Disconnect)
{
  create::Create create;
  // Even though not connected, this should not crash
  create.disconnect();
}
