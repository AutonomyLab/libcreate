/**
Software License Agreement (BSD)

\file      test_packet.cpp
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
#include "create/packet.h"
#include "create/types.h"

#include "gtest/gtest.h"

TEST(PacketTest, Constructor)
{
  create::Packet empty_packet(0, std::string(""));
  EXPECT_EQ(static_cast<int>(empty_packet.nbytes), 0);
  EXPECT_EQ(empty_packet.info, std::string(""));

  create::Packet some_packet(2, std::string("test_packet"));
  EXPECT_EQ(static_cast<int>(some_packet.nbytes), 2);
  EXPECT_EQ(some_packet.info, std::string("test_packet"));
}

TEST(PacketTest, SetValidateAndGetData)
{
  create::Packet packet(2, std::string("test_packet"));

  // Set some data and validate it
  const uint16_t some_data = 123;
  packet.setDataToValidate(some_data);
  packet.validate();
  // Confirm data was validated
  const uint16_t some_data_result = packet.getData();
  EXPECT_EQ(some_data_result, some_data);

  // Set zero data and validate it
  const uint16_t zero_data = 0;
  packet.setDataToValidate(zero_data);
  packet.validate();
  // Confirm data was validated
  const uint16_t zero_data_result = packet.getData();
  EXPECT_EQ(zero_data_result, zero_data);

  // Set some data but do not validate it
  const uint16_t do_not_validate_data = 321;
  packet.setDataToValidate(do_not_validate_data);
  // Confirm data was not validated
  const uint16_t unvalidated_data_result = packet.getData();
  EXPECT_NE(unvalidated_data_result, do_not_validate_data);
}
