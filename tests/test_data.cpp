/**
Software License Agreement (BSD)

\file      test_data.cpp
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
#include "create/data.h"
#include "create/packet.h"
#include "create/types.h"

#include "gtest/gtest.h"

#include <boost/shared_ptr.hpp>

TEST(DataTest, Constructor)
{
  create::Data data_default;

  create::Data data_v_1(create::V_1);

  create::Data data_v_2(create::V_2);

  create::Data data_v_3(create::V_3);

  create::Data data_v_all(create::V_ALL);
}

// Number of packets for a given protocol are determined in the Data() constructor
TEST(DataTest, GetNumPackets)
{
  // Number of packets shared by all protocols is 16
  create::Data data_v_1(create::V_1);
  // Number exclusive to V_1 = 4
  // 16 + 4 = 20
  EXPECT_EQ(static_cast<int>(data_v_1.getNumPackets()), 20);

  create::Data data_v_2(create::V_2);
  // Number exclusive to V_2 = 3
  // 16 + 3 = 19
  EXPECT_EQ(static_cast<int>(data_v_2.getNumPackets()), 19);

  create::Data data_v_3(create::V_3);
  // Number exclusive to V_3 = 13
  // 16 + 13 = 29
  EXPECT_EQ(static_cast<int>(data_v_3.getNumPackets()), 29);

  create::Data data_v_all(create::V_ALL);
  EXPECT_EQ(static_cast<int>(data_v_all.getNumPackets()), 33);
}

TEST(DataTest, GetPacket)
{
  // Get a packet exclusive to V_1
  create::Data data_v_1(create::V_1);
  boost::shared_ptr<create::Packet> v_1_packet_ptr = data_v_1.getPacket(create::ID_OVERCURRENTS);
  EXPECT_NE(v_1_packet_ptr, boost::shared_ptr<create::Packet>())
      << "ID_OVERCURRENTS packet not found for protocol V_1";
  EXPECT_EQ(static_cast<int>(v_1_packet_ptr->nbytes), 1);
  EXPECT_EQ(v_1_packet_ptr->info, std::string("overcurrents"));

  // Get a packet for V_2
  create::Data data_v_2(create::V_2);
  boost::shared_ptr<create::Packet> v_2_packet_ptr = data_v_2.getPacket(create::ID_DISTANCE);
  EXPECT_NE(v_2_packet_ptr, boost::shared_ptr<create::Packet>())
      << "ID_DISTANCE packet not found for protocol V_2";
  EXPECT_EQ(static_cast<int>(v_2_packet_ptr->nbytes), 2);
  EXPECT_EQ(v_2_packet_ptr->info, std::string("distance"));

  // Get a packet exclusive to V_3
  create::Data data_v_3(create::V_3);
  boost::shared_ptr<create::Packet> v_3_packet_ptr = data_v_3.getPacket(create::ID_LIGHT_FRONT_RIGHT);
  EXPECT_NE(v_3_packet_ptr, boost::shared_ptr<create::Packet>())
      << "ID_LIGHT_FRONT_RIGHT packet not found for protocol V_3";
  EXPECT_EQ(static_cast<int>(v_3_packet_ptr->nbytes), 2);
  EXPECT_EQ(v_3_packet_ptr->info, std::string("light_bumper_front_right"));

  // Get a non-existent packet
  boost::shared_ptr<create::Packet> not_a_packet_ptr = data_v_3.getPacket(60);
  EXPECT_EQ(not_a_packet_ptr, boost::shared_ptr<create::Packet>());
}

TEST(DataTest, GetPacketIDs)
{
  create::Data data_v_3(create::V_3);
  const std::vector<uint8_t> packet_ids = data_v_3.getPacketIDs();
  // Vector should have same length as reported by getNumPackets()
  ASSERT_EQ(static_cast<int>(packet_ids.size()), 29);

  // Vector should contain ID_LEFT_ENC
  bool found = false;
  for (std::size_t i = 0; (i < packet_ids.size()) && !found; i++)
  {
    if (packet_ids[i] == create::ID_LEFT_ENC)
    {
      found = true;
    }
  }
  EXPECT_TRUE(found) << "ID_LEFT_ENC packet ID not returned for protocol V_3";
}

TEST(DataTest, GetTotalDataBytes)
{
  // All protocols have 20 mutual data bytes
  // V_1 has an additional 6 bytes
  create::Data data_v_1(create::V_1);
  EXPECT_EQ(static_cast<int>(data_v_1.getTotalDataBytes()), 26);

  // V_2 has an additional 5 bytes
  create::Data data_v_2(create::V_2);
  EXPECT_EQ(static_cast<int>(data_v_2.getTotalDataBytes()), 25);

  // V_3 has an additional 21 bytes
  create::Data data_v_3(create::V_3);
  EXPECT_EQ(static_cast<int>(data_v_3.getTotalDataBytes()), 41);
}

TEST(DataTest, IsValidPacketID)
{
  create::Data data_v_1(create::V_1);
  EXPECT_TRUE(data_v_1.isValidPacketID(create::ID_DIRT_DETECT_RIGHT))
      << "ID_DIRT_DETECT_RIGHT packet not found for protocol V_1";
  EXPECT_FALSE(data_v_1.isValidPacketID(create::ID_OI_MODE))
      << "ID_OI_MODE packet should not exist for protocol V_1";

  // V_2 has an additional 5 bytes
  create::Data data_v_2(create::V_2);
  EXPECT_TRUE(data_v_2.isValidPacketID(create::ID_ANGLE))
      << "ID_ANGLE packet not found for protocol V_2";
  EXPECT_FALSE(data_v_2.isValidPacketID(create::ID_LIGHT))
      << "ID_LIGHT packet should not exist for protocol V_2";

  // V_3 has an additional 21 bytes
  create::Data data_v_3(create::V_3);
  EXPECT_TRUE(data_v_3.isValidPacketID(create::ID_STASIS))
      << "ID_STATIS packet not found for protocol V_3";
  EXPECT_FALSE(data_v_3.isValidPacketID(create::ID_DISTANCE))
      << "ID_DISTANCE packet should not exist for protocol V_3";
}

TEST(DataTest, ValidateAll)
{
  create::Data data_v_3(create::V_3);
  // Don't crash
  data_v_3.validateAll();
}
