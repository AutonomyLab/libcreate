/**
Software License Agreement (BSD)

\file      test_serial_query.cpp
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
#include "create/serial_query.h"

#include "gtest/gtest.h"

#include <boost/shared_ptr.hpp>

TEST(SerialQueryTest, Constructor)
{
  boost::shared_ptr<create::Data> data_ptr = boost::make_shared<create::Data>();
  create::SerialQuery serial_query(data_ptr);
}

TEST(SerialQueryTest, Connected)
{
  boost::shared_ptr<create::Data> data_ptr = boost::make_shared<create::Data>();
  create::SerialQuery serial_query(data_ptr);

  // Did not call connect and nothing to connect to, so expect false
  EXPECT_FALSE(serial_query.connected());
}

TEST(SerialQueryTest, Disconnect)
{
  boost::shared_ptr<create::Data> data_ptr = boost::make_shared<create::Data>();
  create::SerialQuery serial_query(data_ptr);

  // Not connected, but should not fail
  serial_query.disconnect();
}

TEST(SerialQueryTest, NumPackets)
{
  boost::shared_ptr<create::Data> data_ptr = boost::make_shared<create::Data>();
  create::SerialQuery serial_query(data_ptr);

  // Not connected, so zero packets should have been received
  EXPECT_EQ(serial_query.getNumCorruptPackets(), 0);
  EXPECT_EQ(serial_query.getTotalPackets(), 0);
}

TEST(SerialQueryTest, Send)
{
  boost::shared_ptr<create::Data> data_ptr = boost::make_shared<create::Data>();
  create::SerialQuery serial_query(data_ptr);

  // Some bytes to send (to set date)
  uint8_t bytes[4] = { create::OC_DATE, 0, 1, 2 };
  // Not connected, so failure expected
  EXPECT_FALSE(serial_query.send(bytes, 4));
}

TEST(SerialQueryTest, SendOpcode)
{
  boost::shared_ptr<create::Data> data_ptr = boost::make_shared<create::Data>();
  create::SerialQuery serial_query(data_ptr);

  // Not connected, so failure expected
  EXPECT_FALSE(serial_query.sendOpcode(create::OC_POWER));
}
