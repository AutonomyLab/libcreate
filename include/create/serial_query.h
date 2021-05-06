/**
Software License Agreement (BSD)

\file      serial_query.h
\authors   Jacob Perron <jperron@sfu.ca>
\authors   Ben Wolsieffer <benwolsieffer@gmail.com>
\copyright Copyright (c) 2015, Autonomy Lab (Simon Fraser University), All rights reserved.

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

// Based on example from:
//   https://github.com/labust/labust-ros-pkg/wiki/Create-a-Serial-Port-application

#ifndef CREATE_SERIAL_QUERY_H
#define CREATE_SERIAL_QUERY_H

#include <memory>

#include <boost/asio.hpp>

#include "create/data.h"
#include "create/types.h"
#include "create/util.h"
#include "create/serial.h"

namespace create {
  class SerialQuery : public Serial {

    private:
      boost::asio::deadline_timer streamRecoveryTimer;
      uint8_t packetID;
      int8_t packetByte;
      uint16_t packetData;
      const uint8_t maxPacketID;

      bool started;

      void requestSensorData();
      void restartSensorStream(const boost::system::error_code& err);

      void flushInput();

    protected:
      bool startSensorStream();
      void processByte(uint8_t byteRead);

    public:
      SerialQuery(std::shared_ptr<Data> data, bool install_signal_handler = true);
  };
}  // namespace create

#endif // CREATE_SERIAL_H
