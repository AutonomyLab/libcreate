/**
Software License Agreement (BSD)

\file      serial_stream.h
\authors   Jacob Perron <jperron@sfu.ca>
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

#ifndef CREATE_SERIAL_STREAM_H
#define CREATE_SERIAL_STREAM_H

#include <memory>

#include "create/data.h"
#include "create/types.h"
#include "create/util.h"
#include "create/serial.h"

namespace create {
  class SerialStream : public Serial {
    private:
      enum ReadState {
        READ_HEADER,
        READ_NBYTES,
        READ_PACKET_ID,
        READ_PACKET_BYTES,
        READ_CHECKSUM
      };

      // State machine variables
      ReadState readState;
      uint8_t headerByte;
      uint8_t packetID;
      uint8_t expectedNumBytes;
      uint16_t packetBytes;
      uint8_t numBytesRead;
      uint32_t byteSum;
      uint8_t numDataBytesRead;
      uint8_t expectedNumDataBytes;

    protected:
      bool startSensorStream();
      void processByte(uint8_t byteRead);

    public:
      SerialStream(
        std::shared_ptr<Data> data,
        const uint8_t& header = create::util::STREAM_HEADER,
        bool install_signal_handler = true);
      virtual ~SerialStream() = default;

  };
}  // namespace create

#endif // CREATE_SERIAL_H
