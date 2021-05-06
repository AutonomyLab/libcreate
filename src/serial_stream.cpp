#include <iostream>
#include <memory>

#include "create/serial_stream.h"
#include "create/types.h"

namespace create {

  SerialStream::SerialStream(std::shared_ptr<Data> d, const uint8_t& header, bool install_signal_handler) : Serial(d, install_signal_handler), readState(READ_HEADER), headerByte(header) {
  }

  bool SerialStream::startSensorStream() {
    // Request from Create that we want a stream containing all packets
    const uint8_t numPackets = data->getNumPackets();
    std::vector<uint8_t> packetIDs = data->getPacketIDs();
    packetIDs.insert(packetIDs.begin(), numPackets);
    packetIDs.insert(packetIDs.begin(), OC_STREAM);

    // Start streaming data
    send(packetIDs.data(), packetIDs.size());

    expectedNumBytes = data->getTotalDataBytes() + numPackets;

    return true;
  }

  void SerialStream::processByte(uint8_t byteRead) {
    numBytesRead++;
    byteSum += byteRead;
    switch (readState) {
      case READ_HEADER:
        if (byteRead == headerByte) {
          readState = READ_NBYTES;
          byteSum = byteRead;
        }
        break;

      case READ_NBYTES:
        if (byteRead == expectedNumBytes) {
          readState = READ_PACKET_ID;
          numBytesRead = 0;
        }
        else {
          //notifyDataReady();
          readState = READ_HEADER;
        }
        break;

      case READ_PACKET_ID:
        packetID = byteRead;
        if (data->isValidPacketID(packetID)) {
          expectedNumDataBytes = data->getPacket(packetID)->nbytes;
          assert(expectedNumDataBytes == 1 || expectedNumDataBytes == 2);
          numDataBytesRead = 0;
          packetBytes = 0;
          readState = READ_PACKET_BYTES;
        }
        else {
          //notifyDataReady();
          readState = READ_HEADER;
        }
        break;

      case READ_PACKET_BYTES:
        numDataBytesRead++;
        if (expectedNumDataBytes == 2 && numDataBytesRead == 1) {
          // High byte first
          packetBytes = (byteRead << 8) & 0xff00;
        }
        else {
          // Low byte
          packetBytes += byteRead;
        }
        if (numDataBytesRead >= expectedNumDataBytes) {
          data->getPacket(packetID)->setDataToValidate(packetBytes);
          if (numBytesRead >= expectedNumBytes)
            readState = READ_CHECKSUM;
          else
            readState = READ_PACKET_ID;
        }
        break;

      case READ_CHECKSUM:
        if ((byteSum & 0xFF) == 0) {
          notifyDataReady();
        }
        else {
          // Corrupt data
          corruptPackets++;
        }
        totalPackets++;
        // Start again
        readState = READ_HEADER;
        break;
    } // end switch (readState)
  }

} // namespace create
