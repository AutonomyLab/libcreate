#include <iostream>

#include "create/serial_query.h"
#include "create/types.h"

#define SENSORS_RESPONSE_LENGTH 20

namespace create {

  SerialQuery::SerialQuery(boost::shared_ptr<Data> d) : Serial(d),
    streamRecoveryTimer(io),
    packetID(ID_BUMP_WHEELDROP),
    packetByte(0),
    packetData(0),
    maxPacketID(ID_CAPACITY) {
  }

  bool SerialQuery::startSensorStream() {
    if (!started) {
      requestSensorData();
      started = true;
    }
    return true;
  }

  void SerialQuery::requestSensorData() {
    static const uint8_t requestPacket[2] = { OC_SENSORS, ID_GROUP_0 };
    // Prevents previous packet from corrupting next one
    flushInput();
    send(requestPacket, 2);
    // Automatically resend request if no response is received
    streamRecoveryTimer.expires_from_now(boost::posix_time::milliseconds(50));
    streamRecoveryTimer.async_wait(boost::bind(&SerialQuery::restartSensorStream, this, _1));
  }

  void SerialQuery::restartSensorStream(const boost::system::error_code& err) {
    if (err != boost::asio::error::operation_aborted) {
      if (packetID != ID_BUMP_WHEELDROP) {
        ++corruptPackets;
      }
      requestSensorData();
    }
  }

  void SerialQuery::flushInput() {
    // Only works with POSIX support
    tcflush(port.lowest_layer().native_handle(), TCIFLUSH);
  }

  void SerialQuery::processByte(uint8_t byteRead) {
    packetData |= (static_cast<uint16_t>(byteRead) << (8 * packetByte));

    if (packetByte > 0) {
      --packetByte;
    } else if (packetID < maxPacketID) {
      // New packet
      data->getPacket(packetID)->setDataToValidate(packetData);
      packetData = 0;
      ++packetID;
      packetByte = data->getPacket(packetID)->nbytes - 1;
    } else {
      // Response finished
      packetID = ID_BUMP_WHEELDROP;
      packetByte = 0;
      packetData = 0;
      notifyDataReady();
      requestSensorData();
    }
  }
} // namespace create
