#include <iostream>

#include "create/serial.h"
#include "create/types.h"

namespace create {
  
  Serial::Serial(boost::shared_ptr<Data> d, const uint8_t& header) :
    data(d),
    headerByte(header),
    port(io), 
    readState(READ_HEADER), 
    isReading(false),
    dataReady(false),
    corruptPackets(0),
    totalPackets(0) { 
    //std::cout << "# Serial Created" << std::endl;
  }
  
  Serial::~Serial() { 
    disconnect();
    //std::cout << "# Serial Destroyed" << std::endl;
  }
  
  bool Serial::connect(const std::string& portName, const int& baud, boost::function<void()> cb) {
    //std::cout << "## Serial connect start" << std::endl;
    using namespace boost::asio;
    port.open(portName);
    port.set_option(serial_port::baud_rate(baud));
    port.set_option(serial_port::flow_control(serial_port::flow_control::none));
  
    if (port.is_open()) {
      callback = cb;
      bool startReadSuccess = startReading();
      if (!startReadSuccess)
        port.close();
      //std::cout << "## Serial connect done" << std::endl;
      return startReadSuccess;
    }
    //std::cout << "## Serial connect failed" << std::endl;
    return false;
  }
  
  void Serial::disconnect() {
    if (isReading) {
      stopReading();
    }

    if (connected()) {
      //std::cout << "## Serial disconnect start" << std::endl;
      // Ensure not in Safe/Full modes
      sendOpcode(OC_START);
      // Stop OI
      sendOpcode(OC_STOP);
      port.close();
      //std::cout << "## Serial disconnect done" << std::endl;
    }
  }
  
  bool Serial::startReading() {
    if (!connected()) return false;

    if (!data) {
      CERR("[create::Serial] ", "data pointer not initialized.");
      return false;
    }

    // Only allow once
    if (isReading) return true;

    //std::cout << "### Serial start reading" << std::endl;
    // Request from Create that we want a stream containing all packets
    uint8_t numPackets = data->getNumPackets();
    std::vector<uint8_t> packetIDs = data->getPacketIDs();
    uint8_t streamReq[2 + numPackets];
    streamReq[0] = OC_STREAM;
    streamReq[1] = numPackets;
    int i = 2;
    for (std::vector<uint8_t>::iterator it = packetIDs.begin(); it != packetIDs.end(); ++it) {
      streamReq[i] = *it;
      i++;
    }

    // Start OI
    sendOpcode(OC_START);

    // Start streaming data
    send(streamReq, 2 + numPackets); 
 
    expectedNumBytes = data->getTotalDataBytes() + numPackets;
    
    //TODO: handle boost exceptions

    io.reset();

    // Start continuously reading one byte at a time
    boost::asio::async_read(port,
                            boost::asio::buffer(&byteRead, 1),
                            boost::bind(&Serial::onData, this, _1, _2));

    ioThread = boost::thread(boost::bind(&boost::asio::io_service::run, &io)); 

    // Wait for first complete read to finish
    boost::unique_lock<boost::mutex> lock(dataReadyMut);
    //std::cout << "#### Waiting for dataReady" << std::endl;
    int attempts = 1;
    int maxAttempts = 10;
    while (!dataReady) {
      if (!dataReadyCond.timed_wait(lock, boost::get_system_time() + boost::posix_time::milliseconds(500))) {
        if (attempts >= maxAttempts) {
          CERR("[create::Serial] ", "failed to receive data from Create. Check if robot is powered!");
          io.stop();
          ioThread.join();
          return false;
        } 
        attempts++;
        //std::cout << "Requesting data from Create. Attempt " << attempts << std::endl;
        // Request data again
        sendOpcode(OC_START);
        send(streamReq, 2 + numPackets); 
      }
    }
    //std::cout << "#### Data is ready." << std::endl;
    isReading = true;
    //std::cout << "### Serial start reading DONE" << std::endl;
    return true;
  }
  
  void Serial::stopReading() {
    if (isReading) {
      //std::cout << "### Start stopReading" << std::endl;
      io.stop();
      ioThread.join();
      isReading = false;
      {
        boost::lock_guard<boost::mutex> lock(dataReadyMut);
        dataReady = false;
      }
      //std::cout << "### End stopReading" << std::endl;
    }
  }

  void Serial::onData(const boost::system::error_code& e, const std::size_t& size) {
    //std::cout << "#### onData" << std::endl;
    if (e) {
      CERR("[create::Serial] ", "serial error - " << e.message());
      return;
    }
  
    // Should have read exactly one byte
    if (size == 1) {
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
          else 
            readState = READ_HEADER;
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
            readState = READ_HEADER;
          }
        break;
  
        case READ_PACKET_BYTES:
          numDataBytesRead++;
          if (expectedNumDataBytes == 2 && numDataBytesRead == 1) {
            // high byte first
            packetBytes = (byteRead << 8) & 0xff00;
          }
          else {
            // low byte
            packetBytes += byteRead;
          }
          if (numDataBytesRead >= expectedNumDataBytes) {
            data->getPacket(packetID)->setTempData(packetBytes);
            if (numBytesRead >= expectedNumBytes)
              readState = READ_CHECKSUM;
            else
              readState = READ_PACKET_ID;
          }
        break;
  
        case READ_CHECKSUM:
          if ((byteSum & 0xFF) == 0) {
            // Validate all packets
            data->validateAll();

            // Notify first data packets ready
            {
              boost::lock_guard<boost::mutex> lock(dataReadyMut);
             // std::cout << "locking." << std::endl;
              if (!dataReady) {
                dataReady = true;
                dataReadyCond.notify_one();
                //std::cout << "##### Notified." << std::endl;
              }
            }
            // Callback to notify data is ready
            if (callback)
              callback();        
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
    } // end if (size == 1)

    // Read the next byte
    boost::asio::async_read(port,
                            boost::asio::buffer(&byteRead, 1),
                            boost::bind(&Serial::onData, this, _1, _2));
  }
  
  bool Serial::send(const uint8_t* bytes, unsigned int numBytes) {
    if (!connected()) {
      CERR("[create::Serial] ", "send failed, not connected."); 
      return false;
    }
    // TODO: catch boost exceptions
    boost::asio::write(port, boost::asio::buffer(bytes, numBytes));
    return true;
  }

  bool Serial::sendOpcode(const Opcode& code) {
    uint8_t oc = (uint8_t) code;
    return send(&oc, 1);
  }

} // namespace create
