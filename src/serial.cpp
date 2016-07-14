#include <iostream>

#include "create/serial.h"
#include "create/types.h"

namespace create {

  Serial::Serial(boost::shared_ptr<Data> d) :
    data(d),
    port(io),
    isReading(false),
    dataReady(false),
    corruptPackets(0),
    totalPackets(0) {
  }

  Serial::~Serial() {
    disconnect();
  }

  bool Serial::connect(const std::string& portName, const int& baud, boost::function<void()> cb) {
    using namespace boost::asio;
    port.open(portName);
    port.set_option(serial_port::baud_rate(baud));
    port.set_option(serial_port::flow_control(serial_port::flow_control::none));

    usleep(1000000);

    if (port.is_open()) {
      callback = cb;
      bool startReadSuccess = startReading();
      if (!startReadSuccess) {
        port.close();
      }
      return startReadSuccess;
    }
    return false;
  }

  void Serial::disconnect() {
    if (isReading) {
      stopReading();
    }

    if (connected()) {
      // Ensure not in Safe/Full modes
      sendOpcode(OC_START);
      // Stop OI
      sendOpcode(OC_STOP);
      port.close();
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

    // Start OI
    sendOpcode(OC_START);

    if (!startSensorStream()) return false;

    io.reset();

    // Start continuously reading one byte at a time
    boost::asio::async_read(port,
                            boost::asio::buffer(&byteRead, 1),
                            boost::bind(&Serial::onData, this, _1, _2));

    ioThread = boost::thread(boost::bind(&boost::asio::io_service::run, &io));

    // Wait for first complete read to finish
    boost::unique_lock<boost::mutex> lock(dataReadyMut);

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

        // Request data again
        sendOpcode(OC_START);
        startSensorStream();
      }
    }

    isReading = true;
    return true;
  }

  void Serial::stopReading() {
    if (isReading) {
      io.stop();
      ioThread.join();
      isReading = false;
      {
        boost::lock_guard<boost::mutex> lock(dataReadyMut);
        dataReady = false;
      }
    }
  }


  void Serial::notifyDataReady() {
    // Validate all packets
    data->validateAll();

    // Notify first data packets ready
    {
      boost::lock_guard<boost::mutex> lock(dataReadyMut);
      if (!dataReady) {
        dataReady = true;
        dataReadyCond.notify_one();
      }
    }
    // Callback to notify data is ready
    if (callback)
      callback();
  }

  void Serial::onData(const boost::system::error_code& e, const std::size_t& size) {
    if (e) {
      CERR("[create::Serial] ", "serial error - " << e.message());
      return;
    }

    // Should have read exactly one byte
    if (size == 1) {
      processByte(byteRead);
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

  uint64_t Serial::getNumCorruptPackets() const {
    return corruptPackets;
  }

  uint64_t Serial::getTotalPackets() const {
    return totalPackets;
  }
} // namespace create
