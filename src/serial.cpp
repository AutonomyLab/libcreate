#include <chrono>
#include <functional>
#include <iostream>
#include <thread>

#include <serial/serial.h>

#include "create/serial.h"
#include "create/types.h"

namespace create {

  Serial::Serial(std::shared_ptr<Data> d) :
    serial(),
    dataReady(false),
    isReading(false),
    data(d),
    corruptPackets(0),
    totalPackets(0) {
  }

  Serial::~Serial() {
    disconnect();
  }

  // TODO
  // void Serial::signalHandler(const boost::system::error_code& error, int signal_number) {
  //   if (!error) {
  //     if (connected()) {
  //       // Ensure not in Safe/Full modes
  //       sendOpcode(OC_START);
  //       // Stop OI
  //       sendOpcode(OC_STOP);
  //       exit(signal_number);
  //     }
  //   }
  // }

  bool Serial::connect(const std::string& portName, const int& baud, std::function<void()> cb) {
    // using namespace boost::asio;
    serial.setPort(portName);
    serial.setBaudrate(baud);
    serial.open();

    // TODO
    // signals.async_wait(std::bind(&Serial::signalHandler, this, std::placeholders::_1, std::placeholders::_2));

    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (serial.isOpen()) {
      callback = cb;
      bool startReadSuccess = startReading();
      if (!startReadSuccess) {
        serial.close();
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
      serial.close();
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

    // Start continuously reading one byte at a time
    ioThread = std::thread(std::bind(&Serial::onData, this));

    // Wait for first complete read to finish
    // TODO: remove this mutex and use atomic variable?
    std::unique_lock<std::mutex> lock(dataReadyMut);

    int attempts = 1;
    int maxAttempts = 10;
    while (!dataReady) {
      if (dataReadyCond.wait_for(lock, std::chrono::milliseconds(500)) == std::cv_status::timeout) {
        if (attempts >= maxAttempts) {
          CERR("[create::Serial] ", "failed to receive data from Create. Check if robot is powered!");
          serial.close();
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
      serial.close();
      ioThread.join();
      isReading = false;
      {
        std::lock_guard<std::mutex> lock(dataReadyMut);
        dataReady = false;
      }
    }
  }


  void Serial::notifyDataReady() {
    // Validate all packets
    data->validateAll();

    // Notify first data packets ready
    {
      std::lock_guard<std::mutex> lock(dataReadyMut);
      if (!dataReady) {
        dataReady = true;
        dataReadyCond.notify_one();
      }
    }
    // Callback to notify data is ready
    if (callback)
      callback();
  }

  void Serial::onData() {
    // TODO check for program interrupt
    while (true) {
      size_t size = 0u;
      try {
        size = serial.read(&byteRead, 1);
      } catch (const std::runtime_error & e) {
        CERR("[create::Serial] ", "serial error - " << e.what());
        continue;
      }

      // Should have read exactly one byte
      if (size == 1) {
        processByte(byteRead);
      }
    }
  }

  bool Serial::send(const uint8_t* bytes, unsigned int numBytes) {
    if (!connected()) {
      CERR("[create::Serial] ", "send failed, not connected.");
      return false;
    }
    serial.write(bytes, static_cast<size_t>(numBytes));
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
