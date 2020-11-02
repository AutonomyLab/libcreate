#include <memory>

#include "create/packet.h"

namespace create {

  Packet::Packet(const uint8_t& numBytes, const std::string& comment) :
    data(0),
    tmpData(0),
    nbytes(numBytes),
    info(comment) { }

  Packet::~Packet() { }

  void Packet::setDataToValidate(const uint16_t& tmp) {
    std::lock_guard<std::mutex> lock(tmpDataMutex);
    tmpData = tmp;
  }

  void Packet::validate() {
    std::lock_guard<std::mutex> lock(tmpDataMutex);
    setData(tmpData);
  }

  void Packet::setData(const uint16_t& d) {
    std::lock_guard<std::mutex> lock(dataMutex);
    data = d;
  }

  uint16_t Packet::getData() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return data;
  }

} // namespace create
