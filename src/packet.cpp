#include "create/packet.h"

namespace create {

  Packet::Packet(const uint8_t& numBytes, const std::string& comment) : 
    nbytes(numBytes),
    info(comment),
    data(0),
    tmpData(0) { }
  
  Packet::~Packet() { }
  
  void Packet::setTempData(const uint16_t& tmp) {
    // mutex for tmpData ?
    tmpData = tmp;
  }
  
  void Packet::validate() {
    setData(tmpData);
  }
  
  void Packet::setData(const uint16_t& d) {
    boost::mutex::scoped_lock lock(dataMutex);
    data = d;
  }
  
  uint16_t Packet::getData() const {
    boost::mutex::scoped_lock lock(dataMutex);
    return data;
  }

} // namespace create
