#include "create/data.h"

#define ADD_PACKET(id,nbytes,info,enabledVersion) if ((enabledVersion) & version) packets[id]=std::make_shared<Packet>(nbytes,info)

namespace create {

  Data::Data(ProtocolVersion version) {
    // Populate data map
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *  WARNING: Adding too many packets will flood the serial   *
     *           buffer and corrupt the stream.                  *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    ADD_PACKET(ID_BUMP_WHEELDROP, 1, "bumps_wheeldrops", V_ALL);
    ADD_PACKET(ID_WALL, 1, "wall", V_ALL);
    ADD_PACKET(ID_CLIFF_LEFT, 1, "cliff_left", V_ALL);
    ADD_PACKET(ID_CLIFF_FRONT_LEFT, 1, "cliff_front_left", V_ALL);
    ADD_PACKET(ID_CLIFF_FRONT_RIGHT, 1, "cliff_front_right", V_ALL);
    ADD_PACKET(ID_CLIFF_RIGHT, 1, "cliff_right", V_ALL);
    ADD_PACKET(ID_VIRTUAL_WALL, 1, "virtual_wall", V_ALL);
    ADD_PACKET(ID_OVERCURRENTS, 1, "overcurrents", V_ALL);
    ADD_PACKET(ID_DIRT_DETECT_LEFT, 1, "dirt_detect_left", V_ALL);
    ADD_PACKET(ID_DIRT_DETECT_RIGHT, 1, "dirt_detect_right", V_1);
    ADD_PACKET(ID_IR_OMNI, 1, "ir_opcode", V_ALL);
    ADD_PACKET(ID_BUTTONS, 1, "buttons", V_ALL);
    ADD_PACKET(ID_DISTANCE, 2, "distance", V_1 | V_2);
    ADD_PACKET(ID_ANGLE, 2, "angle", V_1 | V_2);
    ADD_PACKET(ID_CHARGE_STATE, 1, "charging_state", V_ALL);
    ADD_PACKET(ID_VOLTAGE, 2, "voltage", V_ALL);
    ADD_PACKET(ID_CURRENT, 2, "current", V_ALL);
    ADD_PACKET(ID_TEMP, 1, "temperature", V_ALL);
    ADD_PACKET(ID_CHARGE , 2, "battery_charge", V_ALL);
    ADD_PACKET(ID_CAPACITY, 2, "battery_capacity", V_ALL);
    ADD_PACKET(ID_OI_MODE, 1, "oi_mode", V_2 | V_3);
    ADD_PACKET(ID_LEFT_ENC, 2, "enc_counts_left", V_3);
    ADD_PACKET(ID_RIGHT_ENC, 2, "enc_counts_right", V_3);
    ADD_PACKET(ID_LIGHT, 1, "light_bumper", V_3);
    ADD_PACKET(ID_LIGHT_LEFT, 2, "light_bumper_left", V_3);
    ADD_PACKET(ID_LIGHT_FRONT_LEFT, 2, "light_bumper_front_left", V_3);
    ADD_PACKET(ID_LIGHT_CENTER_LEFT, 2, "light_bumper_center_left", V_3);
    ADD_PACKET(ID_LIGHT_CENTER_RIGHT, 2, "light_bumper_center_right", V_3);
    ADD_PACKET(ID_LIGHT_FRONT_RIGHT, 2, "light_bumper_front_right", V_3);
    ADD_PACKET(ID_LIGHT_RIGHT, 2, "light_bumper_right", V_3);
    ADD_PACKET(ID_IR_LEFT, 1, "ir_opcode_left", V_3);
    ADD_PACKET(ID_IR_RIGHT, 1, "ir_opcode_right", V_3);
    ADD_PACKET(ID_STASIS, 1, "stasis", V_3);

    totalDataBytes = 0;
    for (std::map<uint8_t, std::shared_ptr<Packet> >::iterator it = packets.begin();
         it != packets.end();
         ++it) {
      ids.push_back(it->first);
      totalDataBytes += it->second->nbytes;
    }
  }

  Data::~Data() { }

  bool Data::isValidPacketID(uint8_t id) const {
    if (packets.count(id)) {
      return true;
    }
    return false;
  }

  std::shared_ptr<Packet> Data::getPacket(uint8_t id) {
    if (isValidPacketID(id)) {
      return packets[id];
    }
    return std::shared_ptr<Packet>();
  }

  void Data::validateAll() {
    for (std::map<uint8_t, std::shared_ptr<Packet> >::iterator it = packets.begin();
         it != packets.end();
         ++it) {
      it->second->validate();
    }
  }

  unsigned int Data::getTotalDataBytes() const {
    return totalDataBytes;
  }

  uint8_t Data::getNumPackets() const {
    return packets.size();
  }

  std::vector<uint8_t> Data::getPacketIDs() {
    return ids;
  }

} // namespace create
