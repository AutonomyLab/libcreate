#include "create/data.h"

#define ADD_PACKET(id,nbytes,info) (packets[id]=boost::make_shared<Packet>(nbytes,info))

namespace create {

  Data::Data(RobotModel model) {
    // Populate data map
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *  WARNING: Adding too many packets will flood the serial   *
     *           buffer and corrupt the stream.                  *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    ADD_PACKET(ID_BUMP_WHEELDROP, 1, "bumps_wheeldrops");
    ADD_PACKET(ID_WALL, 1, "wall");
    ADD_PACKET(ID_CLIFF_LEFT, 1, "cliff_left");
    ADD_PACKET(ID_CLIFF_FRONT_LEFT, 1, "cliff_front_left");
    ADD_PACKET(ID_CLIFF_FRONT_RIGHT, 1, "cliff_front_right");
    ADD_PACKET(ID_CLIFF_RIGHT, 1, "cliff_right");
    ADD_PACKET(ID_IR_OMNI, 1, "ir_opcode");
    ADD_PACKET(ID_BUTTONS, 1, "buttons");
    ADD_PACKET(ID_CHARGE_STATE, 1, "charging_state");
    ADD_PACKET(ID_VOLTAGE, 2, "voltage");
    ADD_PACKET(ID_CURRENT, 2, "current");
    ADD_PACKET(ID_TEMP, 1, "temperature");
    ADD_PACKET(ID_CHARGE , 2, "battery_charge");
    ADD_PACKET(ID_CAPACITY, 2, "battery_capacity");
    ADD_PACKET(ID_VIRTUAL_WALL, 1, "virtual_wall");

    if (model == CREATE_1) {
      ADD_PACKET(ID_DISTANCE, 2, "distance");
      ADD_PACKET(ID_ANGLE, 2, "angle");
    }
    else if (model == CREATE_2) {
      //ADD_PACKET(ID_OVERCURRENTS, 1, "overcurrents");
      ADD_PACKET(ID_DIRT_DETECT, 1, "dirt_detect");
      //ADD_PACKET(ID_UNUSED_1, 1, "unused 1");
      //ADD_PACKET(ID_WALL_SIGNAL, 2, "wall_signal");
      //ADD_PACKET(ID_CLIFF_LEFT_SIGNAL, 2, "cliff_left_signal");
      //ADD_PACKET(ID_CLIFF_FRONT_LEFT_SIGNAL, 2, "cliff_front_left_signal");
      //ADD_PACKET(ID_CLIFF_FRONT_RIGHT_SIGNAL, 2, "cliff_front_right_signal");
      //ADD_PACKET(ID_CLIFF_RIGHT_SIGNAL, 2, "cliff_right_signal");
      //ADD_PACKET(ID_UNUSED_2, 1, "unused 2");
      //ADD_PACKET(ID_UNUSED_3, 2, "unused 3");
      //ADD_PACKET(ID_CHARGE_SOURCE, 1, "charger_available");
      //ADD_PACKET(ID_IO_MODE, 1, "oi_mode");
      //ADD_PACKET(ID_SONG_NUM, 1, "song_number");
      //ADD_PACKET(ID_PLAYING, 1, "song_playing");
      //ADD_PACKET(ID_NUM_STREAM_PACKETS, 1, "oi_stream_num_packets");
      //ADD_PACKET(ID_VEL, 2, "velocity");
      //ADD_PACKET(ID_RADIUS, 2, "radius");
      //ADD_PACKET(ID_RIGHT_VEL, 2, "velocity_right");
      //ADD_PACKET(ID_LEFT_VEL, 2, "velocity_left");
      ADD_PACKET(ID_LEFT_ENC, 2, "enc_counts_left");
      ADD_PACKET(ID_RIGHT_ENC, 2, "enc_counts_right");
      ADD_PACKET(ID_LIGHT, 1, "light_bumper");
      //ADD_PACKET(ID_LIGHT_LEFT, 2, "light_bumper_left");
      //ADD_PACKET(ID_LIGHT_FRONT_LEFT, 2, "light_bumper_front_left");
      //ADD_PACKET(ID_LIGHT_CENTER_LEFT, 2, "light_bumper_center_left");
      //ADD_PACKET(ID_LIGHT_CENTER_RIGHT, 2, "light_bumper_center_right");
      //ADD_PACKET(ID_LIGHT_FRONT_RIGHT, 2, "light_bumper_front_right");
      //ADD_PACKET(ID_LIGHT_RIGHT, 2, "light_bumper_right");
      ADD_PACKET(ID_IR_LEFT, 1, "ir_opcode_left");
      ADD_PACKET(ID_IR_RIGHT, 1, "ir_opcode_right");
      //ADD_PACKET(ID_LEFT_MOTOR_CURRENT, 2, "left_motor_current");
      //ADD_PACKET(ID_RIGHT_MOTOR_CURRENT, 2, "right_motor_current");
      //ADD_PACKET(ID_MAIN_BRUSH_CURRENT, 2, "main_brush_current");
      //ADD_PACKET(ID_SIDE_BRUSH_CURRENT, 2, "side_brush_current");
      ADD_PACKET(ID_STASIS, 1, "stasis");
    } // if CREATE_2

    totalDataBytes = 0;
    for (std::map<uint8_t, boost::shared_ptr<Packet> >::iterator it = packets.begin();
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

  boost::shared_ptr<Packet> Data::getPacket(uint8_t id) {
    if (isValidPacketID(id)) {
      return packets[id];
    }
    std::cout << "Invalid packet " << (int) id << " requested" << std::endl;
    return boost::shared_ptr<Packet>();
  }

  void Data::validateAll() {
    for (std::map<uint8_t, boost::shared_ptr<Packet> >::iterator it = packets.begin();
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
