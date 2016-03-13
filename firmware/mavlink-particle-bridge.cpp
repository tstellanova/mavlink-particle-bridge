// NOTE/NUANCE: When including WITHIN a library, no sub-dir prefix is needed.
#include "mavlink-particle-bridge.h"

namespace ParticleMavlinkLibrary {

void MavlinkBridge::init()
{
  // open serial over TX and RX pins
  Serial1.begin(57600);  
  while (!Serial1.available()) {
    Particle.process();
  }
}

bool MavlinkBridge::readMavlinkMsg(mavlink_message_t& msg)
{
  while(Serial1.available() > 0) { 
    uint8_t c = Serial1.read();

    //hand the next byte to the mavlink parser, try to find a complete msg
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &m_msg, &m_status)) {
      m_mavlink_available = true;
      msg = m_msg;
      return true;
    }
    //TODO look at m_status ?
  }

  return false;
}

bool MavlinkBridge::sendMavlinkMsg(mavlink_message_t& msg)
{
  //TODO implement generic message send?
  return false;
}


int MavlinkBridge::handleCommand(String params)
{
  if (params == "RTL") {
    sendCommandRTL();
    return 0;
  }
  else if (params == "Land") {
    sendCommandLand();
    return 0;
  }
  
  return -1;
}


bool MavlinkBridge::sendCommandRTL()
{
  mavlink_command_long_t packet;
  memset(&packet, 0, sizeof(packet));
  // mavlink_msg_command_long_send(MAVLINK_COMM_0, m_msg.sysid, m_msg.compid, 
  //   MAV_CMD_NAV_RETURN_TO_LAUNCH, 
  //   packet.confirmation, 
  //   packet.param1, 
  //   packet.param2, 
  //   packet.param3, 
  //   packet.param4, 
  //   packet.param5, 
  //   packet.param6, 
  //   packet.param7 );

  return true;
}

bool MavlinkBridge::sendCommandLand()
{
  mavlink_command_long_t packet;
  memset(&packet, 0, sizeof(packet));
  // mavlink_msg_command_long_send(MAVLINK_COMM_0, m_msg.sysid, m_msg.compid, 
  //   MAV_CMD_NAV_LAND, 
  //   packet.confirmation, 
  //   packet.param1, 
  //   packet.param2, 
  //   packet.param3, 
  //   packet.param4, 
  //   packet.param5, 
  //   packet.param6, 
  //   packet.param7);

  return true;
}

}


