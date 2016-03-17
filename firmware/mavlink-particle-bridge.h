#ifndef _PARTICLE_MAVLINK_LIB
#define _PARTICLE_MAVLINK_LIB

// Make library cross-compatiable
// with Arduino, GNU C++ for tests, and Spark.
//#if defined(ARDUINO) && ARDUINO >= 100
//#include "Arduino.h"
//#elif defined(SPARK)
//#include "application.h"
//#endif

// TEMPORARY UNTIL the stuff that supports the code above is deployed to the build IDE
#include "application.h"

#include "mavlink_glue.h"


namespace ParticleMavlinkLibrary 
{
  //Manages a mavlink connection via Serial
  class MavlinkBridge
  {
  protected:
    mavlink_message_t m_msg;
    mavlink_status_t m_status;
    bool m_mavlink_available;

    uint8_t m_system_type;
    uint8_t m_autopilot_type;
    uint8_t m_system_mode;
    uint32_t m_custom_mode;
    uint8_t m_system_state;
    
    uint8_t m_mavlink_sysid;
    uint8_t m_mavlink_compid;

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
  public:
    MavlinkBridge() {};
    void init();//TODO make Serial port and baud rate configurable
    bool readMavlinkMsg(mavlink_message_t& msg);
    bool handleCommand(String params);
    
  protected:
    bool sendCommandRTL();
    bool sendCommandLand();
    bool sendCommandTakeoff();
    bool sendLongCommand(uint16_t command);

  };
}

#endif
