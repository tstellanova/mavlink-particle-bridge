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


#ifndef MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS 1

#include "mavlink_types.h"

//required by helper functions:
mavlink_system_t mavlink_system;

/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    if (chan == MAVLINK_COMM_0)
    {
        Serial1.write(ch);
    }
}

#endif

#include "mavlink.h"



namespace ParticleMavlinkLibrary 
{
  //Manages a mavlink connection via Serial
  class MavlinkBridge
  {
  protected:
    mavlink_message_t m_msg;
    mavlink_status_t m_status;
    bool m_mavlink_available;
  public:
    MavlinkBridge() {};
    void init();//TODO make Serial port and baud rate configurable
    bool readMavlinkMsg(mavlink_message_t& msg);
    bool sendMavlinkMsg(mavlink_message_t& msg);
    bool sendCommandRTL();
    bool sendCommandLand();
  };
}

#endif
