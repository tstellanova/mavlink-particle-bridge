// IMPORTANT: When including a library in a firmware app, a sub dir prefix is needed
// before the particular .h file.
#include "mavlink-particle-bridge.h"

// Initialize objects from the lib; be sure not to call anything
// that requires hardware be initialized here, put those in setup()
ParticleMavlinkLibrary::MavlinkBridge mavBridge;


double	latitude_degrees = 0;
double	longitude_degrees = 0;
float   altitude_amsl_m = 0;
float   groundspeed_m_s = 0; //meters per second
float   heading_degrees = 0; //heading in degrees, 0 = North
float   battery_volts = 0;



int sendCommandRTL(String params)
{
    mavBridge.sendCommandRTL();
    return 0;
}

int sendCommandLand(String params)
{
    mavBridge.sendCommandLand();
    return 0;
}

void setup() {
    // Call functions on initialized library objects that require hardware
    // to be wired up correct and available.
    mavBridge.init();

    // register some cloud variables to be monitored:
    // note that variable names are limited to 12 characters currently
    Particle.variable("latitude", latitude_degrees);
    Particle.variable("longitude", longitude_degrees);
    Particle.variable("altitude", altitude_amsl_m);
    Particle.variable("groundspeed",groundspeed_m_s);
    Particle.variable("heading",heading_degrees);
    Particle.variable("battery_v",battery_volts);

    // register some remote commands
    // note that function names are limited to 12 characters currently
    Particle.function("commandRTL", sendCommandRTL);
    Particle.function("commandLand", sendCommandLand);

}

void handleMavlinkMsg(const mavlink_message_t& msg)
{
    switch(msg.msgid) {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            //Latitude, expressed as degrees * 1E7
            latitude_degrees = ((double)mavlink_msg_global_position_int_get_lat(&msg)) / 1E7;
            longitude_degrees = ((double)mavlink_msg_global_position_int_get_lon(&msg)) / 1E7;

            //Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84)
            altitude_amsl_m = ((float)mavlink_msg_global_position_int_get_alt(&msg))/1000.0f;

            //Altitude above ground in meters, expressed as * 1000 (millimeters)
            //tends to be inaccurate unless we have an AGL sensor
            //mavlink_msg_global_position_int_get_relative_alt(&msg);

            //Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
            heading_degrees = mavlink_msg_global_position_int_get_hdg(&msg) / 100.0f;
            //TODO: NED velocity reporting?
            //mavlink_msg_global_position_int_get_vx
        }
        break;
        case MAVLINK_MSG_ID_SYS_STATUS: {
            //obtain the battery voltage from the reading in millivolts
            battery_volts = mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f;
        }
        break;
        case MAVLINK_MSG_ID_VFR_HUD: {
            //Current ground speed in m/s
            groundspeed_m_s = mavlink_msg_vfr_hud_get_groundspeed(&msg);
            //Current heading in degrees, in compass units (0..360, 0=north)
            //heading_degrees = mavlink_msg_vfr_hud_get_heading(&msg)
        }
        break;
        case MAVLINK_MSG_ID_HEARTBEAT: {
            //TODO: handle mavlink_msg_heartbeat_get_base_mode and so forth
        }
        break;

    }
}


void loop() {
    // Use the library's initialized objects and functions
    mavlink_message_t msg;
    if (mavBridge.readMavlinkMsg(msg)) {
        //received one mavlink message
        handleMavlinkMsg(msg);
    }
    //give up some time to Particle housekeeping
    Particle.process();
}