/****************************************************************************************************************************
  Async_UdpServer.ino

  For Teensy41 with QNEthernet

  AsyncUDP_Teensy41 is a Async UDP library for the Teensy41 using built-in Ethernet and QNEThernet

  Based on and modified from ESPAsyncUDP Library (https://github.com/me-no-dev/ESPAsyncUDP)
  Built by Khoi Hoang https://github.com/khoih-prog/AsyncUDP_Teensy41
 *****************************************************************************************************************************/

#include "defines.h"

#define ASYNC_UDP_TEENSY41_VERSION_MIN_TARGET      "AsyncUDP_Teensy41 v1.2.1"
#define ASYNC_UDP_TEENSY41_VERSION_MIN             1002001

#include <AsyncUDP_Teensy41.h>        // https://github.com/khoih-prog/AsyncUDP_Teensy41

#include <Ticker.h>                   // https://github.com/sstaub/Ticker
#include "c_library_v2_master/ardupilotmega/mavlink.h"

AsyncUDP udp;

// 600s = 10 minutes to not flooding, 60s in testing
#define UDP_REQUEST_INTERVAL_MS     60000  //600000

#define BUFFER_LENGTH 2041 
uint8_t buf[BUFFER_LENGTH];




  // Set the static IP address to use if the DHCP fails to assign
IPAddress myIP(192, 168, 0, 221);
IPAddress myNetmask(255, 255, 255, 0);
IPAddress myGW(192, 168, 0, 1);
IPAddress mydnsServer(8, 8, 8, 8);


//void sendRequest();

// Repeat forever, millis() resolution
//Ticker sendUDPRequest(sendRequest, UDP_REQUEST_INTERVAL_MS, 0, MILLIS);
//
//void sendRequest()
//{
//    UDP_LOGDEBUG("Send broadcast");
//
//    udp.broadcast("Anyone here?");
//}

void setup()
{
    Serial.begin(115200);
    pinMode(33, INPUT);
    pinMode(34, INPUT);
    pinMode(13, OUTPUT);

    digitalWrite(13, HIGH);
    delay(500);

    debug("\nStart AsyncUDPServer on "); debug(BOARD_NAME);
    debug("\n");
    debug(ASYNC_UDP_TEENSY41_VERSION);

    delay(500);
    digitalWrite(13, LOW);

    
#if USING_DHCP
    // Start the Ethernet connection, using DHCP
    debug("\nInitialize Ethernet using DHCP => ");
    Ethernet.begin();
#else
    // Start the Ethernet connection, using static IP
    debug("\nInitialize Ethernet using static IP => ");
    Ethernet.begin(myIP, myNetmask, myGW);
    Ethernet.setDNSServerIP(mydnsServer);
#endif

    if (!Ethernet.waitForLocalIP(5000))
    {
        debug("\nFailed to configure Ethernet");

        if (!Ethernet.linkStatus())
        {
            debug("\nEthernet cable is not connected.");
        }

        // Stay here forever
        while (true)
        {
            delay(1);
        }
    }
    else
    {
        debug("Connected! IP address:"); Serial.println(Ethernet.localIP());
    }

#if USING_DHCP
    delay(1000);
#else  
    delay(2000);
#endif

    if (udp.listen(14550))
    {
        Serial.print("UDP Listening on IP: ");
        Serial.println(Ethernet.localIP());

        udp.onPacket([](AsyncUDPPacket packet)
            {
                mavlink_message_t msg;
                mavlink_status_t status;
                int i;
                uint8_t* d;
                d = packet.data();
                char temp;
                for (i = 0; i < packet.length(); ++i)
                {
                    temp = packet.data()[i];
                    //Serial.print((unsigned char)temp);
                    if (mavlink_parse_char(MAVLINK_COMM_0, packet.data()[i], &msg, &status))
                    {
                        digitalWrite(13, HIGH);
                        // Packet received
                        if ((msg.msgid == MAVLINK_MSG_ID_HEARTBEAT))
                        {
                            debug("HeartBeat received\r\n");
                            
                            if ((digitalRead(33) == 0) && (digitalRead(34) == 1))
                            {
                                // if RTL swith is on
                                uint8_t sysid = msg.sysid;
                                    uint8_t compid = msg.compid;
                                    //mavlink_message_t msg;
                                    uint8_t buf1[MAVLINK_MAX_PACKET_LEN];
                                    mavlink_msg_command_long_pack(1, 144, &msg, sysid, compid, MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0);
                                    uint16_t len = mavlink_msg_to_send_buffer(buf1, &msg);
                                    udp.writeTo(buf1, len, packet.remoteIP(), packet.remotePort());
                                    debug("RTL Sent\r\n");
                            }
                            else
                            {
                                digitalWrite(13, LOW);

                            }
                        }
                    }
                }



            });
    }

    //sendRequest();

    //sendUDPRequest.start(); //start the ticker
}

void loop()
{
    //sendUDPRequest.update();
}


void heartbeat_out(void)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(1, 144, &msg, 0, MAV_AUTOPILOT_INVALID,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // base_mode
        0,                                 // custom_mode
        MAV_STATE_ACTIVE);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    udp.write(buf, len);
}


void debug(const char* format, ...) {
    char buf[128];
    va_list args;
    va_start(args, format);
    vsniprintf(buf, sizeof(buf), format, args); // does not overrun sizeof(buf) including null terminator
    va_end(args);
    // the below assumes that the new data will fit into the I/O buffer. If not, Serial may drop it.
    // if Serial had a get free buffer count, we could delay and retry. Such does exist at the device class level, but not at this level.
    Serial.write(buf); // move chars to I/O buffer, freeing up local buf
}
