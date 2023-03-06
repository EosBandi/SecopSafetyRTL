/****************************************************************************************************************************
  defines.h

  For Teensy41 with QNEthernet

  AsyncUDP_Teensy41 is a Async UDP library for the Teensy41 using built-in Ethernet and QNEThernet

  Based on and modified from ESPAsyncUDP Library (https://github.com/me-no-dev/ESPAsyncUDP)
  Built by Khoi Hoang https://github.com/khoih-prog/AsyncUDP_Teensy41
 *****************************************************************************************************************************/

#ifndef defines_h
#define defines_h

#if !( defined(CORE_TEENSY) && defined(__IMXRT1062__) && defined(ARDUINO_TEENSY41) )
 //#error Only Teensy 4.1 supported
#endif

#define ASYNC_UDP_TEENSY41_DEBUG_PORT       Serial

// Debug Level from 0 to 4
#define _ASYNC_UDP_TEENSY41_LOGLEVEL_       0

#define SHIELD_TYPE     "Teensy4.1 QNEthernet"

#if (_ASYNC_UDP_TEENSY41_LOGLEVEL_ > 3)
#warning Using QNEthernet lib for Teensy 4.1.Must also use Teensy Packages Patch or error
#endif

//#define USING_DHCP            true
#define USING_DHCP            false


#include "QNEthernet.h"       // https://github.com/ssilverman/QNEthernet
using namespace qindesign::network;

#endif    //defines_h
