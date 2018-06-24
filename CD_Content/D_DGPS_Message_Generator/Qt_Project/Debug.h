#ifndef DEBUGFUNC_H
#define DEBUGFUNC_H


#include <iostream>
#include <iomanip>

#include "GPSDataTypes.h"


#define DEBUG_SATPOS        (0)
#define DEBUG_DELTA_REC     (0)
#define DEBUG_PRC           (0)
#define DEBUG_RTCM          (1)
#define DEBUG_SERIAL_OUT    (0)

#define LOG_PRC             (1)
#define LOG_SERIAL_REF      (1)
#define LOG_SERIAL_USER     (1 & BYPASS_XBEE)

#define BYPASS_XBEE         (1)     // enable to send RTCM messages to output serial port

void WGStoLatLonHeight(WGS84 in, double& lat, double& lon, double& height);


#endif // DEBUGFUNC_H
