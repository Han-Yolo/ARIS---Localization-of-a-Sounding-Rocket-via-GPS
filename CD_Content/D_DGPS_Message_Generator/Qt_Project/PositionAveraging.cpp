#include "PositionAveraging.h"

#include "UbloxInterface.h"

/*
 * pushes every NAV_POSECEF message from the ublox interface to the posBuffer
 */
void PositionAveraging::run(void)
{
    /* start ublox interface */
    UbloxInterface ublox(port, baud);
    connect(&ublox, SIGNAL(finished()), &ublox, SLOT(deleteLater()));
    connect(&ublox, SIGNAL(printUbloxStatus(QString)), this, SLOT(printUbloxStatus(QString)));
    connect(&ublox, SIGNAL(printUbloxError(QString)), this, SLOT(printUbloxError(QString)));
    ublox.start();

    while(!interruptionRequested)
    {
        if(ublox.NAV_POSECEF_empty())
        {
            if(!ublox.RXM_RAWX_empty()) ublox.RXM_RAWX_pop();
            if(!ublox.RXM_SFRBX_empty()) ublox.RXM_SFRBX_pop();
            if(!ublox.NAV_CLOCK_empty()) ublox.NAV_CLOCK_pop();
            msleep(1);
            continue;
        }

        NAV_POSECEF ecef = ublox.NAV_POSECEF_pop();

        WGS84 wgs;
        wgs.x = static_cast<double>(ecef.ecefX)/100;
        wgs.y = static_cast<double>(ecef.ecefY)/100;
        wgs.z = static_cast<double>(ecef.ecefZ)/100;
        posBuffer.push_back(wgs);
    }

    ublox.terminate();
    ublox.wait();
    //emit printStatus("u-blox interface closed");

    emit printStatus("PositionAveraging finished");
}

/*
 * calculate average from posBuffer
 */
WGS84 PositionAveraging::getAvgPosition(void)
{
    double xSum = 0, ySum = 0, zSum = 0;

    /* sum up posBuffer */
    size_t size = posBuffer.size();
    for(size_t i = 0; i < size; i++)
    {
        xSum += posBuffer[i].x;
        ySum += posBuffer[i].y;
        zSum += posBuffer[i].z;
    }

    /* calculate average */
    WGS84 average;
    average.x = xSum/size;
    average.y = ySum/size;
    average.z = zSum/size;

    return average;
}


/* ----- slots ----- */

void PositionAveraging::printUbloxStatus(QString message)
{
    emit printStatus("u-blox: " + message);
}

void PositionAveraging::printUbloxError(QString message)
{
    emit printError("u-blox: " + message);
    emit endAll();
}
