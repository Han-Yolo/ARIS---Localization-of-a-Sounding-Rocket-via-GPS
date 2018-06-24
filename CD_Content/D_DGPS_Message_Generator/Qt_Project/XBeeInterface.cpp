#include "XBeeInterface.h"

#include "RTCMEncode.h"
#include "Debug.h"

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <QDateTime>
#include <QDir>


void XBeeInterface::run() try
{  
#if BYPASS_XBEE
    initSerial();
#endif

#if LOG_SERIAL_USER
    if(!QDir("Log").exists()) QDir().mkdir("Log");
    QDateTime dateTime = QDateTime::currentDateTime();
    log_file.setFileName("Log/serial_user" + dateTime.toString("__dd_MM_yyyy__hh_mm") + ".ubx");
    if(log_file.open(QIODevice::WriteOnly))
    {
        log.setDevice(&log_file);
    }
#endif

    emit printXBeeStatus("running...");

    for(;;)
    {
        std::vector<unsigned char> data;

#if LOG_SERIAL_REF
        char buf;
        if(read(serial, &buf, 1) == 1)
        {
            log.writeRawData(&buf, 1);
        }
#endif

        if(!msg1.empty())
        {
            RTCM_msg1 msg = msg1.pop();
            data = encodeMsg1(msg);

        }
        else if(!msg3.empty())
        {
            RTCM_msg3 msg = msg3.pop();
            data = encodeMsg3(msg);
        }
        else
        {
            msleep(1);
            continue;
        }

#if BYPASS_XBEE
        writeToUblox(data);
#endif
    }
}
catch(QString e)
{

#if LOG_SERIAL_REF
    log_file.close();
#endif

    emit printXBeeError(e);
}

void XBeeInterface::writeToUblox(std::vector<unsigned char> data)
{
    for(size_t i = 0; i < data.size(); i++)
    {
        write(serial, &data[i], 1);

#if DEBUG_SERIAL_OUT
        std::cout << std::hex << (int)data[i];
#endif

    }

#if DEBUG_SERIAL_OUT
    std::cout << std::dec << "\t Bytes: " << data.size() << std::endl;
#endif

}

void XBeeInterface::initSerial(void)
{
    serial = open(port.toStdString().c_str(), O_RDWR | O_NOCTTY);

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    /* Error Handling */
    if(tcgetattr(serial, &tty) != 0) {
        throw ("could not open serial port: " + port);
    }

    /* Baud Rate */
    cfsetspeed(&tty, (speed_t)baud);

    /* Read blocking and timeout settings */
    tty.c_cc[VMIN]   =  0;                  // read block until 1 byte is available
    tty.c_cc[VTIME]  =  0;                  // no timeout

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then apply attributes */
    tcflush(serial, TCIFLUSH);
    if(tcsetattr(serial, TCSANOW, &tty) != 0) {
        throw ("could not open serial port: " + port);
    }
}
