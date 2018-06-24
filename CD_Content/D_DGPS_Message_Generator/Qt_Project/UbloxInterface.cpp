/*
 * UbloxInterface.cpp
 *
 *  Created on: Mar 20, 2018
 *      Author: Simon Herzog
 */

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <QDateTime>

#include "UbloxInterface.h"
#include "TypeConversion.h"
#include "Debug.h"

/*
 * searches the serial interface for the UBX messages specified in UbloxMessages.h
 * if a message is received correctly, a message struct is generated and pushed into the respective queue
 * if a problem occurs or a message is not needed, the endless loop starts over without generating a message struct
 */
void UbloxInterface::run(void) try
{
	initSerial();
	initUbloxModule();

    emit printUbloxStatus("running...");

	unsigned char buf;
	unsigned char header[4];
	unsigned char checksum[2];
	UBX_MESSAGE message;

#if LOG_SERIAL_REF
    if(!QDir("Log").exists()) QDir().mkdir("Log");
    QDateTime dateTime = QDateTime::currentDateTime();
    log_file.setFileName("Log/serial_ref" + dateTime.toString("__dd_MM_yyyy__hh_mm") + ".ubx");
    if(log_file.open(QIODevice::WriteOnly))
    {
        log.setDevice(&log_file);
    }
#endif

    for(;;)
	{
        /* read until mu(Hex B5) appears */
        if(!readSerial(&buf, 1)) continue;
        if(buf != 0xb5) continue;

        /* check if b(Hex 62) is next */
        if(!readSerial(&buf, 1)) continue;
        if(buf != 0x62) continue;

        /* read class, ID and length */
        if(!readSerial(header, 4)) continue;

        /* check if this message is needed */
        message = static_cast<UBX_MESSAGE>(array_to_uint(header, 2, 0, false));	// class and ID are byte 1 and 2 of header
        if(
            message != UBX_RXM_RAWX &&
            message != UBX_RXM_SFRBX &&
            message != UBX_NAV_CLOCK &&
            message != UBX_NAV_POSECEF
        ) continue;

        /* define payload buffer */
        unsigned int length = array_to_uint(header, 2, 2, true);	// length is byte 3 and 4 of header in little-endian format
        unsigned char payload[length];

        /* read payload */
        if(!readSerial(payload, length)) continue;

        /* read checksum */
        if(!readSerial(checksum, 2)) continue;

        /* test checksum */
        unsigned char CK_A = 0, CK_B = 0;

        for(unsigned int i = 0; i < 4; i++)
        {
            CK_A = CK_A + header[i];
            CK_B = CK_B + CK_A;
        }

        for(unsigned int i = 0; i < length; i++)
        {
            CK_A = CK_A + payload[i];
            CK_B = CK_B + CK_A;
        }

        if((CK_A != checksum[0]) || (CK_B != checksum[1])) continue;

        /* generate message struct and push it into the respective queue */
        generateMessage(message, payload);
	}
}
catch(QString e)
{

#if LOG_SERIAL_REF
    log_file.close();
#endif

    emit printUbloxError(e);
}

bool UbloxInterface::readSerial(unsigned char *buffer, int length)
{
	for(int i = 0; i < length; i++)
	{
        if(read(serial, buffer + i, 1) != 1)
        {
            throw QString::fromStdString("could not read serial port");
        }
	}

#if LOG_SERIAL_REF
    log.writeRawData(reinterpret_cast<const char*>(buffer), length);
#endif

	return true;
}

void UbloxInterface::generateMessage(UBX_MESSAGE type, unsigned char *payload)
{
	if(type == UBX_RXM_RAWX)
	{
		/* generate header of message */
		RXM_RAWX RAWX;
		RAWX.rcvTow = array_to_double(payload, 0, true);
		RAWX.week = array_to_uint(payload, 2, 8, true);
		RAWX.leapS = array_to_int(payload, 1, 10, true);
		RAWX.numMeas = array_to_uint(payload, 1, 11, true);
		RAWX.recStat = array_to_uint(payload, 1, 12, true);

		/* generate repeated part of message */
		for(unsigned int N = 0; N < RAWX.numMeas; N++)
		{
			RXM_RAWX_repeated RAWX_rep;
			RAWX_rep.prMes 		= array_to_double(payload, 16 + 32*N, true);
			RAWX_rep.cpMes 		= array_to_double(payload, 24 + 32*N, true);
			RAWX_rep.doMes 		= array_to_double(payload, 32 + 32*N, false);
			RAWX_rep.gnssId 	= array_to_uint(payload, 1, 36 + 32*N, true);
			RAWX_rep.svId 		= array_to_uint(payload, 1, 37 + 32*N, true);
			RAWX_rep.freqId 	= array_to_uint(payload, 1, 39 + 32*N, true);
			RAWX_rep.locktime 	= array_to_uint(payload, 2, 40 + 32*N, true);
			RAWX_rep.cno		= array_to_uint(payload, 1, 42 + 32*N, true);
			RAWX_rep.prStdev	= array_to_uint(payload, 1, 43 + 32*N, true);
			RAWX_rep.cpStdev	= array_to_uint(payload, 1, 44 + 32*N, true);
			RAWX_rep.doStdev	= array_to_uint(payload, 1, 45 + 32*N, true);
			RAWX_rep.trkStat	= array_to_uint(payload, 1, 46 + 32*N, true);
			RAWX.repeated.push_back(RAWX_rep);
		}

		/* push message to queue */
		RXM_RAWX_queue.push(RAWX);
	}


	else if(type == UBX_RXM_SFRBX)
	{
		/* generate header of message */
		RXM_SFRBX SFRBX;
		SFRBX.gnssId 			= array_to_uint(payload, 1, 0, true);
		SFRBX.svId				= array_to_uint(payload, 1, 1, true);
		SFRBX.freqId			= array_to_uint(payload, 1, 3, true);
		SFRBX.numWords			= array_to_uint(payload, 1, 4, true);
		SFRBX.version			= array_to_uint(payload, 1, 6, true);

		/* generate repeated part of message */
        SFRBX.dwrd.push_back(0);
		for(unsigned int N = 0; N < SFRBX.numWords; N++)
		{
			SFRBX.dwrd.push_back(array_to_uint(payload, 4, 8 + 4*N, true));
		}

		/* push message to queue */
		RXM_SFRBX_queue.push(SFRBX);
	}

    else if(type == UBX_NAV_CLOCK)
    {
        /* generate message */
        NAV_CLOCK CLOCK;
        CLOCK.iTOW              = array_to_uint(payload, 4, 0, true);
        CLOCK.clkB              = array_to_int(payload, 4, 4, true);
        CLOCK.clkD              = array_to_int(payload, 4, 8, true);
        CLOCK.tAcc              = array_to_uint(payload, 4, 12, true);
        CLOCK.fAcc              = array_to_uint(payload, 4, 16, true);

        /* push message to queue */
        NAV_CLOCK_queue.push(CLOCK);
    }

	else if(type == UBX_NAV_POSECEF)
	{
		/* generate message */
		NAV_POSECEF POSECEF;
		POSECEF.iTOW			= array_to_uint(payload, 4, 0, true);
		POSECEF.ecefX			= array_to_int(payload, 4, 4, true);
		POSECEF.ecefY			= array_to_int(payload, 4, 8, true);
		POSECEF.ecefZ			= array_to_int(payload, 4, 12, true);
		POSECEF.pAcc			= array_to_uint(payload, 4, 16, true);

		/* push message to queue */
		NAV_POSECEF_queue.push(POSECEF);
	}
}

void UbloxInterface::initUbloxModule(void)
{
    /* enable only GPS */
    unsigned char CFG_GNSS[] = {
        0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x14, 0x20, 0x00, 0x01, 0x00,
        0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x01, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x01, 0x01, 0xFF, 0x7E
    };
    write(serial, CFG_GNSS, sizeof(CFG_GNSS));

    /* set stationary mode */
    unsigned char CFG_NAV5[] = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
        0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x71
    };
    write(serial, CFG_NAV5, sizeof(CFG_NAV5));

    /* set UBX_RXM_RAWX message to 1Hz */
    unsigned char CFG_MSG_RAWX[] = {
        0xB5, 0x62,         // header
        0x06, 0x01,         // class / ID
        0x08, 0x00,         // length
        0x02, 0x15, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x28,
        0x4E, 0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0x02, 0x15,   // payload
        0x20, 0x4A          // checksum
    };
    write(serial, CFG_MSG_RAWX, sizeof(CFG_MSG_RAWX));

    /* set UBX_RXM_SFRBX message to 1Hz */
    unsigned char CFG_MSG_SFRBX[] = {
        0xB5, 0x62,         // header
        0x06, 0x01,         // class / ID
        0x08, 0x00,         // length
        0x02, 0x13, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x26,
        0x40, 0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0x02, 0x13,  // payload
        0x1E, 0x48          // checksum
    };
    write(serial, CFG_MSG_SFRBX, sizeof(CFG_MSG_SFRBX));

    /* set UBX_NAV_CLOCK message to 1Hz */
    unsigned char CFG_NAV_CLOCK[] = {
        0xB5, 0x62,         // header
        0x06, 0x01,         // class / ID
        0x08, 0x00,         // length
        0x01, 0x22, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x34,
        0xA1, 0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0x01, 0x22,  // payload
        0x2C, 0x55          // checksum
    };
    write(serial, CFG_NAV_CLOCK, sizeof(CFG_NAV_CLOCK));

    /* set UBX_NAV_POSECEF message to 1Hz */
    unsigned char CFG_NAV_POSECEF[] = {
        0xB5, 0x62,         // header
        0x06, 0x01,         // class / ID
        0x08, 0x00,         // length
        0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x13,
        0xBA, 0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0x01, 0x01,   // payload
        0x0B, 0x34          // checksum
    };
    write(serial, CFG_NAV_POSECEF, sizeof(CFG_NAV_POSECEF));
}

void UbloxInterface::initSerial(void)
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
    tty.c_cc[VMIN]   =  1;                  // read block until 1 byte is available
    tty.c_cc[VTIME]  =  0;                  // no timeout

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then apply attributes */
    tcflush(serial, TCIFLUSH);
    if(tcsetattr(serial, TCSANOW, &tty) != 0) {
        throw ("could not open serial port: " + port);
    }
}

