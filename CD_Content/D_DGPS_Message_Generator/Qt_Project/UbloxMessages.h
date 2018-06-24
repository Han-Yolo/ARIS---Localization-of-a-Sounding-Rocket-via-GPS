/*
 * UbloxMessages.h
 *
 *  Created on: Mar 20, 2018
 *      Author: Simon Herzog
 */

#ifndef UBLOXMESSAGES_H_
#define UBLOXMESSAGES_H_


#include <vector>


enum UBX_MESSAGE
{
	UBX_RXM_RAWX		= 0x0215,
	UBX_RXM_SFRBX		= 0x0213,
    UBX_NAV_CLOCK       = 0x0122,
	UBX_NAV_POSECEF		= 0x0101
};

enum UBX_GNSS
{
    UBX_GPS             = 0,
    UBX_SBAS            = 1,
    UBX_GALILEO         = 2,
    UBX_BEIDOU          = 3,
    UBX_IMES            = 4,
    UBX_QZSS            = 5,
    UBX_GLONASS         = 6
};


struct RXM_RAWX_repeated
{
	double prMes;				// [m] Pseudorange measurement
	double cpMes;				// [cycles] Carrier phase measurement
	double doMes;				// [Hz] Doppler measurement (positive sign for approaching satellites)
	unsigned int gnssId;		// [-] GNSS identifier
	unsigned int svId;			// [-] Satellite identifier
	unsigned int freqId;		// [-] Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13)
	unsigned int locktime;		// [ms] Carrier phase locktime counter (maximum	64500ms)
	unsigned int cno;			// [dBHz] Carrier-to-noise density ratio (signal strength)
	unsigned int prStdev;		// [m] Estimated pseudorange measurement standard deviation
	unsigned int cpStdev;		// [cycles] Estimated carrier phase measurement standard deviation
	unsigned int doStdev;		// [Hz] Estimated Doppler measurement standard deviation
	unsigned int trkStat;		// [-] Tracking status bitfield
};

struct RXM_RAWX
{
	double rcvTow;				// [s] Measurement time of week in receiver local time approximately aligned to the GPS time system
	unsigned int week;			// [weeks] GPS week number in receiver local time
	int leapS;					// [s] GPS leap seconds (GPS-UTC)
	unsigned int numMeas;		// [-] Number of measurements to follow
	unsigned int recStat;		// [-] Receiver tracking status bitfield
	std::vector<RXM_RAWX_repeated> repeated;	// repeated block (numMeas times)
};

struct RXM_SFRBX
{
	unsigned int gnssId;		// [-] GNSS identifier
	unsigned int svId;			// [-] Satellite identifier
	unsigned int freqId;		// [-] Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13)
	unsigned int numWords;		// [-] The number of data words contained in this message (0..16)
	unsigned int version;		// [-] Message version
	std::vector<unsigned int> dwrd;		// [-] The data words
};

struct NAV_CLOCK
{
    unsigned int iTOW;          // [ms] GPS time of week of the navigation epoch
    int clkB;                   // [ns] Clock bias
    int clkD;                   // [ns/s] Clock drift
    unsigned int tAcc;          // [ns] Time accuracy estimate
    unsigned int fAcc;          // [ps/s] Frequency accuracy estimate
};

struct NAV_POSECEF
{
	unsigned int iTOW;			// [ms] GPS time of week of the navigation epoch
	int ecefX;					// [cm] ECEF X coordinate
	int ecefY;					// [cm] ECEF Y coordinate
	int ecefZ;					// [cm] ECEF Z coordinate
	unsigned int pAcc;			// [cm] Position Accuracy Estimate
};


#endif /* UBLOXMESSAGES_H_ */
