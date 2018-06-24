#ifndef GPSDATATYPES_H
#define GPSDATATYPES_H


#include <vector>

#include <QDataStream>


struct WGS84
{
    double x = 0, y = 0, z = 0;    // [m] Earth centered, earth fixed coordinates following the WGS84 standard
};

inline QDataStream &operator>>(QDataStream &in, WGS84 &p)
{
    in >> p.x >> p.y >> p.z;
    return in;
}

inline QDataStream &operator<<(QDataStream &out, const WGS84 &p)
{
    out << p.x << p.y << p.z;
    return out;
}


struct EphemerisData
{
    unsigned int Week_No;       // [week] GPS week modulo 1024
    unsigned int SV_accuracy;   // [-] 4bit, URA index
    unsigned int SV_health;     // [-] 6bit, MSB: 0 = all NAV data are OK
    double T_GD;                // [s] Estimated Group Delay Differential
    double t_oc;                // [s] Reference Time Clock
    double a_f2;                // [s/s²] SV clock correction term 2
    double a_f1;                // [s/s] SV clock correction term 1
    double a_f0;                // [s] SV clock correction term 0

    double M_0;                 // [rad] Mean Anomaly at Reference Time
    double delta_n;             // [rad/s] Mean Motion Difference From Computed Value
    double e;                   // [-] Eccentricity
    double sqrt_A;              // [sqrt(m)] Square Root of the Semi-Major Axis
    double Omega_0;             // [rad] Longitude of Ascending Node of Orbit Plane at Weekly Epoch
    double i_0;                 // [rad] Inclination Angle at Reference Time
    double omega;               // [rad] Argument of Perigee
    double Omega_dot;           // [rad/s] Rate of Right Ascension
    double i_dot;               // [rad/s] Rate of Inclination Angle
    double C_uc;                // [rad] Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude
    double C_us;                // [rad] Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude
    double C_rc;                // [rad] Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius
    double C_rs;                // [rad] Amplitude of the Sine Harmonic Correction Term to the Orbit Radius
    double C_ic;                // [rad] Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination
    double C_is;                // [rad] Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination
    double t_oe;                // [s] Reference Time Ephemeris

    int IODC = -1;              // Issue of Data (Clock) from subframe 1
    int IODE_sub2 = -1;         // Issue of Data (Ephemeris) from subframe 2
    int IODE_sub3 = -1;         // Issue of Data (Ephemeris) from subframe 3
    bool valid = false;         // validity of this struct
};


/* ----- RTCM ----- */

struct CorrectionData
{
    unsigned int satID;         // satellite ID the data is valid for
    double PRC;                 // [m] Pseudo Range Correction
    double RRC;                 // [m/s] Range Rate Correction
    unsigned int IOD;           // [-] Issue Of Data
};

struct RTCM_msg1
{
    double time;                // [s] time of measurement
    std::vector<CorrectionData> data;
};

struct RTCM_msg3
{
    double time;                // [s] time of measurement
    WGS84 pos;
};

#endif // GPSDATATYPES_H
