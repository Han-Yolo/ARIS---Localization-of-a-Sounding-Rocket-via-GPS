#ifndef NAVMESSAGEDECODER_H
#define NAVMESSAGEDECODER_H

#include <QThread>
#include <vector>

#include "UbloxInterface.h"
#include "GPSDataTypes.h"


class NavMessageDecoder : public QThread
{
    Q_OBJECT

private:

    const double pi = 3.1415926535898;  // semi-circle to radian (IS-GPS)

    UbloxInterface *ublox;
    std::vector<EphemerisData> eph;

    double readValue8(unsigned int word, bool twoComp, unsigned int shift, double power);
    double readValue16(unsigned int word, bool twoComp, unsigned int shift, double power);
    double readValue32(unsigned int word1, unsigned int word2, bool twoComp, double power);
    double readA_f0(unsigned int word);
    double readOmegaDot(unsigned int word);
    double readIDot(unsigned int word);

    bool checkParity(std::vector<unsigned int> &dwrd);
    unsigned int exor(unsigned int value);

    void run(void);

public:

    NavMessageDecoder(UbloxInterface *ublox) : ublox(ublox) { eph.resize(33); }
    virtual ~NavMessageDecoder() {}

    std::vector<EphemerisData> getEphemeris(void) { return eph; }

signals:

    void printNavMesStatus(QString message);
    void printNavMesError(QString message);

};

#endif // NAVMESSAGEDECODER_H
