#ifndef DGPSMESGEN_H
#define DGPSMESGEN_H

#include <QThread>

#include "GPSDataTypes.h"
#include "UbloxInterface.h"

#define PRC_HIST_SIZE (10)


class DGPSMesGen : public QThread
{
    Q_OBJECT

private:

    const double c = 299792458;                     // [m/s] speed of light
    const double mu = 3.986005e14;                  // [m³/s²] WGS 84 value of the earth's gravitational constant for GPS user
    const double Omega_e_dot = 7.2921151467e-5;     // [rad/s] WGS 84 value of the earth's rotation rate
    const double F = -4.442807633e-10;              // [s/sqrt(m)] -2*sqrt(mu)/c²

    WGS84 refPos;                                   // position of reference station
    QString inputPort;
    unsigned int inputBaud;
    QString outputPort;
    unsigned int outputBaud;

    int prcHistIndex = 0;
    double prcHist[33][PRC_HIST_SIZE] = {{0.0}};    // [satellite][delay_index]

    bool interruptionRequested = false;

    void run(void);

    WGS84 calcSatPos(EphemerisData eph, double t, double range, double &E_k);

public:

    DGPSMesGen(WGS84 refPos, QString inputPort, unsigned int inputBaud, QString outputPort = "", unsigned int outputBaud = 0)
        : refPos(refPos), inputPort(inputPort), inputBaud(inputBaud), outputPort(outputPort), outputBaud(outputBaud) {}

    void requestInterruption(void) { interruptionRequested = true; }

signals:

    void printStatus(QString status);
    void printError(QString status);
    void endAll(void);

public slots:

    void printUbloxStatus(QString message);
    void printUbloxError(QString message);

    void printNavMesStatus(QString message);
    void printNavMesError(QString message);

    void printXBeeStatus(QString message);
    void printXBeeError(QString message);
};

#endif // DGPSMESGEN_H
