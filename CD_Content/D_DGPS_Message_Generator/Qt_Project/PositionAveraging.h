#ifndef POSITIONAVERAGING_H
#define POSITIONAVERAGING_H

#include <QThread>
#include <vector>

#include "GPSDataTypes.h"


class PositionAveraging : public QThread
{
    Q_OBJECT

private:

    QString port;
    unsigned int baud;
    std::vector<WGS84> posBuffer;
    bool interruptionRequested = false;

    void run(void);

public:

    PositionAveraging(QString port, unsigned int baud) : port(port), baud(baud) {}

    WGS84 getAvgPosition(void);
    void requestInterruption(void) { interruptionRequested = true; }

signals:

    void printStatus(QString status);
    void printError(QString status);
    void endAll(void);

public slots:

    void printUbloxStatus(QString message);
    void printUbloxError(QString message);

};

#endif // POSITIONAVERAGING_H
