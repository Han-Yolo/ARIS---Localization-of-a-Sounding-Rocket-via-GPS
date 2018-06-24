#ifndef XBEEINTERFACE_H
#define XBEEINTERFACE_H


#include <QThread>
#include <QFile>

#include "ThreadQueue.h"
#include "GPSDataTypes.h"


class XBeeInterface : public QThread
{
    Q_OBJECT

private:

    ThreadQueue<RTCM_msg1> msg1;
    ThreadQueue<RTCM_msg3> msg3;

    QString port;
    unsigned int baud;
    int serial;

    QFile log_file;
    QDataStream log;

    void initSerial();
    void writeToUblox(std::vector<unsigned char> data);
    void run(void);

public:

    XBeeInterface(QString port = "", unsigned int baud = 0) : port(port), baud(baud) {}

    void sendRTCM_msg1(RTCM_msg1 msg) { msg1.push(msg); }
    void sendRTCM_msg3(RTCM_msg3 msg) { msg3.push(msg); }

signals:

    void printXBeeStatus(QString message);
    void printXBeeError(QString message);

};

#endif // XBEEINTERFACE_H
