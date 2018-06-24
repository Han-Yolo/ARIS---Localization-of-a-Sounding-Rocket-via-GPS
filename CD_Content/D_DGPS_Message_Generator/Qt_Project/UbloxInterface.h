/*
 * UbloxInterface.h
 *
 *  Created on: Mar 20, 2018
 *      Author: Simon Herzog
 */

#ifndef UBLOXINTERFACE_H_
#define UBLOXINTERFACE_H_


#include <vector>
#include <QtCore>

#include "ThreadQueue.h"
#include "UbloxMessages.h"


class UbloxInterface : public QThread
{
    Q_OBJECT

private:

	ThreadQueue<RXM_RAWX> RXM_RAWX_queue;
	ThreadQueue<RXM_SFRBX> RXM_SFRBX_queue;
    ThreadQueue<NAV_CLOCK> NAV_CLOCK_queue;
	ThreadQueue<NAV_POSECEF> NAV_POSECEF_queue;

    QString port;
    unsigned int baud;
	int serial;

    QFile log_file;
    QDataStream log;

	void initSerial(void);
	void initUbloxModule(void);
    bool readSerial(unsigned char *buffer, int length);
    void generateMessage(UBX_MESSAGE type, unsigned char *payload);
    void run(void);

public:

    UbloxInterface(QString port, unsigned int baud) : port(port), baud(baud) {}
    virtual ~UbloxInterface() {}

    bool RXM_RAWX_empty(void) { return RXM_RAWX_queue.empty(); }
	bool RXM_SFRBX_empty(void) { return RXM_SFRBX_queue.empty(); }
    bool NAV_CLOCK_empty(void) { return NAV_CLOCK_queue.empty(); }
	bool NAV_POSECEF_empty(void) { return NAV_POSECEF_queue.empty(); }

	RXM_RAWX RXM_RAWX_pop(void) { return RXM_RAWX_queue.pop(); }
	RXM_SFRBX RXM_SFRBX_pop(void) { return RXM_SFRBX_queue.pop(); }
    NAV_CLOCK NAV_CLOCK_pop(void) { return NAV_CLOCK_queue.pop(); }
	NAV_POSECEF NAV_POSECEF_pop(void) { return NAV_POSECEF_queue.pop(); }

signals:

    void printUbloxStatus(QString message);
    void printUbloxError(QString message);

};


#endif /* UBLOXINTERFACE_H_ */
