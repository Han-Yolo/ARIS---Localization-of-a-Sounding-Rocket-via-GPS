#include "DGPSMesGen.h"

#include <math.h>
#include <vector>
#include <QDateTime>

#include "XBeeInterface.h"
#include "NavMessageDecoder.h"
#include "Debug.h"

#define SEND_MSG3 (1)


struct LoopData
{
    unsigned int svId;
    EphemerisData eph;
    double rawPr;
    WGS84 satPos;
    double delta_t_sv = 0;
    double I_err = 0;
    double T_err = 0;
    double prc = 0;
    double prcFilt = 0;
};

void DGPSMesGen::run(void)
{
    /* start ublox interface */
    UbloxInterface ublox(inputPort, inputBaud);
    connect(&ublox, SIGNAL(finished()), &ublox, SLOT(deleteLater()));
    connect(&ublox, SIGNAL(printUbloxStatus(QString)), this, SLOT(printUbloxStatus(QString)));
    connect(&ublox, SIGNAL(printUbloxError(QString)), this, SLOT(printUbloxError(QString)));
    ublox.start();

    /* start XBee interface */
    XBeeInterface xbee(outputPort, outputBaud);
    connect(&xbee, SIGNAL(finished()), &xbee, SLOT(deleteLater()));
    connect(&xbee, SIGNAL(printXBeeStatus(QString)), this, SLOT(printXBeeStatus(QString)));
    connect(&xbee, SIGNAL(printXBeeError(QString)), this, SLOT(printXBeeError(QString)));
    xbee.start();

    /* start navigation message decoder */
    NavMessageDecoder navMesDec(&ublox);
    connect(&navMesDec, SIGNAL(finished()), &navMesDec, SLOT(deleteLater()));
    connect(&navMesDec, SIGNAL(printNavMesStatus(QString)), this, SLOT(printNavMesStatus(QString)));
    connect(&navMesDec, SIGNAL(printNavMesError(QString)), this, SLOT(printNavMesError(QString)));
    navMesDec.start();

#if LOG_PRC
    if(!QDir("Log").exists()) QDir().mkdir("Log");
    QDateTime dateTime = QDateTime::currentDateTime();
    QFile log_file("Log/PRC" + dateTime.toString("__dd_MM_yyyy__hh_mm") + ".csv");
    if(log_file.open(QIODevice::WriteOnly))
    {
        QTextStream out(&log_file);
        out << "TOW";
        for(int i = 1; i <= 32; i++)
        {
          out << ",Sat" << i;
          out << ",Sat_filt" << i;
        }
        out << "\n";
    }
#endif

    while(!interruptionRequested)
    {     
        if(ublox.RXM_RAWX_empty())
        {
            if(!ublox.NAV_CLOCK_empty()) ublox.NAV_CLOCK_pop();
            if(!ublox.NAV_POSECEF_empty()) ublox.NAV_POSECEF_pop();
            msleep(1);
            continue;
        }

        /* get UBX_RXM_RAWX message */
        RXM_RAWX rawx = ublox.RXM_RAWX_pop();
        if(rawx.rcvTow == 0) continue;

        /* creat vector for loop data */
        std::vector<LoopData> ldVec;
        double delta_t_rec = 0;

        /* create struct for RTCM message 1 */
        RTCM_msg1 corrVec;
        corrVec.time = rawx.rcvTow;

        /* save satellite data to loop data vector */
        for(unsigned int i = 0; i < rawx.numMeas; i++)
        {
            LoopData ld;

            /* get satellite ID */
            ld.svId = rawx.repeated[i].svId;

            /* check if pseudorange is valid */
            if((rawx.repeated[i].trkStat & 0x1) != 1) continue;
            ld.rawPr = rawx.repeated[i].prMes;

            /* check if ephemeris data is available and satellite is healthy */
            ld.eph = navMesDec.getEphemeris()[rawx.repeated[i].svId];
            if(!ld.eph.valid) continue;
            if(ld.eph.SV_health != 0) continue;

            ldVec.push_back(ld);
        }

        /* calculate pseudorange corrections iteratively */
        for(int k = 0; k < 3; k++)
        {
            /* iterate through all satellites with valid data */
            for(size_t i = 0; i < ldVec.size(); i++)
            {
                /* calculate time of transmission */
                double corrPr = ldVec[i].rawPr + ldVec[i].delta_t_sv*c - ldVec[i].I_err - ldVec[i].T_err;
                double t_sv = rawx.rcvTow - corrPr/c;

                /* calculate satellite position */
                double E_k;
                ldVec[i].satPos = calcSatPos(ldVec[i].eph, t_sv, corrPr, E_k);

                /* calculate distance between reference station and satellite */
                double calcRange = sqrt(pow(refPos.x - ldVec[i].satPos.x, 2) + pow(refPos.y - ldVec[i].satPos.y, 2) + pow(refPos.z - ldVec[i].satPos.z, 2));

                /* calculate satellite clock correction */
                double delta_t_r = F * ldVec[i].eph.e * ldVec[i].eph.sqrt_A * sin(E_k);
                ldVec[i].delta_t_sv = ldVec[i].eph.a_f0 + ldVec[i].eph.a_f1*(t_sv - ldVec[i].eph.t_oc) + ldVec[i].eph.a_f2*pow((t_sv - ldVec[i].eph.t_oc), 2) + delta_t_r;

                /* add clock corrections to raw pseudorange */
                double corrPr_clk = ldVec[i].rawPr + (ldVec[i].delta_t_sv + delta_t_rec)*c;

                /* calculate pseudorange correction */
                ldVec[i].prc = calcRange - corrPr_clk;
            }

            /* estimate receiver clock bias */
            double sum = 0;
            for(size_t i = 0; i < ldVec.size(); i++) sum += ldVec[i].prc;
            delta_t_rec += (sum / static_cast<double>(ldVec.size()))/c;

#if DEBUG_DELTA_REC
            std::cout << "receiver bias iteration " << k << ": " << delta_t_rec*c << "m" << std::endl;
#endif

        }

#if DEBUG_DELTA_REC
        std::cout << std::endl;
#endif

        /* clear old prc values */
        for(int i = 0; i < PRC_HIST_SIZE; i++) prcHist[i][prcHistIndex] = -3001;

        /* calculate prc mean values */
        for(unsigned int i = 0; i < ldVec.size(); i++)
        {
            /* add prc to history */
            prcHist[ldVec[i].svId][prcHistIndex] = ldVec[i].prc;

            /* calculate mean prc */
            int prcCount = 0;
            double prcSum = 0.0;
            for(int k = 0; k < PRC_HIST_SIZE; k++)
            {
                if(prcHist[ldVec[i].svId][k] > -3000)
                {
                    prcSum += prcHist[ldVec[i].svId][k];
                    prcCount++;
                }
            }

            if(prcCount > 0) ldVec[i].prcFilt = prcSum/prcCount;
            else ldVec[i].prcFilt = -3001;
        }

        /* increment ringbuffer index for prc history */
        prcHistIndex++;
        if(prcHistIndex >= PRC_HIST_SIZE) prcHistIndex = 0;

        /* push loop data to correction vector */
        for(size_t i = 0; i < ldVec.size(); i++)
        {
            CorrectionData corr;

            corr.satID = ldVec[i].svId;
            corr.PRC = ldVec[i].prcFilt;
            corr.RRC = 0.0;
            corr.IOD = ldVec[i].eph.IODE_sub2;
            corrVec.data.push_back(corr);

#if DEBUG_SATPOS
            double lat, lon, height;
            WGStoLatLonHeight(ldVec[i].satPos, lat, lon, height);
            std::cout << "Time: " << rawx.rcvTow << "\t Sat " << ldVec[i].svId << std::endl;
            std::cout << std::setprecision(10) << "lat:" << lat << "\t lon: " << lon << "\t height: " << height << std::endl;
            std::cout << "x:" << ldVec[i].satPos.x << "\t y: " << ldVec[i].satPos.y << "\t z: " << ldVec[i].satPos.z << std::endl << std::endl;
#endif

        }

        /* send RTCM message 1 if corrections were calculated */
        if(!corrVec.data.empty())
        {
            xbee.sendRTCM_msg1(corrVec);
            xbee.sendRTCM_msg1(corrVec);

#if DEBUG_PRC
            std::cout << "GPS ToW: " << corrVec.time << std::endl;
            for(size_t i = 0; i < corrVec.data.size(); i++)
            {
                std::cout << "Sat: " << corrVec.data[i].satID << "\t PRC: " << std::setprecision(9) << corrVec.data[i].PRC << "\t RRC: " << corrVec.data[i].RRC << "\t IOD: " << corrVec.data[i].IOD << std::endl;
            }
            std::cout << std::endl;
#endif

#if SEND_MSG3
            static int msg3cnt = 0;
            if(msg3cnt++ == 10)
            {
                RTCM_msg3 msg3;
                msg3.time = corrVec.time;
                msg3.pos = refPos;
                xbee.sendRTCM_msg3(msg3);
                msg3cnt = 0;
            }
#endif

        }

#if LOG_PRC
        if(log_file.isOpen())
        {
            double prc[33] = {-3001};
            double prcFilt[33] = {-3001};
            for(size_t i = 0; i < ldVec.size(); i++)
            {
                prc[ldVec[i].svId] = ldVec[i].prc;
                prcFilt[ldVec[i].svId] = ldVec[i].prcFilt;
            }

            QTextStream out(&log_file);
            out << rawx.rcvTow;

            for(unsigned int i = 1; i < sizeof(prc)/sizeof(double); i++)
            {
                out << ",";
                if(prc[i] > -3000)
                {
                    out << prc[i];
                }
                out << ",";
                if(prc[i] > -3000)
                {
                    out << prcFilt[i];
                }
            }

            out << "\n";
        }
#endif

    }

#if LOG_PRC
    log_file.close();
#endif

    ublox.terminate();
    ublox.wait();
    //emit printStatus("u-blox interface closed");

    xbee.terminate();
    xbee.wait();
    //emit printStatus("xbee interface closed");

    navMesDec.terminate();
    navMesDec.wait();
    //emit printStatus("navigation message decoder closed");

    emit printStatus("DGPSMesGen finished");
}

/*
 * calculates the satellite position at time t from the ephemeris parameters eph
 * according to Table 20-IV in document IS-GPS-200D
 *
 * returns WGS84 position and eccentric anomaly for relativistic satellite clock correction
 */
WGS84 DGPSMesGen::calcSatPos(EphemerisData eph, double t, double range, double &E_k)
{
    /* time from ephemeris reference epoch */
    double t_k = t - eph.t_oe;

    if(t_k > 302400.0) t_k -= 604800.0;
    if(t_k < -302400.0) t_k += 604800.0;

    /* semi-major axis */
    double A = eph.sqrt_A*eph.sqrt_A;

    /* computed mean motion */
    double n_0 = sqrt(mu/pow(A, 3));

    /* corrected mean motion */
    double n = n_0 + eph.delta_n;

    /* mean anomaly */
    double M_k = eph.M_0 + n*t_k;

    /* solve Kepler's equation using Newton's method */
    E_k = M_k;
    double error = E_k - eph.e*sin(E_k) - M_k;

    int i = 0;
    while((fabs(error) > 1.0e-12) && (i < 50))
    {
        E_k = E_k - error/(1-eph.e*cos(E_k));
        error = E_k - eph.e*sin(E_k) - M_k;
        i++;
    }

    /* true anomaly */
    double v_k = atan2( (sqrt(1-pow(eph.e,2)) * sin(E_k)) , (cos(E_k) - eph.e) );

    /* argument of latitude */
    double Phi_k = v_k + eph.omega;

    /* second harmonic perturbations */
    double delta_u_k = eph.C_us*sin(2*Phi_k) + eph.C_uc*cos(2*Phi_k);
    double delta_r_k = eph.C_rs*sin(2*Phi_k) + eph.C_rc*cos(2*Phi_k);
    double delta_i_k = eph.C_is*sin(2*Phi_k) + eph.C_ic*cos(2*Phi_k);

    /* corrected argument of latitude */
    double u_k = Phi_k + delta_u_k;

    /* corrected radius */
    double r_k = A*(1-eph.e*cos(E_k)) + delta_r_k;

    /* corrected inclination */
    double i_k = eph.i_0 + delta_i_k + eph.i_dot*t_k;

    /* positions in orbital plane */
    double x_k_prime = r_k*cos(u_k);
    double y_k_prime = r_k*sin(u_k);

    /* corrected longitude of ascending node */
    double Omega_k = eph.Omega_0 + (eph.Omega_dot - Omega_e_dot)*t_k - Omega_e_dot*eph.t_oe - Omega_e_dot*range/c;

    /* earth-fixed coordinates */
    WGS84 satPos;
    satPos.x = x_k_prime*cos(Omega_k) - y_k_prime*cos(i_k)*sin(Omega_k);
    satPos.y = x_k_prime*sin(Omega_k) + y_k_prime*cos(i_k)*cos(Omega_k);
    satPos.z = y_k_prime*sin(i_k);

    return satPos;
}


/* ----- slots ----- */

void DGPSMesGen::printUbloxStatus(QString message)
{
    emit printStatus("u-blox: " + message);
}

void DGPSMesGen::printUbloxError(QString message)
{
    emit printError("u-blox: " + message);
    emit endAll();
}

void DGPSMesGen::printXBeeStatus(QString message)
{
    emit printStatus("XBee: " + message);
}

void DGPSMesGen::printXBeeError(QString message)
{
    emit printError("XBee: " + message);
    emit endAll();
}

void DGPSMesGen::printNavMesStatus(QString message)
{
    emit printStatus("NavMesDec: " + message);
}

void DGPSMesGen::printNavMesError(QString message)
{
    emit printError("NavMesDec: " + message);
    emit endAll();
}

