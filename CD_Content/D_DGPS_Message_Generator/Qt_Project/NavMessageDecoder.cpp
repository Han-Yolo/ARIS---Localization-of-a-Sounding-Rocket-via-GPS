#include <QString>
#include <stdint.h>

#include "NavMessageDecoder.h"

/*
 * This thread reads the UBX_RXM_SFRBX messages and decodes the raw satellite navigation messages to get the satellite ephemeris data.
 * The data for each satellite is saved in a struct.
 * The eph vector contains the ephemeris data for all satellites. It can be accessed over the getEphemeris() function.
 * A valid falg in each struct tells if the data can be used to calculate the satellite position.
 */
void NavMessageDecoder::run(void)
{
    emit printNavMesStatus("running...");

    for(;;)
    {
        /* check if new SFRBX message is available */
        if(ublox->RXM_SFRBX_empty())
        {
            msleep(1);
            continue;
        }

        /* get the message */
        RXM_SFRBX sfrbx = ublox->RXM_SFRBX_pop();

        /* check if message is GPS navigation message */
        if(sfrbx.gnssId != UBX_GPS) continue;

        /* check which subframe the message contains */
        unsigned int subframe_ID = (sfrbx.dwrd[2] >> 8) & 0x7;

        int IOD = 0;
        switch(subframe_ID)
        {
            case 1:
                /* check if IODC(Issue of Data, Clock) has changed for this satellite */
                IOD = (((sfrbx.dwrd[3] >> 6) & 0x2) << 8) | ((sfrbx.dwrd[8] >> 22) & 0xFF);
                if(IOD == eph[sfrbx.svId].IODC) continue;

                /* save new data to struct */
                eph[sfrbx.svId].IODC        = IOD;
                eph[sfrbx.svId].Week_No     = ((sfrbx.dwrd[3] >> 20) & 0x3FF);
                eph[sfrbx.svId].SV_accuracy = (sfrbx.dwrd[3] >> 14) & 0xF;
                eph[sfrbx.svId].SV_health   = (sfrbx.dwrd[3] >> 8) & 0x3F;
                eph[sfrbx.svId].T_GD        = readValue8(sfrbx.dwrd[7], true, 6, -31);
                eph[sfrbx.svId].t_oc        = readValue16(sfrbx.dwrd[8], false, 6, 4);
                eph[sfrbx.svId].a_f2        = readValue8(sfrbx.dwrd[9], true, 22, -55);
                eph[sfrbx.svId].a_f1        = readValue16(sfrbx.dwrd[9], true, 6, -43);
                eph[sfrbx.svId].a_f0        = readA_f0(sfrbx.dwrd[10]);

                break;

            case 2:
                /* check if IODE(Issue of Data, Ephemeris) has changed for this satellite */
                IOD = (sfrbx.dwrd[3] >> 22) & 0xFF;
                if(IOD == eph[sfrbx.svId].IODE_sub2) continue;

                /* save new data to struct */
                eph[sfrbx.svId].IODE_sub2   = IOD;
                eph[sfrbx.svId].C_rs        = readValue16(sfrbx.dwrd[3], true, 6, -5.0);
                eph[sfrbx.svId].delta_n     = readValue16(sfrbx.dwrd[4], true, 14, -43.0) * pi;
                eph[sfrbx.svId].M_0         = readValue32(sfrbx.dwrd[4], sfrbx.dwrd[5], true, -31.0) * pi;
                eph[sfrbx.svId].C_uc        = readValue16(sfrbx.dwrd[6], true, 14, -29.0);
                eph[sfrbx.svId].e           = readValue32(sfrbx.dwrd[6], sfrbx.dwrd[7], false, -33.0);
                eph[sfrbx.svId].C_us        = readValue16(sfrbx.dwrd[8], true, 14, -29.0);
                eph[sfrbx.svId].sqrt_A      = readValue32(sfrbx.dwrd[8], sfrbx.dwrd[9], false, -19.0);
                eph[sfrbx.svId].t_oe        = readValue16(sfrbx.dwrd[10], false, 14, 4.0);

                break;

            case 3:
                /* check if IODE(Issue of Data, Ephemeris) has changed for this satellite */
                IOD = (sfrbx.dwrd[10] >> 22) & 0xFF;
                if(IOD == eph[sfrbx.svId].IODE_sub3) continue;

                /* save new data to struct */
                eph[sfrbx.svId].IODE_sub3   = IOD;
                eph[sfrbx.svId].C_ic        = readValue16(sfrbx.dwrd[3], true, 14, -29.0);
                eph[sfrbx.svId].Omega_0     = readValue32(sfrbx.dwrd[3], sfrbx.dwrd[4], true, -31.0) * pi;
                eph[sfrbx.svId].C_is        = readValue16(sfrbx.dwrd[5], true, 14, -29.0);
                eph[sfrbx.svId].i_0         = readValue32(sfrbx.dwrd[5], sfrbx.dwrd[6], true, -31.0) * pi;
                eph[sfrbx.svId].C_rc        = readValue16(sfrbx.dwrd[7], true, 14, -5.0);
                eph[sfrbx.svId].omega       = readValue32(sfrbx.dwrd[7], sfrbx.dwrd[8], true, -31.0) * pi;
                eph[sfrbx.svId].Omega_dot   = readOmegaDot(sfrbx.dwrd[9]) * pi;
                eph[sfrbx.svId].i_dot       = readIDot(sfrbx.dwrd[10]) * pi;

                break;

            default: continue;
        }

        if((eph[sfrbx.svId].IODC == eph[sfrbx.svId].IODE_sub2) && (eph[sfrbx.svId].IODC == eph[sfrbx.svId].IODE_sub3))
        {
            eph[sfrbx.svId].valid = true;
            printNavMesStatus("new ephemeris data for satellite " + QString::number(sfrbx.svId));
        }
        else
        {
            eph[sfrbx.svId].valid = false;
        }
    }
}

/*
 * read a 8bit ephemeris parameter from a single data word
 */
double NavMessageDecoder::readValue8(unsigned int word, bool twoComp, unsigned int shift, double power)
{
    /* extract needed bits */
    uint8_t data = static_cast<uint8_t>((word >> shift) & 0xFFFF);

    /* reinterpret data as two's complement if necessary and apply scale factor */
    if(twoComp)
    {
        int8_t dataComp = *reinterpret_cast<int8_t*>(&data);
        return static_cast<double>(dataComp) * pow(2.0, power);
    }
    else
    {
        return static_cast<double>(data) * pow(2.0, power);
    }
}

/*
 * read a 16bit ephemeris parameter from a single data word
 */
double NavMessageDecoder::readValue16(unsigned int word, bool twoComp, unsigned int shift, double power)
{
    /* extract needed bits */
    uint16_t data = static_cast<uint16_t>((word >> shift) & 0xFFFF);

    /* reinterpret data as two's complement if necessary and apply scale factor */
    if(twoComp)
    {
        int16_t dataComp = *reinterpret_cast<int16_t*>(&data);
        return static_cast<double>(dataComp) * pow(2.0, power);
    }
    else
    {
        return static_cast<double>(data) * pow(2.0, power);
    }
}

/*
 * read a 32bit ephemeris parameter from two following data words
 */
double NavMessageDecoder::readValue32(unsigned int word1, unsigned int word2, bool twoComp, double power)
{
    /* extract needed bits */
    uint32_t data = (((word1 >> 6) & 0xFF) << 24) | ((word2 >> 6) & 0xFFFFFF);

    /* reinterpret data as two's complement if necessary and apply scale factor */
    if(twoComp)
    {
        int dataComp = *reinterpret_cast<int*>(&data);
        return static_cast<double>(dataComp) * pow(2.0, power);
    }
    else
    {
        return static_cast<double>(data) * pow(2.0, power);
    }
}

/*
 * read the ephemeris parameter a_f0 from a single data word
 */
double NavMessageDecoder::readA_f0(unsigned int word)
{
    /* extract needed bits */
    unsigned int data = (word >> 8) & 0x3FFFFF;

    /* append ones if Omega_dot is negative */
    if(data & (1 << 21))
    {
        data |= ~0x3FFFFF;
    }

    /* reinterpret data as two's complement and apply scale factor */
    return static_cast<double>(*reinterpret_cast<int*>(&data)) * pow(2.0, -31.0);
}

/*
 * read the ephemeris parameter Omega_dot from a single data word
 */
double NavMessageDecoder::readOmegaDot(unsigned int word)
{
    /* extract needed bits */
    unsigned int data = (word >> 6) & 0xFFFFFF;

    /* append ones if Omega_dot is negative */
    if(data & (1 << 23))
    {
        data |= ~0xFFFFFF;
    }

    /* reinterpret data as two's complement and apply scale factor */
    return static_cast<double>(*reinterpret_cast<int*>(&data)) * pow(2.0, -43.0);
}

/*
 * read the ephemeris parameter i_dot from a single data word
 */
double NavMessageDecoder::readIDot(unsigned int word)
{
    /* extract needed bits */
    unsigned int data = (word >> 8) & 0x3FFF;

    /* append ones if Omega_dot is negative */
    if(data & (1 << 13))
    {
        data |= ~0x3FFF;
    }

    /* reinterpret data as two's complement and apply scale factor */
    return static_cast<double>(*reinterpret_cast<int*>(&data)) * pow(2.0, -43.0);
}


/*
 * parity check for gps navigation message subframes.
 * not used
 */
bool NavMessageDecoder::checkParity(std::vector<unsigned int> &dwrd)
{
    unsigned int D29_1 = 0, D30_1 = 0;

    /* iterate through all 10 words of subframe */
    for(int i = 0; i < 10; i++)
    {
        /* complement data bits if D30_1 is one (xor) */
        if(D30_1 == 1)
        {
            dwrd[i] = (~dwrd[i] & 0x3FFFFFC0) | (dwrd[i] & 0x3f);
        }

        /* calculate paritiy bits according to document IS-GPS-200D table 20-XIV */
        unsigned int D25, D26, D27, D28, D29, D30;
        D25 = exor(D29_1 | (dwrd[i] & 0x3b1f3480));
        D26 = exor(D30_1 | (dwrd[i] & 0x1d8f9a40));
        D27 = exor(D29_1 | (dwrd[i] & 0x2ec7cd00));
        D28 = exor(D30_1 | (dwrd[i] & 0x1763e680));
        D29 = exor(D30_1 | (dwrd[i] & 0x2bb1f340));
        D30 = exor(D29_1 | (dwrd[i] & 0x0b7a89c0));
        unsigned int parity = (D25<<5) | (D26<<4) | (D27<<3) | (D28<<2) | (D29<<1) | D30;

        /* return false if parity does not match */
        if((dwrd[i] & 0x3F) != parity) return false;

        /* set D29_1 and D30_1 for next iteration */
        D29_1 = D29;
        D30_1 = D30;
    }

    return true;
}

/*
 * xor all bits of an unsigned int
 */
unsigned int NavMessageDecoder::exor(unsigned int value)
{
    unsigned int result = 0;
    while(value)
    {
        result ^= value & 0x1;
        value >>= 1;
    }
    return result;
}
