#ifndef RTCMENCODE_H
#define RTCMENCODE_H


#include <vector>
#include <boost/dynamic_bitset.hpp>

#include "GPSDataTypes.h"
#include "Debug.h"


void addToBitset(boost::dynamic_bitset<> &data, int value, int length);
std::vector<unsigned char> packBits(boost::dynamic_bitset<> bits, int nofWords);
void addParity(std::vector<unsigned int> &msg);
unsigned int exor(unsigned int value);


std::vector<unsigned char> encodeMsg1(RTCM_msg1 msg)
{
    boost::dynamic_bitset<> bits;

    /* calc number of words */
    int nofWords = static_cast<int>(ceil((40.0*msg.data.size())/24.0 + 2.0));

    /* calc modified z-count */
    int z_count = static_cast<int>(msg.time/0.6) & 0x1FFF;

    /* add header */
    addToBitset(bits, 0b01100110, 8);       // preamble
    addToBitset(bits, 1, 6);                // message type
    addToBitset(bits, 199, 10);             // station I.D.
    addToBitset(bits, z_count, 13);         // modified z-count
    addToBitset(bits, 0, 3);                // sequence no.
    addToBitset(bits, nofWords, 5);         // no. of data words
    addToBitset(bits, 0, 3);                // station health (1m scale factor)

    /* add body */
    for(size_t i = 0; i < msg.data.size(); i++)
    {
        /* determine scale factor */
        int scaleFactor = 0;
        if((abs(msg.data[i].PRC) > 655.3) || (abs(msg.data[i].RRC) > 0.25)) scaleFactor = 1;

        /* scale PRC and RRC */
        int prc, rrc;
        if(scaleFactor)
        {
            prc = static_cast<int>(msg.data[i].PRC/0.32);
            rrc = static_cast<int>(msg.data[i].RRC/0.032);
        }
        else
        {
            prc = static_cast<int>(msg.data[i].PRC/0.02);
            rrc = static_cast<int>(msg.data[i].RRC/0.002);
        }        

        addToBitset(bits, scaleFactor, 1);          // scale factor
        addToBitset(bits, 0, 2);                    // UDRE (sigma <= 1m)
        addToBitset(bits, msg.data[i].satID, 5);    // satellite ID
        addToBitset(bits, prc, 16);                 // PRC
        addToBitset(bits, rrc, 8);                  // RRC
        addToBitset(bits, msg.data[i].IOD, 8);      // issue of data
    }

    /* add fill bits */
    size_t nofFill = 8*(msg.data.size()%3);
    int toggle = 1;
    for(size_t i = 0; i < nofFill; i++)
    {
        bits.push_back(toggle);
        toggle = !toggle;
    }

    std::vector<unsigned char> data = packBits(bits, nofWords);

#if DEBUG_RTCM
    std::cout << "GPS ToW: " << msg.time << "\t modified z-count: " << (double)z_count*0.6 << "\t Words: " << nofWords << "\t Fill Bits: " << nofFill << std::endl;
    for(size_t i = 0; i < msg.data.size(); i++)
    {
        std::cout << "Sat: " << msg.data[i].satID << "\t PRC: " << std::setprecision(9) << msg.data[i].PRC << "\t RRC: " << msg.data[i].RRC << "\t IOD: " << msg.data[i].IOD << std::endl;
    }

    /* print LSB to MSB, in order */
    std::cout << "Bits: ";
    for (boost::dynamic_bitset<>::size_type i = 0; i < bits.size(); ++i) {
        std::cout << bits[i];
    }
    std::cout << std::endl;

    std::cout << "RTCM_1: " << std::hex;
    for(size_t i = 0; i < data.size(); i++)
    {
        std::cout << (int)data[i];
    }
    std::cout << std::dec << "\t NOF Bytes: " << data.size() << "\t Real NOF Words: " << (6.0*data.size())/30.0 << std::endl << std::endl;
#endif

    return data;
}

std::vector<unsigned char> encodeMsg3(RTCM_msg3 msg)
{
    boost::dynamic_bitset<> bits;

    /* number of words */
    const int nofWords = 6;

    /* add header */
    addToBitset(bits, 0b01100110, 8);                               // preamble
    addToBitset(bits, 3, 6);                                        // message type
    addToBitset(bits, 199, 10);                                     // station I.D.
    addToBitset(bits, static_cast<int>(msg.time/0.6), 13);          // modified z-count
    addToBitset(bits, 1, 3);                                        // sequence no.
    addToBitset(bits, nofWords, 5);                                 // no. of data words
    addToBitset(bits, 0, 3);                                        // station health (1m scale factor)

    /* add body */
    addToBitset(bits, static_cast<int>(msg.pos.x/0.01), 32);
    addToBitset(bits, static_cast<int>(msg.pos.y/0.01), 32);
    addToBitset(bits, static_cast<int>(msg.pos.z/0.01), 32);

    return packBits(bits, nofWords);
}

void addToBitset(boost::dynamic_bitset<> &data, int value, int length)
{
    for(;length > 0; length--)
    {
        data.push_back((value >> (length-1)) & 0x1);
    }
}

/*
 * pack bit steram into bytes to be transported over UART
 */
std::vector<unsigned char> packBits(boost::dynamic_bitset<> bits, int nofWords)
{
    /* bits LSB <--> MSB swap */
    boost::dynamic_bitset<> bits_swapped;
    for(size_t i = 0; i < bits.size(); i++)
    {
        bits_swapped.push_back(bits[bits.size() - 1 - i]);
    }

    /* pack bit stream into 24bit words */
    std::vector<unsigned int> data;
    boost::dynamic_bitset<> mask(bits_swapped.size(), 0xFFFFFF);
    for(int i = 0; i < nofWords; i++)
    {
        unsigned int word = ((bits_swapped >> ((nofWords - 1 - i)*24)) & mask).to_ulong() << 6;
        data.push_back(word);
    }

    addParity(data);

    /* pack words into bytes following the RTCM 10402.3 document chapter 5 */
    std::vector<unsigned char> byteStream;
    for(size_t i = 0; i < data.size(); i++)
    {
        for(int k = 4; k >= 0 ; k--)
        {
            /* extract next 6 bits */
            unsigned char c = static_cast<unsigned char>((data[i] >> (k*6)) & 0x3F);

            /* change bit order */
            c = ((c>>5) & (1<<0)) | ((c>>3) & (1<<1)) | ((c>>1) & (1<<2)) | ((c<<1) & (1<<3)) | ((c<<3) & (1<<4)) | ((c<<5) & (1<<5));

            /* add marking and spacing bits */
            c = (0b01 << 6) | (c & 0x3F);

            byteStream.push_back(c);
        }
    }

    return byteStream;
}

void addParity(std::vector<unsigned int> &msg)
{
    static unsigned int D29_1 = 0, D30_1 = 0;

    /* iterate through all 10 words of message */
    for(size_t i = 0; i < msg.size(); i++)
    {
        /* calculate paritiy bits according to document IS-GPS-200D table 20-XIV */
        unsigned int D25, D26, D27, D28, D29, D30;
        D25 = exor(D29_1 | (msg[i] & 0x3b1f3480));
        D26 = exor(D30_1 | (msg[i] & 0x1d8f9a40));
        D27 = exor(D29_1 | (msg[i] & 0x2ec7cd00));
        D28 = exor(D30_1 | (msg[i] & 0x1763e680));
        D29 = exor(D30_1 | (msg[i] & 0x2bb1f340));
        D30 = exor(D29_1 | (msg[i] & 0x0b7a89c0));
        unsigned int parity = (D25<<5) | (D26<<4) | (D27<<3) | (D28<<2) | (D29<<1) | D30;

        /* complement data bits if D30_1 is one (xor) */
        if(D30_1 == 1)
        {
            msg[i] = (~msg[i] & 0x3FFFFFC0);
        }

        /* add parity to message */
        msg[i] = (msg[i] & 0x3FFFFFC0) | (parity & 0x3F);

        /* set D29_1 and D30_1 for next iteration */
        D29_1 = D29;
        D30_1 = D30;
    }
}

/*
 * xor all bits of an unsigned int
 */
unsigned int exor(unsigned int value)
{
    unsigned int result = 0;
    while(value)
    {
        result ^= value & 0x1;
        value >>= 1;
    }
    return result;
}


#endif // RTCMENCODE_H
