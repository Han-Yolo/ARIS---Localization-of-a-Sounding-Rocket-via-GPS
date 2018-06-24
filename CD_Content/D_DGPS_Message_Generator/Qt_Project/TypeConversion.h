/*
 * TypeConversion.h
 *
 *  Created on: Mar 22, 2018
 *      Author: Simon Herzog
 */

#ifndef TYPECONVERSION_H_
#define TYPECONVERSION_H_


#include <stdlib.h>
#include <algorithm>


/*
 * converts unsigned char array to an unsigned int
 */
unsigned int array_to_uint(unsigned char *input, int length, int offset, bool little_endian)
{
	unsigned int output = 0;

	if(little_endian)
	{
		for(int i = 0; i < length; i++)
		{
			output |= (input[i + offset] << 8*i);
		}
	}
	else
	{
		for(int i = 0; i < length; i++)
		{
			output |= (input[length-1 - i + offset] << 8*i);
		}
	}

	return output;
}

/*
 * converts unsigned char array to an int
 */
int array_to_int(unsigned char *input, int length, int offset, bool little_endian)
{
	int output = 0;

	if(little_endian)
	{
		for(int i = 0; i < length; i++)
		{
			output |= (input[i + offset] << 8*i);
		}
	}
	else
	{
		for(int i = 0; i < length; i++)
		{
			output |= (input[length-1 - i + offset] << 8*i);
		}
	}

	return output;
}

/*
 * converts char array to a double
 */
double array_to_double(unsigned char *input, int offset, bool double_precision)
{
	if(double_precision)
	{
		unsigned char val[8];
		memcpy(val, input + offset, 8);

        return *reinterpret_cast<double*>(val);
	}
	else
	{
		unsigned char val[4];
		memcpy(val, input + offset, 4);
		float f = *reinterpret_cast<float*>(val);

		return static_cast<double>(f);
	}
}


#endif /* TYPECONVERSION_H_ */
