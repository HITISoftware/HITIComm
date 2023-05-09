/*
 * HITIComm
 * HC_Toolbox.cpp
 *
 * Copyright © 2021 Christophe LANDRET
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "HC_Toolbox.h"



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// AVR
#include <string.h>



// *****************************************************************************
// Methods
// *****************************************************************************


// --------------------------------------------------------------------------------
// Math ---------------------------------------------------------------------------
// --------------------------------------------------------------------------------

unsigned long HC_pow(unsigned long b, uint8_t p)
{
    if (p == 0)
        return 1;
    else if (p > 1)
    {
        unsigned long t = b; // record multiplier

        uint8_t i = p;
        while (--i)
            b *= t;
    }

    return b;
}

long HC_divide(long numerator, long denominator)
{
    // if remainder is bigger than half the denominator, rounds up
    if (HCS_abs(numerator % denominator) > HCS_abs(denominator / 2))
        return numerator / denominator + ((numerator / denominator < 0) ? -1 : 1);
    else
        return numerator / denominator;
}

long HC_scale(
        long value, 
        long value_Min, 
        long value_Max, 
        long scaledValue_Min, 
        long scaledValue_Max)
{
    return scaledValue_Min + HC_divide((scaledValue_Max - scaledValue_Min) * (value - value_Min), (value_Max - value_Min));
}


// --------------------------------------------------------------------------------
// Bit count ----------------------------------------------------------------------
// --------------------------------------------------------------------------------

uint8_t HC_countBit(unsigned long b, uint8_t bit_qty)
{
    // limit bit qty
    if (bit_qty > 32)
        bit_qty = 32;

    uint8_t counter = 0;
    //for (uint8_t i = 0; i < bit_qty; ++i)
    uint8_t i = bit_qty;
    while(i)
    {
        --i;

        if (b & 1)
            ++counter;

        b >>= 1;
    }

    return counter;
}

uint8_t HC_countBit(unsigned long b)   { return HC_countBit(b , 32); }
uint8_t HC_countBit(unsigned int b)    { return HC_countBit(b, 16); }
uint8_t HC_countBit(uint8_t b)         { return HC_countBit(b, 8); }



// --------------------------------------------------------------------------------
// Float <-> Hex ------------------------------------------------------------------
// --------------------------------------------------------------------------------

typedef union FloatU
{
    float float_value;
    unsigned long hex_value;
    uint8_t byte_array[4];
}FloatUnion;

float HCI_convertHexToFloat(unsigned long l)
{
    FloatUnion u;
    u.hex_value = l;
    return u.float_value;
}

unsigned long HCI_convertFloatToHex(float f)
{
    FloatUnion u;
    u.float_value = f;
    return u.hex_value;
}

// --------------------------------------------------------------------------------
// Concatenated Bytes <-> String (Concatenated Chars)
//   => Concatenated Bytes = concatenation from high to low bytes
//   => String             = concatenation of chars
//
// max string size: 2 chars => 2 bytes (Word)
// --------------------------------------------------------------------------------

unsigned int HCI_stringToConcByte(char* inputString)
{
    if (strlen(inputString) == 1)
        return inputString[0];
    else if (strlen(inputString) == 2)
        return HCS_createWord(inputString[0], inputString[1]);
    else
        return 0;
}

// return the sum of the char values (useful for Checksum calculation)
unsigned long HCI_concByteToString(unsigned int inputValue, char* destinationString, uint8_t stringLength)
{
    if (stringLength == 1)
    {
        destinationString[0] = HCS_getLowByte(inputValue);
        destinationString[1] = '\0';
        destinationString[2] = '\0';
    }
    else if (stringLength == 2)
    {
        destinationString[0] = HCS_getHighByte(inputValue);
        destinationString[1] = HCS_getLowByte(inputValue);
        destinationString[2] = '\0';
    }

    return (unsigned long) destinationString[0] + (unsigned long) destinationString[1] + (unsigned long) destinationString[2];
}



// -----------------------------------------------------------------------------
// Flag : check for value has changed ------------------------------------------
// -----------------------------------------------------------------------------

bool HCI_readAndConsume(bool* flag)
{
    if (*flag)
    {
        // reset flag
        *flag = false;

        return true;
    }

    return false;
}

