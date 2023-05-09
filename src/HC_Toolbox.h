/*
 * HITIComm
 * HC_Toolbox.h
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


// *****************************************************************************
// Include Guard
// *****************************************************************************

#ifndef HC_Toolbox_h
#define HC_Toolbox_h



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HITICommSupport.h>
#include <HCS_Toolbox.h>



// *****************************************************************************
// Methods
// *****************************************************************************


// --------------------------------------------------------------------------
// Math ---------------------------------------------------------------------
// --------------------------------------------------------------------------

unsigned long HC_pow(unsigned long b, uint8_t p);
long HC_divide(long numerator, long denominator);
long HC_scale(long value, long value_Min, long value_Max, long scaledValue_Min, long scaledValue_Max);



// -------------------------------------------------------------------------- 
// Bit operations -----------------------------------------------------------
// --------------------------------------------------------------------------

uint8_t HC_countBit(unsigned long b, uint8_t bit_qty);
uint8_t HC_countBit(unsigned long b);
uint8_t HC_countBit(unsigned int b);
uint8_t HC_countBit(uint8_t b);



// --------------------------------------------------------------------------
// Float <-> Hex ------------------------------------------------------------
// --------------------------------------------------------------------------

float HCI_convertHexToFloat(unsigned long l);
unsigned long HCI_convertFloatToHex(float f);

// Concatenated Bytes <-> String (Concatenated Chars)
//   => Concatenated Bytes = concatenation from high to low bytes
//   => String             = concatenation of chars
//
// max string size: 2 chars => 2 bytes (Word)
unsigned int HCI_stringToConcByte(char* inputString);
unsigned long HCI_concByteToString(unsigned int inputValue, char* destinationString, uint8_t stringLength);



// --------------------------------------------------------------------------
// Flag : check for value has changed ---------------------------------------
// --------------------------------------------------------------------------

bool HCI_readAndConsume(bool* flag);


#endif
