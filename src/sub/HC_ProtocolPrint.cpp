/*
 * HITIComm
 * HC_ProtocolPrint.cpp
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


#include "sub\HC_Protocol.h"



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// AVR
#include <string.h>
#include <stdlib.h>
#include <avr\pgmspace.h>
#ifdef PROTOBF_USE_INT
	#include <math.h>
#endif

// HITICommSupport
#include <HCS_Serial.h>

// HITIComm
#include "HC_Toolbox.h"
#ifdef PROTOBF_USE_INT
	#include "HC_Data.h"
#endif



// ********************************************************************************
// Define
// ********************************************************************************

#define DEC	10
#define HEX	16



// *****************************************************************************
// Variables
// *****************************************************************************


enum StartChar
{
	//HC_StartChar_diese		= 0x23, // #, A->C (normal response)
	HC_StartChar_exclamation	= 0x21, // !, A->C (error response)
	//HC_StartChar_dollar		= 0x24, // $, C->A (query)
};

enum SpecialChar
{
	SpecialChar_emptyData = '&'
};



// *****************************************************************************
// HC_PROTOCOL : Serial Print
// *****************************************************************************


// Footer (CR + LF) ------------------------------------------------------------

void HC_Protocol::printFooter()
{
	HCS_Serial_println();
}


// char ------------------------------------------------------------------------

void HC_Protocol::printChar(char c)
{
	mOutput_CRC += c;
	HCS_Serial_print(c);
}


// Concatenated Bytes to String ------------------------------------------------

#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
void HC_Protocol::printConcBytesToString(unsigned int hex, uint8_t stringLength)
{
	// convert hex into string
	char str[3];
	mOutput_CRC += HCI_concByteToString(hex, str, stringLength);
	HCS_Serial_print(str);
}
#endif


// Special characters ------------------------------------------------------

void HC_Protocol::printSpecialChar_Separator(bool forcePrinting)
{
	#ifdef PROTOBF_USE_SEPARATOR
		mOutput_CRC += SpecialChar_separator[0];
		HCS_Serial_print(SpecialChar_separator);
	#else
		if (forcePrinting)
		{
			mOutput_CRC += SpecialChar_separator[0];
			HCS_Serial_print(SpecialChar_separator);
		}
	#endif
}

void HC_Protocol::printSpecialChar_Separator()
{
	printSpecialChar_Separator(false);
}

void HC_Protocol::printSpecialChar_EmptyData(bool forcePrintingSeparator)
{
	mOutput_CRC += SpecialChar_emptyData;
	HCS_Serial_print((char) SpecialChar_emptyData);
	printSpecialChar_Separator(forcePrintingSeparator);
}

void HC_Protocol::printSpecialChar_EmptyData()
{
	printSpecialChar_EmptyData(false);
}


// String ------------------------------------------------------------------

void HC_Protocol::printString_withSeparator_P(const char* pgm_str)
{
	HCS_Serial_print_P(pgm_str);

	// calculate CRC
	if (strlen_P(pgm_str) > 0)
	{
		uint8_t i = strlen_P(pgm_str);
		while(i)
			mOutput_CRC += pgm_read_byte_near(pgm_str + (--i));
	}

	printSpecialChar_Separator(true);
}

void HC_Protocol::printString(const char* str, bool forcePrintSeparator)
{
	HCS_Serial_print(str);

	// calculate CRC
	if (strlen(str) > 0)
	{
		uint8_t i = strlen(str);
		while (i)
			mOutput_CRC += str[--i];
	}

	printSpecialChar_Separator(forcePrintSeparator);
}

void HC_Protocol::printString(const char* str)
{
	printString(str, false);
}

void HC_Protocol::printString_withSeparator(const char* str)
{
	printString(str, true);
}


// Start characters --------------------------------------------------------

void HC_Protocol::printStartChar(char StartChar)
{
	printChar(StartChar);
	printSpecialChar_Separator();
}


// Message Type/Error ------------------------------------------------------

#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
	void HC_Protocol::printMessageType(unsigned int messageType)
	{
		printConcBytesToString(messageType, 2);
		printSpecialChar_Separator();
	}
#else
	void HC_Protocol::printMessageType(char messageType)
	{
		printChar(messageType);
		printSpecialChar_Separator();
	}
#endif


#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
void HC_Protocol::printMessageError(unsigned int errorCode)
#else
void HC_Protocol::printMessageError(char errorCode)
#endif
{
	// reset CRC
	mOutput_CRC = 0;

	printStartChar(HC_StartChar_exclamation);	// Header 1
	printMessageType(errorCode);				// Header 2

	#ifdef HC_DISPLAY_INPUT_IN_ERRORMESSAGE
		printString(mInput);
	#endif

	printCRC();									// CRC
	printFooter();								// CR + LF
}


// Control Byte ------------------------------------------------------------

void HC_Protocol::printControlByte(uint8_t b)
{
	// don't fill with leading zero
	printHex(b, false);
}


// CRC ---------------------------------------------------------------------

void HC_Protocol::printCRC()
{
	#ifdef PROTOBF_USE_CRC
		// fill with 0, printed value is CRC
		printHex(HCS_getLowByte(mOutput_CRC), true, true);
	#endif
}

void HC_Protocol::addCRC(unsigned long l)
{
	// CRC with HEX only (base 16)
	do
	{
		// convert number into string, add each char to CRC
		char c = l % 16;
		l /= 16;
		mOutput_CRC += (c < 10) ? c + '0' : c-10 + 'A';
	} while (l);
}


// Print number in Hex format ----------------------------------------------

void HC_Protocol::printHex(bool value)
{
	HCS_Serial_print((value == 0) ? '0' : '1');
	addCRC(value);
	printSpecialChar_Separator();
}

void HC_Protocol::printHex(uint8_t value, bool fillWithZero, bool printedValueIsCRC)
{
	// 2 chars
	#ifdef PROTOBF_FILL_HEX
		// complete with leading 0
		if (fillWithZero && (value < 16))
		{
			HCS_Serial_print('0');
			mOutput_CRC += '0';
		}
	#endif

	if (value == 0)
		HCS_Serial_print('0');
	else
		HCS_Serial_print(value, HEX);

	if (!printedValueIsCRC)
	{
		addCRC(value);
		printSpecialChar_Separator();
	}
}

void HC_Protocol::printHex(uint8_t value, bool fillWithZero)
{
	printHex(value, fillWithZero, false);
}

void HC_Protocol::printHex(uint8_t value)
{
	printHex(value, true, false);
}


void HC_Protocol::printHex(unsigned int value, uint8_t maxLengthWithZeros)
{
	// 4 chars
	#ifdef PROTOBF_FILL_HEX
		// complete with leading zero
		while(--maxLengthWithZeros)
		{
			if (value < HC_pow(16, maxLengthWithZeros))
			{
				HCS_Serial_print('0');
				mOutput_CRC += '0';
			}
			else
				break;
		}
	#endif

	if (value == 0)
		HCS_Serial_print('0');
	else
		HCS_Serial_print(value, HEX);

	addCRC(value);
	printSpecialChar_Separator();
}

void HC_Protocol::printHex(unsigned int value)
{
#if defined ARDUINO_ARCH_SAMD
	printHex(value, (uint8_t)8); // unsigned int = 32 bit
#else
	printHex(value, (uint8_t)4); // unsigned int = 16 bit
#endif
}


void HC_Protocol::printHex(unsigned long value, uint8_t maxLengthWithZeros)
{
	// 8 chars
	#ifdef PROTOBF_FILL_HEX
		// complete with leading zero
		while (--maxLengthWithZeros)
		{
			if (value < HC_pow(16, maxLengthWithZeros))
			{
				HCS_Serial_print('0');
				mOutput_CRC += '0';
			}
			else
				break;
		}
	#endif

	if (value == 0)
		HCS_Serial_print('0');
	else
		HCS_Serial_print(value, HEX);

	addCRC(value);
	printSpecialChar_Separator();
}

void HC_Protocol::printHex(unsigned long value)
{
	printHex(value, (uint8_t)8);
}


// Print number in Hex or Decimal format (depending on options) --------------

void HC_Protocol::printNumber(bool number)
{
	printHex(number);
}

void HC_Protocol::printNumber(uint8_t number)
{
	#ifdef PROTOBF_USE_INT
		HCS_Serial_print(number, DEC);
		printSpecialChar_Separator();
	#else
		printHex(number);
	#endif
}

void HC_Protocol::printNumber(unsigned int number, uint8_t maxHexLengthWithZeros)
{
	#ifdef PROTOBF_USE_INT
		HCS_Serial_print(number, DEC);
		printSpecialChar_Separator();
	#else
		printHex(number, maxHexLengthWithZeros);
	#endif
}

void HC_Protocol::printNumber(unsigned int number)
{
#if defined ARDUINO_ARCH_SAMD
	printNumber(number, (uint8_t)8); // unsigned int = 32 bit
#else
	printNumber(number, (uint8_t)4); // unsigned int = 16 bit
#endif
}

void HC_Protocol::printNumber(unsigned long number, uint8_t maxHexLengthWithZeros)
{
	#ifdef PROTOBF_USE_INT
		HCS_Serial_print(number, DEC);
		printSpecialChar_Separator();
	#else
		printHex(number, maxHexLengthWithZeros);
	#endif
}

void HC_Protocol::printNumber(unsigned long number)
{
	printNumber(number, (uint8_t)8);
}


// print Float in Hex or Decimal format (depending on options) -------------
void HC_Protocol::printFloat(float number)
{
	#ifdef PROTOBF_USE_INT
		double integer;

		// if fraction is null
		if (modf(number, &integer) == 0.0)
			HCS_Serial_print(number, 0);
		else
			HCS_Serial_print(number, HC_DECIMAL_QTY);

		printSpecialChar_Separator();		
	#else
		printHex(HCI_convertFloatToHex(number));
	#endif
}



// *****************************************************************************
// HC_PROTOCOL : String to Number
// *****************************************************************************


// Hex format string to unsigned Long
unsigned long HC_Protocol::hexStringToULong(char* str)
{
	// convert string (hex) to a long
	return strtoul(str, NULL_POINTER, 16);
}


// string to Boolean
bool HC_Protocol::stringToBool(char* str)
{
	// convert string to bool
	return (atoi(str) != 0) ? 1 : 0;
}


// Decimal format string to Unsigned Long (max 4 294 967 295)
unsigned long HC_Protocol::stringToULong(char* str)
{
	#ifdef PROTOBF_USE_INT
		// convert string (decimal) to a long
		return atol(str);
	#else
		// convert string (hex) to a long
		return hexStringToULong(str);
	#endif
}


// Decimal format string to Unsigned Int (max 65535)
unsigned int HC_Protocol::stringToUInt(char* str)
{
	#ifdef PROTOBF_USE_INT
		// convert string (decimal) to a long
		return atoi(str);
	#else
		// convert string (hex) to a long
		return (unsigned int) hexStringToULong(str);
	#endif
}


// Decimal format string to float
float HC_Protocol::stringToFloat(char* str)
{
	#ifdef PROTOBF_USE_INT
		// convert the data string (decimal) to a double
		return atof(str);
	#else
		// convert the data string (hex) to a long, then a float
		return HCI_convertHexToFloat(hexStringToULong(str));
	#endif
}
