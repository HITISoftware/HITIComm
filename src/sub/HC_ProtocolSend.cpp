/*
 * HITIComm
 * HC_ProtocolSend.cpp
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

// HITICommSupport
#include <HCS_LowAccess_Bus.h>
#include <HCS_LowAccess_IO.h>
#include <HCS_Time.h>
#include <HCS_Atmel.h>
#include <HCS_Serial.h>

// HITIComm
#include "HC_Data.h"
#include "HC_Sram.h"
#include "HC_ServoManager.h"



// *****************************************************************************
// Variables
// *****************************************************************************


enum StartChar
{
	HC_StartChar_diese			= 0x23, // #, A->C (normal response)
	//HC_StartChar_exclamation	= 0x21, // !, A->C (error response)
	//HC_StartChar_dollar		= 0x24, // $, C->A (query)
};


// Message Type --------------------------------------------------------


enum MessageType
{
#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
	HC_MessageType_Bf = 0x4266,  // B query request (Board features)
	HC_MessageType_Bq = 0x4271,  // B query replies (Board features 0-3)
	HC_MessageType_BS = 0x4253,  // Board has started (or has reset)

	HC_MessageType_M0 = 0x4D30,  // SRAM (Break value 0, Stack Pointer 0)
	HC_MessageType_FR = 0x4652,  // Free RAM (measurement 0-2)

	HC_MessageType_CT = 0x4354,  // Cycle Time (in us)
	HC_MessageType_TM = 0x544D,  // Arduino Time (in ms)

#ifdef HC_STRINGMESSAGE_COMPILE
	HC_MessageType_S0 = 0x5330,  // HITI string
#endif

#ifdef HC_EEPROM_COMPILE
	HC_MessageType_EE = 0x4545,  // EEPROM access
	HC_MessageType_Es = 0x4573,  // EEPROM (notify start of a reply sequence)	
	HC_MessageType_Ee = 0x4565,  // EEPROM (notify end of a reply sequence)	
	HC_MessageType_Ec = 0x4563,  // EEPROM (IO Config access)
	HC_MessageType_Ep = 0x4570,  // EEPROM (Config Param)
#endif

	HC_MessageType_PM = 0x504D,  // Pin mode / Input mode
	HC_MessageType_DI = 0x4449,  // DI values
	HC_MessageType_DO = 0x444F,  // DO values
	HC_MessageType_AI = 0x4149,  // AI values
	HC_MessageType_OT = 0x4F54,  // Output type
	HC_MessageType_PA = 0x5041,  // PWM availability
	HC_MessageType_PW = 0x5057,  // PWM values
	HC_MessageType_SM = 0x534D,  // Servo mode
	HC_MessageType_SV = 0x5356,  // Servo values
	HC_MessageType_DD = 0x4444,  // DD values
	HC_MessageType_AM = 0x414D,  // AD mask
	HC_MessageType_AD = 0x4144,  // AD values
#ifdef ARDUINO_ARCH_SAMD
	HC_MessageType_DM = 0x444D,  // DAC mode (enable mask)
	HC_MessageType_DA = 0x4441,  // DAC values
#endif

	HC_MessageType_Xs = 0x5873,  // Subscribe to X query
	HC_MessageType_Xu = 0x5875,  // Unsubscribe from X query

	HC_MessageType_X0 = 0x5830,  // XQuery period, Cycle Time
	HC_MessageType_X1 = 0x5831,  // AI values (part 1 : 0 - 7)
	HC_MessageType_X2 = 0x5832,  // AI values (part 2 : 8 - 15)
	HC_MessageType_X3 = 0x5833,  // PWM values (part 1 : 0 - 7)
	HC_MessageType_X4 = 0x5834,  // PWM values (part 1 : 8 - 15)
	HC_MessageType_X5 = 0x5835,  // Servo values (part 1 :  0 - 5)
	HC_MessageType_X6 = 0x5836,  // Servo values (part 2 :  6 - 11)
	HC_MessageType_X7 = 0x5837,  // Servo values (part 3 : 12 - 17) 
	HC_MessageType_X8 = 0x5838,  // Servo values (part 4 : 18 - 23) 
	HC_MessageType_X9 = 0x5839,  // Servo values (part 5 : 24 - 29) 
	HC_MessageType_XA = 0x5841,  // Servo values (part 6 : 30 - 35)
	HC_MessageType_XB = 0x5842,  // Servo values (part 7 : 36 - 41) 
	HC_MessageType_XC = 0x5843,  // Servo values (part 8 : 42 - 47) 
	HC_MessageType_XD = 0x5844,  // AD values (part 1 :  0 - 3)
	HC_MessageType_XE = 0x5845,  // AD values (part 2 :  4 - 7)
	HC_MessageType_XF = 0x5846,  // AD values (part 3 :  8 - 11)
	HC_MessageType_XG = 0x5847,  // AD values (part 4 : 12 - 15)
	HC_MessageType_XH = 0x5848,  // AD values (part 5 : 16 - 19)

	HC_MessageType_Aq = 0x4171,  // A query reply
	HC_MessageType_As = 0x4173,  // Subscribe to A query
	HC_MessageType_Au = 0x4175,  // Unsubscribe from A query
#else
	HC_MessageType_Bf = 0x31,  // B query request (Board features)
	HC_MessageType_Bq = 0x30,  // B query replies (Board features 0-3)
	HC_MessageType_BS = 0x32,  // Board has started (or has reset)

	HC_MessageType_M0 = 0x3A,  // SRAM (Break value 0, Stack Pointer 0)
	HC_MessageType_FR = 0x3D,  // Free RAM (measurement 0-2)

	HC_MessageType_CT = 0x5A,  // Cycle Time (in us)
#ifdef HC_ARDUINOTIME_COMPILE
	HC_MessageType_TM = 0x5C,  // Arduino Time (in ms)
#endif

#ifdef HC_STRINGMESSAGE_COMPILE
	HC_MessageType_S0 = 0x79,  // HITI string
#endif
	
#ifdef HC_EEPROM_COMPILE
	HC_MessageType_EE = 0x70,  // EEPROM access
	HC_MessageType_Es = 0x71,  // EEPROM (notify start of a reply sequence)	
	HC_MessageType_Ee = 0x72,  // EEPROM (notify end of a reply sequence)	
	HC_MessageType_Ec = 0x73,  // EEPROM (IO Config access)
	HC_MessageType_Ep = 0x74,  // EEPROM (Config Param)
#endif

	HC_MessageType_PM = 0x60,  // Pin mode / Input mode
	HC_MessageType_DI = 0x61,  // DI values
	HC_MessageType_DO = 0x62,  // DO values
	HC_MessageType_AI = 0x63,  // AI values
	HC_MessageType_OT = 0x64,  // Output type
	HC_MessageType_PA = 0x65,  // PWM availability
	HC_MessageType_PW = 0x66,  // PWM values
	HC_MessageType_SM = 0x67,  // Servo mode
	HC_MessageType_SV = 0x68,  // Servo values
	HC_MessageType_DD = 0x69,  // DD values
	HC_MessageType_AM = 0x6A,  // AD mask
	HC_MessageType_AD = 0x6B,  // AD values
#ifdef ARDUINO_ARCH_SAMD
	HC_MessageType_DM = 0x6C,  // DAC mode (enable mask)
	HC_MessageType_DA = 0x6D,  // DAC values
#endif
		
	HC_MessageType_Xs = 0x5B,  // Subscribe to X query
	HC_MessageType_Xu = 0x5D,  // Unsubscribe from X query

	HC_MessageType_X0 = 0x40,  // XQuery period, Cycle Time
	HC_MessageType_X1 = 0x41,  // AI values (part 1 : 0 - 7)
	HC_MessageType_X2 = 0x42,  // AI values (part 2 : 8 - 15)
	HC_MessageType_X3 = 0x43,  // PWM values (part 1 : 0 - 7)
	HC_MessageType_X4 = 0x44,  // PWM values (part 1 : 8 - 15)
	HC_MessageType_X5 = 0x45,  // Servo values (part 1 :  0 - 5)
	HC_MessageType_X6 = 0x46,  // Servo values (part 2 :  6 - 11)
	HC_MessageType_X7 = 0x47,  // Servo values (part 3 : 12 - 17) 
	HC_MessageType_X8 = 0x48,  // Servo values (part 4 : 18 - 23) 
	HC_MessageType_X9 = 0x49,  // Servo values (part 5 : 24 - 29) 
	HC_MessageType_XA = 0x4A,  // Servo values (part 6 : 30 - 35)
	HC_MessageType_XB = 0x4B,  // Servo values (part 7 : 36 - 41) 
	HC_MessageType_XC = 0x4C,  // Servo values (part 8 : 42 - 47) 
	HC_MessageType_XD = 0x4D,  // AD values (part 1 :  0 - 3)
	HC_MessageType_XE = 0x4E,  // AD values (part 2 :  4 - 7)
	HC_MessageType_XF = 0x4F,  // AD values (part 3 :  8 - 11)
	HC_MessageType_XG = 0x50,  // AD values (part 4 : 12 - 15)
	HC_MessageType_XH = 0x51,  // AD values (part 5 : 16 - 19)

	HC_MessageType_Aq = 0x7A,  // A query reply
	HC_MessageType_As = 0x7B,  // Subscribe to A query
	HC_MessageType_Au = 0x7D,  // Unsubscribe from A query
#endif
};


// length of hex number to send
#define HEX_LENGTH_CYCLETIME		5  // max 1s
#define HEX_LENGTH_AI				3
#define HEX_LENGTH_SERVO			5

#if defined ARDUINO_ARCH_SAMD
	#define GET_HEXCOUNT_FROM_BITCOUNT(bitCount)	(bitCount/4 + ((bitCount%4) != 0 ? 1 : 0))
	#define HEX_LENGTH_PWM							(GET_HEXCOUNT_FROM_BITCOUNT(HCS_getPwmResolution()))
	#define HEX_LENGTH_DAC							(GET_HEXCOUNT_FROM_BITCOUNT(HCS_getDacResolution()))
#else
	#define HEX_LENGTH_PWM		2
#endif



// *****************************************************************************
// HC_PROTOCOL : Send message
// *****************************************************************************


#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
void HC_Protocol::send(unsigned int messageType)
#else
void HC_Protocol::send(char messageType)
#endif
{
	// flag for checking if message has data
	bool containsData = true;

	// reset CRC
	mOutput_CRC = 0;


    // Header 1 (start byte (1 char))
	printStartChar(HC_StartChar_diese);
	
    // Header 2 (message type)
    printMessageType(messageType);

	// Header 3 (Control Byte, 1 char))
	printControlByte((uint8_t) 0); // no index, no address

	
	// Depending on Mesage Type

	#ifdef HC_DISPLAY_X_QUERY_PERIOD
		unsigned long now;
		uint8_t period);
	#endif

	switch(messageType)
	{
		// Normal query *******************************************************

		// SRAM
		case HC_MessageType_M0:
			// Probe 0-2: Brake value, Stack Pointer. Free RAM is calculated by HITIPanel
			printNumber(HC_sram.getAddress_HeapBreakValue(0));
			printNumber(HC_sram.getAddress_HeapBreakValue(1));
			printNumber(HC_sram.getAddress_HeapBreakValue(2));
			printNumber(HC_sram.getStackPointer(0));
			printNumber(HC_sram.getStackPointer(1));
			printNumber(HC_sram.getStackPointer(2));
			break;

		// Cycle Time (in us)
		case HC_MessageType_CT:
			printNumber(HCS_getCycleTime(), HEX_LENGTH_CYCLETIME);
			break;

		// Time (in ms)
		#ifdef HC_ARDUINOTIME_COMPILE
			case HC_MessageType_TM:
				printNumber(millis());
				break;
		#endif
					
		// HITI String
		#ifdef HC_STRINGMESSAGE_COMPILE
			case HC_MessageType_S0:
				printString(HC_readString());
				break;
		#endif	

		// EEPROM (Config Param)
		/*#ifdef HC_EEPROM_COMPILE
			case HC_MessageType_Ep:
				printHex(HC_eeprom.readConfigRegister());
				break;
		#endif*/

		// Pin Mode
		case HC_MessageType_PM:
			#if HC_VARIANT == HC_VARIANT_MEGA
				// Pin mode
				printHex(HC_readPinsMode_H());
				printHex(HC_readPinsMode_L());

				// input mode
				printHex(HC_readInputsMode_H());
				printHex(HC_readInputsMode_L());
			#elif defined(ARDUINO_ARCH_SAMD)
				// pin mode
				printHex(HC_readPinsMode());

				// input mode
				printHex(HC_readInputsMode());

				// input mode option
				printHex(HC_readInputsModeOption());
			#else
				// pin mode
				printHex(HC_readPinsMode());

				// input mode
				printHex(HC_readInputsMode());
			#endif
			break;

		// DI values
		case HC_MessageType_DI:
			#if HC_VARIANT == HC_VARIANT_MEGA
				printHex(HC_readDI_H());
				printHex(HC_readDI_L());
			#else
				printHex(HC_readDI());
			#endif
			break;

		// DO values
		case HC_MessageType_DO:
			#if HC_VARIANT == HC_VARIANT_MEGA
				printHex(HC_readDO_H());
				printHex(HC_readDO_L());
			#else
				printHex(HC_readDO());
			#endif
			break;

		// Output type
		case HC_MessageType_OT:
			#if HC_VARIANT == HC_VARIANT_MEGA
				printHex(HC_readOutputTypes_H());
				printHex(HC_readOutputTypes_L());
			#else
				printHex(HC_readOutputTypes());
			#endif
			break;

		// PWM availability
		case HC_MessageType_PA:
			#if HC_VARIANT == HC_VARIANT_MEGA
				printHex(HC_PwmIsAvailable_H());
				printHex(HC_PwmIsAvailable_L());
			#else
				printHex(HC_PwmIsAvailable());
			#endif
			break;

		// Servo mode
		case HC_MessageType_SM:
			#if HC_VARIANT == HC_VARIANT_MEGA
				printHex(HC_readServosMode_H());
				printHex(HC_readServosMode_L());
			#else
				printHex(HC_readServosMode());
			#endif
			break;

		// DD values
		case HC_MessageType_DD:
			printHex(HC_readDD());
			break;

		// AD mode
		case HC_MessageType_AM:
			printHex(HCI_getADMask());
			break;

		#ifdef ARDUINO_ARCH_SAMD
		// DAC mode
		case HC_MessageType_DM:
			printHex(HC_readDacsMode());
			break;
		#endif


		// X query *******************************************************

		// X query period (ms), Cycle Time (us)
		case HC_MessageType_X0:
			// XQuery period (ms), 2 chars: max 255ms
			#ifdef HC_DISPLAY_X_QUERY_PERIOD
				now = millis();
				period = (uint8_t)(now - mXquery_previousTimestamp);
				mXquery_previousTimestamp = now;

				printNumber(period);
			#endif

			// Cycle Time (in us)
			printNumber(HCS_getCycleTime(), HEX_LENGTH_CYCLETIME);
			break;

		// AI values (part 1 : 0 - 7)
		case HC_MessageType_X1:
			sendX_AIValues(0, 7);
			break;	

		// AI values (part 2 : 8 - 15)
		case HC_MessageType_X2:
			sendX_AIValues(8, 15);
			break;
				
		// PWM values (part 1 : 0 - 7)
		case HC_MessageType_X3:
			containsData = sendX_AOValues(0, 7);
			break;
				
		// PWM values (part 1 : 8 - 15)
		case HC_MessageType_X4:
			containsData = sendX_AOValues(8, 15);
			break;
			
		// Servo values (part 1 :  0 - 5)
		case HC_MessageType_X5:
			containsData = sendX_ServoValues(0, 5);
			break;
			
		// Servo values (part 2 :  6 - 11)
		case HC_MessageType_X6:
			containsData = sendX_ServoValues(6, 11);
			break;
			
		// Servo values (part 3 : 12 - 17) 
		case HC_MessageType_X7:
			containsData = sendX_ServoValues(12, 17);
			break;
			
		// Servo values (part 4 : 18 - 23) 
		case HC_MessageType_X8:
			containsData = sendX_ServoValues(18, 23);
			break;
			
		// Servo values (part 5 : 24 - 29)
		case HC_MessageType_X9:
			containsData = sendX_ServoValues(24, 29);
			break;
			
		// Servo values (part 6 : 30 - 35)  
		case HC_MessageType_XA:
			containsData = sendX_ServoValues(30, 35);
			break;
			
		// Servo values (part 7 : 36 - 41)  
		case HC_MessageType_XB:
			containsData = sendX_ServoValues(36, 41);
			break;
			
		// Servo values (part 8 : 42 - 47)  
		case HC_MessageType_XC:
			containsData = sendX_ServoValues(42, 47);
			break;
			
		// AD values (part 1 :  0 - 3)
		case HC_MessageType_XD:
			containsData = sendX_ADValues(0, 3);
			break;

		// AD values (part 2 :  4 - 7)
		case HC_MessageType_XE:
			containsData = sendX_ADValues(4, 7);
			break;

		// AD values (part 3 :  8 - 11)
		case HC_MessageType_XF:
			containsData = sendX_ADValues(8, 11);
			break;
		
		// AD values (part 4 : 12 - 15)
		case HC_MessageType_XG:
			containsData = sendX_ADValues(12, 15);
			break;
								
		// AD values (part 4 : 16 - 19)
		case HC_MessageType_XH:
			containsData = sendX_ADValues(16, 19);
			break;
			

#ifndef HC_USE_FTDI
		// A query *******************************************************

		case HC_MessageType_Aq:
			for (int j = 0; j < mAquery_ARRAY_SIZE; ++j)
			{
				bool noDataQuerried = false;

				#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
				switch (HCI_stringToConcByte(mAquery_type_array[j]))
				#else
				switch (mAquery_type_array[j])
				#endif
				{
					// if no data querried ("")
					case 0:
						noDataQuerried = true;
						break;

					// Free RAM (probe 0-2)
					case HC_MessageType_FR:
						printNumber(HC_sram.getFreeRAM(mAquery_index_array[j]));
						break;

					// Cycle Time (in us)
					case HC_MessageType_CT:
						printNumber(HCS_getCycleTime(), HEX_LENGTH_CYCLETIME);
						break;

					// DI value
					case HC_MessageType_DI:
						printNumber(HC_readDI(mAquery_index_array[j]));
						break;

					// DO value
					case HC_MessageType_DO:
						printNumber(HC_readDO(mAquery_index_array[j]));
						break;

					// AI value
					case HC_MessageType_AI:
						printNumber(HC_readAI(mAquery_index_array[j]), HEX_LENGTH_AI);
						break;

					// PWM value
					case HC_MessageType_PW:
						#if defined ARDUINO_ARCH_SAMD
							printNumber(HC_readPWM(mAquery_index_array[j]), HEX_LENGTH_PWM);
						#else
							printNumber(HC_readPWM(mAquery_index_array[j]));
						#endif
						break;

					// Servo value
					case HC_MessageType_SV:
						printNumber(HC_servoRead(mAquery_index_array[j]), HEX_LENGTH_SERVO);
						break;

					// DD value
					case HC_MessageType_DD:
						printNumber(HC_readDD(mAquery_index_array[j]));
						break;

					// AD value
					case HC_MessageType_AD:
						printFloat(HC_readAD(mAquery_index_array[j]));
						break;

					// DAC value
					#if defined ARDUINO_ARCH_SAMD
					case HC_MessageType_DA:
						printNumber(HC_readDAC(mAquery_index_array[j]), HEX_LENGTH_DAC);
						break;
					#endif
				}

				if (noDataQuerried)
					break;
			}
			break;
#endif


		// Other queries (E, Ec, ...) ********************************************

		// For other Types, message is an acknowledge response
		// => should not end with the Empty Data char (&)
		// => containsData = true;
	}
	

	// if message has Empty Data (finishes with &_)
	if (!containsData)
		printSpecialChar_EmptyData();
	
	// CRC
	printCRC();
	
	// FOOTER: CR + LF
    printFooter();
}


// X query: AI values (part i : min - max)  
bool HC_Protocol::sendX_AIValues(uint8_t min, uint8_t max)
{
	bool containsData = false;

	for (uint8_t j = HCS_getAI_startIndex(); j <= HCS_getAI_endIndex(); ++j)
	{
		if ((min <= j) && (j <= max))
		{
			printNumber(HC_readAI(j), HEX_LENGTH_AI);
			containsData = true;
		}
	}

	return containsData;
}


// X query: AO values (part i : min - max)  
bool HC_Protocol::sendX_AOValues(uint8_t min, uint8_t max)
{ 
	bool containsData = false;
	int counter = 0;

	for (uint8_t j = HCS_getDIO_startIndex(); j <= HCS_getDIO_endIndex(); ++j)
	{
		if (HC_PwmIsActivated(j))
		{
			if ((min <= counter) && (counter <= max))
			{
				#if defined ARDUINO_ARCH_SAMD
					printNumber(HC_readPWM(j), HEX_LENGTH_PWM);
				#else
					printNumber(HC_readPWM(j));
				#endif
				containsData = true;
			}

			// increment counter
			++counter;
		}
	}

	return containsData;
}


// X query: Servo values (part i : min - max)  
bool HC_Protocol::sendX_ServoValues(uint8_t min, uint8_t max)
{
	bool containsData = false;
	int counter = 0;

	for (uint8_t j = HCS_getDIO_startIndex(); j <= HCS_getDIO_endIndex(); ++j)
	{
		// if pin is Servo mode
		if (HC_getServoMode(j))
		{
			if ((min <= counter) && (counter <= max))
			{
				// Servo value in millidegrees
				printNumber(HC_servoRead(j), HEX_LENGTH_SERVO);
				containsData = true;
			}

			// increment counter
			++counter;
		}
	}

	return containsData;
}


// X query: AD values (part i : min - max)  
bool HC_Protocol::sendX_ADValues(uint8_t min, uint8_t max)
{ 
	bool containsData = false;
	int counter = 0;

	for (uint8_t j = 0; j < HC_AD_QTY; ++j)
	{
		// if AD != 0
		if (HCI_getADMask(j))
		{
			if ((min <= counter) && (counter <= max))
			{
				printFloat(HC_readAD(j));
				containsData = true;
			}

			// increment counter
			++counter;
		}
	}

	return containsData;
}


// B Query replies
void HC_Protocol::send_BQuery()
{
    /*
    * - Send all Board features in several messages at highest rate (1 message per cycle)
    */
    if (mBquery_run)
    {
        if (mBquery_ID > 4)
        {
            // reset flag and message ID
            mBquery_ID = 0;
            mBquery_run = false;

			// set first time flag
			mFirstTime = true;
        }
        else
			send_withIndex(mBquery_ID++, HC_MessageType_Bq);
    }
}


// E Query replies
#ifdef HC_EEPROM_COMPILE
void HC_Protocol::send_EQuery()
{ 
	/*
	* - Send all EEPROM values in several messages at highest rate (1 message per cycle)
	* - 4 sets of EEPROM address/value are sent in 1 message.
	*/
	uint8_t setQtyPerMessage = 4;

	if(mEquery_run)
	{
		if(mEquery_ID >= HC_eeprom.getSize())
		{	
			// reset flag and message ID
			mEquery_ID = 0;
			mEquery_run = false;
			
			// notify end of a reply sequence
            send(HC_MessageType_Ee);
		}
		else
		{
			// notify start of a reply sequence
			if(mEquery_ID == 0)	
                send(HC_MessageType_Es);
			
			// send several sets of address/value per messages
			send_withConsecutiveAddresses(mEquery_ID, setQtyPerMessage, HC_MessageType_EE);
		
			// increment message ID
			mEquery_ID += setQtyPerMessage;
		}
	}
}


// EC Query replies
/*void HC_Protocol::send_ECQuery()
{
	// - Send all EEPROM IO Config values in several messages at highest rate (1 message per cycle)
	// - 4 sets of EEPROM address/value are sent in 1 message.
	uint8_t setQtyPerMessage = 4;

	if(mECquery_run)
	{
		if(mECquery_ID >= HC_eeprom.getConfigSpace_Size())
		{
			// reset flag and message ID
			mECquery_ID = 0;
			mECquery_run = false;
			
			// notify end of data transmission
            send(HC_MessageType_Ee);
		}
		else
		{
			// notify start of a reply sequence
			if(mECquery_ID == 0)	
                send(HC_MessageType_Es);
				
			// send several sets of address/value per messages
			send_withConsecutiveAddresses(HC_eeprom.getConfigSpace_Address() + mECquery_ID, setQtyPerMessage, HC_MessageType_EE);
		
			// increment message ID
			mECquery_ID += setQtyPerMessage;
		}
	}
}*/
#endif


// X Query replies
// return true if a message was sent (some messages may be skipped (no data changed...)
bool HC_Protocol::send_XQuery()
{ 
	switch(mXquery_ID)
	{
		case 0: 
			send(HC_MessageType_X0);		// X query period (ms), Cycle Time (us)
			break;

		case 1: 
			if (mFirstTime || HC_sram.hasChanged())
			{
				send(HC_MessageType_M0);	// SRAM
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 2: 
			#if defined(ARDUINO_ARCH_SAMD)
			if (mFirstTime || HCI_PinsMode_hasChanged() || HCI_InputsMode_hasChanged() || HCI_InputsModeOption_hasChanged())
			#else
			if (mFirstTime || HCI_PinsMode_hasChanged() || HCI_InputsMode_hasChanged())
			#endif
			{
				send(HC_MessageType_PM);	// Pin Mode + Input Mode
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 3: 
			if (mFirstTime || HCI_ServosMode_hasChanged())
			{
				send(HC_MessageType_SM);	// Servo Mode
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 4: 
			if (mFirstTime || HCI_OutputTypes_hasChanged())
			{
				send(HC_MessageType_OT);	// Output Type
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 5: 
			if (mFirstTime || HCI_PWMavailability_hasChanged())
			{
				send(HC_MessageType_PA);	// PWM availability
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 6: 
			if (mFirstTime || HCI_ADMask_hasChanged())
			{
				send(HC_MessageType_AM);	// AD mask
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 7:
			#ifdef ARDUINO_ARCH_SAMD
				if (mFirstTime || HCI_DacsMode_hasChanged())
				{
					send(HC_MessageType_DM);// DAC mode (enable mask)
					break;
				}
				else
			#endif
			mXquery_ID++;
			// continue to next case
	
		case 8: 
			send(HC_MessageType_DI);		// DI values
			break;

		case 9: 
			send(HC_MessageType_DO);		// DD values
			break;

		case 10: 
			send(HC_MessageType_DD);		// DD values
			break;

		case 11:
			#ifdef HC_STRINGMESSAGE_COMPILE
				if (mFirstTime || HCI_String_hasChanged())
				{
					send(HC_MessageType_S0);// String
					break;
				}
				else
			#endif
			mXquery_ID++;
			// continue to next case

		case 12:
            send(HC_MessageType_X1);		// AI values (part 1 : 0 - 7)
			break;	
			
		case 13:
			if (HCS_getAI_qty() > 8)
			{
				send(HC_MessageType_X2);	// AI values (part 2 : 8 - 15)	
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 14:
			if (mFirstTime || (HCI_getPWM_qty() > 0))
			{
				send(HC_MessageType_X3);	// PWM values (part 1 : 0 - 7)
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 15:
			if (HCI_getPWM_qty() > 8)
			{
				send(HC_MessageType_X4);	// PWM values (part 2 : 8 - 15)
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 16:
			if (mFirstTime || (HCI_getAttachedServosQty() > 0))
			{
				send(HC_MessageType_X5);	// Servo values (part 1 :  0 - 5)
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 17:
			if(HCI_getAttachedServosQty() > 6)
			{
				send(HC_MessageType_X6);	// Servo values (part 2 :  6 - 11)
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 18:
			if (HCI_getAttachedServosQty() > 12)
			{
				send(HC_MessageType_X7);	// Servo values (part 3 : 12 - 17)
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 19:
			if (HCI_getAttachedServosQty() > 18)
			{
				send(HC_MessageType_X8);	// Servo values (part 4 : 18 - 23) 
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 20:
			if (HCI_getAttachedServosQty() > 24)
			{
				send(HC_MessageType_X9);	// Servo values (part 5 : 24 - 29)
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 21:
			if (HCI_getAttachedServosQty() > 30)
			{
				send(HC_MessageType_XA);	// Servo values (part 6 : 30 - 35)
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 22:
			if (HCI_getAttachedServosQty() > 36)
			{
				send(HC_MessageType_XB);	// Servo values (part 7 : 36 - 41)
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 23:
			if (HCI_getAttachedServosQty() > 42)
			{
				send(HC_MessageType_XC);	// Servo values (part 8 : 42 - 47)
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 24:
			if (mFirstTime || (HCI_getAD_NonNullQty() > 0))
			{
				send(HC_MessageType_XD);	// AD values (part 1 :  0 - 3)
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 25:
			if(HCI_getAD_NonNullQty() > 4)
			{
				send(HC_MessageType_XE);	// AD values (part 2 :  4 - 7)
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 26:
			if (HCI_getAD_NonNullQty() > 8)
			{
				send(HC_MessageType_XF);	// AD values (part 3 :  8 - 11)
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 27:
			if (HCI_getAD_NonNullQty() > 12)
			{
				send(HC_MessageType_XG);	// AD values (part 4 : 12 - 15)
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 28:
			if (HCI_getAD_NonNullQty() > 16)
			{
				send(HC_MessageType_XH);	// AD values (part 5 : 16 - 19)
				break;
			}
			else
				mXquery_ID++;
				// continue to next case

		case 29:
			#ifdef ARDUINO_ARCH_SAMD
				if (HC_readDacsMode())
					send(HC_MessageType_DA);// DAC values (0 - 3)
			#endif
		
			// reset first time flag
			if (mFirstTime)
				mFirstTime = false;
			break;
	}

	// 0 - 29: increment ID
	if (mXquery_ID < mXquery_qty)
		mXquery_ID++;
}

	
// specify 1 index
#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
void HC_Protocol::send_withIndex(uint8_t index, unsigned int messageType)
#else
void HC_Protocol::send_withIndex(uint8_t index, char messageType)
#endif
{
	// reset CRC
	mOutput_CRC = 0;


    // Header 1 (start, 1 char)
	printStartChar(HC_StartChar_diese);

    // Header 2 (message type, 1 or 2 char)
    printMessageType(messageType);

	// Header 3 (Control Byte, 1 char)
	printControlByte((uint8_t) 2);	// have index

	// Header 4 (Index, 2 char)
	printNumber(index);


    // Depending on Message Type ------------------------------------------

	switch (messageType)
	{
		
		// Board features
		case HC_MessageType_Bq:
			switch (index)
			{
				// HITIComm version, Board Features (Name, Processor, Arduino library version, DIO startIndex, DIO endIndex, AI startIndex, AI endIndex, Servo qty max, DD qty, AD qty)
				// if applicable (PWM res, DAC res)
				case 0:
					printNumber(mLibraryVersion, 3);
					printString_withSeparator_P(HCS_getBoard_P());		// max 4 char
					printString_withSeparator_P(HCS_getProcessor_P());	// unknown length
					printNumber(HCS_getArduinoLibVersion(), 5);
					printNumber(HCS_getDIO_startIndex());
					printNumber(HCS_getDIO_endIndex());
					printNumber(HCS_getAI_startIndex());
					printNumber(HCS_getAI_endIndex());
					printNumber(HCS_getServo_qty());
					printNumber((uint8_t) HC_DD_QTY);
					printNumber((uint8_t) HC_AD_QTY);
					#if defined ARDUINO_ARCH_SAMD
						printNumber((uint8_t) HCS_getPwmResolution());
						printNumber((uint8_t) HCS_getDacResolution());
					#endif
					break;

				// Board Features (EEPROM: size, Config Space (size), User Spaces (sizes), String Space Max String size) 
				case 1:
					#ifdef HC_EEPROM_COMPILE
						printNumber(HC_eeprom.getSize(), 4);
						printNumber((unsigned int) 0, 4); //HC_eeprom.getConfigSpace_Size(), 4);
						for (uint8_t i = 0; i < HC_USERSPACE_QTY; ++i)
							printNumber(HC_eeprom.getUserSpace_Size(i), 4);
						printNumber(HC_eeprom.getMaxStringLength());
					#else
						printSpecialChar_EmptyData();
					#endif
					break;

				// Board Features (SRAM : ram start, ram end, data_start, data_end, bss_start, bss_end, heap_start, heap_end, malloc_heap_start, malloc_heap_end, malloc_margin)
				case 2:
					printNumber(HCS_getRamStart());
					printNumber(HCS_getRamEnd());
					printNumber(HC_sram.getAddress_DataStart());
					printNumber(HC_sram.getAddress_DataEnd());
					printNumber(HC_sram.getAddress_BssStart());
					printNumber(HC_sram.getAddress_BssEnd());
					printNumber(HC_sram.getAddress_HeapStart());
					printNumber(HC_sram.getAddress_HeapEnd());
					printNumber(HC_sram.getAddress_MallocHeapStart());
					printNumber(HC_sram.getAddress_MallocHeapEnd());
					printNumber(HC_sram.getMallocMargin());
					break;

				/*
				Peripherals
				  - SPI:   1 byte : SPI0-3   (SPI0:   bit 0/1 => is enabled/is master)
				  - TWI:   1 byte : TWI0-3   (TWI0:   bit 0/1 => is enabled/is master)
				  - USART: 1 byte : USART0-3 (USART0: bit 0/1 => TX/RX is enabled)
				  - I2S:   1 byte : I2S0-3   (I2S0:   bit 0/1 => is enabled/is master)
				*/
				case 3:
					printNumber(HCS_getSpiMode()); 
					printNumber(HCS_getTwiMode());
					printNumber(HCS_getUsartMode());
					printNumber(HCS_getI2SMode());
					break;

				// Project ID
				case 4:
					if (HC_readCodeName() != NULL_POINTER)
						printString_withSeparator_P(HC_readCodeName());	  // unknown length
					else
						printSpecialChar_Separator(true); // "" + Separator

					if (HC_readCodeVersion() != NULL_POINTER)
						printString_withSeparator_P(HC_readCodeVersion()); // unknown length
					else
						printSpecialChar_Separator(true); // "" + separator
					break;
			}
			break;

		// Free RAM
		case HC_MessageType_FR:
			printNumber(HC_sram.getFreeRAM(index));
			break;

		// EEPROM (Config register)
		/*#ifdef HC_EEPROM_COMPILE
			case HC_MessageType_Ep:
				printNumber(HC_eeprom.readConfigRegister(index));
				break;
		#endif*/

		// Pin mode
		case HC_MessageType_PM:
			// pin mode
			printNumber(HC_readPinMode(index));

			// input mode
			printNumber(HC_readInputMode(index));

			#if defined(ARDUINO_ARCH_SAMD)
				// input mode option
				printNumber(HC_readInputModeOption(index));
			#endif
			break;

		// DI mode
		case HC_MessageType_DI:
			printNumber(HC_readDI(index));
			break;

		// DO mode
		case HC_MessageType_DO:
			printNumber(HC_readDO(index));
			break;

		// AI mode
		case HC_MessageType_AI:
			printNumber(HC_readAI(index), HEX_LENGTH_AI);
			break;

		// Output type
		case HC_MessageType_OT:
			printNumber(HC_readOutputType(index));
			break;

		// PWM availability
		case HC_MessageType_PA:
			printNumber(HC_PwmIsAvailable(index));
			break;

		// PWM value
		case HC_MessageType_PW:
			#if defined ARDUINO_ARCH_SAMD
				printNumber(HC_readPWM(index), HEX_LENGTH_PWM);
			#else
				printNumber(HC_readPWM(index));
			#endif
			break;

		// Servo mode
		case HC_MessageType_SM:
			printNumber(HC_readServoMode(index));
			break;

		// Servo value
		case HC_MessageType_SV:
			printNumber(HC_servoRead(index), HEX_LENGTH_SERVO);
			break;

		// DD value
		case HC_MessageType_DD:
			printNumber(HC_readDD(index));
			break;

		// AD mode
		case HC_MessageType_AM:
			printFloat(HCI_getADMask(index));
			break;

		// AD value
		case HC_MessageType_AD:
			printFloat(HC_readAD(index));
			break;

		#ifdef ARDUINO_ARCH_SAMD
		// DAC mode
		case HC_MessageType_DM:
			printNumber(HC_readDacMode(index));
			break;

		// DAC mode
		case HC_MessageType_DA:
			printNumber(HC_readDAC(index), HEX_LENGTH_DAC);
			break;
		#endif

	}
    
	// CRC
	printCRC();

    // FOOTER: CR + LF
    printFooter();
}


#ifdef HC_EEPROM_COMPILE

	// specify 1 address
	#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
	void HC_Protocol::send_withAddress(int unsigned address, unsigned int messageType)
	#else
	void HC_Protocol::send_withAddress(int unsigned address, char messageType)
	#endif
	{
		send_withConsecutiveAddresses(address, 1, messageType);
	}


	// specify several consecutive addresses
	#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
	void HC_Protocol::send_withConsecutiveAddresses(int unsigned start_address, uint8_t qty, unsigned int messageType)
	#else
	void HC_Protocol::send_withConsecutiveAddresses(int unsigned start_address, uint8_t qty, char messageType)
	#endif
	{
		// reset CRC
		mOutput_CRC = 0;


		// Header 1 (start, 1 char)
		printStartChar(HC_StartChar_diese);

		// Header 2 (message type, 1 or 2 char)
		printMessageType(messageType);
	
		// Header 3 (Control Byte, 1 char)
		printControlByte((uint8_t) 4);	// Have address


		for(uint8_t i = 0; i < qty; ++i)
		{
			// Header 4 (address, 4 char)
			printNumber(start_address + i, 4);


			// Depending on Message Type ------------------------------------------

			switch (messageType)
			{
				// EEPROM access (2 char)
				case HC_MessageType_EE:
					printHex(HC_eeprom.basic_readByte(start_address + i));
					break;
			}
		}
    
		// CRC
		printCRC();

		// FOOTER: CR + LF
		printFooter();
	}


	// specify 1 index and 1 address
	#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
	void HC_Protocol::send_withIndex_withAddress(uint8_t index, int unsigned address, unsigned int messageType)
	#else
	void HC_Protocol::send_withIndex_withAddress(uint8_t index, int unsigned address, char messageType)
	#endif
	{
		// reset CRC
		mOutput_CRC = 0;


		// Header 1 (start, 1 char)
		printStartChar(HC_StartChar_diese);

		// Header 2 (message type, 1 or 2 char)
		printMessageType(messageType);

		// Header 3 (Control Byte, 1 char)
		printControlByte((uint8_t) 6);	// Have index, have address

		// Header 4 (Index, 2 char)
		printNumber(index);

		// Header 5 (address, 4 char)
		printNumber(address, 4);


		// Depending on Message Type ------------------------------------------

		switch (messageType)
		{
			// EEPROM access (1 char)
			case HC_MessageType_EE:
				printNumber(HC_eeprom.basic_readBit(address, index));
				break;
		}

		// CRC
		printCRC();

		// FOOTER: CR + LF
		printFooter();
	}

#endif // HC_EEPROM_COMPILE


// notify that Board has started
void HC_Protocol::sendMessage_BoardHasStarted()
{
	// FOOTER: CR + LF (to clear incomplete received message by computer during last connection)
	printFooter();

	send(HC_MessageType_BS);
}