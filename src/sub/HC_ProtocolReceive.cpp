/*
 * HITIComm
 * HC_ProtocolReceive.cpp
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

// HITICommSupport
#include <HCS_LowAccess_IO.h>
#include <HCS_Time.h>
#include <HCS_Serial.h>

// HITIComm
#include "HC_Data.h"
#include "HC_Sram.h"
#include "HC_ServoManager.h"



// ********************************************************************************
// Define
// ********************************************************************************

#define HC_HIGH  1
#define HC_LOW   0

#define HC_IN    0
#define HC_OUT   1
#define HC_IN_PU 2
#define HC_IN_PD 3



// *****************************************************************************
// Variables
// *****************************************************************************


enum StartChar
{
	//HC_StartChar_diese		= 0x23, // #, A->C (normal response)
	//HC_StartChar_exclamation	= 0x21, // !, A->C (error response)
	HC_StartChar_dollar			= 0x24, // $, C->A (query)
};

enum MessageError
{
#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
	HC_MessageError_AM = 0x414D,  // Address Missing
	HC_MessageError_IA = 0x4941,  // Index not Allowed
	HC_MessageError_MT = 0x4D54,  // invalid Message Type
	HC_MessageError_RW = 0x5257,  // invalid Read Write mode
	HC_MessageError_IN = 0x494E,  // invalid INdex
	HC_MessageError_AD = 0x4144,  // invalid ADdress
	HC_MessageError_US = 0x5553,  // Unknown Sender
	HC_MessageError_TS = 0x5453,  // message is Too Short
	HC_MessageError_CS = 0x4353,  // Invalid Checksum
	HC_MessageError_CM = 0x434D,  // Checksum mismatch
	HC_MessageError_IR = 0x4952,  // Index Required
#else
	HC_MessageError_AM = 0x30,  // Address Missing
	HC_MessageError_IA = 0x31,  // Index not Allowed
	HC_MessageError_MT = 0x32,  // invalid Message Type
	HC_MessageError_RW = 0x33,  // invalid Read Write mode
	HC_MessageError_IN = 0x34,  // invalid INdex
	HC_MessageError_AD = 0x35,  // invalid ADdress
	HC_MessageError_US = 0x36,  // Unknown Sender
	HC_MessageError_TS = 0x37,  // message is Too Short
	HC_MessageError_CS = 0x38,  // Invalid Checksum
	HC_MessageError_CM = 0x39,  // Checksum mismatch
	HC_MessageError_IR = 0x3A,  // Index Required
#endif
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


#if defined ARDUINO_ARCH_SAMD
	#define ANALOG_WRITE_MAX_BYTE_COUNT_PWM		3
	#define ANALOG_WRITE_MAX_BYTE_COUNT_DAC		3
#else
	#define ANALOG_WRITE_MAX_BYTE_COUNT_PWM		2
	#define ANALOG_WRITE_MAX_BYTE_COUNT_DAC		2
#endif


// length of hex number to send
#define HEX_LENGTH_CYCLETIME	5  // max 1s
#define HEX_LENGTH_AI			3
#define HEX_LENGTH_SERVO		5
#define HEX_LENGTH_SERVO		5

#if defined ARDUINO_ARCH_SAMD
	#define GET_HEXCOUNT_FROM_BITCOUNT(bitCount)	(bitCount/4 + ((bitCount%4) != 0 ? 1 : 0))
	#define HEX_LENGTH_PWM							(GET_HEXCOUNT_FROM_BITCOUNT(HCS_getPwmResolution()))
	#define HEX_LENGTH_DAC							(GET_HEXCOUNT_FROM_BITCOUNT(HCS_getDacResolution()))
#else
	#define HEX_LENGTH_PWM		2
#endif



// *****************************************************************************
// HC_PROTOCOL : Receive Message
// *****************************************************************************


void HC_Protocol::receive()
{
	static uint8_t input_rawIndex = 0;

    // if data received, process and reply to computer
    if(HCS_Serial_isAvailable() > 0)
    {	
		while(HCS_Serial_isAvailable() > 0)
		{
			// read a char and record it in the array
			mInput[input_rawIndex] = HCS_Serial_read();

			// CR detected
			if (mInput[input_rawIndex] == '\r')
			{
				mInput_EndDetectionFlag = 1;
				mInput[input_rawIndex] = '\0';
			}
			// CR then LF detected: message is complete
			else if((mInput_EndDetectionFlag == 1) && (mInput[input_rawIndex] == '\n'))
			{
				mInput_EndDetectionFlag = 2;
				mInput[input_rawIndex] = '\0';

				// analyze message
				analyzeInput();

				// clear buffer
				//for (uint8_t i = 0; i < HC_INPUTMESSAGE_MAX_ARRAY_LENGHT; ++i)
				uint8_t i = HC_INPUTMESSAGE_MAX_ARRAY_LENGHT;
				while(i)
					mInput[--i] = 0;
			}
			else
				// CR or (CR then LF) not detected
				mInput_EndDetectionFlag = 0;

			// if message end detected
			if(mInput_EndDetectionFlag == 2)
			{
				// reset flag
				mInput_EndDetectionFlag = 0;

				// reset index
				input_rawIndex = 0;
			}
			// increment index only if possible. If max is reached, value at index max
			// will be overwritten every cycle until CR + LF are found
			else if (input_rawIndex < HC_INPUTMESSAGE_MAX_ARRAY_LENGHT - 1)
				++input_rawIndex;
		}
    }
	// if no data received, process E, EC, A, X Queries
	else
	{
#ifndef HC_USE_FTDI
		bool send_Xreply = false;
		bool send_Areply = false;
#endif

        // if B Query being processed (B Query has priority on all other Queries)
		if (mBquery_run)
			// execute "B" reply at highest rate (every cycle) until all B queries have been sent
			send_BQuery();


#ifdef HC_EEPROM_COMPILE
		// if E Query being processed (E Query has priority on EC, A, X Queries)
		else if (mEquery_run)
			// execute "E" reply at highest rate (every cycle) until all EEPROM values are sent
			send_EQuery();

		// if Ec Query being processed (EC Query has priority on A, X Queries)
		/*else if(mECquery_run)
			// execute "EC" reply at highest rate (every cycle) until all EEPROM values are sent
			send_ECQuery();*/
#endif


		// if X Query subscription
		else if (mXquery_run)
		{
			// start/run timer (50ms)
			mXquery_timer.run(50);

			// 50ms is the min X query period. 2 cases can happen:
			//    1) X reply sequence finishes execution before 50ms => X reply sequence restarts when timer is over (X query ID is reset, timer is reset)
			//    2) X reply sequence finishes execution after 50ms  => X reply sequence restarts as soon as possible
			bool XsequenceIsExecuting = (mXquery_ID < mXquery_qty);

			if (mXquery_timer.isOver() && !XsequenceIsExecuting)
			{
				// restart reply sequence
				mXquery_ID = 0;

				// reset timer
				mXquery_timer.reset();
			}

#ifdef HC_USE_FTDI
			// Boards using FTDI chip:
			// These boards pack messages into packets and send 1 packet every 40ms at best. 
			// Use of A reply increases packet size and decreases packet sent rate. For these boards:
			//    => Do not use A reply
			//    => Sampling rate in HITIPanel is limited to 50ms.

			if (XsequenceIsExecuting)
				send_XQuery();
		}
#else
			// Boards not using FTDI chip
			// These boards pack messages into packets and send 1 packet every 4ms at best. 
			// For these boards:
			//    => Use A reply and limit sent message every 2ms
			//    => Sampling rate in HITIPanel is limited to 10ms.

			if (XsequenceIsExecuting)
			{
				// For timing optimization: only one Query (X or A) is processed per cycle
				// When both queries have to be sent at same time, one is sent every 2 cycles using a semaphore
				if (mAquery_run)
				{
					if (mSemaphor)
						send_Xreply = true;
					else
						send_Areply = true;
				}
				else
					send_Xreply = true;
			}
			else if (mAquery_run)
				send_Areply = true;
		}

		// if A Query subscription only
		else if (mAquery_run && !mXquery_run)
			// send A reply at highest rate (every cycle)
			send_Areply = true;


		// if X reply must be sent
		if (send_Xreply)
		{
			send_XQuery();
			mSemaphor = false;
		}

		// if A reply must be sent
		else if(send_Areply)
		{
			// only send reply every 2ms
			if (mAquery_timer.delay(2))
				send(HC_MessageType_Aq);
	
			mSemaphor = true;
		}
#endif
	}
}



void HC_Protocol::analyzeInput()
{
	mInput_length = strlen(mInput);
	mInput_index = 0;

	bool message_isCorrect = true;
	

	// if no CRC, remove ending separator, if any ------------------------------
	#ifndef PROTOBF_USE_CRC
		if (mInput_length > 0)
		{
			if (mInput[mInput_length - 1] == SpecialChar_separator[0])
			{
				mInput[mInput_length - 1] = '\0';
				--mInput_length;
			}
		}
	#endif


	// Is the message long enough ? --------------------------------------------

	// Min length : StartString + MessageType + ConfigByte (hex, always 1 char only) + CRC(hex)
	uint8_t minLength = 3;						// Ex : $aB

	#ifdef PROTOBF_USE_SEPARATOR			// Ex : $_a_B
		minLength += 2;
	#endif

	#ifdef PROTOBF_USE_READABLE_MESSAGETYPE // Ex : $PmB
		++minLength;
	#endif

	#ifdef PROTOBF_USE_CRC					// Ex : $_a_B_C or $aBCS
		minLength += 2;
	#endif

	message_isCorrect = (mInput_length >= minLength);


    // if the message is still correct
    if(message_isCorrect)
    {          


		// Does the message starts with the Start string ? ---------------------
	
		#ifdef PROTOBF_USE_SEPARATOR
			#ifdef HC_DISPLAY_INPUT_IN_ERRORMESSAGE
				// copy input message before tokenizing
				char copy[HC_INPUTMESSAGE_MAX_ARRAY_LENGHT];
				strcpy(copy, mInput);

				// get first token (contains Start Char)
				mInput_data = strtok(copy, SpecialChar_separator);
			#else
				mInput_data = strtok(mInput, SpecialChar_separator);
			#endif

			message_isCorrect = (HCI_stringToConcByte(mInput_data) == HC_StartChar_dollar);
		#else
			if (mInput_length > 0)
				message_isCorrect = (mInput[mInput_index++] == HC_StartChar_dollar);
		#endif


		// if the message is still correct
		if (message_isCorrect)
		{
			// Is the CRC valid ? ----------------------------------------------

			#ifdef PROTOBF_USE_CRC
				uint8_t readCRC = 0;
				mInput_CRC_index = mInput_length - 2;
				char CRC_str[3] = { 0 };


				#ifdef PROTOBF_USE_SEPARATOR
					if (mInput[mInput_CRC_index] == SpecialChar_separator[0])
						// case: _C
						CRC_str[0] = mInput[++mInput_CRC_index];
					else
					{
						// case: CS
						CRC_str[0] = mInput[mInput_CRC_index];
						CRC_str[1] = mInput[mInput_CRC_index + 1];
					}
				#else
					CRC_str[0] = mInput[mInput_CRC_index];
					CRC_str[1] = mInput[mInput_CRC_index + 1];
				#endif


				// chars must be valid hex values ('0'->'9' : 0x30->0x39 and 'A'->'F' : 0x41->0x46)
				// else return 0
				readCRC = hexStringToULong(CRC_str);

				if(!readCRC)
				{
					#ifdef PROTOBF_USE_SEPARATOR
						if (strlen(CRC_str) == 1)
							message_isCorrect = (CRC_str[0] == 0x30);
						else
							message_isCorrect = (CRC_str[0] == 0x30) || (CRC_str[1] == 0x30);
					#else
						message_isCorrect = (CRC_str[0] == 0x30) || (CRC_str[1] == 0x30);
					#endif
				}
			#endif


			// if the message is still correct
			if (message_isCorrect)
			{
				// Does the read and calculated CRC match ? ------------------------

				#ifdef PROTOBF_USE_CRC
					unsigned int sum = 0;

					// calculate CRC (CRC value not included in calculation)
					uint8_t i = mInput_CRC_index;
					while(i)
						sum += mInput[--i];

					// check matching
					if(readCRC != HCS_getLowByte(sum))
						message_isCorrect = false;
				#endif


				// if the message is still correct
				if (message_isCorrect)
				{
					// Is the Message Type valid ? -------------------------------------

					// read Message Type
					#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
						unsigned int message_type;
					#else
						char message_type;
					#endif

					#ifdef PROTOBF_USE_SEPARATOR
						// get next token
						mInput_data = strtok(NULL_POINTER, SpecialChar_separator);

						if (mInput_data != 0)
						{
							message_type = HCI_stringToConcByte(mInput_data);

							message_isCorrect = messageTypeIsValid(message_type);
						}
						else
							message_isCorrect = false;
					#else
						#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
							// 2 chars
							char message_type_str[3];
							message_type_str[0] = mInput[mInput_index++];
							message_type_str[1] = mInput[mInput_index++];
							message_type_str[2] = '\0';
							message_type = HCI_stringToConcByte(message_type_str);

							message_isCorrect = messageTypeIsValid(message_type);
						#else
							// 1 char
							message_type = mInput[mInput_index++];

							message_isCorrect = messageTypeIsValid(message_type);
						#endif
					#endif


					// 	if the message is still correct
					if (message_isCorrect)
					{
						// Config Byte ------------------------------------------------------
			
						uint8_t configByte;

						#ifdef PROTOBF_USE_SEPARATOR
							// get next token
							mInput_data = strtok(NULL_POINTER, SpecialChar_separator);

							if (mInput_data != 0)
							{
								if (strlen(mInput_data) == 1)
									configByte = hexStringToULong(mInput_data);
								else
									message_isCorrect = false;
							}
							else
								message_isCorrect = false;
						#else
							mInput_data[0] = mInput[mInput_index++];
							mInput_data[1] = '\0';
							configByte = hexStringToULong(mInput_data);
						#endif


						// if the message is still correct
						if(message_isCorrect)
						{  
							// Are index or address specified ? -----------------------------

							bool ReadWriteMode		= HCS_readBit(configByte, 0); // index 0, Read(0) / Write(1)
							bool indexSpecified		= HCS_readBit(configByte, 1); // index 1
							bool addressSpecified	= HCS_readBit(configByte, 2); // index 2


							// Look for Target Index ----------------------------------------

							uint8_t index = 0;

							if (indexSpecified)
							{
								if (nextToken(2))
									index = stringToULong(mInput_data);
								else
									message_isCorrect = false;
							}


							// if the message is still correct
							if (message_isCorrect)
							{
								// Look for Target Address ----------------------------------

								#ifdef HC_EEPROM_COMPILE
									unsigned int address = 0;
								#endif

								if (addressSpecified)
								{
									if (nextToken(4))
										#ifdef HC_EEPROM_COMPILE
											address = stringToULong(mInput_data);
										#else
											message_isCorrect = true;
										#endif
									else
										message_isCorrect = false;
								}


								// if the message is still correct
								if (message_isCorrect)
								{		
									// 	Target index NOT specified --------------------------

									if(!indexSpecified)
									{	
										// EEPROM access specific variables
										#ifdef HC_EEPROM_COMPILE
											bool setAll = false;
											bool clearAll = false;
										#endif


										// in Write Mode ***************************************

										if(ReadWriteMode)
										{			
											#if HC_VARIANT == HC_VARIANT_MEGA
												unsigned long var1_L = 0;
												unsigned long var1_H = 0;
												unsigned long var2_L = 0;
												unsigned long var2_H = 0;
											#elif defined(ARDUINO_ARCH_SAMD)
												unsigned long var1_L = 0;
												unsigned long var2_L = 0;
												unsigned long var3_L = 0;
											#else
												unsigned long var1_L = 0;
												unsigned long var2_L = 0;
											#endif      
												
											switch(message_type)
											{
												// HITI String
												#ifdef HC_STRINGMESSAGE_COMPILE
													case HC_MessageType_S0:

														nextToken();
														HC_writeString(mInput_data);
														break;
												#endif

												#ifdef HC_EEPROM_COMPILE
													// EEPROM access
													case HC_MessageType_EE:
								
														if (addressSpecified)
														{
															// get cell value
															nextToken(2);
															HC_eeprom.basic_writeByte(address, hexStringToULong(mInput_data));
														}
														else
														{
															// set or clear all EEPROM values
															nextToken(1);
															if (hexStringToULong(mInput_data) == 0)
																clearAll = true;
															else
																setAll = true;
														}
														break;

													// EEPROM (IO Config)
													case HC_MessageType_Ec:

														/*nextToken(1);
														switch (hexStringToULong(mInput_data))
														{
															case 0:
																// save current IO Config to EEPROM
																HC_eeprom.saveIOConfig();
																break;

															case 1:
																// load	IO Config from EEPROM
																HC_eeprom.loadIOConfig();
																break;

															case 2:
																// clear IO Config in EEPROM
																HC_eeprom.clearIOConfig();
																break;
														}*/
														break;

													// EEPROM (Config Register)
													case HC_MessageType_Ep:

														/*nextToken(2);
														HC_eeprom.writeConfigRegister((uint8_t)hexStringToULong(mInput_data));*/
														break;
												#endif


												// Pin Mode
												case HC_MessageType_PM:
													
													#if HC_VARIANT == HC_VARIANT_MEGA
														nextToken(8);
														var1_H = hexStringToULong(mInput_data);

														nextToken(8);
														var1_L = hexStringToULong(mInput_data);

														nextToken(8);
														var2_H = hexStringToULong(mInput_data);

														nextToken(8);
														var2_L = hexStringToULong(mInput_data);

														HC_pinsMode(var1_H, var1_L, var2_H, var2_L);
													#elif defined(ARDUINO_ARCH_SAMD)
														nextToken(8);
														var1_L = hexStringToULong(mInput_data);

														nextToken(8);
														var2_L = hexStringToULong(mInput_data);

														nextToken(8);
														var3_L = hexStringToULong(mInput_data);

														HC_pinsMode(var1_L, var2_L, var3_L);
													#else
														nextToken(8);
														var1_L = hexStringToULong(mInput_data);

														nextToken(8);
														var2_L = hexStringToULong(mInput_data);

														HC_pinsMode(var1_L, var2_L);
													#endif
													break;

												
												// DO values
												case HC_MessageType_DO:

													#if HC_VARIANT == HC_VARIANT_MEGA
														nextToken(8);
														var1_H = hexStringToULong(mInput_data);

														nextToken(8);
														var1_L = hexStringToULong(mInput_data);

														HC_writeDO(var1_H, var1_L);
													#else
														nextToken(8);
														HC_writeDO(hexStringToULong(mInput_data));
													#endif		
													break;

												// Output Type
												case HC_MessageType_OT:

													#if HC_VARIANT == HC_VARIANT_MEGA
														nextToken(8);
														var1_H = hexStringToULong(mInput_data);

														nextToken(8);
														var1_L = hexStringToULong(mInput_data);

														HC_outputTypes(var1_H, var1_L);
													#else
														nextToken(8);
														HC_outputTypes(hexStringToULong(mInput_data));
													#endif		
													break;

												// Servo mode
												case HC_MessageType_SM:

													#if HC_VARIANT == HC_VARIANT_MEGA
														nextToken(8);
														var1_H = hexStringToULong(mInput_data);

														nextToken(8);
														var1_L = hexStringToULong(mInput_data);

														HC_servosMode(var1_H, var1_L);
													#else
														nextToken(8);
														HC_servosMode(hexStringToULong(mInput_data));
													#endif  
													break;

												// DD values
												case HC_MessageType_DD:

													nextToken(8);
													HC_writeDD(hexStringToULong(mInput_data));
													break;

												#ifdef ARDUINO_ARCH_SAMD
												// DAC mode
												case HC_MessageType_DM:

													nextToken(2);
													HC_dacsMode((uint8_t)hexStringToULong(mInput_data));
													break;
												#endif
											}
										}
                    
					
										// specific actions (trigger events, send messages...) *********

										#ifdef HC_EEPROM_COMPILE

										// EEPROM access
										if(message_type == HC_MessageType_EE)
										{
											// 1 value
											if (addressSpecified)
												send_withAddress(address, message_type);

											// All values (E query)
											else
											{				
												// Request sending all EEPROM values in several messages
												// auto-reset after last message is sent
												mEquery_run = true;

												// send message to computer
												// (if write mode: acknowledge first before starting next long lasting task)
												send(message_type);

												// if write mode
												if(ReadWriteMode)
												{
													// set or clear all EEPROM values
													if (setAll)
														HC_eeprom.setAll();
													else if(clearAll)
														HC_eeprom.clearAll();
												}
											}
										}
                    
										// For all other messages
										else
										{

										#endif	// HC_EEPROM_COMPILE

											// A query array index
											uint8_t Aquery_index = 0;
											uint8_t j;

											switch (message_type)
											{
												// Bf query
												case HC_MessageType_Bf:
													// Request sending all Board Data in several messages
													// auto-reset after last message is sent
													mBquery_run = true;
													break;

												// Ec query
												#ifdef HC_EEPROM_COMPILE
													case HC_MessageType_Ec:
														// Request sending all EEPROM IO Config values in several messages
														// auto-reset after last message is sent
														mECquery_run = true;
														break;
												#endif

												// X query: start
												case HC_MessageType_Xs:
													// Request start sending X Data continuously
													mXquery_run = true;

													// reset Xquery ID
													mXquery_ID = 0;
													break;

												// X query: stop
												case HC_MessageType_Xu:
													// Request stop sending all X Data
													mXquery_run = false;
													break;

												// A query: start
												case HC_MessageType_As:
													// Request start sending A Data continuously
													mAquery_run = true;

													// clear arrays
													j = mAquery_ARRAY_SIZE;
													//for (int j = 0; j < mAquery_ARRAY_SIZE; j++)
													while(j)
													{
														// Message Type
														#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
															strcpy(mAquery_type_array[--j], "");
														#else
															mAquery_type_array[--j] = '\0';
														#endif

														// Index
														mAquery_index_array[j] = 0;
													}

													while (nextToken(1)) // Message Type: 1 byte if no separator
													{
														// if too much data, don't retrieve data
														if (Aquery_index < mAquery_ARRAY_SIZE)
														{
															// Message Type
															#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
																strcpy(mAquery_type_array[Aquery_index], mInput_data);
															#else
																mAquery_type_array[Aquery_index] = mInput_data[0];
															#endif
															
															// Index
															nextToken(2);
															mAquery_index_array[Aquery_index] = stringToULong(mInput_data);
															++Aquery_index;
														}
														else
															break;
													}
													break;

												// A query: stop
												case HC_MessageType_Au:
													// Request stop sending all A Data
													mAquery_run = false;
													break;
											}

											// send message
											switch (message_type)
											{
												// Error: index required
												case HC_MessageType_AI:
												case HC_MessageType_PW:
												case HC_MessageType_SV:
												case HC_MessageType_AD:
												#if defined ARDUINO_ARCH_SAMD
												case HC_MessageType_DA:
												#endif
													// Error message: Index Required
													printMessageError(HC_MessageError_IR);
													break;

												default:
													send(message_type);
													break;
											}
										#ifdef HC_EEPROM_COMPILE
										}
										#endif
									}				


									// 	Target index specified ---------------------------------

									else
									{	
										// in Write Mode ***************************************
					
										if(ReadWriteMode)
										{
											switch (message_type)
											{
												#ifdef HC_EEPROM_COMPILE
													// EEPROM access
													case HC_MessageType_EE:
														// Check that the byte address has been specified
														if (addressSpecified)
														{
															nextToken(1);
															HC_eeprom.basic_writeBit(address, index, stringToBool(mInput_data));
														}
														break;
					
													// EEPROM (Config Param)
													case HC_MessageType_Ep:
														/*nextToken(1);
														HC_eeprom.writeConfigRegister(index, stringToBool(mInput_data));*/
														break;
												#endif

												// Pin Mode
												case HC_MessageType_PM:
													nextToken(1);
													if (stringToBool(mInput_data))
														HCS_pinMode(index, HC_OUT);
													else
													{
														nextToken(1);
														if (stringToBool(mInput_data))
														{
															#if defined(ARDUINO_ARCH_SAMD)
																nextToken(1);
																HCS_pinMode(index, stringToBool(mInput_data) ? HC_IN_PU : HC_IN_PD);
															#else
																HCS_pinMode(index, HC_IN_PU);
															#endif
														}
														else
															HCS_pinMode(index, HC_IN);
													}

													// for input with attached servo: detach servo
													if (!HC_readPinMode(index) && HC_readServoMode(index))
														HC_servoMode(index, false);

													// update output
													HCI_updateOutput(index);
													break;

												// DO value
												case HC_MessageType_DO:
													nextToken(1);
													HC_writeDO(index, stringToBool(mInput_data));
													break;
										
												// Output type
												case HC_MessageType_OT:
													nextToken(1);
													HC_outputType(index, stringToBool(mInput_data));
													break;

												// PWM value
												case HC_MessageType_PW:
													nextToken(HEX_LENGTH_PWM);
													HC_writePWM(index, stringToUInt(mInput_data));
													break;

												// Servo mode
												case HC_MessageType_SM:
													nextToken(1);
													HC_servoMode(index, stringToBool(mInput_data));
													break;

												// Servo value
												case HC_MessageType_SV:
													nextToken(HEX_LENGTH_SERVO);
													HC_servoWrite(index, stringToULong(mInput_data));
													break;

												// DD value
												case HC_MessageType_DD:
													nextToken(1);
													HC_writeDD(index, stringToBool(mInput_data));
													break;

												// AD value
												case HC_MessageType_AD:
													nextToken(8);
													HC_writeAD(index, stringToFloat(mInput_data));
													break;

												#ifdef ARDUINO_ARCH_SAMD
												// DAC mode
												case HC_MessageType_DM:
													nextToken(1);
													HC_dacMode(index, stringToBool(mInput_data));
													break;

												// DAC value
												case HC_MessageType_DA:
													nextToken(HEX_LENGTH_DAC);
													HC_writeDAC(index, stringToFloat(mInput_data));
													break;
												#endif
											}
										}


										// specific actions (trigger events, send messages...) *********
					
										#ifdef HC_EEPROM_COMPILE

										// EEPROM access
										if(message_type == HC_MessageType_EE)
										{
											// Check that the byte address has been specified
											if(addressSpecified)
												// send message to computer specifying bit index and byte address
												send_withIndex_withAddress(index, address, message_type);
											else
												// Error message: Address Missing
												printMessageError(HC_MessageError_AM);
										}
					
										// For all other messages accepting an index
										else
										{
										#endif	// HC_EEPROM_COMPILE

											switch (message_type)
											{
												case HC_MessageType_Bf:
												#ifdef HC_EEPROM_COMPILE
												case HC_MessageType_Ep:
												#endif
												case HC_MessageType_PM:
												case HC_MessageType_DI:
												case HC_MessageType_DO:
												case HC_MessageType_AI:
												case HC_MessageType_OT:
												case HC_MessageType_PA:
												case HC_MessageType_PW:
												case HC_MessageType_SM:
												case HC_MessageType_SV:
												case HC_MessageType_DD:
												case HC_MessageType_AM:
												case HC_MessageType_AD:
												#ifdef ARDUINO_ARCH_SAMD
												case HC_MessageType_DM:
												case HC_MessageType_DA:
												#endif					
													send_withIndex(index, message_type);
													break;

												default:
													// Error message: Index not Allowed
													printMessageError(HC_MessageError_IA);
													break;
											}
										#ifdef HC_EEPROM_COMPILE
										}
										#endif
									}
								}
								else
									// Error message: invalid ADdress
									printMessageError(HC_MessageError_AD);
							}
							else
								// Error message: invalid INdex
								printMessageError(HC_MessageError_IN);
						}
						else
							// Error message: invalid Read Write mode
							printMessageError(HC_MessageError_RW);
					}
					else
						// Error message: invalid Message Type
						printMessageError(HC_MessageError_MT);
				}
				else
					// Error message: CRC mismatch
					printMessageError(HC_MessageError_CM);
			}
			else
				// Error message: invalid CRC
				printMessageError(HC_MessageError_CS);
		}
        else
			// Error message: Unknown Sender
			printMessageError(HC_MessageError_US);
    }
    else
		// Error message: message is too short
		printMessageError(HC_MessageError_TS);
}


#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
bool HC_Protocol::messageTypeIsValid(unsigned int messageType)
#else
bool HC_Protocol::messageTypeIsValid(char messageType)
#endif
{
	switch (messageType)
	{
		case HC_MessageType_Bf:

		case HC_MessageType_M0:
		case HC_MessageType_FR:

		case HC_MessageType_CT:

		#ifdef HC_ARDUINOTIME_COMPILE
			case HC_MessageType_TM:
		#endif

		#ifdef HC_STRINGMESSAGE_COMPILE
			case HC_MessageType_S0:
		#endif

		#ifdef HC_EEPROM_COMPILE
			case HC_MessageType_EE:
			case HC_MessageType_Ec:
			case HC_MessageType_Ep:
		#endif

		case HC_MessageType_PM:
		case HC_MessageType_DI:
		case HC_MessageType_DO:
		case HC_MessageType_AI:
		case HC_MessageType_OT:
		case HC_MessageType_PA:
		case HC_MessageType_PW:
		case HC_MessageType_SM:
		case HC_MessageType_SV:
		case HC_MessageType_DD:
		case HC_MessageType_AM:
		case HC_MessageType_AD:
		#ifdef ARDUINO_ARCH_SAMD
			case HC_MessageType_DM:
			case HC_MessageType_DA:
		#endif

		case HC_MessageType_Xs:
		case HC_MessageType_Xu:

		case HC_MessageType_As:
		case HC_MessageType_Au:
			return true;
	}

	return false;
}


// qty: considered only if no separator
bool HC_Protocol::nextToken(uint8_t qty)
{
	#ifndef PROTOBF_USE_SEPARATOR
		#ifdef PROTOBF_USE_CRC
		if (mInput_CRC_index >= mInput_index + qty)
		#else
		if (mInput_length >= mInput_index + qty)
		#endif
		{
			// get substring and append '\0'
			strncpy(mInput_data, &mInput[mInput_index], qty);
			mInput_data[qty] = '\0';

			// update index
			mInput_index += qty;
			return true;
		}
		else
			return false;
	#else
		// get next token
		mInput_data = strtok(NULL_POINTER, SpecialChar_separator);
		return (mInput_data != NULL_POINTER);
	#endif
}

bool HC_Protocol::nextToken()
{
	#ifdef PROTOBF_USE_CRC
		return nextToken((mInput_CRC_index - mInput_index));
	#else
		return nextToken((mInput_length - mInput_index));
	#endif
}



// --------------------------------------------------------------------------------
// communicate --------------------------------------------------------------------
// --------------------------------------------------------------------------------

// Receive and send message to Computer software
void HC_Protocol::communicate()
{
    // calculate cycle time
    HCS_calculateCycleTime();

    // measure SRAM on probe 0 (measurement used in X query)
    HC_sram.setProbe(0);

	// receive new message
	receive();
}
