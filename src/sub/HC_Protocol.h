/*
 * HITIComm
 * HC_Protocol.h
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

#ifndef HC_Protocol_h
#define HC_Protocol_h



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HITICommSupport.h>

// HITIComm
#include "HC_Timer.h"
#include "HC_Eeprom.h"



// *****************************************************************************
// Enable/Disable features
// *****************************************************************************


// Protocol features
#define PROTOBF_OPTIONS_FULLY_READABLE	0 // int, separator, readable
#define PROTOBF_OPTIONS_USE_INT			1 // int, separator
#define PROTOBF_OPTIONS_USE_SEPARATOR	2 // hex, separator,          CRC
#define PROTOBF_OPTIONS_HEX				3 // hex,                     CRC


// select an option
#define PROTOBF_OPTION	PROTOBF_OPTIONS_HEX

#if   PROTOBF_OPTION == PROTOBF_OPTIONS_FULLY_READABLE
	#define PROTOBF_USE_READABLE_MESSAGETYPE
	#define PROTOBF_USE_SEPARATOR
	#define PROTOBF_USE_INT
#elif PROTOBF_OPTION == PROTOBF_OPTIONS_USE_INT
	#define PROTOBF_USE_SEPARATOR
	#define PROTOBF_USE_INT
#elif PROTOBF_OPTION == PROTOBF_OPTIONS_USE_SEPARATOR
	#define PROTOBF_USE_SEPARATOR
	#define PROTOBF_USE_CRC
#elif PROTOBF_OPTION == PROTOBF_OPTIONS_HEX
	#define PROTOBF_USE_CRC
	#define PROTOBF_FILL_HEX
#endif

#define HC_DISPLAY_INPUT_IN_ERRORMESSAGE // if not defined, decrease memory size (64 bytes)
//#define HC_DISPLAY_X_QUERY_PERIOD



// *****************************************************************************
// Class
// *****************************************************************************

class HC_Protocol
{
    public:
	    // constructor
		HC_Protocol():
				mXquery_timer() // X query
				#ifndef HC_USE_FTDI
				, mAquery_timer()   // A query
				#endif
		{
			mXquery_timer.manualReset(); // manual reset
		}

        // Send message : Board has started
        void sendMessage_BoardHasStarted();

        // Communicate with computer : receive, process, send message
        void communicate();


    protected:
    private:
		// ---------------------------------------------------------------------
		// Variables -----------------------------------------------------------
		// ---------------------------------------------------------------------

		unsigned int mLibraryVersion = 125;	// 1.2.5
		
		// protocol
		const char SpecialChar_separator[2] = "_";


		// CRC -----------------------------------------------------------------
		unsigned int mOutput_CRC;
		uint8_t mInput_CRC_index;


        // Received Message: Array Buffer --------------------------------------
		#define HC_INPUTMESSAGE_MAX_ARRAY_LENGHT 40
		char mInput[HC_INPUTMESSAGE_MAX_ARRAY_LENGHT];
		uint8_t mInput_index;	// analyzed input
		uint8_t mInput_length;	// analyzed input

		#ifdef PROTOBF_USE_SEPARATOR
			char* mInput_data;	// analyzed input
		#else
			char mInput_data[HC_EEPROM_STRING_ABSOLUTEMAXSIZE];	// analyzed input
		#endif

        // Received Message: Ending detection (CR then LF) ---------------------
        // 0: no ending character detected
        // 1: CR detected
        // 2: CR then LF detected
        uint8_t mInput_EndDetectionFlag = 0;


		// Queries -------------------------------------------------------------
		// counters (message ID)
		uint8_t mBquery_ID;
		uint8_t mXquery_ID;
		#ifdef HC_EEPROM_COMPILE
			unsigned int mEquery_ID;
			unsigned int mECquery_ID;
		#endif

		// flags (= true if query is being processed)
		bool mBquery_run = false;
		bool mXquery_run = false;
		bool mAquery_run = false;
		#ifdef HC_EEPROM_COMPILE
			bool mEquery_run = false;
			bool mECquery_run = false;
		#endif

		// flags (= true if Xquery is forced to send all messages)
		bool mFirstTime = true;

		// Semaphore used to manage A and X query process when both are running
		// For timing optimization: only one Query is processed per cycle
		// = false : process A query
		// = true : process X query
		bool mSemaphor;


		// X query -------------------------------------------------------------

		#define mXquery_qty 30
		HC_Timer mXquery_timer;	// delay (50ms), manual reset. Used to force X replies period to be higher than 50ms

		#ifdef HC_DISPLAY_X_QUERY_PERIOD
			long unsigned mXquery_previousTimestamp;	// used to calculate the X replies period (time required to send all X replies)
		#endif


		// A query -------------------------------------------------------------

		// arrays of data index and type (max size = 4 data)
		#define mAquery_ARRAY_SIZE 4
		uint8_t mAquery_index_array[mAquery_ARRAY_SIZE];
		#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
			char mAquery_type_array[mAquery_ARRAY_SIZE][3];
		#else
			char mAquery_type_array[mAquery_ARRAY_SIZE];
		#endif

		#ifndef HC_USE_FTDI
			HC_Timer mAquery_timer;		// delay (2ms), auto reset. Used to limit A replies period to 2ms.
		#endif


		// ---------------------------------------------------------------------
		// Output --------------------------------------------------------------
		// ---------------------------------------------------------------------

		#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
			void send(unsigned int messageType);
		#else
			void send(char messageType);
		#endif

		// X query
		bool sendX_AIValues(uint8_t min, uint8_t max);	// AI values	(part i : min - max)  
		bool sendX_AOValues(uint8_t min, uint8_t max);	// AO values	(part i : min - max)  
		bool sendX_ServoValues(uint8_t min, uint8_t max);	// Servo values (part i : min - max)  
		bool sendX_ADValues(uint8_t min, uint8_t max);	// AD values    (part i : min - max)  

		// B,E,EC,X queries replies
		void send_BQuery();
		bool send_XQuery();
		#ifdef HC_EEPROM_COMPILE
			void send_EQuery();
			//void send_ECQuery();
		#endif

		#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
			void send_withIndex(uint8_t index, unsigned int messageType);											// specify 1 index
			#ifdef HC_EEPROM_COMPILE
				void send_withAddress(int unsigned address, unsigned int messageType);								// specify 1 address
				void send_withConsecutiveAddresses(int unsigned start_address, uint8_t qty, unsigned int messageType);	// specify several consecutive addresses
				void send_withIndex_withAddress(uint8_t index, int unsigned address, unsigned int messageType);		// specify 1 index and 1 address
			#endif
		#else
			void send_withIndex(uint8_t index, char messageType);											// specify 1 index
			#ifdef HC_EEPROM_COMPILE
				void send_withAddress(int unsigned address, char messageType);								// specify 1 address
				void send_withConsecutiveAddresses(int unsigned start_address, uint8_t qty, char messageType);	// specify several consecutive addresses
				void send_withIndex_withAddress(uint8_t index, int unsigned address, char messageType);		// specify 1 index and 1 address
			#endif
		#endif


		// ---------------------------------------------------------------------
		// Input ---------------------------------------------------------------
		// ---------------------------------------------------------------------

		void receive();
		void analyzeInput();

		#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
			bool messageTypeIsValid(unsigned int messageType);
		#else
			bool messageTypeIsValid(char messageType);
		#endif
		bool nextToken(uint8_t qty);
		bool nextToken();


		// ---------------------------------------------------------------------
		// Serial print --------------------------------------------------------
		// ---------------------------------------------------------------------

		// Footer (CR + LF)
		void printFooter();

		// char
		void printChar(char c);

		// Concatenated Bytes to String
		#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
			void printConcBytesToString(unsigned int hex, uint8_t stringLength);
		#endif

		// Special characters
		void printSpecialChar_Separator(bool forcePrinting);
		void printSpecialChar_Separator();
		void printSpecialChar_EmptyData(bool forcePrintingSeparator);
		void printSpecialChar_EmptyData();

		// String
		void printString_withSeparator_P(const char* pgm_str);
		void printString(const char* str, bool forcePrintSeparator);
		void printString(const char* str);
		void printString_withSeparator(const char* str);

		// Start characters
		void printStartChar(char StartChar);

		// Message Type/Error
		#ifdef PROTOBF_USE_READABLE_MESSAGETYPE
			void printMessageType(unsigned int messageType);
			void printMessageError(unsigned int errorCode);
		#else
			void printMessageType(char messageType);
			void printMessageError(char errorCode);
		#endif

		// Control Byte
		void printControlByte(uint8_t b);

		// CRC
		void printCRC();
		void addCRC(unsigned long l);
		
		// Print number in Hex format
		void printHex(bool value);
		void printHex(uint8_t value, bool fillWithZero, bool printedValueIsCRC);
		void printHex(uint8_t value, bool fillWithZero);
		void printHex(uint8_t value);
		void printHex(unsigned int value, uint8_t maxLengthWithZeros);
		void printHex(unsigned int value);

		void printHex(unsigned long value, uint8_t maxLengthWithZeros);
		void printHex(unsigned long value);

		// Print number to Hex or Decimal format (depending on options)
		void printNumber(bool number);
		void printNumber(uint8_t number);
		void printNumber(unsigned int number, uint8_t maxHexLengthWithZeros);
		void printNumber(unsigned int number);
		void printNumber(unsigned long number, uint8_t maxHexLengthWithZeros);
		void printNumber(unsigned long number);

		// Print Float to Hex or Decimal format (depending on options)
		void printFloat(float number);


		// ---------------------------------------------------------------------
		// String to Number ----------------------------------------------------
		// ---------------------------------------------------------------------

		// Hex format string to unsigned Long
		unsigned long hexStringToULong(char* str);

		// string to Boolean
		bool stringToBool(char* str);

		// Decimal format string to Unsigned Int/Long
		unsigned long stringToULong(char* str);
		unsigned int stringToUInt(char* str);

		// Decimal format string to float
		float stringToFloat(char* str);
};


#endif
