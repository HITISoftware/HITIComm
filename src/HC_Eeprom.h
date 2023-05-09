/* 
 * HITIComm
 * HC_Eeprom.h
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

#ifndef HC_EEPROM_h
#define HC_EEPROM_h



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HITICommSupport.h>

// HITIComm
#include "sub\HC_CompilationTriggers.h"
#include "HC_Enum.h"



// *****************************************************************************
// Define
// *****************************************************************************

// String Space : absolute max String size (includes end char '\0')
#define HC_EEPROM_STRING_ABSOLUTEMAXSIZE		30


#ifdef HC_EEPROM_COMPILE
// *****************************************************************************
// Class
// *****************************************************************************

class HC_Eeprom //: public HC_EepromInterface
{
	public:
		// constructor ---------------------------------------------------------
		HC_Eeprom();
		
		// setters -------------------------------------------------------------
		void addIOConfigSpace();
		void removeIOConfigSpace();
		void setUserSpace(
			unsigned int boolean_space_size,
			unsigned int byte_space_size,
			unsigned int integer_space_size,
			unsigned int long_space_size,
			unsigned int float_space_size,
			unsigned int string_space_size);
		void setMaxStringLength(uint8_t length);

		// getters -------------------------------------------------------------
		unsigned int getSize() const;
		//unsigned int getConfigSpace_Address() const;
		//unsigned int getConfigSpace_Size() const;
		unsigned int getUserSpace_Address(HC_UserSpace space) const;
		unsigned int getUserSpace_Address(uint8_t i) const;
		unsigned int getUserSpace_Size(HC_UserSpace space) const;
		unsigned int getUserSpace_Size(uint8_t i) const;
		unsigned int getUserSpace_DataQty(HC_UserSpace space) const;
		unsigned int getUserSpace_DataQty(uint8_t i) const;
		uint8_t getMaxStringLength();


		// read/write : Basic --------------------------------------------------

		// read/write : Bit (inside 1 register)
		void basic_writeBit(unsigned int address, uint8_t index, bool value);
		bool basic_readBit(unsigned int address, uint8_t index);

		// read/write : Byte
		void basic_writeByte(unsigned int address, uint8_t data);
		uint8_t basic_readByte(unsigned int address);

		// read/write : Word
		void basic_writeWord(unsigned int address, uint16_t data);
		uint16_t basic_readWord(unsigned int address);

		// read/write : DWord
		void basic_writeDWord(unsigned int address, uint32_t data);
		uint32_t basic_readDWord(unsigned int address);

		// read/write : Float
		void basic_writeFloat(unsigned int address, float data);
		float basic_readFloat(unsigned int address);

		// read/write : String
		void basic_writeString(unsigned int address, char* str);
		char* basic_readString(unsigned int address, unsigned int stringLength);


		// read/write : User Space ---------------------------------------------

		// read/write : Boolean Space
		void writeBoolean(unsigned int index, bool value);
		uint8_t readBoolean(unsigned int index);

		// read/write : Byte Space
		void writeByte(unsigned int index, uint8_t value);
		uint8_t readByte(unsigned int index);

		// read/write : Integer Space
		void writeInteger(unsigned int index, int value);
		int readInteger(unsigned int index);

		// read/write : Long Space
		void writeLong(unsigned int index, long value);
		long readLong(unsigned int index);

		// read/write : Float Space
		void writeFloat(unsigned int index, float value);
		float readFloat(unsigned int index);

		// read/write : String Space
		void writeString(unsigned int index, char* value);
		char* readString(unsigned int index);


		// read/write : All -----------------------------------------------------

		void setAll();
		void clearAll();

		
		// read/write : I/O Config ----------------------------------------------

		// read/write : full config
		/*void saveIOConfig();
		void loadIOConfig();
		void clearIOConfig();

		// read/write : Config Register
		void writeConfigRegister(uint8_t value);
		uint8_t readConfigRegister();
		void writeConfigRegister(uint8_t index, bool value);
		bool readConfigRegister(uint8_t index);

		// read/write : Detection Flag
		void writeDetectionFlag(bool value);
		bool readDetectionFlag();
		bool IOConfigDetected();*/
		

	protected:
	private:
		// User spaces ---------------------------------------------------------
		void updateUserSpace();


		// read/write : Memory Space -------------------------------------------

		// read/write : Bit
		// if index > 8, next addresses are reached
		void space_writeBit(unsigned int spaceSize, unsigned int start_address, unsigned int index, bool value);
		bool space_readBit(unsigned int spaceSize, unsigned int start_address, unsigned int index);

		// read/write : Byte
		void space_writeByte(unsigned int spaceSize, unsigned int start_address, unsigned int index, uint8_t data);
		unsigned int space_readByte(unsigned int spaceSize, unsigned int start_address, unsigned int index);

		// read/write : Word
		void space_writeWord(unsigned int spaceSize, unsigned int start_address, unsigned int index, uint16_t data);
		uint16_t space_readWord(unsigned int spaceSize, unsigned int start_address, unsigned int index);

		// read/write : Word
		void space_writeDWord(unsigned int spaceSize, unsigned int start_address, unsigned int index, uint32_t data);
		uint32_t space_readDWord(unsigned int spaceSize, unsigned int start_address, unsigned int index);

		// read/write : Word
		void space_writeFloat(unsigned int spaceSize, unsigned int start_address, unsigned int index, float data);
		float space_readFloat(unsigned int spaceSize, unsigned int start_address, unsigned int index);

		// read/write : String
		void space_writeString(unsigned int spaceSize, unsigned int start_address, unsigned int index, char* str);
		char* space_readString(unsigned int spaceSize, unsigned int start_address, unsigned int index, unsigned int stringSize);


		// read/write : Config --------------------------------------------------

		// read/write : Pin mode/Input mode
		/*void savePinMode();
		void loadPinMode();

		// read/write : Output type
		void saveOutputType();
		void loadOutputType();

		// read/write : Servo mode
		void saveServoMode();
		void loadServoMode();*/


		// Variables ------------------------------------------------------------

		// Config Space
		bool mConfigSpaceIsEnabled = true;

		// User Spaces:
		// 0 : Boolean
		// 1 : Byte
		// 2 : Integer
		// 3 : Long
		// 4 : Float
		// 5 : String
		unsigned int mSpaces_Address[HC_USERSPACE_QTY];
		unsigned int mSpaces_Size[HC_USERSPACE_QTY];

		// String space
		uint8_t mStringSpace_maxStringLength; //max String length (does not include the terminating char '\0')
};



// *****************************************************************************
// Forward declare a class object
// *****************************************************************************

extern HC_Eeprom HC_eeprom;


#endif // HC_EEPROM_COMPILE
#endif // HC_EEPROM_h
