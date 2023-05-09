/*
 * HITIComm
 * HC_Eeprom.cpp
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


#include "HC_Eeprom.h"



// *****************************************************************************
// Compilation trigger
// *****************************************************************************

#ifdef HC_EEPROM_COMPILE



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// AVR
#include <avr/eeprom.h>
#include <string.h>

// HITICommSupport
#include "HITICommSupport.h"

// HITIComm
#include "HC_Toolbox.h"



// *****************************************************************************
// Define
// *****************************************************************************

// Eeprom size
#if defined(ARDUINO_ARCH_MEGAAVR)
	#define TOTAL_SIZE					EEPROM_SIZE
#else
	#define TOTAL_SIZE					E2END + 1
#endif

// Config Space size
// Pin mode					: 2 long = 8 bytes
// Input mode				: 2 long = 8 bytes
// Output type				: 2 long = 8 bytes
// Servo mode				: 2 long = 8 bytes
// Config flags register	: 1 byte
//   - bit 0: "Config Flag" : true if config has been saved
#define CONFIG_PINMODE_SIZE			8
#define CONFIG_INPUTMODE_SIZE		8
#define CONFIG_OUTPUTTYPE_SIZE		8
#define CONFIG_SERVOMODE_SIZE		8
#define CONFIG_CONFIGREGISTER_SIZE	1
#define CONFIG_MARGIN				11	// reserved for future use
#define CONFIG_SIZE (CONFIG_PINMODE_SIZE + CONFIG_INPUTMODE_SIZE + \
							   CONFIG_OUTPUTTYPE_SIZE + CONFIG_SERVOMODE_SIZE + \
							   CONFIG_CONFIGREGISTER_SIZE + CONFIG_MARGIN)

#define CONFIG_ADDRESS					(TOTAL_SIZE - CONFIG_SIZE)
#define CONFIG_PINMODE_ADDRESS			(CONFIG_ADDRESS)
#define CONFIG_INPUTMODE_ADDRESS		(CONFIG_PINMODE_ADDRESS		+ CONFIG_PINMODE_SIZE)
#define CONFIG_OUTPUTTYPE_ADDRESS		(CONFIG_INPUTMODE_ADDRESS	+ CONFIG_INPUTMODE_SIZE)
#define CONFIG_SERVOMODE_ADDRESS		(CONFIG_OUTPUTTYPE_ADDRESS	+ CONFIG_OUTPUTTYPE_SIZE)
#define CONFIG_CONFIGREGISTER_ADDRESS	(CONFIG_SERVOMODE_ADDRESS	+ CONFIG_SERVOMODE_SIZE)



// *****************************************************************************
// Instanciates an object so that it can be reused in other files (forward declared in .h)
// *****************************************************************************

HC_Eeprom HC_eeprom;



// *****************************************************************************
// Class Methods
// *****************************************************************************

	// constructor -------------------------------------------------------------

	HC_Eeprom::HC_Eeprom()
	{
		// Config Space is disabled by default
		removeIOConfigSpace();

		// User spaces
		setUserSpace(
				50,		// Boolean space
				100,	// Byte space
				200,	// Integer space
				250,	// Long space
				250,	// Float space
				4096);	// String space (set to a high value to fill the remaining User Space))

		// String Space: max string length (does not include the terminating char '\0')
		mStringSpace_maxStringLength = HC_EEPROM_STRING_ABSOLUTEMAXSIZE;
	}
	

	
	// setters -----------------------------------------------------------------

	void HC_Eeprom::addIOConfigSpace()
	{
		mConfigSpaceIsEnabled = true;

		// update space
		updateUserSpace();
	}
	
	void HC_Eeprom::removeIOConfigSpace()
	{
		mConfigSpaceIsEnabled = false;

		// update space
		updateUserSpace();
	}

	void HC_Eeprom::setUserSpace(
			unsigned int boolean_space_size,
			unsigned int byte_space_size,
			unsigned int integer_space_size,
			unsigned int long_space_size,
			unsigned int float_space_size,
			unsigned int string_space_size)
	{
		// set sizes
		mSpaces_Size[0] = boolean_space_size;
		mSpaces_Size[1] = byte_space_size;
		mSpaces_Size[2] = integer_space_size;
		mSpaces_Size[3] = long_space_size;
		mSpaces_Size[4] = float_space_size;
		mSpaces_Size[5] = string_space_size;
		
		// update space
		updateUserSpace();
	}


	void HC_Eeprom::updateUserSpace()
	{
		// limit space sizes
		unsigned int max_space_size = getSize() - (mConfigSpaceIsEnabled ? CONFIG_SIZE : 0);

		for (uint8_t i = 0; i < HC_USERSPACE_QTY; ++i)
		{
			if (mSpaces_Size[i] >= max_space_size)
			{
				mSpaces_Size[i] = max_space_size;

				// set following spaces sizes to 0
				if (i < (HC_USERSPACE_QTY - 1))
				{
					for (uint8_t j = i + 1; j < HC_USERSPACE_QTY; ++j)
						mSpaces_Size[j] = 0;
				}
				break;
			}

			max_space_size -= mSpaces_Size[i];
		}

		// set addresses
		mSpaces_Address[0] = 0;
		for (uint8_t i = 1; i < 6; ++i)
			mSpaces_Address[i] = mSpaces_Address[i - 1] + mSpaces_Size[i - 1];
	}


	void HC_Eeprom::setMaxStringLength(uint8_t length)
	{
		// limit String size
		mStringSpace_maxStringLength = (length > HC_EEPROM_STRING_ABSOLUTEMAXSIZE) ? HC_EEPROM_STRING_ABSOLUTEMAXSIZE : length;
	}



	// getters -----------------------------------------------------------------

	unsigned int HC_Eeprom::getSize() const									{ return TOTAL_SIZE; }
	//unsigned int HC_Eeprom::getConfigSpace_Address() const					{ return CONFIG_ADDRESS; }
	//unsigned int HC_Eeprom::getConfigSpace_Size() const						{ return mConfigSpaceIsEnabled ? CONFIG_SIZE : 0; }
	unsigned int HC_Eeprom::getUserSpace_Address(HC_UserSpace space) const	{ return mSpaces_Address[space]; }
	unsigned int HC_Eeprom::getUserSpace_Address(uint8_t i) const			{ return mSpaces_Address[i]; }
	unsigned int HC_Eeprom::getUserSpace_Size(HC_UserSpace space) const		{ return mSpaces_Size[space]; }
	unsigned int HC_Eeprom::getUserSpace_Size(uint8_t i) const				{ return mSpaces_Size[i]; }
	unsigned int HC_Eeprom::getUserSpace_DataQty(HC_UserSpace space) const
	{
		switch (space)
		{
			case HC_USERSPACE_BOOLEAN:
				return mSpaces_Size[0] * 8;

			case HC_USERSPACE_BYTE:
				return mSpaces_Size[1];

			case HC_USERSPACE_INTEGER:
				return mSpaces_Size[2] / 2;

			case HC_USERSPACE_LONG:
				return mSpaces_Size[3] / 4;

			case HC_USERSPACE_FLOAT:
				return mSpaces_Size[4] / 4;

			case HC_USERSPACE_STRING:
				return mSpaces_Size[5] / mStringSpace_maxStringLength;

			default:
				return 0;
		}
	}
	unsigned int HC_Eeprom::getUserSpace_DataQty(uint8_t i) const			{ return getUserSpace_DataQty(i); }
	uint8_t HC_Eeprom::getMaxStringLength()									{ return mStringSpace_maxStringLength; }



	// read/write : Basic --------------------------------------------------

	// read/write : Bit (inside 1 register) *****************************
	void HC_Eeprom::basic_writeBit(unsigned int address, uint8_t index, bool value)
	{
		// check address range
		if (address < getSize())
			space_writeBit(1, address, index, value);
	}

	bool HC_Eeprom::basic_readBit(unsigned int address, uint8_t index)
	{
		// check address range
		if (address < getSize())
			return space_readBit(1, address, index);
		else
			return 0;
	}


	// read/write : Byte ************************************************
	void HC_Eeprom::basic_writeByte(unsigned int address, uint8_t data)
	{
		// check address range
		if(address < getSize())
			eeprom_update_byte((uint8_t*) address, data);
	}
	
	uint8_t HC_Eeprom::basic_readByte(unsigned int address)
	{ 
		// check address range
		if(address < getSize())
			return eeprom_read_byte((uint8_t*) address);
		else
			return 0;
	}


	// read/write : Word ************************************************
	// - 2 consecutive bytes (Low to High)
	void HC_Eeprom::basic_writeWord(unsigned int address, uint16_t data)
	{
		// check address range
		if (address + 1 < getSize())
			eeprom_update_word((uint16_t*) address, data);
	}

	uint16_t HC_Eeprom::basic_readWord(unsigned int address)
	{
		// check address range
		if (address + 1 < getSize())
			return eeprom_read_word((uint16_t*)address);
		else
			return 0;
	}


	// read/write : DWord ***********************************************
	// - 4 consecutive bytes (Low to High)
	void HC_Eeprom::basic_writeDWord(unsigned int address, uint32_t data)
	{
		// check address range
		if (address + 3 < getSize())
			eeprom_update_dword((uint32_t*)address, data);
	}
	
	uint32_t HC_Eeprom::basic_readDWord(unsigned int address)
	{
		// check address range
		if (address + 3 < getSize())
			return eeprom_read_dword((uint32_t*)address);
		else
			return 0;
	}


	// read/write : Float ***********************************************
	// - 4 consecutive bytes (Low to High)
	void HC_Eeprom::basic_writeFloat(unsigned int address, float data)
	{
		// check address range
		if (address + 3 < getSize())
			eeprom_update_float((float*) address, data);
	}

	float HC_Eeprom::basic_readFloat(unsigned int address)
	{
		// check address range
		if (address + 3 < getSize())
			return eeprom_read_float((float*)address);
		else
			return 0.0;
	}


	// read/write : String **********************************************
	
	// don't write the terminating char '\0'
	void HC_Eeprom::basic_writeString(unsigned int address, char* str)
	{
		// mStringSpace_maxStringLength and strlen() both don't include the terminating char '\0'

		// limit String length
		if (strlen(str) > mStringSpace_maxStringLength)
			str[mStringSpace_maxStringLength] = '\0';

		// check address range
		if (address + strlen(str) - 1 < getSize())
			eeprom_update_block(str, (void*)address, strlen(str));
	}

	// stringLength does NOT include the terminating character '\0'
	char* HC_Eeprom::basic_readString(unsigned int address, unsigned int stringLength)
	{
		static char readString[HC_EEPROM_STRING_ABSOLUTEMAXSIZE + 1];

		// limit string length
		if (stringLength > HC_EEPROM_STRING_ABSOLUTEMAXSIZE)
			stringLength = HC_EEPROM_STRING_ABSOLUTEMAXSIZE;

		// check address range
		if (address + stringLength - 1 < getSize())
		{
			eeprom_read_block(readString, (void*)address, stringLength);
			readString[stringLength] = '\0';
			return readString;
		}
		else
			return 0;
	}



	// read/write : Memory Space -------------------------------------------

	// read/write : Bit ************************************************
	// if index > 8, next addresses are reached
	void HC_Eeprom::space_writeBit(unsigned int spaceSize, unsigned int start_address, unsigned int index, bool value)
	{
		// find at which address (starting from start_address) the supplied index is
		unsigned int address = index / 8;

		// find at which position at this address the supplied index is
		uint8_t indexAtAddress = index % 8;

		// check address range
		if ((address < spaceSize) && (start_address + address < getSize()))
		{
			// read byte value
			uint8_t byteValue = basic_readByte(start_address + address);

			// check if bit value needs being updated
			if (value != HCS_readBit(byteValue, indexAtAddress))
			{
				// update bit value
				HCS_writeBit(byteValue, indexAtAddress, value);

				// write new byte value
				basic_writeByte(start_address + address, byteValue);
			}
		}
	}

	bool HC_Eeprom::space_readBit(unsigned int spaceSize, unsigned int start_address, unsigned int index)
	{
		// find at which address (starting from start_address) the supplied index is
		unsigned int address = index / 8;

		// find at which position at this address the supplied index is
		uint8_t indexAtAddress = index % 8;

		// check address range
		if ((address < spaceSize) && (start_address + address < getSize()))
		{
			// read byte value
			uint8_t byteValue = basic_readByte(start_address + address);

			// read bit value
			return HCS_readBit(byteValue, indexAtAddress);
		}

		return 0;
	}


	// read/write : Byte ************************************************
	void HC_Eeprom::space_writeByte(unsigned int spaceSize, unsigned int start_address, unsigned int index, uint8_t data)
	{
		// check address range
		if ((index < spaceSize) && (start_address + index < getSize()))
			basic_writeByte(start_address + index, data);
	}

	unsigned int HC_Eeprom::space_readByte(unsigned int spaceSize, unsigned int start_address, unsigned int index)
	{
		// check address range
		if ((index < spaceSize) && (start_address + index < getSize()))
			return basic_readByte(start_address + index);

		return 0;
	}


	// read/write : Word ************************************************
	void HC_Eeprom::space_writeWord(unsigned int spaceSize, unsigned int start_address, unsigned int index, uint16_t data)
	{
		// find at which address (starting from start_address) the supplied index is
		unsigned int address = index*2;

		// check address range
		if ((address + 1 < spaceSize) && (start_address + address + 1 < getSize()))
			basic_writeWord(start_address + address, data);
	}

	uint16_t HC_Eeprom::space_readWord(unsigned int spaceSize, unsigned int start_address, unsigned int index)
	{
		// find at which address (starting from start_address) the supplied index is
		unsigned int address = index * 2;

		// check address range
		if ((address + 1 < spaceSize) && (start_address + address + 1 < getSize()))
			return basic_readWord(start_address + address);

		return 0;
	}


	// read/write : DWord ***********************************************
	void HC_Eeprom::space_writeDWord(unsigned int spaceSize, unsigned int start_address, unsigned int index, uint32_t data)
	{
		// find at which address (starting from start_address) the supplied index is
		unsigned int address = index * 4;

		// check address range
		if ((address + 3 < spaceSize) && (start_address + address + 3 < getSize()))
			basic_writeDWord(start_address + address, data);
	}

	uint32_t HC_Eeprom::space_readDWord(unsigned int spaceSize, unsigned int start_address, unsigned int index)
	{
		// find at which address (starting from start_address) the supplied index is
		unsigned int address = index * 4;

		// check address range
		if ((address + 3 < spaceSize) && (start_address + address + 3 < getSize()))
			return basic_readDWord(start_address + address);

		return 0;
	}


	// read/write : Float ***********************************************
	void HC_Eeprom::space_writeFloat(unsigned int spaceSize, unsigned int start_address, unsigned int index, float data)
	{
		// find at which address (starting from start_address) the supplied index is
		unsigned int address = index * 4;

		// check address range
		if ((address + 3 < spaceSize) && (start_address + address + 3 < getSize()))
			basic_writeFloat(start_address + address, data);
	}

	float HC_Eeprom::space_readFloat(unsigned int spaceSize, unsigned int start_address, unsigned int index)
	{
		// find at which address (starting from start_address) the supplied index is
		unsigned int address = index * 4;

		// check address range
		if ((address + 3 < spaceSize) && (start_address + address + 3 < getSize()))
			return basic_readFloat(start_address + address);

		return 0.0;
	}


	// read/write : String **********************************************
	void HC_Eeprom::space_writeString(unsigned int spaceSize, unsigned int start_address, unsigned int index, char* str)
	{
		// find at which address (starting from start_address) the supplied index is
		unsigned int address = index * mStringSpace_maxStringLength;

		// mStringSpace_maxStringLength and strlen() both don't include the terminating char '\0'

		// limit String length
		if (strlen(str) > mStringSpace_maxStringLength)
			str[mStringSpace_maxStringLength] = '\0';

		// check address range
		if ((address + mStringSpace_maxStringLength - 1 < spaceSize) && (start_address + address + mStringSpace_maxStringLength - 1 < getSize()))
			basic_writeString(start_address + address, str);
	}

	char* HC_Eeprom::space_readString(unsigned int spaceSize, unsigned int start_address, unsigned int index, unsigned int stringSize)
	{
		// find at which address (starting from start_address) the supplied index is
		unsigned int address = index * mStringSpace_maxStringLength;

		// check address range
		if ((address + mStringSpace_maxStringLength - 1 < spaceSize) && (start_address + address + mStringSpace_maxStringLength - 1 < getSize()))
			return basic_readString(start_address + address, stringSize);

		return 0;
	}



	// read/write : User Space ---------------------------------------------

	// read/write : Boolean space
	void HC_Eeprom::writeBoolean(unsigned int index, bool value)
	{
		space_writeBit(mSpaces_Size[0], mSpaces_Address[0], index, value);
	}

	uint8_t HC_Eeprom::readBoolean(unsigned int index)
	{
		return space_readBit(mSpaces_Size[0], mSpaces_Address[0], index);
	}


	// read/write : Byte space
	void HC_Eeprom::writeByte(unsigned int index, uint8_t value)
	{
		space_writeByte(mSpaces_Size[1], mSpaces_Address[1], index, value);
	}

	uint8_t HC_Eeprom::readByte(unsigned int index)
	{
		return space_readByte(mSpaces_Size[1], mSpaces_Address[1], index);
	}


	// read/write : Integer space
	void HC_Eeprom::writeInteger(unsigned int index, int value)
	{
		space_writeWord(mSpaces_Size[2], mSpaces_Address[2], index, value);
	}

	int HC_Eeprom::readInteger(unsigned int index)
	{
		return space_readWord(mSpaces_Size[2], mSpaces_Address[2], index);
	}


	// read/write : Long space
	void HC_Eeprom::writeLong(unsigned int index, long value)
	{
		space_writeDWord(mSpaces_Size[3], mSpaces_Address[3], index, value);
	}

	long HC_Eeprom::readLong(unsigned int index)
	{
		return space_readDWord(mSpaces_Size[3], mSpaces_Address[3], index);
	}


	// read/write : Float space
	void HC_Eeprom::writeFloat(unsigned int index, float value)
	{
		space_writeFloat(mSpaces_Size[4], mSpaces_Address[4], index, value);
	}

	float HC_Eeprom::readFloat(unsigned int index)
	{
		return space_readFloat(mSpaces_Size[4], mSpaces_Address[4], index);
	}


	// read/write : String space
	void HC_Eeprom::writeString(unsigned int index, char* value)
	{
		space_writeString(mSpaces_Size[5], mSpaces_Address[5], index, value);
	}

	char* HC_Eeprom::readString(unsigned int index)
	{
		return space_readString(mSpaces_Size[5], mSpaces_Address[5], index, mStringSpace_maxStringLength);
	}



	// read/write : All ----------------------------------------------------

	void HC_Eeprom::setAll()
	{
		//for (unsigned int i = 0; i < getSize(); ++i)
		unsigned int i = getSize();
		while (i)
			eeprom_update_byte((uint8_t*)(--i), 255);
	}

	void HC_Eeprom::clearAll()
	{
		//for (unsigned int i = 0; i < getSize(); ++i)
		unsigned int i = getSize();
		while (i)
			eeprom_update_byte((uint8_t*)(--i), 0);
	}


	// read/write : Full config ***************************************
	/*void HC_Eeprom::saveIOConfig()
	{
		savePinMode();
		saveOutputType();
		saveServoMode();

		// set flag
		writeDetectionFlag(true);
	}

	void HC_Eeprom::loadIOConfig()
	{
		loadPinMode();
		loadOutputType();
		loadServoMode();
	}


	void HC_Eeprom::clearIOConfig()
	{
		if (mConfigSpaceIsEnabled)
		{
			for (unsigned int i = CONFIG_ADDRESS; i < getSize(); ++i)
				eeprom_update_byte((uint8_t*)i, 0);
		}

		// reset flag
		writeDetectionFlag(false);
	}


	// read/write : Config Register *********************************
	void HC_Eeprom::writeConfigRegister(uint8_t value)
	{
		if (mConfigSpaceIsEnabled)
			basic_writeByte(CONFIG_CONFIGREGISTER_ADDRESS, value);
	}

	uint8_t HC_Eeprom::readConfigRegister()
	{
		return mConfigSpaceIsEnabled ? basic_readByte(CONFIG_CONFIGREGISTER_ADDRESS) : 0;
	}

	void HC_Eeprom::writeConfigRegister(uint8_t index, bool value)
	{
		// Space size = 1 byte (8 indexes)
		if (mConfigSpaceIsEnabled)
			basic_writeBit(CONFIG_CONFIGREGISTER_ADDRESS, index, value);
	}

	bool HC_Eeprom::readConfigRegister(uint8_t index)
	{
		// Space size = 1 byte (8 indexes)
		return mConfigSpaceIsEnabled ? basic_readBit(CONFIG_CONFIGREGISTER_ADDRESS, index) : 0;
	}

	void HC_Eeprom::writeDetectionFlag(bool value)
	{
		if (mConfigSpaceIsEnabled)
			writeConfigRegister(0, value);
	}

	bool HC_Eeprom::readDetectionFlag()
	{
		return mConfigSpaceIsEnabled ? readConfigRegister(0) : 0;
	}

	bool HC_Eeprom::IOConfigDetected()
	{
		return readDetectionFlag();
	}


	// Config read/write ---------------------------------------------------
	
	// read/write : Pin mode/Input mode *********************************
	#if HC_VARIANT == HC_VARIANT_MEGA
		void HC_Eeprom::savePinMode()
		{
			if (mConfigSpaceIsEnabled)
			{
				basic_writeDWord(CONFIG_PINMODE_ADDRESS, HC_readPinsMode_L());
				basic_writeDWord(CONFIG_PINMODE_ADDRESS + sizeof(long), HC_readPinsMode_H());
				basic_writeDWord(CONFIG_INPUTMODE_ADDRESS, HC_readInputsMode_L());
				basic_writeDWord(CONFIG_INPUTMODE_ADDRESS + sizeof(long), HC_readInputsMode_H());
			}
		}
		
		void HC_Eeprom::loadPinMode() 
		{
			if (mConfigSpaceIsEnabled)
			{
				HC_pinsMode(
					basic_readDWord(CONFIG_PINMODE_ADDRESS + sizeof(long)),
					basic_readDWord(CONFIG_PINMODE_ADDRESS),
					basic_readDWord(CONFIG_INPUTMODE_ADDRESS + sizeof(long)),
					basic_readDWord(CONFIG_INPUTMODE_ADDRESS));
			}
		}
	#else
		void HC_Eeprom::savePinMode()           
		{
			if (mConfigSpaceIsEnabled)
			{
				basic_writeDWord(CONFIG_PINMODE_ADDRESS, HC_readPinsMode());
				basic_writeDWord(CONFIG_INPUTMODE_ADDRESS, HC_readInputsMode());
			}
		}
		
		void HC_Eeprom::loadPinMode()
		{
			if (mConfigSpaceIsEnabled)
			{
				HC_pinsMode(
					basic_readDWord(CONFIG_PINMODE_ADDRESS),
					basic_readDWord(CONFIG_INPUTMODE_ADDRESS));
			}
		}
	#endif
		
	
	// read/write : Output Type ****************************************
	#if HC_VARIANT == HC_VARIANT_MEGA
		void HC_Eeprom::saveOutputType()
		{
			if (mConfigSpaceIsEnabled)
			{
				basic_writeDWord(CONFIG_OUTPUTTYPE_ADDRESS, HC_readOutputTypes_L());
				basic_writeDWord(CONFIG_OUTPUTTYPE_ADDRESS + sizeof(long), HC_readOutputTypes_H());
			}
		}
		
		void HC_Eeprom::loadOutputType() 
		{
			if (mConfigSpaceIsEnabled)
			{
				HC_outputTypes(
					basic_readDWord(CONFIG_OUTPUTTYPE_ADDRESS + sizeof(long)),
					basic_readDWord(CONFIG_OUTPUTTYPE_ADDRESS));
			}
		}
	#else
		void HC_Eeprom::saveOutputType()           
		{
			if (mConfigSpaceIsEnabled)
				basic_writeDWord(CONFIG_OUTPUTTYPE_ADDRESS, HC_readOutputTypes());
		}
		
		void HC_Eeprom::loadOutputType()
		{
			if (mConfigSpaceIsEnabled)
				HC_outputTypes(basic_readDWord(CONFIG_OUTPUTTYPE_ADDRESS));
		}
	#endif
	

	// read/write : Servo mode ****************************************
	#if HC_VARIANT == HC_VARIANT_MEGA
		void HC_Eeprom::saveServoMode()
		{
			if (mConfigSpaceIsEnabled)
			{
				basic_writeDWord(CONFIG_SERVOMODE_ADDRESS, HC_readServosMode_L());
				basic_writeDWord(CONFIG_SERVOMODE_ADDRESS + sizeof(long), HC_readServosMode_H());
			}
		}
		
		void HC_Eeprom::loadServoMode() 
		{
			if (mConfigSpaceIsEnabled)
			{
				HC_servosMode(
					basic_readDWord(CONFIG_SERVOMODE_ADDRESS + sizeof(long)),
					basic_readDWord(CONFIG_SERVOMODE_ADDRESS));
			}
		}
	#else
		void HC_Eeprom::saveServoMode()           
		{
			if (mConfigSpaceIsEnabled)
				basic_writeDWord(CONFIG_SERVOMODE_ADDRESS, HC_readServosMode());
		}
		
		void HC_Eeprom::loadServoMode()
		{
			if (mConfigSpaceIsEnabled)
				HC_servosMode(basic_readDWord(CONFIG_SERVOMODE_ADDRESS));
		}
	#endif
	*/
	
#endif // HC_EEPROM_COMPILE