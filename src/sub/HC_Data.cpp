/*
 * HITIComm
 * HC_Data.cpp
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



#include "HC_Data.h"



// *****************************************************************************
// Includes
// *****************************************************************************

// HITICommSupport
#include <HCS_LowAccess_IO.h>
#include <HCS_Time.h>

// HITIComm
#include "HC_Toolbox.h"
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
// variables
// *****************************************************************************


// Project ID (pointer in Progmem)
static const char* g_pgm_projectName = NULL_POINTER;
static const char* g_pgm_projectVersion = NULL_POINTER;


// Previous Pins mode (used to detect data changes)
#if HC_VARIANT == HC_VARIANT_MEGA
    // 2 unsigned long (2 x 32 bit = 64 bit)
    static long unsigned g_pinsMode_previous_L = 0;
    static long unsigned g_pinsMode_previous_H = 0;
#else
    // unsigned long (32 bit)
    static long unsigned g_pinsMode_previous = 0;
#endif  

// Previous Inputs mode (used to detect data changes)
#if HC_VARIANT == HC_VARIANT_MEGA
    // 2 unsigned long (2 x 32 bit = 64 bit)
    static long unsigned g_inputsMode_previous_L = 0;     // bit HIGH => Pull-Up enabled, bit LOW => Pull-Up disabled
    static long unsigned g_inputsMode_previous_H = 0;      
#elif defined(ARDUINO_ARCH_SAMD)
    static long unsigned g_inputsMode_previous       = 0; // bit HIGH => Pull enabled,    bit LOW => Pull disabled
    static long unsigned g_inputsModeOption_previous = 0; // bit HIGH => Pull-Up,         bit LOW => Pull-Down
#else
    // unsigned long (32 bit)
    static long unsigned g_inputsMode_previous = 0;       // bit HIGH => Pull-Up enabled, bit LOW => Pull-Up disabled
#endif  

// Output mode register
// => binary mode (0: DIGITAL, 1: ANALOG (PWM))
#if HC_VARIANT == HC_VARIANT_MEGA
    // 2 unsigned long (2 x 32 bit = 64 bit)
    static long unsigned g_outputType_L = 0;
    static long unsigned g_outputType_H = 0;
#else
    // unsigned long (32 bit)
    static long unsigned g_outputType = 0;
#endif  


// Previous PWM availability (used to detect data changes)
#if HC_VARIANT == HC_VARIANT_MEGA
    // 2 unsigned long (2 x 32 bit = 64 bit)
    static long unsigned g_PwmIsAvailable_previous_L = 0;
    static long unsigned g_PwmIsAvailable_previous_H = 0;
#else
    // unsigned long (32 bit)
    static long unsigned g_PwmIsAvailable_previous = 0;
#endif  


// HITI Digital Data register
// => binary mode (0: LOW, 1: HIGH)       
static long unsigned g_DD = 0; // unsigned int (32 bit)

// Previous Digital Data register (used to detect rising/falling edges)
static long unsigned g_DD_previous = 0; // unsigned int (32 bit)

// HITI Analog Data array
static float g_AD[HC_AD_QTY] = { 0.0 };   // float (32 bit) array
static unsigned long g_AD_mask = 0;     // calculated only when looking for changes


// DAC mode
#if defined(ARDUINO_ARCH_SAMD)
static uint8_t g_dacsMode_previous = 0; // bit HIGH => DAC enabled,    bit LOW => DAC disabled
#endif


// HITI String (length 30 max)
#ifdef HC_STRINGMESSAGE_COMPILE
    #define HC_STRING_MAXLENGTH 30 
    static char g_String[30] = { 0 };
#endif

// Flags (to monitor value changes)
static bool g_OutputTypes_hasChanged = false;
#ifdef HC_STRINGMESSAGE_COMPILE
    static bool g_String_hasChanged = false;
#endif



// *****************************************************************************
// Methods forward declaration
// *****************************************************************************

// update DO
#if HC_VARIANT == HC_VARIANT_MEGA
void HCI_updateDO();
#else
void HCI_updateDO();
#endif

void HCI_updateDO(uint8_t index);

// update PWM
void updatePWM();
void updatePWM(uint8_t index);

#ifdef ARDUINO_ARCH_SAMD
// update DAC
void updateDAC();
void updateDAC(uint8_t index);
#endif



// *****************************************************************************
// Methods
// *****************************************************************************


// -----------------------------------------------------------------------------
// Project ID ------------------------------------------------------------------
// -----------------------------------------------------------------------------


void HC_codeName(const char* pgm_str)
{
    g_pgm_projectName = pgm_str;
}

void HC_codeVersion(const char* pgm_str)
{
    g_pgm_projectVersion = pgm_str;
}


const char* HC_readCodeName()
{
    return g_pgm_projectName;
}

const char* HC_readCodeVersion()
{
    return g_pgm_projectVersion;
}



// -----------------------------------------------------------------------------
// Pin mode --------------------------------------------------------------------
// -----------------------------------------------------------------------------


// write register **************************************************************
// internal call to HCS_pinMode()
#if HC_VARIANT == HC_VARIANT_MEGA
    void HC_pinsMode(unsigned long inputOutput_H, unsigned long inputOutput_L, unsigned long inputMode_H, unsigned long inputMode_L)
    {
        // check for changes
        if ((inputOutput_L != HC_readPinsMode_L()) || (inputMode_L != HC_readInputsMode_L()))
        {
            // call HCS_pinMode() on every DIO pins, based on registers
            // for all inputs with attached servo: detach Servo
            for (uint8_t index = HCS_getDIO_startIndex(); index <= 31; ++index)
            {
                // read bit in register, then call HCS_pinMode()
                if (HCS_readBit(inputOutput_L, index))
                    HCS_pinMode(index, HC_OUT);
                else
                {
                    HCS_pinMode(index, HCS_readBit(inputMode_L, index) ? HC_IN_PU : HC_IN);

                    // detach attached servo
                    if (HC_readServoMode(index))
                        HC_servoMode(index, false);
                }
            }

            // update outputs
            HCI_updateOutputs();
        }

        if ((inputOutput_H != HC_readPinsMode_H()) || (inputMode_H != HC_readInputsMode_H()))
        {
            for (uint8_t index = 32; index <= HCS_getDIO_endIndex(); ++index)
            {
                // read bit in register, then call HCS_pinMode()
                if (HCS_readBit(inputOutput_H, index - 32))
                    HCS_pinMode(index, HC_OUT);
                else
                {
                    HCS_pinMode(index, HCS_readBit(inputMode_H, index - 32) ? HC_IN_PU : HC_IN);

                    // detach attached servo
                    if (HC_readServoMode(index))
                        HC_servoMode(index, false);
                }
            }

            // update outputs
            HCI_updateOutputs();
        }
    }
#elif defined(ARDUINO_ARCH_SAMD)
    void HC_pinsMode(unsigned long inputOutput, unsigned long inputMode, unsigned long inputModeOption)
    {
        // check for changes
        if ((inputOutput != HC_readPinsMode()) || (inputMode != HC_readInputsMode()))
        {
            // call HCS_pinMode() on every DIO pins, based on registers
            // for all inputs with attached servo: detach Servo
            for (uint8_t index = HCS_getDIO_startIndex(); index <= HCS_getDIO_endIndex(); ++index)
            {
                // read bit in register, then call HCS_pinMode()
                if (HCS_readBit(inputOutput, index))
                    HCS_pinMode(index, HC_OUT);
                else
                {
                    HCS_pinMode(index, HCS_readBit(inputMode, index) ? (HCS_readBit(inputModeOption, index) ? HC_IN_PU : HC_IN_PD) : HC_IN);

                    // detached attached servo on all inputs
                    if (HC_readServoMode(index))
                        HC_servoMode(index, false);
                }
            }

            // update outputs
            HCI_updateOutputs();
        }
    }
#else
    void HC_pinsMode(unsigned long inputOutput, unsigned long inputMode)
    {
        // check for changes
        if ((inputOutput != HC_readPinsMode()) || (inputMode != HC_readInputsMode()))
        {
            // call HCS_pinMode() on every DIO pins, based on registers
            // for all inputs with attached servo: detach Servo
            for (uint8_t index = HCS_getDIO_startIndex(); index <= HCS_getDIO_endIndex(); ++index)
            {
                // read bit in register, then call HCS_pinMode()
                if (HCS_readBit(inputOutput, index))
                    HCS_pinMode(index, HC_OUT);
                else
                {
                    HCS_pinMode(index, HCS_readBit(inputMode, index) ? HC_IN_PU : HC_IN);

                    // detached attached servo on all inputs
                    if (HC_readServoMode(index))
                        HC_servoMode(index, false);
                }
            }

            // update outputs
            HCI_updateOutputs();
        }
    }
#endif
    

// write boolean ***************************************************************    
// use HCS_pinMode(), from standard library
// when receiving a HCS_pinMode() "Pm" query from the computer (see HITICommProtocol.cpp)
// 1) HC_servoMode(index, false) is performed on an input (if a Servo is attached to it)
// 2) updateOutput(uint8_t index) is performed

    
// read register ***************************************************************
#if HC_VARIANT == HC_VARIANT_MEGA
    long unsigned HC_readPinsMode_L()
    {
        long unsigned reg = 0;
        
		// read pin mode and write to register
        //for(uint8_t index = 0; index <= 31; ++index)
        uint8_t index = 32;
        while (index)
        {
            --index;
            HCS_writeBit(reg, index, (HCS_getPinMode(index) == HC_OUT));
        }

        return reg;
    }

    long unsigned HC_readPinsMode_H()
    {
        long unsigned reg = 0;
     
		// read pin mode and write to register 
        for(uint8_t index = 32; index <= HCS_getDIO_endIndex(); ++index)
            HCS_writeBit(reg, index - 32, (HCS_getPinMode(index) == HC_OUT));
        
        return reg;
    }
#else
    long unsigned HC_readPinsMode()
    {
        long unsigned reg = 0;
        
		// read pin mode and write to register	 
        //for(uint8_t index = 0; index <= HCS_getDIO_endIndex(); ++index)
        uint8_t index = HCS_getDIO_endIndex() + 1;
        while (index)
        {
            --index;
            HCS_writeBit(reg, index, (HCS_getPinMode(index) == HC_OUT));
        }
        return reg;
    }
#endif


// read boolean ****************************************************************
bool HC_readPinMode(uint8_t index)
{        
    if(index <= HCS_getDIO_endIndex())
        // read pin mode
        return (HCS_getPinMode(index) == HC_OUT);
    else
        return 0;
}


// flag : has changed **********************************************************
bool HCI_PinsMode_hasChanged()
{
    bool pinsMode_hasChanged = false;

    #if HC_VARIANT == HC_VARIANT_MEGA
        // read Pins Mode
        unsigned long pinsMode_L = HC_readPinsMode_L();
        unsigned long pinsMode_H = HC_readPinsMode_H();

        // check for changes
        pinsMode_hasChanged = (g_pinsMode_previous_L != pinsMode_L) || (g_pinsMode_previous_H != pinsMode_H);

        // record values
        g_pinsMode_previous_L = pinsMode_L;
        g_pinsMode_previous_H = pinsMode_H;
    #else
        // read Pins Mode
        unsigned long pinsMode = HC_readPinsMode();

        // check for changes
        pinsMode_hasChanged = (g_pinsMode_previous != pinsMode);

        // record values
        g_pinsMode_previous = pinsMode;
    #endif

    return pinsMode_hasChanged;
}



// -----------------------------------------------------------------------------
// Input Mode ------------------------------------------------------------------
// -----------------------------------------------------------------------------


// write register **************************************************************


// write boolean *************************************************************** 
#if defined(ARDUINO_ARCH_SAMD)
    void HC_inputMode(uint8_t index, bool mode, bool option)
    {
        if ((index >= HCS_getDIO_startIndex()) &&
            (index <= HCS_getDIO_endIndex()))
        {
            // if not an OUTPUT
            if (HCS_getPinMode(index) != HC_OUT)
            {
                // check for changes
                if ((HC_readInputMode(index) != mode) || (HC_readInputModeOption(index) != option))
                {
                    HCS_pinMode(index, (mode ? (option ? HC_IN_PU : HC_IN_PD) : HC_IN));

                    // detach attached servo
                    if (HC_readServoMode(index))
                        HC_servoMode(index, false);

                    // update output
                    HCI_updateOutput(index);
                }
            }
        }
    }
#else
    void HC_inputMode(uint8_t index, bool mode)
    {
        if ((index >= HCS_getDIO_startIndex()) &&
            (index <= HCS_getDIO_endIndex()))
        {
            // if not an OUTPUT
            if (HCS_getPinMode(index) != HC_OUT)
            {
                // check for changes
                if (HC_readInputMode(index) != mode)
                {
                    HCS_pinMode(index, (mode ? HC_IN_PU : HC_IN));

                    // detach attached servo
                    if (HC_readServoMode(index))
                        HC_servoMode(index, false);

                    // update output
                    HCI_updateOutput(index);
                }
            }
        }
    }
#endif


// read register ***************************************************************
#if HC_VARIANT == HC_VARIANT_MEGA
    long unsigned HC_readInputsMode_L()
    {
        long unsigned reg = 0;
        
		// read pin mode and write to register
        //for(uint8_t index = 0; index <= 31; ++index)
        uint8_t index = 32;
        while (index)
        {
            --index;
            HCS_writeBit(reg, index, (HCS_getPinMode(index) == HC_IN_PU));
        }

        return reg;
    }

    long unsigned HC_readInputsMode_H()
    {
        long unsigned reg = 0;
        
		// read pin mode and write to register
        for(uint8_t index = 32; index <= HCS_getDIO_endIndex(); ++index)
	        HCS_writeBit(reg, index - 32, (HCS_getPinMode(index) == HC_IN_PU));
        
        return reg;
    }
#else
    long unsigned HC_readInputsMode()
    {
        long unsigned reg = 0;
        
		// read pin mode and write to register
        //for(uint8_t index = 0; index <= HCS_getDIO_endIndex(); ++index)
        uint8_t index = HCS_getDIO_endIndex() + 1;
        while (index)
        {
            --index;

            #if defined(ARDUINO_ARCH_SAMD)
                HCS_writeBit(reg, index, (HCS_getPinMode(index) == HC_IN_PU) || (HCS_getPinMode(index) == HC_IN_PD));
            #else
                HCS_writeBit(reg, index, (HCS_getPinMode(index) == HC_IN_PU));
            #endif
        }

        return reg;
    }
#endif

 
// read boolean ****************************************************************
bool HC_readInputMode(uint8_t index)
{        
    #if defined(ARDUINO_ARCH_SAMD)
        return (index <= HCS_getDIO_endIndex()) ? (HCS_getPinMode(index) == HC_IN_PU) || (HCS_getPinMode(index) == HC_IN_PD) : 0;
    #else
        return (index <= HCS_getDIO_endIndex()) ? (HCS_getPinMode(index) == HC_IN_PU) : 0;
    #endif
}


// flag : has changed **********************************************************
bool HCI_InputsMode_hasChanged()
{
    bool inputsMode_hasChanged = false;

#if HC_VARIANT == HC_VARIANT_MEGA
    // read Pins Mode
    unsigned long inputsMode_L = HC_readInputsMode_L();
    unsigned long inputsMode_H = HC_readInputsMode_H();

    // check for changes
    inputsMode_hasChanged = (g_inputsMode_previous_L != inputsMode_L) || (g_inputsMode_previous_H != inputsMode_H);

    // record values
    g_inputsMode_previous_L = inputsMode_L;
    g_inputsMode_previous_H = inputsMode_H;
#else
    // read Pins Mode
    unsigned long inputsMode = HC_readInputsMode();

    // check for changes
    inputsMode_hasChanged = (g_inputsMode_previous != inputsMode);

    // record values
    g_inputsMode_previous = inputsMode;
#endif

    return inputsMode_hasChanged;
}



// -----------------------------------------------------------------------------
// Input Mode Option -----------------------------------------------------------
// -----------------------------------------------------------------------------


#if defined(ARDUINO_ARCH_SAMD)
    // read register ***************************************************************
    long unsigned HC_readInputsModeOption()
    {
        long unsigned reg = 0;
        
	    // read pin mode and write to register
        //for(uint8_t index = 0; index <= HCS_getDIO_endIndex(); ++index)
        uint8_t index = HCS_getDIO_endIndex() + 1;
        while (index)
        {
            --index;
            HCS_writeBit(reg, index, (HCS_getPinMode(index) == HC_IN_PU));
        }

        return reg;
    }

     
    // read boolean ****************************************************************
    bool HC_readInputModeOption(uint8_t index)
    {      
        return (index <= HCS_getDIO_endIndex()) ? (HCS_getPinMode(index) == HC_IN_PU) : 0;
    }


    // flag : has changed **********************************************************
    bool HCI_InputsModeOption_hasChanged()
    {
        bool inputsModeOption_hasChanged = false;

        // read Inputs Mode Option
        unsigned long inputsModeOption = HC_readInputsModeOption();

        // check for changes
        inputsModeOption_hasChanged = (g_inputsModeOption_previous != inputsModeOption);

        // record values
        g_inputsModeOption_previous = inputsModeOption;

        return inputsModeOption_hasChanged;
    }
#endif // #if defined(ARDUINO_ARCH_SAMD)



// -----------------------------------------------------------------------------
// Output Type -----------------------------------------------------------------
// -----------------------------------------------------------------------------


// Hardware read ***************************************************************
// NO RELIABLE METHODS


// write register **************************************************************
#if HC_VARIANT == HC_VARIANT_MEGA
    void HC_outputTypes(long unsigned Output_type_H, long unsigned Output_type_L)
    {
        // check for changes
        if ((g_outputType_L != Output_type_L) || (g_outputType_H != Output_type_H))
        {
            g_OutputTypes_hasChanged = true;
            g_outputType_L = Output_type_L;
            g_outputType_H = Output_type_H;

            // update Outputs
            HCI_updateOutputs();
        }
    }
#else
    void HC_outputTypes(long unsigned Output_type)
    {
        // check for changes
        if (g_outputType != Output_type)
        {
            g_OutputTypes_hasChanged = true;
            g_outputType = Output_type;

            // update Outputs
            HCI_updateOutputs();
        }
    }
#endif


// write boolean ***************************************************************
void HC_outputType(uint8_t index, bool value)
{
    #if HC_VARIANT == HC_VARIANT_MEGA
        if ((index >= HCS_getDIO_startIndex()) && (index < 32))
        {
            // if data has changed
            if (value != HC_readOutputType(index))
            {
                g_OutputTypes_hasChanged = true;
                HCS_writeBit(g_outputType_L, index, value);
            }
        }

        // pin 32 to End
        else if ((index >= 32) && (index <= HCS_getDIO_endIndex()))
        {
            // if data has changed
            if (value != HC_readOutputType(index))
            {
                g_OutputTypes_hasChanged = true;
                HCS_writeBit(g_outputType_H, index - 32, value);
            }
        }

    #else
        if ((index >= HCS_getDIO_startIndex()) && (index <= HCS_getDIO_endIndex()))
        {
            // if data has changed
            if (value != HC_readOutputType(index))
            {
                g_OutputTypes_hasChanged = true;
                HCS_writeBit(g_outputType, index, value);
            }
        }

    #endif

    if(g_OutputTypes_hasChanged)
        // update Output
        HCI_updateOutput(index);
}


// read register ***************************************************************
#if HC_VARIANT == HC_VARIANT_MEGA
    long unsigned HC_readOutputTypes_L()
    {
        return g_outputType_L;
    }

    long unsigned HC_readOutputTypes_H()
    {
        return g_outputType_H;
    }
#else
    long unsigned HC_readOutputTypes()
    {
        return g_outputType;
    }
#endif


// read boolean ***************************************************************    
bool HC_readOutputType(uint8_t index)
{
    #if HC_VARIANT == HC_VARIANT_MEGA
        // pin 0 to 31
        if(index < 32)
            return HCS_readBit(g_outputType_L, index);

        // pin 32 to End
        else if((index >= 32) && 
                (index <= HCS_getDIO_endIndex()))
			return HCS_readBit(g_outputType_H, index - 32);

        else
			return 0;
    #else
            // pin 0 to End
        if(index <= HCS_getDIO_endIndex())
			return HCS_readBit(g_outputType, index);

        else
            return 0;
    #endif
}


// flag : has changed **********************************************************
bool HCI_OutputTypes_hasChanged()
{
    return HCI_readAndConsume(&g_OutputTypes_hasChanged);
}


// update Outputs **************************************************************
void HCI_updateOutputs()
{
    // update DO, PWM, and Servos
    HCI_updateDO();
    updatePWM();
}

void HCI_updateOutput(uint8_t index)
{
    // update DO, PWM, and Servos
    HCI_updateDO(index);
    updatePWM(index);
}



// -----------------------------------------------------------------------------
// PWM hardware availability ---------------------------------------------------
// Check PWM availability on pin : PIN_HAS_PWM()
// Check PWM deactivation by use of Servos : PWM_IS_ENABLED()
// -----------------------------------------------------------------------------


// read register ***************************************************************
#if HC_VARIANT == HC_VARIANT_MEGA
	unsigned long HC_PwmIsAvailable_L()
	{
		unsigned long reg = 0;
		
		// read PWM availability and write to register
		// PWM availability will not reappear after a Servo.detach()
		//for(uint8_t index = 0; index <= 31; ++index)
        uint8_t index = 32;
        while (index)
        {
            --index;
            HCS_writeBit(reg, index, (HCS_hasPWM(index) && HCS_isPWMEnabled(index, HCI_getAttachedServosQty())));//HCI_getAttachedServosQty_MaxAchieved())));
        }

		return reg;
	}

	unsigned long HC_PwmIsAvailable_H()
	{
		unsigned long reg = 0;
		
		// read PWM availability and write to register
		// PWM availability will not reappear after a Servo.detach()
		for(uint8_t index = 32; index <= HCS_getDIO_endIndex(); ++index)
			HCS_writeBit(reg, index - 32, (HCS_hasPWM(index) && HCS_isPWMEnabled(index, HCI_getAttachedServosQty())));//HCI_getAttachedServosQty_MaxAchieved())));
			
		return reg;
	}
#else
	unsigned long HC_PwmIsAvailable()
	{
		unsigned long reg = 0;
		
		// read PWM availability and write to register
		// PWM availability will not reappear after a Servo.detach()
		//for(uint8_t index = 0; index <= HCS_getDIO_endIndex(); ++index)
        uint8_t index = HCS_getDIO_endIndex() + 1;
        while (index)
        {
            --index;
            HCS_writeBit(reg, index, (HCS_hasPWM(index) && HCS_isPWMEnabled(index, HCI_getAttachedServosQty())));//HCI_getAttachedServosQty_MaxAchieved())));
        }

		return reg;
	}
#endif


// read boolean *****************************************************************
// PIN_HAS_PWM(uint8_t index)
bool HC_PwmIsAvailable(uint8_t index)
{        
    if(index <= HCS_getDIO_endIndex())
        // read PWM availability and return
		// PWM availability will not reappear after a Servo.detach()
        return HCS_hasPWM(index) && HCS_isPWMEnabled(index, HCI_getAttachedServosQty());//HCI_getAttachedServosQty_MaxAchieved());
    else
        return 0;
}


// flag : has changed **********************************************************
bool HCI_PWMavailability_hasChanged()
{
    bool PWMavailability_hasChanged = false;

    #if HC_VARIANT == HC_VARIANT_MEGA
        // read Pins Mode
        unsigned long PWMavailability_L = HC_PwmIsAvailable_L();
        unsigned long PWMavailability_H = HC_PwmIsAvailable_H();

        // check for changes
        PWMavailability_hasChanged = (g_PwmIsAvailable_previous_L != PWMavailability_L) || (g_PwmIsAvailable_previous_H != PWMavailability_H);

        // record values
        g_PwmIsAvailable_previous_L = PWMavailability_L;
        g_PwmIsAvailable_previous_H = PWMavailability_H;
    #else
        // read Pins Mode
        unsigned long PWMavailability = HC_PwmIsAvailable();

        // check for changes
        PWMavailability_hasChanged = (g_PwmIsAvailable_previous != PWMavailability);

        // record values
        g_PwmIsAvailable_previous = PWMavailability;
    #endif

    return PWMavailability_hasChanged;
}



// -----------------------------------------------------------------------------
// DI --------------------------------------------------------------------------
// -----------------------------------------------------------------------------


// read register ***************************************************************
#if HC_VARIANT == HC_VARIANT_MEGA
    long unsigned HC_readDI_L()
    {
        long unsigned reg = 0;
        
		// if input, read digital input
        //for (uint8_t index = 0; index <= 31; ++index)
        uint8_t index = 32;
        while (index)
        {
            --index;
            HCS_writeBit(reg, index, HCS_readDI_LA(index));
        }

        return reg;
    }

    long unsigned HC_readDI_H()
    {
        long unsigned reg = 0;
		
        // if input, read digital input
        for(uint8_t index = 32; index <= HCS_getDIO_endIndex(); ++index)
			HCS_writeBit(reg, index - 32, HCS_readDI_LA(index));

        return reg;
    }
#else
    long unsigned HC_readDI()
    {
        long unsigned reg = 0;
        
        // if input, read digital input
        //for(uint8_t index = 0; index <= HCS_getDIO_endIndex(); ++index)
        uint8_t index = HCS_getDIO_endIndex() + 1;
        while (index)
        {
            --index;
            HCS_writeBit(reg, index, HCS_readDI_LA(index));
        }

        return reg;
    }
#endif


// read boolean *****************************************************************
bool HC_readDI(uint8_t index)
{        
    if(index <= HCS_getDIO_endIndex())
        // if input, read digital input and return
        return HCS_readDI_LA(index);
    
    return 0;
}



// -----------------------------------------------------------------------------
// DO --------------------------------------------------------------------------
// -----------------------------------------------------------------------------


// write register **************************************************************
// internal call to HCS_digitalWrite()
#if HC_VARIANT == HC_VARIANT_MEGA
    void HC_writeDO(unsigned long reg_H, unsigned long reg_L)
    {
        // call HCS_digitalWrite() on every DIO pins, based on registers
        // call to HCS_digitalWrite() reset PWM
        for(uint8_t index = HCS_getDIO_startIndex(); index <= HCS_getDIO_endIndex(); ++index)
        {
            if(index < 32)
                HC_writeDO(index, (bool) HCS_readBit(reg_L, index));
            else
                HC_writeDO(index, (bool) HCS_readBit(reg_H, index - 32));
        }
    }
#else
    void HC_writeDO(unsigned long reg)
    {
        // call HCS_digitalWrite() on every DIO pins, based on registers
        for(uint8_t index = HCS_getDIO_startIndex(); index <= HCS_getDIO_endIndex(); ++index)
            HC_writeDO(index, (bool) HCS_readBit(reg, index));
    }
#endif

    
// write boolean ***************************************************************    
void HC_writeDO(uint8_t index, bool value)
{
    if((index >= HCS_getDIO_startIndex()) && 
       (index <= HCS_getDIO_endIndex()))
    {
        // if output AND 
        // (if output mode = digital OR if pwm is not available) AND
        // if no servo attached AND
        if(HC_readPinMode(index) &&
           (!HC_readOutputType(index) || !HC_PwmIsAvailable(index)) && 
           !HC_readServoMode(index))
            // write DO
            HCS_writeDO_LA(index, value);    
    }
}


// read register ***************************************************************
#if HC_VARIANT == HC_VARIANT_MEGA
    long unsigned HC_readDO_L()
    {
        long unsigned reg = 0;
        
		// if output, read digital output
        //for (uint8_t index = 0; index <= 31; ++index)
        uint8_t index = 32;
        while (index)
        {
            --index;
                HCS_writeBit(reg, index, HC_readDO(index));
        }

        return reg;
    }

    long unsigned HC_readDO_H()
    {
        long unsigned reg = 0;
        
		// if output, read digital output
        for(uint8_t index = 32; index <= HCS_getDIO_endIndex(); ++index)
		    HCS_writeBit(reg, index - 32, HC_readDO(index));
        
        return reg;
    }
#else
    long unsigned HC_readDO()
    {
        long unsigned reg = 0;
        
		// if output, read digital output
        //for(uint8_t index = 0; index <= HCS_getDIO_endIndex(); ++index)
        uint8_t index = HCS_getDIO_endIndex() + 1;
        while (index)
        {
            --index;
            HCS_writeBit(reg, index, HC_readDO(index));
        }

        return reg;
    }
#endif

 
// read boolean ****************************************************************
bool HC_readDO(uint8_t index)
{        
    if (index <= HCS_getDIO_endIndex())
        // if output, read digital output and return
        return HCS_readDO_LA(index);
    
    return 0;
}


// update DO *******************************************************************
#if HC_VARIANT == HC_VARIANT_MEGA
    void HCI_updateDO()
    {
        HC_writeDO(HC_readDO_H(), HC_readDO_L());
    }
#else
    void HCI_updateDO()
    {
        HC_writeDO(0);
    }
#endif

void HCI_updateDO(uint8_t index)
{
    HC_writeDO(index, HC_LOW);
}



// -----------------------------------------------------------------------------
// PWM -------------------------------------------------------------------------
// -----------------------------------------------------------------------------


// write boolean ***************************************************************    
// internal call to HCS_analogWrite()    
#if defined ARDUINO_ARCH_SAMD
void HC_writePWM(uint8_t index, unsigned int value)
#else
void HC_writePWM(uint8_t index, uint8_t value)
#endif
{
    if (HC_PwmIsActivated(index))
        HCS_writePWM_LA(index, value);
}


// read byte *******************************************************************
#if defined ARDUINO_ARCH_SAMD
unsigned int HC_readPWM(uint8_t index)
#else
uint8_t HC_readPWM(uint8_t index)
#endif
{        
    if(index <= HCS_getDIO_endIndex())
    {
        if(HC_PwmIsActivated(index))
            return HCS_readPWM_LA(index);
        else
            return 0;
    }
    
    return 0;
}


// update PWM ******************************************************************
void updatePWM()
{
    for(uint8_t index = HCS_getDIO_startIndex(); index <= HCS_getDIO_endIndex(); ++index)
        updatePWM(index);
}

void updatePWM(uint8_t index)
{
    if (HC_PwmIsActivated(index))
        HC_writePWM(index, 0);
}


// Activated PWM ***************************************************************
bool HC_PwmIsActivated(uint8_t index)
{
    if (index <= HCS_getDIO_endIndex())
    {
        // if output AND 
        // (if output mode = PWM AND pwm is available) AND 
        // if no servo attached
        if (HC_readPinMode(index) &&
            (HC_PwmIsAvailable(index) && HC_readOutputType(index)) &&
            !HC_readServoMode(index))
            return true;
    }

    return false;
}

uint8_t HCI_getPWM_qty()
{
    uint8_t counter;

    //for (uint8_t index = 0; index <= HCS_getDIO_endIndex(); ++index)
    uint8_t index = HCS_getDIO_endIndex() + 1;
    while (index)
    {
        if(HC_PwmIsActivated(--index))
            ++counter;
    }

    return counter;
}



// -----------------------------------------------------------------------------
// DAC Mode --------------------------------------------------------------------
// -----------------------------------------------------------------------------


#if defined ARDUINO_ARCH_SAMD
// write register
void HC_dacsMode(uint8_t mode)
{
    HC_dacMode(0, HCS_readBit(mode, 0));
    HC_dacMode(1, HCS_readBit(mode, 1));
}

// write boolean
void HC_dacMode(uint8_t index, bool enable)
{
    // DAC0 or DAC1
    if(index < 2)
    {
        if (enable)
            HC_writeDAC(index, 0); // enable DAC and set value 0
        else
            HCS_readAI_LA(index, true); // force analog input reading to disable DAC
    }
}

// read register
uint8_t HC_readDacsMode()
{
    return (HC_readDacMode(0) << 0) | (HC_readDacMode(1) << 1);
}

// read boolean 
bool HC_readDacMode(uint8_t index)
{
    return HCS_DacIsEnabled(HCS_getDacPin(index));
}

// flag
bool HCI_DacsMode_hasChanged()
{
    bool mode_hasChanged = false;

    // read Dacs Mode
    unsigned long mode = HC_readDacsMode();

    // check for changes
    mode_hasChanged = (g_dacsMode_previous != mode);

    // record values
    g_dacsMode_previous = mode;

    return mode_hasChanged;
}



// -----------------------------------------------------------------------------
// DAC -------------------------------------------------------------------------
// -----------------------------------------------------------------------------


// write boolean ***************************************************************    
// internal call to HCS_analogWrite()    
void HC_writeDAC(uint8_t index, unsigned int value)
{
    if(index == 0)
        HCS_writeDAC_LA(HCS_getDacPin(index), value);
}


// read byte *******************************************************************
unsigned int HC_readDAC(uint8_t index)
{
    return HCS_readDAC_LA(HCS_getDacPin(index));
}
#endif



// -----------------------------------------------------------------------------
// AI --------------------------------------------------------------------------
// -----------------------------------------------------------------------------
 

// read int ********************************************************************
unsigned int HC_readAI(uint8_t index)
{        
    if (index <= HCS_getAI_endIndex())
        // read analog input and return
        return (unsigned int)HCS_readAI_LA(index);

    return 0;
}



// -----------------------------------------------------------------------------
// HITI Digital Data -----------------------------------------------------------
// -----------------------------------------------------------------------------


// write register **************************************************************
void HC_writeDD(long unsigned data)
{
    g_DD = data;
}
        
void HC_writeDD(uint8_t index, bool data)
{
    // digital data 0 to 31
    if(index < HC_DD_QTY)
        HCS_writeBit(g_DD, index, data);    
}

void HC_digitalDataWrite(uint8_t index, bool data)
{
    HC_writeDD(index, data);
}


// read register ***************************************************************
long unsigned HC_readDD()
{
    return g_DD;
}

bool HC_readDD(uint8_t index)
{
    // digital data 0 to 31
    if(index < HC_DD_QTY)
        return HCS_readBit(g_DD, index);
    else
        return 0;
}
bool HC_digitalDataRead(uint8_t index)
{
    return HC_readDD(index);
}

bool HC_digitalDataRead_click(uint8_t index)
{
    if (HC_digitalDataRead(index))
    {
        // reset/consume digital data
        HC_digitalDataWrite(index, false);

        return true;
    }

    return 0;
}


// check for rising/falling edges **********************************************
void HCI_recordDD()
{
    // record values
    g_DD_previous = g_DD;
}

bool HC_digitalDataRead_risingEdge(uint8_t index)
{
    // digital data 0 to 31
    if (index < HC_DD_QTY)
        return !HCS_readBit(g_DD_previous, index) && HCS_readBit(g_DD, index);
    else
        return 0;
}

bool HC_digitalDataRead_fallingEdge(uint8_t index)
{
    // digital data 0 to 31
    if (index < HC_DD_QTY)
        return HCS_readBit(g_DD_previous, index) && !HCS_readBit(g_DD, index);
    else
        return 0;
}



// -----------------------------------------------------------------------------
// HITI Analog Data ------------------------------------------------------------
// -----------------------------------------------------------------------------


// write boolean ***************************************************************    
void HC_writeAD(uint8_t index, float value)
{
    // analog data 0 to 19
    if(index < HC_AD_QTY)
        g_AD[index] = value;
}

void HC_analogDataWrite(uint8_t index, float value)
{
    HC_writeAD(index, value);
}


// read float ******************************************************************
float HC_readAD(uint8_t index)
{        
    // analog data 0 to 19
    if(index < HC_AD_QTY)
        return g_AD[index];
    
    return 0;
}

float HC_analogDataRead(uint8_t index)
{
    return HC_readAD(index);
}

float HC_analogDataRead_setpoint(uint8_t index, float min, float max)
{
    float input = HC_readAD(index);

    // constrain into given range
    input = HCS_constrain(input, min, max);
    
    // update Analog Data
    HC_analogDataWrite(index, input);

    return input;
}

float HC_analogDataRead_setpoint(uint8_t index, float min, float max, float min_remapped, float max_remapped)
{
    float input = HC_analogDataRead_setpoint(index, min, max);

    // remap from range min-max to range min_remapped-max-remapped
    input = HCS_map(input, min, max, min_remapped, max_remapped);

    return input;
}


// get/read AD mask (mask of non null AD) **************************************

// get g_AD_mode
unsigned long HCI_getADMask()
{
    return g_AD_mask;
}

bool HCI_getADMask(uint8_t index)
{
    if(index < HC_AD_QTY)
        return HCS_readBit(g_AD_mask, index);

    return 0;
}

// read. Does not update g_AD_mask
unsigned long HCI_readADMask()
{
    // check there are available AD
    if (HC_AD_QTY != 0)
    {
        unsigned long mask = 0;
        //for (uint8_t index = 0; index < HC_AD_QTY; ++index)
        uint8_t index = HC_AD_QTY;
        while (index)
        {
            if (g_AD[--index] != 0)
                HCS_writeBit(mask, index, 1);
        }

        return mask;
    }

    return 0;
}

bool HCI_readADMask(uint8_t index)
{
    // check there are available AD
    if (HC_AD_QTY != 0)
        return (g_AD[index] != 0);

    return 0;
}


// check for changes and update g_AD_mask **************************************
bool HCI_ADMask_hasChanged()
{
    bool hasChanged = false;

    // check for changes
    unsigned long mask = HCI_readADMask();
    hasChanged = (g_AD_mask != mask);

    // record values
    g_AD_mask = mask;

    return hasChanged;
}


// non-null qty of AD in g_AD_mask *********************************************
uint8_t HCI_getAD_NonNullQty()
{
    return HC_countBit(g_AD_mask);
}



// -----------------------------------------------------------------------------
// HITI String -----------------------------------------------------------------
// -----------------------------------------------------------------------------


#ifdef HC_STRINGMESSAGE_COMPILE
    // write String ****************************************************************
    void HC_writeString(char* str)
    {
        // limit size
        if (strlen(str) + 1 > HC_STRING_MAXLENGTH)
            str[HC_STRING_MAXLENGTH - 1] = '\0';

        // check for changes
        if (strcmp(g_String, str) != 0)
        {
            g_String_hasChanged = true;
            strcpy(g_String, str);
        }
    }

    // read String *****************************************************************
    char* HC_readString()
    {
        return g_String;
    }

    // flag ************************************************************************
    bool HCI_String_hasChanged()
    {
        return HCI_readAndConsume(&g_String_hasChanged);
    }
#endif