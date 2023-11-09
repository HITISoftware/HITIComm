/*
 * HITIComm
 * HC_Data.h
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

#ifndef HC_Data_h
#define HC_Data_h



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HITICommSupport.h>

// HITIComm
#include "sub\HC_CompilationTriggers.h"



// *****************************************************************************
// Define
// *****************************************************************************


#define HC_DD_QTY 32 // HITI Digital Data qty
#define HC_AD_QTY 20 // HITI Analog Data qty

#define HC_DECIMAL_QTY 3 // Number of decimal to use when Serial Printing float numbers



// *****************************************************************************
// Methods
// *****************************************************************************


// -----------------------------------------------------------------------------
// Code ID ---------------------------------------------------------------------
// -----------------------------------------------------------------------------

void HC_codeName(const char* pgm_str);
void HC_codeVersion(const char* pgm_str);

const char* HC_readCodeName();
const char* HC_readCodeVersion();


// -----------------------------------------------------------------------------
// Pin mode --------------------------------------------------------------------
// -----------------------------------------------------------------------------

// write register
// internal call to pinMode()
#if HC_VARIANT == HC_VARIANT_MEGA
    void HC_pinsMode(unsigned long inputOutput_H, unsigned long inputOutput_L, unsigned long inputMode_H, unsigned long inputMode_L);
#elif defined(ARDUINO_ARCH_SAMD)
    void HC_pinsMode(unsigned long inputOutput, unsigned long inputMode, unsigned long inputModeOption);
#else
    void HC_pinsMode(unsigned long inputOutput, unsigned long inputMode);
#endif

// write boolean    
// use pinMode(), from standard library   
// updateOutput(uint8_t index) is performed when receiving a pinMode() "Pm" query from the computer

// read register
#if HC_VARIANT == HC_VARIANT_MEGA
    long unsigned HC_readPinsMode_L();
    long unsigned HC_readPinsMode_H();
#else
    long unsigned HC_readPinsMode();
#endif

// read boolean
bool HC_readPinMode(uint8_t index);

// flag
bool HCI_PinsMode_hasChanged();


// -----------------------------------------------------------------------------
// Input Mode ------------------------------------------------------------------
// -----------------------------------------------------------------------------

// write register
 
// write boolean
// internal call to pinMode()
#if defined(ARDUINO_ARCH_SAMD)
    void HC_inputMode(uint8_t index, bool mode, bool option);
#else
    void HC_inputMode(uint8_t index, bool mode);
#endif

// read register
#if HC_VARIANT == HC_VARIANT_MEGA
    long unsigned HC_readInputsMode_L();
    long unsigned HC_readInputsMode_H();
#else
    long unsigned HC_readInputsMode();
#endif
 
// read boolean 
bool HC_readInputMode(uint8_t index);

// flag
bool HCI_InputsMode_hasChanged();


// -----------------------------------------------------------------------------
// Input Mode Option -----------------------------------------------------------
// -----------------------------------------------------------------------------

#if defined(ARDUINO_ARCH_SAMD)
    // read register
    long unsigned HC_readInputsModeOption();
 
    // read boolean 
    bool HC_readInputModeOption(uint8_t index);

    // flag
    bool HCI_InputsModeOption_hasChanged();
#endif


// -----------------------------------------------------------------------------
// Output Type -----------------------------------------------------------------
// -----------------------------------------------------------------------------

// Hardware read 
// NO RELIABLE METHODS

// write register 
#if HC_VARIANT == HC_VARIANT_MEGA
    void HC_outputTypes(long unsigned output_mode_H, long unsigned output_mode_L);
#else
    void HC_outputTypes(long unsigned output_mode);
#endif
// write boolean 
void HC_outputType(uint8_t index, bool value);

// read register 
#if HC_VARIANT == HC_VARIANT_MEGA
    long unsigned HC_readOutputTypes_L();
    long unsigned HC_readOutputTypes_H();
#else
    long unsigned HC_readOutputTypes();
#endif
// read boolean 
bool HC_readOutputType(uint8_t index);

// flag
bool HCI_OutputTypes_hasChanged();

// update Outputs 
void HCI_updateOutputs();
void HCI_updateOutput(uint8_t index);


// -----------------------------------------------------------------------------
// PWM hardware availability ---------------------------------------------------
// -----------------------------------------------------------------------------

// read register
#if HC_VARIANT == HC_VARIANT_MEGA
    unsigned long HC_PwmIsAvailable_L();
    unsigned long HC_PwmIsAvailable_H();
#else
    unsigned long HC_PwmIsAvailable();
#endif

// read boolean
bool HC_PwmIsAvailable(uint8_t index);

// flag
bool HCI_PWMavailability_hasChanged();


// -----------------------------------------------------------------------------
// DI --------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// read register
#if HC_VARIANT == HC_VARIANT_MEGA
    long unsigned HC_readDI_L();

    long unsigned HC_readDI_H();
#else
    long unsigned HC_readDI();
#endif

// read boolean
bool HC_readDI(uint8_t index);


// -----------------------------------------------------------------------------
// DO --------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// write register 
// internal call to digitalWrite()
#if HC_VARIANT == HC_VARIANT_MEGA
    void HC_writeDO(unsigned long reg_H, unsigned long reg_L);
#else
    void HC_writeDO(unsigned long reg);
#endif

// write boolean     
// internal call to digitalWrite()
void HC_writeDO(uint8_t index, bool value);

// read register
#if HC_VARIANT == HC_VARIANT_MEGA
    long unsigned HC_readDO_L();

    long unsigned HC_readDO_H();
#else
    long unsigned HC_readDO();
#endif

// read boolean  
bool HC_readDO(uint8_t index);


// -----------------------------------------------------------------------------
// PWM -------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// write integer     
// internal call to analogWrite()
#if defined ARDUINO_ARCH_SAMD
    void HC_writePWM(uint8_t index, unsigned int value);
#else
    void HC_writePWM(uint8_t index, uint8_t value);
#endif

// read integer
#if defined ARDUINO_ARCH_SAMD
    unsigned int HC_readPWM(uint8_t index);
#else
    uint8_t HC_readPWM(uint8_t index);
#endif

// Activated PWM
bool HC_PwmIsActivated(uint8_t index);
uint8_t HCI_getPWM_qty();


// -----------------------------------------------------------------------------
// DAC Mode --------------------------------------------------------------------
// -----------------------------------------------------------------------------

#if defined ARDUINO_ARCH_SAMD
// write register
void HC_dacsMode(uint8_t mode);

// write boolean
void HC_dacMode(uint8_t index, bool enable);

// read register
uint8_t HC_readDacsMode();
 
// read boolean 
bool HC_readDacMode(uint8_t index);

// flag
bool HCI_DacsMode_hasChanged();


// -----------------------------------------------------------------------------
// DAC -------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// write integer     
// internal call to analogWrite()
void HC_writeDAC(uint8_t index, unsigned int value);

// read integer
unsigned int HC_readDAC(uint8_t index);
#endif


// -----------------------------------------------------------------------------
// AI --------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// read integer
unsigned int HC_readAI(uint8_t index);


// -----------------------------------------------------------------------------
// HITI Digital Data -----------------------------------------------------------
// -----------------------------------------------------------------------------

// write register
void HC_writeDD(long unsigned data);

// write boolean
void HC_writeDD(uint8_t index, bool data);
void HC_digitalDataWrite(uint8_t index, bool data);

// read register
long unsigned HC_readDD();

// read boolean
bool HC_readDD(uint8_t index);
bool HC_digitalDataRead(uint8_t index);
bool HC_digitalDataRead_click(uint8_t index);

// check for rising/falling edges
void HCI_recordDD();
bool HC_digitalDataRead_risingEdge(uint8_t index);
bool HC_digitalDataRead_fallingEdge(uint8_t index);


// -----------------------------------------------------------------------------
// HITI Analog Data ------------------------------------------------------------
// -----------------------------------------------------------------------------

// write float    
void HC_writeAD(uint8_t index, float value);
void HC_analogDataWrite(uint8_t index, float value);

// read float
float HC_readAD(uint8_t index);
float HC_analogDataRead(uint8_t index);
float HC_analogDataRead_setpoint(uint8_t index, float min, float max);
float HC_analogDataRead_setpoint(uint8_t index, float min, float max, float min_remapped, float max_remapped);

// mode (mask of non null AD)
// get g_AD_mode
unsigned long HCI_getADMask();
bool HCI_getADMask(uint8_t index);

// read mode (does not update g_AD_mode)
unsigned long HCI_readADMask();
bool HCI_readADMask(uint8_t index);

// check for changes (update g_AD_mode)
bool HCI_ADMask_hasChanged();

// non-null qty of AD in g_AD_mode
uint8_t HCI_getAD_NonNullQty();


// -----------------------------------------------------------------------------
// HITI String -----------------------------------------------------------------
// -----------------------------------------------------------------------------

#ifdef HC_STRINGMESSAGE_COMPILE
    // write String
    void HC_writeString(char* str);

    // read String
    char* HC_readString();

    // flag
    bool HCI_String_hasChanged();
#endif

#endif
