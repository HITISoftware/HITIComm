/*
 * HITIComm
 * HC_ServoManager.h
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


#ifndef HC_ServoManager_h
#define HC_ServoManager_h



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HITICommSupport.h>



// *****************************************************************************
// Forward declaration
// *****************************************************************************

class Servo; // because used as a pointer



// *****************************************************************************
// Methods
// *****************************************************************************


// -----------------------------------------------------------------------------
// Map management --------------------------------------------------------------
// -----------------------------------------------------------------------------

// initialize all Servos
void HCI_initializeServos(bool enableServoManagement);


// -----------------------------------------------------------------------------
// Getter ----------------------------------------------------------------------
// -----------------------------------------------------------------------------

uint8_t HCI_getAttachedServosQty();
uint8_t HCI_getAttachedServosQty_MaxAchieved();

Servo* HC_getServo(uint8_t pin);


// -----------------------------------------------------------------------------
// Map read/write --------------------------------------------------------------
// -----------------------------------------------------------------------------

// write register
// Attach Detach Servo
#if HC_VARIANT == HC_VARIANT_MEGA
    void HC_servosMode(unsigned long reg_H, unsigned long reg_L);
#else
    void HC_servosMode(unsigned long reg);
#endif

// write boolean    
// initial position (°)
// min pulse width : corresponds to 0° (standard = 544 us)
// max pulse width : corresponds to 180° (standard = 2400 us)
void HC_servoMode(
		uint8_t index, 
		bool attach,
		Servo* servo = NULL_POINTER);
void HC_servoMode(
		uint8_t index,
		bool attach,
		unsigned long initPosition,
		Servo* servo = NULL_POINTER);
void HC_servoMode(
		uint8_t index,
		bool attach,
		unsigned int minPulseWidth,
		unsigned int maxPulseWidth,
		Servo* servo = NULL_POINTER);
void HC_servoMode(
		uint8_t index, 
		bool attach,
		unsigned int minPulseWidth,
		unsigned int maxPulseWidth,
		unsigned long initPosition,
		Servo* servo = NULL_POINTER);
void HC_servoMode(
		uint8_t index,
		bool attach,
		bool definePulseWitdh,
		unsigned int minPulseWidth,
		unsigned int maxPulseWidth,
		bool definePosition,
		unsigned long initPosition,
		Servo* servo = NULL_POINTER);
// user-friendly function names
void HC_attachServo(
		uint8_t index, 
		Servo* servo = NULL_POINTER);
void HC_attachServo(
		uint8_t index,
		unsigned long initPosition,
		Servo* servo = NULL_POINTER);
void HC_attachServo(
		uint8_t index,
		unsigned int minPulseWidth,
		unsigned int maxPulseWidth,
		Servo* servo = NULL_POINTER);
void HC_attachServo(
		uint8_t index, 
		unsigned int minPulseWidth,
		unsigned int maxPulseWidth,
		unsigned long initPosition,
		Servo* servo = NULL_POINTER);
void HC_detachServo(
		uint8_t index);
    
// get g_ServosMode
#if HC_VARIANT == HC_VARIANT_MEGA
    long unsigned HC_getServosMode_L();
    long unsigned HC_getServosMode_H();
#else
    long unsigned HC_getServosMode();
#endif

bool HC_getServoMode(uint8_t index);

// read mode (does not update mServoMode)
#if HC_VARIANT == HC_VARIANT_MEGA
	long unsigned HC_readServosMode_L();
	long unsigned HC_readServosMode_H();
#else
	long unsigned HC_readServosMode();
#endif

bool HC_readServoMode(uint8_t index);

// check for changes (update mServoMode)
bool HCI_ServosMode_hasChanged();


// -----------------------------------------------------------------------------
// User access to Servos -------------------------------------------------------
// -----------------------------------------------------------------------------

// millidegrees
// - standard servo            : 0 (0°),             90000 (90° middle position), 180000 (180°)
// - continuous rotation servo : 0 (CCW full speed), 90000 (stopped),             180000 (CW full speed)
void HC_servoWrite(uint8_t index, unsigned long value);
unsigned long HC_servoRead(uint8_t index);

// microseconds
// - standard servo            : 700-1000us (0°),             1500us (90° middle position), 2000-2300us (180°)
// - continuous rotation servo : 700-1000us (CCW full speed), 1500us (stopped),             2000-2300us (CW full speed)
void HC_servoWriteMicroseconds(uint8_t index, unsigned int microseconds);
unsigned int HC_servoReadMicroseconds(uint8_t index);


#endif
