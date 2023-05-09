/*
 * HITIComm
 * HC_ServoManager.cpp
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


#include "HC_ServoManager.h"



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HCS_ServoInterface.h>

// HITIComm
#include "HC_Toolbox.h"
#include "HC_Data.h"



// *****************************************************************************
// variables
// *****************************************************************************

// Servo Structure array (pointer)
extern servo_struct _servo_map[];

// Attached Servos quantity
static uint8_t _attachedServos_qty = 0;

// PWM on some pins is disabled if the quantity of attached servos goes beyond a certain amount (see PWM_IS_ENABLE())
// However, PWM is not reenabled if this quantity decrease, contrary to what is said in the Servo.h library
// github issue : "Bugg in servo library with detach(), after can't use analogWrite on pin 9 or 10"
// So we need to remember the max achieved quantity of attached servos and use it to define pins PWM availability
static uint8_t _attachedServos_qty_maxAchieved = 0;

// Servo mode
#if HC_VARIANT == HC_VARIANT_MEGA
    long unsigned g_ServosMode_H = 0;
    long unsigned g_ServosMode_L = 0;
#else
    long unsigned g_ServosMode = 0;
#endif

// Flag (to monitor value changes)
bool g_ServosMode_hasChanged = false;



// *****************************************************************************
// Methods Declaration
// *****************************************************************************


// -----------------------------------------------------------------------------
// Map management --------------------------------------------------------------
// -----------------------------------------------------------------------------


// initialize Servo
void initializeServo(struct servo_struct* servoStructPointer);

// Find unattached Servo structure
struct servo_struct* getAvailableServoStructure();

// Find attached Servo structure
struct servo_struct* getAttachedServoStructure(uint8_t pin);


// attach pin to an unattached Servo
// initial position (m°)
// min pulse width : corresponds to 0° (standard = 544 us)
// max pulse width : corresponds to 180° (standard = 2400 us)
void attachPinToServo(
    uint8_t pin,
    Servo* servo = NULL_POINTER);
void attachPinToServo(
    uint8_t pin,
    bool definePulseWitdh,
    unsigned int minPulseWidth,
    unsigned int maxPulseWidth,
    bool definePosition,
    unsigned long position,
    Servo* servo = NULL_POINTER);

// detach pin from an attached Servo
void detachPinFromServo(uint8_t pin);

// display Servos data
//void printServosData();



// *****************************************************************************
// Methods definition
// *****************************************************************************


// -----------------------------------------------------------------------------
// Map management --------------------------------------------------------------
// -----------------------------------------------------------------------------

// initialize all Servos
void HCI_initializeServos(bool enableServoManagement)
{
    for(uint8_t servo_index = 0; servo_index < HCS_getServo_qty(); ++servo_index)
	{
		// dynamic allocation: instantiates Servos (Interfaces)
		_servo_map[servo_index].servo = new HCS_ServoInterface(enableServoManagement);
	
		// initialize Servos
        initializeServo(&_servo_map[servo_index]);
	}

    // reset counter
    _attachedServos_qty = 0;
    _attachedServos_qty_maxAchieved = 0;
}

// initialize Servo
void initializeServo(struct servo_struct* servoStructPointer)
{
    // servo detached, value 90, pin 0
    servoStructPointer->servo->detach();
    servoStructPointer->servo->write(90);
    servoStructPointer->pin = 0;
}

// Find unattached Servo structure
struct servo_struct* getAvailableServoStructure()
{
    for(uint8_t servo_index = 0; servo_index < HCS_getServo_qty(); ++servo_index)
    {
        // return the first unattached servo structure found
        if(!_servo_map[servo_index].servo->attached())
            return &_servo_map[servo_index];
    }

    // if all servos are attached
    return NULL_POINTER;
}

// Find attached Servo structure
struct servo_struct* getAttachedServoStructure(uint8_t pin)
{
    for(uint8_t servo_index = 0; servo_index < HCS_getServo_qty(); ++servo_index)
    {
        // return servo structure with searched pin and attached servo
        if((_servo_map[servo_index].pin == pin) && _servo_map[servo_index].servo->attached())
            return &_servo_map[servo_index];
    }

    // if not found
    return NULL_POINTER;
}

// attach pin to an unattached Servo
// min pulse width : corresponds to 0° (standard = 544 us)
// max pulse width : corresponds to 180° (standard = 2400 us)
// initial position, in m°
void attachPinToServo(
		uint8_t pin,
        Servo* servo /*= NULL_POINTER*/)
{
    attachPinToServo(
            pin,
            false,
            0,
            0,
            false,
            0.0,
            servo);
}

void attachPinToServo(
	    uint8_t pin,
	    bool definePulseWitdh,
	    unsigned int minPulseWidth,
	    unsigned int maxPulseWidth,
	    bool definePosition,
	    unsigned long position,
        Servo* servo /*= NULL_POINTER*/)
{
    // if pin is not yet attached
    if(getAttachedServoStructure(pin) == NULL_POINTER)
    {
        // if max amount of Servos has not been reached
        if (_attachedServos_qty < HCS_getServo_qty())
        {
            // get an unattached servo structure
            servo_struct* structPointer = getAvailableServoStructure();

            // if unattached servo structure obtained
            if (structPointer != NULL_POINTER)
            {
                // set Servo object, if supplied by user
                if (servo != NULL_POINTER)
                    structPointer->servo->setServo(servo);

                // set position in m° before attaching
                if (definePosition)
                {
                    // values < MIN_PULSE_WIDTH(= 544 us) are considered as millidegrees
                    // values > MIN_PULSE_WIDTH(= 544 us) are considered as microseconds
                    if (position < 544000)
                    {
                        // limit value between 0 and 180000
                        if (position < 0)
                            position = 0;
                        else if (position > 180000)
                            position = 180000;
                        
                        // remap value from range 0-180000 to range MinPulseWidth-MaxPulseWidth
                        if (definePulseWitdh)
                            position = HC_scale(
                                position,
                                0,
                                180000,
                                minPulseWidth,
                                maxPulseWidth);
                        else
                            position = HC_scale(
                                position,
                                0,
                                180000,
                                structPointer->servo->getMinPulseWidth(),
                                structPointer->servo->getMaxPulseWidth());
                    }

                    // write Position
                    structPointer->servo->writeMicroseconds(position);
                }
                else
                    // default position
                    structPointer->servo->write(90);

                // attach pin
                if (definePulseWitdh)
                    structPointer->servo->attach(pin, minPulseWidth, maxPulseWidth);
                else
                    structPointer->servo->attach(pin);

                // if attachment worked
                if (structPointer->servo->attached())
                {
                    // assign pin to structure
                    structPointer->pin = pin;

                    // increment counter
                    _attachedServos_qty++;

                    // update max achieved quantity
                    if (_attachedServos_qty_maxAchieved < _attachedServos_qty)
                        _attachedServos_qty_maxAchieved = _attachedServos_qty;
                }
            }
        }
    }
}

// detach pin from an attached Servo
void detachPinFromServo(uint8_t pin)
{
    // find an attached servo structure with querried pin
    servo_struct* structPointer = getAttachedServoStructure(pin);
    
    // if found
    if(structPointer != NULL_POINTER)
    {
        // initialize Servo (detach, value 90, pin 0)
        initializeServo(structPointer);

        // decrement counter
        _attachedServos_qty --;
    }
}
/*
// display Servos data
void printServosData()
{
    Serial.println();
    for(uint8_t servo_index = 0; servo_index < 10; servo_index++)
    {
        Serial.print(F("servos "));
        Serial.print(servo_index, DEC);
        Serial.print(F(": is attached : "));
        Serial.print(_servo_map[servo_index].servo->attached(), DEC);
        Serial.print(F(", value : "));
        Serial.print(_servo_map[servo_index].servo->read(), DEC);
        Serial.print(F(", on pin : "));
        Serial.println(_servo_map[servo_index].pin, DEC);
    }    
}
*/


// -----------------------------------------------------------------------------
// Getter ----------------------------------------------------------------------
// -----------------------------------------------------------------------------

// Attached Servos Quantity
uint8_t HCI_getAttachedServosQty()
{
    return _attachedServos_qty;
}

// Max achieved Attached Servos Quantity
// - To use instead of getAttachedServosQty() as long as no fix is done in Servo.h library to correct
//   this bug : PWM availability will not reappear after a Servo.detach()
uint8_t HCI_getAttachedServosQty_MaxAchieved()
{
    return _attachedServos_qty_maxAchieved;
}

// Servo
Servo* HC_getServo(uint8_t pin)
{
    servo_struct* structPointer = getAttachedServoStructure(pin);

    // if pin is attached
    if (structPointer != NULL_POINTER)
        return structPointer->servo->getServo();
    else
        return NULL_POINTER;
}


// -----------------------------------------------------------------------------
// Map read/write --------------------------------------------------------------
// -----------------------------------------------------------------------------

// write register **************************************************************
// Attach Detach Servo
#if HC_VARIANT == HC_VARIANT_MEGA
    void HC_servosMode(unsigned long reg_H, unsigned long reg_L)
    {
      	// dedicated to Serial Communication
      	// pin 0 : Rx : false
      	// pin 1 : Tx : true
      	
        if (reg_L != HC_readServosMode_L())
        {
            g_ServosMode_hasChanged = true;

            //  call attach() or detach() on every DIO pins, based on register
            for (uint8_t index = HCS_getDIO_startIndex(); index <= 31; ++index)
                // if attach is queried : attach pin to an unattached servo, if pin is not yet attached
                // if detach is queried : detach pin from its attached servo
                HCS_readBit(reg_L, index) ? attachPinToServo(index) : detachPinFromServo(index);
        }

        if (reg_H != HC_readServosMode_H())
        {
            g_ServosMode_hasChanged = true;

            //  call attach() or detach() on every DIO pins, based on register
            for (uint8_t index = 32; index <= HCS_getDIO_endIndex(); ++index)
                // if attach is queried : attach pin to an unattached servo, if pin is not yet attached
                // if detach is queried : detach pin from its attached servo
                HCS_readBit(reg_H, index - 32) ? attachPinToServo(index) : detachPinFromServo(index);
        }

        if(g_ServosMode_hasChanged)
            // update Outputs
            HCI_updateOutputs();
    }
#else
    void HC_servosMode(unsigned long reg)
    {
        // check for changes
        if (reg != HC_readServosMode())
        {
            g_ServosMode_hasChanged = true;

            // dedicated to Serial Communication
            // pin 0 : Rx : false
            // pin 1 : Tx : true

            //  call attach() or detach() on every DIO pins, based on register
            for (uint8_t index = HCS_getDIO_startIndex(); index <= HCS_getDIO_endIndex(); ++index)
                // if attach is queried : attach pin to an unattached servo, if pin is not yet attached
                // if detach is queried : detach pin from its attached servo
                HCS_readBit(reg, index) ? attachPinToServo(index) : detachPinFromServo(index);

            // update Outputs
            HCI_updateOutputs();
        }
    }
#endif


// write boolean ***************************************************************
// min pulse width : corresponds to 0° (standard = 544 us)
// max pulse width : corresponds to 180° (standard = 2400 us)
// initial position, in m°
void HC_servoMode(
        uint8_t index, 
        bool attach,
        Servo* servo /*= NULL_POINTER*/)
{
    HC_servoMode(
            index,
            attach,
            false,
            0,
            0,
            false,
            0.0,
            servo);
}

void HC_servoMode(
        uint8_t index,
        bool attach,
        unsigned long initPosition,
        Servo* servo /*= NULL_POINTER*/)
{
    HC_servoMode(
            index,
            attach,
            false,
            0,
            0,
            true,
            initPosition,
            servo);
}

void HC_servoMode(
        uint8_t index,
        bool attach,
        unsigned int minPulseWidth,
        unsigned int maxPulseWidth,
        Servo* servo /*= NULL_POINTER*/)
{
    HC_servoMode(
            index,
            attach,
            true,
            minPulseWidth,
            maxPulseWidth,
            false,
            0.0,
            servo);
}

void HC_servoMode(
		uint8_t index, 
		bool attach,
		unsigned int minPulseWidth, 
		unsigned int maxPulseWidth, 
        unsigned long initPosition,
        Servo* servo /*= NULL_POINTER*/)
{
    HC_servoMode(
            index,
            attach,
            true,
            minPulseWidth,
            maxPulseWidth,
            true,
            initPosition,
            servo);
}

void HC_servoMode(
        uint8_t index,
        bool attach,
        bool definePulseWitdh,
        unsigned int minPulseWidth,
        unsigned int maxPulseWidth,
        bool definePosition,
        unsigned long initPosition,
        Servo* servo /*= NULL_POINTER*/)
{
    if ((index >= HCS_getDIO_startIndex()) && (index <= HCS_getDIO_endIndex()))
    {
        // check for changes
        if (attach != HC_readServoMode(index))
        {
            g_ServosMode_hasChanged = true;

            // if attach is queried : attach pin to an unattached servo, if pin is not yet attached
            // if detach is queried : detach pin from its attached servo
            attach ?
                attachPinToServo(
                    index,
                    definePulseWitdh,
                    minPulseWidth,
                    maxPulseWidth,
                    definePosition,
                    initPosition,
                    servo) :
                detachPinFromServo(index);

            // update Outputs
            HCI_updateOutput(index);
        }
    }
}

// user-friendly functions
void HC_attachServo(
		uint8_t index, 
		Servo* servo /*= NULL_POINTER*/)
{
	HC_servoMode(
			index,
			true,
			servo);
}

void HC_attachServo(
		uint8_t index,
        unsigned long initPosition,
		Servo* servo /*= NULL_POINTER*/)
{
	HC_servoMode(
			index,
			true,
			initPosition,
			servo);
}

void HC_attachServo(
		uint8_t index,
		unsigned int minPulseWidth,
		unsigned int maxPulseWidth,
		Servo* servo /*= NULL_POINTER*/)
{
	HC_servoMode(
			index,
			true,
			minPulseWidth,
			maxPulseWidth,
			servo);
}

void HC_attachServo(
		uint8_t index, 
		unsigned int minPulseWidth, 
		unsigned int maxPulseWidth, 
		unsigned long initPosition,
		Servo* servo /*= NULL_POINTER*/)
{
	HC_servoMode(
			index, 
			true,
			minPulseWidth, 
			maxPulseWidth, 
			initPosition,
			servo);
}

void HC_detachServo(
		uint8_t index)
{
	HC_servoMode(
			index,
			false);
}


// get g_ServosMode ***********************************************************

#if HC_VARIANT == HC_VARIANT_MEGA
    long unsigned HC_getServosMode_L()
    {
        return g_ServosMode_L;
    }

    long unsigned HC_getServosMode_H()
    {
        return g_ServosMode_H;
    }
#else
    long unsigned HC_getServosMode()
    {
        return g_ServosMode;
    }
#endif

bool HC_getServoMode(uint8_t index)
{
    if (index <= HCS_getDIO_endIndex())
    {
        #if HC_VARIANT == HC_VARIANT_MEGA
            if (index <= 31)
                return HCS_readBit(g_ServosMode_L, index);
            else
                return HCS_readBit(g_ServosMode_H, index - 32);
        #else
            return HCS_readBit(g_ServosMode, index);
        #endif
    }

    return 0;
}


// read ***********************************************************************

#if HC_VARIANT == HC_VARIANT_MEGA
    long unsigned HC_readServosMode_L()
    {
        g_ServosMode_L = 0;
        
    	// read Servo Attached state and write to register
        for(uint8_t index = HCS_getDIO_startIndex(); index <= 31; ++index)
    	    HCS_writeBit(g_ServosMode_L, index, (getAttachedServoStructure(index) != NULL_POINTER));
        
        return g_ServosMode_L;
    }

    long unsigned HC_readServosMode_H()
    {
        g_ServosMode_H = 0;
        
    	// read Servo Attached state and write to register
        for(uint8_t index = 32; index <= HCS_getDIO_endIndex(); ++index)
            HCS_writeBit(g_ServosMode_H, index - 32, (getAttachedServoStructure(index) != NULL_POINTER));
        
        return g_ServosMode_H;
    }
#else
    long unsigned HC_readServosMode()
    {
        g_ServosMode = 0;
        
        // read Servo Attached state and write to register
        for(uint8_t index = HCS_getDIO_startIndex(); index <= HCS_getDIO_endIndex(); ++index)
	        HCS_writeBit(g_ServosMode, index, (getAttachedServoStructure(index) != NULL_POINTER));
        
        return g_ServosMode;
    }
#endif
 
bool HC_readServoMode(uint8_t index)
{        
    if((index >= HCS_getDIO_startIndex()) && (index <= HCS_getDIO_endIndex()))
        // if found an attached servo structure with querried pin
        return getAttachedServoStructure(index) != NULL_POINTER;
    else
        return 0;
}

// check for changes (update mServoMode) ***************************************
bool HCI_ServosMode_hasChanged()
{
    if(HCI_readAndConsume(&g_ServosMode_hasChanged))
    {
        // update g_ServosMode
        #if HC_VARIANT == HC_VARIANT_MEGA
            HC_readServosMode_L();
            HC_readServosMode_H();
        #else
            HC_readServosMode();
        #endif

        return true;
    }

    return false;
}


// -----------------------------------------------------------------------------
// User access to Servos -------------------------------------------------------
// -----------------------------------------------------------------------------


// millidegrees *************************************************************
// - standard servo            : 0 (0°),             90000 (90° middle position), 180000 (180°)
// - continuous rotation servo : 0 (CCW full speed), 90000 (stopped),             180000 (CW full speed)

void HC_servoWrite(uint8_t index, unsigned long value)
{
    // find an attached servo structure with querried pin
    servo_struct* structPointer = getAttachedServoStructure(index);

    // if found
    if (structPointer != NULL_POINTER)
    {
        // values < MIN_PULSE_WIDTH(= 544 us) are considered as millidegrees
        // values > MIN_PULSE_WIDTH(= 544 us) are considered as microseconds
        if (value < 544000)
        {
            // limit value between 0 and 180000
            if (value < 0)
                value = 0;
            else if (value > 180000)
                value = 180000;

            // convert ° to us
            value = HC_scale(
                    value,
                    0,
                    180000,
                    structPointer->servo->getMinPulseWidth(),
                    structPointer->servo->getMaxPulseWidth());

            #if defined (ARDUINO_ARCH_MEGAAVR)
                // low counter resolution of 0.25 ticks/us (AVR board has 1 ticks/us)
                value += 2; // offset to center value 
            #endif
        }

        // write value
        structPointer->servo->writeMicroseconds(value);
    }
}

unsigned long HC_servoRead(uint8_t index)
{
    // find an attached servo structure with querried pin
    servo_struct* structPointer = getAttachedServoStructure(index);

    // if found
    if (structPointer != NULL_POINTER)
    {
        // convert us to °
        // us read can be slightly different from us querried => converted value can be outside 0 - 180000
        long value = HC_scale(
            structPointer->servo->readMicroseconds(),
            structPointer->servo->getMinPulseWidth(),
            structPointer->servo->getMaxPulseWidth(),
            0,
            180000);

        // limit value between 0 and 180000
        if (value < 0)
            value = 0;
        else if (value > 180000)
            value = 180000;

        return value;
    }
    else
        return 0;
}


// microseconds *************************************************************
// - standard servo            : 700-1000us (0°),             1500us (90° middle position), 2000-2300us (180°)
// - continuous rotation servo : 700-1000us (CCW full speed), 1500us (stopped),             2000-2300us (CW full speed)

void HC_servoWriteMicroseconds(uint8_t index, unsigned int microseconds)
{
    // find an attached servo structure with querried pin
    servo_struct* structPointer = getAttachedServoStructure(index);

    // if found
    if(structPointer != NULL_POINTER)
        // write value
        structPointer->servo->writeMicroseconds(microseconds);
}

unsigned int HC_servoReadMicroseconds(uint8_t index)
{
    // find an attached servo structure with querried pin
    servo_struct* structPointer = getAttachedServoStructure(index);

    // if found
    if(structPointer != NULL_POINTER)
        // return read value
        return structPointer->servo->readMicroseconds();
  	else
		return 0;
}
