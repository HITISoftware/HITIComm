/*
 * HITIComm
 * HC_Servo.cpp
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
// Include header
// *****************************************************************************

#include "HC_Servo.h"



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITIComm
#include "sub\HC_AbstractMotor.h"
#include "HC_ServoManager.h"



// *****************************************************************************
// Class Methods
// *****************************************************************************

	// constructor ********************************************************

	HC_Servo::HC_Servo() {}
	
	
	// destructor *********************************************************
	
	HC_Servo::~HC_Servo()
	{
		// detach Servo from pin
		detachServo();
	}
	
	
	// setters *********************************************************
	
	void HC_Servo::init(
			uint8_t id, 
			uint8_t pin, 
			bool invertedDirection, 
			float offset, 
			float initPosition,
			Servo* servo /*= NULL_POINTER*/)
	{
		HC_AbstractMotor::init(
				id, 
				pin, 
				invertedDirection, 
				offset);

		// attach servo to pin
		// set standard motion parameters
		attachServo( 
				initPosition,
				servo);
	}
	
	void HC_Servo::init(
			uint8_t id, 
			uint8_t pin, 
			bool invertedDirection, 
			float offset, 
			unsigned int minPulseWidth,
			unsigned int maxPulseWidth,
			float initPosition,
			Servo* servo /*= NULL_POINTER*/)
	{
		HC_AbstractMotor::init(
				id,
				pin,
				invertedDirection,
				offset);

		// attach servo to pin
		// set standard motion parameters
		attachServo(
				minPulseWidth, 
				maxPulseWidth, 
				initPosition,
				servo);
	}
								
	
	// getters *********************************************************

	// current position
	float HC_Servo::getCurrentPosition() const	{ return getUserPosition(getRawPosition()); }

	// raw position (servo position)
	float HC_Servo::getRawPosition() const		{ return ((float)HC_servoRead(mPin)) / 1000.0; }

	// servo
	Servo* HC_Servo::getServo() const			{ return HC_getServo(mPin); }

	
	// attach servo ****************************************************		
	
	// attach servo to pin and initialize motion parameters
	// initial position, in User m°
	// min pulse width : corresponds to 0° (standard = 544 us)
	// max pulse width : corresponds to 180° (standard = 2400 us)
	void HC_Servo::attachServo(
			Servo* servo /*= NULL_POINTER*/)
	{
		attachServo(
				false,
				0,
				0,
				false,
				0,
				servo);
	}

	void HC_Servo::attachServo(
			float initPosition,
			Servo* servo /*= NULL_POINTER*/)
	{
		attachServo(
				false,
				0,
				0,
				true,
				initPosition,
				servo);
	}

	void HC_Servo::attachServo(
			unsigned int minPulseWidth,
			unsigned int maxPulseWidth,
			Servo* servo /*= NULL_POINTER*/)
	{
		attachServo(
				true,
				minPulseWidth,
				maxPulseWidth,
				false,
				0,
				servo);
	}

	void HC_Servo::attachServo(
			unsigned int minPulseWidth,
			unsigned int maxPulseWidth,
			float initPosition,
			Servo* servo /*= NULL_POINTER*/)
	{
		attachServo(
				true,
				minPulseWidth,
				maxPulseWidth,
				true,
				initPosition,
				servo);
	}

	void HC_Servo::attachServo(
			bool definePulseWitdh,
			unsigned int minPulseWidth,
			unsigned int maxPulseWidth,
			bool definePosition,
			float initPosition,
			Servo* servo /*= NULL_POINTER*/)
	{
		// attach Servo to pin
		HC_servoMode(
				mPin, 
				true, // attach
				definePulseWitdh,
				minPulseWidth, 
				maxPulseWidth, 
				definePosition,
			    (unsigned long)(getRawPositionSetpoint(initPosition, true) * 1000.0),
				servo);	
	}	


	void HC_Servo::detachServo()
	{
		// detach Servo to pin
		HC_servoMode(mPin, false);
	} 
	

	// position resolution is around 0.1°
	void HC_Servo::checkTravelLimits()
	{ 
		HC_MotionManager::checkTravelLimits(getRawPosition(), 0.2); 
	}

	
	// Motion Profile callback using "this" pointer ***************************************************
	
	// The profile generates a new position at a regular rate (motion cycle)
	// Each time a new position is made available, this function is called by the object mMotionManager
	void HC_Servo::action_motorMotion()
	{
		// move Servo to this new position
		HC_servoWrite(mPin, (unsigned long) (getNewPosition()*1000.0));
	}