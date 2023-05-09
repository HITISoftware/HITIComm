/*
 * HITIComm
 * HC_Servo.h
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

#ifndef HITICommServo_h
#define HITICommServo_h



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HITICommSupport.h>

// HITIComm
#include "sub\HC_AbstractMotor.h" // because of inheritance



// *****************************************************************************
// Forward declaration
// *****************************************************************************

class Servo; // because used as a pointer



// *****************************************************************************
// Class
// *****************************************************************************

class HC_Servo : public HC_AbstractMotor
{
	public:
		// constructor *****************************************************
		HC_Servo();
		
		// destructor ******************************************************
		~HC_Servo();
		
		// setters *********************************************************
		void init(
				uint8_t id, 
				uint8_t pin, 
				bool invertedDirection, 
				float offset,
				float initPosition,
				Servo* servo = NULL_POINTER);
		void init(
				uint8_t id, 
				uint8_t pin, 
				bool invertedDirection, 
				float offset,
				unsigned int minPulseWidth,
				unsigned int maxPulseWidth,
				float initPosition,
				Servo* servo = NULL_POINTER);
												
		// getters *********************************************************
		
		virtual float getCurrentPosition() const override;

		float getRawPosition() const;

		// servo
		Servo* getServo() const;
		
		// methods *********************************************************

		void detachServo();
		
	protected:
	
	private:
		// Attach servo ****************************************************
		// initial position, in User m°
		// min pulse width : corresponds to 0° (standard = 544 us)
		// max pulse width : corresponds to 180° (standard = 2400 us)
		void attachServo(
				Servo* servo = NULL_POINTER);
		void attachServo(
				float initPosition,
				Servo* servo = NULL_POINTER);
		void attachServo(
				unsigned int minPulseWidth,
				unsigned int maxPulseWidth,
				Servo* servo = NULL_POINTER);
		void attachServo(
				unsigned int minPulseWidth,
				unsigned int maxPulseWidth,
				float initPosition,
				Servo* servo = NULL_POINTER);
		void attachServo(
				bool definePulseWitdh,
				unsigned int minPulseWidth,
				unsigned int maxPulseWidth,
				bool definePosition,
				float initPosition,
				Servo* servo = NULL_POINTER);
		
		virtual void checkTravelLimits() override;

		virtual void action_motorMotion() override;
};

#endif
