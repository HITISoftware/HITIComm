/*
 * HITIComm
 * HC_AbstractMotor.h
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

#ifndef HC_AbstractMotor_h
#define HC_AbstractMotor_h



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HCS_Time.h>

// HITIComm
#include "sub\HC_MotionManager.h" // because of inheritance



// *****************************************************************************
// Abstract Class
// *****************************************************************************

class HC_AbstractMotor : public HC_MotionManager
{
	// (= 0: means a function is purely abstract)
	public:	
		// destructor ******************************************************
		virtual ~HC_AbstractMotor();
		
		// setters *********************************************************		
		void init(
			uint8_t id,
			uint8_t pin,
			bool invertedDirection,
			float offset);

		void absolutePosition(float posSetpoint);
		void relativePosition(float posSetpoint);

		virtual void travelLimits(float min, float max);
				
		// getters *********************************************************
		uint8_t getID() const;

		virtual float getTargetPosition() const override;
		virtual float getTargetSpeed() const override;

		virtual float getCurrentPosition() const = 0;
		virtual float getCurrentSpeed() const;
		virtual float getContinuousCurrentSpeed() const;

		bool isNegLimitReached();
		bool isPosLimitReached();

		// motion **********************************************************
		void movePositive(bool trigger);
		void moveNegative(bool trigger);

	protected:
		// constructor *****************************************************
		HC_AbstractMotor();

		// methods *********************************************************

		// convert position: raw <-> user
		float getUserPosition(float raw_position) const;
		float getRawPosition(float corrected_position) const;

		// raw position (servo position, sensor feedback, etc...)
		virtual float getRawPosition() const = 0;

		// raw position setpoint
		float getRawPositionSetpoint(float user_position, bool isPosMode_absolute);

		virtual void checkTravelLimits() = 0;

		// variables *******************************************************

		uint8_t mID = 0;	// unique motor ID
		uint8_t mPin = 0;
		bool mInvertedDirection = false;
		float mOffset = 0;

	private:
		// methods *********************************************************
		void moveContinuously(bool trigger, bool direction);
};


#endif
