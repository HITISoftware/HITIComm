/*
 * HITIComm
 * HC_MotorGroup.h
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

#ifndef HC_MotorGroup_h
#define HC_MotorGroup_h



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HITICommSupport.h>



// *****************************************************************************
// Forward declaration of class
// *****************************************************************************

class HC_AbstractMotor;	// because it is a member declared as a pointer (of array)



// *****************************************************************************
// Define
// *****************************************************************************



// *****************************************************************************
// Class
// *****************************************************************************

class HC_MotorGroup
{
	public:	
		// constructor *****************************************************
		HC_MotorGroup();
		
		// destructor ******************************************************
		~HC_MotorGroup();
		
		// setters *********************************************************
		void init(uint8_t motor_qty);

		// Group
		void motionTime(float motionTime);
		
		// Motor
		void absolutePosition(uint8_t motorID, float setpoint);
		void relativePosition(uint8_t motorID, float setpoint);

		// reset State
		void reset();
		void manualReset();
		void autoReset();
		
		// getters *********************************************************

		uint8_t getMotorQuantity() const;
		
		float getMotionTime() const;
		
		HC_AbstractMotor& getMotor(uint8_t motorID) const;
		
		bool isReady() const;
		bool isStarting() const;
		bool isMoving() const;
		bool isEnding();
		bool isDone() const;
		
		void add(HC_AbstractMotor* motor_pointer);
		
		void moveOnTrigger(bool trigger);
		void moveNow();
		void stopOnTrigger(bool trigger);
		void stopNow();

	protected:
	
	private:
		// enum ************************************************************
		enum Action
		{
			HC_RESET,
			HC_ABSOLUTE_POSITION,
			HC_RELATIVE_POSITION,
			HC_APPLY_MOTION_TIME,
			HC_IS_READY,
			HC_IS_MOVING,
			HC_IS_DONE,
			HC_MOVE_ON_TRIGGER,
			HC_STOP_ON_TRIGGER
		};

		// methods *********************************************************
		void forAll(Action action, bool boolValue = false);
		bool forID(uint8_t motorID, Action action, float floatValue = 0);

		bool isState(Action action) const;

		void adjustMotionTimes();

		void clear();

		// variables *******************************************************
		uint8_t mMotor_qty = 0;
		HC_AbstractMotor** mMotor_pointer_array = 0;
		uint8_t mMotor_counter = 0;

		// motion parameters
		float mMotionTime;		  // (s)

		// motion state
		bool mIsStarting = false;
		bool mIsEnding = false;
		bool mAutoReset = true;	  // if true: autoreset when Done
};


#endif
