/*
 * HITIComm
 * HC_MotionManager.h
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

#ifndef HC_MotionManager_h
#define HC_MotionManager_h



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HITICommSupport.h>



// *****************************************************************************
// Forward declaration of class
// *****************************************************************************

class HC_AbstractMotor;		// because it is used in member functions



// *****************************************************************************
// Define
// *****************************************************************************



// *****************************************************************************
// Class
// *****************************************************************************

class HC_MotionManager
{
	public:
		// constructor ****************************************************
		HC_MotionManager();
		
		// setters ********************************************************
		
		// position setpoint	
		void absolutePosition(float startPosition, float setpoint);
		void relativePosition(float startPosition, float setpoint);

		// speed setpoint
		void speed(float setpoint);
		void motionTime(float setpoint);

		// continuous speed setpoint
		void continuousSpeed(float setpoint);
		
		// travel limits
		void travelLimits(float min, float max);

		// Cycle Times
		// min = 10 ms (limited by Serial Communication which disturbs loop() cycle time)
		void cycleTime(uint8_t cycleTime);
		void continuousCycleTime(uint8_t cycleTime);
		
		// max values
		void maxSpeed(float maxSpeed);
		//void setMaxAcceleration(float maxAcceleration);

		// reset State
		void reset();
		void manualReset();
		void autoReset();
				
		// getters ********************************************************

		float getStroke() const;
		virtual float getTargetPosition() const;
		virtual float getTargetSpeed() const;
		float getContinuousTargetSpeed() const;

		float getMotionTime() const;

		uint8_t getCycleTime() const;
		uint8_t getContinuousCycleTime() const;

		float getMaxSpeed() const;

		float getIncrement() const;
		float getContinuousIncrement() const;

		bool isReady() const;
		bool isStarting() const;
		bool isGenerating() const;
		bool isGeneratingContinuouslyNeg() const;
		bool isGeneratingContinuouslyPos() const;
		bool isMoving() const;
		bool isEnding();
		bool isDone() const;
		
		bool isNegLimitReached() const;
		bool isPosLimitReached() const;
			
		// methods ********************************************************
		void moveOnTrigger(bool trigger, void (*action_motorMotion)(void));
		void moveOnTrigger(bool trigger);
		void moveNow(void (*action_motorMotion)(void));
		void moveNow();
			
		void moveContinuously(
				bool trigger,
				bool direction,
				float startPosition,
				void (*action_motorMotion)(void));
		void moveContinuously(
				bool trigger,
				bool direction,
				float startPosition);
			
		void stopOnTrigger(bool trigger);
		void stopNow();

	protected:
		// methods ********************************************************
		bool isNewPositionAvailable();
		float getNewPosition() const;
		
		virtual void checkTravelLimits(float checkedPosition, float sensitivity);

		// action corresponding to a motor motion increment during a motion profile generation 
		virtual void action_motorMotion(); // to redefine in daugther class

	private:
		// enum ***********************************************************
		enum class State
		{
			HC_READY,
			HC_GENERATING,
			HC_GENERATING_CONTINUOUSLY_POS,
			HC_GENERATING_CONTINUOUSLY_NEG,
			HC_DONE
		};

		enum class Profile
		{
			HC_LINEAR,
			HC_TRAPEZOIDAL
		};

		enum class PositionMode
		{
			HC_ABSOLUTE,
			HC_RELATIVE
		};

		enum class SpeedMode
		{
			HC_SPEED,
			HC_MOTION_TIME
		};

		/*enum class AccelerationMode
		{
			ACCELERATION,
			TIME
		};*/

		// methods ********************************************************
		
		// calculate increment
		void calculateFiniteIncrement();
		void calculateContinuousIncrement();

		// adjust speed to max speed
		void limitFiniteSpeed();
		void limitContinuousSpeed();
	
		// calculate Finite Motion parameters
		void calculateFiniteParameters();

		// generate position profile
		bool generate();
		bool generateContinuously(
				bool command,
				bool direction,
				float startPosition);

		bool incrementPositionSetpoint();
		bool incrementPositionSetpointContinuously(
				bool command,
				bool direction);

		void move(
			bool continuousMotion,
			bool trigger,
			bool direction,
			float startPosition,
			void (*action_motorMotion)(void));

		bool isMotionAllowed(
				float posSetpoint, 
				bool direction);
		bool willTravelStopBeReached(
				float posSetpoint,
				bool direction);

		// variables ******************************************************
		
		// Motion PROFILE
		Profile mProfile = Profile::HC_LINEAR;
		
		// FINITE motion (absolute or relative)
		PositionMode mPositionMode;			// Position mode
		SpeedMode mSpeedMode;				// Speed mode
		//AccelerationMode mAccelerationMode;	// Acceleration mode
		float mMotionTime;					// motion time (s)
		float mTargetSpeed;					// target speed (/s)
		uint8_t mCycleTime = 10;				// cycle time (ms). Min 10
		float mIncrement;					// motion performed each cycle 
		float mTargetPosition_setpoint;		// absolute position setpoint
		bool  mNewPositionAvailable;		// true if a new position is available
		float mInitialPosition;				// initial absolute position
		float mTargetPosition;				// end absolute setpoint
		unsigned long mLastIncrementTime; 	// time at which last increment move was scheduled (ms)
		
		// CONTINOUS motion
		float mConTargetSpeed;     				// motion speed (/s) (setpoint, >= 0)
		uint8_t mConCycleTime = 10;				// cycle time (ms). Min 10
		float mConIncrement; 					// motion performed each cycle (setpoint, >= 0)
		float mConEffectiveIncrement; 			// motion performed each cycle (used for calculations, > 0 or < 0)
		unsigned long mLastConIncrementTime;	// time at which last increment move was scheduled (ms)
				
		// Motion STATE
		State mState = State::HC_READY;		// state (ready (0) -> generating (1)              -> done (2) -> ready(0))
											// state (ready (0) -> generating continuously (1) -> done (2) -> ready(0))
		bool mIsStarting = false;
		bool mIsEnding = false;
		bool mAutoReset = true;				// if true: autoreset when Done

		// Software Travel Stops				
		float mAbsPosition_min = 0;		    // min value
		float mAbsPosition_max = 180;		// max value
		bool mNegativeStopReached = false;	// true if negative stop is reached
		bool mPositiveStopReached = false;	// true if positive stop is reached
		
		// Safety
		float mMaxSpeed = 0;				// max speed (/s)
		//float mMaxAcceleration = 0;			// max acceleration (/s²)
};


#endif
