/*
 * HITIComm
 * HC_MotionManager.cpp
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



#include "sub\HC_MotionManager.h"



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITIComm
#include "sub\HC_AbstractMotor.h"
#include "HC_Toolbox.h"



// *****************************************************************************
// Class Methods
// *****************************************************************************

	// constructor ********************************************************
	
	HC_MotionManager::HC_MotionManager()
	{
		absolutePosition(0, 0);
		speed(1);
		continuousSpeed(1);
	}


	// setters ********************************************************

	// position setpoint
	void HC_MotionManager::absolutePosition(
			float startPosition, // absolute start position
			float setpoint)		 // absolute position setpoint
	{
		if (isReady())
		{
			// ABSOLUTE mode
			mPositionMode = PositionMode::HC_ABSOLUTE;

			mInitialPosition = startPosition;
			mTargetPosition = setpoint;
		}

		calculateFiniteParameters();
	}

	void HC_MotionManager::relativePosition(
		float startPosition, // absolute start position
		float setpoint)		 // relative position setpoint
	{
		if (isReady())
		{
			// RELATIVE mode
			mPositionMode = PositionMode::HC_RELATIVE;

			mInitialPosition = startPosition;
			mTargetPosition = startPosition + setpoint;
		}
		
		calculateFiniteParameters();
	}


	// speed setpoint
	void HC_MotionManager::speed(float setpoint)
	{
		if (isReady())
		{
			// SPEED mode
			mSpeedMode = SpeedMode::HC_SPEED;

			mTargetSpeed = (setpoint != 0) ? HCS_abs(setpoint) : 1;
			mMotionTime = (HCS_abs(getStroke()) / mTargetSpeed);
		}

		calculateFiniteParameters();
	}

	void HC_MotionManager::motionTime(float setpoint)
	{
		if (isReady())
		{
			// MOTION TIME mode
			mSpeedMode = SpeedMode::HC_MOTION_TIME;

			mMotionTime = (setpoint != 0) ? HCS_abs(setpoint) : 1;
			mTargetSpeed = (HCS_abs(getStroke()) / mMotionTime);
		}

		calculateFiniteParameters();
	}

	// continuous speed setpoint
	void HC_MotionManager::continuousSpeed(float setpoint)
	{
		// setting allowed only if in Ready state
		if(isReady())
		{
			mConTargetSpeed = (setpoint != 0) ? HCS_abs(setpoint) : 1;
			
			// limit speed
			//limitContinuousSpeed();

			// calculate continuous motion increment to perform each cycle
			calculateContinuousIncrement();
		}
	}
	

	// Travel limits
	void HC_MotionManager::travelLimits(
			float min, 
			float max)
	{
		mAbsPosition_min = min;
		mAbsPosition_max = max;
	}
				

	// Cycle Time
	// min = 10 ms (limited by Serial Communication which disturbs loop() cycle time)
	void HC_MotionManager::cycleTime(uint8_t cycleTime)
	{
		// setting allowed only if in Ready state
		if(isReady())
		{
			mCycleTime = (cycleTime < 10) ? 10 : cycleTime;
		
			// calculate motion increment to perform each cycle
			calculateFiniteIncrement();
		}
	}
	
	// continuous motion Cycle Time
	// min = 10 ms (limited by Serial Communication which disturbs loop() cycle time)
	void HC_MotionManager::continuousCycleTime(uint8_t cycleTime)
	{
		// setting allowed only if in Ready state
		if(isReady())
		{
			mConCycleTime = (cycleTime < 10) ? 10 : cycleTime;
			
			// calculate continuous motion increment to perform each cycle
			calculateContinuousIncrement();
		}
	}

		
	// Max speed
	void HC_MotionManager::maxSpeed(float maxSpeed)
	{
		if (isReady())
			mMaxSpeed = maxSpeed;
		
		// limit speed
		limitFiniteSpeed();
		limitContinuousSpeed();
	}
	
	// adjust speed to max speed
	void HC_MotionManager::limitFiniteSpeed()
	{
		if (isReady())
		{
			// if max speed has been set
			if (mMaxSpeed != 0)
			{
				// limit motion speed to max speed
				if (mTargetSpeed > mMaxSpeed)
				{
					mTargetSpeed = mMaxSpeed;
					mMotionTime = (HCS_abs(getStroke()) / mTargetSpeed);
				}
			}
		}
	}

	// adjust speed to max speed
	void HC_MotionManager::limitContinuousSpeed()
	{
		if (isReady())
		{
			// if max speed has been set
			if (mMaxSpeed != 0)
			{
				// limit motion speed to max speed
				if (mConTargetSpeed > mMaxSpeed)
					mConTargetSpeed = mMaxSpeed;
			}
		}
	}

	// Max acceleration
	/*void HC_MotionManager::setMaxAcceleration(float maxAcceleration)
	{
		mMaxAcceleration = maxAcceleration;

		// limit acceleration
		//limitAcceleration();
	}*/
	

	void HC_MotionManager::calculateFiniteParameters()
	{
		// limit Finite speed
		limitFiniteSpeed();

		// calculate motion increment to perform each cycle
		calculateFiniteIncrement();
	}


	// reset
	void HC_MotionManager::reset()			{ mState = State::HC_READY; }	
	void HC_MotionManager::manualReset()	{ mAutoReset = false; }
	void HC_MotionManager::autoReset()		{ mAutoReset = true; }

	
	// getter *************************************************************

	float HC_MotionManager::getStroke() const					{ return mTargetPosition - mInitialPosition; }	// positive or negative
	float HC_MotionManager::getTargetPosition() const			{ return mTargetPosition; }
	float HC_MotionManager::getTargetSpeed() const				{ return mIncrement < 0 ? -mTargetSpeed : mTargetSpeed; }
	float HC_MotionManager::getContinuousTargetSpeed() const	{ return mConEffectiveIncrement < 0 ? -mConTargetSpeed : mConTargetSpeed; }
	float HC_MotionManager::getMotionTime() const				{ return mMotionTime; }

	uint8_t HC_MotionManager::getCycleTime() const					{ return mCycleTime; }
	uint8_t HC_MotionManager::getContinuousCycleTime() const		{ return mConCycleTime; }

	float HC_MotionManager::getMaxSpeed() const					{ return mMaxSpeed; };

	float HC_MotionManager::getIncrement() const				{ return mIncrement; }
	float HC_MotionManager::getContinuousIncrement() const		{ return mConEffectiveIncrement; }

	bool HC_MotionManager::isReady() const						{ return (mState == State::HC_READY); }
	bool HC_MotionManager::isStarting() const					{ return mIsStarting; }
	bool HC_MotionManager::isGenerating() const					{ return (mState == State::HC_GENERATING); }
	bool HC_MotionManager::isGeneratingContinuouslyNeg() const	{ return (mState == State::HC_GENERATING_CONTINUOUSLY_NEG); }
	bool HC_MotionManager::isGeneratingContinuouslyPos() const	{ return (mState == State::HC_GENERATING_CONTINUOUSLY_POS); }
	bool HC_MotionManager::isMoving() const						{ return isGenerating() || isGeneratingContinuouslyNeg() || isGeneratingContinuouslyPos(); }
	bool HC_MotionManager::isEnding()							{ return HCI_readAndConsume(&mIsEnding); }
	bool HC_MotionManager::isDone() const						{ return (mState == State::HC_DONE); }

	bool HC_MotionManager::isNegLimitReached() const			{ return mNegativeStopReached; }
	bool HC_MotionManager::isPosLimitReached() const			{ return mPositiveStopReached; }
	
	
	// profile generation *************************************************
	
	void HC_MotionManager::calculateFiniteIncrement()
	{
		// value
		mIncrement = mTargetSpeed * ((float) mCycleTime) / 1000.0; // >= 0
		
		// sign
		if(getStroke() < 0)
			mIncrement *= -1;
	}
	
	void HC_MotionManager::calculateContinuousIncrement()
	{
		// setpoint (>= 0)
		mConIncrement = mConTargetSpeed * ((float) mConCycleTime) / 1000.0; // >= 0
	}

	// generate position profile
	// calculate a setpoint position at each call. This setpoint changes every cycle
	// => return false = request to end motion
	bool HC_MotionManager::generate()
	{
		// if Ready to start
		if(isReady())
		{
			// reset flag
			mNewPositionAvailable = false;
			
			if (mIncrement == 0)
				return false;
			else
			{
				// State: -> Generating (1)
				mState = State::HC_GENERATING;

				// record start time
				mLastIncrementTime = HCS_millis();
				
				// initialize setpoint
				mTargetPosition_setpoint = mInitialPosition;
				
				// increment position setpoint
				return incrementPositionSetpoint();
			}
		}
		
		// if generating
		else if(isGenerating())
		{
			// when a cycle time is reached
			if(HCS_millis() - mLastIncrementTime >= mCycleTime)
			{
				// update the time on which last increment move was scheduled
				mLastIncrementTime += mCycleTime;
				
				// set flag: new position available
				mNewPositionAvailable = true;
					
				// increment position setpoint
				return incrementPositionSetpoint();
			}
			
			return true;
		}
	}
	

	// generate continuous position profile
	// calculate a setpoint position at each call. This setpoint changes every cycle
	// => return false = request to end motion
	bool HC_MotionManager::generateContinuously(
			bool command,
			bool direction,
			float startPosition)
	{
		// if Ready to start
		if(isReady())
		{
			// reset flag
			mNewPositionAvailable = false;
				
			if(mConIncrement == 0)
				return false;
			else
			{
				// State -> Generating continuously
				// calculate effective increment
				if (direction)
				{
					mState = State::HC_GENERATING_CONTINUOUSLY_POS;
					mConEffectiveIncrement = mConIncrement;
				}
				else
				{
					mState = State::HC_GENERATING_CONTINUOUSLY_NEG;
					mConEffectiveIncrement = -mConIncrement;
				}

				// record start time
				mLastConIncrementTime = HCS_millis();
					
				// initialize setpoint
				mTargetPosition_setpoint = startPosition;
				
				// increment position setpoint and update motion state	
				return incrementPositionSetpointContinuously(command, direction);
			}
		}
		
		// if generating
		else if((isGeneratingContinuouslyNeg() && !direction) ||
		        (isGeneratingContinuouslyPos() && direction))
		{				
			// when a cycle time is reached
			if(HCS_millis() - mLastConIncrementTime >= mConCycleTime)
			{					
				// update the time on which last increment move was scheduled
				mLastConIncrementTime += mConCycleTime;
				
				// set flag: new position available
				mNewPositionAvailable = true;
					
				// increment position setpoint and update motion state	
				return incrementPositionSetpointContinuously(command, direction);
			}
			
			return true;
		}
	}
	
	
	// increment position setpoint and update motion state
	// => return false = request to end motion
	bool HC_MotionManager::incrementPositionSetpoint()
	{
		// if motion is allowed in this direction
		if(isMotionAllowed(mTargetPosition_setpoint, (mIncrement >= 0)))
		{
			// if the remaining motion size is bigger than the increment size
			if((HCS_abs(mTargetPosition - mTargetPosition_setpoint)) >= HCS_abs(mIncrement))
				// add increment to position setpoint
				mTargetPosition_setpoint += mIncrement;
			else
			{
				if(mTargetPosition != mTargetPosition_setpoint)
					// add remaining sub-increment to reach target
					mTargetPosition_setpoint = mTargetPosition;
					
				// if target position has been reached (at previous cycle)
				else
					return false;
			}
			
			// if limit in this direction will be reached, constrain setpoint
			if(willTravelStopBeReached(mTargetPosition_setpoint, (mIncrement >= 0)))
			{
				if(mIncrement >= 0)
					mTargetPosition_setpoint = mAbsPosition_max;
				else
					mTargetPosition_setpoint = mAbsPosition_min;
			}
			
			return true;
		}

		return false;
	}

	
	// increment position setpoint and update motion state 
	// => return false = request to end motion
	bool HC_MotionManager::incrementPositionSetpointContinuously(
			bool command,
			bool direction)
	{
		// if motion is allowed in this direction
		if(isMotionAllowed(mTargetPosition_setpoint, direction))
		{
			if (command)
				// add increment to position setpoint
				mTargetPosition_setpoint += mConEffectiveIncrement;
			else
				return false;
				
			// if limit in this direction will be reached, constrain setpoint
			if(willTravelStopBeReached(mTargetPosition_setpoint, direction))
			{
				if(direction)
					mTargetPosition_setpoint = mAbsPosition_max;
				else
					mTargetPosition_setpoint = mAbsPosition_min;
			}
			
			return true;
		}
		
		return false;
	}
	
	
	// return true if a new position is available
	bool HC_MotionManager::isNewPositionAvailable()
	{
		if(mNewPositionAvailable)
		{
			// reset flag
			mNewPositionAvailable = false;
			
			return true;
		}
		else
			return false;
	}
	
	// return a setpoint position at each call. This setpoint is updated every motion cycle
	// call it if newPositionIsAvailable() returns true
	float HC_MotionManager::getNewPosition() const
	{
		return mTargetPosition_setpoint;
	}
	

	// motion management ****************************************************
	
	// start a continuous move
	void HC_MotionManager::move(
			bool continuousMotion,
			bool trigger, // initiate finite motion or move continuously while this trigger is activated
			bool direction,						// continuous motion: true => positive direction
			float startPosition,				// continuous motion: absolute start position
			void (*externalCallback)(void))		// Motor motion: external callback function
	{
		if(isReady())
		{
			mIsStarting = false;
			mIsEnding = false;
			
			// if the command is activated
			if(trigger)
			{
				// start generating a motion profile
				if((!continuousMotion && generate()) ||
				   (continuousMotion && generateContinuously(trigger, direction, startPosition)))
				{
					mIsStarting = true;
					
					// motor motion
					if (externalCallback != 0)
						(*externalCallback)();
					else
						action_motorMotion();
				}
			}
		}
		else if((!continuousMotion && isGenerating()) ||
				((isGeneratingContinuouslyNeg() && !direction) ||
		        (isGeneratingContinuouslyPos() && direction)))
		{
			mIsStarting = false;
			
			// generate profile
			if ((!continuousMotion && generate()) ||
				(continuousMotion && generateContinuously(trigger, direction, startPosition)))
			{
				// if a new position has been generated
				if (isNewPositionAvailable())
				{
					// motor motion
					if (externalCallback != 0)
						(*externalCallback)();
					else
						action_motorMotion();
				}
			}
			else
				stopNow();
		}
		// required in manual reset
		else if(isDone())
		{
			mIsEnding = false;
				
			// required if switch to autoreset in this state
			if(mAutoReset)
				reset();
		}
	}

	// start a move when trigger received
	void HC_MotionManager::moveOnTrigger(
			bool trigger, 
			void (*action_motorMotion)(void))
	{
		move(
				false,
				trigger,
				false,
				0.0,
				action_motorMotion);
	}
	
	void HC_MotionManager::moveOnTrigger(bool trigger)
	{
		move(
				false,
				trigger,
				false,
				0.0,
				NULL_POINTER);
	}
	
	
	// start a move immediately
	void HC_MotionManager::moveNow(void (*action_motorMotion)(void))
	{
		moveOnTrigger(true, action_motorMotion);
	}

	void HC_MotionManager::moveNow()
	{
		moveOnTrigger(true);
	}
	

	// start a continuous move
	void HC_MotionManager::moveContinuously(
		bool trigger,						// move continuously while this command is activated
		bool direction,						// true : positive direction
		float startPosition,				// absolute start position
		void (*action_motorMotion)(void))	// Motor motion: use of callback function
	{
		move(
				true,
				trigger,
				direction,
				startPosition,
				action_motorMotion);
	}
	
	// start a continuous move
	void HC_MotionManager::moveContinuously(
		bool trigger,						// move continuously while this command is activated
		bool direction,						// true : positive direction
		float startPosition)				// absolute start position
	{
		move(
				true,
				trigger,
				direction,
				startPosition,
				NULL_POINTER);
	}


	// stop a move when trigger received 
	void HC_MotionManager::stopOnTrigger(
			bool trigger)
	{
		// if the trigger is activated
		if (trigger)
		{
			mIsEnding = true; // reading of this value consumes it (reset it)

			if (mAutoReset)
				reset();
			else
				mState = State::HC_DONE;
		}
	}
	
	// stop a move immediately
	void HC_MotionManager::stopNow()
	{
		stopOnTrigger(true);
	}
	
	void HC_MotionManager::checkTravelLimits(
			float checkedPosition, 
			float sensitivity)
	{
		// negative limit
		mNegativeStopReached = (checkedPosition < mAbsPosition_min) || (HCS_abs(checkedPosition - mAbsPosition_min) <= HCS_abs(sensitivity));

		// positive limit
		mPositiveStopReached = (checkedPosition > mAbsPosition_max) || (HCS_abs(checkedPosition - mAbsPosition_max) <= HCS_abs(sensitivity));
	}
	
	
	bool HC_MotionManager::isMotionAllowed(
			float currentPosition,
			bool direction)
	{
		// if limit - exceeded, and trying to go -
		if((currentPosition <= mAbsPosition_min) && !direction)
			return false;
			
		// if limit + exceeded, and trying to go +
		if((currentPosition >= mAbsPosition_max) && direction)
			return false;
		
		return true;
	}
	
	bool HC_MotionManager::willTravelStopBeReached(
			float posSetpoint,
			bool direction)
	{
		// if limit - will be reached
		if((posSetpoint <= mAbsPosition_min) && !direction)
			return true;
			
		// if limit + will be reached
		if((posSetpoint >= mAbsPosition_max) && direction)
			return true;
		
		return false;
	}
	

	// action corresponding to a motor motion increment during a motion profile generation 
	void HC_MotionManager::action_motorMotion() {} // to redefine in daugther class