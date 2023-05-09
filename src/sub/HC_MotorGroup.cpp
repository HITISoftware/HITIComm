/*
 * HITIComm
 * HC_MotorGroup.cpp
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


#include "HC_MotorGroup.h"



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITIComm
#include "HC_Toolbox.h"
#include "sub\HC_AbstractMotor.h"



// *****************************************************************************
// Class Methods
// *****************************************************************************


	// constructor *****************************************************
	
	HC_MotorGroup::HC_MotorGroup() {}
	
	
	// destructor ******************************************************
	HC_MotorGroup::~HC_MotorGroup()
	{
		// clear dynamic array and motor count
		clear();
	}
		
		
	// setters **********************************************************
		
	void HC_MotorGroup::init(/*uint8_t groupID, */uint8_t motor_qty)
	{
		if(isReady())
		{
			//mID = groupID;
			
			// there must be at least 1 motor
			mMotor_qty = (motor_qty > 1) ? motor_qty : 1;

			// clear dynamic array and motor count
			clear();
			
			// create dynamic array of motor pointers
			mMotor_pointer_array = new HC_AbstractMotor*[mMotor_qty];
		}
	}

	
	// reset State of all motors
	void HC_MotorGroup::reset()			{ forAll(HC_RESET); }
	void HC_MotorGroup::manualReset()	{ mAutoReset = false; }
	void HC_MotorGroup::autoReset()		{ mAutoReset = true; }


	// Group: motion time
	void HC_MotorGroup::motionTime(float motionTime)
	{
		mMotionTime = motionTime;

		// adjust motion times of all motors
		adjustMotionTimes();
	}


	// Motor: position setpoint
	void HC_MotorGroup::absolutePosition(uint8_t motorID, float posSetpoint)
	{
		forID(motorID, HC_ABSOLUTE_POSITION, posSetpoint);
	}
	
	void HC_MotorGroup::relativePosition(uint8_t motorID, float posSetpoint)
	{
		forID(motorID, HC_RELATIVE_POSITION, posSetpoint);
	}

	void HC_MotorGroup::adjustMotionTimes()
	{
		// apply motion times to all motors
		forAll(HC_APPLY_MOTION_TIME);

		//for (int i = 0; i < mMotor_qty; ++i)
		uint8_t i = mMotor_qty;
		while (i)
		{
			--i;
			if (mMotor_pointer_array[i] != NULL_POINTER)
			{
				// set motion time
				mMotor_pointer_array[i]->motionTime(mMotionTime);

				// if speed is limited for this motor, motion time may be increased
				if (mMotor_pointer_array[i]->getMotionTime() > mMotionTime)
					mMotionTime = mMotor_pointer_array[i]->getMotionTime();
			}
		}

		// if motion time was increased by some motors, apply new motion time to all
		forAll(HC_APPLY_MOTION_TIME);
	}
	
	
	// getters *********************************************************
	
	//uint8_t HC_MotorGroup::getID() const 				{ return mID; }
	uint8_t HC_MotorGroup::getMotorQuantity() const	{ return mMotor_qty; }

	HC_AbstractMotor& HC_MotorGroup::getMotor(uint8_t motorID) const
	{
		//for(int i = 0; i < mMotor_qty; i++)
		uint8_t i = mMotor_qty;
		while (i)
		{
			--i;
			if (mMotor_pointer_array[i] != NULL_POINTER)
			{
				if (mMotor_pointer_array[i]->getID() == motorID)
					return *mMotor_pointer_array[i];
			}
		}
		
		// by default, return first motor. If array is empty, code will bug
		return *mMotor_pointer_array[0];
	}

	float HC_MotorGroup::getMotionTime() const		{ return mMotionTime; }
	
	bool HC_MotorGroup::isReady() const				{ return isState(HC_IS_READY); }	// true if all servos are Ready
	bool HC_MotorGroup::isStarting() const			{ return mIsStarting; }
	bool HC_MotorGroup::isMoving() const			{ return isState(HC_IS_MOVING); }	// true if any servo is Moving
	bool HC_MotorGroup::isEnding()					{ return HCI_readAndConsume(&mIsEnding); }
	bool HC_MotorGroup::isDone() const				{ return isState(HC_IS_DONE); }	// true if all servos are Done
	
	
	// motor array management ******************************************
	
	void HC_MotorGroup::add(HC_AbstractMotor* motor_pointer)
	{
		// add only if there is room for the additional motor
		if(mMotor_counter < mMotor_qty)
		{
			// add to array
			mMotor_pointer_array[mMotor_counter] = motor_pointer;
				
			// manual reset
			mMotor_pointer_array[mMotor_counter]->manualReset();

			mMotor_counter++;
		}
	}
	
	void HC_MotorGroup::clear()
	{
		// if memory allocated, clear memory
		if(mMotor_pointer_array != 0)
		{
			delete mMotor_pointer_array;
			mMotor_pointer_array = 0;
		}
		
		// reset counter
		mMotor_counter = 0;
	}
	
	
	// motion **********************************************************
	
	void HC_MotorGroup::moveOnTrigger(bool trigger)
	{
		if (isReady())
		{
			mIsEnding = false; // required in autoreset
		
			// if the trigger is activated
			if (trigger)
			{
				mIsStarting = true;

				forAll(HC_MOVE_ON_TRIGGER, trigger);
			}
		}
		else if (isMoving())
		{
			mIsStarting = false;
			
			forAll(HC_MOVE_ON_TRIGGER, trigger);

			if(isDone())
				stopNow();
		}
		// required in manual reset, or if stopped in auto reset
		else if(isDone())
		{
			mIsEnding = false;
				
			if(mAutoReset)
				reset();
		}
	}
	
	void HC_MotorGroup::moveNow()	
	{ 
		moveOnTrigger(true); 
	}
	
	void HC_MotorGroup::stopOnTrigger(bool trigger)
	{
		if (trigger)
		{
			mIsEnding = true;

			forAll(HC_STOP_ON_TRIGGER);
		}
	}
	
	void HC_MotorGroup::stopNow()	
	{ 
		stopOnTrigger(true); 
	}


	// Utility **********************************************************

	void HC_MotorGroup::forAll(
			Action action,
			bool boolValue /*= false*/)
	{
		//for (int i = 0; i < mMotor_qty; ++i)
		uint8_t i = mMotor_qty;
		while (i)
		{
			--i;
			if (mMotor_pointer_array[i] != NULL_POINTER)
			{
				switch (action)
				{
					case HC_MotorGroup::HC_RESET:
						mMotor_pointer_array[i]->reset();
						break;
					case HC_MotorGroup::HC_APPLY_MOTION_TIME:
						mMotor_pointer_array[i]->motionTime(mMotionTime);
						break;
					case HC_MotorGroup::HC_MOVE_ON_TRIGGER:
						mMotor_pointer_array[i]->moveOnTrigger(boolValue);
						break;
					case HC_MotorGroup::HC_STOP_ON_TRIGGER:
						if (mAutoReset)
							mMotor_pointer_array[i]->reset();
						else
							mMotor_pointer_array[i]->stopNow();
						break;
				}
			}
		}
	}

	bool HC_MotorGroup::forID(
			uint8_t motorID, 
			Action action,
			float floatValue /*= 0*/)
	{
		//for (int i = 0; i < mMotor_qty; ++i)
		uint8_t i = mMotor_qty;
		while (i)
		{
			--i;
			if (mMotor_pointer_array[i] != NULL_POINTER)
			{
				// find the motor with the supplied ID
				if (mMotor_pointer_array[i]->getID() == motorID)
				{
					switch (action)
					{
						case HC_MotorGroup::HC_ABSOLUTE_POSITION:
							mMotor_pointer_array[i]->absolutePosition(floatValue);
							break;
						case HC_MotorGroup::HC_RELATIVE_POSITION:
							mMotor_pointer_array[i]->relativePosition(floatValue);
							break;
					}

					// adjust motion times of all motors
					adjustMotionTimes();
					break;
				}
			}
		}
	}

	bool HC_MotorGroup::isState(Action action) const
	{
		if (mMotor_counter < mMotor_qty)
			return false;

		//for (int i = 0; i < mMotor_qty; ++i)
		uint8_t i = mMotor_qty;
		while (i)
		{
			--i;
			if (mMotor_pointer_array[i] != NULL_POINTER)
			{
				switch (action)
				{
					case HC_IS_READY:
						if (!mMotor_pointer_array[i]->isReady())
							return false;
						break;
					case HC_IS_MOVING:
						if (mMotor_pointer_array[i]->isMoving())
							return true;
						break;
					case HC_IS_DONE:
						if(!mMotor_pointer_array[i]->isDone())
							return false;
						break;
				}
			}
			else
				return false;
		}

		if (action == HC_IS_MOVING)
			return false;
		else
			return true;
	}
