/*
 * HITIComm
 * HC_ServoRobot.cpp
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


#include "sub\HC_ServoRobot.h"



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITIComm
#include "HC_Servo.h"
#include "HC_MotorGroup.h"



// *****************************************************************************
// Class Methods
// *****************************************************************************

	// constructor ********************************************************

	HC_ServoRobot::HC_ServoRobot()
	{
		init(0, 1, 1);
	}
	
	HC_ServoRobot::HC_ServoRobot(
			uint8_t ID,
			uint8_t motor_qty,
			uint8_t motorGroup_qty)
	{
		init(ID, motor_qty, motorGroup_qty);
	}
	
	
	// destructor *********************************************************
	
	HC_ServoRobot::~HC_ServoRobot()
	{
		// clear dynamically allocated memory
		clear();
	}
		
			
	// setters ************************************************************
			
	void HC_ServoRobot::init(
			uint8_t ID,
			uint8_t motor_qty,
			uint8_t motorGroup_qty)
	{
		// set robot ID
		mID = ID;
	
		// there must be at least 1 Motor
		mMotor_qty = (motor_qty >= 1) ? motor_qty : 1;
		
		// there must be at least 1 Group
		mGroup_qty = (motorGroup_qty >= 1) ? motorGroup_qty : 1;		

		// clear dynamic arrays
		clear();
		
		// create dynamic arrays
		mMotor_array = new HC_Servo[mMotor_qty];
		mGroup_array = new HC_MotorGroup[mGroup_qty];		
	}
	
		
	void HC_ServoRobot::initServo(
			uint8_t motorIndex, 
			uint8_t pin, 
			bool invertedDirection, 
			int offset,
			float initPosition,
			Servo* servo /*= NULL_POINTER*/)
	{
		// check range
		if(motorIndex < mMotor_qty)
		{
			mMotor_array[motorIndex].init(
					motorIndex, 
					pin, 
					invertedDirection, 
					offset,
					initPosition,
					servo);
		}
	}
	
	void HC_ServoRobot::initServo(
			uint8_t motorIndex,
			uint8_t pin,
			bool invertedDirection,
			int offset,
			int minPulseWidth,
			int maxPulseWidth,
			float initPosition,
			Servo* servo /*= NULL_POINTER*/)
	{
		// check range
		if(motorIndex < mMotor_qty)
		{
			mMotor_array[motorIndex].init(
					motorIndex, 
					pin, 
					invertedDirection, 
					offset,
					minPulseWidth,
					maxPulseWidth,
					initPosition,
					servo);
		}
	}
	
			
	void HC_ServoRobot::initMotorGroup(
			uint8_t groupIndex,
			uint8_t motorQty)
	{
		// check range
		/*if(groupIndex < mGroup_qty)
		{
			// record Motor Group
			mGroup_array[groupIndex].init(
					groupIndex, 
					motorQty);
		}*/
	}

			
/*	void HC_ServoRobot::setMotionMode(
			HC_PositionMode positionMode,
			HC_SpeedMode speedMode)
	{
		// for all motors, same settings
		for(int i = 0; i < mMotor_qty; ++i)
		{
			mMotor_array[i].setMotionMode(
					positionMode,
					speedMode);
		}
	}
	
			
	void HC_ServoRobot::setMotionMode(
			uint8_t motorIndex,
			HC_PositionMode positionMode,
			HC_SpeedMode speedMode)
	{
		// check range
		if(motorIndex < mMotor_qty)
		{
			mMotor_array[motorIndex].setMotionMode(
					positionMode,
					speedMode);	
		}	
	}
	*/
	/*
	void HC_ServoRobot::setMotionSetpoint(
			uint8_t motorIndex,
			float posSetpoint,	
			float speSetpoint)
	{
		// check range
		if(motorIndex < mMotor_qty)
		{
			mMotor_array[motorIndex].posSetpoint(posSetpoint);
			mMotor_array[motorIndex].speSetpoint(speSetpoint);
		}	
	}
	*/
				
	void HC_ServoRobot::reset(uint8_t groupIndex)
	{
		// check range
		if (groupIndex < mGroup_qty)
			mGroup_array[groupIndex].reset();
	}
	
	
	// getters ************************************************************
	
	HC_Servo& HC_ServoRobot::getServo(uint8_t motorIndex) const
	{
		return (motorIndex < mMotor_qty) ? mMotor_array[motorIndex] : mMotor_array[0];
	}
	
	
	// return true only if all servos are Ready
	bool HC_ServoRobot::isReady() const
	{
		bool flag = true;
		
		// if one servo is not Ready, set flag to false
		for(int i = 0; i < mMotor_qty; ++i)
		{
			if(!mMotor_array[i].isReady())
				flag = false;
		}
		
		return flag;
	}
	
	// return true if any servo is Moving
	bool HC_ServoRobot::isMoving() const
	{
		for(int i = 0; i < mMotor_qty; ++i)
		{
			if(mMotor_array[i].isMoving())
				return true;
		}
		
		return false;
	}	
	
	// return true only if all servos are Done
	bool HC_ServoRobot::isDone() const
	{
		bool flag = true;
		
		// if one servo is not Done, set flag to false
		for(int i = 0; i < mMotor_qty; ++i)
		{
			if(!mMotor_array[i].isDone())
				flag = false;
		}
		
		return flag;
	}
	
	
	// return true only if all servos of the group are Ready
	bool HC_ServoRobot::isReady(uint8_t groupIndex) const
	{
		return (groupIndex < mGroup_qty) ? mGroup_array[groupIndex].isReady() : false;
	}
	
	// return true if any servo of the group is Moving
	bool HC_ServoRobot::isMoving(uint8_t groupIndex) const
	{
		return (groupIndex < mGroup_qty) ? mGroup_array[groupIndex].isMoving() : false;
	}	
	
	// return true only if all servos of the group are Done
	bool HC_ServoRobot::isDone(uint8_t groupIndex) const
	{
		return (groupIndex < mGroup_qty) ? mGroup_array[groupIndex].isDone() : false;
	}	
	
	
	// arrays management ***********************************************
	
	void HC_ServoRobot::clear()
	{
		// if memory allocated, clear memory
		if(mMotor_array != 0)
		{
			delete[] mMotor_array;
			mMotor_array = 0;
		}
		
		// if memory allocated, clear memory
		if(mGroup_array != 0)
		{
			delete[] mGroup_array;
			mGroup_array = 0;
		}
	}
	
	
	// group management ************************************************
	
	void HC_ServoRobot::addMotorToGroup(
			uint8_t groupIndex,
			uint8_t motorIndex)
	{
		// check range
		if(groupIndex < mGroup_qty)
			mGroup_array[groupIndex].add(&mMotor_array[motorIndex]);
	}
	
	
	// motion **********************************************************
	
	void HC_ServoRobot::moveOnTrigger(
			bool trigger,
			uint8_t groupIndex)
	{
		// check range
		if(groupIndex < mGroup_qty)
			mGroup_array[groupIndex].moveOnTrigger(trigger);	
	}
			
	void HC_ServoRobot::moveNow(
			uint8_t groupIndex)
	{
		// check range
		if(groupIndex < mGroup_qty)
			mGroup_array[groupIndex].moveNow();		
	}
	
	void HC_ServoRobot::stopOnTrigger(
			bool trigger,
			uint8_t groupIndex)
	{
		// check range
		if(groupIndex < mGroup_qty)
			mGroup_array[groupIndex].stopOnTrigger(trigger);		
	}

	void HC_ServoRobot::stopNow(
			uint8_t groupIndex)
	{
		// check range
		if(groupIndex < mGroup_qty)
			mGroup_array[groupIndex].stopNow();		
	}
