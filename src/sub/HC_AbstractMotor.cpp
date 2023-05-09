/*
 * HITIComm
 * HC_AbstractMotor.cpp
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



#include "sub\HC_AbstractMotor.h"



// *****************************************************************************
// Class Methods
// *****************************************************************************

	// constructor ********************************************************
	
	HC_AbstractMotor::HC_AbstractMotor() {}
	
	
	// destructor *********************************************************
	
	HC_AbstractMotor::~HC_AbstractMotor() {}
	

	// setters ************************************************************

	void HC_AbstractMotor::init(
		uint8_t id,
		uint8_t pin,
		bool invertedDirection,
		float offset)
	{
		mID = id;
		mPin = pin;
		mInvertedDirection = invertedDirection;
		mOffset = offset;
	}
	
	void HC_AbstractMotor::absolutePosition(float posSetpoint)
	{
		HC_MotionManager::absolutePosition(
				getRawPosition(),								// absolute start position (°)
				getRawPositionSetpoint(posSetpoint, true));		// absolute raw position setpoint
	}

	void HC_AbstractMotor::relativePosition(float posSetpoint)
	{
		HC_MotionManager::relativePosition(
				getRawPosition(),								// absolute start position (°)
				getRawPositionSetpoint(posSetpoint, false));	// relative raw position setpoint
	}

	void HC_AbstractMotor::travelLimits(
		float min,
		float max)
	{
		// set position limits
		if (mInvertedDirection)
			HC_MotionManager::travelLimits(getRawPosition(max), getRawPosition(min));
		else
			HC_MotionManager::travelLimits(getRawPosition(min), getRawPosition(max));
	}


	// getters *********************************************************
	
	uint8_t HC_AbstractMotor::getID() const { return mID; }

	// target position and speed
	float HC_AbstractMotor::getTargetPosition() const	{ return getUserPosition(HC_MotionManager::getTargetPosition()); }
	float HC_AbstractMotor::getTargetSpeed() const		{ return mInvertedDirection ? -HC_MotionManager::getTargetSpeed() : HC_MotionManager::getTargetSpeed(); }

	// current speed
	float HC_AbstractMotor::getCurrentSpeed() const				{ return isMoving() ? getTargetSpeed() : 0; }
	float HC_AbstractMotor::getContinuousCurrentSpeed() const	{ return isMoving() ? getContinuousTargetSpeed() : 0; }

	// convert position: raw <-> user
	float HC_AbstractMotor::getUserPosition(float raw_position) const { return mInvertedDirection ? 180 - (raw_position + mOffset) : raw_position - mOffset; }
	float HC_AbstractMotor::getRawPosition(float user_position) const { return mInvertedDirection ? 180 - (user_position + mOffset) : user_position + mOffset; }

	// raw position setpoint
	float HC_AbstractMotor::getRawPositionSetpoint(float user_position, bool isPosMode_absolute)
	{
		// ABSOLUTE
		if (isPosMode_absolute)
			return getRawPosition(user_position);

		// RELATIVE
		else
			return mInvertedDirection ? -user_position : user_position;
	}

	// limit -/+
	bool HC_AbstractMotor::isNegLimitReached()
	{
		checkTravelLimits();

		return mInvertedDirection ? HC_MotionManager::isPosLimitReached() : HC_MotionManager::isNegLimitReached();
	}

	bool HC_AbstractMotor::isPosLimitReached()
	{
		checkTravelLimits();

		return mInvertedDirection ? HC_MotionManager::isNegLimitReached() : HC_MotionManager::isPosLimitReached();
	}


	// motion *********************************************************

	void HC_AbstractMotor::moveContinuously(bool trigger, bool direction)
	{
		if (mInvertedDirection)
			direction = !direction;

		HC_MotionManager::moveContinuously(
			trigger,
			direction,
			getRawPosition()); // initial position)
	}

	void HC_AbstractMotor::movePositive(bool trigger) { moveContinuously(trigger, true); }
	void HC_AbstractMotor::moveNegative(bool trigger) { moveContinuously(trigger, false); }
