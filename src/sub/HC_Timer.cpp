/*
 * HITIComm
 * HC_Timer.cpp
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


#include "HC_Timer.h"



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HCS_Time.h>

// HITIComm
#include "HC_Toolbox.h"



// *****************************************************************************
// Class Methods
// *****************************************************************************


	// constructor -------------------------------------------------------------
	
	// 1000 ms, autoreset
	HC_Timer::HC_Timer()
	{
	}

	// autoreset
	HC_Timer::HC_Timer(unsigned long duration):
			mDuration(duration)
	{ 
	}


	// setter ------------------------------------------------------------------
	
	void HC_Timer::setDuration(unsigned long duration)
	{
		if (isReady())
			mDuration = duration;
	}

	void HC_Timer::setStartTime(unsigned long startTime)
	{
		if (isReady() && mStartTimeControl_isEnabled)
			mStartTime = startTime;
	}

	void HC_Timer::manualReset()
	{
		if (isReady())
			mAutoReset = false;
	}

	void HC_Timer::autoReset()
	{
		if (isReady())
			mAutoReset = true;
	}

	void HC_Timer::enableStartTimeControl(bool enable)
	{
		mStartTimeControl_isEnabled = enable;
	}

	void HC_Timer::normalMode()
	{
		mIsFrequencyGenerator = false;
	}

	void HC_Timer::frequencyGeneratorMode()
	{
		mIsFrequencyGenerator = true;
	}


	// getter ------------------------------------------------------------------
	
	unsigned long HC_Timer::getElapsedTime() const
	{
		switch (mState)
		{
			default:
			case State::HC_READY:
				return 0;
				
			case State::HC_RUNNING:
				return HCS_millis() - mStartTime;

			case State::HC_OVER:
				return mDuration;
		}
	}

	unsigned long HC_Timer::getStartTime() const	{ return mStartTime; }
	unsigned long HC_Timer::getDuration() const		{ return mDuration; }
/*
	bool HC_Timer::getAutoReset() const				{ return mAutoReset; }
	uint8_t HC_Timer::getState() const					{ return (uint8_t) mState; }
	*/

	
	// management --------------------------------------------------------------
	
	// millis() overflows after approx 50 days
	// micros() overflows after approx 70 min
	
	// return true if Running
	bool HC_Timer::run()
	{
		switch (mState)
		{
			case State::HC_READY:
				// set start time, if not set by MultiTimer
				if (!mStartTimeControl_isEnabled)
				{
					// if Frequency Generator AND not the first run
					if (mIsFrequencyGenerator && (mStartTime != 0))
						mStartTime = mStartTime + mDuration;	// calculate in autoreset
					else
						mStartTime = HCS_millis();		            // initialize start time
				}

				// Ready -> Running
				mState = State::HC_RUNNING;
				mIsStarting = true;
				mIsEnding = false;

				return true;

			case State::HC_RUNNING:
				mIsStarting = false;

				// when time is over
				if (HCS_millis() - mStartTime >= mDuration)
				{
					mIsEnding = true;

					// if autoreset
					if (mAutoReset || mIsFrequencyGenerator)
					{
						// Running -> Ready 
						mState = State::HC_READY;
						return true; // continuous Running
					}
					else
					{
						// Running -> Over
						mState = State::HC_OVER;
						return false;
					}
				}

				return true;

			// useful in manual reset
			case State::HC_OVER:
				mIsEnding = false;

				return false;

			default:
				return false;
		}
	}


	// timer runs during action, return run()
	bool HC_Timer::run(unsigned long duration)
	{
		// set duration (possible only if in Ready state)
		setDuration(duration);

		// run Timer
		return run();
	}

	// timer runs before action, return isEnding()
	bool HC_Timer::delay(unsigned long duration)
	{
		// set duration (possible only if in Ready state)
		setDuration(duration);

		// run Timer
		run();

		return isEnding();
	}


	void HC_Timer::reset()
	{
		mState = State::HC_READY;
	}


	bool HC_Timer::isReady() const		{ return mState == State::HC_READY; }
	bool HC_Timer::isStarting() const	{ return mIsStarting; }
	bool HC_Timer::isRunning() const	{ return mState == State::HC_RUNNING; }
	bool HC_Timer::isEnding()			{ return HCI_readAndConsume(&mIsEnding); }
	bool HC_Timer::isOver() const		{ return mState == State::HC_OVER; }	
