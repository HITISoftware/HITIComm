/*
 * HITIComm
 * HC_MultiTimer.cpp
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


#include "HC_MultiTimer.h"



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HCS_Time.h>

// HITIComm
#include "HC_Timer.h"



// *****************************************************************************
// Class Methods
// *****************************************************************************

	// constructor -------------------------------------------------------------

	// auto reset
	HC_MultiTimer::HC_MultiTimer(uint8_t qty)
	{
		setQuantity(qty);
	}


	// destructor --------------------------------------------------------------

	HC_MultiTimer::~HC_MultiTimer()
	{
		// clear dynamically allocated memory
		clear();
	}


	// setter ------------------------------------------------------------------
	
	void HC_MultiTimer::setQuantity(uint8_t qty)
	{
		// there must be at least 1 Timer
		mQty = (qty >= 1) ? qty : 1;

		// clear dynamic array
		clear();

		// create dynamic array
		mArray = new HC_Timer[mQty];

		// autoreset
		autoReset();

		// Start Time control by MultiTimer
		for (uint8_t i = 0; i < mQty; ++i)
			mArray[i].enableStartTimeControl(true);
	}

	void HC_MultiTimer::manualReset()
	{
		if (isReady())
		{
			mAutoReset = false;

			// all Timers must be in Manual Reset Mode
			for (uint8_t i = 0; i < mQty; ++i)
				mArray[i].manualReset();
		}
	}

	void HC_MultiTimer::autoReset()
	{
		if (isReady())
		{
			mAutoReset = true;

			// all Timers must be in Manual Reset Mode
			for (uint8_t i = 0; i < mQty; ++i)
				mArray[i].manualReset();
		}
	}
	

	// getters -----------------------------------------------------------------

	HC_Timer& HC_MultiTimer::getTimer(uint8_t i)
	{
		return (i < mQty) ? mArray[i] : mArray[0];
	}


	// memory deallocation -----------------------------------------------------

	void HC_MultiTimer::clear()
	{
		// if memory allocated, clear memory
		if (mArray != 0)
		{
			delete[] mArray;
			mArray = 0;
		}
	}

	
	// management --------------------------------------------------------------

	// set duration for Timer i
	// run Timer i
	bool HC_MultiTimer::execute(uint8_t i, unsigned long duration, Action action)
	{
		// check range
		if (i < mQty)
		{
			// if Timers before Timer i are over
			if (areOverBefore(i))
			{
				if (mArray[i].isReady())
				{
					// set duration of Timer i (done only if Ready) 
					mArray[i].setDuration(duration);

					// at first run, for Timer 0
					if (!mAlreadyRunOnce && (i == 0))
						// initialize with current time
						mArray[i].setStartTime(HCS_millis());
					else
					{
						// calculate Start Times
						if(i == 0)
							mArray[i].setStartTime(mArray[mQty - 1].getStartTime() + mArray[mQty - 1].getDuration());
						else
							mArray[i].setStartTime(mArray[i - 1].getStartTime() + mArray[i - 1].getDuration());
					}
				}

				// run Timer i
				bool runReturn = mArray[i].run();

				// if last Timer is Over
				if ((i == (mQty - 1)) && mArray[i].isOver() && mAutoReset)
				{
					// immediatelly run one more time while it is over to unset flag mIsEnding
					mArray[i].run();

					// reset all Timers
					reset();

					// set flag
					mAlreadyRunOnce = true;

					return (action == Action::HC_DELAY_BEFORE) ? true : false;
				}

				switch (action)
				{
					case Action::HC_RUN:
						return runReturn;

					case Action::HC_DELAY_BEFORE:
						return mArray[i].isEnding();

					case Action::HC_DELAY_AFTER:
						return mArray[i].isStarting();
				}
			}
		}

		return false;
	}


	// timer runs during action, return run()
	bool HC_MultiTimer::run(uint8_t i, unsigned long duration)
	{
		return execute(i, duration, Action::HC_RUN);
	}

	// timer runs before action, return isEnding()
	bool HC_MultiTimer::delay(uint8_t i, unsigned long duration)
	{
		return execute(i, duration, Action::HC_DELAY_BEFORE);
	}


	void HC_MultiTimer::reset()
	{
		for (uint8_t i = 0; i < mQty; ++i)
			mArray[i].reset();
	}


	// true if all Timers are ready
	bool HC_MultiTimer::isReady() const
	{
		for (uint8_t i = 0; i < mQty; ++i)
		{
			if (!mArray[i].isReady())
				return false;
		}

		return true;
	}

	// true if the 1st Timer is starting
	bool HC_MultiTimer::isStarting() const
	{
		return mArray[0].isStarting();
	}

	// true if at least 1 Timer is running
	bool HC_MultiTimer::isRunning() const
	{
		for (uint8_t i = 0; i < mQty; ++i)
		{
			if (mArray[i].isRunning())
				return true;
		}

		return false;
	}

	// true if the last Timer is ending
	bool HC_MultiTimer::isEnding() const
	{
		return mArray[mQty-1].isEnding();
	}

	// true if all Timers are over
	bool HC_MultiTimer::isOver() const
	{
		for (uint8_t i = 0; i < mQty; ++i)
		{
			if (!mArray[i].isOver())
				return false;
		}

		return true;
	}

	// true if all Timers before Timer i are over
	bool HC_MultiTimer::areOverBefore(uint8_t i)
	{
		if (i == 0)
			return true;

		for (uint8_t j = 0; j < i; ++j)
		{ 
			if (!mArray[j].isOver())
				return false;
		}

		return true;
	}
