/*
 * HITIComm
 * HC_MultiTimer.h
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


#ifndef HC_Timers_h
#define HC_Timers_h



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HITICommSupport.h>



// *****************************************************************************
// Forward declaration of class
// *****************************************************************************

class HC_Timer;	// because used as a pointer (to an array)



// *****************************************************************************
// Class
// *****************************************************************************


class HC_MultiTimer
{
	public:
		// constructor
		HC_MultiTimer(uint8_t qty);

		// destructor
		~HC_MultiTimer();

		// setters
		void setQuantity(uint8_t qty);
		void manualReset();
		void autoReset();

		// getters
		HC_Timer& getTimer(uint8_t i);

		// management
		bool run(uint8_t i, unsigned long duration);		// returns isRunning()
		bool delay(uint8_t i, unsigned long duration);		// returns isEnding()
		void reset();

		bool isReady() const;
		bool isStarting() const;
		bool isRunning() const;
		bool isEnding() const;
		bool isOver() const;
		
	protected:
	
	private:
		// enum
		enum Action
		{
			HC_RUN,
			HC_DELAY_BEFORE,
			HC_DELAY_AFTER
		};

		// memory deallocation
		void clear();	

		// management
		bool execute(uint8_t i, unsigned long duration, Action action);
		bool areOverBefore(uint8_t i);

		// variables 
		uint8_t mQty = 1;					// Timers quantity
		HC_Timer* mArray;				// array of Timers
		bool mAutoReset = true;			// if true: autoreset when over
		bool mAlreadyRunOnce = false;	// used to control Start Time calculation
};


#endif
