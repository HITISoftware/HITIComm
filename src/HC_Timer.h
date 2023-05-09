/*
 * HITIComm
 * HC_Timer.h
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

#ifndef HC_Timer_h
#define HC_Timer_h



// *****************************************************************************
// Class
// *****************************************************************************

class HC_Timer
{
	public:
		// constructor
		HC_Timer();
		HC_Timer(unsigned long duration);
		
		// setters
		void setDuration(unsigned long duration);
		void manualReset();
		void autoReset();
		void setStartTime(unsigned long startTime);
		void enableStartTimeControl(bool enable);
		void normalMode();
		void frequencyGeneratorMode();

		// getter
		unsigned long getElapsedTime() const;
		unsigned long getStartTime() const;
		unsigned long getDuration() const;
		/*bool getAutoReset() const;
		uint8_t getState() const;*/

		// management
		bool run();
		bool run(unsigned long duration);		// returns isRunning()
		bool delay(unsigned long duration);		// returns isEnding()
		void reset();

		bool isReady() const;
		bool isStarting() const;
		bool isRunning() const;
		bool isEnding();
		bool isOver() const;
		
	protected:

	private:
		// enum
		enum class State
		{
			HC_READY,
			HC_RUNNING,
			HC_OVER
		};

		// variables
		unsigned long mDuration = 1000;				// duration (ms)
		unsigned long mStartTime = 0;				// start time (ms)
		State mState = State::HC_READY;				// state (ready -> starting running -> over -> ready)
		bool mAutoReset = true;						// if true: autoreset when over
		bool mIsStarting = false;
		bool mIsEnding = false;
		bool mIsFrequencyGenerator = false;         // if true: trigger next start time calculation. Timer can then be used as a frequency generator
		bool mStartTimeControl_isEnabled = false;	// enabled by MultiTimer
};


#endif
