/*
 * HITIComm
 * HC_SignalFilter.h
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

#ifndef HC_SignalFilter_h
#define HC_SignalFilter_h



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HITICommSupport.h>



// *****************************************************************************
// Class
// *****************************************************************************

class HC_SignalFilter
{
	public:
		// constructor
		HC_SignalFilter();
		HC_SignalFilter(uint8_t size);
		
		// setters
		void setBufferSize(uint8_t size);
		void clear();

		// getters
		uint8_t getBufferSize() const;
		 
		// filter
		float average(float data);
		float median(float data);
		
	protected:
	
	private:
		// methods *********************************************************
		void addData(float data);

		void sort(
				float* array, 
				bool* mask,
				uint8_t size, 
				bool ascending);

		void sortAscending(
				float* array,
				bool* mask,
				uint8_t size);
		void sortDescending(
				float* array,
				bool* mask,
				uint8_t size);

		float average(
				float* array,
				bool* mask,
				uint8_t size);
		float median(
				float* array,
				bool* mask,
				uint8_t size);
	
		// variables *******************************************************
		uint8_t mSize;	// buffer size
		float* mBuffer;		// array containing the data to process
		bool* mBufferMask;	// mask telling which values of mBuffer should be processed
};

#endif
