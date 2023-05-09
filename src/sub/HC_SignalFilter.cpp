/*
 * HITIComm
 * HC_SignalFilter.cpp
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


#include "HC_SignalFilter.h"



// *****************************************************************************
// Class Methods
// *****************************************************************************


	// constructor ********************************************************
	
	HC_SignalFilter::HC_SignalFilter()
	{
		setBufferSize(10);
	}

	HC_SignalFilter::HC_SignalFilter(uint8_t bufferSize)
	{
		setBufferSize(bufferSize);
	}
	

	// setter *************************************************************

	void HC_SignalFilter::setBufferSize(uint8_t size)
	{
		// min buffer size : 3
		mSize = (size < 3) ? 3 : size;
		
		// clear and create dynamic array
		clear();
		mBuffer = new float[mSize];
		mBufferMask = new bool[mSize];

		// init array
		for (uint8_t i = 0; i < mSize; ++i)
		{
			mBuffer[i] = 0.0;
			mBufferMask[i] = false;
		}
	}
	
	void HC_SignalFilter::clear()
	{
		// if memory allocated, clear memory
		if (mBuffer != 0)
		{
			delete mBuffer;
			delete mBufferMask;
			mBuffer = 0;
			mBufferMask = 0;
		}
	}


	// getter *************************************************************

	uint8_t HC_SignalFilter::getBufferSize() const	{ return mSize; }
	
	
	// filter *************************************************************

	float HC_SignalFilter::average(float data)
	{
		addData(data);
		return average(mBuffer, mBufferMask, mSize);
	}
	
	float HC_SignalFilter::median(float data)
	{
		addData(data);
		return median(mBuffer, mBufferMask, mSize);
	}

		
	// array processing ************************************************

	void HC_SignalFilter::addData(float data)
	{
		// data shift to left (last data is left unchanged)
		for (uint8_t i = 0; i < mSize; ++i)
		{
			if (i < (mSize - 1))
			{
				mBuffer[i] = mBuffer[i + 1];
				mBufferMask[i] = mBufferMask[i + 1];;
			}
		}

		// put new data at last position
		mBuffer[mSize - 1] = data;
		mBufferMask[mSize - 1] = true;
	}


	void HC_SignalFilter::sort(float* array, bool* mask, uint8_t size, bool ascending)
	{
		// method:
		// - ascending: value at index i is the min value of all values (j) at its right
		// - descending: value at index i is the max value of all values (j) at its right
		// - put all non-assigned values at the end
		
		for(uint8_t i = 0; i < (size-1); ++i)
		{
			for(uint8_t j = i; j < size; ++j)
			{
				if(mask[j])
				{
					if(mask[i])
					{
						// compare and invert values at index i and j
						if(((array[i] > array[j]) && ascending) ||
							((array[i] < array[j]) && !ascending))
						{
							float buffer = array[i];
							array[i] = array[j];
							array[j] = buffer;
						}
					}
					else
					{
						// invert non-assigned value at index i, with the first assigned value found at its right
						array[i] = array[j];
						mask[i] = true;
						array[j] = 0.0;
						mask[j] = false;
						break;
					}
				}
			}
		}
	}

	void HC_SignalFilter::sortAscending(float* array, bool* mask, uint8_t size)
	{
		sort(array, mask, size, true);
	}

	void HC_SignalFilter::sortDescending(float* array, bool* mask, uint8_t size)
	{
		sort(array, mask, size, false);
	}


	float HC_SignalFilter::average(float* array, bool* mask, uint8_t size)
	{
		// sum data and count number of available data
		float sum = 0.0;
		uint8_t counter = 0;
		for (uint8_t i = 0; i < size; ++i)
		{
			if (mask[i])
			{
				sum += array[i];
				counter ++;
			}
		}

		return (counter == 0) ? 0 : (sum / ((float)counter));
	}

	float HC_SignalFilter::median(float* array, bool* mask, uint8_t size)
	{
		// sort array in ascending order
		sortAscending(array, mask, size);

		// count amount of non-NULL_POINTER data
		uint8_t counter = 0;
		for (uint8_t i = 0; i < size; ++i)
		{
			if (mask[i])
				counter++;
		}

		return array[counter / 2];
	}