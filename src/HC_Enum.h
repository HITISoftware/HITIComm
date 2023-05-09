/*
 * HITIComm
 * HC_Enum.h
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


#ifndef HC_Enum_h
#define HC_Enum_h



// *****************************************************************************
// IO
// *****************************************************************************

typedef enum
{
	DIGITAL	= 0,
	PWM		= 1
}HC_OutputType_t;



// *****************************************************************************
// EEPROM
// *****************************************************************************

/*
enum HC_ConfigSpace
{
	HC_CONFIGSPACE_PINMODE			= 0,
	HC_CONFIGSPACE_INPUTMODE		= 1,
	HC_CONFIGSPACE_OUTPUTTYPE		= 2,
	HC_CONFIGSPACE_SERVOMODE		= 3,
	HC_CONFIGSPACE_CONFIGREGISTER	= 4,
	HC_CONFIGSPACE_QTY				= 5
};
*/
typedef enum
{
	HC_USERSPACE_BOOLEAN			= 0,
	HC_USERSPACE_BYTE				= 1,
	HC_USERSPACE_INTEGER			= 2,
	HC_USERSPACE_LONG				= 3,
	HC_USERSPACE_FLOAT				= 4,
	HC_USERSPACE_STRING				= 5,
	HC_USERSPACE_QTY				= 6
}HC_UserSpace;


#endif
