/*
 * HITIComm
 * HC_CompilationTriggers.h
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

#ifndef HC_CompilationTriggers_h
#define HC_CompilationTriggers_h



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HITICommSupport.h>



// *****************************************************************************
// Compilation triggers
// *****************************************************************************

// to comment to prevent compiling
//#define HC_MAIN_TRY_COMPILE
#define HC_EEPROM_TRY_COMPILE

#ifdef HC_MAIN_TRY_COMPILE
	#define HC_STRINGMESSAGE_COMPILE
	#define HC_ARDUINOTIME_COMPILE
#endif

// if no EEPROM on-board
#if defined(HC_EEPROM_ONBOARD) && defined(HC_EEPROM_TRY_COMPILE)
	#define HC_EEPROM_COMPILE
#endif

#endif