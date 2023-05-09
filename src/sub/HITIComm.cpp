/*
 * HITIComm
 * HITIComm.cpp
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


#include "HITIComm.h"



// *****************************************************************************
// Variables
// *****************************************************************************

static HC_Protocol protocol;



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include "HCS_Serial.h"



// *****************************************************************************
// Methods
// *****************************************************************************

void HC_begin(long baudrate)
{
    // Set Serial baudrate
    HCS_Serial_setBaudrate(baudrate);

	// instantiates all Servos
	HCI_initializeServos(true);

    // inform computer that Arduino has started
    protocol.sendMessage_BoardHasStarted();
}


void HC_begin()
{
    HC_begin(250000);
}


void HC_communicate()
{
    protocol.communicate();
}


/* 
HITIPanel accept the following baudrates:
(errors on exact baudrate values are indicated for Atmega328P at 16MHz)
    300
    600,
    1200,
    2400,		// -0.1%
    4800,		//  0.2%
    9600,		//  0.2%
    14400,	    //  0.6%
    19200,	    //  0.2%
    28800,	    // -0.8%
    38400,	    //  0.2%
    57600,	    //  2.1%
    115200,	    // -3.5%
    230400,	    //  8.5%
    250000,	    //  0%
    500000,	    //  0%
    1000000,	//  0%
*/