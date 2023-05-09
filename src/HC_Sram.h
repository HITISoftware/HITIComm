/*
 * HITIComm
 * HC_Sram.h
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

#ifndef HC_SRAM_h
#define HC_SRAM_h



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITIComm
#include "HC_Toolbox.h"



// *****************************************************************************
// Class
// *****************************************************************************

class HC_Sram
{
    public:
        // static getter
        static unsigned int getAddress_DataStart();
        static unsigned int getAddress_DataEnd();
        static unsigned int getAddress_BssStart();
        static unsigned int getAddress_BssEnd();
        static unsigned int getAddress_HeapStart();
        static unsigned int getAddress_HeapEnd();

        static unsigned int getAddress_MallocHeapStart();
        static unsigned int getAddress_MallocHeapEnd();
        static unsigned int getMallocMargin();

        // to call anywhere in the code to measure Stack Pointer, Brake Value Pointer and Free Ram
        // 3 probes can be set (0,1,2) 
        static void setProbe(unsigned char i);

        static unsigned int getAddress_HeapBreakValue(unsigned char i);
        static unsigned int getStackPointer(unsigned char i);
        static unsigned int getFreeRAM(unsigned char i);

        // flag
        bool hasChanged();

    protected:
    private:
        // static variables
        static unsigned int sStackPointer[3];
        static unsigned int sHeapBreakValuePointer[3];
        static unsigned int sFreeRAM[3];
        static bool sSRAM_hasChanged; // Flag (to monitor value changes)
};


// *****************************************************************************
// Forward declare an object
// *****************************************************************************

extern HC_Sram HC_sram;


#endif