/*
 * HITIComm
 * HC_Sram.cpp
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


#include "HC_Sram.h"



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// AVR
#include <stddef.h>
#include <string.h>

// HITICommSupport
#include <HCS_Atmel.h>



// *****************************************************************************
// External Variables
// *****************************************************************************

#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
    extern char __data_start, __data_end;   // .data
    extern char __bss_start, __bss_end;     // .bss

    extern char __heap_start;               // .noinit
    extern char __heap_end;                 // seems unused
    extern char* __brkval;                  // break value (pointer to top of the heap)

    extern char* __malloc_heap_start;
    extern char* __malloc_heap_end;
    extern size_t __malloc_margin;
#elif defined(ARDUINO_ARCH_SAMD)
    extern uint32_t __data_start__, __data_end__;   // .data
    extern uint32_t __bss_start__, __bss_end__;     // .bss

    extern uint32_t __end__;                        // .heap (= heap start)
    extern uint32_t __HeapLimit;                    // .heap (= heap end after linking)

    extern "C" void* sbrk(int incr);                // break value (pointer to top of the heap)
    
    extern uint32_t __malloc_sbrk_start;
    extern uint32_t __malloc_free_list;

    extern uint32_t __StackTop;                     // = ramstart + ram size
    extern uint32_t __StackLimit;                   // = StackTop - .stack_dummy size  

    //extern uint32_t __get_MSP(void);                // get Main Stack Pointer (there are 2 SP, but only that one is used by Arduino)
#endif


// *****************************************************************************
// static variables
// *****************************************************************************

// 3 probes
unsigned int HC_Sram::sHeapBreakValuePointer[3] = {0};
unsigned int HC_Sram::sStackPointer[3] = {0};
unsigned int HC_Sram::sFreeRAM[3] = {0};

// Flag (to monitor value changes)
bool HC_Sram::sSRAM_hasChanged = false;



// *****************************************************************************
// Instanciates an object so that it can be reused in other files (forward declared in .h)
// *****************************************************************************

HC_Sram HC_sram;



// *****************************************************************************
// Getter
// *****************************************************************************

#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
    unsigned int HC_Sram::getAddress_DataStart()        { return (unsigned int) &__data_start; }
    unsigned int HC_Sram::getAddress_DataEnd()          { return (unsigned int) &__data_end; }
    unsigned int HC_Sram::getAddress_BssStart()         { return (unsigned int) &__bss_start; }
    unsigned int HC_Sram::getAddress_BssEnd()           { return (unsigned int) &__bss_end; }
    unsigned int HC_Sram::getAddress_HeapStart()        { return (unsigned int) &__heap_start; }
    unsigned int HC_Sram::getAddress_HeapEnd()          { return (unsigned int) &__heap_end; }

    unsigned int HC_Sram::getAddress_MallocHeapStart()  { return (unsigned int) __malloc_heap_start; }
    unsigned int HC_Sram::getAddress_MallocHeapEnd()    { return (unsigned int) __malloc_heap_end; }
    unsigned int HC_Sram::getMallocMargin()             { return (unsigned int) __malloc_margin; }
#elif defined(ARDUINO_ARCH_SAMD)
    // on SAMD, unsigned int = uint32_t
    unsigned int HC_Sram::getAddress_DataStart()        { return (unsigned int) &__data_start__; }
    unsigned int HC_Sram::getAddress_DataEnd()          { return (unsigned int) &__data_end__; }
    unsigned int HC_Sram::getAddress_BssStart()         { return (unsigned int) &__bss_start__; }
    unsigned int HC_Sram::getAddress_BssEnd()           { return (unsigned int) &__bss_end__; }
    unsigned int HC_Sram::getAddress_HeapStart()        { return (unsigned int) &__end__; }
    unsigned int HC_Sram::getAddress_HeapEnd()          { return (unsigned int) &__HeapLimit; }

    unsigned int HC_Sram::getAddress_MallocHeapStart()  { return 0; }
    unsigned int HC_Sram::getAddress_MallocHeapEnd()    { return 0; }
    unsigned int HC_Sram::getMallocMargin()             { return 0; }
#endif

// on SAMD, unsigned int = uint32_t
unsigned int HC_Sram::getAddress_HeapBreakValue(unsigned char i)    { return (i > 2) ? sHeapBreakValuePointer[2] : sHeapBreakValuePointer[i]; }
unsigned int HC_Sram::getStackPointer(unsigned char i)              { return (i > 2) ? sStackPointer[2] : sStackPointer[i]; }
unsigned int HC_Sram::getFreeRAM(unsigned char i)                   { return (i > 2) ? sFreeRAM[2] : sFreeRAM[i]; }



// *****************************************************************************
// Functions
// *****************************************************************************

// to call anywhere in the code to measure Stack Pointer, Break Value Pointer and Free Ram
// 3 probes can be set (0,1,2) 
void HC_Sram::setProbe(unsigned char i)
{
    // limit i
    if (i > 2) i = 2;

#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
    // measure Stack Pointer
    unsigned int stackPointer;
    #ifdef SP
        stackPointer = SP;
    #else
        bool varInStack;
        stackPointer = (unsigned int) &varInStack;
    #endif

    // measure Heap Break Value
    unsigned int heapBreakValue = (unsigned int)__brkval;

#elif defined(ARDUINO_ARCH_SAMD)
    // measure Stack Pointer
    unsigned int stackPointer = HCS_getMSP(); // on SAMD, unsigned int = uint32_t

    // measure Heap Break Value
    unsigned int heapBreakValue = (unsigned int)sbrk(0);
#endif

    // check for changes
    if ((sStackPointer[i] != stackPointer) || (sHeapBreakValuePointer[i] != heapBreakValue))
    {
        sSRAM_hasChanged = true;
        sStackPointer[i] = stackPointer;
        sHeapBreakValuePointer[i] = heapBreakValue;

        // calculate Free RAM
        sFreeRAM[i] = (sHeapBreakValuePointer[i] == 0) ? (sStackPointer[i] - getAddress_HeapStart()) : (sStackPointer[i] - sHeapBreakValuePointer[i]);
    }
}


// flag 
bool HC_Sram::hasChanged()
{
    return HCI_readAndConsume(&sSRAM_hasChanged);
}
