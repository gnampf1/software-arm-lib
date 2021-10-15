/*
 *  interrupt.h - Functions and classes for interrupt handling
 *
 *  Copyright (c) 2014 Stefan Taferner <stefan.taferner@gmx.at>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3 as
 *  published by the Free Software Foundation.
 */
#ifndef sblib_interrupt_h
#define sblib_interrupt_h

#include <sblib/platform.h>
#include <sblib/types.h>
#include <sblib/utils.h>

/**
 * Interrupt handlers have fixed names. You need to give your interrupt handler the
 * correct name, then it is used automatically when it's interrupt is enabled. The
 * interrupt handler must be declared as extern "C":
 *
 * extern "C" void TIMER16_1_IRQHandler() { ...handle the interrupt.... }
 *
 * Use the following function names when writing an interrupt handler:
 *
 * ADC_IRQHandler()
 * BOD_IRQHandler()
 * CAN_IRQHandler()
 * I2C_IRQHandler()
 * PIOINT0_IRQHandler()
 * PIOINT1_IRQHandler()
 * PIOINT2_IRQHandler()
 * PIOINT3_IRQHandler()
 * SSP0_IRQHandler()
 * SSP1_IRQHandler()
 * TIMER16_0_IRQHandler()
 * TIMER16_1_IRQHandler()
 * TIMER32_0_IRQHandler()
 * TIMER32_1_IRQHandler()
 * UART_IRQHandler()
 * WAKEUP_IRQHandler()
 * WDT_IRQHandler()
 */

/**
 * Disable all interrupts.
 */
void noInterrupts();

/**
 * Re-enable all interrupts after they have been disabled with noInterrupts().
 */
void interrupts();

/**
 * Wait for an interrupt. Puts the processor to sleep until an interrupt occurs.
 */
void waitForInterrupt();

/**
 * Enable an interrupt.
 *
 * @param interruptType - the interrupt to enable: TIMER_16_0_IRQn, I2C_IRQn, ...
 */
void enableInterrupt(IRQn_Type interruptType);

/**
 * Disable an interrupt.
 *
 * @param interruptType - the interrupt to enable: TIMER_16_0_IRQn, I2C_IRQn, ...
 */
void disableInterrupt(IRQn_Type interruptType);

/**
 * Clear the pending status of an interrupt.
 *
 * @param interruptType - the interrupt to clear: TIMER_16_0_IRQn, I2C_IRQn, ...
 */
void clearPendingInterrupt(IRQn_Type interruptType);

/**
 * Set the pending status of an interrupt.
 *
 * @param interruptType - the interrupt to set: TIMER_16_0_IRQn, I2C_IRQn, ...
 */
void setPendingInterrupt(IRQn_Type interruptType);

/**
 * @fn bool isInsideInterrupt()
 * @brief Returns if within an Isr
 *
 * @return true if called inside a Isr otherwise false
 */
bool isInsideInterrupt(void);

/**
 * @fn bool getInterruptEnabled(IRQn_Type)
 * @brief Returns the enabled status of an interrupt
 *        doesnt work for NonMaskableInt_IRQn,
 *                        HardFault_IRQn,
 *                        SVCall_IRQn,
 *                        PendSV_IRQn,
 *                        SysTick_IRQn
 *
 * @param interruptType - the interrupt to get enabled status must be >=0
 * @return true if interrupt is enabled, otherwise false
 */
bool getInterruptEnabled(IRQn_Type interruptType);

/**
 * This define creates an interrupt handler that calls a callback function.
 *
 * @param handler - the name of the interrupt handler, e.g. TIMER16_0_IRQHandler
 * @param callback - the function to call in the interrupt handler
 */
#define CREATE_INTERRUPT_HANDLER(handler, callback) \
extern "C" void handler() { callback; }


//
// Inline functions
//

ALWAYS_INLINE void noInterrupts()
{
    __disable_irq();
}

ALWAYS_INLINE void interrupts()
{
    __enable_irq();
}

ALWAYS_INLINE void waitForInterrupt()
{
    __WFI();
}

ALWAYS_INLINE void enableInterrupt(IRQn_Type interruptType)
{
    NVIC->ISER[0] = 1 << (interruptType & 0x1f);
}

ALWAYS_INLINE void disableInterrupt(IRQn_Type interruptType)
{
    NVIC->ICER[0] = 1 << (interruptType & 0x1f);
}

ALWAYS_INLINE void clearPendingInterrupt(IRQn_Type interruptType)
{
    NVIC->ICPR[0] = 1 << (interruptType & 0x1f);
}

ALWAYS_INLINE void setPendingInterrupt(IRQn_Type interruptType)
{
    NVIC->ICPR[0] = 1 << (interruptType & 0x1f);
}

ALWAYS_INLINE bool isInsideInterrupt(void)
{
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0 ;
}

ALWAYS_INLINE bool getInterruptEnabled(IRQn_Type interruptType)
{
    if (interruptType >= 0)
    {
        return ((NVIC->ICER[0] >> (interruptType & 0x1f)) == 1);
    }
    else
    {
        fatalError();
    }
}
#endif /*sblib_interrupt_h*/
