/*
 * Arduino.c
 *
 *  Created on: 04.09.2021
 *      Author: dridders
 */

#include "Arduino.h"

void nothing() {}

extern "C" {
	volatile voidFuncPtr lala;
	static volatile voidFuncPtr intFunc[4][11] = {
			{ nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing },
			{ nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing },
			{ nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing },
			{ nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing },
	};

	void attachInterrupt(uint8_t pinNum, void (*userFunc)(void), int mode)
	{
		IRQn_Type irq;
		switch (digitalPinToPort(pinNum))
		{
		case 0:
			irq = EINT0_IRQn;
			break;
		case 1:
			irq = EINT1_IRQn;
			break;
		case 2:
			irq = EINT2_IRQn;
			break;
		default:
			irq = EINT3_IRQn;
			break;
		}
		intFunc[digitalPinToPort(pinNum)][digitalPinToPinNum(pinNum)] = userFunc;
		PinInterruptMode lpcMode = INTERRUPT_LEVEL_HIGH;
		switch (mode)
		{
		case LOW:
			lpcMode = INTERRUPT_LEVEL_LOW;
			break;
		case CHANGE:
			lpcMode = INTERRUPT_EDGE_BOTH;
			break;
		case RISING:
			lpcMode = INTERRUPT_EDGE_RISING;
			break;
		case FALLING:
			lpcMode = INTERRUPT_EDGE_FALLING;
		}
		pinInterruptMode(pinNum, lpcMode | INTERRUPT_ENABLED);
		enableInterrupt(irq);
	}

	void detachInterrupt(uint8_t pinNum){
		intFunc[digitalPinToPort(pinNum)][digitalPinToPinNum(pinNum)] = nothing;
		pinDisableInterrupt(pinNum);
	}

	void handleIRQ(LPC_GPIO_TypeDef* port, int portNum)
	{
		for (int i = 0; i < 11; i++)
		{
			if (port->MIS & (1 << i)) {
				intFunc[portNum][i]();
				port->IC = (1 << i);
			}
		}
	}

	void PIOINT0_IRQHandler() {
		handleIRQ(LPC_GPIO0, 0);
	}

	void PIOINT1_IRQHandler() {
		handleIRQ(LPC_GPIO1, 1);
	}
	void PIOINT2_IRQHandler() {
		handleIRQ(LPC_GPIO2, 2);
	}
	void PIOINT3_IRQHandler() {
		handleIRQ(LPC_GPIO3, 3);
	}
}
