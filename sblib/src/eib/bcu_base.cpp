/*
 *  bcu.h - EIB bus coupling unit.
 *
 *  Copyright (c) 2014 Stefan Taferner <stefan.taferner@gmx.at>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3 as
 *  published by the Free Software Foundation.
 */

#define INSIDE_BCU_CPP
#include <sblib/io_pin_names.h>
#include <sblib/eib/bcu_base.h>
#include <sblib/eib/user_memory.h>
#include <sblib/eib/addr_tables.h>
#include <sblib/internal/functions.h>
#include <sblib/internal/variables.h>
#include <sblib/internal/iap.h>
#include <string.h>

#ifdef DUMP_TELEGRAMS
#include <sblib/serial.h>
#endif

// The interrupt handler for the EIB bus access object
BUS_TIMER_INTERRUPT_HANDLER(TIMER16_1_IRQHandler, bus);

extern unsigned int writeUserEepromTime;
extern volatile unsigned int systemTime;

BcuBase::BcuBase()
:progButtonDebouncer()
{
    progPin = PIN_PROG;
    progPinInv = true;
    enabled = false;
}

// The method begin_BCU() is renamed during compilation to indicate the BCU type.
// If you get a link error then the library's BCU_TYPE is different from your application's BCU_TYPE.
void BcuBase::begin_BCU(int manufacturer, int deviceType, int version)
{
	_begin(); // load flash/rom data to usereeprom, init bcu

#ifdef DUMP_TELEGRAMS
    serial.println("Telegram dump enabled");
#endif

    sendTelegram[0] = 0;
    sendCtrlTelegram[0] = 0;

    connectedSeqNo = 0;
    incConnectedSeqNo = false;
    lastAckSeqNo = -1;

    connectedAddr = 0;

    userRam.status = BCU_STATUS_LL | BCU_STATUS_TL | BCU_STATUS_AL | BCU_STATUS_USR;
    userRam.deviceControl = 0;
    userRam.runState = 1;

    userEeprom.runError = 0xff;
    userEeprom.portADDR = 0;

    userEeprom.manufacturerH = manufacturer >> 8;
    userEeprom.manufacturerL = manufacturer;

    userEeprom.deviceTypeH = deviceType >> 8;
    userEeprom.deviceTypeL = deviceType;

    userEeprom.version = version;

#if BCU_TYPE != BCU1_TYPE
    unsigned int serial;
    iapReadPartID(& serial);
    memcpy (userEeprom.serial, &serial, 4);
    userEeprom.serial[4] = SBLIB_VERSION >> 8;
    userEeprom.serial[5] = SBLIB_VERSION;

    userRam.peiType = 0;     // PEI type: 0=no adapter connected to PEI.
    userEeprom.appType = 0;  // Set to BCU2 application. ETS reads this when programming.
#endif

    writeUserEepromTime = 0;
    enabled = true;
    bus.begin();
    progButtonDebouncer.init(1);
}

void BcuBase::_begin()
{

}

void BcuBase::end()
{
    enabled = false;

    bus.end();
}

void BcuBase::setOwnAddress(int addr)
{
    userEeprom.addrTab[0] = addr >> 8;
    userEeprom.addrTab[1] = addr;
#if BCU_TYPE != BCU1_TYPE
    if (userEeprom.loadState[OT_ADDR_TABLE] == LS_LOADING)
    {
        byte * addrTab =  addrTable() + 1;

        * (addrTab + 0)  = addr >> 8;
        * (addrTab + 1)  = addr;
    }
#endif
    userEeprom.modified();

    bus.ownAddr = addr;
}

void BcuBase::loop()
{
	if (!enabled)
		return;


#ifdef DUMP_TELEGRAMS
	extern unsigned char telBuffer[];
	extern unsigned int telLength ; // db_state;
	extern unsigned int telRXtime;
	extern bool telcollision;

	if (telLength > 0)
	{
		//serial.println();
		serial.print("RCV: (");

		serial.print(telRXtime, DEC, 8);
		serial.print(") ");
		if (telcollision)  serial.print("collision ");

		for (unsigned int i = 0; i < telLength; ++i)
		{
			if (i) serial.print(" ");
			serial.print(telBuffer[i], HEX, 2);
		}
		serial.println();
		telLength = 0;
	}
#endif

#ifdef DEBUG_BUS
	// trace buffer contend:
	// trace point id (start with s) followed by trace data, coding: sittee
	// i: state machine trace point code
	//  0000-3999  one timer value
	//  4000-5999 one hex value
	//  6000-7999 one dec value
	//  8000-8999 all timer values at interrupt
	//  9000 - rx tel data values
	// tt: trace point number within certain state
	// ee: state of state machine at trace point



	static unsigned int t,l, l1, lt,lt1, s, cv,tv, tmv;
	bool cf;
	l=0; l1=0;
	while (tb_in != tb_out && l1 < 5) {
	//while (tb_in != tb_out) {
		l1++;
		s= td_b[tb_out].ts;
		t= td_b[tb_out].tt;
		tv= td_b[tb_out].ttv;
		cv= td_b[tb_out].tcv;
		tmv= td_b[tb_out].ttmv;
		cf= td_b[tb_out].tc;
		if ((s>=8000 && s<=8999) ) {
			serial.println();
			serial.print("s");
			serial.print( (unsigned int) s, DEC, 3);
			serial.print(" t");
			serial.print( (unsigned int) t, DEC, 6);
			serial.print(" dt");
			serial.print( (unsigned int) t-lt, DEC,4);
			serial.print(" f");
			serial.print((unsigned int)cf, DEC, 1);
			serial.print(" c");
			serial.print((unsigned int)cv, DEC, 4);
			serial.print(" t");
			serial.print((unsigned int)tv, DEC, 4);
			serial.print(" m");
			serial.print((unsigned int)tmv, DEC,4);
/*			serial.print(" i");
			serial.print((unsigned int)tb_in, DEC,3);
			serial.print(" o");
			serial.print((unsigned int)tb_out, DEC,3);
*/
			//			serial.print("*");
			l=1;
			lt = t;
			lt1= t;
		}
		else if ( s>=9000) {
			serial.println();
			serial.print("s");
			serial.print( (unsigned int) s, DEC, 3);
			serial.print(" c/v");
			serial.print((unsigned int)cf, HEX, 2);
			serial.print(" L");
			serial.print((unsigned int)tmv, DEC, 2);
			serial.print(" t");
			serial.print( (unsigned int) t, HEX, 8);
			serial.print(" ");
			serial.print((unsigned int)cv, HEX, 4);
			serial.print(" ");
			serial.print((unsigned int)tv, HEX, 4);
			//serial.print("*");
		}else if ( s>=9005) {
			serial.println();
			serial.print("s");
			serial.print( (unsigned int) s, DEC, 3);
			serial.print(" ");
			serial.print((unsigned int)tmv, HEX,4);
			serial.print(" ");
			serial.print( (unsigned int) t, HEX, 8);
			serial.print(" ");
			serial.print((unsigned int)cv, HEX, 4);
			serial.print(" ");
			serial.print((unsigned int)tv, HEX, 4);
			serial.print(" d");
			serial.print((unsigned int)cf, HEX, 4);
			//serial.print("*");
		}
		else  if (s < 4000) { // one  delta timer
			serial.print("s");
			serial.print( (unsigned int) s -2000, DEC, 3);
			serial.print(" dt");
			serial.print( (unsigned int) t-lt1, DEC, 6);
			lt1 = t;
			l++;
		}
		else if (s < 5000) { // one hex
			serial.print("s");
			serial.print( (unsigned int) s- 4000, DEC, 3);
			serial.print(" h");
			serial.print( (unsigned int) t,HEX,4);
			l++;
		}
		else if (s < 6000) { // one dec
			serial.print("s");
			serial.print( (unsigned int) s- 5000, DEC, 3);
			serial.print(" d");
			serial.print( (unsigned int) t,DEC,4);
			l++;
		}
		if(l >5) {
			l=0;
			serial.println();
//			serial.print("* ");
		} else  serial.print(" ");
		if (++tb_out >= tb_lngth){ tb_out =0; tb_in_ov = false;}
		if(tb_in_ov && tb_out <= tb_in)  serial.print(" !!OV**");
	}
#endif


	if (bus.telegramReceived() && !bus.sendingTelegram() && (userRam.status & BCU_STATUS_TL))
		processTelegram();

	if (progPin)
	{
		// Detect the falling edge of pressing the prog button
		pinMode(progPin, INPUT|PULL_UP);
		int oldValue = progButtonDebouncer.value();
		if (!progButtonDebouncer.debounce(digitalRead(progPin), 50) && oldValue)
		{
			userRam.status ^= 0x81;  // toggle programming mode and checksum bit
		}
		pinMode(progPin, OUTPUT);
		digitalWrite(progPin, (userRam.status & BCU_STATUS_PROG) ^ progPinInv);
	}
}

void BcuBase::processTelegram()
{
}
