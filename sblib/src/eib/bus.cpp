/*
 *  bus.cpp - Low level EIB bus access.
 *
 *  Copyright (c) 2014 Stefan Taferner <stefan.taferner@gmx.at>
 *
 * last changes: Jan 2021 by Horst Rauch
 * 				- added some commenmts
 * 				- fixed collision detectction issue while sending ACK/NACK/BUSY
 * 				- introduction of collision detection window according to spec
 * 				  for pulse duration of 25us - 70us due to propagation delay
 * 				- added default phy-adr for normal devices and router (compilation with ROUTER defined)
 * 				- todo added some states to SM for better readability of the code, Busy handling,...
 * 				- todo added error handling, convey info to upper layers
 *
 * Sending of char is using the PWM function in a tricky way. As we need a positive pulse for 35us at first part of the bit
 * and 69us low pulse as second part of bit we need to have tricky usage: Sending of a bit with the PWM starts with a low
 * phase of the PWM which is equivalent to the high level on the bus. -> the bit sending is split in the two half bit:
 * high phase of bit n-1 followed by low phase of bit n.
 *
 * PWM start/low phase is 69us (using pwm match), following high phase 35us using timer-match to stop PWM
 * sending cycle. By that we have the bit n separated in the high phase (69us) of bit n-1 and the low phase
 * (35us) of bit n only. Therefore we need to have at last 69us pre-start time of a new bit before
 * we start the PWM and correct the waiting time after the last bit by 69us.
 * In addition the falling edge of the 35us pulse is triggering the capture event (and an interrupt if enabled)
 * in parallel.
 *
 * todo: change that to a more bit-compliant solution to allow simplification/readability of the code
 *
 *
 * *************** Main requirements based on knx spec****************
 * Physical Layer
 *
 *  Bit decoding timing:			min				typ.			max
 *   Bit time  										104us
 *   bit pulse duration 	 		25us			35us			70us	due to overlay of same pulse and propagation delay
 *
 *   acceptance window for leading
 *    edge relative to start bit	n*104us-7us		n*104us			n*104us+33us
 *
 *  time distance from start bit to
 *   start bit of next char			13*104us-30us	13*104us		13*104us+30us : 1322us, 1352us, 1382us
 *
 *
 *  Bit coding timing:				min				typ.			max
 *   Bit time  										104us
 *   bit pulse duration 	 		34us			35us			36us
 *
 *   time from start bit to following bits
 *   within single char				n*104us-2us		n*104us			n*104us+2us
 *
 *  time distance from start bit to	start bit
 *   of next char					13*104us-2us	13*104us		13*104us+5us  : 1350us, 1352us, 1357us
 *
 *	time for bus idle(no traffic on bus)			>50* 104us						: 5200us
 *
 *
 * Character/telegram (frame) timing
 *  (without poll frame timings):
 *   time between 2 normal frames:	53*104us -50us	>=53 * 104us		--			: >=5512us
 *
 *   time between telegram and
 *    repeated telegram:			50*104us -50us	50 * 104us			--			: 5200us
 *
 *   time between end of telegram
 *    and start of ACK/NACK/BUSY:
 *    		for bit coding:				15*104us-5us	15*104us		15*104us+20us	: 1560us
 *    		for bit decoding:			15*104us-5us	15*104us		15*104us+30us	: 1555us, 1560us, 1590us
 *
 *   timeout between end of frame
 *    and wait for ACK/NACK/BUSY:						>=30*104us+15*104us				: >=3120us+1560 =4680us
 *
 *   time between end of ACK/NACK/BUSY
 *    and begin of next telegram:					>=50 * 104us					: >=5200us
 *
 *   time between two characters within a telegram  >=2*104us <= 2,5*104us			: 208us <= t <= 260us
 *
 *
 *  Line BUSY detection:
 *   immediatly befor transmisson of the start bit of a first charcter of a frame the bus is check
 *   if any other device is already transmitting.
 *   This must be done for start of normal and repeated frame (and fill characters in poll situations - not used).
 *   In case the bus is busy no transmission is started and line_busy is indicated to upper layers.
 *   For the transmission of an ACK/NACK/BUSY and inner frame characters no line-busy detection
 *   will be done befor the start bit.
 *	 In case of line-busy, no trasmission should be started.
 *
 *  Collision detection:
 *   Shall be never disabled during transmission ( disable for sent of ACK is an option).
 *   In cased of a collision, the transmission should be stop immediately and indicated to the uppler layer functions.
 *   A collision is detected if a logical 1 (high level) was sent but at the same time a low level
 *   pulse was detected.
 *
 *
 * Data Link Layer transmit
 *  Typical frames of a device will be acknowledged by the remote receiver with ACK/NACK/BUSY. If the acknowledgment
 *  is positive, the ACK should be indicated to upper layers. In case of BUSY, the frame should be resent after waiting
 *  at least 150 bit times line idle. Resent should be done for max busy_retry times. If BUSY is received for more than
 *  busy_retry times, the upper layer should be informed with transmission status not_ok/error.
 *  In all other cases (NACK, corrupted frame or time out after 30 bit times..) the frame shall be repeated for nak_retry
 *  times after 50 bit times waiting. The status should be indicated to the upper layers.
 *
 *
 * Data Link Layer receive
 *  A received frame should be processed only if it is addresed to the own phy. Address or Group Adr. found in the adr. table
 *   or (broadcast) Group Adr. zero  (not appl. to BUS Monitor etc). If the frame is for us and:
 *   -If the received frame is not correct (parity/checksume,length...error) a NACK should be sent
 *    to remote device.
 *   -If frame is correct but the device is busy (e.g. input queue full,...) a BUSY should be sent to remote device
 *   -If frame is correct an ACK should be sent
 *
 *
 *
 * Interface to upper layers:
 *
 *  For interfacing to higher layers there are data buffer for sending of telegrams: bcu.SendTelegram[]
 *  and bus.telegram[] for receiving
 *  To check if there is activity on the BUS layer (data, Link) or a received telegram is waiting for processing
 *  bus.sendingTelegram bus.telegramReceived and bus.idle provide the respective info as bool
 *  The sending of a new telegram is triggered by the bus.sendTelegram method and is blocking if no
 *  buffer is available (current transmit ongoing).
 *
 *
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3 as
 *  published by the Free Software Foundation.
 */

#include <sblib/eib/bus.h>

#include <sblib/core.h>
#include <sblib/interrupt.h>
#include <sblib/platform.h>
#include <sblib/eib/addr_tables.h>
#include <sblib/eib/user_memory.h>
#include <sblib/eib/properties.h>

/*
 * The timer16_1 is used as follows:
 *
 * Capture register CR0 is used for receiving
 * Match register MR0 or MR1 is used as PWM for sending, depending on which output pin is used
 * Match register MR3 is used for timeouts while sending / receiving
 *
 */

// Enable debug statements for debugging the bus access in this file
#ifdef DEBUG_BUS
#define DB(x) x
#else
#define DB(x)
#endif

#define D(x)


/* L1/L2 msg header control field data bits meaning */
#define Allways0       	   0          //  bit 0 is allways 0
#define ACK_REQ_FLAG       1          // 0 ack is requested
#define PRIO0_FLAG         2          // 00 system priority, 10 alarm prio, 01 high prio, 11 low prio
#define PRIO1_FLAG         3	//
#define ACK_FRAME_FLAG     4          // 1 frame is data or poll, 0 ack frame
#define REPEAT_FLAG        5          // 1 not repeated, 0 repeated frame
#define DATA_FRAME_FLAG    6          // 1 poll data frame, 0 data frame
#define LONG_FRAME_FLAG    7          // 1,  short frame <=15 char, 0 long frame > 15 char
#define HEADER_LENGHT      7          // eib msg header lenght by length indicator =0

// telegram control byte bit definitions
#define SB_TEL_ACK_REQ_FLAG 	( 1<< ACK_REQ_FLAG)
#define SB_TEL_PRIO0_FLAG   	( 1<< PRIO0_FLAG )
#define SB_TEL_PRIO1_FLAG   	( 1<< PRIO1_FLAG)
#define SB_TEL_ACK_FRAME_FLAG   ( 1<< ACK_FRAME_FLAG )
#define SB_TEL_REPEAT_FLAG 		( 1<< REPEAT_FLAG)
#define SB_TEL_DATA_FRAME_FLAG 	( 1<< DATA_FRAME_FLAG)
#define SB_TEL_LONG_FRAME_FLAG 	( 1<< LONG_FRAME_FLAG)
#define SB_TEL_PRIO_FLAG   	    ( SB_TEL_PRIO0_FLAG | SB_TEL_PRIO1_FLAG )

#define PRIO_FLAG_HIGH	 (SB_TEL_PRIO0_FLAG)
#define PRIO_FLAG_LOW	 (SB_TEL_PRIO0_FLAG |SB_TEL_PRIO1_FLAG )


static int debugLine = 0;

// Mask for the timer flag of the capture channel
#define CAPTURE_FLAG (8 << captureChannel)

// Mask for the timer flag of the time channel
#define TIME_FLAG (8 << timeChannel)

// Default time between two bits (104 usec)
#define BIT_TIME 104

// Time between two bits (69 usec)
#define BIT_WAIT_TIME 69

// Pulse duration of a bit (35 usec)
#define BIT_PULSE_TIME 35

// Maximum time from start bit to stop bit, including a safety extra: BIT_TIME*10 + BIT_TIME/2
#define BYTE_TIME 1090

// Time to wait before sending an ACK: approximately BIT_TIME * 11 + (BIT_TIME / 4)
#define SEND_ACK_WAIT_TIME 1177

// Time to wait before starting to send: BIT_TIME * 50
#define SEND_WAIT_TIME 5200

// Time to listen for bus activity before sending starts: BIT_TIME * 1
#define PRE_SEND_TIME 104

// The value for the prescaler
#define TIMER_PRESCALER (SystemCoreClock / 1000000 - 1)

#ifdef DUMP_TELEGRAMS
unsigned char telBuffer[32];
unsigned int telLength = 0;
unsigned int telRXtime = 0;
bool telcollision;
#endif

// constructor for Bus object. Initialize basic interface parameter to bus and set SM to IDLE
Bus::Bus(Timer& aTimer, int aRxPin, int aTxPin, TimerCapture aCaptureChannel, TimerMatch aPwmChannel)
:timer(aTimer)
,rxPin(aRxPin)
,txPin(aTxPin)
,captureChannel(aCaptureChannel)
,pwmChannel(aPwmChannel)
{
    timeChannel = (TimerMatch) ((pwmChannel + 2) & 3);  // +2 to be compatible to old code during refactoring
    state = Bus::IDLE;
}

/**
 * Start BUS operation
 *
 * get our own phy addr, reset operating parameters, start the timer
 * set the pwm parameter so that we have no pulse on bus and no interrupt from timer
 * activate capture interrupt, set bus pins to respective mode
 *
 * In case we are a normal device (ROUTER not defined for compilation) we check for 0.0.0 as phy addr
 * and change to default 15.15.252 if  zero is our addr
 *
 */
void Bus::begin()
{
//todo get defined values from usereprom for busy-retry and nack-retry

	ownAddr = (userEeprom.addrTab[0] << 8) | userEeprom.addrTab[1];
#if BCU_TYPE != BCU1_TYPE
    if (userEeprom.loadState[OT_ADDR_TABLE] == LS_LOADING)
    {
        byte * addrTab = addrTable() + 1;
        ownAddr = (*(addrTab) << 8) | *(addrTab + 1);
    }
#endif
    //check own addr - are we a router then 0 is allowed
    //0.0.0 is not allowed for normal devices
    //set default addr  15.15.252 in case we have PhyAdr of 0.0.0
#ifndef ROUTER
    if (ownAddr == 0) ownAddr =0xfffc;
#endif
    telegramLen = 0;

    state = Bus::IDLE;
    sendAck = 0;
    sendCurTelegram = 0;
    sendNextTel = 0;
    sendTriesMax = 4;
    collision = false;

    timer.begin();
    timer.pwmEnable(pwmChannel);
    timer.captureMode(captureChannel, FALLING_EDGE | INTERRUPT);
    timer.start();
    timer.interrupts();
    timer.prescaler(TIMER_PRESCALER);

    timer.match(timeChannel, 0xfffe);
    timer.matchMode(timeChannel, RESET);
    timer.match(pwmChannel, 0xffff);

   	DB(serial.print("Bus begin - Timer prescaler: ");)
    DB(serial.print((unsigned int)TIMER_PRESCALER, DEC, 6);)
    DB(serial.println("");)

    // wait until output is driven low before enabling output pin.
    // Using digitalWrite(txPin, 0) does not work with MAT channels.
    timer.value(0xffff); // trigger the next event immediately
    while (timer.getMatchChannelLevel(pwmChannel) == true);
    pinMode(txPin, OUTPUT_MATCH);   // Configure bus output
    pinMode(rxPin, INPUT_CAPTURE | HYSTERESIS);  // Configure bus input

    //
    // Init GPIOs for debugging
    //
    D(pinMode(PIO3_0, OUTPUT));
    D(pinMode(PIO3_1, OUTPUT));
    D(pinMode(PIO1_5, OUTPUT));
    D(pinMode(PIO1_4, OUTPUT));
    D(pinMode(PIO0_6, OUTPUT));
    D(pinMode(PIO0_7, OUTPUT));
    //D(pinMode(PIO2_8, OUTPUT));
    //D(pinMode(PIO2_9, OUTPUT));
    //D(pinMode(PIO2_10, OUTPUT));

    D(digitalWrite(PIO3_0, 0));
    D(digitalWrite(PIO3_1, 0));
    D(digitalWrite(PIO1_5, 0));
    D(digitalWrite(PIO1_4, 0));
    D(digitalWrite(PIO0_6, 0));
    D(digitalWrite(PIO0_7, 0));
    //D(digitalWrite(PIO2_8, 0));
    //D(digitalWrite(PIO2_9, 0));
    //D(digitalWrite(PIO2_10, 0));
}




/**
 * Prepare the telegram for sending. Set the sender address to our own
 * address, and calculate the checksum of the telegram.
 * Stores the checksum at telegram[length].
 *
 * @param telegram - the telegram to process
 * @param length - the length of the telegram
 */
void Bus::prepareTelegram(unsigned char* telegram, unsigned short length) const
{
    unsigned char checksum = 0xff;
    unsigned short i;

    // Set the sender address
    telegram[1] = ownAddr >> 8;
    telegram[2] = ownAddr;

    // Calculate the checksum
    for (i = 0; i < length; ++i)
        checksum ^= telegram[i];
    telegram[length] = checksum;
}

/**
 *       Interface to upper layer for sending a telgram
 *
 * Is called from within the BCU-loop method. Is blocking if there is no space
 * in the Telegrambuffer ( as we have only one buffer at BCU level, the check for buffer free is
 * in the BCU-loop on bus.sendingTelegram())
 *
 * Send a telegram. The checksum byte will be added at the end of telegram[].
 * Ensure that there is at least one byte space at the end of telegram[].
 *
 * @param telegram - the telegram to be sent.
 * @param length - the length of the telegram in sbSendTelegram[], without the checksum
 */
void Bus::sendTelegram(unsigned char* telegram, unsigned short length)
{
    prepareTelegram(telegram, length);

    // Wait until there is space in the sending queue
    while (sendNextTel)
    {
    }

    if (!sendCurTelegram) sendCurTelegram = telegram;
    else if (!sendNextTel) sendNextTel = telegram;
    else fatalError();   // soft fault: send buffer overflow

    // Start sending if the bus is idle
    noInterrupts();
    if (state == IDLE)
    {
        sendTries = 0;
        state = Bus::SEND_INIT;
        D(debugLine = __LINE__);

        timer.match(timeChannel, 1);
        timer.matchMode(timeChannel, INTERRUPT | RESET);
        timer.value(0);
    }
    interrupts();

#ifdef DUMP_TELEGRAMS_BUS
	{ unsigned int t;
		t= millis();
    	serial.print("QSD: (");
        serial.print(t, DEC, 6);
        serial.print(") ");

        for (int i = 0; i <= length; ++i)
        {
            if (i) serial.print(" ");
            serial.print(telegram[i], HEX, 2);
        }
        serial.println();
	}
#endif

}

/**
 *  set the Bus SM to idle state
 *  configure the capture to falling edge and interrupt
 *  match register for low pwm output, no ACK to send
 *
 *  todo: idle state should be entered only after 53bit times of inactivity on the bus
 *     ??????????need to find out where this is done in current SW
 */
void Bus::idleState()
{
    timer.captureMode(captureChannel, FALLING_EDGE | INTERRUPT);

    timer.matchMode(timeChannel, RESET);
    timer.match(timeChannel, 0xfffe); // stop pwm pulse generation, set output to low
    timer.match(pwmChannel, 0xffff);

    state = Bus::IDLE;
    sendAck = 0;

//    digitalWrite(txPin, 0); // Set bus-out pin to 0
//    pinMode(txPin, INPUT);
}


/**
 * ***********part of the interrupt processing -> keep as short as posssible**********
 *
 * We arrive here after a collision during sending or we received a complete Telegram from bus.
 *
 * If we received some bytes, process data and provide to upper layers and initialize timer
 * for inter telegram wait (50bit times) and set send-init state
 * If we had a colission, we only initialize timer for inter telegram wait (50bit times) and set send-init state
 *
 * End of telegram is indicated by a timeout of least max 2bit times +30us after last received char
 * (or 13*bit time -30us till 13*bit time+30us after start bit of received frame)
 *
 *  todo: inform upper layer on reception/sending status of telegram
 *
 *
 * Provide some Data-layer functions:
 *  check telegram type
 *  check if it is for us (phy adr, or Grp Adr belong to us)
 *  check if we want it anyhow (TL, LL info)
 *  send ACK/NACK if requested
 *  todo check for BUSY frome remote side in ACK-Telegram
 *
 * indicate telegram reception to the looping function by setting <processTel> to true
 * received telegram in telegram buffer <telegram[]>, length in telegramLen
 * todo: info on frame status to upper layer in telStatus
 *
 * @param bool of all received char parity and frame checksum error
 *
 */
void Bus::handleTelegram(bool valid)
{
//    D(digitalWrite(PIO1_4, 1));         // purple: end of telegram
    sendAck = 0;
#ifdef DUMP_TELEGRAMS_BUS

    telRXtime= millis();
    if (nextByteIndex){
    	for (int i = 0; i <= nextByteIndex; ++i)
    	{
    		telBuffer[i] = telegram[i];
    	}
    	telLength = nextByteIndex;
    	telcollision = collision;
    }

#endif

    if (collision) // A collision occurred during sening. Ignore the received bytes
    	// we use the same code for initializing the send-init process
    {
    	//todo - do we need to reset the sendACK flag?? if collision occured during ACK sending
    	// in order to wait the right time befor start of next send interval
    }

    else if (nextByteIndex >= 8 && valid) // Received a valid telegram with correct checksum
    {// we need to check the start bit condition- parity and checksum and give upper layer error info
        int destAddr = (telegram[3] << 8) | telegram[4];
        bool processTel = false;

        // We ACK the telegram only if it's for us
        if (telegram[5] & 0x80)
        {
            if (destAddr == 0 || indexOfAddr(destAddr) >= 0)
                processTel = true;
        }
        else if (destAddr == ownAddr)
        {
            processTel = true;
        }

        // Only process the telegram if it is for us or if we want to get all telegrams
        if (!(userRam.status & BCU_STATUS_TL))
        {
            telegramLen = nextByteIndex;

            if (userRam.status & BCU_STATUS_LL)
                sendAck = SB_BUS_ACK;
        }
        else if (processTel)
        {
            telegramLen = nextByteIndex;
            //todo check if an ACK is requested by the sender of the telegram -> bit1 in the control field
            sendAck = SB_BUS_ACK;
        }
    }
    else if (nextByteIndex == 1)   // Received a spike or a bus acknowledgment
    {
        currentByte &= 0xff;
        //  did we send a telegram ( sendTries>=1 and the received telegram is ACK or repetition max  -> send next telegram
        // todo we need to check if the remote side send a BUSY as answer back
        if ((currentByte == SB_BUS_ACK || sendTries > sendTriesMax) && sendCurTelegram && sendTries > 0)
        {
             sendNextTelegram();
        }
    }
    else // Received wrong checksum or parity, or more than one byte but too short for a telegram
    {
        telegramLen = 0;
        sendAck = SB_BUS_NACK;
    }

    // Wait before sending. In SEND_INIT  (wait for bus-idle) we will cancel if there is nothing to be sent.
    // We need to wait anyways to avoid triggering sending from the application code when
    // the bus is in cooldown (time between to telegrams: 50/53 bit times). This could happen if we set state to Bus::IDLE here.

    // if we need to send ack - wait 15bit times before sending ACK or 53 bit times
    // after end of previous frame before sending next telegram
    //*   time between end of telegram and start of ACK/NACK/BUSY:
    //*    		for bit coding:				15*104us-5us	15*104us		15*104us+20us	: 1560us
    // as we waited already for the frame end timeout 2*104 us +30us  ????

    timer.match(timeChannel, sendAck ? SEND_ACK_WAIT_TIME - PRE_SEND_TIME : SEND_WAIT_TIME - PRE_SEND_TIME);
    timer.matchMode(timeChannel, INTERRUPT | RESET);

    timer.captureMode(captureChannel, FALLING_EDGE | INTERRUPT);

    collision = false;
    state = Bus::SEND_INIT;
    debugLine = __LINE__;
}

void Bus::sendNextTelegram()
{
    sendCurTelegram[0] = 0;
    sendCurTelegram = sendNextTel;
    sendNextTel = 0;
    sendTries = 0;
    sendTelegramLen = 0;
    //todo save the transmit state for upper layer
}


/*
 * State Machine - driven by interrupts of timer and capture input
 *
 */
void Bus::timerInterruptHandler()
{
    D(static unsigned short tick = 0);
    bool timeout;
    int time;

    // Debug output
    D(digitalWrite(PIO0_6, ++tick & 1));  // brown: interrupt tick
    D(digitalWrite(PIO3_0, state==Bus::SEND_BIT_0)); // red
    D(digitalWrite(PIO3_1, 0));           // orange
    D(digitalWrite(PIO1_5, 0));           // yellow
    D(digitalWrite(PIO1_4, 0));           // purple
    //D(digitalWrite(PIO2_8, 0));           // blue
//    D(digitalWrite(PIO2_9, 0));           //

STATE_SWITCH:
    switch (state)
    {
    // The bus is idle. Usually we come here when there is a capture event on bus-in.
    case Bus::IDLE:
        if (!timer.flag(captureChannel)) // Not a bus-in signal: do nothing
            break;
        nextByteIndex = 0;
        collision = false;
        checksum = 0xff;
        sendAck = 0;
        valid = 1;

     // no break here as we have received a capture event - falling edge of the start bit
     // we continue with the receiving of start bit state

    // A start bit is expected to arrive here. If we have a timeout instead, the
    // transmission of a frame is over.  (after 13*104us +30us after start of last char)
    case Bus::RECV_START:
        //D(digitalWrite(PIO3_1, 1));   // orange
        if (!timer.flag(captureChannel))  // No start bit: then it is a timeout of end of frame
        {
        	//handle telegram and wait inter frame time of >50*104us or if we need to send ack back time for ACK sending
        	//initialize SM for new reception :state SEND_INT
            handleTelegram(valid && !checksum);
            break;
        }
        // we received a start bit interrupt - reset timer for next byte reception, byte time is without stop bit (10*bit +1/2bit ~1090us)???
        timer.match(timeChannel, BYTE_TIME);
        timer.restart();
        timer.matchMode(timeChannel, INTERRUPT | RESET);

        state = Bus::RECV_BYTE;
        currentByte = 0;
        bitTime = 0;
        bitMask = 1;
        parity = 1;
        break;

        // we received next capture event- check if we have end of byte receive or a low bit at position n*104us
    case Bus::RECV_BYTE:
        timeout = timer.flag(timeChannel); // end of rx byte

        if (timeout) time = BYTE_TIME;
        else time = timer.capture(captureChannel); // we received an capt. event: new low bit

        // find the bit position after last low bit and add high bits accordingly
        //window for the reception of falling edge of a bit is
        //	min: n*104us-7us, typ: n*104us, max: n*104us+33us
        if (time >= bitTime + BIT_WAIT_TIME) // todo check window
        {
            bitTime += BIT_TIME;
            while (time >= bitTime + BIT_WAIT_TIME && bitMask <= 0x100) // bit found or bit 9(parity bit) found
            {
                currentByte |= bitMask; // add high bit until we found current position
                parity = !parity;

                bitTime += BIT_TIME;
                bitMask <<= 1; // next bit is in bitmask
            }

            bitMask <<= 1; //next bit or stop bit
        }

        if (timeout)  // Timer timeout: end of byte
        {
            D(digitalWrite(PIO1_5, 1));     // yellow: end of byte
            D(digitalWrite(PIO3_1, parity));// orange: parity bit ok

            valid &= parity;
            if (nextByteIndex < SB_TELEGRAM_SIZE)
            {
                telegram[nextByteIndex++] = currentByte;
                checksum ^= currentByte;
            }

            // wait for the next byte's start bit and set timer accordingly
            state = Bus::RECV_START;

            // timeout was at 10.5 bit times (1090us) next start bit:
            //time distance from start bit to start bit of next char min: 13*104us-30us, typ:13*104us, max:	13*104us+30us : 1322us, 1352us, 1382us
            timer.match(timeChannel, BIT_TIME * 4); // max 1382us -1090us= 292us !!!todo  that to long
        }
        break;


/* ************Sending states************** */

    // SEND_INIT is entered some usec before sending the start bit of the first byte. It
    // is always entered after receiving or sending is done and we waited the respective time for next action:
    // send ACK?NACK?BUSY or next frame,  even if nothing is to be sent.
    // ???we waited alrady about 49/50  since last activity bit time before we arrive here???

    case Bus::SEND_INIT:
        D(digitalWrite(PIO1_5, 1)); // yellow: prepare transmission

        // capture event: start bit detected
        if (timer.flag(captureChannel))  // Bus input, enter receive mode
        {
            state = Bus::IDLE;
            goto STATE_SWITCH;
        }

        // waiting timeout  before next action
        if (sendAck)  // Send an acknowledgement?
        {
            time = PRE_SEND_TIME; // for sending an ACK, we wait 104us before start
            sendTelegramLen = 0;
        	}
        else
        {
        	if (sendTries > sendTriesMax) // check if we have max resend for last telegram
                sendNextTelegram();	// then send next  todo: info upper layer on sending error of last telegram

            if (sendCurTelegram)  // Send a telegram?
            {

                if (sendTries == 1) // check for need of repetition
                {
                    // If it is the first repeat, then mark the telegram as being repeated and correct the checksum
                    sendCurTelegram[0] &= ~SB_TEL_REPEAT_FLAG;
                    sendCurTelegram[sendTelegramLen - 1] ^= SB_TEL_REPEAT_FLAG;

                    // We increase sendTries here to avoid inverting the repeat flag again
                    // if sending fails due to collision.
                    ++sendTries;
                }

                // if  we have repetition of telegram or system or alarm prio, we wait only 50bit time
               	// check priority and extend waiting time, for low/normal prio add 3*104us ->53bit times between telegrams
                if ((sendCurTelegram[0] & SB_TEL_REPEAT_FLAG) || ((sendCurTelegram[0] & SB_TEL_PRIO_FLAG) == PRIO_FLAG_LOW)
                		|| ((sendCurTelegram[0] & SB_TEL_PRIO_FLAG) == PRIO_FLAG_HIGH))
                {
                	time = PRE_SEND_TIME + 3 * BIT_TIME;
                }
                else time = PRE_SEND_TIME;

                	sendTelegramLen = telegramSize(sendCurTelegram) + 1;

            }
            else  // Send nothing
            {
                idleState();
                break;
            }
        }

        timer.match(pwmChannel, time); // waiting time till start of first bit- falling edge 104us + prio*104us
        timer.match(timeChannel, time + BIT_PULSE_TIME); // end of bit pulse 35us later
        timer.matchMode(timeChannel, RESET | INTERRUPT); //reset timer after bit pulse end
        timer.captureMode(captureChannel, FALLING_EDGE | INTERRUPT); // next state interrupt at start bit  - falling edge

        nextByteIndex = 0;
        state = Bus::SEND_START_BIT;
        break;


    // The start bit of the first byte is being sent. We should come here when the edge
    // of the start bit is captured by bus-in of the pwmChannel. We might come here when somebody else started
    // sending before us, or if a timeout occurred. In case of a timeout, we have a hardware
    // problem as receiving our sent signal does not work.
    case Bus::SEND_START_BIT:
        if (timer.flag(captureChannel))
        {
            //  Bus busy check: Abort sending if we receive a start bit early enough to abort.
            // We will receive our own start bit here too.
            if (timer.value() < timer.match(pwmChannel) - 10) // received edge of bit before our own bit was triggered
            {
                timer.match(pwmChannel, 0xffff); // stop our bit  ste pwm output to low
                state = Bus::RECV_START; // start reception of byte
                goto STATE_SWITCH;
            }

            state = Bus::SEND_BIT_0; // our sending trigger, set next state
            break;
        }
        else if (timer.flag(timeChannel))
        {
            // Timeout: we have a hardware problem as receiving our sent signal does not work.
            // for now we will just continue
            //D(digitalWrite(PIO2_8, 1));  // blue: sending bits does not work
        }
        // No break here, todo inform on hw error


        // start bit low pulse end now after 35us by time match interrupt, start sending byte
        // we are in the middle of the start bit at rising edge of  start bit pulse
        // prepare next byte for sending
    case Bus::SEND_BIT_0:
        if (sendAck)
            currentByte = sendAck;
        else
        	currentByte = sendCurTelegram[nextByteIndex++];

        // Calculate the parity bit
        for (bitMask = 1; bitMask < 0x100; bitMask <<= 1)
        {
            if (currentByte & bitMask)  // bit current bit high
                currentByte ^= 0x100;  // toggle/xor parity bit
        }

        bitMask = 1;
        // no break here, continue sending first bit/ LSB

        // bit low pulse end now after 35us by time match interrupt, sending next bit of byte
        // we are in the middle of the n-bit at rising edge of n-bit pulse

    case Bus::SEND_BIT:
        D(digitalWrite(PIO1_5, 1));    // yellow: send next bits

        // Search for the next zero bit and count the one bits for the wait time
        // only till we reach the parity bit- next bit after parity will be low in telegram-byte
        time = BIT_TIME;
        while ((currentByte & bitMask) && bitMask <= 0x100)
        {
            bitMask <<= 1;
            time += BIT_TIME;
        }
        bitMask <<= 1; // next low bit or stop bit if mask > 0x200

        if (time <= BIT_TIME) // low bit to send?
            state = Bus::SEND_BIT;
        else state = Bus::SEND_BIT_WAIT; // high bit to send, detect collisions while sending high bits

        if (bitMask > 0x200) // stop bit reached
        {
            time += BIT_TIME * 3; // Stop bit + inter-byte timeout, as we are at the raising edge position this is 3*104us - 69us = 243us????

            if (nextByteIndex < sendTelegramLen && !sendAck) // if tel. end or ack, stop sending
            {
                state = Bus::SEND_BIT_0;  // state for next byte to send
            }
            else
            {
                state = Bus::SEND_END;
            }
        }

        // set next match/interrupt
        // if we are sending high bits, we wait for next low bit edge by cap interrupt which will be a collision
        // or timeout interrupt indicating end of bit pulse sending (high or low)
        if (state == Bus::SEND_BIT_WAIT)
            timer.captureMode(captureChannel, FALLING_EDGE | INTERRUPT);
        else timer.captureMode(captureChannel, FALLING_EDGE);

        if (state == Bus::SEND_END)
            timer.match(pwmChannel, 0xffff); //stop pwm pulses - low output
        // as we are at the raisng edge of the last pulse, the next falling edge will be n*104 - 35us (min69us) away
        else timer.match(pwmChannel, time - BIT_PULSE_TIME); // start of pulse for next low bit - falling edge on bus will not trigger cap interrupt

        timer.match(timeChannel, time); // interrupt at end of low/high bit pulse - next raising edge
        break;

    // Wait for a capture event from bus-in. This should be from us sending a zero bit, but it
    // might as well be from somebody else in case of a collision.
    // Check for collision during sending of high bits
   // start of falling edge of next low bit by cap intr.
    case Bus::SEND_BIT_WAIT:
    	// check if intr is from our pulse
    	// collision window starts 69us after rising edge of last low bit and ends 69us before last high bit end
    	if ((timer.capture(captureChannel) < timer.match(pwmChannel) - BIT_WAIT_TIME) &&
    			(timer.capture(captureChannel) > BIT_WAIT_TIME) )
        {
            // A collision. Stop sending and ignore the current transmission.
            D(digitalWrite(PIO1_4, 1));  // purple
            timer.match(pwmChannel, 0xffff); // set PWM bit to low next interrupt is on timeChannel match (value :time)
            state = Bus::RECV_BYTE;
            collision = true;
            break;
        }
    	// start of our pulse, next state is raising edge of pulse by timer match
        state = Bus::SEND_BIT;
        break;

        // we arrive here after last byte was send and 243us after the stop bit
    case Bus::SEND_END:
        //D(digitalWrite(PIO2_9, 1));
        timer.match(timeChannel, SEND_WAIT_TIME);
        timer.captureMode(captureChannel, FALLING_EDGE | INTERRUPT);

        if (sendAck) sendAck = 0;
        else ++sendTries;

        state = Bus::SEND_WAIT;
        break;

    // Wait for ACK or resend / send next telegram
    case Bus::SEND_WAIT:
        if (timer.flag(captureChannel) && timer.capture(captureChannel) < SEND_ACK_WAIT_TIME)
        {
            // Ignore bits that arrive too early
            break;
        }
        state = Bus::SEND_INIT;  // Receiving will be handled there too
        goto STATE_SWITCH;

    default:
        idleState();
        break;
    }

    timer.resetFlags();
}

