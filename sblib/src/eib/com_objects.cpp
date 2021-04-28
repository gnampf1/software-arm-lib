/**
 *  com_objects.cpp - EIB Communication objects.
 *
 *  Copyright (C) 2014-2015 Stefan Taferner <stefan.taferner@gmx.at>
 *
 *  last update: Jan. 2021 Horst Rauch
 *  			 - added some explanation on the functions of this interface between
 *  			   the user application and the communication functions of the knx bus.
 *  			 - changed some function for the transmission of object from
 *  			 user appl. to knx bus and vice versa as there was an endless loop in some cases
 *
 *  This program is the main interface for the communication of the user application of the knx bus device,
 *  based on the functions of a BCUx or BIMx provided by the complete library.
 *  The basic interface as describe in knx spec 3.3.7, 3.4.1, 3.5.1 is hidden by some functions to write and read
 *  certain object types. We include some parts of these specs for a better understanding of the interface functions.
 *  The program implements functions of the Application Interface Layer (AIL) and the Application Layer of the KNX stack.
 *  Within these layers, a mapping of the GroupObject (of the user application) to the GroupAddress (of the knx bus/system)
 *  is provided via the Association table and the GroupAddress table. As there could be a n:m relation between the GroupObject
 *  and the GroupAddresses the Appl. layer will send respective msg to all matching associations (local in the device and on the bus).
 *
 *  The interface between the user application and the knx system is based on so called GroupObjects
 *  (GO, also communication objects).  The GroupObject Server act as an interface between the user application and
 *  the application layer of the knx stack. As the (communication) stack layers and the knx bus system has no information
 *  on the type and characteristics of a GO from the user application, data is provided by the
 *  user appl. and partly configured by the ETS:
 *
 *  The GO data consist of three parts:
 *		1. the Group Object description,  comprising Value Length/Type, Priority and Config Flags
 *		2. the Group Object value and
 *		3. the Group Object communication flags ( RAM Flags).
 *
 *	The Group Object description must at least include the Group Object Type and the transmission priority.
 *
 *	The Config Flags include static information about the Group Object:
 *		Bit	Field name
 *		7 	update enable, Shall be 1 in BCU1, for receive of GroupValue_Read.resp: 0=object not updated,
 *			1= object updated (update RAM Flag)
 *		6 	Transmit Enable, request from user application to trasnmit an object (write/response): 0=no transmit,
 *			1=msg generated if communication flag enabled
 *		5 	Segment Selector Type, storage segment of the object value: 0=RAM,1= Eeprom
 *		4 	Write Enable, for GroupValue_Write.ind msg: 0=no update of the obj, 1= obj.Value is updated
 *			if communication flag enabled
 *		3 	Read Enable, for GroupValue_Read.ind msg: 0=no response is generated, 1=response send
 *			if communication flag enabled
 *		2 	Communication Enable, enables/disable communication by this obj. e.g. messages are handled/not handled
 *		1-0 Transmission Priority, 00=system, 10=urgent, 01=normal or 11=low.
 *
 *
 *	The communication flags (RAM FLags) show the state of a Group Object and are encoded as follows:
 *		Bit 	Name 				Coding
 *		3 		update-flag 		0= not updated, 1= the value of the group object has been from the bus by
 *									an A_GroupValue_Write.ind or an A_GroupValue_Read.res.
 *		2 		data request flag 	0= idle/response, 1= data-request
 *		1-0		transmission status 00= idle/ok, 01= idle/error, 10= transmitting, 11= transmit request
 *
 *  The RAM Flags are used to link the asynchronous processes of the user application with the kxn stack (BCU lib).
 *
 *//*
 * Group Object value transfer functionality (see 3.4.1) of the Application Interface Layer:
 *
 *	The application process triggers Group Object value transfers by "setting" or "clearing" the relevant
 *	communication flags of a Group Object. The Group Objects, or their images, are be held in the Group
 *	Object Server (AIL/AL). The communication flags play an essential role in triggering the Application Interface
 *	Layer's Group Object service to initiate the transfers. The local access to a Group Object of the Group
 *	Object server stimulates the Group Object server to initiate a network wide update of that Group
 *	Object. Complementary, if an update has been received, the local application shall be triggered to use the
 *	new value.
 *
 *	There are four cases to be considered:
 *	-the application wants
 *		1. to read the Group Object's value
 *		2. to write the Group Object's value or
 *	-the Group Object service has received from the application layer (e.g. from knx bus or from the user application)
 *		3. a request to read the Group Object’s value
 *		4. an update on the Group Object's value.
 *
 * !note: If one device sends an A_GroupValue_Service each device that is member of this group shall receive the A_GroupValue_Service.
 *  With this requirement, all members of a Group will have to all times the same status (Object value) in the distributed system.
 *
 *  It is also necessary to sync local objects on one single device if they belong to the same group. E.g. a local object-write will
 *  trigger update of local GO if they share the same GA.
 *
 *  A local data request (mapped to read-request service on the bus which triggers a read-response from remote device) must check
 *  all local objects and in case they share the GA a local read-response as well as a bus read-response are  generated. Whereas overall
 *  multiple response are possible, one one single response should be generated on local side.
 *
 * These functions are provided and handled by this module.
 *
 *
 *
 * *********Interface to the user application***********************
 * The interface to the user application is implemented by none blocking functions to read or write the GO, the detailed
 * handling of the GO flags and configuration is hidden within the module from user application. Only ObjectRead(),
 * ObjectWrite() and ObjectUpdate() are mainly used by the user application.
 *
 * The transmission status of a read or write request can be checked by reading the GO RAM flags via the objectFlagsTable().
 * The AIL/AL layer will set the transmission status accordingly (bit 0,1 of the RAM Flag).
 * In order to avoid data loss (acc. to knx spec, a new transmission should not be initiated as
 * long as the transmit flags are not in idle state) the user application could check the transmission status
 * and trigger a new write/read if the GO transmission status is idle. The user application might check the error state
 * of a transmission and act accordingly.
 *
 * Any new request from the user application via the ObjectRead() or ObjectWrite() functions will
 * clear the previous state of transmission and data request flag.
 *
 *
 *
 * ********** Functions of the Application Interface Layer (AIL) and Application layer (AL)******
 *
 * 	---User Application triggered---
 *
 * 	1. user appl. wants to read a Group Object value  by a call to respective function (received by the AIL) requestObjectRead():
 * 	   	Check if config flag communication enabled is set, if set: set RAM flag data request and transmission request.
 *		If service request was not successful, return false.
 *
 *
 * 	2. user appl. wants to write a Group Object value (received by the AIL)by a call to respective function objectwrite():
 * 		Check if config flag transmission enable (not available in BCU1, always enabled) and communication enabled are set,
 * 		if set: set transmission request flag.
 *		If service request was not successful, return false.
 *

 * 	The respective requests will be handled by the asynchronous process of the AIL/AL by a call from the
 * 	bcu.loop function to sendnextGroupTelegram() which will check the transmission request state of the RAM flag.
 *
 * 	If the transmission request and data request flag are set, then the user wants to read the object:
 *
 *	 	1. Search the Association table for the GroupObject number (ASAP) and get the GroupAddr-index (TSAP)
 *		of the associated obj-number (ASAP). Get the GroupAddress from the GroupAddressTable with the GroupAddr-Index (TSAP)
 *		and generate a GroupValue_read.req msg for the AL and send the msg to the bus.
 *
 *		2. todo: Set the RAM flag transmission status to transmitting if positively send to local transport/network layer.
 *
 *		3. todo: Search the association table with the TSAP for further associations with TSAP. For each found, get the ASAP,
 *		check the ConfigFlags communication enable and read enable. If all enabled, stop search, get value of the found GO
 *		and update the initial GO with the new GO value and set the update RAM flag and send a read.response with
 *		the found object value and association (GroupAddress)to the bus.
 *
 *	If only the transmission request is set, then the user wants to write the object:
 *
 *	 	1. Search the Association table from top for the first GroupObject number (ASAP) and get the GroupAddr-index (TSAP)
 * 		of the associated obj-number (ASAP). Get the GroupAddress from the GroupAddressTable with the GroupAddr-Index (TSAP)
 * 		and generate a GroupValue_write.req msg for the AL and send the msg to the bus.
 *
 * 		2. todo:Set the RAM flag transmission status to transmitting if positively send to local transport/network layer.
 *
 * 		3. Search the association table with the TSAP for further associations with TSAP. For each found, get the ASAP,
 * 		check the ConfigFlags communication enable and write enable and set the update flag and store the new
 * 		GO value (value from the triggering GO) if enabled by the config flags
 *
 *
 * 	---BUS received Telegram triggered---
 * A call to processTelegram() from the bcu.loop initiates the handling of a new received telegram on the app-layer.
 * The received telegram is based on a valid GA (found in the GA Address Table) associated to a local object.
 *
 *		If the service request (APCI) is a write.request:
 *	 	1. Search the Association table for the the GroupAddr-index (TSAP)and get GroupObject number (ASAP).
 *	 	For each found, get the ASAP, check the ConfigFlags communication enable and write enable.
 *	 	If both enabled set the RAM update flag and store the new GO value (value from the triggering Telegram).
 *
 *		If the service request (APCI) is a read.response:
 *	 	1. Search the Association table for the the GroupAddr-index (TSAP)and get GroupObject number (ASAP).
 *	 	For each found, get the ASAP, check the ConfigFlags communication enable and update enable (Update enable
 *	 	flag is not available in BCU1, always enabled).
 *	 	If both enabled set the RAM update flag and store the new GO value (value from the triggering Telegram).
 *
 *		If the service request (APCI) is a read.request:
 *		1. Search the Association table for the the GroupAddr-index (TSAP)and get GroupObject number (ASAP).
 *		For each found, get the ASAP, check the ConfigFlags communication enable, transmit enable and read enable. If all enabled,
 *		stop search, get value of the found GO send a read.response with the found object value and association (GroupAddress)to the bus.
 *
 *
 *
 * todo: reflect the BUS result of sending/receiving of a telegram in the respective RAM-Flags for an object
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3 as
 *  published by the Free Software Foundation.
 */

#include <sblib/eib/com_objects.h>

#include <sblib/eib/addr_tables.h>
#include <sblib/eib/apci.h>
#include <sblib/eib/property_types.h>
#include <sblib/eib/user_memory.h>
#include <sblib/internal/functions.h>

#include <sblib/serial.h>
#ifdef DEBUG_COM_OBJ
#define d(x) x
#else
#define d(x)
#endif
//#define d(x)


// The COMFLAG_UPDATE flag, moved to the high nibble
#define COMFLAG_UPDATE_HIGH (COMFLAG_UPDATE << 4)

// The COMFLAG_TRANS_MASK mask, moved to the high nibble
#define COMFLAG_TRANS_MASK_HIGH (COMFLAG_TRANS_MASK << 4)


// The size of the object types BIT_7...VARDATA in bytes
const byte objectTypeSizes[10] = { 1, 1, 2, 3, 4, 6, 8, 10, 14, 15 };

int le_ptr = BIG_ENDIAN;

int objectSize(int objno)
{
    int type = objectType(objno);
    if (type < BIT_7) return 1;
    return objectTypeSizes[type - BIT_7];
}

/*
 * Get the size of the com-object in bytes, for sending/receiving telegrams.
 * 0 is returned if the object's size is <= 6 bit.
 */
int telegramObjectSize(int objno)
{
    int type = objectType(objno);
    if (type < BIT_7) return 0;
    return objectTypeSizes[type - BIT_7];
}

/*
 * Add one or more flags to the flags of a communication object.
 * This does not clear any flag of the communication object.
 *
 * @param objno - the ID of the communication object
 * @param flags - the flags to add
 *
 * @see objectWritten(int)
 * @see requestObjectRead(int)
 */
void addObjectFlags(int objno, int flags)
{
    byte* flagsTab = objectFlagsTable();
    if(flagsTab == 0)
    	return;


    if (objno & 1)
        flags <<= 4;

    d(serial.print(" addObjFlags in (obj, flags): ");)
  	d(serial.print(objno, DEC, 2);)
    d(serial.print(", ");)
  	d(serial.print(flags, HEX, 2);)
  	d(serial.print(", is: ");)
  	d(serial.print(flagsTab[objno >> 1], HEX, 2);)

    flagsTab[objno >> 1] |= flags;

    d(serial.print(", out: ");)
	d(serial.print(flagsTab[objno >> 1], HEX, 2);)
	d(serial.println();)

}

/*
 * Set the flags of a communication object.
 *
 * @param objno - the ID of the communication object
 * @param flags - the new communication object flags
 *
 * @see objectWritten(int)
 * @see requestObjectRead(int)
 */
void setObjectFlags(int objno, int flags)
{
    byte* flagsPtr = objectFlagsTable();
    flagsPtr += objno >> 1;


    d(serial.print(" setObjFlags in (obj, flags): ");)
	d(serial.print(objno, DEC, 2);)
    d(serial.print(", to: ");)
	d(serial.print(flags, HEX, 2);)
	d(serial.print(", is: ");)
	d(serial.print(*flagsPtr, HEX, 2);)
    d(serial.print(", out: ");)

    if (objno & 1)
    {
        *flagsPtr &= 0x0f;
        *flagsPtr |= flags << 4;
    }
    else
    {
        *flagsPtr &= 0xf0;
        *flagsPtr |= flags;
    }

	d(serial.print(*flagsPtr, HEX, 2);)
	d(serial.println();)

}

byte* objectValuePtr(int objno)
{
    // The object configuration
    const ComConfig& cfg = objectConfig(objno);

#if BCU_TYPE == BCU1_TYPE
    if (cfg.config & COMCONF_VALUE_TYPE) // 0 if user RAM, >0 if user EEPROM
        return userEepromData + cfg.dataPtr;
    return userRamData + cfg.dataPtr;
#else
    // TODO Should handle userRam.segment0addr and userRam.segment1addr here
    // if (cfg.config & COMCONF_VALUE_TYPE) // 0 if segment 0, !=0 if segment 1
    const byte * addr = (const byte *) &cfg.dataPtr;
    if (le_ptr == LITTLE_ENDIAN)
        return userMemoryPtr(makeWord(addr[1], addr[0]));
    else
        return userMemoryPtr(makeWord(addr[0], addr[1]));
#endif
}

unsigned int objectRead(int objno)
{
	int sz = objectSize(objno);
	byte* ptr = objectValuePtr(objno) + sz;
	unsigned int value = *--ptr;

	while (--sz > 0)
	{
		value <<= 8;
		value |= *--ptr;
	}
    return value;
}

void _objectWrite(int objno, unsigned int value, int flags)
{
    byte* ptr = objectValuePtr(objno);
    int sz = objectSize(objno);

    if(ptr == 0)
        return;

    for (; sz > 0; --sz)
    {
        *ptr++ = value;
        value >>= 8;
    }

    addObjectFlags(objno, flags);
}

void _objectWriteBytes(int objno, byte* value, int flags)
{
    byte* ptr = objectValuePtr(objno);
    int sz = objectSize(objno);

    for (; sz > 0; --sz)
        *ptr++ = *value++;

    addObjectFlags(objno, flags);
}

/*
 * @return The number of communication objects.
 */
inline int objectCount()
{
    // The first byte of the config table contains the number of com-objects
    return *objectConfigTable();
}

/*
 * Find the first group address for the communication object. This is the
 * address that is used when sending a read-value or a write-value telegram.
 *
 * @param objno - the ID of the communication object
 * @return The group address, or 0 if none found.
 */
int firstObjectAddr(int objno)
{
    byte* assocTab = assocTable();
    byte* assocTabEnd = assocTab + (*assocTab << 1);

    for (++assocTab; assocTab < assocTabEnd; assocTab += 2)
    {
        if (assocTab[1] == objno)
        {
            byte* addr = addrTable() + 1 + (assocTab[0] << 1);
            return (addr[0] << 8) | addr[1];
        }
    }

    return 0;
}

/**
 * Create and send a group read request telegram.
 *
 * In order to avoid overwriting a telegram in the send buffer while the bus is still sending the last telegram
 * we wait for a free buffer
 *
 * @param objno - the ID of the communication object
 * @param addr - the group address to read
 */
void sendGroupReadTelegram(int objno, int addr)
{
	while (bcu.sendTelegram[0]);  // wait for a free buffer

    bcu.sendTelegram[0] = 0xbc; // Control byte
    //todo, set routing cnt and prio according to the parameters set from ETS in the EPROM, add ID/objno for result association from bus-layer
    // todo check additional associations to Grp Addr for local read and possible response
    // 1+2 contain the sender address, which is set by bus.sendTelegram()
    bcu.sendTelegram[3] = addr >> 8;
    bcu.sendTelegram[4] = addr;
    bcu.sendTelegram[5] = 0xe1;
    bcu.sendTelegram[6] = 0;
    bcu.sendTelegram[7] = 0x00;

    bus.sendTelegram(bcu.sendTelegram, 8);
    // todo check for local response to initial read request
}

/**
 * Create and send a group write or group response telegram.
 *
 * In order to avoid overwriting a telegram in the send buffer while the bus is still sending the last telegram
 * we wait for a free buffer
 *
 * @param objno - the ID of the communication object
 * @param addr - the destination group address
 * @param isResponse - true if response telegram, false if write telegram
 */
void sendGroupWriteTelegram(int objno, int addr, bool isResponse)
{
    byte* valuePtr = objectValuePtr(objno);
    int sz = telegramObjectSize(objno);

	while (bcu.sendTelegram[0]);  // wait for a free buffer

    bcu.sendTelegram[0] = 0xbc; // Control byte
    //todo, set routing cnt and prio according to the parameters set from ETS in the EPROM, add ID/objno for result association from bus-layer
    // 1+2 contain the sender address, which is set by bus.sendTelegram()
    bcu.sendTelegram[3] = addr >> 8;
    bcu.sendTelegram[4] = addr;
    bcu.sendTelegram[5] = 0xe0 | ((sz + 1) & 15);
    bcu.sendTelegram[6] = 0;
    bcu.sendTelegram[7] = isResponse ? 0x40 : 0x80;

    if (sz) reverseCopy(bcu.sendTelegram + 8, valuePtr, sz);
    else bcu.sendTelegram[7] |= *valuePtr & 0x3f;

    // Process this telegram in the receive queue (if there is a local receiver of this group address)
    processGroupTelegram(addr, APCI_GROUP_VALUE_WRITE_PDU, bcu.sendTelegram, objno );

    bus.sendTelegram(bcu.sendTelegram, 8 + sz);
}

/*
 *  Send next Group read/write Telegram based on RAM flag status
 *
 *  Periodically called from BCU-loop function
 *  scan RAM flags of objects if there is a read or write request from the app.
 *  If object config flag allows communication and transmission requests from app send respective message
 *  and reset the RAM flags and return true.
 *  If no request is found in RAM flag return false
 *
 *  @return true if a telegram was send, 0 if none found/send.
 *
 */
int sndStartIdx = 0;

bool sendNextGroupTelegram()
{

    const ComConfig* configTab = &objectConfig(0);
    byte* flagsTab = objectFlagsTable();
    if(flagsTab == 0)
    	return false;

    int addr, flags, objno, config, numObjs = objectCount();

    // scan all objects, read config and Grp Addr of object
    for (objno = sndStartIdx; objno < numObjs; ++objno)
    {
        config = configTab[objno].config;
        addr = firstObjectAddr(objno);

        // we need to check if <transmit enable> and <communication enable>
        // is set in the config for the resp. object.
        if (addr == 0 || !(config & COMCONF_COMM)|| !(config & COMCONF_TRANS))
             continue;  // no communication allowed or no grp-adr associated, next obj.

        // check ram-flags for read or write request
    	flags = flagsTab[objno >> 1];
        if (objno & 1) flags >>= 4;

        if ((flags & COMFLAG_TRANSREQ) == COMFLAG_TRANSREQ)
        {//app is triggering a object read or write request on the bus

            if (flags & COMFLAG_DATAREQ)
            	// app triggeres a read request on the bus and check for additional associations to Grp Addr for local read
                sendGroupReadTelegram(objno, addr);
            else
            	// app triggeres a write request on the bus  and check for additional associations to Grp Addr for local writes
            	sendGroupWriteTelegram(objno, addr, false);

            // (if) msg is sent successfully on the bus, we need to reset the respective flags for the object
            //todo use the return-status of the BUS to reflect the result in the RAM-Flags
            // we should set here the status to TRANSMITING (0x02)
          	unsigned int mask = (COMFLAG_TRANS_MASK | COMFLAG_DATAREQ)  << (objno & 1 ? 4 :  0);
            flagsTab[objno >> 1] &= ~mask;

            sndStartIdx = objno + 1;
            return true;
        }
    }

    sndStartIdx = 0;
    return false;
}

/*
 * Get the ID of the next communication object that was updated
 * over the bus by a write-value-request telegram.
 *
 * @return The ID of the next updated com-object, -1 if none.
 */

int nextUpdatedObject()
{
    static int startIdx = 0;

    byte* flagsTab = objectFlagsTable();
    if(flagsTab == 0)
    	return -1;
	//d(serial.print(" next updated obj- ");)
	//d(serial.print("n");)
    //d(serial.print("start idx: ");)
 	//d(serial.println(startIdx, DEC, 3);)

    int flags, objno, numObjs = objectCount();

    for (objno = startIdx; objno < numObjs; ++objno)
    {
        flags = flagsTab[objno >> 1];

        if (objno & 1) flags &= COMFLAG_UPDATE_HIGH;
        else flags &= COMFLAG_UPDATE;

        //d(serial.print(" obj: ");)
        //d(serial.print(objno, DEC, 2);)
        //d(serial.print(", ");)

 		//d(serial.print(" fi: ");)
 		//d(serial.print(flags, HEX, 2);)
        //d(serial.print("; ");)
		//d(serial.println();)

        if (flags)
        {
            d(serial.print(" flags set obj: ");)
          	d(serial.print(objno, DEC, 2);)
            d(serial.print(", ");)
      		d(serial.print(" fi: ");)
      		d(serial.print(flags, HEX, 2);)
            d(serial.print("; ");)

        	flagsTab[objno >> 1] &= ~flags;
            startIdx = objno + 1;

            int nflags = flagsTab[objno >> 1];

            d(serial.print(" fo: ");)
     		d(serial.print(nflags, HEX, 2);)
			d(serial.println();)

        return objno;
        }
    }

    startIdx = 0;
    return -1;
}

void processGroupWriteTelegram(int objno, byte* tel)
{
    byte* valuePtr = objectValuePtr(objno);
    int count = telegramObjectSize(objno);

    if (count > 0) reverseCopy(valuePtr, tel + 8, count);
    else *valuePtr = tel[7] & 0x3f;

    addObjectFlags(objno, COMFLAG_UPDATE);
}



void processGroupTelegram(int addr, int apci, byte* tel, int trg_objno)
{
    const ComConfig* configTab = &objectConfig(0);
    const byte* assocTab = assocTable();
    const int endAssoc = 1 + (*assocTab) * 2;
    int objno, objConf;

    // Convert the group address into the index into the group address table
    const int gapos = indexOfAddr(addr);
    if (gapos < 0) return;

    // Loop over all entries in the association table, as one group address
    // could be assigned to multiple com-objects.
    for (int idx = 1; idx < endAssoc; idx += 2)
    {
        // Check if grp-address index in assoc table matches the dest grp address index
        if (gapos == assocTab[idx]) // We found an association for our addr
        {
            objno = assocTab[idx + 1];  // Get the com-object number from the assoc table

            if (objno == trg_objno)
            	continue; // no update of the object triggered by the app

            objConf = configTab[objno].config;

            if (apci == APCI_GROUP_VALUE_WRITE_PDU || apci == APCI_GROUP_VALUE_RESPONSE_PDU)
            {
                // Check if communication and write are enabled
                if ((objConf & COMCONF_WRITE_COMM) == COMCONF_WRITE_COMM)
                    processGroupWriteTelegram(objno, tel);
            }
            else if (apci == APCI_GROUP_VALUE_READ_PDU)
            {
                // Check if communication and read are enabled
                if ((objConf & COMCONF_READ_COMM) == COMCONF_READ_COMM)
                    sendGroupWriteTelegram(objno, addr, true);
            }
        }
    }
}

byte* objectConfigTable()
{
#if BCU_TYPE == BCU1_TYPE
    return userEepromData + userEeprom.commsTabPtr;
#else
    byte * addr = (byte* ) & userEeprom.commsTabAddr;
    return userMemoryPtr (makeWord (*(addr + 1), * addr));
#endif
}

byte* objectFlagsTable()
{
#if BCU_TYPE == BCU1_TYPE
    return userRamData + userEepromData[userEeprom.commsTabPtr + 1];
#else
    const byte* configTable = objectConfigTable();
    if(le_ptr == LITTLE_ENDIAN)
    	return userMemoryPtr(makeWord(configTable[2], configTable[1]));

    return userMemoryPtr(makeWord(configTable[1], configTable[2]));
#endif
}
