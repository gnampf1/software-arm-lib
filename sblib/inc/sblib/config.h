/**************************************************************************//**
 * @file     config.h
 * @brief    set pre-processor symbols which apply to all sblib build-configs
 * @date     2021/04/02
 *
 * @note
 *
 ******************************************************************************/

#ifndef SBLIB_CONFIG_H_
#define SBLIB_CONFIG_H_


#define SERIAL_DEBUG_PORT   //!> sets serial port to PIO2_7 and PIO_2_8 for dumping debug data
//#define DEBUG_BUS           //!> dump interrupt/SM data on byte and telegram level over serial interface
//#define DEBUG_BUS_BITLEVEL  //!> dump interrupt/SM data of each rx/tx bit over serial interface

//to avoid trace buffer overflow DUMP_TELEGRAMS should no be used in parallel to DEBUG_BUS/DEBUG_BUS_BITLEVEL
#define DUMP_TELEGRAMS      //!> dump rx and tx telegrams, incl received ack over serial interface
#define DEBUG_COM_OBJ       //!> dump object handling information on app-server level over serial interface


//#define DEBUG_OUT8_BI       //!>

#define DUMP_MEM_OPS    //!> dump APCI_MEMORY_WRITE_PDU and APCI_MEMORY_READ_PDU over serial interface
#define DEBUG_SERIAL    //!> dump KNX-serialnumber over serial interface
#define DUMP_PROPERTIES //!> dump interface object and property handling over serial interface


#endif /* SBLIB_CONFIG_H_ */
