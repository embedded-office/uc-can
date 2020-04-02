/*
*********************************************************************************************************
*                                              uC/CAN
*                                      The Embedded CAN suite
*
*                 Copyright by Embedded Office GmbH & Co. KG www.embedded-office.com
*                    Copyright 1992-2020 Silicon Laboratories Inc. www.silabs.com
*
*                                 SPDX-License-Identifier: APACHE-2.0
*
*               This software is subject to an open source license and is distributed by
*                Silicon Laboratories Inc. pursuant to the terms of the Apache License,
*                    Version 2.0 available at www.apache.org/licenses/LICENSE-2.0.
*
*********************************************************************************************************
*/


/*
****************************************************************************************************
*                                         CAN DRIVER CODE
*
*                                           Kinetis_Kxx
*
* Filename : drv_can.h
* Version  : V2.42.01
****************************************************************************************************
* Note(s)  : (1) Supports Freescale's Kinetis K Series MCUs as described in various datasheets.
*                This driver has been tested with or should work with the following K Series MCUs :
*
*                    Kinetis K40
*                    Kinetis K60
*                    Kinetis K64
*                    Kinetis K70
****************************************************************************************************
*/

#ifndef _DRV_CAN_H_
#define _DRV_CAN_H_


/*
****************************************************************************************************
*                                             INCLUDES
****************************************************************************************************
*/

#include "cpu.h"


/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/

/*
****************************************************************************************************
*                                           DRIVER NAME
*
* Note(s) : Unique driver name for installation and searching.
****************************************************************************************************
*/

#define  KXX_CAN_NAME                "KXX:CAN Module"


/*
****************************************************************************************************
*                                         STANDARD ID MASK
*
* Note(s) : Max. possible identifier if using standard CAN frames.
****************************************************************************************************
*/

#define  KXX_CAN_STD_ID_MASK              0x07FFu


/*
****************************************************************************************************
*                                         EXTENDED ID MASK
*
* Note(s) : Max. possible identifier if using extended CAN frames.
****************************************************************************************************
*/

#define  KXX_CAN_EXT_ID_MASK          0x1FFFFFFFu


/*
****************************************************************************************************
*                                         EXTENDED ID FLAG
*
* Note(s) : (1) This flag must be set in the CAN identifier to mark an extended ID.
*
*           (2) See KXX_CAN_FRM
****************************************************************************************************
*/

#define  KXX_CAN_EXT_ID_FLAG          0x20000000u


/*
****************************************************************************************************
*                                       STATUS: CAN BUS IDLE
*
* Note(s) : This value defines the status value, which is set when the CAN bus is idle.
****************************************************************************************************
*/

#define  KXX_CAN_IDLE                       0x00u


/*
****************************************************************************************************
*                                       STATUS: CAN BUS OPEN
*
* Note(s) : This value defines the status value, which is set when the CAN bus is opened.
****************************************************************************************************
*/

#define  KXX_CAN_OPEN                       0x01u


/*
****************************************************************************************************
*                                     FIRST AND LAST RX BUFFER
*
* Note(s) : This value defines the message buffers used to receive data.
****************************************************************************************************
*/

#define  KXX_CAN_RX_FIRST_MSG_BUF              0u
#define  KXX_CAN_RX_LAST_MSG_BUF               5u


/*
****************************************************************************************************
*                                     FIRST AND LAST TX BUFFER
*
* Note(s) : This value defines the message buffers used to transmit data.
****************************************************************************************************
*/

#define  KXX_CAN_TX_MSG_BUF0                  13u
#define  KXX_CAN_TX_MSG_BUF1                  14u
#define  KXX_CAN_TX_MSG_BUF2                  15u

#define  KXX_CAN_TX_MSG_BUF_MASK             ((1u << KXX_CAN_TX_MSG_BUF0) | \
                                              (1u << KXX_CAN_TX_MSG_BUF1) | \
                                              (1u << KXX_CAN_TX_MSG_BUF2))


/*
****************************************************************************************************
*                                            DATA TYPES
****************************************************************************************************
*/


/*
****************************************************************************************************
*                                           DEVICE NAMES
*
* Note(s) : This enum defines the available device names for the driver XXXCANInit() function.
****************************************************************************************************
*/

enum {
    KXX_CAN0_BUS   = 0u,                                        /* FlexCAN                                              */
    KXX_CAN1_BUS,                                               /* FlexCAN                                              */
    KXX_CAN_DEV_N                                               /* Number of devices                                    */
};


/*
****************************************************************************************************
*                                           ERROR CODES
*
* Note(s) : This enum defines the possible error codes.
****************************************************************************************************
*/

enum {
    /*
    ****************************************************************************************************
    *                                             NO ERROR
    *
    * Note(s) : This code is used if everything is OK.
    ****************************************************************************************************
    */
    KXX_CAN_NO_ERR = 0u,
    /*
    ****************************************************************************************************
    *                                            BUS ERROR
    *
    * Note(s) : This code indicates, that a wrong bus was chosen.
    ****************************************************************************************************
    */
    KXX_CAN_BUS_ERR,
    /*
    ****************************************************************************************************
    *                                            BUSY ERROR
    *
    * Note(s) : This code indicates, that a message cannot be sent because the bus is busy.
    ****************************************************************************************************
    */
    KXX_CAN_BUSY_ERR,
    /*
    ****************************************************************************************************
    *                                            INIT ERROR
    *
    * Note(s) : This code indicates, that the devices were not initialized
    *           because they are not in reset state.
    ****************************************************************************************************
    */
    KXX_CAN_INIT_ERR,
    /*
    ****************************************************************************************************
    *                                            MODE ERROR
    *
    * Note(s) : This code indicates, that the device cannot be accessed with the wanted mode.
    ****************************************************************************************************
    */
    KXX_CAN_MODE_ERR,
    /*
    ****************************************************************************************************
    *                                            OPEN ERROR
    *
    * Note(s) : This code indicates, that a device cannot be used because it is not opened.
    ****************************************************************************************************
    */
    KXX_CAN_OPEN_ERR,
    /*
    ****************************************************************************************************
    *                                           CLOSE ERROR
    *
    * Note(s) : This code indicates, that the device cannot be closed.
    ****************************************************************************************************
    */
    KXX_CAN_CLOSE_ERR,
    /*
    ****************************************************************************************************
    *                                       FUNCTION CODE ERROR
    *
    * Note(s) : This code indicates, that the given function code is not valid.
    ****************************************************************************************************
    */
    KXX_CAN_FUNC_ERR,
    /*
    ****************************************************************************************************
    *                                          ARGUMENT ERROR
    *
    * Note(s) : This code indicates, that an argument check has failed.
    ****************************************************************************************************
    */
    KXX_CAN_ARG_ERR,
    /*
    ****************************************************************************************************
    *                                          NO DATA ERROR
    *
    * Note(s) : This code indicates, that no data is available.
    ****************************************************************************************************
    */
    KXX_CAN_NO_DATA_ERR,
    /*
    ****************************************************************************************************
    *                                         TX TIMEOUT ERROR
    *
    * Note(s) : This code indicates, that a timeout during transmission occurred.
    ****************************************************************************************************
    */
    KXX_CAN_TX_TIMEOUT_ERR
};


/*
****************************************************************************************************
*                                    I/O CONTROL FUNCTION CODES
*
* Note(s) : This enum defines the available function codes for the driver XXXIoCtl() function.
****************************************************************************************************
*/

enum {
    /*
    ****************************************************************************************************
    *                                     GET DRIVER IDENT CODE
    *
    * Note(s) : This standard function code gets the driver identification code.
    *
    *           arg = pointer to local ident variable (CPU_INT32U *)
    ****************************************************************************************************
    */
    IO_KXX_CAN_GET_IDENT    = 0x0u,
    /*
    ****************************************************************************************************
    *                                       GET DRIVER ERRORCODE
    *
    * Note(s) : This standard function code gets the driver errorcode.
    *
    *           arg = pointer to local errorcode variable (CPU_INT16U *)
    ****************************************************************************************************
    */
    IO_KXX_CAN_GET_ERRNO,
    /*
    ****************************************************************************************************
    *                                         GET DRIVER NAME
    *
    * Note(s) : This standard function code gets the (human readable) driver name.
    *
    *           arg = pointer to local string variable (char *)
    ****************************************************************************************************
    */
    IO_KXX_CAN_GET_DRVNAME,
    /*
    ****************************************************************************************************
    *                                         SET BUS BAUDRATE
    *
    * Note(s) : This function code sets the bus baudrate.
    *
    *           arg = pointer to local baudrate variable (CPU_INT32U *)
    ****************************************************************************************************
    */
    IO_KXX_CAN_SET_BAUDRATE = 0x10u,
    /*
    ****************************************************************************************************
    *                                            ENABLE BUS
    *
    * Note(s) : This function code starts the CAN controller.
    *
    *           arg = unused
    ****************************************************************************************************
    */
    IO_KXX_CAN_START,
    /*
    ****************************************************************************************************
    *                                           DISABLE BUS
    *
    * Note(s) : This function code stops the CAN controller.
    *
    *           arg = unused
    ****************************************************************************************************
    */
    IO_KXX_CAN_STOP,
    /*
    ****************************************************************************************************
    *                               SET RECEIVER TO STANDARD IDENTIFIER
    *
    * Note(s) : This function code configures the CAN receiver to receive only CAN standard
    *           identifiers.
    *
    *           arg = unused
    ****************************************************************************************************
    */
    IO_KXX_CAN_RX_STANDARD,
    /*
    ****************************************************************************************************
    *                               SET RECEIVER TO EXTENDED IDENTIFIER
    *
    * Note(s) : This function code configures the CAN receiver to receive only CAN extended
    *           identifiers.
    *
    *           arg = unused
    ****************************************************************************************************
    */
    IO_KXX_CAN_RX_EXTENDED,
    /*
    ****************************************************************************************************
    *                                        GET TX READY STATUS
    *
    * Note(s) : This function code gets the Tx ready status.
    *
    *           The parameter pointer shall point to a CPU_INT08U variable, where the status shall
    *           be written to.
    *
    *           0 = Tx not ready
    *           1 = Tx ready
    *
    *           arg = unused
    ****************************************************************************************************
    */
    IO_KXX_CAN_TX_READY,
    /*
    ****************************************************************************************************
    *                                         GET NODE STATUS
    *
    * Note(s) : This function code gets the node status.
    *
    *           The parameter pointer shall point to a CPU_INT08U variable, where the status shall
    *           be written to.
    *
    *           0 = Bus active
    *           1 = Error passive state
    *           2 = Bus-off state
    *
    *           arg = unused
    ****************************************************************************************************
    */
    IO_KXX_CAN_GET_NODE_STATUS,
    /*
    ****************************************************************************************************
    *                                         SET RX FILTER 1
    *
    * Note(s) : This function code sets the acceptance filter 1 for the CAN controller.
    *
    *           arg = CPU_INT32U[2]: arg[0] = mask
    *                                arg[1] = identifier
    ****************************************************************************************************
    */
    IO_KXX_CAN_SET_RX_FILTER_1,
    /*
    ****************************************************************************************************
    *                                         SET RX FILTER 2
    *
    * Note(s) : This function code sets the acceptance filter for the CAN controller.
    *
    *           arg = CPU_INT32U[2]: arg[0] = mask
    *                                arg[1] = identifier
    ****************************************************************************************************
    */
    IO_KXX_CAN_SET_RX_FILTER_2,
    /*
    ****************************************************************************************************
    *                                     NUMBER OF FUNCTION CODES
    *
    * Note(s) : Number of function codes.
    ****************************************************************************************************
    */
    IO_KXX_CAN_IO_FUNC_N
};


/*
****************************************************************************************************
*                                        DEVICE RUNTIME DATA
*
* Note(s) : This struct holds the device runtime data.
****************************************************************************************************
*/

typedef struct {
    /*
    ****************************************************************************************************
    *                                      BASE ADDRESS OF DEVICE
    *
    * Note(s) : This member holds the base address of the device.
    ****************************************************************************************************
    */
    CPU_INT32U  Base;
    /*
    ****************************************************************************************************
    *                                              FILTER
    *
    * Note(s) : This member holds the index of a single filter.
    ****************************************************************************************************
    */
    CPU_INT32U  Filter;
    /*
    ****************************************************************************************************
    *                                      TX BUFFER AVAILABILITY
    *
    * Note(s) : This member holds the availability of the TX buffer.
    ****************************************************************************************************
    */
    CPU_INT32U  TxBufAvail;
    /*
    ****************************************************************************************************
    *                                          DEVICE STATUS
    *
    * Note(s) : This member holds the device status:
    *           - bit 0: 0 = device idle, 1 = device busy
    *           - bit 1: 0 = non blocking mode, 1 = blocking mode
    *           - bit 2: 0 = no message transmitted, 1 = at least one message transmitted
    *           - bit 3..7: not used (always 0)
    ****************************************************************************************************
    */
    CPU_INT08U  Status;
    /*
    ****************************************************************************************************
    *                                             BAUDRATE
    *
    * Note(s) : This member holds the baudrate.
    ****************************************************************************************************
    */
    CPU_INT32U  Baudrate;
    /*
    ****************************************************************************************************
    *                                         BIT SAMPLE POINT
    *
    * Note(s) : This member holds the bit sample point in 1/10 percent.
    ****************************************************************************************************
    */
	CPU_INT32U  SamplePoint;
    /*
    ****************************************************************************************************
    *                                  RE-SYNCHRONIZATION JUMP WIDTH
    *
    * Note(s) : This member holds the Re-synchronization Jump Width in 1/10 percent.
    ****************************************************************************************************
    */
	CPU_INT32U  ResynchJumpWith;
    /*
    ****************************************************************************************************
    *                                     PRESCALER DEVIDE FACTOR
    *
    * Note(s) : This member holds the prescaler devide factor.
    ****************************************************************************************************
    */
	CPU_INT08U  PRESDIV;
    /*
    ****************************************************************************************************
    *                                  RE-SYNCHRONIZATION JUMP WIDTH
    *
    * Note(s) : This member holds the Re-synchronization Jump Width (StdValue = 1).
    ****************************************************************************************************
    */
	CPU_INT08U  RJW;
    /*
    ****************************************************************************************************
    *                                     PROPAGATION SEGMENT TIME
    *
    * Note(s) : This member holds the propagation segment time (StdValue = 2).
    ****************************************************************************************************
    */
	CPU_INT08U  PROPSEG;
    /*
    ****************************************************************************************************
    *                                      PHASE BUFFER SEGMENT 1
    *
    * Note(s) : This member holds the phase buffer segment 1 (StdValue = 7).
    ****************************************************************************************************
    */
	CPU_INT08U  PSEG1;
    /*
    ****************************************************************************************************
    *                                      PHASE BUFFER SEGMENT 2
    *
    * Note(s) : This member holds the phase buffer segment 2 (StdValue = 7).
    ****************************************************************************************************
    */
	CPU_INT08U  PSEG2;
} KXX_CAN_DATA;


/*
****************************************************************************************************
*                                            CAN FRAME
*
* Note(s) : This structure defines a CAN frame.
****************************************************************************************************
*/

typedef  struct {
    /*
    ****************************************************************************************************
    *                                          CAN IDENTIFIER
    *
    *           This member holds the CAN identifier. NOTE: to differentiate standard and extended
    *           identifiers the following addition to the identifier is implemented:
    *
    *           - bit31-30: reserved (always 0)
    *           - bit29:    marks an extended identifier (1=extended, 0=standard)
    *           - bit28-0:  the identifier (standard or extended)
    ****************************************************************************************************
    */
    CPU_INT32U  Identifier;
    /*
    ****************************************************************************************************
    *                                           CAN PAYLOAD
    *
    * Note(s) : This member holds up to 8 bytes, which can be handled with a single CAN message.
    ****************************************************************************************************
    */
    CPU_INT08U  Data[8u];
    /*
    ****************************************************************************************************!
    *                                             CAN DLC
    *
    * Note(s) : This member holds the number of valid datas in the payload.
    ****************************************************************************************************
    */
    CPU_INT08U  DLC;
    /*
    ****************************************************************************************************
    *                                            CAN Spare
    *
    * Note(s) : This member holds three spare bytes.
    ****************************************************************************************************
    */
    CPU_INT08U  Spare[3u];

} KXX_CAN_FRM;


/*
****************************************************************************************************
*                                       FUNCTION PROTOTYPES
****************************************************************************************************
*/

CPU_INT16S  KXX_CAN_Init (CPU_INT32U   arg);

CPU_INT16S  KXX_CAN_Open (CPU_INT16S   drv,
                          CPU_INT32U   devName,
                          CPU_INT16U   mode);

CPU_INT16S  KXX_CAN_Close(CPU_INT16S   devId);

CPU_INT16S  KXX_CAN_IoCtl(CPU_INT16S   devId,
                          CPU_INT16U   func,
                          void        *arg);

CPU_INT16S  KXX_CAN_Read (CPU_INT16S   devId,
                          CPU_INT08U  *buf,
                          CPU_INT16U   size);

CPU_INT16S  KXX_CAN_Write(CPU_INT16S   devId,
                          CPU_INT08U  *buf,
                          CPU_INT16U   size);


#endif  /* #ifndef _DRV_CAN_H_ */

