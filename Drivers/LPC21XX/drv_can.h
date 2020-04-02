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
* Filename : drv_can.h
* Version  : V2.42.01
****************************************************************************************************
*/

#ifndef _DRVCAN_H
#define _DRVCAN_H

#include "cpu.h"                                              /* basic type definitions          */
#include "can_bsp.h"

/*
***************************************************************************************************
*                            CONSTANTS
***************************************************************************************************
*/

/*! \brief The unique driver name for installation and searching */
#define LPC21XX_CAN_NAME "LPC21XX:CAN Module"

/*! \brief  following bit definitions as defined in can_frame.h                                   */
/*!       - bit30: marks a remote transmission request (1=rtr, 0=data frame)                      */
/*!       - bit29: marks an extended identifier (1=extended, 0=standard)                          */
#define LPC21XX_CAN_RTR_FRAME_BIT   0x40000000
#define LPC21XX_CAN_FF_FRAME_BIT    0x20000000

/*
***************************************************************************************************
*                            TYPE DEFINITIONS
***************************************************************************************************
*/

enum
{
  LPC21XX_CAN_BUS_0,                                          /*!< Internal can controller #0             */
  LPC21XX_CAN_BUS_1,                                          /*!< Internal can controller #1             */
  LPC21XX_CAN_N_DEV                                           /*!< Number of used can controller          */
};


/*
*           The global errorcode variable 'DrvError' is supported with the following errorcodes.
*/
enum {
    /*! \brief             NO ERROR
     *
     *       This code is used, if everything is ok.
     */
    LPC21XX_CAN_NO_ERR = 0,

    /*! \brief             BUS ERROR
     *
     *       This code indicates, that a wrong bus was chosen..
     */
    LPC21XX_CAN_BUS_ERR,

    /*! \brief             BUSY ERROR
     *
     *       This code indicates, that a a msg can not ne send because the bus is busy.
     */
    LPC21XX_CAN_BUSY_ERR,

    /*! \brief             INIT ERROR
     *
     *       This code indicates, that the devices were not initialised because they are not in reset state.
     */
    LPC21XX_CAN_INIT_ERR,

    /*! \brief             MODE ERROR
     *
     *       This code indicates, that the device cannot be accessed with the wanted mode.
     */
    LPC21XX_CAN_MODE_ERR,

    /*! \brief             OPEN ERROR
     *
     *       This code indicates, that a device cannot be used, because it is not opened.
     */
    LPC21XX_CAN_OPEN_ERR,

    /*! \brief             CLOSE ERROR
     *
     *       This code indicates, that the device cannot be closed.
     */
    LPC21XX_CAN_CLOSE_ERR,

    /*! \brief             FUNCTION CODE ERROR
     *
     *       This code indicates, that the given function code is not valid.
     */
    LPC21XX_CAN_FUNC_ERR,

    /*! \brief             ARGUMENT ERROR
     *
     *       This code indicates, that an argument check has failed.
     */
    LPC21XX_CAN_ARG_ERR,

    /*! \brief             NO DATA ERROR
     *
     *       This code indicates, that no data is available.
     */
    LPC21XX_CAN_NO_DATA_ERR
};


/*------------------------------------------------------------------------------------------------*/
/*! \brief Functioncodes for CANIoCtl() */
/*------------------------------------------------------------------------------------------------*/
/*! \brief                          I/O CONTROL FUNCTIONCODES
*
*            This enumeration defines the required functioncode values for the lowlevel
*            device driver function IoCtl().
*/
/*------------------------------------------------------------------------------------------------*/
enum  {
    /*! \brief             GET DRIVER IDENT CODE
     *
     *       This standard function code gets the driver identification code.
     *
     * arg = pointer to local ident variable (CPU_INT32U *)
     */
    IO_LPC21XX_CAN_GET_IDENT    = 0x0,

    /*! \brief             GET DRIVER ERRORCODE
     *
     *       This standard function code gets the driver errorcode.
     *
     * arg = pointer to local errorcode variable (CPU_INT16U *)
     */
    IO_LPC21XX_CAN_GET_ERRNO,

    /*! \brief             GET DRIVER NAME
     *
     *       This standard function code gets the (human readable) driver name.
     *
     * arg = pointer to local string variable (char *)
     */
    IO_LPC21XX_CAN_GET_DRVNAME,

    /*! \brief             SET BUS BAUDRATE
     *
     *       This function code sets the bus baudrate.
     *
     * arg = pointer to local baudrate variable (CPU_INT32U *)
     */
    IO_LPC21XX_CAN_SET_BAUDRATE = 0x10,
    /*! \brief Enable Bus
     *
     * This enum value is the functioncode to start the CAN controller interface. Most common
     * is to set the CAN controller in active mode.
     *
     * The parameter pointer is not used for this function.
     */
    IO_LPC21XX_CAN_START,
    /*! \brief Disable Bus
     *
     * This enum value is the functioncode to stop the CAN controller interface. Most common
     * is to set the CAN controller in passive mode.
     *
     * The parameter pointer is not used for this function.
     */
    IO_LPC21XX_CAN_STOP,
    /*! \brief Set Receiver to Standard Identifier
     *
     * This enum value is the functioncode to configure the CAN receiver to receive only
     * CAN standard identifiers.
     *
     * The parameter pointer is not used for this function.
     */
    IO_LPC21XX_CAN_RX_STANDARD,
    /*! \brief Set Receiver to Extended Identifier
     *
     * This enum value is the functioncode to configure the CAN receiver to receive only
     * CAN extended identifiers.
     *
     * The parameter pointer is not used for this function.
     */
    IO_LPC21XX_CAN_RX_EXTENDED,
    /*! \brief Get TX Buffer Status
     *
     * This enum value is the functioncode to get the status of the current transmit
     * buffer.
     *
     * The parameter pointer shall point to a CPU_INT08U variable, where the status
     * shall be written to.
     */
    IO_LPC21XX_CAN_TX_READY,
    /*! \brief Disable RX Interrupts
     *
     * This enum value is the functioncode to disable the receiption complete interrupt.
     *
     */
    IO_LPC21XX_CAN_GET_NODE_STATUS,
    /*! \brief Set standard filter
     *
     * This enum value is the functioncode to set standard acceptance filter for the
     * CAN controller.
    *
     * The parameter pointer is not used for this function.
     */
    IO_LPC21XX_CAN_SET_STD_FILTER,
    /*! \brief Set standard group filter
     *
     * This enum value is the functioncode to set standard group acceptance filter for the
     * CAN controller.
    *
     * The parameter pointer is not used for this function.
     */
    IO_LPC21XX_CAN_SET_STD_GROUP_FILTER,
    /*! \brief Set extended filter
     *
     * This enum value is the functioncode to set extended acceptance filter for the
     * CAN controller.
    *
     * The parameter pointer is not used for this function.
     */
    IO_LPC21XX_CAN_SET_EXT_FILTER,
    /*! \brief Set extended group filter
     *
     * This enum value is the functioncode to set extended group acceptance filter for the
     * CAN controller.
    *
     * The parameter pointer is not used for this function.
     */
    IO_LPC21XX_CAN_SET_EXT_GROUP_FILTER,
    /*! \brief Number of Needed IO Function Codes
     *
     * This enum value holds the number of function codes, which are used within the
     * can bus layer.
     */
    IO_LPC21XX_CAN_IO_FUNC_N
};

/*------------------------------------------------------------------------------------------------*/
/*! \brief Dynamic CAN driver data  */
typedef struct
{
  /*! \brief Use Marker
   *
   * This member holds a marker which indicates, that this device is in use:
   * 0 = Device idle,
   * 1 = Device in use
   */
  CPU_INT08U Use;

} LPC21XX_CAN_DATA;


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN FRAME
*
*           This structure contains all needed data to handle a single CAN frame
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct {
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief                  CAN IDENTIFIER
    *
    *       This member holds the CAN identifier.
    *
    */
    /*--------------------------------------------------------------------------------------------*/
    CPU_INT32U Identifier;
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief                  CAN PAYLOAD
    *
    *       This member holds up to 8 bytes, which can be handled with a single CAN message.
    */
    /*--------------------------------------------------------------------------------------------*/
    CPU_INT08U Data[8];
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief                  CAN DLC
    *
    *       This member holds the number of valid datas in the payload.
    */
    /*--------------------------------------------------------------------------------------------*/
    CPU_INT08U DLC;
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief                  SPARE
    *
    *       These bytes are added to get a frame size of an integral number of pointers.
    */
    /*--------------------------------------------------------------------------------------------*/
    CPU_INT08U Spare[3];

} LPC21XX_CANFRM;

/*
***************************************************************************************************
*                            FUNCTION PROTOTYPES
***************************************************************************************************
*/

CPU_INT16S LPC21XXCANInit(CPU_INT32U arg);
CPU_INT16S LPC21XXCANOpen(CPU_INT16S drvId, CPU_INT32U devName, CPU_INT16U mode);
CPU_INT16S LPC21XXCANClose(CPU_INT16S paraId);
CPU_INT16S LPC21XXCANIoCtl(CPU_INT16S paraId, CPU_INT16U func, void *argp);
CPU_INT16S LPC21XXCANRead(CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size);
CPU_INT16S LPC21XXCANWrite(CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size);


/*
***************************************************************************************************
*                            ERROR SECTION
***************************************************************************************************
*/


#endif

/*! } */
