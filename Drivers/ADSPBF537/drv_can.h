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

#ifndef _DRV_CAN_H_
#define _DRV_CAN_H_

/*
****************************************************************************************************
*                                             INCLUDES
****************************************************************************************************
*/
#include "cpu.h"
#include "drv_can_reg.h"

/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/

/*! \brief The unique driver name for installation and searching */
#define ADSPBF537_CAN_NAME "ADSPBF537:CAN Module"

/*! \brief  following bit definitions as defined in can_frame.h                                   */
/*!       - bit30: marks a remote transmission request (1=rtr, 0=data frame)                      */
/*!       - bit29: marks an extended identifier (1=extended, 0=standard)                          */
#define ADSPBF537_CAN_RTR_FRAME_BIT   0x40000000
#define ADSPBF537_CAN_FF_FRAME_BIT    0x20000000

/*
****************************************************************************************************
*                                              MACROS
****************************************************************************************************
*/

/*
****************************************************************************************************
*                                            DATA TYPES
****************************************************************************************************
*/

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
    IO_ADSPBF537_CAN_GET_IDENT    = 0x0,

    /*! \brief             GET DRIVER ERRORCODE
     *
     *       This standard function code gets the driver errorcode.
     *
     * arg = pointer to local errorcode variable (CPU_INT16U *)
     */
    IO_ADSPBF537_CAN_GET_ERRNO,

    /*! \brief             GET DRIVER NAME
     *
     *       This standard function code gets the (human readable) driver name.
     *
     * arg = pointer to local string variable (char *)
     */
    IO_ADSPBF537_CAN_GET_DRVNAME,

    /*! \brief             SET BUS BAUDRATE
     *
     *       This function code sets the bus baudrate.
     *
     * arg = pointer to local baudrate variable (CPU_INT32U *)
     */
    IO_ADSPBF537_CAN_SET_BAUDRATE = 0x10,
    /*! \brief Enable Bus
     *
     * This enum value is the functioncode to start the CAN controller interface. Most common
     * is to set the CAN controller in active mode.
     *
     * The parameter pointer is not used for this function.
     */
    IO_ADSPBF537_CAN_START,
    /*! \brief Disable Bus
     *
     * This enum value is the functioncode to stop the CAN controller interface. Most common
     * is to set the CAN controller in passive mode.
     *
     * The parameter pointer is not used for this function.
     */
    IO_ADSPBF537_CAN_STOP,
    /*! \brief Set Receiver to Standard Identifier
     *
     * This enum value is the functioncode to configure the CAN receiver to receive only
     * CAN standard identifiers.
     *
     * The parameter pointer is not used for this function.
     */
    IO_ADSPBF537_CAN_RX_STANDARD,
    /*! \brief Set Receiver to Extended Identifier
     *
     * This enum value is the functioncode to configure the CAN receiver to receive only
     * CAN extended identifiers.
     *
     * The parameter pointer is not used for this function.
     */
    IO_ADSPBF537_CAN_RX_EXTENDED,
    /*! \brief Get TX Buffer Status
     *
     * This enum value is the functioncode to get the status of the current transmit
     * buffer.
     *
     * The parameter pointer shall point to a CPU_INT08U variable, where the status
     * shall be written to.
     */
     IO_ADSPBF537_CAN_TX_READY,
    /*! \brief Get Node Status
     *
     * This enum value is the functioncode to get the node status from the
     * CAN controller.
     *
     * The parameter pointer shall point to a CPU_INT08U variable, where the status
     * shall be written to.
     */
    IO_ADSPBF537_CAN_GET_NODE_STATUS,
    /*! \brief             SET RX FILTER
     *
     *       This function code sets the identifier RX filter
     *       The arguments are:
     *        - the identifier mask
     *        - the identifier
     *
     * arg = pointer to local filter variables (CPU_INT32U *)
     */
    IO_ADSPBF537_CAN_SET_RX_FILTER,
    /*! \brief Number of Needed IO Function Codes
     *
     * This enum value holds the number of function codes, which are used within the
     * can bus layer.
     */

    IO_ADSPBF537_CAN_IO_FUNC_N
};

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      USE MARKER
* \ingroup  ADSPBF537_CAN
*
*           This member holds a marker which indicates, that this
*           device is in use: 0 = Device idle, 1 = Device in use
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct
{
    /*! This member open/close information the module */
    CPU_INT08U            Use;

    /*! This member holds the base adresses of the module */
    volatile CPU_INT32U * BaseAddr;

    /*! This member holds the baudrate. */
    CPU_INT32U Baudrate;

    /*! This member holds the bit sample point in 1/10 percent. */
    CPU_INT32U SamplePoint;

    /*! This member holds the Re-synchronization Jump Width in 1/10 percent. */
    CPU_INT32U ResynchJumpWith;

    /*! This member holds the prescaler devide factor. */
    CPU_INT16U PRESDIV;

    /*! This member holds the resynchronization jump width (StdValue = 1). */
    CPU_INT08U RJW;

    /*! This member holds the propagation segment time (StdValue = 2). */
    CPU_INT08U PROPSEG;

    /*! This member holds the phase buffer segment 1 (StdValue = 7). */
    CPU_INT08U PSEG1;

    /*! This member holds the phase buffer segment 2 (StdValue = 7). */
    CPU_INT08U PSEG2;

} CAN_DATA;


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      AVAILABLE CAN DEVICES
* \ingroup  ADSPBF537_CAN
*
*           Available CAN devices
*/
/*------------------------------------------------------------------------------------------------*/

enum
{
  ADSPBF537_CAN_BUS_0,                                          /*!< Internal can controller #0             */
  ADSPBF537_CAN_N_DEV                                           /*!< Number of used can controller          */
};


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ERRORCODES
* \ingroup  ADSPBF537_CAN
*
*           The global errorcode variable 'DrvError' is supported with the following errorcodes.
*/
/*------------------------------------------------------------------------------------------------*/
enum {
    /*! \brief             NO ERROR
     *
     *       This code is used, if everything is ok.
     */
    ADSPBF537_CAN_NO_ERR = 0,

    /*! \brief             BUS ERROR
     *
     *       This code indicates, that a wrong bus was chosen.
     */
    ADSPBF537_CAN_BUS_ERR,

    /*! \brief             BUSY ERROR
     *
     *       This code indicates, that a a msg can not ne send because the bus is busy.
     */
    ADSPBF537_CAN_BUSY_ERR,

    /*! \brief             INIT ERROR
     *
     *       This code indicates, that the devices were not initialised because they are not
     *       in reset state.
     */
    ADSPBF537_CAN_INIT_ERR,

    /*! \brief             MODE ERROR
     *
     *       This code indicates, that the device cannot be accessed with the wanted mode.
     */
    ADSPBF537_CAN_MODE_ERR,

    /*! \brief             OPEN ERROR
     *
     *       This code indicates, that a device cannot be used, because it is not opened.
     */
    ADSPBF537_CAN_OPEN_ERR,

    /*! \brief             CLOSE ERROR
     *
     *       This code indicates, that the device cannot be closed.
     */
    ADSPBF537_CAN_CLOSE_ERR,

    /*! \brief             FUNCTION CODE ERROR
     *
     *       This code indicates, that the given function code is not valid.
     */
    ADSPBF537_CAN_FUNC_ERR,

    /*! \brief             ARGUMENT ERROR
     *
     *       This code indicates, that an argument check has failed.
     */
    ADSPBF537_CAN_ARG_ERR,

    /*! \brief             NO DATA ERROR
     *
     *       This code indicates, that no data is available.
     */
    ADSPBF537_CAN_NO_DATA_ERR
};

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
    /*! \brief                SPARE
    *
    *       These bytes are added to get a frame size of an integral number of pointers.
    */
    /*--------------------------------------------------------------------------------------------*/
    CPU_INT08U Spare[3];

} ADSPBF537_CANFRM;


/*
****************************************************************************************************
*                                       FUNCTION PROTOTYPES
****************************************************************************************************
*/


CPU_INT16S ADSPBF537CANInit    (CPU_INT32U arg);
CPU_INT16S ADSPBF537CANOpen    (CPU_INT16S devId,  CPU_INT32U devName, CPU_INT16U mode);
CPU_INT16S ADSPBF537CANClose   (CPU_INT16S paraId);
CPU_INT16S ADSPBF537CANIoCtl   (CPU_INT16S paraId, CPU_INT16U func,    void* arg);
CPU_INT16S ADSPBF537CANRead    (CPU_INT16S paraId, CPU_INT08U *buffer,  CPU_INT16U cnt);
CPU_INT16S ADSPBF537CANWrite   (CPU_INT16S paraId, CPU_INT08U *buffer,  CPU_INT16U cnt);

/*
****************************************************************************************************
*                                          ERROR SECTION
****************************************************************************************************
*/


#endif  /* #ifndef _DRV_CAN_H_ */






