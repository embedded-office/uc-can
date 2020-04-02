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

#ifndef _DRV_CAN_SJA1000_H_
#define _DRV_CAN_SJA1000_H_

/*
****************************************************************************************************
*                                             INCLUDES
****************************************************************************************************
*/
#include "cpu.h"
#include "lib_def.h"
#include "drv_can_reg.h"

/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/

/*! \brief The unique driver name for installation and searching */
#define SJA1000_CAN_NAME "SJA1000:CAN Module"

#define SJA1000_CAN_ARG_CHK_CFG          1            /* Enable(1) / Disable(0) argument checking */

#define SJA1000_CAN_INTERRUPT_EN         0            /* Enable(1) / Disable(0) interrupt routines*/

#define SJA1000_CAN_N_DEV                1            /* Number of SJA1000 devices                */

/*! \brief The type for the base address may be different for targets and compiler so the right   */
/* type has to set here. This example was for an 8051 processor with a Tasking compiler and a     */
/* reentrant memory model. Normally an 'CPU_INT32U' type may suffice.                             */
#define SJA1000_CAN_ADDR_TYPE            CPU_INT32U

/*! \brief Before calling the CAN Init function make sure that following defines are set to the   */
/*  right base address(es) of the device.                                                         */
#define SJA1000_CAN_BASE_ADDR1           (SJA1000_CAN_ADDR_TYPE *) 0xFC80
#define SJA1000_CAN_BASE_ADDR2           (SJA1000_CAN_ADDR_TYPE *) 0x00530000

/*! \brief The settings of the baud rate registers depends on the clock rate with which           */
/*  the SJA1000 is clocked. Therefore there is only one example given for a clock rate of 16MHz   */
/*  configured to 125kBaud CAN baud rate.                                                         */
#define SJA1000_CAN_BTR1                 (CPU_INT08U)  0x43
#define SJA1000_CAN_BTR2                 (CPU_INT08U)  0x49


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
    IO_SJA1000_CAN_GET_IDENT    = 0x0,

    /*! \brief             GET DRIVER ERRORCODE
     *
     *       This standard function code gets the driver errorcode.
     *
     * arg = pointer to local errorcode variable (CPU_INT16U *)
     */
    IO_SJA1000_CAN_GET_ERRNO,

    /*! \brief             GET DRIVER NAME
     *
     *       This standard function code gets the (human readable) driver name.
     *
     * arg = pointer to local string variable (char *)
     */
    IO_SJA1000_CAN_GET_DRVNAME,

    /*! \brief             SET BUS BAUDRATE
     *
     *       This function code sets the bus baudrate.
     *
     * arg = pointer to local baudrate variable (CPU_INT32U *)
     */
    IO_SJA1000_CAN_SET_BAUDRATE = 0x10,
    /*! \brief Enable Bus
     *
     * This enum value is the functioncode to start the CAN controller interface. Most common
     * is to set the CAN controller in active mode.
     *
     * The parameter pointer is not used for this function.
     */
    IO_SJA1000_CAN_START,
    /*! \brief Disable Bus
     *
     * This enum value is the functioncode to stop the CAN controller interface. Most common
     * is to set the CAN controller in passive mode.
     *
     * The parameter pointer is not used for this function.
     */
    IO_SJA1000_CAN_STOP,
    /*! \brief Set Receiver to Standard Identifier
     *
     * This enum value is the functioncode to configure the CAN receiver to receive only
     * CAN standard identifiers.
     *
     * The parameter pointer is not used for this function.
     */
    IO_SJA1000_CAN_RX_STANDARD,
    /*! \brief Set Receiver to Extended Identifier
     *
     * This enum value is the functioncode to configure the CAN receiver to receive only
     * CAN extended identifiers.
     *
     * The parameter pointer is not used for this function.
     */
    IO_SJA1000_CAN_RX_EXTENDED,
    /*! \brief Get TX Buffer Status
     *
     * This enum value is the functioncode to get the status of the current transmit
     * buffer.
     *
     * The parameter pointer shall point to a CPU_INT08U variable, where the status
     * shall be written to.
     */
     IO_SJA1000_CAN_TX_READY,
    /*! \brief Get Node Status
     *
     * This enum value is the functioncode to get the node status from the
     * CAN controller.
     *
     * The parameter pointer shall point to a CPU_INT08U variable, where the status
     * shall be written to.
     */
    IO_SJA1000_CAN_GET_NODE_STATUS,
    /*! \brief             SET RX ID FILTER 1
     *
     *       This function code sets the identifier RX filter
     *       The arguments are:
     *        - the identifier mask
     *        - the identifier
     *
     * arg = pointer to local filter variables (CPU_INT32U *)
     */
    IO_SJA1000_CAN_SET_RX_FILTER_1,
    /*! \brief Number of Needed IO Function Codes
     *
     * This enum value holds the number of function codes, which are used within the
     * can bus layer.
     */

    IO_SJA1000_CAN_IO_FUNC_N
};

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      USE MARKER
* \ingroup  SJA1000_CAN
*
*           This member holds a marker which indicates, that this
*           device is in use: 0 = Device idle, 1 = Device in use
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct
{
  CPU_INT08U  Use;
} CAN_DATA;


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ADDRESS MARKER
* \ingroup  SJA1000_CAN
*
*           This member holds the base addresses of the sja1000 devices which are in use.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct
{
  SJA1000_CAN_ADDR_TYPE  *BaseAddress;
} CAN_ADDRESSES;


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      BAUDRATE LIST
* \ingroup  SJA1000_CAN
*
*           Static CAN driver baudrate list.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct
{
  /*! \brief Baudrate in bit/s */
  CPU_INT32U Baudrate;
  /*! \brief Corresponding Bit-Timing Register1 */
  CPU_INT08U CANBT1;
  /*! \brief Corresponding Bit-Timing Register2 */
  CPU_INT08U CANBT2;
} CAN_BAUD;


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ERRORCODES
* \ingroup  SJA1000_CAN
*
*           The global errorcode variable 'DrvError' is supported with the following errorcodes.
*/
/*------------------------------------------------------------------------------------------------*/
enum {
    /*! \brief             NO ERROR
     *
     *       This code is used, if everything is ok.
     */
    SJA1000_CAN_NO_ERR = 0,

    /*! \brief             BUS ERROR
     *
     *       This code indicates, that a wrong bus was chosen.
     */
    SJA1000_CAN_BUS_ERR = 2000,

    /*! \brief             BUSY ERROR
     *
     *       This code indicates, that a a msg can not ne send because the bus is busy.
     */
    SJA1000_CAN_BUSY_ERR,

    /*! \brief             INIT ERROR
     *
     *       This code indicates, that the devices were not initialised because they are not
     *       in reset state.
     */
    SJA1000_CAN_INIT_ERR,

    /*! \brief             MODE ERROR
     *
     *       This code indicates, that the device cannot be accessed with the wanted mode.
     */
    SJA1000_CAN_MODE_ERR,

    /*! \brief             OPEN ERROR
     *
     *       This code indicates, that a device cannot be used, because it is not opened.
     */
    SJA1000_CAN_OPEN_ERR,

    /*! \brief             CLOSE ERROR
     *
     *       This code indicates, that the device cannot be closed.
     */
    SJA1000_CAN_CLOSE_ERR,

    /*! \brief             FUNCTION CODE ERROR
     *
     *       This code indicates, that the given function code is not valid.
     */
    SJA1000_CAN_FUNC_ERR,

    /*! \brief             ARGUMENT ERROR
     *
     *       This code indicates, that an argument check has failed.
     */
    SJA1000_CAN_ARG_ERR,

    /*! \brief             NO DATA ERROR
     *
     *       This code indicates, that no data is available.
     */
    SJA1000_CAN_NO_DATA_ERR
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

} SJA1000_CANFRM;


/*
****************************************************************************************************
*                                       FUNCTION PROTOTYPES
****************************************************************************************************
*/

CPU_INT16S SJA1000CANInit    (CPU_INT32U arg);
CPU_INT16S SJA1000CANOpen    (CPU_INT16S devId,  CPU_INT32U devName, CPU_INT16U mode);
CPU_INT16S SJA1000CANClose   (CPU_INT16S paraId);
CPU_INT16S SJA1000CANIoCtl   (CPU_INT16S paraId, CPU_INT16U func,    void* arg);
CPU_INT16S SJA1000CANRead    (CPU_INT16S paraId, CPU_INT08U *buffer,  CPU_INT16U cnt);
CPU_INT16S SJA1000CANWrite   (CPU_INT16S paraId, CPU_INT08U *buffer,  CPU_INT16U cnt);

#if SJA1000_CAN_INTERRUPT_EN > 0
void       SJA1000CANISR     (void);
#endif

/*
****************************************************************************************************
*                                          ERROR SECTION
****************************************************************************************************
*/


#endif  /* #ifndef _DRV_CAN_SJA1000_H_ */






