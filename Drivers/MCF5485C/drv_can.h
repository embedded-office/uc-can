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
#include "cpu.h"                              /* CPU configuration definitions and constants      */

/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/
/*! \brief The unique driver name for installation and searching */
#define MCF5485C_CAN_NAME "MCF5485C:CAN Module"

/*!
* \brief                      MCF5485C CAN MODULE FIRST RX BUFFER
* \ingroup  MCF5485C_CAN
*
*           Defines the first message buffers used to receive data.
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_RX_FIRST_MSG_BUF     1

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MCF5485C CAN MODULE LAST RX BUFFER
* \ingroup  MCF5485C_CAN
*
*           Defines the last message buffers used to receive data.
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_RX_LAST_MSG_BUF      13

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MCF5485C CAN MODULE MSG BUFFER 14 MASK
*           Defines the interupt mask of the message buffer 14.
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_MSG_BUF_14_MASK      0x00004000L

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MCF5485C CAN MODULE MSG BUFFER 15 MASK
*           Defines the interupt mask of the message buffer 15.
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_MSG_BUF_15_MASK      0x00008000L

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      STANDARD ID MASK
* \ingroup  MCF5485C_CAN
*
*           Max. possible Identifier if using standard CAN frames.
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_STD_ID_MASK          0x000007FF

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      EXTENDED ID MASK
* \ingroup  MCF5485C_CAN
*
*           Max. possible Identifier if using extended CAN frames.
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_EXT_ID_MASK          0x1FFFFFFF

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      EXTENDED ID FLAG
* \ingroup  MCF5485C_CAN
*
*           This Flag must be set in the CAN Identifier if a standard CAN ID shall be send as a
*           extended message frame.
*           In a received CAN message frame this flag indicates a extended Identifier.
*
* \see      MCF5485C_CAN_FRM
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_EXT_ID_FLAG          0x20000000

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DEVICES
* \ingroup  MCF5485C_CAN
*
*           This list holds the device names and the maximal number of supported physical can
*           controllers within the MCF5485C controller derivat.
*/
/*------------------------------------------------------------------------------------------------*/
enum mpc5554_can_devname {

    MCF5485C_CAN_BUS_A = 0,                          /*!< FlexCAN Bus Module A                   */
    MCF5485C_CAN_BUS_B,                              /*!< FlexCAN Bus Module B                   */

    MCF5485C_CAN_DEV_N                               /*!> Number of Devices                     */
};

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      STATUS: CAN BUS IDLE
* \ingroup  MCF5485C_CAN
*
*           This constant holds the status value, which is set when the CAN bus is idle.
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_IDLE              0x00

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      STATUS: CAN BUS OPEN
* \ingroup  MCF5485C_CAN
*
*           This constant holds the status value, which is set when the CAN bus is opened.
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_OPEN              0x01

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      STATUS: BLOCKING ACCESS
* \ingroup  MCF5485C_CAN
*
*           This constant holds the status value, which is set when the driver works in blocking
*           access mode.
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_BLOCKING          0x02

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      STATUS: FIRST TX DONE
* \ingroup  MCF5485C_CAN
*
*           This constant holds the status value, which is set when the driver has transmitted
*           at least one message.
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_FIRST_TX          0x04

/*!
* \brief                      MCF5485C CAN MODULE FIRST RX BUFFER
* \ingroup  MCF5485C_CAN
*
*           Defines the first message buffers used to receive data.
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_RX_FIRST_MSG_BUF     1

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MCF5485C CAN MODULE LAST RX BUFFER
* \ingroup  MCF5485C_CAN
*
*           Defines the last message buffers used to receive data.
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_RX_LAST_MSG_BUF      13

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MCF5485C CAN MODULE MSG BUFFER 14 MASK
*           Defines the interupt mask of the message buffer 14.
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_MSG_BUF_14_MASK      0x00004000L

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MCF5485C CAN MODULE MSG BUFFER 15 MASK
*           Defines the interupt mask of the message buffer 15.
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_MSG_BUF_15_MASK      0x00008000L

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ERRORCODES
* \ingroup  MCF5485C_CAN
*
*           The global errorcode variable 'DrvError' is supported with the following errorcodes.
*/
/*------------------------------------------------------------------------------------------------*/
enum {
    /*! \brief             NO ERROR
     *
     *       This code is used, if everything is ok.
     */
    MCF5485C_CAN_NO_ERR = 0,

    /*! \brief             BUS ERROR
     *
     *       This code indicates, that a wrong bus was chosen.
     */
    MCF5485C_CAN_BUS_ERR,

    /*! \brief             BUSY ERROR
     *
     *       This code indicates, that a a msg can not ne send because the bus is busy.
     */
    MCF5485C_CAN_BUSY_ERR,

    /*! \brief             INIT ERROR
     *
     *       This code indicates, that the devices were not initialised because they are not
     *       in reset state.
     */
    MCF5485C_CAN_INIT_ERR,

    /*! \brief             MODE ERROR
     *
     *       This code indicates, that the device cannot be accessed with the wanted mode.
     */
    MCF5485C_CAN_MODE_ERR,

    /*! \brief             OPEN ERROR
     *
     *       This code indicates, that a device cannot be used, because it is not opened.
     */
    MCF5485C_CAN_OPEN_ERR,

    /*! \brief             CLOSE ERROR
     *
     *       This code indicates, that the device cannot be closed.
     */
    MCF5485C_CAN_CLOSE_ERR,

    /*! \brief             FUNCTION CODE ERROR
     *
     *       This code indicates, that the given function code is not valid.
     */
    MCF5485C_CAN_FUNC_ERR,

    /*! \brief             ARGUMENT ERROR
     *
     *       This code indicates, that an argument check has failed.
     */
    MCF5485C_CAN_ARG_ERR,

    /*! \brief             NO DATA ERROR
     *
     *       This code indicates, that no data is available.
     */
    MCF5485C_CAN_NO_DATA_ERR,

    /*! \brief             TX TIMEOUT ERROR
     *
     *       This code indicates, that a timeout during transmission occurs.
     */
    MCF5485C_CAN_TX_TIMEOUT_ERR
};

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      FUNCTION CODES
* \ingroup  MCF5485C_CAN
*
*           The supported function codes for use as parameter 'func' in the IoCtl() function.
*/
/*------------------------------------------------------------------------------------------------*/
enum mpc5554_can_ioctl_func {
    /*! \brief             GET DRIVER IDENT CODE
     *
     *       This standard function code gets the driver identification code.
     *
     * arg = pointer to local ident variable (CPU_INT32U *)
     */
    IO_MCF5485C_CAN_GET_IDENT    = 0x0,

    /*! \brief             GET DRIVER ERRORCODE
     *
     *       This standard function code gets the driver errorcode.
     *
     * arg = pointer to local errorcode variable (CPU_INT16U *)
     */
    IO_MCF5485C_CAN_GET_ERRNO,

    /*! \brief             GET DRIVER NAME
     *
     *       This standard function code gets the (human readable) driver name.
     *
     * arg = pointer to local string variable (char *)
     */
    IO_MCF5485C_CAN_GET_DRVNAME,

    /*! \brief             SET BUS BAUDRATE
     *
     *       This function code sets the bus baudrate.
     *
     * arg = pointer to local baudrate variable (CPU_INT32U *)
     */
    IO_MCF5485C_CAN_SET_BAUDRATE = 0x10,

    /*! \brief             ENABLE BUS
     *
     *       This enum value is the functioncode to start the CAN controller interface.
     *       Most common is to set the CAN controller in active mode.
     *
     * The parameter pointer is not used for this function.
     */
    IO_MCF5485C_CAN_START,

    /*! \brief             DISABLE BUS
     *
     *       This enum value is the functioncode to stop the CAN controller interface.
     *       Most common is to set the CAN controller in passive mode.
     *
     * The parameter pointer is not used for this function
     */
    IO_MCF5485C_CAN_STOP,

    /*! \brief             SET RECEIVER TO STANDARD IDENTIFIER
     *
     *       This enum value is the functioncode to configure the CAN receiver to
     *       receive only CAN standard identifiers.
     *
     * The parameter pointer is not used for this function
     */
    IO_MCF5485C_CAN_RX_STANDARD,

    /*! \brief             SET RECEIVER TO EXTENDED IDENTIFIER
     *
     *       This enum value is the functioncode to configure the CAN receiver to
     *       receive only CAN extended identifiers
     *
     * The parameter pointer is not used for this function
     */
    IO_MCF5485C_CAN_RX_EXTENDED,

    /*! \brief Get TX Buffer Status
     *
     *       This enum value is the functioncode to get the status of the current transmit
     *       buffer.
     *
     * The parameter pointer shall point to a CPU_INT08U variable, where the status
     * shall be written to.
     */
    IO_MCF5485C_CAN_TX_READY,

    /*! \brief GET NODE STATUS
    *
    * This enum value is the functioncode to get the node status from the
    * CAN controller.
    *
    * The parameter pointer shall point to a CPU_INT08U variable, where the status
    * shall be written to.
    */
    IO_MCF5485C_CAN_GET_NODE_STATUS,

    /*! \brief             SET RX ID FILTER 1
     *
     *       This function code sets the first identifier RX filter
     *       The arguments are:
     *        - the identifier mask
     *        - the identifier
     *
     * arg = pointer to local filter variables (CPU_INT32U *)
     */
    IO_MCF5485C_CAN_SET_RX_FILTER_1,

    /*! \brief             SET RX ID FILTER 2
     *
     *       This function code sets the second identifier RX filter
     *       The arguments are:
     *        - the identifier mask
     *        - the identifier
     *
     * arg = pointer to local filter variables (CPU_INT32U *)
     */
    IO_MCF5485C_CAN_SET_RX_FILTER_2,

    /*! \brief             SET RX ID FILTER 3
     *
     *       This function code sets the third identifier RX filter
     *       The arguments are:
     *        - the identifier mask
     *        - the identifier
     *
     * arg = pointer to local filter variables (CPU_INT32U *)
     */
    IO_MCF5485C_CAN_SET_RX_FILTER_3,

    /*! \brief             CONFIGURE BIT TIMING
     *
     *       This function configure the bit timing registers.
     *       The arguments are:
     *        - the bit sample point in 1/10 percent
     *        - the Re-synchronization Jump Width in 1/10 percent
     *
     * arg = pointer to local timing variables (CPU_INT32U *)
     */
    IO_MCF5485C_CAN_CFG_BIT_TIMING,

    /*! \brief             SET BLOCKING MODE
     *
     *       This function code sets the read/write operations to blocking mode
     *
     * arg = not used (should be 0)
     */
    IO_MCF5485C_CAN_SET_BLOCKING_MODE,

    /*! \brief             SET NON BLOCKING MODE
     *
     *       This function code sets the read/write operations to non blocking mode
     *
     * arg = not used (should be 0)
     */
    IO_MCF5485C_CAN_SET_NON_BLOCKING_MODE

};

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
/*!
* \brief                      DEVICE PARAMETER
* \ingroup  MCF5485C_CAN
*
*           This structure holds the global data for the FlexCAN module
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct {

    /*! This member holds the interrupt prio for the RX message buffers */
    CPU_INT08U MBuffIrqPrio;

    /*! This member holds the interrupt prio for the Buss off interrupt */
    CPU_INT08U BusOffIrqPrio;

    /*! This member holds the interrupt prio for the error interrupt */
    CPU_INT08U ErrIrqPrio;

} MCF5485C_CAN_PARA;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DEVICE CONFIGURATION DATA
* \ingroup  MCF5485C_CAN
*
*           This structure holds the dynamic configuration data for a device.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct {
    /*! This member holds the interrupt prio for the RX message buffers */
    CPU_INT08U MBuffIrqPrio;

    /*! This member holds the interrupt prio for the Buss off interrupt */
    CPU_INT08U BusOffIrqPrio;

    /*! This member holds the interrupt prio for the error interrupt */
    CPU_INT08U ErrIrqPrio;

} MPC5XX_CAN_PARA;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DEVICE RUNTIME DATA
* \ingroup  MPC5XX_CAN
*
*           This structure holds the dynamic runtime data for a device.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct {
    /*! This member holds the base adress of the FlexCAN module */
    CPU_INT32U Base;

    /*! \brief             DEVICE STATUS
    *
    *        This member holds the device status:
    *        - bit 0: 0 = Device idle, 1 = Device busy
    *        - bit 1: 0 = non blocking mode, 1 = blocking mode
    *        - bit 2: 0 = no message transmitted, 1 = at least one message transmitted
    *        - bit 3..7: not used (always 0)
    */
    CPU_INT08U Status;

    /*! \brief             CURRENT RX MESSAGE BUFFER NUMBER
    *
    *        This member holds the number of the current activated RX message buffer.
    */
    CPU_INT08U WrRxMB;

    /*! \brief             CURRENT RX MESSAGE BUFFER MASK
    *
    *        This member holds the mask of the current activated RX message buffer.
    */
    CPU_INT16U WrRxMask;

    /*! \brief             NEXT RX MESSAGE BUFFER NUMBER
    *
    *        This member holds the number of the next activated RX message buffer.
    */
    CPU_INT08U RdRxMB;

    /*! \brief             LAST TRANSMITTED MESSAGE ID (UPPER REG)
    *
    *        This member holds the last transmitted CAN identifier register content.
    */
    CPU_INT32U ID;

    /*! \brief             TX TIMEOUT
    *
    *        This member holds the timeout for transmit polling (non blocking mode).
    */
    CPU_INT32U TxTimeout;

    /*! This member holds the baudrate. */
    CPU_INT32U Baudrate;

    /*! This member holds the bit sample point in 1/10 percent. */
    CPU_INT32U SamplePoint;

    /*! This member holds the Re-synchronization Jump Width in 1/10 percent. */
    CPU_INT32U ResynchJumpWith;

    /*! This member holds the prescaler devide factor. */
    CPU_INT08U PRESDIV;

    /*! This member holds the resynchronization jump width (StdValue = 1). */
    CPU_INT08U RJW;

    /*! This member holds the propagation segment time (StdValue = 2). */
    CPU_INT08U PROPSEG;

    /*! This member holds the phase buffer segment 1 (StdValue = 7). */
    CPU_INT08U PSEG1;

    /*! This member holds the phase buffer segment 2 (StdValue = 7). */
    CPU_INT08U PSEG2;

} MCF5485C_CAN_DATA;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN FRAME STRUCTURE
* \ingroup  MCF5485C_CAN
*
*           This structure holds the informations for a single CAN frame.
*           This structure is used to transfer data from and to the CAN controller using
*           the read/write functions.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct {
    /*! \brief             CAN IDENTIFIER
     *
     *       This member holds the CAN identifier. Note, to differentiate standard and extended
     *       identifiers the following addition to the identifier is implemented:
     *
     *       - bit31-30: reserved (always 0)
     *       - bit29: marks an extended identifier (1=extended, 0=standard)
     *       - bit28-0: the identifier (standard or extended)
     */
    CPU_INT32U Identifier;

    /*! \brief             CAN PAYLOAD
     *
     *       This member holds up to 8 bytes, which can be handled with a single CAN message.
     */
    CPU_INT08U Data[8];

    /*! \brief             CAN DLC
    *
    *        This member holds the number of valid datas in the payload.
    */
    CPU_INT08U DLC;

    CPU_INT08U Spare[3];

} MCF5485C_CAN_FRM;

/*
****************************************************************************************************
*                                       FUNCTION PROTOTYPES
****************************************************************************************************
*/
CPU_INT16S MCF5485CCANInit  (CPU_INT32U arg);
CPU_INT16S MCF5485CCANOpen  (CPU_INT16S drv, CPU_INT32U devName, CPU_INT16U mode);
CPU_INT16S MCF5485CCANClose (CPU_INT16S devId);
CPU_INT16S MCF5485CCANIoCtl (CPU_INT16S devId, CPU_INT16U func, void *arg);
CPU_INT16S MCF5485CCANRead  (CPU_INT16S devId, CPU_INT08U *buf, CPU_INT16U size);
CPU_INT16S MCF5485CCANWrite (CPU_INT16S devId, CPU_INT08U *buf, CPU_INT16U size);


#endif  /* #ifndef _DRV_CAN_H_ */

