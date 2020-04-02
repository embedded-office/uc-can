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
#include "cpu.h"                                /* CPU configuration definitions and constants      */
#include "drv_can_reg.h"                        /* Register definitions for CAN module              */
#include "can_bsp.h"
/*
 ****************************************************************************************************
 *                                             DEFINES
 ****************************************************************************************************
 */

/*! \brief The unique driver name for installation and searching */
#define MSCAN_CAN_NAME "MPC5200B:MSCAN Module"

/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief                      DEFAULT SAMPLE POINT
 * \ingroup  MSCAN
 *
 *           Default bit sample point in 1/10 percent.
 */
/*------------------------------------------------------------------------------------------------*/
#define MSCAN_DEF_SP            750

/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief                      DEFAULT RJW
 * \ingroup  MSCAN
 *
 *           Default Re-synchronization Jump Width in 1/10 percent.
 */
/*------------------------------------------------------------------------------------------------*/
#define MSCAN_DEF_RJW           125

/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief                      DEFAULT BAUDRATE
 * \ingroup  MSCAN
 *
 *           Default baudrate of the CAN modules.
 */
/*------------------------------------------------------------------------------------------------*/
#define MSCAN_DEF_BAUDRATE      1000000L

/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief                      STANDARD ID MASK
 * \ingroup  MSCAN
 *
 *           Max. possible Identifier if using standard CAN frames.
 */
/*------------------------------------------------------------------------------------------------*/
#define MSCAN_STD_ID_MASK       0x000007FF

/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief                      EXTENDED ID MASK
 * \ingroup  MSCAN
 *
 *           Max. possible Identifier if using extended CAN frames.
 */
/*------------------------------------------------------------------------------------------------*/
#define MSCAN_EXT_ID_MASK       0x1FFFFFFF

/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief                      EXTENDED ID FLAG
 * \ingroup  MSCAN
 *
 *           This Flag must be set in the CAN Identifier if a standard CAN ID shall be send as a
 *           extended message frame.
 *           In a received CAN message frame this flag indicates a extended Identifier.
 *
 * \see      MSCAN_FRM
 */
/*------------------------------------------------------------------------------------------------*/
#define MSCAN_EXT_ID_FLAG       0x20000000

/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief                      REMOTE TRANSMISSION FLAG
 * \ingroup  MSCAN
 *
 *           This Flag is set in the CAN Identifier if a remote transmission frame is received.
 *
 * \see      MSCAN_FRM
 */
/*------------------------------------------------------------------------------------------------*/
#define MSCAN_RTR_FLAG         0x40000000

/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief                      STATUS: CAN BUS IDLE
 * \ingroup  MSCAN
 *
 *           This constant holds the status value, which is set when the CAN bus is idle.
 */
/*------------------------------------------------------------------------------------------------*/
#define MSCAN_IDLE              0x00

/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief                      STATUS: CAN BUS OPEN
 * \ingroup  MSCAN
 *
 *           This constant holds the status value, which is set when the CAN bus is opened.
 */
/*------------------------------------------------------------------------------------------------*/
#define MSCAN_OPEN              0x01

/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief                      STATUS: BLOCKING ACCESS
 * \ingroup  MSCAN
 *
 *           This constant holds the status value, which is set when the driver works in blocking
 *           access mode.
 */
/*------------------------------------------------------------------------------------------------*/
#define MSCAN_BLOCKING          0x02

/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief                      ERRORCODES
 * \ingroup  MSCAN
 *
 *           The global errorcode variable 'DrvError' is supported with the following errorcodes.
 */
/*------------------------------------------------------------------------------------------------*/
enum {
    MSCAN_CAN_NO_ERR = 0,
    /*! \brief BUS ERROR
     *
     * This code indicates, that a wrong bus was chosen..
     */
    MSCAN_CAN_BUS_ERR,

    /*! \brief BUSY ERROR
     *
     * This code indicates, that a msg can not be send because the bus is busy.
     */
    MSCAN_CAN_BUSY_ERR,

    /*! \brief INIT ERROR
     *
     * This code indicates that the devices cannot initialize.
     */
    MSCAN_CAN_INIT_ERR,

    /*! \brief MODE ERROR
     *
     * This code indicates that the device cannot be accessed with the wanted mode.
     */
    MSCAN_CAN_MODE_ERR,

    /*! \brief OPEN ERROR
     *
     * This code indicates, that a device can not be used, because it is not opened.
     */
    MSCAN_CAN_OPEN_ERR,

    /*! \brief CLOSE ERROR
     *
     * This code indicates, that the device can not be closed.
     */
    MSCAN_CAN_CLOSE_ERR,

    /*! \brief FUNCTION CODE ERROR
     *
     * This code indicates, that the given function code is not valid.
     */
    MSCAN_CAN_FUNC_ERR,

    /*! \brief ARGUMENT ERROR
     *
     */
    MSCAN_CAN_ARG_ERR,

    /*! \brief NO DATA ERROR
     *
     * This code indicates, that no data is available.
     */
    MSCAN_CAN_NO_DATA_ERR
};
/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief                      FUNCTION CODES
 * \ingroup  MSCAN
 *
 *           The supported function codes for use as parameter 'func' in the IoCtl() function.
 */
/*------------------------------------------------------------------------------------------------*/
enum mscan_ioctl_func {
    /*! \brief GET DRIVER IDENT CODE
     *
     * This standard function code gets the driver identification code.
     *
     * arg = pointer to local ident variable (CPU_INT32U *)
     */
    IO_MSCAN_CAN_GET_IDENT = 0x0,
    /*! \brief GET DRIVER ERRORCODE
     *
     * This standard function code gets the driver errorcode.
     *
     * arg = pointer to local errorcode variable (CPU_INT16U *)
     */
    IO_MSCAN_CAN_GET_ERRNO,
    /*! \brief GET DRIVER NAME
     *
     * This standard function code gets the (human readable) driver name.
     *
     * arg = pointer to local string variable (char *)
     */
    IO_MSCAN_CAN_GET_DRVNAME,
    /*! \brief SET BUS BAUDRATE
     *
     * This function code sets the bus baudrate.
     *
     * arg = pointer to local baudrate variable (CPU_INT32U *)
     */
    IO_MSCAN_CAN_SET_BAUDRATE = 0x10,
    /*! \brief ENABLE BUS
     *
     * This enum value is the functioncode to start the CAN controller interface.
     * Most common is to set the CAN controller in active mode.
     *
     * The parameter pointer is not used for this function.
     */
    IO_MSCAN_CAN_START,
    /*! \brief DISABLE BUS
     *
     * This enum value is the functioncode to stop the CAN controller interface.
     * Most common is to set the CAN controller in passive mode.
     *
     * The parameter pointer is not used for this function.
     */
    IO_MSCAN_CAN_STOP,
    /*! \brief SET RECEIVER TO STANDARD IDENTIFIER
     *
     * This enum value is the functioncode to configure the CAN receiver to receive only
     * CAN standard identifiers.
     *
     * The parameter pointer is not used for this function.
     */
    IO_MSCAN_CAN_RX_STANDARD,
    /*! \brief SET RECEIVER TO EXTENDED IDENTIFIER
     *
     * This enum value is the functioncode to configure the CAN receiver to receive only
     * CAN extended identifiers.
     *
     * The parameter pointer is not used for this function.
     */
    IO_MSCAN_CAN_RX_EXTENDED,
    /*! \brief Get TX Buffer Status
     *
     * This enum value is the functioncode to get the status of the current transmit
     * buffer.
     *
     * The parameter pointer shall point to a CPU_INT8U variable, where the status
     * shall be written to.
     */
    IO_MSCAN_CAN_TX_READY,

    /*! \brief GET NODE STATUS
     *
     * This enum value is the functioncode to get the node status from the
     * CAN controller.
     *
     * The parameter pointer shall point to a CPU_INT32U variable, where the status
     * shall be written to.
     */
    IO_MSCAN_CAN_GET_NODE_STATUS,

    /*! \brief SET RX ID FILTER 1
     *
     * This function code sets the first identifier RX filter
     * The arguments are:
     * - the identifier mask
     * - the identifier
     *
     * arg = pointer to local filter variables (CPU_INT32U *)
     */
    IO_MSCAN_CAN_SET_RX_FILTER_1,

    /*! \brief             SET RX ID FILTER 2
     *
     *       This function code sets the second identifier RX filter
     *       The arguments are:
     *        - the identifier mask
     *        - the identifier
     *
     * arg = pointer to local filter variables (CPU_INT32U *)
     */
    IO_MSCAN_CAN_SET_RX_FILTER_2,

    /*! \brief             ENABLE LOOPBACK
     *
     *       This function code enables the loopback mode
     *
     */
    IO_MSCAN_CAN_ENABLE_LOOPBACK,

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
 * \ingroup  MSCAN
 *
 *           This structure holds the global data for the MSCAN module
 */
/*------------------------------------------------------------------------------------------------*/
typedef struct {
    /*! This member holds the base adress of the MSCAN module */
    void    *Base;


} MSCAN_PARA;

/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief                      DEVICE RUNTIME DATA
 * \ingroup  MSCAN
 *
 *           This structure holds the dynamic runtime data for a device.
 */
/*------------------------------------------------------------------------------------------------*/
typedef struct {
    /*! \brief             DEVICE STATUS
     *
     *        This member holds the device status:
     *        - bit 0: 0 = Device idle, 1 = Device busy
     *        - bit 1: 0 = non blocking mode, 1 = blocking mode
     *        - bit 2..7: not used (always 0)
     */
    CPU_INT08U Status;

    /*! This member holds the initializing state of the module */
    CPU_INT08U Initialized;

    /*! This member holds the baudrate. */
    CPU_INT32U Baudrate;

    /*! This member holds the base adress of the MSCAN module */
    void      *Base;

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

} MSCAN_DATA;


/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief                      CAN FRAME STRUCTURE
 * \ingroup  MSCAN
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
     *       - bit31: reserved (always 0)
     *       - bit30: marks a remote transmission request (1=rtr, 0=data frame)
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
} MSCAN_FRM;

/*
 ****************************************************************************************************
 *                                    COMPILE TIME CONFIGURATION
 ****************************************************************************************************
 */
#ifndef MSCAN_ARG_CHK_CFG
#define MSCAN_ARG_CHK_CFG          1           /* default: enable runtime argument checking  */
#elif MSCAN_ARG_CHK_CFG < 0 || MSCAN_ARG_CHK_CFG > 1
#error "MSCAN/drv_can.h: The configuration MSCAN_ARG_CHK_CFG must be 0 or 1"
#endif


/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief                      MSCAN CAN ENABLE ISR HANDLER
 * \ingroup  MSCAN
 *
 *           This field enables the CAN bus ISR handler for the uC/CAN layer. If this define is
 *           set to one, the code for the interrupt handler is included.
 *           Default: Include ISR handler
 */
/*------------------------------------------------------------------------------------------------*/
#ifndef MSCAN_EN_ISRH_CFG
#define MSCAN_EN_ISRH_CFG        0
#elif MSCAN_EN_ISRH_CFG < 0 || MSCAN_EN_ISRH_CFG > 1
#error "MSCAN/drv_can.h: The configuration MSCAN_EN_ISRH_CFG must be 0 or 1"
#endif

/*
 ****************************************************************************************************
 *                                       FUNCTION PROTOTYPES
 ****************************************************************************************************
 */
extern CPU_INT16S MSCANInit  (CPU_INT32U arg);
CPU_INT16S MSCANOpen  (CPU_INT16S devId, CPU_INT32U devName, CPU_INT16U mode);
CPU_INT16S MSCANClose (CPU_INT16S devId);
CPU_INT16S MSCANIoCtl (CPU_INT16S devId, CPU_INT16U func, void *arg);
CPU_INT16S MSCANRead  (CPU_INT16S devId, CPU_INT08U *buf, CPU_INT16U size);
CPU_INT16S MSCANWrite (CPU_INT16S devId, CPU_INT08U *buf, CPU_INT16U size);

CPU_INT16S MSCANCalcTimingReg(MSCAN_DATA *data);

#if MSCAN_EN_ISRH_CFG == 1
void MSCANIsrHandler  (void);
#endif

/*
 ****************************************************************************************************
 *                                          ERROR SECTION
 ****************************************************************************************************
 */

#if MSCAN_DISCARD_RTR_FRAMES < 0 || MSCAN_DISCARD_RTR_FRAMES > 1
#error "MSCAN/can_bsp.h: The configuration MSCAN_DISCARD_RTR_FRAMES must be 0 or 1"
#endif

#if MSCAN_FILTER_INIT_HOOK_EN < 0 || MSCAN_FILTER_INIT_HOOK_EN > 1
#error "MSCAN/can_bsp.h: The configuration MSCAN_FILTER_INIT_HOOK_EN must be 0 or 1"
#endif


#endif  /* #ifndef _DRV_CAN_H_ */

