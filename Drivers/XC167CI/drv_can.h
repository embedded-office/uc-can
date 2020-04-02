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
#include "cpu.h"                                /* CPU configuration definitions and constants    */
#include "can_bsp.h"                            /* User configurable definitions and constants    */
/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/

/*! \brief The unique driver name for installation and searching */
#define XC167_CAN_NAME "XC167:CAN Module"

#define XC167_CAN_NODE_B                ((CPU_INT16U) 0x0002)
#define XC167_CAN_NODE_A                ((CPU_INT16U) 0xFFFD)
#define XC167_CAN_MSG_RX_IRQ            ((CPU_INT16U) 0xFFF9)
#define XC167_CAN_MSG_TX_IRQ            ((CPU_INT16U) 0xFFEF)
#define XC167_CAN_RESET_MSG_IRQ         ((CPU_INT16U) 0xFFD7)
#define XC167_CAN_MSG_OBJ_VALID         ((CPU_INT16U) 0xFFBF)
#define XC167_CAN_NEWDAT                ((CPU_INT16U) 0x0200)
#define XC167_CAN_NEWDAT_MASK           ((CPU_INT16U) 0x0300)
#define XC167_CAN_DIR_MASK              ((CPU_INT16U) 0x0008)
#define XC167_CAN_RX_DIR                ((CPU_INT16U) 0x0000)
#define XC167_CAN_TXRQ_MASK             ((CPU_INT16U) 0x3000)
#define XC167_CAN_TX_BUSY               ((CPU_INT16U) 0x1000)
#define XC167_CAN_TX_DIR                ((CPU_INT16U) 0x0008)
#define XC167_CAN_START_TX_PROG         ((CPU_INT16U) 0xD9FF)
#define XC167_CAN_END_TX_PROG           ((CPU_INT16U) 0xE6FF)
#define XC167_CAN_MSG_TX                ((CPU_INT16U) 0x0008)
#define XC167_CAN_MSG_RX                ((CPU_INT16U) 0xFFF7)
#define XC167_CAN_XTD_ID                ((CPU_INT16U) 0x0004)
#define XC167_CAN_DLC_MASK              ((CPU_INT16U) 0x00F0)

#define XC167_CAN_MSGO_VALID      0x5595      /*!< Message Control Register, Msg. Object valid  */
#define XC167_CAN_MSGO_RXIE       0x5599
#define XC167_CAN_MSGO_TXIE       0x55A5

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN DEVICES
* \ingroup  XC167_CAN
*
*           This list holds the device names and the maximal number of supported physical CAN
*           devices available within the Infineon XC167-16 derivatives.
*/
/*------------------------------------------------------------------------------------------------*/
enum {
    XC167_CAN_BUS_0,                                /*!< Internal can controller #0             */
    XC167_CAN_BUS_1,                                /*!< Internal can controller #1             */
    XC167_CAN_N_DEV                                 /*!< Number of used can controller          */
};


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN STATUS
* \ingroup  XC167_CAN
*
*           This constant holds the use status value of the CAN device.
*/
/*------------------------------------------------------------------------------------------------*/
enum {
    /*! \brief                CAN IDLE
     *
     *      This value indicates that the RS232 device is idle.
     */
    XC167_CAN_IDLE = 0,

    /*! \brief                CAN BUSY
    *
    *       This value indicates that the RS232 device is busy (opened).
    */
    XC167_CAN_BUSY
};


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ERRORCODES
* \ingroup  XC167_CAN
*
*           The global errorcode variable 'DrvError' is supported with the following errorcodes.
*/
/*------------------------------------------------------------------------------------------------*/
enum {
    /*! \brief             NO ERROR
     *
     *       This code is used, if everything is ok.
     */
    XC167_CAN_NO_ERR = 0,

    /*! \brief             BUS ERROR
     *
     *       This code indicates, that a wrong bus was chosen..
     */
    XC167_CAN_BUS_ERR,

    /*! \brief             BUSY ERROR
     *
     *       This code indicates, that a a msg can not ne send because the bus is busy.
     */
    XC167_CAN_BUSY_ERR,

    /*! \brief             INIT ERROR
     *
     *       This code indicates, that the devices were not initialised because they are not
     *       in reset state.
     */
    XC167_CAN_INIT_ERR,

    /*! \brief             MODE ERROR
     *
     *       This code indicates, that the device cannot be accessed with the wanted mode.
     */
    XC167_CAN_MODE_ERR,

    /*! \brief             OPEN ERROR
     *
     *       This code indicates, that a device cannot be used, because it is not opened.
     */
    XC167_CAN_OPEN_ERR,

    /*! \brief             CLOSE ERROR
     *
     *       This code indicates, that the device cannot be closed.
     */
    XC167_CAN_CLOSE_ERR,

    /*! \brief             FUNCTION CODE ERROR
     *
     *       This code indicates, that the given function code is not valid.
     */
    XC167_CAN_FUNC_ERR,

    /*! \brief             ARGUMENT ERROR
     *
     *       This code indicates, that an argument check has failed.
     */
    XC167_CAN_ARG_ERR,

    /*! \brief             NO DATA ERROR
     *
     *       This code indicates, that no data is available.
     */
    XC167_CAN_NO_DATA_ERR
};


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      I/O CONTROL FUNCTION CODES
* \ingroup  XC167_CAN
*
*           This enumeration defines the required functioncode values for the lowlevel
*           device driver function IoCtl().
*/
/*------------------------------------------------------------------------------------------------*/

enum {
    /*! \brief             GET DRIVER IDENT CODE
     *
     *       This standard function code gets the driver identification code.
     *
     * arg = pointer to local ident variable (CPU_INT32U *)
     */
    IO_XC167_CAN_GET_IDENT = 0x0,

    /*! \brief             GET DRIVER ERRORCODE
     *
     *       This standard function code gets the driver errorcode.
     *
     * arg = pointer to local errorcode variable (CPU_INT16U *)
     */
    IO_XC167_CAN_GET_ERRNO,

    /*! \brief             GET DRIVER NAME
     *
     *       This standard function code gets the (human readable) driver name.
     *
     * arg = pointer to local string variable (char *)
     */
    IO_XC167_CAN_GET_DRVNAME,

    /*! \brief             SET BUS BAUDRATE
     *
     *       This function code sets the bus baudrate.
     *
     * arg = pointer to local baudrate variable (CPU_INT32U *)
     */
    IO_XC167_CAN_SET_BAUDRATE,

    /*! \brief             ENABLE BUS
     *
     *       This enum value is the functioncode to start the CAN controller interface.
     *       Most common is to set the CAN controller in active mode.
     *
     * The parameter pointer is not used for this function.
     */
    IO_XC167_CAN_START,

    /*! \brief             DISABLE BUS
     *
     *       This enum value is the functioncode to stop the CAN controller interface.
     *       Most common is to set the CAN controller in passive mode.
     *
     * The parameter pointer is not used for this function.
     */
    IO_XC167_CAN_STOP,

    /*! \brief             SET RECEIVER TO STANDARD IDENTIFIER
     *
     *       This enum value is the functioncode to configure the CAN receiver to receive only
     *       CAN standard identifiers.
     *
     * The parameter pointer is not used for this function.
     */
    IO_XC167_CAN_RX_STANDARD,

    /*! \brief             SET RECEIVER TO EXTENDED IDENTIFIER
     *
     *       This enum value is the functioncode to configure the CAN receiver to receive only
     *       CAN extended identifiers.
     *
     * The parameter pointer is not used for this function.
     */
    IO_XC167_CAN_RX_EXTENDED,

    /*! \brief Get TX Buffer Status
     *
     * This enum value is the functioncode to get the status of the current transmit
     * buffer.
     *
     * The parameter pointer shall point to a CPU_INT08U variable, where the status
     * shall be written to.
     */
    IO_XC167_CAN_TX_READY,

    /*! \brief             GET NODE STATUS
     *
     *       This enum value is the functioncode to get the node status from the
     *       CAN controller.
     *
     * The parameter pointer shall point to a CPU_INT32U variable, where the status
     * shall be written to.
     */
    IO_XC167_CAN_GET_NODE_STATUS,

    /*! \brief             SET RX ID FILTER
     *
     *       This function code sets the first identifier RX filter
     *       The arguments are:
     *        - the identifier mask
     *        - the identifier
     *
     * arg = pointer to local filter variables (CPU_INT32U *)
     */
    IO_XC167_CAN_SET_RX_FILTER,

    /*! \brief             NUMBER OF NEEDED IO FUNCTION CODES
     *
     *       This enum value holds the number of function codes, which are used within the
     *       CAN bus layer.
     */
    IO_XC167_CAN_IOCTL_MAX
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
* \brief                      DYNAMIC CAN DRIVER DATA
* \ingroup  XC167_CAN
*
*           This structure holds the dynamic CAN driver data.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct
{
  /*! \brief Use Marker
   *
   * This member holds a marker which indicates, that this device is in use:
   * 0 = Device idle,
   * 1 = Device in use
   */
  CPU_INT08U Use;

} XC167_CAN_DATA;


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      STATIC CAN BAUDRATE LIST
* \ingroup  XC167_CAN
*
*           This structure holds the static CAN driver baudrate list.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct
{
  /*! \brief Baudrate in bit/s */
  CPU_INT32U Baudrate;

  /*! \brief Corresponding Bit-Timing Register */
  CPU_INT32U BTR;

} XC167_CAN_BAUD;


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN FRAME
* \ingroup  XC167_CAN
*
*           This structure contains all needed data to handle a single CAN frame
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct {
    /*!
    * \brief                  CAN IDENTIFIER
    *
    *       This member holds the CAN identifier.
    *
    */
    CPU_INT32U Identifier;

    /*!
    * \brief                  CAN PAYLOAD
    *
    *       This member holds up to 8 bytes, which can be handled with a single CAN message.
    */
    CPU_INT08U Data[8];

    /*!
    * \brief                  CAN DLC
    *
    *       This member holds the number of valid datas in the payload.
    */
    CPU_INT08U DLC;

     /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief                  SPARE
    *
    *       These bytes are added to get a frame size of an integral number of pointers.
    */
    /*--------------------------------------------------------------------------------------------*/
    CPU_INT08U Spare[3];

} XC167_CANFRM;


/*
****************************************************************************************************
*                                    COMPILE TIME CONFIGURATION
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      XC167 CAN RX INTERRUPT ENABLE
* \ingroup  XC167_CAN
*
*           This define enables resp. disable the CAN Rx Interrupt.
*           Default value is 1 (enable irq).
*/
/*------------------------------------------------------------------------------------------------*/
#ifndef XC167_CAN_RX_INTERRUPT_EN
#define XC167_CAN_RX_INTERRUPT_EN 1
#elif XC167_CAN_RX_INTERRUPT_EN < 0 || XC167_CAN_RX_INTERRUPT_EN > 1
#error "xc167ci/drv_can.h: The configuration XC167_CAN_RX_INTERRUPT_EN must be 0 or 1"
#endif

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      XC167 CAN TX INTERRUPT ENABLE
* \ingroup  XC167_CAN
*
*           This define enables resp. disable the CAN Tx Interrupt.
*           Default value is 1 (enable irq).
*/
/*------------------------------------------------------------------------------------------------*/
#ifndef XC167_CAN_TX_INTERRUPT_EN
#define XC167_CAN_TX_INTERRUPT_EN 1
#elif XC167_CAN_TX_INTERRUPT_EN < 0 || XC167_CAN_TX_INTERRUPT_EN > 1
#error "xc167ci/drv_can.h: The configuration XC167_CAN_TX_INTERRUPT_EN must be 0 or 1"
#endif

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      XC167 CAN NS INTERRUPT ENABLE
* \ingroup  XC167_CAN
*
*           This define enables resp. disable the CAN Ns Interrupt.
*           Default value is 1 (enable irq).
*/
/*------------------------------------------------------------------------------------------------*/
#ifndef XC167_CAN_NS_INTERRUPT_EN
#define XC167_CAN_NS_INTERRUPT_EN 1
#elif XC167_CAN_NS_INTERRUPT_EN < 0 || XC167_CAN_NS_INTERRUPT_EN > 1
#error "xc167ci/drv_can.h: The configuration XC167_CAN_NS_INTERRUPT_EN must be 0 or 1"
#endif

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      XC167 CAN ARGUMENT CHECK CONFIGURATION
* \ingroup  XC167_CAN
*
*           This define enables resp. disable the runtime argument checking.
*           Default value is 1 (enable check).
*/
/*------------------------------------------------------------------------------------------------*/
#ifndef XC167_CAN_ARG_CHK_CFG
#define XC167_CAN_ARG_CHK_CFG     1
#elif XC167_CAN_ARG_CHK_CFG < 0 || XC167_CAN_ARG_CHK_CFG > 1
#error "xc167ci/drv_can.h: The configuration XC167_CAN_ARG_CHK_CFG must be 0 or 1"
#endif

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      XC167 CAN RX1 INTERRUPT CONFIGURATION
* \ingroup  XC167_CAN
*
*           This define enables resp. disable the Rx1 interrupt service routine.
*           Default value is 1 (enabled).
*/
/*------------------------------------------------------------------------------------------------*/
#ifndef XC167_CAN_RX1_ISR_CFG
#define XC167_CAN_RX1_ISR_CFG                 1
#elif XC167_CAN_RX1_ISR_CFG < 0 || XC167_CAN_RX1_ISR_CFG > 1
#error "xc167ci/drv_can.h: The configuration XC167_CAN_RX1_ISR_CFG must be 0 or 1"
#endif

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      XC167 CAN RX2 INTERRUPT CONFIGURATION
* \ingroup  XC167_CAN
*
*           This define enables resp. disable the Rx2 interrupt service routine.
*           Default value is 1 (enabled).
*/
/*------------------------------------------------------------------------------------------------*/
#ifndef XC167_CAN_RX2_ISR_CFG
#define XC167_CAN_RX2_ISR_CFG                 1
#elif XC167_CAN_RX2_ISR_CFG < 0 || XC167_CAN_RX2_ISR_CFG > 1
#error "xc167ci/drv_can.h: The configuration XC167_CAN_RX2_ISR_CFG must be 0 or 1"
#endif


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      XC167 CAN TX1 INTERRUPT CONFIGURATION
* \ingroup  XC167_CAN
*
*           This define enables resp. disable the Tx1 interrupt service routine.
*           Default value is 1 (enabled).
*/
/*------------------------------------------------------------------------------------------------*/
#ifndef XC167_CAN_TX1_ISR_CFG
#define XC167_CAN_TX1_ISR_CFG                 1
#elif XC167_CAN_TX1_ISR_CFG < 0 || XC167_CAN_TX1_ISR_CFG > 1
#error "xc167ci/drv_can.h: The configuration XC167_CAN_TX1_ISR_CFG must be 0 or 1"
#endif

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      XC167 CAN TX2 INTERRUPT CONFIGURATION
* \ingroup  XC167_CAN
*
*           This define enables resp. disable the Tx2 interrupt service routine.
*           Default value is 1 (enabled).
*/
/*------------------------------------------------------------------------------------------------*/
#ifndef XC167_CAN_TX2_ISR_CFG
#define XC167_CAN_TX2_ISR_CFG                 1
#elif XC167_CAN_TX2_ISR_CFG < 0 || XC167_CAN_TX2_ISR_CFG > 1
#error "xc167ci/drv_can.h: The configuration XC167_CAN_TX2_ISR_CFG must be 0 or 1"
#endif

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      XC167 CAN NS INTERRUPT CONFIGURATION
* \ingroup  XC167_CAN
*
*           This define enables resp. disable the Tx2 interrupt service routine.
*           Default value is 1 (enabled).
*/
/*------------------------------------------------------------------------------------------------*/
#ifndef XC167_CAN_NS_ISR_CFG
#define XC167_CAN_NS_ISR_CFG                  1
#elif XC167_CAN_NS_ISR_CFG < 0 || XC167_CAN_NS_ISR_CFG > 1
#error "xc167ci/drv_can.h: The configuration XC167_CAN_NS_ISR_CFG must be 0 or 1"
#endif

/*
****************************************************************************************************
*                                       FUNCTION PROTOTYPES
****************************************************************************************************
*/
CPU_INT16S XC167CANInit  (CPU_INT32U arg);
CPU_INT16S XC167CANOpen  (CPU_INT16S drvId, CPU_INT32U devName, CPU_INT16U mode);
CPU_INT16S XC167CANClose (CPU_INT16S devId);
CPU_INT16S XC167CANIoCtl (CPU_INT16S devId, CPU_INT16U func, void *argp);
CPU_INT16S XC167CANRead  (CPU_INT16S devId, CPU_INT08U *buf, CPU_INT16U size);
CPU_INT16S XC167CANWrite (CPU_INT16S devId, CPU_INT08U *buf, CPU_INT16U size);

CPU_INT16S XC167CANIoCtlGetIdent      (CPU_INT16S devId, void *argp);
CPU_INT16S XC167CANIoCtlGetErrno      (CPU_INT16S devId, void *argp);
CPU_INT16S XC167CANIoCtlGetDrvName    (CPU_INT16S devId, void *argp);
CPU_INT16S XC167CANIoCtlSetBaudrate   (CPU_INT16S devId, void *argp);
CPU_INT16S XC167CANIoCtlStart         (CPU_INT16S devId, void *argp);
CPU_INT16S XC167CANIoCtlStop          (CPU_INT16S devId, void *argp);
CPU_INT16S XC167CANIoCtlRxStandard    (CPU_INT16S devId, void *argp);
CPU_INT16S XC167CANIoCtlRxExtended    (CPU_INT16S devId, void *argp);
CPU_INT16S XC167CANIoCtlGetNodeStatus (CPU_INT16S devId, void *argp);
CPU_INT16S XC167CANIoCtlSetRxFilter   (CPU_INT16S devId, void *argp);
CPU_INT16S XC167CANIoCtlGetTxReadyStatus (CPU_INT16S devId, void *argp);
CPU_INT16S XC167CANIoCtlNotSupported  (CPU_INT16S devId, void *argp);


/*
****************************************************************************************************
*                                          ERROR SECTION
****************************************************************************************************
*/


#endif  /* #ifndef _DRV_CAN_H_ */

