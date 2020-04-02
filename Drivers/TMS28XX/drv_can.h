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

#ifndef _TMS28XX_DRV_CAN_H_
#define _TMS28XX_DRV_CAN_H_

/*
****************************************************************************************************
*                                             INCLUDES
****************************************************************************************************
*/
#include "drv_def.h"                                  /* Driver layer definitions and constants   */
#include "cpu.h"                                      /* Basic Type definitions                   */
#include "can_bsp.h"                                  /* CAN board support package                */

/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ECANA SYSTEMCLOCK
* \ingroup  TMS28XX_ECAN
*
*           The system clock frequency for ECANA [Hz].
*/
/*------------------------------------------------------------------------------------------------*/
#ifndef TMS28XX_ECANA_CLK
#define TMS28XX_ECANA_CLK            150000000
#endif

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ECANB SYSTEMCLOCK
* \ingroup  TMS28XX_ECAN
*
*           The system clock frequency for ECANB [Hz].
*/
/*------------------------------------------------------------------------------------------------*/
#ifndef TMS28XX_ECANB_CLK
#define TMS28XX_ECANB_CLK            150000000
#endif

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DEFAULT BAUDRATE
* \ingroup  TMS28XX_ECAN
*
*           Default baudrate of the eCANA module.
*/
/*------------------------------------------------------------------------------------------------*/
#ifndef TMS28XX_ECANA_DEF_BAUDRATE
#define TMS28XX_ECANA_DEF_BAUDRATE      1000000L
#endif

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DEFAULT BAUDRATE
* \ingroup  TMS28XX_ECAN
*
*           Default baudrate of the eCANB module.
*/
/*------------------------------------------------------------------------------------------------*/
#ifndef TMS28XX_ECANB_DEF_BAUDRATE
#define TMS28XX_ECANB_DEF_BAUDRATE      1000000L
#endif

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DEFAULT TSEG1
* \ingroup  TMS28XX_ECAN
*
*           Default Time Segment 1 for eCANA.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECANA_TSEG1REG 9

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DEFAULT TSEG1
* \ingroup  TMS28XX_ECAN
*
*           Default Time Segment 1 for eCANB.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECANB_TSEG1REG 9

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DEFAULT TSEG2
* \ingroup  TMS28XX_ECAN
*
*           Default Time Segment 2 for eCANA.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECANA_TSEG2REG 3

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DEFAULT TSEG2
* \ingroup  TMS28XX_ECAN
*
*           Default Time Segment 2 for eCANB.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECANB_TSEG2REG 3

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DEFAULT SJW
* \ingroup  TMS28XX_ECAN
*
*           Default Synchronization jump width for eCANA.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECANA_SJWREG   2

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DEFAULT SJW
* \ingroup  TMS28XX_ECAN
*
*           Default Synchronization jump width for eCANB.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECANB_SJWREG   2

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      STANDARD ID MASK
* \ingroup  TMS28XX_ECAN
*
*           Max. possible Identifier in standard CAN frames.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECAN_STD_ID_MASK       0x000007FF

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      EXTENDED ID MASK
* \ingroup  TMS28XX_ECAN
*
*           Max. possible Identifier in extended CAN frames.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECAN_EXT_ID_MASK       0x1FFFFFFF

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      EXTENDED ID FLAG
* \ingroup  TMS28XX_ECAN
*
*           This Flag must be set in the CAN Identifier if a standard CAN ID shall be send as a
*           extended message frame.
*           In a received CAN message frame this flag indicates a extended Identifier.
*
* \see      TMS28XX_ECAN_FRM
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECAN_EXT_ID_FLAG       0x20000000

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER NAME
* \ingroup  TMS28XX_ECAN
*
*           The unique driver name for installation and searching in the common driver interface.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECAN_NAME              "TMS28XX:eCAN Module:V2.4.0"

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DEVICES
* \ingroup  TMS28XX_ECAN
*
*           This list holds the device names and the maximal number of supported physical can
*           controllers within the TMS28XX eCAN controller derivat.
*/
/*------------------------------------------------------------------------------------------------*/
enum TMS28XX_ECAN_DEVNAME {

    TMS28XX_ECAN_BUS_A = 0,                                 /*!< eCAN Bus Module A                */
#ifdef TMS320F280X
    TMS28XX_ECAN_BUS_B,                                     /*!< eCAN Bus Module B                */
#endif
    TMS28XX_ECAN_DEV_N                                      /*!> Number of Devices                */
};

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      STATUS: CAN BUS IDLE
* \ingroup  TMS28XX_ECAN
*
*           This constant holds the status value, which is set when the CAN bus is idle.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECAN_IDLE              0x00

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      STATUS: CAN BUS BUSY
* \ingroup  TMS28XX_ECAN
*
*           This constant holds the status value, which is set when the CAN bus is busy.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECAN_BUSY              0x01

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      STATUS: NON BLOCKING ACCESS
* \ingroup  TMS28XX_ECAN
*
*           This constant holds the status value, which is set when the driver works in non
*           blocking access mode.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECAN_NON_BLOCKING      0x00

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      STATUS: BLOCKING ACCESS
* \ingroup  TMS28XX_ECAN
*
*           This constant holds the status value, which is set when the driver works in blocking
*           access mode.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECAN_BLOCKING          0x02

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      BUS STATUS ERROR ACTIVE
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the value for Bus Status Error Active.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECAN_ERROR_ACTIVE      0x00

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      BUS STATUS ERROR PASSIVE
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the value for Bus Status Error Passive.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECAN_ERROR_PASSIVE     0x01

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      BUS STATUS ERROR BUS OFF
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the value for Bus Status Error Bus Off.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECAN_ERROR_BUS_OFF     0x02

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                          I/O CONTROL FUNCTIONCODES
* \ingroup  TMS28XX_ECAN
*
*           This enumeration defines the required functioncode values for the lowlevel
*           device driver function IoCtl().
*/
/*------------------------------------------------------------------------------------------------*/
enum TMS28XX_ECAN_IOCTL_FUNC {
    /*! \brief Get Driver Ident Code
     *
     *       This standard function code gets the driver identification code.
     *
     * arg = pointer to local ident variable (CPU_INT32U *)
     */
    IO_TMS28XX_CAN_GET_IDENT    = 0x00,
    /*! \brief Get Driver Error Code
     *
     *       This standard function code gets the driver error code.
     *
     * arg = pointer to local ident variable (CPU_INT16U *)
     */
    IO_TMS28XX_CAN_GET_ERRNO    = 0x01,
    /*! \brief Get Driver Name
     *
     *       This standard function code gets the driver name.
     *
     * arg = pointer to local driver name (char *)
     */
    IO_TMS28XX_CAN_GET_DRVNAME  = 0x02,
    /*! \brief Set Baudrate
     *
     * This enum value is the functioncode to set the baudrate of the CAN controller interface.
     *
     * The parameter pointer shall point to an CPU_INT32U variable, which holds the baudrate
     * in bit/s.
     */
    IO_TMS28XX_CAN_SET_BAUDRATE = 0x10,
    /*! \brief Enable Bus
     *
     * This enum value is the functioncode to start the CAN controller interface. Most common
     * is to set the CAN controller in active mode.
     *
     * The parameter pointer is not used for this function.
     */
    IO_TMS28XX_CAN_START,
    /*! \brief Disable Bus
     *
     * This enum value is the functioncode to stop the CAN controller interface. Most common
     * is to set the CAN controller in passive mode.
     *
     * The parameter pointer is not used for this function.
     */
    IO_TMS28XX_CAN_STOP,
    /*! \brief Set Receiver to Standard Identifier
     *
     * This enum value is the functioncode to configure the CAN receiver to receive only
     * CAN standard identifiers.
     *
     * The parameter pointer is not used for this function.
     */
    IO_TMS28XX_CAN_RX_STANDARD,
    /*! \brief Set Receiver to Extended Identifier
     *
     * This enum value is the functioncode to configure the CAN receiver to receive only
     * CAN extended identifiers.
     *
     * The parameter pointer is not used for this function.
     */
    IO_TMS28XX_CAN_RX_EXTENDED,
    /*! \brief Get Node Status
     *
     * This enum value is the functioncode to get the node status from the
     * CAN controller.
     *
     * The parameter pointer shall point to a CPU_INT08U variable, where the status
     * shall be written to.
     */
    IO_TMS28XX_CAN_GET_NODE_STATUS,
    /*! \brief Get TX Buffer Status
     *
     * This enum value is the functioncode to get the status of the current transmit
     * buffer.
     *
     * The parameter pointer shall point to a CPU_INT08U variable, where the status
     * shall be written to.
     */
    IO_TMS28XX_CAN_TX_READY,
    /*! \brief Number of Needed IO Function Codes
     *
     * This enum value holds the number of function codes, which are used within the
     * can bus layer.
     */
    IO_TMS28XX_CAN_IO_FUNC_N
};

/*
****************************************************************************************************
*                                            DATA TYPES
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DEVICE RUNTIME DATA
* \ingroup  TMS28XX_ECAN
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

} TMS28XX_ECAN_DATA;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DEVICE PARAMETER
* \ingroup  TMS28XX_ECAN
*
*           This structure holds the global data for the eCAN module
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct {
    /*! This member holds the base address of the eCAN Register module */
    CPU_INT32U ECANBase;

    /*! This member holds the base address of the Local Acceptance Mask Register module */
    CPU_INT32U LAMBase;

    /*! This member holds the base address of the Mailbox Register module */
    CPU_INT32U MBoxBase;

    /*! This member holds the CAN Clock. */
    CPU_INT32U Clock;

    /*! This member holds the default baudrate. */
    CPU_INT32U Baudrate;

    /*! This member holds the Synchronization jump width. */
    CPU_INT08U SJW;

    /*! This member holds the Time segment 1. */
    CPU_INT08U TSEG1;

    /*! This member holds the Time Segment 2. */
    CPU_INT08U TSEG2;

} TMS28XX_ECAN_PARA;


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN FRAME STRUCTURE
* \ingroup  TMS28XX_ECAN
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

    /*!
    * \brief                  SPARE
    *
    *       These bytes are added to get a frame size of an integral number of pointers.
    */
    CPU_INT08U Spare[3];

} TMS28XX_ECAN_FRM;

/*
****************************************************************************************************
*                                    COMPILE TIME CONFIGURATION
****************************************************************************************************
*/
#ifndef TMS28XX_ECAN_ARG_CHK_CFG
#define TMS28XX_ECAN_ARG_CHK_CFG      1            /* default: enable runtime argument checking  */
#elif TMS28XX_ECAN_ARG_CHK_CFG < 0 || TMS28XX_ECAN_ARG_CHK_CFG > 1
#error "tms28xx/drv_can.h: The configuration TMS28XX_ECAN_ARG_CHK_CFG must be 0 or 1"
#endif

#ifndef TMS28XX_ECANA_CLK
#error "tms28xx/drv_can.h: The CANA system clock TMS28XX_ECANA_CLK must be defined."
#endif

#ifdef TMS320F280X
#ifndef TMS28XX_ECANB_CLK
#error "tms28xx/drv_can.h: The CANB system clock TMS28XX_ECANB_CLK must be defined."
#endif
#endif

#ifndef TMS28XX_ECAN_RX_INTERRUPT_EN
#define TMS28XX_ECAN_RX_INTERRUPT_EN  1               /* default: enable rx interrupt handling    */
#endif

#ifndef TMS28XX_ECAN_TX_INTERRUPT_EN
#define TMS28XX_ECAN_TX_INTERRUPT_EN  1               /* default: enable rx interrupt handling    */
#endif

#ifndef TMS28XX_ECAN_NS_INTERRUPT_EN
#define TMS28XX_ECAN_NS_INTERRUPT_EN  1               /* default: enable rx interrupt handling    */
#endif

/*
****************************************************************************************************
*                                       FUNCTION PROTOTYPES
****************************************************************************************************
*/
CPU_INT16S TMS28XXCANInit  (CPU_INT32U arg);
CPU_INT16S TMS28XXCANOpen  (CPU_INT16S devId, CPU_INT32U devName, CPU_INT16U mode);
CPU_INT16S TMS28XXCANClose (CPU_INT16S devId);
CPU_INT16S TMS28XXCANIoCtl (CPU_INT16S devId, CPU_INT16U func, void *arg);
CPU_INT16S TMS28XXCANRead  (CPU_INT16S devId, CPU_INT08U *buf, CPU_INT16U size);
CPU_INT16S TMS28XXCANWrite (CPU_INT16S devId, CPU_INT08U *buf, CPU_INT16U size);

/*
****************************************************************************************************
*                                          ERROR SECTION
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ERRORCODES
* \ingroup  TMS28XXECAN
*
*           The global errorcode variable 'DrvError' is supported with the following errorcodes.
*/
/*------------------------------------------------------------------------------------------------*/
enum {
    /*! \brief             NO ERROR
     *
     *       This code is used, if everything is ok.
     */
    TMS28XX_CAN_NO_ERR = 0,

    /*! \brief             BUS ERROR
     *
     *       This code indicates, that a wrong bus was chosen.
     */
    TMS28XX_CAN_BUS_ERR,

    /*! \brief             BUSY ERROR
     *
     *       This code indicates, that a msg could not be sent, because the bus is busy.
     */
    TMS28XX_CAN_BUSY_ERR,

    /*! \brief             INIT ERROR
     *
     *       This code indicates, that the devices were not initialised because they are not
     *       in reset state.
     */
    TMS28XX_CAN_INIT_ERR,

    /*! \brief             MODE ERROR
     *
     *       This code indicates, that the device cannot be accessed with the wanted mode.
     */
    TMS28XX_CAN_MODE_ERR,

    /*! \brief             OPEN ERROR
     *
     *       This code indicates, that a device cannot be used, because it is not opened.
     */
    TMS28XX_CAN_OPEN_ERR,

    /*! \brief             CLOSE ERROR
     *
     *       This code indicates, that a device cannot be closed.
     */
    TMS28XX_CAN_CLOSE_ERR,

    /*! \brief             FUNCTION CODE ERROR
     *
     *       This code indicates, that the given function code is not valid.
     */
    TMS28XX_CAN_FUNC_ERR,

    /*! \brief             ARGUMENT ERROR
     *
     *       This code indicates, that a runtime parameter check fails.
     */
    TMS28XX_CAN_ARG_ERR,

    /*! \brief             NO DATA ERROR
     *
     *       This code indicates, that no data is available.
     */
    TMS28XX_CAN_NO_DATA_ERR
};


#endif  /* #ifndef _TMS28XX_DRV_CAN_H_ */
