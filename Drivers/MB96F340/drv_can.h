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
#include "can_bsp.h"                                  /* CAN board support package                */
#include "drv_can_reg.h"                              /* MB96F340 CAN register definitions        */


/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      IO CONTROL FUNCTION CODES
* \ingroup  MB96F340_CAN
*
*           This enumeration specifies the function codes for the CAN device driver.
*/
/*------------------------------------------------------------------------------------------------*/
enum MB96F340_CAN_IO_FUNC {

    /*! \brief Get Driver Ident Code
     *
     *       This standard function code gets the driver identification code.
     *
     * arg = pointer to local ident variable (CPU_INT32U *)
     */
    IO_MB96F340_CAN_GET_IDENT    = 0x0,
    /*! \brief Get Driver Error Code
     *
     *       This standard function code gets the driver error code.
     *
     * arg = pointer to local ident variable (CPU_INT16U *)
     */
    IO_MB96F340_CAN_GET_ERRNO    = 0x1,
    /*! \brief Get Driver Name
     *
     *       This standard function code gets the driver name.
     *
     * arg = pointer to local driver name (char *)
     */
    IO_MB96F340_CAN_GET_DRVNAME  = 0x2,
    /*! \brief Set Baudrate
     *
     * This enum value is the functioncode to set the baudrate of the CAN controller interface.
     *
     * The parameter pointer shall point to an CPU_INT32U variable, which holds the baudrate
     * in bit/s.
     */
    IO_MB96F340_CAN_SET_BAUDRATE = 0x10,
    /*! \brief Enable Bus
     *
     * This enum value is the functioncode to start the CAN controller interface. Most common
     * is to set the CAN controller in active mode.
     *
     * The parameter pointer is not used for this function.
     */
    IO_MB96F340_CAN_START,
    /*! \brief Disable Bus
     *
     * This enum value is the functioncode to stop the CAN controller interface. Most common
     * is to set the CAN controller in passive mode.
     *
     * The parameter pointer is not used for this function.
     */
    IO_MB96F340_CAN_STOP,

    /*! \brief Set Receiver to Standard Identifier
     *
     * This enum value is the functioncode to configure the CAN receiver to receive only
     * CAN standard identifiers.
     *
     * The parameter pointer is not used for this function.
     */
    IO_MB96F340_CAN_RX_STANDARD,
    /*! \brief Set Receiver to Extended Identifier
     *
     * This enum value is the functioncode to configure the CAN receiver to receive only
     * CAN extended identifiers.
     *
     * The parameter pointer is not used for this function.
     */
    IO_MB96F340_CAN_RX_EXTENDED,

    /*! \brief Get Node Status
     *
     * This enum value is the functioncode to get the node status from the
     * CAN controller.
     *
     * The parameter pointer shall point to a CPU_INT08U variable, where the status
     * shall be written to.
     */
    IO_MB96F340_CAN_GET_NODE_STATUS,
    /*! \brief Get TX Buffer Status
     *
     * This enum value is the functioncode to get the status of the current transmit
     * buffer.
     *
     * The parameter pointer shall point to a CPU_INT08U variable, where the status
     * shall be written to.
     */
    IO_MB96F340_CAN_TX_READY,
    /*! \brief Number of Needed IO Function Codes
     *
     * This enum value holds the number of function codes, which are used within the
     * can bus layer.
     */
    IO_MB96F340_CAN_IO_FUNC_N
};


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ERRORCODES
* \ingroup  MB96F340_CAN
*
*           The global errorcode variable 'DrvError' is supported with the following errorcodes.
*/
/*------------------------------------------------------------------------------------------------*/
enum {
    /*! \brief             NO ERROR
     *
     *       This code is used, if everything is ok.
     */
    MB96F340_CAN_NO_ERR = 0,

    /*! \brief             BUS ERROR
     *
     *       This code indicates, that a wrong bus was chosen.
     */
    MB96F340_CAN_BUS_ERR,

    /*! \brief             BUSY ERROR
     *
     *       This code indicates, that a msg could not be sent, because the bus is busy.
     */
    MB96F340_CAN_BUSY_ERR,

    /*! \brief             INIT ERROR
     *
     *       This code indicates, that the devices were not initialised because they are not
     *       in reset state.
     */
    MB96F340_CAN_INIT_ERR,

    /*! \brief             MODE ERROR
     *
     *       This code indicates, that the device cannot be accessed with the wanted mode.
     */
    MB96F340_CAN_MODE_ERR,

    /*! \brief             OPEN ERROR
     *
     *       This code indicates, that a device cannot be used, because it is not opened.
     */
    MB96F340_CAN_OPEN_ERR,

    /*! \brief             CLOSE ERROR
     *
     *       This code indicates, that a device cannot be closed.
     */
    MB96F340_CAN_CLOSE_ERR,

    /*! \brief             FUNCTION CODE ERROR
     *
     *       This code indicates, that the given function code is not valid.
     */
    MB96F340_CAN_FUNC_ERR,

    /*! \brief             ARGUMENT ERROR
     *
     *       This code indicates, that a runtime parameter check fails.
     */
    MB96F340_CAN_ARG_ERR,

    /*! \brief             NO DATA ERROR
     *
     *       This code indicates, that no data is available.
     */
    MB96F340_CAN_NO_DATA_ERR
};


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      STATUS: DEVICE BUSY
* \ingroup  MB96F340_CAN
*
*           This define holds the mask for the device busy flag.
*/
/*------------------------------------------------------------------------------------------------*/
#define MB96F340_CAN_BUSY  0x01

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      BUS STATUS ERROR ACTIVE
* \ingroup  MB96F340_CAN
*
*           This Constant holds the value for Bus Status Error Active.
*/
/*------------------------------------------------------------------------------------------------*/
#define MB96F340_CAN_ERROR_ACTIVE      0x00

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      BUS STATUS ERROR PASSIVE
* \ingroup  MB96F340_CAN
*
*           This Constant holds the value for Bus Status Error Passive.
*/
/*------------------------------------------------------------------------------------------------*/
#define MB96F340_CAN_ERROR_PASSIVE     0x01

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      BUS STATUS ERROR BUS OFF
* \ingroup  MB96F340_CAN
*
*           This Constant holds the value for Bus Status Error Bus Off.
*/
/*------------------------------------------------------------------------------------------------*/
#define MB96F340_CAN_ERROR_BUS_OFF     0x02

/*
****************************************************************************************************
*                                            DATA TYPES
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN RUNTIME DATA
* \ingroup  MB96F340_CAN
*
*           This structure holds the CAN device driver runtime data.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct {
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief                  STATUS
    *
    *       This member holds the status of the CAN device. The encoding of the status is:
    *       - Bit 0: 0 = device idle; 1 = device busy (e.g. opened device)
    *       - Bit 1..7: reserved
    */
    /*--------------------------------------------------------------------------------------------*/
    CPU_INT08U Use;

} MB96F340_CAN_DATA;


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN CONFIGURATION
* \ingroup  MB96F340_CAN
*
*           This structure holds the (static) CAN configuration.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct {
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief                  CAN REGISTERS
    *
    *       This member holds a pointer of the CAN registers.
    */
    /*--------------------------------------------------------------------------------------------*/
    CAN_REG       *CanReg;

} MB96F340_CAN_PARA;


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN BAUDRATE TABLE
* \ingroup  MB96F340_CAN
*
*           This structure holds all informations to the standard baudrates.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct {
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief                  CAN BAUDRATE
    *
    *       This member holds the CAN bus baudrate in bit/s.
    */
    /*--------------------------------------------------------------------------------------------*/
    CPU_INT32U Baudrate;
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief                  CAN BIT TIMING
    *
    *       This member holds the bit timing, which realizes the corresponding baudrate (see member
    *       Baudrate).
    *
    * \note The encoding of the bit-timing is equal to the encoding of the BTR register.
    */
    /*--------------------------------------------------------------------------------------------*/
    CPU_INT16U BTRVal;

} MB96F340_CAN_BAUD;


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN FRAME
* \ingroup  MB96F340_CAN
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
    * \note To differentiate standard and extended identifiers the following addition to the
    *       identifier is implemented:
    *       - bit31-30: reserved (always 0)
    *       - bit29: marks an extended identifier (1=extended, 0=standard)
    *       - bit28-0: the identifier (standard or extended)
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

} MB96F340_CAN_FRM;


/*
****************************************************************************************************
*                                          ERROR SECTION
****************************************************************************************************
*/

#ifndef MB96F340_CAN_SELECT_CFG
#define MB96F340_CAN_DEV_N  5
#elif MB96F340_CAN_SELECT_CFG < 0 || MB96F340_CAN_SELECT_CFG > 4
#error "MB96F340/drv_can.h: MB96F340_CAN_SELECT_CFG must be in range 0 .. 4"
#else
#define MB96F340_CAN_DEV_N  1
#endif

#ifndef MB96F340_CAN_IRQ_LVL_CFG
#define MB96F340_CAN_IRQ_LVL_CFG 4
#elif MB96F340_CAN_IRQ_LVL_CFG < 0 || MB96F340_CAN_IRQ_LVL_CFG > 7
#error "MB96F340/drv_can.h: MB96F340_CAN_IRQ_LVL_CFG must be in range 0 .. 7"
#endif

#ifndef MB96F340_CAN_ARG_CHK_CFG
#define MB96F340_CAN_ARG_CHK_CFG 1
#elif MB96F340_CAN_ARG_CHK_CFG < 0 || MB96F340_CAN_ARG_CHK_CFG > 1
#error "MB96F340/drv_can.h: MB96F340_CAN_ARG_CHK_CFG must be 0 or 1"
#endif

#if MB96F340_CAN_TX_INTERRUPT_EN != 0
#error "MB96F340/drv_can.h: MB96F340_CAN_TX_INTERRUPT_EN must be 0"
#endif

/*
****************************************************************************************************
*                                       FUNCTION PROTOTYPES
****************************************************************************************************
*/

CPU_INT16S MB96F340CANInit (CPU_INT32U arg);
CPU_INT16S MB96F340CANOpen (CPU_INT16S drvId, CPU_INT32U devName, CPU_INT16U mode);
CPU_INT16S MB96F340CANClose(CPU_INT16S devId);
CPU_INT16S MB96F340CANIoCtl(CPU_INT16S devId, CPU_INT16U func,    void *argp);
CPU_INT16S MB96F340CANRead (CPU_INT16S devId, CPU_INT08U *buffer, CPU_INT16U size);
CPU_INT16S MB96F340CANWrite(CPU_INT16S devId, CPU_INT08U *buffer, CPU_INT16U size);


#endif  /* #ifndef _DRV_CAN_H_ */

