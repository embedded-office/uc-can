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
#include "can_bsp.h"

/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/

/*! \brief The unique driver name for installation and searching */
#define STM32F20X_CAN_NAME "STM32F20X:CAN Module"

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief    DEFAULT BIT SAMPLE POINT
*
* \ingroup  STM32F20X_CAN
*
*           Default bit sample point in 1/10 percent.
*/
/*------------------------------------------------------------------------------------------------*/
#define STM32F20X_DEF_SP            750

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief    DEFAULT RE-SYNCHRONIZATION JUMP WIDTH
*
* \ingroup  STM32F20X_CAN
*
*           Default Re-synchronization Jump Width in 1/10 percent.
*/
/*------------------------------------------------------------------------------------------------*/
#define STM32F20X_DEF_RJW           125

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief    DEFAULT BAUDRATE
*
* \ingroup  STM32F20X_CAN
*
*           Default baudrate of the CAN modules.
*/
/*------------------------------------------------------------------------------------------------*/
#define STM32F20X_DEF_BAUDRATE      1000000L

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief    EXTENDED ID FLAG
*
* \ingroup  STM32F20X_CAN
*
*           following bit definitions as defined in can_frame.h
*           - bit30: marks a remote transmission request (1=rtr, 0=data frame)
*           - bit29: marks an extended identifier (1=extended, 0=standard)
*/
/*------------------------------------------------------------------------------------------------*/
#define STM32F20X_FF_FRAME_BIT    0x20000000
#define STM32F20X_RTR_FRAME_BIT   0x40000000

/*
****************************************************************************************************
*                                            DATA TYPES
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief    DEVICE NAMES
*
* \ingroup  STM32F20X_CAN
*
*           This enum defines the available device names for the driver XXXCANInit() function.
*/
/*------------------------------------------------------------------------------------------------*/
enum {
    STM32F20X_CAN_BUS_0,                              /*!< Internal CAN controller #0             */
    STM32F20X_CAN_BUS_1,                              /*!< Internal CAN controller #1             */
    STM32F20X_CAN_N_DEV                               /*!< Number of CAN controllers              */
};

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief    DRIVER ERROR CODES
*
* \ingroup  STM32F20X_CAN
*
*           This enum defines the possible driver error codes.
*/
/*------------------------------------------------------------------------------------------------*/
enum {
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    NO ERROR
    *
    *           This code is used if everything is OK.
    */
    /*--------------------------------------------------------------------------------------------*/
    STM32F20X_CAN_NO_ERR = 0,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    BUS ERROR
    *
    *           This code indicates, that a wrong bus was chosen.
    */
    /*--------------------------------------------------------------------------------------------*/
    STM32F20X_CAN_BUS_ERR,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    BUSY ERROR
    *
    *           This code indicates, that a message cannot be sent because the bus is busy.
    */
    /*--------------------------------------------------------------------------------------------*/
    STM32F20X_CAN_BUSY_ERR,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    INIT ERROR
    *
    *           This code indicates, that the devices were not initialized because they are not in reset state.
    */
    /*--------------------------------------------------------------------------------------------*/
    STM32F20X_CAN_INIT_ERR,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    MODE ERROR
    *
    *           This code indicates, that the device cannot be accessed with the wanted mode.
    */
    /*--------------------------------------------------------------------------------------------*/
    STM32F20X_CAN_MODE_ERR,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    OPEN ERROR
    *
    *           This code indicates, that a device cannot be used because it is not opened.
    */
    /*--------------------------------------------------------------------------------------------*/
    STM32F20X_CAN_OPEN_ERR,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    CLOSE ERROR
    *
    *           This code indicates, that the device cannot be closed.
    */
    /*--------------------------------------------------------------------------------------------*/
    STM32F20X_CAN_CLOSE_ERR,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    FUNCTION CODE ERROR
    *
    *           This code indicates, that the given function code is not valid.
    */
    /*--------------------------------------------------------------------------------------------*/
    STM32F20X_CAN_FUNC_ERR,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    ARGUMENT ERROR
    *
    *           This code indicates, that an argument check has failed.
    */
    /*--------------------------------------------------------------------------------------------*/
    STM32F20X_CAN_ARG_ERR,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    NO DATA ERROR
    *
    *           This code indicates, that no data is available.
    */
    /*--------------------------------------------------------------------------------------------*/
    STM32F20X_CAN_NO_DATA_ERR
};

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief    I/O CONTROL FUNCTION CODES
*
* \ingroup  STM32F20X_CAN
*
*           This enum defines the available function codes for the driver XXXIoCtl() function.
*/
/*------------------------------------------------------------------------------------------------*/
enum {
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    GET DRIVER IDENT CODE
    *
    *           This standard function code gets the driver identification code.
    *
    *           arg = pointer to local ident variable (CPU_INT32U *)
    */
    /*--------------------------------------------------------------------------------------------*/
    IO_STM32F20X_CAN_GET_IDENT    = 0x0,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    GET DRIVER ERRORCODE
    *
    *           This standard function code gets the driver errorcode.
    *
    *           arg = pointer to local errorcode variable (CPU_INT16U *)
    */
    /*--------------------------------------------------------------------------------------------*/
    IO_STM32F20X_CAN_GET_ERRNO,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    GET DRIVER NAME
    *
    *           This standard function code gets the (human readable) driver name.
    *
    *           arg = pointer to local string variable (char *)
    */
    /*--------------------------------------------------------------------------------------------*/
    IO_STM32F20X_CAN_GET_DRVNAME,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    SET BUS BAUDRATE
    *
    *           This function code sets the bus baudrate.
    *
    *           arg = pointer to local baudrate variable (CPU_INT32U *)
    */
    /*--------------------------------------------------------------------------------------------*/
    IO_STM32F20X_CAN_SET_BAUDRATE = 0x10,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    ENABLE BUS
    *
    *           This enum value is the function code to start the CAN controller interface. Most
    *           common is to set the CAN controller in active mode.
    *
    *           The parameter pointer is not used for this function.
    */
    /*--------------------------------------------------------------------------------------------*/
    IO_STM32F20X_CAN_START,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    DISABLE BUS
    *
    *           This enum value is the function code to stop the CAN controller interface. Most
    *           common is to set the CAN controller in passive mode.
    *
    *           The parameter pointer is not used for this function.
    */
    /*--------------------------------------------------------------------------------------------*/
    IO_STM32F20X_CAN_STOP,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    SET RECEIVER TO STANDARD IDENTIFIER
    *
    *           This enum value is the function code to configure the CAN receiver to receive only
    *           CAN standard identifiers.
    *
    *           The parameter pointer is not used for this function.
    */
    /*--------------------------------------------------------------------------------------------*/
    IO_STM32F20X_CAN_RX_STANDARD,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    SET RECEIVER TO EXTENDED IDENTIFIER
    *
    *           This enum value is the function code to configure the CAN receiver to receive only
    *           CAN extended identifiers.
    *
    *           The parameter pointer is not used for this function.
    */
    /*--------------------------------------------------------------------------------------------*/
    IO_STM32F20X_CAN_RX_EXTENDED,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    GET TX READY STATUS
    *
    *           This enum value is the function code to get the Tx ready status.
    *
    *           The parameter pointer shall point to a CPU_INT08U variable, where the status shall
    *           be written to.
    *
    *           0 = Tx not ready
    *           1 = Tx ready
    */
    /*--------------------------------------------------------------------------------------------*/
    IO_STM32F20X_CAN_TX_READY,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    GET NODE STATUS
    *
    *           This enum value is the function code to get the node status.
    *
    *           The parameter pointer shall point to a CPU_INT08U variable, where the status shall
    *           be written to.
    *
    *           0 = Bus active
    *           1 = Error passive state
    *           2 = Bus-off state
    */
    /*--------------------------------------------------------------------------------------------*/
    IO_STM32F20X_CAN_GET_NODE_STATUS,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    SET RX FILTER 1
    *
    *           This enum value is the function code to set the acceptance filter 1 for the
    *           CAN controller.
    *
    *           arg = CPU_INT32U[2]: arg[0] = mask, arg[1] = ID
    */
    /*--------------------------------------------------------------------------------------------*/
    IO_STM32F20X_CAN_SET_RX_FILTER_1,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    SET RX FILTER 2
    *
    *           This enum value is the function code to set the acceptance filter 2 for the
    *           CAN controller.
    *
    *           arg = CPU_INT32U[2]: arg[0] = mask, arg[1] = ID
    */
    /*--------------------------------------------------------------------------------------------*/
    IO_STM32F20X_CAN_SET_RX_FILTER_2,
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    NUMBER OF FUNCTION CODES
    *
    *           This enum value holds the number of available function codes.
    */
    /*--------------------------------------------------------------------------------------------*/
    IO_STM32F20X_CAN_IO_FUNC_N
};

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief    DEVICE RUNTIME DATA
*
* \ingroup  STM32F20X_CAN
*
*           This struct holds the device runtime data.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct {
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    USE MARKER
    *
    *           This member holds a marker which indicates if this device is in use:
    *           0 = Device idle
    *           1 = Device in use
    */
    /*--------------------------------------------------------------------------------------------*/
	CPU_INT08U Use;
	/*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    BAUDRATE
    *
    *           This member holds the baudrate.
    */
    /*--------------------------------------------------------------------------------------------*/
    CPU_INT32U Baudrate;
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    BASE ADDRESS
    *
    *           This member holds the base address of the device.
    */
    /*--------------------------------------------------------------------------------------------*/
	void      *Base;
	/*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    BIT SAMPLE POINT
    *
    *           This member holds the bit sample point in 1/10 percent.
    */
    /*--------------------------------------------------------------------------------------------*/
	CPU_INT32U SamplePoint;
	/*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    RE-SYNCHRONIZATION JUMP WIDTH
    *
    *           This member holds the Re-synchronization Jump Width in 1/10 percent.
    */
    /*--------------------------------------------------------------------------------------------*/
	CPU_INT32U ResynchJumpWith;
	/*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    PRESCALER DEVIDE FACTOR
    *
    *           This member holds the prescaler devide factor.
    */
    /*--------------------------------------------------------------------------------------------*/
	CPU_INT16U PRESDIV;
	/*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    RE-SYNCHRONIZATION JUMP WIDTH
    *
    *           This member holds the Re-synchronization Jump Width (StdValue = 1).
    */
    /*--------------------------------------------------------------------------------------------*/
	CPU_INT08U RJW;
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    PROPAGATION SEGMENT TIME
    *
    *           This member holds the propagation segment time (StdValue = 2).
    */
    /*--------------------------------------------------------------------------------------------*/
	CPU_INT08U PROPSEG;
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    PHASE BUFFER SEGMENT 1
    *
    *           This member holds the phase buffer segment 1 (StdValue = 7).
    */
    /*--------------------------------------------------------------------------------------------*/
	CPU_INT08U PSEG1;
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    PHASE BUFFER SEGMENT 2
    *
    *           This member holds the phase buffer segment 2 (StdValue = 7).
    */
    /*--------------------------------------------------------------------------------------------*/
	CPU_INT08U PSEG2;
} STM32F20X_CAN_DATA;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief    CAN FRAME
*
* \ingroup  STM32F20X_CAN
*
*           This structure defines a CAN frame.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct {
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    CAN IDENTIFIER
    *
    *           This member holds the CAN identifier.
    */
    /*--------------------------------------------------------------------------------------------*/
    CPU_INT32U Identifier;
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    CAN PAYLOAD
    *
    *           This member holds up to 8 bytes, which can be handled with a single CAN message.
    */
    /*--------------------------------------------------------------------------------------------*/
    CPU_INT08U Data[8];
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    CAN DLC
    *
    *           This member holds the number of valid datas in the payload.
    */
    /*--------------------------------------------------------------------------------------------*/
    CPU_INT08U DLC;
    /*--------------------------------------------------------------------------------------------*/
    /*!
    * \brief    SPARE
    *
    *           These bytes are added to get a frame size of an integral number of pointers.
    */
    /*--------------------------------------------------------------------------------------------*/
    CPU_INT08U Spare[3];
} STM32F20X_CANFRM;

/*
****************************************************************************************************
*                                       FUNCTION PROTOTYPES
****************************************************************************************************
*/

CPU_INT16S STM32F20XCANInit  (CPU_INT32U arg);
CPU_INT16S STM32F20XCANOpen  (CPU_INT16S drvId, CPU_INT32U devName, CPU_INT16U mode);
CPU_INT16S STM32F20XCANClose (CPU_INT16S paraId);
CPU_INT16S STM32F20XCANIoCtl (CPU_INT16S paraId, CPU_INT16U func, void *argp);
CPU_INT16S STM32F20XCANRead  (CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size);
CPU_INT16S STM32F20XCANWrite (CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size);

CPU_INT16S STM32F20XCAN_CalcTimingReg (STM32F20X_CAN_DATA *data);
void       STM32F20XCAN_PinCfg        (CPU_INT32U arg);

/*
****************************************************************************************************
*                                          ERROR SECTION
****************************************************************************************************
*/

#if ((STM32F20X_CAN_ARG_CHK_CFG) < 0 || (STM32F20X_CAN_ARG_CHK_CFG > 1))
#error "STM32F20X/drv_can.h : The configuration STM32F20X_CAN_ARG_CHK_CFG must be 0 or 1"
#endif

#endif
