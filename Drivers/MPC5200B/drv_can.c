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
* Filename : drv_can.c
* Version  : V2.42.01
****************************************************************************************************
*/

/*
****************************************************************************************************
*                                             INCLUDES
****************************************************************************************************
*/

#include "drv_def.h"                            /* Driver layer definitions and constants         */
#include "drv_can_reg.h"                        /* Register definitions for CAN module            */
#include "can_bsp.h"                            /* BSP definitions for CAN module                 */
#include "drv_can.h"                            /* Driver function prototypes and data typedefs   */
#include "cpu.h"                                /* CPU defintions                                 */

/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MSCAN CAN MODULE START
* \ingroup  MSCAN
*
*           Defines all settings in the module configuration word to start the CAN bus:
*            - MSCAN enable
*/
/*------------------------------------------------------------------------------------------------*/
#define MSCAN_MODULE_START ( MSCAN_CANCTL1_CANE_EN + MSCAN_CANCTL1_CLKSRC_SEL )

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MSCAN CAN MODULE CONTORL 1
* \ingroup  MSCAN
*
*           Defines all settings in the module control 1 register excluding the propagation
*           segment time:
*            - Sample bits one times
*            - Loop back disabled
*            - Timer synchr. disabled
*            - Send lowest ID first
*/
/*------------------------------------------------------------------------------------------------*/

#define MSCAN_ALL_TXEIE (MSCAN_CANTIER_TXEIE0|MSCAN_CANTIER_TXEIE1|MSCAN_CANTIER_TXEIE2)
/*
****************************************************************************************************
*                                              MACROS
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ERROR LOGIN
* \ingroup  MSCAN
*
*           This macro is a dummy function for future error management.
*/
/*------------------------------------------------------------------------------------------------*/
#define ErrLogin(drvIdent) (MSCAN_CAN_NO_ERR)

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ERROR REGISTER
* \ingroup  MSCAN
*
*           This macro writes the given errorcode to the global error variable.
*/
/*------------------------------------------------------------------------------------------------*/
#define ErrRegister(drvIdent, errorcode) (DrvError = errorcode)

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CREATE IDHIGH REGISTER VALUE FOR STANDARD ID
* \ingroup  MSCAN
*
*           Creates the IDHigh register value for the given standard id.
*/
/*------------------------------------------------------------------------------------------------*/
#define MSCAN_STD_REG_IDHIGH(id) (CPU_INT16U)(id << 5)

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MASKS EXTENDED ID FOR REGISTER
* \ingroup  MSCAN
*
*           Masks the given extended id to set the can register value.
*/
/*------------------------------------------------------------------------------------------------*/
#define MSCAN_EXT_REG_MASK(id) (((id & 0x1FFC0000) << 3) | ((id & 0x0003FFFF) << 1))

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CREATE IDLOW REGISTER VALUE FOR EXTENDED ID
* \ingroup  MSCAN
*
*           Creates the IDLow register value for the given already masked standard id. The
*           extended CAN ID must be masked with MSCAN_EXT_REG_MASK before calling this macro.
*/
/*------------------------------------------------------------------------------------------------*/
#define MSCAN_EXT_REG_IDLOW(idmask) (CPU_INT16U)((idmask & 0xFFFE))

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CREATE IDHIGH REGISTER VALUE FOR EXTENDED ID
* \ingroup  MSCAN
*
*           Creates the IDHigh register value for the given already masked standard id. The
*           extended CAN ID must be masked with MSCAN_EXT_REG_MASK before calling this macro.
*/
/*------------------------------------------------------------------------------------------------*/
#define MSCAN_EXT_REG_IDHIGH(idmask) (CPU_INT16U)((idmask >> 16) | MSCAN_MB_IDR1_SRR_MASK | MSCAN_MB_IDR1_IDE_MASK)

/*
****************************************************************************************************
*                                            LOCAL DATA
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER IDENTIFICATION
* \ingroup  MSCAN
*
*           This constant variable holds the unique driver identification code.
*/
/*------------------------------------------------------------------------------------------------*/
static const CPU_INT32U DrvIdent = 0x243F2201;  /* version 2.4, 6 functions, ID 2201 */

/*
****************************************************************************************************
*                                             GLOBAL DATA
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER ERRORCODE
* \ingroup  MSCAN
*
*           This variable holds the detailed errorcode, if an error is detected.
*/
/*------------------------------------------------------------------------------------------------*/
static CPU_INT32U DrvError;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER DATA
* \ingroup  MSCAN
*
*           Global bus data of the CAN modules.
*/
/*------------------------------------------------------------------------------------------------*/
MSCAN_DATA CanData[MSCAN_DEV_N] = {{0,0,0,0,0,0,0,0,0,0,0},
                                   {0,0,0,0,0,0,0,0,0,0,0}};

/*
****************************************************************************************************
*                                            FUNCTIONS
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MSCAN CAN INITIALIZATION
* \ingroup  MSCAN
*
*           Initializes the MSCAN module and sets the default CAN bus baudrate.
*
* \param    arg               CAN device port number.
*
* \return   Errorcode
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MSCANInit (CPU_INT32U arg)
{
    volatile MSCAN_REG *reg;                               /* Local: Pointer to CAN module        */
             CPU_INT16S error;                             /* Local: Function result              */
                                                           /*-------------------------------------*/

                                                           /*-------------------------------------*/
    DrvError = MSCAN_CAN_NO_ERR;                           /* Set driver error to no error        */
    error = ErrLogin((CPU_INT16U)DrvIdent);                /* Connect to error management         */
                                                           /*-------------------------------------*/
    if (arg < (CPU_INT32U)MSCAN_DEV_N) {                   /* see if argument is in range         */
        if (CanData[arg].Initialized == 0) {               /* Initialize only unitialized devices */

            MSCAN_BSP_SetGPIO();

            CanData[arg].Base = MSCAN_BSP_GetBaseAddress(arg);

            reg = (MSCAN_REG *)CanData[arg].Base;
                                                           /* Initialize parameter struct of the  */
            CanData[arg].Initialized     = 1;              /* current device                      */
            CanData[arg].Baudrate        = MSCAN_DEF_BAUDRATE;
            CanData[arg].SamplePoint     = MSCAN_DEF_SP;
            CanData[arg].ResynchJumpWith = MSCAN_DEF_RJW;
                                                           /*-------------------------------------*/
                                                           /* Calculate the values for the timing */
            error = MSCANCalcTimingReg (&CanData[arg]);    /* register with the parameter setting */
            if (error != MSCAN_CAN_NO_ERR) {
                DrvError = MSCAN_CAN_INIT_ERR;             /* if baudrate can't achieved          */
                error = MSCAN_CAN_INIT_ERR;                /* stop initializing and return        */
                return error;
            }
                                                           /*-------------------------------------*/
                                                           /* Set module configuration and        */
                                                           /* control register                    */
            reg->CANCTL0 = MSCAN_CANCTL0_INITRQ_REQ;       /* Set Init Mode Req and wait          */
                                                           /* subsequent for acknowledge          */
            while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) == 0) {
            }

            reg->CANCTL1      = MSCAN_MODULE_START;
            reg->CANBTR0      = (CPU_INT08U)((CanData[arg].RJW << 6) + CanData[arg].PRESDIV);
            reg->CANBTR1      = (CPU_INT08U)((CanData[arg].PSEG2 << 4) + CanData[arg].PSEG1);

            reg->CANIDAC      = MSCAN_CANIDAC_TWO_32_BIT_FILTERS;
            reg->CANIDMR0     = 0xff;                 /* receive mask: Every bit in the ID...     */
            reg->CANIDMR1     = 0xff;                 /* 1 = ignore bits, 0 = match bit           */
            reg->CANIDMR2     = 0xff;
            reg->CANIDMR3     = 0xff;
            reg->CANIDMR4     = 0xff;
            reg->CANIDMR5     = 0xff;
            reg->CANIDMR6     = 0xff;
            reg->CANIDMR7     = 0xff;                 /* ...is ignored                            */

#if MSCAN_FILTER_INIT_HOOK_EN == 1
            MSCAN_BSP_InitHook (arg);                 /* can be used to set different filter      */
#endif
            reg->CANCTL0      = 0;                    /* Reset Init Mode Req and wait             */
                                                      /* subsequent for neg aknowledge            */
            while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) != 0) {
            }

#if MSCAN_EN_ISRH_CFG == 1
                                                      /* Enable RX and node status interrupts     */
            reg->CANRIER |= MSCAN_CANRIER_RXFIE_EN |
                            MSCAN_CANRIER_CSCIE_EN |
                            MSCAN_CANRIER_TSTATE_EN |
                            MSCAN_CANRIER_RSTATE_EN;
#endif
                                                      /* TX Interrupts are enabled later, if      */
                                                      /* during a message write the last free     */
                                                      /* buffer was used                          */
        } else {
            DrvError = MSCAN_CAN_INIT_ERR;            /* set driver error to init error           */
            error = MSCAN_CAN_INIT_ERR;
        }
    } else {
        DrvError = MSCAN_CAN_INIT_ERR;                /* set driver error to init error           */
        error = MSCAN_CAN_INIT_ERR;
    }                                                 /*------------------------------------------*/

    return (error);
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MSCAN CAN OPEN MESSAGE BUFFER
* \ingroup  MSCAN
*
*           Opens the CAN bus after checking that the bus is not previously opened.
*
* \param    devId             Unused, but needed for common interface
* \param    devName           The MSCAN device name
* \param    mode              must be DEV_RW for this device.
*
* \return   The device identifier for further access or -1 if an error occurs
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MSCANOpen (CPU_INT16S devId,
                      CPU_INT32U devName,
                      CPU_INT16U mode)
{
    CPU_INT16S result = -1;                           /* Local: Function result                   */
    CPU_SR_ALLOC();                                   /* Local: variable for critical section     */
                                                      /*------------------------------------------*/

    (void)devId;                                      /* prevent compiler warning                 */

                                                      /*------------------------------------------*/
#if MSCAN_ARG_CHK_CFG > 0                             /* Checking arguments (if enabled)          */
    if (devName >= (CPU_INT32U)MSCAN_DEV_N) {         /* devName out of range?                    */
        ErrRegister((CPU_INT16U)DrvIdent, MSCAN_CAN_BUS_ERR);
        return (result);
    }
    if (mode != DEV_RW) {                             /* mode not supported?                      */
        ErrRegister((CPU_INT16U)DrvIdent, MSCAN_CAN_MODE_ERR);
        return (result);
    }
#else
    (void) mode;
#endif
    CPU_CRITICAL_ENTER();
                                                      /* check, that CAN device is not open       */
    if ((CanData[devName].Status & MSCAN_OPEN) == 0) {
        CanData[devName].Status |= MSCAN_OPEN;        /* Mark CAN device to be opened             */
        CPU_CRITICAL_EXIT();
        result = (CPU_INT16S)devName;                 /* Set return value to devName              */
    } else {                                          /*------------------------------------------*/
        CPU_CRITICAL_EXIT();
        ErrRegister((CPU_INT16U)DrvIdent, MSCAN_CAN_OPEN_ERR);
    }
                                                      /*------------------------------------------*/
    return (result);                                  /* return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MSCAN CAN CLOSE SUBDEVICE
* \ingroup  MSCAN
*
*           Close the CAN bus after checking that this  device is opened.
*
* \param    devId             The device identifier, returned by MSCANOpen()
*
* \return   Errorcode (0 if ok, -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MSCANClose (CPU_INT16S devId)
{
    CPU_INT16S result = -1;                           /* Local: Function result              */
                                                      /*-------------------------------------*/
#if MSCAN_ARG_CHK_CFG > 0                             /* Checking arguments (if enabled)     */
    if ((devId < 0) ||                                /* devId out of range?                 */
        (devId >= (CPU_INT16S)MSCAN_DEV_N)) {
        ErrRegister((CPU_INT16U)DrvIdent, MSCAN_CAN_BUS_ERR);
        return (-1);
    }
#endif

    if (CanData[devId].Status & MSCAN_OPEN) {         /* see, if CAN device is opened        */
        CanData[devId].Status = MSCAN_IDLE;           /* yes: reset CAN device status        */
        CanData[devId].Initialized = 0;               /* set to not initialized              */
        result = MSCAN_CAN_NO_ERR;                    /* Ok, device is closed                */
    } else {                                          /*-------------------------------------*/
        ErrRegister((CPU_INT16U)DrvIdent,             /* set device close error              */
                    MSCAN_CAN_CLOSE_ERR);
    }

    return (result);                                  /* return function result              */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MSCAN CAN I/O CONTROL
* \ingroup  MSCAN
*
*           This function performs a special action on the opened device. The functioncode func
*           defines what the caller want to do. For description of functioncodes see the headerfile.
*
* \param    devId             The device identifier, returned by MSCANOpen()
* \param    func              Function code (see MSCAN_ioctl_func)
* \param    arg               Argument list, specific to the function code
*
* \return   Errorcode (0 if ok, -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MSCANIoCtl (CPU_INT16S   devId,
                       CPU_INT16U   func,
                       void        *arg)
{
    volatile MSCAN_REG *reg;                          /* Local: Pointer to CAN module             */
    MSCAN_DATA  *data;                                /* Local: Pointer to CAN device data        */
    CPU_INT16S   result = MSCAN_CAN_NO_ERR;           /* Local: Function result                   */
    CPU_INT32U   reg_data;                            /* Local: register value                    */
    CPU_INT32U   mask;                                /* Local: mask for filter                   */
    CPU_INT32U   canId;                               /* Local: id for filter                     */
    CPU_INT16U   canIdl;                              /* Local: id for filter                     */
    CPU_INT16U   canIdh;                              /* Local: id for filter                     */
    CPU_INT16U   maskh;                               /* Local: id for filter                     */
    CPU_INT08U   freetxbuf;                           /* Local: variable to store free tx buffer  */
    CPU_SR_ALLOC();                                   /* Local: variable for critical section     */
                                                      /*------------------------------------------*/

#if MSCAN_ARG_CHK_CFG > 0                             /* Checking arguments (if enabled)          */
    if ((devId < 0) ||                                /* devId out of range?                      */
        (devId >= (CPU_INT16S)MSCAN_DEV_N)) {
        ErrRegister((CPU_INT16U)DrvIdent, MSCAN_CAN_BUS_ERR);
        return (-1);
    }
#endif                                                /*------------------------------------------*/

    reg  = (MSCAN_REG *)CanData[devId].Base;          /* Set pointer to CAN module                */
    data = &CanData[devId];                           /* Set pointer to CAN device data           */

    if ((data->Status & MSCAN_OPEN) == 0) {           /* see, if CAN device is open               */
        ErrRegister((CPU_INT16U)DrvIdent,
                    MSCAN_CAN_OPEN_ERR);              /* yes: set device not opened error         */
        return (-1);
    }

    CPU_CRITICAL_ENTER();

    switch (func) {                                   /* select: function code                    */
                                                      /*------------------------------------------*/
        case IO_MSCAN_CAN_GET_IDENT:                  /*                GET IDENT                 */
            (*(CPU_INT32U*)arg) = DrvIdent;           /* return driver ident code                 */
            break;
                                                      /*------------------------------------------*/
        case IO_MSCAN_CAN_GET_ERRNO:                  /*              GET ERRORCODE               */
            (*(CPU_INT32U*)arg) = DrvError;           /* return last detected errorcode           */
            break;
                                                      /*------------------------------------------*/
        case IO_MSCAN_CAN_GET_DRVNAME:                /*             GET DRIVER NAME              */
                                                      /* return human readable driver name        */
            (*(CPU_INT08U**)arg) = (CPU_INT08U*)MSCAN_CAN_NAME; /*lint !e926 conversion is ok     */
            break;
                                                      /*------------------------------------------*/
        case IO_MSCAN_CAN_SET_BAUDRATE:               /*               SET BAUDRATE               */
            data->Baudrate = *(CPU_INT32U *)arg;      /* Set baudrate parameter to given argument */
                                                      /* and calculate the values for the timing  */
                                                      /* register with the parameter setting      */
            if ( MSCANCalcTimingReg(data) != MSCAN_CAN_NO_ERR) {
                ErrRegister((CPU_INT16U)DrvIdent,
                             MSCAN_CAN_ARG_ERR);      /* set errorcode if an error occured and    */
                result = -1;                          /* indicate error on function result.       */
            } else {                                  /* set can timing registers if timing       */
                                                      /* calculation success.                     */
                reg_data = reg->CANRIER;              /* store irq settings                       */
                reg->CANCTL0 = MSCAN_CANCTL0_INITRQ_REQ; /* Set Init Mode Req and wait            */
                                                         /* subsequent for aknowledge             */
                while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) == 0) {
                }
                reg->CANBTR0 = (CPU_INT08U)((data->RJW << 6) + data->PRESDIV);
                reg->CANBTR1 = (CPU_INT08U)((data->PSEG2 << 4) + data->PSEG1);
                reg->CANCTL0 = 0;                     /* Reset Init Mode Req and wait             */
                                                      /* subsequent for neg aknowledge            */
                while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) != 0) {
                }
                reg->CANRIER = (CPU_INT08U)reg_data;  /* restore irq settings                     */
            }

            break;
                                                      /*------------------------------------------*/
        case IO_MSCAN_CAN_START:                      /*                  START                   */
            reg->CANCTL0 = MSCAN_CANCTL0_INITRQ_REQ;  /* Set Init Mode Req and wait               */
                                                      /* subsequent for aknowledge                */
            while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) == 0) {
            }
            reg->CANCTL1 = MSCAN_MODULE_START;
            reg->CANCTL0  = 0;                        /* Reset Init Mode Req and wait             */
                                                      /* subsequent for neg aknowledge            */
            while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) != 0) {
            }
#if MSCAN_EN_ISRH_CFG == 1
                                                      /* Enable RX and node status interrupts     */
            reg->CANRIER |= MSCAN_CANRIER_RXFIE_EN |
                            MSCAN_CANRIER_CSCIE_EN |
                            MSCAN_CANRIER_TSTATE_EN |
                            MSCAN_CANRIER_RSTATE_EN;
#endif
            break;
                                                      /*------------------------------------------*/
        case IO_MSCAN_CAN_STOP:                       /*                  STOP                    */
            reg->CANCTL0 = MSCAN_CANCTL0_INITRQ_REQ;  /* Set Init Mode Req and wait               */
                                                      /* subsequent for aknowledge                */
            while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) == 0) {
            }
            break;
                                                      /*------------------------------------------*/
        case IO_MSCAN_CAN_RX_STANDARD:                /*           STANDARD RX IDENTIFIER         */
            if ((reg->CANIDAC &                       /* see if 32 Bit Acc. Filter is used:       */
                  MSCAN_CANIDAC_FILTER_CLOSED) == 0){
                reg_data = reg->CANRIER;              /* store irq settings                       */
                reg->CANCTL0 = MSCAN_CANCTL0_INITRQ_REQ;  /* Set Init Mode Req and wait           */
                                                          /* subsequent for aknowledge            */
                while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) == 0) {
                }

                reg->CANIDMR1 &= ~MSCAN_MB_IDR1_IDE_MASK; /* Filter 1                             */
                reg->CANIDAR1 &= ~MSCAN_MB_IDR1_IDE_MASK;
                reg->CANIDMR5 &= ~MSCAN_MB_IDR1_IDE_MASK; /* Filter 2                             */
                reg->CANIDAR5 &= ~MSCAN_MB_IDR1_IDE_MASK;
                reg->CANCTL0      = 0;                /* Reset Init Mode Req and wait             */
                                                      /* subsequent for neg aknowledge            */
                while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) != 0) {
                }
                reg->CANRIER = (CPU_INT08U)reg_data;  /* restore irq settings                      */
            }
            break;
                                                      /*------------------------------------------*/
        case IO_MSCAN_CAN_RX_EXTENDED:                /*           EXTENDED RX IDENTIFIER         */
            if ((reg->CANIDAC &                       /* see if 32 Bit Acc. Filter is used:       */
                    MSCAN_CANIDAC_FILTER_CLOSED) == 0){
                reg_data = reg->CANRIER;              /* store irq settings                       */
                reg->CANCTL0 = MSCAN_CANCTL0_INITRQ_REQ;   /* Set Init Mode Req and wait          */
                                                      /* subsequent for acknowledge               */
                while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) == 0) {
                }
                reg->CANIDMR1 &= ~MSCAN_MB_IDR1_IDE_MASK;
                reg->CANIDAR1 |=  MSCAN_MB_IDR1_IDE_MASK;
                reg->CANIDMR5 &= ~MSCAN_MB_IDR1_IDE_MASK;
                reg->CANIDAR5 |=  MSCAN_MB_IDR1_IDE_MASK;
                reg->CANCTL0      = 0;                /* Reset Init Mode Req and wait             */
                                                      /* subsequent for neg aknowledge            */
                while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) != 0) {
                }
                reg->CANRIER = (CPU_INT08U)reg_data;  /* restore irq settings                     */
            }
            break;
                                                      /*------------------------------------------*/
        case IO_MSCAN_CAN_TX_READY:                   /*              GET TX STATUS               */
            freetxbuf = reg->CANTFLG;                 /* check availability of buffers, so that   */
                                                      /* no higher priority messages than those   */
                                                      /* already scheduled are released           */
                                                      /* see MPC5200B errata 5.1                  */
            if ((freetxbuf == (MSCAN_CANTFLG_TXE0 | MSCAN_CANTFLG_TXE1 | MSCAN_CANTFLG_TXE2)) ||
                (freetxbuf == (MSCAN_CANTFLG_TXE1 | MSCAN_CANTFLG_TXE2)) ||
                (freetxbuf ==  MSCAN_CANTFLG_TXE2)) {
                *((CPU_BOOLEAN *)arg) = 1;            /* set return value to true                 */
            } else {
                *((CPU_BOOLEAN *)arg) = 0;            /* Otherwise                                */
            }                                         /* set return value to false.               */
            break;
                                                      /*------------------------------------------*/
        case IO_MSCAN_CAN_GET_NODE_STATUS:            /*            GET NODE STATUS               */
            reg_data = reg->CANRFLG;
            if ((reg_data & 0x3C) == 0) {             /* Bit 2-5 not set - bus active             */
                *((CPU_INT08U *)arg) = 0;
            } else if (((reg_data & 0x30)== 0x30) ||  /* Bus Off Bits set                         */
                     ((reg_data & 0x0C)== 0x0C)) {
                *((CPU_INT08U *)arg) = 2;
            } else  {                                 /* Error Warning Bits                       */
                *((CPU_INT08U *)arg) = 1;
            }
            break;
                                                      /*------------------------------------------*/
        case IO_MSCAN_CAN_SET_RX_FILTER_1:            /*               SET RX FILTER 1            */
            mask  = ((CPU_INT32U*)arg)[0];
            canId = ((CPU_INT32U*)arg)[1];
            if ((reg->CANIDAC &                       /* see if 32 Bit Acc. Filter is used:       */
                  MSCAN_CANIDAC_FILTER_CLOSED) == 0){
                reg_data = reg->CANRIER;              /* store irq settings                       */
                reg->CANCTL0 = MSCAN_CANCTL0_INITRQ_REQ;  /* Set Init Mode Req and wait           */
                                                      /* subsequent for aknowledge                */
                while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) == 0) {
                }

                if (canId > MSCAN_STD_ID_MASK) {      /* depending on the CAN ID                  */
                    /* use bit 0..28 */
                    canId  &= MSCAN_EXT_ID_MASK;
                    canId   = MSCAN_EXT_REG_MASK(canId);
                    canIdl  = MSCAN_EXT_REG_IDLOW(canId);
                    canIdh  = MSCAN_EXT_REG_IDHIGH(canId);
                    canIdh |= MSCAN_MB_IDR1_IDE_MASK;
                    mask   &= MSCAN_EXT_ID_MASK;
                    mask    = MSCAN_EXT_REG_MASK(mask);
                                                      /* set bit for RTR frames                   */
                    mask    &= ~MSCAN_DISCARD_RTR_FRAMES; /* set to 0 to check bit                */
                    canIdl  &= ~MSCAN_DISCARD_RTR_FRAMES; /* set to 0 to get only data frames     */

                    reg->CANIDMR0 = (CPU_INT08U)((mask & 0xff000000) >> 24);
                    reg->CANIDMR1 = (CPU_INT08U)((mask & 0x00ff0000) >> 16);
                    reg->CANIDMR2 = (CPU_INT08U)((mask & 0x0000ff00) >> 8);
                    reg->CANIDMR3 = (CPU_INT08U) (mask & 0x000000ff);

                    reg->CANIDAR0 = (CPU_INT08U)((canIdh & 0xff00) >> 8);
                    reg->CANIDAR1 = (CPU_INT08U) (canIdh & 0xff) ;
                    reg->CANIDAR2 = (CPU_INT08U)((canIdl & 0xff00) >> 8);
                    reg->CANIDAR3 = (CPU_INT08U) (canIdl & 0xff);

                } else {
                    /* use bit 0..10 */
                    canId   &= MSCAN_STD_ID_MASK;
                    canIdh   = MSCAN_STD_REG_IDHIGH(canId);
                    mask    &= MSCAN_STD_ID_MASK;
                    maskh    = MSCAN_STD_REG_IDHIGH(mask);
                                                      /* set bit for RTR frames                   */
                    maskh   &= ~(MSCAN_DISCARD_RTR_FRAMES << 4); /* set to 0 to check bit         */
                    canIdh  &= ~(MSCAN_DISCARD_RTR_FRAMES << 4); /* set to 0 to get only data frames */

                    reg->CANIDMR0 = (CPU_INT08U)((maskh & 0xff00) >> 8);
                    reg->CANIDMR1 = (CPU_INT08U) (maskh & 0xff);

                    reg->CANIDAR0 = (CPU_INT08U)((canIdh & 0xff00) >> 8);
                    reg->CANIDAR1 = (CPU_INT08U) (canIdh & 0xff);
                }

                reg->CANCTL0  = 0;                    /* Reset Init Mode Req and wait             */
                                                      /* subsequent for neg aknowledge            */
                while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) != 0) {
                }
                reg->CANRIER = (CPU_INT08U)reg_data;  /* restore irq settings                     */
            }

            break;
                                                      /*------------------------------------------*/
        case IO_MSCAN_CAN_SET_RX_FILTER_2:            /*               SET RX FILTER 2            */
            mask  = ((CPU_INT32U*)arg)[0];
            canId = ((CPU_INT32U*)arg)[1];
            if ((reg->CANIDAC &                       /* see if 32 Bit Acc. Filter is used:       */
                  MSCAN_CANIDAC_FILTER_CLOSED) == 0){
                reg_data = reg->CANRIER;              /* store irq settings                       */
                reg->CANCTL0 = MSCAN_CANCTL0_INITRQ_REQ;  /* Set Init Mode Req and wait           */
                                                      /* subsequent for aknowledge                */
                while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) == 0) {
                }

                if (canId > MSCAN_STD_ID_MASK) {      /* depending on the CAN ID                  */
                                                      /* use bit 0..28                            */
                    canId  &= MSCAN_EXT_ID_MASK;
                    canId   = MSCAN_EXT_REG_MASK(canId);
                    canIdl  = MSCAN_EXT_REG_IDLOW(canId);
                    canIdh  = MSCAN_EXT_REG_IDHIGH(canId);
                    canIdh |= MSCAN_MB_IDR1_IDE_MASK;
                    mask   &= MSCAN_EXT_ID_MASK;
                    mask    = MSCAN_EXT_REG_MASK(mask);
                                                      /* mask bit for RTR frames                  */
                    mask    &= ~MSCAN_DISCARD_RTR_FRAMES; /* set to 0 to check bit                */
                    canIdl  &= ~MSCAN_DISCARD_RTR_FRAMES; /* set to 0 to get only data frames     */

                    reg->CANIDMR4 = (CPU_INT08U)((mask & 0xff000000) >> 24);
                    reg->CANIDMR5 = (CPU_INT08U)((mask & 0x00ff0000) >> 16);
                    reg->CANIDMR6 = (CPU_INT08U)((mask & 0x0000ff00) >> 8);
                    reg->CANIDMR7 = (CPU_INT08U) (mask & 0x000000ff);

                    reg->CANIDAR4 = (CPU_INT08U)((canIdh & 0xff00) >> 8);
                    reg->CANIDAR5 = (CPU_INT08U) (canIdh & 0xff) ;
                    reg->CANIDAR6 = (CPU_INT08U)((canIdl & 0xff00) >> 8);
                    reg->CANIDAR7 = (CPU_INT08U) (canIdl & 0xff);

                } else {
                    /* use bit 0..10 */
                    canId &= MSCAN_STD_ID_MASK;
                    canIdh = MSCAN_STD_REG_IDHIGH(canId);
                    mask &= MSCAN_STD_ID_MASK;
                    maskh = MSCAN_STD_REG_IDHIGH(mask);

                                                        /* set bit for RTR frames                    */
                    maskh   &= ~(MSCAN_DISCARD_RTR_FRAMES << 4); /* set to 0 to check bit            */
                    canIdh  &= ~(MSCAN_DISCARD_RTR_FRAMES << 4); /* set to 0 to get only data frames */
                    reg->CANIDMR4 = (CPU_INT08U)((maskh & 0xff00) >> 8);
                    reg->CANIDMR5 = (CPU_INT08U) (maskh & 0xff);

                    reg->CANIDAR4 = (CPU_INT08U)((canIdh & 0xff00) >> 8);
                    reg->CANIDAR5 = (CPU_INT08U) (canIdh & 0xff);
                }

                reg->CANCTL0  = 0;                    /* Reset Init Mode Req and wait             */
                                                      /* subsequent for neg aknowledge            */
                while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) != 0) {
                }
                reg->CANRIER = (CPU_INT08U)reg_data;  /* restore irq settings                     */

            }
            break;
                                                      /*------------------------------------------*/
        case IO_MSCAN_CAN_ENABLE_LOOPBACK:            /*               ENABLE LOOPBACK            */
            reg_data = reg->CANRIER;                  /* store irq settings                       */
            reg->CANCTL0 = MSCAN_CANCTL0_INITRQ_REQ;  /* Set Init Mode Req and wait               */
                                                      /* subsequent for aknowledge                */
            while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) == 0) {
            }
            reg->CANCTL1 = MSCAN_CANCTL1_LOOPB_EN +   /* set the loopback mode                    */
                           MSCAN_MODULE_START;        /* and start the CAN device                 */
            reg->CANCTL0  = 0;                        /* Reset Init Mode Req and wait             */
                                                      /* subsequent for neg aknowledge            */
            while ((reg->CANCTL1 & MSCAN_CANCTL1_INITAK) != 0) {
            }
            reg->CANRIER = (CPU_INT08U)reg_data;      /* restore irq settings                     */
            break;
        default:                                      /*           WRONG FUNCTION CODE            */
            result = -1;                              /* indicate error on function result        */
            break;
    }
                                                      /*------------------------------------------*/
    if (result < 0) {                                 /* If an error occured,                     */
        ErrRegister((CPU_INT16U)DrvIdent,             /* register error in error management       */
                    MSCAN_CAN_FUNC_ERR);
    }
    CPU_CRITICAL_EXIT();
                                                      /*------------------------------------------*/
    return (result);                                  /* Return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MSCAN CAN READ DATA
* \ingroup  MSCAN
*
*           Read a received message from a message buffer after checking, that this  device is
*           opened.
*
* \param    devId             The device identifier, returned by MSCANOpen()
* \param    buf               Byte array for received data
* \param    size              Length of can frame memory
*
* \return   Errorcode (0 if ok, -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MSCANRead (CPU_INT16S devId,
                      CPU_INT08U *buf,
                      CPU_INT16U size)
{
    volatile MSCAN_REG *reg;                          /* Local: Pointer to CAN module             */
    MSCAN_DATA         *data;                         /* Local: Pointer to CAN device data        */
    MSCAN_FRM          *frm;                          /* Local: Pointer to can frame              */
    CPU_INT16S          result   = MSCAN_CAN_NO_ERR;  /* Local: Function result                   */
    CPU_BOOLEAN         update;                       /* Local: update flag                       */
    CPU_SR_ALLOC();                                   /* Local: variable for critical section     */
                                                      /*------------------------------------------*/

#if MSCAN_ARG_CHK_CFG > 0                             /* Checking arguments (if enabled)          */
    if ((devId < 0) ||                                /* devId out of range?                      */
        (devId >= (CPU_INT16S)MSCAN_DEV_N)) {
        ErrRegister((CPU_INT16U)DrvIdent, MSCAN_CAN_BUS_ERR);
        return (-1);
    }
    if (buf == (void*)0) {                            /* valid buffer pointer?                    */
        ErrRegister((CPU_INT16U)DrvIdent, MSCAN_CAN_ARG_ERR);
        return (-1);
    }
    if (size != sizeof(MSCAN_FRM)) {                 /* size in range?                           */
        ErrRegister((CPU_INT16U)DrvIdent, MSCAN_CAN_ARG_ERR);
        return (-1);
    }
#else
   (void)size;
#endif                                                /*------------------------------------------*/

    reg  = (MSCAN_REG *)CanData[devId].Base;          /* Set pointer to CAN module                */
    data = &CanData[devId];                           /* Set pointer to CAN device data           */
                                                      /* Set pointer to can frame                 */
    frm  = (MSCAN_FRM *)buf;                          /*lint !e927 !e826 conversion is OK         */
                                                      /*------------------------------------------*/
    if (!(data->Status & MSCAN_OPEN)) {               /* see, if CAN device is not opened         */
        ErrRegister((CPU_INT16U)DrvIdent,
                    MSCAN_CAN_OPEN_ERR);              /* yes: set device not opened error         */
        return (-1);
    }                                                 /*------------------------------------------*/
    update = 0;
    if (data->Status & MSCAN_BLOCKING) {              /* if driver is in blocking mode            */
        while (!(reg->CANRFLG & MSCAN_CANRFLG_RXF)) { /* wait until rx complete flag is set       */
        }
        update = 1;
    } else {
        if (reg->CANRFLG & MSCAN_CANRFLG_RXF) {
            update = 1;
        }
    }
                                                      /*------------------------------------------*/
    if (update != 0) {                                /* do following if update happend           */
        CPU_CRITICAL_ENTER();

        frm->DLC = reg->MsgBufRx.DLC & 0xF;           /* get number of received bytes             */

        frm->Data[0] = reg->MsgBufRx.Data.Byte.Data0; /* get received data bytes                  */
        frm->Data[1] = reg->MsgBufRx.Data.Byte.Data1;
        frm->Data[2] = reg->MsgBufRx.Data.Byte.Data2;
        frm->Data[3] = reg->MsgBufRx.Data.Byte.Data3;
        frm->Data[4] = reg->MsgBufRx.Data.Byte.Data4;
        frm->Data[5] = reg->MsgBufRx.Data.Byte.Data5;
        frm->Data[6] = reg->MsgBufRx.Data.Byte.Data6;
        frm->Data[7] = reg->MsgBufRx.Data.Byte.Data7;
                                                      /* get identifier format and extract        */
        if ((reg->MsgBufRx.IDHigh &                   /* identifier from the registers            */
             MSCAN_MB_IDR1_IDE_MASK) == 0) {          /*  - standard identifier resp.             */
            frm->Identifier = (reg->MsgBufRx.IDHigh >> 5);

            if ((reg->MsgBufRx.IDHigh & 0x10) != 0) { /* remote frame                             */
                frm->Identifier |= MSCAN_RTR_FLAG;
            }
        } else {
                                                      /*  - extended identifier                   */
            frm->Identifier =  ((reg->MsgBufRx.IDHigh & 0xFFE0) << 13);
            frm->Identifier += ((reg->MsgBufRx.IDHigh & 0x0007) << 15);
            frm->Identifier += ((reg->MsgBufRx.IDLow  & 0xFFFE) >> 1);
            frm->Identifier |= MSCAN_EXT_ID_FLAG;
            if ((reg->MsgBufRx.IDLow & 0x01) != 0) { /* remote frame                             */
                frm->Identifier |= MSCAN_RTR_FLAG;
            }
        }

        CPU_CRITICAL_EXIT();
    } else {                                          /* No message received,                     */
        ErrRegister((CPU_INT16U)DrvIdent,             /* register error in error management       */
                MSCAN_CAN_NO_DATA_ERR);
        result = -1;
    }
                                                      /*------------------------------------------*/
    return (result);                                  /* Return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MSCAN CAN WRITE DATA
* \ingroup  MSCAN
*
*           Write a message to a message buffer after checking, that this  device is opened.
*
* \param    devId             The device identifier, returned by MSCANOpen()
* \param    buf               Byte array for transmitting data
* \param    size              Length of can frame memory
*
* \return   size of(MSCAN_FRM) if ok, -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MSCANWrite (CPU_INT16S devId,
                       CPU_INT08U *buf,
                       CPU_INT16U size)
{
    volatile MSCAN_REG *reg;                          /* Local: Pointer to CAN module             */
    MSCAN_DATA         *data;                         /* Local: Pointer to CAN device data        */
    MSCAN_FRM          *frm;                          /* Local: Pointer to can frame              */
    CPU_INT16S          result = MSCAN_CAN_BUSY_ERR;  /* Local: Function result                   */
    CPU_INT32U          canId;                        /* Local: CAN Identifier                    */
    CPU_INT32U          mask;                         /* Local: Mask for identifier caluclations  */
    CPU_INT08U          blockingLoop;                 /* Local: Flag for finishing endless loop   */
    CPU_INT08U          freetxbuf;                    /* Local: variable to store free tx buffer  */
    CPU_SR_ALLOC();                                   /* Local: variable for critical section     */
                                                      /*------------------------------------------*/

#if MSCAN_ARG_CHK_CFG > 0                             /* Checking arguments (if enabled)          */
    if ((devId < 0) ||                                /* devId out of range?                      */
        (devId >= (CPU_INT16S)MSCAN_DEV_N)) {
        ErrRegister((CPU_INT16U)DrvIdent,
                    MSCAN_CAN_BUS_ERR);
        return (-1);
    }
    if (buf == (void*)0) {                            /* valid buffer pointer?                    */
        ErrRegister((CPU_INT16U)DrvIdent,
                    MSCAN_CAN_ARG_ERR);
        return (-1);
    }
    if (size != sizeof(MSCAN_FRM)) {                  /* size in range?                           */
        ErrRegister((CPU_INT16U)DrvIdent,
                    MSCAN_CAN_ARG_ERR);
        return (-1);
    }
#else
   (void)size;
#endif                                                /*------------------------------------------*/

    reg  = (MSCAN_REG *)CanData[devId].Base;          /* Set pointer to CAN module                */
    data = &CanData[devId];                           /* Set pointer to CAN device data           */
                                                      /* Set pointer to can frame                 */
    frm  = (MSCAN_FRM *)buf;                          /*lint !e927 !e826 conversion is OK         */
                                                      /*------------------------------------------*/
    if (!(data->Status & MSCAN_OPEN)) {               /* see, if CAN device is not opened         */
        ErrRegister((CPU_INT16U)DrvIdent,
                    MSCAN_CAN_OPEN_ERR);              /* yes: set device not opened error         */
        return (-1);
    }
                                                      /*------------------------------------------*/
    if (data->Status & MSCAN_BLOCKING) {              /* if driver is in blocking mode            */
        do {                                          /* check state of  tx buffers               */
            blockingLoop = reg->CANTFLG;              /* finish loops                             */
        } while (blockingLoop == 0);                  /*------------------------------------------*/
    }

    CPU_CRITICAL_ENTER();
                                                      /*------------------------------------------*/
    freetxbuf = reg->CANTFLG;                         /* check availability of buffers, so that   */
                                                      /* no higher priority messages than those   */
                                                      /* already scheduled are released           */
                                                      /* see MPC5200B errata 5.1                  */
    if ((freetxbuf == (MSCAN_CANTFLG_TXE0 | MSCAN_CANTFLG_TXE1 | MSCAN_CANTFLG_TXE2)) ||
        (freetxbuf == (MSCAN_CANTFLG_TXE1 | MSCAN_CANTFLG_TXE2)) ||
        (freetxbuf ==  MSCAN_CANTFLG_TXE2)) {

        if (frm->DLC > 8) {                           /* Limit DLC to 8 bytes                     */
            frm->DLC = 8;
        }
        reg->CANTBSEL = freetxbuf;                    /* select a free tx buffer                  */

        canId = frm->Identifier;                      /* Write the ID_HIGH and ID_LOW registers   */
        if (canId > MSCAN_STD_ID_MASK) {              /* depending on the CAN ID                  */
                                                      /* use bit 0..28                            */
            canId &= MSCAN_EXT_ID_MASK;
            mask   = MSCAN_EXT_REG_MASK(canId);
            reg->MsgBufTx.IDLow  = MSCAN_EXT_REG_IDLOW(mask);
            reg->MsgBufTx.IDHigh = MSCAN_EXT_REG_IDHIGH(mask);
        } else {
                                                      /* use bit 0..10                            */
            canId &= MSCAN_STD_ID_MASK;
            reg->MsgBufTx.IDHigh = MSCAN_STD_REG_IDHIGH(canId);
        }
                                                      /* Write the data bytes into the CAN buffer */
        reg->MsgBufTx.Data.Byte.Data0 = frm->Data[0];
        reg->MsgBufTx.Data.Byte.Data1 = frm->Data[1];
        reg->MsgBufTx.Data.Byte.Data2 = frm->Data[2];
        reg->MsgBufTx.Data.Byte.Data3 = frm->Data[3];
        reg->MsgBufTx.Data.Byte.Data4 = frm->Data[4];
        reg->MsgBufTx.Data.Byte.Data5 = frm->Data[5];
        reg->MsgBufTx.Data.Byte.Data6 = frm->Data[6];
        reg->MsgBufTx.Data.Byte.Data7 = frm->Data[7];

        reg->MsgBufTx.DLC       = frm->DLC;           /* set DLC in buffer                        */
        reg->MsgBufTx.TBPR      = 0;                  /* set buffer prio to default               */
        reg->CANTFLG            = reg->CANTBSEL;      /* send msg and clear flag                  */
        if (reg->CANTFLG == 0) {                      /* see if tx queue is full:                 */
            reg->CANTIER = MSCAN_ALL_TXEIE;           /* enable tx interrupts                     */
        }
        result = sizeof(MSCAN_FRM);                   /* Set return value to no error             */
                                                      /*------------------------------------------*/
    } else {                                          /* No free tx buffer found                  */
        ErrRegister((CPU_INT16U)DrvIdent,             /* register error in error management       */
                    MSCAN_CAN_BUSY_ERR);
        result = -1;
    }

    CPU_CRITICAL_EXIT();
                                                      /*------------------------------------------*/
    return (result);                                  /* Return function result                   */
}


