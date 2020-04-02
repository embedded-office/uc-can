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
*                                         CAN DRIVER CODE
*
*                                           Kinetis_Kxx
*
* Filename : drv_can.c
* Version  : V2.42.01
****************************************************************************************************
* Note(s)  : (1) Supports Freescale's Kinetis K Series MCUs as described in various datasheets.
*                This driver has been tested with or should work with the following K Series MCUs :
*
*                    Kinetis K40
*                    Kinetis K60
*                    Kinetis K64
*                    Kinetis K70
****************************************************************************************************
*/

/*
****************************************************************************************************
*                                             INCLUDES
****************************************************************************************************
*/

#include "drv_def.h"
#include "can_bsp.h"
#include "drv_can_reg.h"
#include "drv_can.h"
#include "can_bus.h"


/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/


/*
****************************************************************************************************
*                                    CR REGISTER DEFAULT MASK
*
* Description : This value defines the initial state of the CR register:
*               - bus off interrupt disabled
*               - error interrupt disabled
*               - clk source is the system clock
*               - loop back disabled
*               - just one sample is used
*               - atom. recovering from bus off enabled
*               - timer sync feature disabled
*               - lowest number buffer is transmitted first
*               - normal active operation mode
****************************************************************************************************
*/

#define  KXX_CAN_CR_DEF_MASK (KXX_CAN_CLK_SRC_SYSCLK     + \
                              KXX_CAN_LBUF_LOWBUF_FIRST)


/*
****************************************************************************************************
*                                              MACROS
****************************************************************************************************
*/


/*
****************************************************************************************************
*                                         ERROR REGISTER
*
* Description : This macro writes the given error code to the global error variable.
****************************************************************************************************
*/

#define  ErrRegister(drvIdent, errorcode) (DrvError = errorcode)


/*
****************************************************************************************************
*                                            LOCAL DATA
****************************************************************************************************
*/


/*
****************************************************************************************************
*                                     DRIVER IDENTIFICATION CODE
*
*           This constant holds the unique driver identification code.
****************************************************************************************************
*/

static  const  CPU_INT32U DrvIdent = 0x243F1A01u;


/*
****************************************************************************************************
*                                         DRIVER ERROR CODE
*
* Description : This variable holds the detailed error code if an error is detected.
****************************************************************************************************
*/

static  CPU_INT16U  DrvError;


/*
****************************************************************************************************
*                                           GLOBAL DATA
****************************************************************************************************
*/


/*
****************************************************************************************************
*                                       DEVICE RUNTIME DATA
*
* Description : This variable holds the device runtime data.
****************************************************************************************************
*/

KXX_CAN_DATA  CanData[KXX_CAN_DEV_N] = {{0u}};


/*
****************************************************************************************************
*                                            FUNCTIONS
****************************************************************************************************
*/


/*
****************************************************************************************************
*                                       CAN INITIALIZATION
*
* Description : Initializes the CAN device with the given device name.
*
* Argument(s) : arg        the CAN device name
*
* Caller(s)   : Application.
*
* Return(s)   : error code (0 if OK, -1 if an error occurred)
*
* Note(s)     : none.
****************************************************************************************************
*/

CPU_INT16S  KXX_CAN_Init (CPU_INT32U  arg)
{
              KXX_CAN_REG   *reg;                               /* Local: Pointer to CAN module                         */
              KXX_CAN_DATA  *data;                              /* Local: Pointer to CAN device data                    */
    volatile  CPU_INT32U     dummy;                             /* Local: Dummy variable                                */
              CPU_INT16U     dev;                               /* Local: Loop variable over devices                    */
              CPU_INT16U     i;                                 /* Local: Loop variable                                 */
              CPU_INT16U     j;                                 /* Local: Loop variable                                 */
              CPU_INT16S     error;                             /* Local: Function result                               */
                                                                /* ---------------------------------------------------- */

#if KXX_CAN_ARG_CHK_CFG > 0u                                    /* Checking arguments (if enabled)                      */
    if (arg >= KXX_CAN_DEV_N) {                                 /* can dev out of range?                                */
        ErrRegister((CPU_INT16U)DrvIdent,
                    KXX_CAN_INIT_ERR);
        return (-1);
    }
#endif                                                          /* ---------------------------------------------------- */
    DrvError = KXX_CAN_NO_ERR;                                  /* Set driver error to no error                         */
    dev      = (CPU_INT16U)arg;                                 /* take the channl ID out of the arg.                   */
                                                                /* ---------------------------------------------------- */
                                                                /* Set the base register to the current                 */
                                                                /* device, set interrupt vector table base              */
    data = &CanData[dev];                                       /* set driver data pointer                              */

    reg = KXX_CAN_BSP_Init(dev);

                                                                /* ---------------------------------------------------- */
                                                                /* Initialize parameter struct of the                   */
    data->Base            = (CPU_INT32U)reg;                    /* current device                                       */
    data->Status          = 0u;
    data->Baudrate        = KXX_CAN_DEF_BAUD;
    data->SamplePoint     = KXX_CAN_DEF_SP;
    data->ResynchJumpWith = KXX_CAN_DEF_RJW;
    data->Filter          = 0u;
    data->TxBufAvail      = KXX_CAN_TX_MSG_BUF_MASK;
                                                                /* Calculate the values for the timing                  */
    error = KXX_CAN_CalcTimingReg(data);                        /* register with the parameter setting                  */
    if (error != KXX_CAN_NO_ERR) {                              /* If an error occured                                  */
        DrvError = KXX_CAN_INIT_ERR;                            /* set driver error to init error                       */
        return -1;
    }                                                           /* ---------------------------------------------------- */

    reg->MCR.R  = 0x5000000Fu;                                  /* Enter freeze mode                                    */
    while ((reg->MCR.R & 0x01000000u) == 0u) {};                /* wait until freeze mode is entered                    */

    reg->MCR.B.MAXMB   = KXX_CAN_N_MSG_BUFF-1u;                 /* set max number of MBs                                */
    reg->MCR.B.RFEN    = 1u;                                    /* enable RX fifo                                       */
    reg->MCR.B.IRMQ    = 1u;                                    /* enable individual Rx masking                         */
    reg->MCR.B.SRXDIS  = 1u;                                    /* disable self reception                               */
                                                                /* ---------------------------------------------------- */
    reg->CR.R = ((data->PRESDIV << 24u) +                       /* Prescale devision factor                             */
                 (data->RJW     << 22u) +                       /* Resync jump width                                    */
                 (data->PSEG1   << 19u) +                       /* Phase segment 1                                      */
                 (data->PSEG2   << 16u) +                       /* Phase segment 2                                      */
                  data->PROPSEG        +                        /* Propagation segment.                                 */
                  KXX_CAN_CR_DEF_MASK);                         /* default control settings                             */
                                                                /* ---------------------------------------------------- */

    reg->IMASK1        = 0u;                                    /* Disable all interrupts                               */
    reg->IFLAG1        = 0xFFFFFFFFu;                           /* Clear all interrupt flags                            */

    reg->ESR1.B.BOFFINT = 1u;                                   /* clear pening bus off interrupt                       */
    reg->ESR1.B.ERRINT  = 1u;                                   /* clear error interrupt                                */
    reg->CR.B.BOFFREC   = 1u;                                   /* disable automatic recovery                           */
                                                                /* ---------------------------------------------------- */
                                                                /* initialize RX Fifo                                   */

    reg->CTRL2 = 0x02000000u;                                   /* Select 24 ID filter elements                         */
    reg->BUF[0u].CS.R    = KXX_CAN_BUFF_NOT_ACTIVE;
    reg->BUF[0u].ID      = 0u;
    for (i = 0u; i < 8u; i++) {
        reg->BUF[0u].Data[i] = 0u;
    }
    for (i = 6; i <KXX_CAN_N_MSG_BUFF; i++) {                   /* clear rest of buffers/ID Filter                      */
        reg->BUF[i].CS.R = 0u;
        reg->BUF[i].ID   = 0u;
        for (j = 0u; j < 8u; j++) {
            reg->BUF[i].Data[j] = 0u;
        }
    }
    reg->RXFGMASK = 0u;                                         /* set all masks to don't care                          */
    for (i = 0u; i < 16u; i++) {
        reg->RXIMR[i] = 0u;
    }
                                                                /* Initialize tx message buffer                         */
    reg->BUF[KXX_CAN_TX_MSG_BUF0].CS.R = KXX_CAN_BUFF_NOT_ACTIVE;
    reg->BUF[KXX_CAN_TX_MSG_BUF1].CS.R = KXX_CAN_BUFF_NOT_ACTIVE;
    reg->BUF[KXX_CAN_TX_MSG_BUF2].CS.R = KXX_CAN_BUFF_NOT_ACTIVE;
                                                                /* Activate TX message buffer                           */
    reg->BUF[KXX_CAN_TX_MSG_BUF0].CS.B.CODE = KXX_CAN_TX_BUFF_NOT_READY;
    reg->BUF[KXX_CAN_TX_MSG_BUF1].CS.B.CODE = KXX_CAN_TX_BUFF_NOT_READY;
    reg->BUF[KXX_CAN_TX_MSG_BUF2].CS.B.CODE = KXX_CAN_TX_BUFF_NOT_READY;

#if KXX_CAN_RX_INTERRUPT_EN > 0u
                                                                /* setup and enable INTC for RX interrupts              */
    KXX_BSP_EnableRxIrq (dev);
                                                                /* setup and enable INTC for RX interrupts              */
    reg->IMASK1 |= 0x20u;                                       /* Set the interrupt enable bits of the                 */
                                                                /* receive buffers.                                     */
#endif

#if KXX_CAN_RX_INTERRUPT_EN > 0u
                                                                /* Set the interrupt enable bit                         */
    reg->IMASK1 |= (1u << KXX_CAN_TX_MSG_BUF0);
    reg->IMASK1 |= (1u << KXX_CAN_TX_MSG_BUF1);
    reg->IMASK1 |= (1u << KXX_CAN_TX_MSG_BUF2);
#endif

#if KXX_CAN_NS_INTERRUPT_EN > 0u
                                                                /* setup and enable INTC for NS interrupts              */
    KXX_BSP_EnableNSIrq (dev);

    reg->CR.B.BOFFMSK = 1u;                                     /* ENABLE NS INTERRUPTS                                 */
    reg->CR.B.ERRMSK  = 1u;
#endif
                                                                /* ---------------------------------------------------- */
    dummy             = reg->TIMER;                             /* dummy read to the timer register                     */
    reg->MCR.B.MDIS   = 1u;                                     /* Disable module                                       */
    reg->MCR.B.FRZ    = 0u;                                     /* exit the freeze mode                                 */
    reg->MCR.B.HALT   = 0u;                                     /* ---------------------------------------------------- */

    return (0u);
}


/*
****************************************************************************************************
*                                       OPEN CAN DEVICE
*
* Description : Unlocks the device, i.e. IoCtl/Read/Write-function will take effect.
*
* Argument(s) : drv        the bus node name which must be used by the interrupt routine to access the
*                          CAN bus layer.
*
*               devName    the CAN device name
*
*               mode       the mode in which the CAN device will be used
*
* Caller(s)  : Application.
*
* Return(s)  :  the device identifier for further access or -1 if an error occurred
*
* Note(s)    : none.
****************************************************************************************************
*/

CPU_INT16S  KXX_CAN_Open (CPU_INT16S  drv,
                          CPU_INT32U  devName,
                          CPU_INT16U  mode)
{                                                               /* ---------------------------------------------------- */
    KXX_CAN_DATA  *data;                                        /* Local: Pointer to CAN device data                    */
    CPU_INT16S     result;                                      /* Local: Function result                               */
    CPU_SR_ALLOC();                                             /* Allocate storage for CPU status register             */
                                                                /* ---------------------------------------------------- */

    (void)drv;                                                  /* prevent compiler warning                             */
                                                                /* ---------------------------------------------------- */
    result = -1;

#if KXX_CAN_ARG_CHK_CFG > 0u                                    /* Checking arguments (if enabled)                      */
    if (devName >= KXX_CAN_DEV_N) {                             /* devName out of range?                                */
        ErrRegister((CPU_INT16U)DrvIdent,
                    KXX_CAN_BUS_ERR);
        return (result);
    }
    if (mode != DEV_RW) {                                       /* mode not supported?                                  */
        ErrRegister((CPU_INT16U)DrvIdent,
                    KXX_CAN_MODE_ERR);
        return (result);
    }
#endif                                                          /* ---------------------------------------------------- */
    data = &CanData[devName];                                   /* set driver data pointer                              */
    CPU_CRITICAL_ENTER();                                       /* Enter critical section                               */
                                                                /* Check, that CAN device is not open                   */
    if ((data->Status & KXX_CAN_OPEN) == 0u) {
        data->Status |= KXX_CAN_OPEN;                           /* Mark CAN device to be opened                         */
        CPU_CRITICAL_EXIT();                                    /* Leave critical section                               */

#if ((KXX_CAN_RX_INTERRUPT_EN > 0u) || \
     (KXX_CAN_TX_INTERRUPT_EN > 0u) || \
     (KXX_CAN_NS_INTERRUPT_EN > 0u))
                                                                /* store the received Node Id for the irqs              */
        KXX_BSP_SetDevIds((CPU_INT08U)drv,
                          (CPU_INT08U)devName);
#endif
        result = (CPU_INT16S)devName;                           /* Set return value to devName                          */
    } else {                                                    /* ---------------------------------------------------- */
        CPU_CRITICAL_EXIT();                                    /* Leave critical section                               */
        ErrRegister((CPU_INT16U)DrvIdent,
                    KXX_CAN_OPEN_ERR);
    }                                                           /* ---------------------------------------------------- */

    return (result);                                            /* return function result                               */
}


/*
****************************************************************************************************
*                                       CLOSE CAN DEVICE
*
* Description : Locks the device, i.e. IoCtl/Read/Write-function will have no effect.
*
* Argument(s) : devId      the device identifier, returned by XXXCANOpen()
*
* Caller(s)   : Application.
*
* Return(s)   : error code (0 if OK, -1 if an error occurred)
*
* Note(s)     : none.
****************************************************************************************************
*/

CPU_INT16S  KXX_CAN_Close (CPU_INT16S  devId)
{                                                               /* ---------------------------------------------------- */
    CPU_INT16S  result;                                         /* Local: Function result                               */
    CPU_SR_ALLOC();                                             /* Allocate storage for CPU status register             */
                                                                /* ---------------------------------------------------- */

    result = -1;

#if KXX_CAN_ARG_CHK_CFG > 0u                                    /* Checking arguments (if enabled)                      */
    if ((devId <  0) ||                                         /* devId out of range?                                  */
        (devId >= KXX_CAN_DEV_N)) {
        ErrRegister((CPU_INT16U)DrvIdent,
                    KXX_CAN_BUS_ERR);
        return (-1);
    }
#endif                                                          /* ---------------------------------------------------- */
    CPU_CRITICAL_ENTER();                                       /* Enter critical section                               */
    if (CanData[devId].Status & KXX_CAN_OPEN) {                 /* see, if CAN device is opened                         */
        CanData[devId].Status = KXX_CAN_IDLE;                   /* yes: reset CAN device status                         */
        CPU_CRITICAL_EXIT();                                    /* Leave critical section                               */
        result = KXX_CAN_NO_ERR;                                /* Ok, device is closed                                 */
    } else {                                                    /* ---------------------------------------------------- */
        CPU_CRITICAL_EXIT();                                    /* Leave critical section                               */
        ErrRegister((CPU_INT16U)DrvIdent,                       /* set device close error                               */
                    KXX_CAN_CLOSE_ERR);
    }                                                           /* ---------------------------------------------------- */

    return (result);                                            /* return function result                               */
}


/*
****************************************************************************************************
*                                       CAN I/O CONTROL
*
* Description : This function performs a special action on the opened device. The function code func
*               defines what the caller wants to do. Description of function codes as defined in
*               header file.
*
* Argument(s) : devId      device identifier, returned by XXXCANOpen()
*
*               func       function code
*
*               argp       argument list, specific to the function code
*
* Caller(s)   : Application.
*
* Return(s)   : error code (0 if OK, -1 if an error occurred)
*
* Note(s)     : none.
****************************************************************************************************
*/

CPU_INT16S  KXX_CAN_IoCtl (CPU_INT16S   devId,
                           CPU_INT16U   func,
                           void        *argp)
{                                                               /* ---------------------------------------------------- */
    KXX_CAN_REG   *reg;                                         /* Local: Pointer to CAN module                         */
    KXX_CAN_DATA  *data;                                        /* Local: Pointer to CAN device data                    */
    CPU_INT32U    *buf;                                         /* Local: temporary buf pointer                         */
    CPU_INT32U     tmp;                                         /* Local: temporary value                               */
    CPU_INT32U     shft;                                        /* Local: temporary value                               */
    CPU_INT32U     canId;                                       /* Local: CAN Identifier                                */
    CPU_INT32U     mask;                                        /* Local: Mask for identifier caluclations              */
    CPU_INT16U     i;                                           /* Local: Loop variable                                 */
    CPU_INT16S     result;                                      /* Local: Function result                               */
    CPU_SR_ALLOC();                                             /* Allocate storage for CPU status register             */
                                                                /* ---------------------------------------------------- */

    result = KXX_CAN_NO_ERR;

#if KXX_CAN_ARG_CHK_CFG > 0u                                    /* Checking arguments (if enabled)                      */
    if ((devId <  0) ||                                         /* devId out of range?                                  */
        (devId >= KXX_CAN_DEV_N)) {
        ErrRegister((CPU_INT16U)DrvIdent,
                    KXX_CAN_BUS_ERR);
        return (-1);
    }
#endif                                                          /* ---------------------------------------------------- */
    reg  = (KXX_CAN_REG *)CanData[devId].Base;                  /* Set pointer to CAN module                            */
    data = &CanData[devId];                                     /* Set pointer to CAN device data                       */

    if ((data->Status & KXX_CAN_OPEN) == 0) {                   /* see, if CAN device is closed                         */
        ErrRegister((CPU_INT16U)DrvIdent,
                    KXX_CAN_OPEN_ERR);                          /* yes: set device not opened error                     */
        return (-1);
    }

    CPU_CRITICAL_ENTER();                                       /* Enter critical section                               */
                                                                /* ---------------------------------------------------- */
    switch (func) {                                             /* select: function code                                */
        case IO_KXX_CAN_GET_IDENT:                              /* GET IDENT                                            */
             (*(CPU_INT32U*)argp) = DrvIdent;                   /* return driver ident code                             */
             break;


        case IO_KXX_CAN_GET_ERRNO:                              /* GET ERRORCODE                                        */
             (*(CPU_INT16U*)argp) = DrvError;                   /* return last detected errorcode                       */
             break;


        case IO_KXX_CAN_GET_DRVNAME:                            /* GET DRIVER NAME                                      */
             (*(CPU_INT08U**)argp) = (CPU_INT08U*)              /* return human readable driver name                    */
                                      KXX_CAN_NAME;
             break;


        case IO_KXX_CAN_SET_BAUDRATE:                           /* SET BAUDRATE                                         */
             tmp = reg->MCR.R;                                  /* store MCR value                                      */
             reg->MCR.B.HALT   = 1u;                            /* Enter the freeze mode                                */
             reg->MCR.B.FRZ    = 1u;
             reg->MCR.B.MDIS   = 0u;                            /* Enable module                                        */
             while ((reg->MCR.R & 0x01000000u) == 0u) {};       /* wait until freeze mode is entered                    */

             data->Baudrate = *(CPU_INT32U *)argp;              /* Set baudrate parameter to given argument             */
                                                                /* and calculate the values for the timing              */
                                                                /* register with the parameter setting                  */
             if (KXX_CAN_CalcTimingReg(data) != KXX_CAN_NO_ERR) {
                 ErrRegister((CPU_INT16U)DrvIdent,
                             KXX_CAN_ARG_ERR);                  /* set errorcode if an error occured and                */
                 result = -1;                                   /* indicate error on function result.                   */
             } else {                                           /* set can timing registers if timing                   */
                                                                /* calculation success.                                 */
                 mask  = (reg->CR.R & KXX_CAN_CR_MASK);
                 mask |= ((data->PRESDIV << 24u) |              /* Prescale devision factor                             */
                          (data->RJW     << 22u) |              /* Resync jump width                                    */
                          (data->PSEG1   << 19u) |              /* Phase segment 1                                      */
                          (data->PSEG2   << 16u) |              /* Phase segment 2                                      */
                           data->PROPSEG);                      /* Propagation segment.                                 */
                 reg->CR.R = mask;
             }
             reg->MCR.R = tmp;                                  /* restore previous mode                                */
             break;


        case IO_KXX_CAN_START:                                  /* START                                                */
             reg->MCR.B.MDIS = 0u;
             break;


        case IO_KXX_CAN_STOP:                                   /* STOP                                                 */
             reg->MCR.B.MDIS = 1u;
             break;


        case IO_KXX_CAN_RX_STANDARD:                            /* STANDARD RX IDENTIFIER                               */
             tmp = reg->MCR.R;                                  /* store MCR value                                      */
             reg->MCR.B.HALT   = 1u;                            /* Enter the freeze mode                                */
             reg->MCR.B.FRZ    = 1u;
             reg->MCR.B.MDIS   = 0u;                            /* Enable module                                        */
             while ((reg->MCR.R & 0x01000000u) == 0u) {};       /* wait until freeze mode is entered                    */

             buf = (CPU_INT32U *)&reg->BUF[6u].CS.R;
             for (i = 0u; i < 24u; i++) {                       /* set in configured 24 ID filters                      */
                 *buf++ &= ~0x40000000u;                        /* clear IDE flag                                       */
             }
             for (i = 0u; i < 16u; i++) {                       /* set in individual mask registers                     */
                 reg->RXIMR[i] |= 0x40000000u;
             }
             reg->RXFGMASK |= 0x40000000u;
             reg->MCR.R = tmp;                                  /* restore previous mode                                */
             break;


        case IO_KXX_CAN_RX_EXTENDED:                            /* EXTENDED RX IDENTIFIER                               */
             tmp = reg->MCR.R;                                  /* store MCR value                                      */
             reg->MCR.B.HALT   = 1u;                            /* Enter the freeze mode                                */
             reg->MCR.B.FRZ    = 1u;
             reg->MCR.B.MDIS   = 0u;                            /* Enable module                                        */
             while ((reg->MCR.R & 0x01000000u) == 0u) {};       /* wait until freeze mode is entered                    */

             buf = (CPU_INT32U *)&reg->BUF[6u].CS.R;
             for (i = 0u; i < 24u; i++) {                       /* set in configured 24 ID filters                      */
                 *buf++ |= 0x40000000u;                         /* set IDE flag                                         */
             }
             for (i = 0u; i < 16u; i++) {                       /* set in individual mask registers                     */
                 reg->RXIMR[i] |= 0x40000000u;
             }
             reg->RXFGMASK |= 0x40000000u;
             reg->MCR.R = tmp;                                  /* restore previous mode                                */
             break;


        case IO_KXX_CAN_TX_READY:
             tmp = ((reg->IFLAG1 | data->TxBufAvail) &          /* check availability of buffers, so that               */
                    KXX_CAN_TX_MSG_BUF_MASK);                   /* no higher priority messages than those               */
                                                                /* already scheduled are released                       */
             if ((tmp == ((1u << KXX_CAN_TX_MSG_BUF0) | (1u << KXX_CAN_TX_MSG_BUF1) | (1u << KXX_CAN_TX_MSG_BUF2))) ||
                 (tmp == ((1u << KXX_CAN_TX_MSG_BUF1) | (1u << KXX_CAN_TX_MSG_BUF2))) ||
                 (tmp ==  (1u << KXX_CAN_TX_MSG_BUF2))) {
                 *((CPU_BOOLEAN *)argp) = 1u;                   /* set return value to true                             */
             } else {
                 *((CPU_BOOLEAN *)argp) = 0u;                   /* Otherwise                                            */
             }                                                  /* set return value to false.                           */
             break;


        case IO_KXX_CAN_GET_NODE_STATUS:                        /*             GET NODE STATUS                          */
             if ((reg->ESR1.B.FLTCONF & 0x1u) != 0u) {          /* Error Status Bit                                     */
                 (*(CPU_INT08U*)argp) = (CPU_INT08U)1u;
             }
             if ((reg->ESR1.B.FLTCONF & 0x2u) != 0u) {          /* Bus Status Bit                                       */
                 (*(CPU_INT08U*)argp) = (CPU_INT08U)2u;
             }
             if ((reg->ESR1.B.FLTCONF & 0x3u) == 0u) {          /* Error and Bus Status Bit                             */
                 (*(CPU_INT08U*)argp) = (CPU_INT08U)0u;
             }
             break;


        case IO_KXX_CAN_SET_RX_FILTER_1:                        /* SET RX FILTER 1                                      */
             mask  = ((CPU_INT32U*)argp)[0u];
             canId = ((CPU_INT32U*)argp)[1u];

             tmp = reg->MCR.R;                                  /* store MCR value                                      */
             reg->MCR.B.HALT   = 1u;                            /* Enter the freeze mode                                */
             reg->MCR.B.FRZ    = 1u;
             reg->MCR.B.MDIS   = 0u;                            /* Enable module                                        */
             while ((reg->MCR.R & 0x01000000u) == 0u) {};       /* wait until freeze mode is entered                    */

             if (canId > KXX_CAN_STD_ID_MASK) {
                 shft = 1u;                                     /* use bit 0..28                                        */
             } else {
                 shft = 19u;                                    /* use bit 0..10                                        */
             }
             buf = (CPU_INT32U *)&reg->BUF[6u].CS.R;
             for (i = 0u; i < 24u; i++) {                       /* set in configured 24 ID filters                      */
                 *buf++ = (canId << shft);
             }
             for (i = 0u; i < 16u; i++) {                       /* set in individual mask registers                     */
                 reg->RXIMR[i] = (mask << shft);
             }

             reg->RXFGMASK = (mask << shft);
             reg->MCR.R = tmp;                                  /* restore previous mode                                */
             break;


        case IO_KXX_CAN_SET_RX_FILTER_2:                        /* SET RX FILTER 2                                      */
             mask  = ((CPU_INT32U*)argp)[0u];
             canId = ((CPU_INT32U*)argp)[1u];

             tmp = reg->MCR.R;                                  /* store MCR value                                      */
             reg->MCR.B.HALT   = 1u;                            /* Enter the freeze mode                                */
             reg->MCR.B.FRZ    = 1u;
             reg->MCR.B.MDIS   = 0u;                            /* Enable module                                        */
             while ((reg->MCR.R & 0x01000000u) == 0u) {};       /* wait until freeze mode is entered                    */

             if (canId > KXX_CAN_STD_ID_MASK) {
                 shft = 1u;                                     /* use bit 0..28                                        */
             } else {
                 shft = 19u;                                    /* use bit 0..10                                        */
             }
                                                                /* set in next ID filter                                */
             buf = (CPU_INT32U *)&reg->BUF[6+data->Filter].CS.R;
             *buf = (canId << shft);
                                                                /* set in individual mask registers                     */
             reg->RXIMR[data->Filter] = (mask << shft);
             data->Filter++;
             if (data->Filter >= 16u) {
                 data->Filter = 0u;
             }
             reg->MCR.R = tmp;                                  /* restore previous mode                                */
             break;


        default:                                                /*           WRONG FUNCTION CODE                        */
             result = -1;                                       /* indicate error on function result                    */
             break;
    }
    CPU_CRITICAL_EXIT();                                        /* Leave critical section                               */
                                                                /* ---------------------------------------------------- */
    if (result < 0) {                                           /* If an error occured,                                 */
        ErrRegister((CPU_INT16U)DrvIdent,                       /* register error in error management                   */
                    KXX_CAN_FUNC_ERR);
    }
                                                                /* ---------------------------------------------------- */
    return (result);                                            /* Return function result                               */
}


/*
****************************************************************************************************
*                                       CAN READ DATA
*
* Description : Read a received CAN frame from a message buffer. The buffer must have space for only
*               one CAN frame.
*
* Argument(s) : devId      device identifier, returned by XXXCANOpen()
*
*               buf        pointer to CAN frame
*
*               size       length of CAN frame memory
*
* Caller(s)   : Application.
*
* Return(s)   : error code (CAN frame DLC if OK, -1 if an error occurred)
*
* Note(s)     : none.
****************************************************************************************************
*/

CPU_INT16S  KXX_CAN_Read (CPU_INT16S   devId,
                          CPU_INT08U  *buf,
                          CPU_INT16U   size)
{                                                               /* ---------------------------------------------------- */
    KXX_CAN_REG   *reg;                                         /* Local: Pointer to CAN module                         */
    KXX_CAN_DATA  *data;                                        /* Local: Pointer to CAN device data                    */
    KXX_CAN_FRM   *frm;                                         /* Local: Pointer to can frame                          */
    CPU_INT16S     result;                                      /* Local: Function result                               */


    data = &CanData[devId];                                     /* Set pointer to CAN device data                       */
                                                                /* ---------------------------------------------------- */
#if KXX_CAN_ARG_CHK_CFG > 0u                                    /* Checking arguments (if enabled)                      */
    if ((devId <  0) ||                                         /* devId out of range?                                  */
        (devId >= KXX_CAN_DEV_N)) {
        ErrRegister((CPU_INT16U)DrvIdent,
                    KXX_CAN_BUS_ERR);
        return (-1);
    }
    if (buf == (CPU_INT08U *)0u) {                              /* valid buffer pointer?                                */
        ErrRegister((CPU_INT16U)DrvIdent,
                    KXX_CAN_ARG_ERR);
        return (-1);
    }
    if (size != sizeof(KXX_CAN_FRM)) {                          /* size in range?                                       */
        ErrRegister((CPU_INT16U)DrvIdent,
                    KXX_CAN_ARG_ERR);
        return (-1);
    }
    if (!(data->Status & KXX_CAN_OPEN)) {                       /* see, if CAN device is not opened                     */
        ErrRegister((CPU_INT16U)DrvIdent,
                    KXX_CAN_OPEN_ERR);                          /* yes: set device not opened error                     */
        return (-1);
    }                                                           /* ---------------------------------------------------- */
#endif

    reg = (KXX_CAN_REG *)CanData[devId].Base;                   /* Set pointer to CAN module                            */
    frm = (KXX_CAN_FRM *)buf;                                   /* Set pointer to can frame                             */
                                                                /* ---------------------------------------------------- */

    if ((reg->IFLAG1 & 0x20u) != 0u) {                          /* see, if a CAN frame is received                      */
        frm->DLC = (CPU_INT08U)reg->BUF[0u].CS.B.LENGTH;        /* get number of received bytes                         */

        frm->Data[0] = reg->BUF[0u].Data[3u];                   /* get received data bytes                              */
        frm->Data[1] = reg->BUF[0u].Data[2u];
        frm->Data[2] = reg->BUF[0u].Data[1u];
        frm->Data[3] = reg->BUF[0u].Data[0u];
        frm->Data[4] = reg->BUF[0u].Data[7u];
        frm->Data[5] = reg->BUF[0u].Data[6u];
        frm->Data[6] = reg->BUF[0u].Data[5u];
        frm->Data[7] = reg->BUF[0u].Data[4u];
                                                                /* ---------------------------------------------------- */
                                                                /* get identifier format and extract                    */
        if (reg->BUF[0u].CS.B.IDE ==                            /* identifier from the message buffer                   */
            KXX_CAN_MB_ID_STANDARD) {                           /* - standard identifier resp.                          */
            frm->Identifier  = (reg->BUF[0u].ID) >> 18u;
        } else {                                                /* - extended identifier                                */
            frm->Identifier  = reg->BUF[0u].ID;
            frm->Identifier |= KXX_CAN_EXT_ID_FLAG;
        }                                                       /* ---------------------------------------------------- */
        reg->IFLAG1 = 0x20u;                                    /* reset interrupt flag                                 */

        result = sizeof(KXX_CAN_FRM);                           /* Set function result to size of CANFRM                */
                                                                /* ---------------------------------------------------- */
    } else {                                                    /* No message received,                                 */
        frm->Identifier                = 0u;
        frm->DLC                       = 0u;
        *(CPU_INT32U*)(&frm->Data[0u]) = 0u;
        *(CPU_INT32U*)(&frm->Data[4u]) = 0u;
        result = -1;
    }
                                                                /* ---------------------------------------------------- */
    return (result);                                            /* Return function result                               */
}


/*
****************************************************************************************************
*                                   CAN WRITE DATA
*
* Description : Write a CAN frame to a message buffer. The buffer must contain only one CAN frame.
*
* Argument(s) : devId      device identifier, returned by XXXCANOpen()
*
*               buf        pointer to CAN frame
*
*               size       length of CAN frame memory
*
* Caller(s)   : Application.
*
* Return(s)   : error code (CAN frame DLC if OK, -1 if an error occurred)
*
* Note(s)     : none.
****************************************************************************************************
*/

CPU_INT16S  KXX_CAN_Write (CPU_INT16S   devId,
                           CPU_INT08U  *buf,
                           CPU_INT16U   size)
 {                                                              /* ---------------------------------------------------- */
    KXX_CAN_REG   *reg;                                         /* Local: Pointer to CAN module                         */
    KXX_CAN_DATA  *data;                                        /* Local: Pointer to CAN device data                    */
    KXX_CAN_FRM   *frm;                                         /* Local: Pointer to can frame                          */
    CPU_INT32U     canId;                                       /* Local: CAN Identifier                                */
    CPU_INT32U     mb;                                          /* Local: message buffer                                */
    CPU_INT16S     result;                                      /* Local: Function result                               */


    data = &CanData[devId];                                     /* Set pointer to CAN device data                       */

                                                                /* ---------------------------------------------------- */
#if KXX_CAN_ARG_CHK_CFG > 0u                                    /* Checking arguments (if enabled)                      */
    if ((devId <  0) ||                                         /* devId out of range?                                  */
        (devId >= KXX_CAN_DEV_N)) {
        ErrRegister((CPU_INT16U)DrvIdent,
                    KXX_CAN_BUS_ERR);
        return (-1);
    }
    if (buf == (CPU_INT08U *)0u) {                              /* valid buffer pointer?                                */
        ErrRegister((CPU_INT16U)DrvIdent,
                    KXX_CAN_ARG_ERR);
        return (-1);
    }
    if (size != sizeof(KXX_CAN_FRM)) {                          /* size in range?                                       */
        ErrRegister((CPU_INT16U)DrvIdent,
                    KXX_CAN_ARG_ERR);
        return (-1);
    }
    if (!(data->Status & KXX_CAN_OPEN)) {                       /* see, if CAN device is not opened                     */
        ErrRegister((CPU_INT16U)DrvIdent,
                    KXX_CAN_OPEN_ERR);                          /* yes: set device not opened error                     */
        return (-1);
    }
#endif                                                          /* ---------------------------------------------------- */

    reg  = (KXX_CAN_REG *)CanData[devId].Base;                  /* Set pointer to CAN module                            */
    frm  = (KXX_CAN_FRM *)buf;                                  /* Set pointer to can frame                             */
                                                                /* ---------------------------------------------------- */
    if (frm->DLC > 8u) {                                        /* Limit DLC to 8 bytes                                 */
        frm->DLC = 8u;
    }                                                           /* ---------------------------------------------------- */
                                                                /* check availabilty of message buffer                  */
    mb = (reg->IFLAG1 | data->TxBufAvail) & KXX_CAN_TX_MSG_BUF_MASK;
    if (mb != 0u) {
                                                                /* get highest prio mb                                  */
        if (mb & (1u << KXX_CAN_TX_MSG_BUF0)) {
            mb = KXX_CAN_TX_MSG_BUF0;
        } else if (mb & (1u << KXX_CAN_TX_MSG_BUF1)) {
            mb = KXX_CAN_TX_MSG_BUF1;
        } else {
            mb = KXX_CAN_TX_MSG_BUF2;
        }
        data->TxBufAvail &= ~(1u << mb);                        /* reset buffer availability                            */
                                                                /* Write the control/status word to hold                */
                                                                /* the transmit buffer inactive and set                 */
        reg->BUF[mb].CS.B.CODE = KXX_CAN_TX_BUFF_NOT_READY;
        reg->BUF[mb].CS.B.LENGTH = frm->DLC;                    /* the length in the control/status word                */
                                                                /* ---------------------------------------------------- */
        canId = frm->Identifier;                                /* Write the ID register                                */
        if (canId > KXX_CAN_STD_ID_MASK) {                      /* depending on the CAN ID                              */
            canId &= KXX_CAN_EXT_ID_MASK;                       /* use bit 0..28                                        */
            reg->BUF[mb].CS.B.IDE = KXX_CAN_MB_ID_EXTENDED;
            reg->BUF[mb].ID = canId;
        } else {
            canId &= KXX_CAN_STD_ID_MASK;                       /* use bit 0..10                                        */
            reg->BUF[mb].ID = (canId << 18u);
            reg->BUF[mb].CS.B.IDE = KXX_CAN_MB_ID_STANDARD;
        }                                                       /* ---------------------------------------------------- */
                                                                /* Write the data bytes into the CAN buffer             */
        buf = (CPU_INT08U *)&(reg->BUF[mb].Data[0u]);
        *buf++ = frm->Data[3u];
        *buf++ = frm->Data[2u];
        *buf++ = frm->Data[1u];
        *buf++ = frm->Data[0u];
        *buf++ = frm->Data[7u];
        *buf++ = frm->Data[6u];
        *buf++ = frm->Data[5u];
        *buf   = frm->Data[4u];
                                                                /* Set the control/status word to                       */
                                                                /* TX_BUFFER_SEND                                       */
        reg->BUF[mb].CS.B.CODE = KXX_CAN_TX_BUFF_SEND;

        result = (CPU_INT16S)size;                              /* Set return value to no error                         */
    } else {
         result = -1;
    }
    return (result);                                            /* Return function result                               */
}
