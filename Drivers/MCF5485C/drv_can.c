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
#include "drv_can.h"                            /* Driver function prototypes and data typedefs   */
#include "can_bsp.h"                            /* Driver board support definitions and functions */
#include "can_bus.h"

/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      FLEXCAN CR REGISTER DEFAULT MASK
* \ingroup  MCF5485C_CAN
*
*           This define the initial state of the FlexCan CR register.
*             bus off interrupt disabled
*             error interrupt disabled
*             clk source is the system clock
*             Loop back disabled
*             just one sample are used
*             atom. recovering from bus off enabled
*             timer sync feature disabled
*             lowest number buffer is transmitted first
*             normal active operation mode
*/
/*------------------------------------------------------------------------------------------------*/
#define MCF5485C_CAN_CR_DEF_MASK (MCF5485C_CAN_CLK_SRC_SYSCLK     + \
                                 MCF5485C_CAN_LBUF_LOWBUF_FIRST)
/*
****************************************************************************************************
*                                              MACROS
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ERROR LOGIN
* \ingroup  MCF5485C_CAN
*
*           This macro is a dummy function for future error management.
*/
/*------------------------------------------------------------------------------------------------*/
#define ErrLogin(drvIdent) (MCF5485C_CAN_NO_ERR)

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ERROR REGISTER
* \ingroup  MCF5485C_CAN
*
*           This macro writes the given errorcode to the global error variable.
*/
/*------------------------------------------------------------------------------------------------*/
#define ErrRegister(drvIdent, errorcode) (DrvError = errorcode)

/*
****************************************************************************************************
*                                            LOCAL DATA
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER IDENTIFICATION
* \ingroup  MCF5485C_CAN
*
*           This constant variable holds the unique driver identification code.
*/
/*------------------------------------------------------------------------------------------------*/
static const CPU_INT32U DrvIdent = 0x243F1A01;  /* version 2.4 code 1A01 */

/*
****************************************************************************************************
*                                             GLOBAL DATA
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MBAR ADRESS
* \ingroup  MCF5485C_CAN
*
*           This variable holds the address from the MBAR as defined in the linker command file.
*/
/*------------------------------------------------------------------------------------------------*/
extern CPU_INT08U __MBAR[];                           /*!< MBAR from linker command file         */

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER ERRORCODE
* \ingroup  MCF5485C_CAN
*
*           This variable holds the detailed errorcode, if an error is detected.
*/
/*------------------------------------------------------------------------------------------------*/
static CPU_INT16U DrvError;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER PARAMETER
* \ingroup  MCF5485C_CAN
*
*           Global bus parameters of the CAN modules.
*/
/*------------------------------------------------------------------------------------------------*/
static const MCF5485C_CAN_PARA CanPara[MCF5485C_CAN_DEV_N] = {
    {MCF5485C_FLEXCAN_A_BUFF_IRQP_CFG, MCF5485C_FLEXCAN_A_BUS_OFF_IRQP_CFG, MCF5485C_FLEXCAN_A_ERROR_IRQP_CFG},
    {MCF5485C_FLEXCAN_B_BUFF_IRQP_CFG, MCF5485C_FLEXCAN_B_BUS_OFF_IRQP_CFG, MCF5485C_FLEXCAN_B_ERROR_IRQP_CFG}
};

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER DATA
* \ingroup  MCF5485C_CAN
*
*           Global bus data of the CAN modules.
*/
/*------------------------------------------------------------------------------------------------*/
MCF5485C_CAN_DATA CanData[MCF5485C_CAN_DEV_N] = {{0}};

/*
****************************************************************************************************
*                                            FUNCTIONS
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MCF5485C CAN INITIALIZATION
* \ingroup  MCF5485C_CAN
*
*           Initializes the FlexCAN module.
*
* \param    arg               CAN bus id
*
* \return   Errorcode
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MCF5485CCANInit (CPU_INT32U arg)
{
    volatile MCF5485C_CAN_REG  *reg;                  /* Local: Pointer to CAN module             */
             MCF5485C_CAN_DATA *data;                 /* Local: Pointer to CAN device data        */
    volatile CPU_INT32U        dummy;                 /* Local: Dummy variable                    */
             MCF5485C_CAN_PARA *para;                 /* Local: Pointer to CAN parameter          */
             CPU_INT16U        dev;                   /* Local: Loop variable over devices        */
             CPU_INT16U        i;                     /* Local: Loop variable                     */
             CPU_INT16S        error;                 /* Local: Function result                   */
             CPU_INT16S        result = -1;           /* Local: Function result                   */
                                                      /*------------------------------------------*/
#if MCF5485C_CAN_ARG_CHK_CFG > 0                       /* Checking arguments (if enabled)          */
    if (arg >= MCF5485C_CAN_DEV_N) {                   /* can dev out of range?                    */
        ErrRegister((CPU_INT16U)DrvIdent, MCF5485C_CAN_INIT_ERR);
        return(result);
    }
#endif                                                /*------------------------------------------*/

    DrvError = MCF5485C_CAN_NO_ERR;                   /* Set driver error to no error             */
    dev      = (CPU_INT16U)arg;                       /* take the channl ID out of the arg.       */
    result = ErrLogin((CPU_INT16U)DrvIdent);          /* Connect to error management              */
                                                      /*------------------------------------------*/
                                                      /* Set the base register to the current     */
                                                      /* device, set interrupt vector table base  */
    data = &CanData[arg];                             /* set driver data pointer                  */
    para = (MCF5485C_CAN_PARA *)&CanPara[arg];        /* Set pointer to CAN parameter             */
    if (dev == MCF5485C_CAN_BUS_A) {                  /* address and set the CAN_TX/CAN_RX pins   */
        reg = (MCF5485C_CAN_REG *)MCF5485C_CAN_A_BASE_ADDRESS;
        CANTX0_REG &= ~CANTX0_MASK;
        CANTX0_REG |= CANTX0_VAL;
        CANRX0_REG &= ~CANRX0_MASK;
        CANRX0_REG |= CANRX0_VAL;

    } else if (dev == MCF5485C_CAN_BUS_B) {
        reg = (MCF5485C_CAN_REG *)MCF5485C_CAN_B_BASE_ADDRESS;
        CANTX1_REG &= ~CANTX1_MASK;
        CANTX1_REG |= CANTX1_VAL;
        CANRX1_REG &= ~CANRX1_MASK;
        CANRX1_REG |= CANRX1_VAL;
    }                                                 /*------------------------------------------*/
                                                      /* Initialize parameter struct of the       */
    data->Base            = (CPU_INT32U)reg;          /* current device                           */
    data->Status          = 0;
    data->Baudrate        = MCF5485C_CAN_DEF_BAUD;
    data->SamplePoint     = MCF5485C_CAN_DEF_SP;
    data->ResynchJumpWith = MCF5485C_CAN_DEF_RJW;
    data->WrRxMB          = 1;                        /* reset write RX message buffer ID         */
    data->WrRxMask        = (1 << 1);                 /* reset write RX message buffer mask       */
    data->RdRxMB          = 1;                        /* reset read RX message buffer ID          */
    data->ID              = 0;                        /* clear last TX-ID                         */
    data->TxTimeout       = 1000000 / data->Baudrate * 1000;

                                                      /* Calculate the values for the timing      */
    error = MCF5485CCANCalcTimingReg(data);           /* register with the parameter setting      */
                                                      /*------------------------------------------*/
    reg->MCR.B.MAXMB  = MCF5485C_CAN_N_MSG_BUF;       /* set max number of MB's                   */
                                                      /*------------------------------------------*/
    reg->CR.R = ((CanData[dev].PRESDIV << 24) +       /* Prescale devision factor                 */
                 (CanData[dev].RJW     << 22) +       /* Resync jump width                        */
                 (CanData[dev].PSEG1   << 19) +       /* Phase segment 1                          */
                 (CanData[dev].PSEG2   << 16) +       /* Phase segment 2                          */
                  CanData[dev].PROPSEG        +       /* Propagation segment.                     */
                  MCF5485C_CAN_CR_DEF_MASK);          /* default control settings                 */
                                                      /*------------------------------------------*/
    reg->RXGMASK.R     = 0x00000000;                  /* Receive every ID in first MsgBuff        */
    reg->RX14MASK.R    = 0xffffffff;                  /* Set buffer14 and buffer15 receive        */
    reg->RX15MASK.R    = 0xffffffff;                  /* mask: Every bit in the ID must match     */
                                                      /*------------------------------------------*/
    reg->IMASK.R       = 0;                           /* Disable all interrupts                   */
    reg->IFLAG.R       = 0xFFFFFFFF;                  /* Clear all interrupt flags                */
    reg->ESR.B.BOFFINT = 1;                           /* clear pening bus off interrupt           */
    reg->ESR.B.ERRINT  = 1;                           /* clear error interrupt                    */
    reg->CR.B.BOFFREC  = 1;                           /* disable automatic recovery               */
                                                      /*------------------------------------------*/
    for (i = 0; i < MCF5485C_CAN_N_MSG_BUF; i++) {    /* Deactivate all message buffers           */
        reg->BUF[i].CS.R = MCF5485C_CAN_BUFFER_NOT_ACTIVE;
        reg->BUF[i].ID   = 0;
        reg->BUF[i].DATA.Word[0] = 0;                 /* clear the message buffers                */
        reg->BUF[i].DATA.Word[1] = 0;
    }                                                 /* -----------------------------------------*/

#if MCF5485C_CAN_NS_INTERRUPT_EN > 0
    if (dev == MCF5485C_CAN_BUS_A) {                  /* setup INTC for NS interrupts             */
        MCF_INTC_ICR49 = para->ErrIrqPrio;
        MCF_INTC_ICR50 = para->BusOffIrqPrio;
        MCF_INTC_IMRH &= ~(1 << (MCF_CAN0_ERROR - 32));/* clear IRQ mask                           */
        MCF_INTC_IMRH &= ~(1 << (MCF_CAN0_BUSOFF - 32));
    } else {
        MCF_INTC_ICR55 = para->ErrIrqPrio;
        MCF_INTC_ICR56 = para->BusOffIrqPrio;
        MCF_INTC_IMRH &= ~(1 << (MCF_CAN1_ERROR - 32));/* clear IRQ mask                           */
        MCF_INTC_IMRH &= ~(1 << (MCF_CAN1_BUSOFF - 32));
    }
    reg->CR.B.BOFFMSK             = 1;                /* ENABLE NS INTERRUPTS                     */
    reg->CR.B.ERRMSK              = 1;
#endif

#if MCF5485C_CAN_RX_INTERRUPT_EN > 0
    reg->IMASK.R                  |=                  /* Set the interrupt enable bits of the     */
        (1 << data->RdRxMB) |                         /* receive buffers.                         */
        MCF5485C_CAN_MSG_BUF_14_MASK |
        MCF5485C_CAN_MSG_BUF_15_MASK;

    if (dev == MCF5485C_CAN_BUS_A) {
        MCF_INTC_IMRH &= ~(1 << (MCF_CAN0_MBOR - 32));/* clear IRQ mask                           */
        MCF_INTC_ICR51 = para->MBuffIrqPrio;          /* set priority                             */
    } else {
        MCF_INTC_IMRH &= ~(1 << (MCF_CAN1_MBOR - 32));
        MCF_INTC_ICR57 = para->MBuffIrqPrio;
    }
#endif
                                                      /* -----------------------------------------*/
    dummy                         = reg->TIMER;       /* dummy read to the timer register         */
    reg->MCR.B.FRZ                = 0;                /* exit the freeze mode                     */
    reg->MCR.B.HALT               = 0;                /* -----------------------------------------*/

    if (error != MCF5485C_CAN_NO_ERR) {               /* If an error occured                      */
        DrvError = MCF5485C_CAN_INIT_ERR;             /* set driver error to init error           */
    }                                                 /*------------------------------------------*/

    return(result);
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MCF5485C CAN OPEN MESSAGE BUFFER
* \ingroup  MCF5485C_CAN
*
*           Opens the CAN device.
*
* \param    drv               Unused, but needed for common interface
* \param    devName           The FlexCAN device name
* \param    mode              FlexCAN interface access mode
*
* \return   The device identifier for further access or -1 if an error occurs
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MCF5485CCANOpen (CPU_INT16S drv,
                           CPU_INT32U devName,
                           CPU_INT16U mode)
{                                                     /* -----------------------------------------*/
    volatile MCF5485C_CAN_REG *reg;                   /* Local: Pointer to CAN module             */
             MCF5485C_CAN_DATA         *data;         /* Local: Pointer to CAN device data        */
             CPU_INT16S result = -1;                  /* Local: Function result                   */
             CPU_INT08U mb;

#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
             CPU_SR_ALLOC();                          /* Allocate storage for CPU status reg.*/
#endif                                                /*------------------------------------------*/
    (void)drv;                                        /* prevent compiler warning                 */
                                                      /*------------------------------------------*/
#if MCF5485C_CAN_ARG_CHK_CFG > 0                      /* Checking arguments (if enabled)          */
    if (devName >= MCF5485C_CAN_DEV_N) {              /* devName out of range?                    */
        ErrRegister((CPU_INT16U)DrvIdent, MCF5485C_CAN_BUS_ERR);
        return (result);
    }
    if (mode != DEV_RW) {                                  /* mode not supported?                 */
        ErrRegister((CPU_INT16U)DrvIdent, MCF5485C_CAN_MODE_ERR);
        return (result);
    }
#endif                                                /*------------------------------------------*/
    data = &CanData[devName];                         /* set driver data pointer                  */
    CPU_CRITICAL_ENTER();                             /* Enter critical section                   */
                                                      /* Check, that CAN device is not open       */
    if ((data->Status & MCF5485C_CAN_OPEN) == 0) {
        data->Status |= MCF5485C_CAN_OPEN;            /* Mark CAN device to be opened             */
        CPU_CRITICAL_EXIT();                          /* Leave critical section                   */
        reg  = (MCF5485C_CAN_REG *)CanData[devName].Base; /* Set local pointer to CAN module      */
                                                      /*------------------------------------------*/
        reg->MCR.B.HALT   = 1;                        /* Enter the freeze mode                    */
        reg->MCR.B.FRZ    = 1;                        /*                                          */
        while (reg->MCR.B.FRZ != 1) {                 /* wait for ackn of the freeze mode         */
        }                                             /*------------------------------------------*/
                                                      /* Configure receiver                       */
        mb = data->WrRxMB;
        reg->RXGMASK.R = 0x00000000;                  /* Receive every ID                         */
        reg->BUF[mb].CS.B.CODE =                      /* Activate RX message buffer               */
            MCF5485C_CAN_RX_BUFFER_EMPTY;
        reg->BUF[0].CS.B.CODE   =                     /* Activate TX message buffer               */
            MCF5485C_CAN_TX_BUFFER_NOT_READY;

        reg->MCR.B.FRZ    = 0;                        /* exit the freeze mode                     */
        reg->MCR.B.HALT   = 0;                        /*                                          */
#if (MCF5485C_CAN_RX_INTERRUPT_EN > 0)
                                                      /* store the received Node Id for the irqs  */
        MCF5485C_BSP_SetDevIds ((CPU_INT08U) drv,
                               (CPU_INT08U) devName);
#endif
        result = devName;                             /* Set return value to devName              */
    } else {                                          /*------------------------------------------*/
        CPU_CRITICAL_EXIT();                          /* Leave critical section                   */

        ErrRegister((CPU_INT16U)DrvIdent, MCF5485C_CAN_OPEN_ERR);
    }                                                 /*------------------------------------------*/

    return (result);                                  /* return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MCF5485C CAN CLOSE SUBDEVICE
* \ingroup  MCF5485C_CAN
*
*           Close the CAN bus or a message buffer after checking, that this  device is opened.
*
* \param    devId             The device identifier, returned by MCF5485CCANOpen()
*
* \return   Errorcode (0 if ok, -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MCF5485CCANClose (CPU_INT16S devId)
{                                                     /* -----------------------------------------*/
    CPU_INT16S result = -1;                           /* Local: Function result                   */
#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
#endif                                                /*------------------------------------------*/
#if MCF5485C_CAN_ARG_CHK_CFG > 0                      /* Checking arguments (if enabled)          */
    if ((devId < 0) ||                                /* devId out of range?                      */
        (devId >= MCF5485C_CAN_DEV_N)) {
        ErrRegister((CPU_INT16U)DrvIdent, MCF5485C_CAN_BUS_ERR);
        return (-1);
    }
#endif                                                /* -----------------------------------------*/
    CPU_CRITICAL_ENTER();                             /* Enter critical section                   */
    if (CanData[devId].Status & MCF5485C_CAN_OPEN) {  /* see, if CAN device is opened             */
        CanData[devId].Status = MCF5485C_CAN_IDLE;    /* yes: reset CAN device status             */
        CPU_CRITICAL_EXIT();                          /* Leave critical section                   */
        result = MCF5485C_CAN_NO_ERR;                 /* Ok, device is closed                     */
    } else {                                          /*------------------------------------------*/
        CPU_CRITICAL_EXIT();                          /* Leave critical section                   */
        ErrRegister((CPU_INT16U)DrvIdent,             /* set device close error                   */
                    MCF5485C_CAN_CLOSE_ERR);
    }                                                 /* -----------------------------------------*/

    return (result);                                  /* return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MCF5485C CAN I/O CONTROL
* \ingroup  MCF5485C_CAN
*
*           Controls the bus after or a message buffer after checking, that this  device is opened.
*
*           The following functioncodes are allowed:
*           - IO_MCF5485C_CAN_GET_IDENT:         Return driver ident code
*           - IO_MCF5485C_CAN_GET_ERRNO:         Return last detected errorcode
*           - IO_MCF5485C_CAN_GET_DRVNAME:       Return human readable driver name
*           - IO_MCF5485C_CAN_SET_BAUDRATE:      Set the baudrate
*           - IO_MCF5485C_CAN_START:             Start CAN controller.
*           - IO_MCF5485C_CAN_STOP:              Stop CAN controller.
*           - IO_MCF5485C_CAN_RX_STANDARD:       Set Receiver to Standard Identifier
*           - IO_MCF5485C_CAN_RX_EXTENDED:       Set Receiver to Extended Identifier
*           - IO_MCF5485C_CAN_TX_READY:          Get TX Ready Status
*           - IO_MCF5485C_CAN_GET_NODE_STATUS:   Get Node Status
*           - IO_MCF5485C_CAN_SET_BLOCKING_MODE: Sets the read/write operations to blocking mode
*           - IO_MCF5485C_CAN_SET_NON_BLOCKING_MODE: Sets the read/write operations to non blocking mode
*           - IO_MCF5485C_CAN_SET_RX_FILTER_1:   Sets the ID filter for the first buffer
*           - IO_MCF5485C_CAN_SET_RX_FILTER_2:   Sets the ID filter for the second buffer
*           - IO_MCF5485C_CAN_SET_RX_FILTER_3:   Sets the ID filter for the third buffer
*           - IO_MCF5485C_CAN_CFG_BIT_TIMING:    Configure the bit timing registers
*
* \param    devId             The device identifier, returned by MCF5485CCANOpen()
* \param    func              Function code (see MCF5485C_can_ioctl_func)
* \param    argp              Argument list, specific to the function code
*
* \return   Errorcode (0 if ok, -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MCF5485CCANIoCtl (CPU_INT16S devId,
                           CPU_INT16U func,
                           void *argp)
{                                                     /*------------------------------------------*/
    volatile MCF5485C_CAN_REG  *reg;                  /* Local: Pointer to CAN module             */
             MCF5485C_CAN_DATA *data;                 /* Local: Pointer to CAN device data        */
             CPU_INT32U        canId;                 /* Local: CAN Identifier                    */
             CPU_INT32U        mask;                  /* Local: Mask for identifier caluclations  */
             CPU_INT16U        i;                     /* Local: Loop variable                     */
             CPU_INT16S        result;                /* Local: Function result                   */
#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status register */
#endif                                                /*------------------------------------------*/
    result = MCF5485C_CAN_NO_ERR;

#if MCF5485C_CAN_ARG_CHK_CFG > 0                      /* Checking arguments (if enabled)          */
    if ((devId < 0) ||                                /* devId out of range?                      */
        (devId >= MCF5485C_CAN_DEV_N)) {
        ErrRegister((CPU_INT16U)DrvIdent, MCF5485C_CAN_BUS_ERR);
        return (-1);
    }
#endif                                                /*------------------------------------------*/
    reg  = (MCF5485C_CAN_REG *)CanData[devId].Base;   /* Set pointer to CAN module               */
    data = &CanData[devId];                           /* Set pointer to CAN device data           */

    if ((data->Status & MCF5485C_CAN_OPEN) == 0) {    /* see, if CAN device is closed             */
        ErrRegister((CPU_INT16U)DrvIdent,
                    MCF5485C_CAN_OPEN_ERR);           /* yes: set device not opened error         */
        return (-1);
    }

    CPU_CRITICAL_ENTER();                             /* Enter critical section                   */
                                                      /*------------------------------------------*/
    switch (func) {                                   /* select: function code                    */
                                                      /*------------------------------------------*/
        case IO_MCF5485C_CAN_GET_IDENT:               /* GET IDENT                                */
            (*(CPU_INT32U*)argp) = DrvIdent;          /* return driver ident code                 */
            break;                                    /*------------------------------------------*/
                                                      /*------------------------------------------*/
        case IO_MCF5485C_CAN_GET_ERRNO:               /* GET ERRORCODE                            */
            (*(CPU_INT16U*)argp) = DrvError;          /* return last detected errorcode           */
            break;                                    /*------------------------------------------*/
                                                      /*------------------------------------------*/
        case IO_MCF5485C_CAN_GET_DRVNAME:             /* GET DRIVER NAME                          */
            (*(CPU_INT08U**)argp) = (CPU_INT08U*)     /* return human readable driver name        */
                                     MCF5485C_CAN_NAME;
            break;                                    /*------------------------------------------*/
                                                      /*------------------------------------------*/
        case IO_MCF5485C_CAN_SET_BAUDRATE:            /* SET BAUDRATE                             */
            data->Baudrate = *(CPU_INT32U *)argp;     /* Set baudrate parameter to given argument */
                                                      /* and calculate the values for the timing  */
                                                      /* register with the parameter setting      */
            if ( MCF5485CCANCalcTimingReg(data) != MCF5485C_CAN_NO_ERR) {
                ErrRegister((CPU_INT16U)DrvIdent,
                            MCF5485C_CAN_ARG_ERR);    /* set errorcode if an error occured and    */
                result = -1;                          /* indicate error on function result.       */
            } else {                                  /* set can timing registers if timing       */
                                                      /* calculation success.                     */
                                                      /*------------------------------------------*/
                mask  = (reg->CR.R & MCF5485C_CAN_CR_MASK);
                mask |= ((data->PRESDIV << 24) |      /* Prescale devision factor                 */
                         (data->RJW     << 22) |      /* Resync jump width                        */
                         (data->PSEG1   << 19) |      /* Phase segment 1                          */
                         (data->PSEG2   << 16) |      /* Phase segment 2                          */
                          data->PROPSEG);             /* Propagation segment.                     */
                reg->CR.R = mask;
                data->TxTimeout       = 1000000 / data->Baudrate * 1000;
            }
            break;
                                                      /*------------------------------------------*/
        case IO_MCF5485C_CAN_START:                   /* START                                    */
            reg->MCR.B.MDIS = 0;
            break;
                                                      /*------------------------------------------*/
        case IO_MCF5485C_CAN_STOP:                    /* STOP                                     */
            reg->MCR.B.MDIS = 1;
            break;
                                                      /*------------------------------------------*/
        case IO_MCF5485C_CAN_RX_STANDARD:             /* STANDARD RX IDENTIFIER                   */
            for (i = MCF5485C_CAN_RX_FIRST_MSG_BUF;
                 i <= MCF5485C_CAN_RX_LAST_MSG_BUF;
                 i++) {
                reg->BUF[i].CS.B.IDE = 0;
            }
            break;
                                                      /*------------------------------------------*/
        case IO_MCF5485C_CAN_RX_EXTENDED:             /* EXTENDED RX IDENTIFIER                   */
            for (i = MCF5485C_CAN_RX_FIRST_MSG_BUF;
                 i <= MCF5485C_CAN_RX_LAST_MSG_BUF;
                 i++) {
                reg->BUF[i].CS.B.IDE = 1;
            }
            break;                                    /* -----------------------------------------*/

        case IO_MCF5485C_CAN_TX_READY:
            (*(CPU_INT08U*)argp) = 1;                 /* return allways TRUE                      */
            break;
                                                      /*------------------------------------------*/
        case IO_MCF5485C_CAN_GET_NODE_STATUS:         /*             GET NODE STATUS              */
            if ((reg->ESR.B.FLTCONF & 0x1u) != 0) {   /* Error Status Bit                         */
                (*(CPU_INT08U*)argp) = (CPU_INT08U)1;
            }
            if ((reg->ESR.B.FLTCONF & 0x2u) != 0) {   /* Bus Status Bit                           */
                (*(CPU_INT08U*)argp) = (CPU_INT08U)2;
            }
            if ((reg->ESR.B.FLTCONF & 0x3u) == 0) {   /* Error and Bus Status Bit                 */
                (*(CPU_INT08U*)argp) = (CPU_INT08U)0;
            }
            break;
                                                      /*------------------------------------------*/
        case IO_MCF5485C_CAN_SET_BLOCKING_MODE:       /* BLOCKING MODE                            */
            data->Status |= MCF5485C_CAN_BLOCKING;    /* Enable blocking mode                     */
            break;
                                                      /*------------------------------------------*/
        case IO_MCF5485C_CAN_SET_NON_BLOCKING_MODE:   /* NON BLOCKING MODE                        */
            data->Status &= ~MCF5485C_CAN_BLOCKING;   /* Disable blocking mode                    */
            break;
                                                      /*------------------------------------------*/
        case IO_MCF5485C_CAN_SET_RX_FILTER_1:         /* SET RX FILTER 1                          */
            mask  = ((CPU_INT32U*)argp)[0];
            canId = ((CPU_INT32U*)argp)[1];
            if (canId > MCF5485C_CAN_STD_ID_MASK) {
                                                      /* use bit 0..28                            */
                reg->RXGMASK.R =  mask;
                canId &= MCF5485C_CAN_EXT_ID_MASK;
                for (i = MCF5485C_CAN_RX_FIRST_MSG_BUF;
                     i <= MCF5485C_CAN_RX_LAST_MSG_BUF;
                     i++) {
                    reg->BUF[i].ID = canId;
                }
            } else {
                                                      /* use bit 0..10                            */
                reg->RXGMASK.R = (mask << 18);
                canId &= MCF5485C_CAN_STD_ID_MASK;
                for (i = MCF5485C_CAN_RX_FIRST_MSG_BUF;
                     i <= MCF5485C_CAN_RX_LAST_MSG_BUF;
                     i++) {
                    reg->BUF[i].ID = (canId << 18);
                }
            }
            break;
                                                      /*------------------------------------------*/
        case IO_MCF5485C_CAN_SET_RX_FILTER_2:         /* SET RX FILTER 2                          */
            mask  = ((CPU_INT32U*)argp)[0];
            canId = ((CPU_INT32U*)argp)[1];
            if (canId > MCF5485C_CAN_STD_ID_MASK) {
                                                      /* use bit 0..28                            */
                reg->RX14MASK.R =  mask;

                canId &= MCF5485C_CAN_EXT_ID_MASK;
                reg->BUF[14].ID = canId;
            } else {
                                                      /* use bit 0..10                            */
                reg->RX14MASK.R = (mask << 18);

                canId &= MCF5485C_CAN_STD_ID_MASK;
                reg->BUF[14].ID = (canId << 18);
            }
            reg->BUF[14].CS.B.CODE =                  /* enable message buffer 14 for receive     */
                MCF5485C_CAN_RX_BUFFER_EMPTY;
            break;
                                                      /*------------------------------------------*/
        case IO_MCF5485C_CAN_SET_RX_FILTER_3:         /* SET RX FILTER 3                          */
            mask  = ((CPU_INT32U*)argp)[0];
            canId = ((CPU_INT32U*)argp)[1];
            if (canId > MCF5485C_CAN_STD_ID_MASK) {
                                                      /* use bit 0..28                            */
                reg->RX15MASK.R =  mask;

                canId &= MCF5485C_CAN_EXT_ID_MASK;
                reg->BUF[15].ID = canId;

            } else {
                                                      /* use bit 0..10                            */
                reg->RX15MASK.R = (mask << 18);

                canId &= MCF5485C_CAN_STD_ID_MASK;
                reg->BUF[15].ID = (canId << 18);
            }
            reg->BUF[15].CS.B.CODE =                  /* enable message buffer 14 for receive     */
                MCF5485C_CAN_RX_BUFFER_EMPTY;
            break;
                                                      /*------------------------------------------*/
        case IO_MCF5485C_CAN_CFG_BIT_TIMING:          /* CONFIUGRE BIT TIMING                     */
            data->SamplePoint     = ((CPU_INT32U *)argp)[0];
            data->ResynchJumpWith = ((CPU_INT32U *)argp)[1];
            if ( MCF5485CCANCalcTimingReg(data) < 0) {
                ErrRegister((CPU_INT16U)DrvIdent,
                            MCF5485C_CAN_ARG_ERR);    /* set errorcode                            */
                result = -1;                          /* indicate error on function result        */
            } else {                                  /*------------------------------------------*/
                mask  = (reg->CR.R & MCF5485C_CAN_CR_MASK);
                mask |= ((data->PRESDIV << 24) |      /* Prescale devision factor                 */
                         (data->RJW     << 22) |      /* Resync jump width                        */
                         (data->PSEG1   << 19) |      /* Phase segment 1                          */
                         (data->PSEG2   << 16) |      /* Phase segment 2                          */
                          data->PROPSEG);             /* Propagation segment.                     */
                reg->CR.R = mask;
            }
            break;
                                                      /*------------------------------------------*/
        default:                                      /*           WRONG FUNCTION CODE            */
            result = -1;                              /* indicate error on function result        */
            break;                                    /* -----------------------------------------*/
    }
                                                      /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* Leave critical section                   */
                                                      /*------------------------------------------*/
    if (result < 0) {                                 /* If an error occured,                     */
        ErrRegister((CPU_INT16U)DrvIdent,             /* register error in error management       */
                    MCF5485C_CAN_FUNC_ERR);
    }
                                                      /*------------------------------------------*/
    return (result);                                  /* Return function result                   */
}
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MCF5485C CAN READ DATA
* \ingroup  MCF5485C_CAN
*
*           Read a received message from a message buffer after checking, that this  device is
*           opened.
*
* \param    devId             The device identifier, returned by MCF5485CCANOpen()
* \param    buf               Byte array for received can frame (must be a ptr to CANFRM)
* \param    size              Length of can frame memory
*
* \return   Number of received bytes, -1 if an error occured.
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MCF5485CCANRead (CPU_INT16S devId,
                          CPU_INT08U *buf,
                          CPU_INT16U size)
{                                                     /* -----------------------------------------*/
    volatile MCF5485C_CAN_REG  *reg;                  /* Local: Pointer to CAN module             */
             MCF5485C_CAN_DATA *data;                 /* Local: Pointer to CAN device data        */
    volatile CPU_INT32U        dummy;                 /* Local: dummy to read the timer register  */
             MCF5485C_CAN_FRM  *frm;                  /* Local: Pointer to can frame              */
             CPU_INT08U        mb;                    /* Local: Message Buffer number             */
             CPU_INT16S        result;                /* Local: Function result                   */

#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status register */
#endif                                                /*------------------------------------------*/

    result   = MCF5485C_CAN_NO_ERR;

#if MCF5485C_CAN_ARG_CHK_CFG > 0                      /* Checking arguments (if enabled)          */
    if ((devId < 0) ||                                /* devId out of range?                      */
        (devId >= MCF5485C_CAN_DEV_N)) {
        ErrRegister((CPU_INT16U)DrvIdent, MCF5485C_CAN_BUS_ERR);
        return (-1);
    }
    if (buf == 0) {                                   /* valid buffer pointer?                    */
        ErrRegister((CPU_INT16U)DrvIdent, MCF5485C_CAN_ARG_ERR);
        return (-1);
    }
    if (size != sizeof(MCF5485C_CAN_FRM)) {            /* size in range?                           */
        ErrRegister((CPU_INT16U)DrvIdent, MCF5485C_CAN_ARG_ERR);
        return (-1);
    }
#endif                                                /*------------------------------------------*/

    reg  = (MCF5485C_CAN_REG *)CanData[devId].Base;   /* Set pointer to CAN module                */
    data = &CanData[devId];                           /* Set pointer to CAN device data           */
    frm  = (MCF5485C_CAN_FRM *)buf;                   /* Set pointer to can frame                 */
                                                      /*------------------------------------------*/
    if (!(data->Status & MCF5485C_CAN_OPEN)) {        /* see, if CAN device is not opened         */
        ErrRegister((CPU_INT16U)DrvIdent,
                    MCF5485C_CAN_OPEN_ERR);           /* yes: set device not opened error         */
        return (-1);
    }                                                 /*------------------------------------------*/

    mb = data->RdRxMB;                                /* get read RX message buffer               */

    if ((data->Status & MCF5485C_CAN_BLOCKING) != 0) { /* if driver is in blocking mode           */
        while (!(reg->IFLAG.R & (1 << mb))) {         /* wait until rx complete flag is set       */
            /* nothing to do */
        }
    }
                                                      /*------------------------------------------*/
    if (( reg->IFLAG.R & (1 << mb)) != 0) {           /* see, if a CAN frame is received          */
        CPU_CRITICAL_ENTER();                         /* Enter critical section                   */
                                                      /* check, that buffer is ready for reading  */
        while ((reg->BUF[mb].CS.B.CODE &              /* (not indicating 'RXBufferBusy')          */
                    MCF5485C_CAN_RX_BUFFER_BUSY) ==   /* wait until this bit is negated           */
                    MCF5485C_CAN_RX_BUFFER_BUSY) {
        }                                             /*------------------------------------------*/
        frm->DLC =                                    /* get number of received bytes             */
            (CPU_INT08U)reg->BUF[mb].CS.B.LENGTH;
        *(CPU_INT32U*)(&frm->Data[0]) =               /* get received data bytes                  */
            *(CPU_INT32U*)(&reg->BUF[mb].DATA.Word[0]);
        *(CPU_INT32U*)(&frm->Data[4]) =
            *(CPU_INT32U*)(&reg->BUF[mb].DATA.Word[1]);
                                                      /* -----------------------------------------*/
                                                      /* get identifier format and extract        */
        if (reg->BUF[mb].CS.B.IDE ==                  /* identifier from the message buffer       */
            MCF5485C_CAN_MB_ID_STANDARD) {            /* - standard identifier resp.              */
            frm->Identifier = (reg->BUF[mb].ID) >> 18;
        } else {                                      /* - extended identifier                    */
            frm->Identifier = reg->BUF[mb].ID;
            frm->Identifier |= MCF5485C_CAN_EXT_ID_FLAG;
        }                                             /*------------------------------------------*/
        reg->IFLAG.R = (1 << mb);                     /* reset interrupt flag and                 */
        result = sizeof(MCF5485C_CAN_FRM);            /* Set function result to size of CANFRM    */
                                                      /* Global release of any locked message     */
        dummy = reg->TIMER;                           /* buffer                                   */

        CPU_CRITICAL_EXIT();                          /* Leave critical section                   */
                                                      /*------------------------------------------*/
    } else {                                          /* No message received,                     */
        frm->Identifier               = 0L;
        frm->DLC                      = 0;
        *(CPU_INT32U*)(&frm->Data[0]) = 0L;
        *(CPU_INT32U*)(&frm->Data[4]) = 0L;
    }
                                                      /*------------------------------------------*/
    return (result);                                  /* Return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MCF5485C CAN WRITE DATA
* \ingroup  MCF5485C_CAN
*
*           Write a message to a message buffer after checking, that this  device is opened.
*
* \param    devId             The device identifier, returned by MCF5485CCANOpen()
* \param    buf               Byte array for transmitting data
* \param    size              Length of can frame memory
*
* \return   Errorcode (0 if ok, -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MCF5485CCANWrite (CPU_INT16S devId,
                           CPU_INT08U *buf,
                           CPU_INT16U size)
 {                                                    /*------------------------------------------*/
    volatile MCF5485C_CAN_REG  *reg;                  /* Local: Pointer to CAN module             */
    volatile CPU_INT32U        dummy;                 /* Local: dummy to read the timer register  */
             MCF5485C_CAN_DATA *data;                 /* Local: Pointer to CAN device data        */
             MCF5485C_CAN_FRM  *frm;                  /* Local: Pointer to can frame              */
             CPU_INT32U        canId;                 /* Local: CAN Identifier                    */
             CPU_INT32U        timeout;               /* Local: TX timeout counter                */
             CPU_INT16U        i;                     /* Local: Loop variable                     */
#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
             CPU_SR_ALLOC();                          /* Allocate storage for CPU status register */
#endif
             CPU_INT16S        result;                /* Local: Function result                   */
                                                      /* -----------------------------------------*/
#if MCF5485C_CAN_ARG_CHK_CFG > 0                      /* Checking arguments (if enabled)          */
    if ((devId < 0) ||                                /* devId out of range?                      */
        (devId >= MCF5485C_CAN_DEV_N)) {
        ErrRegister((CPU_INT16U)DrvIdent,
                    MCF5485C_CAN_BUS_ERR);
        return (-1);
    }
    if (buf == 0) {                                   /* valid buffer pointer?                    */
        ErrRegister((CPU_INT16U)DrvIdent,
                    MCF5485C_CAN_ARG_ERR);
        return (-1);
    }
    if (size != sizeof(MCF5485C_CAN_FRM)) {           /* size in range?                           */
        ErrRegister((CPU_INT16U)DrvIdent,
                    MCF5485C_CAN_ARG_ERR);
        return (-1);
    }
#endif                                                /*------------------------------------------*/

    reg  = (MCF5485C_CAN_REG *)CanData[devId].Base;   /* Set pointer to CAN module                */
    data = &CanData[devId];                           /* Set pointer to CAN device data           */
    frm  = (MCF5485C_CAN_FRM *)buf;                   /* Set pointer to can frame                 */
                                                      /*------------------------------------------*/
    if (!(data->Status & MCF5485C_CAN_OPEN)) {        /* see, if CAN device is not opened         */
        ErrRegister((CPU_INT16U)DrvIdent,
                    MCF5485C_CAN_OPEN_ERR);           /* yes: set device not opened error         */
        return (-1);
    }

    if ((data->Status & MCF5485C_CAN_FIRST_TX) == 0) { /* see, if this was first transmission     */
        data->Status |= MCF5485C_CAN_FIRST_TX;        /* yes: indicate first transmission done.   */
    } else {                                          /* otherwise: we have transmitted a msg.    */
        timeout = data->TxTimeout;
        while ((reg->IFLAG.R & 0x0001L) == 0) {       /* wait until previous transmission is done */
            if ((data->Status & MCF5485C_CAN_BLOCKING) == 0) { /* see, if driver is not in blocking mode   */
                timeout--;                            /* decrement timeout counter                */
                if (timeout == 0) {                   /* see, if timeout counter reaches zero     */
                    ErrRegister((CPU_INT16U)DrvIdent, /* yes: indicate timeout error              */
                        MCF5485C_CAN_TX_TIMEOUT_ERR);
                    return(-1);
                }
            }
        }
    }
                                                      /*------------------------------------------*/
    CPU_CRITICAL_ENTER();                             /* Enter critical section                   */
    reg->IFLAG.R = 0x0001L;                           /* clear TX complete flag                   */
                                                      /* -----------------------------------------*/
    if (frm->DLC > 8) {                               /* Limit DLC to 8 bytes                     */
        frm->DLC = 8;
    }                                                 /* -----------------------------------------*/
                                                      /* Write the control/status word to hold    */
    reg->BUF[0].CS.B.CODE =                           /* the transmit buffer inactive and set     */
      MCF5485C_CAN_TX_BUFFER_NOT_READY;
    reg->BUF[0].CS.B.LENGTH = frm->DLC;               /* the length in the control/status word    */
                                                      /* -----------------------------------------*/
    canId = frm->Identifier;                          /* Write the ID register                    */
    if (canId > MCF5485C_CAN_STD_ID_MASK) {           /* depending on the CAN ID                  */
        canId &= MCF5485C_CAN_EXT_ID_MASK;            /* use bit 0..28                            */
        reg->BUF[0].CS.B.IDE =
            MCF5485C_CAN_MB_ID_EXTENDED;
        reg->BUF[0].ID = canId;
        data->ID = reg->BUF[0].ID;
    } else {
        canId &= MCF5485C_CAN_STD_ID_MASK;            /* use bit 0..10                            */
        reg->BUF[0].ID = (canId << 18);
        reg->BUF[0].CS.B.IDE =
            MCF5485C_CAN_MB_ID_STANDARD;
        data->ID = reg->BUF[0].ID;
    }                                                 /* -----------------------------------------*/
                                                      /* Write the data bytes into the CAN buffer */
    buf = (CPU_INT08U *)&(reg->BUF[0].DATA.Byte);
    for (i = 0; i < frm->DLC; i++, buf++) {
        *buf = frm->Data[i];
    }                                                 /*------------------------------------------*/
    reg->BUF[0].CS.B.CODE =                           /* Set the control/status word to           */
         MCF5485C_CAN_TX_BUFFER_SEND;                 /* TX_BUFFER_SEND                           */
    dummy = reg->TIMER;                               /* Unlock CAN message buffer                */

                                                      /* -----------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* Leave critical section                   */
    result = sizeof(MCF5485C_CAN_FRM);                /* Set return value to no error             */

    return (result);                                  /* Return function result                   */
}

