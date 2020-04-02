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
#include "drv_def.h"                                  /* Common driver definitions                */
#include "drv_can.h"                                  /* MB96F340 CAN driver declarations         */
#include "drv_can_reg.h"                              /* register definitions for CAN module      */

/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/

#define FIRST_RX   1
#define LAST_RX    16
#define FIRST_TX   17
#define LAST_TX    32

/*
****************************************************************************************************
*                                        REGISTER SHORTCUTS
****************************************************************************************************
*/

#if MB96F340_CAN_DEV_N > 1                            /*====== CHECK FOR MULTIPLE DEVICES ========*/

#define COER          cfg->CanReg->COER               /*!< CAN Output enable register             */

#define CTRLR         cfg->CanReg->CTRLR              /*!< CAN Control Register                   */
#define BTR           cfg->CanReg->BTR                /*!< CAN BittTiming Register                */
#define TESTR         cfg->CanReg->TESTR              /*!< CAN Test Register                      */
#define STATR         cfg->CanReg->STATR              /*!< CAN Status Register                    */

#define IF1ARB1       cfg->CanReg->IF1ARB1            /*!< CAN IF1 Arbitration register           */
#define IF1ARB2       cfg->CanReg->IF1ARB2            /*!< CAN IF1 Arbitration register           */
#define IF1MSK1       cfg->CanReg->IF1MSK1            /*!< CAN IF1 Mask Register                  */
#define IF1MSK2       cfg->CanReg->IF1MSK2            /*!< CAN IF1 Mask Register                  */
#define IF1MCTR       cfg->CanReg->IF1MCTR            /*!< CAN IF1 Message Control Register       */
#define IF1CREQ       cfg->CanReg->IF1CREQ            /*!< CAN IF1 Command Request Register       */
#define IF1CMSK       cfg->CanReg->IF1CMSK            /*!< IF1 Command Mask Register              */
#define IF1DTA1       cfg->CanReg->IF1DTA1            /*!< CAN IF1 Data                           */
#define IF1DTA2       cfg->CanReg->IF1DTA2            /*!< CAN IF1 Data                           */
#define IF1DTB1       cfg->CanReg->IF1DTB1            /*!< CAN IF1 Data                           */
#define IF1DTB2       cfg->CanReg->IF1DTB2            /*!< CAN IF1 Data                           */

#define IF2ARB1       cfg->CanReg->IF2ARB1            /*!< CAN IF2 Arbitration register           */
#define IF2ARB2       cfg->CanReg->IF2ARB2            /*!< CAN IF2 Arbitration register           */
#define IF2MSK1       cfg->CanReg->IF2MSK1            /*!< CAN IF2 Mask Register                  */
#define IF2MSK2       cfg->CanReg->IF2MSK2            /*!< CAN IF2 Mask Register                  */
#define IF2MCTR       cfg->CanReg->IF2MCTR            /*!< CAN IF2 Message Control Register       */
#define IF2CREQ       cfg->CanReg->IF2CREQ            /*!< CAN IF2 Command Request Register       */
#define IF2CMSK       cfg->CanReg->IF2CMSK            /*!< CAN IF2 Command Mask Register          */
#define IF2DTA1       cfg->CanReg->IF2DTA1            /*!< CAN IF2 Data                           */
#define IF2DTA2       cfg->CanReg->IF2DTA2            /*!< CAN IF2 Data                           */
#define IF2DTB1       cfg->CanReg->IF2DTB1            /*!< CAN IF2 Data                           */
#define IF2DTB2       cfg->CanReg->IF2DTB2            /*!< CAN IF2 Data                           */

#define NEWDAT1       cfg->CanReg->NEWDT1             /*!< CAN New Data Register                  */
#define NEWDAT2       cfg->CanReg->NEWDT2             /*!< CAN New Data Register                  */
#define TREQR1        cfg->CanReg->TREQR1             /*!< CAN Transmission Request Register      */
#define TREQR2        cfg->CanReg->TREQR2             /*!< CAN Transmission Request Register      */


#else                                                 /*============= SINGLE DEVICE ==============*/

#define COER          CanReg.COER                     /*!< CAN Output enable register             */

#define CTRLR         CanReg.CTRLR                    /*!< CAN Control Register                   */
#define BTR           CanReg.BTR                      /*!< CAN BittTiming Register                */
#define TESTR         CanReg.TESTR                    /*!< CAN Test Register                      */
#define STATR         CanReg.STATR                    /*!< CAN Status Register                    */

#define IF1ARB1       CanReg.IF1ARB1                  /*!< CAN IF1 Arbitration register           */
#define IF1ARB2       CanReg.IF1ARB2                  /*!< CAN IF1 Arbitration register           */
#define IF1MSK1       CanReg.IF1MSK1                  /*!< CAN IF1 Mask Register                  */
#define IF1MSK2       CanReg.IF1MSK2                  /*!< CAN IF1 Mask Register                  */
#define IF1MCTR       CanReg.IF1MCTR                  /*!< CAN IF1 Message Control Register       */
#define IF1CREQ       CanReg.IF1CREQ                  /*!< CAN IF1 Command Request Register       */
#define IF1CMSK       CanReg.IF1CMSK                  /*!< IF1 Command Mask Register              */
#define IF1DTA1       CanReg.IF1DTA1                  /*!< CAN IF1 Data                           */
#define IF1DTA2       CanReg.IF1DTA2                  /*!< CAN IF1 Data                           */
#define IF1DTB1       CanReg.IF1DTB1                  /*!< CAN IF1 Data                           */
#define IF1DTB2       CanReg.IF1DTB2                  /*!< CAN IF1 Data                           */

#define IF2ARB1       CanReg.IF2ARB1                  /*!< CAN IF2 Arbitration register           */
#define IF2ARB2       CanReg.IF2ARB2                  /*!< CAN IF2 Arbitration register           */
#define IF2MSK1       CanReg.IF2MSK1                  /*!< CAN IF2 Mask Register                  */
#define IF2MSK2       CanReg.IF2MSK2                  /*!< CAN IF2 Mask Register                  */
#define IF2MCTR       CanReg.IF2MCTR                  /*!< CAN IF2 Message Control Register       */
#define IF2CREQ       CanReg.IF2CREQ                  /*!< CAN IF2 Command Request Register       */
#define IF2CMSK       CanReg.IF2CMSK                  /*!< CAN IF2 Command Mask Register          */
#define IF2DTA1       CanReg.IF2DTA1                  /*!< CAN IF2 Data                           */
#define IF2DTA2       CanReg.IF2DTA2                  /*!< CAN IF2 Data                           */
#define IF2DTB1       CanReg.IF2DTB1                  /*!< CAN IF2 Data                           */
#define IF2DTB2       CanReg.IF2DTB2                  /*!< CAN IF2 Data                           */

#define NEWDAT1       CanReg.NEWDT1                   /*!< CAN New Data Register                  */
#define NEWDAT2       CanReg.NEWDT2                   /*!< CAN New Data Register                  */
#define TREQR1        CanReg.TREQR1                   /*!< CAN Transmission Request Register      */
#define TREQR2        CanReg.TREQR2                   /*!< CAN Transmission Request Register      */

#endif                                                /*==========================================*/


/*
****************************************************************************************************
*                                             MACROS
****************************************************************************************************
*/

#if MB96F340_CAN_DEV_N > 1                            /*=== CHECK FOR MULTIPLE DEVICES ===========*/
#define SET_CONFIG(x) const MB96F340_CAN_PARA *cfg = &DrvPara[x]
#else                                                 /* === SINGLE DEVICE =======================*/
#define SET_CONFIG(x) x=x
#endif                                                /*==========================================*/


/*
****************************************************************************************************
*                                            LOCAL DATA
****************************************************************************************************
*/

#if MB96F340_CAN_DEV_N > 1                            /*====== CHECK FOR MULTIPLE DEVICES ========*/

static const MB96F340_CAN_PARA DrvPara[MB96F340_CAN_DEV_N] = {
                                                      /*                C A N   0                 */
    {        (CAN_REG *)0x0700,                       /* Base Address                             */
    },                                                /*                C A N   1                 */
    {        (CAN_REG *)0x0800,                       /* Base Address                             */
    },                                                /*                C A N   2                 */
    {        (CAN_REG *)0x0900,                       /* Base Address                             */
    },                                                /*                C A N   3                 */
    {        (CAN_REG *)0x0A00,                       /* Base Address                             */
    },                                                /*                C A N   4                 */
    {        (CAN_REG *)0x0B00,                       /* Base Address                             */
    }
};

#else                                                 /*============= SINGLE DEVICE ==============*/

extern volatile CAN_REG        CanReg;                /*!< CAN Registers (mapped variable)        */

#endif                                                /*==========================================*/

static CPU_INT16U DrvError;                           /*!< Driver Error Code                      */

static MB96F340_CAN_DATA DevData[MB96F340_CAN_DEV_N]; /*!< Device runtime data                    */

static const MB96F340_CAN_BAUD DevBaud[] =            /*!< Driver baudrate table                  */
{
  { 1000000, 0x3880 },                                /* 1M,    SP=68%, BTL=16, RSJ=3             */
  {  500000, 0x3881 },                                /* 500k,  SP=68%, BTL=16, RSJ=3             */
  {  250000, 0x3883 },                                /* 250k,  SP=68%, BTL=16, RSJ=3             */
  {  125000, 0x3887 },                                /* 125k,  SP=68%, BTL=16, RSJ=3             */
  {  100000, 0x4BC7 },                                /* 100k,  SP=70%, BTL=20, RSJ=4             */
  {   83333, 0x6DC7 },                                /* 83,3k, SP=66%, BTL=24, RSJ=4             */
  {   50000, 0x4BCF },                                /* 50k,   SP=70%, BTL=20, RSJ=4             */
  {       0,      0 }                                 /* baudrate = 0 marks end of baudrate table */
};

static       CPU_INT08U TxBuffer;                     /*!< Actual used tx buffer                  */

static const CPU_INT32U DrvIdent = 0x243F2401L;       /*!< Driver Identification Code             */

/*
****************************************************************************************************
*                                            FUNCTIONS
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN INIT
*
*           This function initializes the selected CAN controller to a known state
*           - not forcing active state.
*           If configured with multiple devices and an error is detected, the initialization
*           is stopped after detecting the error.
*
* \param    arg               The CAN device.
*
* \return   Return value is 0, if function was successful. If an error is detected, the return
*           value is -1 and the error code contains one of the possible values:
*           - see return values of MB96F340CANHalt()
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MB96F340CANInit(CPU_INT32U arg)
{                                                     /*------------------------------------------*/
    CPU_INT16U i;                                     /* Local: counter                           */
    CPU_INT16S result;                                /* Local: Function result                   */
                                                      /*------------------------------------------*/
    SET_CONFIG(arg);                                  /* Set device configuration                 */
#if MB96F340_CAN_ARG_CHK_CFG > 0
    if (arg >= MB96F340_CAN_DEV_N) {                  /* devName out of range?                    */
        DrvError = MB96F340_CAN_BUS_ERR;              /* set argument error code                  */
        return (-1);
    }                                                 /*------------------------------------------*/
#endif
    MB96F340CANSetPins((CPU_INT16S)arg);

    DevData[arg].Use = 0;                             /* Set device to unused                     */

    COER  |= (1<<0);                                  /* enable CAN output                        */
    CTRLR  = (1<<0);                                  /* Stop CAN Operation                       */

                                                      /* set rx buffer                            */
    IF1ARB1  =  0;                                    /* clear id register                        */
    IF1ARB2  =  (1<<15);                              /* IF1 RX Buffer valid                      */
    IF1MSK1  =  0;                                    /* clear mask register                      */
    IF1MSK2  =  (1<<15);                              /* MXTD ID type and direction for filter    */

    IF1CMSK  = 0xF0;                                  /* Prepare Interface Command Mask Register: */
                                                      /* WRRD    = 1 : Write information from     */
                                                      /* interface to object                      */
                                                      /* MASK    = 1 : Write Mask regsiter        */
                                                      /* ARB     = 1 : Write Arbitration Register */
                                                      /* (ID Register)                            */
                                                      /* CONTROL = 1 : Write object control       */
                                                      /* register (e.g. Data lenght,              */
                                                      /* IRQ enable, etc)                         */
                                                      /* CIP     = 0 : (dont care for writing     */
                                                      /* to object)                               */
                                                      /* TXREQ   = 0 : No Transmission Request    */
                                                      /* DATAA   = 0 : Do not change Data A reg   */
                                                      /* DATAB   = 0 : Do not change Data B reg   */

#if MB96F340_CAN_RX_INTERRUPT_EN > 0
    IF1MCTR  = (1<<12)| (1<<10);                      /* set umask, rx ie                         */
#else
    IF1MCTR  = (1<<12);                               /* set umask                                */
#endif
    for (i = FIRST_RX; i <= LAST_RX; i++) {
        if (i == LAST_RX) {
#if MB96F340_CAN_RX_INTERRUPT_EN > 0
            IF1MCTR  = (1<<12)| (1<<7)| (1<<10);      /* set umask, eob, rx ie                    */
#else
            IF1MCTR  = (1<<12)| (1<<7);               /* set umask, eob                           */
#endif
        }
        IF1CREQ = i;                                  /* Transfer the Interface Register          */
    }                                                 /*------------------------------------------*/

                                                      /* set tx buffer                            */
    IF1MCTR  =  0;                                    /* clear control register                   */

    IF1ARB2  =  (1<<13);                              /* IF1 TX Buffer                            */

#if MB96F340_CAN_TX_INTERRUPT_EN > 0
    IF1MCTR |=  (1<<11);                              /* Enable transmit complete interrupt       */
#endif

    IF1CMSK  =  0xF0;                                 /* Prepare Interface Command Mask Register: */
                                                      /* WRRD    = 1 : Write information from     */
                                                      /* interface to object                      */
                                                      /* MASK    = 1 : Write Mask regsiter        */
                                                      /* ARB     = 1 : Write Arbitration Register */
                                                      /* (ID Register)                            */
                                                      /* CONTROL = 1 : Write object control       */
                                                      /* register (e.g. Data lenght,              */
                                                      /* IRQ enable, etc)                         */
                                                      /* CIP     = 0 : (dont care for writing     */
                                                      /* to object)                               */
                                                      /* TXREQ   = 0 : No Transmission Request    */
                                                      /* DATAA   = 0 : Do not change Data A reg   */
                                                      /* DATAB   = 0 : Do not change Data B reg   */

    for (TxBuffer = FIRST_TX; TxBuffer <= LAST_TX; TxBuffer++) {
        IF1CREQ  =  TxBuffer;                         /* .. Contents to the Message Buffer        */
    }
    TxBuffer = FIRST_TX;                              /* set tx buffer                            */

    result = 0;
    if (result >= 0) {                                /* see, if initialization was successful    */
        DrvError = MB96F340_CAN_NO_ERR;               /* yes: reset device errorcode              */
    }                                                 /*------------------------------------------*/
    return (result);                                  /* return function result                   */
}                                                     /*------------------------------------------*/


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN OPEN
*
*           This function opens a device for further access. This implementation supports
*           only the exclusive read/write mode. Therefore the device is exclusively locked
*           for the opened function/task.
*
* \param    drvId             Bus node name which must be used by the interrupt routine to access
*                             the can bus layer.
*
* \param    devName           The device name. Note: The device name is identical to the
*                             device identifier.
*
* \param    mode              The open mode
*
* \return   Return value is 0, if function was successful. If an error is detected, the return
*           value is -1 and the error code contains one of the possible values:
*           - MB96F340_CAN_BUS_ERR  : Device id out of range
*           - MB96F340_CAN_OPEN_ERR : Device is already opened
*           - MB96F340_CAN_MODE_ERR : Open mode is not supported
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MB96F340CANOpen(CPU_INT16S drvId, CPU_INT32U devName, CPU_INT16U mode)
{                                                     /*------------------------------------------*/
    MB96F340_CAN_DATA *can;                           /* Local: Pointer to can device             */
    CPU_INT16S         result = -1;                   /* Local: Function result                   */
    CPU_INT08U         devId  = (CPU_INT08U)devName;  /* Local: Device identifier                 */
#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
    CPU_SR             cpu_sr;                        /* Allocate storage for CPU status register */
#endif
                                                      /*------------------------------------------*/
#if ((MB96F340_CAN_RX_INTERRUPT_EN > 0) || \
     (MB96F340_CAN_TX_INTERRUPT_EN > 0) || \
     (MB96F340_CAN_NS_INTERRUPT_EN > 0))
                                                      /* store the received Node Id for the irqs  */
    MB96F340CANSetDevIds ((CPU_INT08U) drvId,
                          (CPU_INT08U) devName);
#else
    (void)drvId;                                      /* prevent compiler warning                 */
#endif

#if MB96F340_CAN_ARG_CHK_CFG > 0
    if (devId >= MB96F340_CAN_DEV_N) {                /* devName out of range?                    */
        DrvError = MB96F340_CAN_BUS_ERR;              /* set bus error                            */
        return (-1);
    }                                                 /*------------------------------------------*/
    if (mode != DEV_RW) {                             /* unsupported mode?                        */
        DrvError = MB96F340_CAN_MODE_ERR;             /* set mode error code                      */
        return (-1);
    }                                                 /*------------------------------------------*/
#else
    (void)mode;                                       /* prevent compiler warning                 */
#endif

    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    can = &DevData[devId];                            /* set pointer to can device                */
    if ((can->Use & MB96F340_CAN_BUSY) == 0) {        /* check, that can device is unused         */
        can->Use = 1;                                 /* mark can device as used                  */
        result   = (CPU_INT16S)devId;                 /* Okay, device is opened                   */
    }
    if (result < 0) {                                 /* see, if device can be opened             */
        DrvError = MB96F340_CAN_OPEN_ERR;             /* set busy errorcode                       */
    }
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return (result);                                  /* return function result                   */
}                                                     /*------------------------------------------*/


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CLOSE CAN DEVICE
*
*           This function removes the exclusive lock for a CAN device.
*
* \param    devId             Device identifier, returned by MB96F340CANOpen()
*
* \return   Return value is 0, if function was successful. If an error is detected, the return
*           value is -1 and the error code contains one of the possible values:
*           - MB96F340_CAN_BUS_ERR   : Device id out of range
*           - MB96F340_CAN_CLOSE_ERR : Device is not opened
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MB96F340CANClose(CPU_INT16S devId)
{                                                     /*------------------------------------------*/
    CPU_INT16S         result = 0;                    /* Local: Function result                   */
#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
    CPU_SR             cpu_sr;                        /* Allocate storage for CPU status register */
#endif
                                                      /*------------------------------------------*/

#if MB96F340_CAN_ARG_CHK_CFG > 0
    if ((devId >= MB96F340_CAN_DEV_N) ||              /* devId out of range?                      */
        (devId < 0)) {
        DrvError = MB96F340_CAN_BUS_ERR;              /* set bus error                            */
        return (-1);
    }                                                 /*------------------------------------------*/
#endif
    if ((DevData[devId].Use &                         /* device opened?                           */
         MB96F340_CAN_BUSY) == 0) {
        DrvError = MB96F340_CAN_CLOSE_ERR;            /* set close error code                     */
        return (-1);
    }                                                 /*------------------------------------------*/

    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    DevData[devId].Use &= ~(MB96F340_CAN_BUSY);       /* mark can device as unused                */
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return (result);                                  /* return function result                   */
}                                                     /*------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN I/O CONTROL
*
*           This function configures the features of the CAN controler and the devices during
*           runtime.
*
* \param    devId             Device identifier, returned by MB96F340CANOpen()
*
* \param    func              I/O function code, see enum MB96F340_CAN_IO_FUNC
*
* \param    argp              Depends on func, see enum MB96F340_CAN_IO_FUNC
*
* \return   Return value is 0, if function was successful. If an error is detected, the return
*           value is -1 and the error code contains one of the possible values:
*           - MB96F340_CAN_ARG_ERR  : Parameter not plausible
*           - MB96F340_CAN_OPEN_ERR : Device is not opened
*           - MB96F340_CAN_BUS_ERR  : Device id out of range
*           - MB96F340_CAN_FUNC_ERR : Function code not supported
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MB96F340CANIoCtl(CPU_INT16S devId, CPU_INT16U func, void *argp)
{                                                     /*------------------------------------------*/
    const MB96F340_CAN_BAUD *btbl;                    /* Local: Pointer to can bus baudrate table */
    CPU_INT32U               baud;                    /* Local: (Arg) Baudrate                    */
    CPU_INT16S               result = 0;              /* Local: Function result                   */
    CPU_INT16U               i;                       /* Local: counter                           */
    CPU_INT08U              *value;                   /* Local: (Arg) Pointer to Value            */
#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
    CPU_SR                   cpu_sr;                  /* Allocate storage for CPU status register */
#endif
                                                      /*------------------------------------------*/
    SET_CONFIG(devId);                                /* Set device configuration                 */

#if MB96F340_CAN_ARG_CHK_CFG > 0
    if ((devId >= MB96F340_CAN_DEV_N) ||              /* devId out of range?                      */
        (devId < 0)) {
        DrvError = MB96F340_CAN_BUS_ERR;              /* set bus error                            */
        return (-1);
    }                                                 /*------------------------------------------*/
#endif
    if ((DevData[devId].Use &                         /* device opened?                           */
         MB96F340_CAN_BUSY) == 0) {
        DrvError = MB96F340_CAN_OPEN_ERR;             /* set open error code                      */
        return (-1);
    }                                                 /*------------------------------------------*/

    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    switch(func) {                                    /* select: function code                    */
                                                      /*------------------------------------------*/
        case IO_MB96F340_CAN_GET_IDENT:               /*            GET DRIVER IDENT CODE         */
                                                      /*------------------------------------------*/
            *((CPU_INT32U *)(argp)) = DrvIdent;       /* set driver ident code to arg pointer     */
            break;
                                                      /*------------------------------------------*/
        case IO_MB96F340_CAN_GET_ERRNO:               /*            GET DRIVER ERROR CODE         */
                                                      /*------------------------------------------*/
            *((CPU_INT16U *)(argp)) = DrvError;       /* set driver error code to arg pointer     */
            break;
                                                      /*------------------------------------------*/
        case IO_MB96F340_CAN_GET_DRVNAME:             /*              GET DRIVER NAME             */
                                                      /*------------------------------------------*/
            *((CPU_INT08U **)(argp)) =                /* set ptr to driver name to arg ptr        */
                (CPU_INT08U *)("MB96F340 CAN");
            break;
                                                      /*------------------------------------------*/
        case IO_MB96F340_CAN_SET_BAUDRATE:            /*               SET BAUDRATE               */
                                                      /*------------------------------------------*/

            CTRLR  |= (1<<0);                         /* Stop CAN Operation                       */

            CTRLR  |= (1<<6);                         /* Enable Configuration Change              */

            baud   = *(CPU_INT32U*)(argp);            /* Get baudrate from argument pointer       */
            btbl   = &DevBaud[0];                     /* Set pointer to can bus baudrate table    */
            result = -1;                              /* set result to 'baudrate not found'       */
            while (btbl->Baudrate != 0L) {            /* Loop through whole baudrate table        */
                if (btbl->Baudrate == baud) {         /* if baudrate matches given argument       */
                    BTR    =  btbl->BTRVal;           /* Write value to BTR register              */
                    CTRLR &=~ (1<<6);                 /* Disable Configuration Change */
                    result = 0;                       /* Indicate sucessfull function execution   */
                    break;                            /* break loop                               */
                }
                btbl++;                               /* increment baudrate table pointer         */
            }
            if (result < 0) {                         /* see, if baudrate is set successfully     */
                DrvError = MB96F340_CAN_ARG_ERR;      /* Set baudrate error code                  */
            }
            break;
                                                      /*------------------------------------------*/
        case IO_MB96F340_CAN_START:                   /*          START CAN COMMUNICATION         */
                                                      /*------------------------------------------*/
#if MB96F340_CAN_NS_INTERRUPT_EN > 0
            CTRLR   |=  (1<<2) | (1<<3);              /* Enable status change interrupt           */
#endif

#if (MB96F340_CAN_RX_INTERRUPT_EN == 1) ||\
    (MB96F340_CAN_TX_INTERRUPT_EN == 1) ||\
    (MB96F340_CAN_NS_INTERRUPT_EN == 1)
            CTRLR  |=  (1<<1);                        /* Enable Module Interrupt                  */

#endif

            CTRLR  &=~ (1<<0);                        /* Start CAN Operation                      */
            break;
                                                      /*------------------------------------------*/
        case IO_MB96F340_CAN_STOP:                    /*           STOP CAN COMMUNICATION         */
                                                      /*------------------------------------------*/
#if MB96F340_CAN_NS_INTERRUPT_EN > 0
            CTRLR   &=~ ((1<<2) | (1<<3));            /* Disable status change interrupt          */
#endif

#if (MB96F340_CAN_RX_INTERRUPT_EN == 1) ||\
    (MB96F340_CAN_TX_INTERRUPT_EN == 1) ||\
    (MB96F340_CAN_NS_INTERRUPT_EN == 1)
            CTRLR  &=~ (1<<1);                        /* Disable Module Interrupt                 */

#endif
            CTRLR  |=  (1<<0);                        /* Stop CAN Operation                       */
            break;
                                                      /*------------------------------------------*/
        case IO_MB96F340_CAN_RX_STANDARD:             /* SET RX TO STANDARD ID                    */
                                                      /*------------------------------------------*/
            for (i = FIRST_RX; i <= LAST_RX; i++) {
                IF1CMSK  =  0x20;                     /* Prepare Interface Command Mask Register: */
                                                      /* ARB     = 1 : Read Arbitration Register  */
                                                      /*               (ID Register)              */

                IF1CREQ  =  i;                        /* Transfer the Interface Register          */

                IF1ARB2 &=~ (1<<14);                  /* Clear extended flag                      */

                IF1CMSK  =  0xA0;                     /* Prepare Interface Command Mask Register: */
                                                      /* ARB     = 1 : Write Arbitration Register */
                                                      /*               (ID Register)              */

                IF1CREQ  =  i;                        /* Transfer the Interface Register          */
            }
            break;
                                                      /*------------------------------------------*/
        case IO_MB96F340_CAN_RX_EXTENDED:             /* SET RX TO EXTENDED ID                    */
                                                      /*------------------------------------------*/
            for (i = FIRST_RX; i <= LAST_RX; i++) {
                IF1CMSK  =  0x20;                     /* Prepare Interface Command Mask Register: */
                                                      /* ARB     = 1 : Read Arbitration Register  */
                                                      /*               (ID Register)              */

                IF1CREQ  =  i;                        /* Transfer the Interface Register          */

                IF1ARB2 |= (1<<14);                   /* Set extended flag                        */

                IF1CMSK  = 0xA0;                      /* Prepare Interface Command Mask Register: */
                                                      /* ARB     = 1 : Write Arbitration Register */
                                                      /*               (ID Register)              */

                IF1CREQ  =  i;                        /* Transfer the Interface Register          */
            }
            break;
                                                      /*------------------------------------------*/
        case IO_MB96F340_CAN_TX_READY:                /* GET TX READY                             */
                                                      /*------------------------------------------*/
#if MB96F340_CAN_TX_INTERRUPT_EN > 0
            if ((TREQR1 & 0x02) != 0) {               /* if transmission pending                  */
                (*((CPU_INT08U*)argp)) = 0;
            } else {                                  /* otherwise. transmission not pending      */
                (*((CPU_INT08U*)argp)) = 1;
            }
#else
            (*((CPU_INT08U*)argp)) = 1;
#endif
            break;
                                                      /*------------------------------------------*/
        case IO_MB96F340_CAN_GET_NODE_STATUS:         /*           GET NODE STATUS                */
                                                      /*------------------------------------------*/
            value  = (CPU_INT08U *)(argp);            /* Get pointer to status from para. list    */
            if ((STATR & (1<<7)) != 0) {
                *value = MB96F340_CAN_ERROR_BUS_OFF;  /* error bus off                            */
            } else if ((STATR & (1<<5)) != 0) {
                *value = MB96F340_CAN_ERROR_PASSIVE;  /* error passive                            */
            } else {
                *value = MB96F340_CAN_ERROR_ACTIVE;   /* error active                             */
            }                                         /*------------------------------------------*/
            break;
                                                      /*------------------------------------------*/
        default:                                      /*      NOT SUPPORTED FUNCTION CODE         */
                                                      /*------------------------------------------*/
            result = -1;                              /* indicate error                           */
            DrvError = MB96F340_CAN_FUNC_ERR;         /* set function code error                  */
            break;
    }                                                 /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return (result);                                  /* Return function result                   */
}                                                     /*------------------------------------------*/


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN READ
*
*           This function reads an already received CAN frame.
*
* \param    devId             Device identifier, returned by MB96F340CANOpen()
*
* \param    buffer            Pointer to read buffer (must be from type MB96F340_CAN_FRM)
*
* \param    size              Size of read buffer (must be equal to sizeof(MB96F340_CAN_FRM))
*
* \return   Return value is the number of read bytes, if function was successful. If an error is
*           detected, the return value is -1 and the error code contains one of the possible
*           values:
*           - MB96F340_CAN_ARG_ERR : Parameter not plausible
*           - MB96F340_CAN_OPEN_ERR : Device is not opened
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MB96F340CANRead (CPU_INT16S devId, CPU_INT08U *buffer, CPU_INT16U size)
{                                                     /*------------------------------------------*/
    CPU_INT16U        i;                              /* Local: counter                           */
    CPU_INT16S        result = -1;                    /* Local: Function result                   */
    MB96F340_CAN_FRM *frm;                            /* Local: Pointer to can frame              */
#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
    CPU_SR            cpu_sr;                         /* Allocate storage for CPU status register */
#endif
                                                      /*------------------------------------------*/
    SET_CONFIG(devId);                                /* Set device configuration                 */

#if MB96F340_CAN_ARG_CHK_CFG > 0
    if ((devId >= MB96F340_CAN_DEV_N) ||              /* devId out of range?                      */
        (devId < 0)) {
        DrvError = MB96F340_CAN_BUS_ERR;              /* set bus error                            */
        return (result);
    }                                                 /*------------------------------------------*/
    if (buffer == NULL) {                             /* buffer not valid?                        */
        DrvError = MB96F340_CAN_ARG_ERR;              /* Set argument error code                  */
        return (result);
    }                                                 /*------------------------------------------*/
    if (size != sizeof(MB96F340_CAN_FRM)) {           /* size not correct?                        */
        DrvError = MB96F340_CAN_NO_DATA_ERR;          /* Set no data error code                   */
        return (result);
    }                                                 /*------------------------------------------*/
#endif
    if ((DevData[devId].Use &                         /* device opened?                           */
         MB96F340_CAN_BUSY) == 0) {
        DrvError = MB96F340_CAN_OPEN_ERR;             /* set open error code                      */
        return (-1);
    }                                                 /*------------------------------------------*/

    frm = (MB96F340_CAN_FRM *)buffer;                 /* Set pointer to can frame                 */

    for (i = FIRST_RX; i <= LAST_RX; i++) {           /* check for receive buffer                 */
        if ((NEWDAT1 & (1<<(i-1))) != 0) {
            break;                                    /* receive buffer found                     */
        }
    }

    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    if (i <= LAST_RX) {                               /* if message received                      */

        IF1CMSK  = 0x3F;                              /* Prepare Interface Command Mask Register: */
                                                      /* Receive Control Information, Message     */
                                                      /* data and Arbitration from Message Buffer */
                                                      /* WRRD    = 0 : Write information from     */
                                                      /*                  object to interface     */
                                                      /* MASK    = 0 : Do not transfer Mask reg   */
                                                      /* ARB     = 1 : Transfer Arbitration       */
                                                      /*               Register (ID Register)     */
                                                      /* CONTROL = 1 : Transfer object control    */
                                                      /*   reg (e.g. Data lenght, IRQ enable, etc)*/
                                                      /* CIP     = 1 : (Clear Interrupt Pending   */
                                                      /*                 flag)                    */
                                                      /* TXREQ   = 1 : Set Transmission Request   */
                                                      /* DATAA   = 1 : Transfer Data A register   */
                                                      /* DATAB   = 1 : Transfer Data B register   */

        IF1CREQ = i;                                  /* Start transfer                           */

        if((IF1MCTR & (1<<14)) != 0 ) {               /* check whether or not a message was lost  */

            IF1MCTR &=~ (1<<14);                      /* Clear MSGLST Flag                        */

            IF1CMSK  =  0x90;                         /* Prepare Interface Command Mask Register: */
                                                      /* write control data only                  */
                                                      /* WRRD    = 1 : Write information from     */
                                                      /* interface to object                      */
                                                      /* MASK    = 0 : Do not transfer Mask reg   */
                                                      /* ARB     = 0 : Do not transfer Arbitration*/
                                                      /*                   Register (ID Register) */
                                                      /* CONTROL = 1 : Transfer object control    */
                                                      /* reg (e.g. Data lenght, IRQ enable, etc)  */
                                                      /* CIP     = 0 : (dont care for writing to  */
                                                      /*                      object)             */
                                                      /* TXREQ   = 0 : No Transmission Request    */
                                                      /* DATAA   = 0 : Do not transfer Data A reg */
                                                      /* DATAB   = 0 : Do not transfer Data B reg */

            IF1CREQ  =  i;                            /* Start transfer                           */
        }

        frm->DLC     = IF1MCTR & 0x0F;                /* get dlc                                  */

        if (frm->DLC > 0) {
            frm->Data[0] = IF1DTA1;                   /* get data                                 */
            frm->Data[1] = IF1DTA1 >> 8;
            frm->Data[2] = IF1DTA2;
            frm->Data[3] = IF1DTA2 >> 8;
        }
        if (frm->DLC > 3) {
            frm->Data[4] = IF1DTB1;
            frm->Data[5] = IF1DTB1 >> 8;
            frm->Data[6] = IF1DTB2;
            frm->Data[7] = IF1DTB2 >> 8;
        }

        if ((IF1ARB2 & (1<<14)) != 0) {               /* if extended id                           */
            frm->Identifier = ((IF1ARB1 |
                                ((CPU_INT32U)IF1ARB2 << 16)) &
                               0x1FFFFFFFL) | (1L<<29);
        } else {                                      /* otherwise: standard id                   */
            frm->Identifier = (IF1ARB2 >> 2) & 0x7FF;
        }
        if ((IF1ARB2 & (1<<13)) != 0) {               /* if remote frame received                 */
            frm->Identifier |= (1UL << 30);           /* set bit 30                               */
        }
        result = sizeof(MB96F340_CAN_FRM);            /* set success                              */
    }
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
                                                      /*------------------------------------------*/
    return (result);                                  /* Return function result                   */
}                                                     /*------------------------------------------*/


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN WRITE
*
*           This function writes a CAN frame to the CAN controler.
*
* \param    devId             Device identifier, returned by MB96F340CANOpen()
*
* \param    buffer            Pointer to frame buffer (must be from type MB96F340_CAN_FRM)
*
* \param    size              Size of frame buffer (must be equal to sizeof(MB96F340_CAN_FRM))
*
* \return   Return value is the number of written bytes, if function was successful. If an error
*           is detected, the return value is -1 and the error code contains one of the possible
*           values:
*           - MB96F340_CAN_ARG_ERR : Parameter not plausible
*           - MB96F340_CAN_OPEN_ERR : Device is not opened
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S MB96F340CANWrite (CPU_INT16S devId, CPU_INT08U *buffer, CPU_INT16U size)
{                                                     /*------------------------------------------*/
    MB96F340_CAN_FRM *frm;                            /* Local: Pointer to can frame              */
#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
    CPU_SR            cpu_sr;                         /* Allocate storage for CPU status register */
#endif
                                                      /*------------------------------------------*/
    SET_CONFIG(devId);                                /* Set device configuration                 */

#if MB96F340_CAN_ARG_CHK_CFG > 0
    if ((devId >= MB96F340_CAN_DEV_N) ||              /* devId out of range?                      */
        (devId < 0)) {
        DrvError = MB96F340_CAN_BUS_ERR;              /* Set bus error code                       */
        return (-1);
    }                                                 /*------------------------------------------*/
    if (buffer == NULL) {                             /* buffer not valid?                        */
        DrvError = MB96F340_CAN_ARG_ERR;              /* Set argument error code                  */
        return (-1);
    }                                                 /*------------------------------------------*/
    if (size != sizeof(MB96F340_CAN_FRM)) {           /* size not correct?                        */
        DrvError = MB96F340_CAN_NO_DATA_ERR;          /* Set no data error code                   */
        return (-1);
    }                                                 /*------------------------------------------*/
#endif
    if ((DevData[devId].Use &                         /* device opened?                           */
         MB96F340_CAN_BUSY) == 0) {
        DrvError = MB96F340_CAN_OPEN_ERR;             /* set open error code                      */
        return (-1);
    }                                                 /*------------------------------------------*/

    if ((TREQR2 & (1 << (TxBuffer - FIRST_TX))) != 0) { /* if transmission pending                */
        DrvError = MB96F340_CAN_BUSY_ERR;             /* set busy error code                      */
        return (-1);
    }

    frm = (MB96F340_CAN_FRM *)buffer;                 /* Set pointer to can frame                 */

    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    if (frm->Identifier > 0x7FF) {                    /* if extended id                           */
        IF1ARB1  = (CPU_INT16U)frm->Identifier;
        IF1ARB2  = (CPU_INT16U)(frm->Identifier >> 16) & 0x1FFF;
        IF1ARB2 |= (1<<14);                           /* set extended id bit                      */
    } else {                                          /* otherwise: standard id                   */
        IF1ARB1  = 0;
        IF1ARB2  = (CPU_INT16U)(frm->Identifier << 2);
    }                                                 /*------------------------------------------*/

    IF1ARB2 |= (1<<13) | (1<<15);                     /* set dir to tx and msg valid              */

    IF1MCTR  = (1<<7) | (0x0F & frm->DLC);            /* clear NewDat, MsgLst, IntPnd, no filter, */
                                                      /* .. no remote */

                                                      /* set data                                 */
    IF1DTA1  = (CPU_INT16U)frm->Data[0] | (CPU_INT16U)frm->Data[1] << 8;
    IF1DTA2  = (CPU_INT16U)frm->Data[2] | (CPU_INT16U)frm->Data[3] << 8;
    IF1DTB1  = (CPU_INT16U)frm->Data[4] | (CPU_INT16U)frm->Data[5] << 8;
    IF1DTB2  = (CPU_INT16U)frm->Data[6] | (CPU_INT16U)frm->Data[7] << 8;


    IF1CMSK  = 0xF7;                                  /* write mask, arb, ctrl, tx request, data  */

    IF1CREQ  = TxBuffer;                              /* transfer to message buffer               */

    TxBuffer++;                                       /* increment tx buffer for next transmission*/
    if (TxBuffer > LAST_TX) {                         /* check for overflow                       */
        TxBuffer = FIRST_TX;
    }
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */

    return (size);                                    /* Return function result                   */
}                                                     /*------------------------------------------*/

