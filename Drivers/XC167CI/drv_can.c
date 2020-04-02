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
#include "drv_can.h"                                  /* driver declarations                      */
#include "drv_can_reg.h"                              /* register declarations                    */
#include "drv_def.h"                                  /* common definitions                       */

/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/

/*
****************************************************************************************************
*                                              MACROS
****************************************************************************************************
*/


/*
****************************************************************************************************
*                                            LOCAL DATA
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER IDENTIFICATION
* \ingroup  XC167_CAN
*
*           This constant variable holds the unique driver identification code.
*/
/*------------------------------------------------------------------------------------------------*/
static const CPU_INT32U DrvIdent = (CPU_INT32U)0x243F1701;


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      GLOBAL CAN BUS BAUDRATE TABLE
* \ingroup  XC167_CAN
*
*           Constants calculated for 10Mhz clock and PLL factor 6 = Systemclock 60MHz
*/
/*------------------------------------------------------------------------------------------------*/
static const XC167_CAN_BAUD CanBaud[] =
{
#ifdef XC167_CAN_1000000_BAUD_CFG
    { 1000000, XC167_CAN_1000000_BAUD_CFG},         /* Enable 1000 kBaud Cfg.                   */
#endif
#ifdef XC167_CAN_727272_BAUD_CFG
    { 727272, XC167_CAN_727272_BAUD_CFG},           /* Enable 500 kBaud Cfg.                    */
#endif
#ifdef XC167_CAN_500000_BAUD_CFG
    { 500000, XC167_CAN_500000_BAUD_CFG},           /* Enable 500 kBaud Cfg.                    */
#endif
#ifdef XC167_CAN_250000_BAUD_CFG
    { 250000, XC167_CAN_250000_BAUD_CFG},           /* Enable 250 kBaud Cfg.                    */
#endif
#ifdef XC167_CAN_125000_BAUD_CFG
    { 125000, XC167_CAN_125000_BAUD_CFG},           /* Enable 125 kBaud Cfg.                    */
#endif
#ifdef XC167_CAN_100000_BAUD_CFG
    { 100000, XC167_CAN_100000_BAUD_CFG},           /* Enable 100 kBaud Cfg.                    */
#endif
#ifdef XC167_CAN_83333_BAUD_CFG
    { 83333, XC167_CAN_83333_BAUD_CFG},             /* Enable 83.333 kBaud Cfg.                 */
#endif
#ifdef XC167_CAN_50000_BAUD_CFG
    { 50000, XC167_CAN_50000_BAUD_CFG},             /* Enable 50 kBaud Cfg.                     */
#endif
    { 0,       0}                                     /* baudrate = 0 marks end of baudrate table */
};

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN IOCTL FUNCTIONTABLE
* \ingroup  XC167_CAN
*
*           This array holds the static CAN IO control function table.
*/
/*------------------------------------------------------------------------------------------------*/
extern _huge const CPU_INT16S (* _huge CANIoCtlFunc[IO_XC167_CAN_IOCTL_MAX])(CPU_INT16S, void *);


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER RUNTIME DATA
* \ingroup  XC167_CAN
*
*           Global can device data.
*/
/*------------------------------------------------------------------------------------------------*/
static XC167_CAN_DATA DevData[XC167_CAN_N_DEV];

/*
****************************************************************************************************
*                                             GLOBAL DATA
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER ERRORCODE
* \ingroup  XC167_CAN
*
*           This variable holds the detailed errorcode, if an error is detected.
*/
/*------------------------------------------------------------------------------------------------*/
static CPU_INT16U DrvError;


/*
****************************************************************************************************
*                                            FUNCTIONS
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN INITIALISATION
* \ingroup  XC167_CAN
*
*           Initializes the XC167 controllers if the devices are in reset mode.
*
* \param    arg               indicates can bus
*
* \return   errorcode (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANInit (CPU_INT32U arg)
{
    CPU_INT16S result    = 0;                         /* Local: Function result                   */
    MsgBOX_t  *MsgObject = (MsgBOX_t *)XC167_CAN_MSG0_BASEADDRESS;
                                                      /*------------------------------------------*/
    DrvError = XC167_CAN_NO_ERR,

    DevData[arg].Use = XC167_CAN_IDLE;

    XC167PinSetting ();
                                                      /*------------------------------------------*/
                                                      /* INTERRUPT SETTINGS                       */
    XC167_CAN_4IC = 0;                                /* disable error interrupt nodes            */
    XC167_CAN_5IC = 0;
    XC167_CAN_6IC = 0;                                /* disable unused interupt nodes            */
    XC167_CAN_7IC = 0;
                                                      /*------------------------------------------*/
                                                      /* ERROR INTERRUPT SETTINGS                 */
    XC167_CAN_A_GINP = 4;                             /* assign error int to node 4               */
    XC167_CAN_B_GINP = 5;                             /* assign error int to node 5               */

    XC167_CAN_A_IMR4 = 6;                             /* enable genration of INT ID for rx/tx/err */
    XC167_CAN_B_IMR4 = 6;

    switch (arg) {
                                                      /*------------------------------------------*/
        case 0 :                                      /* CAN A SETTINGS                           */
            XC167_CAN_0IC = 0;                        /* disable all can int nodes                */
            XC167_CAN_1IC = 0;
            XC167_CAN_A_CR = 0x41;                    /* stop bus activity, allow change of regs  */
                                                      /* and reset SIE/EIE/LECIE bits to 0        */
            XC167_CAN_A_IMRL0 = 0;                    /* disable all msg objs masks               */
            XC167_CAN_A_IMRH0 = 0;

            XC167_CAN_A_BTRL =                        /* BAUDRATE  SETTINGS                       */
                 XC167_CAN_DEFAULT_BAUD_RATE;         /* Set default baud rate                    */
            XC167_CAN_A_BTRH = 0;                     /* Loop-Back mode disabled                  */

            MsgObject[0].MSGCFGL  = 0x08;             /* set msg obj. 0 to transmit at can node a */
            MsgObject[1].MSGCFGL  = 0x00;             /* set msg obj. 1 to receive  at can node a */
            MsgObject[0].MSGCFGH  = 0x00;             /* select int node 0 for tx int             */
            MsgObject[1].MSGCFGH  = 0x01;             /* select int node 1 for rx int             */
            MsgObject[0].MSGFGCRL = 0;                /* std behaviour, no fifo, no gateway       */
            MsgObject[0].MSGFGCRH = 0;                /* set can pointer to itself                */
            MsgObject[1].MSGFGCRL = 0;                /* std behaviour, no fifo, no gateway       */
            MsgObject[1].MSGFGCRH = 1;                /* set can pointer to itself                */
            MsgObject[0].MSGAMRL  = 0x0000;           /* set msg acceptance mask to receive all   */
            MsgObject[0].MSGAMRH  = 0xE000;
            MsgObject[1].MSGAMRL  = 0x0000;           /* set msg acceptance mask to receive all   */
            MsgObject[1].MSGAMRH  = 0xE000;
            MsgObject[0].MSGCTRL  = 0x5595;           /* set msg obj. 0 to valid & disable TX int */
            MsgObject[1].MSGCTRL  = 0x5595;           /* set msg obj. 1 to valid & disable RX int */

#if XC167_CAN_RX1_ISR_CFG == 1
            MsgObject[1].MSGCTRL = XC167_CAN_MSGO_RXIE; /* set msg object 3 RXIE enable           */
#endif
#if XC167_CAN_TX1_ISR_CFG == 1
            MsgObject[0].MSGCTRL = XC167_CAN_MSGO_TXIE; /* set msg object 0 TXIE enable           */
#endif
#if XC167_CAN_NS_ISR_CFG  == 1
            XC167_CAN_A_CR |= 0x08;                   /* enable error interrupt                   */
#endif

#if (XC167_CAN_RX1_ISR_CFG == 1) || \
    (XC167_CAN_TX1_ISR_CFG == 1) || \
    (XC167_CAN_NS_ISR_CFG  == 1)
            XC167_CAN_0IC       = 0x48;               /* enable all can int nodes 0-3 at          */
            XC167_CAN_1IC       = 0x48;               /* prio level 2 group level 0               */
            XC167_CAN_4IC       = 0x48;
            XC167_CAN_A_IMRL0   = 0x3;                /* enable INTID generation for msg bf 0 & 1 */
#endif
            break;
                                                      /*------------------------------------------*/
        case 1:                                       /* CAN B SETTINGS                           */
            XC167_CAN_2IC = 0;                        /* disable all can int nodes                */
            XC167_CAN_3IC = 0;
            XC167_CAN_B_CR = 0x41;                    /* stop bus activity, allow change of regs  */
                                                      /* and reset SIE/EIE/LECIE bits to 0        */
            XC167_CAN_B_IMRL0 = 0;                    /* disable all msg objs masks               */
            XC167_CAN_B_IMRH0 = 0;

            XC167_CAN_B_BTRL =                        /* BAUDRATE  SETTINGS                       */
                 XC167_CAN_DEFAULT_BAUD_RATE;         /* Set default baud rate                    */
            XC167_CAN_B_BTRH = 0;                     /* Loop-Back mode disabled                  */

            MsgObject[2].MSGCFGL  = 0x0A;             /* set msg obj. 2 to transmit at can node b */
            MsgObject[3].MSGCFGL  = 0x02;             /* set msg obj. 3 to receive  at can node b */
            MsgObject[2].MSGCFGH  = 0x20;             /* select int node 2 for tx int             */
            MsgObject[3].MSGCFGH  = 0x03;             /* select int node 3 for rx int             */
            MsgObject[2].MSGFGCRL = 0;                /* std behaviour, no fifo, no gateway       */
            MsgObject[2].MSGFGCRH = 2;                /* set can pointer to itself                */
            MsgObject[3].MSGFGCRL = 0;                /* std behaviour, no fifo, no gateway       */
            MsgObject[3].MSGFGCRH = 3;                /* set can pointer to itself                */
            MsgObject[2].MSGAMRL  = 0x0000;           /* set msg acceptance mask to receive all   */
            MsgObject[2].MSGAMRH  = 0xE000;
            MsgObject[3].MSGAMRL  = 0x0000;           /* set msg acceptance mask to receive all   */
            MsgObject[3].MSGAMRH  = 0xE000;
            MsgObject[2].MSGCTRL  = 0x5595;           /* set msg obj. 2 to valid & disable TX int */
            MsgObject[3].MSGCTRL  = 0x5595;           /* set msg obj. 3 to valid & disable RX int */
#if XC167_CAN_RX2_ISR_CFG == 1
            MsgObject[3].MSGCTRL = XC167_CAN_MSGO_RXIE; /* set msg object 3 RXIE enable           */
#endif
#if XC167_CAN_TX2_ISR_CFG == 1
            MsgObject[2].MSGCTRL = XC167_CAN_MSGO_TXIE; /* set msg object 2 TXIE enable           */
#endif
#if XC167_CAN_NS_ISR_CFG  == 1
            XC167_CAN_A_CR |= 0x08;                   /* enable error interrupt                   */
#endif
#if (XC167_CAN_RX1_ISR_CFG == 1) || \
    (XC167_CAN_TX1_ISR_CFG == 1) || \
    (XC167_CAN_NS_ISR_CFG  == 1)
            XC167_CAN_2IC       = 0x48;               /* enable all can int nodes 0-3 at          */
            XC167_CAN_3IC       = 0x48;               /* prio level 2 group level 0               */
            XC167_CAN_5IC       = 0x48;
            XC167_CAN_B_IMRL0   = 0xC;                /* enable INTID generation for msg bf 2 & 3 */
#endif
            break;

        default:
            result = -1;
            break;
    }
    if (result < 0) {                                 /*------------------------------------------*/
        DrvError = XC167_CAN_INIT_ERR;
    }
                                                      /*------------------------------------------*/
    return result;                                    /* return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      OPEN THE CAN DEVICE
* \ingroup  XC167_CAN
*
*           Unlocks the device, i.e. IoCtl/Read/Write-function will take effect.
*
* \param    devId             unused
* \param    devName           the CAN device name
* \param    mode              the mode in which CAN devices will be used
*
* \return   the device identifier for further access or -1 if an error occurs
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANOpen (CPU_INT16S devId, CPU_INT32U devName, CPU_INT16U mode)
{
    XC167_CAN_DATA *can;                              /* Local: Pointer to can device             */
    CPU_INT16S result = -1;                           /* Local: Function result                   */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status register */
                                                      /*------------------------------------------*/

    devId = devId;                                    /* prevent compiler warning                 */
    mode = mode;                                      /* prevent compiler warning                 */
#if XC167_CAN_ARG_CHK_CFG > 0
    if ((CPU_INT08U)devName >= XC167_CAN_N_DEV) {     /* check that device name is in range       */
        DrvError = XC167_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
    if (mode != DEV_RW) {                             /* mode not supported?                      */
        DrvError = XC167_CAN_MODE_ERR;
        return(result);
    }
#endif
    can = &DevData[devName];                          /* set pointer to can device                */
    CPU_CRITICAL_ENTER();
    if (can->Use == XC167_CAN_IDLE) {                 /* check, that can device is unused         */
        can->Use = XC167_CAN_BUSY;                    /* mark can device as used                  */
        result = (CPU_INT08U)devName;                 /* Okay, device is opened                   */
    } else {
        DrvError = XC167_CAN_OPEN_ERR;
    }
    CPU_CRITICAL_EXIT();
                                                      /*------------------------------------------*/
    return(result);                                   /* return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CLOSE THE CAN DEVICE
* \ingroup  XC167_CAN
*
*           Locks the device, i.e. IoCtl/Read/Write-function will have no effect.
*
* \param    devId             the device identifier, returned by XC167CANOpen()
*
* \return   errorcode (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANClose (CPU_INT16S devId)
{
    XC167_CAN_DATA *can;                              /* Local: Pointer to can device             */
    CPU_INT16S      result = -1;                      /* Local: Function result                   */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status register */
                                                      /*------------------------------------------*/

#if XC167_CAN_ARG_CHK_CFG > 0
    if ((devId >= XC167_CAN_N_DEV) || (devId < 0)) {  /* check that devId is in range            */
        DrvError = XC167_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
#endif
    can = &DevData[devId];                            /* Set pointer to can device                */
    CPU_CRITICAL_ENTER();
    if (can->Use != XC167_CAN_IDLE) {                 /* check, that can device is used           */
        can->Use = XC167_CAN_IDLE;                    /* mark can device as unused                */
        result = XC167_CAN_NO_ERR;                    /* Indicate sucessfull function execution   */
    } else {
        DrvError = XC167_CAN_CLOSE_ERR;
    }
    CPU_CRITICAL_EXIT();
                                                      /*------------------------------------------*/
    return(result);                                   /* return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN I/O CONTROL
* \ingroup  XC167_CAN
*
*           This function performs a special action on the opened device. The functioncode func
*           defines what the caller want to do. Description of functioncodes as defined in
*           headerfile.
*
* \param    devId             device identifier, returned by XC167CANOpen()
* \param    func              function code
* \param    argp              argument list, specific to the function code
*
* \return   errorcode (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANIoCtl (CPU_INT16S devId, CPU_INT16U func, void *argp)
{
    CPU_INT16S        result = -1;                    /* Local: Function result                   */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status register */
                                                      /*------------------------------------------*/

#if XC167_CAN_ARG_CHK_CFG > 0                         /* Checking arguments (if enabled)          */
    XC167_CAN_DATA *can;                              /* Local: Pointer to can device             */
                                                      /*------------------------------------------*/
    if ((devId >= XC167_CAN_N_DEV) || (devId < 0)) {  /* check that devId is in range             */
        DrvError = XC167_CAN_BUS_ERR;
        return(result);                               /* Return function result                   */
    }
    can = &DevData[devId];                            /* set pointer to can device                */
    if (can->Use != XC167_CAN_BUSY) {                 /* check, that CAN device is opened         */
        DrvError = XC167_CAN_OPEN_ERR;
        return(result);                               /* Return function result                   */
    }
    if (func >= IO_XC167_CAN_IOCTL_MAX) {             /* Check if function code is in range       */
        DrvError = XC167_CAN_ARG_ERR;
        return(result);                               /* Return function result                   */
    }
#endif                                                /*------------------------------------------*/
    CPU_CRITICAL_ENTER();
    result = CANIoCtlFunc[func](devId, argp);         /* Call function code                       */
#if 0
    switch (func) {
        case 0:
    result = XC167CANIoCtlGetIdent(devId, argp);

        break;
        case 1:
    result = XC167CANIoCtlGetErrno(devId, argp);

        break;
        case 2:
    result = XC167CANIoCtlGetDrvName(devId, argp);

        break;
        case 3:
    result = XC167CANIoCtlSetBaudrate(devId, argp);

        break;
        case 4:
    result = XC167CANIoCtlStart(devId, argp);

        break;
        case 5:
   result =  XC167CANIoCtlStop(devId, argp);

        break;
        case 6:
    result = XC167CANIoCtlRxStandard(devId, argp);

        break;
        case 7:
   result =  XC167CANIoCtlRxExtended(devId, argp);

        break;
        case 8:
    result = XC167CANIoCtlGetTxReadyStatus(devId, argp);

        break;
        case 9:
    result = XC167CANIoCtlGetNodeStatus(devId, argp);

        break;
        case 10:
    result = XC167CANIoCtlSetRxFilter(devId, argp);

        break;
     }

#endif
    CPU_CRITICAL_EXIT();
    if (result < 0) {                                 /* Check if function result is 'no error'   */
        DrvError = XC167_CAN_FUNC_ERR;
    }
                                                      /*------------------------------------------*/
    return(result);                                   /* Return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      GET DRIVER IDENT CODE
* \ingroup  XC167_CAN
*
*           Return driver ident code.
*
* \note     There is no parameter check; the parameter shall be checked before given to this
*           algorithm!
*
* \param    devId             The device identifier returned by XC167CANOpen()
* \param    argp              Pointer to local ident variable (CPU_INT32U *)
*
* \return   errorcode (always 0)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANIoCtlGetIdent (CPU_INT16S devId, void *argp)
{
    devId = devId;                                    /* Prevent compiler warning                 */
    (*(CPU_INT32U*)argp) = DrvIdent;                  /* Return driver ident code                 */
                                                      /*------------------------------------------*/
    return(XC167_CAN_NO_ERR);                         /* Return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      GET ERRORCODE
* \ingroup  XC167_CAN
*
*           Return last detected errorcode.
*
* \note     There is no parameter check; the parameter shall be checked before given to this
*           algorithm!
*
* \param    devId             The device identifier returned by XC167CANOpen()
* \param    argp              Pointer to local errorcode variable (CPU_INT16U *)
*
* \return   errorcode (always 0)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANIoCtlGetErrno (CPU_INT16S devId, void *argp)
{
    devId = devId;                                    /* Prevent compiler warning                 */
#ifndef _DRV_ERR_H_
    (*(CPU_INT16U*)argp) = DrvError;                  /* Return last detected errorcode           */
#else
    argp = argp;                                      /* Prevent compiler warning                 */
#endif
                                                      /*------------------------------------------*/
    return(XC167_CAN_NO_ERR);                         /* Return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      GET DRIVER NAME
* \ingroup  XC167_CAN
*
*           Return the (human readable) driver name.
*
* \note     There is no parameter check; the parameter shall be checked before given to this
*           algorithm!
*
* \param    devId             The device identifier returned by XC167CANOpen()
* \param    argp              Pointer to local string variable (char *)
*
* \return   errorcode (always 0)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANIoCtlGetDrvName (CPU_INT16S devId, void *argp)
{
    volatile XC167_CAN_POINTER_KEYWORD CPU_INT08U* addr;
    devId = devId;                                    /* Prevent compiler warning                 */
                                                      /*------------------------------------------*/
                                                      /* Return human readable driver name        */
    addr = (XC167_CAN_POINTER_KEYWORD CPU_INT08U*) XC167_CAN_NAME;
    (*(XC167_CAN_POINTER_KEYWORD CPU_INT08U**)argp) = addr;
                                                      /*------------------------------------------*/
    return(XC167_CAN_NO_ERR);                         /* Return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      SET BUS BAUDRATE
* \ingroup  XC167_CAN
*
*           This function sets the bus baudrate.
*
* \note     There is no parameter check; the parameter shall be checked before given to this
*           algorithm!
*
* \param    devId             The device identifier returned by XC167CANOpen()
* \param    argp              Pointer to local baudrate variable (CPU_INT32U *)
*
* \return   errorcode (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANIoCtlSetBaudrate (CPU_INT16S devId, void *argp)
{
    const XC167_CAN_BAUD *btbl;                       /* Local: Pointer to can bus baudrate table */
    CPU_INT16S              result = -1;              /* Local: Function result                   */
                                                      /*------------------------------------------*/
    btbl = &CanBaud[0];                               /* Set pointer to can bus baudrate table    */
    while (btbl->Baudrate != 0L) {                    /* Loop through whole baudrate table        */
        if (btbl->Baudrate==(*((CPU_INT32U *)argp))) { /* if baudrate matches given argument      */
            switch (devId) {
                case XC167_CAN_BUS_0 :
                    XC167_CAN_A_BTRL = btbl->BTR;     /* Set  baud rate                           */
                    result = XC167_CAN_NO_ERR;        /* Indicate sucessfull function execution   */
                    break;
                case XC167_CAN_BUS_1 :
                    XC167_CAN_B_BTRL = btbl->BTR;     /* Set baud rate                            */
                    result = XC167_CAN_NO_ERR;        /* Indicate sucessfull function execution   */
                    break;
            }
        }
        btbl++;                                       /* increment baudrate table pointer         */
    }
                                                      /*------------------------------------------*/
    return(result);                                   /* Return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ENABLE BUS
* \ingroup  XC167_CAN
*
*           This function start the CAN controller interface.
*           Most common is to set the CAN controller in active mode.
*
* \note     There is no parameter check; the parameter shall be checked before given to this
*           algorithm!
*
* \param    devId             The device identifier returned by XC167CANOpen()
* \param    argp              Not used
*
* \return   errorcode (always 0)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANIoCtlStart (CPU_INT16S devId, void *argp)
{
    argp = argp;                                      /* Prevent compiler warning                 */
    switch (devId) {
        case XC167_CAN_BUS_0 :
            XC167_CAN_A_CR &= (CPU_INT16U)~0x41;      /* end of initialisation, go on bus         */
            break;
        case XC167_CAN_BUS_1 :
            XC167_CAN_B_CR &= (CPU_INT16U)~0x41;      /* end of initialisation, go on bus         */
            break;
    }
                                                      /*------------------------------------------*/
    return(XC167_CAN_NO_ERR);                         /* Return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DISABLE BUS
* \ingroup  XC167_CAN
*
*           This function stop the CAN controller interface.
*           Most common is to set the CAN controller in passive mode.
*
* \note     There is no parameter check; the parameter shall be checked before given to this
*           algorithm!
*
* \param    devId             The device identifier returned by XC167CANOpen()
* \param    argp              Not used
*
* \return   errorcode (always 0)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANIoCtlStop (CPU_INT16S devId, void *argp)
{
    argp = argp;                                      /* Prevent compiler warning                 */
    switch (devId) {
        case XC167_CAN_BUS_0 :
            XC167_CAN_A_CR   = 0x41;                  /* stop bus activity, allow change of regs  */
            break;
        case XC167_CAN_BUS_1 :
            XC167_CAN_B_CR   = 0x41;                  /* stop bus activity, allow change of regs  */
            break;
    }
                                                      /*------------------------------------------*/
    return(XC167_CAN_NO_ERR);                         /* Return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      SET RECEIVER TO STANDARD IDENTIFIER
* \ingroup  XC167_CAN
*
*           This function configure the CAN receiver to receive only CAN standard identifiers.
*
* \note     There is no parameter check; the parameter shall be checked before given to this
*           algorithm!
*
* \param    devId             The device identifier returned by XC167CANOpen()
* \param    argp              Not used
*
* \return   errorcode (always 0)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANIoCtlRxStandard (CPU_INT16S devId, void *argp)
{
    MsgBOX_t *MsgObject = (MsgBOX_t *)XC167_CAN_MSG0_BASEADDRESS;
                                                      /*------------------------------------------*/
    argp = argp;                                      /* Prevent compiler warning                 */
    switch (devId) {
        case XC167_CAN_BUS_0 :
            MsgObject[1].MSGCFGL &= ~XC167_CAN_XTD_ID; /* reset extended id bit                 */
            break;
        case XC167_CAN_BUS_1 :
            MsgObject[3].MSGCFGL &= ~XC167_CAN_XTD_ID; /* reset extended id bit                 */
            break;
    }
                                                      /*------------------------------------------*/
    return(XC167_CAN_NO_ERR);                         /* Return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      SET RECEIVER TO EXTENDED IDENTIFIER
* \ingroup  XC167_CAN
*
*           This function configure the CAN receiver to receive only CAN extended identifiers.
*
* \note     There is no parameter check; the parameter shall be checked before given to this
*           algorithm!
*
* \param    devId             The device identifier returned by XC167CANOpen()
* \param    argp              Not used
*
* \return   errorcode (always 0)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANIoCtlRxExtended (CPU_INT16S devId, void *argp)
{
    MsgBOX_t *MsgObject = (MsgBOX_t *)XC167_CAN_MSG0_BASEADDRESS;
                                                      /*------------------------------------------*/
    argp = argp;                                      /* Prevent compiler warning                 */
    switch (devId) {
        case XC167_CAN_BUS_0 :
            MsgObject[1].MSGCFGL |= XC167_CAN_XTD_ID; /* set extended id bit                    */
            break;
        case XC167_CAN_BUS_1 :
            MsgObject[3].MSGCFGL |= XC167_CAN_XTD_ID; /* set extended id bit                    */
            break;
    }
                                                      /*------------------------------------------*/
    return(XC167_CAN_NO_ERR);                         /* Return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      GET TX BUFFER READY STATUS
* \ingroup  XC167_CAN
*
*           This function shall set argp to 1 if the tx buffer is ready
*           to send a CAN frame. Otherwise it shall set argp to 0.
*
* \param    devId             The device identifier returned by XC167CANOpen()
* \param    argp              Pointer to status variable (CPU_INT08U *)
*
* \return   errorcode (always 0)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANIoCtlGetTxReadyStatus (CPU_INT16S devId, void *argp)
{
    MsgBOX_t *MsgObject = (MsgBOX_t *)XC167_CAN_MSG0_BASEADDRESS;
    CPU_INT16U bufId = 1;                             /* Local: buffer to be read from            */
                                                      /*------------------------------------------*/
    switch (devId) {
        case XC167_CAN_BUS_0:
            bufId = 0;
            break;
        case XC167_CAN_BUS_1:
            bufId = 2;
            break;
        default:
            break;
    }

    if ((MsgObject[bufId].MSGCTRL & XC167_CAN_TXRQ_MASK) == XC167_CAN_TX_BUSY) {
        *((CPU_INT08U *)argp) = 1;
    } else {
        *((CPU_INT08U *)argp) = 0;
    }
                                                      /*------------------------------------------*/
    return(XC167_CAN_NO_ERR);                         /* Return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      GET NODE STATUS
* \ingroup  XC167_CAN
*
*           This function get the node status from the CAN controller.
*
* \note     There is no parameter check; the parameter shall be checked before given to this
*           algorithm!
*
* \param    devId             The device identifier returned by XC167CANOpen()
* \param    argp              Pointer to a CPU_INT08U variable, where the status shall be written to
*
* \return   errorcode (always 0)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANIoCtlGetNodeStatus (CPU_INT16S devId, void *argp)
{
    CPU_INT08U status_reg;

    switch (devId) {
        case XC167_CAN_BUS_0 :
            status_reg = XC167_CAN_A_SR;              /* read status reg                          */
             if ((status_reg & 0x20) != 0) {          /* Bit 6 Error Status Bit                   */
                *((CPU_INT08U *)argp) = 1;
            }
            if ((status_reg & 0x40) != 0) {           /* Bit 7 Bus Off Bit                        */
                *((CPU_INT08U *)argp) = 2;
            }
            if ((status_reg & 0x60) == 0) {           /* Bit 6/7 not set - bus active             */
                *((CPU_INT08U *)argp) = 0;
            }
            break;
        case XC167_CAN_BUS_1 :
            status_reg = XC167_CAN_B_SR;              /* read status reg                          */
             if ((status_reg & 0x20) != 0) {          /* Bit 6 Error Status Bit                   */
                *((CPU_INT08U *)argp) = 1;
            }
            if ((status_reg & 0x40) != 0) {           /* Bit 7 Bus Off Bit                        */
                *((CPU_INT08U *)argp) = 2;
            }
            if ((status_reg & 0x60) == 0) {           /* Bit 6/7 not set - bus active             */
                *((CPU_INT08U *)argp) = 0;
            }
            break;
        default:
            break;
    }
                                                      /*------------------------------------------*/
    return(XC167_CAN_NO_ERR);                         /* Return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      SET RX ID FILTER
* \ingroup  XC167_CAN
*
*           This function sets the first identifier RX filter
*           The arguments are:
*            - the identifier mask
*            - the identifier
*
* \note     There is no parameter check; the parameter shall be checked before given to this
*           algorithm!
*
* \param    devId             The device identifier returned by XC167CANOpen()
* \param    argp              Pointer to local filter variables (CPU_INT32U *)
*
* \return   errorcode (always 0)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANIoCtlSetRxFilter (CPU_INT16S devId, void *argp)
{
    MsgBOX_t *MsgObject = (MsgBOX_t *)XC167_CAN_MSG0_BASEADDRESS;
    CPU_INT32U mask;                                  /* Local: Mask for identifier caluclations  */
    CPU_INT32U canId;                                 /* Local: CAN Identifier                    */
    CPU_INT16U bufId = 1;                             /* Local: buffer to be read from            */
                                                      /*------------------------------------------*/
    switch (devId) {
        case XC167_CAN_BUS_0:
            bufId = 1;
            break;
        case XC167_CAN_BUS_1:
            bufId = 3;
            break;
        default:
            break;
    }

    mask  = ((CPU_INT32U*)argp)[0];
    canId = ((CPU_INT32U*)argp)[1];
    if (canId <= 0x7FF) {                             /* standard id                              */
        MsgObject[bufId].MSGAMRH = mask << 2;         /* set mask                                 */
        MsgObject[bufId].MSGARH  = canId << 2;        /* set can id                               */
    } else {
        MsgObject[bufId].MSGAMRL = (mask & 0x0000FFFFL);  /* set mask                             */
        MsgObject[bufId].MSGAMRH = (mask & 0x1FFF0000L) >> 16;
        MsgObject[bufId].MSGARL  = (canId & 0x0000FFFFL); /* set can id                           */
        MsgObject[bufId].MSGARH  = (canId & 0x1FFF0000L);
    }
                                                      /*------------------------------------------*/
    return(XC167_CAN_NO_ERR);                         /* Return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      IOCTL FUNCTION NOT SUPPORTED
* \ingroup  XC167_CAN
*
*           This is the dummy function for all disabled IoCtl-Functions.
*           If a disabled IoCtl-Function is called, nothing can be done. The return value
*           is set to 'error'.
*
* \param    devId             Not used
* \param    argp              Not used
*
* \return   errorcode (always -1)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANIoCtlNotSupported (CPU_INT16S devId, void *argp)
{
    devId = devId;                                    /* Prevent compiler warning                 */
    argp  = argp;                                     /* Prevent compiler warning                 */
                                                      /*------------------------------------------*/
    return(-1);                                       /* Return function result 'error'           */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN READ DATA
* \ingroup  XC167_CAN
*
*           Read a received can frame from a message buffer. The buffer must have space for
*           only one can frame.
*
* \param    devId             parameter identifier, returned by XC167CANOpen()
* \param    buf               Pointer to can frame
* \param    size              Length of can frame memory
*
* \return   errorcode (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANRead (CPU_INT16S devId, CPU_INT08U *buf, CPU_INT16U size)
{
    XC167_CANFRM   *frm;                              /* Local: Pointer to can frame              */
    CPU_INT16S      result    = -1;                   /* Local: Function result                   */
    CPU_INT16U      bufId     =  1;                   /* Local: buffer to be read from            */
    MsgBOX_t       *MsgObject = (MsgBOX_t *) XC167_CAN_MSG0_BASEADDRESS;
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status register */

                                                      /*------------------------------------------*/
#if XC167_CAN_ARG_CHK_CFG > 0
    XC167_CAN_DATA *can;                              /* Local: Pointer to can device             */

    if ((devId >= XC167_CAN_N_DEV) || (devId < 0)) {  /* check that devId is in range             */
        DrvError = XC167_CAN_BUS_ERR;
        return(result);
    }
    if (size != sizeof(XC167_CANFRM)) {               /* check that size is plausible             */
        DrvError = XC167_CAN_NO_DATA_ERR;
        return(result);
    }
    if (buf == (void *)0) {                           /* invalid buffer pointer                   */
        DrvError = XC167_CAN_ARG_ERR;
        return(result);
    }
    can = &DevData[devId];                            /* set pointer to can device                */
    if (can->Use != XC167_CAN_BUSY) {                 /* check, that can device is opened         */
        DrvError = XC167_CAN_OPEN_ERR;
        return(result);
    }
#else
    size = size;                                      /* prevent compiler warning                 */
#endif

    CPU_CRITICAL_ENTER();
    switch (devId) {
        case XC167_CAN_BUS_0:
            bufId = 1;
            break;
        case XC167_CAN_BUS_1:
            bufId = 3;
            break;
        default:
            break;
    }

    frm = (XC167_CANFRM *)buf;
    frm->Identifier = 0;
                                                      /* Check if MsgBox is a configured for      */
                                                      /* receiption                               */
    if ((MsgObject[bufId].MSGCFGL & XC167_CAN_DIR_MASK) == 0) {
                                                      /* check if data is available               */
        if ((MsgObject[bufId].MSGCTRL & XC167_CAN_NEWDAT_MASK) == XC167_CAN_NEWDAT) {
            MsgObject[bufId].MSGCTRL = (CPU_INT16U)(~XC167_CAN_NEWDAT);
                                                      /* check if std. or ext identifier          */
            if ((MsgObject[bufId].MSGCFGL & XC167_CAN_XTD_ID) != 0) {
                frm->Identifier  = (CPU_INT32U) MsgObject[bufId].MSGARH << 16;
                frm->Identifier |= (CPU_INT32U) MsgObject[bufId].MSGARL;
            } else {                                  /* standard identifier                      */
                frm->Identifier = (CPU_INT32U) MsgObject[bufId].MSGARH >> 2;
                frm->Identifier &= 0x07FF;
            }

            frm->DLC = (MsgObject[bufId].MSGCFGL & XC167_CAN_DLC_MASK) >> 4; /* set DLC           */
                                                      /* copy data                                */
            frm->Data[0] = MsgObject[bufId].MSGDRL0  & 0x00FF;
            frm->Data[1] = (MsgObject[bufId].MSGDRL0 & 0xFF00) >> 8;
            frm->Data[2] = MsgObject[bufId].MSGDRH0  & 0x00FF;
            frm->Data[3] = (MsgObject[bufId].MSGDRH0 & 0xFF00) >> 8;
            frm->Data[4] = MsgObject[bufId].MSGDRL4  & 0x00FF;
            frm->Data[5] = (MsgObject[bufId].MSGDRL4 & 0xFF00) >> 8;
            frm->Data[6] = MsgObject[bufId].MSGDRH4  & 0x00FF;
            frm->Data[7] = (MsgObject[bufId].MSGDRH4 & 0xFF00) >> 8;
            result = sizeof(XC167_CANFRM);
        } else {
            DrvError = XC167_CAN_NO_DATA_ERR;
        }
    } else {
        DrvError = XC167_CAN_MODE_ERR;
    }
    CPU_CRITICAL_EXIT();

    return result;                                    /* Return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN WRITE DATA
* \ingroup  XC167_CAN
*
*           Write a can frame to a message buffer. The buffer must contain only one can frame,
*           which will be written to a predefined message buffer.
*
* \param    devId             parameter identifier, returned by XC167CANOpen()
* \param    buf               Pointer to can frame
* \param    size              Length of can frame memory
*
* \return   errorcode (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S XC167CANWrite (CPU_INT16S devId, CPU_INT08U *buf, CPU_INT16U size)
{
    XC167_CANFRM   *frm;                              /* Local: Pointer to can frame              */
    CPU_INT16S      result    = -1;
    CPU_INT16U      bufId     = 1;                    /* Local: buffer to be read from            */
    MsgBOX_t       *MsgObject = (MsgBOX_t *) XC167_CAN_MSG0_BASEADDRESS;
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status register */
                                                      /*------------------------------------------*/
#if XC167_CAN_ARG_CHK_CFG > 0
    XC167_CAN_DATA *can;                              /* Local: Pointer to can device             */

    if ((devId >= XC167_CAN_N_DEV) || (devId < 0)) {  /* check that devId is in range             */
        DrvError = XC167_CAN_BUS_ERR;
        return(result);
    }
    if (size != sizeof(XC167_CANFRM)) {               /* check that size is plausible             */
        DrvError = XC167_CAN_NO_DATA_ERR;
        return(result);
    }
    if (buf == (void *) 0) {                          /* invalid buffer pointer                   */
        DrvError = XC167_CAN_ARG_ERR;
        return(result);
    }
    can = &DevData[devId];                            /* set pointer to can device                */
    if (can->Use != XC167_CAN_BUSY) {                 /* check, that can device is opened         */
        DrvError = XC167_CAN_OPEN_ERR;
        return(result);
    }
#else
    size = size;                                      /* prevent compiler warning                 */
#endif

    CPU_CRITICAL_ENTER();
    switch (devId) {
        case XC167_CAN_BUS_0:
            bufId = 0;
            break;
        case XC167_CAN_BUS_1:
            bufId = 2;
            break;
        default:
            break;
    }
                                                      /* Check if MsgBox if configured for tx     */
    if ((MsgObject[bufId].MSGCFGL & XC167_CAN_DIR_MASK) != 0) {
        if ((MsgObject[bufId].MSGCTRL & XC167_CAN_TXRQ_MASK) == XC167_CAN_TX_BUSY) {

            frm = (XC167_CANFRM *)buf;                /* Set pointer to can frame                 */

            MsgObject[bufId].MSGCTRL  = XC167_CAN_START_TX_PROG;

            if (frm->Identifier <= 0x7FF) {           /* check if ID is an extended identifier    */
                MsgObject[bufId].MSGCFGL &= ~XC167_CAN_XTD_ID;  /* clear extended id bit          */
                MsgObject[bufId].MSGARH   = frm->Identifier << 2; /* Set standard ID              */
            } else {                                  /* Set extended ID                          */
                MsgObject[bufId].MSGCFGL |= XC167_CAN_XTD_ID; /* set extended id bit              */
                MsgObject[bufId].MSGARH   = (frm->Identifier & 0x1FFF0000L) >> 16;
                MsgObject[bufId].MSGARL   = (frm->Identifier & 0x0000FFFFL);
            }

            MsgObject[bufId].MSGCFGL &= 0xFF0F;       /* Set DLC                                  */
            MsgObject[bufId].MSGCFGL |= frm->DLC << 4;

            MsgObject[bufId].MSGDRL0  = (CPU_INT16U) frm->Data[0];
            MsgObject[bufId].MSGDRL0 |= (CPU_INT16U) frm->Data[1] << 8;
            MsgObject[bufId].MSGDRH0  = (CPU_INT16U) frm->Data[2];
            MsgObject[bufId].MSGDRH0 |= (CPU_INT16U) frm->Data[3] << 8;
            MsgObject[bufId].MSGDRL4  = (CPU_INT16U) frm->Data[4];
            MsgObject[bufId].MSGDRL4 |= (CPU_INT16U) frm->Data[5] << 8;
            MsgObject[bufId].MSGDRH4  = (CPU_INT16U) frm->Data[6];
            MsgObject[bufId].MSGDRH4 |= (CPU_INT16U) frm->Data[7] << 8;
            MsgObject[bufId].MSGCTRL  = XC167_CAN_END_TX_PROG;

            result = sizeof(XC167_CANFRM);

        } else {
            DrvError = XC167_CAN_BUSY_ERR;
        }
    } else {
        DrvError = XC167_CAN_MODE_ERR;
    }
    CPU_CRITICAL_EXIT();
                                                      /*------------------------------------------*/
    return result;                                    /* Return function result                   */
}

