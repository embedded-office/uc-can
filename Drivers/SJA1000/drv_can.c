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
#include "drv_can.h"
#include "can_bus.h"

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

/*
****************************************************************************************************
*                                             GLOBAL DATA
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER ERRORCODE
* \ingroup  SJA1000_CAN
*
*           This variable holds the detailed errorcode, if an error is detected.
*/
/*------------------------------------------------------------------------------------------------*/
static CPU_INT16U DrvError;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER IDENTIFICATION
* \ingroup  SJA1000_CAN
*
*           This constant variable holds the unique driver identification code.
*/
/*------------------------------------------------------------------------------------------------*/
static const CPU_INT32U DrvIdent = 0x243FF002;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER CONFIG DATA
* \ingroup  SJA1000_CAN
*
*           Baud rate table. Must be filled with desired baud rates before calling the ioctl-fct
*           IO_SJA1000_CAN_SET_BAUDRATE.
*/
/*------------------------------------------------------------------------------------------------*/
static const CAN_BAUD CanBaud[] =
{
    { 250000, SJA1000_CAN_BTR1, SJA1000_CAN_BTR2},    /* 250kBaud, SP=75%                         */
    {      0,    0,    0}                             /* baudrate = 0 marks end of baudrate table */
};


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER CONFIG DATA
* \ingroup  SJA1000_CAN
*
*           Base Address table. Must be filled with base addresses of all sja1000 can devices.
*           Second base address is commented out because default configuration in header file
*           sets SJA1000_CAN_N_DEV to 1.
*/
/*------------------------------------------------------------------------------------------------*/
static const CAN_ADDRESSES CanAddr[SJA1000_CAN_N_DEV] =
{
    SJA1000_CAN_BASE_ADDR1,
//    SJA1000_CAN_BASE_ADDR2
};

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER RUNTIME DATA
* \ingroup  SJA1000_CAN
*
*           Can table.
*/
/*------------------------------------------------------------------------------------------------*/
static CAN_DATA CanTbl[SJA1000_CAN_N_DEV];


/*
****************************************************************************************************
*                                            FUNCTIONS
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN INITIALISATION
* \ingroup  SJA1000_CAN
*
*           Initializes the CAN module.
*
* \param    arg               identifies can device
*
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S SJA1000CANInit (CPU_INT32U arg)
{
    CPU_INT16S result  =  SJA1000_CAN_NO_ERR;         /* Local: Function result                   */
    CPU_INT08U cnt     =  0;                          /* Local: Function counter                  */
    CPU_INT16U i;                                     /* Local: loop variable                     */
    SJA_1000  *sja1000;                               /* Local: pointer to sja1000 device         */
                                                      /*------------------------------------------*/

#if SJA1000_CAN_ARG_CHK_CFG > 0
    if ((CPU_INT08U)arg >= SJA1000_CAN_N_DEV) {       /* check that device name is in range       */
        DrvError= SJA1000_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
#endif

    DrvError= SJA1000_CAN_NO_ERR;                     /* set to defined value                     */

    sja1000 = (SJA_1000 *)CanAddr[arg].BaseAddress;
    CanTbl[arg].Use = 0;
                                                      /* set reset mode/request (Note: after      */
                                                      /* power-on SJA1000 is in BasicCAN mode);   */
    while ((sja1000->ModeControlReg & RM_RR_BIT ) == 0) {
        sja1000->ModeControlReg = RM_RR_BIT ;         /* if not in reset mode, set reset mode     */
        cnt++;                                        /* increment counter                        */
        if (cnt == 10) {                              /* check 10 times for reset                 */
            result = -1;                              /* no reset, set error                      */
            break;                                    /* break the loop                           */
        }
    }
                                                      /* select PeliCAN mode, comparator bit      */
    sja1000->ClockDivideReg = (CANMODE_BIT | CBP_BIT | DIVBY1);
                                                      /* enable, clockout = oscfreq               */
    sja1000->InterruptEnableReg = 0x00;               /* disable all CAN interrupts               */

    for (i=0;i<4;i++) {                               /* accept all                               */
        sja1000->shared.accept.AcceptanceCodeReg[i] = 0x00;
        sja1000->shared.accept.AcceptanceMaskReg[i] = 0xFF;
    }

    sja1000->BusTiming0Reg  = 0;                      /* must be set via io ctrl function         */
    sja1000->BusTiming1Reg  = 0;
    sja1000->OutputControlReg =                       /* configure CAN outputs: float on TX1,     */
        (TX1FLOAT | TX0PSHPULL | NORMALMODE);         /* Push/Pull on TX0, normal output mode     */
                                                      /* don't exit reset mode here               */

#if SJA1000_CAN_INTERRUPT_EN > 0
    sja1000->InterruptEnableReg |= RIE_BIT;           /* enable rx interrupt                      */
    sja1000->InterruptEnableReg |= TIE_BIT;           /* enable tx interrupt                      */
    sja1000->InterruptEnableReg |=
        (EIE_BIT | EPIE_BIT | BEIE_BIT);              /* enable error interrupts                  */
#endif

    return(result);                                   /* return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      OPEN THE CAN BUS
* \ingroup  SJA1000_CAN
*
*           Open the CAN Bus
*
* \param    devId             unused
* \param    devName           the bus id
* \param    mode              unused
*
* \return   parameter identifier for further access or -1 if an error occurs
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S SJA1000CANOpen(CPU_INT16S devId, CPU_INT32U devName, CPU_INT16U mode)
{
    CPU_INT16S result = -1;                           /* Local: Function result                   */
                                                      /*------------------------------------------*/
#if (CPU_CFG_CRITICAL_METHOD == 3)
    CPU_INT32U  cpu_sr;
#endif

    mode  = mode;
    devId = devId;
#if SJA1000_CAN_ARG_CHK_CFG > 0
    if ((CPU_INT08U)devName >= SJA1000_CAN_N_DEV) {   /* check that device name is in range       */
        DrvError= SJA1000_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
#endif
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    if (CanTbl[devName].Use == DEF_FALSE) {           /* Check, that device is not in use         */
        CanTbl[devName].Use =  DEF_TRUE;              /* mark can device as used                  */
        result = devName;                             /* Okay, device is opened                   */
    }                                                 /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return(result);                                   /* return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CLOSE THE CAN BUS
* \ingroup  SJA1000_CAN
*
*           Close the CAN bus
*
* \param    paraId            unused
*
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S SJA1000CANClose(CPU_INT16S paraId)
{
    CPU_INT16S result = -1;                           /* Local: Function result                   */

#if (CPU_CFG_CRITICAL_METHOD == 3)
    CPU_INT32U  cpu_sr;
#endif

#if SJA1000_CAN_ARG_CHK_CFG > 0
                                                      /* check that paraId is in range            */
    if ((paraId >= SJA1000_CAN_N_DEV) || (paraId < 0)) {
        DrvError= SJA1000_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
#endif
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */                                                      /*------------------------------------------*/
    if (CanTbl[paraId].Use == DEF_TRUE) {             /* Check, that device is in use             */
        CanTbl[paraId].Use =  DEF_FALSE;              /* mark can device as unused                */
        result = 0;                                   /* Okay, device is closed                   */
    }                                                 /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return(result);                                   /* return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN I/O CONTROL
* \ingroup  SJA1000_CAN
*
*           This function performs a special action on the opened device. The functioncode 'func'
*           defines what the caller want to do. Description of functioncodes as defined in
*           headerfile.
*
* \param    paraId            unused
* \param    func              function code
* \param    arg               argument list, specific to the function code
*
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S SJA1000CANIoCtl (CPU_INT16S paraId, CPU_INT16U func, void* argp)
{
    const CAN_BAUD *btbl;                             /* Local: Pointer to can bus baudrate table */
    CPU_INT08U      cnt;                              /* Local: counter used by Macro             */
    CPU_INT16S      result  = -1;                     /* Local: Function result                   */
    CPU_INT32U      canId;                            /* Local: CAN Identifier                    */
    CPU_INT32U      mask;                             /* Local: Mask for identifier caluclations  */
    SJA_1000       *sja1000 =
    (SJA_1000 *)CanAddr[paraId].BaseAddress;          /* Local: pointer to sja1000 device         */
                                                      /*------------------------------------------*/
#if (CPU_CFG_CRITICAL_METHOD == 3)
    CPU_INT32U  cpu_sr;
#endif

#if SJA1000_CAN_ARG_CHK_CFG > 0
    if ((paraId >= SJA1000_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range         */
        DrvError= SJA1000_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
    if (CanTbl[paraId].Use != 1) {                    /* check, that can device is opened         */
        DrvError= SJA1000_CAN_OPEN_ERR;
        return(result);                               /* return function result                   */
    }
#endif

    result = SJA1000_CAN_NO_ERR;
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */                                                      /*------------------------------------------*/
    switch (func) {                                   /* select: function code                    */
        case IO_SJA1000_CAN_GET_IDENT:                /* GET IDENT                                */
            (*(CPU_INT32U*)argp) = DrvIdent;          /* return driver ident code                 */
            break;
                                                      /*------------------------------------------*/
        case IO_SJA1000_CAN_GET_ERRNO:                /* GET ERRORCODE                            */
            (*(CPU_INT16U*)argp) = DrvError;          /* return last detected errorcode           */
            break;
                                                      /*------------------------------------------*/
        case IO_SJA1000_CAN_GET_DRVNAME:              /* GET DRIVER NAME                          */
            (*(CPU_INT08U**)argp) = (CPU_INT08U*)      /* return human readable driver name        */
                                   SJA1000_CAN_NAME;
            break;
                                                      /*------------------------------------------*/
        case IO_SJA1000_CAN_SET_BAUDRATE:             /* Function: Set can bus baudrate           */
                                                      /*------------------------------------------*/
            btbl = &CanBaud[0];                       /* Set pointer to can bus baudrate table    */
            while (btbl->Baudrate != 0L) {            /* Loop through whole baudrate table        */
                                                      /*------------------------------------------*/
                                                      /* if baudrate matches given argument       */
                if (btbl->Baudrate == (*((CPU_INT32U*)argp))) {
                                                      /*------------------------------------------*/
                    result = 0;                       /* Indicate sucessfull function execution   */
                    SetCanResetMode();                /* enter reset mode                         */
                    sja1000->BusTiming0Reg = btbl->CANBT1; /* Write value to BTR register 0       */
                    sja1000->BusTiming1Reg = btbl->CANBT2; /* Write value to BTR register 1       */
                                                      /* don't exit reset mode                    */
                    break;                            /* break loop                               */
                }                                     /*------------------------------------------*/
                btbl++;                               /* increment baudrate table pointer         */
            }                                         /*------------------------------------------*/
            break;                                    /*------------------------------------------*/
        case IO_SJA1000_CAN_START:                    /* Function: Start can bus communication    */
                                                      /*------------------------------------------*/
            ClearCanResetMode();                      /* exit reset mode                          */
            sja1000->ModeControlReg = 0x80;           /* set single filter mode                   */
            break;                                    /*------------------------------------------*/
        case IO_SJA1000_CAN_STOP:                     /* Function: Stop can bus communication     */
                                                      /*------------------------------------------*/
            SetCanResetMode();                        /* enter reset mode                         */
            break;                                    /*------------------------------------------*/

        case IO_SJA1000_CAN_RX_STANDARD:              /* Function: receive standard format ids    */
                                                      /*------------------------------------------*/
                                                      /* not possible for SJA1000                 */
            break;                                    /*------------------------------------------*/
        case IO_SJA1000_CAN_RX_EXTENDED:              /* Function: receive extended format ids    */
                                                      /*------------------------------------------*/
                                                      /* not possible for SJA1000                 */
            break;                                    /*------------------------------------------*/
        case IO_SJA1000_CAN_TX_READY:
            if ((sja1000->StatusReg & TBS_BIT) != 0) {/* if ready to transmit                     */
                *((CPU_INT08U *)argp) = 1;
            } else {
                *((CPU_INT08U *)argp) = 0;
            }
            break;                                    /*------------------------------------------*/
        case IO_SJA1000_CAN_GET_NODE_STATUS:          /* Function: Get Node Status                */
                                                      /*------------------------------------------*/
            (*((CPU_INT08U*)argp)) =
                    sja1000->StatusReg;               /* return status register                   */
            break;                                    /*------------------------------------------*/
        case IO_SJA1000_CAN_SET_RX_FILTER_1:          /*               SET RX FILTER              */
            mask  = ((CPU_INT32U*)argp)[0];
            canId = ((CPU_INT32U*)argp)[1];
            if (canId > 0x000007FF) {                 /* check for extended frame                 */
                mask  = mask  << 3;
                canId = canId << 3;
                SetCanResetMode();                    /* enter reset mode                         */

                sja1000->shared.accept.AcceptanceMaskReg[3] = (CPU_INT08U)  (mask & 0x000000FF);
                sja1000->shared.accept.AcceptanceMaskReg[2] = (CPU_INT08U) ((mask & 0x0000FF00) >> 8);
                sja1000->shared.accept.AcceptanceMaskReg[1] = (CPU_INT08U) ((mask & 0x00FF0000) >> 16);
                sja1000->shared.accept.AcceptanceMaskReg[0] = (CPU_INT08U) ((mask & 0xFF000000) >> 24);

                sja1000->shared.accept.AcceptanceCodeReg[3] = (CPU_INT08U)  (canId & 0x000000FF);
                sja1000->shared.accept.AcceptanceCodeReg[2] = (CPU_INT08U) ((canId & 0x0000FF00) >> 8);
                sja1000->shared.accept.AcceptanceCodeReg[1] = (CPU_INT08U) ((canId & 0x00FF0000) >> 16);
                sja1000->shared.accept.AcceptanceCodeReg[0] = (CPU_INT08U) ((canId & 0xFF000000) >> 24);

                sja1000->ModeControlReg = 0x9;        /* set single filter mode                   */
                ClearCanResetMode();                  /* exit reset mode                          */
            } else {
                mask  = mask  << 5;
                canId = canId << 5;
                SetCanResetMode();                    /* enter reset mode                         */

                sja1000->shared.accept.AcceptanceMaskReg[3] = (CPU_INT08U) 0xFF;
                sja1000->shared.accept.AcceptanceMaskReg[2] = (CPU_INT08U) 0xFF;
                sja1000->shared.accept.AcceptanceMaskReg[1] = (CPU_INT08U)  (mask & 0x000000FF);
                sja1000->shared.accept.AcceptanceMaskReg[0] = (CPU_INT08U) ((mask & 0x0000FF00) >> 8);

                sja1000->shared.accept.AcceptanceCodeReg[3] = (CPU_INT08U) 0;
                sja1000->shared.accept.AcceptanceCodeReg[2] = (CPU_INT08U) 0;
                sja1000->shared.accept.AcceptanceCodeReg[1] = (CPU_INT08U)  (canId & 0x000000FF);
                sja1000->shared.accept.AcceptanceCodeReg[0] = (CPU_INT08U) ((canId & 0x0000FF00) >> 8);

                sja1000->ModeControlReg = 0x9;        /* set single filter mode                   */
                ClearCanResetMode();                  /* exit reset mode                          */
            }

            break;
        default:
            result = SJA1000_CAN_FUNC_ERR;
            break;

    }                                                 /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return result;                                    /* return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN READ DATA
* \ingroup  SJA1000_CAN
*
*           Read a received message from a message buffer after checking, that this
*           device is opened.
*
* \param    paraId            unused, but needed for common interface
* \param    buffer            byte array for received data
* \param    size              length of byte array
*
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S SJA1000CANRead (CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size)
{
    CPU_INT08U       idb;                              /* Local: Number of identifier bytes        */
    SJA1000_CANFRM *frm;                              /* Local: Pointer to can frame              */
    CPU_INT08U       i;                                /* Local: Counter                           */
    CPU_INT16S      result = -1;                     /* Local: Result of function                */
    SJA_1000       *sja1000 =                         /* Local: pointer to sja1000 device         */
            (SJA_1000 *)CanAddr[paraId].BaseAddress;
                                                      /*------------------------------------------*/
#if (CPU_CFG_CRITICAL_METHOD == 3)
    CPU_INT32U  cpu_sr;
#endif

#if SJA1000_CAN_ARG_CHK_CFG > 0

    if ((paraId >= SJA1000_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range         */
        DrvError= SJA1000_CAN_BUS_ERR;
        return(result);
    }
    if (size != sizeof(SJA1000_CANFRM)) {             /* check that size is plausible             */
        DrvError = SJA1000_CAN_NO_DATA_ERR;
        return(result);
    }
    if (buffer == (void *)0) {                        /* invalid buffer pointer                   */
        DrvError= SJA1000_CAN_ARG_ERR;
        return(result);
    }
    if (CanTbl[paraId].Use != 1) {                    /* check, that can device is opened         */
        DrvError= SJA1000_CAN_OPEN_ERR;
        return(result);
    }
#endif
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */

    if ((sja1000->StatusReg & RBS_BIT) != 0x00) {     /* if data is received                      */

        frm = (SJA1000_CANFRM *)buffer;               /* Set pointer to can frame                 */
                                                      /* get number of received bytes             */
        frm->DLC = sja1000->shared.rx.RxFrameInfoReg & 0x0F;
                                                      /* check std or ext format                  */
        if ((sja1000->shared.rx.RxFrameInfoReg & EXT_FRAME_FORMAT) != 0x00) {
            idb = 4;
        } else {
            idb = 2;
        }

        frm->Identifier = 0;
        for (i = 0; i < idb; i++) {                   /* read  Identifier                         */
            frm->Identifier = frm->Identifier << 8;
            frm->Identifier |= (CPU_INT08U)sja1000->shared.rx.RxIdDataReg[i];
        }
                                                      /* prepare can ID according to frame format */
        if (idb == 2) {
            frm->Identifier =                         /* standard frame format, ID left jusified  */
                                                      (frm->Identifier & (CPU_INT32U)0xFFFF) >> 5;
        } else {
            frm->Identifier = (frm->Identifier &      /* extended frame format, ID left jusified  */
                               (CPU_INT32U)0xFFFFFFFF) >> 3;
            frm->Identifier |= (CPU_INT32U)(1L<<29);  /* set extended flag in identifier          */
        }

        for (i=0; i < frm->DLC; i++) {                /* read received bytes                      */
            frm->Data[i] = sja1000->shared.rx.RxIdDataReg[(idb+i)]; /* get message                */
        }

        sja1000->CommandReg = RRB_BIT;                /* release receive buffer bit               */
                                                      /* will also reset rx interrupt flag        */
        result = size;                                /* set successfull result                   */
    }                                                 /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return(result);                                   /* Return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN WRITE DATA
* \ingroup  SJA1000_CAN
*
*           Write a message to a message buffer after checking, that this
*           device is opened.
*
* \param    paraId            unused, but needed for common interface
* \param    buffer            byte array for transmitting data
* \param    size              length of byte array
*
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S SJA1000CANWrite (CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size)
{
    CPU_INT32U      id;                               /* Local: Variable for preparing id         */
    CPU_INT08U       idb;                              /* Local: Number of identifier bytes        */
    SJA1000_CANFRM *frm;                              /* Local: Pointer to can frame              */
    CPU_INT08U       i;                                /* Local: loop variable                     */
    CPU_INT16S      result  = -1;                     /* Local: Result of function                */
    SJA_1000       *sja1000 =                         /* Local: pointer to sja1000 device         */
              (SJA_1000 *)CanAddr[paraId].BaseAddress;
#if (CPU_CFG_CRITICAL_METHOD == 3)
    CPU_INT32U  cpu_sr;
#endif

#if SJA1000_CAN_ARG_CHK_CFG > 0

    if ((paraId >= SJA1000_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range         */
        DrvError= SJA1000_CAN_BUS_ERR;
        return(result);
    }
    if (size != sizeof(SJA1000_CANFRM)) {             /* check that size is plausible             */
        DrvError = SJA1000_CAN_NO_DATA_ERR;
        return(result);
    }
    if (buffer == (void *)0) {                        /* invalid buffer pointer                   */
        DrvError= SJA1000_CAN_ARG_ERR;
        return(result);
    }
    if (CanTbl[paraId].Use != 1) {                    /* check, that can device is opened         */
        DrvError= SJA1000_CAN_OPEN_ERR;
        return(result);
    }
#endif

    CPU_CRITICAL_ENTER();                             /* enter critical section                   */                                                      /*------------------------------------------*/

    if ((sja1000->StatusReg & TBS_BIT) != 0) {        /* if ready to transmit                     */
        frm = (SJA1000_CANFRM *)buffer;               /* Set pointer to can frame                 */
        if ((frm->Identifier & (1L<<29)) ||           /* check id for ext or std format           */
            (frm->Identifier > 0x0007FFL)) {
            idb   = 4;
            id = frm->Identifier << 3;
            sja1000->shared.tx.TxFrameInfoReg =       /* set extended frame format                */
                                                      (EXT_FRAME_FORMAT | frm->DLC);
        } else {
            idb   = 2;                                /* standard id                              */
            id = frm->Identifier << 5;
                                                      /* reset extended frame format              */
            sja1000->shared.tx.TxFrameInfoReg = frm->DLC;
        }
        i = 0;
        switch (idb) {
            case 4:
                sja1000->shared.tx.TxIdDataReg[i++] = (CPU_INT08U)((id & 0xFF000000) >> 24);
                sja1000->shared.tx.TxIdDataReg[i++] = (CPU_INT08U)((id & 0x00FF0000) >> 16);
            case 2:
                sja1000->shared.tx.TxIdDataReg[i++] = (CPU_INT08U)((id & 0x0000FF00) >> 8);
                sja1000->shared.tx.TxIdDataReg[i++] = (CPU_INT08U)(id & 0x000000FF);
                break;
        }
        for (i = 0; i < frm->DLC; i++) {              /* write data bytes                         */
            sja1000->shared.tx.TxIdDataReg[(idb + i)] = frm->Data[i];
        }
        sja1000->CommandReg = TR_BIT;                 /* start transmission                       */

        result = size;                                /* set successfull result                   */
    }                                                 /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */                                                      /*------------------------------------------*/

    return(result);                                   /* Return function result                   */
}


/*************************************************************************************************/
/*!
* \brief                      CAN INTERRUPT SERVICE ROUTINE
*
*           Interrupt Service Routine for CAN interrupt on SJA1000_CAN_BUS
*
*
* \param void
* \return void
*/
/*************************************************************************************************/
#if SJA1000_CAN_INTERRUPT_EN > 0
void SJA1000CANISR (void)
{
    volatile CPU_INT08U value;                        /* Local: variable for register content     */
    SJA_1000           *sja1000 =
             (SJA_1000 *)SJA1000_CAN_BASE_ADDR1;      /* Local: pointer to sja1000 device         */

    value = sja1000->InterruptReg;                    /* reading this register will reset all bits*/
                                                      /* except rx irq bit                        */
                                                      /* Acknowledge of rx irq bit is done in     */
                                                      /* read-fct by release of receive buffer    */

    if ((value & RI_BIT) != 0) {                      /* RX INTERRUPT                             */
        CanBusRxHandler (0);

    } else if ((value & TI_BIT) != 0) {               /* TX INTERRUPT                             */
        CanBusTxHandler (0);

    } else {                                          /* NS INTERRUPT                             */
        CanBusNSHandler (0);
    }


}
#endif



/*! } */

