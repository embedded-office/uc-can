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
#include "drv_def.h"                                  /* driver layer declarations                */
#include "can_bsp.h"                                  /* user definable definitions               */

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
* \ingroup  LM3S9B96_CAN
*
*           This variable holds the detailed errorcode, if an error is detected.
*/
/*------------------------------------------------------------------------------------------------*/
static CPU_INT16U DrvError;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER IDENTIFICATION
* \ingroup  LM3S9B96_CAN
*
*           This constant variable holds the unique driver identification code.
*/
/*------------------------------------------------------------------------------------------------*/
static const CPU_INT32U DrvIdent = 0x243F2A01;


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER RUNTIME DATA
* \ingroup  LM3S9B96_CAN
*
*           Can table.
*/
/*------------------------------------------------------------------------------------------------*/
static CAN_DATA CanTbl[LM3S9B96_CAN_N_DEV];

/*
****************************************************************************************************
*                                       LOCAL FUNCTIONS
****************************************************************************************************
*/

void       LM3S9B96CAN_SetUnusedMsgObj     (CPU_INT16S paraId);
CPU_INT32S LM3S9B96CAN_SetTxMsgObj         (CPU_INT16S paraId);
CPU_INT32S LM3S9B96CAN_SetRxMsgObj         (CPU_INT16S paraId, CPU_INT32U msg_obj,CPU_BOOLEAN eob);

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      SEARCHS THE FIRST FREE MESSAGE INTERFACE,
*
*           Searchs the first free message interface, starting from 0.
*
* \return   A free message interface number (0 or 1) if found, else 2
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT32S LM3S9B96CAN_GetFreeIF (CPU_INT16S paraId)
{
    CPU_INT32S      result = -1;                      /* Local: Function result                   */
    CAN_LM3S9B96    *lm3S9B96;                        /* Local: pointer to CAN device             */
                                                      /*------------------------------------------*/
    lm3S9B96 = (CAN_LM3S9B96 *)CanTbl[paraId].BaseAddr;
                                                      /*------------------------------------------*/

    if ((lm3S9B96->MsgObj[0].CRR & CAN_CRR_BUSY) == 0) {
        result = 0;
    } else if ((lm3S9B96->MsgObj[1].CRR & CAN_CRR_BUSY) == 0) {
        result = 1;
    } else {
        result = 2;
    }
    return result;
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      WAIT UNTIL MESSAGE OBJECT IS READY
* \ingroup  LM3S9B96
*
*           Wait untile write operation to message object is finished
*
* \param    msg_if           specifies the Message interface
*
* \return   -1 on error, else 0
*/
/*------------------------------------------------------------------------------------------------*/
void LM3S9B96CAN_WaitUntilReady (CPU_INT16S paraId, CPU_INT32U  msg_if)
{
    CPU_INT32U i = 0;                                 /* Local: loop index                        */
    CAN_LM3S9B96    *lm3S9B96;                        /* Local: pointer to CAN device             */
                                                      /*------------------------------------------*/
    lm3S9B96 = (CAN_LM3S9B96 *)CanTbl[paraId].BaseAddr;


    while (((lm3S9B96->MsgObj[msg_if].CRR & CAN_CRR_BUSY) != 0) &&
            (i < 100)) {
        i++;
    }
    if (i >= 100) {
        DrvError = LM3S9B96_CAN_BUSY_ERR;
    }
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CONFIGURES THE MESSAGE OBJECT AS UNUSED
* \ingroup  LM3S9B96
*
*           Configures the message object as unused
*
*/
/*------------------------------------------------------------------------------------------------*/
void LM3S9B96CAN_SetUnusedMsgObj (CPU_INT16S paraId)
{
    CPU_INT32U      msg_if = 0;                       /* Local: mesage interface                  */
    CPU_INT32U      i      = 0;                       /* Local: loop variable                     */
    CAN_LM3S9B96   *lm3S9B96;                         /* Local: pointer to CAN device             */
                                                      /*------------------------------------------*/
    lm3S9B96 = (CAN_LM3S9B96 *)CanTbl[paraId].BaseAddr;

    for (i = 0; i < 32; i++) {

        msg_if = LM3S9B96CAN_GetFreeIF(paraId);

        if (msg_if != 2) {

            lm3S9B96->MsgObj[msg_if].CMR = CAN_CMR_WRRD
                                     | CAN_CMR_MASK
                                     | CAN_CMR_ARB
                                     | CAN_CMR_CONTROL
                                     | CAN_CMR_DATAA
                                     | CAN_CMR_DATAB
                                     | CAN_CMR_CLRINTPND;

            lm3S9B96->MsgObj[msg_if].M1R = 0;
            lm3S9B96->MsgObj[msg_if].M2R = 0;

            lm3S9B96->MsgObj[msg_if].A1R = 0;
            lm3S9B96->MsgObj[msg_if].A2R = 0;

            lm3S9B96->MsgObj[msg_if].MCR = 0;

            lm3S9B96->MsgObj[msg_if].DA1R = 0;
            lm3S9B96->MsgObj[msg_if].DA2R = 0;
            lm3S9B96->MsgObj[msg_if].DB1R = 0;
            lm3S9B96->MsgObj[msg_if].DB2R = 0;

            lm3S9B96->MsgObj[msg_if].CRR = 1 + i;

            lm3S9B96->MsgObj[msg_if].CMR = CAN_CMR_CLRINTPND; /* clear pending irq of msg pbject         */
            lm3S9B96->MsgObj[msg_if].CRR = 1 + i;

         }
    }
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CONFIGURES A TX  MESSAGE OBJECT
* \ingroup  LM3S9B96
*
*           Configures a TX message object
*
* \return   -1 on error, else 0
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT32S LM3S9B96CAN_SetTxMsgObj (CPU_INT16S paraId)
{
    CPU_INT32S      result = -1;                      /* Local: Function result                   */
    CPU_INT32U      msg_if = 0;                       /* Local: message interface                 */
    CAN_LM3S9B96   *lm3S9B96;                         /* Local: pointer to CAN device             */
                                                      /*------------------------------------------*/
    lm3S9B96 = (CAN_LM3S9B96 *)CanTbl[paraId].BaseAddr;
                                                      /*------------------------------------------*/

    msg_if = LM3S9B96CAN_GetFreeIF(paraId);

    if (msg_if != 2) {

        lm3S9B96->MsgObj[msg_if].CMR = CAN_CMR_WRRD
                                 | CAN_CMR_MASK
                                 | CAN_CMR_ARB
                                 | CAN_CMR_CONTROL
                                 | CAN_CMR_DATAA
                                 | CAN_CMR_DATAB;

        lm3S9B96->MsgObj[msg_if].M1R = 0;
        lm3S9B96->MsgObj[msg_if].A1R = 0;

        lm3S9B96->MsgObj[msg_if].M2R = CAN_M2R_MDIR;
        lm3S9B96->MsgObj[msg_if].A2R = CAN_A2R_MSGVAL | CAN_A2R_DIR;

        lm3S9B96->MsgObj[msg_if].MCR = CAN_MCR_TXIE | CAN_MCR_EOB;

        lm3S9B96->MsgObj[msg_if].DA1R = 0;
        lm3S9B96->MsgObj[msg_if].DA2R = 0;
        lm3S9B96->MsgObj[msg_if].DB1R = 0;
        lm3S9B96->MsgObj[msg_if].DB2R = 0;

        lm3S9B96->MsgObj[msg_if].CRR = 1 + LM3S9B96_CAN_TX_BUFFER;
        result = 0;
    }

  return result;
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CONFIGURES A RX  MESSAGE OBJECT
* \ingroup  LM3S9B96
*
*           Configures a RX  message object
*
* \param    msg_obj           specifies the Message object number, from 0 to 31
* \param    eob               last fifo element must set eob bit
*
* \return   -1 on error, else 0
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT32S LM3S9B96CAN_SetRxMsgObj (CPU_INT16S paraId, CPU_INT32U msg_obj, CPU_BOOLEAN eob)
{
    CPU_INT32S      result  = -1;                     /* Local: Function result                   */
    CPU_INT32U      msg_if = 0;                       /* Local: message interface                 */
    CAN_LM3S9B96   *lm3S9B96;                         /* Local: pointer to CAN device             */
                                                      /*------------------------------------------*/
    lm3S9B96 = (CAN_LM3S9B96 *)CanTbl[paraId].BaseAddr;
                                                      /*------------------------------------------*/

    msg_if = LM3S9B96CAN_GetFreeIF(paraId);
    if (msg_if != 2) {

        lm3S9B96->MsgObj[msg_if].CMR = CAN_CMR_WRRD
                                 | CAN_CMR_MASK
                                 | CAN_CMR_ARB
                                 | CAN_CMR_CONTROL
                                 | CAN_CMR_DATAA
                                 | CAN_CMR_DATAB;

        lm3S9B96->MsgObj[msg_if].M1R = 0;             /* mask to be set via IoCtls                */
        lm3S9B96->MsgObj[msg_if].M2R = CAN_M2R_MXTD;

        lm3S9B96->MsgObj[msg_if].A1R = 0;             /* accept all                               */
        lm3S9B96->MsgObj[msg_if].A2R = CAN_A2R_MSGVAL | 0x1FFC;
        if (eob == 1) {
            lm3S9B96->MsgObj[msg_if].MCR = CAN_MCR_RXIE | /* enable rx interrupt                  */
                                        CAN_MCR_EOB  | /* single msg object                       */
                                        CAN_MCR_UMASK; /* use mask - accept all                   */
        } else {
            lm3S9B96->MsgObj[msg_if].MCR = CAN_MCR_RXIE | /* enable rx interrupt                  */
                                        CAN_MCR_UMASK; /* use mask - accept all                   */
        }
        lm3S9B96->MsgObj[msg_if].DA1R = 0;
        lm3S9B96->MsgObj[msg_if].DA2R = 0;
        lm3S9B96->MsgObj[msg_if].DB1R = 0;
        lm3S9B96->MsgObj[msg_if].DB2R = 0;

        lm3S9B96->MsgObj[msg_if].CRR = 1 + msg_obj;
        result = 0;
    }
    return result;
}
/*
****************************************************************************************************
*                                            FUNCTIONS
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN INITIALISATION
* \ingroup  LM3S9B96_CAN
*
*           Initializes the CAN module.
*
* \param    arg               identifies can device
*
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S LM3S9B96CANInit (CPU_INT32U arg)
{
    CPU_INT16S     result = LM3S9B96_CAN_NO_ERR;      /* Local: Function result                   */
    CPU_INT32U     i;                                 /* Local: loop variable                     */
    CAN_LM3S9B96  *lm3S9B96;                          /* Local: pointer to CAN device             */
                                                      /*------------------------------------------*/
#if LM3S9B96_CAN_ARG_CHK_CFG > 0
    if ((CPU_INT08U)arg >= LM3S9B96_CAN_N_DEV) {      /* check that device name is in range       */
        DrvError= LM3S9B96_CAN_INIT_ERR;
        return(-1);                                   /* return function result                   */
    }
#endif

    if (arg == LM3S9B96_CAN_BUS_0) {
        CanTbl[arg].BaseAddr = LM3S9B96_CAN0_BASE_ADDR;
    } else {
        CanTbl[arg].BaseAddr = LM3S9B96_CAN1_BASE_ADDR;
    }

    DrvError= LM3S9B96_CAN_NO_ERR;                    /* set to defined value                     */
    CanTbl[arg].Use = 0;
    lm3S9B96 = (CAN_LM3S9B96 *) CanTbl[arg].BaseAddr;
                                                      /*------------------------------------------*/
    LM3S9B96CANPinSettings(arg);
                                                      /*------------------------------------------*/
    lm3S9B96->CR = CAN_CR_CCE | CAN_CR_INIT;          /* Init mode and Configuration Change Enable */
    lm3S9B96->SR = 0;                                 /* reset the status                         */

                                                      /*------------------------------------------*/
                                                      /* Initialize baud rate parameter           */
    CanTbl[arg].Baudrate        = LM3S9B96CAN_DEF_BAUD_RATE;
    CanTbl[arg].SamplePoint     = LM3S9B96CAN_DEF_SP;
    CanTbl[arg].ResynchJumpWith = LM3S9B96CAN_DEF_RJW;

    result = LM3S9B96CAN_CalcTimingReg (&CanTbl[arg]);
    if (result != LM3S9B96_CAN_NO_ERR) {
        DrvError = LM3S9B96_CAN_INIT_ERR;
        result   = LM3S9B96_CAN_INIT_ERR;
    } else {

                                                      /* Set the bit rate register                */
        lm3S9B96->BTR = (CanTbl[arg].PRESDIV)     |
                        (CanTbl[arg].RJW   << 6)  |
                        (CanTbl[arg].PSEG1 << 8)  |
                        (CanTbl[arg].PSEG2 << 12);

        lm3S9B96->BRPR = 0;                           /* clear the Extended Baud Rate Prescaler   */
    }

    LM3S9B96CAN_SetUnusedMsgObj(arg);                 /* sets all msg object to unused status     */
    LM3S9B96CAN_SetTxMsgObj (arg);
    for (i=LM3S9B96_CAN_RX_BUFFER; i<LM3S9B96_CAN_SIZE_RX_FIFO-1; i++) {
        LM3S9B96CAN_SetRxMsgObj (arg, i, 0);
    }
    LM3S9B96CAN_SetRxMsgObj (arg, i, 1);              /* last fifo object                         */

                                                      /*------------------------------------------*/
#if LM3S9B96_CAN_INTERRUPT_EN > 0                     /* INTERRUPT Settings                       */
    lm3S9B96->CR |= CAN_CR_IE | CAN_CR_EIE;           /* enable interrupts in CAN module          */
    LM3S9B96CANIrqSettings (arg);
#endif

    return(result);                                   /* return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      OPEN THE CAN BUS
* \ingroup  LM3S9B96_CAN
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
CPU_INT16S LM3S9B96CANOpen(CPU_INT16S devId, CPU_INT32U devName, CPU_INT16U mode)
{
    CPU_INT16S result = -1;                           /* Local: Function result                   */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/

    mode  = mode;
#if LM3S9B96_CAN_ARG_CHK_CFG > 0
    if ((CPU_INT08U)devName >= LM3S9B96_CAN_N_DEV) {  /* check that device name is in range       */
        DrvError= LM3S9B96_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
    if (mode != DEV_RW) {                             /* mode not supported?                      */
        DrvError = LM3S9B96_CAN_MODE_ERR;
        return(result);
    }
#endif

#if (LM3S9B96_CAN_INTERRUPT_EN > 0)
                                                      /* store the received Node Id for the irqs  */
    LM3S9B96_BSP_SetDevIds ((CPU_INT08U) devId,
                         (CPU_INT08U) devName);
#else
    devId = devId;                                    /* prevent compiler warning                 */
#endif

    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    if (CanTbl[devName].Use == 0) {                   /* Check, that device is not in use         */
        CanTbl[devName].Use = 1;                      /* mark can device as used                  */
        result = devName;                             /* Okay, device is opened                   */
    }                                                 /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return(result);                                   /* return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CLOSE THE CAN BUS
* \ingroup  LM3S9B96_CAN
*
*           Close the CAN bus
*
* \param    paraId            unused
*
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S LM3S9B96CANClose(CPU_INT16S paraId)
{
    CPU_INT16S result = -1;                           /* Local: Function result                   */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/

#if LM3S9B96_CAN_ARG_CHK_CFG > 0
                                                      /* check that paraId is in range            */
    if ((paraId >= LM3S9B96_CAN_N_DEV) || (paraId < 0)) {
        DrvError= LM3S9B96_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
#endif
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */                                                      /*------------------------------------------*/
    if (CanTbl[paraId].Use == 1) {                    /* Check, that device is in use             */
        CanTbl[paraId].Use = 0;                       /* mark can device as unused                */
        result = 0;                                   /* Okay, device is closed                   */
    } else {
        DrvError= LM3S9B96_CAN_CLOSE_ERR;             /* not opened - set close error             */
    }
                                                      /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return(result);                                   /* return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN I/O CONTROL
* \ingroup  LM3S9B96_CAN
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
CPU_INT16S LM3S9B96CANIoCtl (CPU_INT16S paraId, CPU_INT16U func, void* argp)
{
    CPU_INT32U      i;                                /* Local: counter variable                  */
    CPU_INT32U      msg_obj;                          /* Local: message object                    */
    CPU_INT32U      msg_if = 0;                       /* Local: message interface                 */
    CPU_INT32U      canId;                            /* Local: CAN Identifier                    */
    CPU_INT32U      mask;                             /* Local: Mask for identifier caluclations  */
    CPU_INT32U      status_reg;                       /* Local: status register value             */
    CPU_INT16S      result  = -1;                     /* Local: Function result                   */
    CAN_LM3S9B96    *lm3S9B96;                        /* Local: pointer to CAN device             */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/
    lm3S9B96 = (CAN_LM3S9B96    *)CanTbl[paraId].BaseAddr;


#if LM3S9B96_CAN_ARG_CHK_CFG > 0
    if ((paraId >= LM3S9B96_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range        */
        DrvError= LM3S9B96_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
    if (CanTbl[paraId].Use != 1) {                    /* check, that can device is opened         */
        DrvError= LM3S9B96_CAN_OPEN_ERR;
        return(result);                               /* return function result                   */
    }
#endif

    result = LM3S9B96_CAN_NO_ERR;
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    switch (func) {                                   /* select: function code                    */
        case IO_LM3S9B96_CAN_GET_IDENT:               /* GET IDENT                                */
            (*(CPU_INT32U*)argp) = DrvIdent;          /* return driver ident code                 */
            break;
                                                      /*------------------------------------------*/
        case IO_LM3S9B96_CAN_GET_ERRNO:               /* GET ERRORCODE                            */
            (*(CPU_INT16U*)argp) = DrvError;          /* return last detected errorcode           */
            break;
                                                      /*------------------------------------------*/
        case IO_LM3S9B96_CAN_GET_DRVNAME:             /* GET DRIVER NAME                          */
            (*(CPU_INT08U**)argp) = (CPU_INT08U*)     /* return human readable driver name        */
                                   LM3S9B96_CAN_NAME;
            break;
                                                      /*------------------------------------------*/
        case IO_LM3S9B96_CAN_SET_BAUDRATE:            /* Function: Set can bus baudrate           */
                                                      /*------------------------------------------*/

            lm3S9B96->CR = CAN_CR_CCE |               /* Init mode and Configuration Change Enable */
                           CAN_CR_INIT;
            lm3S9B96->SR = 0;                         /* reset the status                         */

                                                      /* Initialize baud rate parameter           */
            CanTbl[paraId].Baudrate = *(CPU_INT32U *)argp;

            result = LM3S9B96CAN_CalcTimingReg (&CanTbl[paraId]);
            if (result != LM3S9B96_CAN_NO_ERR) {
                DrvError = LM3S9B96_CAN_ARG_ERR;
                result   = LM3S9B96_CAN_ARG_ERR;
            } else {

                                                      /* Set the bit rate register                */
                lm3S9B96->BTR = (CanTbl[paraId].PRESDIV)     |
                                (CanTbl[paraId].RJW   << 6)  |
                                (CanTbl[paraId].PSEG1 << 8)  |
                                (CanTbl[paraId].PSEG2 << 12);

                lm3S9B96->BRPR = 0;                   /* clear the Extended Baud Rate Prescaler   */
            }                                         /*------------------------------------------*/
            lm3S9B96->CR &= ~(CAN_CR_INIT |           /* leave init mode and clear CCE            */
                              CAN_CR_CCE);
            break;                                    /*------------------------------------------*/
        case IO_LM3S9B96_CAN_START:                   /* Function: Start can bus communication    */
                                                      /*------------------------------------------*/
            lm3S9B96->CR &= ~(CAN_CR_INIT |           /* leave init mode and clear CCE            */
                              CAN_CR_CCE);
#if LM3S9B96_CAN_INTERRUPT_EN > 0
            lm3S9B96->CR |= CAN_CR_IE | CAN_CR_EIE;   /* enable interrupts in CAN module          */
#endif
          break;                                      /*------------------------------------------*/
        case IO_LM3S9B96_CAN_STOP:                    /* Function: Stop can bus communication     */
                                                      /*------------------------------------------*/
            lm3S9B96->CR = CAN_CR_INIT;               /* Set Init mode                            */
            break;                                    /*------------------------------------------*/

        case IO_LM3S9B96_CAN_RX_STANDARD:             /* Function: receive standard format ids    */
                                                      /*------------------------------------------*/

            for (msg_obj=LM3S9B96_CAN_RX_BUFFER; msg_obj<LM3S9B96_CAN_SIZE_RX_FIFO; msg_obj++) {
                msg_if = LM3S9B96CAN_GetFreeIF(paraId);
                if (msg_if != 2) {
                    lm3S9B96->MsgObj[msg_if].CMR = CAN_CMR_ARB;
                    lm3S9B96->MsgObj[msg_if].CRR = 1 + msg_obj; /* read arb bit from msg object   */
                    LM3S9B96CAN_WaitUntilReady(paraId, msg_if);
                    lm3S9B96->MsgObj[msg_if].CMR = CAN_CMR_WRRD |
                                                CAN_CMR_ARB;
                    lm3S9B96->MsgObj[msg_if].A2R &= ~CAN_A2R_XTD; /* clear extened bit            */
                    lm3S9B96->MsgObj[msg_if].CRR = 1 + msg_obj;   /* write back to msg object     */
                    LM3S9B96CAN_WaitUntilReady(paraId, msg_if);
                } else {
                    DrvError = LM3S9B96_CAN_BUSY_ERR;
                }
            }
            break;                                    /*------------------------------------------*/
        case IO_LM3S9B96_CAN_RX_EXTENDED:             /* Function: receive extended format ids    */
                                                      /*------------------------------------------*/
            for (msg_obj=LM3S9B96_CAN_RX_BUFFER; msg_obj<LM3S9B96_CAN_SIZE_RX_FIFO; msg_obj++) {
                msg_if = LM3S9B96CAN_GetFreeIF(paraId);
                if (msg_if != 2) {
                    lm3S9B96->MsgObj[msg_if].CMR = CAN_CMR_ARB;
                    lm3S9B96->MsgObj[msg_if].CRR = 1 + msg_obj; /* read arb and mask from msg object */
                    LM3S9B96CAN_WaitUntilReady(paraId, msg_if);
                    lm3S9B96->MsgObj[msg_if].CMR = CAN_CMR_WRRD |
                                                CAN_CMR_ARB;
                    lm3S9B96->MsgObj[msg_if].A2R |= CAN_A2R_XTD; /* set extended bit              */
                    lm3S9B96->MsgObj[msg_if].CRR = 1 + msg_obj;  /* write back to msg object      */
                    LM3S9B96CAN_WaitUntilReady(paraId, msg_if);
                } else {
                    DrvError = LM3S9B96_CAN_BUSY_ERR;
                }
            }
            break;                                    /*------------------------------------------*/
        case IO_LM3S9B96_CAN_TX_READY:
            if (LM3S9B96_CAN_TX_BUFFER <16) {
                                                      /* if ready to transmit                     */
                if ((lm3S9B96->TXR1R & (1 << LM3S9B96_CAN_TX_BUFFER)) != 0) {
                    *((CPU_INT08U *)argp) = 0;
                } else {
                    *((CPU_INT08U *)argp) = 1;
                }
            } else {
                                                      /* if ready to transmit                     */
                if ((lm3S9B96->TXR2R & (1 << (LM3S9B96_CAN_TX_BUFFER - 16))) != 0) {
                    *((CPU_INT08U *)argp) = 0;
                } else {
                    *((CPU_INT08U *)argp) = 1;
                }
            }

            break;                                    /*------------------------------------------*/
        case IO_LM3S9B96_CAN_GET_NODE_STATUS:         /* Function: Get Node Status                */
                                                      /*------------------------------------------*/
            status_reg = lm3S9B96->SR;

            if ((status_reg & 0x40) != 0) {           /* Bit 6 Error Status Bit                   */
                *((CPU_INT08U *)argp) = 1;
            }
            if ((status_reg & 0x80) != 0) {           /* Bit 7 Bus Off Bit                        */
                *((CPU_INT08U *)argp) = 2;
            }
            if ((status_reg & 0xC0) == 0) {           /* Bit 6/7 not set - bus active             */
                *((CPU_INT08U *)argp) = 0;
            }

            break;                                    /*------------------------------------------*/
         case IO_LM3S9B96_CAN_SET_RX_FILTER:          /* SET RX FILTER                            */
                                                      /*------------------------------------------*/
            mask  = ((CPU_INT32U*)argp)[0];
            canId = ((CPU_INT32U*)argp)[1];

            for (msg_obj=LM3S9B96_CAN_RX_BUFFER; msg_obj<LM3S9B96_CAN_SIZE_RX_FIFO; msg_obj++) {
                msg_if = LM3S9B96CAN_GetFreeIF(paraId);
                if (msg_if != 2) {

                    lm3S9B96->MsgObj[msg_if].CMR = CAN_CMR_CONTROL; /* read control reg           */
                    lm3S9B96->MsgObj[msg_if].CRR = 1 + msg_obj;
                    i = 0;
                    while (((lm3S9B96->MsgObj[msg_if].CRR & CAN_CRR_BUSY) != 0) &&
                            (i < 10)) {
                        i++;
                    }
                    if (i >= 10) {
                        DrvError = LM3S9B96_CAN_BUSY_ERR;
                        return LM3S9B96_CAN_BUSY_ERR;
                    }

                    lm3S9B96->MsgObj[msg_if].CMR = CAN_CMR_WRRD
                                             | CAN_CMR_MASK
                                             | CAN_CMR_ARB
                                             | CAN_CMR_CONTROL;

                    if ((canId & LM3S9B96_CAN_FF_FRAME_BIT) == 0) {
                        lm3S9B96->MsgObj[msg_if].M1R = 0;
                        lm3S9B96->MsgObj[msg_if].M2R = (mask & 0x7FF) << 2;;

                        lm3S9B96->MsgObj[msg_if].A1R = 0;
                        lm3S9B96->MsgObj[msg_if].A2R = CAN_A2R_MSGVAL | ((canId & 0x7FF) << 2);
                    } else {
                        lm3S9B96->MsgObj[msg_if].M1R = (mask & 0xFFFF);
                        lm3S9B96->MsgObj[msg_if].M2R = CAN_M2R_MXTD | (mask >> 16);

                        lm3S9B96->MsgObj[msg_if].A1R = (canId & 0xFFFF);
                        lm3S9B96->MsgObj[msg_if].A2R = CAN_A2R_MSGVAL | CAN_A2R_XTD | ((canId & 0x7FF) >> 16);
                    }

                    lm3S9B96->MsgObj[msg_if].MCR = (lm3S9B96->MsgObj[msg_if].MCR & CAN_MCR_EOB) |
                                                CAN_MCR_RXIE | /* enable rx interrupt             */
                                                CAN_MCR_UMASK;

                    lm3S9B96->MsgObj[msg_if].CRR = 1 + msg_obj;
                }
            }
            break;                                    /*------------------------------------------*/
        default:
            DrvError = LM3S9B96_CAN_FUNC_ERR;
            result = -1;
            break;

    }                                                 /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return result;                                    /* return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN READ DATA
* \ingroup  LM3S9B96_CAN
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
CPU_INT16S LM3S9B96CANRead (CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size)
{
    LM3S9B96_CANFRM *frm;                             /* Local: Pointer to can frame              */
    CPU_INT16S    result = -1;                        /* Local: Result of function                */
    CPU_INT32U    i;                                  /* Local: loop variable                     */
    CPU_INT32U    msg_obj;                            /* Local: message object                    */
    CPU_INT32U    msg_if = 0;                         /* Local: message interface                 */
    CAN_LM3S9B96    *lm3S9B96;                        /* Local: pointer to CAN device             */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/
    lm3S9B96 = (CAN_LM3S9B96    *)CanTbl[paraId].BaseAddr;


#if LM3S9B96_CAN_ARG_CHK_CFG > 0

    if ((paraId >= LM3S9B96_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range        */
        DrvError= LM3S9B96_CAN_BUS_ERR;
        return(result);
    }
    if (size != sizeof(LM3S9B96_CANFRM)) {            /* check that size is plausible             */
        DrvError = LM3S9B96_CAN_NO_DATA_ERR;
        return(result);
    }
    if (buffer == 0) {                                /* invalid buffer pointer                   */
        DrvError= LM3S9B96_CAN_ARG_ERR;
        return(result);
    }
    if (CanTbl[paraId].Use != 1) {                    /* check, that can device is opened         */
        DrvError= LM3S9B96_CAN_OPEN_ERR;
        return(result);
    }
#endif
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    msg_obj = 0xFF;
    for (i=LM3S9B96_CAN_RX_BUFFER; i<LM3S9B96_CAN_SIZE_RX_FIFO; i++) {
        if ((lm3S9B96->ND1R & (1 << i)) != 0) {       /* search fifo for new data                 */
            msg_obj = i;
            break;
        }
    }

    if (msg_obj < LM3S9B96_CAN_SIZE_RX_FIFO) {
        frm = (LM3S9B96_CANFRM *)buffer;              /* Set pointer to can frame                 */

        msg_if = LM3S9B96CAN_GetFreeIF(paraId);       /* get free interface                       */
        while (msg_if == 2) {
            msg_if = LM3S9B96CAN_GetFreeIF(paraId);
        }

        lm3S9B96->SR &= ~CAN_SR_RXOK;

                                                      /* read the message contents                */
        lm3S9B96->MsgObj[msg_if].CMR = CAN_CMR_MASK
                            | CAN_CMR_ARB
                            | CAN_CMR_CONTROL
                            | CAN_CMR_CLRINTPND       /* clear int pending bit                    */
                            | CAN_CMR_TXRQSTNEWDAT    /* clear new data bit                       */
                            | CAN_CMR_DATAA
                            | CAN_CMR_DATAB;

        lm3S9B96->MsgObj[msg_if].CRR = 1 + msg_obj;
        i = 0;
        while (((lm3S9B96->MsgObj[msg_if].CRR & CAN_CRR_BUSY) != 0) &&
                (i < 10)) {
            i++;
        }
        if (i >= 10) {
            DrvError = LM3S9B96_CAN_BUSY_ERR;
            return LM3S9B96_CAN_BUSY_ERR;
        }

        if ((lm3S9B96->MsgObj[msg_if].A2R & CAN_A2R_XTD) == 0) { /* standard ID                   */
            frm->Identifier = (lm3S9B96->MsgObj[msg_if].A2R >> 2) & 0x07FF;
        } else {                                              /* extended ID                      */
            frm->Identifier = ((CPU_INT32U)lm3S9B96->MsgObj[msg_if].A1R);
            frm->Identifier |= (((CPU_INT32U)lm3S9B96->MsgObj[msg_if].A2R) << 16) & 0x1FFFFFFFu;
                                                              /* mark std/extended frame          */
            frm->Identifier |= LM3S9B96_CAN_FF_FRAME_BIT;
        }

        frm->DLC = lm3S9B96->MsgObj[msg_if].MCR & 0x0F;

        frm->Data[0] = (CPU_INT08U) lm3S9B96->MsgObj[msg_if].DA1R;
        frm->Data[1] = (CPU_INT08U)(lm3S9B96->MsgObj[msg_if].DA1R >> 8);
        frm->Data[2] = (CPU_INT08U) lm3S9B96->MsgObj[msg_if].DA2R;
        frm->Data[3] = (CPU_INT08U)(lm3S9B96->MsgObj[msg_if].DA2R >> 8);
        frm->Data[4] = (CPU_INT08U) lm3S9B96->MsgObj[msg_if].DB1R;
        frm->Data[5] = (CPU_INT08U)(lm3S9B96->MsgObj[msg_if].DB1R >> 8);
        frm->Data[6] = (CPU_INT08U) lm3S9B96->MsgObj[msg_if].DB2R;
        frm->Data[7] = (CPU_INT08U)(lm3S9B96->MsgObj[msg_if].DB2R >> 8);

        result = size;                                /* set successfull result                   */

    } else {
        result = LM3S9B96_CAN_NO_DATA_ERR;
        DrvError = result;
    }

                                                      /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return(result);                                   /* Return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN WRITE DATA
* \ingroup  LM3S9B96_CAN
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
CPU_INT16S LM3S9B96CANWrite (CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size)
{
    LM3S9B96_CANFRM *frm;                             /* Local: Pointer to can frame              */
    CPU_INT16S       result = -1;                     /* Local: Result of function                */
    CPU_INT32U       cnt    = 0;                      /* Local: Result of function                */
    CPU_INT32U       msg_if = 0;                      /* Local: message interface                 */
    CAN_LM3S9B96    *lm3S9B96;                        /* Local: pointer to CAN device             */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/
    lm3S9B96 = (CAN_LM3S9B96    *)CanTbl[paraId].BaseAddr;

#if LM3S9B96_CAN_ARG_CHK_CFG > 0

    if ((paraId >= LM3S9B96_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range        */
        DrvError= LM3S9B96_CAN_BUS_ERR;
        return(result);
    }
    if (size != sizeof(LM3S9B96_CANFRM)) {            /* check that size is plausible             */
        DrvError = LM3S9B96_CAN_NO_DATA_ERR;
        return(result);
    }
    if (buffer == 0) {                                /* invalid buffer pointer                   */
        DrvError= LM3S9B96_CAN_ARG_ERR;
        return(result);
    }
    if (CanTbl[paraId].Use != 1) {                    /* check, that can device is opened         */
        DrvError= LM3S9B96_CAN_OPEN_ERR;
        return(result);
    }
#endif

    CPU_CRITICAL_ENTER();                             /* enter critical section                   */

    if (LM3S9B96_CAN_TX_BUFFER <16) {                 /* check availability of tx buffer          */
        if ((lm3S9B96->TXR1R & (1 << LM3S9B96_CAN_TX_BUFFER)) != 0) {
            DrvError = LM3S9B96_CAN_BUSY_ERR;
            return result;
        }
    } else {
        if ((lm3S9B96->TXR2R & (1 << (LM3S9B96_CAN_TX_BUFFER - 16))) != 0) {
            DrvError = LM3S9B96_CAN_BUSY_ERR;
            return result;
        }
    }

    frm = (LM3S9B96_CANFRM *)buffer;                  /* Set pointer to can frame                 */

    msg_if = LM3S9B96CAN_GetFreeIF(paraId);           /* get free interface to msg object         */
    while (msg_if == 2) {
        msg_if = LM3S9B96CAN_GetFreeIF(paraId);
    }
                                                      /* read the Arbitration and Message Control */
    lm3S9B96->MsgObj[msg_if].CMR = CAN_CMR_ARB | CAN_CMR_CONTROL;
    lm3S9B96->MsgObj[msg_if].CRR = 1 + LM3S9B96_CAN_TX_BUFFER;
                                                      /* wait until read action is completed      */
    while (((lm3S9B96->MsgObj[msg_if].CRR & CAN_CRR_BUSY) != 0) &&
            (cnt < 10)) {
        cnt++;
    }
    if (cnt >= 10) {
        DrvError = LM3S9B96_CAN_BUSY_ERR;
        return result;
    }
                                                      /* update the contents needed for transmission */
    lm3S9B96->MsgObj[msg_if].CMR = CAN_CMR_WRRD    |
                           CAN_CMR_ARB     |
                           CAN_CMR_CONTROL |
                           CAN_CMR_DATAA   |
                           CAN_CMR_DATAB;

                                                      /* settings for standard or extended id     */
    if (frm->Identifier <= 0x7FF) {                   /* if Id is standard Id                     */
      lm3S9B96->MsgObj[msg_if].A1R = 0;
      lm3S9B96->MsgObj[msg_if].A2R = (lm3S9B96->MsgObj[msg_if].A2R & 0xA000) |
                              STD_FIXED_ID_ARB(frm->Identifier);
    } else {                                          /* else if Id is extended Id                */
      lm3S9B96->MsgObj[msg_if].A1R = EXT_FIXED_ID_ARB_L(frm->Identifier);
      lm3S9B96->MsgObj[msg_if].A2R = (lm3S9B96->MsgObj[msg_if].A2R & 0xE000) |
                              CAN_A2R_XTD                    |
                              EXT_FIXED_ID_ARB_H(frm->Identifier);
    }

    lm3S9B96->MsgObj[msg_if].MCR = (lm3S9B96->MsgObj[msg_if].MCR & 0xFEF0) |
                            CAN_MCR_NEWDAT |
                            CAN_MCR_TXRQST |
                            frm->DLC;
                                                      /* copy data to msg object via interface reg */
    lm3S9B96->MsgObj[msg_if].DA1R = ((CPU_INT16U)frm->Data[1]<<8) | frm->Data[0];
    lm3S9B96->MsgObj[msg_if].DA2R = ((CPU_INT16U)frm->Data[3]<<8) | frm->Data[2];
    lm3S9B96->MsgObj[msg_if].DB1R = ((CPU_INT16U)frm->Data[5]<<8) | frm->Data[4];
    lm3S9B96->MsgObj[msg_if].DB2R = ((CPU_INT16U)frm->Data[7]<<8) | frm->Data[6];

                                                      /* execute request to update msg object     */
    lm3S9B96->MsgObj[msg_if].CRR = 1 + LM3S9B96_CAN_TX_BUFFER;

    result = size;                                    /* set successfull result                   */
                                                      /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return(result);                                   /* Return function result                   */
}

/*! } */

