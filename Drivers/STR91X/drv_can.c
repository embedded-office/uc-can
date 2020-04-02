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
#include "can_cfg.h"

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
* \ingroup  ST91X_CAN
*
*           This variable holds the detailed errorcode, if an error is detected.
*/
/*------------------------------------------------------------------------------------------------*/
static CPU_INT16U DrvError;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER IDENTIFICATION
* \ingroup  ST91X_CAN
*
*           This constant variable holds the unique driver identification code.
*/
/*------------------------------------------------------------------------------------------------*/
static const CPU_INT32U DrvIdent = 0x243F2101;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER CONFIG DATA
* \ingroup  ST91X_CAN
*
*           Baud rate table. Must be filled with desired baud rates before calling the ioctl-fct
*           IO_ST91X_CAN_SET_BAUDRATE.
*/
/*------------------------------------------------------------------------------------------------*/
extern const CAN_BAUD CanBaud[];

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER RUNTIME DATA
* \ingroup  ST91X_CAN
*
*           Can table.
*/
/*------------------------------------------------------------------------------------------------*/
static CAN_DATA CanTbl[ST91X_CAN_N_DEV];

/*
****************************************************************************************************
*                                       LOCAL FUNCTIONS
****************************************************************************************************
*/

void       ST91XCAN_SetUnusedMsgObj     (void);
CPU_INT32S ST91XCAN_SetTxMsgObj         (void);
CPU_INT32S ST91XCAN_SetRxMsgObj         (CPU_INT32U msg_obj,CPU_BOOLEAN eob);

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      SEARCHS THE FIRST FREE MESSAGE INTERFACE,
*
*           Searchs the first free message interface, starting from 0.
*
* \return   A free message interface number (0 or 1) if found, else 2
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT32S ST91XCAN_GetFreeIF (void)
{
    CPU_INT32S      result = -1;                      /* Local: Function result                   */
    CAN_ST91X      *st91x  =                          /* Local: pointer to CAN device             */
                (CAN_ST91X *)ST91X_CAN_BASE_ADDR;
                                                      /*------------------------------------------*/

    if ((st91x->MsgObj[0].CRR & CAN_CRR_BUSY) == 0) {
        result = 0;
    } else if ((st91x->MsgObj[1].CRR & CAN_CRR_BUSY) == 0) {
        result = 1;
    } else {
        result = 2;
    }
    return result;
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      WAIT UNTIL MESSAGE OBJECT IS READY
* \ingroup  ST91X
*
*           Wait untile write operation to message object is finished
*
* \param    msg_if           specifies the Message interface
*
* \return   -1 on error, else 0
*/
/*------------------------------------------------------------------------------------------------*/
void ST91XCAN_WaitUntilReady (CPU_INT32U  msg_if)
{
    CPU_INT32U i = 0;                                 /* Local: loop index                        */
    CAN_ST91X      *st91x  =                          /* Local: pointer to CAN device             */
                (CAN_ST91X *)ST91X_CAN_BASE_ADDR;
                                                      /*------------------------------------------*/

    while (((st91x->MsgObj[msg_if].CRR & CAN_CRR_BUSY) != 0) &&
            (i < 100)) {
        i++;
    }
    if (i >= 100) {
        DrvError = ST91X_CAN_BUSY_ERR;
    }
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CONFIGURES THE MESSAGE OBJECT AS UNUSED
* \ingroup  ST91X
*
*           Configures the message object as unused
*
*/
/*------------------------------------------------------------------------------------------------*/
void ST91XCAN_SetUnusedMsgObj (void)
{
    CPU_INT32U      msg_if = 0;                       /* Local: mesage interface                  */
    CPU_INT32U      i      = 0;                       /* Local: loop variable                     */
    CAN_ST91X      *st91x  =                          /* Local: pointer to CAN device             */
                (CAN_ST91X *)ST91X_CAN_BASE_ADDR;
                                                      /*------------------------------------------*/


    for (i = 0; i < 32; i++) {

        msg_if = ST91XCAN_GetFreeIF();

        if (msg_if != 2) {

            st91x->MsgObj[msg_if].CMR = CAN_CMR_WRRD
                                     | CAN_CMR_MASK
                                     | CAN_CMR_ARB
                                     | CAN_CMR_CONTROL
                                     | CAN_CMR_DATAA
                                     | CAN_CMR_DATAB
                                     | CAN_CMR_CLRINTPND;

            st91x->MsgObj[msg_if].M1R = 0;
            st91x->MsgObj[msg_if].M2R = 0;

            st91x->MsgObj[msg_if].A1R = 0;
            st91x->MsgObj[msg_if].A2R = 0;

            st91x->MsgObj[msg_if].MCR = 0;

            st91x->MsgObj[msg_if].DA1R = 0;
            st91x->MsgObj[msg_if].DA2R = 0;
            st91x->MsgObj[msg_if].DB1R = 0;
            st91x->MsgObj[msg_if].DB2R = 0;

            st91x->MsgObj[msg_if].CRR = 1 + i;

            st91x->MsgObj[msg_if].CMR = CAN_CMR_CLRINTPND; /* clear pending irq of msg pbject         */
            st91x->MsgObj[msg_if].CRR = 1 + i;

         }
    }
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CONFIGURES A TX  MESSAGE OBJECT
* \ingroup  ST91X
*
*           Configures a TX message object
*
* \return   -1 on error, else 0
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT32S ST91XCAN_SetTxMsgObj (void)
{
    CPU_INT32S      result = -1;                      /* Local: Function result                   */
    CPU_INT32U      msg_if = 0;                       /* Local: message interface                 */
    CAN_ST91X      *st91x =                           /* Local: pointer to CAN device             */
                (CAN_ST91X *)ST91X_CAN_BASE_ADDR;
                                                      /*------------------------------------------*/

    msg_if = ST91XCAN_GetFreeIF();

    if (msg_if != 2) {

        st91x->MsgObj[msg_if].CMR = CAN_CMR_WRRD
                                 | CAN_CMR_MASK
                                 | CAN_CMR_ARB
                                 | CAN_CMR_CONTROL
                                 | CAN_CMR_DATAA
                                 | CAN_CMR_DATAB;

        st91x->MsgObj[msg_if].M1R = 0;
        st91x->MsgObj[msg_if].A1R = 0;

        st91x->MsgObj[msg_if].M2R = CAN_M2R_MDIR;
        st91x->MsgObj[msg_if].A2R = CAN_A2R_MSGVAL | CAN_A2R_DIR;

        st91x->MsgObj[msg_if].MCR = CAN_MCR_TXIE | CAN_MCR_EOB;

        st91x->MsgObj[msg_if].DA1R = 0;
        st91x->MsgObj[msg_if].DA2R = 0;
        st91x->MsgObj[msg_if].DB1R = 0;
        st91x->MsgObj[msg_if].DB2R = 0;

        st91x->MsgObj[msg_if].CRR = 1 + ST91X_CAN_TX_BUFFER;
        result = 0;
    }

  return result;
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CONFIGURES A RX  MESSAGE OBJECT
* \ingroup  ST91X
*
*           Configures a RX  message object
*
* \param    msg_obj           specifies the Message object number, from 0 to 31
* \param    eob               last fifo element must set eob bit
*
* \return   -1 on error, else 0
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT32S ST91XCAN_SetRxMsgObj (CPU_INT32U msg_obj, CPU_BOOLEAN eob)
{
    CPU_INT32S      result  = -1;                     /* Local: Function result                   */
    CPU_INT32U      msg_if = 0;                       /* Local: message interface                 */
    CAN_ST91X      *st91x =                           /* Local: pointer to CAN device             */
                (CAN_ST91X *)ST91X_CAN_BASE_ADDR;
                                                      /*------------------------------------------*/

    msg_if = ST91XCAN_GetFreeIF();
    if (msg_if != 2) {

        st91x->MsgObj[msg_if].CMR = CAN_CMR_WRRD
                                 | CAN_CMR_MASK
                                 | CAN_CMR_ARB
                                 | CAN_CMR_CONTROL
                                 | CAN_CMR_DATAA
                                 | CAN_CMR_DATAB;

        st91x->MsgObj[msg_if].M1R = 0;                /* mask to be set via IoCtls                */
        st91x->MsgObj[msg_if].M2R = CAN_M2R_MXTD;

        st91x->MsgObj[msg_if].A1R = 0;                /* accept all                               */
        st91x->MsgObj[msg_if].A2R = CAN_A2R_MSGVAL | 0x1FFC;
        if (eob == CAN_TRUE) {
            st91x->MsgObj[msg_if].MCR = CAN_MCR_RXIE | /* enable rx interrupt                     */
                                        CAN_MCR_EOB  | /* single msg object                       */
                                        CAN_MCR_UMASK; /* use mask - accept all                   */
        } else {
            st91x->MsgObj[msg_if].MCR = CAN_MCR_RXIE | /* enable rx interrupt                     */
                                        CAN_MCR_UMASK; /* use mask - accept all                   */
        }
        st91x->MsgObj[msg_if].DA1R = 0;
        st91x->MsgObj[msg_if].DA2R = 0;
        st91x->MsgObj[msg_if].DB1R = 0;
        st91x->MsgObj[msg_if].DB2R = 0;

        st91x->MsgObj[msg_if].CRR = 1 + msg_obj;
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
* \ingroup  ST91X_CAN
*
*           Initializes the CAN module.
*
* \param    arg               identifies can device
*
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S ST91XCANInit (CPU_INT32U arg)
{
    CPU_INT16S result  =  ST91X_CAN_NO_ERR;           /* Local: Function result                   */
    CPU_INT32U i;                                     /* Local: loop variable                     */
    CAN_ST91X  *st91x;                                /* Local: pointer to CAN device             */
                                                      /*------------------------------------------*/
#if ST91X_CAN_ARG_CHK_CFG > 0
    if ((CPU_INT08U)arg >= ST91X_CAN_N_DEV) {         /* check that device name is in range       */
        DrvError= ST91X_CAN_INIT_ERR;
        return(-1);                                   /* return function result                   */
    }
#endif

    DrvError= ST91X_CAN_NO_ERR;                       /* set to defined value                     */
    st91x = (CAN_ST91X *)ST91X_CAN_BASE_ADDR;
    CanTbl[arg].Use = 0;
                                                      /*------------------------------------------*/
    ST91XCANPinSettings ();
                                                      /*------------------------------------------*/
    st91x->CR = CAN_CR_CCE | CAN_CR_INIT;             /* Init mode and Configuration Change Enable */
    st91x->SR = 0;                                    /* reset the status                         */
    st91x->BTR = ST91X_CAN_DEFAULT_BAUD_RATE;         /* set to default baud rate                 */
    st91x->BRPR = 0;                                  /* clear the Extended Baud Rate Prescaler   */

    ST91XCAN_SetUnusedMsgObj();                       /* sets all msg object to unused status     */
    ST91XCAN_SetTxMsgObj ();
    for (i=ST91X_CAN_RX_BUFFER; i<ST91X_CAN_SIZE_RX_FIFO-1; i++) {
        ST91XCAN_SetRxMsgObj (i, CAN_FALSE);
    }
    ST91XCAN_SetRxMsgObj (i, CAN_TRUE);               /* last fifo object                         */

                                                      /*------------------------------------------*/
#if ST91X_CAN_INTERRUPT_EN > 0                        /* INTERRUPT Settings                       */
    st91x->CR |= CAN_CR_IE | CAN_CR_EIE;              /* enable interrupts in CAN module          */
    ST91XCANIrqSettings ();
#endif

    return(result);                                   /* return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      OPEN THE CAN BUS
* \ingroup  ST91X_CAN
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
CPU_INT16S ST91XCANOpen(CPU_INT16S devId, CPU_INT32U devName, CPU_INT16U mode)
{
    CPU_INT16S result = -1;                           /* Local: Function result                   */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/

    mode  = mode;
#if ST91X_CAN_ARG_CHK_CFG > 0
    if ((CPU_INT08U)devName >= ST91X_CAN_N_DEV) {     /* check that device name is in range       */
        DrvError= ST91X_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
    if (mode != DEV_RW) {                             /* mode not supported?                      */
        DrvError = ST91X_CAN_MODE_ERR;
        return(result);
    }
#endif

#if (ST91X_CAN_INTERRUPT_EN > 0)
                                                      /* store the received Node Id for the irqs  */
    ST91X_BSP_SetDevIds ((CPU_INT08U) devId,
                         (CPU_INT08U) devName);
#else
    devId = devId;                                    /* prevent compiler warning                 */
#endif

    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    if (CanTbl[devName].Use == CAN_FALSE) {           /* Check, that device is not in use         */
        CanTbl[devName].Use = CAN_TRUE;               /* mark can device as used                  */
        result = devName;                             /* Okay, device is opened                   */
    }                                                 /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return(result);                                   /* return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CLOSE THE CAN BUS
* \ingroup  ST91X_CAN
*
*           Close the CAN bus
*
* \param    paraId            unused
*
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S ST91XCANClose(CPU_INT16S paraId)
{
    CPU_INT16S result = -1;                           /* Local: Function result                   */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */

#if ST91X_CAN_ARG_CHK_CFG > 0
                                                      /* check that paraId is in range            */
    if ((paraId >= ST91X_CAN_N_DEV) || (paraId < 0)) {
        DrvError= ST91X_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
#endif
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */                                                      /*------------------------------------------*/
    if (CanTbl[paraId].Use == CAN_TRUE) {             /* Check, that device is in use             */
        CanTbl[paraId].Use = CAN_FALSE;               /* mark can device as unused                */
        result = 0;                                   /* Okay, device is closed                   */
    } else {
        DrvError= ST91X_CAN_CLOSE_ERR;                /* not opened - set close error             */
    }
                                                      /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return(result);                                   /* return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN I/O CONTROL
* \ingroup  ST91X_CAN
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
CPU_INT16S ST91XCANIoCtl (CPU_INT16S paraId, CPU_INT16U func, void* argp)
{
    const CAN_BAUD *btbl;                             /* Local: Pointer to can bus baudrate table */
    CPU_INT16S      result  = -1;                     /* Local: Function result                   */
    CPU_INT32U      i;                                /* Local: counter variable                  */
    CPU_INT32U      msg_obj;                          /* Local: message object                    */
    CPU_INT32U      msg_if = 0;                       /* Local: message interface                 */
    CPU_INT32U      canId;                            /* Local: CAN Identifier                    */
    CPU_INT32U      mask;                             /* Local: Mask for identifier caluclations  */
    CPU_INT32U      status_reg;                       /* Local: status register value             */
    CAN_ST91X      *st91x =                           /* Local: pointer to CAN device             */
                (CAN_ST91X *)ST91X_CAN_BASE_ADDR;
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/

#if ST91X_CAN_ARG_CHK_CFG > 0
    if ((paraId >= ST91X_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range         */
        DrvError= ST91X_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
    if (CanTbl[paraId].Use != 1) {                    /* check, that can device is opened         */
        DrvError= ST91X_CAN_OPEN_ERR;
        return(result);                               /* return function result                   */
    }
#endif

    result = ST91X_CAN_NO_ERR;
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    switch (func) {                                   /* select: function code                    */
        case IO_ST91X_CAN_GET_IDENT:                  /* GET IDENT                                */
            (*(CPU_INT32U*)argp) = DrvIdent;          /* return driver ident code                 */
            break;
                                                      /*------------------------------------------*/
        case IO_ST91X_CAN_GET_ERRNO:                  /* GET ERRORCODE                            */
            (*(CPU_INT16U*)argp) = DrvError;          /* return last detected errorcode           */
            break;
                                                      /*------------------------------------------*/
        case IO_ST91X_CAN_GET_DRVNAME:                /* GET DRIVER NAME                          */
            (*(CPU_INT08U**)argp) = (CPU_INT08U*)     /* return human readable driver name        */
                                   ST91X_CAN_NAME;
            break;
                                                      /*------------------------------------------*/
        case IO_ST91X_CAN_SET_BAUDRATE:               /* Function: Set can bus baudrate           */
                                                      /*------------------------------------------*/
            btbl = &CanBaud[0];                       /* Set pointer to can bus baudrate table    */
            while (btbl->Baudrate != 0L) {            /* Loop through whole baudrate table        */
                                                      /*------------------------------------------*/
                                                      /* if baudrate matches given argument       */
                if (btbl->Baudrate == (*((CPU_INT32U*)argp))) {

                    st91x->CR = CAN_CR_CCE |          /* Init mode and Configuration Change Enable */
                                CAN_CR_INIT;
                    st91x->SR = 0;                    /* reset the status                         */
                    st91x->BTR = btbl->CANBTR;        /* set to default baud rate                 */
                    st91x->BRPR = 0;                  /* clear the Extended Baud Rate Prescaler   */
                    break;                            /* break loop                               */
                }                                     /*------------------------------------------*/
                btbl++;                               /* increment baudrate table pointer         */
            }                                         /*------------------------------------------*/
            break;                                    /*------------------------------------------*/
        case IO_ST91X_CAN_START:                      /* Function: Start can bus communication    */
                                                      /*------------------------------------------*/
            st91x->CR &= ~(CAN_CR_INIT |              /* leave init mode and clear CCE            */
                           CAN_CR_CCE);
#if ST91X_CAN_INTERRUPT_EN > 0
            st91x->CR |= CAN_CR_IE | CAN_CR_EIE;      /* enable interrupts in CAN module          */
#endif
          break;                                      /*------------------------------------------*/
        case IO_ST91X_CAN_STOP:                       /* Function: Stop can bus communication     */
                                                      /*------------------------------------------*/
            st91x->CR = CAN_CR_INIT;                  /* Set Init mode                            */
            break;                                    /*------------------------------------------*/

        case IO_ST91X_CAN_RX_STANDARD:                /* Function: receive standard format ids    */
                                                      /*------------------------------------------*/

            for (msg_obj=ST91X_CAN_RX_BUFFER; msg_obj<ST91X_CAN_SIZE_RX_FIFO; msg_obj++) {
                msg_if = ST91XCAN_GetFreeIF();
                if (msg_if != 2) {
                    st91x->MsgObj[msg_if].CMR = CAN_CMR_ARB;
                    st91x->MsgObj[msg_if].CRR = 1 + msg_obj; /* read arb bit from msg object */
                    ST91XCAN_WaitUntilReady(msg_if);
                    st91x->MsgObj[msg_if].CMR = CAN_CMR_WRRD |
                                                CAN_CMR_ARB;
                    st91x->MsgObj[msg_if].A2R &= ~CAN_A2R_XTD; /* clear extened bit               */
                    st91x->MsgObj[msg_if].CRR = 1 + msg_obj; /* write back to msg object          */
                    ST91XCAN_WaitUntilReady(msg_if);
                } else {
                    DrvError = ST91X_CAN_BUSY_ERR;
                }
            }
            break;                                    /*------------------------------------------*/
        case IO_ST91X_CAN_RX_EXTENDED:                /* Function: receive extended format ids    */
                                                      /*------------------------------------------*/
            for (msg_obj=ST91X_CAN_RX_BUFFER; msg_obj<ST91X_CAN_SIZE_RX_FIFO; msg_obj++) {
                msg_if = ST91XCAN_GetFreeIF();
                if (msg_if != 2) {
                    st91x->MsgObj[msg_if].CMR = CAN_CMR_ARB;
                    st91x->MsgObj[msg_if].CRR = 1 + msg_obj; /* read arb and mask from msg object */
                    ST91XCAN_WaitUntilReady(msg_if);
                    st91x->MsgObj[msg_if].CMR = CAN_CMR_WRRD |
                                                CAN_CMR_ARB;
                    st91x->MsgObj[msg_if].A2R |= CAN_A2R_XTD; /* set extended bit                 */
                    st91x->MsgObj[msg_if].CRR = 1 + msg_obj; /* write back to msg object          */
                    ST91XCAN_WaitUntilReady(msg_if);
                } else {
                    DrvError = ST91X_CAN_BUSY_ERR;
                }
            }
            break;                                    /*------------------------------------------*/
        case IO_ST91X_CAN_TX_READY:
            if (ST91X_CAN_TX_BUFFER <16) {
                                                      /* if ready to transmit                     */
                if ((st91x->TXR1R & (1 << ST91X_CAN_TX_BUFFER)) != 0) {
                    *((CPU_INT08U *)argp) = 0;
                } else {
                    *((CPU_INT08U *)argp) = 1;
                }
            } else {
                                                      /* if ready to transmit                     */
                if ((st91x->TXR2R & (1 << (ST91X_CAN_TX_BUFFER - 16))) != 0) {
                    *((CPU_INT08U *)argp) = 0;
                } else {
                    *((CPU_INT08U *)argp) = 1;
                }
            }

            break;                                    /*------------------------------------------*/
        case IO_ST91X_CAN_GET_NODE_STATUS:            /* Function: Get Node Status                */
                                                      /*------------------------------------------*/
            status_reg = st91x->SR;

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
         case IO_ST91X_CAN_SET_RX_FILTER:             /* SET RX FILTER                            */
                                                      /*------------------------------------------*/
            mask  = ((CPU_INT32U*)argp)[0];
            canId = ((CPU_INT32U*)argp)[1];

            for (msg_obj=ST91X_CAN_RX_BUFFER; msg_obj<ST91X_CAN_SIZE_RX_FIFO; msg_obj++) {
                msg_if = ST91XCAN_GetFreeIF();
                if (msg_if != 2) {

                    st91x->MsgObj[msg_if].CMR = CAN_CMR_CONTROL; /* read control reg              */
                    st91x->MsgObj[msg_if].CRR = 1 + msg_obj;

                    while (((st91x->MsgObj[msg_if].CRR & CAN_CRR_BUSY) != 0) &&
                            (i < 10)) {
                        i++;
                    }
                    if (i >= 10) {
                        DrvError = ST91X_CAN_BUSY_ERR;
                        return ST91X_CAN_BUSY_ERR;
                    }

                    st91x->MsgObj[msg_if].CMR = CAN_CMR_WRRD
                                             | CAN_CMR_MASK
                                             | CAN_CMR_ARB
                                             | CAN_CMR_CONTROL;

                    if ((canId & ST91X_CAN_FF_FRAME_BIT) == 0) {
                        st91x->MsgObj[msg_if].M1R = 0;
                        st91x->MsgObj[msg_if].M2R = (mask & 0x7FF) << 2;;

                        st91x->MsgObj[msg_if].A1R = 0;
                        st91x->MsgObj[msg_if].A2R = CAN_A2R_MSGVAL | ((canId & 0x7FF) << 2);
                    } else {
                        st91x->MsgObj[msg_if].M1R = (mask & 0xFFFF);
                        st91x->MsgObj[msg_if].M2R = CAN_M2R_MXTD | (mask >> 16);

                        st91x->MsgObj[msg_if].A1R = (canId & 0xFFFF);
                        st91x->MsgObj[msg_if].A2R = CAN_A2R_MSGVAL | CAN_A2R_XTD | ((canId & 0x7FF) >> 16);
                    }

                    st91x->MsgObj[msg_if].MCR = (st91x->MsgObj[msg_if].MCR & CAN_MCR_EOB) |
                                                CAN_MCR_RXIE | /* enable rx interrupt             */
                                                CAN_MCR_UMASK;

                    st91x->MsgObj[msg_if].CRR = 1 + msg_obj;
                }
            }
            break;                                    /*------------------------------------------*/
        default:
            DrvError = ST91X_CAN_FUNC_ERR;
            result = -1;
            break;

    }                                                 /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return result;                                    /* return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN READ DATA
* \ingroup  ST91X_CAN
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
CPU_INT16S ST91XCANRead (CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size)
{
    ST91X_CANFRM *frm;                                /* Local: Pointer to can frame              */
    CPU_INT16S    result = -1;                        /* Local: Result of function                */
    CPU_INT32U    i;                                  /* Local: loop variable                     */
    CPU_INT32U    msg_obj;                            /* Local: message object                    */
    CPU_INT32U    msg_if = 0;                         /* Local: message interface                 */
    CAN_ST91X    *st91x  =                            /* Local: pointer to CAN device             */
                (CAN_ST91X *)ST91X_CAN_BASE_ADDR;
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/

#if ST91X_CAN_ARG_CHK_CFG > 0

    if ((paraId >= ST91X_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range           */
        DrvError= ST91X_CAN_BUS_ERR;
        return(result);
    }
    if (size != sizeof(ST91X_CANFRM)) {               /* check that size is plausible             */
        DrvError = ST91X_CAN_NO_DATA_ERR;
        return(result);
    }
    if (buffer == 0) {                                /* invalid buffer pointer                   */
        DrvError= ST91X_CAN_ARG_ERR;
        return(result);
    }
    if (CanTbl[paraId].Use != 1) {                    /* check, that can device is opened         */
        DrvError= ST91X_CAN_OPEN_ERR;
        return(result);
    }
#endif
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    msg_obj = 0xFF;
    for (i=ST91X_CAN_RX_BUFFER; i<ST91X_CAN_SIZE_RX_FIFO; i++) {
        if ((st91x->ND1R & (1 << i)) != 0) {          /* search fifo for new data                 */
            msg_obj = i;
            break;
        }
    }

    if (msg_obj < ST91X_CAN_SIZE_RX_FIFO) {
        frm = (ST91X_CANFRM *)buffer;                 /* Set pointer to can frame                 */

        msg_if = ST91XCAN_GetFreeIF();                    /* get free interface                       */
        while (msg_if == 2) {
            msg_if = ST91XCAN_GetFreeIF();
        }

        st91x->SR &= ~CAN_SR_RXOK;

                                                      /* read the message contents                */
        st91x->MsgObj[msg_if].CMR = CAN_CMR_MASK
                            | CAN_CMR_ARB
                            | CAN_CMR_CONTROL
                            | CAN_CMR_CLRINTPND       /* clear int pending bit                    */
                            | CAN_CMR_TXRQSTNEWDAT    /* clear new data bit                       */
                            | CAN_CMR_DATAA
                            | CAN_CMR_DATAB;

        st91x->MsgObj[msg_if].CRR = 1 + msg_obj;
        i = 0;
        while (((st91x->MsgObj[msg_if].CRR & CAN_CRR_BUSY) != 0) &&
                (i < 10)) {
            i++;
        }
        if (i >= 10) {
            DrvError = ST91X_CAN_BUSY_ERR;
            return ST91X_CAN_BUSY_ERR;
        }

        if ((st91x->MsgObj[msg_if].A2R & CAN_A2R_XTD) == 0) { /* standard ID                           */
            frm->Identifier = (st91x->MsgObj[msg_if].A2R >> 2) & 0x07FF;
        } else {                                              /* extended ID                           */
            frm->Identifier = ((CPU_INT32U)st91x->MsgObj[msg_if].A1R);
            frm->Identifier |= (((CPU_INT32U)st91x->MsgObj[msg_if].A2R) << 16);
                                                              /* mark std/extended frame               */
            frm->Identifier |= ST91X_CAN_FF_FRAME_BIT;
        }

        frm->DLC = st91x->MsgObj[msg_if].MCR & 0x0F;

        frm->Data[0] = (CPU_INT08U) st91x->MsgObj[msg_if].DA1R;
        frm->Data[1] = (CPU_INT08U)(st91x->MsgObj[msg_if].DA1R >> 8);
        frm->Data[2] = (CPU_INT08U) st91x->MsgObj[msg_if].DA2R;
        frm->Data[3] = (CPU_INT08U)(st91x->MsgObj[msg_if].DA2R >> 8);
        frm->Data[4] = (CPU_INT08U) st91x->MsgObj[msg_if].DB1R;
        frm->Data[5] = (CPU_INT08U)(st91x->MsgObj[msg_if].DB1R >> 8);
        frm->Data[6] = (CPU_INT08U) st91x->MsgObj[msg_if].DB2R;
        frm->Data[7] = (CPU_INT08U)(st91x->MsgObj[msg_if].DB2R >> 8);

        result = size;                                /* set successfull result                   */

    } else {
        result = ST91X_CAN_NO_DATA_ERR;
        DrvError = result;
    }

                                                      /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return(result);                                   /* Return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN WRITE DATA
* \ingroup  ST91X_CAN
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
CPU_INT16S ST91XCANWrite (CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size)
{
    ST91X_CANFRM *frm;                                /* Local: Pointer to can frame              */
    CPU_INT16S    result = -1;                        /* Local: Result of function                */
    CPU_INT32U    cnt    = 0;                         /* Local: Result of function                */
    CPU_INT32U    msg_if = 0;                         /* Local: message interface                 */
    CAN_ST91X    *st91x  =                            /* Local: pointer to CAN device             */
                (CAN_ST91X *)ST91X_CAN_BASE_ADDR;
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/

#if ST91X_CAN_ARG_CHK_CFG > 0

    if ((paraId >= ST91X_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range           */
        DrvError= ST91X_CAN_BUS_ERR;
        return(result);
    }
    if (size != sizeof(ST91X_CANFRM)) {               /* check that size is plausible             */
        DrvError = ST91X_CAN_NO_DATA_ERR;
        return(result);
    }
    if (buffer == 0) {                                /* invalid buffer pointer                   */
        DrvError= ST91X_CAN_ARG_ERR;
        return(result);
    }
    if (CanTbl[paraId].Use != 1) {                    /* check, that can device is opened         */
        DrvError= ST91X_CAN_OPEN_ERR;
        return(result);
    }
#endif

    CPU_CRITICAL_ENTER();                             /* enter critical section                   */

    if (ST91X_CAN_TX_BUFFER <16) {                    /* check availability of tx buffer          */
        if ((st91x->TXR1R & (1 << ST91X_CAN_TX_BUFFER)) != 0) {
            DrvError = ST91X_CAN_BUSY_ERR;
            return result;
        }
    } else {
        if ((st91x->TXR2R & (1 << (ST91X_CAN_TX_BUFFER - 16))) != 0) {
            DrvError = ST91X_CAN_BUSY_ERR;
            return result;
        }
    }

    frm = (ST91X_CANFRM *)buffer;                     /* Set pointer to can frame                 */

    msg_if = ST91XCAN_GetFreeIF();                    /* get free interface to msg object         */
    while (msg_if == 2) {
        msg_if = ST91XCAN_GetFreeIF();
    }
                                                      /* read the Arbitration and Message Control */
    st91x->MsgObj[msg_if].CMR = CAN_CMR_ARB | CAN_CMR_CONTROL;
    st91x->MsgObj[msg_if].CRR = 1 + ST91X_CAN_TX_BUFFER;
                                                      /* wait until read action is completed      */
    while (((st91x->MsgObj[msg_if].CRR & CAN_CRR_BUSY) != 0) &&
            (cnt < 10)) {
        cnt++;
    }
    if (cnt >= 10) {
        DrvError = ST91X_CAN_BUSY_ERR;
        return result;
    }
                                                      /* update the contents needed for transmission */
    st91x->MsgObj[msg_if].CMR = CAN_CMR_WRRD    |
                           CAN_CMR_ARB     |
                           CAN_CMR_CONTROL |
                           CAN_CMR_DATAA   |
                           CAN_CMR_DATAB;

                                                      /* settings for standard or extended id     */
    if (frm->Identifier <= 0x7FF) {                   /* if Id is standard Id                     */
      st91x->MsgObj[msg_if].A1R = 0;
      st91x->MsgObj[msg_if].A2R = (st91x->MsgObj[msg_if].A2R & 0xA000) |
                              STD_FIXED_ID_ARB(frm->Identifier);
    } else {                                          /* else if Id is extended Id               */
      st91x->MsgObj[msg_if].A1R = EXT_FIXED_ID_ARB_L(frm->Identifier);
      st91x->MsgObj[msg_if].A2R = (st91x->MsgObj[msg_if].A2R & 0xE000) |
                              CAN_A2R_XTD                    |
                              EXT_FIXED_ID_ARB_H(frm->Identifier);
    }

    st91x->MsgObj[msg_if].MCR = (st91x->MsgObj[msg_if].MCR & 0xFEF0) |
                            CAN_MCR_NEWDAT |
                            CAN_MCR_TXRQST |
                            frm->DLC;
                                                      /* copy data to msg object via interface reg */
    st91x->MsgObj[msg_if].DA1R = ((CPU_INT16U)frm->Data[1]<<8) | frm->Data[0];
    st91x->MsgObj[msg_if].DA2R = ((CPU_INT16U)frm->Data[3]<<8) | frm->Data[2];
    st91x->MsgObj[msg_if].DB1R = ((CPU_INT16U)frm->Data[5]<<8) | frm->Data[4];
    st91x->MsgObj[msg_if].DB2R = ((CPU_INT16U)frm->Data[7]<<8) | frm->Data[6];

                                                      /* execute request to update msg object     */
    st91x->MsgObj[msg_if].CRR = 1 + ST91X_CAN_TX_BUFFER;

    result = size;                                    /* set successfull result                   */
                                                      /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return(result);                                   /* Return function result                   */
}

/*! } */

