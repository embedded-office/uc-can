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
*********************************************************************************************************
* Filename : can_os.c
* Version  : V2.42.01
* Note(s)  : This include file declares all operating system specific symbolic constants.
*            Per default, this file implements the operating specific functions for uC/OS-II.
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                              INCLUDES
*********************************************************************************************************
*/

#include "can_os.h"                                   /* CAN OS abstraction layer                      */
#include "can_err.h"                                  /* CAN error codes                               */


/*
*********************************************************************************************************
*                                         GLOBAL OS VARIABLES
*********************************************************************************************************
*/

/*-----------------------------------------------------------------------------------------------------*/
/*!
* \brief                     CAN BUS SEMAPHORES
* \ingroup  UCCAN
*
*           Allocation of semaphores for the buffering ressources. The following
*           implementation is an implementation for uC/OS-II.
*/
/*-----------------------------------------------------------------------------------------------------*/

OS_EVENT    *CANOS_TxSem[CANBUS_N];
OS_EVENT    *CANOS_RxSem[CANBUS_N];


/*
*********************************************************************************************************
*                                              FUNCTIONS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            CANOS_Init()
*
* Description : This function shall create and initialize all needed OS objects like semaphores, queues,
*               etc.
*
* Argument(s) : None.
*
* Return(s)   : Errorcode CANOS_INIT_ERR if any OS object creation/initialization is failed. If all
*               objects are sucessfully initialized, the return value shall be CANOS_NO_ERR.
*
* Note(s)     : This function is a wrapper around the wanted operating system. The following
*               implementation is an implementation for uC/OS-II.
*********************************************************************************************************
*/

CPU_INT16S CANOS_Init(void)
{
    CPU_INT08U  i;                                    /* Local: loop variable                          */


    for (i = 0u; i < CANBUS_N; i++) {                 /* loop through all busses                       */
        CANOS_TxSem[i] = OSSemCreate(CANBUS_TX_QSIZE - 1u); /* Initialize TX buffer counting semaphore */
        if (CANOS_TxSem[i] == NULL_PTR) {             /* Check result                                  */
            can_errnum = CAN_ERR_OSSEM;
            return CAN_ERR_OSSEM;                     /* and leave initialization with errorcode       */
        }
        CANOS_RxSem[i] = OSSemCreate(0u);             /* Initialize RX buffer counting semaphore       */
        if (CANOS_RxSem[i] == NULL_PTR) {             /* Check result                                  */
            can_errnum = CAN_ERR_OSSEM;
            return CAN_ERR_OSSEM;                     /* and leave initialization with errorcode       */
        }
    }                                                 /*-----------------------------------------------*/
    return CAN_ERR_NONE;                              /* return function result                        */
}


/*
*********************************************************************************************************
*                                           CANOS_ResetRx()
*
* Description : This function resets the receive semaphore.
*
* Argument(s) : busId    identifies CAN bus
*
* Return(s)   : None.
*
* Note(s)     : This function is a wrapper around the wanted operating system. The following
*               implementation is an implementation for uC/OS-II.
*********************************************************************************************************
*/

void  CANOS_ResetRx (CPU_INT16S  busId)
{
    CPU_INT08U  err;                                  /* Local: Errorcode of OS function               */


#if CANOS_ARG_CHK_EN > 0
    if ((busId < 0) || ((CPU_INT16U)busId >= CANBUS_N)) { /* is busId out of range?                    */
        can_errnum = CAN_ERR_BUSID;
        return;
    }
#endif

    OSSemSet(CANOS_RxSem[busId], 0u, &err);           /* reset semaphore counter value                 */
    if (err != CANOS_NO_ERR) {                        /* see, if no error is detected                  */
        can_errnum = CAN_ERR_OSSEM;                   /* set error indication                          */
    }
}


/*
*********************************************************************************************************
*                                           CANOS_ResetTx()
*
* Description : This function resets the transmit semaphore.
*
* Argument(s) : busId    identifies CAN bus
*
* Return(s)   : None.
*
* Note(s)     : This function is a wrapper around the wanted operating system. The following
*               implementation is an implementation for uC/OS-II.
*********************************************************************************************************
*/

void  CANOS_ResetTx (CPU_INT16S  busId)
{
    CPU_INT08U  err;                                  /* Local: Errorcode of OS function               */


#if CANOS_ARG_CHK_EN > 0
    if ((busId < 0) || ((CPU_INT16U)busId >= CANBUS_N)) { /* is busId out of range?                    */
        can_errnum = CAN_ERR_BUSID;
        return;
    }
#endif

    OSSemSet(CANOS_TxSem[(CPU_INT16U)busId],          /* reset semaphore counter value                 */
             CANBUS_TX_QSIZE - 1u,
             &err);
    if (err != CANOS_NO_ERR) {                        /* see, if no error is detected                  */
        can_errnum = CAN_ERR_OSSEM;                   /* set error indication                          */
    }
}


/*
*********************************************************************************************************
*                                         CANOS_PendRxFrame()
*
* Description : This function shall wait for a CAN frame within the CAN receive buffer. If a timeout of 0
*               ticks is given, this function shall wait forever, otherwise this function shall wait for
*               maximal timeout ticks.
*
* Argument(s) : timeout    Timeout in OS time ticks
*
*               busId      identifies CAN bus
*
* Return(s)   : Indication of a received frame:
*
*                   1 = at least one frame is received
*                   0 = no frame received until timeout
*
* Note(s)     : This function is a wrapper around the wanted operating system. The following
*               implementation is an implementation for uC/OS-II.
*********************************************************************************************************
*/

CPU_INT08U  CANOS_PendRxFrame (CPU_INT16U  timeout,
                               CPU_INT16S  busId)
{
    CPU_INT08U  result;                               /* Local: Function result                        */
    CPU_INT08U  err;                                  /* Local: Errorcode of OS function               */


#if CANOS_ARG_CHK_EN > 0
    if ((busId < 0) || ((CPU_INT16U)busId >= CANBUS_N)) { /* is busId out of range?                    */
        can_errnum = CAN_ERR_BUSID;
        return (0u);
    }
#endif

    OSSemPend(CANOS_RxSem[busId], timeout, &err);     /* Wait for a received frame with timeout        */
    if (err == CANOS_NO_ERR) {                        /* see, if no error is detected                  */
        result     = 1u;                              /* yes: there a frame in receive buffer          */
    } else {                                          /* otherwise: an error is detected               */
        can_errnum = CAN_ERR_OSSEMPEND;               /* set error indication                          */
        result     = 0u;                              /* no free space in transmit buffer              */
    }
    return (result);                                  /* return function result                        */
}


/*
*********************************************************************************************************
*                                         CANOS_PostRxFrame()
*
* Description : This function shall check for a CAN frame within the CAN receive buffer.
*
* Argument(s) : busId     identifies CAN bus
*
* Return(s)   : None.
*
* Note(s)     : This function is a wrapper around the wanted operating system. The following
*               implementation is an implementation for uC/OS-II.
*********************************************************************************************************
*/

void  CANOS_PostRxFrame (CPU_INT16S  busId)
{
    CPU_INT08U  err;                                  /* Local: OS error code                          */


#if CANOS_ARG_CHK_EN > 0
    if ((busId < 0) || ((CPU_INT16U)busId >= CANBUS_N)) {  /* is busId out of range?                   */
        can_errnum = CAN_ERR_BUSID;
        return;
    }
#endif

    err = OSSemPost(CANOS_RxSem[(CPU_INT16U)busId]);  /* post for a received frame                     */
    if (err != CAN_ERR_NONE) {                        /* see, if no error is detected                  */
        can_errnum = CAN_ERR_OSSEMPOST;               /* set error indication                          */
    }
}


/*
*********************************************************************************************************
*                                         CANOS_PendTxFrame()
*
* Description : This function shall wait for a free space within the transmit buffer. If a timeout of 0
*               ticks is given, this function shall wait forever, otherwise this function shall wait for
*               maximal timeout ticks.
*
* Argument(s) : timeout    Timeout in OS time ticks
*
*               busId      identifies CAN bus
*
* Return(s)   : Indication of transmit buffer status:
*
*                   1 = space for a frame is found
*                   0 = no space for a frame until timeout
*
* Note(s)     : This function is a wrapper around the wanted operating system. The following
*               implementation is an implementation for uC/OS-II.
*********************************************************************************************************
*/

CPU_INT08U  CANOS_PendTxFrame (CPU_INT16U  timeout,
                               CPU_INT16S  busId)
{
    CPU_INT08U  result;                               /* Local: Pointer to allocated CAN frame         */
    CPU_INT08U  err;                                  /* Local: OS errorcode                           */


#if CANOS_ARG_CHK_EN > 0
    if ((busId < 0) || ((CPU_INT16U)busId >= CANBUS_N)) { /* is busId out of range?                    */
        can_errnum = CAN_ERR_BUSID;
        return (0u);
    }
#endif

    OSSemPend(CANOS_TxSem[busId], timeout, &err);     /* Get semaphore for memory partition            */
    if (err == CANOS_NO_ERR) {                        /* see, if no error is detected                  */
        result     = 1u;                              /* yes: space for frame in transmit buffer       */
    } else {                                          /* otherwise: an error is detected               */
        can_errnum = CAN_ERR_OSSEMPEND;               /* set error indication                          */
        result     = 0u;                              /* no free space in transmit buffer              */
    }
    return (result);                                  /* return function result                        */
}


/*
*********************************************************************************************************
*                                         CANOS_PostTxFrame()
*
* Description : This function shall release a CAN frame reservation within the CAN transmit buffer.
*
* Argument(s) : busId     identifies CAN bus
*
* Return(s)   : None.
*
* Note(s)     : This function is a wrapper around the wanted operating system. The following
*               implementation is an implementation for uC/OS-II.
*********************************************************************************************************
*/

void  CANOS_PostTxFrame (CPU_INT16S  busId)
{
    CPU_INT08U  err;                                  /* Local: OS error code                          */


#if CANOS_ARG_CHK_EN > 0
    if ((busId < 0) || ((CPU_INT16U)busId >= CANBUS_N)) { /* is busId out of range?                    */
        can_errnum = CAN_ERR_BUSID;
        return;
    }
#endif

    err = OSSemPost(CANOS_TxSem[(CPU_INT16U)busId]);  /* release transmit buffer resource              */
    if (err != CAN_ERR_NONE) {                        /* see, if no error is detected                  */
        can_errnum = CAN_ERR_OSSEMPOST;               /* set error indication                          */
    }
}


/*
*********************************************************************************************************
*                                           CANOS_GetTime()
*
* Description : This function shall receive a time from the underliing OS. The time value will have the
*               resultion and format of the OS.
*
* Argument(s) : None.
*
* Return(s)   : 32 bit value that represent a time value.
*
* Note(s)     : This function is a wrapper around the wanted operating system. The following
*               implementation is an implementation for uC/OS-II.
*********************************************************************************************************
*/

CPU_INT32U  CANOS_GetTime (void)
{
    return OSTimeGet();
}


/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/
