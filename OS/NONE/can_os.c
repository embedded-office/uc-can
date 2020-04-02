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
*            Normally, this file implements the operating specific functions for an RTOS which
*            provides semaphore handling. This file implements a simple counting semaphore.
*            There is still needed the protection of the semaphore
*            (CPU_CRITICAL_ENTER/CPU_CRITICAL_EXIT) and a BSP function to get time information.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              INCLUDES
*********************************************************************************************************
*/

#include "can_os.h"                                   /* CAN OS abstraction layer                      */
#include "can_err.h"                                  /* CAN error codes                               */
#include "bsp.h"                                      /* timer function                                */
#include "cpu.h"


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/

typedef struct semaphore {
    CPU_INT32U count;
}CAN_SEM;


/*
*********************************************************************************************************
*                                           LOCAL VARIABLES
*********************************************************************************************************
*/

/*-----------------------------------------------------------------------------------------------------*/
/*!
* \brief                     CAN BUS SEMAPHORES
* \ingroup  UCCAN
*
*           Allocation of simple counting semaphores for the buffering ressources. The following
*           implementation is an implementation for usage without an RTOS.
*/
/*-----------------------------------------------------------------------------------------------------*/

CAN_SEM   CANOS_TxSem[CANBUS_N];
CAN_SEM   CANOS_RxSem[CANBUS_N];


/*
*********************************************************************************************************
*                                    EXTNERNAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

extern  CPU_INT32U  BSPTimeGet (void);


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
* Note(s)     : None.
*********************************************************************************************************
*/

CPU_INT16S  CANOS_Init (void)
{
    CPU_INT08U  i;                                    /* Local: loop variable                          */


    for (i = 0u; i < CANBUS_N; i++) {                 /* loop through all busses                       */
        CANOS_TxSem[i].count = CANBUS_TX_QSIZE - 1u;  /* Initialize TX buffer counting semaphore       */
        CANOS_RxSem[i].count = 0u;                    /* Initialize RX buffer counting semaphore       */
    }

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
* Note(s)     : None.
*********************************************************************************************************
*/

void  CANOS_ResetRx (CPU_INT16S  busId)
{
    CPU_SR_ALLOC();


#if CANOS_ARG_CHK_EN > 0
    if ((busId < 0) || ((CPU_INT16U)busId >= CANBUS_N)) {  /* is busId out of range?                   */
        can_errnum = CAN_ERR_BUSID;
    }
#endif

    CPU_CRITICAL_ENTER();
    CANOS_RxSem[(CPU_INT16U)busId].count = 0u;
    CPU_CRITICAL_EXIT();
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
* Note(s)     : None.
*********************************************************************************************************
*/

void  CANOS_ResetTx (CPU_INT16S  busId)
{
    CPU_SR_ALLOC();


#if CANOS_ARG_CHK_EN > 0
    if ((busId < 0) || ((CPU_INT16U)busId >= CANBUS_N)) { /* is busId out of range?                    */
        can_errnum = CAN_ERR_BUSID;
    }
#endif

    CPU_CRITICAL_ENTER();
    CANOS_TxSem[(CPU_INT16U)busId].count = CANBUS_TX_QSIZE - 1u;
    CPU_CRITICAL_EXIT();
}


/*
*********************************************************************************************************
*                                         CANOS_PendRxFrame()
*
* Description : This function shall wait for a CAN frame within the CAN receive buffer. If a timeout of 0
*               ticks is given, this function shall wait forever, otherwise this function shall wait for
*               maximal timeout ticks.
*
* Argument(s) : timeout    Timeout in time ticks as provided by the board support package
*
*               busId      identifies CAN bus
*
* Return(s)   : Indication of a received frame:
*
*                   1 = at least one frame is received
*                   0 = no frame received until timeout
*
* Note(s)     : None.
*********************************************************************************************************
*/

CPU_INT08U  CANOS_PendRxFrame (CPU_INT16U  timeout,
                               CPU_INT16S  busId)
{
    CPU_INT32U  cnt;                                  /* Local: semaphore count                        */
    CPU_INT32U  time;                                 /* Local: actual time                            */
    CPU_INT32U  start;                                /* Local: timeout start time                     */
    CPU_INT32U  end;                                  /* Local: timeout end time                       */
    CPU_INT08U  result = 0u;                          /* Local: Function result                        */
    CPU_SR_ALLOC();


#if CANOS_ARG_CHK_EN > 0
    if ((busId < 0) || ((CPU_INT16U)busId >= CANBUS_N)) { /* is busId out of range?                    */
        can_errnum = CAN_ERR_BUSID;
        return (0u);
    }
#endif

    CPU_CRITICAL_ENTER();
    cnt = CANOS_RxSem[(CPU_INT16U)busId].count;
    CPU_CRITICAL_EXIT();

    if (cnt == 0u) {
        if (timeout == 0u) {                          /* blocking wait                                 */
            do {
                CPU_CRITICAL_ENTER();
                cnt = CANOS_RxSem[(CPU_INT16U)busId].count;
                CPU_CRITICAL_EXIT();
            } while (cnt == 0u);

        } else {                                      /* wait with timeout                             */

            start = BSPTimeGet();
            end   = start + timeout;
            do {
                CPU_CRITICAL_ENTER();
                cnt = CANOS_RxSem[(CPU_INT16U)busId].count;
                CPU_CRITICAL_EXIT();

                time = BSPTimeGet();
            } while ((cnt == 0u) && (time < end));

            if (cnt == 0u) {
                can_errnum = CAN_ERR_OSSEMPEND;       /* set error indication                          */
            }
        }
    }
    if (cnt > 0u) {
        CPU_CRITICAL_ENTER();
        CANOS_RxSem[(CPU_INT16U)busId].count--;       /* decrement  semaphore count                    */
        CPU_CRITICAL_EXIT();
        result     = 1u;                              /* yes: there a frame in receive buffer          */
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
* Note(s)     : None.
*********************************************************************************************************
*/

void  CANOS_PostRxFrame (CPU_INT16S  busId)
{
    CPU_SR_ALLOC();


#if CANOS_ARG_CHK_EN > 0
    if ((busId < 0) || ((CPU_INT16U)busId >= CANBUS_N)) {  /* is busId out of range?                   */
        can_errnum = CAN_ERR_BUSID;
    }
#endif

    CPU_CRITICAL_ENTER();
    CANOS_RxSem[(CPU_INT16U)busId].count++;
    CPU_CRITICAL_EXIT();
}


/*
*********************************************************************************************************
*                                         CANOS_PendTxFrame()
*
* Description : This function shall wait for a free space within the transmit buffer. If a timeout of 0
*               ticks is given, this function shall wait forever, otherwise this function shall wait for
*               maximal timeout ticks.
*
* Argument(s) : timeout    Timeout in time ticks as provided by the board support package
*
*               busId      identifies CAN bus
*
* Return(s)   : Indication of transmit buffer status:
*
*                   1 = space for a frame is found
*                   0 = no space for a frame until timeout
*
* Note(s)     : None.
*********************************************************************************************************
*/

CPU_INT08U  CANOS_PendTxFrame (CPU_INT16U  timeout,
                               CPU_INT16S  busId)
{
    CPU_INT32U  cnt;                                  /* Local: semaphore count                        */
    CPU_INT32U  time;                                 /* Local: actual time                            */
    CPU_INT32U  start;                                /* Local: timeout start time                     */
    CPU_INT32U  end;                                  /* Local: timeout end time                       */
    CPU_INT08U  result = 0u;                          /* Local: Pointer to allocated CAN frame         */
    CPU_SR_ALLOC();


#if CANOS_ARG_CHK_EN > 0
    if ((busId < 0) || ((CPU_INT16U)busId >= CANBUS_N)) { /* is busId out of range?                    */
        can_errnum = CAN_ERR_BUSID;
        return (0u);
    }
#endif

    CPU_CRITICAL_ENTER();
    cnt = CANOS_TxSem[(CPU_INT16U)busId].count;
    CPU_CRITICAL_EXIT();

    if (cnt == 0u) {
        if (timeout == 0u) {                          /* blocking wait                                 */
            do {
                CPU_CRITICAL_ENTER();
                cnt = CANOS_TxSem[(CPU_INT16U)busId].count;
                CPU_CRITICAL_EXIT();
            } while (cnt == 0u);

        } else {                                      /* wait with timeout                             */

            start = BSPTimeGet();                     /* get start time                                */
            end   = start + timeout;                  /* calc end time                                 */
            do {
                CPU_CRITICAL_ENTER();
                cnt = CANOS_TxSem[(CPU_INT16U)busId].count;
                CPU_CRITICAL_EXIT();

                time = BSPTimeGet();
            } while ((cnt == 0u) && (time < end));

            if (cnt == 0u) {
                can_errnum = CAN_ERR_OSSEMPEND;       /* set error indication                          */
            }
        }
    }
    if (cnt > 0u) {
        CPU_CRITICAL_ENTER();
        CANOS_TxSem[(CPU_INT16U)busId].count--;       /* decrement  semaphore count                    */
        CPU_CRITICAL_EXIT();
        result     = 1u;                              /* yes: there a frame in transmit buffer         */
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
* Note(s)     : None.
*********************************************************************************************************
*/

void  CANOS_PostTxFrame (CPU_INT16S  busId)
{
    CPU_SR_ALLOC();


#if CANOS_ARG_CHK_EN > 0
    if ((busId < 0) || ((CPU_INT16U)busId >= CANBUS_N)) {  /* is busId out of range?                   */
        can_errnum = CAN_ERR_BUSID;
    }
#endif

    CPU_CRITICAL_ENTER();
    CANOS_TxSem[(CPU_INT16U)busId].count++;
    CPU_CRITICAL_EXIT();
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
* Note(s)     : None.
*********************************************************************************************************
*/

CPU_INT32U  CANOS_GetTime (void)
{
    return BSPTimeGet();
}


/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/
