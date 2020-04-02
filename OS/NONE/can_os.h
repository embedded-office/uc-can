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
* Filename : can_os.h
* Version  : V2.42.01
*********************************************************************************************************
*/

#ifndef _CAN_OS_H_
#define _CAN_OS_H_


#ifdef __cplusplus
extern "C" {
#endif


/*
*********************************************************************************************************
*                                              INCLUDES
*********************************************************************************************************
*/

#include "can_frm.h"                                  /* CAN Frame handling                            */
#include "can_cfg.h"                                  /* CAN Configuration defines                     */


/*-----------------------------------------------------------------------------------------------------*/
/*! \brief                                OS: NO ERROR
*
*            This errorcode indicates 'no error detected'.
*/
/*-----------------------------------------------------------------------------------------------------*/

#define CANOS_NO_ERR      (CPU_INT08U)0


/*
*********************************************************************************************************
*                                       FUNCTION PROTOTYPES
*********************************************************************************************************
*/

CPU_INT16S  CANOS_Init       (void);

CPU_INT08U  CANOS_PendRxFrame(CPU_INT16U  timeout, 
                              CPU_INT16S  busId);

void        CANOS_PostRxFrame(CPU_INT16S  busId);

void        CANOS_ResetRx    (CPU_INT16S  busId);

CPU_INT08U  CANOS_PendTxFrame(CPU_INT16U  timeout,
                              CPU_INT16S  busId);

void        CANOS_PostTxFrame(CPU_INT16S  busId);

void        CANOS_ResetTx    (CPU_INT16S  busId);

CPU_INT32U  CANOS_GetTime    (void);

#ifdef __cplusplus
}
#endif


/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/

#endif                                                /* #ifndef _CAN_OS_H                             */
