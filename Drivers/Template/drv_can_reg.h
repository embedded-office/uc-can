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
*                                      CAN DRIVER REGISTER CODE
*
*                                              Template
*
* Filename : drv_can_reg.h
* Version  : V2.42.01
*********************************************************************************************************
*/

#ifndef  _DRV_CAN_REG_H
#define  _DRV_CAN_REG_H


/*
*********************************************************************************************************
*                                              INCLUDES
*********************************************************************************************************
*/

#include  "cpu.h"
#include  "can_cfg.h"


/*
*********************************************************************************************************
*                                        DEFAULT CONFIGURATION
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          ADDRESS REGISTERS
*********************************************************************************************************
*/


/*
****************************************************************************************************
*                                           BIT DEFINITIONS
****************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               MACROS
*
* Note(s) : (1) When Using Extended IDs for Rx'd Messages, its required to save both the Standard
*               Section and Extended Section of the ID and placed both in the Rx Frame in the
*               following Format:
*                           Bits:  31     30    29                   18                   0
*                   uC/CAN Frame: [ 0u | RTR | IDE |    Standard ID    |    Extended ID    ]
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             DATA ARRAY
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            CAN REGISTER
*
* Description : Structure defines the CAN Register Structure.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  volatile  struct  <drv_name>_can_reg {                 /* ---------- CAN CONTROLLER REGISTER SUMMARY --------- */
    /* $$$ - Insert CAN Register Map Structure - $$$ */
} <Drv_Name>_CAN_REG;


/*
*********************************************************************************************************
*                                        CAN BAUDRATE REGISTER
*
* Description : Structure defines the CAN BaudRate Register Structure.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  struct  <drv_name>_can_baud {    
    CPU_INT32U  BaudRate;                                       /* Holds the Baudrate.                                  */
    CPU_INT32U  SamplePoint;                                    /* Holds the Sample point in 0.1%                       */
    CPU_INT32U  ReSynchJumpWidth;                               /* Holds the Re-Synchronization Jump Width in 0.1%      */
    CPU_INT08U  PrescalerDiv;                                   /* Holds the Prescaler Divide Factor                    */
    CPU_INT08U  SJW;                                            /* Holds the Re-Synch Jump Width         (StdValue = 1) */
    CPU_INT08U  PropagationSeg;                                 /* Holds the Propagation Segment Time    (StdValue = 2) */
    CPU_INT08U  PhaseBufSeg1;                                   /* Holds the Phase Buffer Segment 1      (StdValue = 7) */
    CPU_INT08U  PhaseBufSeg2;                                   /* Holds the Phase Buffer Segment 2      (StdValue = 7) */
} <Drv_Name>_CAN_BAUD;


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            ERROR SECTION
*********************************************************************************************************
*/

#endif
