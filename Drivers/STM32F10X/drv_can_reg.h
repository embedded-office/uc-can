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
* Filename : drv_can_reg.h
* Version  : V2.42.01
****************************************************************************************************
*/

#ifndef _DRVSTM32F10X_CANREG_H
#define _DRVSTM32F10X_CANREG_H

#include "cpu.h"

/*
****************************************************************************************************
*                                              DEFINES
****************************************************************************************************
*/

#define STM32F10X_CAN1_BASE        0x40006400               /* CAN 1 base address                     */
#define STM32F10X_CAN2_BASE        0x40006800               /* CAN 2 base address                     */

/* STM32F10X_CAN Master Control Register bits */
#define STM32F10X_CAN_MCR_INRQ     ((CPU_INT32U)0x00000001) /* Initialization request                 */
#define STM32F10X_CAN_MCR_SLEEP    ((CPU_INT32U)0x00000002) /* Sleep mode request                     */
#define STM32F10X_CAN_MCR_TXFP     ((CPU_INT32U)0x00000004) /* Transmit FIFO priority                 */
#define STM32F10X_CAN_MCR_RFLM     ((CPU_INT32U)0x00000008) /* Receive FIFO locked mode               */
#define STM32F10X_CAN_MCR_NART     ((CPU_INT32U)0x00000010) /* No automatic retransmission            */
#define STM32F10X_CAN_MCR_AWUM     ((CPU_INT32U)0x00000020) /* Automatic wake up mode                 */
#define STM32F10X_CAN_MCR_ABOM     ((CPU_INT32U)0x00000040) /* Automatic bus-off management           */
#define STM32F10X_CAN_MCR_TTCM     ((CPU_INT32U)0x00000080) /* time triggered communication           */

/* STM32F10X_CAN Master Status Register bits */
#define STM32F10X_CAN_MSR_INAK     ((CPU_INT32U)0x00000001) /* Initialization acknowledge             */
#define STM32F10X_CAN_MSR_ERRI     ((CPU_INT32U)0x00000004) /* Error interrupt                        */
#define STM32F10X_CAN_MSR_WKUI     ((CPU_INT32U)0x00000008) /* Wake-up interrupt                      */
#define STM32F10X_CAN_MSR_SLAKI    ((CPU_INT32U)0x00000010) /* Sleep acknowledge interrupt            */

/* STM32F10X_CAN Transmit Status Register bits */
#define STM32F10X_CAN_TSR_RQCP0    ((CPU_INT32U)0x00000001) /* Request completed mailbox0             */
#define STM32F10X_CAN_TSR_TXOK0    ((CPU_INT32U)0x00000002) /* Transmission OK of mailbox0            */
#define STM32F10X_CAN_TSR_ABRQ0    ((CPU_INT32U)0x00000080) /* Abort request for mailbox0             */
#define STM32F10X_CAN_TSR_RQCP1    ((CPU_INT32U)0x00000100) /* Request completed mailbox1             */
#define STM32F10X_CAN_TSR_TXOK1    ((CPU_INT32U)0x00000200) /* Transmission OK of mailbox1            */
#define STM32F10X_CAN_TSR_ABRQ1    ((CPU_INT32U)0x00008000) /* Abort request for mailbox1             */
#define STM32F10X_CAN_TSR_RQCP2    ((CPU_INT32U)0x00010000) /* Request completed mailbox2             */
#define STM32F10X_CAN_TSR_TXOK2    ((CPU_INT32U)0x00020000) /* Transmission OK of mailbox2            */
#define STM32F10X_CAN_TSR_ABRQ2    ((CPU_INT32U)0x00800000) /* Abort request for mailbox2             */
#define STM32F10X_CAN_TSR_TME0     ((CPU_INT32U)0x04000000) /* Transmit mailbox 0 empty               */
#define STM32F10X_CAN_TSR_TME1     ((CPU_INT32U)0x08000000) /* Transmit mailbox 1 empty               */
#define STM32F10X_CAN_TSR_TME2     ((CPU_INT32U)0x10000000) /* Transmit mailbox 2 empty               */

/* STM32F10X_CAN Receive FIFO 0 Register bits */
#define STM32F10X_CAN_RF0R_FULL0   ((CPU_INT32U)0x00000008) /* FIFO 0 full                            */
#define STM32F10X_CAN_RF0R_FOVR0   ((CPU_INT32U)0x00000010) /* FIFO 0 overrun                         */
#define STM32F10X_CAN_RF0R_RFOM0   ((CPU_INT32U)0x00000020) /* Release FIFO 0 output mailbox          */

/* STM32F10X_CAN Receive FIFO 1 Register bits */
#define STM32F10X_CAN_RF1R_FULL1   ((CPU_INT32U)0x00000008) /* FIFO 1 full                            */
#define STM32F10X_CAN_RF1R_FOVR1   ((CPU_INT32U)0x00000010) /* FIFO 1 overrun                         */
#define STM32F10X_CAN_RF1R_RFOM1   ((CPU_INT32U)0x00000020) /* Release FIFO 1 output mailbox          */

/* STM32F10X_CAN Error Status Register bits */
#define STM32F10X_CAN_ESR_EWGF     ((CPU_INT32U)0x00000001) /* Error warning flag                     */
#define STM32F10X_CAN_ESR_EPVF     ((CPU_INT32U)0x00000002) /* Error passive flag                     */
#define STM32F10X_CAN_ESR_BOFF     ((CPU_INT32U)0x00000004) /* Bus-off flag                           */

/* STM32F10X_CAN Mailbox Transmit Request */
#define STM32F10X_CAN_TMIDxR_TXRQ  ((CPU_INT32U)0x00000001) /* Transmit mailbox request               */

/* STM32F10X_CAN Filter Master Register bits */
#define STM32F10X_CAN_FMR_FINIT    ((CPU_INT32U)0x00000001) /* Filter init mode                       */

/*
****************************************************************************************************
*                                            DATA TYPES
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*! \brief                          CAN DEVICE REGISTER LAYOUT
*
*            This types defines the register layout of the STM32F10X CAN TRANSCEIVER
*/
/*------------------------------------------------------------------------------------------------*/

typedef volatile struct {
    CPU_INT32U TIR;
    CPU_INT32U TDTR;
    CPU_INT32U TDLR;
    CPU_INT32U TDHR;
} STM32F10X_CAN_TxMailBox_t;

typedef volatile struct {
    CPU_INT32U RIR;
    CPU_INT32U RDTR;
    CPU_INT32U RDLR;
    CPU_INT32U RDHR;
} STM32F10X_CAN_FIFOMailBox_t;

typedef volatile struct {
    CPU_INT32U FR1;
    CPU_INT32U FR2;
} STM32F10X_CAN_FilterRegister_t;

typedef volatile struct {
    CPU_INT32U                     MCR;
    CPU_INT32U                     MSR;
    CPU_INT32U                     TSR;
    CPU_INT32U                     RF0R;
    CPU_INT32U                     RF1R;
    CPU_INT32U                     IER;
    CPU_INT32U                     ESR;
    CPU_INT32U                     BTR;
    CPU_INT32U                     RESERVED0[88];
    STM32F10X_CAN_TxMailBox_t      TxMailBox[3];
    STM32F10X_CAN_FIFOMailBox_t    FIFOMailBox[2];
    CPU_INT32U                     RESERVED1[12];
    CPU_INT32U                     FMR;
    CPU_INT32U                     FM1R;
    CPU_INT32U                     RESERVED2;
    CPU_INT32U                     FS1R;
    CPU_INT32U                     RESERVED3;
    CPU_INT32U                     FFA1R;
    CPU_INT32U                     RESERVED4;
    CPU_INT32U                     FA1R;
    CPU_INT32U                     RESERVED5[8];
    STM32F10X_CAN_FilterRegister_t FilterRegister[28];
} STM32F10X_CAN_t;

#endif
