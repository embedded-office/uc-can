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

#ifndef _DRV_CAN_REG_H_
#define _DRV_CAN_REG_H_

/*
****************************************************************************************************
*                                             INCLUDES
****************************************************************************************************
*/
#include "cpu.h"

/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/
/* CAN base address              */
#define LM3S9B96_CAN0_BASE_ADDR      (volatile CPU_INT32U *) 0x40040000
#define LM3S9B96_CAN1_BASE_ADDR      (volatile CPU_INT32U *) 0x40041000

/* registers used for CAN initialisation    */

/* Clock Settings */
#define RCGC0_REG                   *(volatile CPU_INT32U *) 0x400FE100 /* Run Mode Clock Gating Control Register 0 */

#define RCGC0_CAN0_BIT               0x01000000
#define RCGC0_CAN1_BIT               0x02000000

#define RCGC2_REG                   *(volatile CPU_INT32U *) 0x400FE108 /* Run Mode Clock Gating Control Register 2 */

#define RCGC0_GPIOA_BIT              0x00000001
#define RCGC0_GPIOB_BIT              0x00000002
#define RCGC0_GPIOC_BIT              0x00000004
#define RCGC0_GPIOD_BIT              0x00000008
#define RCGC0_GPIOE_BIT              0x00000010
#define RCGC0_GPIOF_BIT              0x00000020
#define RCGC0_GPIOG_BIT              0x00000040
#define RCGC0_GPIOH_BIT              0x00000080
#define RCGC0_GPIOJ_BIT              0x00000100


/* Pin Settings */
#define GPIO_PORTA_REG               0x40004000
#define GPIO_PORTB_REG               0x40005000
#define GPIO_PORTD_REG               0x40007000
#define GPIO_PORTF_REG               0x40025000
#define GPIO_AFSEL_OFFSET            0x420             /* GPIO Alternate function select */
#define GPIO_PCTL_OFFSET             0x52C             /* GPIO Port Control */
#define GPIO_DIR_OFFSET              0x400             /* GPIO Dir Control */

#define PIN_PA4                      0x5               /* Bit field encoding for GPIOPCTL PMCx */
#define PIN_PA5                      0x5
#define PIN_PA6                      0x6
#define PIN_PA7                      0x6
#define PIN_PB4                      0x5
#define PIN_PB5                      0x5
#define PIN_PD0                      0x2
#define PIN_PD1                      0x2
#define PIN_PF0                      0x1
#define PIN_PF1                      0x1

#define PIN_PA4_PA5                    1
#define PIN_PA6_PA7                    2
#define PIN_PB4_PB5                    3
#define PIN_PD0_PD1                    4
#define PIN_PF0_PF1                    5


/* Interrupt Settings */
#define VIC0_INTER_REG              *(volatile CPU_INT32U *) 0xFFFFF010
#define VIC0_VAR_REG                *(volatile CPU_INT32U *) 0xFFFFF030
#define VIC0_VA0R_ADDR               (volatile CPU_INT32U *) 0xFFFFF100
#define VIC0_VC0R_ADDR               (volatile CPU_INT32U *) 0xFFFFF200


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      BIT DEFINITIONS
* \ingroup  LM3S9B96_CAN
*
*           This member holds a bit defintions for various registers of LM3S9B96
*/
/*------------------------------------------------------------------------------------------------*/

/* Control register*/
#define CAN_CR_TEST            0x0080
#define CAN_CR_CCE             0x0040
#define CAN_CR_DAR             0x0020
#define CAN_CR_EIE             0x0008
#define CAN_CR_SIE             0x0004
#define CAN_CR_IE              0x0002
#define CAN_CR_INIT            0x0001

/* Status register */
#define CAN_SR_BOFF            0x0080
#define CAN_SR_EWARN           0x0040
#define CAN_SR_EPASS           0x0020
#define CAN_SR_RXOK            0x0010
#define CAN_SR_TXOK            0x0008
#define CAN_SR_LEC             0x0007

/* Test register*/
#define CAN_TESTR_RX           0x0080
#define CAN_TESTR_TX1          0x0040
#define CAN_TESTR_TX0          0x0020
#define CAN_TESTR_LBACK        0x0010
#define CAN_TESTR_SILENT       0x0008
#define CAN_TESTR_BASIC        0x0004

/* IFn / Command Request register*/
#define CAN_CRR_BUSY           0x8000

/* IFn / Command Mask register*/
#define CAN_CMR_WRRD           0x0080
#define CAN_CMR_MASK           0x0040
#define CAN_CMR_ARB            0x0020
#define CAN_CMR_CONTROL        0x0010
#define CAN_CMR_CLRINTPND      0x0008
#define CAN_CMR_TXRQSTNEWDAT   0x0004
#define CAN_CMR_DATAA          0x0002
#define CAN_CMR_DATAB          0x0001

/* IFn / Mask 2 register*/
#define CAN_M2R_MXTD           0x8000
#define CAN_M2R_MDIR           0x4000

/* IFn / Arbitration 2 register*/
#define CAN_A2R_MSGVAL         0x8000
#define CAN_A2R_XTD            0x4000
#define CAN_A2R_DIR            0x2000

/* IFn / Message Control register*/
#define CAN_MCR_NEWDAT         0x8000
#define CAN_MCR_MSGLST         0x4000
#define CAN_MCR_INTPND         0x2000
#define CAN_MCR_UMASK          0x1000
#define CAN_MCR_TXIE           0x0800
#define CAN_MCR_RXIE           0x0400
#define CAN_MCR_RMTEN          0x0200
#define CAN_MCR_TXRQST         0x0100
#define CAN_MCR_EOB            0x0080

/*
****************************************************************************************************
*                                            DATA TYPES
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      REGISTER LAYOUT CAN MODULE OF LM3S9B96
* \ingroup  LM3S9B96_CAN
*
*           This definition holds the register layout of the CAN module of LM3S9B96. For detailed
*           register descriptions please refer to STR91F reference manual.
*/
/*------------------------------------------------------------------------------------------------*/

typedef struct {
    CPU_INT16U CRR;                                   /* IFn Command request Register             */
    CPU_INT16U EMPTY1;
    CPU_INT16U CMR;                                   /* IFn Command Mask Register                */
    CPU_INT16U EMPTY2;
    CPU_INT16U M1R;                                   /* IFn Message Mask 1 Register              */
    CPU_INT16U EMPTY3;
    CPU_INT16U M2R;                                   /* IFn Message Mask 2 Register              */
    CPU_INT16U EMPTY4;
    CPU_INT16U A1R;                                   /* IFn Message Arbitration 1 Register       */
    CPU_INT16U EMPTY5;
    CPU_INT16U A2R;                                   /* IFn Message Arbitration 2 Register       */
    CPU_INT16U EMPTY6;
    CPU_INT16U MCR;                                   /* IFn Message Control Register             */
    CPU_INT16U EMPTY7;
    CPU_INT16U DA1R;                                  /* IFn DATA A 1 Register                    */
    CPU_INT16U EMPTY8;
    CPU_INT16U DA2R;                                  /* IFn DATA A 2 Register                    */
    CPU_INT16U EMPTY9;
    CPU_INT16U DB1R;                                  /* IFn DATA B 1 Register                    */
    CPU_INT16U EMPTY10;
    CPU_INT16U DB2R;                                  /* IFn DATA B 2 Register                    */
    CPU_INT16U EMPTY11[27];
} CAN_MSG_OBJS;


typedef struct {
    CPU_INT16U CR;                                    /* Control Register                         */
    CPU_INT16U EMPTY1;
    CPU_INT16U SR;                                    /* Status Register                          */
    CPU_INT16U EMPTY2;
    CPU_INT16U ERR;                                   /* Error counter Register                   */
    CPU_INT16U EMPTY3;
    CPU_INT16U BTR;                                   /* Bit Timing Register                      */
    CPU_INT16U EMPTY4;
    CPU_INT16U IDR;                                   /* Interrupt Identifier Register            */
    CPU_INT16U EMPTY5;
    CPU_INT16U TESTR;                                 /* Test Register                            */
    CPU_INT16U EMPTY6;
    CPU_INT16U BRPR;                                  /* BRP Extension Register                   */
    CPU_INT16U EMPTY7[3];
    CAN_MSG_OBJS MsgObj[2];
    CPU_INT16U EMPTY8[16];
    CPU_INT16U TXR1R;                                 /* Transmission request 1 Register          */
    CPU_INT16U EMPTY9;
    CPU_INT16U TXR2R;                                 /* Transmission Request 2 Register          */
    CPU_INT16U EMPTY10[13];
    CPU_INT16U ND1R;                                  /* New Data 1 Register                      */
    CPU_INT16U EMPTY11;
    CPU_INT16U ND2R;                                  /* New Data 2 Register                      */
    CPU_INT16U EMPTY12[13];
    CPU_INT16U IP1R;                                  /* Interrupt Pending 1 Register             */
    CPU_INT16U EMPTY13;
    CPU_INT16U IP2R;                                  /* Interrupt Pending 2 Register             */
    CPU_INT16U EMPTY14[13];
    CPU_INT16U MV1R;                                  /* Message Valid 1 Register                 */
    CPU_INT16U EMPTY15;
    CPU_INT16U MV2R;                                  /* Message VAlid 2 Register                 */
    CPU_INT16U EMPTY16;
} CAN_LM3S9B96;

/*
****************************************************************************************************
*                                       FUNCTION PROTOTYPES
****************************************************************************************************
*/

/*
****************************************************************************************************
*                                          ERROR SECTION
****************************************************************************************************
*/


#endif  /* #ifndef _DRV_CAN_REG_H_ */




