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
/* unbuffered CAN base address              */
#define ST91X_CAN_BASE_ADDR          (volatile CPU_INT32U *) 0x5C009000

/* registers used for CAN initialisation    */
#define SCU_PRR1_REG                *(volatile CPU_INT32U *) 0x5C002020
#define SCU_PCGR1_REG               *(volatile CPU_INT32U *) 0x5C002018
#define SCU_GPIOOUT3_REG            *(volatile CPU_INT32U *) 0x5C002050
#define SCU_GPIOOUT5_REG            *(volatile CPU_INT32U *) 0x5C002058
#define SCU_GPIOIN3_REG             *(volatile CPU_INT32U *) 0x5C002070
#define SCU_GPIOIN5_REG             *(volatile CPU_INT32U *) 0x5C002078

#define GPIO3_DIR_REG               *(volatile CPU_INT32U *) 0x58009400
#define GPIO5_DIR_REG               *(volatile CPU_INT32U *) 0x5800B400

#define VIC0_INTER_REG              *(volatile CPU_INT32U *) 0xFFFFF010
#define VIC0_VAR_REG                *(volatile CPU_INT32U *) 0xFFFFF030
#define VIC0_VA0R_ADDR               (volatile CPU_INT32U *) 0xFFFFF100
#define VIC0_VC0R_ADDR               (volatile CPU_INT32U *) 0xFFFFF200


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      BIT DEFINITIONS
* \ingroup  ST91X_CAN
*
*           This member holds a bit defintions for various registers of ST91X
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
* \brief                      REGISTER LAYOUT CAN MODULE OF ST91X
* \ingroup  ST91X_CAN
*
*           This definition holds the register layout of the CAN module of ST91X. For detailed 
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
} CAN_ST91X;

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




