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

#define ADSPBF537_CAN0_BASE_ADDR        (CPU_REG32 *)0xFFC02A00



/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      BIT DEFINITIONS
* \ingroup  ADSPBF537_CAN
*
*           This member holds a bit defintions for various registers of ADSPBF537
*/
/*------------------------------------------------------------------------------------------------*/

#define CAN_PORT_MUX       *(CPU_REG16 *)0xFFC0320C   /* Port Multiplexer Control Register        */
#define CAN_PJCE           0x0002                     /* Enable CAN RX/TX                         */
#define CAN_PJCE_MASK      0x0006                     /* CAN RX/TX mask                           */

#define CAN_SIC_IAR0       *(CPU_REG32 *)0xFFC00110   /* Interrupt Assignment register            */
#define CAN_SIC_IAR1       *(CPU_REG32 *)0xFFC00114
#define CAN_SIC_IAR2       *(CPU_REG32 *)0xFFC00118
                                                      /* Interrupt Mask Register                  */
#define CAN_SIC_IMASK            *(CPU_REG32 *)0xFFC0010C
#define CAN_SIC_IMASK_TX_IRQ     0x00010000            
#define CAN_SIC_IMASK_RX_IRQ     0x00008000            
#define CAN_SIC_IMASK_ERR_IRQ    0x00000004            


#define CAN_CONTROL_CCR    0x0080                     /* CAN Configuration Mode Request           */

#define CAN_STATUS_CCA     0x0080                     /* CAN Configuration mode acknowledge       */
#define CAN_STATUS_WTR     0x0003                     /* CAN Warning flags RX/TX                  */
#define CAN_STATUS_EP      0x0004                     /* CAN Error passive flag                   */
#define CAN_STATUS_EBO     0x0008                     /* CAN Bus Off flag                         */

#define CAN_MB_ID1_IDE     0x2000                     /* Identifier Extension (IDE) bit           */
#define CAN_MB_ID1_RTR     0x4000                     /* Remote transmission request bit          */
#define CAN_MB_ID1_AME     0x8000                     /* Acceptance Mask Identification bit       */

#define CAN_AMxx_AMIDE     0x2000                     /* Acceptance Mask IDE  bit                 */

#define CAN_GIM_EPIM       0x0004                     /* Error Passive Interrupt                  */
#define CAN_GIM_BOIM       0x0008                     /* Bus Off Interrupt                        */

#define CAN_GIS_EPIS       0x0004                     /* CAN Error passive flag                   */
#define CAN_GIS_EBOIS      0x0008                     /* CAN Bus Off flag                         */

/*
****************************************************************************************************
*                                            DATA TYPES
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      REGISTER LAYOUT CAN MODULE OF ADSPBF537
* \ingroup  ADSPBF537_CAN
*
*           This definition holds the register layout of the CAN module of ADSPBF537. For detailed
*           register descriptions please refer to the reference manual.
*/
/*------------------------------------------------------------------------------------------------*/

/* Mailbox Registers */

typedef struct {
    CPU_INT16U AM_x_L;                                /* Mailbox 0 Low Acceptance Mask */
    CPU_INT16U Spare43;
    CPU_INT16U AM_x_H;                                /* Mailbox 0 Low Acceptance Mask */
    CPU_INT16U Spare44;
} CAN_MB_ACC;

                                                      /* 0xFFC02C00 -- 0xFFC02FFC */
typedef struct {
    CPU_INT16U DATA0;                                 /* Mailbox Data Word 0 [15:0] Register  */
    CPU_INT16U Spare45;
    CPU_INT16U DATA1;                                 /* Mailbox Data Word 1 [31:16] Register */
    CPU_INT16U Spare46;
    CPU_INT16U DATA2;                                 /* Mailbox Data Word 2 [47:32] Register */
    CPU_INT16U Spare47;
    CPU_INT16U DATA3;                                 /* Mailbox Data Word 3 [63:48] Register */
    CPU_INT16U Spare48;
    CPU_INT16U LENGTH;                                /* Mailbox Data Length Code Register    */
    CPU_INT16U Spare49;
    CPU_INT16U TIMESTAMP;                             /* Mailbox Time Stamp Value Register    */
    CPU_INT16U Spare50;
    CPU_INT16U ID0;                                   /* Mailbox Identifier Low Register      */
    CPU_INT16U Spare51;
    CPU_INT16U ID1;                                   /* Mailbox Identifier High Register     */
    CPU_INT16U Spare52;
} CAN_MB;


/* CAN Controller (0xFFC02A00 - 0xFFC02FFF) */


typedef volatile struct {
    CPU_INT16U MC1;                                   /* Mailbox config reg 1 */
    CPU_INT16U Spare0;
    CPU_INT16U MD1;                                   /* Mailbox direction reg 1 */
    CPU_INT16U Spare1;
    CPU_INT16U TRS1;                                  /* Transmit Request Set reg 1 */
    CPU_INT16U Spare2;
    CPU_INT16U TRR1;                                  /* Transmit Request Reset reg 1 */
    CPU_INT16U Spare3;
    CPU_INT16U TA1;                                   /* Transmit Acknowledge reg 1 */
    CPU_INT16U Spare4;
    CPU_INT16U AA1;                                   /* Transmit Abort Acknowledge reg 1 */
    CPU_INT16U Spare5;
    CPU_INT16U RMP1;                                  /* Receive Message Pending reg 1 */
    CPU_INT16U Spare6;
    CPU_INT16U RML1;                                  /* Receive Message Lost reg 1 */
    CPU_INT16U Spare7;
    CPU_INT16U MBTIF1;                                /* Mailbox Transmit Interrupt Flag reg 1 */
    CPU_INT16U Spare8;
    CPU_INT16U MBRIF1;                                /* Mailbox Receive  Interrupt Flag reg 1 */
    CPU_INT16U Spare9;
    CPU_INT16U MBIM1;                                 /* Mailbox Interrupt Mask reg 1 */
    CPU_INT16U Spare10;
    CPU_INT16U RFH1;                                  /* Remote Frame Handling reg 1 */
    CPU_INT16U Spare11;
    CPU_INT16U OPSS1;                                 /* Overwrite Protection Single Shot Xmit reg 1 */
    CPU_INT16U Spare12;

    CPU_INT32U Rsvd0[3];

    CPU_INT16U MC2;                                   /* Mailbox config reg 2 */
    CPU_INT16U Spare13;
    CPU_INT16U MD2;                                   /* Mailbox direction reg 2 */
    CPU_INT16U Spare14;
    CPU_INT16U TRS2;                                  /* Transmit Request Set rg 2 */
    CPU_INT16U Spare15;
    CPU_INT16U TRR2;                                  /* Transmit Request Reset reg 2 */
    CPU_INT16U Spare16;
    CPU_INT16U TA2;                                   /* Transmit Acknowledge reg 2 */
    CPU_INT16U Spare17;
    CPU_INT16U AA2;                                   /* Transmit Abort Acknowledge reg 2 */
    CPU_INT16U Spare18;
    CPU_INT16U RMP2;                                  /* Receive Message Pending reg 2 */
    CPU_INT16U Spare19;
    CPU_INT16U RML2;                                  /* Receive Message Lost reg 2 */
    CPU_INT16U Spare20;
    CPU_INT16U MBTIF2;                                /* Mailbox Transmit Interrupt Flag reg 2 */
    CPU_INT16U Spare21;
    CPU_INT16U MBRIF2;                                /* Mailbox Receive  Interrupt Flag reg 2 */
    CPU_INT16U Spare22;
    CPU_INT16U MBIM2;                                 /* Mailbox Interrupt Mask reg 2 */
    CPU_INT16U Spare23;
    CPU_INT16U RFH2;                                  /* Remote Frame Handling reg 2 */
    CPU_INT16U Spare24;
    CPU_INT16U OPSS2;                                 /* Overwrite Protection Single Shot Xmit reg 2 */
    CPU_INT16U Spare25;

    CPU_INT32U Rsvd1[3];
    
    CPU_INT16U CLOCK;                                 /* Bit Timing Configuration register 0 0xFFC02A80*/
    CPU_INT16U Spare26;
    CPU_INT16U TIMING;                                /* Bit Timing Configuration register 1 */
    CPU_INT16U Spare27;
    CPU_INT16U DEBUG;                                 /* Debug Register */
    CPU_INT16U Spare28;
    CPU_INT16U STATUS;                                /* Global Status Register */
    CPU_INT16U Spare29;
    CPU_INT16U CEC;                                   /* Error Counter Register */
    CPU_INT16U Spare30;
    CPU_INT16U GIS;                                   /* Global Interrupt Status Register */
    CPU_INT16U Spare31;
    CPU_INT16U GIM;                                   /* Global Interrupt Mask Register */
    CPU_INT16U Spare32;
    CPU_INT16U GIF;                                   /* Global Interrupt Flag Register */
    CPU_INT16U Spare33;
    CPU_INT16U CONTROL;                               /* Master Control Register */
    CPU_INT16U Spare34;
    CPU_INT16U INTR;                                  /* Interrupt Pending Register 0xFFC02AA4 */
    CPU_INT16U Spare35;
    CPU_INT16U Rsvd2;
    CPU_INT16U Spare36;
    CPU_INT16U MBTD;                                  /* Mailbox Temporary Disable Feature 0xFFC02AAC */
    CPU_INT16U Spare37;
    CPU_INT16U EWR;                                   /* Programmable Warning Level */
    CPU_INT16U Spare38;
    CPU_INT16U ESR;                                   /* Error Status Register */
    CPU_INT16U Spare39;

    CPU_INT32U Rsvd3[4];

    CPU_INT16U UCCNT;                                 /* Universal Counter 0xFFC02AC4 */
    CPU_INT16U Spare40;
    CPU_INT16U UCRC;                                  /* Universal Counter Reload/Capture Register */
    CPU_INT16U Spare41;
    CPU_INT16U UCCNF;                                 /* Universal Counter Configuration Register 0xFFC02ACC */
    CPU_INT16U Spare42;

    CPU_INT32U Rsvd4[11];

    CAN_MB_ACC MB_ACC[32];                            /* Mailbox Acceptance Masks 0-31 0xFFC02B00--0xFFC02BFC */

    CAN_MB     MB[32];                                /* Mailbox register 0-31 0xFFC02C00--0xFFC02FFC  */
    
}CAN_ADSPBF537;




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




