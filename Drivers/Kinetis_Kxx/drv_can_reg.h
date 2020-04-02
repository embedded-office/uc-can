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
*                                         CAN DRIVER CODE
*
*                                           Kinetis_Kxx
*
* Filename : drv_can_reg.h
* Version  : V2.42.01
****************************************************************************************************
* Note(s)  : (1) Supports Freescale's Kinetis K Series MCUs as described in various datasheets.
*                This driver has been tested with or should work with the following K Series MCUs :
*
*                    Kinetis K40
*                    Kinetis K60
*                    Kinetis K64
*                    Kinetis K70
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
                                                                /* ---------------------------------------------------- */
                                                                /* The base address of the FlexCAN module               */
                                                                /* ---------------------------------------------------- */
#define  KXX_CAN0_BASE_ADDR                0x40024000u          /* Base address of FlexCAN 0                            */
#define  KXX_CAN1_BASE_ADDR                0x400A4000u          /* Base address of FlexCAN 1                            */
                                                                /* ---------------------------------------------------- */
                                                                /* FlexCan message buffer defines                       */
                                                                /* ---------------------------------------------------- */
#define  KXX_CAN_BUFF_NOT_ACTIVE           0x00000000u          /* Message buffer is not active                         */

#define  KXX_CAN_RX_BUFF_NOT_ACTIVE               0x0u          /* Rx Message buffer is not active                      */
#define  KXX_CAN_RX_BUFF_EMPTY                    0x4u          /* Rx Message buffer is active and empty                */
#define  KXX_CAN_RX_BUFF_FULL                     0x2u          /* Rx Message buffer is active and full                 */
#define  KXX_CAN_RX_BUFF_OVERRUN                  0x6u          /* Rx Message buffer overrun                            */
#define  KXX_CAN_RX_BUFF_BUSY                     0x1u          /* Rx Message buffer is busy                            */

#define  KXX_CAN_TX_BUFF_NOT_READY                0x8u          /* Tx Message buffer is not ready                       */
#define  KXX_CAN_TX_BUFF_RESPONSE                 0xAu          /* Tx Message buffer automaticly                        */
#define  KXX_CAN_TX_BUFF_SEND                     0xCu          /* Tx Message buffer is ready to send                   */

#define  KXX_CAN_MB_ID_STANDARD                   0x0u          /* Standard ID flag in MB_CS[IDE] register              */
#define  KXX_CAN_MB_ID_EXTENDED                   0x1u          /* Extended ID flag in MB_CS[IDE] register              */

#define  KXX_CAN_N_MSG_BUFF                        16u          /* Number of FLexCAN message buffers                    */
                                                                /* ---------------------------------------------------- */
                                                                /* FlexCan CR register defines                          */
                                                                /* ---------------------------------------------------- */
#define  KXX_CAN_CLK_SRC_SYSCLK            DEF_BIT_13           /* clk source is the system clock                       */
#define  KXX_CAN_LBUF_LOWBUF_FIRST         DEF_BIT_04           /* lowest number buffer transmitted first               */
#define  KXX_CAN_BOFFMSK_EN                DEF_BIT_15           /* bus off interrupt enabled                            */
#define  KXX_CAN_ERRMSK_EN                 DEF_BIT_14           /* error interrupt enabled                              */
#define  KXX_CAN_LPB_EN                    DEF_BIT_12           /* Loop back enabled                                    */
#define  KXX_CAN_SMP_THREE                 DEF_BIT_07           /* three sample are used                                */
#define  KXX_CAN_BOFFREC_DIS               DEF_BIT_06          /* atom. recovering from bus off disabled               */
#define  KXX_CAN_TSYN_EN                   DEF_BIT_05          /* timer sync feature enabled                           */
#define  KXX_CAN_LOM_LISTEN_ONLY           DEF_BIT_03          /* listen only mode enabled                             */
#define  KXX_CAN_CR_MASK                   0x0000F0F8u
                                                                /* ---------------------------------------------------- */
                                                                /* FlexCan ESR register defines                         */
                                                                /* ---------------------------------------------------- */
#define  KXX_CAN_ESR_BOFFINT               0x00000004u          /* Interrupt is due to Bus off Event                    */
#define  KXX_CAN_ESR_ERRINT                0x00000002u          /* Interrupt is due to any Error Bit set                */


/*
****************************************************************************************************
*                                            DATA TYPES
****************************************************************************************************
*/


/*
****************************************************************************************************
*                                        KXX CAN REGISTERS
*
* Note(s) : This structure defines the FlexCAN registers.
****************************************************************************************************
*/

typedef volatile struct {
    union {
        CPU_INT32U R;
        struct {                                                /* ---------------------------------------------------- */
            CPU_INT32U MAXMB    :7;                             /* Maximum number of message buffers                    */
            CPU_INT32U          :1;                             /* reserved                                             */
            CPU_INT32U IDAM     :2;                             /* reserved                                             */
            CPU_INT32U          :2;                             /* reserved                                             */
            CPU_INT32U AEN      :1;                             /* reserved                                             */
            CPU_INT32U LPPRIOEN :1;                             /* reserved                                             */
            CPU_INT32U          :2;                             /* reserved                                             */
            CPU_INT32U IRMQ     :1;                             /* Message buffer filter enable                         */
            CPU_INT32U SRXDIS   :1;                             /* Self reception disable                               */
            CPU_INT32U DOZE     :1;                             /* Doze Mode enable                                     */
            CPU_INT32U          :1;                             /* reserved                                             */
            CPU_INT32U LPMSACK  :1;                             /* Low power mode acknowledge                           */
            CPU_INT32U WRNEN    :1;                             /* Warning interrupt enable                             */
            CPU_INT32U SLFWAK   :1;                             /* reserved                                             */
            CPU_INT32U SUPV     :1;                             /* Supervisor mode                                      */
            CPU_INT32U FRZACK   :1;                             /* Freeze mode acknowledge                              */
            CPU_INT32U SOFTRST  :1;                             /* Soft reset initiated                                 */
            CPU_INT32U WAKMSK   :1;                             /* reserved                                             */
            CPU_INT32U NOTRDY   :1;                             /* FlexCAN not ready                                    */
            CPU_INT32U HALT     :1;                             /* Halt FlexCAN                                         */
            CPU_INT32U RFEN     :1;                             /* reserved                                             */
            CPU_INT32U FRZ      :1;                             /* Freeze enable                                        */
            CPU_INT32U MDIS     :1;                             /* Module disable                                       */
        } B;                                                    /* ---------------------------------------------------- */
    } MCR;                                                      /* Base+0x0000 Module Configuration Register */
                                                                /* ---------------------------------------------------- */
    union {
        CPU_INT32U R;
        struct {                                                /* ---------------------------------------------------- */
            CPU_INT32U PROPSEG  :3;                             /* Propagation segment def.                             */
            CPU_INT32U LOM      :1;                             /* Listen-only mode                                     */
            CPU_INT32U LBUF     :1;                             /* Lowest buffer transmitted first                      */
            CPU_INT32U TSYN     :1;                             /* Timer sync mode                                      */
            CPU_INT32U BOFFREC  :1;                             /* Bus off recovery mode                                */
            CPU_INT32U SMP      :1;                             /* Sampling mode                                        */
            CPU_INT32U          :2;                             /* reserved                                             */
            CPU_INT32U RWRNMSK  :1;                             /* Rx warning interrupt enable                          */
            CPU_INT32U TWRNMSK  :1;                             /* Tx warning interrupt enable                          */
            CPU_INT32U LPB      :1;                             /* Loop back enable                                     */
            CPU_INT32U CLK_SRC  :1;                             /* CAN engine clock source                              */
            CPU_INT32U ERRMSK   :1;                             /* Error mask enable                                    */
            CPU_INT32U BOFFMSK  :1;                             /* Bus off mask enable                                  */
            CPU_INT32U PSEG2    :3;                             /* Phase segment 2                                      */
            CPU_INT32U PSEG1    :3;                             /* Phase segment 1                                      */
            CPU_INT32U RJW      :2;                             /* Resync jump wide                                     */
            CPU_INT32U PRESDIV  :8;                             /* Prescaler division factor                            */
        } B;                                                    /* ---------------------------------------------------- */
    } CR;                                                       /* Base+0x0004 Control Register                         */
                                                                /* ---------------------------------------------------- */
    CPU_INT32U TIMER;                                           /* Base+0x0008 Free Running Timer                       */
                                                                /* ---------------------------------------------------- */
    const CPU_INT32U Res_0x000C;                                /* Base+0x000C reserved                                 */
                                                                /* ---------------------------------------------------- */
    union {
        CPU_INT32U R;
        struct {                                                /* ---------------------------------------------------- */
            CPU_INT32U MI    :29;                               /* ID mask bits                                         */
            CPU_INT32U       :3;                                /* reserved                                             */
        } B;                                                    /* ---------------------------------------------------- */
    } RXGMASK;                                                  /* Base+0x0010 RX Global Mask                           */
                                                                /* ---------------------------------------------------- */
    union {
        CPU_INT32U R;
        struct {                                                /* ---------------------------------------------------- */
            CPU_INT32U MI    :29;                               /* ID mask bits                                         */
            CPU_INT32U       :3;                                /* reserved                                             */
        } B;                                                    /* ---------------------------------------------------- */
    } RX14MASK;                                                 /* Base+0x0014 RX 14 Mask                               */
                                                                /* ---------------------------------------------------- */
    union {
        CPU_INT32U R;
        struct {                                                /* ---------------------------------------------------- */
            CPU_INT32U MI    :29;                               /* ID mask bits                                         */
            CPU_INT32U       :3;                                /* reserved                                             */
        } B;                                                    /* ---------------------------------------------------- */
    } RX15MASK;                                                 /* Base+0x0018 RX 15 Mask                               */
                                                                /* ---------------------------------------------------- */
    union {
        CPU_INT32U R;
        struct {                                                /* ---------------------------------------------------- */
            CPU_INT32U TXECNT :8;                               /* TX error counter                                     */
            CPU_INT32U RXECNT :8;                               /* RX error counter                                     */
            CPU_INT32U        :16;                              /* reserved                                             */
        } B;                                                    /* ---------------------------------------------------- */
    } ECR;                                                      /* Base+0x001C Error Counter Register                   */
                                                                /* ---------------------------------------------------- */
    union {
        CPU_INT32U R;
        struct {                                                /* ---------------------------------------------------- */
            CPU_INT32U          :1;                             /* reserved                                             */
            CPU_INT32U ERRINT   :1;                             /* Error interrupt                                      */
            CPU_INT32U BOFFINT  :1;                             /* Bus off interrupt                                    */
            CPU_INT32U          :1;                             /* reserved                                             */
            CPU_INT32U FLTCONF  :2;                             /* Fault confinement state                              */
            CPU_INT32U TXRX     :1;                             /* Current FlexCAN2 status                              */
            CPU_INT32U IDLE     :1;                             /* CAN bus idle state                                   */
            CPU_INT32U RXWRN    :1;                             /* RX error counter                                     */
            CPU_INT32U TXWRN    :1;                             /* TX error counter                                     */
            CPU_INT32U STFERR   :1;                             /* Stuffing error                                       */
            CPU_INT32U FRMERR   :1;                             /* Form error                                           */
            CPU_INT32U CRCERR   :1;                             /* Cyclic redundancy code error                         */
            CPU_INT32U ACKERR   :1;                             /* Acknowlege error                                     */
            CPU_INT32U BIT0ERR  :1;                             /* Bit 0 error                                          */
            CPU_INT32U BIT1ERR  :1;                             /* Bit 1 error                                          */
            CPU_INT32U RWRNINT  :1;                             /* RXECTR >= 96                                         */
            CPU_INT32U TWRNINT  :1;                             /* TXECTR >= 96                                         */
            CPU_INT32U          :14;                            /* reserved                                             */
        } B;                                                    /* ---------------------------------------------------- */
    } ESR1;                                                     /* Base-0x0020 Error and Status Register                */
                                                                /* ---------------------------------------------------- */
    CPU_INT32U IMASK2;                                          /* Base+0x0024 Interrupt Masks2 Register                */
    CPU_INT32U IMASK1;                                          /* Base+0x0028 Interrupt Masks1 Register                */
                                                                /* ---------------------------------------------------- */
    CPU_INT32U IFLAG2;                                          /* Base+0x2C Interrupt Flag 2 Register                  */
    CPU_INT32U IFLAG1;                                          /* Base+0x30 Interrupt Flag 2 Register                  */

                                                                /* ---------------------------------------------------- */
    CPU_INT32U CTRL2;                                           /* Base+0x0034 Control Register 2                       */
    CPU_INT32U ESR2;                                            /* Base+0x0038 Error & Status Register 2                */
    CPU_INT32U Reserved1[2u];
    CPU_INT32U CRC;                                             /* Base+0x0044 CRC Register                             */
    CPU_INT32U RXFGMASK;                                        /* Base+0x0048 FIFO Global Mask                         */
    CPU_INT32U RXFIR;                                           /* Base+0x004C FIFO Information                         */
                                                                /* ---------------------------------------------------- */
    CPU_INT32U Reserved2[12u];

    struct canbuf_t {
        union {
            CPU_INT32U R;
            struct {                                            /* ---------------------------------------------------- */
                CPU_INT32U TIMESTAMP :16;                       /* Freerunning counter time stamp                       */
                CPU_INT32U LENGTH    :4;                        /* Lengh of data in bytes                               */
                CPU_INT32U RTR       :1;                        /* Remote transmission request                          */
                CPU_INT32U IDE       :1;                        /* ID extended bit                                      */
                CPU_INT32U SRR       :1;                        /* Substitute remote request                            */
                CPU_INT32U           :1;                        /* reserved                                             */
                CPU_INT32U CODE      :4;                        /* Message buffer code                                  */
                CPU_INT32U           :4;                        /* reserved                                             */
            } B;                                                /* ---------------------------------------------------- */
        } CS;                                                   /* 0x80 MB Control and Status CS                        */
                                                                /* ---------------------------------------------------- */
        CPU_INT32U ID;                                          /* 0x84 MB Frame identifier                             */
                                                                /* ---------------------------------------------------- */
        CPU_INT08U Data[8u];                                    /*< 0x88-0x8F MB Data field                             */
                                                                /* ---------------------------------------------------- */
    } BUF[64u];                                                 /* Base+0x0080  Message buffer MB0                      */
                                                                /* Base+0x047F  Message buffer MB63                     */
    CPU_INT32U Reserved3[0x100u];
    CPU_INT32U RXIMR[16u];                                      /* Base+0x0880  Individual Mask                         */


} KXX_CAN_REG;                                                  /* ---------------------------------------------------- */


#endif  /* #ifndef _DRV_CAN_REG_H_ */
