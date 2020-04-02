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
*                                            Renesas RX600
*
* Filename : drv_can_reg.h
* Version  : V2.42.01
*********************************************************************************************************
*/

#ifndef _DRV_CAN_REG_H
#define _DRV_CAN_REG_H


/*
*********************************************************************************************************
*                                              INCLUDES
*********************************************************************************************************
*/

#include "cpu.h"
#include "can_cfg.h"


/*
*********************************************************************************************************
*                                          ADDRESS REGISTERS
*********************************************************************************************************
*/
                                                                /* ------------------- BASE ADDRESS ------------------- */
#define  RX600_CAN0_ADDR                        0x00090200u
#define  RX600_CAN1_ADDR                        0x00091200u
#define  RX600_CAN2_ADDR                        0x00092200u


/*
*********************************************************************************************************
*                                           BIT DEFINITIONS
*********************************************************************************************************
*/
                                                                /* -------------- CAN CONTROL REGISTER ---------------- */
#define  RX600_CAN_CTRL_MBM                     DEF_BIT_00

#define  RX600_CAN_CTRL_IDFM_STANDARD           (0u << 1u)
#define  RX600_CAN_CTRL_IDFM_EXTENDED           (1u << 1u)
#define  RX600_CAN_CTRL_IDFM_MIXED              (2u << 1u)

#define  RX600_CAN_CTRL_MLM                     DEF_BIT_03
#define  RX600_CAN_CTRL_TPM                     DEF_BIT_04
#define  RX600_CAN_CTRL_TSRC                    DEF_BIT_05

#define  RX600_CAN_CTRL_TSPS_EVERY              (0u << 6u)
#define  RX600_CAN_CTRL_TSPS_2BIT               (1u << 6u)
#define  RX600_CAN_CTRL_TSPS_4BIT               (2u << 6u)
#define  RX600_CAN_CTRL_TSPS_8BIT               (3u << 6u)

#define  RX600_CAN_CTRL_CANM                    (3u << 8u)
#define  RX600_CAN_CTRL_CANM_RESET              (1u << 8u)
#define  RX600_CAN_CTRL_CANM_HALT               (2u << 8u)
#define  RX600_CAN_CTRL_CANM_FORCE_RESET        (3u << 8u)

#define  RX600_CAN_CTRL_SLPM                    DEF_BIT_10

#define  RX600_CAN_CTRL_BOM_BUS_OFF_ENTRY       (1u << 11u)
#define  RX600_CAN_CTRL_BOM_BUS_OFF_END         (2u << 11u)
#define  RX600_CAN_CTRL_BOM_BUS_OFF_RECOVERY    (3u << 11u)


#define  RX600_CAN_CTRL_RBOC                    DEF_BIT_13

                                                                /* ------- FIFO RECIEVED ID COMPARE REGISTER 0 -------- */
#define  RX600_CAN_FIDCR0_RTR                   DEF_BIT_30
#define  RX600_CAN_FIDCR0_IDE                   DEF_BIT_31

                                                                /* ------- FIFO RECIEVED ID COMPARE REGISTER 1 -------- */
#define  RX600_CAN_FIDCR1_RTR                   DEF_BIT_30
#define  RX600_CAN_FIDCR1_IDE                   DEF_BIT_31

                                                                /* ---------------- MAILBOX REGISTER ------------------ */
#define  RX600_CAN_MB_ID_RTR                    DEF_BIT_30
#define  RX600_CAN_MB_ID_IDE                    DEF_BIT_31

                                                                /* -------- MAILBOX INTERRUPT ENABLE REGISTER --------- */
#define  RX600_CAN_MIER_TX_FIFO_INT_EN          DEF_BIT_24
#define  RX600_CAN_MIER_RX_FIFO_INT_EN          DEF_BIT_28

                                                                /* ------------ MESSAGE CONTROL REGISTER -------------- */
#define  RX600_CAN_MCTL_SENTDATA                DEF_BIT_00
#define  RX600_CAN_MCTL_NEWDATA                 DEF_BIT_00
#define  RX600_CAN_MCTL_TRMACTIVE               DEF_BIT_01
#define  RX600_CAN_MCTL_INVALDATA               DEF_BIT_01
#define  RX600_CAN_MCTL_TRMABT                  DEF_BIT_02
#define  RX600_CAN_MCTL_MSGLOST                 DEF_BIT_02
#define  RX600_CAN_MCTL_ONESHOT                 DEF_BIT_04
#define  RX600_CAN_MCTL_RECREQ                  DEF_BIT_06
#define  RX600_CAN_MCTL_TRMREQ                  DEF_BIT_07

                                                                /* ---------- RECIEVE FIFO CONTROL REGISTER ----------- */
#define  RX600_CAN_RFCR_RFE                     DEF_BIT_00

                                                                /* ---------- TRANSMIT FIFO CONTROL REGISTER ---------- */
#define  RX600_CAN_TFCR_TFE                     DEF_BIT_00

                                                                /* ----------------- STATUS REGISTER ------------------ */
#define  RX600_CAN_STR_NDST                     DEF_BIT_00
#define  RX600_CAN_STR_SDST                     DEF_BIT_01
#define  RX600_CAN_STR_RFST                     DEF_BIT_02
#define  RX600_CAN_STR_TFST                     DEF_BIT_03
#define  RX600_CAN_STR_NMLST                    DEF_BIT_04
#define  RX600_CAN_STR_FMLST                    DEF_BIT_05
#define  RX600_CAN_STR_TABST                    DEF_BIT_06
#define  RX600_CAN_STR_EST                      DEF_BIT_07
#define  RX600_CAN_STR_RSTST                    DEF_BIT_08
#define  RX600_CAN_STR_HLTST                    DEF_BIT_09
#define  RX600_CAN_STR_SLPST                    DEF_BIT_10
#define  RX600_CAN_STR_EPST                     DEF_BIT_11
#define  RX600_CAN_STR_BOST                     DEF_BIT_12
#define  RX600_CAN_STR_TRMST                    DEF_BIT_13
#define  RX600_CAN_STR_RECST                    DEF_BIT_14

                                                                /* ---------- MAILBOX SEARCH STATUS REGISTER ---------- */
#define  RX600_CAN_MSSR_SEST                    DEF_BIT_07

                                                                /* ---------- ERROR INTERRUPT ENABLE REGISTER --------- */
#define  RX600_CAN_EIER_BEIE                    DEF_BIT_00
#define  RX600_CAN_EIER_EWIE                    DEF_BIT_01
#define  RX600_CAN_EIER_EPIE                    DEF_BIT_02
#define  RX600_CAN_EIER_BOEIE                   DEF_BIT_03
#define  RX600_CAN_EIER_BORIE                   DEF_BIT_04
#define  RX600_CAN_EIER_ORIE                    DEF_BIT_05
#define  RX600_CAN_EIER_OLIE                    DEF_BIT_06
#define  RX600_CAN_EIER_BLIE                    DEF_BIT_07

                                                                /* ------- ERROR INTERRUPT FACTOR JUDGE REGISTER ------ */
#define  RX600_CAN_EIFR_BEIF                    DEF_BIT_00
#define  RX600_CAN_EIFR_EWIF                    DEF_BIT_01
#define  RX600_CAN_EIFR_EPIF                    DEF_BIT_02
#define  RX600_CAN_EIFR_BOEIF                   DEF_BIT_03
#define  RX600_CAN_EIFR_BORIF                   DEF_BIT_04
#define  RX600_CAN_EIFR_ORIF                    DEF_BIT_05
#define  RX600_CAN_EIFR_OLIF                    DEF_BIT_06
#define  RX600_CAN_EIFR_BLIF                    DEF_BIT_07

                                                                /* ------------ ERROR CODE STORE REGISTER ------------- */
#define  RX600_CAN_ECSR_SEF                     DEF_BIT_00
#define  RX600_CAN_ECSR_FEF                     DEF_BIT_01
#define  RX600_CAN_ECSR_AEF                     DEF_BIT_02
#define  RX600_CAN_ECSR_CEF                     DEF_BIT_03
#define  RX600_CAN_ECSR_BE1F                    DEF_BIT_04
#define  RX600_CAN_ECSR_BE0F                    DEF_BIT_05
#define  RX600_CAN_ECSR_ADEF                    DEF_BIT_06
#define  RX600_CAN_ECSR_EDPM                    DEF_BIT_07

                                                                /* -------------- TEST CONTROL REGISTER --------------- */
#define  RX600_CAN_TCR_TSTE                     DEF_BIT_00

#define  RX600_CAN_TCR_TSTM_OTHER_CAN_TEST      (0u << 1u)
#define  RX600_CAN_TCR_TSTM_LISTEN              (1u << 1u)
#define  RX600_CAN_TCR_TSTM_EXTERNAL_LOOP       (2u << 1u)
#define  RX600_CAN_TCR_TSTM_INTERNAL_LOOP       (3u << 1u)


/*
*********************************************************************************************************
*                                               MACROS
*********************************************************************************************************
*/
                                                                /* --------- MAILBOX INTERRUPT ENABLE REGISTER -------- */
#define  RX600_CAN_MAILBOX_MIER_INT_EN(x)       (1u << (x))     /* Enable / Disable Secific Mailbox Interrupt.          */

                                                                /* --------------- MASK INVALID REGISTER -------------- */
#define  RX600_CAN_MKIVLR_MB(x)                 (1u << (x))     /* Validate / Invalidate Specific Mailbox.              */


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

typedef  volatile  struct  rx600_can_reg {
    struct {
        CPU_REG32  ID;
        CPU_REG16  DLC;
        CPU_REG08  DATA[8u];
        CPU_REG16  TS;
    } MB[32u];
    CPU_REG32  MKR[8u];
    CPU_REG32  FIDCR0;
    CPU_REG32  FIDCR1;
    CPU_REG32  MKIVLR;
    CPU_REG32  MIER;
    CPU_REG08  RSVD_00[1008u];
    CPU_REG08  MCTL[32u];
    CPU_REG16  CTLR;
    CPU_REG16  STR;
    CPU_REG32  BCR;
    CPU_REG08  RFCR;
    CPU_REG08  RFPCR;
    CPU_REG08  TFCR;
    CPU_REG08  TFPCR;
    CPU_REG08  EIER;
    CPU_REG08  EIFR;
    CPU_REG08  RECR;
    CPU_REG08  TECR;
    CPU_REG08  ECSR;
    CPU_REG08  CSSR;
    CPU_REG08  MSSR;
    CPU_REG08  MSMR;
    CPU_REG16  TSR;
    CPU_REG16  AFSR;
    CPU_REG08  TCR;
} RX600_CAN_REG;


/*
*********************************************************************************************************
*                                        CAN BAUDRATE REGISTER
*
* Description : Structure defines the CAN BaudRate Register Structure.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  struct  rx600_can_baud {    
    CPU_INT32U  BaudRate;                                       /* Holds the Baudrate.                                  */
    CPU_INT32U  SamplePoint;                                    /* Holds the Sample point in 0.1%                       */
    CPU_INT32U  ReSynchJumpWidth;                               /* Holds the Re-Synchronization Jump Width in 0.1%      */
    CPU_INT08U  PrescalerDiv;                                   /* Holds the Prescaler Divide Factor                    */
    CPU_INT08U  SJW;                                            /* Holds the Re-Synch Jump Width         (StdValue = 1) */
    CPU_INT08U  PropagationSeg;                                 /* Holds the Propagation Segment Time    (StdValue = 2) */
    CPU_INT08U  PhaseBufSeg1;                                   /* Holds the Phase Buffer Segment 1      (StdValue = 7) */
    CPU_INT08U  PhaseBufSeg2;                                   /* Holds the Phase Buffer Segment 2      (StdValue = 7) */
} RX600_CAN_BAUD;


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
