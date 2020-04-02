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
*                                            Renesas RX200
*
* Filename : drv_can_reg.h
* Version  : V2.42.01
*********************************************************************************************************
* Note(s)  : (1) This CAN Driver supports the following Series/Families:
*                    RX200 Driver - Renesas RX200    Family.
*                                 - Renesas RL78-F13 Series.
*                                 - Renesas RL78-F14 Series.
*
*                Set by the Technical Reference Manual(s) obtained from the Renesas website. This driver
*                has been tested with or should work with the families mentioned above.
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
#define  RX200_CAN_ADDR                         0x000A8300u
#define  RL78F_CAN_ADDR                         0x000F0300u

                                                                /* --------- RECEIVE RULE ENTRY ADDRESS OFFSET -------- */
#define  RX200_CAN_RR_ADDR_OFFSET               0xA0u


/*
*********************************************************************************************************
*                                           BIT DEFINITIONS
*********************************************************************************************************
*/
                                                                /* ---------------- CONTROL REGISTER L ---------------- */
#define  RX200_CAN_CTRL_CHMDC_COMM             (0u << 0u)
#define  RX200_CAN_CTRL_CHMDC_RESET            (1u << 0u)
#define  RX200_CAN_CTRL_CHMDC_HALT             (2u << 0u)
#define  RX200_CAN_CTRL_CHMDC_MASK              0x03u

#define  RX200_CAN_CTRL_CSLPR                   DEF_BIT_02
#define  RX200_CAN_CTRL_RTBO                    DEF_BIT_03
#define  RX200_CAN_CTRL_BEIE                    DEF_BIT_08
#define  RX200_CAN_CTRL_EWIE                    DEF_BIT_09
#define  RX200_CAN_CTRL_EPIE                    DEF_BIT_10
#define  RX200_CAN_CTRL_BOEIE                   DEF_BIT_11
#define  RX200_CAN_CTRL_BORIE                   DEF_BIT_12
#define  RX200_CAN_CTRL_OLIE                    DEF_BIT_13
#define  RX200_CAN_CTRL_BLIE                    DEF_BIT_14
#define  RX200_CAN_CTRL_ALIE                    DEF_BIT_15

                                                                /* ---------------- CONTROL REGISTER H ---------------- */
#define  RX200_CAN_CTRH_TAIE                    DEF_BIT_00

#define  RX200_CAN_CTRH_BOM_ISO11898           (0u << 5u)
#define  RX200_CAN_CTRH_BOM_HALT_ENTRY         (1u << 5u)
#define  RX200_CAN_CTRH_BOM_HALT_END           (2u << 5u)
#define  RX200_CAN_CTRH_BOM_HALT_PROGRAM       (3u << 5u)

#define  RX200_CAN_CTRH_ERRD                    DEF_BIT_07
#define  RX200_CAN_CTRH_CTME                    DEF_BIT_08

#define  RX200_CAN_CTRH_CTMS_STANDARD          (0u << 9u)
#define  RX200_CAN_CTRH_CTMS_LISTEN_ONLY       (1u << 9u)
#define  RX200_CAN_CTRH_CTMS_EXT_LOOP          (2u << 9u)
#define  RX200_CAN_CTRH_CTMS_INT_LOOP          (3u << 9u)

                                                                /* ----------------- STATUS REGISTER L ---------------- */
#define  RX200_CAN_STSL_CRSTSTS                 DEF_BIT_00
#define  RX200_CAN_STSL_CHLTSTS                 DEF_BIT_01
#define  RX200_CAN_STSL_CSLPSTS                 DEF_BIT_02
#define  RX200_CAN_STSL_EPSTS                   DEF_BIT_03
#define  RX200_CAN_STSL_BOSTS                   DEF_BIT_04
#define  RX200_CAN_STSL_TRMSTS                  DEF_BIT_05
#define  RX200_CAN_STSL_RECSTS                  DEF_BIT_06
#define  RX200_CAN_STSL_COMSTS                  DEF_BIT_07

                                                                /* --------------- ERROR FLAG REGISTER L -------------- */
#define  RX200_CAN_ERFLL_BEF                    DEF_BIT_00
#define  RX200_CAN_ERFLL_EWF                    DEF_BIT_01
#define  RX200_CAN_ERFLL_EPF                    DEF_BIT_02
#define  RX200_CAN_ERFLL_BOEF                   DEF_BIT_03
#define  RX200_CAN_ERFLL_BORF                   DEF_BIT_04
#define  RX200_CAN_ERFLL_OVLF                   DEF_BIT_05
#define  RX200_CAN_ERFLL_BLF                    DEF_BIT_06
#define  RX200_CAN_ERFLL_ALF                    DEF_BIT_07
#define  RX200_CAN_ERFLL_SERR                   DEF_BIT_08
#define  RX200_CAN_ERFLL_FERR                   DEF_BIT_09
#define  RX200_CAN_ERFLL_AERR                   DEF_BIT_10
#define  RX200_CAN_ERFLL_CERR                   DEF_BIT_11
#define  RX200_CAN_ERFLL_B1ERR                  DEF_BIT_12
#define  RX200_CAN_ERFLL_B0ERR                  DEF_BIT_13
#define  RX200_CAN_ERFLL_ADERR                  DEF_BIT_14

                                                                /* ---------- GLOBAL CONFIGURATION REGISTER L --------- */
#define  RX200_CAN_GCFGL_TPRI                   DEF_BIT_00
#define  RX200_CAN_GCFGL_DCE                    DEF_BIT_01
#define  RX200_CAN_GCFGL_DRE                    DEF_BIT_02
#define  RX200_CAN_GCFGL_MME                    DEF_BIT_03
#define  RX200_CAN_GCFGL_DCS                    DEF_BIT_04

#define  RX200_CAN_GCFGL_TSP_DIV_NONE          (0u  << 8u)
#define  RX200_CAN_GCFGL_TSP_DIV_2             (1u  << 8u)
#define  RX200_CAN_GCFGL_TSP_DIV_4             (2u  << 8u)
#define  RX200_CAN_GCFGL_TSP_DIV_8             (3u  << 8u)
#define  RX200_CAN_GCFGL_TSP_DIV_16            (4u  << 8u)
#define  RX200_CAN_GCFGL_TSP_DIV_32            (5u  << 8u)
#define  RX200_CAN_GCFGL_TSP_DIV_64            (6u  << 8u)
#define  RX200_CAN_GCFGL_TSP_DIV_128           (7u  << 8u)
#define  RX200_CAN_GCFGL_TSP_DIV_256           (8u  << 8u)
#define  RX200_CAN_GCFGL_TSP_DIV_512           (9u  << 8u)
#define  RX200_CAN_GCFGL_TSP_DIV_1024          (10u << 8u)
#define  RX200_CAN_GCFGL_TSP_DIV_2048          (11u << 8u)
#define  RX200_CAN_GCFGL_TSP_DIV_4096          (12u << 8u)
#define  RX200_CAN_GCFGL_TSP_DIV_8192          (13u << 8u)
#define  RX200_CAN_GCFGL_TSP_DIV_16384         (14u << 8u)
#define  RX200_CAN_GCFGL_TSP_DIV_32768         (15u << 8u)

#define  RX200_CAN_GCFGL_TSSS                   DEF_BIT_12

                                                                /* ------------- GLOBAL CONTROL REGISTER L ------------ */
#define  RX200_CAN_GCTRL_GMDC_OPERATING        (0u << 0u)
#define  RX200_CAN_GCTRL_GMDC_RESET            (1u << 0u)
#define  RX200_CAN_GCTRL_GMDC_TEST             (2u << 0u)
#define  RX200_CAN_GCTRL_GMDC_MASK              0x03u

#define  RX200_CAN_GCTRL_GSLPR                  DEF_BIT_02
#define  RX200_CAN_GCTRL_DEIE                   DEF_BIT_08
#define  RX200_CAN_GCTRL_MEIE                   DEF_BIT_09
#define  RX200_CAN_GCTRL_THLEIE                 DEF_BIT_10

                                                                /* ------------- GLOBAL CONTROL REGISTER H ------------ */
#define  RX200_CAN_GCTRH_TSRST                  DEF_BIT_00

                                                                /* -------------- GLOBAL STATUS REGISTER -------------- */
#define  RX200_CAN_GSTS_GRSTSTS                 DEF_BIT_00
#define  RX200_CAN_GSTS_GHLTSTS                 DEF_BIT_01
#define  RX200_CAN_GSTS_GSLPSTS                 DEF_BIT_02
#define  RX200_CAN_GSTS_GRAMINIT                DEF_BIT_03

                                                                /* ------------ GLOBAL ERROR FLAG REGISTER ------------ */
#define  RX200_CAN_GERFLL_DEF                   DEF_BIT_00
#define  RX200_CAN_GERFLL_MES                   DEF_BIT_01
#define  RX200_CAN_GERFLL_THLES                 DEF_BIT_02

                                                                /* ----------- GLOBAL TX INT STATUS REGISTER ---------- */
#define  RX200_CAN_GTINTSTS_TSIF0               DEF_BIT_00
#define  RX200_CAN_GTINTSTS_TAIF0               DEF_BIT_01
#define  RX200_CAN_GTINTSTS_CFTIF0              DEF_BIT_02
#define  RX200_CAN_GTINTSTS_THIF0               DEF_BIT_03

                                                                /* ------------- Rx BUFFER / FIFO REG xAH ------------- */
#define  RX200_CAN_RxIDHn_RMRTR                 DEF_BIT_14
#define  RX200_CAN_RxIDHn_RMIDE                 DEF_BIT_15

                                                                /* ---------- Rx FIFO & Tx/Rx FIFO CTRL REG x --------- */
#define  RX200_CAN_xFCCn_xFE                    DEF_BIT_00
#define  RX200_CAN_xFCCn_xFIE                   DEF_BIT_01
#define  RX200_CAN_xFCCn_CFTXIE                 DEF_BIT_02      /* Applies only to Tx/Rx FIFO Control Register 0L.      */

#define  RX200_CAN_xFCCn_xFDC_0_MSGs           (0u << 8u)
#define  RX200_CAN_xFCCn_xFDC_4_MSGs           (1u << 8u)
#define  RX200_CAN_xFCCn_xFDC_8_MSGs           (2u << 8u)
#define  RX200_CAN_xFCCn_xFDC_16_MSGs          (3u << 8u)

#define  RX200_CAN_xFCCn_xFIM                   DEF_BIT_12

#define  RX200_CAN_xFCCn_xFIGCV_1_8_FULL       (0u << 13u)
#define  RX200_CAN_xFCCn_xFIGCV_2_8_FULL       (1u << 13u)
#define  RX200_CAN_xFCCn_xFIGCV_3_8_FULL       (2u << 13u)
#define  RX200_CAN_xFCCn_xFIGCV_4_8_FULL       (3u << 13u)
#define  RX200_CAN_xFCCn_xFIGCV_5_8_FULL       (4u << 13u)
#define  RX200_CAN_xFCCn_xFIGCV_6_8_FULL       (5u << 13u)
#define  RX200_CAN_xFCCn_xFIGCV_7_8_FULL       (6u << 13u)
#define  RX200_CAN_xFCCn_xFIGCV_FULL           (7u << 13u)

                                                                /* ------------- Tx/Rx FIFO CONTROL REG 0H ------------ */
#define  RX200_CAN_CFCCH0_CFM_Rx               (0u << 0u)
#define  RX200_CAN_CFCCH0_CFM_Tx               (1u << 0u)

#define  RX200_CAN_CFCCH0_CFITSS                DEF_BIT_02
#define  RX200_CAN_CFCCH0_CFITR                 DEF_BIT_03

                                                                /* --------- Rx FIFO & Tx/Rx FIFO STATUS REG x -------- */
#define  RX200_CAN_xFSTSn_xFEMP                 DEF_BIT_00
#define  RX200_CAN_xFSTSn_xFFLL                 DEF_BIT_01
#define  RX200_CAN_xFSTSn_xFMLT                 DEF_BIT_02
#define  RX200_CAN_xFSTSn_xFIF                  DEF_BIT_03
#define  RX200_CAN_xFSTSn_CFTXIF                DEF_BIT_04      /* Applies only to Tx/Rx FIFO Status Register 0.        */

                                                                /* ------ Rx FIFO & Tx/Rx FIFO POINTER CTRL REG x ----- */
#define  RX200_CAN_xFPCTRn_xFPC_0xFFu           0xFFu

                                                                /* ------------ Rx FIFO MSG LOST STATUS REG ----------- */
#define  RX200_CAN_RFMSTS_RF0MLT                DEF_BIT_00
#define  RX200_CAN_RFMSTS_RF1MLT                DEF_BIT_01

                                                                /* ---------- Tx/Rx FIFO MSG LOST STATUS REG ---------- */
#define  RX200_CAN_CFMSTS_CF0MLT                DEF_BIT_00

                                                                /* --------------- Rx FIFO INT STATUS REG ------------- */
#define  RX200_CAN_RFISTS_RF0IF                 DEF_BIT_00
#define  RX200_CAN_RFISTS_RF1IF                 DEF_BIT_01

                                                                /* ------------- Tx/Rx FIFO INT STATUS REG ------------ */
#define  RX200_CAN_CFISTS_CF0IF                 DEF_BIT_00

                                                                /* --------- TRANSMIT BUFFER CONTROL REGISTER --------- */
#define  RX200_CAN_TMCx_TMTR                    DEF_BIT_00
#define  RX200_CAN_TMCx_TMTAR                   DEF_BIT_01
#define  RX200_CAN_TMCx_TMOM                    DEF_BIT_02

                                                                /* ---------- TRANSMIT BUFFER STATUS REGISTER --------- */
#define  RX200_CAN_TMSTSx_TMTSTS                DEF_BIT_00

#define  RX200_CAN_TMSTSx_TMTRF_Tx_PROG        (0u << 1u)
#define  RX200_CAN_TMSTSx_TMTRF_Tx_COMP        (1u << 1u)
#define  RX200_CAN_TMSTSx_TMTRF_Tx_NO_ABRT     (2u << 1u)
#define  RX200_CAN_TMSTSx_TMTRF_Tx_ABRT        (3u << 1u)

#define  RX200_CAN_TMSTSx_TMTRM                 DEF_BIT_03
#define  RX200_CAN_TMSTSx_TMTARM                DEF_BIT_04

                                                                /* ----- Tx BUFF Tx REQUEST/COMP/ABORT STATUS REG ----- */
#define  RX200_CAN_TMTxSTS_TMTxSTS0             DEF_BIT_00
#define  RX200_CAN_TMTxSTS_TMTxSTS1             DEF_BIT_01
#define  RX200_CAN_TMTxSTS_TMTxSTS2             DEF_BIT_02
#define  RX200_CAN_TMTxSTS_TMTxSTS3             DEF_BIT_03

                                                                /* ---------- TRANSMIT BUFFER INT ENABLE REG ---------- */
#define  RX200_CAN_TMIEC_TMIE0                  DEF_BIT_00
#define  RX200_CAN_TMIEC_TMIE1                  DEF_BIT_01
#define  RX200_CAN_TMIEC_TMIE2                  DEF_BIT_02
#define  RX200_CAN_TMIEC_TMIE3                  DEF_BIT_03

                                                                /* -------- Tx/Rx FIFO & Tx BUFFER REGISTER xAH ------- */
#define  RX200_CAN_TxIDHn_THELN                 DEF_BIT_13
#define  RX200_CAN_TxIDHn_TMRTR                 DEF_BIT_14
#define  RX200_CAN_TxIDHn_TMIDE                 DEF_BIT_15

                                                                /* ----------- Tx HISTORY BUFFER CONTROL REG ---------- */
#define  RX200_CAN_THLCC0_THLE                  DEF_BIT_00
#define  RX200_CAN_THLCC0_THLIE                 DEF_BIT_08
#define  RX200_CAN_THLCC0_THLIM                 DEF_BIT_09
#define  RX200_CAN_THLCC0_THLDTE                DEF_BIT_10

                                                                /* ----------- Tx HISTORY BUFFER STATUS REG ----------- */
#define  RX200_CAN_THLSTS0_THLEMP               DEF_BIT_00
#define  RX200_CAN_THLSTS0_THLFLL               DEF_BIT_01
#define  RX200_CAN_THLSTS0_THLELT               DEF_BIT_02
#define  RX200_CAN_THLSTS0_THLIF                DEF_BIT_03

                                                                /* ----------- Tx HISTORY BUFFER ACCESS REG ----------- */
#define  RX200_CAN_THLACC0_BT_Tx                DEF_BIT_00
#define  RX200_CAN_THLACC0_BT_Tx_FIFO           DEF_BIT_01

                                                                /* --------- Tx HISTORY BUFF POINTER CTRL REG --------- */
#define  RX200_CAN_THLPCTR0_THLPC_0xFFu         0xFFu

                                                                /* ----------- GLOBAL RAM WINDOW CONTROL REG ---------- */
#define  RX200_CAN_GRWCR_RPAGE                  DEF_BIT_00

                                                                /* ----------- GLOBAL TEST CONTROL REGISTER ----------- */
#define  RX200_CAN_GTSTCTRL_RTME                DEF_BIT_02

                                                                /* --------- GLOBAL TEST PROTECTION UNLOCK REG -------- */
#define  RX200_CAN_GLOCKK_KEY1                  0x7575u
#define  RX200_CAN_GLOCKK_KEY2                  0x8A8Au

                                                                /* ------------ RECEIVE RULE ENTRY REG xAH ------------ */
#define  RX200_CAN_GAFLIDHx_GAFLLB              DEF_BIT_13
#define  RX200_CAN_GAFLIDHx_GAFLRTR             DEF_BIT_14
#define  RX200_CAN_GAFLIDHx_GAFLIDE             DEF_BIT_15

                                                                /* ------------ RECEIVE RULE ENTRY REG xBH ------------ */
#define  RX200_CAN_GAFLMHx_GAFLRTRM             DEF_BIT_14
#define  RX200_CAN_GAFLMHx_GAFLIDEM             DEF_BIT_15

                                                                /* ------------ RECEIVE RULE ENTRY REG xCL ------------ */
#define  RX200_CAN_GAFLPLx_GAFLFDP0             DEF_BIT_00
#define  RX200_CAN_GAFLPLx_GAFLFDP1             DEF_BIT_01
#define  RX200_CAN_GAFLPLx_GAFLFDP4             DEF_BIT_04
#define  RX200_CAN_GAFLPLx_GAFLRMV              DEF_BIT_15


/*
*********************************************************************************************************
*                                               MACROS
*********************************************************************************************************
*/
                                                                /*               WINDOW SELECT : WINDOW 0               */
                                                                /* ---------------------------------------------------- */
                                                                /* -------------- RECEIVE RULE CONFIG REG ------------- */
#define  RX200_CAN_GAFLCFG_RNC0(x)            ((x) & 0x1Fu)     /* Set the Number of Receive Rules of CAN Channel.      */

                                                                /* ---------- RECEIVE RULE ENTRY x ID & MASK ---------- */
#define  RX200_CAN_GAFLIxLn_GAFLIDx_STD(x)    ((x) & 0x07FFu)   /* Standard ID ONLY.  ID (xAL) & Mask (xBL) Register.   */
#define  RX200_CAN_GAFLIxLn_GAFLIDx_EXT(x)    ((x) & 0xFFFFu)   /* Extended ID START. ID (xAL) & Mask (xBL) Register.   */
                                                                /* Extended ID CONT.  ID (xAL) & Mask (xBL) Register.   */
#define  RX200_CAN_GAFLIxHn_GAFLIDx(x)       (((x) >> 16u) & 0x1FFFu)

                                                                /* ------------ RECEIVE RULE ENTRY REG xCL ------------ */
                                                                /* Set the Rx Buffer # to Store the Rx'd Messages.      */
#define  RX200_CAN_GAFLPLx_GAFLRMDP(x)       (((x) & 0x7Fu) << 8u)

                                                                /* ------------- RECEIVE RULE ENTRY x DLC ------------- */
#define  RX200_CAN_GAFLPHx_GAFLPTR(x)         ((x) & 0x0FFFu)   /* Receive Rule Set Label Data.                         */
                                                                /* Receive Rule DLC Size Check.                         */
#define  RX200_CAN_GAFLPHx_GAFLDLC(x)        (((x) & 0x0Fu) << 12u)

                                                                /*               WINDOW SELECT : WINDOW 1               */
                                                                /* ---------------------------------------------------- */
                                                                /* ----------- BIT CONFIGURATION REGISTER L ----------- */
#define  RX200_CAN_CFGL_BRP(x)                ((x) & 0x03FFu)   /* Set Prescaler Division Ratio.                        */

                                                                /* ----------- BIT CONFIGURATION REGISTER H ----------- */
                                                                /* Set Time Segment 1 Control. Based on Reg. Defines.   */
#define  RX200_CAN_CFGH_TSEG1(x)             (((x) - 1u) & 0x0Fu)
                                                                /* Set Time Segment 2 Control. Based on Reg. Defines.   */
#define  RX200_CAN_CFGH_TSEG2(x)            ((((x) - 1u) & 0x07u) << 4u)
                                                                /* Set Resynchronization Jump Width Control. Reg Defs.  */
#define  RX200_CAN_CFGH_SJW(x)              ((((x) - 1u) & 0x03u) << 8u)

                                                                /* ----------------- STATUS REGISTER H ---------------- */
#define  RX200_CAN_STSH_REC(x)                ((x) & 0xFFu)     /* Read Receive  Error Counter.                         */
                                                                /* Read Transmit Error Counter.                         */
#define  RX200_CAN_STSH_TEC(x)               (((x) >> 8u) & 0xFFu)

                                                                /* --------------- ERROR FLAG REGISTER H -------------- */
                                                                /* Read RCR Value based on Tx'd Message.                */
#define  RX200_CAN_ERFLH_CRCREG               ((x) & 0x7FFFFFFFu)

                                                                /* --------- Rx BUFFER NUMBER CONFIG REGISTER --------- */
#define  RX200_CAN_RMNB_NRXMB(x)              ((x) & 0x1Fu)     /* Set the Number of Rx Buffers. Value [0 -> 16].       */

                                                                /* ------------- Rx BUFFER / FIFO REG x ID ------------ */
#define  RX200_CAN_RxIDLn_RMID_STD(x)         ((x) & 0x07FFu)   /* Standard ID ONLY.  Rx Buffer / FIFO ID Data L Reg.   */
#define  RX200_CAN_RxIDLn_RMID_EXT(x)         ((x) & 0xFFFFu)   /* Extended ID START. Rx Buffer / FIFO ID Data L Reg.   */
#define  RX200_CAN_RxIDHn_RMID(x)             ((x) & 0x1FFFu)   /* Extended ID CONT.  Rx Buffer / FIFO ID Data L Reg.   */

                                                                /* ------------ Rx BUFFER / FIFO REG x DLC ------------ */
#define  RX200_CAN_RxPTRn_RMPTR(x)            ((x) & 0x0FFFu)   /* Receive Buffer Label Data. Label Info of Rx'd Msg.   */
                                                                /* Get Rx'd Buffer DLC Size.                            */
#define  RX200_CAN_RxPTRn_RMDLC(x)           (((x) >> 12u) & 0x0Fu)

                                                                /* ------------ Rx BUFFER / FIFO REG x DATA ----------- */
#define  RX200_CAN_RxDFnX_DATA_L(x)           ((x) & 0xFFu)     /* Rx Data From RMDFnX & RFDFnX Regs. Lower -Byte Data. */
                                                                /* Rx Data From RMDFnX & RFDFnX Regs. Higher-Byte Data. */
#define  RX200_CAN_RxDFnX_DATA_H(x)          (((x) >> 8u) & 0xFFu)

                                                                /* ------------- Rx FIFO STATUS REGISTER x ------------ */
                                                                /* Rx FIFO Number of Unread Messages.                   */
#define  RX200_CAN_RFSTSx_RFMC(x)            (((x) >> 8u) & 0x3Fu)

                                                                /* ------------- Tx/Rx FIFO CONTROL REG 0H ------------ */
                                                                /* Set Tx Buffer Number Linked to Tx/Rx FIFO Buffer.    */
#define  RX200_CAN_CFCCH0_CFTML(x)           (((x) & 0x03u) << 4u)
                                                                /* Set Message Transmission Interval.                   */
#define  RX200_CAN_CFCCH0_CFITT(x)           (((x) & 0xFFu) << 8u)

                                                                /* ------------- Tx/Rx FIFO STATUS REG 0H ------------- */
#define  RX200_CAN_CFSTS0_CFMC(x)            (((x) >> 8u) & 0x3Fu)

                                                                /* ------------ Tx/Rx FIFO & TX BUFFER x ID ----------- */
#define  RX200_CAN_TxIDLn_CFID_STD(x)         ((x) & 0x07FFu)   /* Standard ID ONLY.  Tx/Rx FIFO & Tx BUFFER xL Reg.    */
#define  RX200_CAN_TxIDLn_CFID_EXT(x)         ((x) & 0xFFFFu)   /* Extended ID START. Tx/Rx FIFO & Tx BUFFER xL Reg.    */
                                                                /* Extended ID CONT.  Tx/Rx FIFO & Tx BUFFER xL Reg.    */
#define  RX200_CAN_TxIDHn_CFID(x)            (((x) >> 16u) & 0x1FFFu)

                                                                /* ----------- Tx/Rx FIFO & TX BUFFER x DLC ----------- */
#define  RX200_CAN_TxPTRn_TxPTR(x)            ((x) & 0x0FFFu)   /* Transmit Buffer Label Data. Label Info of Tx'd Msg.  */
                                                                /* Set Tx'd Buffer DLC Size.                            */
#define  RX200_CAN_TxPTRn_TxDLC(x)           (((x) & 0x0Fu) << 12u)

                                                                /* ----------- Tx/Rx FIFO & TX BUFFER x DATA ---------- */
#define  RX200_CAN_TxDF0n_DATA_L(x)           ((x) & 0xFFu)     /* Tx Data To CFDF0,1,2,3 Register. Lower -Byte Data.   */
                                                                /* Tx Data To CFDF0,1,2,3 Register. Higher-Byte Data.   */
#define  RX200_CAN_TxDF0n_DATA_H(x)          (((x) & 0xFFu) << 8u)

                                                                /* ----------- Tx HISTORY BUFFER STATUS REG ----------- */
                                                                /* Tx History Buffer Unread Data Sets.                  */
#define  RX200_CAN_THLSTS0_THLMC(x)          (((x) >> 8u) & 0x0Fu)

                                                                /* ----------- Tx HISTORY BUFFER ACCESS REG ----------- */
                                                                /* Buffer Number of Tx Source.                          */
#define  RX200_CAN_THLACC0_BN(x)             (((x) >> 3u) & 0x03u)
                                                                /* Label Information of Tx Stored Data.                 */
#define  RX200_CAN_THLACC0_TID(x)            (((x) >> 8u) & 0xFFu)


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
                                                                /* -------------- RX200 CAN REGISTER MAP -------------- */
typedef  volatile  struct  rx200_can_reg {                      /* ------------ CONTROL & STATUS REGISTERs ------------ */
    CPU_REG16  CFGL;                                            /* Bit Configuration    Register L.                     */
    CPU_REG16  CFGH;                                            /* Bit Configuration    Register  H.                    */
    CPU_REG16  CTRL;                                            /* Control              Register L.                     */
    CPU_REG16  CTRH;                                            /* Control              Register  H.                    */
    CPU_REG16  STSL;                                            /* Status               Register L.                     */
    CPU_REG16  STSH;                                            /* Status               Register  H.                    */
    CPU_REG16  ERFLL;                                           /* Error Flag           Register L.                     */
    CPU_REG16  ERFLH;                                           /* Error Flag           Register  H.                    */
    CPU_REG16  RSVD_00[9u];                                     /* Reserved.    [0x000A 8310 -> 0x000A 8320]            */
    CPU_REG16  GCFGL;                                           /* Global Configuration Register L.                     */
    CPU_REG16  GCFGH;                                           /* Global Configuration Register  H.                    */
    CPU_REG16  GCTRL;                                           /* Global Control       Register L.                     */
    CPU_REG16  GCTRH;                                           /* Global Control       Register  H.                    */
    CPU_REG16  GSTS;                                            /* Global Status        Register.                       */
    CPU_REG08  GERFLL;                                          /* Global Error Flag    Register.                       */
    CPU_REG08  RSVD_01;                                         /* Reserved.    [0x000A 832D]                           */
    CPU_REG16  GTSC;                                            /* Timestamp            Register.                       */
    CPU_REG16  GAFLCFG;                                         /* Receive Rule   Number Configuration  Register.       */
    CPU_REG16  RMNB;                                            /* Receive Buffer Number Configuration  Register.       */
    CPU_REG16  RMND0;                                           /* Receive Buffer Receive Complete Flag Register.       */
    CPU_REG16  RSVD_02;                                         /* Reserved.    [0x000A 8336]                           */
    CPU_REG16  RFCC0;                                           /* Receive FIFO Control Register 0.                     */
    CPU_REG16  RFCC1;                                           /* Receive FIFO Control Register 1.                     */
    CPU_REG16  RSVD_03[2u];                                     /* Reserved.                                            */
    CPU_REG16  RFSTS0;                                          /* Receive FIFO Status  Register 0.                     */
    CPU_REG16  RFSTS1;                                          /* Receive FIFO Status  Register 1.                     */
    CPU_REG16  RSVD_04[2u];                                     /* Reserved.    [0x000A 8344 -> 0x000A 8346]            */
    CPU_REG16  RFPCTR0;                                         /* Receive FIFO Pointer Control Register 0.             */
    CPU_REG16  RFPCTR1;                                         /* Receive FIFO Pointer Control Register 1.             */
    CPU_REG16  RSVD_05[2u];                                     /* Reserved.    [0x000A 834C -> 0x000A 834E]            */
    CPU_REG16  CFCCL0;                                          /* Transmit / Receive FIFO Control Reigster 0 L.        */
    CPU_REG16  CFCCH0;                                          /* Transmit / Receive FIFO Control Reigster 0  H.       */
    CPU_REG16  RSVD_06[2u];                                     /* Reserved.    [0x000A 8354 -> 0x000A 8356]            */
    CPU_REG16  CFSTS0;                                          /* Transmit / Receive FIFO Status  Register 0.          */
    CPU_REG16  RSVD_07;                                         /* Reserved.    [0x000A 835A]                           */
    CPU_REG16  CFPCTR0;                                         /* Transmit / Receive FIFO Pointer Control Register 0.  */
    CPU_REG16  RSVD_08;                                         /* Reserved.    [0x000A 835E]                           */
    CPU_REG08  RFMSTS;                                          /* Receive FIFO Message Lost Status Register.           */
    CPU_REG08  CFMSTS;                                          /* Transmit / Receive FIFO Message Lost Register.       */
    CPU_REG08  RFISTS;                                          /* Receive FIFO Interrupt Status Register.              */
    CPU_REG08  CFISTS;                                          /* Transmit / Receive FIFO Rx Interrupt Status Register.*/
    CPU_REG08  TMCx[4u];                                        /* Transmit Buffer Control Register(s) : 0, 1, 2, 3.    */
    CPU_REG08  RSVD_09[4u];                                     /* Reserved.    [0x000A 8368 -> 0x000A 836B]            */
    CPU_REG08  TMSTSx[4u];                                      /* Transmit Buffer Status  Register(s) : 0, 1, 2, 3.    */
    CPU_REG08  RSVD_10[4u];                                     /* Reserved.    [0x000A 8370 -> 0x000A 8373]            */
    CPU_REG16  TMTRSTS;                                         /* Transmit Buffer Transmit Request  Status Register.   */
    CPU_REG16  TMTCSTS;                                         /* Transmit Buffer Transmit Complete Status Register.   */
    CPU_REG16  TMTASTS;                                         /* Transmit Buffer Transmit Abort    Status Register.   */
    CPU_REG16  TMIEC;                                           /* Transmit Buffer Interrupt Enable         Register.   */
    CPU_REG16  THLCC0;                                          /* Transmit History Buffer Control          Register.   */
    CPU_REG16  RSVD_11;                                         /* Reserved.    [0x000A 837C]                           */
    CPU_REG16  THLSTS0;                                         /* Transmit History Buffer Status           Register.   */
    CPU_REG16  RSVD_12;                                         /* Reserved.    [0x000A 8382]                           */
    CPU_REG16  THLPCTR0;                                        /* Transmit History Buffer Pointer Control  Register.   */
    CPU_REG16  RSVD_13;                                         /* Reserved.    [0x000A 8386]                           */
    CPU_REG16  GTINTSTS;                                        /* Global Transmit Interrupt Status Register.           */
    CPU_REG16  GRWCR;                                           /* Global RAM Window Control        Register.           */
    CPU_REG16  GTSTCFG;                                         /* Global Test Configuration        Register.           */
    CPU_REG16  GTSTCTRL;                                        /* Global Test Control              Register.           */
    CPU_REG16  RSVD_14[2u];                                     /* Reserved.    [0x000A 8390 -> 0x000A 8392]            */
    CPU_REG16  GLOCKK;                                          /* Global Test Protection Unlock    Register.           */
    CPU_REG16  RSVD_15[5u];                                     /* Reserved.    [0x000A 8396 -> 0x000A 839E]            */
    struct {                                                    /* ------------- RECEIVE BUFFER REGISTERs ------------- */
        CPU_REG16  RMIDLx;                                      /* Receive Buffer Register xAL. [ID Register]           */
        CPU_REG16  RMIDHx;                                      /* Receive Buffer Register xAH. [ID Register]           */
        CPU_REG16  RMTSx;                                       /* Receive Buffer Register xBL. [Timestamp Register]    */
        CPU_REG16  RMPTRx;                                      /* Receive Buffer Register xBH. [DLC Register]          */
        CPU_REG16  RMDF0x;                                      /* Receive Buffer Register xCL. [Data 0 & 1 Register]   */
        CPU_REG16  RMDF1x;                                      /* Receive Buffer Register xCH. [Data 2 & 3 Register]   */
        CPU_REG16  RMDF2x;                                      /* Receive Buffer Register xDL. [Data 4 & 5 Register]   */
        CPU_REG16  RMDF3x;                                      /* Receive Buffer Register xDH. [Data 6 & 7 Register]   */
    } RX[16u];
    CPU_REG16  RSVD_16[128u];                                   /* Reserved.    [0x000A 84A0 -> 0x000A 859E]            */
    struct {                                                    /* ----------- RECEIVE FIFO ACCESS REGISTERs ---------- */
        CPU_REG16  RFIDLx;                                      /* Receive FIFO Access Register xAL. [ID Register]      */
        CPU_REG16  RFIDHx;                                      /* Receive FIFO Access Register xAH. [ID Register]      */
        CPU_REG16  RFTSx;                                       /* Receive FIFO Access Register xBL. [Timestamp Reg]    */
        CPU_REG16  RFPTRx;                                      /* Receive FIFO Access Register xBH. [DLC Register]     */
        CPU_REG16  RFDF0x;                                      /* Receive FIFO Access Register xCL. [Data 0 & 1 Reg]   */
        CPU_REG16  RFDF1x;                                      /* Receive FIFO Access Register xCH. [Data 2 & 3 Reg]   */
        CPU_REG16  RFDF2x;                                      /* Receive FIFO Access Register xDL. [Data 4 & 5 Reg]   */
        CPU_REG16  RFDF3x;                                      /* Receive FIFO Access Register xDH. [Data 6 & 7 Reg]   */
    } RX_FIFO[2u];
    CPU_REG16  RSVD_17[16u];                                    /* Reserved.    [0x000A 85C0 -> 0x000A 85DE]            */
                                                                /* ----------- Tx / Rx FIFO ACCESS REGISTERs ---------- */
    CPU_REG16  CFIDL0;                                          /* Tx / Rx FIFO Access Register 0AL. [ID Register]      */
    CPU_REG16  CFIDH0;                                          /* Tx / Rx FIFO Access Register 0AH. [ID Register]      */
    CPU_REG16  CFTS0;                                           /* Tx / Rx FIFO Access Register 0BL. [Timestamp Reg]    */
    CPU_REG16  CFPTR0;                                          /* Tx / Rx FIFO Access Register 0BH. [DLC Register]     */
    CPU_REG16  CFDF00;                                          /* Tx / Rx FIFO Access Register 0CL. [Data 0 & 1 Reg]   */
    CPU_REG16  CFDF10;                                          /* Tx / Rx FIFO Access Register 0CH. [Data 2 & 3 Reg]   */
    CPU_REG16  CFDF20;                                          /* Tx / Rx FIFO Access Register 0DL. [Data 4 & 5 Reg]   */
    CPU_REG16  CFDF30;                                          /* Tx / Rx FIFO Access Register 0DH. [Data 6 & 7 Reg]   */
    CPU_REG16  RSVD_18[8u];                                     /* Reserved.    [0x000A 85F0 -> 0x000A 85FE]            */
    struct {                                                    /* ------------- TRANSMIT BUFFER REGISTERs ------------ */
        CPU_REG16  TMIDLx;                                      /* Transmit Buffer Register xAL. [ID Register]          */
        CPU_REG16  TMIDHx;                                      /* Transmit Buffer Register xAH. [ID Register]          */
        CPU_REG16  RSVD_19;                                     /* Reserved     [0x000A 00x4]    [No Timestamp]         */
        CPU_REG16  TMPTRx;                                      /* Transmit Buffer Register xBH. [DLC Register]         */
        CPU_REG16  TMDF0x;                                      /* Transmit Buffer Register xCL. [Data 0 & 1 Register]  */
        CPU_REG16  TMDF1x;                                      /* Transmit Buffer Register xCH. [Data 2 & 3 Register]  */
        CPU_REG16  TMDF2x;                                      /* Transmit Buffer Register xDL. [Data 4 & 5 Register]  */
        CPU_REG16  TMDF3x;                                      /* Transmit Buffer Register xDH. [Data 6 & 7 Register]  */
    } TX[4u];
    CPU_REG16  RSVD_20[32u];                                    /* Reserved.    [0x000A 8640 -> 0x000A xxxx]            */
    CPU_REG16  THLACC0;                                         /* Transmit History Buffer Access Register.             */
} RX200_CAN_REG;


typedef  volatile  struct  rx200_can_rr_reg {                   /* -------- RX200 CAN RECEIVE RULE REGISTER MAP ------- */
    struct {
    CPU_REG16  GAFLIDLx;                                        /* Receive Rule Entry Register xAL. [ID     Register]   */
    CPU_REG16  GAFLIDHx;                                        /* Receive Rule Entry Register xAH. [ID     Register]   */
    CPU_REG16  GAFLMLx;                                         /* Receive Rule Entry Register xBL. [Mask   Register]   */
    CPU_REG16  GAFLMHx;                                         /* Receive Rule Entry Register xBH. [Mask   Register]   */
    CPU_REG16  GAFLPLx;                                         /* Receive Rule Entry Register xCL. [Config Register]   */
    CPU_REG16  GAFLPHx;                                         /* Receive Rule Entry Register xCH. [Config Register]   */
    } RX_RULE[16u];
} RX200_CAN_RR_REG;


/*
*********************************************************************************************************
*                                        CAN BAUDRATE REGISTER
*
* Description : Structure defines the CAN BaudRate Register Structure.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  struct  rx200_can_baud {    
    CPU_INT32U  BaudRate;                                       /* Holds the Baudrate.                                  */
    CPU_INT32U  SamplePoint;                                    /* Holds the Sample point in 0.1%                       */
    CPU_INT32U  ReSynchJumpWidth;                               /* Holds the Re-Synchronization Jump Width in 0.1%      */
    CPU_INT08U  PrescalerDiv;                                   /* Holds the Prescaler Divide Factor                    */
    CPU_INT08U  SJW;                                            /* Holds the Re-Synch Jump Width         (StdValue = 1) */
    CPU_INT08U  PropagationSeg;                                 /* Holds the Propagation Segment Time    (StdValue = 2) */
    CPU_INT08U  PhaseBufSeg1;                                   /* Holds the Phase Buffer Segment 1      (StdValue = 7) */
    CPU_INT08U  PhaseBufSeg2;                                   /* Holds the Phase Buffer Segment 2      (StdValue = 7) */
} RX200_CAN_BAUD;


/*
*********************************************************************************************************
*                                              MODULE END
*********************************************************************************************************
*/

#endif
