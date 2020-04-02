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
*                                       Freescale iMX6 Series
*
* Filename : drv_can_reg.h
* Version  : V2.42.01
*********************************************************************************************************
* Note(s)  : (1) This CAN Driver supports the following Series/Families:
*                    iMX6 Series - i.MX6 Quad
*                                - i.MX6 Dual
*                                - i.MX6 DualLite
*                                - i.MX6 Solo
*
*                Set by the Technical Reference Manual(s) obtained from the Freescale website. This driver
*                has been tested with or should work with the families mentioned above.
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

#ifndef  CAN_MODULE_CHANNEL_1
#define  CAN_MODULE_CHANNEL_1                   DEF_DISABLED
#endif

#ifndef  CAN_MODULE_CHANNEL_2
#define  CAN_MODULE_CHANNEL_2                   DEF_DISABLED
#endif


/*
*********************************************************************************************************
*                                          ADDRESS REGISTERS
*********************************************************************************************************
*/
                                                                /* ------------------- BASE ADDRESS ------------------- */
#define  iMX6_CAN1_ADDR                         0x02090000u
#define  iMX6_CAN2_ADDR                         0x02094000u

                                                                /* ------------------ Rx FIFO OFFSETs ----------------- */
#define  iMX6_CAN_MB_OFFSET                           0x80u
#define  iMX6_CAN_RX_MB_FILTER_ID_OFFSET              0xE0u


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/
                                                                /* ---------------- MAILBOX SETTINGS ------------------ */
#define  iMX6_CAN_RX_FIFO_LIMIT                          6u     /* Number of Rx FIFO Mailboxes / Filters.               */
#define  iMX6_CAN_RX_FIFO_ID_FILTER_MIN_LIMIT            8u     /* Min Number of ID Filters for Rx FIFO Engine.         */
#define  iMX6_CAN_TX_MB_LIMIT                            6u     /* Number of Tx Mailboxes.                              */
#define  iMX6_CAN_MB_TOTAL                     (iMX6_CAN_RX_FIFO_LIMIT + iMX6_CAN_TX_MB_LIMIT)

                                                                /* ---------------- TX MSG BUFFER CODEs --------------- */
#define  iMX6_CAN_TX_CODE_INACTIVE                    0x08u     /* Tx Mailbox is Inactive, Not Active in Arbitration.   */
#define  iMX6_CAN_TX_CODE_ABORT                       0x09u     /* Abort Tx Mailbox in  Tx. Not Active in Arbitration.  */
#define  iMX6_CAN_TX_CODE_DATA                        0x0Cu     /* Tx Data Frame Once. Places MB in Arbitration.        */
#define  iMX6_CAN_TX_CODE_REMOTE                      0x0Cu     /* Tx Remote Request Frame. MB becomes Rx Empty After.  */
#define  iMX6_CAN_TX_CODE_TANSWER                     0x0Eu     /* Intermediate Code Auto written to MB as RTR Match.   */

                                                                /* --------------- INTERRUPT MASK & FLAG -------------- */
#define  iMX6_CAN_IMASKn_IFLAGn_LIMIT                   32u     /* Max Bit Qty that separates Int Mask/Flag Reg 1 & 2.  */

#define  iMX6_CAN_SID_MASK                          0x07FFu
#define  iMX6_CAN_EID_MASK                        0x03FFFFu

                                                                /* ------------------- CONTROL 1 REG ------------------ */
#define  iMX6_CAN_CTRL1_TIMING_MASK             0xFFFF0007u     /* Mask encompases PRESDIV, RJW, PSEG1 & 2, and PROP_SEG*/

/*
*********************************************************************************************************
*                                           BIT DEFINITIONS
*********************************************************************************************************
*/
                                                                /* ------------------ MODE CONFIG REG ----------------- */
#define  iMX6_CAN_MCR_MDIS                      DEF_BIT_31
#define  iMX6_CAN_MCR_FRZ                       DEF_BIT_30
#define  iMX6_CAN_MCR_RFEN                      DEF_BIT_29
#define  iMX6_CAN_MCR_HALT                      DEF_BIT_28
#define  iMX6_CAN_MCR_NOT_RDY                   DEF_BIT_27
#define  iMX6_CAN_MCR_WAK_MSK                   DEF_BIT_26
#define  iMX6_CAN_MCR_SOFT_RST                  DEF_BIT_25
#define  iMX6_CAN_MCR_FRZ_ACK                   DEF_BIT_24
#define  iMX6_CAN_MCR_SUPV                      DEF_BIT_23
#define  iMX6_CAN_MCR_SLF_WAK                   DEF_BIT_22
#define  iMX6_CAN_MCR_WRN_EN                    DEF_BIT_21
#define  iMX6_CAN_MCR_LMP_ACK                   DEF_BIT_20
#define  iMX6_CAN_MCR_WAK_SRC                   DEF_BIT_19
#define  iMX6_CAN_MCR_SRX_DIS                   DEF_BIT_17
#define  iMX6_CAN_MCR_IRMQ                      DEF_BIT_16
#define  iMX6_CAN_MCR_LPRIO_EN                  DEF_BIT_13
#define  iMX6_CAN_MCR_AEN                       DEF_BIT_12

#define  iMX6_CAN_MCR_IDAM_FMT_A               (0u << 8u)
#define  iMX6_CAN_MCR_IDAM_FMT_B               (1u << 8u)
#define  iMX6_CAN_MCR_IDAM_FMT_C               (2u << 8u)
#define  iMX6_CAN_MCR_IDAM_FMT_D               (3u << 8u)

                                                                /* ------------------- CONTROL 1 REG ------------------ */
#define  iMX6_CAN_CTRL1_BOFF_MSK                DEF_BIT_15
#define  iMX6_CAN_CTRL1_ERR_MSK                 DEF_BIT_14
#define  iMX6_CAN_CTRL1_LPB                     DEF_BIT_12
#define  iMX6_CAN_CTRL1_TWRN_MSK                DEF_BIT_11
#define  iMX6_CAN_CTRL1_RWRN_MSK                DEF_BIT_10
#define  iMX6_CAN_CTRL1_SMP                     DEF_BIT_07
#define  iMX6_CAN_CTRL1_BOFF_REC                DEF_BIT_06
#define  iMX6_CAN_CTRL1_TSYN                    DEF_BIT_05
#define  iMX6_CAN_CTRL1_LBUF                    DEF_BIT_04
#define  iMX6_CAN_CTRL1_LOM                     DEF_BIT_03

                                                                /* --------------- ERROR & STATUS 1 REG --------------- */
#define  iMX6_CAN_ESR1_SYNCH                    DEF_BIT_18
#define  iMX6_CAN_ESR1_TWRN_INT                 DEF_BIT_17
#define  iMX6_CAN_ESR1_RWRN_INT                 DEF_BIT_16
#define  iMX6_CAN_ESR1_BIT1_ERR                 DEF_BIT_15
#define  iMX6_CAN_ESR1_BIT0_ERR                 DEF_BIT_14
#define  iMX6_CAN_ESR1_ACK_ERR                  DEF_BIT_13
#define  iMX6_CAN_ESR1_CRC_ERR                  DEF_BIT_12
#define  iMX6_CAN_ESR1_FRM_ERR                  DEF_BIT_11
#define  iMX6_CAN_ESR1_STF_ERR                  DEF_BIT_10
#define  iMX6_CAN_ESR1_TX_WRN                   DEF_BIT_09
#define  iMX6_CAN_ESR1_RX_WRN                   DEF_BIT_08
#define  iMX6_CAN_ESR1_IDLE                     DEF_BIT_07
#define  iMX6_CAN_ESR1_TX                       DEF_BIT_06

#define  iMX6_CAN_ESR1_FLT_CONF_ERR_ACT        (0u << 4u)
#define  iMX6_CAN_ESR1_FLT_CONF_ERR_PASS       (1u << 4u)
#define  iMX6_CAN_ESR1_FLT_CONF_BUS_OFF        (3u << 4u)

#define  iMX6_CAN_ESR1_RX                       DEF_BIT_03
#define  iMX6_CAN_ESR1_BOFF_INT                 DEF_BIT_02
#define  iMX6_CAN_ESR1_ERR_INT                  DEF_BIT_01
#define  iMX6_CAN_ESR1_WAK_INT                  DEF_BIT_00

                                                                /* --------------- INTERRUPT FLAG 1 REG --------------- */
#define  iMX6_CAN_IFLAG1_RX_FIFO_OVRFLW         DEF_BIT_07      /* In Rx FIFO Mode: Sets FIFO Overflow Flag.            */
#define  iMX6_CAN_IFLAG1_RX_FIFO_ALMOST_FULL    DEF_BIT_06      /*                  Sets FIFO Buff is Almost Full Flag. */
#define  iMX6_CAN_IFLAG1_RX_FIFO_RD_RDY         DEF_BIT_05      /*                  Sets least 1 Frm is Available to Rd.*/

                                                                /* ------------------- CONTROL 2 REG ------------------ */
#define  iMX6_CAN_CTRL2_WRMFRZ                  DEF_BIT_28
#define  iMX6_CAN_CTRL2_MRP                     DEF_BIT_18
#define  iMX6_CAN_CTRL2_RRS                     DEF_BIT_17
#define  iMX6_CAN_CTRL2_EACEN                   DEF_BIT_16

                                                                /* --------------- ERROR & STATUS 2 REG --------------- */
#define  iMX6_CAN_ESR2_VPS                      DEF_BIT_14
#define  iMX6_CAN_ESR2_IMB                      DEF_BIT_13

                                                                /* -------------- MAILBOX STRUCT DEFINES -------------- */
#define  iMX6_CAN_MB_DLC_SRR                    DEF_BIT_22
#define  iMX6_CAN_MB_DLC_IDE                    DEF_BIT_21
#define  iMX6_CAN_MB_DLC_RTR                    DEF_BIT_20


/*
*********************************************************************************************************
*                                               MACROS
*
* Note(s) : (1) When Using Extended IDs for Rx'd Messages, its required to save both the Standard
*               Section and Extended Section of the ID and placed both in the Rx Frame in the
*               following Format:
*                           Bits:  31     30    29                   18                   0
*                   uC/CAN Frame: [ 0u | RTR | IDE |    Standard ID    |    Extended ID    ]
*
*           (2) iMX6 Rx FIFO & MB Structure follows the same format as the uC/CAN Frame for Standard
*               and Extended IDs, thus if saving both Standard & Extended IDs, the Register will only
*               have to be masked with the first 28 Bits of the Rx FIFO ID Register.
*********************************************************************************************************
*/
                                                                /* ---------------- MODE CONFIG MACROs ---------------- */
#define  iMX6_CAN_MCR_MAXMB(x)                ((x) & 0x7Fu)     /* Set Number of Last MB used in Arbitration Process.   */

                                                                /* ----------------- CONTROL 1 MACROs ----------------- */
                                                                /* Set Baud Rate Prescaler.                             */
#define  iMX6_CAN_CTRL1_PRESDIV(x)           (((x) & 0xFFu) << 24u)

                                                                /* Set Bit Timing Settings.                             */
#define  iMX6_CAN_CTRL1_RJW(x)               (((x) & 0x03u) << 22u)
#define  iMX6_CAN_CTRL1_PSEG1(x)             (((x) & 0x07u) << 19u)
#define  iMX6_CAN_CTRL1_PSEG2(x)             (((x) & 0x07u) << 16u)

#define  iMX6_CAN_CTRL1_PROP_SEG(x)           ((x) & 0x07u)     /* Set Propagation Segment Bit Time.                    */

                                                                /* ----------------- CONTROL 2 MACROs ----------------- */
                                                                /* Set Number of Rx FIFO Filters.                       */
#define  iMX6_CAN_CTRL2_RFEN(x)              (((x) & 0x0Fu) << 24u)
                                                                /* Set Qty of CAN bits are delayed in the Arb. process. */
#define  iMX6_CAN_CTRL2_TASD(x)              (((x) & 0x1Fu) << 19u)

                                                                /* -------------- ERROR & STATUS 2 MACROs ------------- */
                                                                /* Transmit MB with Lowest Prio. Only Based on CAN Cfg. */
#define  iMX6_CAN_ESR2_LPTM(x)               (((x) >> 16u) & 0x3Fu)


                                                                /* --------------- MAILBOX STRUCT MACROs -------------- */
                                                                /*  - TRANSMISSION -                                    */
                                                                /* Standard & Extended Message ID(s).                   */
#define  iMX6_CAN_Tx_ID_STD(x)               (((x) & iMX6_CAN_SID_MASK) << 18u)
#define  iMX6_CAN_Tx_ID_EXT(x)                ((x) & iMX6_CAN_EID_MASK)
                                                                /* Data Length Count. Determines Qty of Data Tx'd.      */
#define  iMX6_CAN_Tx_DLC_DLC(x)              (((x) & 0x0Fu) << 16u)
                                                                /* Writes Data Byte(s) based on DLC Count Value.        */
#define  iMX6_CAN_Tx_DATA_BYTES(x, y)        (((x) & 0xFFu) << (24u - (8u * (y))))
                                                                /* Put / Get the Tx MB Message Code for Tx Buffers.     */
#define  iMX6_CAN_Tx_PUT_CODE(x)             (((x) & 0x0Fu) << 24u)
#define  iMX6_CAN_Tx_GET_CODE(x)             (((x) >> 24u) & 0x0Fu)

                                                                /*  - RECEPTION -                                       */
                                                                /* Standard & Extended Message ID(s).                   */
#define  iMX6_CAN_Rx_ID_STD(x)               (((x) >> 18u) & iMX6_CAN_SID_MASK)
#define  iMX6_CAN_Rx_ID_EXT(x)                ((x) & iMX6_CAN_EID_MASK)
                                                                /* Gets BOTH Standard & Extended IDs. See Note (1)&(2). */
#define  iMX6_CAN_Rx_ID_BOTH(x)               ((x) & 0x1FFFFFFFu)
                                                                /* Data Length Count. Determines Qty of Rx'd Data.      */
#define  iMX6_CAN_Rx_DLC_DLC(x)              (((x) >> 16u) & 0x0Fu)
                                                                /* Read Data Byte(s) based on DLC Count Value.          */
#define  iMX6_CAN_Rx_DATA_BYTES(x, y)        (((x) >> (24u - (8u * (y)))) & 0xFFu)


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
*                                        CAN MAILBOX STRUCTURE
*
* Description : Structure defines the CAN Mailbox Message Buffer Structure.
*
* Note(s)     : (1) Since Mailboxes are placed in RAM (based on the quantity of mailboxes created at one
*                   time, then there needs to be a separate message buffer mailbox structure that isn't
*                   tied to the CAN Register Set.
*********************************************************************************************************
*/

typedef  volatile  struct  imx6_can_mb_struct {                 /* --------- CAN MAILBOX MESSAGE BUFFER STRUCT -------- */
    CPU_REG32  RX_DLC;                                          /* CAN RX MB DLC    : Contains DLC, SSR, RTR, IDE Info. */
    CPU_REG32  RX_ID;                                           /* CAN RX MB ID     : Contains Standard & Extended ID.  */
    CPU_REG32  RX_DATA1;                                        /* CAN RX MB DATA 1 : Contains Data Byte(s) 0 -> 3.     */
    CPU_REG32  RX_DATA2;                                        /* CAN RX MB DATA 2 : Contains Data Byte(s) 4 -> 7.     */
    CPU_REG32  RSVD0[20u];                                      /*                  : Reserved for Rx FIFO Engine.      */
    CPU_REG32  ID_FILTER[iMX6_CAN_RX_FIFO_ID_FILTER_MIN_LIMIT]; /* CAN RX ID FILTER : ID Filter Table Elements.         */
    struct {
        CPU_REG32  DLC;                                         /* CAN TX MB DLC    : Contains DLC, SSR, IDE, RTR, CODE.*/
        CPU_REG32  ID;                                          /* CAN TX MB ID     : Contains Standard & Extended ID.  */
        CPU_REG32  DATA1;                                       /* CAN TX MB DATA 1 : Contains Data Byte(s) 0 -> 3.     */
        CPU_REG32  DATA2;                                       /* CAN TX MB DATA 2 : Contains Data Byte(s) 4 -> 7.     */
    } TX[iMX6_CAN_TX_MB_LIMIT];                                 /* CAN TX MB QTY    : Quantity of Tx MB Configured.     */
} iMX6_CAN_MB_STRUCT;


/*
*********************************************************************************************************
*                                            CAN REGISTER
*
* Description : Structure defines the CAN Register Structure.
*
* Note(s)     : (1) The iMX6 Reference Manual places both CAN 1 & CAN 2 under a continuous Memory Map. For
*                   However, the CAN Register Structure for both CAN 1 & CAN 2 is the same, they are just
*                   offset from each other.
*
*               (2) The '_' in the 0x_00C Register Range of the comment is to differentiate between CAN 1
*                   or CAN 2. So it could be either 0x000C or 0x400C. Same applies to all Reserved Regs.
*********************************************************************************************************
*/

typedef  volatile  struct  imx6_can_reg {                       /* ---------- CAN CONTROLLER REGISTER SUMMARY --------- */
    CPU_REG32  MCR;                                             /* Module Configuration     Register.                   */
    CPU_REG32  CTRL1;                                           /* Control 1                Register.                   */
    CPU_REG32  TIMER;                                           /* Free Running Timer       Register.                   */
    CPU_REG32  RSVD1;                                           /* Reserved. Register Range: 0x_00C. Note (2).          */
    CPU_REG32  RXMGMASK;                                        /* Rx Mailbox Global Mask   Register.                   */
    CPU_REG32  RX14MASK;                                        /* Rx Buffer 14 Mask        Register.                   */
    CPU_REG32  RX15MASK;                                        /* Rx Buffer 15 Mask        Register.                   */
    CPU_REG32  ECR;                                             /* Error Counter            Register.                   */
    CPU_REG32  ESR1;                                            /* Error and Status 1       Register.                   */
    CPU_REG32  IMASK2;                                          /* Interrupt Mask 2         Register.                   */
    CPU_REG32  IMASK1;                                          /* Interrupt Mask 1         Register.                   */
    CPU_REG32  IFLAG2;                                          /* Interrupt Flags 2        Register.                   */
    CPU_REG32  IFLAG1;                                          /* Interrupt Flags 1        Register.                   */
    CPU_REG32  CTRL2;                                           /* Control 2                Register.                   */
    CPU_REG32  ESR2;                                            /* Error and Status 2       Register.                   */
    CPU_REG32  RSVD2[2u];                                       /* Reserved. Register Range: 0x_03C -> 0x_040.          */
    CPU_REG32  CRCR;                                            /* CRC                      Register.                   */
    CPU_REG32  RXFGMASK;                                        /* Rx FIFO Global Mask      Register.                   */
    CPU_REG32  RXFIR;                                           /* Rx FIFO Information      Register.                   */
    CPU_REG32  RSVD3[524u];                                     /* Reserved. Register Range: 0x_050 -> 0x_87C.          */
    CPU_REG32  RXIMR[64u];                                      /* Rx Individual Mask       Register.                   */
    CPU_REG32  RSVD4[24u];                                      /* Reserved. Register Range: 0x_980 -> 0x_9DC.          */
    CPU_REG32  GFWR;                                            /* Glitch Filter Width      Register.                   */
} iMX6_CAN_REG;


/*
*********************************************************************************************************
*                                        CAN BAUDRATE REGISTER
*
* Description : Structure defines the CAN BaudRate Register Structure.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  struct  imx6_can_baud {    
    CPU_INT32U  BaudRate;                                       /* Holds the Baudrate.                                  */
    CPU_INT32U  SamplePoint;                                    /* Holds the Sample point in 0.1%                       */
    CPU_INT32U  ReSynchJumpWidth;                               /* Holds the Re-Synchronization Jump Width in 0.1%      */
    CPU_INT08U  PrescalerDiv;                                   /* Holds the Prescaler Divide Factor                    */
    CPU_INT08U  SJW;                                            /* Holds the Re-Synch Jump Width         (StdValue = 1) */
    CPU_INT08U  PropagationSeg;                                 /* Holds the Propagation Segment Time    (StdValue = 2) */
    CPU_INT08U  PhaseBufSeg1;                                   /* Holds the Phase Buffer Segment 1      (StdValue = 7) */
    CPU_INT08U  PhaseBufSeg2;                                   /* Holds the Phase Buffer Segment 2      (StdValue = 7) */
} iMX6_CAN_BAUD;


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
