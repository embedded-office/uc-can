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
* Filename : drv_can.h
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

#ifndef _DRV_CAN_H_
#define _DRV_CAN_H_

/*
*********************************************************************************************************
*                                              INCLUDES
*********************************************************************************************************
*/

#include  "cpu.h"
#include  "can_bsp.h"
#include  <lib_def.h>
#include  "drv_can_reg.h"

/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

#define  RX200_CAN_NAME           "RX200:CAN Module"            /* Unique Driver Name for Installation                  */

                                                                /* ----------------- RX & TX MAILBOXES ---------------- */
#define  RX200_CAN_RX_MBOX_HALF                  8u             /*   Half  Number of Rx Mailboxes.                      */
#define  RX200_CAN_RX_MBOX_MAX                  16u             /* Maximum Number of Rx Mailboxes.                      */
#define  RX200_CAN_TX_MBOX                       4u             /*   Set   Number of Tx Mailboxes.                      */

                                                                /* --------------- ID & REGISTER LIMITs --------------- */
#define  RX200_CAN_SID_LIMIT                0x07FFu             /* CAN Standard ID Value Limit                          */

#define  RX200_CAN_EID_LIMIT_TOT        0x1FFFFFFFu             /* CAN Extended ID Value Limit                          */
#define  RX200_CAN_EID_LIMIT_H              0x1FFFu             /* CAN Extended ID Value Limit Register xxH.            */

#define  RX200_CAN_REG_LIMIT            DEF_INT_16_MASK         /* CAN Register Limit.                                  */

                                                                /* ------------------- uC/CAN FRAME ------------------- */
#define  RX200_CAN_FRM_IDE              DEF_BIT_29
#define  RX200_CAN_FRM_RTR              DEF_BIT_30


/*
*********************************************************************************************************
*                                               MACROS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             DATA TYPES
*********************************************************************************************************
*/

typedef  CPU_INT32U  RX200_CAN_RX_ID;


/*
*********************************************************************************************************
*                                            DEVICE NAMES
*
* Description : Enumeration defines the Available Device Names for the Driver xxx_CAN_Init() Function.
*
* Note(s)     : The RX200 Driver supports ONLY 1 Internal CAN Controller.
*********************************************************************************************************
*/

typedef  enum  rx200_can_dev {
    RX200_CAN_BUS_0 = 0u,                                       /* Internal CAN controller #0                           */
    RX200_CAN_N_DEV = 1u                                        /* Number of CAN controllers                            */
} RX200_CAN_DEV;


/*
*********************************************************************************************************
*                                         DRIVER ERROR CODES
*
* Description : Enumeration defines the possible Driver Error Codes.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  enum rx200_can_err {
    RX200_CAN_ERR_NONE = 0,                                     /* NO        ERR: Everything is OK                      */
    RX200_CAN_ERR_BUS,                                          /* BUS       ERR: Wrong Bus Was Chosen                  */
    RX200_CAN_ERR_BUSY,                                         /* BUSY      ERR: Msg Can't be Sent, Bus is Busy        */
    RX200_CAN_ERR_INIT,                                         /* INIT      ERR: Initial State not Set, Device Fail    */
    RX200_CAN_ERR_MODE,                                         /* MODE      ERR: Error Accessing Wanted Mode on Device */
    RX200_CAN_ERR_OPEN,                                         /* OPEN      ERR: Device can't be Used, Device un-Opened*/
    RX200_CAN_ERR_CLOSE,                                        /* CLOSE     ERR: Device can't be Closed                */
    RX200_CAN_ERR_FUNC,                                         /* FUNCTION  ERR: Given Function Code is not Valid      */
    RX200_CAN_ERR_ARG,                                          /* ARGUMENT  ERR: Argument Check has Failed             */
    RX200_CAN_ERR_NO_DATA,                                      /* DATA      ERR: No Data is Available                  */
    RX200_CAN_ERR_MAILBOX,                                      /* MAILBOX   ERR: Mailbox Contains no Data.             */
} RX200_CAN_ERR;


/*
*********************************************************************************************************
*                                     I/O CONTROL FUNCTION CODES
*
* Description : Enumeration defines the available Function Codes for the Driver xxx_CAN_IoCtl() Function.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  enum  rx200_can_io_list {
    IO_RX200_CAN_GET_IDENT = 0x0,                               /* -------- GET DRIVER IDENTIFICATION CODE ------------ */
                                                                /* arg = Pointer to Local Ident Variable (CPU_INT32U)   */
    IO_RX200_CAN_GET_ERRNO,                                     /* ------------ GET DRIVER ERROR CODE ----------------- */
                                                                /* arg = Pointer to Local Error Code Var. (CPU_INT16U)  */
    IO_RX200_CAN_GET_DRVNAME,                                   /* --------------- GET DRIVER NAME -------------------- */
                                                                /* arg = Pointer to Local String Variable (char)        */
    IO_RX200_CAN_SET_BAUDRATE,                                  /* -------------- SET BUS BAUDRATE -------------------- */
                                                                /* arg = Pointer to Local Baudrate Var. (CPU_INT32U)    */
    IO_RX200_CAN_START,                                         /* ----------------- ENABLE BUS ----------------------- */
                                                                /* No Pointer: Fnct Code Starts the CAN Cntrl Interface */
    IO_RX200_CAN_STOP,                                          /* ----------------- DISABLE BUS ---------------------- */
                                                                /* No Pointer: Fnct Code Stops the CAN Cntrl Interface  */
    IO_RX200_CAN_RX_STANDARD,                                   /* ------- SET  RECIEVER TO STANDARD IDENTIFIER ------- */
                                                                /* No Pointer: CAN Rx recieves only CAN Standard IDs    */
    IO_RX200_CAN_RX_EXTENDED,                                   /* ------- SET  RECIEVER TO EXTENDED IDENTIFIER ------- */
                                                                /* No Pointer: CAN Rx recieves only CAN Extended IDs    */
    IO_RX200_CAN_TX_READY,                                      /* --------------- GET TX READY STATUS ---------------- */
                                                                /* arg = Pointer to TX Rdy Status Variable (CPU_INT08U) */
    IO_RX200_CAN_GET_NODE_STATUS,                               /* ---------------- GET NODE STATUS ------------------- */
                                                                /* arg = Pointer to Node Status Variable (CPU_INT08U)   */
    IO_RX200_CAN_SET_RX_FILTER,                                 /* ------------------ SET RX FILTER 1 ----------------- */
                                                                /* arg = CPU_INT32U[2]: arg[0] = mask, arg[1] = ID      */
    IO_RX200_CAN_IO_FUNC_N                                      /* -------------- NUMBER OF FUCNTION CODES ------------ */
} RX200_CAN_IO_LIST;                                            /* No Pointer: Holds number of Function Codes Available */


/*
*********************************************************************************************************
*                                           RX200 CAN MODES
*
* Description : Enumeration defines for the RX200 CAN Driver.
*
* Note(s)     : (1) The following enumerations define the generic CAN Modes that the driver will be
*                   set to, to perform configuration options. Also, it will define internal
*                   (Global & Channel Based) operating modes that will be used to check the "status"
*                   and "location" of the CAN Module and adjust the mode according to what the generic
*                   CAN Modes desire.
*********************************************************************************************************
*/

typedef  enum  rx200_can_mode {                                 /* ----------------- GENERIC CAN MODEs ---------------- */
    RX200_CAN_MODE_OPERATE = 0u,                                /* Set Mode : Generic Operating Mode                    */
    RX200_CAN_MODE_TEST,                                        /* Set Mode : Generic    Test   Mode                    */
    RX200_CAN_MODE_GL_RESET,                                    /* Set Mode : Global    Reset   Mode                    */
    RX200_CAN_MODE_CH_RESET,                                    /* Set Mode : Channel   Reset   Mode                    */
    RX200_CAN_MODE_STOP,                                        /* Set Mode : Generic    Stop   Mode                    */
    RX200_CAN_MODE_HALT                                         /* Set Mode : Generic    Halt   Mode                    */
} RX200_CAN_MODE;


typedef  enum  rx200_can_drv_gl_mode {                          /* ------------ DRIVER BASED : GLOBAL MODEs ----------- */
    RX200_CAN_DRV_GL_MODE_STOP = 10u,                           /* Driver Setting : Global    Stop   Mode               */
    RX200_CAN_DRV_GL_MODE_RESET,                                /* Driver Setting : Global   Reset   Mode               */
    RX200_CAN_DRV_GL_MODE_TEST,                                 /* Driver Setting : Global    Test   Mode               */
    RX200_CAN_DRV_GL_MODE_OPERATING                             /* Driver Setting : Global Operating Mode               */
} RX200_CAN_DRV_GL_MODE;


typedef  enum  rx200_can_drv_ch_mode {                          /* ----------- DRIVER BASED : CHANNEL MODEs ----------- */
    RX200_CAN_DRV_CH_MODE_STOP = 20u,                           /* Driver Setting : Channel      Stop     Mode          */
    RX200_CAN_DRV_CH_MODE_RESET,                                /* Driver Setting : Channel     Reset     Mode          */
    RX200_CAN_DRV_CH_MODE_HALT,                                 /* Driver Setting : Channel      Halt     Mode          */
    RX200_CAN_DRV_CH_MODE_COMM                                  /* Driver Setting : Channel Communication Mode          */
} RX200_CAN_DRV_CH_MODE;


/*
*********************************************************************************************************
*                                         DRIVER RUNTIME DATA
*
* Description : Structure holds the Driver Runtime data.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  struct  rx200_can_data {
    CPU_BOOLEAN  Use;                                           /* USE MARKER : Marker Indicating if Dev is In Use      */
    CPU_INT32U   RegPtr;                                        /* REGISTER   : CAN Base Address(s)                     */
    CPU_INT32U   RR_RegPtr;                                     /* REGISTER   : Receive Rule Register Base Address      */
} RX200_CAN_DATA;


/*
*********************************************************************************************************
*                                          CAN FRAME STRUCT
*
* Description : Structure defines a CAN Frame.
*
* Note(s)     : To Differentiate between Standard and Extended IDs, the following Addition to the
*               ID is implemented: (Based on the Structure found in uC/CAN Frame files).
*                   - Bit #31     : Reserved (Always 0u)
*                   - Bit #30     : Remote Transmission Request Flag (1u = RTR, 0u = Data Frame)
*                   - Bit #29     : Extended ID Flag (1u = Extended, 0u = Standard)
*                   - Bit #28 - 0 : Identifier (Standard, Extended, or Both)
*********************************************************************************************************
*/

typedef  struct  rx200_can_frm {
    CPU_INT32U  Identifier;                                     /* CAN IDENTIFIER : Can Identifier.                     */
    CPU_INT08U  Data[8u];                                       /* CAN PAYLOAD    : Bytes [Max 8] in Single CAN Msg.    */
    CPU_INT08U  DLC;                                            /* CAN DLC        : Num of Valid Data(s) in Payload.    */
    CPU_INT08U  Spare[3u];                                      /* SPARE          : Sets FRM w/ Integral Num of Ptrs.   */
} RX200_CAN_FRM;


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

CPU_INT16S  RX200_CAN_Init    (CPU_INT32U   para_id);

CPU_INT16S  RX200_CAN_Open    (CPU_INT16S   dev_id,
                               CPU_INT32U   dev_name,
                               CPU_INT16U   mode);

CPU_INT16S  RX200_CAN_Close   (CPU_INT16S   para_id);

CPU_INT16S  RX200_CAN_IoCtl   (CPU_INT16S   para_id,
                               CPU_INT16U   func,
                               void        *p_arg);

CPU_INT16S  RX200_CAN_Read    (CPU_INT16S   para_id,
                               CPU_INT08U  *buf,
                               CPU_INT16U   size);

CPU_INT16S  RX200_CAN_Write   (CPU_INT16S   para_id,
                               CPU_INT08U  *buf,
                               CPU_INT16U   size);

void        RX200_CAN_ErrCheck(CPU_INT32U   para_id);


/*
*********************************************************************************************************
*                                        DEFAULT CONFIGURATION
*********************************************************************************************************
*/
                                                                /* ------------------- MODULE SELECT ------------------ */
#ifndef  CAN_MODULE_CHANNEL_0
#define  CAN_MODULE_CHANNEL_0                   DEF_DISABLED
#warning "CAN Module Channel 0 is DISABLED by default but not defined in can_cfg.h."
#endif

                                                                /* ------------------- DRIVER SELECT ------------------ */
#ifndef  CAN_DRV_RX200
#define  CAN_DRV_RX200                          0u
#endif

#ifndef  CAN_DRV_RL78_F1x
#define  CAN_DRV_RL78_F1x                       1u
#endif

#ifndef  CAN_DRV_SELECT
#define  CAN_DRV_SELECT                         CAN_DRV_RX200
#endif

                                                                /* -------------------- TEST SELECT ------------------- */
#ifndef  CAN_TEST_NO_TEST                                       /* Definition for CAN_TEST_MODE, Options:               */
#define  CAN_TEST_NO_TEST                       0u              /*              NO        Test Mode                     */
#endif

#ifndef  CAN_TEST_INTERNAL
#define  CAN_TEST_INTERNAL                      1u              /*      Internal Loopback Test Mode                     */
#endif

#ifndef  CAN_TEST_EXTERNAL
#define  CAN_TEST_EXTERNAL                      2u              /*      External Loopback Test Mode                     */
#endif

#ifndef  CAN_TEST_LISTEN_ONLY
#define  CAN_TEST_LISTEN_ONLY                   3u              /*         Listen-Only    Test Mode                     */
#endif

#ifndef  CAN_TEST_MODE
#define  CAN_TEST_MODE                          CAN_TEST_NO_TEST
#endif

#if    ((CAN_TEST_MODE != CAN_TEST_NO_TEST ) && \
        (CAN_TEST_MODE != CAN_TEST_INTERNAL) && \
        (CAN_TEST_MODE != CAN_TEST_EXTERNAL) && \
        (CAN_TEST_MODE != CAN_TEST_LISTEN_ONLY))
#error  "CAN_TEST_MODE must be either CAN_TEST_NO_TEST, CAN_TEST_INTERNAL, CAN_TEST_INTERNAL, or CAN_TEST_LISTEN_ONLY."
#error  "Please select CAN_TEST_MODE as one of these options only in can_cfg.h                                        "
#endif

                                                                /* --------------- MAILBOX CONFIGURATION -------------- */
#ifndef  CAN_MB_ARRAY
#define  CAN_MB_ARRAY    /* - Rx Mailbox  0 ID - */    {0x00u, \
                         /* - Rx Mailbox  1 ID - */     0x00u, \
                         /* - Rx Mailbox  2 ID - */     0x00u, \
                         /* - Rx Mailbox  3 ID - */     0x00u, \
                         /* - Rx Mailbox  4 ID - */     0x00u, \
                         /* - Rx Mailbox  5 ID - */     0x00u, \
                         /* - Rx Mailbox  6 ID - */     0x00u, \
                         /* - Rx Mailbox  7 ID - */     0x00u, \
                         /* - Rx Mailbox  8 ID - */     0x00u, \
                         /* - Rx Mailbox  9 ID - */     0x00u, \
                         /* - Rx Mailbox 10 ID - */     0x00u, \
                         /* - Rx Mailbox 11 ID - */     0x00u, \
                         /* - Rx Mailbox 12 ID - */     0x00u, \
                         /* - Rx Mailbox 13 ID - */     0x00u, \
                         /* - Rx Mailbox 14 ID - */     0x00u, \
                         /* - Rx Mailbox 15 ID - */     0x00u}
#endif


/*
*********************************************************************************************************
*                                              MODULE END
*********************************************************************************************************
*/

#endif  /* #ifndef _DRV_CAN_H_ */
