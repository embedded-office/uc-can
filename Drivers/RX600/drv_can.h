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
*                                           CAN DRIVER CODE
*
*                                            Renesas RX600
*
* Filename : drv_can.h
* Version  : V2.42.01
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


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

#define  RX600_CAN_NAME           "RX600:CAN Module"            /* Unique Driver Name for Installation                  */


#define  RX600_CAN_NBR_MBOX                     32u             /* Number of Mailboxes                                  */
#define  RX600_CAN_NBR_MASK                      8u             /* Number of Masks                                      */

#define  RX600_CAN_SID_LIMIT                0x07FFu             /* CAN Standard ID Value Limit                          */
#define  RX600_CAN_SID_SHIFT                    18u             /* CAN Standard ID Value Shift                          */
#define  RX600_CAN_SID_RX_MASK          0x1FFC0000u             /* CAN Standard ID Mask Value                           */

#define  RX600_CAN_EID_LIMIT            0x1FFFFFFFu             /* CAN Extended ID Value Limit                          */

#define  RX600_CAN_FIFO_MSK_VALID       0x00FFFFFFu             /* CAN FIFO MKIVLR Mask Valid Value                     */

#define  RX600_CAN_DLC_LIMIT                  0x0Fu             /* Limit DLC to 8 Bytes                                 */

                                                                /* ------------------- uC/CAN FRAME ------------------- */
#define  RX600_CAN_FRM_IDE              DEF_BIT_29
#define  RX600_CAN_FRM_RTR              DEF_BIT_30


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

typedef  CPU_INT08U  CAN_TX_RX_SEL;

typedef  struct  rx600_can_mb_array {
    CAN_TX_RX_SEL  tx_rx;
    CPU_INT32U     ID;
} RX600_CAN_MB_ARRAY;


/*
*********************************************************************************************************
*                                            DEVICE NAMES
*
* Description : Enumeration defines the Available Device Names for the Driver XXXCANInit() Function.
*
* Note(s)     : The RX600 Driver only Supports 3 Internal CAN Controllers (0->2).
*********************************************************************************************************
*/

typedef  enum  rx600_can_dev {
    RX600_CAN_BUS_0 = 0u,                                       /* Internal CAN controller #0                           */
    RX600_CAN_BUS_1 = 1u,                                       /* Internal CAN controller #1                           */
    RX600_CAN_BUS_2 = 2u,                                       /* Internal CAN controller #2                           */
    RX600_CAN_N_DEV = 3u                                        /* Number of CAN controllers                            */
} RX600_CAN_DEV;


/*
*********************************************************************************************************
*                                         DRIVER ERROR CODES
*
* Description : Enumeration defines the possible Driver Error Codes.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  enum rx600_can_err {
    RX600_CAN_ERR_NONE = 0,                                     /* NO        ERR: Everything is OK                      */
    RX600_CAN_ERR_BUS,                                          /* BUS       ERR: Wrong Bus Was Chosen                  */
    RX600_CAN_ERR_BUSY,                                         /* BUSY      ERR: Msg Can't be Sent, Bus is Busy        */
    RX600_CAN_ERR_INIT,                                         /* INIT      ERR: Reset State not Set, Dev Init Fail    */
    RX600_CAN_ERR_MODE,                                         /* MODE      ERR: Error Accessing Wanted Mode on Device */
    RX600_CAN_ERR_OPEN,                                         /* OPEN      ERR: Device can't be Used, Device un-Opened*/
    RX600_CAN_ERR_CLOSE,                                        /* CLOSE     ERR: Device can't be Closed                */
    RX600_CAN_ERR_FUNC,                                         /* FUNCTION  ERR: Given Function Code is not Valid      */
    RX600_CAN_ERR_ARG,                                          /* ARGUMENT  ERR: Argument Check has Failed             */
    RX600_CAN_ERR_NO_DATA,                                      /* DATA      ERR: No Data is Available                  */
    RX600_CAN_ERR_MAILBOX,                                      /* MAILBOX   ERR: Improper Mailbox Configuration Array. */
    RX600_CAN_ERR_MSG_LOST,                                     /* MSG LOST  ERR: Message is Overwritten/Overrun or Lost*/
    RX600_CAN_ERR_FREE_MBOX = 99                                /* FREE MBOX ERR: Search Free Mailbox Returned Error    */
} RX600_CAN_ERR;


/*
*********************************************************************************************************
*                                     I/O CONTROL FUNCTION CODES
*
* Description : Enumeration defines the available Function Codes for the Driver XXXIoCtl() Function.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  enum  rx600_can_io_list {
    IO_RX600_CAN_GET_IDENT = 0x0,                               /* -------- GET DRIVER IDENTIFICATION CODE ------------ */
                                                                /* arg = Pointer to Local Ident Variable (CPU_INT32U)   */
    IO_RX600_CAN_GET_ERRNO,                                     /* ------------ GET DRIVER ERROR CODE ----------------- */
                                                                /* arg = Pointer to Local Error Code Var. (CPU_INT16U)  */
    IO_RX600_CAN_GET_DRVNAME,                                   /* --------------- GET DRIVER NAME -------------------- */
                                                                /* arg = Pointer to Local String Variable (char)        */
    IO_RX600_CAN_SET_BAUDRATE,                                  /* -------------- SET BUS BAUDRATE -------------------- */
                                                                /* arg = Pointer to Local Baudrate Var. (CPU_INT32U)    */
    IO_RX600_CAN_START,                                         /* ----------------- ENABLE BUS ----------------------- */
                                                                /* No Pointer: Fnct Code Starts the CAN Cntrl Interface */
    IO_RX600_CAN_STOP,                                          /* ----------------- DISABLE BUS ---------------------- */
                                                                /* No Pointer: Fnct Code Stops the CAN Cntrl Interface  */
    IO_RX600_CAN_RX_STANDARD,                                   /* ------- SET  RECIEVER TO STANDARD IDENTIFIER ------- */
                                                                /* No Pointer: CAN Rx recieves only CAN Standard IDs    */
    IO_RX600_CAN_RX_EXTENDED,                                   /* ------- SET  RECIEVER TO EXTENDED IDENTIFIER ------- */
                                                                /* No Pointer: CAN Rx recieves only CAN Extended IDs    */
    IO_RX600_CAN_TX_READY,                                      /* --------------- GET TX READY STATUS ---------------- */
                                                                /* arg = Pointer to TX Rdy Status Variable (CPU_INT08U) */
    IO_RX600_CAN_GET_NODE_STATUS,                               /* ---------------- GET NODE STATUS ------------------- */
                                                                /* arg = Pointer to Node Status Variable (CPU_INT08U)   */
    IO_RX600_CAN_SET_RX_FILTER,                                 /* ------------------ SET RX FILTER 1 ----------------- */
                                                                /* arg = CPU_INT32U[2]: arg[0] = mask, arg[1] = ID      */
    IO_RX600_CAN_IO_FUNC_N                                      /* -------------- NUMBER OF FUCNTION CODES ------------ */
} RX600_CAN_IO_LIST;                                            /* No Pointer: Holds number of Function Codes Available */


/*
********************************************************************************************************
*                                           RX600 CAN MODES
*
* Description : Enumeration defines the RX600 CAN Modes
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  enum rx600_can_mode {
    RX600_CAN_MODE_EXIT_SLEEP = 0u,                             /* Exit  Sleep Mode                                     */
    RX600_CAN_MODE_RESET,                                       /* Enter Reset Mode                                     */
    RX600_CAN_MODE_HALT,                                        /* Enter Halt  Mode                                     */
    RX600_CAN_MODE_OPERATE                                      /* Enter Operating Mode                                 */
} RX600_CAN_MODE;

typedef  enum rx600_can_mail_search {
    RX600_CAN_RX_SEARCH_MODE = 0u,                              /* Recieve Mailbox Search Mode                          */
    RX600_CAN_TX_SEARCH_MODE,                                   /* Transmit Mailbox Search Mode                         */
    RX600_CAN_MSG_SEARCH_MODE,                                  /* Message Lost Search Mode                             */
    RX600_CAN_CHNL_SEARCH_MODE                                  /* Channel Search Mode                                  */
} RX600_CAN_MAIL_SELECT;


/*
*********************************************************************************************************
*                                         DRIVER RUNTIME DATA
*
* Description : Structure holds the Driver Runtime data.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  struct  rx600_can_data {
    CPU_BOOLEAN     Use;                                        /* USE MARKER: Marker Indicating if Dev is In Use       */
    RX600_CAN_REG  *RegPtr;                                     /* REGISTER  : Pointer to the CAN Base Address(s)       */
    CPU_BOOLEAN     InitCmpl;                                   /* INIT COMP : Completion Variable of TX MBoxes         */
    CPU_INT08U      AvailTxMBox;                                /* AVAILABLE : Tx MBox is Available for Config          */
} RX600_CAN_DATA;


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

typedef  struct  rx600_can_frm {
    CPU_INT32U  Identifier;                                     /* CAN IDENTIFIER: Can Identifier                       */
    CPU_INT08U  Data[8u];                                       /* CAN PAYLOAD   : Bytes[Max 8] in Single CAN Msg       */
    CPU_INT08U  DLC;                                            /* CAN DLC       : Num of Valid Data(s) in Payload      */
    CPU_INT08U  Spare[3u];                                      /* SPARE         : Sets FRM w/ Integral Num of Pointers */
} RX600_CAN_FRM;


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

CPU_INT16S  RX600_CAN_Init    (CPU_INT32U   para_id);

CPU_INT16S  RX600_CAN_Open    (CPU_INT16S   dev_id,
                               CPU_INT32U   dev_name,
                               CPU_INT16U   mode);

CPU_INT16S  RX600_CAN_Close   (CPU_INT16S   para_id);

CPU_INT16S  RX600_CAN_IoCtl   (CPU_INT16S   para_id,
                               CPU_INT16U   func,
                               void        *p_arg);

CPU_INT16S  RX600_CAN_Read    (CPU_INT16S   para_id,
                               CPU_INT08U  *buf,
                               CPU_INT16U   size);

CPU_INT16S  RX600_CAN_Write   (CPU_INT16S   para_id,
                               CPU_INT08U  *buf,
                               CPU_INT16U   size);

void        RX600_CAN_ErrCheck(CPU_INT32U   para_id);


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

#ifndef  CAN_MODULE_CHANNEL_1
#define  CAN_MODULE_CHANNEL_1                   DEF_DISABLED
#warning "CAN Module Channel 1 is DISABLED by default but not defined in can_cfg.h."
#endif

#ifndef  CAN_MODULE_CHANNEL_2
#define  CAN_MODULE_CHANNEL_2                   DEF_DISABLED
#warning "CAN Module Channel 2 is DISABLED by default but not defined in can_cfg.h."
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

#ifndef  CAN_TEST_MODE
#define  CAN_TEST_MODE                          CAN_TEST_NO_TEST
#endif

#if    ((CAN_TEST_MODE != CAN_TEST_NO_TEST ) && \
        (CAN_TEST_MODE != CAN_TEST_INTERNAL) && \
        (CAN_TEST_MODE != CAN_TEST_EXTERNAL))
#error  "CAN_TEST_MODE must be either CAN_TEST_NO_TEST, CAN_TEST_INTERNAL, or CAN_TEST_INTERNAL."
#error  "Please select CAN_TEST_MODE as one of these options only.                              "
#endif

                                                                /* ------------------ MAILBOX SELECT ------------------ */
#ifndef  NORMAL_MAILBOX_MODE
#define  NORMAL_MAILBOX_MODE                    0u              /* NORMAL_MAILBOX_MODE:     MB[27]-MB[31]               */
#endif                                                          /*                        Considered NORMAL Mailbox(es) */

#ifndef  FIFO_MAILBOX_MODE
#define  FIFO_MAILBOX_MODE                      1u              /*   FIFO_MAILBOX_MODE:     MB[27]-MB[31]               */
#endif                                                          /*                        Considered   FIFO Mailbox(es) */

#ifndef  CAN_MAILBOX_MODE
#define  CAN_MAILBOX_MODE                       NORMAL_MAILBOX_MODE
#endif

#if    ((CAN_MAILBOX_MODE != NORMAL_MAILBOX_MODE ) && \
        (CAN_MAILBOX_MODE != FIFO_MAILBOX_MODE   ))
#error  "CAN_MAILBOX_MODE must be either NORMAL_MAILBOX_MODE, or FIFO_MAILBOX_MODE."
#error  "Please select CAN_MAILBOX_MODE as one of these options only.              "
#endif

                                                                /* --------------- MAILBOX CONFIGURATION -------------- */
#ifndef  MB_OVERWRITE_MODE
#define  MB_OVERWRITE_MODE                      0u
#endif

#ifndef  MB_OVERRUN_MODE
#define  MB_OVERRUN_MODE                        1u
#endif

#ifndef  CAN_MB_MSG_LOST_MODE
#define  CAN_MB_MSG_LOST_MODE                   MB_OVERRUN_MODE
#endif

#if    ((CAN_MB_MSG_LOST_MODE != MB_OVERRUN_MODE  ) && \
        (CAN_MB_MSG_LOST_MODE != MB_OVERWRITE_MODE))
#error  "CAN_MB_MSG_LOST_MODE must be either MB_OVERRUN_MODE, or MB_OVERWRITE_MODE."
#error  "Please select CAN_MB_MSG_LOST_MODE as one of these options only.          "
#endif

                                                                /*      - TX / RX BOOLEAN DEFINES -                     */
#ifndef  CAN_NONE
#define  CAN_NONE                               0u
#endif

#ifndef  CAN_TX
#define  CAN_TX                                 1u
#endif

#ifndef  CAN_RX
#define  CAN_RX                                 2u
#endif

                                                                /*      - CAN MAILBOX ARRAY(S) -                        */
#ifndef  RX600_CAN_CFG_MB_EN                                    /* Configure RX600 CAN Mailbox Channels.                */
#define  RX600_CAN_CFG_MB_EN                    DEF_DISABLED
#endif
                                                                /*      - DEFAULT CAN MAILBOX ARRAY -                   */
#if (RX600_CAN_CFG_MB_EN == DEF_DISABLED)

    #if ((CAN_MODULE_CHANNEL_0 == DEF_ENABLED) || \
         (CAN_MODULE_CHANNEL_1 == DEF_ENABLED) || \
         (CAN_MODULE_CHANNEL_2 == DEF_ENABLED))

        #if (CAN_MAILBOX_MODE == NORMAL_MAILBOX_MODE)              /* -- MB [n*4] ----- MB [n*4 + 1] --- MB [n*4 + 2] --- MB [n*4 + 3] ------ N -- */
static  RX600_CAN_MB_ARRAY  CANx_MB_ARRAY_CFG[RX600_CAN_NBR_MBOX] = {{CAN_RX, 0x00u}, {CAN_RX, 0x00u}, {CAN_RX, 0x00u}, {CAN_RX, 0x00u},  /* n = 0 */
                                                                     {CAN_RX, 0x00u}, {CAN_RX, 0x00u}, {CAN_RX, 0x00u}, {CAN_RX, 0x00u},  /* n = 1 */
                                                                     {CAN_RX, 0x00u}, {CAN_RX, 0x00u}, {CAN_RX, 0x00u}, {CAN_RX, 0x00u},  /* n = 2 */
                                                                     {CAN_RX, 0x00u}, {CAN_RX, 0x00u}, {CAN_RX, 0x00u}, {CAN_RX, 0x00u},  /* n = 3 */
                                                                     {CAN_RX, 0x00u}, {CAN_RX, 0x00u}, {CAN_RX, 0x00u}, {CAN_RX, 0x00u},  /* n = 4 */
                                                                     {CAN_RX, 0x00u}, {CAN_RX, 0x00u}, {CAN_RX, 0x00u}, {CAN_RX, 0x00u},  /* n = 5 */
                                                                     {CAN_RX, 0x00u}, {CAN_RX, 0x00u}, {CAN_RX, 0x00u}, {CAN_RX, 0x00u},  /* n = 6 */
                                                                     {CAN_RX, 0x00u}, {CAN_RX, 0x00u}, {CAN_TX, 0x00u}, {CAN_TX, 0x00u}}; /* n = 7 */
        #else                                                      /* <------------ RX -------------|  <------------- TX ------------->            */
static  RX600_CAN_MB_ARRAY  CANx_MB_ARRAY_CFG[RX600_CAN_NBR_MBOX] = {{CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  ,  /* n = 0 */
                                                                     {CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  ,  /* n = 1 */
                                                                     {CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  ,  /* n = 2 */
                                                                     {CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  ,  /* n = 3 */
                                                                     {CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  ,  /* n = 4 */
                                                                     {CAN_RX, 0x00u}  , {CAN_RX, 0x00u}  , {CAN_TX, 0x00u}  , {CAN_TX, 0x00u}  ,  /* n = 5 */
                                                                     {CAN_NONE, 0x00u}, {CAN_NONE, 0x00u}, {CAN_NONE, 0x00u}, {CAN_NONE, 0x00u},  /* n = 6 */
                                                                     {CAN_NONE, 0x00u}, {CAN_NONE, 0x00u}, {CAN_NONE, 0x00u}, {CAN_NONE, 0x00u}}; /* n = 7 */
        #endif

    #endif    
#else
    #ifdef  CANn_MB_ARRAY
    #error  "CANn_MB_ARRAY is an example placed in can_cfg.h, please replace with either CAN0_MB_ARRAY, CAN1_MB_ARRAY, or CAN2_MB_ARRAY. "
    #error  "You can make multiple copies of CANn_MB_ARRAY depenending on the CAN_MODULE_CHANNEL Enabled. "
    #endif

    #if (CAN_MODULE_CHANNEL_0 == DEF_ENABLED)
        #ifndef  CAN0_MB_ARRAY
        #error  "CAN0_MB_ARRAY MUST be Configured if RX_CAN_CFG_MB_EN is ENABLED for CAN Channel 0."
        #endif
    #endif

    #if (CAN_MODULE_CHANNEL_1 == DEF_ENABLED)
        #ifndef  CAN1_MB_ARRAY
        #error  "CAN1_MB_ARRAY MUST be Configured if RX_CAN_CFG_MB_EN is ENABLED for CAN Channel 1."
        #endif
    #endif

    #if (CAN_MODULE_CHANNEL_2 == DEF_ENABLED)
        #ifndef  CAN2_MB_ARRAY
        #error  "CAN2_MB_ARRAY MUST be Configured if RX_CAN_CFG_MB_EN is ENABLED for CAN Channel 2."
        #endif
    #endif
#endif


/*
*********************************************************************************************************
*                                            ERROR SECTION
*********************************************************************************************************
*/

#endif  /* #ifndef _DRV_CAN_H_ */
