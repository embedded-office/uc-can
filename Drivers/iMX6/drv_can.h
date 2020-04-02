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
*                                       Freescale iMX6 Series
*
* Filename : drv_can.h
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

#ifndef  _DRV_CAN_H_
#define  _DRV_CAN_H_

/*
*********************************************************************************************************
*                                              INCLUDES
*********************************************************************************************************
*/

#include  <drv_can_reg.h>
#include  "drv_def.h"
#include  "can_bsp.h"

#include  <lib_def.h>
#include  <lib_mem.h>
#include  "can_bus.h"
#include  "cpu.h"


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

#define  iMX6_CAN_NAME              "Freescale iMX6:CAN Module" /* Unique Driver Name for Installation                  */

                                                                /* ---------------- MAILBOX SETTINGS ------------------ */
#define  iMX6_CAN_DLC_LIMIT                  0x0Fu              /* Limit DLC to 8 Bytes                                 */
#define  iMX6_CAN_DLC_DATA_SPLIT                4u              /* DLC Value for Split between Data1 & Data2 Registers. */

                                                                /* --------------- INTERRUPT MASK & FLAG -------------- */
#define  iMX6_CAN_RX_FIFO_ISR_BITs           0xE0u              /* Rx FIFO ISR Bits can be Bits: 5, 6, or 7.            */
#define  iMX6_CAN_TX_MB_ISR_BITs           0x3F00u              /* Tx Mailbox ISR Bits can Be Bits: 8 -> 13.            */
#define  iMX6_CAN_RX_TX_ISR_TOT_BITs       (iMX6_CAN_TX_MB_ISR_BITs | iMX6_CAN_RX_FIFO_ISR_BITs)

                                                                /* ---------------- CAN FRAME SETTINGS ---------------- */
#define  iMX6_CAN_FRM_RTR_FLAG                  DEF_BIT_30
#define  iMX6_CAN_FRM_IDE_FLAG                  DEF_BIT_29


/*
*********************************************************************************************************
*                                        DEFAULT CONFIGURATION
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               MACROS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            DEVICE NAMES
*
* Description : Enumeration defines the Available Device Names for the Driver XXXCANInit() Function.
*
* Note(s)     : The Freescale iMX6 Series Driver only Supports 2 Internal CAN Controllers (1->2).
*********************************************************************************************************
*/

typedef  enum  imx6_can_dev {
    iMX6_CAN_BUS_1 = 0u,                                        /* Internal CAN controller #1                           */
    iMX6_CAN_BUS_2 = 1u,                                        /* Internal CAN controller #2                           */
    iMX6_CAN_N_DEV = 2u                                         /* Number of CAN controllers                            */
} iMX6_CAN_DEV;


/*
*********************************************************************************************************
*                                         DRIVER ERROR CODES
*
* Description : Enumeration defines the possible Driver Error Codes.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  enum  imx6_can_err {
    iMX6_CAN_ERR_NONE = 0u,                                     /* NO       ERR: Everything is OK                       */
    iMX6_CAN_ERR_BUS,                                           /* BUS      ERR: Wrong Bus Was Chosen                   */
    iMX6_CAN_ERR_BUSY,                                          /* BUSY     ERR: Msg Can't be Sent, Bus is Busy         */
    iMX6_CAN_ERR_INIT,                                          /* INIT     ERR: Reset State not Set, Dev Init Fail     */
    iMX6_CAN_ERR_MODE,                                          /* MODE     ERR: Error Accessing Wanted Mode on Device  */
    iMX6_CAN_ERR_OPEN,                                          /* OPEN     ERR: Device can't be Used, Device un-Opened */
    iMX6_CAN_ERR_CLOSE,                                         /* CLOSE    ERR: Device can't be Closed                 */
    iMX6_CAN_ERR_FUNC,                                          /* FUNCTION ERR: Given Function Code is not Valid       */
    iMX6_CAN_ERR_ARG,                                           /* ARGUMENT ERR: Argument Check has Failed              */
    iMX6_CAN_ERR_NO_DATA,                                       /* DATA     ERR: No Data is Available                   */
} iMX6_CAN_ERR;


/*
*********************************************************************************************************
*                                     I/O CONTROL FUNCTION CODES
*
* Description : Enumeration defines the available Function Codes for the Driver XXXIoCtl() Function.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  enum  imx6_can_io_list {
    IO_iMX6_CAN_GET_IDENT = 0u,                                 /* ---------- GET DRIVER IDENTIFICATION CODE ---------- */
                                                                /* arg = Pointer to Local Ident Variable (CPU_INT32U)   */
    IO_iMX6_CAN_GET_ERRNO,                                      /* --------------- GET DRIVER ERROR CODE -------------- */
                                                                /* arg = Pointer to Local Error Code Var. (CPU_INT16U)  */
    IO_iMX6_CAN_GET_DRVNAME,                                    /* ------------------ GET DRIVER NAME ----------------- */
                                                                /* arg = Pointer to Local String Variable (char)        */
    IO_iMX6_CAN_SET_BAUDRATE,                                   /* ----------------- SET BUS BAUDRATE ----------------- */
                                                                /* arg = Pointer to Local Baudrate Var. (CPU_INT32U)    */
    IO_iMX6_CAN_START,                                          /* -------------------- ENABLE BUS -------------------- */
                                                                /* No Pointer: Fnct Code sets CAN to Operational Mode.  */
    IO_iMX6_CAN_CONFIG,                                         /* ----------------- SET CAN TO CONFIG ---------------- */
                                                                /* No Pointer: Fnct Code sets CAN to 'CONFIG' Mode.     */
    IO_iMX6_CAN_RX_STANDARD,                                    /* ------- SET  RECIEVER TO STANDARD IDENTIFIER ------- */
                                                                /* No Pointer: CAN Rx recieves only CAN Standard IDs    */
    IO_iMX6_CAN_RX_EXTENDED,                                    /* ------- SET  RECIEVER TO EXTENDED IDENTIFIER ------- */
                                                                /* No Pointer: CAN Rx recieves only CAN Extended IDs    */
    IO_iMX6_CAN_TX_READY,                                       /* ---------------- GET TX READY STATUS --------------- */
                                                                /* arg = Pointer to TX Rdy Status Variable (CPU_INT08U) */
    IO_iMX6_CAN_GET_NODE_STATUS,                                /* ------------------ GET NODE STATUS ----------------- */
                                                                /* arg = Pointer to Node Status Variable (CPU_INT08U)   */
    IO_iMX6_CAN_SET_RX_FILTER,                                  /* ------------------ SET RX FILTER 1 ----------------- */
                                                                /* arg = CPU_INT32U[2]: arg[0] = mask, arg[1] = ID      */
    IO_iMX6_CAN_IO_FUNC_N                                       /* ------------- NUMBER OF FUNCTION CODES ------------- */
} iMX6_CAN_IO_LIST;                                             /* No Pointer: Holds number of Function Codes Available */


/*
*********************************************************************************************************
*                                           iMX6 CAN MODES
*
* Description : Enumeration defines the Freescale iMX6 Series Supported CAN Modes.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  enum  imx6_can_mode {
    iMX6_CAN_MODE_FREEZE = 0u,                                  /* CAN Controller Set To: Configure Mode / Freeze Mode. */
    iMX6_CAN_MODE_RESET,                                        /* CAN Controller Set To: Reset CAN Module.             */
    iMX6_CAN_MODE_NORMAL                                        /* CAN Controller Set To: Normal    Mode.               */
} iMX6_CAN_MODE;


/*
*********************************************************************************************************
*                                         DRIVER RUNTIME DATA
*
* Description : Structure holds the Driver Runtime data.
*
* Note(s)     : (1) 'MB_Ptr' is used to point to the CAN Mailbox Address Region located within the CAN's
*                   dedicated memory block in RAM. Since the CAN MB scheme is highly configurable, it's
*                   easier to have a dedicated structure 'iMX6_CAN_MB_STRUCT' for the Mailbox Configuration
*                   as well as a dedicated memory pointer.
*********************************************************************************************************
*/

typedef  struct  imx6_can_data {
    CPU_BOOLEAN          Use;                                   /* USE MARKER: Marker Indicating if Dev is In Use       */
    iMX6_CAN_REG        *RegPtr;                                /* REGISTER  : Pointer to the CAN Base Address(s)       */
    iMX6_CAN_MB_STRUCT  *MB_Ptr;                                /* MAILBOX   : Pointer to the CAN Mailbox Addr Struct.  */
} iMX6_CAN_DATA;


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

typedef  struct  imx6_can_frm {
    CPU_INT32U  Identifier;                                     /* CAN IDENTIFIER: Can Identifier                       */
    CPU_INT08U  Data[8u];                                       /* CAN PAYLOAD   : Bytes[Max 8] in Single CAN Msg       */
    CPU_INT08U  DLC;                                            /* CAN DLC       : Num of Valid Data(s) in Payload      */
    CPU_INT08U  Spare[3u];                                      /* SPARE         : Sets FRM w/ Integral Num of Pointers */
} iMX6_CAN_FRM;


/*
*********************************************************************************************************
*                                         CAN REG FRM STRUCT
*
* Description : Structure defines a iMX6 Rx & Tx Message Format.
*
* Note(s)     : Since all 4 registers [ID, DLC, Data1, Data2] are required to be written to / read
*               from regardless of the actual number of data bytes & valid fields in the message, a
*               structure is used as a 'middle man' to read from / write to specific parts of the
*               Rx / Tx message prior to receiving / sending.
*********************************************************************************************************
*/

typedef  struct  imx6_can_reg_frm {
    CPU_REG32  ID;                                              /* IDENTIFIER : CAN Rx / Tx ID     Register             */
    CPU_REG32  DLC;                                             /* DLC        : CAN Rx / Tx DLC    Register             */
    CPU_REG32  Data1;                                           /* DATA 1 REG : CAN Rx / Tx DATA 1 Register             */
    CPU_REG32  Data2;                                           /* DATA 2 REG : CAN Rx / Tx DATA 2 Register             */
} iMX6_CAN_REG_FRM;


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

CPU_INT16S  iMX6_CAN_Init (CPU_INT32U   para_id);

CPU_INT16S  iMX6_CAN_Open (CPU_INT16S   dev_id,
                           CPU_INT32U   dev_name,
                           CPU_INT16U   mode);

CPU_INT16S  iMX6_CAN_Close(CPU_INT16S   para_id);

CPU_INT16S  iMX6_CAN_IoCtl(CPU_INT16S   para_id,
                           CPU_INT16U   func,
                           void        *p_arg);

CPU_INT16S  iMX6_CAN_Read (CPU_INT16S   para_id,
                           CPU_INT08U  *buf,
                           CPU_INT16U   size);

CPU_INT16S  iMX6_CAN_Write(CPU_INT16S   para_id,
                           CPU_INT08U  *buf,
                           CPU_INT16U   size);


/*
*********************************************************************************************************
*                                            ERROR SECTION
*********************************************************************************************************
*/

#endif                                                          /* #ifndef _DRV_CAN_H_                                  */
