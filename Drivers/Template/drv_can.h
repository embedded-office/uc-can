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
*                                              Template
*
* Filename : drv_can.h
* Version  : V2.42.01
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

#include  "lib_def.h"
#include  "can_bus.h"
#include  "cpu.h"


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

#define  <Drv_Name>_CAN_NAME        "<Drv_Name>:CAN Module"     /* Unique Driver Name for Installation                  */


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
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  enum  <drv_name>_can_dev {
    <Drv_Name>_CAN_BUS_0 = 0u,                                  /* Internal CAN controller #0                           */
    <Drv_Name>_CAN_BUS_1 = 1u,                                  /* Internal CAN controller #1                           */
    <Drv_Name>_CAN_N_DEV = 2u                                   /* Number of CAN controllers                            */
} <Drv_Name>_CAN_DEV;


/*
*********************************************************************************************************
*                                         DRIVER ERROR CODES
*
* Description : Enumeration defines the possible Driver Error Codes.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  enum  <drv_name>_can_err {
    <Drv_Name>_CAN_ERR_NONE = 0u,                               /* NO       ERR: Everything is OK                       */
    <Drv_Name>_CAN_ERR_BUS,                                     /* BUS      ERR: Wrong Bus Was Chosen                   */
    <Drv_Name>_CAN_ERR_BUSY,                                    /* BUSY     ERR: Msg Can't be Sent, Bus is Busy         */
    <Drv_Name>_CAN_ERR_INIT,                                    /* INIT     ERR: Reset State not Set, Dev Init Fail     */
    <Drv_Name>_CAN_ERR_MODE,                                    /* MODE     ERR: Error Accessing Wanted Mode on Device  */
    <Drv_Name>_CAN_ERR_OPEN,                                    /* OPEN     ERR: Device can't be Used, Device un-Opened */
    <Drv_Name>_CAN_ERR_CLOSE,                                   /* CLOSE    ERR: Device can't be Closed                 */
    <Drv_Name>_CAN_ERR_FUNC,                                    /* FUNCTION ERR: Given Function Code is not Valid       */
    <Drv_Name>_CAN_ERR_ARG,                                     /* ARGUMENT ERR: Argument Check has Failed              */
    <Drv_Name>_CAN_ERR_NO_DATA,                                 /* DATA     ERR: No Data is Available                   */
} <Drv_Name>_CAN_ERR;


/*
*********************************************************************************************************
*                                     I/O CONTROL FUNCTION CODES
*
* Description : Enumeration defines the available Function Codes for the Driver XXXIoCtl() Function.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  enum  <drv_name>_can_io_list {
    IO_<Drv_Name>_CAN_GET_IDENT = 0u,                           /* ---------- GET DRIVER IDENTIFICATION CODE ---------- */
                                                                /* arg = Pointer to Local Ident Variable (CPU_INT32U)   */
    IO_<Drv_Name>_CAN_GET_ERRNO,                                /* --------------- GET DRIVER ERROR CODE -------------- */
                                                                /* arg = Pointer to Local Error Code Var. (CPU_INT16U)  */
    IO_<Drv_Name>_CAN_GET_DRVNAME,                              /* ------------------ GET DRIVER NAME ----------------- */
                                                                /* arg = Pointer to Local String Variable (char)        */
    IO_<Drv_Name>_CAN_SET_BAUDRATE,                             /* ----------------- SET BUS BAUDRATE ----------------- */
                                                                /* arg = Pointer to Local Baudrate Var. (CPU_INT32U)    */
    IO_<Drv_Name>_CAN_START,                                    /* -------------------- ENABLE BUS -------------------- */
                                                                /* No Pointer: Fnct Code sets CAN to Operational Mode.  */
    IO_<Drv_Name>_CAN_STOP,                                     /* ------------------ SET CAN TO STOP ----------------- */
                                                                /* No Pointer: Fnct Code sets CAN to 'STOP' Mode.       */
    IO_<Drv_Name>_CAN_RX_STANDARD,                              /* ------- SET  RECIEVER TO STANDARD IDENTIFIER ------- */
                                                                /* No Pointer: CAN Rx recieves only CAN Standard IDs    */
    IO_<Drv_Name>_CAN_RX_EXTENDED,                              /* ------- SET  RECIEVER TO EXTENDED IDENTIFIER ------- */
                                                                /* No Pointer: CAN Rx recieves only CAN Extended IDs    */
    IO_<Drv_Name>_CAN_TX_READY,                                 /* ---------------- GET TX READY STATUS --------------- */
                                                                /* arg = Pointer to TX Rdy Status Variable (CPU_INT08U) */
    IO_<Drv_Name>_CAN_GET_NODE_STATUS,                          /* ------------------ GET NODE STATUS ----------------- */
                                                                /* arg = Pointer to Node Status Variable (CPU_INT08U)   */
    IO_<Drv_Name>_CAN_SET_RX_FILTER,                            /* ------------------- SET RX FILTER ------------------ */
                                                                /* arg = Based on Filter Register requirements.         */
    IO_<Drv_Name>_CAN_IO_FUNC_N                                 /* ------------- NUMBER OF FUNCTION CODES ------------- */
} <Drv_Name>_CAN_IO_LIST;                                       /* No Pointer: Holds number of Function Codes Available */


/*
*********************************************************************************************************
*                                        <Drv_Name> CAN MODES
*
* Description : Enumeration defines the <Drv_Name> Series CAN Modes.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  enum  <drv_name>_can_mode {
    <Drv_Name>_CAN_MODE_<Mode> = 0u,
} <Drv_Name>_CAN_MODE;


/*
*********************************************************************************************************
*                                         DRIVER RUNTIME DATA
*
* Description : Structure holds the Driver Runtime data.
*
* Note(s)     : none.
*********************************************************************************************************
*/

typedef  struct  <drv_name>_can_data {
    CPU_BOOLEAN          Use;                                   /* USE MARKER: Marker Indicating if Dev is In Use       */
    <Drv_Name>_CAN_REG  *RegPtr;                                /* REGISTER  : Pointer to the CAN Base Address(s)       */
} <Drv_Name>_CAN_DATA;


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

typedef  struct  <drv_name>_can_frm {
    CPU_INT32U  Identifier;                                     /* CAN IDENTIFIER: Can Identifier                       */
    CPU_INT08U  Data[8u];                                       /* CAN PAYLOAD   : Bytes[Max 8] in Single CAN Msg       */
    CPU_INT08U  DLC;                                            /* CAN DLC       : Num of Valid Data(s) in Payload      */
    CPU_INT08U  Spare[3u];                                      /* SPARE         : Sets FRM w/ Integral Num of Pointers */
} <Drv_Name>_CAN_FRM;


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

CPU_INT16S  <Drv_Name>_CAN_Init    (CPU_INT32U   para_id);

CPU_INT16S  <Drv_Name>_CAN_Open    (CPU_INT16S   dev_id,
                                    CPU_INT32U   dev_name,
                                    CPU_INT16U   mode);

CPU_INT16S  <Drv_Name>_CAN_Close   (CPU_INT16S   para_id);

CPU_INT16S  <Drv_Name>_CAN_IoCtl   (CPU_INT16S   para_id,
                                    CPU_INT16U   func,
                                    void        *p_arg);

CPU_INT16S  <Drv_Name>_CAN_Read    (CPU_INT16S   para_id,
                                    CPU_INT08U  *buf,
                                    CPU_INT16U   size);

CPU_INT16S  <Drv_Name>_CAN_Write   (CPU_INT16S   para_id,
                                    CPU_INT08U  *buf,
                                    CPU_INT16U   size);

void        <Drv_Name>_CAN_ErrCheck(CPU_INT32U   para_id);


/*
*********************************************************************************************************
*                                            ERROR SECTION
*********************************************************************************************************
*/

#endif                                                          /* #ifndef _DRV_CAN_H_                                  */
