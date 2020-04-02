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

#ifndef _TMS28XX_DRV_CAN_REG_H_
#define _TMS28XX_DRV_CAN_REG_H_

/*
****************************************************************************************************
*                                             INCLUDES
****************************************************************************************************
*/
#include "cpu.h"                                      /* cpu declarations                         */

/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ECANA CONTROL AND STATUS REGISTER BASE ADDRESS
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Base Address of the eCANA Control and Status Register Base
*           Address.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECANA_BASE_ADDRESS       0x006000

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ECANB CONTROL AND STATUS REGISTER BASE ADDRESS
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Base Address of the eCANB Control and Status Register Base
*           Address.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECANB_BASE_ADDRESS       0x006200

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ECAN LOCAL ACCEPTANCE MASK ADDRESS OFFSET
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Address Offset of the eCAN Local Acceptance Mask.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECAN_LAM_OFFS_ADDRESS    0x000040

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ECAN MAILBOX REGISTERS ADDRESS OFFESET
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Address Offeset of the eCAN Mailbox Registers.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECAN_MBOX_OFFS_ADDRESS   0x000100

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                       ECAN RX MESSAGE NUMBER
* \ingroup  TMS28XX_ECAN
*
*          This Constant holds the Number of the used Receive Message Buffer.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS_28XX_ECAN_RX_BUFFER          16

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                       ECAN TX MESSAGE NUMBER
* \ingroup  TMS28XX_ECAN
*
*          This Constant holds the Number of the used Transmit Message Buffer.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS_28XX_ECAN_TX_BUFFER          0L

#if (TMS_28XX_ECAN_RX_BUFFER == TMS_28XX_ECAN_TX_BUFFER)
#error: "Not Allowed! RX and TX Buffer must be different!"
#endif

/*
****************************************************************************************************
*                                            DATA TYPES
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ECAN CONTROL & STATUS REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This structure holds the eCAN Control and Status registers.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct  {
   volatile CPU_INT32U  CANME;                        /*!< Mailbox Enable                         */
   volatile CPU_INT32U  CANMD;                        /*!< Mailbox Direction                      */
   volatile CPU_INT32U  CANTRS;                       /*!< Transmit Request Set                   */
   volatile CPU_INT32U  CANTRR;                       /*!< Transmit Request Reset                 */
   volatile CPU_INT32U  CANTA;                        /*!< Transmit Acknowledge                   */
   volatile CPU_INT32U  CANAA;                        /*!< Abort Acknowledge                      */
   volatile CPU_INT32U  CANRMP;                       /*!< Received Message Pending               */
   volatile CPU_INT32U  CANRML;                       /*!< Received Message Lost                  */
   volatile CPU_INT32U  CANRFP;                       /*!< Remote Frame Pending                   */
   volatile CPU_INT32U  CANGAM;                       /*!< Global Acceptance Mask                 */
   volatile CPU_INT32U  CANMC;                        /*!< Master Control                         */
   volatile CPU_INT32U  CANBTC;                       /*!< Bit Timing                             */
   volatile CPU_INT32U  CANES;                        /*!< Error Status                           */
   volatile CPU_INT32U  CANTEC;                       /*!< Transmit Error Counter                 */
   volatile CPU_INT32U  CANREC;                       /*!< Receive Error Counter                  */
   volatile CPU_INT32U  CANGIF0;                      /*!< Global Interrupt Flag 0                */
   volatile CPU_INT32U  CANGIM;                       /*!< Global Interrupt Mask 0                */
   volatile CPU_INT32U  CANGIF1;                      /*!< Global Interrupt Flag 1                */
   volatile CPU_INT32U  CANMIM;                       /*!< Mailbox Interrupt Mask                 */
   volatile CPU_INT32U  CANMIL;                       /*!< Mailbox Interrupt Level                */
   volatile CPU_INT32U  CANOPC;                       /*!< Overwrite Protection Control           */
   volatile CPU_INT32U  CANTIOC;                      /*!< TX I/O Control                         */
   volatile CPU_INT32U  CANRIOC;                      /*!< RX I/O Control                         */
   volatile CPU_INT32U  CANTSC;                       /*!< Time-stamp counter                     */
   volatile CPU_INT32U  CANTOC;                       /*!< Time-out Control                       */
   volatile CPU_INT32U  CANTOS;                       /*!< Time-out Status                        */

}TMS28XX_ECAN_REGS;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ECAN MAILBOX REGISTERS
* \ingroup  TMS28XX_ECAN
*
*           This structure holds the eCAN Mailbox Registers.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct {
   volatile CPU_INT32U MSGID;                         /*!< eCAN Message ID (MSGID)                */
   volatile CPU_INT32U MSGCTRL;                       /*!< eCAN Message Control Field             */
                                                      /* (MSGCTRL)                                */
   volatile CPU_INT32U MDL;                           /*!< eCAN Message Data Register low         */
                                                      /* (MDR_H)                                  */
   volatile CPU_INT32U MDH;                           /*!< eCAN Message Data Register high        */
                                                      /* (MDR_H)                                  */
}TMS28XX_ECAN_MBOX;


typedef interrupt void(*ECANINT)(void);



#endif  /* #ifndef _TMS28XX_DRV_CAN_REG_H_ */
