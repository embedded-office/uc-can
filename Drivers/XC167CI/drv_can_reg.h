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
/*
****************************************************************************************************
*                                         NODE A REGISTERS
****************************************************************************************************
*/
#define XC167_CAN_A_CR                (*(volatile CPU_INT16U *) 0x200200L)
#define XC167_CAN_A_SR                (*(volatile CPU_INT16U *) 0x200204L)
#define XC167_CAN_A_IR                (*(volatile CPU_INT16U *) 0x200208L)
#define XC167_CAN_A_BTRL              (*(volatile CPU_INT16U *) 0x20020CL)
#define XC167_CAN_A_BTRH              (*(volatile CPU_INT16U *) 0x20020EL)
#define XC167_CAN_A_GINP              (*(volatile CPU_INT16U *) 0x200210L)
#define XC167_CAN_A_FCRL              (*(volatile CPU_INT16U *) 0x200214L)
#define XC167_CAN_A_FCRH              (*(volatile CPU_INT16U *) 0x200216L)
#define XC167_CAN_A_IMRL0             (*(volatile CPU_INT16U *) 0x200218L)
#define XC167_CAN_A_IMRH0             (*(volatile CPU_INT16U *) 0x20021AL)
#define XC167_CAN_A_IMR4              (*(volatile CPU_INT16U *) 0x20021CL)
#define XC167_CAN_A_ECNTL             (*(volatile CPU_INT16U *) 0x200220L)
#define XC167_CAN_A_ECNTH             (*(volatile CPU_INT16U *) 0x200222L)

/*
****************************************************************************************************
*                                         NODE B REGISTERS
****************************************************************************************************
*/
#define XC167_CAN_B_CR                (*(volatile CPU_INT16U *) 0x200240L)
#define XC167_CAN_B_SR                (*(volatile CPU_INT16U *) 0x200244L)
#define XC167_CAN_B_IR                (*(volatile CPU_INT16U *) 0x200248L)
#define XC167_CAN_B_BTRL              (*(volatile CPU_INT16U *) 0x20024CL)
#define XC167_CAN_B_BTRH              (*(volatile CPU_INT16U *) 0x20024EL)
#define XC167_CAN_B_GINP              (*(volatile CPU_INT16U *) 0x200250L)
#define XC167_CAN_B_FCRL              (*(volatile CPU_INT16U *) 0x200254L)
#define XC167_CAN_B_FCRH              (*(volatile CPU_INT16U *) 0x200256L)
#define XC167_CAN_B_IMRL0             (*(volatile CPU_INT16U *) 0x200258L)
#define XC167_CAN_B_IMRH0             (*(volatile CPU_INT16U *) 0x20025AL)
#define XC167_CAN_B_IMR4              (*(volatile CPU_INT16U *) 0x20025CL)
#define XC167_CAN_B_ECNTL             (*(volatile CPU_INT16U *) 0x200250L)
#define XC167_CAN_B_ECNTH             (*(volatile CPU_INT16U *) 0x200252L)

/*
****************************************************************************************************
*                                     GLOBLA CONTROL REGISTERS
****************************************************************************************************
*/
#define XC167_CAN_RXIPNDL             (*(volatile CPU_INT16U *) 0x200284L)
#define XC167_CAN_RXIPNDH             (*(volatile CPU_INT16U *) 0x200286L)
#define XC167_CAN_TXIPNDL             (*(volatile CPU_INT16U *) 0x200288L)
#define XC167_CAN_TXIPNDH             (*(volatile CPU_INT16U *) 0x20028AL)

/*
****************************************************************************************************
*                                    MESSAGE OBJECTS 0 REGISTERS
****************************************************************************************************
*/
#define XC167_CAN_MSG0_BASEADDRESS    (volatile CPU_INT16U *) 0x200300L

/*
****************************************************************************************************
*                                    INTERRUPT CONTROL REGISTERS
****************************************************************************************************
*/
#define XC167_CAN_0IC                 (* (volatile CPU_INT16U *) 0x00F196L)
#define XC167_CAN_1IC                 (* (volatile CPU_INT16U *) 0x00F142L)
#define XC167_CAN_2IC                 (* (volatile CPU_INT16U *) 0x00F144L)
#define XC167_CAN_3IC                 (* (volatile CPU_INT16U *) 0x00F146L)
#define XC167_CAN_4IC                 (* (volatile CPU_INT16U *) 0x00F148L)
#define XC167_CAN_5IC                 (* (volatile CPU_INT16U *) 0x00F14AL)
#define XC167_CAN_6IC                 (* (volatile CPU_INT16U *) 0x00F14CL)
#define XC167_CAN_7IC                 (* (volatile CPU_INT16U *) 0x00F14EL)

#define XC167_ALTSEL0P9               (* (volatile CPU_INT16U *) 0xF138L)
#define XC167_ALTSEL1P9               (* (volatile CPU_INT16U *) 0xF13AL)

#define XC167_DP9                     (* (volatile CPU_INT16U *) 0xFF18L)


/*
****************************************************************************************************
*                                              MACROS
****************************************************************************************************
*/

/*
****************************************************************************************************
*                                            DATA TYPES
****************************************************************************************************
*/
typedef volatile struct MsgBOX_tag
{
     CPU_INT16U MSGDRL0;
     CPU_INT16U MSGDRH0;
     CPU_INT16U MSGDRL4;
     CPU_INT16U MSGDRH4;
     CPU_INT16U MSGARL;
     CPU_INT16U MSGARH;
     CPU_INT16U MSGAMRL;
     CPU_INT16U MSGAMRH;
     CPU_INT16U MSGCTRL;
     CPU_INT16U MSGCTRH;
     CPU_INT16U MSGCFGL;
     CPU_INT16U MSGCFGH;
     CPU_INT16U MSGFGCRL;
     CPU_INT16U MSGFGCRH;
     CPU_INT32U Spare;
} MsgBOX_t;

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

