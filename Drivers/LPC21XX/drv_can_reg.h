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

#ifndef _DRVCANREG_H
#define _DRVCANREG_H

#include "cpu.h"

/* Vectored Interrupt Controller (VIC) */
#define LPC21XX_CAN_VICINTENABLE   (*((volatile CPU_INT32U *) 0xFFFFF010))
#define LPC21XX_CAN_VICINTENCLR    (*((volatile CPU_INT32U *) 0xFFFFF014))
#define LPC21XX_CAN_VICVECTADDR    (*((volatile CPU_INT32U *) 0xFFFFF030))
#define LPC21XX_CAN_VICINTSELECT   (*((volatile CPU_INT32U *) 0xFFFFF00C))

#define LPC21XX_CAN_VICVECTADDR4   (*((volatile CPU_INT32U *) 0xFFFFF110))
#define LPC21XX_CAN_VICVECTADDR5   (*((volatile CPU_INT32U *) 0xFFFFF114))
#define LPC21XX_CAN_VICVECTADDR6   (*((volatile CPU_INT32U *) 0xFFFFF118))
#define LPC21XX_CAN_VICVECTADDR7   (*((volatile CPU_INT32U *) 0xFFFFF11C))
#define LPC21XX_CAN_VICVECTADDR8   (*((volatile CPU_INT32U *) 0xFFFFF120))
#define LPC21XX_CAN_VICVECTADDR9   (*((volatile CPU_INT32U *) 0xFFFFF124))

#define LPC21XX_CAN_VICVECTCTRL4   (*((volatile CPU_INT32U *) 0xFFFFF210))
#define LPC21XX_CAN_VICVECTCTRL5   (*((volatile CPU_INT32U *) 0xFFFFF214))
#define LPC21XX_CAN_VICVECTCTRL6   (*((volatile CPU_INT32U *) 0xFFFFF218))
#define LPC21XX_CAN_VICVECTCTRL7   (*((volatile CPU_INT32U *) 0xFFFFF21C))
#define LPC21XX_CAN_VICVECTCTRL8   (*((volatile CPU_INT32U *) 0xFFFFF220))
#define LPC21XX_CAN_VICVECTCTRL9   (*((volatile CPU_INT32U *) 0xFFFFF224))

/* Pin Connect Block */
#define LPC21XX_CAN_PINSEL1        (*((volatile CPU_INT32U *) 0xE002C004))

/* CAN Acceptance Filter */
#define LPC21XX_CAN_AFMR           (*((volatile CPU_INT32U *) 0xE003C000))
#define LPC21XX_CAN_SFF_SA         (*((volatile CPU_INT32U *) 0xE003C004))
#define LPC21XX_CAN_SFF_GRP_SA     (*((volatile CPU_INT32U *) 0xE003C008))
#define LPC21XX_CAN_EFF_SA         (*((volatile CPU_INT32U *) 0xE003C00C))
#define LPC21XX_CAN_EFF_GRP_SA     (*((volatile CPU_INT32U *) 0xE003C010))
#define LPC21XX_CAN_END_OF_TABLE   (*((volatile CPU_INT32U *) 0xE003C014))

/* CAN Central Registers */
#define LPC21XX_CAN_CANTXSR        (*((volatile CPU_INT32U *) 0xE0040000))
#define LPC21XX_CAN_CANRXSR        (*((volatile CPU_INT32U *) 0xE0040004))
#define LPC21XX_CAN_CANMSR         (*((volatile CPU_INT32U *) 0xE0040008))

/* CAN Controller 1 (CAN1) */
#define LPC21XX_CAN_C1MOD          (*((volatile CPU_INT32U *) 0xE0044000))
#define LPC21XX_CAN_C1CMR          (*((volatile CPU_INT32U *) 0xE0044004))
#define LPC21XX_CAN_C1GSR          (*((volatile CPU_INT32U *) 0xE0044008))
#define LPC21XX_CAN_C1ICR          (*((volatile CPU_INT32U *) 0xE004400C))
#define LPC21XX_CAN_C1IER          (*((volatile CPU_INT32U *) 0xE0044010))
#define LPC21XX_CAN_C1BTR          (*((volatile CPU_INT32U *) 0xE0044014))
#define LPC21XX_CAN_C1EWL          (*((volatile CPU_INT32U *) 0xE0044018))
#define LPC21XX_CAN_C1SR           (*((volatile CPU_INT32U *) 0xE004401C))
#define LPC21XX_CAN_C1RFS          (*((volatile CPU_INT32U *) 0xE0044020))
#define LPC21XX_CAN_C1RID          (*((volatile CPU_INT32U *) 0xE0044024))
#define LPC21XX_CAN_C1RDA          (*((volatile CPU_INT32U *) 0xE0044028))
#define LPC21XX_CAN_C1RDB          (*((volatile CPU_INT32U *) 0xE004402C))
#define LPC21XX_CAN_C1TFI1         (*((volatile CPU_INT32U *) 0xE0044030))
#define LPC21XX_CAN_C1TID1         (*((volatile CPU_INT32U *) 0xE0044034))
#define LPC21XX_CAN_C1TDA1         (*((volatile CPU_INT32U *) 0xE0044038))
#define LPC21XX_CAN_C1TDB1         (*((volatile CPU_INT32U *) 0xE004403C))
#define LPC21XX_CAN_C1TFI2         (*((volatile CPU_INT32U *) 0xE0044040))
#define LPC21XX_CAN_C1TID2         (*((volatile CPU_INT32U *) 0xE0044044))
#define LPC21XX_CAN_C1TDA2         (*((volatile CPU_INT32U *) 0xE0044048))
#define LPC21XX_CAN_C1TDB2         (*((volatile CPU_INT32U *) 0xE004404C))
#define LPC21XX_CAN_C1TFI3         (*((volatile CPU_INT32U *) 0xE0044050))
#define LPC21XX_CAN_C1TID3         (*((volatile CPU_INT32U *) 0xE0044054))
#define LPC21XX_CAN_C1TDA3         (*((volatile CPU_INT32U *) 0xE0044058))
#define LPC21XX_CAN_C1TDB3         (*((volatile CPU_INT32U *) 0xE004405C))

/* CAN Controller 2 (CAN2) */
#define LPC21XX_CAN_C2MOD          (*((volatile CPU_INT32U *) 0xE0048000))
#define LPC21XX_CAN_C2CMR          (*((volatile CPU_INT32U *) 0xE0048004))
#define LPC21XX_CAN_C2GSR          (*((volatile CPU_INT32U *) 0xE0048008))
#define LPC21XX_CAN_C2ICR          (*((volatile CPU_INT32U *) 0xE004800C))
#define LPC21XX_CAN_C2IER          (*((volatile CPU_INT32U *) 0xE0048010))
#define LPC21XX_CAN_C2BTR          (*((volatile CPU_INT32U *) 0xE0048014))
#define LPC21XX_CAN_C2EWL          (*((volatile CPU_INT32U *) 0xE0048018))
#define LPC21XX_CAN_C2SR           (*((volatile CPU_INT32U *) 0xE004801C))
#define LPC21XX_CAN_C2RFS          (*((volatile CPU_INT32U *) 0xE0048020))
#define LPC21XX_CAN_C2RID          (*((volatile CPU_INT32U *) 0xE0048024))
#define LPC21XX_CAN_C2RDA          (*((volatile CPU_INT32U *) 0xE0048028))
#define LPC21XX_CAN_C2RDB          (*((volatile CPU_INT32U *) 0xE004802C))
#define LPC21XX_CAN_C2TFI1         (*((volatile CPU_INT32U *) 0xE0048030))
#define LPC21XX_CAN_C2TID1         (*((volatile CPU_INT32U *) 0xE0048034))
#define LPC21XX_CAN_C2TDA1         (*((volatile CPU_INT32U *) 0xE0048038))
#define LPC21XX_CAN_C2TDB1         (*((volatile CPU_INT32U *) 0xE004803C))
#define LPC21XX_CAN_C2TFI2         (*((volatile CPU_INT32U *) 0xE0048040))
#define LPC21XX_CAN_C2TID2         (*((volatile CPU_INT32U *) 0xE0048044))
#define LPC21XX_CAN_C2TDA2         (*((volatile CPU_INT32U *) 0xE0048048))
#define LPC21XX_CAN_C2TDB2         (*((volatile CPU_INT32U *) 0xE004804C))
#define LPC21XX_CAN_C2TFI3         (*((volatile CPU_INT32U *) 0xE0048050))
#define LPC21XX_CAN_C2TID3         (*((volatile CPU_INT32U *) 0xE0048054))
#define LPC21XX_CAN_C2TDA3         (*((volatile CPU_INT32U *) 0xE0048058))
#define LPC21XX_CAN_C2TDB3         (*((volatile CPU_INT32U *) 0xE004805C))

/* CAN Int Enable Register Bit definitions */
#define LPC21XX_CAN_IER_RX         0x00000001        /* RIE             */
#define LPC21XX_CAN_IER_TX         0x00000602        /* TIE1-3          */
#define LPC21XX_CAN_IER_NS         0x000000A4        /* EIE, EPIE, BEIE */

/* CAN Interrupt and Capture Register Bit definitions */
#define LPC21XX_CAN_ICR_RI         0x00000001
#define LPC21XX_CAN_ICR_TI1        0x00000002
#define LPC21XX_CAN_ICR_EI         0x00000004
#define LPC21XX_CAN_ICR_DOI        0x00000008
#define LPC21XX_CAN_ICR_WUI        0x00000010
#define LPC21XX_CAN_ICR_EPI        0x00000020
#define LPC21XX_CAN_ICR_ALI        0x00000040
#define LPC21XX_CAN_ICR_BEI        0x00000080
#define LPC21XX_CAN_ICR_IDI        0x00000100
#define LPC21XX_CAN_ICR_TI2        0x00000200
#define LPC21XX_CAN_ICR_TI3        0x00000400

/* CAN frame status register Bit definitions */
#define LPC21XX_CAN_DLC_MASK               0x000F0000L
#define LPC21XX_CAN_RTR_MASK               0x40000000L
#define LPC21XX_CAN_FF_MASK                0x80000000L

#endif
