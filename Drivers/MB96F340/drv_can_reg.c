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
* Filename : drv_can_reg.c
* Version  : V2.42.01
****************************************************************************************************
*/

/*
****************************************************************************************************
*                                             INCLUDES
****************************************************************************************************
*/
#include "drv_can.h"                                  /* MB96F340 CAN driver declarations         */


/*
****************************************************************************************************
*                                             GLOBAL DATA
****************************************************************************************************
*/

#if MB96F340_CAN_DEV_N == 1

#if MB96F340_CAN_SELECT_CFG == 0                      /*=== CHECK FOR SELECTED BUS: CAN 0 ========*/

#pragma asm
         .SECTION CAN_REG, DATA, LOCATE=0x0700
         .GLOBAL _CanReg
_CanReg: .RES.B  16

#pragma endasm

#elif MB96F340_CAN_SELECT_CFG == 1                    /*=== CHECK FOR SELECTED BUS: CAN 1 ========*/

#pragma asm
         .SECTION CAN_REG, DATA, LOCATE=0x0800
         .GLOBAL _CanReg
_CanReg: .RES.B  16

#pragma endasm

#elif MB96F340_CAN_SELECT_CFG == 2                    /*=== CHECK FOR SELECTED BUS: CAN 2 ========*/

#pragma asm
         .SECTION CAN_REG, DATA, LOCATE=0x0900
         .GLOBAL _CanReg
_CanReg: .RES.B  16

#pragma endasm

#elif MB96F340_CAN_SELECT_CFG == 3                    /*=== CHECK FOR SELECTED BUS: CAN 3 ========*/

#pragma asm
         .SECTION CAN_REG, DATA, LOCATE=0x0A00
         .GLOBAL _CanReg
_CanReg: .RES.B  16

#pragma endasm

#elif MB96F340_CAN_SELECT_CFG == 4                    /*=== CHECK FOR SELECTED BUS: CAN 4 ========*/

#pragma asm
         .SECTION CAN_REG, DATA, LOCATE=0x0B00
         .GLOBAL _CanReg
_CanReg: .RES.B  16

#pragma endasm

#endif                                                /*==========================================*/

#endif  /* #if MB96F340_CAN_DEV_N == 1 */
