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
#include "cpu.h"                                      /* CPU configuration                        */

/*
****************************************************************************************************
*                                            DATA TYPES
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                          OVERALL CAN REGISTERS
* \ingroup  MB96F340_CAN
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct {
    CPU_INT16U CTRLR;                                 /*!< CAN Control Register +0x0              */
    CPU_INT16U STATR;                                 /*!< CAN Status Register  +0x2              */
    CPU_INT08U ERRCNTL;                               /*!< CAN Error Counter (Transmit) +0x4      */
    CPU_INT08U ERRCNTH;                               /*!< CAN Error Counter (Receive) +0x5       */
    CPU_INT16U BTR;                                   /*!< CAN BittTiming Register +0x6           */
    CPU_INT16U INTR;                                  /*!< CAN Interrupt Register +0x8            */
    CPU_INT16U TESTR;                                 /*!< CAN Test Register +0xA                 */
    CPU_INT16U BRPER;                                 /*!< CAN BRP Extension Register +0xC        */
    CPU_INT16U _r1;                                   /*!< reserved +0xE                          */
    CPU_INT16U IF1CREQ;                               /*!< CAN IF1 Command Request Register +0x10 */
    CPU_INT16U IF1CMSK;                               /*!< CAN IF1 Command Mask Register +0x12    */
    CPU_INT16U IF1MSK1;                               /*!< CAN IF1 Mask Register +0x14            */
    CPU_INT16U IF1MSK2;                               /*!< CAN IF1 Mask Register +0x16            */
    CPU_INT16U IF1ARB1;                               /*!< CAN IF1 Arbitration register +0x18     */
    CPU_INT16U IF1ARB2;                               /*!< CAN IF1 Arbitration register +0x1A     */
    CPU_INT16U IF1MCTR;                               /*!< CAN IF1 Message Control Register +0x1C */
    CPU_INT16U IF1DTA1;                               /*!< CAN IF1 Data A1 +0x1E                  */
    CPU_INT16U IF1DTA2;                               /*!< CAN IF1 Data A2 +0x20                  */
    CPU_INT16U IF1DTB1;                               /*!< CAN IF1 Data B1 +0x22                  */
    CPU_INT16U IF1DTB2;                               /*!< CAN IF1 Data B2 +0x24                  */

    CPU_INT16U _r2[0xD];                              /*!< reserved +0x26 .. 0x3E                 */

    CPU_INT16U IF2CREQ;                               /*!< CAN IF2 Command Request Register +0x40 */
    CPU_INT16U IF2CMSK;                               /*!< CAN IF2 Command Mask Register +0x42    */
    CPU_INT16U IF2MSK1;                               /*!< CAN IF2 Mask Register +0x44            */
    CPU_INT16U IF2MSK2;                               /*!< CAN IF2 Mask Register +0x46            */
    CPU_INT16U IF2ARB1;                               /*!< CAN IF2 Arbitration register +0x48     */
    CPU_INT16U IF2ARB2;                               /*!< CAN IF2 Arbitration register +0x4A     */
    CPU_INT16U IF2MCTR;                               /*!< CAN IF2 Message Control Register +0x4C */
    CPU_INT16U IF2DTA1;                               /*!< CAN IF2 Data A1 +0x4E                  */
    CPU_INT16U IF2DTA2;                               /*!< CAN IF2 Data A2 +0x50                  */
    CPU_INT16U IF2DTB1;                               /*!< CAN IF2 Data B1 +0x52                  */
    CPU_INT16U IF2DTB2;                               /*!< CAN IF1 Data B2 +0x54                  */

    CPU_INT16U _r3[0x15];                             /*!< reserved +0x56 .. 0x7E                 */

    CPU_INT16U TREQR1;                                /*!< CAN Transmission Request Register +0x80 */
    CPU_INT16U TREQR2;                                /*!< CAN Transmission Request Register +0x82 */

    CPU_INT16U _r4[0x6];                              /*!< reserved +0x84 .. 0x8E                 */

    CPU_INT16U NEWDT1;                                /*!< CAN New Data Register +0x90            */
    CPU_INT16U NEWDT2;                                /*!< CAN New Data Register +0x92            */

    CPU_INT16U _r5[0x6];                              /*!< reserved +0x94 .. 0x9E                 */

    CPU_INT16U INTPND1;                               /*!< CAN Interrupt Pending Register +0xA0   */
    CPU_INT16U INTPND2;                               /*!< CAN Interrupt Pending Register +0xA2   */

    CPU_INT16U _r6[0x6];                              /*!< reserved +0xA4 .. 0xAE                 */

    CPU_INT16U MSGVAL1;                               /*!< CAN Message Valid Register +0xB0       */
    CPU_INT16U MSGVAL2;                               /*!< CAN Message Valid Register +0xB2       */

    CPU_INT16U _r7[0xD];                              /*!< reserved +0xB4 .. 0xCC                 */

    CPU_INT16U COER;                                  /*!< CAN Output enable register +0xCE       */




} CAN_REG;


#endif  /* #ifndef _DRV_CAN_REG_H_ */

