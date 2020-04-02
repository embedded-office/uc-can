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

#ifndef _DRVSTM32F10X_CANREG_H
#define _DRVSTM32F10X_CANREG_H

#include "cpu.h"


/*
****************************************************************************************************
*                                              DEFINES
****************************************************************************************************
*/

#define FCN0_CAN_BASE                    0xFF480000
#define FCN1_CAN_BASE                    0xFF4A0000
#define FCN2_CAN_BASE                    0xFF4C0000
#define FCN3_CAN_BASE                    0xFF4E0000
#define FCN4_CAN_BASE                    0xFF500000
#define DCN0_CAN_BASE                    0xFF520000

#define FCN0_NUMBER_MB                    64
#define FCN1_NUMBER_MB                    64
#define FCN2_NUMBER_MB                    64
#define FCN3_NUMBER_MB                    128
#define FCN4_NUMBER_MB                    128
#define DCN0_NUMBER_MB                    128


/* V850E2-Fx4 Clock domain registers */
#define CKSC_113_ADDR      (CPU_REG32 *) 0xFF42A0D0
#define CKSC_115_ADDR      (CPU_REG32 *) 0xFF42A0F0
#define CKSC_PROTCMD1      (CPU_REG32 *) 0xFF428000

#define CKSC_CLK_SRC_ID_HIGHSPEED        0x07                   /* High Speed IntOsc */
#define CKSC_CLK_SRC_ID_MAINOSC          0x0C                   /* MainOsc */
#define CKSC_CLK_SRC_ID_PLL1_1           0x1C                   /* PLL1/1 */
#define CKSC_CLK_SRC_ID_PLL1_2           0x1D                   /* PLL1/2 */
#define CKSC_CLK_SRC_ID_PLL1_4           0x1F                   /* PLL1/4 */
#define CKSC_CLK_SRC_ID_PLL1_8           0x22                   /* PLL1/8 */


/* 8 Bit Message Buffer Registers */
#define FCNxM000DAT0B                    0x00001000

/* 16 Bit Message Buffer Register */
#define FCNxM000DAT0H                    0x00009000

/* Module Registers - mask control register */
#define FCNxCMMKCTL01H                   0x00008300

/* Module Register - mask control */
#define FCNxCMMKCTL01W                   0x00010300


/*
****************************************************************************************************
*                                            DATA TYPES
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*! \brief                          CAN DEVICE REGISTER LAYOUT
*
*            This type defines the global and module CAN register layout.
*/
/*------------------------------------------------------------------------------------------------*/

typedef volatile struct {

    CPU_INT08U  spare0[8];        
    CPU_INT08U  FCNnGMCSPRE;
    CPU_INT08U  spare1[23];        
    CPU_INT08U  FCNnGMADCTL;
    CPU_INT08U  spare2[0x227];        
    CPU_INT08U  FCNnCMLCSTR;
    CPU_INT08U  spare3[3]; 
    CPU_INT08U  FCNnCMINSTR;
    CPU_INT08U  spare4[27]; 
    CPU_INT08U  FCNnCMBRPRS;
    CPU_INT08U  spare5[15]; 
    CPU_INT08U  FCNnCMLISTR;
    CPU_INT08U  spare6[15]; 
    CPU_INT08U  FCNnCMLOSTR;
    CPU_INT08U  spare7[0x7D77]; 
    
    CPU_INT16U  FCNnGMCLCTL;
    CPU_INT08U  spare8[0x16]; 
    CPU_INT16U  FCNnGMABCTL;
    CPU_INT08U  spare9[0x226]; 
    CPU_INT16U  FCNnCMCLCTL;
    CPU_INT08U  spare10[0xE]; 
    CPU_INT16U  FCNnCMERCNT;
    CPU_INT08U  spare11[0x6]; 
    CPU_INT16U  FCNnCMIECTL;
    CPU_INT08U  spare12[0x6]; 
    CPU_INT16U  FCNnCMISCTL;
    CPU_INT08U  spare13[0xE]; 
    CPU_INT16U  FCNnCMBTCTL;
    CPU_INT08U  spare14[0xE]; 
    CPU_INT16U  FCNnCMRGRX;
    CPU_INT08U  spare15[0xE]; 
    CPU_INT16U  FCNnCMTGTX;
    CPU_INT08U  spare16[0x6]; 
    CPU_INT16U  FCNnCMTSCTL;
} V850E2_GLOB_MOD_REG;

/*------------------------------------------------------------------------------------------------*/
/*! \brief                          CAN DEVICE REGISTER LAYOUT
*
*            This type defines the message buffer register layout in 8 bit and 16 bit format.
*/
/*------------------------------------------------------------------------------------------------*/
typedef volatile struct {
    struct {
        CPU_INT08U  DATA;
        CPU_INT08U  spare1[3];        
    }FCNnMmDATA[8];
    
    CPU_INT08U  FCNnMmDTLGB;
    CPU_INT08U  spare3[3];        
    CPU_INT08U  FCNnMmSTRB;
    CPU_INT08U  spare4[27];        
} V850E2_MB_8BIT;

typedef volatile struct {
    struct {
        CPU_INT16U  DATA;
        CPU_INT16U  spare1[3];        
    }FCNnMmDAT[4];
    
    CPU_INT16U  spare2[4];        
    CPU_INT16U  FCNnMmMID0;
    CPU_INT16U  spare3[3];        
    CPU_INT16U  FCNnMmMID1;
    CPU_INT16U  spare4[3];        
    CPU_INT16U  FCNnMmCTL;
    CPU_INT16U  spare5[3];
} V850E2_MB_16BIT;

/*------------------------------------------------------------------------------------------------*/
/*! \brief                          CAN DEVICE REGISTER LAYOUT
*
*            This type defines the mask control register layout for 32 bit access.
*/
/*------------------------------------------------------------------------------------------------*/
typedef volatile struct {
    CPU_INT32U  FCnNCMMKCTL01;
    CPU_INT08U  spare1[12];
    CPU_INT32U  FCnNCMMKCTL03;
    CPU_INT08U  spare2[12];
    CPU_INT32U  FCnNCMMKCTL05;
    CPU_INT08U  spare3[12];
    CPU_INT32U  FCnNCMMKCTL07;
    CPU_INT08U  spare4[12];
    CPU_INT32U  FCnNCMMKCTL09;
    CPU_INT08U  spare5[12];
    CPU_INT32U  FCnNCMMKCTL11;
    CPU_INT08U  spare6[12];
    CPU_INT32U  FCnNCMMKCTL13;
    CPU_INT08U  spare7[12];
    CPU_INT32U  FCnNCMMKCTL15;
    CPU_INT08U  spare8[12];
} V850E2_MASK_CONTROL;


#endif
