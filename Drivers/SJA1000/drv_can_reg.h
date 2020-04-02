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

#ifndef _DRV_CAN_REG_SJA1000_H_
#define _DRV_CAN_REG_SJA1000_H_

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

#define PeliCANMode 0x01


#define TIMING_REG0  0x03u                            /* This will set BPR to 4, TSEG1 to 7       */
#define TIMING_REG1  0x56u                            /* TSEG2 to 6, SYNC to 1, SJW to x          */

/*! \brief Can Reset Pin Definition */
#define CAN_RESET      0x08u

#define PELICANMODE    0x01u


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      BIT DEFINITIONS
* \ingroup  SJA1000_CAN
*
*           This member holds a bit defintions for various registers of SJA1000
*/
/*------------------------------------------------------------------------------------------------*/

#define RM_RR_BIT  0x01u                              /*!< reset mode (request) bit               */

#define LOM_BIT  0x02u                                /*!<  listen only mode bit                  */
#define STM_BIT  0x04u                                /*!<  self test mode bit                    */
#define AFM_BIT  0x08u                                /*!<  acceptance filter mode bit            */
                                                      /* 0 => dual filter mode                    */
                                                      /* 1 => single filter mode                  */
#define SM_BIT   0x10u                                /*!<  enter sleep mode bit                  */
#define WA_DUAL_MORMALMODE 0x00                       /*!< \brief  wake up, dual filter mode      */
                                                      /* no selftest, no listen-only              */


/*! \brief address and bit definitions for the Interrupt Enable & Control Register                */
#define RIE_BIT  0x01u                                /*!<  receive interrupt enable bit          */
#define TIE_BIT  0x02u                                /*!<  transmit interrupt enable bit         */
#define EIE_BIT  0x04u                                /*!<  error warning interrupt enable bit    */
#define DOIE_BIT 0x08u                                /*!<  data overrun interrupt enable bit     */
#define WUIE_BIT 0x10u                                /*!<  wake-up interrupt enable bit          */
#define EPIE_BIT 0x20u                                /*!<  error passive interrupt enable bit    */
#define ALIE_BIT 0x40u                                /*!<  arbitration lost interr. enable bit   */
#define BEIE_BIT 0x80u                                /*!<  bus error interrupt enable bit        */


/*! \brief address and bit definitions for the Command Register                                   */
#define TR_BIT   0x01u                                /*!<  transmission request bit              */
#define AT_BIT   0x02u                                /*!<  abort transmission bit                */
#define RRB_BIT  0x04u                                /*!<  release receive buffer bit            */
#define CDO_BIT  0x08u                                /*!<  clear data overrun bit                */

/*! \brief address and bit definitions for the Status Register                                    */
#define RBS_BIT  0x01u                                /*!<  receive buffer status bit             */
#define DOS_BIT  0x02u                                /*!<  data overrun status bit               */
#define TBS_BIT  0x04u                                /*!<  transmit buffer status bit            */
#define TCS_BIT  0x08u                                /*!<  transmission complete status bit      */
#define RS_BIT   0x10u                                /*!<  receive status bit                    */
#define TS_BIT   0x20u                                /*!<  transmit status bit                   */
#define ES_BIT   0x40u                                /*!<  error status bit                      */
#define BS_BIT   0x80u                                /*!<  bus status bit                        */

/*! \brief  address and bit definitions for the Interrupt Register                                */
#define RI_BIT   0x01u                                /*!<  receive interrupt bit                 */
#define TI_BIT   0x02u                                /*!<  transmit interrupt bit                */
#define EI_BIT   0x04u                                /*!<  error warning interrupt bit           */
#define DOI_BIT  0x08u                                /*!<  data overrun interrupt bit            */
#define WUI_BIT  0x10u                                /*!<  wake-up interrupt bit                 */
#define EPI_BIT  0x20u                                /*!<  error passive interrupt bit           */
#define ALI_BIT  0x40u                                /*!<  arbitration lost interrupt bit        */
#define BEI_BIT  0x80u                                /*!<  bus error interrupt bit               */

/*! \brief  address and bit definitions for the Bus Timing Registers                              */
#define SAM_BIT  0x80u                                /*!<  sample mode bit                       */
                                                      /* 1 == the bus is sampled 3 times          */
                                                      /* 0 == the bus is sampled once             */
#define SJW_1    0x00u
#define SJW_2    0x01u
#define SJW_3    0x02u
#define SJW_4    0x03u


/*! \brief  address and bit definitions for the Output Control Register                           */
/*  OCMODE1, OCMODE0                                                                              */
#define BIPHASEMODE  0x00u                            /*!<  bi-phase output mode                  */
#define NORMALMODE   0x02u                            /*!<  normal output mode                    */
#define CLKOUTMODE   0x03u                            /*!<  clock output mode                     */

/*! \brief  output pin configuration for TX1                                                      */
#define OCPOL1_BIT   0x20u                            /*!<  output polarity control bit           */
#define TX1FLOAT     0x00u                            /*!<  configured as float                   */
#define TX1PULLDN    0x40u                            /*!<  configured as pull-down               */
#define TX1PULLUP    0x80u                            /*!<  configured as pull-up                 */
#define TX1PSHPULL   0xC0u                            /*!<  configured as push/pull               */

/*! \brief  output pin configuration for TX0                                                      */
#define OCPOL0_BIT   0x04u                            /*!<  output polarity control bit           */
#define TX0FLOAT     0x00u                            /*!<  configured as float                   */
#define TX0PULLDN    0x08u                            /*!<  configured as pull-down               */
#define TX0PULLUP    0x10u                            /*!<  configured as pull-up                 */
#define TX0PSHPULL   0x18u                            /*!<  configured as push/pull               */


/*! \brief  address definitions of the Rx-Buffer                                                  */
#define EXT_FRAME_FORMAT     0x80u

/*! \brief  address and bit definitions for the Clock Divider Register                            */
#define DIVBY1        0x07u                           /*!<  CLKOUT = oscillator frequency         */
#define RXINTEN_BIT   0x20u                           /*!<  pin TX1 used for receive interrupt    */
#define CBP_BIT       0x40u                           /*!<  CAN comparator bypass control bit     */
#define CANMODE_BIT   0x80u                           /*!<  CAN mode definition bit               */



/*
****************************************************************************************************
*                                              MACROS
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*! \brief CAN Set and Clear Reset Mode Macros
 * Variables needed for SetCanResetMode and Clear ResetMode
 * CPU_INT08U cnt, counter to get no loop forever
 * CPU_INT16S result, error
 */
#define SetCanResetMode() {cnt=0;\
                            while((sja1000->ModeControlReg&RM_RR_BIT)==0)\
                            {\
                              sja1000->ModeControlReg|=RM_RR_BIT;\
                              cnt++;\
                              if(cnt==10)\
                              {\
                                result=-1;\
                                break;\
                              }\
                            };\
                          }

#define ClearCanResetMode() {cnt=0;\
                              while((sja1000->ModeControlReg&RM_RR_BIT)!=0)\
                              {\
                                sja1000->ModeControlReg&=(CPU_INT08U)~RM_RR_BIT;\
                                cnt++;\
                                if(cnt==10)\
                                {\
                                  result=-1;\
                                  break;\
                                }\
                              };\
                            }

/*
****************************************************************************************************
*                                            DATA TYPES
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      REGISTER LAYOUT FOR PELICAN MODE
* \ingroup  SJA1000_CAN
*
*           This definition holds the register layout of the SJA1000 in Pelican mode. For detailed
*           register descriptions please refer to SJA1000 datasheet.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct sja1000
{
    CPU_INT08U ModeControlReg;
    CPU_INT08U CommandReg;
    CPU_INT08U StatusReg;
    CPU_INT08U InterruptReg;
    CPU_INT08U InterruptEnableReg;
    CPU_INT08U Reserved1;
    CPU_INT08U BusTiming0Reg;
    CPU_INT08U BusTiming1Reg;
    CPU_INT08U OutputControlReg;
    CPU_INT08U TestReg;
    CPU_INT08U Reserved2;
    CPU_INT08U ArbLostCapReg;
    CPU_INT08U ErrorCodeCapReg;
    CPU_INT08U ErrorWarningLimitReg;
    CPU_INT08U RxErrorCounterReg;
    CPU_INT08U TxErrorCounterReg;

    union
    {
        struct
        {
            CPU_INT08U AcceptanceCodeReg[4];
            CPU_INT08U AcceptanceMaskReg[4];
            CPU_INT08U Reserved[5];
        }accept;

        struct
        {
            CPU_INT08U TxFrameInfoReg;
            CPU_INT08U TxIdDataReg[12];
        }tx;

        struct
        {
            CPU_INT08U RxFrameInfoReg;
            CPU_INT08U RxIdDataReg[12];
        }rx;
    }shared;

    CPU_INT08U RxMsgCountReg;
    CPU_INT08U RxBufStartAdr;
    CPU_INT08U ClockDivideReg;

}SJA_1000;


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


#endif  /* #ifndef _DRV_CAN_REG_SJA1000_H_ */




