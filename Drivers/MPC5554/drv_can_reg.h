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
#include "cpu.h"                              /* CPU configuration definitions and constants      */

/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/
                                                      /*------------------------------------------*/
                                                      /* The base address of the MPC5554 interrupt controller */
                                                      /*------------------------------------------*/
#define MPC5554_INCT_BASE_ADDRESS          0xFFF48000 /*!< Base address of INCT                   */
#define MPC5554_CAN_A_PSR_BASE_VECTOR      152        /*!< Base vector within the PSR table       */
#define MPC5554_CAN_B_PSR_BASE_VECTOR      280        /*!< Base vector within the PSR table       */
#define MPC5554_CAN_C_PSR_BASE_VECTOR      173        /*!< Base vector within the PSR table       */
                                                      /*------------------------------------------*/
                                                      /* The base address of the FlexCAN module   */
                                                      /*------------------------------------------*/
#define MPC5554_CAN_A_BASE_ADDRESS         0xFFFC0000 /*!< Base address of FlexCAN Module A       */
#define MPC5554_CAN_B_BASE_ADDRESS         0xFFFC4000 /*!< Base address of FlexCAN Module B       */
#define MPC5554_CAN_C_BASE_ADDRESS         0xFFFC8000 /*!< Base address of FlexCAN Module C       */
                                                      /*------------------------------------------*/
                                                      /* FlexCan message buffer defines           */
                                                      /*------------------------------------------*/
#define MPC5554_CAN_BUFFER_NOT_ACTIVE       0x0       /*!< Message buffer is not active           */

#define MPC5554_CAN_RX_BUFFER_NOT_ACTIVE    0x0       /*!< Rx Message buffer is not active        */
#define MPC5554_CAN_RX_BUFFER_EMPTY         0x4       /*!< Rx Message buffer is active and empty  */
#define MPC5554_CAN_RX_BUFFER_FULL          0x2       /*!< Rx Message buffer is active and full   */
#define MPC5554_CAN_RX_BUFFER_OVERRUN       0x6       /*!< Rx Message buffer overrun              */
#define MPC5554_CAN_RX_BUFFER_BUSY          0x1       /*!< Rx Message buffer is busy              */

#define MPC5554_CAN_TX_BUFFER_NOT_READY     0x8       /*!< Tx Message buffer is not ready         */
#define MPC5554_CAN_TX_BUFFER_RESPONSE      0xA       /*!< Tx Message buffer automaticly          */
#define MPC5554_CAN_TX_BUFFER_SEND          0xC       /*!< Tx Message buffer is ready to send     */

#define MPC5554_CAN_MB_ID_STANDARD          0         /*!< Standard ID flag in MB_CS[IDE] register */
#define MPC5554_CAN_MB_ID_EXTENDED          1         /*!< Extended ID flag in MB_CS[IDE] register */

#define MPC5554_CAN_STD_DATA_LENGTH         8         /*!< Standard number of data bytes per msg. */

#define MPC5554_CAN_N_MSG_BUF               64        /*!< Number of FLexCAN message buffers       */

                                                      /*------------------------------------------*/
                                                      /* FlexCan CR register defines              */
                                                      /*------------------------------------------*/
#define MPC5554_CAN_CLK_SRC_SYSCLK         0x2000L    /*!< clk source is the system clock         */
#define MPC5554_CAN_LBUF_LOWBUF_FIRST      0x0010L    /*!< lowest number buffer transmitted first */
#define MPC5554_CAN_BOFFMSK_EN             0x8000L    /*!< bus off interrupt enabled              */
#define MPC5554_CAN_ERRMSK_EN              0x4000L    /*!< error interrupt enabled                */
#define MPC5554_CAN_LPB_EN                 0x1000L    /*!< Loop back enabled                      */
#define MPC5554_CAN_SMP_THREE              0x0080L    /*!< three sample are used                  */
#define MPC5554_CAN_BOFFREC_DIS            0x0040L    /*!< atom. recovering from bus off disabled */
#define MPC5554_CAN_TSYN_EN                0x0020L    /*!< timer sync feature enabled             */
#define MPC5554_CAN_LOM_LISTEN_ONLY        0x0008L    /*!< listen only mode enabled               */
#define MPC5554_CAN_CR_MASK            0x0000F0F8L
                                                      /*------------------------------------------*/
                                                      /* FlexCan ESR register defines             */
                                                      /*------------------------------------------*/
#define MPC5554_CAN_ESR_BOFFINT           0x0004L    /*!< Interrupt is due to Bus off Event       */
#define MPC5554_CAN_ESR_ERRINT            0x0002L    /*!< Interrupt is due to any Error Bit set   */

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

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MPC5554 SUI ADDRESS MAP
* \ingroup  MPC5554_CAN
*
*           This defines the registers on the MPC5554 SUI module of the 
*           Pad configuration registers for the FlexCAN2 signals.
*/
/*------------------------------------------------------------------------------------------------*/
#define SUI_PCR_CNTX_A *(CPU_INT16U *)0xC3F900E6 /*!< Baseadress of the PCR[83] SUI register */
#define SUI_PCR_CNRX_A *(CPU_INT16U *)0xC3F900E8 /*!< Baseadress of the PCR[84] SUI register */
#define SUI_PCR_CNTX_B *(CPU_INT16U *)0xC3F900EA /*!< Baseadress of the PCR[85] SUI register */
#define SUI_PCR_CNRX_B *(CPU_INT16U *)0xC3F900EC /*!< Baseadress of the PCR[86] SUI register */
#define SUI_PCR_CNTX_C *(CPU_INT16U *)0xC3F900EE /*!< Baseadress of the PCR[87] SUI register */
#define SUI_PCR_CNRX_C *(CPU_INT16U *)0xC3F900F0 /*!< Baseadress of the PCR[88] SUI register */


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MPC5554 SUI CAN PIN CONFIGURATION MASKS
* \ingroup  MPC5554_CAN
*
*           This defines can used to config the Pad configuration registers masks:
*           SUI_PCR_CNTX_CONFIG_MASK
*           SUI_PCR_CNRX_CONFIG_MASK            
*/
/*------------------------------------------------------------------------------------------------*/
#define CNTX      0x0400                                                                  
#define CNRX      0x0400
#define OBE       0x0200                                                  
#define IBE       0x0100                                                     
#define DSC_10pF  0x0000                   
#define DSC_20pF  0x0040
#define DSC_30pF  0x0080
#define DSC_50pF  0x00C0
#define ODE       0x0020      
#define HYS       0x0010      
#define WPE       0x0002      
#define WPS       0x0001      

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MPC5554 SUI CAN PIN CONFIGURATION
* \ingroup  MPC5554_CAN
*
*           This defines holds the configuration of the  
*           Pad configuration registers for the FlexCAN2 signals.
*/
/*------------------------------------------------------------------------------------------------*/
#define SUI_PCR_CNTX_CONFIG_MASK       (CNTX | OBE | DSC_10pF)
#define SUI_PCR_CNRX_CONFIG_MASK       (CNRX | IBE | DSC_10pF)
                                                                                                      
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MPC5554 CAN ADDRESS MAP
* \ingroup  MPC5554_CAN
*
*           This structure defines the registers on the MPC5554 FlexCAN modules.
*
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct mpc5554_can_reg {
    union {
        CPU_INT32U R;
        struct {                                      /*------------------------------------------*/
            CPU_INT32U MDIS     :1;                   /*!< Module disable                         */
            CPU_INT32U FRZ      :1;                   /*!< Freeze enable                          */
            CPU_INT32U          :1;                   /*!< resserved                              */
            CPU_INT32U HALT     :1;                   /*!< Halt FlexCAN                           */
            CPU_INT32U NOTRDY   :1;                   /*!< FlexCAN2 not ready                     */
            CPU_INT32U          :1;                   /*!< resserved                              */
            CPU_INT32U SOFTRST  :1;                   /*!< Soft reset initiated                   */
            CPU_INT32U FRZACK   :1;                   /*!< Freeze mode acknowledge                */
            CPU_INT32U          :1;                   /*!< resserved                              */
            CPU_INT32U          :1;                   /*!< resserved                              */
            CPU_INT32U          :1;                   /*!< resserved                              */
            CPU_INT32U MDISACK  :1;                   /*!< Low power mode acknowledge             */
            CPU_INT32U          :1;                   /*!< resserved                              */
            CPU_INT32U          :1;                   /*!< resserved                              */
            CPU_INT32U          :12;                  /*!< resserved                              */
            CPU_INT32U MAXMB    :6;                   /*!< Maximum number of message buffers      */
        } B;                                          /*------------------------------------------*/
    } MCR;                                            /*!< Base+0x0000 Module Configuration Register */
                                                      /*------------------------------------------*/
    union {
        CPU_INT32U R;
        struct {                                      /*------------------------------------------*/
            CPU_INT32U PRESDIV  :8;                   /*!< Prescaler division factor              */
            CPU_INT32U RJW      :2;                   /*!< Resync jump wide                       */
            CPU_INT32U PSEG1    :3;                   /*!< Phase segment 1                        */
            CPU_INT32U PSEG2    :3;                   /*!< Phase segment 2                        */
            CPU_INT32U BOFFMSK  :1;                   /*!< Bus off mask enable                    */
            CPU_INT32U ERRMSK   :1;                   /*!< Error mask enable                      */
            CPU_INT32U CLK_SRC  :1;                   /*!< CAN engine clock source                */
            CPU_INT32U LPB      :1;                   /*!< Loop back enable                       */
            CPU_INT32U          :4;                   /*!< reserved                               */
            CPU_INT32U SMP      :1;                   /*!< Sampling mode                          */
            CPU_INT32U BOFFREC  :1;                   /*!< Bus off recovery mode                  */
            CPU_INT32U TSYN     :1;                   /*!< Timer sync mode                        */
            CPU_INT32U LBUF     :1;                   /*!< Lowest buffer transmitted first        */
            CPU_INT32U LOM      :1;                   /*!< Listen-only mode                       */
            CPU_INT32U PROPSEG  :3;                   /*!< Propagation segment def.               */
        } B;                                          /*------------------------------------------*/
    } CR;                                             /*!< Base+0x0004 Control Register           */
                                                      /*------------------------------------------*/
    CPU_INT32U TIMER;                                 /*!< Base+0x0008 Free Running Timer         */
                                                      /*------------------------------------------*/
    const CPU_INT32U Res_0x000C;                      /*!< Base+0x000C reserved                   */
                                                      /*------------------------------------------*/
    union {
        CPU_INT32U R;
        struct {                                      /*------------------------------------------*/
            CPU_INT32U       :3;                      /*!< reserved                               */
            CPU_INT32U MI    :29;                     /*!< ID mask bits                           */
        } B;                                          /*------------------------------------------*/
    } RXGMASK;                                        /*!< Base+0x0010 RX Global Mask             */
                                                      /*------------------------------------------*/
    union {
        CPU_INT32U R;
        struct {                                      /*------------------------------------------*/
            CPU_INT32U       :3;                      /*!< reserved                               */
            CPU_INT32U MI    :29;                     /*!< ID mask bits                           */
        } B;                                          /*------------------------------------------*/
    } RX14MASK;                                       /*!< Base+0x0014 RX 14 Mask                 */
                                                      /*------------------------------------------*/
    union {
        CPU_INT32U R;
        struct {                                      /*------------------------------------------*/
            CPU_INT32U       :3;                      /*!< reserved                               */
            CPU_INT32U MI    :29;                     /*!< ID mask bits                           */
        } B;                                          /*------------------------------------------*/
    } RX15MASK;                                       /*!< Base+0x0018 RX 15 Mask                 */
                                                      /*------------------------------------------*/
    union {
        CPU_INT32U R;
        struct {                                      /*------------------------------------------*/
            CPU_INT32U        :16;                    /*!< reserved                               */
            CPU_INT32U RXECNT :8;                     /*!< RX error counter                       */
            CPU_INT32U TXECNT :8;                     /*!< TX error counter                       */
        } B;                                          /*------------------------------------------*/
    } ECR;                                            /*!< Base+0x001C Error Counter Register     */
                                                      /*------------------------------------------*/
    union {
        CPU_INT32U R;
        struct {                                      /*------------------------------------------*/
            CPU_INT32U          :16;                  /*!< reserved                               */
            CPU_INT32U BIT1ERR  :1;                   /*!< Bit 1 error                            */
            CPU_INT32U BIT0ERR  :1;                   /*!< Bit 0 error                            */
            CPU_INT32U ACKERR   :1;                   /*!< Acknowlege error                       */
            CPU_INT32U CRCERR   :1;                   /*!< Cyclic redundancy code error           */
            CPU_INT32U FRMERR   :1;                   /*!< Form error                             */
            CPU_INT32U STFERR   :1;                   /*!< Stuffing error                         */
            CPU_INT32U TXWRN    :1;                   /*!< TX error counter                       */
            CPU_INT32U RXWRN    :1;                   /*!< RX error counter                       */
            CPU_INT32U IDLE     :1;                   /*!< CAN bus idle state                     */
            CPU_INT32U TXRX     :1;                   /*!< Current FlaxCAN2 status                */
            CPU_INT32U FLTCONF  :2;                   /*!< Fault confinement state                */
            CPU_INT32U          :1;                   /*!< reserved                               */
            CPU_INT32U BOFFINT  :1;                   /*!< Bus off interrupt                      */
            CPU_INT32U ERRINT   :1;                   /*!< Error interrupt                        */
            CPU_INT32U          :1;                   /*!< reserved                               */
        } B;                                          /*------------------------------------------*/
    } ESR;                                            /*!< Base-0x0020 Error and Status Register  */
                                                      /*------------------------------------------*/
    union {
        CPU_INT32U R;
        struct {                                      /*------------------------------------------*/
            CPU_INT32U BUF63M:1;                      /*!< Message buffer 63 mask                 */
            CPU_INT32U BUF62M:1;                      /*!< Message buffer 62 mask                 */
            CPU_INT32U BUF61M:1;                      /*!< Message buffer 61 mask                 */
            CPU_INT32U BUF60M:1;                      /*!< Message buffer 60 mask                 */
            CPU_INT32U BUF59M:1;                      /*!< Message buffer 59 mask                 */
            CPU_INT32U BUF58M:1;                      /*!< Message buffer 58 mask                 */
            CPU_INT32U BUF57M:1;                      /*!< Message buffer 57 mask                 */
            CPU_INT32U BUF56M:1;                      /*!< Message buffer 56 mask                 */
            CPU_INT32U BUF55M:1;                      /*!< Message buffer 55 mask                 */
            CPU_INT32U BUF54M:1;                      /*!< Message buffer 54 mask                 */
            CPU_INT32U BUF53M:1;                      /*!< Message buffer 53 mask                 */
            CPU_INT32U BUF52M:1;                      /*!< Message buffer 52 mask                 */
            CPU_INT32U BUF51M:1;                      /*!< Message buffer 51 mask                 */
            CPU_INT32U BUF50M:1;                      /*!< Message buffer 50 mask                 */
            CPU_INT32U BUF49M:1;                      /*!< Message buffer 49 mask                 */
            CPU_INT32U BUF48M:1;                      /*!< Message buffer 48 mask                 */
            CPU_INT32U BUF47M:1;                      /*!< Message buffer 47 mask                 */
            CPU_INT32U BUF46M:1;                      /*!< Message buffer 46 mask                 */
            CPU_INT32U BUF45M:1;                      /*!< Message buffer 45 mask                 */
            CPU_INT32U BUF44M:1;                      /*!< Message buffer 44 mask                 */
            CPU_INT32U BUF43M:1;                      /*!< Message buffer 43 mask                 */
            CPU_INT32U BUF42M:1;                      /*!< Message buffer 42 mask                 */
            CPU_INT32U BUF41M:1;                      /*!< Message buffer 41 mask                 */
            CPU_INT32U BUF40M:1;                      /*!< Message buffer 40 mask                 */
            CPU_INT32U BUF39M:1;                      /*!< Message buffer 39 mask                 */
            CPU_INT32U BUF38M:1;                      /*!< Message buffer 38 mask                 */
            CPU_INT32U BUF37M:1;                      /*!< Message buffer 37 mask                 */
            CPU_INT32U BUF36M:1;                      /*!< Message buffer 36 mask                 */
            CPU_INT32U BUF35M:1;                      /*!< Message buffer 35 mask                 */
            CPU_INT32U BUF34M:1;                      /*!< Message buffer 34 mask                 */
            CPU_INT32U BUF33M:1;                      /*!< Message buffer 33 mask                 */
            CPU_INT32U BUF32M:1;                      /*!< Message buffer 32 mask                 */
        } B;                                          /*------------------------------------------*/
    } IMRH;                                           /*!< Base+0x0024 Interrupt Masks High Register */
                                                      /*------------------------------------------*/
    union {
        CPU_INT32U R;
        struct {                                      /*------------------------------------------*/
            CPU_INT32U BUF31M:1;                      /*!< Message buffer 31 mask                 */
            CPU_INT32U BUF30M:1;                      /*!< Message buffer 30 mask                 */
            CPU_INT32U BUF29M:1;                      /*!< Message buffer 29 mask                 */
            CPU_INT32U BUF28M:1;                      /*!< Message buffer 28 mask                 */
            CPU_INT32U BUF27M:1;                      /*!< Message buffer 27 mask                 */
            CPU_INT32U BUF26M:1;                      /*!< Message buffer 26 mask                 */
            CPU_INT32U BUF25M:1;                      /*!< Message buffer 25 mask                 */
            CPU_INT32U BUF24M:1;                      /*!< Message buffer 24 mask                 */
            CPU_INT32U BUF23M:1;                      /*!< Message buffer 23 mask                 */
            CPU_INT32U BUF22M:1;                      /*!< Message buffer 22 mask                 */
            CPU_INT32U BUF21M:1;                      /*!< Message buffer 21 mask                 */
            CPU_INT32U BUF20M:1;                      /*!< Message buffer 20 mask                 */
            CPU_INT32U BUF19M:1;                      /*!< Message buffer 19 mask                 */
            CPU_INT32U BUF18M:1;                      /*!< Message buffer 18 mask                 */
            CPU_INT32U BUF17M:1;                      /*!< Message buffer 17 mask                 */
            CPU_INT32U BUF16M:1;                      /*!< Message buffer 16 mask                 */
            CPU_INT32U BUF15M:1;                      /*!< Message buffer 15 mask                 */
            CPU_INT32U BUF14M:1;                      /*!< Message buffer 14 mask                 */
            CPU_INT32U BUF13M:1;                      /*!< Message buffer 13 mask                 */
            CPU_INT32U BUF12M:1;                      /*!< Message buffer 12 mask                 */
            CPU_INT32U BUF11M:1;                      /*!< Message buffer 11 mask                 */
            CPU_INT32U BUF10M:1;                      /*!< Message buffer 10 mask                 */
            CPU_INT32U BUF09M:1;                      /*!< Message buffer 9 mask                  */
            CPU_INT32U BUF08M:1;                      /*!< Message buffer 8 mask                  */
            CPU_INT32U BUF07M:1;                      /*!< Message buffer 7 mask                  */
            CPU_INT32U BUF06M:1;                      /*!< Message buffer 6 mask                  */
            CPU_INT32U BUF05M:1;                      /*!< Message buffer 5 mask                  */
            CPU_INT32U BUF04M:1;                      /*!< Message buffer 4 mask                  */
            CPU_INT32U BUF03M:1;                      /*!< Message buffer 3 mask                  */
            CPU_INT32U BUF02M:1;                      /*!< Message buffer 2 mask                  */
            CPU_INT32U BUF01M:1;                      /*!< Message buffer 1 mask                  */
            CPU_INT32U BUF00M:1;                      /*!< Message buffer 0 mask                  */
        } B;                                          /*------------------------------------------*/
    } IMRL;                                           /*!< Base+0x0028 Interrupt Masks Low Register */
                                                      /*------------------------------------------*/
    union {
        CPU_INT32U R;
        struct {                                      /*------------------------------------------*/
            CPU_INT32U BUF63I:1;                      /*!< Message buffer 63 interrupt occure     */
            CPU_INT32U BUF62I:1;                      /*!< Message buffer 62 interrupt occure     */
            CPU_INT32U BUF61I:1;                      /*!< Message buffer 61 interrupt occure     */
            CPU_INT32U BUF60I:1;                      /*!< Message buffer 60 interrupt occure     */
            CPU_INT32U BUF59I:1;                      /*!< Message buffer 59 interrupt occure     */
            CPU_INT32U BUF58I:1;                      /*!< Message buffer 58 interrupt occure     */
            CPU_INT32U BUF57I:1;                      /*!< Message buffer 57 interrupt occure     */
            CPU_INT32U BUF56I:1;                      /*!< Message buffer 56 interrupt occure     */
            CPU_INT32U BUF55I:1;                      /*!< Message buffer 55 interrupt occure     */
            CPU_INT32U BUF54I:1;                      /*!< Message buffer 54 interrupt occure     */
            CPU_INT32U BUF53I:1;                      /*!< Message buffer 53 interrupt occure     */
            CPU_INT32U BUF52I:1;                      /*!< Message buffer 52 interrupt occure     */
            CPU_INT32U BUF51I:1;                      /*!< Message buffer 51 interrupt occure     */
            CPU_INT32U BUF50I:1;                      /*!< Message buffer 50 interrupt occure     */
            CPU_INT32U BUF49I:1;                      /*!< Message buffer 49 interrupt occure     */
            CPU_INT32U BUF48I:1;                      /*!< Message buffer 48 interrupt occure     */
            CPU_INT32U BUF47I:1;                      /*!< Message buffer 47 interrupt occure     */
            CPU_INT32U BUF46I:1;                      /*!< Message buffer 46 interrupt occure     */
            CPU_INT32U BUF45I:1;                      /*!< Message buffer 45 interrupt occure     */
            CPU_INT32U BUF44I:1;                      /*!< Message buffer 44 interrupt occure     */
            CPU_INT32U BUF43I:1;                      /*!< Message buffer 43 interrupt occure     */
            CPU_INT32U BUF42I:1;                      /*!< Message buffer 42 interrupt occure     */
            CPU_INT32U BUF41I:1;                      /*!< Message buffer 41 interrupt occure     */
            CPU_INT32U BUF40I:1;                      /*!< Message buffer 40 interrupt occure     */
            CPU_INT32U BUF39I:1;                      /*!< Message buffer 39 interrupt occure     */
            CPU_INT32U BUF38I:1;                      /*!< Message buffer 38 interrupt occure     */
            CPU_INT32U BUF37I:1;                      /*!< Message buffer 37 interrupt occure     */
            CPU_INT32U BUF36I:1;                      /*!< Message buffer 36 interrupt occure     */
            CPU_INT32U BUF35I:1;                      /*!< Message buffer 35 interrupt occure     */
            CPU_INT32U BUF34I:1;                      /*!< Message buffer 34 interrupt occure     */
            CPU_INT32U BUF33I:1;                      /*!< Message buffer 33 interrupt occure     */
            CPU_INT32U BUF32I:1;                      /*!< Message buffer 32 interrupt occure     */
        } B;                                          /*------------------------------------------*/
    } IFRH;                                           /*!< Base+0x2C Interrupt Flag High Register */
    union {                                           /*------------------------------------------*/
        CPU_INT32U R;
        struct {                                      /*------------------------------------------*/
            CPU_INT32U BUF31I:1;                      /*!< Message buffer 31 interrupt occure     */
            CPU_INT32U BUF30I:1;                      /*!< Message buffer 30 interrupt occure     */
            CPU_INT32U BUF29I:1;                      /*!< Message buffer 29 interrupt occure     */
            CPU_INT32U BUF28I:1;                      /*!< Message buffer 28 interrupt occure     */
            CPU_INT32U BUF27I:1;                      /*!< Message buffer 27 interrupt occure     */
            CPU_INT32U BUF26I:1;                      /*!< Message buffer 26 interrupt occure     */
            CPU_INT32U BUF25I:1;                      /*!< Message buffer 25 interrupt occure     */
            CPU_INT32U BUF24I:1;                      /*!< Message buffer 24 interrupt occure     */
            CPU_INT32U BUF23I:1;                      /*!< Message buffer 23 interrupt occure     */
            CPU_INT32U BUF22I:1;                      /*!< Message buffer 22 interrupt occure     */
            CPU_INT32U BUF21I:1;                      /*!< Message buffer 21 interrupt occure     */
            CPU_INT32U BUF20I:1;                      /*!< Message buffer 20 interrupt occure     */
            CPU_INT32U BUF19I:1;                      /*!< Message buffer 19 interrupt occure     */
            CPU_INT32U BUF18I:1;                      /*!< Message buffer 18 interrupt occure     */
            CPU_INT32U BUF17I:1;                      /*!< Message buffer 17 interrupt occure     */
            CPU_INT32U BUF16I:1;                      /*!< Message buffer 16 interrupt occure     */
            CPU_INT32U BUF15I:1;                      /*!< Message buffer 15 interrupt occure     */
            CPU_INT32U BUF14I:1;                      /*!< Message buffer 14 interrupt occure     */
            CPU_INT32U BUF13I:1;                      /*!< Message buffer 13 interrupt occure     */
            CPU_INT32U BUF12I:1;                      /*!< Message buffer 12 interrupt occure     */
            CPU_INT32U BUF11I:1;                      /*!< Message buffer 11 interrupt occure     */
            CPU_INT32U BUF10I:1;                      /*!< Message buffer 10 interrupt occure     */
            CPU_INT32U BUF09I:1;                      /*!< Message buffer  9 interrupt occure     */
            CPU_INT32U BUF08I:1;                      /*!< Message buffer  8 interrupt occure     */
            CPU_INT32U BUF07I:1;                      /*!< Message buffer  7 interrupt occure     */
            CPU_INT32U BUF06I:1;                      /*!< Message buffer  6 interrupt occure     */
            CPU_INT32U BUF05I:1;                      /*!< Message buffer  5 interrupt occure     */
            CPU_INT32U BUF04I:1;                      /*!< Message buffer  4 interrupt occure     */
            CPU_INT32U BUF03I:1;                      /*!< Message buffer  3 interrupt occure     */
            CPU_INT32U BUF02I:1;                      /*!< Message buffer  2 interrupt occure     */
            CPU_INT32U BUF01I:1;                      /*!< Message buffer  1 interrupt occure     */
            CPU_INT32U BUF00I:1;                      /*!< Message buffer  0 interrupt occure     */
        } B;                                          /*------------------------------------------*/
    } IFRL;                                           /*!< Base+0x0030 Interrupt Flag Low Register */
                                                      /*------------------------------------------*/
    const CPU_INT32U Res_0x0034;                      /*!< Base+0x0034 reserved                   */
    const CPU_INT32U Res_0x0038;                      /*!< Base+0x0038 reserved                   */
    const CPU_INT32U Res_0x003C;                      /*!< Base+0x003C reserved                   */
    const CPU_INT32U Res_0x0040;                      /*!< Base+0x0040 reserved                   */
    const CPU_INT32U Res_0x0044;                      /*!< Base+0x0044 reserved                   */
    const CPU_INT32U Res_0x0048;                      /*!< Base+0x0048 reserved                   */
    const CPU_INT32U Res_0x004C;                      /*!< Base+0x004C reserved                   */
    const CPU_INT32U Res_0x0050;                      /*!< Base+0x0050 reserved                   */
    const CPU_INT32U Res_0x0054;                      /*!< Base+0x0054 reserved                   */
    const CPU_INT32U Res_0x0058;                      /*!< Base+0x0058 reserved                   */
    const CPU_INT32U Res_0x005C;                      /*!< Base+0x005C reserved                   */
    const CPU_INT32U Res_0x0060;                      /*!< Base+0x0060 reserved                   */
    const CPU_INT32U Res_0x0064;                      /*!< Base+0x0064 reserved                   */
    const CPU_INT32U Res_0x0068;                      /*!< Base+0x0068 reserved                   */
    const CPU_INT32U Res_0x006C;                      /*!< Base+0x006C reserved                   */
    const CPU_INT32U Res_0x0070;                      /*!< Base+0x0070 reserved                   */
    const CPU_INT32U Res_0x0074;                      /*!< Base+0x0074 reserved                   */
    const CPU_INT32U Res_0x0078;                      /*!< Base+0x0078 reserved                   */
    const CPU_INT32U Res_0x007C;                      /*!< Base+0x007C reserved                   */
    struct canbuf_t {                                 /*------------------------------------------*/
        union {
            CPU_INT32U R;
            struct {                                  /*------------------------------------------*/
                CPU_INT32U           :4;              /*!< reserved                               */
                CPU_INT32U CODE      :4;              /*!< Message buffer code                    */
                CPU_INT32U           :1;              /*!< reserved                               */
                CPU_INT32U SRR       :1;              /*!< Substitute remote request              */
                CPU_INT32U IDE       :1;              /*!< ID extended bit                        */
                CPU_INT32U RTR       :1;              /*!< Remote transmission request            */
                CPU_INT32U LENGTH    :4;              /*!< Lengh of data in bytes                 */
                CPU_INT32U TIMESTAMP :16;             /*!< Freerunning counter time stamp         */
            } B;                                      /*------------------------------------------*/
        } CS;                                         /*!< 0x80 MB Control and Status CS          */
                                                      /*------------------------------------------*/
        CPU_INT32U ID;                                /*!< 0x84 MB Frame identifier               */
        union {                                       /*------------------------------------------*/
            CPU_INT08U Byte[8];                       /* Data buffer in Bytes (8 bits)            */
            CPU_INT32U Word[2];                       /* Data buffer in words (32 bits)           */
                                                      /*------------------------------------------*/
        } DATA;                                       /*!< 0x88-0x8F MB Data field                */
                                                      /*------------------------------------------*/
    } BUF[MPC5554_CAN_N_MSG_BUF];                     /*!< Base+0x0080  Message buffer MB0        */
                                                      /* Base+0x047F  Message buffer MB63         */
}MPC5554_CAN_REG;                                     /*------------------------------------------*/




/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MPC5554 INCT ADDRESS MAP
* \ingroup  MPC5554_CAN
*
*           This structure defines the registers on the MPC5554 Interrupt controler modules.
*           Baseadresse = 0xFFF4 8000
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct mpc5554_intc_reg {
     union {
         CPU_INT32U R;
         struct {                                     /* -----------------------------------------*/
             CPU_INT32U        :26;                   /*!< reserved                               */
             CPU_INT32U VTES   :1;                    /*!< Vector table entry size                */
             CPU_INT32U        :4;                    /*!< reserved                               */
             CPU_INT32U HVEN   :1;                    /*!< Hardware vector mode enable            */
         } B;                                         /* -----------------------------------------*/
     } MCR;                                           /* Base+0x0000 Module Configuration Register */
                                                      /* -----------------------------------------*/
     const CPU_INT32U Res_0x0004;                     /*!< reserved                               */
     union {                                          /* -----------------------------------------*/
         CPU_INT32U R;
         struct {                                     /* -----------------------------------------*/
             CPU_INT32U       :28;                    /*!< reserved                               */
             CPU_INT32U PRI   :4;                     /*!< Priority                               */
         } B;                                         /* -----------------------------------------*/
     } CPR;                                           /* Base+0x0008 Current Priority Register    */
                                                      /* -----------------------------------------*/
     const CPU_INT32U Res_0x000C;                     /*!< reserved                               */
                                                      /* -----------------------------------------*/
     union {
         CPU_INT32U R;
         struct {                                     /* -----------------------------------------*/
             CPU_INT32U VTBA   :21;                   /*!< Vector table base adress               */
             CPU_INT32U INTVEC :9;                    /*!< Interruptvector                        */
             CPU_INT32U        :2;                    /*!< reserved                               */
         } B;                                         /* -----------------------------------------*/
     } IACKR;                                         /* Base+0x0010 Interrupt Acknowledge Register */
                                                      /* -----------------------------------------*/
     CPU_INT32U Res_0x0014;                           /*!< reserved                               */
                                                      /* -----------------------------------------*/
     union {
         CPU_INT32U R;
         struct {                                     /* -----------------------------------------*/
             CPU_INT32U         :32;                  /*!< End of Interrupt                       */
         } B;                                         /* -----------------------------------------*/
     } EOIR;                                          /* Base+0x0018 End of Interrupt Register    */
                                                      /* -----------------------------------------*/
     CPU_INT32U REs_0x001C;                           /*!< reserved                               */
                                                      /* -----------------------------------------*/
     union {
         CPU_INT08U R;
         struct {                                     /* -----------------------------------------*/
             CPU_INT08U       :6;                     /*!< reserved                               */
             CPU_INT08U SET   :1;                     /*!< set flag bit                           */
             CPU_INT08U CLR   :1;                     /*!< clear flag bit                         */
         } B;                                         /* -----------------------------------------*/
     } SSCIR[8];                                      /* Base+0x0020 Software Set/Clear Interrupt Register */
                                                      /* -----------------------------------------*/
     CPU_INT32U Res_0x0028;                           /*!< reserved                               */
     CPU_INT32U Res_0x002C;                           /*!< reserved                               */
     CPU_INT32U Res_0x0030;                           /*!< reserved                               */
     CPU_INT32U Res_0x0034;                           /*!< reserved                               */
     CPU_INT32U Res_0x0038;                           /*!< reserved                               */
     CPU_INT32U Res_0x003C;                           /*!< reserved                               */
                                                      /* -----------------------------------------*/
     union {
         CPU_INT08U R;
         struct {                                     /* -----------------------------------------*/
             CPU_INT08U       :4;                     /*!< reserved                               */
             CPU_INT08U PRI   :4;                     /*!< Priority                               */
         } B;                                         /* -----------------------------------------*/
     } PSR[307];                                      /* Base+0x0040 to Base+0x0173               */
                                                      /* Software Set/Clear Interrupt Register    */
                                                      /* -----------------------------------------*/
 } MPC5554_INTC_MAP;

/*
****************************************************************************************************
*                                          ERROR SECTION
****************************************************************************************************
*/

#endif  /* #ifndef _DRV_CAN_REG_H_ */
