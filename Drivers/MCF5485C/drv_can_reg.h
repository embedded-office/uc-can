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

#define MCF_MBAR                      (CPU_INT32U)&__MBAR[0]

                                                      /*------------------------------------------*/
                                                      /* The base address of the FlexCAN module   */
                                                      /*------------------------------------------*/
#define MCF5485C_CAN_A_BASE_ADDRESS  (void *)(MCF_MBAR + 0xA000) /*!< Base address of FlexCAN Module A  */
#define MCF5485C_CAN_B_BASE_ADDRESS  (void *)(MCF_MBAR + 0xA800) /*!< Base address of FlexCAN Module B  */
                                                      /*------------------------------------------*/
                                                      /* FlexCan message buffer defines           */
                                                      /*------------------------------------------*/
#define MCF5485C_CAN_BUFFER_NOT_ACTIVE       0x0      /*!< Message buffer is not active           */

#define MCF5485C_CAN_RX_BUFFER_NOT_ACTIVE    0x0      /*!< Rx Message buffer is not active        */
#define MCF5485C_CAN_RX_BUFFER_EMPTY         0x4      /*!< Rx Message buffer is active and empty  */
#define MCF5485C_CAN_RX_BUFFER_FULL          0x2      /*!< Rx Message buffer is active and full   */
#define MCF5485C_CAN_RX_BUFFER_OVERRUN       0x6      /*!< Rx Message buffer overrun              */
#define MCF5485C_CAN_RX_BUFFER_BUSY          0x1      /*!< Rx Message buffer is busy              */

#define MCF5485C_CAN_TX_BUFFER_NOT_READY     0x8      /*!< Tx Message buffer is not ready         */
#define MCF5485C_CAN_TX_BUFFER_RESPONSE      0xA      /*!< Tx Message buffer automaticly          */
#define MCF5485C_CAN_TX_BUFFER_SEND          0xC      /*!< Tx Message buffer is ready to send     */

#define MCF5485C_CAN_MB_ID_STANDARD          0        /*!< Standard ID flag in MB_CS[IDE] register */
#define MCF5485C_CAN_MB_ID_EXTENDED          1        /*!< Extended ID flag in MB_CS[IDE] register */

#define MCF5485C_CAN_STD_DATA_LENGTH         8        /*!< Standard number of data bytes per msg. */

#define MCF5485C_CAN_N_MSG_BUF               16       /*!< Number of FLexCAN message buffers      */

                                                      /*------------------------------------------*/
                                                      /* FlexCan CR register defines              */
                                                      /*------------------------------------------*/
#define MCF5485C_CAN_CLK_SRC_SYSCLK         0x2000L   /*!< clk source is the system clock         */
#define MCF5485C_CAN_LBUF_LOWBUF_FIRST      0x0010L   /*!< lowest number buffer transmitted first */
#define MCF5485C_CAN_BOFFMSK_EN             0x8000L   /*!< bus off interrupt enabled              */
#define MCF5485C_CAN_ERRMSK_EN              0x4000L   /*!< error interrupt enabled                */
#define MCF5485C_CAN_LPB_EN                 0x1000L   /*!< Loop back enabled                      */
#define MCF5485C_CAN_SMP_THREE              0x0080L   /*!< three sample are used                  */
#define MCF5485C_CAN_BOFFREC_DIS            0x0040L   /*!< atom. recovering from bus off disabled */
#define MCF5485C_CAN_TSYN_EN                0x0020L   /*!< timer sync feature enabled             */
#define MCF5485C_CAN_LOM_LISTEN_ONLY        0x0008L   /*!< listen only mode enabled               */
#define MCF5485C_CAN_CR_MASK            0x0000F0F8L
                                                      /*------------------------------------------*/
                                                      /* FlexCan ESR register defines             */
                                                      /*------------------------------------------*/
#define MCF5485C_CAN_ESR_BOFFINT           0x0004L    /*!< Interrupt is due to Bus off Event      */
#define MCF5485C_CAN_ESR_ERRINT            0x0002L    /*!< Interrupt is due to any Error Bit set  */


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MCF5485C GPIO ADDRESS MAP
* \ingroup  MCF5485C_CAN
*
*           This defines the registers on the MCF5485C for CAN PIN Settings.
*
*/
/*------------------------------------------------------------------------------------------------*/

#define MCF_GPIO_PAR_FECI2CIRQ      (*(volatile CPU_INT16U *)(void*)(MCF_MBAR + 0x0A44))
#define MCF_GPIO_PAR_PSC2           (*(volatile CPU_INT08U *)(void*)(MCF_MBAR + 0x0A4D))
#define MCF_GPIO_PAR_DSPI           (*(volatile CPU_INT16U *)(void*)(MCF_MBAR + 0x0A50))
#define MCF_GPIO_PAR_TIMER          (*(volatile CPU_INT08U *)(void*)(MCF_MBAR + 0x0A52))

                                                      /*------------------------------------------*/
                                                      /* Possible configuration for CANRX0        */
                                                      /*------------------------------------------*/
#define PAR_E1MDIO_VAL       (1 << 8)                 /*!< reg PAR_FEC/I2C/IRQ bits 9-8 value 0x  */
#define PAR_E1MDIO_MASK      (3 << 8)                 /*!< reg PAR_FEC/I2C/IRQ bits 9-8 value 0x  */
#define PAR_CTS2_VAL         (1 << 6)                 /*!< reg PAR_PSC2 bits 7-6 value 01         */
#define PAR_CTS2_MASK        (3 << 6)                 /*!< reg PAR_PSC2 bits 7-6 value 01         */

                                                      /*------------------------------------------*/
                                                      /* Possible configuration for CANRX1        */
                                                      /*------------------------------------------*/
#define PAR_IRQ6_VAL          0                       /*!< reg PAR_FEC/I2C/IRQ bit 1 value 0      */
#define PAR_IRQ6_MASK        (1 << 1)                 /*!< reg PAR_FEC/I2C/IRQ bit 1 value 0      */
#define PAR_IRQ5_VAL          0                       /*!< reg PAR_FEC/I2C/IRQ bit 0 value 0      */
#define PAR_IRQ5_MASK         1                       /*!< reg PAR_FEC/I2C/IRQ bit 0 value 0      */
#define PAR_TIN3_VAL         (1 << 4)                 /*!< reg PAR_TIMER bit 5-4 value 0x         */
#define PAR_TIN3_MASK        (3 << 4)                 /*!< reg PAR_TIMER bit 5-4 value 0x         */
#define PAR_TIN2_VAL         (1 << 1)                 /*!< reg PAR_TIMER bit 2-1 value 0x         */
#define PAR_TIN2_MASK        (3 << 1)                 /*!< reg PAR_TIMER bit 2-1 value 0x         */

                                                      /*------------------------------------------*/
                                                      /* Possible configuration for CANTX1        */
                                                      /*------------------------------------------*/
#define PAR_TOUT3_VAL        0                        /*!< reg PAR_TIMER bit 3 value 0            */
#define PAR_TOUT3_MASK       (1 << 3)                 /*!< reg PAR_TIMER bit 3 value 0            */
#define PAR_CS2_VAL          (1 << 8)                 /*!< reg PAR_DSPI bits 9-8 value 01         */
#define PAR_CS2_MASK         (3 << 8)                 /*!< reg PAR_DSPI bits 9-8 value 01         */
#define PAR_CS3_VAL          (1 << 10)                /*!< reg PAR_DSPI bits 11-10 value 01       */
#define PAR_CS3_MASK         (3 << 10)                /*!< reg PAR_DSPI bits 11-10 value 01       */
#define PAR_TOUT2_VAL        0                        /*!< reg PAR_TIMER bit 0 value 0 */
#define PAR_TOUT2_MASK       1                        /*!< reg PAR_TIMER bit 0 value 0 */

                                                      /*------------------------------------------*/
                                                      /* Possible configuration for CANTX0 */
                                                      /*------------------------------------------*/
#define PAR_E1MDC_VAL        (1 << 6)                 /*!< reg PAR_FEC/I2C/IRQ bits 7-6 value 0x  */
#define PAR_E1MDC_MASK       (3 << 6)                 /*!< reg PAR_FEC/I2C/IRQ bits 7-6 value 0x  */
#define PAR_RTS2_VAL         (1 << 4)                 /*!< reg PAR_PSC2 bits 5-4 value 01         */
#define PAR_RTS2_MASK        (3 << 4)                 /*!< reg PAR_PSC2 bits 5-4 value 01         */


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MCF5485C INTC ADDRESS MAP
* \ingroup  MCF5485C_CAN
*
*           This defines the registers on the MCF5485C for ISR Settings.
*
*/
/*------------------------------------------------------------------------------------------------*/

#define MCF_INTC_IMRH         (*(volatile CPU_INT32U *)(void*)(MCF_MBAR + 0x0708))
#define MCF_INTC_ICR49        (*(volatile CPU_INT08U *)(void*)(MCF_MBAR + 0x0771))
#define MCF_INTC_ICR50        (*(volatile CPU_INT08U *)(void*)(MCF_MBAR + 0x0772))
#define MCF_INTC_ICR51        (*(volatile CPU_INT08U *)(void*)(MCF_MBAR + 0x0773))
#define MCF_INTC_ICR55        (*(volatile CPU_INT08U *)(void*)(MCF_MBAR + 0x0777))
#define MCF_INTC_ICR56        (*(volatile CPU_INT08U *)(void*)(MCF_MBAR + 0x0778))
#define MCF_INTC_ICR57        (*(volatile CPU_INT08U *)(void*)(MCF_MBAR + 0x0779))

#define MCF_CAN0_ERROR        49                      /*!< CAN Interrupt sources                  */
#define MCF_CAN0_BUSOFF       50
#define MCF_CAN0_MBOR         51

#define MCF_CAN1_ERROR        55
#define MCF_CAN1_BUSOFF       56
#define MCF_CAN1_MBOR         57

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
* \brief                      MCF5485C CAN ADDRESS MAP
* \ingroup  MCF5485C_CAN
*
*           This structure defines the registers on the MCF5485C FlexCAN modules.
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
    const CPU_INT32U Res_0x0024;                      /*!< Base+0x0024 reserved                   */
                                                      /*------------------------------------------*/
    union {
        CPU_INT32U R;
        struct {                                      /*------------------------------------------*/
            CPU_INT32U Resevrd:16;                    /*!< unused bytes                           */
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
    } IMASK;                                          /*!< Base+0x0028 Interrupt Masks Register   */
                                                      /*------------------------------------------*/
    const CPU_INT32U Res_0x002C;                      /*!< Base+0x002C reserved                   */
                                                      /*------------------------------------------*/
    union {
        CPU_INT32U R;
        struct {                                      /*------------------------------------------*/
            CPU_INT32U Reserved:16;                   /*!< unused bytes                           */
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
    } IFLAG;                                          /*!< Base+0x0030 Interrupt Flag Register    */
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
    } BUF[MCF5485C_CAN_N_MSG_BUF];                    /*!< Base+0x0080  Message buffer MB0        */
                                                      /* Base+0x017F  Message buffer MB15         */
}MCF5485C_CAN_REG;                                    /*------------------------------------------*/




/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MCF5485C INCT ADDRESS MAP
* \ingroup  MCF5485C_CAN
*
*           This structure defines the registers on the MCF5485C Interrupt controler modules.
*           Baseadresse = 0xFFF4 8000
*/
/*------------------------------------------------------------------------------------------------*/

/*
****************************************************************************************************
*                                          ERROR SECTION
****************************************************************************************************
*/

#endif  /* #ifndef _DRV_CAN_REG_H_ */
