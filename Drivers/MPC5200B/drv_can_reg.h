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

/* CANCTL0 */
#define MSCAN_CANCTL0_TIME_DIS                     0       /*!< CANCTL0 Reg.: Timer Enable disabled      */
#define MSCAN_CANCTL0_TIME_EN                      0x08    /*!< CANCTL0 Reg.: Timer Enable enabled       */
#define MSCAN_CANCTL0_WUPE_DIS                     0       /*!< CANCTL0 Reg.: Wake-Up Enable disabled    */
#define MSCAN_CANCTL0_WUPE_EN                      0x04    /*!< CANCTL0 Reg.: Wake-Up Enable enabled     */
#define MSCAN_CANCTL0_SLPRQ_REQ                    0x02    /*!< CANCTL0 Reg.: Sleep Mode Request         */
#define MSCAN_CANCTL0_INITRQ_REQ                   0x01    /*!< CANCTL0 Reg.: Init Mode Request          */

/* CANCTL1 */
#define MSCAN_CANCTL1_CANE_EN                      0x80    /*!< CANCTL1 Reg.: MSCAN Enable                */
#define MSCAN_CANCTL1_LOOPB_EN                     0x20    /*!< CANCTL1 Reg.: MSCAN Loop Back Mode enable */
#define MSCAN_CANCTL1_LISTEN_EN                    0x10    /*!< CANCTL1 Reg.: MSCAN Listen Only Mode en.  */
#define MSCAN_CANCTL1_LISTEN_DIS                   0       /*!< CANCTL1 Reg.: MSCAN Listen Only Mode dis. */
#define MSCAN_CANCTL1_WUPM_EN                      0x04    /*!< CANCTL1 Reg.: MSCAN Wake up Mode enable   */
#define MSCAN_CANCTL1_WUPM_DIS                     0       /*!< CANCTL1 Reg.: MSCAN Wake up Mode disable  */
#define MSCAN_CANCTL1_INITAK                       0x01    /*!< CANCTL1 Reg.: Init Mode Aknowledge        */

/* CANBTR1 */
#define MSCAN_CANBTR1_SAMP_ONE                     0       /*!< CANBTR1 Reg.: One Sample per Bit          */
#define MSCAN_CANBTR1_SAMP_THREE                   0x80    /*!< CANBTR1 Reg.: Three Samples per Bit       */

/* CANRFLG */
#define MSCAN_CANRFLG_CSCIF                        0x40    /*!< CANRFLG Reg.: CAN Status Interrupt Flag   */
#define MSCAN_CANRFLG_RXF                          0x01    /*!< CANRFLG Reg.: Receiver Buffer Full Flag   */
#define MSCAN_CANRFLG_OVRIF                        0x02    /*!< CANRFLG Reg.: Overun Interrupt Flag       */

/* CANRIER */
#define MSCAN_CANRIER_CSCIE_EN                     0x40    /*!< CANRFLG Reg.: CAN Status Interrupt Enable */
#define MSCAN_CANRIER_RXFIE_EN                     0x01    /*!< CANRFLG Reg.: Receiver Buffer Full Enable */
#define MSCAN_CANRIER_TSTATE_EN                    0x08    /*!< CANRFLG Reg.: TSTATE on TxErr and BusOff Enable */
#define MSCAN_CANRIER_RSTATE_EN                    0x20    /*!< CANRFLG Reg.: RSTATE on TxErr and BusOff Enable */

/* CANTFLG */
#define MSCAN_CANTFLG_TXE0                         0x01    /*!< CANTFLG Reg.: TX0 Buffer                  */
#define MSCAN_CANTFLG_TXE1                         0x02    /*!< CANTFLG Reg.: TX1 Buffer                  */
#define MSCAN_CANTFLG_TXE2                         0x04    /*!< CANTFLG Reg.: TX2 Buffer                  */

/* CANTIER */
#define MSCAN_CANTIER_TXEIE0                       0x01    /*!< CANTIER Reg.: TX0 Buffer interrupt enable */
#define MSCAN_CANTIER_TXEIE1                       0x02    /*!< CANTIER Reg.: TX1 Buffer interrupt enable */
#define MSCAN_CANTIER_TXEIE2                       0x04    /*!< CANTIER Reg.: TX2 Buffer interrupt enable */

/* CANTBSEL */
#define MSCAN_CANTBSEL_TX0                         0x01    /*!< CANTFLG Reg.: TX0 Buffer select           */
#define MSCAN_CANTBSEL_TX1                         0x02    /*!< CANTFLG Reg.: TX1 Buffer select           */
#define MSCAN_CANTBSEL_TX2                         0x04    /*!< CANTFLG Reg.: TX2 Buffer select           */

/* CANIDAC */
#define MSCAN_CANIDAC_TWO_32_BIT_FILTERS           0       /*!< CANIDAC Reg.: 2x 32 Bit Filters           */
#define MSCAN_CANIDAC_FOUR_16_BIT_FILTERS          0x10    /*!< CANIDAC Reg.: 4x 16 Bit Filters           */
#define MSCAN_CANIDAC_EIGHT_8_BIT_FILTERS          0x20    /*!< CANIDAC Reg.: 8x 8 Bit Filters            */
#define MSCAN_CANIDAC_FILTER_CLOSED                0x30    /*!< CANIDAC Reg.: no msg receiving possible   */

/* Message Buffer Registers */
#define MSCAN_MB_IDR1_IDE_MASK                     0x08    /*!< Extended ID flag in IDR1 register         */
#define MSCAN_MB_IDR1_SRR_MASK                     0x10    /*!< SRR flag in IDR1 register                 */


#define MSCAN_STD_DATA_LENGTH                      8       /*!< Standard number of data bytes per msg.    */

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
typedef union msg_data {
    /*! \brief Data Bytes with Word Access */
    struct word {
        CPU_INT16U Data0;                      /*  8..9  */
        const CPU_INT16U _sp0w;
        CPU_INT16U Data1;                      /*  C..D  */
        const CPU_INT16U _sp1w;
        CPU_INT16U Data2;                      /* 10..11 */
        const CPU_INT16U _sp2w;
        CPU_INT16U Data3;                      /* 14..15 */
        const CPU_INT16U _sp3w;
    } Word;

    /*! \brief Data Bytes with Byte Access */
    struct byte {
        CPU_INT08U Data0;                      /* 8  */
        CPU_INT08U Data1;                      /* 9  */
        const CPU_INT16U _sp0b;
        CPU_INT08U Data2;                      /* C  */
        CPU_INT08U Data3;                      /* D  */
        const CPU_INT16U _sp1b;
        CPU_INT08U Data4;                      /* 10 */
        CPU_INT08U Data5;                      /* 11 */
        const CPU_INT16U _sp2b;
        CPU_INT08U Data6;                      /* 14 */
        CPU_INT08U Data7;                      /* 15 */
        const CPU_INT16U _sp3b;
    } Byte;
} MsgData;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MSCAN CAN ADDRESS MAP
* \ingroup  MSCAN
*
*           This structure defines the registers on the MSCAN modules.
*
* \note     All addresses are relative to a module offset.
*/
/*------------------------------------------------------------------------------------------------*/
typedef struct MSCAN_reg {
    /*! \brief MSCAN Control Register 0 (CANCTL0)
     *
     * Address: 0x0000,
     * Access: Read/Write
     */
    CPU_INT08U CANCTL0;

     /*! \brief MSCAN Control Register 1 (CANCTL1)
     *
     * Address: 0x0001,
     * Access: Read/Write
     */
    CPU_INT08U CANCTL1;

    const CPU_INT16U _sp0;
   /*! \brief MSCAN Bus Timing Register 0 (CANBTR0)
     *
     * Address: 0x0004,
     * Access: Read/Write
     */
    CPU_INT08U CANBTR0;

    /*! \brief MSCAN Bus Timing Register 1 (CANBTR1)
     *
     * Address: 0x0005,
     * Access: Read/Write
     */
    CPU_INT08U CANBTR1;

    const CPU_INT16U _sp1;
    /*! \brief MSCAN Receive Flag Register (CANRFLG)
     *
     * Address: 0x0008,
     * Access: Read/Write
     */
    CPU_INT08U CANRFLG;

    /*! \brief MSCAN Receiver Interrupt Enable Register (CANRIER)
     *
     * Address: 0x0009,
     * Access: Read/Write
     */
    CPU_INT08U CANRIER;

    const CPU_INT16U _sp2;
    /*! \brief MSCAN Transmitter Flag Register (CANTFLG)
     *
     * Address: 0x000C,
     * Access: Read/Write
     * 
     * Definition: TXE[2:0] Transmitter Buffer Empty flag indicates the associated Tx message buffer
     *                      is empty, and thus not scheduled for transmission. CPU must clear the 
     *                      flag after a message is set up in the Tx buffer and is due for 
     *                      transmission. MSCAN sets flag after message is successfully sent. 
     *                      Flag is also set by MSCAN when Tx request is successfully aborted due 
     *                      to a pending abort request. If not masked, a Tx interrupt is pending 
     *                      while this flag is set. Clearing a TxEx flag also clears the 
     *                      corresponding ABTAKx. When a TxEx flag is set, the corresponding ABTRQx
     *                      bit is cleared. When listen-mode is active TxEx flags cannot be cleared 
     *                      and no transmission is started.
     *                      0 = associated message buffer full (loaded with message due for Tx)
     *                      1 = associated message buffer empty (not scheduled)
     * 
     */
    CPU_INT08U CANTFLG;

    /*! \brief MSCAN Transmitter Interrupt Enable Register (CANTIER)
     *
     * Address: 0x000D,
     * Access: Read/Write
     * 
     * Definition: TXEIE[2:0] Transmitter Empty Interrupt Enable
     *                        0 = No interrupt request is generated from this event.
     *                        1 = Transmitter empty (Tx buffer available) event causes Tx empty interrupt request.
     */
    CPU_INT08U CANTIER;

    const CPU_INT16U _sp3;
    /*! \brief MSCAN Transmitter Message Abort Request Register (CANTARQ)
     *
     * Address: 0x0010,
     * Access: Read/Write
     */
    CPU_INT08U CANTARQ;

    /*! \brief MSCAN Transmitter Message Abort Aknowledge Register (CANTAAK)
     *
     * Address: 0x0011,
     * Access: Read/Write
     */
    CPU_INT08U CANTAAK;

    const CPU_INT16U _sp4;
    /*! \brief MSCAN Transmit Buffer Selection Register (CANTBSEL)
     *
     * Address: 0x0014,
     * Access: Read/Write
     * 
     * Definition: TX[2:0] Transmit Buffer Select lowest numbered bit places respective Tx buffer 
     *                     in CANTxFG register space (e.g., Tx1=1 and Tx0=1 selects Tx buffer Tx0, 
     *                     Tx1=1 and Tx0=0 selects Tx buffer Tx1)
     *                     0 = associated message buffer deselected
     *                     1 = associated message buffer selected, if lowest numbered bit
     * 
     */
    CPU_INT08U CANTBSEL;

    /*! \brief MSCAN Identifier Acceptance Control Register (CANIDAC)
     *
     * Address: 0x0015,
     * Access: Read/Write
     */
    CPU_INT08U CANIDAC;

    const CPU_INT08U _sp5;
    const CPU_INT16U _sp6[2];

    /*! \brief MSCAN Receive Error Counter (CANRXERR)
     *
     * Address: 0x001C,
     * Access: Read only
     */
    const CPU_INT08U CANRXERR;

    /*! \brief MSCAN Transmit Error Counter (CANTXERR)
     *
     * Address: 0x001D,
     * Access: Read only
     */
    const CPU_INT08U CANTXERR;

    const CPU_INT16U _sp7;
    /*! \brief MSCAN Identifier Acceptance Register 0 (CANIDAR0)
     *
     * Address: 0x0020,
     * Access: Read/Write
     * 
     * Definition: AC[7:0] Acceptance Code bits comprise a user defined sequence with which 
     *                     corresponding bits of the related identifier register (IDRn) of the
     *                     receive message buffer are compared. Result of this comparison is 
     *                     then masked with the corresponding identifier mask register.
     */
    CPU_INT08U CANIDAR0;

    /*! \brief MSCAN Identifier Acceptance Register 1 (CANIDAR1)
     *
     * Address: 0x0021,
     * Access: Read/Write
     * 
     * Definition: AC[7:0] Acceptance Code bits comprise a user defined sequence with which 
     *                     corresponding bits of the related identifier register (IDRn) of the
     *                     receive message buffer are compared. Result of this comparison is 
     *                     then masked with the corresponding identifier mask register.
     */
    CPU_INT08U CANIDAR1;

    const CPU_INT16U _sp8;
    /*! \brief MSCAN Identifier Acceptance Register 2 (CANIDAR2)
     *
     * Address: 0x0024,
     * Access: Read/Write
     * 
     * Definition: AC[7:0] Acceptance Code bits comprise a user defined sequence with which 
     *                     corresponding bits of the related identifier register (IDRn) of the
     *                     receive message buffer are compared. Result of this comparison is 
     *                     then masked with the corresponding identifier mask register.
     */
    CPU_INT08U CANIDAR2;

    /*! \brief MSCAN Identifier Acceptance Register 3 (CANIDAR3)
     *
     * Address: 0x0025,
     * Access: Read/Write
     * 
     * Definition: AC[7:0] Acceptance Code bits comprise a user defined sequence with which 
     *                     corresponding bits of the related identifier register (IDRn) of the
     *                     receive message buffer are compared. Result of this comparison is 
     *                     then masked with the corresponding identifier mask register.
     */
    CPU_INT08U CANIDAR3;

    const CPU_INT16U _sp9;
    /*! \brief MSCAN Identifier Mask Register 0 (CANIDMR0)
     *
     * Address: 0x0028,
     * Access: Read/Write
     * 
     * Definition: AM[7:0] Acceptance Mask bits if a particular bit in this register is cleared,
     *                     this indicates the corresponding bit in the identifier acceptance 
     *                     register must be the same as its identifier bit before a match is 
     *                     detected. The message is accepted if all such bits match. If a bit is 
     *                     set, it indicates the state of the corresponding bit in the identifier
     *                     acceptance register does not affect whether or not message is accepted.
     *                     0 = Match corresponding acceptance code register and identifier bits
     *                     1 = Ignore corresponding acceptance code register bit
     */
    CPU_INT08U CANIDMR0;

    /*! \brief MSCAN Identifier Mask Register 1 (CANIDMR1)
     *
     * Address: 0x0029,
     * Access: Read/Write
     * 
     * Definition: AM[7:0] Acceptance Mask bits if a particular bit in this register is cleared,
     *                     this indicates the corresponding bit in the identifier acceptance 
     *                     register must be the same as its identifier bit before a match is 
     *                     detected. The message is accepted if all such bits match. If a bit is 
     *                     set, it indicates the state of the corresponding bit in the identifier
     *                     acceptance register does not affect whether or not message is accepted.
     *                     0 = Match corresponding acceptance code register and identifier bits
     *                     1 = Ignore corresponding acceptance code register bit
     */
    CPU_INT08U CANIDMR1;

    const CPU_INT16U _sp10;
    /*! \brief MSCAN Identifier Mask Register 2 (CANIDMR2)
     *
     * Address: 0x002C,
     * Access: Read/Write
     * 
     * Definition: AM[7:0] Acceptance Mask bits if a particular bit in this register is cleared,
     *                     this indicates the corresponding bit in the identifier acceptance 
     *                     register must be the same as its identifier bit before a match is 
     *                     detected. The message is accepted if all such bits match. If a bit is 
     *                     set, it indicates the state of the corresponding bit in the identifier
     *                     acceptance register does not affect whether or not message is accepted.
     *                     0 = Match corresponding acceptance code register and identifier bits
     *                     1 = Ignore corresponding acceptance code register bit
     */
    CPU_INT08U CANIDMR2;

    /*! \brief MSCAN Identifier Mask Register 3 (CANIDMR3)
     *
     * Address: 0x002D,
     * Access: Read/Write
     * 
     * Definition: AM[7:0] Acceptance Mask bits if a particular bit in this register is cleared,
     *                     this indicates the corresponding bit in the identifier acceptance 
     *                     register must be the same as its identifier bit before a match is 
     *                     detected. The message is accepted if all such bits match. If a bit is 
     *                     set, it indicates the state of the corresponding bit in the identifier
     *                     acceptance register does not affect whether or not message is accepted.
     *                     0 = Match corresponding acceptance code register and identifier bits
     *                     1 = Ignore corresponding acceptance code register bit
     */
    CPU_INT08U CANIDMR3;

    const CPU_INT16U _sp11;
    /*! \brief MSCAN Identifier Acceptance Register 4 (CANIDAR4)
     *
     * Address: 0x0030,
     * Access: Read/Write
     */
    CPU_INT08U CANIDAR4;

    /*! \brief MSCAN Identifier Acceptance Register 5 (CANIDAR5)
     *
     * Address: 0x0031,
     * Access: Read/Write
     */
    CPU_INT08U CANIDAR5;

    const CPU_INT16U _sp12;
    /*! \brief MSCAN Identifier Acceptance Register 6 (CANIDAR6)
     *
     * Address: 0x0034,
     * Access: Read/Write
     */
    CPU_INT08U CANIDAR6;

    /*! \brief MSCAN Identifier Acceptance Register 7 (CANIDAR7)
     *
     * Address: 0x0035,
     * Access: Read/Write
     */
    CPU_INT08U CANIDAR7;

    const CPU_INT16U _sp13;
    /*! \brief MSCAN Identifier Mask Register 4 (CANIDMR4)
     *
     * Address: 0x0038,
     * Access: Read/Write
     */
    CPU_INT08U CANIDMR4;

    /*! \brief MSCAN Identifier Mask Register 5 (CANIDMR5)
     *
     * Address: 0x0039,
     * Access: Read/Write
     */
    CPU_INT08U CANIDMR5;

    const CPU_INT16U _sp14;
    /*! \brief MSCAN Identifier Mask Register 6 (CANIDMR6)
     *
     * Address: 0x003C,
     * Access: Read/Write
     */
    CPU_INT08U CANIDMR6;

    /*! \brief MSCAN Identifier Mask Register 7 (CANIDMR7)
     *
     * Address: 0x003D,
     * Access: Read/Write
     */
    CPU_INT08U CANIDMR7;

    const CPU_INT16U _sp15;
    /*! \brief MSCAN Message Rx Buffer Address Map
     *
     * RxFG (Receiver Foreground)
     * Address: 0x00X0..0x00XF with 10h bytes per message buffer
     * Access: Read/Write (except the last 2 bytes)
     */
    struct msgbuf_rx {
       /*! \brief ID High
         *
         * Offset: 0x0
         */
        CPU_INT16U IDHigh;

        const CPU_INT16U _sp0r;
        /*! \brief ID Low
         *
         * Offset: 0x4
         */
        CPU_INT16U IDLow;

        const CPU_INT16U _sp1r;
        /*! \brief Message Buffer Data Bytes
         *
         * Offset: 0x8..0xB
         */
        MsgData    Data;

       /*! \brief Data Length
         *
         * Offset: 0x18
         */
        CPU_INT08U DLC;

       /*! \brief Transmit Buffer Priority (TBPR)
         *
         * Offset: 0x19
         */
        const CPU_INT08U TBPR;

        const CPU_INT16U _sp2r;
        /*! \brief Timestamp (Read Only) (TSRH/TSRL)
         *
         * Offset: 0xE
         */
        const CPU_INT16U TSR;
        
        const CPU_INT16U _sp3r;
    } MsgBufRx;

   /*! \brief MSCAN Message Tx Buffer Address Map
     *
     * TxFG (Transmitter Foreground)
     * Address: 0x0060..0x007F 
     * Access: Read/Write (except the last 2 bytes)
     */
    struct msgbuf_tx {
        /*! \brief ID High
          *
          * Offset: 0x0
          */
         CPU_INT16U IDHigh;

         const CPU_INT16U _sp0w;
         /*! \brief ID Low
          *
          * Offset: 0x4
          */
         CPU_INT16U IDLow;

         const CPU_INT16U _sp1w;
         /*! \brief Message Buffer Data Bytes
          *
          * Offset: 0x8..0xB
          */
         MsgData    Data;

        /*! \brief Data Length
          *
          * Offset: 0x18
          */
         CPU_INT08U DLC;

        /*! \brief Transmit Buffer Priority (TBPR)
          *
          * Offset: 0x19
          */
         CPU_INT08U TBPR;

         const CPU_INT16U _sp2w;
         /*! \brief Timestamp (Read Only) (TSRH/TSRL)
          *
          * Offset: 0xE
          */
         const CPU_INT16U TSR;
        
    } MsgBufTx;
    
} MSCAN_REG;

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

