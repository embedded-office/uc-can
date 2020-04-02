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
* Filename : drv_can.c
* Version  : V2.42.01
****************************************************************************************************
*/

/*
****************************************************************************************************
*                                             INCLUDES
****************************************************************************************************
*/
#include "drv_can_reg.h"                         /* register definitions for eCAN module          */
#include "drv_can.h"                             /* driver function prototypes and data typedefs  */
#include "drv_def.h"                             /* driver layer declarations                     */
#include "can_bsp.h"                             /* CAN board support package                     */

/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MAILBOX-ENABLE REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to Mailbox-Enable Register of the used
*           Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_ME   ECanRegPtr->CANME

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      TX I/O CONTROL REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to TX I/O Control Register of the used
*           Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_TIOC ECanRegPtr->CANTIOC

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      RX I/O CONTROL REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to RX I/O Control Register of the used
*           Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_RIOC ECanRegPtr->CANRIOC

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ERROR AND STATUS REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to Error and Status Register of the used
*           Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_ES   ECanRegPtr->CANES

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      BIT-TIMING CONFIGURATION REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Bit-Timing Configuration Register of the used
*           Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_BTC  ECanRegPtr->CANBTC

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MAILBOX-DIRECTION REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Mailbox-Direction Register of the used Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_MD   ECanRegPtr->CANMD

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      OVERWRITE PROTECTION CONTROL REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Overwrite Protection Control Register of the
*           used Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_OPC  ECanRegPtr->CANOPC

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MASTER CONTROL REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Master Control Register of the used Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_MC   ECanRegPtr->CANMC

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MAILBOX INTERRUPT LEVEL REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Mailbox Interrupt Level Register of the used Can
*           Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_MIL  ECanRegPtr->CANMIL

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      GLOBAL INTERRUPT MASK REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Global Interrupt Mask Register of the used Can
*           Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_GIM  ECanRegPtr->CANGIM

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MAILBOX INTERRUPT MASK REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Mailbox Interrupt Mask Register of the used Can
*           Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_MIM  ECanRegPtr->CANMIM

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      RECEIVED-MESSAGE-PENDING REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Received-Message-Pending Register of the used
*           Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_RMP  ECanRegPtr->CANRMP

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      GLOBAL INTERRUPT FLAG REGISTER 1
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Global Interrupt Flag Register 1 of the used Can
*           Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_GIF1 ECanRegPtr->CANGIF1

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      TRANSMISSION-ACKNOWLEDGE REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Transmission-Acknowledge Register of the used
*           Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_TA   ECanRegPtr->CANTA

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      RECEIVED-MESSAGE-LOST REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Received-Message-Lost Register of the used Can
*           Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_RML  ECanRegPtr->CANRML

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      TRANSMISSION-REQUEST SET REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Transmission-Request Set Register of the used
*           Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_TRS  ECanRegPtr->CANTRS

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      LOCAL ACCEPTANCE MASK
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Local Acceptance Mask Register for the used RX
*           Buffer of the used Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_RXLAM  *(ECanLamPtr+TMS_28XX_ECAN_RX_BUFFER)

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MESSAGE IDENTIFIER REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Message Identifier Register for the used RX
*           Buffer of the used Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_RX_MSGID (ECanMBoxPtr+TMS_28XX_ECAN_RX_BUFFER)->MSGID

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MESSAGE IDENTIFIER REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Message Identifier Register for the used TX
*           Buffer of the used Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_TX_MSGID (ECanMBoxPtr+TMS_28XX_ECAN_TX_BUFFER)->MSGID

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MESSAGE-CONTROL REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Message-Control Register for the used RX Buffer
*           of the used Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_RX_MSGCTRL (ECanMBoxPtr+TMS_28XX_ECAN_RX_BUFFER)->MSGCTRL

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MESSAGE-CONTROL REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Message-Control Register for the used TX Buffer
*           of the used Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_TX_MSGCTRL (ECanMBoxPtr+TMS_28XX_ECAN_TX_BUFFER)->MSGCTRL

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MESSAGE-DATA-LOW REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Message-Data-Low Register for the used RX Buffer
*           of the used Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_RXMDL (ECanMBoxPtr+TMS_28XX_ECAN_RX_BUFFER)->MDL

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MESSAGE-DATA-HIGH REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Message-Data-High Register for the used RX Buffer
*           of the used Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_RXMDH (ECanMBoxPtr+TMS_28XX_ECAN_RX_BUFFER)->MDH

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MESSAGE-DATA-LOW REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Message-Data-Low Register for the used TX Buffer
*           of the used Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_TXMDL (ECanMBoxPtr+TMS_28XX_ECAN_TX_BUFFER)->MDL

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      MESSAGE-DATA-HIGH REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This Constant holds the Pointer to the Message-Data-High Register for the used TX Buffer
*           of the used Can Bus.
*/
/*------------------------------------------------------------------------------------------------*/
#define CAN_TXMDH (ECanMBoxPtr+TMS_28XX_ECAN_TX_BUFFER)->MDH



/*
****************************************************************************************************
*                                              MACROS
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      ENABLE WRITING TO PROTECTED REGISTERS
* \ingroup  TMS28XX_ECAN
*
*           This Macros enables writing to EALLOW protected Registers.
*           \Note: The following registers are EALLOW protected (see spru074d chapter 3.1.4):
*                  - CANMC[15..9] & MCR[7..6]
*                  - CANBTC
*                  - CANGIM
*                  - MIM[31..0]
*                  - TSC[31..0]
*                  - IOCONT1[3]
*                  - IOCONT2[3]
*
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECAN_EALLOW() asm(" EALLOW")

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DISABLE WRITING TO PROTECTED REGISTERS
* \ingroup  TMS28XX_ECAN
*
*           This Macro disables writing to EALLOW protected Registers.
*/
/*------------------------------------------------------------------------------------------------*/
#define TMS28XX_ECAN_EDIS()   asm(" EDIS")

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      WRITE TO REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This macro writes to a register twice with 4 NOPs delay.
*           \Note: According to the DSP Silicon Errata (sprz193j) this is needed for writing to
*                  MSGID/MSGCTRL/MDH/MDL/LAM/MOT/MOTS to ensure that the write will not fail
*                  to execute.
*/
/*------------------------------------------------------------------------------------------------*/
#define WR_REG(reg,val) (reg)=(val);\
                        asm(" nop");\
                        asm(" nop");\
                        asm(" nop");\
                        asm(" nop");\
                        (reg)=(val);

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      READ REGISTER
* \ingroup  TMS28XX_ECAN
*
*           This macro reads a register, checks if the value and if it's 0, the register will be
*           read again to check, if it was really 0.
*           \Note: According to the DSP Silicon Errate (sprz192j) this is needed for reading from
*                  MSGID/MSGCTRL/MDH/MDL/LAM/MOT/MOTS to ensure that the read was successfull.
*/
/*------------------------------------------------------------------------------------------------*/
#define RD_REG(val,reg) (val)=(reg);\
                        if(val==0){ \
                        (val)=(reg);\
                        }

/*
****************************************************************************************************
*                                            LOCAL DATA
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER DATA
* \ingroup  TMS28XX_ECAN
*
*           Global bus data of the eCAN modules.
*/
/*------------------------------------------------------------------------------------------------*/
static TMS28XX_ECAN_DATA ECANData[TMS28XX_ECAN_DEV_N];


const TMS28XX_ECAN_PARA ECANPara[TMS28XX_ECAN_DEV_N] =
{
    {
        TMS28XX_ECANA_BASE_ADDRESS,
        TMS28XX_ECANA_BASE_ADDRESS + TMS28XX_ECAN_LAM_OFFS_ADDRESS,
        TMS28XX_ECANA_BASE_ADDRESS + TMS28XX_ECAN_MBOX_OFFS_ADDRESS,
        TMS28XX_ECANA_CLK,
        TMS28XX_ECANA_DEF_BAUDRATE,
        TMS28XX_ECANA_SJWREG,
        TMS28XX_ECANA_TSEG1REG,
        TMS28XX_ECANA_TSEG2REG
    },
#ifdef TMS320F280X
    {
        TMS28XX_ECANB_BASE_ADDRESS,
        TMS28XX_ECANB_BASE_ADDRESS + TMS28XX_ECAN_LAM_OFFS_ADDRESS,
        TMS28XX_ECANB_BASE_ADDRESS + TMS28XX_ECAN_MBOX_OFFS_ADDRESS,
        TMS28XX_ECANB_CLK,
        TMS28XX_ECANB_DEF_BAUDRATE,
        TMS28XX_ECANB_SJWREG,
        TMS28XX_ECANB_TSEG1REG,
        TMS28XX_ECANB_TSEG2REG
    }
#endif
};


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER ERRORCODE
* \ingroup  TMS28XX_ECAN
*
*           This variable holds the detailed errorcode, if an error is detected.
*/
/*------------------------------------------------------------------------------------------------*/
static CPU_INT16U DrvError;

/*
****************************************************************************************************
*                                             GLOBAL DATA
****************************************************************************************************
*/
/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER IDENTIFICATION
* \ingroup  TMS28XX_ECAN
*
*           This constant variable holds the unique driver identification code.
*/
/*------------------------------------------------------------------------------------------------*/
static const CPU_INT32U DrvIdent = 0x243F1601;

/*
****************************************************************************************************
*                                            FUNCTIONS
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      TMS28XX ECAN INITIALIZATION
* \ingroup  TMS28XX_ECAN
*
*           Initializes the eCAN module and sets the CAN bus baudrate with standard identifiers.
*
* \param    arg               Unused, but needed for common interface
*
* \return   Errorcode (always 0)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S TMS28XXCANInit (CPU_INT32U arg)
{                                                     /*------------------------------------------*/
    CPU_INT32U         baudrate;                      /* Local: Variable for Baudrate             */
    CPU_INT32U         TempBuf;                       /* Local: Temporary Buffer                  */
    TMS28XX_ECAN_REGS *ECanRegPtr;                    /* Local: Pointer to eCAN Registers         */
    TMS28XX_ECAN_MBOX *ECanMBoxPtr;                   /* Local: Pointer to Mailbox                */
    CPU_INT32U        *ECanLamPtr;                    /* Local: Pointer to Local Acceptance Mask  */
    CPU_INT08U         i;                             /* Local: Counter variable                  */
    CPU_INT08U         j;                             /* Local: Counter variable                  */
#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
    CPU_SR             cpu_sr;                        /* Allocate storage for CPU status register */
#endif
                                                      /*------------------------------------------*/
    (void) arg;                                       /* prevent compiler warnings                */
    DrvError = TMS28XX_CAN_NO_ERR;                    /* init DrvError to no error                */
                                                      /*------------------------------------------*/
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
                                                      /*------------------------------------------*/
    for (i=0; i< TMS28XX_ECAN_DEV_N; i++) {           /* loop through all CAN devices             */
        ECANData[i].Status = TMS28XX_ECAN_IDLE;       /* set init Status                          */
                                                      /*------------------------------------------*/
        ECanRegPtr  = (TMS28XX_ECAN_REGS *)ECANPara[i].ECANBase; /* set pointer to can registers  */
        ECanLamPtr  = (CPU_INT32U *)ECANPara[i].LAMBase;         /* set pointer to local acceptance mask */
        ECanMBoxPtr = (TMS28XX_ECAN_MBOX *)ECANPara[i].MBoxBase; /* set pointer to mailbox        */
                                                      /*------------------------------------------*/
        TMS28XX_ECAN_EALLOW();                        /* allow register access                    */
        CAN_TIOC = 0x00000008;                        /* Configure the eCAN TX pin for eCAN transmissions */
        CAN_RIOC = 0x00000008;                        /* Configure the eCAN RX pin for eCAN transmissions */
        TMS28XX_ECAN_EDIS();                          /* disable register access                  */
                                                      /*------------------------------------------*/
        TMS28XXECANSetPins();                         /* setup I/O pins                           */
                                                      /*------------------------------------------*/
        CAN_ME       = 0x00000000;                    /* Disable all Mailboxes                    */
        WR_REG(CAN_RX_MSGID, 0x40000000);             /* use Local Acceptance Mask for RX buffer  */
        WR_REG(CAN_RXLAM, 0x1FFFFFFF);                /* accept standard OR extended IDs (Bit 31 = '0') */
                                                      /* accept all IDs (Bit 0 - 28 = '1')        */
                                                      /*------------------------------------------*/
        CAN_MD  = (1L << TMS_28XX_ECAN_RX_BUFFER);    /* Configure RX Mailboxe(s) as Rx, the others as TX */
        CAN_OPC = (1L << TMS_28XX_ECAN_RX_BUFFER);    /* overwrite-protection for RX Mailboxe(s)  */
                                                      /*------------------------------------------*/
        for (j=0; j<32; j++) {                        /* repeat for all mailboxes                 */
            (ECanMBoxPtr+j)->MSGCTRL = 0x00000000;    /* No remote frame is requested             */
        }                                             /*------------------------------------------*/
        TMS28XX_ECAN_EALLOW();                        /* allow register access                    */
        TempBuf  = CAN_MC;                            /* Request permission to change configuration */
        TempBuf |= 0x00001000;                        /* set Change-configuration request bit     */
        CAN_MC   = TempBuf;
        TMS28XX_ECAN_EDIS();                          /* disable register access                  */
                                                      /*------------------------------------------*/
        do {                                          /* Wait until the CPU has been granted permission to change */
            TempBuf = CAN_ES;                         /* read status register                     */
        } while ((TempBuf & 0x00000010) == 0);        /* check if bit CCE is set                  */
                                                      /*------------------------------------------*/
                                                      /* Configure the eCAN timing                */
        baudrate = ECANPara[0].Baudrate;              /* get default baudrate                     */
                                                      /*------------------------------------------*/
        TMS28XX_ECAN_EALLOW();                        /* allow register access                    */
        TempBuf  = ((CPU_INT32U)
                    (((ECANPara[0].Clock /            /* set baud rate prescaler                  */
                       baudrate /
                       (ECANPara[0].TSEG1 +
                        ECANPara[0].TSEG2 + 3)) - 1)) << 16) |
                   ((CPU_INT32U)ECANPara[0].TSEG2) |  /* set time segment 2                       */
                   (((CPU_INT32U)ECANPara[0].TSEG1) << 3) | /* set time segment 1                 */
                   (((CPU_INT32U)ECANPara[0].SJW) << 8); /* set synchronization jump width.       */
                                                      /*------------------------------------------*/
        CAN_BTC  = TempBuf;                           /* write to baudrate register               */
                                                      /*------------------------------------------*/
        TempBuf  = CAN_MC;                            /* reset permission request                 */
        TempBuf &= ~ 0x00001000;
        CAN_MC   = TempBuf;
        TMS28XX_ECAN_EDIS();                          /* disable register access                  */

        do {                                          /* Wait until CPU no longer has permission to change */
            TempBuf = CAN_ES;                         /* configuration registers                  */
        } while ((TempBuf & 0x00000010) != 0);        /* check if bit CCE is reset                */
                                                      /*------------------------------------------*/
        TMS28XX_ECAN_EALLOW();                        /* allow register access                    */
        TempBuf  = CAN_MC;
        TempBuf |= 0x00002400;                        /* set eCAN mode & data significant byte first */
        CAN_MC = TempBuf;

#if TMS28XX_ECAN_RX_INTERRUPT_EN == 1
        TempBuf  = CAN_MIM;                           /* set interrupt enable flag for rx         */
        TempBuf |= (1L << TMS_28XX_ECAN_RX_BUFFER);   /* message buffer                           */
        CAN_MIM  = TempBuf;
#endif
#if TMS28XX_ECAN_TX_INTERRUPT_EN == 1
        TempBuf  = CAN_MIM;                           /* set interrupt enable flag for tx         */
        TempBuf |= (1L << TMS_28XX_ECAN_TX_BUFFER);   /* message buffer                           */
        CAN_MIM  = TempBuf;
#endif
#if TMS28XX_ECAN_NS_INTERRUPT_EN == 1
        TempBuf  = CAN_GIM;                           /* enable error passive, bus-off interrupt  */
        TempBuf |= 0x00000600;
        CAN_GIM  = TempBuf;
#endif
#if (TMS28XX_ECAN_RX_INTERRUPT_EN == 1) ||\
    (TMS28XX_ECAN_TX_INTERRUPT_EN == 1) ||\
    (TMS28XX_ECAN_NS_INTERRUPT_EN == 1)
        CAN_MIL  = 0xFFFFFFFF;                        /* all Mailbox Interrupts on ECAN1INT       */
        TempBuf  = CAN_GIM;                           /* interrupts mapped to ECAN1INT,           */
        TempBuf |= 0x00000006;                        /* ECAN1INT enabled                         */
        CAN_GIM  = TempBuf;
#endif
        TMS28XX_ECAN_EDIS();                          /* disable register access                  */

#if (TMS28XX_ECAN_RX_INTERRUPT_EN == 1) ||\
    (TMS28XX_ECAN_TX_INTERRUPT_EN == 1) ||\
    (TMS28XX_ECAN_NS_INTERRUPT_EN == 1)
        TMS28XXECANSetInterrupt ();                   /* set interrupt                            */
#endif

    }                                                 /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return 0;                                         /* return 0                                 */
}                                                     /*------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      TMS28X ECAN OPEN MESSAGE BUFFER
* \ingroup  TMS28XX_ECAN
*
*           Opens the CAN bus after checking, that no message buffer is in use or a message buffer
*           after checking, that the bus is not opened, this device is not used and a free
*           parameter table slot is available.
*
* \param    devId             Unused, but needed for common interface
* \param    devName           The eCAN device name (see ecan_devname)
* \param    mode              Unused, but needed for common interface
*
* \return   The device identifier for further access or -1 if an error occurs
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S TMS28XXCANOpen (CPU_INT16S devId,
                           CPU_INT32U devName,
                           CPU_INT16U mode)
{                                                     /*------------------------------------------*/
    CPU_INT16S result = -1;                           /* Local: result variable                   */
#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
    CPU_SR     cpu_sr;                                /* Allocate storage for CPU status register */
#endif
                                                      /*------------------------------------------*/
    devId = devId;                                    /* suppress compiler warning                */
                                                      /*------------------------------------------*/
#if TMS28XX_ECAN_ARG_CHK_CFG > 0
    if (devName >= TMS28XX_ECAN_DEV_N) {              /* devId out of range?                      */
        DrvError = TMS28XX_CAN_BUS_ERR;               /* set bus error                            */
        return(result);                               /* return error                             */
    }                                                 /*------------------------------------------*/
    if (mode != DEV_RW) {                             /* mode not supported?                      */
        DrvError = TMS28XX_CAN_MODE_ERR;              /* set mode error                           */
        return(result);                               /* return error                             */
    }                                                 /*------------------------------------------*/
#endif

    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    if ((ECANData[devName].Status & TMS28XX_ECAN_BUSY) == 0) { /* if device not busy              */
        ECANData[devName].Status |= TMS28XX_ECAN_BUSY; /* mark CAN device to be busy              */
        result   = devName;                           /* set return value to devName              */
    } else {                                          /* otherwise: set busy error                */
        DrvError = TMS28XX_CAN_OPEN_ERR;              /* set open error                           */
    }                                                 /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
                                                      /*------------------------------------------*/
    return(result);                                   /* return function result                   */
}                                                     /*------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      TMS28XX ECAN CLOSE SUBDEVICE
* \ingroup  TMS28XX_ECAN
*
*           Close the CAN bus or a message buffer after checking, that this  device is opened.
*
* \param    devId             The device identifier, returned by TMS28XXCANOpen()
*
* \return   Errorcode
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S TMS28XXCANClose (CPU_INT16S devId)
{                                                     /*------------------------------------------*/
    CPU_INT16S result = -1;                           /* Local: Function result                   */
#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
    CPU_SR     cpu_sr;                                /* Allocate storage for CPU status register */
#endif
                                                      /*------------------------------------------*/
#if TMS28XX_ECAN_ARG_CHK_CFG > 0
    if ((devId < 0) ||                                /* devId out of range?                      */
        (devId >= TMS28XX_ECAN_DEV_N)) {
        DrvError = TMS28XX_CAN_BUS_ERR;               /* set bus error                            */
        return(-1);                                   /* return error                             */
    }                                                 /*------------------------------------------*/
#endif
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    if ((ECANData[devId].Status & TMS28XX_ECAN_BUSY) != 0) { /* see, if CAN device is opened      */
        ECANData[devId].Status &=~ TMS28XX_ECAN_BUSY; /* yes: reset CAN device status             */
        result = 0;                                   /* Ok, device is closed                     */
    } else {                                          /* otherwise:                               */
        DrvError = TMS28XX_CAN_CLOSE_ERR;             /* set device not opened error              */
    }
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
                                                      /*------------------------------------------*/
    return(result);                                   /* return function result                   */
}                                                     /*------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      TMS28XX ECAN I/O CONTROL
* \ingroup  TMS28XX_ECAN
*
*           Controls the bus after or a message buffer after checking, that this  device is opened.
*
*           The following functioncodes are allowed:
*               - IO_TMS28XX_CAN_GET_IDENT:         Get Driver Ident Code
*               - IO_TMS28XX_CAN_SET_BAUDRATE:      Set Baudrate
*               - IO_TMS28XX_CAN_START:             Start CAN controller.
*               - IO_TMS28XX_CAN_STOP:              Stop CAN controller.
*               - IO_TMS28XX_CAN_RX_STANDARD:       Set Receiver to Standard Identifier
*               - IO_TMS28XX_CAN_RX_EXTENDED:       Set Receiver to Extended Identifier
*               - IO_TMS28XX_CAN_GET_NODE_STATUS:   Get Node Status
*               - IO_TMS28XX_CAN_TX_READY:          Get TX ready status
*
* \param    devId             The device identifier, returned by TMS28XXCANOpen()
* \param    func              Function code (see ecan_ioctl_func)
* \param    arg               Argument list, specific to the function code
*
* \return   Errorcode
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S TMS28XXCANIoCtl (CPU_INT16S devId,
                            CPU_INT16U func,
                            void *arg)
{                                                     /*------------------------------------------*/
    CPU_INT32U           baudrate;                    /* Local: baudrate                          */
    CPU_INT32U           TempBuf;                     /* Local: temporary buffer                  */
    CPU_INT16S           result         = 0;          /* Local: Function result                   */
    TMS28XX_ECAN_REGS   *ECanRegPtr;                  /* Local: Pointer to eCAN Registers         */
    TMS28XX_ECAN_MBOX   *ECanMBoxPtr;                 /* Local: Pointer to Mailbox                */
#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
    CPU_SR               cpu_sr;                      /* Allocate storage for CPU status register */
#endif
                                                      /*------------------------------------------*/
#if TMS28XX_ECAN_ARG_CHK_CFG > 0
    if ((devId < 0) ||                                /* devId out of range?                      */
        (devId >= TMS28XX_ECAN_DEV_N)) {
        DrvError = TMS28XX_CAN_BUS_ERR;               /* set bus error                            */
        return(-1);                                   /* return error                             */
    }                                                 /*------------------------------------------*/
#endif
    if ((ECANData[devId].Status &                     /* see, if CAN device is not opened         */
         TMS28XX_ECAN_BUSY) == 0) {
        DrvError = TMS28XX_CAN_OPEN_ERR;              /* yes: set device not opened error         */
        return(-1);                                   /* return error                             */
    }                                                 /*------------------------------------------*/
    ECanRegPtr  = (TMS28XX_ECAN_REGS *)ECANPara[devId].ECANBase; /* set pointer to can registers  */
    ECanMBoxPtr = (TMS28XX_ECAN_MBOX *)ECANPara[devId].MBoxBase; /* set pointer to mailbox        */
                                                      /*------------------------------------------*/
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
                                                      /*------------------------------------------*/
    switch (func) {                                   /* select: function code                    */
                                                      /*------------------------------------------*/
        case IO_TMS28XX_CAN_GET_IDENT:               /* Get Ident                                */
            (*(CPU_INT32U*)arg) = DrvIdent;           /* return driver ident code                 */
            break;
        case IO_TMS28XX_CAN_GET_ERRNO:               /* Get Error Code                           */
            (*(CPU_INT16U*)arg) = DrvError;           /* return driver error code                 */
            break;
        case IO_TMS28XX_CAN_GET_DRVNAME:             /* Get Driver Name                          */
            *((CPU_INT08U **)(arg)) =                 /* set ptr to driver name to arg ptr        */
                (CPU_INT08U *)(TMS28XX_ECAN_NAME);
            break;
        case IO_TMS28XX_CAN_SET_BAUDRATE:            /* Set Baudrate                             */
            baudrate = *(CPU_INT32U *)arg;            /* get baudrate                             */
            if (baudrate == 0) {                      /* if baudrate 0                            */
                DrvError = TMS28XX_CAN_ARG_ERR;       /* set errorcode                            */
                result   = -1;                        /* indicate error on function result        */
            } else {                                  /* otherwise: */
                TMS28XX_ECAN_EALLOW();                /* allow register access                    */
                TempBuf  = CAN_MC;                    /* Request permission to change configuration */
                TempBuf |= 0x00001000;                /* set Change-configuration request bit     */
                CAN_MC   = TempBuf;
                TMS28XX_ECAN_EDIS();                  /* disable register access                  */
                                                      /*------------------------------------------*/
                do {                                  /* Wait until the CPU has been granted permission to change */
                    TempBuf = CAN_ES;                 /* read status register                     */
                } while ((TempBuf & 0x00000010) == 0); /* check if bit CCE is set                 */
                                                      /*------------------------------------------*/
                TMS28XX_ECAN_EALLOW();                /* allow register access                    */
                TempBuf  = ((CPU_INT32U)
                            (((ECANPara[0].Clock /    /* set baud rate prescaler                  */
                               baudrate /
                               (ECANPara[0].TSEG1 +
                                ECANPara[0].TSEG2 +
                                3)) - 1)) << 16) |
                           ((CPU_INT32U)ECANPara[0].TSEG2) | /* set time segment 2                */
                           (((CPU_INT32U)ECANPara[0].TSEG1) << 3) | /* set time segment 1         */
                           (((CPU_INT32U)ECANPara[0].SJW) << 8); /* set synchronization jump width. */
                                                      /*------------------------------------------*/
                CAN_BTC = TempBuf;                    /* write to baudrate register               */
                                                      /*------------------------------------------*/
                TempBuf  = CAN_MC;                    /* reset permission request                 */
                TempBuf &= ~ 0x00001000;
                CAN_MC   = TempBuf;
                TMS28XX_ECAN_EDIS();                  /* disable register access                  */
                                                      /*------------------------------------------*/
                do {                                  /* Wait until CPU no longer has permission to change */
                    TempBuf = CAN_ES;                 /* configuration registers                  */
                } while ((TempBuf & 0x00000010) != 0); /* check if bit CCE is reset               */
            }                                         /*------------------------------------------*/
            break;
        case IO_TMS28XX_CAN_START:                   /* Start CAN controller.                    */
            CAN_ME = (1L << TMS_28XX_ECAN_RX_BUFFER) | /* Enable used Mailboxes                   */
                     (1L << TMS_28XX_ECAN_TX_BUFFER);
            break;
        case IO_TMS28XX_CAN_STOP:                    /* Stop CAN controller.                     */
            CAN_ME = 0;                               /* Disable Mailboxes                        */
            break;
        case IO_TMS28XX_CAN_RX_STANDARD:             /* Set Receiver to Standard Identifier      */
            RD_REG(TempBuf, CAN_RX_MSGID);            /* read register                            */
            TempBuf &=~ 0x80000000;                   /* reset IDE-Bit                            */
            WR_REG(CAN_RX_MSGID, TempBuf);            /* write back to register                   */
            break;
        case IO_TMS28XX_CAN_RX_EXTENDED:             /* Set Receiver to Extended Identifier      */
            RD_REG(TempBuf, CAN_RX_MSGID);            /* read register                            */
            TempBuf |= 0x80000000;                    /* set IDE-Bit                              */
            WR_REG(CAN_RX_MSGID, TempBuf);            /* write back to register                   */
            break;
        case IO_TMS28XX_CAN_GET_NODE_STATUS:         /* Get Node Status                          */
            if ((CAN_GIF1 & 0x00000200) != 0) {
                (*((CPU_INT08U*)arg)) =               /* Error passive                            */
                    TMS28XX_ECAN_ERROR_PASSIVE;
            } else if ((CAN_GIF1 & 0x00000400) != 0) {
                (*((CPU_INT08U*)arg)) =               /* Error Bus off                            */
                    TMS28XX_ECAN_ERROR_BUS_OFF;
            } else {
                (*((CPU_INT08U*)arg)) =               /* Error active                             */
                    TMS28XX_ECAN_ERROR_ACTIVE;
            }
            break;
        case IO_TMS28XX_CAN_TX_READY:                /* Get TX ready status                      */
            if ((CAN_TRS &                            /* if buffer is empty                       */
                 (1L << TMS_28XX_ECAN_TX_BUFFER)) == 0){
                (*((CPU_INT08U*)arg)) = 1;
            } else {
                (*((CPU_INT08U*)arg)) = 0;
            }
            break;
                                                      /*------------------------------------------*/
        default:                                      /* WRONG FUNCTION CODE                      */
            DrvError = TMS28XX_CAN_FUNC_ERR;          /* set errorcode                            */
            result   = -1;                            /* indicate error on function result        */
            break;
    }                                                 /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
                                                      /*------------------------------------------*/
    return(result);                                   /* Return function result                   */
}                                                     /*------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      TMS28XX ECAN READ DATA
* \ingroup  TMS28XX_ECAN
*
*           Read a received message from a message buffer after checking, that this  device is
*           opened.
*
* \param    devId             The device identifier, returned by TMS28XXCANOpen()
* \param    buf               Byte array for received data
* \param    size              Length of can frame memory
*
* \return   Errorcode
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S TMS28XXCANRead (CPU_INT16S devId,
                           CPU_INT08U *buf,
                           CPU_INT16U size)
{                                                     /*------------------------------------------*/
    CPU_INT32U           TempBuf;                     /* Local: temporary Buffer                  */
    CPU_INT16S           result         = 0;          /* Local: Function result                   */
    TMS28XX_ECAN_REGS   *ECanRegPtr;                  /* Local: Pointer to eCAN Registers         */
    TMS28XX_ECAN_MBOX   *ECanMBoxPtr;                 /* Local: Pointer to Mailbox                */
    TMS28XX_ECAN_FRM    *frm;                         /* Local: Pointer to can frame              */
#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
    CPU_SR               cpu_sr;                      /* Allocate storage for CPU status register */
#endif
                                                      /*------------------------------------------*/
#if TMS28XX_ECAN_ARG_CHK_CFG > 0
    if ((devId < 0) ||                                /* if devId out of range?                   */
        (devId >= TMS28XX_ECAN_DEV_N)) {
        DrvError = TMS28XX_CAN_BUS_ERR;               /* yes: set bus error                       */
        return(-1);                                   /* return error                             */
    }                                                 /*------------------------------------------*/
    if (buf == (void*)0) {                            /* if valid buffer pointer?                 */
        DrvError = TMS28XX_CAN_ARG_ERR;               /* yes: set argument checking error         */
        return(-1);                                   /* return error                             */
    }                                                 /*------------------------------------------*/
    if (size != sizeof(TMS28XX_ECAN_FRM)) {           /* size in range?                           */
        DrvError = TMS28XX_CAN_NO_DATA_ERR;           /* yes: set no data error                   */
        return(-1);                                   /* return error                             */
    }                                                 /*------------------------------------------*/
#endif

    ECanRegPtr  = (TMS28XX_ECAN_REGS *)ECANPara[devId].ECANBase; /* Set pointer to eCAN registers */
    ECanMBoxPtr = (TMS28XX_ECAN_MBOX *)ECANPara[devId].MBoxBase; /* set pointer to base mailbox   */
    frm         = (TMS28XX_ECAN_FRM *)buf;            /* Set pointer to can frame                 */
                                                      /*------------------------------------------*/
    if ((ECANData[devId].Status &                     /* see, if CAN device is not opened         */
         TMS28XX_ECAN_BUSY) == 0) {
        DrvError = TMS28XX_CAN_OPEN_ERR;              /* yes: set device not opened error         */
        return(-1);                                   /* return error                             */
    }                                                 /*------------------------------------------*/

    if ((CAN_RMP &                                    /* no message received?                     */
         (1L << TMS_28XX_ECAN_RX_BUFFER)) == 0) {
        DrvError = TMS28XX_CAN_NO_DATA_ERR;           /* yes: set no data error                   */
        return(-1);                                   /* return error                             */
    } else {                                          /* otherwise: message received              */
        CPU_CRITICAL_ENTER();                         /* enter critical section                   */

        RD_REG(TempBuf, CAN_RX_MSGCTRL);              /* get DLC out of message control register  */
        frm->DLC = TempBuf & 0x000F;
        if (frm->DLC >= 1) {                          /* if bytes received                        */
            RD_REG(TempBuf, CAN_RXMDL);               /* read message low register                */
            frm->Data[0] = (CPU_INT08U)(TempBuf       /* get byte 0                               */
                                        & 0x000000FF);
            frm->Data[1] = (CPU_INT08U)((TempBuf >> 8) /* get byte 1                              */
                                        & 0x000000FF);
            frm->Data[2] = (CPU_INT08U)((TempBuf >> 16) /* get byte 2                             */
                                        & 0x000000FF);
            frm->Data[3] = (CPU_INT08U)((TempBuf >> 24) /* get byte 3                             */
                                        & 0x000000FF);
        }                                             /*------------------------------------------*/
        if (frm->DLC >= 5) {                          /* if more than 4 bytes received            */
            RD_REG(TempBuf, CAN_RXMDH);               /* read message high register               */
            frm->Data[4] = (CPU_INT08U)(CAN_RXMDH     /* get byte 4                               */
                                        & 0x000000FF);
            frm->Data[5] = (CPU_INT08U)((CAN_RXMDH >> 8) /* get byte 5                            */
                                        & 0x000000FF);
            frm->Data[6] = (CPU_INT08U)((CAN_RXMDH >> 16) /* get byte 6                           */
                                        & 0x000000FF);
            frm->Data[7] = (CPU_INT08U)((CAN_RXMDH >> 24) /* get byte 7                           */
                                        & 0x000000FF);
        }                                             /*------------------------------------------*/
        RD_REG(TempBuf, CAN_RX_MSGID);                /* read message identifier register         */
        if ((TempBuf & 0x80000000) != 0) {            /* check for extended ID (IDE bit)          */
            frm->Identifier = (CPU_INT32U)(TempBuf &  /* get message id                           */
                                         0x1FFFFFFF) |
                                        (1L<<29);     /* set extended indentifier bit             */
        } else {                                      /* otherwise: standard ID                   */
            frm->Identifier = (CPU_INT32U)(TempBuf >> 18) & /* get message id                     */
                                          0x000007FF;
        }                                             /*------------------------------------------*/
        TempBuf = CAN_RMP &                           /* clear receive message pending register   */
                  (1L << TMS_28XX_ECAN_RX_BUFFER);
        CAN_RMP = TempBuf;
                                                      /*------------------------------------------*/
        CPU_CRITICAL_EXIT();                          /* exit critical section                    */
                                                      /*------------------------------------------*/
        result = sizeof(TMS28XX_ECAN_FRM);            /* setup result                             */
    }                                                 /*------------------------------------------*/
    return(result);                                   /* Return function result                   */
}                                                     /*------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      TMS28XX ECAN WRITE DATA
* \ingroup  TMS28XX_ECAN
*
*           Write a message to a message buffer after checking, that this  device is opened.
*
* \param    devId             The device identifier, returned by TMS28XXCANOpen()
* \param    buf               Byte array for transmitting data
* \param    size              Length of can frame memory
*
* \return   Errorcode (-1 for error, length of can frame memory for success)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S TMS28XXCANWrite (CPU_INT16S devId,
                            CPU_INT08U *buf,
                            CPU_INT16U size)
{                                                     /*------------------------------------------*/
    CPU_INT32U           TempBuf;                     /* Local: temporary Buffer                  */
    CPU_INT16S           result         = 0;          /* Local: Function result                   */
    TMS28XX_ECAN_REGS   *ECanRegPtr;                  /* Local: Pointer to eCAN Registers         */
    TMS28XX_ECAN_MBOX   *ECanMBoxPtr;                 /* Local: Pointer to Mailbox                */
    TMS28XX_ECAN_FRM    *frm;                         /* Local: Pointer to can frame              */
#if CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL
    CPU_SR               cpu_sr;                      /* Allocate storage for CPU status register */
#endif
                                                      /*------------------------------------------*/
#if TMS28XX_ECAN_ARG_CHK_CFG > 0
    if ((devId < 0) ||                                /* if devId out of range?                   */
        (devId >= TMS28XX_ECAN_DEV_N)) {
        DrvError = TMS28XX_CAN_BUS_ERR;               /* yes: set bus error                       */
        return(-1);                                   /* return error                             */
    }                                                 /*------------------------------------------*/
    if (buf == (void*)0) {                            /* if valid buffer pointer?                 */
        DrvError = TMS28XX_CAN_ARG_ERR;               /* yes: set argument checking error         */
        return(-1);                                   /* return error                             */
    }                                                 /*------------------------------------------*/
    if (size != sizeof(TMS28XX_ECAN_FRM)) {           /* size in range?                           */
        DrvError = TMS28XX_CAN_NO_DATA_ERR;           /* yes: set no data error                   */
        return(-1);                                   /* return error                             */
    }                                                 /*------------------------------------------*/
#endif

    ECanRegPtr  = (TMS28XX_ECAN_REGS *)ECANPara[devId].ECANBase; /* Set pointer to eCAN registers */
    ECanMBoxPtr = (TMS28XX_ECAN_MBOX *)ECANPara[devId].MBoxBase; /* set pointer to base mailbox   */
    frm         = (TMS28XX_ECAN_FRM *)buf;            /* Set pointer to can frame                 */
                                                      /*------------------------------------------*/
    if ((ECANData[devId].Status &                     /* see, if CAN device is not opened         */
         TMS28XX_ECAN_BUSY) == 0) {
        DrvError = TMS28XX_CAN_OPEN_ERR;              /* yes: set device not opened error         */
        return(-1);                                   /* return error                             */
    }                                                 /*------------------------------------------*/
    if ((CAN_TRS &                                    /* if buffer is empty                       */
         (1L << TMS_28XX_ECAN_TX_BUFFER)) == 0){
        CPU_CRITICAL_ENTER();                         /* enter critical section                   */
        TMS28XX_ECAN_EALLOW();                        /* enable register access                   */
                                                      /*------------------------------------------*/
        TempBuf  = CAN_ME;                            /* disable the TX Mailbox                   */
        TempBuf &=~ (1L << TMS_28XX_ECAN_TX_BUFFER);
        CAN_ME   = TempBuf;
                                                      /*------------------------------------------*/
        RD_REG(TempBuf,CAN_TX_MSGCTRL);               /* read message control register            */
        TempBuf = (TempBuf & 0xFFF0) |                /* set DLC                                  */
                  (frm->DLC & 0x000F);
        WR_REG(CAN_TX_MSGCTRL,TempBuf);               /* write back to message control register   */
                                                      /*------------------------------------------*/
        if (frm->DLC >= 1) {                          /* if data bytes to be written              */
            TempBuf  =  ((CPU_INT32U)(frm->Data[0]    /* get bytes into buffer                    */
                                      & 0x00FF)) |
                        ((CPU_INT32U)(frm->Data[1]
                                      & 0x00FF) << 8) |
                        ((CPU_INT32U)(frm->Data[2]
                                      & 0x00FF) << 16) |
                        ((CPU_INT32U)(frm->Data[3]
                                      & 0x00FF) << 24);
            WR_REG(CAN_TXMDL,TempBuf);                /* write to message low register            */
        }                                             /*------------------------------------------*/
        if (frm->DLC >= 5) {                          /* if more than 4 bytes to be written       */
            TempBuf  = ((CPU_INT32U)(frm->Data[4]     /* get bytes into buffer                    */
                                     & 0x00FF)) |
                       ((CPU_INT32U)(frm->Data[5]
                                     & 0x00FF) << 8) |
                       ((CPU_INT32U)(frm->Data[6]
                                     & 0x00FF) << 16) |
                       ((CPU_INT32U)(frm->Data[7]
                                     & 0x00FF) << 24);
            WR_REG(CAN_TXMDH,TempBuf);                /* write to message high register           */
        }                                             /*------------------------------------------*/
        if (frm->Identifier > 0x7FF) {                /* check for extended ID                    */
            TempBuf = frm->Identifier | 0x80000000;   /* set ext identifier and IDE bit           */
        } else {                                      /* otherwise:                               */
            TempBuf = frm->Identifier<<18;            /* set std identifier                       */
        }                                             /*------------------------------------------*/
        WR_REG(CAN_TX_MSGID,TempBuf);                 /* write message identifier                 */
                                                      /*------------------------------------------*/
        TempBuf  = CAN_ME;                            /* read mailbox enable register             */
        TempBuf |= (1L << TMS_28XX_ECAN_TX_BUFFER);   /* enable TX mailbox                        */
        CAN_ME   = TempBuf;                           /* write back to mailbox enable register    */
        CAN_TRS  = (1L << TMS_28XX_ECAN_TX_BUFFER);   /* Set TRS for TX Mailbox                   */
        TMS28XX_ECAN_EDIS();                          /* disable register access                  */
        CPU_CRITICAL_EXIT();                          /* exit critical section                    */
                                                      /*------------------------------------------*/
        result = sizeof(TMS28XX_ECAN_FRM);            /* setup result                             */
    } else {                                          /* otherwise: device is busy                */
        DrvError = TMS28XX_CAN_BUSY_ERR;              /* yes: set device busy error               */
        return(-1);                                   /* return error                             */
    }                                                 /*------------------------------------------*/
    return(result);                                   /* Return function result                   */
}                                                     /*------------------------------------------*/


