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
*********************************************************************************************************
*                                           CAN DRIVER CODE
*
*                                            Renesas RX600
*
* Filename : drv_can.c
* Version  : V2.42.01
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              INCLUDES
*********************************************************************************************************
*/

#include  <drv_can.h>
#include  "drv_def.h"
#include  "drv_can_reg.h"
#include  <lib_def.h>
#include  <lib_mem.h>
#include  "can_bus.h"


/*
*********************************************************************************************************
*                                               MACROS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                         INTERNAL DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             LOCAL DATA
*********************************************************************************************************
*/

static  const  CPU_INT32U          RX600_DrvId = 0x243F2801u;   /* Unique Driver Identification Code                    */

static         RX600_CAN_ERR       RX600_DrvErr;                /* Holds Detailed Error Code if Detected                */
                                                                /* Array Holds Driver Runtime Data                      */
static         RX600_CAN_DATA      RX600_DevData[RX600_CAN_N_DEV];

                                                                /* ---------- CAN MAILBOX CONFIG GLOBAL ARRAY --------- */
#if (RX600_CAN_CFG_MB_EN == DEF_ENABLED)

    #if (CAN_MODULE_CHANNEL_0 == DEF_ENABLED)
static         RX600_CAN_MB_ARRAY  CAN0_MB_ARRAY_CFG[RX600_CAN_NBR_MBOX] = CAN0_MB_ARRAY;
    #endif

    #if (CAN_MODULE_CHANNEL_1 == DEF_ENABLED)
static         RX600_CAN_MB_ARRAY  CAN1_MB_ARRAY_CFG[RX600_CAN_NBR_MBOX] = CAN1_MB_ARRAY;
    #endif

    #if (CAN_MODULE_CHANNEL_2 == DEF_ENABLED)
static         RX600_CAN_MB_ARRAY  CAN2_MB_ARRAY_CFG[RX600_CAN_NBR_MBOX] = CAN2_MB_ARRAY;
    #endif
#endif


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/
                                                                /* ---------------- MAILBOX SETTINGS ------------------ */
#define  RX600_CAN_TX_FIFO                  24u                 /* Tx FIFO Mailbox                                      */
#define  RX600_CAN_RX_FIFO                  28u                 /* Rx FIFO Mailbox                                      */


/*
*********************************************************************************************************
*                                              FUNCTIONS
*********************************************************************************************************
*/

static  CPU_INT08U   RX600_CAN_MailSearch(CPU_INT32U             para_id,
                                          RX600_CAN_MAIL_SELECT  mail_mode);

static  CPU_BOOLEAN  RX600_CAN_SetMode   (CPU_INT32U             para_id,
                                          RX600_CAN_MODE         mode);


/*
*********************************************************************************************************
*                                           RX600_CAN_Init()
*
* Description : Initializes the CAN Driver with the given Device Name
*
* Argument(s) : para_id     Device ID. [RX600_CAN_BUS_0, RX600_CAN_BUS_1, RX600_CAN_BUS_2]
*
* Return(s)   : Error code:  0 = No Error
*                           -1 = Error Occurred
*
* Caller(s)   : CanBusEnable()
*
* Note(s)     : (1) If the CAN Mailbox Array is an incomplete Array (Meaning that the Max amount of Mailboxes
*                   isn't configured) then all remaining entries will be set to 'CAN_NONE' and can be
*                   configured to CAN_TX. The only case that this isn't true is when FIFO Mode is used. If
*                   FIFO is used, there can still be 'CAN_NONE' entries, as long as they don't conflict
*                   with the CAN FIFO Mailbox Registers.
*
*               (2) CAN Control Configuration:
*                       RX600_CAN_CTRL_MBM         = Sets Mailbox Mode to FIFO or NORMAL Mode when Cleared
*                       RX600_CAN_CTRL_IDFM_MIXED  = ID Format set to Mixed ID Mode
*                       RX600_CAN_CTRL_BOM         = Sets the Bus-Off Recovery Mode
*                       RX600_CAN_CTRL_TSPS_8BIT   = Time Stamp Prescaler set to every 8-bit time
*
*               (3) Initially all mailboxes will be set to 'Invalid' in the MKIVLR register. This means
*                   that without any ID they will not accept any incomming CAN Frame. However, if the
*                   Mailbox Array contains an RX Mailbox with NO initial ID Filter, that mailbox will
*                   be reconfigured to a 'Valid' Mask Mailbox in the MKIVLR, meaning it will recieve any
*                   all incomming CAN Frames.
*
*               (4) Tx Mailbox(es) are configured completely from the RX600_CAN_Write() function.
*********************************************************************************************************
*/

CPU_INT16S  RX600_CAN_Init (CPU_INT32U  para_id)
{
    RX600_CAN_REG       *p_reg;
    RX600_CAN_BAUD       baud;
    RX600_CAN_MB_ARRAY  *p_mbox;
    CPU_INT08U           i;
    CPU_INT08U           j;
    CPU_BOOLEAN          tx_mb_chk;
    CPU_BOOLEAN          rx_mb_chk;
    CPU_BOOLEAN          drv_err;
    CPU_INT16S           result;
#if (RX600_CAN_CFG_MB_EN == DEF_ENABLED)
#if (CAN_MODULE_CHANNEL_0 == DEF_ENABLED)    
    RX600_CAN_MB_ARRAY   dflt_can0_mb[RX600_CAN_NBR_MBOX] = CAN0_MB_ARRAY;
#endif
#if (CAN_MODULE_CHANNEL_1 == DEF_ENABLED)
    RX600_CAN_MB_ARRAY   dflt_can1_mb[RX600_CAN_NBR_MBOX] = CAN1_MB_ARRAY;
#endif
#if (CAN_MODULE_CHANNEL_2 == DEF_ENABLED)    
    RX600_CAN_MB_ARRAY   dflt_can2_mb[RX600_CAN_NBR_MBOX] = CAN2_MB_ARRAY;
#endif
#endif
    
    
    result    = -1;                                             /* Initialize Variable(s)                               */
    tx_mb_chk = DEF_NO;
    rx_mb_chk = DEF_NO;
    i         = 0u;

    switch (para_id) {
        
#if  (CAN_MODULE_CHANNEL_0 == DEF_ENABLED)
        case RX600_CAN_BUS_0:
             RX600_DevData[RX600_CAN_BUS_0].RegPtr = (RX600_CAN_REG *)RX600_CAN0_ADDR;
             
    #if (RX600_CAN_CFG_MB_EN == DEF_ENABLED)
             Mem_Clr((void *)&CAN0_MB_ARRAY_CFG[0], sizeof(CAN0_MB_ARRAY_CFG));
             Mem_Copy((void *)&CAN0_MB_ARRAY_CFG[0], (void *)&dflt_can0_mb, sizeof(CAN0_MB_ARRAY_CFG));
             p_mbox = &CAN0_MB_ARRAY_CFG[0u];
    #endif
             break;
#endif
             
#if  (CAN_MODULE_CHANNEL_1 == DEF_ENABLED)
        case RX600_CAN_BUS_1:
             RX600_DevData[RX600_CAN_BUS_1].RegPtr = (RX600_CAN_REG *)RX600_CAN1_ADDR;
             
    #if (RX600_CAN_CFG_MB_EN == DEF_ENABLED)
             Mem_Clr((void *)&CAN1_MB_ARRAY_CFG[0], sizeof(CAN1_MB_ARRAY_CFG));
             Mem_Copy((void *)&CAN1_MB_ARRAY_CFG[0], (void *)&dflt_can1_mb, sizeof(CAN1_MB_ARRAY_CFG));
             p_mbox = &CAN1_MB_ARRAY_CFG[0u];
    #endif
             break;
#endif
             
#if (CAN_MODULE_CHANNEL_2 == DEF_ENABLED)
        case RX600_CAN_BUS_2:
             RX600_DevData[RX600_CAN_BUS_2].RegPtr = (RX600_CAN_REG *)RX600_CAN2_ADDR;
             
    #if (RX600_CAN_CFG_MB_EN == DEF_ENABLED)
             Mem_Clr((void *)&CAN2_MB_ARRAY_CFG[0], sizeof(CAN2_MB_ARRAY_CFG));
             Mem_Copy((void *)&CAN2_MB_ARRAY_CFG[0], (void *)&dflt_can2_mb, sizeof(CAN2_MB_ARRAY_CFG));
             p_mbox = &CAN2_MB_ARRAY_CFG[0u];
    #endif
             break;
#endif
                
        default:
             RX600_DrvErr = RX600_CAN_ERR_INIT;                 /* Check if CAN Device is out of Range.                 */
             return (result);
    }
    
#if (RX600_CAN_CFG_MB_EN != DEF_ENABLED)                        /* If No Advanced Mbox Cfg, use Default Configuration.  */
    p_mbox = &CANx_MB_ARRAY_CFG[0u];
#endif
    
    while (i < RX600_CAN_NBR_MBOX) {                            /* Check if Array has at LEAST 1 Rx Mbox & 1 Tx Mbox.   */
        if (rx_mb_chk != DEF_YES) {
            if (p_mbox[i].tx_rx == CAN_RX) {
                rx_mb_chk = DEF_YES;
            }
        }
        
        if (tx_mb_chk != DEF_YES) {
            if ((p_mbox[i].tx_rx == CAN_TX) || (p_mbox[i].tx_rx == CAN_NONE)) {
                
#if (CAN_MAILBOX_MODE == FIFO_MAILBOX_MODE)                     /* See Note (1).                                        */
                if (i < RX600_CAN_TX_FIFO) {
                    tx_mb_chk = DEF_YES;
                    RX600_DevData[para_id].AvailTxMBox = i;     /* Set First Mailbox Available for Tx.                  */
                }
#else
                tx_mb_chk = DEF_YES;
                RX600_DevData[para_id].AvailTxMBox = i;         /* Set First Mailbox Available for Tx.                  */
#endif
            }
        }
                                                                /* If Array has both (1 Rx) & (1 Tx) Mbox, exit loop.   */
        if ((tx_mb_chk == DEF_YES) && (rx_mb_chk == DEF_YES)) {
            i = RX600_CAN_NBR_MBOX;
        } else {
            i++;
        }
    }
    
    if ((tx_mb_chk != DEF_YES) || (rx_mb_chk != DEF_YES)) {
        RX600_DrvErr = RX600_CAN_ERR_MAILBOX;                   /* There MUST be at least 1 Tx & Rx Mbox. Mbox Cfg Err  */
        return (result);
    }

#if (CAN_MAILBOX_MODE == FIFO_MAILBOX_MODE)
    for (i = RX600_CAN_TX_FIFO; i < RX600_CAN_NBR_MBOX; i++) {
        if (p_mbox[i].tx_rx == CAN_RX) {                        /* Check if RX Mailbox Config Array Conflict with FIFO  */
            RX600_DrvErr = RX600_CAN_ERR_MAILBOX;
            return (result);
        } else if (p_mbox[i].tx_rx == CAN_TX) {                 /* Check if RX Mailbox Config Array Conflict with FIFO  */
            RX600_DrvErr = RX600_CAN_ERR_MAILBOX;
            return (result);
        } else {
            ;
        }
    }
#endif
                                                                /* ----------------- PIN CONFIGURATION ---------------- */
    RX600_DevData[para_id].Use      = DEF_NO;                   /* Set Proper Can Device to UNUSED Status               */
    RX600_DevData[para_id].InitCmpl = DEF_FALSE;                /* Re-Initialize Ready Transmit Variable                */
    p_reg = RX600_DevData[para_id].RegPtr;                      /* Set Base Address for CAN Device(s)                   */
    
                                                                /* No Error with Mailboxes, Proceed with Init           */
    RX600_DrvErr = RX600_CAN_ERR_NONE;                          /* Reset Driver Error                                   */
    
    drv_err = RX600_CAN_PinSetting(para_id);                    /* Configure Pin Settings for CAN Device                */
    if (drv_err == DEF_FAIL) {
        RX600_DrvErr = RX600_CAN_ERR_FUNC;                      /* PinSettings Timeout, Return Error                    */
        return (result);
    }
                                                                /* Exit Sleep Mode                                      */
    drv_err = RX600_CAN_SetMode(para_id, RX600_CAN_MODE_EXIT_SLEEP);
    if (drv_err == DEF_FAIL) {
        RX600_DrvErr = RX600_CAN_ERR_FUNC;                      /* CAN Exit Sleep Timeout, Return Error                 */
        return (result);
    }
                                                                /* Enter Reset Mode                                     */
    drv_err = RX600_CAN_SetMode(para_id, RX600_CAN_MODE_RESET);
    if (drv_err == DEF_FAIL) {      
        RX600_DrvErr = RX600_CAN_ERR_FUNC;                      /* CAN Reset Mode Timeout, Return Error                 */
        return (result);
    }
                                                                /* ------------------ CAN CTRL CONFIG ----------------- */
#if (CAN_MAILBOX_MODE == FIFO_MAILBOX_MODE)
    DEF_BIT_SET(p_reg->CTLR, RX600_CAN_CTRL_MBM);               /* Sets CAN Mailbox Mode Bit to FIFO Mode               */
#else
    DEF_BIT_CLR(p_reg->CTLR, RX600_CAN_CTRL_MBM);               /* Sets CAN Mailbox Mode Bit to Normal Mode             */
#endif
                                                                /* Set Control Register Config. See Note(2)             */
    DEF_BIT_SET(p_reg->CTLR,(RX600_CAN_CTRL_IDFM_MIXED      | \
                             RX600_CAN_CTRL_BOM_BUS_OFF_END | \
                             RX600_CAN_CTRL_TSPS_8BIT));    

#if (CAN_MB_MSG_LOST_MODE > 0u)                                 /*      - Overrun vs Overwrite Mode -                   */
    DEF_BIT_SET(p_reg->CTLR, RX600_CAN_CTRL_MLM);               /* Sets Message Lost Mode : Overrun   Mode.             */
#else
    DEF_BIT_CLR(p_reg->CTLR, RX600_CAN_CTRL_MLM);               /* Sets Message Lost Mode : Overwrite Mode.             */
#endif
    
                                                                /* -------------- BAUDRATE CONFIGURATION -------------- */
    baud.BaudRate         = CAN_DEFAULT_BAUDRATE;               /* Set Default Baud Rate Settings                       */
    baud.SamplePoint      = CAN_DEFAULT_SP;
    baud.ReSynchJumpWidth = CAN_DEFAULT_RJW;

    drv_err = RX600_CAN_CalcTimingReg(&baud);                   /* Calculate Bit Timing Register Values                 */
    if (drv_err == DEF_FAIL) {
        RX600_DrvErr = RX600_CAN_ERR_FUNC;                      /* Bit Timing Error, Return Error                       */
        return (result);
    }
    
    p_reg->BCR = (baud.PrescalerDiv   << 16u) |                 /* Set Bit Timing Register                              */
                 (baud.SJW            << 12u) |
                 (baud.PhaseBufSeg1   << 28u) |
                 (baud.PhaseBufSeg2   << 8u);
                                                                /* ---------- DISABLE, CLR, & SET INTERRUPTS ---------- */
    p_reg->MIER = 0u;                                           /* Disable All Interrupts                               */
    p_reg->EIER = 0u;                                           /* Disable All Error Interrupts                         */
    p_reg->EIFR = 0u;                                           /* Reset Error Interrupt Factor Judge Register          */
    p_reg->RECR = 0u;                                           /* Reset Rx Error Count Register                        */
    p_reg->TECR = 0u;                                           /* Reset Tx Error Count Register                        */
    
#if ((CANBUS_RX_HANDLER_EN > 0) || \
     (CANBUS_TX_HANDLER_EN > 0) || \
     (CANBUS_NS_HANDLER_EN > 0))
    RX600_CAN_IntSetting(para_id);                              /* Configure Interrupt Settings                         */
#endif
                                                                /* Enter Halt Mode                                      */
    drv_err = RX600_CAN_SetMode(para_id, RX600_CAN_MODE_HALT);
    if (drv_err == DEF_FAIL) {
        RX600_DrvErr = RX600_CAN_ERR_FUNC;                      /* CAN Halt Mode Timeout, Return Error                  */
        return (result);
    }
                                                                /* --------------- MAILBOX CONFIGURATION -------------- */
    p_reg->MKIVLR = DEF_INT_32_MASK;                            /* Initally, Set All Mailbox Masks as Invalid. Note (3) */
    
    for (i = 0u; i < RX600_CAN_NBR_MBOX; i++) {                 /* Configure Mailboxes in Halt Mode                     */
        p_reg->MCTL[i]   = 0u;                                  /* Clear Message Control Register(s)                    */
        p_reg->MB[i].ID  = 0u;                                  /* Clear Mailboxes of all Data                          */
        p_reg->MB[i].DLC = 0u;
        for (j = 0u; j < 8u; j++) {
            p_reg->MB[i].DATA[j] = 0u;
        }
        for (j = 0u; j < 2u; j++) {
            p_reg->MB[i].TS = 0u;
        }
    }
                               
    for (i = 0u; i < RX600_CAN_NBR_MBOX; i++) {                 /* Configure Mailboxes set for Reception, see Note (4). */
        if (p_mbox[i].tx_rx == CAN_RX) {
            if (p_mbox[i].ID != DEF_BIT_NONE) {                 /* See Note (2).                                        */
                if(p_mbox[i].ID > RX600_CAN_SID_LIMIT) {        /* If Mailbox has Extended ID                           */
                    p_mbox[i].ID &= RX600_CAN_EID_LIMIT;        /* Set Extended ID Flag in MB.ID register.              */
                    DEF_BIT_SET(p_mbox[i].ID, RX600_CAN_MB_ID_IDE);
                } else {                                        /* Else, Mailbox has Standard ID.                       */
                    p_mbox[i].ID = (p_mbox[i].ID & RX600_CAN_SID_LIMIT) << RX600_CAN_SID_SHIFT;
                }
                
                p_reg->MB[i].ID = p_mbox[i].ID;                 /* Configure Controller Mailbox with Proper ID.         */
            } else {
                                                                /* Validate Mask to recieve all CAN Frames when ID = 0. */
                DEF_BIT_CLR(p_reg->MKIVLR, RX600_CAN_MKIVLR_MB(i));
            }
                                                                /* Configure Mailbox for Reception                      */
            DEF_BIT_SET(p_reg->MCTL[i],  RX600_CAN_MCTL_RECREQ);
                                                                /* Set No New Data Recieved, No Message Lost            */
            DEF_BIT_CLR(p_reg->MCTL[i], (RX600_CAN_MCTL_NEWDATA | \
                                         RX600_CAN_MCTL_MSGLOST));  
        }
    }

#if (CAN_MAILBOX_MODE == FIFO_MAILBOX_MODE)
    p_reg->MKIVLR &= RX600_CAN_FIFO_MSK_VALID;                  /* Set Mailbox Masks Valid for FIFO Registers           */
#endif

    for (i = 0u; i < RX600_CAN_NBR_MASK; i++) {                 /* Reset All Masks                                      */
        p_reg->MKR[i] = 0u;
    }
    
#if (CAN_MAILBOX_MODE == FIFO_MAILBOX_MODE)                     /* ------------- MILBOX FIFO CONFIGURATION ------------ */
    DEF_BIT_SET(p_reg->RFCR, RX600_CAN_RFCR_RFE);               /* Enable Rx FIFO                                       */
    p_reg->FIDCR0 = 0u;                                         /* Reset FIFO Rx ID Compare Register 1                  */
    p_reg->FIDCR1 = 0u;                                         /* Reset FIFO Rx ID Compare Register 2                  */
    DEF_BIT_SET(p_reg->TFCR, RX600_CAN_TFCR_TFE);               /* Enable Tx FIFO                                       */
#endif
                                                                /* --------------- ENABLE RX INTERRUPTS --------------- */
    for (i = 0u; i < RX600_CAN_NBR_MBOX; i++) {                 /* Enable RX Mailbox Interrupts                         */
        if (p_mbox[i].tx_rx == CAN_RX) {                        /* Configure Mailboxes set for Reception, see Note (4). */
            DEF_BIT_SET(p_reg->MIER, RX600_CAN_MAILBOX_MIER_INT_EN(i));
        }
    }

#if (CAN_MAILBOX_MODE == FIFO_MAILBOX_MODE)
    DEF_BIT_SET(p_reg->MIER, RX600_CAN_MIER_RX_FIFO_INT_EN);    /* Enable Rx FIFO Interrupt                             */
    DEF_BIT_SET(p_reg->MIER, RX600_CAN_MIER_TX_FIFO_INT_EN);    /* Enable Tx FIFO Interrupt                             */
#endif
                                                                /* ----------------- ENABLE CAN MODULE ---------------- */
#if (CAN_TEST_MODE > 0)                                         /* If CAN Test Mode is Enabled                          */
    if (CAN_TEST_MODE == CAN_TEST_INTERNAL) {
        p_reg->TCR = RX600_CAN_TCR_TSTM_INTERNAL_LOOP;          /* Test Mode Select: Internal Loopback                  */
    } else if (CAN_TEST_MODE == CAN_TEST_EXTERNAL) {
        p_reg->TCR = RX600_CAN_TCR_TSTM_EXTERNAL_LOOP;          /* Test Mode Select: External Loopback                  */
    } else {
        p_reg->TCR = RX600_CAN_TCR_TSTM_OTHER_CAN_TEST;         /* Test Mode Select: Disabled                           */
    }
    DEF_BIT_SET(p_reg->TCR, RX600_CAN_TCR_TSTE);                /* Enable CAN Test Mode                                 */
#endif

    result = RX600_CAN_ERR_NONE;                                /* Set Function Result: No Error                        */
    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                           RX600_CAN_Open()
*
* Description : Unlocks the CAN device, i.e. Read/Write/IoCtl Functions will take effect
*
* Argument(s) : dev_id      Bus Node Name, used to interface with the CAN Bus Layer.
*
*               dev_name    Driver Device Name, used to interface with the Lowlevel Device Driver.
*                           Possible Values for Device Name:
*                                   RX600_CAN_BUS_0     [CAN Bus 0]
*                                   RX600_CAN_BUS_1     [CAN Bus 1]
*                                   RX600_CAN_BUS_2     [CAN Bus 2]
*
*               mode        Mode in which CAN device will be used. Possible Modes are:
*                                   DEV_RW              [EXCLUSIVE READ/WRITE ACCESS]
*                                   DEV_RWX             [EXCLUSIVE READ/WRITE/EXECUTE ACCESS]
*                                   DEV_SHWO            [SHARED WRITE ACCESS]
*                                   DEV_SHRO            [SHARED READ ACCESS]
*                                   DEV_SHRW            [SHARED READ/WRITE ACCESS]
*                                   DEV_SHRWX           [SHARED READ/WRITE/EXECUTE ACCESS]
*
* Return(s)   : Parameter Identifier for further access or (-1) if error occured.
*
* Caller(s)   : CanCfg in can_cfg.c
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT16S  RX600_CAN_Open (CPU_INT16S  dev_id,
                            CPU_INT32U  dev_name,
                            CPU_INT16U  mode)
{
    CPU_INT16S  result;
    CPU_SR_ALLOC();

    
    result = -1;                                                /* Initializing Variable(s)                             */
    
    if (dev_name >= RX600_CAN_N_DEV) {                          /* Check if Device Name is out of Range                 */                        
        RX600_DrvErr = RX600_CAN_ERR_BUS;
        return (result);
    }   

    if (mode != DEV_RW) {                                       /* Check if Mode is not Supported                       */
        RX600_DrvErr = RX600_CAN_ERR_MODE;
        return (result);
    }

    CPU_CRITICAL_ENTER();

    if (RX600_DevData[dev_name].Use == DEF_NO) {                /* Check if CAN Device is Unused                        */
        RX600_DevData[dev_name].Use =  DEF_YES;                 /* Mark CAN Device as Used                              */

#if ((CANBUS_RX_HANDLER_EN > 0) || \
     (CANBUS_TX_HANDLER_EN > 0) || \
     (CANBUS_NS_HANDLER_EN > 0))
        
        RX600_CAN_SetDevIds((CPU_INT08U)dev_id,                 /* Set Device IDs for the ISRs                          */
                            (CPU_INT08U)dev_name);
#else
        (void)&dev_id;                                          /* Prevent Compiler Warning                             */
#endif
        
        result = (CPU_INT16S)dev_name;                          /* OK, Device is Opened                                 */
    } else {
        RX600_DrvErr = RX600_CAN_ERR_OPEN;
    }
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                           RX600_CAN_Close()
*
* Description : Locks the CAN device, i.e. Read/Write/IoCtl Functions will not take effect
*
* Argument(s) : para_id     Parameter Identifier, returned by RX600_CAN_Open()
*
* Return(s)   : Error code:  0 = No Error
*                           -1 = Error Occurred
*
* Caller(s)   : CanCfg in can_cfg.c
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT16S  RX600_CAN_Close (CPU_INT16S  para_id)
{
    CPU_INT16S  result;
    CPU_SR_ALLOC();

    
    result = -1;                                                /* Initializing Variable(s)                             */

    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= RX600_CAN_N_DEV)) {
        RX600_DrvErr = RX600_CAN_ERR_BUS;
        return (result);
    }

    CPU_CRITICAL_ENTER();
    
    if (RX600_DevData[para_id].Use != DEF_NO) {                 /* Check if CAN Device is Used                          */
        RX600_DevData[para_id].Use  = DEF_NO;                   /* Mark CAN Device as Unused                            */
        result = RX600_CAN_ERR_NONE;                            /* OK, Device is Closed                                 */
    } else {
        RX600_DrvErr = RX600_CAN_ERR_CLOSE;
    }
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                           RX600_CAN_IoCtl()
*
* Description : Performs Special Action on the Opened Device. The Function Code 'func' defines 
*               what the caller wants to do. Description of Function Codes are defined in the header.
*
* Argument(s) : para_id     Parameter Identifier, returned by RX600_CAN_Open().
*
*               func        Function Code.
*
*               p_arg       Argument List, Specific to the Function Code.
*
* Return(s)   : Error code:  0 = No Error
*                           -1 = Error Occurred
*
* Caller(s)   : CanCfg in can_cfg.c
*
* Note(s)     : (1) At Startup (especially using Normal Mode), when the first IO_RX600_CAN_TX_READY 
*                   Function Code is set, the Normal Tx Mailbox(es) have not been configured to 
*                   transmit [since this is done in the xxxCANWrite()], therefore the Tx Ready flag
*                   will be initially set to transmit the first frame to configure the Tx Mailbox
*                   and will do this everytime the SENTDATA bit in the STR Register is NOT set. If
*                   SENTDATA bit is not set but there is still a Tx Mailbox that can be configured
*                   then the TX READY will be set and then the Tx Mailbox will be configured in 
*                   the xxxCANWrite().
*
*               (2) If p_arg is set to '0' improperly (meaning that the p_arg is passed a '0'
*                   instead of assigning a local variable the value of '0' and passing the variable)
*                   then the p_arg will address '0' but the value in that address will be a junk
*                   value. Therefore, if mailbox > Max # of Mailboxes [32] then it will set all
*                   Mailboxes with the Standard or Extended Flag, based on the Function Code.
*                       Below will be an example of an improper assignment of Mailbox [0] which
*                       will result in all Mailboxes being set to Standard ID (in this example).
*                           Improper Mailbox[0] Assignment:
*                               xxx_CAN_IoCtl(para_id, IO_RX600_CAN_RX_STANDARD, 0u);
*
*                           Proper Mailbox[0] Assignment:
*                               mbox_var = 0u;
*                               xxx_CAN_IoCtl(para_id, IO_RX600_CAN_RX_STANDARD, &mbox_var);
*
*               (3) The Mask Register is in charge of Acceptance Filtering for RX Mailboxes.
*                   (A) The Mask Register filters the Mailboxes by comparing the Mailbox ID 
*                       with the corresponding Recieved ID based on whether the EID[17:0] 
*                       or SID[10:0] are set to '1' or '0'.
*                           The relationship between Mask Register and Mailbox is as follows:
*                               Mask Register MKR[x] | Mailbox(es) MB[y]
*                               ----------------------------------------
*                                         0          |      0-3
*                                         1          |      4-7
*                                         2          |      8-11
*                                         3          |      12-15
*                                         4          |      16-19
*                                         5          |      20-23
*                                         6          |      24-27
*                                         7          |      28-31
*
*                   (B) In FIFO Mode, Normal Mailboxes use the single corresponding registers
*                       (MKR[0]-> MKR[5]) for Acceptance Filtering. Recieve FIFO Mailboxes
*                       (MB[28]-> MB[31]) use two registers (MKR[6] & MKR[7]) for Acceptance Filtering.
*********************************************************************************************************
*/

CPU_INT16S  RX600_CAN_IoCtl (CPU_INT16S   para_id,
                             CPU_INT16U   func,
                             void        *p_arg)
{
    RX600_CAN_REG       *p_reg;
    RX600_CAN_BAUD       baud;
    RX600_CAN_MB_ARRAY  *p_mbox;
    CPU_INT32U           mask;
    CPU_INT32U           can_id;
    CPU_INT32U           mailbox;
    CPU_INT08U           init_cmpl;
    CPU_INT08U           i;
    CPU_BOOLEAN          drv_err;
    CPU_INT16S           result;
    CPU_SR_ALLOC();


     result  = -1;                                              /* Initialize Variable(s)                               */
     drv_err = DEF_OK;
    
    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= RX600_CAN_N_DEV)) {
        RX600_DrvErr = RX600_CAN_ERR_BUS;
        return (result);
    }

    if (RX600_DevData[para_id].Use != DEF_YES) {                /* Check if CAN Device is Opened                        */
        RX600_DrvErr = RX600_CAN_ERR_OPEN;
        return (result);
    }
    
    p_reg     = RX600_DevData[para_id].RegPtr;                  /* Set Base Address for CAN Device(s)                   */
    init_cmpl = RX600_DevData[para_id].InitCmpl;                /* Set Tx Init MBox Variable to CAN Controller          */
    
    CPU_CRITICAL_ENTER();

    switch (func) {                                             /*                SELECT: FUNCTION CODE                 */
        case IO_RX600_CAN_GET_IDENT:                            /* -------------------- GET IDENT --------------------- */
             *(CPU_INT32U*)p_arg = RX600_DrvId;                 /* Return Driver Identification Code                    */
             result = RX600_CAN_ERR_NONE;                       /* Indicate Successful Function Execution               */
             break;
             
             
        case IO_RX600_CAN_GET_ERRNO:                            /* ------------------ GET ERRORCODE ------------------- */
             *(CPU_INT16U*)p_arg = RX600_DrvErr;                /* Return Last Detected Error Code                      */
             result = RX600_CAN_ERR_NONE;                       /* Indicate Successful Function Execution               */
             break;
              
             
        case IO_RX600_CAN_GET_DRVNAME:                          /* ----------------- GET DRIVER NAME ------------------ */
                                                                /* Return Human Readable Driver Name                    */
             *(CPU_INT08U**)p_arg = (CPU_INT08U*)RX600_CAN_NAME;
             result = RX600_CAN_ERR_NONE;                       /* Indicate Successful Function Execution               */
             break;
               
             
        case IO_RX600_CAN_SET_BAUDRATE:                         /* ------------------ SET BAUD RATE ------------------- */

             baud.BaudRate         = *((CPU_INT32U *)p_arg);
             baud.SamplePoint      = CAN_DEFAULT_SP;
             baud.ReSynchJumpWidth = CAN_DEFAULT_RJW;

             drv_err = RX600_CAN_CalcTimingReg(&baud);          /* Calculate Bit Timing Register Values                 */
             if (drv_err == DEF_FAIL) {
                 RX600_DrvErr = RX600_CAN_ERR_FUNC;             /* Bit Timing Error, Return Error                       */
                 break;
             }
                                                                /* Enter Halt Mode                                      */
             drv_err = RX600_CAN_SetMode(para_id, RX600_CAN_MODE_HALT);
             if (drv_err == DEF_FAIL) {
                 RX600_DrvErr = RX600_CAN_ERR_FUNC;             /* CAN Halt Mode Timeout, Return Error                  */
                 break;
             }
    
             p_reg->BCR = (baud.PrescalerDiv   << 16u) |        /* Set Bit Timing Register                              */
                          (baud.SJW            << 12u) |
                          (baud.PhaseBufSeg1   << 28u) |
                          (baud.PhaseBufSeg2   << 8u);
                                                                /* Back to Operating Mode                               */
             drv_err = RX600_CAN_SetMode(para_id, RX600_CAN_MODE_OPERATE);          
             break;
            

        case IO_RX600_CAN_TX_READY:                             /* -------------------- TX READY ---------------------- */
                                                                /* Mailbox(es) w/ Data to send, OK to Transmit          */
             if (DEF_BIT_IS_SET(p_reg->STR, RX600_CAN_STR_SDST) == DEF_YES) {
                 *((CPU_INT08U *)p_arg) = 1u;
                
             } else if (init_cmpl == DEF_FALSE) {               /* Initialize New Tx Mbox Transfer. See Note(1)         */
                 *((CPU_INT08U *)p_arg) = 1u;                   /* New Tx Mbox is able to be Configured                 */
                                                        
                                                                /* Tx FIFO is Not Full, OK to Transmit                  */
             } else if (DEF_BIT_IS_SET(p_reg->STR, RX600_CAN_STR_TFST) == DEF_YES) {
                 *((CPU_INT08U *)p_arg) = 1u;
             } else {                                           /* Tx FIFO is Full, No Data to Send                     */
                 *((CPU_INT08U *)p_arg) = 0u;
             }
          
             result = RX600_CAN_ERR_NONE;                       /* Indicate Successful Function Execution               */
             break;

             
        case IO_RX600_CAN_START:                                /* ------------- START CAN COMMUNICATION -------------- */
                                                                /* Enter Operating Mode                                 */
             drv_err = RX600_CAN_SetMode(para_id, RX600_CAN_MODE_OPERATE);
             break;

             
        case IO_RX600_CAN_STOP:                                 /* ------------- STOP CAN COMMUNICATION --------------- */
                                                                /* Enter Halt Mode                                      */
             drv_err = RX600_CAN_SetMode(para_id, RX600_CAN_MODE_HALT);
             break;

             
        case IO_RX600_CAN_RX_STANDARD:                          /* --------------- SET RX STANDARD -------------------- */
                                                                /* Enter Halt Mode                                      */
             drv_err = RX600_CAN_SetMode(para_id, RX600_CAN_MODE_HALT);
             if (drv_err == DEF_FAIL) {
                 result = RX600_CAN_ERR_FUNC;                   /* CAN Halt Mode Timeout, Return Error                  */
                 break;
             }

             mailbox = *((CPU_INT32U *)p_arg);                  /* Mailbox Desired for SID Configuration                */
             
             if (mailbox > RX600_CAN_NBR_MBOX) {                /* If p_arg = 0 set inproperly. See Note (2).           */
                 
#if (RX600_CAN_CFG_MB_EN != DEF_ENABLED)
                p_mbox = &CANx_MB_ARRAY_CFG[0u];                /* If No Advanced Mbox Cfg, use Default Configuration.  */
#else
                switch (para_id) {
                    
    #if  (CAN_MODULE_CHANNEL_0 == DEF_ENABLED)
                    case RX600_CAN_BUS_0:
                         p_mbox = &CAN0_MB_ARRAY_CFG[0u];
                         break;
    #endif
             
    #if  (CAN_MODULE_CHANNEL_1 == DEF_ENABLED)
                    case RX600_CAN_BUS_1:
                         p_mbox = &CAN1_MB_ARRAY_CFG[0u];
                         break;
    #endif

             
    #if (CAN_MODULE_CHANNEL_2 == DEF_ENABLED)
                    case RX600_CAN_BUS_2:
                         p_mbox = &CAN2_MB_ARRAY_CFG[0u];
                         break;
    #endif
                
                    default:
                         drv_err = DEF_FAIL;                    /* Return with Error, No Bus selected found.            */
                         break;
                }
                
                if (drv_err == DEF_FAIL) {
                    break;
                }
#endif
                
                for (i = 0u; i < RX600_CAN_NBR_MBOX; i++) {
                    if (p_mbox[i].tx_rx == CAN_RX) {            /* Set All Rx Mailboxes to Extended ID Configuration.   */
                        DEF_BIT_CLR(p_reg->MB[i].ID, RX600_CAN_MB_ID_IDE);
                    }
                }

             } else {
                 if (mailbox >= RX600_CAN_TX_FIFO) {            /* If FIFO is used, and MB is in FIFO territory.        */
                     
#if (CAN_MAILBOX_MODE == FIFO_MAILBOX_MODE)
                                                                /* Set Standard ID in Compare Registers 0 and 1.        */
                     DEF_BIT_CLR(p_reg->FIDCR0, RX600_CAN_FIDCR0_IDE);
                     DEF_BIT_CLR(p_reg->FIDCR1, RX600_CAN_FIDCR1_IDE);
#else
                     DEF_BIT_CLR(p_reg->MB[mailbox].ID, RX600_CAN_MB_ID_IDE);
#endif
                     
                 } else {
                                                                /* Set Mailbox to Standard ID Configuration             */
                     DEF_BIT_CLR(p_reg->MB[mailbox].ID, RX600_CAN_MB_ID_IDE);
                 }
             }
                                                                /* Back to Operating Mode                               */
             drv_err = RX600_CAN_SetMode(para_id, RX600_CAN_MODE_OPERATE);
             break;

             
        case IO_RX600_CAN_RX_EXTENDED:                          /* --------------- SET RX EXTENDED -------------------- */
                                                                /* Enter Halt Mode                                      */
             drv_err = RX600_CAN_SetMode(para_id, RX600_CAN_MODE_HALT);
             if (drv_err == DEF_FAIL) {
                 result = RX600_CAN_ERR_FUNC;                   /* CAN Halt Mode Timeout, Return Error                  */
                 break;
             }

             mailbox = *((CPU_INT32U *)p_arg);                  /* Mailbox Desired for EID Configuration                */
             
             if (mailbox > RX600_CAN_NBR_MBOX) {                /* If p_arg = 0 set inproperly. See Note (2).           */
                 
#if (RX600_CAN_CFG_MB_EN != DEF_ENABLED)
                p_mbox = &CANx_MB_ARRAY_CFG[0u];                /* If No Advanced Mbox Cfg, use Default Configuration.  */
#else
                switch (para_id) {
                    
    #if  (CAN_MODULE_CHANNEL_0 == DEF_ENABLED)
                    case RX600_CAN_BUS_0:
                         p_mbox = &CAN0_MB_ARRAY_CFG[0u];
                         break;
    #endif
             
    #if  (CAN_MODULE_CHANNEL_1 == DEF_ENABLED)
                    case RX600_CAN_BUS_1:
                         p_mbox = &CAN1_MB_ARRAY_CFG[0u];
                         break;
    #endif

             
    #if (CAN_MODULE_CHANNEL_2 == DEF_ENABLED)
                    case RX600_CAN_BUS_2:
                         p_mbox = &CAN2_MB_ARRAY_CFG[0u];
                         break;
    #endif
                
                    default:
                         drv_err = DEF_FAIL;                    /* Return with Error, No Bus selected found.            */
                         break;
                }
                
                if (drv_err == DEF_FAIL) {
                    break;
                }
#endif
                
                for (i = 0u; i < RX600_CAN_NBR_MBOX; i++) {
                    if (p_mbox[i].tx_rx == CAN_RX) {            /* Set All Rx Mailboxes to Extended ID Configuration.   */
                        DEF_BIT_SET(p_reg->MB[i].ID, RX600_CAN_MB_ID_IDE);
                    }
                }
                
             } else {
                 if (mailbox >= RX600_CAN_TX_FIFO) {            /* If FIFO is used, and MB is in FIFO territory.        */
                     
#if (CAN_MAILBOX_MODE == FIFO_MAILBOX_MODE)
                                                                /* Set Standard ID in Compare Registers 0 and 1.        */
                     DEF_BIT_SET(p_reg->FIDCR0, RX600_CAN_FIDCR0_IDE);
                     DEF_BIT_SET(p_reg->FIDCR1, RX600_CAN_FIDCR1_IDE);
#else
                     DEF_BIT_SET(p_reg->MB[mailbox].ID, RX600_CAN_MB_ID_IDE);
#endif
                     
                 } else {
                                                                /* Set Mailbox to Standard ID Configuration             */
                     DEF_BIT_SET(p_reg->MB[mailbox].ID, RX600_CAN_MB_ID_IDE);
                 }
             }
                                                                /* Back to Operating Mode                               */
             drv_err = RX600_CAN_SetMode(para_id, RX600_CAN_MODE_OPERATE);
             break;

             
        case IO_RX600_CAN_GET_NODE_STATUS:                      /* ---------------- GET NODE STATUS ------------------- */
             *((CPU_INT08U *)p_arg) = 0u;                       /* Set the Initial Argument Pointer to Zero             */

             if ((DEF_BIT_IS_SET(p_reg->EIFR, RX600_CAN_EIFR_EPIF) == DEF_YES) || 
                 (DEF_BIT_IS_SET(p_reg->STR, RX600_CAN_STR_EPST)   == DEF_YES)) {
                 *((CPU_INT08U *)p_arg) = 1u;                   /* CAN Module has reached Error Passive State           */
             }
             
             if ((DEF_BIT_IS_SET(p_reg->EIFR, RX600_CAN_EIFR_BOEIF) == DEF_YES) || 
                 (DEF_BIT_IS_SET(p_reg->STR, RX600_CAN_STR_BOST)    == DEF_YES)) {
                 *((CPU_INT08U *)p_arg) = 2u;                   /* CAN Module has reached Bus-Off State                 */
             }
            
             result = RX600_CAN_ERR_NONE;                       /* Indicate Successful Function Execution               */
             break;
            
             
        case IO_RX600_CAN_SET_RX_FILTER:                        /* ---------------- SET RX FILTER --------------------- */
             mailbox = ((CPU_INT32U *)p_arg)[0u];               /* Mailbox Desired for Filter Configuration             */
             can_id  = ((CPU_INT32U *)p_arg)[1u];               /* ID Value Desired for Mailbox                         */
             mask    = ((CPU_INT32U *)p_arg)[2u];               /* Mask Value for MKR Register                          */

             if (can_id > RX600_CAN_SID_LIMIT) {                /* Extended ID                                          */
                 mask   &= RX600_CAN_EID_LIMIT;
                 can_id &= RX600_CAN_EID_LIMIT;
                 DEF_BIT_SET(can_id, RX600_CAN_MB_ID_IDE);
             } else {                                           /* Standard ID                                          */
                 mask    = (mask   & RX600_CAN_SID_LIMIT) << RX600_CAN_SID_SHIFT;
                 can_id  = (can_id & RX600_CAN_SID_LIMIT) << RX600_CAN_SID_SHIFT;
             }
                                                                /* Remote Frame?                                        */
             if (DEF_BIT_IS_SET(((CPU_INT32U *)p_arg)[1u], RX600_CAN_MB_ID_RTR) == DEF_YES) {
                 DEF_BIT_SET(can_id, RX600_CAN_MB_ID_RTR);
             }
                                                                /* Enter Halt Mode                                      */
             drv_err = RX600_CAN_SetMode(para_id, RX600_CAN_MODE_HALT);
             if (drv_err == DEF_FAIL) {
                 result = RX600_CAN_ERR_FUNC;                   /* CAN Halt Mode Timeout, Return Error                  */
                 break;
             }
            
             p_reg->MB[mailbox].ID  = can_id;                   /* Set RX Filtering for Mailbox                         */
             p_reg->MKR[mailbox/4u] = mask;                     /* Set Mask Register. See Note(3a)                      */

                                                                /* Set RX Filtering for FIFO Mailbox(es)                */
#if (CAN_MAILBOX_MODE == FIFO_MAILBOX_MODE)
             if (can_id >= RX600_CAN_RX_FIFO) {
                 p_reg->FIDCR0  = can_id;                       /* Set Rx FIFO Compare Register 0                       */
                 p_reg->FIDCR1  = can_id;                       /* Set Rx FIFO Compare Register 1                       */
                 p_reg->MKR[6u] = mask;                         /* Set Mask Register. See Note(3b)                      */
                 p_reg->MKR[7u] = mask;                         /* Set Mask Register. See Note(3b)                      */
             }
#endif
                                                                /* Back to Operating Mode                               */
             drv_err = RX600_CAN_SetMode(para_id, RX600_CAN_MODE_OPERATE);
             break;


        case IO_RX600_CAN_IO_FUNC_N:                            /* --------------- GET FUNCTION CODE QTY -------------- */
                                                                /* Set the Size of IO Function Number for return.       */
            *((CPU_INT08U *)p_arg) = IO_RX600_CAN_IO_FUNC_N + 1u;

             result = RX600_CAN_ERR_NONE;                       /* Indicate Successful Function Execution               */
             break;
             
             
        default:                                                /* -------------- UNKNOWN FUNCTION CODE --------------- */
             break;
    }
    
    if (drv_err == DEF_FAIL) {
        RX600_DrvErr = RX600_CAN_ERR_FUNC;
        result       = RX600_CAN_ERR_FUNC;                      /* Error occurred in function, Return with Error.       */
    } else {
        result       = RX600_CAN_ERR_NONE;                      /* Indicate Successful Function Execution               */  
    }  

    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                           RX600_CAN_Read()
*
* Description : Read a received CAN Frame from a Message Buffer. The Buffer must have space for only
*               one CAN Frame.
*
* Argument(s) : para_id     Parameter Identifier, returned by RX600_CAN_Open().
*
*               buf         Pointer to CAN Frame.
*
*               size        Length of CAN Frame Memory.
*
* Return(s)   : Error code:  0 = No Error
*                           -1 = Error Occurred
*
* Caller(s)   : CanCfg in can_cfg.c
*
* Note(s)     : (1) Mailbox Search Mode Register is set to Recieve Mailbox Search Mode, and will return
*                   the Highest Priority Mailbox number (Smallest Mailbox number open) that is found in the
*                   search results. This will only search Mailboxes configured for Reception. The Found
*                   Mailbox will thus be used to Read the CAN Frame.
*********************************************************************************************************
*/

CPU_INT16S  RX600_CAN_Read (CPU_INT16S   para_id,
                            CPU_INT08U  *buf,
                            CPU_INT16U   size)
{
    RX600_CAN_REG  *p_reg;
    RX600_CAN_FRM  *p_frm;
    CPU_INT32U      frm_id;
    CPU_INT08U      i;
    CPU_INT08U      free_rx_mbox;
    CPU_INT16S      result;
    CPU_SR_ALLOC();
    
    
    result = -1;                                                /* Initializing Variable(s)                             */
                                                        
    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= RX600_CAN_N_DEV)) {
        RX600_DrvErr = RX600_CAN_ERR_BUS;
        return (result);
    }
    
    if (size != sizeof(RX600_CAN_FRM)) {                        /* Check if Size is Plausible                           */
        RX600_DrvErr = RX600_CAN_ERR_NO_DATA;
        return (result);
    }
    
    if (buf == (void *)0) {                                     /* Check if Buffer Pointer is Invalid                   */
        RX600_DrvErr = RX600_CAN_ERR_ARG;
        return (result);
    }
    
    if (RX600_DevData[para_id].Use != DEF_YES) {                /* Check if CAN Device is Opened                        */
        RX600_DrvErr = RX600_CAN_ERR_OPEN;
        return (result);
    }
    
    p_reg = RX600_DevData[para_id].RegPtr;                      /* Set Base Address for CAN Device(s)                   */
    p_frm = (RX600_CAN_FRM *)buf;
    
    CPU_CRITICAL_ENTER();
                                                                /* Message in Rx FIFO                                   */
    if (DEF_BIT_IS_SET(p_reg->STR, RX600_CAN_STR_RFST) == DEF_YES) {

        frm_id = p_reg->MB[RX600_CAN_RX_FIFO].ID;
                                                                /* Extended ID                                          */
        if (DEF_BIT_IS_SET(frm_id, RX600_CAN_MB_ID_IDE) == DEF_YES) {
            p_frm->Identifier = frm_id & RX600_CAN_EID_LIMIT;   /* Copy Both Extended and Standard IDs.                 */
            DEF_BIT_SET(p_frm->Identifier, RX600_CAN_FRM_IDE);
        } else {                                                /* Standard ID                                          */
            p_frm->Identifier = ((frm_id & RX600_CAN_SID_RX_MASK) >> RX600_CAN_SID_SHIFT);
            DEF_BIT_CLR(p_frm->Identifier, RX600_CAN_FRM_IDE);
        }
                                                                /* Remote Frame                                         */
        if (DEF_BIT_IS_SET(frm_id, RX600_CAN_MB_ID_RTR) == DEF_YES) {
            DEF_BIT_SET(p_frm->Identifier, RX600_CAN_FRM_RTR);
        }
                                                                /* Set DLC & Limit DLC to 8 bytes                       */
        p_frm->DLC = (p_reg->MB[RX600_CAN_RX_FIFO].DLC & RX600_CAN_DLC_LIMIT);

        for (i = 0u; i < p_frm->DLC; i++) {                     /* Copy Mailbox Data into Frame                         */
            p_frm->Data[i] = p_reg->MB[RX600_CAN_RX_FIFO].DATA[i];
        }

        p_reg->RFPCR = 0xFFu;                                   /* Increment ptr to next Mailbox location               */

        result = size;
                                                                /* NEWDATA Status Flag is checked (RX)                  */
    } else if (DEF_BIT_IS_SET(p_reg->STR, RX600_CAN_STR_NDST) == DEF_YES) {
                                                                /* Set Recieve Mailbox Search Mode. See Note(1)         */
        free_rx_mbox = RX600_CAN_MailSearch(para_id, RX600_CAN_RX_SEARCH_MODE);
    
        if (free_rx_mbox == RX600_CAN_ERR_FREE_MBOX) {          /* Check for Timeout Error in MSSR Register             */
            RX600_DrvErr = RX600_CAN_ERR_MAILBOX;               /* Mailbox could not be found, Return Error             */
            result = -1;
            return (result);
        }
        frm_id = p_reg->MB[free_rx_mbox].ID;                    /* Set Free RX MBOX as the Reception MBOX ID            */
        
                                                                /* ID Extension: Extended ID                            */
        if (DEF_BIT_IS_SET(frm_id, RX600_CAN_MB_ID_IDE) == DEF_YES) {
            p_frm->Identifier = frm_id & RX600_CAN_EID_LIMIT;   /* Copy Both Extended and Standard IDs.                 */
            DEF_BIT_SET(p_frm->Identifier, RX600_CAN_FRM_IDE);
        } else {                                                /* ID Extension: Standard ID                            */
            p_frm->Identifier = ((frm_id & RX600_CAN_SID_RX_MASK) >> RX600_CAN_SID_SHIFT);
            DEF_BIT_CLR(p_frm->Identifier, RX600_CAN_FRM_IDE);
        }
                                                                /* Remote Frame                                         */
        if (DEF_BIT_IS_SET(frm_id, RX600_CAN_MB_ID_RTR) == DEF_YES) {
            DEF_BIT_SET(p_frm->Identifier, RX600_CAN_FRM_RTR);
        }
                                                                /* Set DLC & Limit DLC to 8 bytes                       */
        p_frm->DLC = (p_reg->MB[free_rx_mbox].DLC & RX600_CAN_DLC_LIMIT);
        
        for (i = 0u; i < p_frm->DLC; i++) {                     /* Copy Mailbox Data into Frame                         */
            p_frm->Data[i] = p_reg->MB[free_rx_mbox].DATA[i];
        }
                                                                /* CLEAR NEWDATA Bit, Mailbox has been Stored           */
        DEF_BIT_CLR(p_reg->MCTL[free_rx_mbox], RX600_CAN_MCTL_NEWDATA);

        result = size;

    } else {                                                    /* No Message in RX for Normal or FIFO MBOXs            */
        RX600_DrvErr = RX600_CAN_ERR_NO_DATA;
    }
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                           RX600_CAN_Write()
*
* Description : Write a CAN Frame to a Message Buffer. The Buffer must contain only one CAN Frame
*               which will be written to a predefined Message Buffer.
*
* Argument(s) : para_id     Parameter Identifier, returned by RX600_CAN_Open().
*
*               buf         Pointer to CAN Frame.
*
*               size        Length of CAN Frame Memory.
*
* Return(s)   : Error code:  0 = No Error
*                           -1 = Error Occurred
*
* Caller(s)   : CanCfg in can_cfg.c
*
* Note(s)     : (1) If Transmitting a Frame to the CAN Bus for the first time, or previous Tx Mailbox
*                   is busy and cannot transmit the needed packet, then the SENTDATA bit in the STR 
*                   Register will not be enabled. If this bit isn't enabled, then configure the next 
*                   available Tx Mailbox found in the CANx Array. 
*                       No other Mailbox is configured unless it is needed.
*
*               (2) Once a Mailbox is Configured for Transmission, that Mailbox will transfer it's frame
*                   to the CAN Bus. This is why Tx Interrupts are enabled inside the write and why it is
*                   necessary to clean the Tx Mailbox before Configuring the Mailbox for Transmission, to
*                   prevent the Tx Mailbox from sending "Junk" on the CAN Bus.
*
*               (3) If the CAN Mailbox Array is an incomplete Array (Meaning that the Max amount of Mailboxes
*                   isn't configured) then all remaining entries will be set to 'CAN_NONE' and can be
*                   configured. The only case that this isn't true is when FIFO Configuration is used. If
*                   FIFO is used, there can still be 'CAN_NONE' entries, as long as they don't conflict
*                   with the CAN FIFO Mailbox Registers.
*
*               (4) Mailbox Search Mode Register is set to Transmit Mailbox Search Mode, and will return
*                   the Highest Priority Mailbox number (Smallest Mailbox number open) that is found in the
*                   search results. This will only search Mailboxes configured for Transmittion. The Found
*                   Mailbox will thus be used to Write to the CAN Bus.
*********************************************************************************************************
*/

CPU_INT16S  RX600_CAN_Write (CPU_INT16S   para_id,
                             CPU_INT08U  *buf,
                             CPU_INT16U   size)
{
    RX600_CAN_REG       *p_reg;
    RX600_CAN_FRM       *p_frm;
    RX600_CAN_MB_ARRAY  *p_mbox;
    CPU_INT32U           frm_id;
    CPU_INT08U           tx_mbox;
    CPU_BOOLEAN          nxt_mbox_found;
    CPU_INT08U           i;
    CPU_INT16S           result;
    CPU_SR_ALLOC();

    
    result = -1;                                                /* Initializing Variable(s)                             */
    i      =  0u;
    
    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= RX600_CAN_N_DEV)) {
        RX600_DrvErr = RX600_CAN_ERR_BUS;
        return (result);
    }
    
    if (size != sizeof(RX600_CAN_FRM)) {                        /* Check if Size is Plausible                           */
        RX600_DrvErr = RX600_CAN_ERR_NO_DATA;
        return (result);
    }
    
    if (buf == (void *)0) {                                     /* Check if Buffer Pointer is Invalid                   */
        RX600_DrvErr = RX600_CAN_ERR_ARG;
        return (result);
    }
    
    if (RX600_DevData[para_id].Use != DEF_YES) {                /* Check if CAN Device is Opened                        */
        RX600_DrvErr = RX600_CAN_ERR_OPEN;
        return (result);      
    }

    p_reg = RX600_DevData[para_id].RegPtr;                      /* Set Base Address for CAN Device(s)                   */
    p_frm = (RX600_CAN_FRM *)buf;
    
    CPU_CRITICAL_ENTER();
    
                                                                /* ---------------- CONFIGURE FRAME ID ---------------- */
    if (p_frm->Identifier > RX600_CAN_SID_LIMIT){               /* Extended ID                                          */
        frm_id = p_frm->Identifier & RX600_CAN_EID_LIMIT;
        DEF_BIT_SET(frm_id, RX600_CAN_MB_ID_IDE);
    } else {                                                    /* Standard ID                                          */
        frm_id = ((p_frm->Identifier & RX600_CAN_SID_LIMIT) << RX600_CAN_SID_SHIFT);
        DEF_BIT_CLR(frm_id, RX600_CAN_MB_ID_IDE);
    }
                                                                /* Remote Frame                                         */
    if (DEF_BIT_IS_SET(p_frm->Identifier, RX600_CAN_FRM_RTR) == DEF_YES) {
        DEF_BIT_SET(frm_id, RX600_CAN_MB_ID_RTR);
    }
                                                                /* ------------- SET TRANSPORTATION METHOD ------------ */
                                                                /* --------------- CONFIGURE NEW TX MBOX -------------- */
                                                                /* SENTDATA Flag is Cleared. See Note(1)                */
    if (DEF_BIT_IS_CLR(p_reg->STR, RX600_CAN_STR_SDST) == DEF_YES) {
        tx_mbox   = RX600_DevData[para_id].AvailTxMBox;         /* Set Tx Available Mailbox based on CAN Contrl         */
  
#if (RX600_CAN_CFG_MB_EN != DEF_ENABLED)
        p_mbox = &CANx_MB_ARRAY_CFG[0u];                        /* If No Advanced Mbox Cfg, use Default Configuration.  */
#else
        switch (para_id) {
            
    #if  (CAN_MODULE_CHANNEL_0 == DEF_ENABLED)
            case RX600_CAN_BUS_0:
                 p_mbox = &CAN0_MB_ARRAY_CFG[0u];
                 break;
    #endif
     
    #if  (CAN_MODULE_CHANNEL_1 == DEF_ENABLED)
            case RX600_CAN_BUS_1:
                 p_mbox = &CAN1_MB_ARRAY_CFG[0u];
                 break;
    #endif

     
    #if (CAN_MODULE_CHANNEL_2 == DEF_ENABLED)
            case RX600_CAN_BUS_2:
                 p_mbox = &CAN2_MB_ARRAY_CFG[0u];
                 break;
    #endif
        
            default:
                 RX600_DrvErr = RX600_CAN_ERR_BUS;              /* Return with Error, No Bus selected found.            */
                 CPU_CRITICAL_EXIT();
                 return (result);
        }
#endif                                                          /*  - CONFIGURE & TRANSMIT MSG -                        */
                                                                /* Disable TX Mailbox Interrupts                        */
        DEF_BIT_CLR(p_reg->MIER, RX600_CAN_MAILBOX_MIER_INT_EN(tx_mbox));
        
        p_reg->MCTL[tx_mbox]  = 0u;                             /* Clear Tx Mailbox to Prepare for New Data             */
        p_reg->MB[tx_mbox].ID = frm_id;                         /* Set ID                                               */
                                                                /* Set DLC & Limit DLC to 8 bytes                       */
        p_reg->MB[tx_mbox].DLC = (p_frm->DLC & RX600_CAN_DLC_LIMIT);
                                                                /* Copy Frame Data into Mailbox                         */
        for (i = 0u; i < p_reg->MB[tx_mbox].DLC; i++) {
            p_reg->MB[tx_mbox].DATA[i] = p_frm->Data[i];
        }
                                                                /* Enable TX Mailbox Interrupts                         */
        DEF_BIT_SET(p_reg->MIER, RX600_CAN_MAILBOX_MIER_INT_EN(tx_mbox));
        
                                                                /* Clear SENTDATA, Sending New Transmission             */
        DEF_BIT_CLR(p_reg->MCTL[tx_mbox], RX600_CAN_MCTL_SENTDATA);
                                                                /* Start Transfer to CAN Bus. See Note(2)               */
        DEF_BIT_SET(p_reg->MCTL[tx_mbox], RX600_CAN_MCTL_TRMREQ);
        
        result = size;
                                                                /*  - FIND NEXT NEEDED Tx MAILBOX -                     */
        nxt_mbox_found = DEF_NO;
        i              = (tx_mbox + 1u);                        /* Search CANx Array for NEXT Available Tx Mailbox.     */
        while ((nxt_mbox_found != DEF_YES) && (i < RX600_CAN_NBR_MBOX)) {
            if ((p_mbox[i].tx_rx == CAN_TX) || (p_mbox[i].tx_rx == CAN_NONE)) {
                
#if (CAN_MAILBOX_MODE == FIFO_MAILBOX_MODE)                     /* See Note (3).                                        */
                if (i < RX600_CAN_TX_FIFO) {
                    nxt_mbox_found = DEF_YES;
                }
#else
                nxt_mbox_found = DEF_YES;
#endif 
            } 
            
            i++;
        }
        
        if (nxt_mbox_found != DEF_YES) {                        /* No more Tx Mailboxes found, use only found ones.     */
            RX600_DevData[para_id].InitCmpl = DEF_TRUE;         /* Set Tx Init Complete flag, no new Tx Mbox's.         */
        } else {                                   
            RX600_DevData[para_id].AvailTxMBox = (i - 1u);      /* Set Next Available Mailbox, if Needed.               */
        }
                                                                /* ------------ TX THROUGH CONFIGURED MBOX ------------ */
                                                                /* Previous Mbox is Configured, SENTDATA is Set         */
    } else if (DEF_BIT_IS_SET(p_reg->STR, RX600_CAN_STR_SDST) == DEF_YES) {
                                                                /* Set Transmit Mailbox Search Mode. See Note(4)        */
        tx_mbox = RX600_CAN_MailSearch(para_id, RX600_CAN_TX_SEARCH_MODE);
        
        if (tx_mbox == RX600_CAN_ERR_FREE_MBOX) {               /* Check for Error Mailbox                              */
            RX600_DrvErr = RX600_CAN_ERR_MAILBOX;               /* Mailbox could not be found, Return Error             */
            CPU_CRITICAL_EXIT();
            return (result);
        }
                                                                /* Disable TX Mailbox Interrupts                        */
        DEF_BIT_CLR(p_reg->MIER, RX600_CAN_MAILBOX_MIER_INT_EN(tx_mbox));
        
        p_reg->MCTL[tx_mbox]  = 0u;                             /* Clear Tx Mailbox to Prepare for New Data             */
        p_reg->MB[tx_mbox].ID = frm_id;                         /* Set ID                                               */

                                                                /* Set DLC & Limit DLC to 8 bytes                       */
        p_reg->MB[tx_mbox].DLC = (p_frm->DLC & RX600_CAN_DLC_LIMIT);
                                                                /* Copy Frame Data into Mailbox                         */
        for (i = 0u; i < p_reg->MB[tx_mbox].DLC; i++) {
            p_reg->MB[tx_mbox].DATA[i] = p_frm->Data[i];
        }
                                                                /* Enable TX Mailbox Interrupts                         */
        DEF_BIT_SET(p_reg->MIER, RX600_CAN_MAILBOX_MIER_INT_EN(tx_mbox));
        
                                                                /* Clear SENTDATA, Sending New Transmission             */
        DEF_BIT_CLR(p_reg->MCTL[tx_mbox], RX600_CAN_MCTL_SENTDATA);
                                                                /* Start Transfer to CAN Bus. See Note(2)               */
        DEF_BIT_SET(p_reg->MCTL[tx_mbox], RX600_CAN_MCTL_TRMREQ);
        
        result = size;

#if (CAN_MAILBOX_MODE == FIFO_MAILBOX_MODE)                     /* ------------------- FIFO MODE TX ------------------- */
                                                                /* If FIFO is Enabled, and is NOT Full, Tx from FIFO.   */
    } else if (DEF_BIT_IS_SET(p_reg->STR, RX600_CAN_STR_TFST) == DEF_YES) {
        p_reg->MB[RX600_CAN_TX_FIFO].ID  = frm_id;              /* Set ID                                               */
                                                                /* Set DLC & Limit DLC to 8 bytes                       */
        p_reg->MB[RX600_CAN_TX_FIFO].DLC = (p_frm->DLC & RX600_CAN_DLC_LIMIT);
                                                                /* Copy Frame Data into Mailbox                         */
        for (i = 0u; i < p_reg->MB[RX600_CAN_TX_FIFO].DLC; i++) {
            p_reg->MB[RX600_CAN_TX_FIFO].DATA[i] = p_frm->Data[i];
        }
    
        p_reg->TFPCR = 0xFFu;                                   /* Increment ptr to Next Mailbox Location               */
        result       = size;
#endif

    } else {                                                    /* ----------------- TX MAILBOXES BUSY ---------------- */
        RX600_DrvErr = RX600_CAN_ERR_BUSY;                      /* All Tx Mailboxes are Busy                            */
    }
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                         RX600_CAN_SetMode()
*
* Description : Sets the CAN Mode.
*
* Argument(s) : para_id     Desired CAN Module ID.
*
*               mode        Desired Mode to set CAN.
*
* Return(s)   : none.
*
* Caller(s)   : RX600_CAN_Init()
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_BOOLEAN  RX600_CAN_SetMode (CPU_INT32U      para_id,
                                RX600_CAN_MODE  mode)
{
    RX600_CAN_REG  *p_reg;
    CPU_BOOLEAN     err;
    CPU_INT32U      i;

    
    p_reg = RX600_DevData[para_id].RegPtr;                      /* Set Base Address for CAN Device(s)                   */
    i     = 0u;                                                 /* Initialize Variable(s)                               */
    
    switch (mode) {
        case RX600_CAN_MODE_EXIT_SLEEP:                         /* ------------------- SLEEP MODE --------------------- */
                                                                /* Exit Sleep Mode                                      */
             DEF_BIT_CLR(p_reg->CTLR, (CPU_INT16U)RX600_CAN_CTRL_SLPM);
                                                                /* Wait until Sleep Mode is Exited                      */
             while (DEF_BIT_IS_SET(p_reg->STR, RX600_CAN_STR_SLPST) == DEF_YES) {
                 i++;                                           /* Wait till Bit is Set                                 */
                
                 if (i > CAN_TIMEOUT_ERR_VAL) {                 /* Checks for Timeout Error                             */
                     return (DEF_FAIL);
                 }
             }
             break;

             
        case RX600_CAN_MODE_RESET:                              /* ------------------- RESET MODE --------------------- */
             DEF_BIT_CLR(p_reg->CTLR, RX600_CAN_CTRL_CANM);     /* Reset STR Register of CAN Status Flag(s)             */
             p_reg->CTLR = RX600_CAN_CTRL_CANM_RESET;           /* Enter Reset Mode                                     */
                                                                /* Wait until Reset Mode is Entered                     */
             while (DEF_BIT_IS_CLR(p_reg->STR, RX600_CAN_STR_RSTST) == DEF_YES) {
                 i++;                                            /* Wait till Bit is Set                                 */
                
                 if (i > CAN_TIMEOUT_ERR_VAL) {                  /* Checks for Timeout Error                             */
                     return (DEF_FAIL);
                 }
             }
             break;

             
        case RX600_CAN_MODE_HALT:                               /* ------------------- HALT MODE ---------------------- */
             DEF_BIT_CLR(p_reg->CTLR, RX600_CAN_CTRL_CANM);     /* Reset STR Register of CAN Status Flag(s)             */
                                                                /* Enter Halt Mode                                      */
             DEF_BIT_SET(p_reg->CTLR, RX600_CAN_CTRL_CANM_HALT);
                                                                /* Wait until Halt Mode is Entered                      */
             while (DEF_BIT_IS_CLR(p_reg->STR, RX600_CAN_STR_HLTST) == DEF_YES){
                 i++;                                            /* Wait till Bit is Set                                 */
                
                 if (i > CAN_TIMEOUT_ERR_VAL) {                  /* Checks for Timeout Error                             */
                     return (DEF_FAIL);
                 }
             }
             break;
            
             
        case RX600_CAN_MODE_OPERATE:                            /* ------------------ OPERATE MODE -------------------- */
             err = RX600_CAN_BSP_Start(para_id);                /* Starts the Desired CAN Module                        */
             if (err == DEF_FAIL) {
                 RX600_DrvErr = RX600_CAN_ERR_FUNC;             /* CAN_BSP_START Timed Out, Return Error                */
                 return (DEF_FAIL);
             }

             DEF_BIT_CLR(p_reg->CTLR, RX600_CAN_CTRL_CANM);     /* Enter Operating Mode                                 */
                                                                /* Wait until Operating Mode is Entered                 */
             while (DEF_BIT_IS_SET(p_reg->STR, RX600_CAN_STR_RSTST) == DEF_YES) {
                 i++;                                            /* Wait till Bit is Set                                 */
                
                 if (i > CAN_TIMEOUT_ERR_VAL) {                  /* Checks for Timeout Error                             */
                     return (DEF_FAIL);
                 }
             }
             break;

             
        default:                                                /* --------------- UNKNOWN DEFAULT MODE --------------- */
             break;
    }
    
    return (DEF_OK);
}


/*
*********************************************************************************************************
*                                        RX600_CAN_MailSearch()
*
* Description : Uses Mailbox Mode Register and Mailbox Search Status Register to return results of 
*               Highest Priority Mailbox to place info into.
*
* Argument(s) : para_id     Desired CAN Module ID.
*
*               mail_mode   Search Mode selection in Mailbox Mode Register.
*
* Return(s)   : Highest Priority Mailbox for Search Mode selection.
*
* Caller(s)   : RX600_CAN_Read()
*               RX600_CAN_Write()
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT08U  RX600_CAN_MailSearch (CPU_INT32U             para_id,
                                  RX600_CAN_MAIL_SELECT  mail_mode)
{
    RX600_CAN_REG  *p_reg;
    CPU_INT08U      result;
    CPU_INT32U      i;

    
    i      = 0u;                                                /* Initialize Variable(s)                               */
    result = 0u;
    
    p_reg = RX600_DevData[para_id].RegPtr;                      /* Set Base Address for CAN Device(s)                   */
    
    p_reg->MSMR = mail_mode;                                    /* Set Mailbox Search Mode                              */
    while (DEF_BIT_IS_SET(p_reg->MSSR, RX600_CAN_MSSR_SEST) == DEF_YES) {
        i++;                                                    /* Wait till Search Result is found                     */
        
        if (i > CAN_TIMEOUT_ERR_VAL) {                          /* Checks for Timeout Error                             */
            result = RX600_CAN_ERR_FREE_MBOX;                   /* Set Result to return an Error Mailbox                */
            return (result);
        }
    }
    
    result = (p_reg->MSSR & 0x1F);                              /* Limit Search Result: Mailbox Number Found            */
    
    return (result);
}


/*
*********************************************************************************************************
*                                        RX600_CAN_ErrCheck()
*
* Description : Corrects Errors found in the Status & Error Interrupt Factor Judge Register(s).
*
* Argument(s) : para_id     Desired CAN Module ID.
*
* Return(s)   : none.
*
* Caller(s)   : RX600_CAN_NSHandler(), RX600_CAN_IoCtl()
*
* Note(s)     : (1) The RX600_CAN_Open() Function requires the BUS Device ID and BUS Device Node Name
*                   which are the same number for all 3 channels since each BUS ID and BUS NODE (found 
*                   in can_cfg.c) for each channel have to be different. The Mode in which to place the
*                   re-initialized Channel is for Exclusive Read/Write Access to the Can Bus.
*********************************************************************************************************
*/

void  RX600_CAN_ErrCheck (CPU_INT32U  para_id)
{
    RX600_CAN_REG  *p_reg;
    CPU_INT16S      dev_id;
    CPU_INT32U      baud_rate;
    CPU_INT08U      i;
    CPU_INT08U      mbox_result;
    
    
    if (para_id < RX600_CAN_N_DEV) {                            /* Check if Parameter ID is out of Range, w/o return    */
        i           = 0u;                                       /* Initialize Variable(s)                               */
        mbox_result = 0u;
        p_reg       = RX600_DevData[para_id].RegPtr;            /* Set Base Address for CAN Device(s)                   */

                                                                /* Check if there is an Error in the EIFR Reg.          */
        if (DEF_BIT_IS_SET(p_reg->STR, RX600_CAN_STR_EST) == DEF_YES) {
            baud_rate = CAN_DEFAULT_BAUDRATE;                   /* Set Restart Baudrate to Default Baudrate             */

                                                                /* -------------- BUS-OFF ENTRY DETECTED -------------- */
            if (DEF_BIT_IS_SET(p_reg->EIFR, RX600_CAN_EIFR_BOEIF) == DEF_YES) {
                                                                /* ------------ BUS-OFF RECOVERY DETECTED ------------- */
                if (DEF_BIT_IS_SET(p_reg->EIFR, RX600_CAN_EIFR_BORIF) == DEF_YES) {
                                                                /* Stop CAN Module, for Re-Initialization               */
                    RX600_CAN_IoCtl(para_id, IO_RX600_CAN_STOP, 0u);
                    RX600_CAN_Init(para_id);                    /* Re-Initialize CAN Module                             */
                                                                /* Re-Open CAN Module. See Note(1)                      */
                    dev_id = RX600_CAN_Open(para_id, para_id, DEV_RW);
                                                                /* Configure Controller with Baudrate                   */
                    RX600_CAN_IoCtl(dev_id, IO_RX600_CAN_SET_BAUDRATE, (void *) &baud_rate);
                                                                /* Re-Enable CAN Module                                 */
                    RX600_CAN_IoCtl(dev_id, IO_RX600_CAN_START, 0u);
                }
                                                                /* -------------- ERROR-PASSIVE DETECTED -------------- */
            } else if (DEF_BIT_IS_SET(p_reg->EIFR, RX600_CAN_EIFR_EPIF) == DEF_YES) {
                                                                /* Stop CAN Module, for Re-Initialization               */
                RX600_CAN_IoCtl(para_id, IO_RX600_CAN_STOP, 0u);
                RX600_CAN_Init(para_id);                        /* Re-Initialize CAN Module                             */
                                                                /* Re-Open CAN Module. See Note(1)                      */
                dev_id = RX600_CAN_Open(para_id, para_id, DEV_RW);
                                                                /* Configure Controller with Baudrate                   */
                RX600_CAN_IoCtl(dev_id, IO_RX600_CAN_SET_BAUDRATE, (void *) &baud_rate);
                                                                /* Re-Enable CAN Module                                 */
                RX600_CAN_IoCtl(dev_id, IO_RX600_CAN_START, 0u);

                                                                /* ---------------- OVERRUN DETECTED ------------------ */
            } else if (DEF_BIT_IS_SET(p_reg->EIFR, RX600_CAN_EIFR_ORIF) == DEF_YES) {
                                                                /* Search for Mbox that recorded the Lost Msg           */
                mbox_result = RX600_CAN_MailSearch(para_id, RX600_CAN_MSG_SEARCH_MODE);

                if (mbox_result != RX600_CAN_ERR_FREE_MBOX) {   /* Clear Mailbox's Message Lost Error Flag..            */
                    DEF_BIT_CLR( p_reg->MCTL[mbox_result],      /* .. and New Data Error Flag                           */
                                (RX600_CAN_MCTL_MSGLOST | RX600_CAN_MCTL_NEWDATA));

                    DEF_BIT_CLR(p_reg->EIFR,                    /* Clear Overrun Flag                                   */
                                RX600_CAN_EIFR_ORIF);

                } else {                                        /* Mailbox Not Found, Even Though Error Detected.       */
                                                                /* Stop CAN Module, for Re-Initialization               */
                    RX600_CAN_IoCtl(para_id, IO_RX600_CAN_STOP, 0u);
                    RX600_CAN_Init(para_id);                    /* Re-Initialize CAN Module                             */
                                                                /* Re-Open CAN Module. See Note(1)                      */
                    dev_id = RX600_CAN_Open(para_id, para_id, DEV_RW);
                                                                /* Configure Controller with Baudrate                   */
                    RX600_CAN_IoCtl(dev_id, IO_RX600_CAN_SET_BAUDRATE, (void *) &baud_rate);
                                                                /* Re-Enable CAN Module                                 */
                    RX600_CAN_IoCtl(dev_id, IO_RX600_CAN_START, 0u);
                }

                                                                /* ------- OVERLOAD FRAME TRANSMISSION DETECTED ------- */
            } else if (DEF_BIT_IS_SET(p_reg->EIFR, RX600_CAN_EIFR_OLIF) == DEF_YES) {
                                                                /* Search for Mbox that has Overloaded Frame            */
                mbox_result = RX600_CAN_MailSearch(para_id, RX600_CAN_TX_SEARCH_MODE);

                if (mbox_result != RX600_CAN_ERR_FREE_MBOX) {
                    p_reg->MCTL[mbox_result]   = 0u;            /* Clear Message Control Register(s)                    */
                    p_reg->MB[mbox_result].ID  = 0u;            /* Clear Overloaded Mailbox of all Data                 */
                    p_reg->MB[mbox_result].DLC = 0u;
                    for (i = 0u; i < 8u; i++) {
                        p_reg->MB[mbox_result].DATA[i] = 0u;
                    }
                    for (i = 0u; i < 2u; i++) {
                        p_reg->MB[mbox_result].TS = 0u;
                    }
                                                                /* Clear Overload Frame Transmission Flag               */
                    DEF_BIT_CLR(p_reg->EIFR, RX600_CAN_EIFR_OLIF);

                } else {                                        /* Mailbox Not Found, Even Though Error Detected.       */
                                                                /* Stop CAN Module, for Re-Initialization               */
                    RX600_CAN_IoCtl(para_id, IO_RX600_CAN_STOP, 0u);
                    RX600_CAN_Init(para_id);                    /* Re-Initialize CAN Module                             */
                                                                /* Re-Open CAN Module. See Note(1)                      */
                    dev_id = RX600_CAN_Open(para_id, para_id, DEV_RW);
                                                                /* Configure Controller with Baudrate                   */
                    RX600_CAN_IoCtl(dev_id, IO_RX600_CAN_SET_BAUDRATE, (void *) &baud_rate);
                                                                /* Re-Enable CAN Module                                 */
                    RX600_CAN_IoCtl(dev_id, IO_RX600_CAN_START, 0u);
                }
                                                                /* --------------- BUS ERROR DETECTED ----------------- */
            } else if (DEF_BIT_IS_SET(p_reg->EIFR, RX600_CAN_EIFR_BEIF) == DEF_YES) {
                DEF_BIT_CLR(p_reg->EIFR, RX600_CAN_EIFR_BEIF);  /* Clear BUS Error Detect Flag                          */

                                                                /* ---------------- BUS LOCK DETECTED ----------------- */
            } else if (DEF_BIT_IS_SET(p_reg->EIFR, RX600_CAN_EIFR_BLIF) == DEF_YES) {
                DEF_BIT_CLR(p_reg->EIFR, RX600_CAN_EIFR_BLIF);  /* Clear BUS Lock Detect Flag                           */

                                                                /* -------------- ERROR-WARNING DETECTED -------------- */
            } else if (DEF_BIT_IS_SET(p_reg->EIFR, RX600_CAN_EIFR_EWIF) == DEF_YES) {
                DEF_BIT_CLR(p_reg->EIFR, RX600_CAN_EIFR_EWIF);  /* Clear Error-Warning Detect Flag                      */
            } else {
                ;                                               /* ----------------- UNKNOWN ERROR -------------------- */
            }
        }
    }
}
