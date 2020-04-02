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
*                                      CAN DRIVER REGISTER CODE
*
*                                            Renesas RX200
*
* Filename : drv_can.c
* Version  : V2.42.01
*********************************************************************************************************
* Note(s)  : (1) This CAN Driver supports the following Series/Families:
*                    RX200 Driver - Renesas RX200    Family.
*                                 - Renesas RL78-F13 Series.
*                                 - Renesas RL78-F14 Series.
*
*                Set by the Technical Reference Manual(s) obtained from the Renesas website. This driver
*                has been tested with or should work with the families mentioned above.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              INCLUDES
*********************************************************************************************************
*/

#include  <drv_can.h>
#include  "drv_def.h"
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

static  const  CPU_INT32U       RX200_DrvId = 0x200F78F1u;      /* Unique Driver Identification Code                    */

static         RX200_CAN_ERR    RX200_DrvErr;                   /* Holds Detailed Error Code if Detected                */

static         RX200_CAN_DATA   RX200_DevData[RX200_CAN_N_DEV]; /* Array Holds Driver Runtime Data                      */

                                                                /* ----------------- CAN MAILBOX ARRAY ---------------- */
                                                                /* Array Holds Initial CAN Rx Mailbox IDs.              */
static         RX200_CAN_RX_ID  RX200_CAN_Rx_MB_ID[RX200_CAN_RX_MBOX_MAX] = CAN_MB_ARRAY;


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                              FUNCTIONS
*********************************************************************************************************
*/

static  CPU_BOOLEAN  RX200_CAN_SetMode   (CPU_INT32U             para_id,
                                          RX200_CAN_MODE         can_mode);

static  CPU_INT08U   RX200_CAN_DrvGetMode(CPU_INT32U             para_id,
                                          CPU_BOOLEAN            global_en);

static  CPU_BOOLEAN  RX200_CAN_DrvSetMode(CPU_INT32U             para_id,
                                          RX200_CAN_DRV_GL_MODE  gl_mode,
                                          RX200_CAN_DRV_CH_MODE  ch_mode);


/*
*********************************************************************************************************
*                                           RX200_CAN_Init()
*
* Description : Initializes the CAN Driver with the given Device Name
*
* Argument(s) : para_id     Device ID. [RX200_CAN_BUS_0]
*
* Return(s)   : Error code:  0 = No Error
*                           -1 = Error Occurred
*
* Caller(s)   : CanBusEnable()
*
* Note(s)     : (1) The RX200 CAN Driver has two 'Window' options. The RAM Window Select Bit selects
*                   between the two following windows and the Options they provide.
*
*                       WINDOW 0 : Receive Rule Entry Registers & RAM Test Registers.
*                       WINDOW 1 : Rx Buffers, Rx FIFO Buffers, Tx/Rx FIFO Buffers,
*                                  Tx Buffers, and Tx History Data Access Registers.
*
*                   The RX200 CAN Driver will configure the Receive Rule Entry Registers with the 
*                   user-desired Mailbox Map (done using WINDOW 0). Once the Receive Rule Registers
*                   have been configured it will return to WINDOW 1 and continue Configuration. RAM
*                   Test Registers are NOT supported in this Driver.
*
*               (2) By Clearing the Receive Rule Entry [i] Configuration Registers (GAFLPLx & GAFLPHx)
*                   it prevents the Receive Rule ID & Mask to save the Message to a rogue/unused Rx
*                   Buffer, as well as does not do a DLC Check of the Received Message. The Receive Rule
*                   will be saved only to Rx FIFO Buffer 0 & 1. The first 8 MB ID's will be saved to the
*                   Rx FIFO Buffer 0 and the last 8 to the Rx FIFO Buffer 1.
*                       Prevents Rx Saved To:                       Saves Rx'd Message To:
*                           - Tx/Rx FIFO Buffer 0                       - First 8 ID's : Rx FIFO 0
-                           - Rx Buffer [0 -> 15]                       - Last  8 ID's : Rx FIFO 1
*
*               (3) Based on the RX200 CAN Technical Reference Manual, the CAN Receive Buffers, although
*                   lengthy (16 Buffers Max), do NOT tie to any Interrupt Source. The Rx Buffers would
*                   need to be polled to wait for a new Rx Frame to arrive. Based on this, no Rx Buffers
*                   will be used. Rx FIFO Buffers will be used instead, and the Tx Buffers will be used
*                   for Transmission (since they do have an Int Source).
*
*               (4) Enable the Rx FIFO Buffer Interrupt while the Rx FIFO Buffer is not in use (I.E. when
*                   the RFE in the RFCCn Register is set to 0). However, the Rx FIFO Buffer RFE bit can
*                   only be set in Global Operating or Global Test Mode. Therefore, the Rx FIFO Buffers
*                   will be enabled when the Driver goes to Global Operating Mode in IoCtl() Function
*                   Code: IO_RX200_CAN_START.
*********************************************************************************************************
*/

CPU_INT16S  RX200_CAN_Init (CPU_INT32U  para_id)
{
    RX200_CAN_REG     *p_reg;
    RX200_CAN_RR_REG  *p_rr_reg;
    RX200_CAN_BAUD     baud;
    RX200_CAN_RX_ID   *p_mbox_id;
    CPU_INT08U         i;
    CPU_BOOLEAN        drv_err;
    CPU_INT16S         result;

    
    result = -1;                                                /* Initialize Variable(s)                               */
    
    switch (para_id) {
        
#if  (CAN_MODULE_CHANNEL_0 == DEF_ENABLED)
        case RX200_CAN_BUS_0:                                   /* ---------------- CAN ADDRESS SELECT ---------------- */
    #if (CAN_DRV_SELECT == CAN_DRV_RL78_F1x)                    /*      - CAN Address: RL78-F1x Driver Address.         */
             RX200_DevData[RX200_CAN_BUS_0].RegPtr    = RL78F_CAN_ADDR;
             RX200_DevData[RX200_CAN_BUS_0].RR_RegPtr = RL78F_CAN_ADDR + RX200_CAN_RR_ADDR_OFFSET;
    #else                                                       /*      - CAN Address: RX200    Driver Address.         */
             RX200_DevData[RX200_CAN_BUS_0].RegPtr    = RX200_CAN_ADDR;
             RX200_DevData[RX200_CAN_BUS_0].RR_RegPtr = RX200_CAN_ADDR + RX200_CAN_RR_ADDR_OFFSET;
    #endif
             break;
#endif
        
        default:
             RX200_DrvErr = RX200_CAN_ERR_INIT;                 /* CAN Device is out of Range, Return Error.            */
             return (result);
    }
    
                                                                /* ----------------- PIN CONFIGURATION ---------------- */
    RX200_DevData[para_id].Use = DEF_NO;                        /* Set Proper Can Device to UNUSED Status               */
                                                                /* Set Base Addr for CAN Device & Rx Rule Register Set. */
    p_reg        = (RX200_CAN_REG    *)RX200_DevData[para_id].RegPtr;
    p_rr_reg     = (RX200_CAN_RR_REG *)RX200_DevData[para_id].RR_RegPtr;
    p_mbox_id    = &RX200_CAN_Rx_MB_ID[0u];                     /* Use Ptr for Rx Mailbox ID Array.                     */
    RX200_DrvErr =  RX200_CAN_ERR_NONE;                         /* No Error so far, Reset Driver Error                  */
    
    drv_err = RX200_CAN_PinSetting(para_id);                    /* Configure Pin Settings for CAN Device                */
    if (drv_err == DEF_FAIL) {
        RX200_DrvErr = RX200_CAN_ERR_INIT;                      /* PinSettings Timeout, Return Error                    */
        return (result);
    }
                                                                /* --------------- DRIVER CONFIGURATION --------------- */
                                                                /* Enter Reset Mode, Start Configuring CAN Driver.      */
    drv_err = RX200_CAN_SetMode(para_id, RX200_CAN_MODE_GL_RESET);
    if (drv_err != DEF_OK) {
        RX200_DrvErr = RX200_CAN_ERR_INIT;                      /* CAN Reset Mode Timeout, Return Error.                */
        return (result);
    }
                                                                /* ------------- Window 0 Initial Config -------------  */
    DEF_BIT_CLR(p_reg->GRWCR, RX200_CAN_GRWCR_RPAGE);           /* Select Window 0 Option. See Note (1).                */
                                                                /* Set a Receive Rule for Each Rx Mailbox Allowed.      */
    p_reg->GAFLCFG = RX200_CAN_GAFLCFG_RNC0(RX200_CAN_RX_MBOX_MAX);
    
    for (i = 0u; i < RX200_CAN_RX_MBOX_MAX; i++) {              /* ------------- Receive Rule IDs & Config ------------ */
        if (*p_mbox_id > 0u) {                                  /* If the ID != 0, Configure & Compare ID.              */
                                                                /* If IDE Bit is Set or SID Limit has been Reached.     */
            if ((DEF_BIT_IS_SET(*p_mbox_id, RX200_CAN_FRM_IDE) == DEF_YES) ||
                (*p_mbox_id > RX200_CAN_SID_LIMIT)) {
                                                                /* Set Extended ID in Receive Rule [i] Registers.       */
                p_rr_reg->RX_RULE[i].GAFLIDLx = RX200_CAN_GAFLIxLn_GAFLIDx_EXT(*p_mbox_id);
                p_rr_reg->RX_RULE[i].GAFLIDHx = RX200_CAN_GAFLIxHn_GAFLIDx(*p_mbox_id);
                                                                /* IDE Select : Extended ID.                            */
                DEF_BIT_SET(p_rr_reg->RX_RULE[i].GAFLIDHx, RX200_CAN_GAFLIDHx_GAFLIDE);
                
            } else {                                            /* Set Standard ID in Receive Rule [i] Registers.       */
                p_rr_reg->RX_RULE[i].GAFLIDLx = RX200_CAN_GAFLIxLn_GAFLIDx_STD(*p_mbox_id);
                                                                /* IDE Select : Standard ID.                            */
                DEF_BIT_CLR(p_rr_reg->RX_RULE[i].GAFLIDHx, RX200_CAN_GAFLIDHx_GAFLIDE);
            }
            
            if (DEF_BIT_IS_SET(*p_mbox_id, RX200_CAN_FRM_RTR) == DEF_YES) {
                DEF_BIT_SET(p_rr_reg->RX_RULE[i].GAFLIDHx, RX200_CAN_GAFLIDHx_GAFLRTR);
                DEF_BIT_SET(p_rr_reg->RX_RULE[i].GAFLMHx,  RX200_CAN_GAFLMHx_GAFLRTRM);
            }
                                                                /* Mask Register(s) : Compare ALL ID Bit(s) & IDE Bit.  */
            p_rr_reg->RX_RULE[i].GAFLMLx =  RX200_CAN_REG_LIMIT;
            p_rr_reg->RX_RULE[i].GAFLMHx = (RX200_CAN_GAFLIxHn_GAFLIDx(RX200_CAN_EID_LIMIT_H) |
                                    RX200_CAN_GAFLMHx_GAFLIDEM);
        } else {
            p_rr_reg->RX_RULE[i].GAFLMLx = 0u;                  /* Mask Register(s) : Do NOT Compare any Bit(s).        */
            p_rr_reg->RX_RULE[i].GAFLMHx = 0u;
        }
                    
        p_rr_reg->RX_RULE[i].GAFLPLx = 0u;                      /* Clear Config Registers. See Note (2).                */
        p_rr_reg->RX_RULE[i].GAFLPHx = 0u;
        
        if (i < RX200_CAN_RX_MBOX_HALF) {                       /* Configure Receive Rule To : Rx FIFO Buffer 0.        */
            DEF_BIT_SET(p_rr_reg->RX_RULE[i].GAFLPLx, RX200_CAN_GAFLPLx_GAFLFDP0);
        } else {                                                /* Configure Receive Rule To : Rx FIFO Buffer 1.        */
            DEF_BIT_SET(p_rr_reg->RX_RULE[i].GAFLPLx, RX200_CAN_GAFLPLx_GAFLFDP1);
        }
        
        p_mbox_id++;                                            /* Increment Mailbox ID Ptr to next set MBox ID Value.  */
    }

                                                                /* ------------- Window 1 Initial Config -------------  */    
    DEF_BIT_SET(p_reg->GRWCR, RX200_CAN_GRWCR_RPAGE);           /* Return to Window 1. See Note (1).                    */

                                                                /* If RAM Test is Enabled, Disable it.                  */
    if (DEF_BIT_IS_SET(p_reg->GTSTCTRL, RX200_CAN_GTSTCTRL_RTME) == DEF_YES) {
        p_reg->GLOCKK = RX200_CAN_GLOCKK_KEY1;                  /* Unlock Global RAM Test Protection Register.          */
        p_reg->GLOCKK = RX200_CAN_GLOCKK_KEY2;
        DEF_BIT_CLR(p_reg->GTSTCTRL, RX200_CAN_GTSTCTRL_RTME);  /* Clear 'RAM Test Enable' Bit.                         */
        if (DEF_BIT_IS_SET(p_reg->GTSTCTRL, RX200_CAN_GTSTCTRL_RTME) == DEF_YES) {
            RX200_DrvErr = RX200_CAN_ERR_INIT;                  /* 'RAM Test Enable', Not Supported. Return Error.      */
            return(result);
        }
    }
    
    DEF_BIT_SET(p_reg->CTRH, (RX200_CAN_CTRH_BOM_ISO11898 |     /* Bus Off Recovery Mode : ISO11898-1 Compliant.        */
                              RX200_CAN_CTRH_ERRD));            /* Error Flags of All Errors are Indicated.             */

    DEF_BIT_SET(p_reg->GCFGL, (RX200_CAN_GCFGL_DCS          |   /* Set CAN Clock Source Select to CANCLK.               */
                               RX200_CAN_GCFGL_TSP_DIV_NONE |   /* Timestamp Clock Division is set to NONE.             */
                               RX200_CAN_GCFGL_TSSS));          /* Timestamp Clock Source Select is CANCLK.             */
    
    DEF_BIT_CLR(p_reg->GCFGL, RX200_CAN_GCFGL_TPRI);            /* Set Tx Priority : ID Priority.                       */
    
                                                                /* -------------- BAUDRATE CONFIGURATION -------------- */
    baud.BaudRate         = CAN_DEFAULT_BAUDRATE;               /* Set Default Baud Rate Settings                       */
    baud.SamplePoint      = CAN_DEFAULT_SP;
    baud.ReSynchJumpWidth = CAN_DEFAULT_RJW;
    
    drv_err = RX200_CAN_CalcTimingReg(&baud);                   /* Calculate Bit Timing Register Values                 */
    if (drv_err == DEF_FAIL) {
        RX200_DrvErr = RX200_CAN_ERR_INIT;                      /* Bit Timing Error, Return Error                       */
        return (result);
    }
    
    p_reg->CFGL =  RX200_CAN_CFGL_BRP(baud.PrescalerDiv);       /* Load Baud Rate Timing Values.                        */
    p_reg->CFGH = (RX200_CAN_CFGH_TSEG1(baud.PhaseBufSeg1) |
                   RX200_CAN_CFGH_TSEG2(baud.PhaseBufSeg2) |
                   RX200_CAN_CFGH_SJW(baud.SJW));
    
                                                                /* -------------- INTERRUPT CONFIGURATION ------------- */
#if ((CANBUS_RX_HANDLER_EN > 0u) || \
     (CANBUS_TX_HANDLER_EN > 0u) || \
     (CANBUS_NS_HANDLER_EN > 0u))
    RX200_CAN_IntSetting(para_id);                              /* Configure Interrupt Sources.                         */
#endif
                                                                /* ------------------- Rx Interrupts ------------------ */
    DEF_BIT_SET(p_reg->CTRL, (RX200_CAN_CTRL_EWIE  |            /* Enable  : Error Warning     Interrupt.               */
                              RX200_CAN_CTRL_EPIE  |            /* Enable  : Error Passive     Interrupt.               */
                              RX200_CAN_CTRL_BOEIE |            /* Enable  : Bus Off Entry     Interrupt.               */
                              RX200_CAN_CTRL_BORIE |            /* Enable  : Bus Off Recovery  Interrupt.               */
                              RX200_CAN_CTRL_OLIE  |            /* Enable  : Overload Frame Tx Interrupt.               */
                              RX200_CAN_CTRL_BLIE  |            /* Enable  : Bus Lock          Interrupt.               */
                              RX200_CAN_CTRL_ALIE));            /* Enable  : Arbitration Lost  Interrupt.               */
    
    DEF_BIT_SET(p_reg->GCTRL, RX200_CAN_GCTRL_MEIE);            /* Enable  : FIFO Message Lost Interrupt.               */
    
    DEF_BIT_SET(p_reg->RFCC0, RX200_CAN_xFCCn_xFIE);            /* Enable  : Rx FIFO - 0       Interrupt.               */
    DEF_BIT_SET(p_reg->RFCC1, RX200_CAN_xFCCn_xFIE);            /* Enable  : Rx FIFO - 1       Interrupt.               */
    
                                                                /* ------------------- Tx Interrupts ------------------ */
    DEF_BIT_CLR(p_reg->CFCCL0, RX200_CAN_xFCCn_xFIE);           /* Disable : Tx/Rx FIFO Rx     Interrupt.               */
    DEF_BIT_CLR(p_reg->CFCCL0, RX200_CAN_xFCCn_CFTXIE);         /* Disable : Tx/Rx FIFO Tx     Interrupt.               */
    
    DEF_BIT_SET(p_reg->TMIEC, (RX200_CAN_TMIEC_TMIE0 |          /* Enable  : Tx Buffer - 0     Interrupt.               */
                               RX200_CAN_TMIEC_TMIE1 |          /* Enable  : Tx Buffer - 1     Interrupt.               */
                               RX200_CAN_TMIEC_TMIE2 |          /* Enable  : Tx Buffer - 2     Interrupt.               */
                               RX200_CAN_TMIEC_TMIE3));         /* Enable  : Tx Buffer - 3     Interrupt.               */
    
    DEF_BIT_CLR(p_reg->THLCC0, RX200_CAN_THLCC0_THLIE);         /* Disable : Tx History Buffer Interrupt.               */

                                                                /* --------------- MAILBOX CONFIGURATION -------------- */
                                                                /* -------------------- Rx Mailbox -------------------- */
    p_reg->RMNB = 0u;                                           /* Do not use Receive Buffers. See Note (3).            */

    DEF_BIT_SET(p_reg->RFCC0, (RX200_CAN_xFCCn_xFDC_8_MSGs |    /* Rx FIFO Buffer Depth : 8 Messages. See Note (4).     */
                               RX200_CAN_xFCCn_xFIM));          /* Rx FIFO Interrupt Occurs EACH TIME a Msg is Rx'd.    */

    DEF_BIT_SET(p_reg->RFCC1, (RX200_CAN_xFCCn_xFDC_8_MSGs |    /* Rx FIFO Buffer Depth : 8 Messages. See Note (4).     */
                               RX200_CAN_xFCCn_xFIM));          /* Rx FIFO Interrupt Occurs EACH TIME a Msg is Rx'd.    */
    
                                                                /* -------------------- Tx Mailbox -------------------- */
    DEF_BIT_SET(p_reg->CFCCL0, RX200_CAN_xFCCn_xFDC_0_MSGs);    /* Tx/Rx FIFO Buffer Depth : 0 Msgs. Set to ensure Off. */
    DEF_BIT_CLR(p_reg->CFCCL0, RX200_CAN_xFCCn_xFE);            /* Disable : Tx/Rx FIFO Buffers.                        */
    
    DEF_BIT_CLR(p_reg->THLCC0, RX200_CAN_THLCC0_THLE);          /* Disable : Tx History Buffers.                        */
    
#if (CAN_TEST_MODE > 0u)                                        /* ------------------- CAN TEST MODE ------------------ */
    drv_err = RX200_CAN_SetMode(para_id, RX200_CAN_MODE_TEST);  /* Enter CAN Global Test Mode.                          */
    if (drv_err != DEF_OK) {
        RX200_DrvErr = RX200_CAN_ERR_INIT;                      /* CAN Global Test Mode Timeout, Return Error.          */
        return (result);
    }

    drv_err = RX200_CAN_SetMode(para_id, RX200_CAN_MODE_HALT);  /* Enter Channel Halt Mode. Configure Test Mode.        */
    if (drv_err != DEF_OK) {
        RX200_DrvErr = RX200_CAN_ERR_INIT;                      /* CAN Channel Halt Mode Timeout, Return Error.         */
        return (result);
    }
    
    DEF_BIT_SET(p_reg->CTRH, RX200_CAN_CTRH_CTME);              /* Enable Communication Test Mode.                      */
    
    if (CAN_TEST_MODE == CAN_TEST_INTERNAL) {                   /* Test Mode Selected : Internal Loopback Mode.         */
        DEF_BIT_SET(p_reg->CTRH, RX200_CAN_CTRH_CTMS_INT_LOOP);
    } else if (CAN_TEST_MODE == CAN_TEST_EXTERNAL) {            /* Test Mode Selected : External Loopback Mode.         */
        DEF_BIT_SET(p_reg->CTRH, RX200_CAN_CTRH_CTMS_EXT_LOOP);
    } else if (CAN_TEST_MODE == CAN_TEST_LISTEN_ONLY) {         /* Test Mode Selected : Listen-Only       Mode.         */
        DEF_BIT_SET(p_reg->CTRH, RX200_CAN_CTRH_CTMS_LISTEN_ONLY);
    } else {                                                    /* Test Mode Selected : Standard Test     Mode.         */
        DEF_BIT_SET(p_reg->CTRH, RX200_CAN_CTRH_CTMS_STANDARD);
    }
#endif
                                                                /* ----------------- START CAN MODULE ----------------- */
    result = RX200_CAN_ERR_NONE;                                /* Set Function Result: No Error                        */
    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                           RX200_CAN_Open()
*
* Description : Unlocks the CAN device, i.e. Read/Write/IoCtl Functions will take effect
*
* Argument(s) : dev_id      Bus Node Name, used to interface with the CAN Bus Layer.
*
*               dev_name    Driver Device Name, used to interface with the Lowlevel Device Driver.
*                           Possible Values for Device Name:
*                                   RX200_CAN_BUS_0     [CAN Bus 0]
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

CPU_INT16S  RX200_CAN_Open (CPU_INT16S  dev_id,
                            CPU_INT32U  dev_name,
                            CPU_INT16U  mode)
{
    CPU_INT16S  result;
    CPU_SR_ALLOC();

    
    result = -1;                                                /* Initializing Variable(s)                             */
    
    if (dev_name >= RX200_CAN_N_DEV) {                          /* Check if Device Name is out of Range                 */                        
        RX200_DrvErr = RX200_CAN_ERR_BUS;
        return (result);
    }   

    if (mode != DEV_RW) {                                       /* Check if Mode is not Supported                       */
        RX200_DrvErr = RX200_CAN_ERR_MODE;
        return (result);
    }

    CPU_CRITICAL_ENTER();

    if (RX200_DevData[dev_name].Use == DEF_NO) {                /* Check if CAN Device is Unused                        */
        RX200_DevData[dev_name].Use =  DEF_YES;                 /* Mark CAN Device as Used                              */

#if ((CANBUS_RX_HANDLER_EN > 0u) || \
     (CANBUS_TX_HANDLER_EN > 0u) || \
     (CANBUS_NS_HANDLER_EN > 0u))
        
        RX200_CAN_SetDevIds((CPU_INT08U)dev_id,                 /* Set Device IDs for the ISRs                          */
                            (CPU_INT08U)dev_name);
#else
        (void)&dev_id;                                          /* Prevent Compiler Warning                             */
#endif
        
        result = (CPU_INT16S)dev_name;                          /* OK, Device is Opened                                 */
    } else {
        RX200_DrvErr = RX200_CAN_ERR_OPEN;
    }
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                           RX200_CAN_Close()
*
* Description : Locks the CAN device, i.e. Read/Write/IoCtl Functions will not take effect
*
* Argument(s) : para_id     Parameter Identifier, returned by RX200_CAN_Open()
*
* Return(s)   : Error code:  0 = No Error
*                           -1 = Error Occurred
*
* Caller(s)   : CanCfg in can_cfg.c
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT16S  RX200_CAN_Close (CPU_INT16S  para_id)
{
    CPU_INT16S  result;
    CPU_SR_ALLOC();

    
    result = -1;                                                /* Initializing Variable(s)                             */

    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= RX200_CAN_N_DEV)) {
        RX200_DrvErr = RX200_CAN_ERR_BUS;
        return (result);
    }

    CPU_CRITICAL_ENTER();
    
    if (RX200_DevData[para_id].Use != DEF_NO) {                 /* Check if CAN Device is Used                          */
        RX200_DevData[para_id].Use  = DEF_NO;                   /* Mark CAN Device as Unused                            */
        result = RX200_CAN_ERR_NONE;                            /* OK, Device is Closed                                 */
    } else {
        RX200_DrvErr = RX200_CAN_ERR_CLOSE;
    }
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                           RX200_CAN_IoCtl()
*
* Description : Performs Special Action on the Opened Device. The Function Code 'func' defines 
*               what the caller wants to do. Description of Function Codes are defined in the header.
*
* Argument(s) : para_id     Parameter Identifier, returned by RX200_CAN_Open().
*
*               func        Function Code.
*
*               p_arg       Argument List, Specific to the Function Code.
*
* Return(s)   : Error code:  0 = No Error
*                           -1 = Error Occurred
*
* Caller(s)   : CanCfg in can_cfg.c
*               Application
*
* Note(s)     : (1) The Baud Rate Registers (CFGL & CFGH) can only be modified when the CAN
*                   Module is in Channel Reset Mode or Channel Halt Mode. If Channel Halt Mode is
*                   being configured, and it cannot Transition properly, then Channel Reset Mode
*                   will be transitioned to (Based on the Channel Transition Table).
*
*               (2) Based on the RSCAN-Lite Reference Manual, the Rx FIFO Buffer RFE Bits in the
*                   RFCCn Registers can only be Modified (Set to ENABLED) in Global Operating or
*                   in Global Test Mode(s). The IO_RX200_CAN_START Function code will check if
*                   the bits have been previously set (in previous Operating Mode Calls) if not, then
*                   it will Enable the bits after the Driver is configured to Global Operating Mode.
*
*               (3) If p_arg is set to '0' improperly (meaning that the p_arg is passed a '0'
*                   instead of assigning a local variable the value of '0' and passing the variable)
*                   then the p_arg will address '0' but the value in that address will be a junk
*                   value. Therefore, if mailbox > Max # of Rx Mailboxes [16] then it will set all
*                   Mailboxes with the Standard or Extended Flag, based on the Function Code.
*                       Below will be an example of an improper assignment of Rx Mailbox [0] which
*                       will result in all Mailboxes being set to Standard ID (in this example).
*                           Improper Rx Mailbox[0] Assignment:
*                               xxx_CAN_IoCtl(para_id, IO_RX200_CAN_RX_STANDARD, 0u);
*
*                           Proper   Rx Mailbox[0] Assignment:
*                               mbox_var = 0u;
*                               xxx_CAN_IoCtl(para_id, IO_RX200_CAN_RX_STANDARD, &mbox_var);
*
*               (4) The Same concept found in Note (3) can be used for the 'Set Rx Filter' Function Code.
*                   The following Rules are tied to this the IO_RX200_CAN_SET_RX_FILTER code.
*                     RULES:
*                       - If p_arg[0] is set to '0' improperly (so the 'Mailbox' argument), then the
*                         Driver will clear all Rx Mailboxes of their Rx IDs and Masks.
*
*                       - If p_arg[0] is valid, but p_arg[1] is set to '0' (properly) then the ID & Mask
*                         registers of that specified Mailbox will be cleared.
*
*                       - If p_arg[0 & 1] are valid, but p_arg[2] is set to '0' (properly) then ONLY the
*                         Mask Registers of that specified Mailbox will be cleared.
*
*                       - p_arg[3] MUST be passed as part of the argument, if the 'mask' argument is not
*                         passed properly, then the Driver will assign the 'junk' found in this memory
*                         location to the Mailbox's Mask Registers. A Mask value of 0xFFFFFFFFu will
*                         compare all bits of the Mailbox ID value.
*********************************************************************************************************
*/

CPU_INT16S  RX200_CAN_IoCtl (CPU_INT16S   para_id,
                             CPU_INT16U   func,
                             void        *p_arg)
{
    RX200_CAN_REG     *p_reg;
    RX200_CAN_RR_REG  *p_rr_reg;
    RX200_CAN_BAUD     baud;
    CPU_INT08U         gl_curr_mode;
    CPU_INT32U         mask;
    CPU_INT32U         mb_id;
    CPU_INT08U         mailbox;
    CPU_INT08U         i;
    CPU_BOOLEAN        drv_err;
    CPU_INT16S         result;
    CPU_SR_ALLOC();


     result  = -1;                                              /* Initialize Variable(s)                               */
     drv_err = DEF_OK;
    
    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= RX200_CAN_N_DEV)) {
        RX200_DrvErr = RX200_CAN_ERR_BUS;
        return (result);
    }

    if (RX200_DevData[para_id].Use != DEF_YES) {                /* Check if CAN Device is Opened                        */
        RX200_DrvErr = RX200_CAN_ERR_OPEN;
        return (result);
    }
                                                                /* Set Base Addr for CAN Device & Rx Rule Register Set. */
    p_reg    = (RX200_CAN_REG    *)RX200_DevData[para_id].RegPtr;
    p_rr_reg = (RX200_CAN_RR_REG *)RX200_DevData[para_id].RR_RegPtr;
    
    CPU_CRITICAL_ENTER();

    switch (func) {                                             /*                SELECT: FUNCTION CODE                 */
        case IO_RX200_CAN_GET_IDENT:                            /* -------------------- GET IDENT --------------------- */
             *(CPU_INT32U*)p_arg = RX200_DrvId;                 /* Return Driver Identification Code                    */
             break;
             
             
        case IO_RX200_CAN_GET_ERRNO:                            /* ------------------ GET ERRORCODE ------------------- */
             *(CPU_INT16U*)p_arg = RX200_DrvErr;                /* Return Last Detected Error Code                      */
             break;
              
             
        case IO_RX200_CAN_GET_DRVNAME:                          /* ----------------- GET DRIVER NAME ------------------ */
                                                                /* Return Human Readable Driver Name                    */
             *(CPU_INT08U**)p_arg = (CPU_INT08U *)RX200_CAN_NAME;
             break;
               
             
        case IO_RX200_CAN_SET_BAUDRATE:                         /* ------------------ SET BAUD RATE ------------------- */

             baud.BaudRate         = *((CPU_INT32U *)p_arg);
             baud.SamplePoint      = CAN_DEFAULT_SP;
             baud.ReSynchJumpWidth = CAN_DEFAULT_RJW;

             drv_err = RX200_CAN_CalcTimingReg(&baud);          /* Calculate Bit Timing Register Values                 */
             if (drv_err != DEF_OK) {                           /* Bit Timing Error, Return Error                       */
                 break;
             }

             gl_curr_mode = RX200_CAN_DrvGetMode(para_id,       /* Get Current Global Mode. See Note (1).               */
                                                 DEF_YES);
                                                                /* If in Global Operating or Test Mode, Enter Ch Halt.  */
             if ((gl_curr_mode == RX200_CAN_DRV_GL_MODE_OPERATING) || \
                 (gl_curr_mode == RX200_CAN_DRV_GL_MODE_TEST)) {

                 drv_err = RX200_CAN_SetMode(para_id, RX200_CAN_MODE_HALT);
                 if (drv_err != DEF_OK) {                       /* CAN Halt Mode Timeout, Return Error                  */
                     break;
                 }
                                                                /* If in STOP Mode, set to Global & Channel Reset Mode. */
             } else if (gl_curr_mode == RX200_CAN_DRV_GL_MODE_STOP) {
                 drv_err = RX200_CAN_SetMode(para_id, RX200_CAN_MODE_GL_RESET);
                 if (drv_err != DEF_OK) {                       /* CAN Halt Mode Timeout, Return Error                  */
                     break;
                 }
             }
                                                                /* Load Baud Rate Timing Values.                        */
             p_reg->CFGL =  RX200_CAN_CFGL_BRP(baud.PrescalerDiv);
             p_reg->CFGH = (RX200_CAN_CFGH_TSEG1(baud.PhaseBufSeg1) |
                            RX200_CAN_CFGH_TSEG2(baud.PhaseBufSeg2) |
                            RX200_CAN_CFGH_SJW(baud.SJW));
             
                                                                /* Return to Previous Global Mode.                      */
             if (gl_curr_mode == RX200_CAN_DRV_GL_MODE_OPERATING) {
                 drv_err = RX200_CAN_SetMode(para_id, RX200_CAN_MODE_OPERATE);
             } else if (gl_curr_mode == RX200_CAN_DRV_GL_MODE_TEST) {
                 drv_err = RX200_CAN_SetMode(para_id, RX200_CAN_MODE_TEST);
             } else if (gl_curr_mode == RX200_CAN_DRV_GL_MODE_STOP) {
                 drv_err = RX200_CAN_SetMode(para_id, RX200_CAN_MODE_STOP);
             } else {
                 ;
             }
             
             break;
            

        case IO_RX200_CAN_TX_READY:                             /* -------------------- TX READY ---------------------- */
                                                                /* At least 1 Tx Buffer has Not Tx Request. Ok to Send  */
             if (DEF_BIT_IS_CLR_ANY(p_reg->TMTRSTS, (RX200_CAN_TMTxSTS_TMTxSTS0 |
                                                     RX200_CAN_TMTxSTS_TMTxSTS1 |
                                                     RX200_CAN_TMTxSTS_TMTxSTS2 |
                                                     RX200_CAN_TMTxSTS_TMTxSTS3)) == DEF_YES) {
                 *((CPU_INT08U *)p_arg) = 1u;
             } else {                                           /* All Tx Buffers have a Tx Request Present. Don't Send */
                 *((CPU_INT08U *)p_arg) = 0u;
             }
             break;

             
        case IO_RX200_CAN_START:                                /* ------------- START CAN COMMUNICATION -------------- */
                                                                /* Enter Global Operating Mode & Channel Comm Mode.     */
             drv_err = RX200_CAN_SetMode(para_id, RX200_CAN_MODE_OPERATE);
             
                                                                /* Enable Rx FIFO Buffers. See Note (2).                */
             if (DEF_BIT_IS_CLR(p_reg->RFCC0, RX200_CAN_xFCCn_xFE) == DEF_YES) {
                 DEF_BIT_SET(p_reg->RFCC0, RX200_CAN_xFCCn_xFE);
             }
             
             if (DEF_BIT_IS_CLR(p_reg->RFCC1, RX200_CAN_xFCCn_xFE) == DEF_YES) {
                 DEF_BIT_SET(p_reg->RFCC1, RX200_CAN_xFCCn_xFE);
             }
             break;

             
        case IO_RX200_CAN_STOP:                                 /* ------------- STOP CAN COMMUNICATION --------------- */
                                                                /* Enter Global & Channel Halt Mode                     */
             drv_err = RX200_CAN_SetMode(para_id, RX200_CAN_MODE_STOP);
             break;

             
        case IO_RX200_CAN_GET_NODE_STATUS:                      /* ---------------- GET NODE STATUS ------------------- */
             *((CPU_INT08U *)p_arg) = 0u;                       /* Set the Initial Argument Pointer to Zero             */

             if (DEF_BIT_IS_SET(p_reg->STSL, RX200_CAN_STSL_EPSTS) == DEF_YES) {
                 *((CPU_INT08U *)p_arg) = 1u;                   /* CAN Module has reached Error Passive State           */
                 break;
             }
             
             if (DEF_BIT_IS_SET(p_reg->STSL, RX200_CAN_STSL_BOSTS) == DEF_YES) {
                 *((CPU_INT08U *)p_arg) = 2u;                   /* CAN Module has reached Bus-Off State                 */
             }
             break;
             
             
        case IO_RX200_CAN_RX_STANDARD:                          /* --------------- SET RX STANDARD -------------------- */
             mailbox = ((CPU_INT08U *)p_arg)[0u];               /* Mailbox Desired for SID Flag Configuration           */

             DEF_BIT_CLR(p_reg->GRWCR, RX200_CAN_GRWCR_RPAGE);  /* Set to Window 0, to Configure Receive Rule(s).       */
             
             if (mailbox >= RX200_CAN_RX_MBOX_MAX) {            /* If p_arg = 0 is set Improperly. See Note (3).        */
                 for (i = 0u; i < RX200_CAN_RX_MBOX_MAX; i++) {
                                                                /* Set SID Flag, and Set Mask Register to Compare SID.  */
                     DEF_BIT_CLR(p_rr_reg->RX_RULE[i].GAFLIDHx, RX200_CAN_GAFLIDHx_GAFLIDE);
                     DEF_BIT_SET(p_rr_reg->RX_RULE[i].GAFLMHx,  RX200_CAN_GAFLMHx_GAFLIDEM);
                 }
             } else {                                           /* Only Configure specified Rx Mailbox with SID Flag.   */
                                                                /* Set SID Flag, and Set Mask Register to Compare SID.  */
                 DEF_BIT_CLR(p_rr_reg->RX_RULE[mailbox].GAFLIDHx, RX200_CAN_GAFLIDHx_GAFLIDE);
                 DEF_BIT_SET(p_rr_reg->RX_RULE[mailbox].GAFLMHx,  RX200_CAN_GAFLMHx_GAFLIDEM);
             }
             
             DEF_BIT_SET(p_reg->GRWCR, RX200_CAN_GRWCR_RPAGE);  /* Set to Window 1, to Rx & Tx CAN Messages.            */
             break;
             
             
        case IO_RX200_CAN_RX_EXTENDED:                          /* --------------- SET RX EXTENDED -------------------- */
             mailbox = ((CPU_INT08U *)p_arg)[0u];               /* Mailbox Desired for EID Flag Configuration           */

             DEF_BIT_CLR(p_reg->GRWCR, RX200_CAN_GRWCR_RPAGE);  /* Set to Window 0, to Configure Receive Rule(s).       */
             
             if (mailbox >= RX200_CAN_RX_MBOX_MAX) {            /* If p_arg = 0 is set Improperly. See Note (3).        */
                 for (i = 0u; i < RX200_CAN_RX_MBOX_MAX; i++) {
                                                                /* Set EID Flag, and Set Mask Register to Compare EID.  */
                     DEF_BIT_SET(p_rr_reg->RX_RULE[i].GAFLIDHx, RX200_CAN_GAFLIDHx_GAFLIDE);
                     DEF_BIT_SET(p_rr_reg->RX_RULE[i].GAFLMHx,  RX200_CAN_GAFLMHx_GAFLIDEM);
                 }
             } else {                                           /* Only Configure specified Rx Mailbox with EID Flag.   */
                                                                /* Set EID Flag, and Set Mask Register to Compare EID.  */
                 DEF_BIT_SET(p_rr_reg->RX_RULE[mailbox].GAFLIDHx, RX200_CAN_GAFLIDHx_GAFLIDE);
                 DEF_BIT_SET(p_rr_reg->RX_RULE[mailbox].GAFLMHx,  RX200_CAN_GAFLMHx_GAFLIDEM);
             }
             
             DEF_BIT_SET(p_reg->GRWCR, RX200_CAN_GRWCR_RPAGE);  /* Set to Window 1, to Rx & Tx CAN Messages.            */
             break;
             
             
        case IO_RX200_CAN_SET_RX_FILTER:                        /* ---------------- SET RX FILTER --------------------- */
             mailbox = ((CPU_INT08U *)p_arg)[0u];               /* Mailbox Desired for Filter Configuration             */
             mb_id   = ((CPU_INT32U *)p_arg)[1u];               /* ID Value Desired for Mailbox                         */
             mask    = ((CPU_INT32U *)p_arg)[2u];               /* Mask Value for Mask Register                         */
             
             DEF_BIT_CLR(p_reg->GRWCR, RX200_CAN_GRWCR_RPAGE);  /* Set to Window 0, to Configure Receive Rule(s).       */
             
             if (mailbox > RX200_CAN_RX_MBOX_MAX) {             /* If p_arg[0u] = 0 is set Improperly. See Note (4).    */
                 for (i = 0u; i < RX200_CAN_RX_MBOX_MAX; i++) {
                     p_rr_reg->RX_RULE[i].GAFLIDLx = 0u;        /* Clear All Rx Mailbox ID & Mask Registers.            */
                     p_rr_reg->RX_RULE[i].GAFLIDHx = 0u;
                     p_rr_reg->RX_RULE[i].GAFLMLx  = 0u;
                     p_rr_reg->RX_RULE[i].GAFLMHx  = 0u;
                 }
                 
             } else {
                 if (mb_id > 0u) {                              /* Set Specified Mailbox with Desired Mailbox ID.       */
                     if ((DEF_BIT_IS_SET(mb_id, RX200_CAN_FRM_IDE) == DEF_YES) || 
                         (mb_id > RX200_CAN_SID_LIMIT)) {       /* If Extended ID Flag is set, Or ID exceeds SID Limit. */
                         p_rr_reg->RX_RULE[mailbox].GAFLIDLx = RX200_CAN_GAFLIxLn_GAFLIDx_EXT(mb_id);
                         p_rr_reg->RX_RULE[mailbox].GAFLIDHx = RX200_CAN_GAFLIxHn_GAFLIDx(mb_id);
                         
                         DEF_BIT_SET(p_rr_reg->RX_RULE[mailbox].GAFLIDHx, RX200_CAN_GAFLIDHx_GAFLIDE);
                         
                     } else {                                   /* Else, it's a Standard ID, clear Upper-Half ID Reg.   */
                         p_rr_reg->RX_RULE[mailbox].GAFLIDLx = RX200_CAN_GAFLIxLn_GAFLIDx_STD(mb_id);
                         DEF_BIT_CLR(p_rr_reg->RX_RULE[mailbox].GAFLIDHx, RX200_CAN_EID_LIMIT_H);
                         DEF_BIT_CLR(p_rr_reg->RX_RULE[mailbox].GAFLIDHx, RX200_CAN_GAFLIDHx_GAFLIDE);
                     }
                                                                /* Check if the Mailbox ID is a Remote Frame            */
                     if (DEF_BIT_IS_SET(mb_id, RX200_CAN_FRM_RTR) == DEF_YES) {
                         DEF_BIT_SET(p_rr_reg->RX_RULE[mailbox].GAFLIDHx, RX200_CAN_GAFLIDHx_GAFLRTR);
                     }
                     
                     if (mask > 0u) {                           /* Set Specified Mailbox with Desired Mask Values.      */
                         p_rr_reg->RX_RULE[mailbox].GAFLMLx = RX200_CAN_GAFLIxLn_GAFLIDx_EXT(mask);
                         p_rr_reg->RX_RULE[mailbox].GAFLMHx = (mask >> 16u);
                         
                     } else {                                   /* Clear Specified Mailbox's Mask Registers Only.       */
                         p_rr_reg->RX_RULE[mailbox].GAFLMLx = 0u;
                         p_rr_reg->RX_RULE[mailbox].GAFLMHx = 0u;
                     }
                     
                 } else {
                     p_rr_reg->RX_RULE[mailbox].GAFLIDLx = 0u;  /* Clear Specified Mailbox's ID & Mask Registers.       */
                     p_rr_reg->RX_RULE[mailbox].GAFLIDHx = 0u;
                     p_rr_reg->RX_RULE[mailbox].GAFLMLx  = 0u;
                     p_rr_reg->RX_RULE[mailbox].GAFLMHx  = 0u;
                 }
             }

             DEF_BIT_SET(p_reg->GRWCR, RX200_CAN_GRWCR_RPAGE);  /* Set to Window 1, to Rx & Tx CAN Messages.            */
             break;


        case IO_RX200_CAN_IO_FUNC_N:                            /* --------------- GET FUNCTION CODE QTY -------------- */
                                                                /* Set the Size of IO Function Number for return.       */
            *((CPU_INT08U *)p_arg) = IO_RX200_CAN_IO_FUNC_N + 1u;
             break;

             
        default:                                                /* -------------- UNKNOWN FUNCTION CODE --------------- */
             break;
    }
    
    if (drv_err != DEF_OK) {
        RX200_DrvErr = RX200_CAN_ERR_FUNC;
        result       = RX200_CAN_ERR_FUNC;                      /* Error occurred in function, Return with Error.       */
    } else {
        result       = RX200_CAN_ERR_NONE;                      /* Indicate Successful Function Execution               */  
    }  

    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                           RX200_CAN_Read()
*
* Description : Read a received CAN Frame from a Message Buffer. The Buffer must have space for only
*               one CAN Frame.
*
* Argument(s) : para_id     Parameter Identifier, returned by RX200_CAN_Open().
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
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT16S  RX200_CAN_Read (CPU_INT16S   para_id,
                            CPU_INT08U  *buf,
                            CPU_INT16U   size)
{
    RX200_CAN_REG  *p_reg;
    RX200_CAN_FRM  *p_frm;
    CPU_INT08U     *p_data;
    CPU_INT32U      frm_id;
    CPU_INT08U      i;
    CPU_INT16S      result;
    CPU_SR_ALLOC();
    
    
    result = -1;                                                /* Initializing Variable(s)                             */
                                                        
    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= RX200_CAN_N_DEV)) {
        RX200_DrvErr = RX200_CAN_ERR_BUS;
        return (result);
    }
    
    if (size != sizeof(RX200_CAN_FRM)) {                        /* Check if Size is Plausible                           */
        RX200_DrvErr = RX200_CAN_ERR_NO_DATA;
        return (result);
    }
    
    if (buf == (void *)0) {                                     /* Check if Buffer Pointer is Invalid                   */
        RX200_DrvErr = RX200_CAN_ERR_ARG;
        return (result);
    }
    
    if (RX200_DevData[para_id].Use != DEF_YES) {                /* Check if CAN Device is Opened                        */
        RX200_DrvErr = RX200_CAN_ERR_OPEN;
        return (result);
    }

    p_reg = (RX200_CAN_REG *)RX200_DevData[para_id].RegPtr;     /* Set Base Address for CAN Device Register Set.        */
    p_frm = (RX200_CAN_FRM *)buf;
    
    CPU_CRITICAL_ENTER();
                                                                /* ----------------- READ CAN MESSAGE ----------------- */
                                                                /* ----------------- Rx FIFO Channel 0 ---------------- */
    if (DEF_BIT_IS_CLR(p_reg->RFSTS0, RX200_CAN_xFSTSn_xFEMP) == DEF_YES) {
                                                                /*  - Extended ID -                                     */
        if (DEF_BIT_IS_SET(p_reg->RX_FIFO[0u].RFIDHx, RX200_CAN_RxIDHn_RMIDE) == DEF_YES) {
            frm_id  =  p_reg->RX_FIFO[0u].RFIDLx;               /* Read Bottom-Half of Rx ID, then Top-Half of Rx ID.   */
            frm_id |= (RX200_CAN_RxIDHn_RMID(p_reg->RX_FIFO[0u].RFIDHx) << 16u);
                
            DEF_BIT_SET(frm_id, RX200_CAN_FRM_IDE);             /* Set   CAN Frame IDE Bit.                             */
            
        } else {                                                /*  - Standard ID -                                     */
            frm_id = RX200_CAN_RxIDLn_RMID_STD(p_reg->RX_FIFO[0u].RFIDLx);
            DEF_BIT_CLR(frm_id, RX200_CAN_FRM_IDE);             /* Clear CAN Frame IDE Bit.                             */
        }
                                                                /*  - Remote Frame Check -                              */
        if (DEF_BIT_IS_SET(p_reg->RX_FIFO[0u].RFIDHx, RX200_CAN_RxIDHn_RMRTR) == DEF_YES) {
            DEF_BIT_SET(frm_id, RX200_CAN_FRM_RTR);             /* Set CAN Frame RTR Bit, if Rx'd Msg is an RTR Msg.    */
        }
        
        p_frm->Identifier = frm_id;                             /* Save CAN Frame ID.                                   */
                                                                /* Get Rx'd CAN Msg DLC Size.                           */
        p_frm->DLC        =                RX200_CAN_RxPTRn_RMDLC(p_reg->RX_FIFO[0u].RFPTRx);
                                                                /* Set Start Address of Rx'd FIFO Data.                 */
        p_data            = (CPU_INT08U *)&p_reg->RX_FIFO[0u].RFDF0x;
        
        for (i = 0u; i < p_frm->DLC; i++) {                     /* Get Rx FIFO Data Bytes. Copy to CAN Frame.           */
            p_frm->Data[i] = *p_data;
            p_data++;                                           /* Increment Pointer to next Data Location.             */
        }
        
        p_reg->RFPCTR0 = RX200_CAN_xFPCTRn_xFPC_0xFFu;          /* Increment Rx FIFO To next Unread Msg in Rx FIFO.     */
        result         = size;
                                                                /* ------------------ CLR ISR Rx FLAG ----------------- */
        DEF_BIT_CLR(p_reg->RFSTS0, RX200_CAN_xFSTSn_xFIF);      /* Clear Rx FIFO Interrupt Request, Once Frame is Rx'd. */
        
                                                                /* ----------------- Rx FIFO Channel 1 ---------------- */
    } else if (DEF_BIT_IS_CLR(p_reg->RFSTS1, RX200_CAN_xFSTSn_xFEMP) == DEF_YES) {
                                                                /*  - Extended ID -                                     */
        if (DEF_BIT_IS_SET(p_reg->RX_FIFO[1u].RFIDHx, RX200_CAN_RxIDHn_RMIDE) == DEF_YES) {
            frm_id  =  p_reg->RX_FIFO[1u].RFIDLx;               /* Read Bottom-Half of Rx ID, then Top-Half of Rx ID.   */
            frm_id |= (RX200_CAN_RxIDHn_RMID(p_reg->RX_FIFO[1u].RFIDHx) << 16u);
                
            DEF_BIT_SET(frm_id, RX200_CAN_FRM_IDE);             /* Set   CAN Frame IDE Bit.                             */
            
        } else {                                                /*  - Standard ID -                                     */
            frm_id = RX200_CAN_RxIDLn_RMID_STD(p_reg->RX_FIFO[1u].RFIDLx);
            DEF_BIT_CLR(frm_id, RX200_CAN_FRM_IDE);             /* Clear CAN Frame IDE Bit.                             */
        }
                                                                /*  - Remote Frame Check -                              */
        if (DEF_BIT_IS_SET(p_reg->RX_FIFO[1u].RFIDHx, RX200_CAN_RxIDHn_RMRTR) == DEF_YES) {
            DEF_BIT_SET(frm_id, RX200_CAN_FRM_RTR);             /* Set CAN Frame RTR Bit, if Rx'd Msg is an RTR Msg.    */
        }

        p_frm->Identifier = frm_id;                             /* Save CAN Frame ID.                                   */
                                                                /* Get Rx'd CAN Msg DLC Size.                           */
        p_frm->DLC        =                RX200_CAN_RxPTRn_RMDLC(p_reg->RX_FIFO[1u].RFPTRx);
                                                                /* Set Start Address of Rx'd FIFO Data.                 */
        p_data            = (CPU_INT08U *)&p_reg->RX_FIFO[1u].RFDF0x;
        
        for (i = 0u; i < p_frm->DLC; i++) {                     /* Get Rx FIFO Data Bytes. Copy to CAN Frame.           */
            p_frm->Data[i] = *p_data;
            p_data++;                                           /* Increment Pointer to next Data Location.             */
        }
        
        p_reg->RFPCTR1 = RX200_CAN_xFPCTRn_xFPC_0xFFu;          /* Increment Rx FIFO To next Unread Msg in Rx FIFO.     */
        result         = size;
                                                                /* ------------------ CLR ISR Rx FLAG ----------------- */
        DEF_BIT_CLR(p_reg->RFSTS1, RX200_CAN_xFSTSn_xFIF);      /* Clear Rx FIFO Interrupt Request, Once Frame is Rx'd. */
        
    } else {                                                    /* No Message in Rx FIFO Channel 0 or 1.                */
        RX200_DrvErr = RX200_CAN_ERR_MAILBOX;
    }
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                           RX200_CAN_Write()
*
* Description : Write a CAN Frame to a Message Buffer. The Buffer must contain only one CAN Frame
*               which will be written to a predefined Message Buffer.
*
* Argument(s) : para_id     Parameter Identifier, returned by RX200_CAN_Open().
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
* Note(s)     : (1) For Transmission, the Tx Buffer Status Register(s) of each configured Tx Buffer
*                   are read to make sure that no Tx is in Progress, no Tx Request has been
*                   made, and that no Tx Abort Request has been made. The Driver will select the first
*                   Tx Buffer available that meets these requirements and will transmit the CAN Frame
*                   on that buffer.
*********************************************************************************************************
*/

CPU_INT16S  RX200_CAN_Write (CPU_INT16S   para_id,
                             CPU_INT08U  *buf,
                             CPU_INT16U   size)
{
    RX200_CAN_REG  *p_reg;
    RX200_CAN_FRM  *p_frm;
    CPU_INT08U     *p_tx_sts;
    CPU_INT08U     *p_tx_ctrl;
    CPU_INT08U     *p_data;
    CPU_INT08U      tx_mbox;
    CPU_INT08U      i;
    CPU_INT16S      result;
    CPU_SR_ALLOC();

    
    result = -1;                                                /* Initializing Variable(s)                             */
    i      =  0u;
    
    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= RX200_CAN_N_DEV)) {
        RX200_DrvErr = RX200_CAN_ERR_BUS;
        return (result);
    }
    
    if (size != sizeof(RX200_CAN_FRM)) {                        /* Check if Size is Plausible                           */
        RX200_DrvErr = RX200_CAN_ERR_NO_DATA;
        return (result);
    }
    
    if (buf == (void *)0) {                                     /* Check if Buffer Pointer is Invalid                   */
        RX200_DrvErr = RX200_CAN_ERR_ARG;
        return (result);
    }
    
    if (RX200_DevData[para_id].Use != DEF_YES) {                /* Check if CAN Device is Opened                        */
        RX200_DrvErr = RX200_CAN_ERR_OPEN;
        return (result);      
    }
                                                                /* Set Base Address for CAN Device Register Set.        */
    p_reg     = (RX200_CAN_REG *) RX200_DevData[para_id].RegPtr;
    p_frm     = (RX200_CAN_FRM *) buf;
    p_tx_sts  = (CPU_INT08U    *)&p_reg->TMSTSx[0u];            /* Set Start Address for Tx Buffer Status  Registers.   */
    p_tx_ctrl = (CPU_INT08U    *)&p_reg->TMCx[0u];              /* Set Start Address for Tx Buffer Control Registers.   */
    
    CPU_CRITICAL_ENTER();
    
                                                                /* ---------- SEARCH FOR AVAILABLE TX BUFFER ---------- */
    for (i = 0u; i < RX200_CAN_TX_MBOX; i++) {                  /* Check if Tx Mbox is Available to Tx. See Note (1).   */
        if (DEF_BIT_IS_CLR(*p_tx_sts, (RX200_CAN_TMSTSx_TMTSTS |
                                       RX200_CAN_TMSTSx_TMTRM  |
                                       RX200_CAN_TMSTSx_TMTARM)) == DEF_YES) {
            tx_mbox = i;                                        /* Use this Tx Mbox for Transmission.                   */
            break;
        } else {
            p_tx_sts++;                                         /* Increment Tx Buffer Status Registers to next Tx Buff.*/
        }
    }
    
    if (i >= RX200_CAN_TX_MBOX) {                               /* No Tx Buffer is Available. Return from Tx.           */
        RX200_DrvErr = RX200_CAN_ERR_BUSY;
        CPU_CRITICAL_EXIT();
        return (result);
    }
                                                                /* ------------------- TX CAN FRAME ------------------- */    
                                                                /*  - Extended ID -                                     */
    if ((DEF_BIT_IS_SET(p_frm->Identifier, RX200_CAN_FRM_IDE) == DEF_YES) || 
        (p_frm->Identifier > RX200_CAN_SID_LIMIT)) {            /* Set Bottom-Half of Frm ID, then Top-Half of Frm ID.  */
        p_reg->TX[tx_mbox].TMIDLx = RX200_CAN_TxIDLn_CFID_EXT(p_frm->Identifier);
        p_reg->TX[tx_mbox].TMIDHx = RX200_CAN_TxIDHn_CFID(p_frm->Identifier);
                                                                /* Set Extended ID Flag.                                */
        DEF_BIT_SET(p_reg->TX[tx_mbox].TMIDHx, RX200_CAN_TxIDHn_TMIDE);
            
    } else {                                                    /*  - Standard ID -                                     */
        p_reg->TX[tx_mbox].TMIDLx = RX200_CAN_TxIDLn_CFID_STD(p_frm->Identifier);
        p_reg->TX[tx_mbox].TMIDHx = 0u;                         /* Clear Standard ID Flag, and Tx History Data Enable.  */
    }
                                                                /*  - Remote Frame Check -                              */
    if (DEF_BIT_IS_SET(p_frm->Identifier, RX200_CAN_FRM_RTR) == DEF_YES) {
        DEF_BIT_SET(p_reg->TX[tx_mbox].TMIDHx, RX200_CAN_TxIDHn_TMRTR);
    }
                                                                /* Set CAN Frame DLC Size.                              */
    p_reg->TX[tx_mbox].TMPTRx = RX200_CAN_TxPTRn_TxDLC(p_frm->DLC);
    
    p_data = (CPU_INT08U *)&p_reg->TX[tx_mbox].TMDF0x;          /* Set Start Address for Tx Buffer Address Area.        */
    
    for (i = 0u; i < p_frm->DLC; i++) {                         /* Set CAN Frame Data & Clear Previous Data in Buffer.  */
       *p_data = p_frm->Data[i];                                /* Save Byte by Byte of CAN Frame Data to Tx Buffer.    */
        p_data++;
    }
    
    p_tx_ctrl = (p_tx_ctrl + tx_mbox);                          /* Update the Tx Buffer CTRL Reg with the proper Tx Buf.*/
    DEF_BIT_SET(*p_tx_ctrl, RX200_CAN_TMCx_TMTR);               /* Request Transmission of CAN Frame. (Tx Enable).      */
    
    CPU_CRITICAL_EXIT();
    
    result = size;
    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                         RX200_CAN_SetMode()
*
* Description : Sets the Generic CAN Mode.
*
* Argument(s) : para_id     Desired CAN Module ID.
*
*               mode        Desired Mode to set CAN.
*
* Return(s)   : Returns error value of 'DEF_FAIL' if Timeout occurred, if not returns 'DEF_OK'
*
* Caller(s)   : RX200_CAN_Init()
*               RX200_CAN_IoCtl()
*
* Note(s)     : (1) According to the Reference Manual, found in the Channel Mode Transition Chart, "While
*                   the CAN bus is locked at the dominant level tranisition to Channel Halt Mode is
*                   not made. In that case, enter Channel Reset Mode".
*********************************************************************************************************
*/

CPU_BOOLEAN  RX200_CAN_SetMode (CPU_INT32U      para_id,
                                RX200_CAN_MODE  can_mode)
{
    CPU_INT08U      gl_mode;
    CPU_INT08U      ch_mode;
    CPU_BOOLEAN     can_err;


    if (para_id >= RX200_CAN_N_DEV) {                           /* Improper CAN Module ID.                              */
        return (DEF_FAIL);
    }
    
    can_err = DEF_OK;                                           /* Initialize Variable(s)                               */
    
    switch (can_mode) {
        case RX200_CAN_MODE_OPERATE:                            /* ----------------- CAN OPERATE MODE ----------------- */
             can_err = RX200_CAN_BSP_Start(para_id);            /* Starts the Desired CAN Module                        */
             if (can_err == DEF_FAIL) {
                 RX200_DrvErr = RX200_CAN_ERR_FUNC;             /* CAN_BSP_START Timed Out, Return Error                */
                 return (DEF_FAIL);
             }
             
             gl_mode = RX200_CAN_DrvGetMode(para_id,            /* Get Current Global Mode.                             */
                                            DEF_YES);
             
             ch_mode = RX200_CAN_DrvGetMode(para_id,            /* Get Current Channel Mode.                            */
                                            DEF_NO);
             
             if (gl_mode == RX200_CAN_DRV_GL_MODE_STOP) {       /* Transition to Global Reset Mode Prior to Operate Mode*/
                 can_err = RX200_CAN_DrvSetMode(                       para_id,
                                                                       RX200_CAN_DRV_GL_MODE_RESET,
                                                (RX200_CAN_DRV_CH_MODE)ch_mode);
                 if (can_err != DEF_OK) {
                     break;
                 }
             }
             
             can_err = RX200_CAN_DrvSetMode(                       para_id,
                                                                   RX200_CAN_DRV_GL_MODE_OPERATING,
                                            (RX200_CAN_DRV_CH_MODE)ch_mode);
             
             if (can_err != DEF_OK) {
                 break;
             }
             
             if (ch_mode == RX200_CAN_DRV_CH_MODE_STOP) {       /* Transition to Channel Reset Mode Prior to Comm Mode. */
                 can_err = RX200_CAN_DrvSetMode(para_id,
                                                RX200_CAN_DRV_GL_MODE_OPERATING,
                                                RX200_CAN_DRV_CH_MODE_RESET);
                 if (can_err != DEF_OK) {
                     break;
                 }
             }
             
             can_err = RX200_CAN_DrvSetMode(para_id,            /* Set to Global Operating & Channel Communication Mode.*/
                                            RX200_CAN_DRV_GL_MODE_OPERATING,
                                            RX200_CAN_DRV_CH_MODE_COMM);
             break;

             
        case RX200_CAN_MODE_TEST:                               /* ------------------- CAN TEST MODE ------------------ */
             gl_mode = RX200_CAN_DrvGetMode(para_id,            /* Get Current Global Mode when Calling Global Test.    */
                                            DEF_YES);
             
             ch_mode = RX200_CAN_DrvGetMode(para_id,            /* Get Current Channel Mode.                            */
                                            DEF_NO);
             
             if (gl_mode == RX200_CAN_DRV_GL_MODE_STOP) {       /* Transition to Global Reset Mode Prior to Test Mode.  */
                 can_err = RX200_CAN_DrvSetMode(                       para_id,
                                                                       RX200_CAN_DRV_GL_MODE_RESET,
                                                (RX200_CAN_DRV_CH_MODE)ch_mode);
                 if (can_err != DEF_OK) {
                     break;
                 }
             }
                                                                /* Set Global Test Mode with Current Channel Mode.      */
             can_err = RX200_CAN_DrvSetMode(                       para_id,
                                                                   RX200_CAN_DRV_GL_MODE_TEST,
                                            (RX200_CAN_DRV_CH_MODE)ch_mode);
             break;

             
        case RX200_CAN_MODE_GL_RESET:                           /* ----------------- GLOBAL RESET MODE ---------------- */
             can_err = RX200_CAN_DrvSetMode(para_id,
                                            RX200_CAN_DRV_GL_MODE_RESET,
                                            RX200_CAN_DRV_CH_MODE_RESET);
             break;
             
             
        case RX200_CAN_MODE_CH_RESET:                           /* ---------------- CHANNEL RESET MODE ---------------- */
             gl_mode = RX200_CAN_DrvGetMode(para_id,            /* Get Current Global Mode when calling Channel Reset.  */
                                            DEF_YES);
             
             if (gl_mode == RX200_CAN_DRV_GL_MODE_STOP) {       /* No way to Call Channel Reset from Global Stop.       */
                 can_err = DEF_FAIL;
                 break;
             }
                                                                /* Set CAN Module to Channel Reset Mode.                */
             can_err = RX200_CAN_DrvSetMode(                       para_id,
                                            (RX200_CAN_DRV_GL_MODE)gl_mode,
                                                                   RX200_CAN_DRV_CH_MODE_RESET);
             break;

             
        case RX200_CAN_MODE_STOP:                               /* ------------------- CAN STOP MODE ------------------ */
             gl_mode = RX200_CAN_DrvGetMode(para_id,            /* Get Global Mode when Calling CAN Stop (Global & Ch). */
                                            DEF_YES);
                                                                /* If not in Global Reset Mode, place in Global Reset.  */
             if ((gl_mode == RX200_CAN_DRV_GL_MODE_OPERATING) || \
                 (gl_mode == RX200_CAN_DRV_GL_MODE_TEST)) {
                 can_err = RX200_CAN_DrvSetMode(para_id,
                                                RX200_CAN_DRV_GL_MODE_RESET,
                                                RX200_CAN_DRV_CH_MODE_STOP);
                 if (can_err != DEF_OK) {
                     break;
                 }
             } else if (gl_mode == RX200_CAN_DRV_GL_MODE_STOP) {
                 break;                                         /* If already in this Global Mode, Return.              */
             }
             
             can_err = RX200_CAN_DrvSetMode(para_id,            /* Set to Global and Channel Mode Stop.                 */
                                  RX200_CAN_DRV_GL_MODE_STOP,
                                  RX200_CAN_DRV_CH_MODE_STOP);
             break;
             
             
        case RX200_CAN_MODE_HALT:                               /* ------------------- CAN HALT MODE ------------------ */
             gl_mode = RX200_CAN_DrvGetMode(para_id,            /* Get Current Global Mode.                             */
                                            DEF_YES);
             
             ch_mode = RX200_CAN_DrvGetMode(para_id,            /* Get Current Channel Mode.                            */
                                            DEF_NO);
             
             if ((gl_mode == RX200_CAN_DRV_GL_MODE_RESET) || \
                 (gl_mode == RX200_CAN_DRV_GL_MODE_STOP)) {
                 can_err = DEF_FAIL;                            /* Must be Called from Global Operating OR Test Mode.   */
                 break;
             }
             
             if (ch_mode == RX200_CAN_DRV_CH_MODE_STOP) {       /* Transition to Channel Reset Mode Prior to Halt Mode. */
                 can_err = RX200_CAN_DrvSetMode(                       para_id,
                                                (RX200_CAN_DRV_GL_MODE)gl_mode,
                                                                       RX200_CAN_DRV_CH_MODE_RESET);
                 if (can_err != DEF_OK) {
                     break;
                 }
             }
             
                                                                /* Set the CAN Module to Channel HALT Mode.             */
             can_err = RX200_CAN_DrvSetMode(                       para_id,
                                            (RX200_CAN_DRV_GL_MODE)gl_mode,
                                                                   RX200_CAN_DRV_CH_MODE_HALT);
             
             if (can_err != DEF_OK) {                           /* See Note (1).                                        */
                 can_err = RX200_CAN_DrvSetMode(                       para_id,
                                                (RX200_CAN_DRV_GL_MODE)gl_mode,
                                                                       RX200_CAN_DRV_CH_MODE_RESET);
             }
             break;
            
             
        default:                                                /* --------------- UNKNOWN DEFAULT MODE --------------- */
             break;
    }
    
    return (can_err);
}


/*
*********************************************************************************************************
*                                        RX200_CAN_DrvGetMode()
*
* Description : Gets the Current Mode of the CAN Driver based on Global or Channel Mode selection.
*
* Argument(s) : para_id     Desired CAN Module ID.
*
*               global_en   Boolean value to select to check Global or Channel Modes.
*                               DEF_YES = Global  Modes
*                               DEF_NO  = Channel Modes
*
* Return(s)   : Value of Mode CAN Driver is currently in based on Global or Channel selection.
*                   Value = 10+ -> Global  Mode
*                   Value = 20+ -> Channel Mode
*
* Caller(s)   : RX200_CAN_SetMode()
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT08U  RX200_CAN_DrvGetMode(CPU_INT32U   para_id,
                                 CPU_BOOLEAN  global_en)
{
    RX200_CAN_REG  *p_reg;
    CPU_INT08U      mode;
    
    
    p_reg = (RX200_CAN_REG *)RX200_DevData[para_id].RegPtr;     /* Set Base Address for CAN Device Register Set.        */
    
    if (global_en == DEF_YES) {                                 /* ---------------- GET GLOBAL CAN MODE --------------- */
        if (DEF_BIT_IS_SET(p_reg->GSTS, RX200_CAN_GSTS_GSLPSTS) == DEF_YES) {
            mode = RX200_CAN_DRV_GL_MODE_STOP;                  /* CAN Module in Global Stop      Mode.                 */
        } else if (DEF_BIT_IS_SET(p_reg->GSTS, RX200_CAN_GSTS_GRSTSTS) == DEF_YES) {
            mode = RX200_CAN_DRV_GL_MODE_RESET;                 /* CAN Module in Global Reset     Mode.                 */
        } else if (DEF_BIT_IS_SET(p_reg->GSTS, RX200_CAN_GSTS_GHLTSTS) == DEF_YES) {
            mode = RX200_CAN_DRV_GL_MODE_TEST;                  /* CAN Module in Global Test      Mode.                 */
        } else {
            mode = RX200_CAN_DRV_GL_MODE_OPERATING;             /* CAN Module in Global Operating Mode.                 */
        }
        
    } else {                                                    /* --------------- GET CHANNEL CAN MODE --------------- */
        if (DEF_BIT_IS_SET(p_reg->STSL, RX200_CAN_STSL_CSLPSTS) == DEF_YES) {
            mode = RX200_CAN_DRV_CH_MODE_STOP;                  /* CAN Module in Channel Stop          Mode.            */
        } else if (DEF_BIT_IS_SET(p_reg->STSL, RX200_CAN_STSL_CRSTSTS) == DEF_YES) {
            mode = RX200_CAN_DRV_CH_MODE_RESET;                 /* CAN Module in Channel Reset         Mode.            */
        } else if (DEF_BIT_IS_SET(p_reg->STSL, RX200_CAN_STSL_CHLTSTS) == DEF_YES) {
            mode = RX200_CAN_DRV_CH_MODE_HALT;                  /* CAN Module in Channel Halt          Mode.            */
        } else {
            mode = RX200_CAN_DRV_CH_MODE_COMM;                  /* CAN Module in Channel Communication Mode.            */
        }
    }
    
    return (mode);
}


/*
*********************************************************************************************************
*                                        RX200_CAN_DrvSetMode()
*
* Description : Sets the Driver CAN Mode, in accordance to what the Current Mode is (Global & Channel
*               Based).
*
* Argument(s) : para_id     Desired CAN Module ID.
*
*               gl_mode     Desired Global  CAN Mode.
*
*               ch_mode     Desired Channel CAN Mode.
*
* Return(s)   : Returns error value of 'DEF_FAIL' if Timeout occurred, if not returns 'DEF_OK'.
*
* Caller(s)   : RX200_CAN_SetMode()
*
* Note(s)     : (1) Default Mode for both Global/Channel Modes will be Operating/Communication Mode
*                   accordingly.
*
*               (2) The Driver must be in Global/Channel Reset mode prior to setting Stop Mode. Vice
*                   Versa, once exiting Stop Mode, the CAN Driver will be placed in Reset Mode.
*
*               (3) The following equations are used to tranition between Global/Channel Modes on the 
*                   GCTRL/CTRL Register's GMDC/CHMDC Bits without having to clear the Previous Mode prior
*                   to entering the Desired Mode. This does not apply for Global/Channel Stop Mode.
*                       KEY:
*                           Reg  = GCTRL (Global) / CTRL (Channel)
*                           Mask = 0x03u (Bits Used for GMDC/CHMDC Mode)
*
*                               Previous Mode  |  Desired Mode  |   Equation   
*                             ------------------------------------------------
*                                 Reset Mode   | Halt/Test Mode |  Reg ^=  Mask
*                                              | Oper/Comm Mode |  Reg &= ~Mask
*                               Halt/Test Mode |   Reset Mode   |  Reg ^=  Mask
*                                              | Oper/Comm Mode |  Reg &= ~Mask
*                               Oper/Comm Mode | Halt/Test Mode |  Reg |=  Mask
*                                              |   Reset Mode   |  Reg |=  Mask
*********************************************************************************************************
*/

CPU_BOOLEAN  RX200_CAN_DrvSetMode (CPU_INT32U             para_id,
                                   RX200_CAN_DRV_GL_MODE  gl_mode,
                                   RX200_CAN_DRV_CH_MODE  ch_mode)
{
    RX200_CAN_REG  *p_reg;
    CPU_INT32U      timeout;
    CPU_INT08U      gl_curr_mode;
    CPU_INT08U      gl_status_flag;
    CPU_INT08U      ch_curr_mode;
    CPU_INT08U      ch_status_flag;
    
    
    timeout = 0u;
    p_reg   = (RX200_CAN_REG *)RX200_DevData[para_id].RegPtr;   /* Set Base Address for CAN Device Register Set.        */

                                                                /* ------------------ CAN GLOBAL MODE ----------------- */
    gl_curr_mode = RX200_CAN_DrvGetMode(para_id,                /* Get Global Current Mode.                             */
                                        DEF_YES);

    if (gl_curr_mode != gl_mode) {                              /* If Global Current Mode != Desired Global Mode.       */
        switch (gl_mode) {
            case RX200_CAN_DRV_GL_MODE_STOP:                    /* Global Mode : Set Global Stop  & Stop  Status Flag.  */
                                                                /* See Note (2).                                        */
                 DEF_BIT_SET(p_reg->GCTRL, RX200_CAN_GCTRL_GSLPR);
                 gl_status_flag = RX200_CAN_GSTS_GSLPSTS;
                 break;
                 
                 
            case RX200_CAN_DRV_GL_MODE_RESET:                   /* Global Mode : Set Global Reset & Reset Status Flag.  */
                                                                /* See Note (2) & (3).                                  */
                 if (gl_curr_mode == RX200_CAN_DRV_GL_MODE_STOP) {
                     DEF_BIT_CLR(p_reg->GCTRL, RX200_CAN_GCTRL_GSLPR);
                 } else if (gl_curr_mode == RX200_CAN_DRV_GL_MODE_OPERATING) {
                     DEF_BIT_SET(p_reg->GCTRL, RX200_CAN_GCTRL_GMDC_RESET);
                 } else {
                     p_reg->GCTRL ^= RX200_CAN_GCTRL_GMDC_MASK;
                 }
                 
                 gl_status_flag = RX200_CAN_GSTS_GRSTSTS;
                 break;
                 
                 
            case RX200_CAN_DRV_GL_MODE_TEST:                    /* Global Mode : Set Global Test  & Test  Status Flag.  */
                                                                /* See Note (3).                                        */
                 if (gl_curr_mode == RX200_CAN_DRV_GL_MODE_OPERATING) {
                     DEF_BIT_SET(p_reg->GCTRL, RX200_CAN_GCTRL_GMDC_TEST);
                 } else {
                     p_reg->GCTRL ^= RX200_CAN_GCTRL_GMDC_MASK;
                 }
                 
                 gl_status_flag = RX200_CAN_GSTS_GHLTSTS;
                 break;
                 
                 
            case RX200_CAN_DRV_GL_MODE_OPERATING:               /* Global Mode : Set Global Operate & Check All Flags.  */
            default:                                            /* See Note (1) & (3).                                  */
                 DEF_BIT_CLR(p_reg->GCTRL, RX200_CAN_GCTRL_GMDC_MASK);
                 
                 gl_status_flag = (RX200_CAN_GSTS_GSLPSTS |
                                   RX200_CAN_GSTS_GRSTSTS |
                                   RX200_CAN_GSTS_GHLTSTS);
                 break;
        }
                                                                /* Wait to Transition to Desired Global Mode for...     */
        if (gl_mode != RX200_CAN_DRV_GL_MODE_OPERATING) {       /* ... All Modes except for the Operating Mode.         */
            while (DEF_BIT_IS_CLR(p_reg->GSTS, gl_status_flag) == DEF_YES) {
                timeout++;
                
                if (timeout > CAN_TIMEOUT_ERR_VAL) {            /* Check for Timeout Error.                             */
                    return (DEF_FAIL);
                }
            }
        } else {                                                /* Check if Reset, Test, & Stop Flags are Off.          */
            while (DEF_BIT_IS_SET_ANY(p_reg->GSTS, gl_status_flag) == DEF_YES) {
                timeout++;
                
                if (timeout > CAN_TIMEOUT_ERR_VAL) {            /* Check for Timeout Error.                             */
                    return (DEF_FAIL);
                }
            }
        }
    }
    
                                                                /* ----------------- CAN CHANNEL MODE ----------------- */
    timeout      = 0u;                                          /* Reset Timeout Value.                                 */
    ch_curr_mode = RX200_CAN_DrvGetMode(para_id,                /* Get Channel Current Mode.                            */
                                        DEF_NO);
    if (ch_curr_mode != ch_mode) {                              /* If Channel Current Mode != Desired Channel Mode.     */
        switch (ch_mode) {
            case RX200_CAN_DRV_CH_MODE_STOP:                    /* Channel Mode : Set Channel Stop  & Stop  Status Flag.*/
                                                                /* See Note (2).                                        */
                 DEF_BIT_SET(p_reg->CTRL, RX200_CAN_CTRL_CSLPR);
                 ch_status_flag = RX200_CAN_STSL_CSLPSTS;
                 break;
                 
                 
            case RX200_CAN_DRV_CH_MODE_RESET:                   /* Channel Mode : Set Channel Reset & Reset Status Flag.*/
                                                                /* See Note (2) & (3).                                  */
                 if (ch_curr_mode == RX200_CAN_DRV_CH_MODE_STOP) {
                     DEF_BIT_CLR(p_reg->CTRL, RX200_CAN_CTRL_CSLPR);
                 } else if (ch_curr_mode == RX200_CAN_DRV_CH_MODE_COMM) {
                     DEF_BIT_SET(p_reg->CTRL, RX200_CAN_CTRL_CHMDC_RESET);
                 } else {
                     p_reg->CTRL ^= RX200_CAN_CTRL_CHMDC_MASK;
                 }
                 
                 ch_status_flag = RX200_CAN_STSL_CRSTSTS;
                 break;
                 
                 
            case RX200_CAN_DRV_CH_MODE_HALT:                    /* Channel Mode : Set Channel Halt  & Stop  Status Flag.*/
                                                                /* See Note (3).                                        */
                 if (ch_curr_mode == RX200_CAN_DRV_CH_MODE_COMM) {
                     DEF_BIT_SET(p_reg->CTRL, RX200_CAN_CTRL_CHMDC_HALT);
                 } else {
                     p_reg->CTRL ^= RX200_CAN_CTRL_CHMDC_MASK;
                 }
                 
                 ch_status_flag = RX200_CAN_STSL_CHLTSTS;
                 break;
                 
                 
            case RX200_CAN_DRV_CH_MODE_COMM:                    /* Channel Mode : Set Channel Comm  & Check All Flag.   */
            default:                                            /* See Note (1) & (3).                                  */
                 DEF_BIT_CLR(p_reg->CTRL, RX200_CAN_CTRL_CHMDC_MASK);
                 
                 ch_status_flag = (RX200_CAN_STSL_CSLPSTS |
                                   RX200_CAN_STSL_CRSTSTS |
                                   RX200_CAN_STSL_CHLTSTS);
                 break;
        }
                                                                /* Wait to Transition to Desired Channel Mode for...    */
        if (ch_mode != RX200_CAN_DRV_CH_MODE_COMM) {            /* ... All Modes except for the Communication Mode.     */
            while (DEF_BIT_IS_CLR(p_reg->STSL, ch_status_flag) == DEF_YES) {
                timeout++;
                
                if (timeout > CAN_TIMEOUT_ERR_VAL) {            /* Check for Timeout Error.                             */
                    return (DEF_FAIL);
                }
            }
        } else {                                                /* Check if Reset, Test, & Stop Flags are Off.          */
            while (DEF_BIT_IS_SET_ANY(p_reg->STSL, ch_status_flag) == DEF_YES) {
                timeout++;
                
                if (timeout > CAN_TIMEOUT_ERR_VAL) {            /* Check for Timeout Error.                             */
                    return (DEF_FAIL);
                }
            }
        }
    }

    return (DEF_OK);                                            /* If No Errors, Return DEF_OK.                         */
}


/*
*********************************************************************************************************
*                                        RX200_CAN_ErrCheck()
*
* Description : Corrects Errors found in the Status & Error Interrupt Factor Judge Register(s).
*
* Argument(s) : para_id     Desired CAN Module ID.
*
* Return(s)   : none.
*
* Caller(s)   : RX200_CAN_NSHandler(),
*
* Note(s)     : (1) The RX200_CAN_Open() Function requires the BUS Device ID and BUS Device Node Name
*                   which are the same number for all channels since each BUS ID and BUS NODE (found 
*                   in can_cfg.c) for each channel have to be different. The Mode in which to place the
*                   re-initialized Channel is for Exclusive Read/Write Access to the Can Bus.
*********************************************************************************************************
*/

void  RX200_CAN_ErrCheck (CPU_INT32U  para_id)
{
    RX200_CAN_REG  *p_reg;
    CPU_INT16S      dev_id;
    CPU_INT32U      baud_rate;
    
    
    if (para_id < RX200_CAN_N_DEV) {                            /* Check if Parameter ID is out of Range, w/o return    */
        p_reg = (RX200_CAN_REG *)RX200_DevData[para_id].RegPtr; /* Set Base Address for CAN Device Register Set.        */
        
                                                                /* ------------- BUS OFF or ERROR PASSIVE ------------- */
        if (DEF_BIT_IS_SET_ANY(p_reg->ERFLL, (RX200_CAN_ERFLL_BORF |
                                              RX200_CAN_ERFLL_BOEF |
                                              RX200_CAN_ERFLL_EPF)) == DEF_YES) {
            baud_rate = CAN_DEFAULT_BAUDRATE;                   /* Set Restart Baudrate to Default Baudrate             */
                                              
            RX200_CAN_IoCtl(para_id,                            /* Stop CAN Module, for Re-Initialization               */
                            IO_RX200_CAN_STOP,
                            0u);
            
            RX200_CAN_Init(para_id);                            /* Re-Initialize CAN Module                             */
            
            dev_id = RX200_CAN_Open(para_id,                    /* Re-Open CAN Module. See Note(1)                      */
                                    para_id,
                                    DEV_RW);
            
            RX200_CAN_IoCtl(         dev_id,                    /* Configure Controller with Baudrate                   */
                                     IO_RX200_CAN_SET_BAUDRATE,
                            (void *)&baud_rate);
            
            RX200_CAN_IoCtl(dev_id,                             /* Re-Enable CAN Module                                 */
                            IO_RX200_CAN_START,
                            0u);
                                                                /* ------------------- OVERLOAD FLAG ------------------ */
        } else if (DEF_BIT_IS_SET(p_reg->ERFLL, RX200_CAN_ERFLL_OVLF) == DEF_YES) {
            DEF_BIT_CLR(p_reg->ERFLL, RX200_CAN_ERFLL_OVLF);    /* Clear   Overload Flag  Detected Flag.                */
        
                                                                /* --------------------- BUS LOCK --------------------- */
        } else if (DEF_BIT_IS_SET(p_reg->ERFLL, RX200_CAN_ERFLL_BLF) == DEF_YES) {
            DEF_BIT_CLR(p_reg->ERFLL, RX200_CAN_ERFLL_BLF);     /* Clear     Bus Lock     Detected Flag.                */
            
                                                                /* ----------------- ARBITRATION LOST ----------------- */
        } else if (DEF_BIT_IS_SET(p_reg->ERFLL, RX200_CAN_ERFLL_ALF) == DEF_YES) {
            DEF_BIT_CLR(p_reg->ERFLL, RX200_CAN_ERFLL_ALF);     /* Clear Arbitration Lost Detected Flag.                */
            
                                                                /* ------------------- ERROR WARNING ------------------ */
        } else if (DEF_BIT_IS_SET(p_reg->ERFLL, RX200_CAN_ERFLL_EWF) == DEF_YES) {
            DEF_BIT_CLR(p_reg->ERFLL, RX200_CAN_ERFLL_EWF);     /* Clear   Error Warning  Detected Flag.                */
        } else {
            ;                                                   /* ------------------- UNKNOWN ERROR ------------------ */
        }
    }
}
