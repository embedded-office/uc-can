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
*                                         ZYNQ-ZC7000 Series
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

static  const  CPU_INT32U       ZC7xxx_DrvId = 0x7C4A15D9u;     /* Unique Driver Identification Code                    */

static         ZC7xxx_CAN_ERR   ZC7xxx_DrvErr;                  /* Holds Detailed Error Code if Detected                */

                                                                /* Array Holds Driver Runtime Data                      */
static         ZC7xxx_CAN_DATA  ZC7xxx_DevData[ZC7xxx_CAN_N_DEV];


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

static  CPU_BOOLEAN  ZC7xxx_CAN_SetMode(CPU_INT32U       para_id,
                                        ZC7xxx_CAN_MODE  mode);


/*
*********************************************************************************************************
*                                          ZC7xxx_CAN_Init()
*
* Description : Initializes the CAN Driver with the given Device Name.
*
* Argument(s) : para_id     Device ID. [ZC7xxx_CAN_BUS_0, ZC7xxx_CAN_BUS_1].
*
* Return(s)   : Error code:  0 = No Error.
*                           -1 = Error Occurred.
*
* Caller(s)   : CanBusEnable().
*
* Note(s)     : (1) Once the CAN Controller has be 'Reset', it should come up in 'Config' Mode. To confirm
*                   'Config' Mode will be set once more, following a 'Reset' at Initialization.
*
*               (2) In order for an Rx Mailbox (MB) to be added on to the Rx FIFO MB Register, when
*                   Acceptance Filtering is used, then the Rx'd MB must satisfy the following Equation.
*                       EQ:
*                               IF (CAN.AFMR & Rx_MB.ID) == (CAN.AFMR & CAN.AFIR) is:
*                                   TRUE  -> Capture Message, Store in Rx FIFO.
*                                   FALSE -> Reject Message, Do not store in Rx FIFO.
*
*                   Acceptance Filtering Mask & ID registers must be set by using the ZC7xxx_CAN_IoCtl() API
*                   Function call in their application code, with the 'IO_ZC7xxx_CAN_SET_RX_FILTER' opt code
*                   selected.
*
*               (3) CAN OPERATION MODE CONFIGURATION Section of the Init() function is based on User Input.
*                   By default, the Operation Mode of the CAN controller is configured to 'NORMAL' Mode.
*                   For diagnostic checking, additional operating modes have been included in the init but,
*                   a specific define must be redefined in can_cfg.h to select either 'LOOP BACK' or 'SNOOP'
*                   Mode(s). Once CAN has been initialized, it is possible to change between operating modes
*                   at runtime using the xxx_CAN_IoCtl() API Function Call.
*                       The necessary redefined #define is -> CAN_DIAGNOSTIC_SELECT
*********************************************************************************************************
*/

CPU_INT16S  ZC7xxx_CAN_Init (CPU_INT32U  para_id)
{
    ZC7xxx_CAN_REG   *p_reg;
    ZC7xxx_CAN_BAUD   baud;
    CPU_BOOLEAN       drv_err;
    CPU_INT16S        result;
    
    
    result = -1;                                                /* Initialize Variable(s)                               */

    switch (para_id) {                                          /* ------------ INITIALIZE CAN BUS SETTINGS ----------- */
    #if  (CAN_MODULE_CHANNEL_0 == DEF_ENABLED)
        case ZC7xxx_CAN_BUS_0:
             ZC7xxx_DevData[ZC7xxx_CAN_BUS_0].RegPtr = (ZC7xxx_CAN_REG *)ZC7xxx_CAN0_ADDR;
             break;
    #endif
             
    #if  (CAN_MODULE_CHANNEL_1 == DEF_ENABLED)
        case ZC7xxx_CAN_BUS_1:
             ZC7xxx_DevData[ZC7xxx_CAN_BUS_1].RegPtr = (ZC7xxx_CAN_REG *)ZC7xxx_CAN1_ADDR;
             break;
    #endif
                
        default:
             ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_INIT;               /* Return Error if CAN Device is out of Range.          */
             return (result);
    }
    
    ZC7xxx_DevData[para_id].Use       = DEF_NO;                 /* Set Proper Can Device to UNUSED Status               */
    ZC7xxx_DevData[para_id].Prev_Mode = ZC7xxx_CAN_MODE_NORMAL; /* Default setting set to 'Normal' Mode.                */

    p_reg = ZC7xxx_DevData[para_id].RegPtr;                     /* Set Base Address for specified CAN Device.           */
    
                                                                /* No Error with Mailboxes, Proceed with Init           */
    ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_NONE;                        /* Reset Driver Error                                   */
    
                                                                /* ---------------- CONFIGURE CAN MODE ---------------- */
    drv_err = ZC7xxx_CAN_PinSetting(para_id);                   /* Configure Pin & Clk Settings for CAN Device.         */
    if (drv_err == DEF_FAIL) {
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_INIT;                    /* Pin & Clk Settings Timeout, Return Error             */
        return (result);
    }
                                                                /* Reset CAN Controller, & CAN Register(s).             */
    drv_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_RESET);
    if (drv_err == DEF_FAIL) {
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_INIT;                    /* CAN Reset Timeout Occurred, Return Error             */
        return (result);
    }
                                                                /* Enter Config Mode. See Note (1).                     */
    drv_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_CONFIG);
    if (drv_err == DEF_FAIL) {
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_INIT;                    /* CAN Configuration Mode Timeout, Return Error         */
        return (result);
    }

                                                                /* ---------------- CONFIGURE BAUD RATE --------------- */
    baud.BaudRate         = CAN_DEFAULT_BAUDRATE;               /* Set Default Baud Rate Settings                       */
    baud.SamplePoint      = CAN_DEFAULT_SP;
    baud.ReSynchJumpWidth = CAN_DEFAULT_RJW;

    drv_err = ZC7xxx_CAN_CalcTimingReg(&baud);                  /* Calculate Bit Timing Register Values                 */
    if (drv_err == DEF_FAIL) {
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_INIT;                    /* Bit Timing Error, Return Error                       */
        return (result);
    }

    p_reg->BRPR =  ZC7xxx_CAN_BRPR_BRP(baud.PrescalerDiv);      /* Set Bit Timing Baud Rate Prescaler.                  */

    p_reg->BTR  = (ZC7xxx_CAN_BTR_SJW(baud.SJW)          |      /* Set Sync Jump Width.                                 */
                   ZC7xxx_CAN_BTR_TS2(baud.PhaseBufSeg2) |      /* Set Time Phase Buffer Segment 2.                     */
                   ZC7xxx_CAN_BTR_TS1(baud.PhaseBufSeg1));      /* Set Time Phase Buffer Segment 1.                     */

                                                                /* ---------------- DISABLE INTERRUPTS ---------------- */
    p_reg->IER =      0u;                                       /* Disable All Interrupts.                              */
    p_reg->ESR =   0xFFu;                                       /* Clear All Error Register Bits.                       */
    p_reg->ICR = 0xFFFFu;                                       /* Clear All Interrupt Flags.                           */

#if ((CANBUS_RX_HANDLER_EN > 0u) || \
     (CANBUS_TX_HANDLER_EN > 0u) || \
     (CANBUS_NS_HANDLER_EN > 0u))
    ZC7xxx_CAN_IntSetting(para_id);                             /* Configure Interrupt Settings                         */
#endif
                                                                /* ------------- CONFIGURE INIT FILTERING ------------- */
                                                                /* Disable ALL Acceptance Filter Mask & ID Registers... */
    p_reg->AFR = 0u;                                            /* .. All Rx Messages are Stored in Rx FIFO. Note (2).  */

                                                                /* ------------ Rx & Tx MAILBOX INIT CONFIG ----------- */
    p_reg->TCR = 1u;                                            /* Clear Timestamp Counter.                             */

#ifdef CAN_WATERMARK_Rx_Tx_SIZE
    if ((CAN_WATERMARK_Rx_Tx_SIZE == DEF_BIT_NONE)        ||    /* Check if Defined Watermark Level Exceeds Max. & Min. */
        (CAN_WATERMARK_Rx_Tx_SIZE >= ZC7xxx_CAN_Tx_Rx_FIFO)) {
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_INIT;                    /* Watermark Level Max Error, Return Error              */
        return (result);
    }
                                                                /* Set the TxFIFO Empty & RxFIFO Full Watermark Levels. */
    p_reg->WIR = (ZC7xxx_CAN_WIR_EW(CAN_WATERMARK_Rx_Tx_SIZE) |
                  ZC7xxx_CAN_WIR_FW(CAN_WATERMARK_Rx_Tx_SIZE));

    p_reg->ICR = 0xFFFFu;                                       /* Re-Clear All Interrupt Flag.                         */
#endif

                                                                /* ----------------- ENABLE INTERRUPTS ---------------- */
    p_reg->IER = (ZC7xxx_CAN_IER_ERXFWMFLL |                    /*  - Enable Int: Rx FIFO Watermark Full Interrupt.     */
                  ZC7xxx_CAN_IER_EWKUP     |                    /*  - Enable Int: Wake Up Interrupt.                    */
                  ZC7xxx_CAN_IER_ESLP      |                    /*  - Enable Int: Sleep Interrupt.                      */
                  ZC7xxx_CAN_IER_EBSOFF    |                    /*  - Enable Int: Bus OFF Interrupt.                    */
                  ZC7xxx_CAN_IER_EERROR    |                    /*  - Enable Int: Error Interrupt.                      */
                  ZC7xxx_CAN_IER_ERXOFLW   |                    /*  - Enable Int: Rx FIFO Overflow Interrupt.           */
                  ZC7xxx_CAN_IER_ERXOK     |                    /*  - Enable Int: New Message Received Interrupt.       */
                  ZC7xxx_CAN_IER_ETXOK     |                    /*  - Enable Int: Transmission Successful Interrupt.    */
                  ZC7xxx_CAN_IER_EARBLST);                      /*  - Enable Int: Arbitration Lost Interrupt.           */

                                                                /* --------- CAN OPERATION MODE CONFIGURATION --------- */
                                                                /* See Note (3).                                        */
    if (CAN_DIAGNOSTIC_SELECT == CAN_DIAGNOSTIC_LOOPBACK) {     /* Diagnostic Config Selected: LoopBack Mode.           */
        drv_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_LOOP_BACK);
        if (drv_err == DEF_FAIL) {
            ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_INIT;                /* CAN Configuration Mode Timeout, Return Error         */
            return (result);
        }

    } else if (CAN_DIAGNOSTIC_SELECT == CAN_DIAGNOSTIC_SNOOP) { /* Diagnostic Config Selected: Snoop Mode.              */
        drv_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_SNOOP);
        if (drv_err == DEF_FAIL) {
            ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_INIT;                /* CAN Configuration Mode Timeout, Return Error         */
            return (result);
        }

    } else {                                                    /* Diagnostic Selected off, Place CAN to NORMAL Mode.   */
        drv_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_NORMAL);
        if (drv_err == DEF_FAIL) {
            ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_INIT;                /* CAN Configuration Mode Timeout, Return Error         */
            return (result);
        }
    }

    result = ZC7xxx_CAN_ERR_NONE;                               /* Set Function Result: No Error                        */
    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                          ZC7xxx_CAN_Open()
*
* Description : Unlocks the CAN device, i.e. Read/Write/IoCtl Functions will take effect.
*
* Argument(s) : dev_id      Bus Node Name, used to interface with the CAN Bus Layer.
*
*               dev_name    Driver Device Name, used to interface with the Low-Level Device Driver.
*                           Possible Values for Device Name:
*                                   ZC7xxx_CAN_BUS_0     [CAN Bus 0]
*                                   ZC7xxx_CAN_BUS_1     [CAN Bus 1]
*
*               mode        Mode in which CAN device will be used. Possible Modes are:
*                                   DEV_RW              [EXCLUSIVE READ/WRITE ACCESS]
*                                   DEV_RWX             [EXCLUSIVE READ/WRITE/EXECUTE ACCESS]
*                                   DEV_SHWO            [SHARED WRITE ACCESS]
*                                   DEV_SHRO            [SHARED READ ACCESS]
*                                   DEV_SHRW            [SHARED READ/WRITE ACCESS]
*                                   DEV_SHRWX           [SHARED READ/WRITE/EXECUTE ACCESS]
*
* Return(s)   : Parameter Identifier for further access or (-1) if error occurred.
*
* Caller(s)   : CanCfg in 'can_cfg.c'
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT16S  ZC7xxx_CAN_Open (CPU_INT16S  dev_id,
                             CPU_INT32U  dev_name,
                             CPU_INT16U  mode)
{
    CPU_INT16S  result;
    CPU_SR_ALLOC();

    
    result = -1;                                                /* Initializing Variable(s)                             */

    if (dev_name >= ZC7xxx_CAN_N_DEV) {                         /* Check if Device Name is out of Range                 */
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_BUS;
        return (result);
    }   

    if (mode != DEV_RW) {                                       /* Check if Mode is not Supported                       */
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_MODE;
        return (result);
    }

    CPU_CRITICAL_ENTER();

    if (ZC7xxx_DevData[dev_name].Use == DEF_NO) {               /* Check if CAN Device is Unused                        */
        ZC7xxx_DevData[dev_name].Use = DEF_YES;                 /* Mark CAN Device as Used                              */

#if ((CANBUS_RX_HANDLER_EN > 0u) || \
     (CANBUS_TX_HANDLER_EN > 0u) || \
     (CANBUS_NS_HANDLER_EN > 0u))
        ZC7xxx_CAN_SetDevIds((CPU_INT08U)dev_id,                /* Set Device IDs for the ISRs                          */
                             (CPU_INT08U)dev_name);
#else
        (void)&dev_id;                                          /* Prevent Compiler Warning                             */
#endif
        
        result = (CPU_INT16S)dev_name;                          /* OK, Device is Opened                                 */
    } else {
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_OPEN;
    }
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                         ZC7xxx_CAN_Close()
*
* Description : Locks the CAN device, i.e. Read/Write/IoCtl Functions will not take effect.
*
* Argument(s) : para_id     Parameter Identifier, returned by ZC7xxx_CAN_Open().
*
* Return(s)   : Error code:  0 = No Error.
*                           -1 = Error Occurred.
*
* Caller(s)   : CanCfg in 'can_cfg.c'
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT16S  ZC7xxx_CAN_Close (CPU_INT16S  para_id)
{
    CPU_INT16S  result;
    CPU_SR_ALLOC();

    
    result = -1;                                                /* Initializing Variable(s)                             */

    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= (CPU_INT08S)ZC7xxx_CAN_N_DEV)) {
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_BUS;
        return (result);
    }

    CPU_CRITICAL_ENTER();
    
    if (ZC7xxx_DevData[para_id].Use != DEF_NO) {                /* Check if CAN Device is Used                          */
        ZC7xxx_DevData[para_id].Use = DEF_NO;                   /* Mark CAN Device as Unused                            */
        result = ZC7xxx_CAN_ERR_NONE;                           /* OK, Device is Closed                                 */
    } else {
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_CLOSE;
    }
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                         ZC7xxx_CAN_IoCtl()
*
* Description : Performs Special Action on the Opened Device. The Function Code 'func' defines 
*               what the caller wants to do. Description of Function Codes are defined in the header.
*
* Argument(s) : para_id     Parameter Identifier, returned by ZC7xxx_CAN_Open().
*
*               func        Function Code.
*
*               p_arg       Argument List, Specific to the Function Code. Possible Function Codes are:
*                                       IO_ZC7xxx_CAN_GET_IDENT
*                                       IO_ZC7xxx_CAN_GET_ERRNO
*                                       IO_ZC7xxx_CAN_GET_DRVNAME
*                                       IO_ZC7xxx_CAN_SET_BAUDRATE
*                                       IO_ZC7xxx_CAN_START
*                                       IO_ZC7xxx_CAN_CONFIG
*                                       IO_ZC7xxx_CAN_SLEEP
*                                       IO_ZC7xxx_CAN_LBACK
*                                       IO_ZC7xxx_CAN_SNOOP
*                                       IO_ZC7xxx_CAN_RX_STANDARD
*                                       IO_ZC7xxx_CAN_RX_EXTENDED
*                                       IO_ZC7xxx_CAN_TX_READY
*                                       IO_ZC7xxx_CAN_GET_NODE_STATUS
*                                       IO_ZC7xxx_CAN_SET_RX_FILTER
*                                       IO_ZC7xxx_CAN_IO_FUNC_N
*
* Return(s)   : Error code:  0 = No Error.
*                           -1 = Error Occurred.
*
* Caller(s)   : CanCfg in 'can_cfg.c'
*
* Note(s)     : (1) The SET_RX_FILTER Function code is based on the Acceptance Filter Mask & ID registers
*                   presented in the ZYNQ-7000 Technical Reference Manual under the CAN Register Details.
*                   If the proper bits are not set as explained in the Reference Manual, the incorrect
*                   Acceptance Filter settings will be configured onto the CAN controller. The only section
*                   that is being re-checked is the Extended Frame Identifier, if the Mask or ID register
*                   are enabled for Extended Frame. For more Information refer to Section [18.2.5] of the
*                   TRM on Rx Message Filtering.
*                   (A) If the MASK Register is set to 0u, then the appropriate Mask ID Register (mask_id_sel)
*                       Select bit will be DISABLED in the Acceptance Filter Register (AFR).
*********************************************************************************************************
*/

CPU_INT16S  ZC7xxx_CAN_IoCtl (CPU_INT16S   para_id,
                              CPU_INT16U   func,
                              void        *p_arg)
{
    ZC7xxx_CAN_REG   *p_reg;
    ZC7xxx_CAN_BAUD   baud;
    CPU_INT32U        mask_reg;
    CPU_INT32U        can_id_reg;
    CPU_INT32U        mask_id_sel;
    CPU_REG32        *p_mask_reg;
    CPU_REG32        *p_id_reg;
    CPU_BOOLEAN       can_err;
    CPU_INT16S        result;
    CPU_SR_ALLOC();


    result  = -1;                                               /* Initialize Variable(s)                               */
    can_err = DEF_OK;
    
    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= (CPU_INT08S)ZC7xxx_CAN_N_DEV)) {
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_BUS;
        return (result);
    }

    if (ZC7xxx_DevData[para_id].Use != DEF_YES) {               /* Check if CAN Device is Opened                        */
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_OPEN;
        return (result);
    }
    
    p_reg = ZC7xxx_DevData[para_id].RegPtr;                     /* Set Base Address for CAN Device(s)                   */
    
    CPU_CRITICAL_ENTER();

    switch (func) {                                             /*                SELECT: FUNCTION CODE                 */
        case IO_ZC7xxx_CAN_GET_IDENT:                           /* -------------------- GET IDENT --------------------- */
             *(CPU_INT32U*)p_arg = ZC7xxx_DrvId;                /* Return Driver Identification Code                    */
             result = ZC7xxx_CAN_ERR_NONE;                      /* Indicate Successful Function Execution               */
             break;


        case IO_ZC7xxx_CAN_GET_ERRNO:                           /* ------------------ GET ERRORCODE ------------------- */
             *(CPU_INT16U*)p_arg = ZC7xxx_DrvErr;               /* Return Last Detected Error Code                      */
             result = ZC7xxx_CAN_ERR_NONE;                      /* Indicate Successful Function Execution               */
             break;


        case IO_ZC7xxx_CAN_GET_DRVNAME:                         /* ----------------- GET DRIVER NAME ------------------ */
                                                                /* Return Human Readable Driver Name                    */
             *(CPU_INT08U**)p_arg = (CPU_INT08U*)ZC7xxx_CAN_NAME;
             result = ZC7xxx_CAN_ERR_NONE;                      /* Indicate Successful Function Execution               */
             break;


        case IO_ZC7xxx_CAN_SET_BAUDRATE:                        /* ------------------ SET BAUD RATE ------------------- */

             baud.BaudRate         = *((CPU_INT32U *)p_arg);
             baud.SamplePoint      = CAN_DEFAULT_SP;
             baud.ReSynchJumpWidth = CAN_DEFAULT_RJW;

             can_err = ZC7xxx_CAN_CalcTimingReg(&baud);         /* Calculate Bit Timing Register Values                 */
             if (can_err == DEF_FAIL) {
                 ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_FUNC;           /* Bit Timing Error, Return Error                       */
                 return (result);
             }
                                                                /* Enter CAN Config Mode                                */
             can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_CONFIG);
             if (can_err == DEF_FAIL) {
                 ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_FUNC;           /* CAN Halt Mode Timeout, Return Error                  */
                 return (result);
             }
                                                                /* Set Bit Timing Baud Rate Prescaler                   */
             p_reg->BRPR =  ZC7xxx_CAN_BRPR_BRP(baud.PrescalerDiv);
                                                                /* Set Sync Jump Width, Time Phase Buff Segment 1 & 2   */
             p_reg->BTR  = (ZC7xxx_CAN_BTR_SJW(baud.SJW)          |
                            ZC7xxx_CAN_BTR_TS2(baud.PhaseBufSeg2) |
                            ZC7xxx_CAN_BTR_TS1(baud.PhaseBufSeg1));

                                                                /* Check if Previous Mode was LoopBack or Snoop Mode.   */
             if (ZC7xxx_DevData[para_id].Prev_Mode == ZC7xxx_CAN_MODE_LOOP_BACK) {
                 can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_LOOP_BACK);

             } else if (ZC7xxx_DevData[para_id].Prev_Mode == ZC7xxx_CAN_MODE_SNOOP) {
                 can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_SNOOP);

             } else {                                           /* Default to Normal. This maintains Diagnostic mode(s) */
                 can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_NORMAL);
             }
             break;


        case IO_ZC7xxx_CAN_START:                               /* ------------- START CAN COMMUNICATION -------------- */
                                                                /* Check if Previous Mode was LoopBack or Snoop Mode.   */
             if (ZC7xxx_DevData[para_id].Prev_Mode == ZC7xxx_CAN_MODE_LOOP_BACK) {
                 can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_LOOP_BACK);

             } else if (ZC7xxx_DevData[para_id].Prev_Mode == ZC7xxx_CAN_MODE_SNOOP) {
                 can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_SNOOP);

             } else {                                           /* Default to Normal. This maintains Diagnostic mode(s) */
                 can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_NORMAL);
             }
             break;


        case IO_ZC7xxx_CAN_CONFIG:                              /* ----------------- SET CAN TO CONFIG ---------------- */
                                                                /* Enter CAN Config Operating Mode                      */
             can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_CONFIG);
             break;


        case IO_ZC7xxx_CAN_SLEEP:                               /* ----------------- SET CAN TO SLEEP ----------------- */
                                                                /* Enter CAN Sleep Operating Mode                       */
             can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_SLEEP);
             break;


        case IO_ZC7xxx_CAN_LBACK:                               /* ---------------- SET CAN TO LOOPBACK --------------- */
                                                                /* Enter CAN LoopBack Diagnostics Operating Mode        */
             can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_LOOP_BACK);
             break;


        case IO_ZC7xxx_CAN_SNOOP:                               /* ----------------- SET CAN TO SNOOP ----------------- */
                                                                /* Enter CAN Snoop Diagnostics Operating Mode           */
             can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_SNOOP);
             break;


        case IO_ZC7xxx_CAN_RX_STANDARD:                         /* --------------- SET RX STANDARD -------------------- */
                                                                /* Enter CAN Config Operating Mode                      */
             can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_CONFIG);
             if (can_err == DEF_FAIL) {
                 result = ZC7xxx_CAN_ERR_FUNC;                  /* CAN Halt Mode Timeout, Return Error                  */
                 break;
             }

             mask_id_sel = *((CPU_INT08U *)p_arg);              /* Get Mask & ID # Register selected to Standard Mode.  */

             if (mask_id_sel <= DEF_BIT_NONE) {
                 result = ZC7xxx_CAN_ERR_FUNC;                  /* Incorrect Argument Pointer. Must be >= 1.            */
                 break;
             }

             p_reg->AFR |= 1u << (mask_id_sel - 1u);            /* Enable Acceptance Filter Register, based on selection*/

                                                                /* Copy Location of Initial Acceptance Filter Reg Set.  */
                                                                /* Set Acceptance Filter Mask & ID Reg Addr based on #. */
             p_mask_reg = &p_reg->AFMR1 + (2u * (mask_id_sel - 1u));
             p_id_reg   =  p_mask_reg   + 1u;

             DEF_BIT_CLR(*p_mask_reg, ZC7xxx_CAN_AFMRx_AMIDE);  /* Clear the Extension Mask on the proper Mask Register.*/
             DEF_BIT_CLR(*p_id_reg  , ZC7xxx_CAN_AFIRx_AIIDE);  /* Clear the Identifier Extension on the ID Register.   */

                                                                /* Check if Previous Mode was LoopBack or Snoop Mode.   */
             if (ZC7xxx_DevData[para_id].Prev_Mode == ZC7xxx_CAN_MODE_LOOP_BACK) {
                 can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_LOOP_BACK);

             } else if (ZC7xxx_DevData[para_id].Prev_Mode == ZC7xxx_CAN_MODE_SNOOP) {
                 can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_SNOOP);

             } else {                                           /* Default to Normal. This maintains Diagnostic mode(s) */
                 can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_NORMAL);
             }
             break;


        case IO_ZC7xxx_CAN_RX_EXTENDED:                         /* --------------- SET RX EXTENDED -------------------- */
                                                                /* Enter CAN Config Operating Mode                      */
             can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_CONFIG);
             if (can_err == DEF_FAIL) {
                 result = ZC7xxx_CAN_ERR_FUNC;                  /* CAN Halt Mode Timeout, Return Error                  */
                 break;
             }

             mask_id_sel = *((CPU_INT08U *)p_arg);              /* Get Mask & ID # Register selected to Extended Mode.  */

             if (mask_id_sel <= DEF_BIT_NONE) {
                 result = ZC7xxx_CAN_ERR_FUNC;                  /* Incorrect Argument Pointer. Must be >= 1.            */
                 break;
             }

             p_reg->AFR |= 1u << (mask_id_sel - 1u);            /* Enable Acceptance Filter Register, based on selection*/

                                                                /* Copy Location of Initial Acceptance Filter Reg Set.  */
                                                                /* Set Acceptance Filter Mask & ID Reg Addr based on #. */
             p_mask_reg = &p_reg->AFMR1 + (2u * (mask_id_sel - 1u));
             p_id_reg   =  p_mask_reg   + 1u;

             DEF_BIT_SET(*p_mask_reg, ZC7xxx_CAN_AFMRx_AMIDE);  /* Set the Extension Mask on the proper Mask Register.  */
             DEF_BIT_SET(*p_id_reg  , ZC7xxx_CAN_AFIRx_AIIDE);  /* Set the Identifier Extension on the ID Register.     */

                                                                /* Check if Previous Mode was LoopBack or Snoop Mode.   */
             if (ZC7xxx_DevData[para_id].Prev_Mode == ZC7xxx_CAN_MODE_LOOP_BACK) {
                 can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_LOOP_BACK);

             } else if (ZC7xxx_DevData[para_id].Prev_Mode == ZC7xxx_CAN_MODE_SNOOP) {
                 can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_SNOOP);

             } else {                                           /* Default to Normal. This maintains Diagnostic mode(s) */
                 can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_NORMAL);
             }
             break;


        case IO_ZC7xxx_CAN_TX_READY:                            /* -------------------- TX READY ---------------------- */
             if (DEF_BIT_IS_SET(p_reg->SR, ZC7xxx_CAN_SR_TXFLL) == DEF_YES) {
                 *((CPU_INT08U *)p_arg) = 0u;                   /* Tx FIFO is Full. No Data to Send.                    */
             } else if (DEF_BIT_IS_SET(p_reg->ISR, ZC7xxx_CAN_ISR_TXFLL) == DEF_YES) {
                 *((CPU_INT08U *)p_arg) = 0u;                   /* Tx FIFO is Full. No Data to Send.                    */
             } else {
                 *((CPU_INT08U *)p_arg) = 1u;                   /* Tx FIFO is NOT Full. OK to Transmit.                 */
             }

             result = ZC7xxx_CAN_ERR_NONE;                      /* Indicate Successful Function Execution               */
             break;


        case IO_ZC7xxx_CAN_GET_NODE_STATUS:                     /* ---------------- GET NODE STATUS ------------------- */
             *((CPU_INT08U *)p_arg) = 0u;                       /* Set the Initial Argument Pointer to Zero             */

             if (DEF_BIT_IS_SET(p_reg->SR, ZC7xxx_CAN_SR_ESTAT_ERR_PASS)   == DEF_YES) {
                 *((CPU_INT08U *)p_arg) = 1u;                   /* CAN Module has reached Error Passive State           */
             }
             
             if ((DEF_BIT_IS_SET(p_reg->SR, ZC7xxx_CAN_SR_ESTAT_BUS_OFF) == DEF_YES) ||
                 (DEF_BIT_IS_SET(p_reg->ISR, ZC7xxx_CAN_ISR_BSOFF)       == DEF_YES)) {
                 *((CPU_INT08U *)p_arg) = 2u;                   /* CAN Module has reached Bus-Off State                 */
             }
             
             ZC7xxx_CAN_ErrCheck(para_id);                      /* Check & Correct Errors in CAN Module                 */
            
             result = ZC7xxx_CAN_ERR_NONE;                      /* Indicate Successful Function Execution               */
             break;
            

        case IO_ZC7xxx_CAN_SET_RX_FILTER:                       /* ---------------- SET RX FILTER --------------------- */
             mask_id_sel = ((CPU_INT08U *)p_arg)[0u];           /* Get Mask & ID # Register desired for Filtration.     */
             mask_reg    = ((CPU_INT32U *)p_arg)[1u];           /* Get Rx Filter desired Mask Flags. See Note (1).      */
             can_id_reg  = ((CPU_INT32U *)p_arg)[2u];           /* Get Rx Filter desired ID Flags. See Note (1).        */

                                                                /* Enter Halt Mode                                      */
             can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_CONFIG);
             if (can_err == DEF_FAIL) {
                 result = ZC7xxx_CAN_ERR_FUNC;                  /* CAN Halt Mode Timeout, Return Error                  */
             break;
             }

             if (DEF_BIT_IS_SET_ANY(mask_reg, DEF_INT_32_MASK) == DEF_NO) {
                 p_reg->AFR &= ~(1u << (mask_id_sel - 1u));     /* Disable Acceptance Filter Register, based on select. */
             } else {
                 p_reg->AFR |=   1u << (mask_id_sel - 1u);      /* Enable Acceptance Filter Register, based on select.  */

                                                                /* Copy Location of Initial Acceptance Filter Reg Set.  */
                                                                /* Set Acceptance Filter Mask & ID Reg Addr based on #. */
                 p_mask_reg = &p_reg->AFMR1 + (2u * (mask_id_sel - 1u));
                 p_id_reg   =  p_mask_reg   + 1u;

                *p_mask_reg = mask_reg;                         /* Copy Full content of Mask & ID Register(s) to proper.*/
                *p_id_reg   = can_id_reg;                       /* .. Acceptance Filter.                                */

                                                                /* Double Check: Extension Mask & ID Enabled Bit.       */
                                                                /* If ANY bit is set in the Extended ID or Mask Section.*/
                 if ((DEF_BIT_IS_SET_ANY(mask_reg,   ZC7xxx_CAN_EID_MASK) == DEF_YES) ||
                     (DEF_BIT_IS_SET_ANY(can_id_reg, ZC7xxx_CAN_EID_MASK) == DEF_YES)) {
                                                                /* Set the Extension Mask & ID on the proper Registers. */
                     DEF_BIT_SET(*p_mask_reg, ZC7xxx_CAN_AFMRx_AMIDE);
                     DEF_BIT_SET(*p_id_reg,   ZC7xxx_CAN_AFIRx_AIIDE);
                 } else {                                       /* Clr the Extension Mask & ID on the proper Registers. */
                     DEF_BIT_CLR(*p_mask_reg, ZC7xxx_CAN_AFMRx_AMIDE);
                     DEF_BIT_CLR(*p_id_reg,   ZC7xxx_CAN_AFIRx_AIIDE);
                 }
             }

                                                                /* Check if Previous Mode was LoopBack or Snoop Mode.   */
             if (ZC7xxx_DevData[para_id].Prev_Mode == ZC7xxx_CAN_MODE_LOOP_BACK) {
                 can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_LOOP_BACK);

             } else if (ZC7xxx_DevData[para_id].Prev_Mode == ZC7xxx_CAN_MODE_SNOOP) {
                 can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_SNOOP);

             } else {                                           /* Default to Normal. This maintains Diagnostic mode(s) */
                 can_err = ZC7xxx_CAN_SetMode(para_id, ZC7xxx_CAN_MODE_NORMAL);
             }
             break;


        case IO_ZC7xxx_CAN_IO_FUNC_N:
                                                                /* Set the Size of IO Function Number for return.       */
            *((CPU_INT08U *)p_arg) = IO_ZC7xxx_CAN_IO_FUNC_N + 1u;

             result = ZC7xxx_CAN_ERR_NONE;                      /* Indicate Successful Function Execution               */
             break;


        default:                                                /* -------------- UNKNOWN FUNCTION CODE --------------- */
             break;
    }

    if (can_err == DEF_FAIL) {
        result = ZC7xxx_CAN_ERR_FUNC;                           /* Error occurred in function, Return with Error.       */
    } else {
        result = ZC7xxx_CAN_ERR_NONE;                           /* Indicate Successful Function Execution               */
    }

    CPU_CRITICAL_EXIT();

    if (result == -1) {                                         /* Set Driver Error if Needed                           */
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_FUNC;
    }

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                          ZC7xxx_CAN_Read()
*
* Description : Read a received CAN Frame from a Message Buffer. The Buffer must have space for only
*               one CAN Frame.
*
* Argument(s) : para_id     Parameter Identifier, returned by ZC7xxx_CAN_Open().
*
*               buf         Pointer to CAN Frame.
*
*               size        Length of CAN Frame Memory.
*
* Return(s)   : Error code:  0 = No Error.
*                           -1 = Error Occurred.
*
* Caller(s)   : CanCfg in 'can_cfg.c'
*
* Note(s)     : (1) Software must read all four registers of an Rx Message in the RxFIFO regardless of
*                   how many data bytes (DLC) in the message. (Section [18.2.3] of TRM). Based on the
*                   Register Details (Appendix B) of the TRM, Data Byte #s read from the Data1 & Data2
*                   Registers will return invalid data if the message (DLC) has fewer bytes than being
*                   read.
*                   (A) Thus a 'middle man' is used to split the Data Bytes based on the amount of data
*                       bytes in the CAN Message (DLC Value) as the following format:
*
*                             Bit(s)          : 31                          0
*                                  Data 1 Reg : [  DB0  | DB1  | DB2  | DB3  ]
*                                  Data 2 Reg : [  DB4  | DB5  | DB6  | DB7  ]
*                           p_frm->Data[8u]   : { DB0 , DB1 , DB2 , DB3 , DB4 , DB5 , DB6 , DB7}
*
*               (2) When Using Extended IDs for Rx'd Messages, its required to save both the Standard
*                   Section and Extended Section of the ID and placed both in the Rx Frame in the
*                   following Format:
*                               Bits:  31     30    29                   18                   0
*                       uC/CAN Frame: [ 0u | RTR | IDE |    Standard ID    |    Extended ID    ]
*********************************************************************************************************
*/

CPU_INT16S  ZC7xxx_CAN_Read (CPU_INT16S   para_id,
                             CPU_INT08U  *buf,
                             CPU_INT16U   size)
{
    ZC7xxx_CAN_REG      *p_reg;
    ZC7xxx_CAN_FRM      *p_frm;
    ZC7xxx_CAN_REG_FRM   rx_fifo;
    CPU_INT08U           cnt;
    CPU_INT16S           result;
    CPU_SR_ALLOC();
    
    
    result = -1;                                                /* Initializing Variable(s)                             */
                                                        
    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= (CPU_INT08S)ZC7xxx_CAN_N_DEV)) {
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_BUS;
        return (result);
    }
    
    if (size != sizeof(ZC7xxx_CAN_FRM)) {                       /* Check if Size is Plausible                           */
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_NO_DATA;
        return (result);
    }
    
    if (buf == (void *)0) {                                     /* Check if Buffer Pointer is Invalid                   */
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_ARG;
        return (result);
    }
    
    if (ZC7xxx_DevData[para_id].Use != DEF_YES) {               /* Check if CAN Device is Opened                        */
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_OPEN;
        return (result);
    }
    
    p_reg = ZC7xxx_DevData[para_id].RegPtr;                     /* Set Base Address for CAN Device(s)                   */
    p_frm = (ZC7xxx_CAN_FRM *)buf;
    
    CPU_CRITICAL_ENTER();

                                                                /* ---------------- READ Rx'D FIFO MSG ---------------- */
    if ((DEF_BIT_IS_SET(p_reg->ISR, ZC7xxx_CAN_ISR_RXNEMP) == DEF_YES) || \
        (DEF_BIT_IS_SET(p_reg->ISR, ZC7xxx_CAN_ISR_RXOK)   == DEF_YES)) {

        rx_fifo.ID    = p_reg->RXFIFO_ID;                       /* Read New Message. See Note (1).                      */
        rx_fifo.DLC   = p_reg->RXFIFO_DLC;
        rx_fifo.Data1 = p_reg->RXFIFO_DATA1;
        rx_fifo.Data2 = p_reg->RXFIFO_DATA2;

                                                                /* ---------------- Rx FIFO ID REGISTER --------------- */
                                                                /*  - EXTENDED FRAME ID -                               */
        if (DEF_BIT_IS_SET(rx_fifo.ID, ZC7xxx_CAN_ID_IDE) == DEF_YES) {

                                                                /* Copy Both Standard & Extended IDs. See Note (2).     */
            p_frm->Identifier = ZC7xxx_CAN_Rx_ID_BOTH(rx_fifo.ID);

                                                                /* Set Extended ID Flag on Rx Frame.                    */
            DEF_BIT_SET(p_frm->Identifier, ZC7xxx_CAN_FRM_IDE_FLAG);

                                                                /* Check for Remote Frame Flag on Extended Frames.      */
            if (DEF_BIT_IS_SET(rx_fifo.ID, ZC7xxx_CAN_ID_RTR) == DEF_YES) {
                DEF_BIT_SET(p_frm->Identifier, ZC7xxx_CAN_FRM_RTR_FLAG);
            }

        } else {                                                /*  - STANDARD FRAME ID -                               */
                                                                /* Copy only Standard ID.                               */
            p_frm->Identifier = ZC7xxx_CAN_Rx_ID_IDH(rx_fifo.ID);

                                                                /* Check for Remote Frame Flag on Standard Frames.      */
            if (DEF_BIT_IS_SET(rx_fifo.ID, ZC7xxx_CAN_ID_SRRRTR) == DEF_YES) {
                DEF_BIT_SET(p_frm->Identifier, ZC7xxx_CAN_FRM_RTR_FLAG);
            }
        }
                                                                /* --------------- Rx FIFO DLC REGISTER --------------- */
        p_frm->DLC = ZC7xxx_CAN_Rx_DLC_DLC(rx_fifo.DLC);        /* Read and Store DLC Amount onto Frame.                */

                                                                /* ---------- Rx FIFO DATA1 & DATA2 REGISTERs --------- */
        for (cnt = 0u; cnt < p_frm->DLC; cnt++) {               /* Read Data Bytes based on DLC Value. See Note (1).    */
            if (cnt < ZC7xxx_CAN_DLC_DATA_SPLIT) {              /* MACRO splits Data based on cnt value See Note (1A).  */
                p_frm->Data[cnt] = ZC7xxx_CAN_Rx_DATA_BYTES(rx_fifo.Data1, cnt);
            } else {
                p_frm->Data[cnt] = ZC7xxx_CAN_Rx_DATA_BYTES(rx_fifo.Data2, (cnt - ZC7xxx_CAN_DLC_DATA_SPLIT));
            }
        }

        result = size;

    } else {                                                    /* No Message in Rx FIFO Mailbox Array.                 */
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_NO_DATA;
    }

    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                         ZC7xxx_CAN_Write()
*
* Description : Write a CAN Frame to a Message Buffer. The Buffer must contain only one CAN Frame
*               which will be written to a predefined Message Buffer.
*
* Argument(s) : para_id     Parameter Identifier, returned by ZC7xxx_CAN_Open().
*
*               buf         Pointer to CAN Frame.
*
*               size        Length of CAN Frame Memory.
*
* Return(s)   : Error code:  0 = No Error.
*                           -1 = Error Occurred.
*
* Caller(s)   : CanCfg in 'can_cfg.c'
*
* Note(s)     : (1) Software must write to all four registers of an Tx Message in the TxFIFO regardless of
*                   how many data bytes (DLC) in the message. Based on the Register Details (Appendix B)
*                   of the TRM, Data Byte #s read from the Data1 & Data2 Registers will return invalid
*                   data if the message (DLC) has fewer bytes than being Tx.
*                   (A) Thus a 'middle man' is used to merge the Data Bytes based on the amount of data
*                       bytes in the CAN Message (DLC Value) as the following format:
*
*                           p_frm->Data[8u]   : { DB0 , DB1 , DB2 , DB3 , DB4 , DB5 , DB6 , DB7}
*                             Bit(s)          : 31                          0
*                                  Data 1 Reg : [  DB0  | DB1  | DB2  | DB3  ]
*                                  Data 2 Reg : [  DB4  | DB5  | DB6  | DB7  ]
*********************************************************************************************************
*/

CPU_INT16S  ZC7xxx_CAN_Write (CPU_INT16S   para_id,
                              CPU_INT08U  *buf,
                              CPU_INT16U   size)
{
    ZC7xxx_CAN_REG      *p_reg;
    ZC7xxx_CAN_FRM      *p_frm;
    ZC7xxx_CAN_REG_FRM   tx_fifo;
    CPU_INT08U           cnt;
    CPU_INT16S           result;
    CPU_SR_ALLOC();

    
    result        = -1;                                         /* Initializing Variable(s)                             */
    cnt           =  0u;
    
    tx_fifo.ID    =  0u;                                        /* Pre-Clr 'Middle-Man' Registers. See Note (1).        */
    tx_fifo.DLC   =  0u;
    tx_fifo.Data1 =  0u;
    tx_fifo.Data2 =  0u;

    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= (CPU_INT08S)ZC7xxx_CAN_N_DEV)) {
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_BUS;
        return (result);
    }
    
    if (size != sizeof(ZC7xxx_CAN_FRM)) {                       /* Check if Size is Plausible                           */
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_NO_DATA;
        return (result);
    }
    
    if (buf == (void *)0) {                                     /* Check if Buffer Pointer is Invalid                   */
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_ARG;
        return (result);
    }
    
    if (ZC7xxx_DevData[para_id].Use != DEF_YES) {               /* Check if CAN Device is Opened                        */
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_OPEN;
        return (result);      
    }

    p_reg =  ZC7xxx_DevData[para_id].RegPtr;                    /* Set Base Address for CAN Device(s)                   */
    p_frm = (ZC7xxx_CAN_FRM *)buf;
    
    CPU_CRITICAL_ENTER();
                                                                /* ---------------- WRITE Tx'D FIFO MSG --------------- */
                                                                /* If Tx FIFO is NOT Full, Send Messages.               */
    if (DEF_BIT_IS_CLR(p_reg->ISR, ZC7xxx_CAN_ISR_TXFLL) == DEF_YES) {
                                                                /* ---------------- Tx FIFO ID REGISTER --------------- */
        if ((DEF_BIT_IS_SET(p_frm->Identifier, ZC7xxx_CAN_FRM_IDE_FLAG) == DEF_YES) ||
            (p_frm->Identifier > ZC7xxx_CAN_SID_LIMIT)) {       /*  - EXTENDED FRAME ID -                               */
                                                                /* Set Extended ID in Tx FIFO ID Register.              */
            tx_fifo.ID  = ZC7xxx_CAN_Tx_ID_IDL(p_frm->Identifier);
                                                                /* Set Standard ID as well from CAN Frame.              */
            tx_fifo.ID |= ZC7xxx_CAN_Tx_ID_IDH(p_frm->Identifier >> 18u);

            DEF_BIT_SET(tx_fifo.ID, ZC7xxx_CAN_ID_IDE);         /* Set Extended Identifier Flag.                        */

                                                                /* If Frame RTR Flag set, Set Extended Frame RTR Flag.  */
            if (DEF_BIT_IS_SET(p_frm->Identifier, ZC7xxx_CAN_FRM_RTR_FLAG) == DEF_YES) {
                DEF_BIT_SET(tx_fifo.ID, ZC7xxx_CAN_ID_RTR);
            }
        } else {                                                /*  - STANDARD FRAME ID -                               */
                                                                /* Clears Tx FIFO Msg (and Extended ID Flag & Msg ID).  */
            tx_fifo.ID = ZC7xxx_CAN_Tx_ID_IDH(p_frm->Identifier);

                                                                /* If Frame RTR Flag set, Set Standard Frame RTR Flag.  */
            if (DEF_BIT_IS_SET(p_frm->Identifier, ZC7xxx_CAN_FRM_RTR_FLAG) == DEF_YES) {
                DEF_BIT_SET(tx_fifo.ID, ZC7xxx_CAN_ID_SRRRTR);
            }
        }
                                                                /* --------------- Tx FIFO DLC REGISTER --------------- */
        tx_fifo.DLC = ZC7xxx_CAN_Tx_DLC_DLC(p_frm->DLC);        /* Set Frame DLC in Tx FIFO DLC Register.               */

                                                                /* ---------- Tx FIFO DATA1 & DATA2 REGISTERs --------- */
        for (cnt = 0u; cnt < p_frm->DLC; cnt++) {               /* Write Data Bytes based on DLC Qty.                   */
            if (cnt < ZC7xxx_CAN_DLC_DATA_SPLIT) {              /* MACRO saves splits Data based on cnt value.          */
                tx_fifo.Data1 |= ZC7xxx_CAN_Tx_DATA_BYTES(p_frm->Data[cnt], cnt);
            } else {
                tx_fifo.Data2 |= ZC7xxx_CAN_Tx_DATA_BYTES(p_frm->Data[cnt], (cnt - ZC7xxx_CAN_DLC_DATA_SPLIT));
            }
        }

        p_reg->TXFIFO_ID    = tx_fifo.ID;                       /* Once Buf's have been configured, Send all Data.      */
        p_reg->TXFIFO_DLC   = tx_fifo.DLC;
        p_reg->TXFIFO_DATA1 = tx_fifo.Data1;
        p_reg->TXFIFO_DATA2 = tx_fifo.Data2;

        result = size;

    } else {                                                    /* Tx FIFO is Full                                      */
        ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_BUSY;                    /* All Tx Mailboxes are Busy                            */
    }
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                        ZC7xxx_CAN_SetMode()
*
* Description : Sets the CAN Mode.
*
* Argument(s) : para_id     Desired CAN Module ID.
*
*               mode        Desired Mode to set CAN.
*
* Return(s)   : none.
*
* Caller(s)   : ZC7xxx_CAN_Init().
*
* Note(s)     : (1) This bit is self-clearing. If a '1' is written to this bit, the CAN Controller config
*                   register(s), including the SRR register, are Reset. The controller comes up in 'CONFIG'
*                   mode, thus once 'Reset' has been set, the polling for the 'Config' bit in the Status
*                   Register will be done.
*********************************************************************************************************
*/

static  CPU_BOOLEAN  ZC7xxx_CAN_SetMode (CPU_INT32U       para_id,
                                         ZC7xxx_CAN_MODE  mode)
{
    ZC7xxx_CAN_REG  *p_reg;
    CPU_REG16        reg16;
    CPU_INT32U       i;
    CPU_BOOLEAN      can_err;

    
    p_reg = ZC7xxx_DevData[para_id].RegPtr;                     /* Set Base Address for CAN Device(s)                   */
    i     = 0u;                                                 /* Initialize Variable(s)                               */
    
    switch (mode) {
        case ZC7xxx_CAN_MODE_CONFIG:                            /* ------------------ CONFIGURE MODE ------------------ */
             DEF_BIT_CLR(p_reg->SRR, ZC7xxx_CAN_SRR_CEN);       /* Disable CAN Controller, placing it in Config Mode.   */

             DEF_BIT_CLR(p_reg->MSR, ZC7xxx_CAN_MSR_SNOOP);     /* Clear Snoop, Sleep, and LoopBack CAN Modes.          */
             DEF_BIT_CLR(p_reg->MSR, ZC7xxx_CAN_MSR_LBACK);
             DEF_BIT_CLR(p_reg->MSR, ZC7xxx_CAN_MSR_SLEEP);

             DEF_BIT_CLR(p_reg->SRR, ZC7xxx_CAN_SRR_CEN);       /* Keep CAN Controller Disabled.                        */

             while (DEF_BIT_IS_CLR(p_reg->SR, ZC7xxx_CAN_SR_CONFIG) == DEF_YES) {
                 i++;                                           /* Wait for "Config" Bit to be Set in Status Register.  */

                 if (i > CAN_TIMEOUT_ERR_VAL) {                 /* Check for Timeout Error.                             */
                     return (DEF_FAIL);
                 }
             }
             break;


        case ZC7xxx_CAN_MODE_RESET:
             can_err = ZC7xxx_CAN_BSP_Start(para_id);           /* Restart the Desired CAN Module                       */
             if (can_err != DEF_OK) {
                 ZC7xxx_DrvErr = ZC7xxx_CAN_ERR_FUNC;           /* CAN_BSP_START Timed Out, Return Error                */
                 return (DEF_FAIL);
             }

             DEF_BIT_CLR(p_reg->SRR, ZC7xxx_CAN_SRR_CEN);       /* Disable CAN Controller, placing it in Config Mode.   */

             DEF_BIT_SET(p_reg->SRR, ZC7xxx_CAN_SRR_SRST);      /* Set the CAN Controller Reset Bit. See Note (1).      */

             while (DEF_BIT_IS_CLR(p_reg->SR, ZC7xxx_CAN_SR_CONFIG) == DEF_YES) {
                 i++;                                           /* Wait for "Config" Bit to be Set in Status Register.  */

                 if (i > CAN_TIMEOUT_ERR_VAL) {                 /* Check for Timeout Error.                             */
                     return (DEF_FAIL);
                 }
             }
             break;


        case ZC7xxx_CAN_MODE_SLEEP:                             /* -------------------- SLEEP MODE -------------------- */
             DEF_BIT_CLR(p_reg->SRR, ZC7xxx_CAN_SRR_CEN);       /* Disable CAN Controller, placing it in Config Mode.   */

             DEF_BIT_CLR(p_reg->MSR, ZC7xxx_CAN_MSR_SNOOP);     /* Clear Snoop and LoopBack CAN Modes.                  */
             DEF_BIT_CLR(p_reg->MSR, ZC7xxx_CAN_MSR_LBACK);

             DEF_BIT_SET(p_reg->MSR, ZC7xxx_CAN_MSR_SLEEP);     /* Place CAN Controller in "Sleep Mode".                */
             DEF_BIT_SET(p_reg->SRR, ZC7xxx_CAN_SRR_CEN);       /* Enable CAN Controller.                               */

             while (DEF_BIT_IS_CLR(p_reg->SR, ZC7xxx_CAN_SR_SLEEP) == DEF_YES) {
                 i++;                                           /* Wait for "Sleep" Bit to be Set in Status Register.   */

                 if (i > CAN_TIMEOUT_ERR_VAL) {                 /* Check for Timeout Error.                             */
                     return (DEF_FAIL);
                 }
             }
             break;


        case ZC7xxx_CAN_MODE_LOOP_BACK:                         /* ------------------ LOOP BACK MODE ------------------ */
             DEF_BIT_CLR(p_reg->SRR, ZC7xxx_CAN_SRR_CEN);       /* Disable CAN Controller, placing it in Config Mode.   */

             DEF_BIT_CLR(p_reg->MSR, ZC7xxx_CAN_MSR_SNOOP);     /* Clear Snoop and Sleep CAN Modes.                     */
             DEF_BIT_CLR(p_reg->MSR, ZC7xxx_CAN_MSR_SLEEP);

             DEF_BIT_SET(p_reg->MSR, ZC7xxx_CAN_MSR_LBACK);     /* Place CAN Controller in "Loop Back Mode".            */
             DEF_BIT_SET(p_reg->SRR, ZC7xxx_CAN_SRR_CEN);       /* Enable CAN Controller.                               */

             while (DEF_BIT_IS_CLR(p_reg->SR, ZC7xxx_CAN_SR_LBACK) == DEF_YES) {
                 i++;                                           /* Wait for "Loop Back " Bit to be Set in Status Reg.   */

                 if (i > CAN_TIMEOUT_ERR_VAL) {                 /* Check for Timeout Error.                             */
                     return (DEF_FAIL);
                 }
             }
                                                                /* Set Pre_Mode to Loop Back, Diagnostic Configuration. */
             ZC7xxx_DevData[para_id].Prev_Mode = ZC7xxx_CAN_MODE_LOOP_BACK;
             break;


        case ZC7xxx_CAN_MODE_SNOOP:
             DEF_BIT_CLR(p_reg->SRR, ZC7xxx_CAN_SRR_CEN);       /* Disable CAN Controller, placing it in Config Mode.   */

             DEF_BIT_CLR(p_reg->MSR, ZC7xxx_CAN_MSR_LBACK);     /* Clear Loopback and Sleep CAN Modes.                  */
             DEF_BIT_CLR(p_reg->MSR, ZC7xxx_CAN_MSR_SLEEP);

             DEF_BIT_SET(p_reg->MSR, ZC7xxx_CAN_MSR_SNOOP);     /* Place CAN Controller in "Snoop Mode".                */
             DEF_BIT_SET(p_reg->SRR, ZC7xxx_CAN_SRR_CEN);       /* Enable CAN Controller.                               */

             while (DEF_BIT_IS_CLR(p_reg->SR, ZC7xxx_CAN_SR_NORMAL) == DEF_YES) {
                 i++;                                           /* Wait for "Normal" Bit to be Set in Status Register.  */

                 if (i > CAN_TIMEOUT_ERR_VAL) {                 /* Check for Timeout Error.                             */
                     return (DEF_FAIL);
                 }
             }
                                                                 /* Set Pre_Mode to Loop Back, Diagnostic Configuration. */
             ZC7xxx_DevData[para_id].Prev_Mode = ZC7xxx_CAN_MODE_SNOOP;
             break;


        case ZC7xxx_CAN_MODE_NORMAL:                            /* -------------------- NORMAL MODE ------------------- */
             DEF_BIT_CLR(p_reg->SRR, ZC7xxx_CAN_SRR_CEN);       /* Disable CAN Controller, placing it in Config Mode.   */

             DEF_BIT_CLR(p_reg->MSR, ZC7xxx_CAN_MSR_SNOOP);     /* Clear Snoop, Sleep, and LoopBack CAN Modes.          */
             DEF_BIT_CLR(p_reg->MSR, ZC7xxx_CAN_MSR_LBACK);
             DEF_BIT_CLR(p_reg->MSR, ZC7xxx_CAN_MSR_SLEEP);

             DEF_BIT_SET(p_reg->SRR, ZC7xxx_CAN_SRR_CEN);       /* Enable CAN Controller.                               */

             while (DEF_BIT_IS_CLR(p_reg->SR, ZC7xxx_CAN_SR_NORMAL) == DEF_YES) {
                 i++;                                           /* Wait for "Normal" Bit to be Set in Status Register.  */

                 if (i > CAN_TIMEOUT_ERR_VAL) {                 /* Check for Timeout Error.                             */
                     return (DEF_FAIL);
                 }
             }
                                                                /* Set Pre_Mode to Normal Mode Configuration.           */
             ZC7xxx_DevData[para_id].Prev_Mode = ZC7xxx_CAN_MODE_NORMAL;
             break;


        default:                                                /* --------------- UNKNOWN DEFAULT MODE --------------- */
             break;
    }
    
    return (DEF_OK);
}


/*
*********************************************************************************************************
*                                        ZC7xxx_CAN_ErrCheck()
*
* Description : Corrects Errors found in the Status & Error Interrupt Factor Judge Register(s).
*
* Argument(s) : para_id     Desired CAN Module ID.
*
* Return(s)   : none.
*
* Caller(s)   : ZC7xxx_CAN_NSHandler().
*               ZC7xxx_CAN_IoCtl().
*
* Note(s)     : (1) The ZC7xxx_CAN_Open() Function requires the BUS Device ID and BUS Device Node Name
*                   which are the same number for all 2 channels since each BUS ID and BUS NODE (found
*                   in can_cfg.c) for each channel have to be different. The Mode in which to place the
*                   re-initialized Channel is for Exclusive Read/Write Access to the Can Bus.
*********************************************************************************************************
*/

void  ZC7xxx_CAN_ErrCheck (CPU_INT32U  para_id)
{
    ZC7xxx_CAN_REG  *p_reg;
    CPU_INT16S       dev_id;
    CPU_INT32U       baud_rate;


    if (para_id < ZC7xxx_CAN_N_DEV) {                           /* Check if Parameter ID is out of Range, w/o return    */
        p_reg     = ZC7xxx_DevData[para_id].RegPtr;             /* Set Base Address for CAN Device(s)                   */
        baud_rate = CAN_DEFAULT_BAUDRATE;                       /* Set Restart Baudrate to Default Baudrate             */

                                                                /* -------------- BUS-OFF/ERROR DETECTED -------------- */
        if ((DEF_BIT_IS_SET(p_reg->ISR, ZC7xxx_CAN_ISR_BSOFF) == DEF_YES) || \
            (DEF_BIT_IS_SET(p_reg->ISR, ZC7xxx_CAN_ISR_ERROR))) {
                                                                /* Reset CAN Module, for Re-Initialization              */
            ZC7xxx_CAN_IoCtl(para_id, IO_ZC7xxx_CAN_CONFIG, 0u);
            ZC7xxx_CAN_Init(para_id);                           /* Re-Initialize CAN Module                             */
                                                                /* Re-Open CAN Module. See Note(1)                      */
            dev_id = ZC7xxx_CAN_Open(para_id, para_id, DEV_RW);
                                                                /* Configure Controller with Baudrate                   */
            ZC7xxx_CAN_IoCtl(dev_id, IO_ZC7xxx_CAN_SET_BAUDRATE, (void *) &baud_rate);
                                                                /* Re-Enable CAN Module                                 */
            ZC7xxx_CAN_IoCtl(dev_id, IO_ZC7xxx_CAN_START, 0u);
                                                                /* ------------- RX FIFO OVERFLOW DETECTED ------------ */
        } else if (DEF_BIT_IS_SET(p_reg->ISR, ZC7xxx_CAN_ISR_RXOFLW) == DEF_YES) {
            DEF_BIT_SET(p_reg->ICR, ZC7xxx_CAN_ICR_CRXOFLW);    /* Clear Rx Overflow Flag. CAN Set to Overrun.          */

                                                                /* ---------- RX FIFO WATERMARK FULL DETECTED --------- */
        } else if (DEF_BIT_IS_SET(p_reg->ISR, ZC7xxx_CAN_ISR_RXFWMFLL) == DEF_YES) {
            DEF_BIT_SET(p_reg->ICR, ZC7xxx_CAN_ICR_CRXFWMFLL);  /* Clear Rx FIFO Watermark Full Flag.                   */

                                                                /* ------------------ SLEEP DETECTED ------------------ */
        } else if (DEF_BIT_IS_SET(p_reg->ISR, ZC7xxx_CAN_ISR_SLP) == DEF_YES) {
            DEF_BIT_SET(p_reg->ICR, ZC7xxx_CAN_ICR_CSLP);       /* Clear Sleep Interrupt Bit. (CAN in Sleep Mode)       */

                                                                /* ----------------- WAKE UP DETECTED ----------------- */
        } else if (DEF_BIT_IS_SET(p_reg->ISR, ZC7xxx_CAN_ISR_WKUP) == DEF_YES) {
            DEF_BIT_SET(p_reg->ICR, ZC7xxx_CAN_ICR_CWKUP);      /* Clear Wake Up Interrupt Bit.                         */

                                                                /* ----------- RX FIFO UNDERFLOW DETECTED ------------- */
        } else if (DEF_BIT_IS_SET(p_reg->ISR, ZC7xxx_CAN_ISR_RXUFLW) == DEF_YES) {
            DEF_BIT_SET(p_reg->ICR, ZC7xxx_CAN_ICR_CRXUFLW);    /* Clear Rx FIFO Underflow Flag.                        */

                                                                /* ------------- ARBITRATION LOST DETECTED ------------ */
        } else if (DEF_BIT_IS_SET(p_reg->ISR, ZC7xxx_CAN_ISR_ARBLST) == DEF_YES) {
            DEF_BIT_SET(p_reg->ICR, ZC7xxx_CAN_ICR_CARBLST);    /* Clear Arbitration Lost Flag.                         */

        } else {
            ;                                                   /* ----------------- UNKNOWN ERROR -------------------- */
        }           /* $$$ Possible ISR Errors: TXFEMP, TXFWMEMP, RXFWMFLL, RXNEMP, TXBFLL, TXFLL. $$$ */
    }
}
