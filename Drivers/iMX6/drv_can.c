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
*                                       Freescale iMX6 Series
*
* Filename : drv_can.c
* Version  : V2.42.01
*********************************************************************************************************
* Note(s)  : (1) This CAN Driver supports the following Series/Families:
*                    iMX6 Series - i.MX6 Quad
*                                - i.MX6 Dual
*                                - i.MX6 DualLite
*                                - i.MX6 Solo
*
*                Set by the Technical Reference Manual(s) obtained from the Freescale website. This driver
*                has been tested with or should work with the families mentioned above.
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

static  const  CPU_INT32U     iMX6_DrvId = 0x6F1B55E6u;         /* Unique Driver Identification Code                    */

static         iMX6_CAN_ERR   iMX6_DrvErr;                      /* Holds Detailed Error Code if Detected                */

static         iMX6_CAN_DATA  iMX6_DevData[iMX6_CAN_N_DEV];     /* Array Holds Driver Runtime Data                      */


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

static  CPU_BOOLEAN  iMX6_CAN_SetMode(CPU_INT32U     para_id,
                                      iMX6_CAN_MODE  mode);


/*
*********************************************************************************************************
*                                           iMX6_CAN_Init()
*
* Description : Initializes the CAN Driver with the given Device Name.
*
* Argument(s) : para_id     Device ID. [iMX6_CAN_BUS_1, iMX6_CAN_BUS_2].
*
* Return(s)   : Error code:  0 = No Error.
*                           -1 = Error Occurred.
*
* Caller(s)   : CanBusEnable().
*
* Note(s)     : (1) Once the CAN Controller has be 'Reset', the CAN Module automatically goes into
*                   'Freeze' Mode. To confirm that this has occurred, the Driver will be set once more,
*                   to 'Freeze' Mode following a 'Reset' at Initialization.
*
*               (2) Although the generation of the Tx & Rx Warning Counters are Enabled in the MCR
*                   Register, the Tx & Rx Warning Counter Interrupts have been disabled as there is no
*                   error checking or correction done for this Driver.  Therefore, for debugging purposes
*                   The values of the Tx & Rx Warning Counters are accessible but only by reading the
*                   specified register(s).
*
*               (3) The CTRL1 Timing Mask encompases the PRESDIV, RJW, PSEG1, PSEG2, and PROP_SEG options
*                   of the FLEXCANx_CTRL1 Register. This will "clear" any previous configuration prior to
*                   updating the CAN Module with the updated Baud Rate configuration without affecting
*                   other bits/options found in this register.
*
*               (4) For Interrupt Mask Register(s), since using the Rx FIFO Engine for Message Reception,
*                   then the Interrupt Flag Register(s) use Bits 5->7 for FIFO, and therefore the 6 Tx
*                   Mailboxes will start at Bit 8 and end up in Bit 13. Therefore all Bits up to, and
*                   including, Bit 13 will be ENABLED. For the Rx FIFO Engine, only Bit(s) 5, 6, 7 will be
*                   used for Rx Mailbox Checking with the following rule:
*                       Bit(s)  |   Rule
*                      ------------------
*                         7     | Rx FIFO Overflow
*                         6     | Rx FIFO Almost Full
*                         5     | Rx FIFO One Rx Frame Available
*********************************************************************************************************
*/

CPU_INT16S  iMX6_CAN_Init (CPU_INT32U  para_id)
{
    iMX6_CAN_REG        *p_reg;
    iMX6_CAN_MB_STRUCT  *p_mb;
    iMX6_CAN_BAUD        baud;
    CPU_INT08U           i;
    CPU_BOOLEAN          drv_err;
    CPU_INT16S           result;
    
    
    result = -1;                                                /* Initialize Variable(s)                               */

    switch (para_id) {                                          /* ------------ INITIALIZE CAN BUS SETTINGS ----------- */
#if  (CAN_MODULE_CHANNEL_1 == DEF_ENABLED)
        case iMX6_CAN_BUS_1:
             iMX6_DevData[iMX6_CAN_BUS_1].RegPtr = (iMX6_CAN_REG       *) iMX6_CAN1_ADDR;
             iMX6_DevData[iMX6_CAN_BUS_1].MB_Ptr = (iMX6_CAN_MB_STRUCT *)(iMX6_CAN1_ADDR + iMX6_CAN_MB_OFFSET);
             break;
#endif
             
#if  (CAN_MODULE_CHANNEL_2 == DEF_ENABLED)
        case iMX6_CAN_BUS_2:
             iMX6_DevData[iMX6_CAN_BUS_2].RegPtr = (iMX6_CAN_REG       *) iMX6_CAN2_ADDR;
             iMX6_DevData[iMX6_CAN_BUS_2].MB_Ptr = (iMX6_CAN_MB_STRUCT *)(iMX6_CAN2_ADDR + iMX6_CAN_MB_OFFSET);
             break;
#endif
                
        default:
             iMX6_DrvErr = iMX6_CAN_ERR_INIT;                   /* Return Error if CAN Device is out of Range.          */
             return (result);
    }
    
    iMX6_DevData[para_id].Use = DEF_NO;                         /* Set Proper Can Device to UNUSED Status               */
    p_reg = iMX6_DevData[para_id].RegPtr;                       /* Set Base Address for specified CAN Device.           */
    p_mb  = iMX6_DevData[para_id].MB_Ptr;                       /* Set Rx FIFO Structure Mailbox Base Address.          */
    
                                                                /* No Error with Mailboxes, Proceed with Init           */
    iMX6_DrvErr = iMX6_CAN_ERR_NONE;                            /* Reset Driver Error                                   */
    
                                                                /* ---------------- CONFIGURE CAN MODE ---------------- */
    drv_err = iMX6_CAN_PinSetting(para_id);                     /* Configure Pin & Clk Settings for CAN Device.         */
    if (drv_err == DEF_FAIL) {
        iMX6_DrvErr = iMX6_CAN_ERR_INIT;                        /* Pin & Clk Settings Timeout, Return Error             */
        return (result);
    }
                                                                /* Reset CAN Controller, & CAN Register(s).             */
    drv_err = iMX6_CAN_SetMode(para_id, iMX6_CAN_MODE_RESET);
    if (drv_err == DEF_FAIL) {
        iMX6_DrvErr = iMX6_CAN_ERR_INIT;                        /* CAN Reset Timeout Occurred, Return Error             */
        return (result);
    }
    
    drv_err = iMX6_CAN_SetMode(para_id, iMX6_CAN_MODE_FREEZE);  /* Enter Freeze Mode / Configuration Mode. See Note (1) */
    if (drv_err == DEF_FAIL) {
        iMX6_DrvErr = iMX6_CAN_ERR_INIT;                        /* CAN Configuration Mode Timeout, Return Error         */
        return (result);
    }
                                                                /* -------------- CONFIGURE CAN SETTINGS -------------- */
                                                                /* Clr Default MAX MB Buffer Field Value.               */
    DEF_BIT_FIELD_WR(p_reg->MCR, 0u, iMX6_CAN_MCR_MAXMB(0xFFu));
    
    p_reg->MCR   |=  (iMX6_CAN_MCR_RFEN        |                /* Enable Rx FIFO Feature. MBs 0 -> 5 used for FIFO.    */
                      iMX6_CAN_MCR_SUPV        |                /* Set CAN Registers to Supervisor Mode (Default).      */
                      iMX6_CAN_MCR_WRN_EN      |                /* En Generation TWRN_INT & RWRN_INT bit, Note (2).     */
                      iMX6_CAN_MCR_SRX_DIS     |                /* Disable Self Reception in CAN.                       */
                      iMX6_CAN_MCR_IRMQ        |                /* Enable Individual Rx Masking & Queue Feature.        */
                      iMX6_CAN_MCR_AEN         |                /* Enable Tx Abort for Safe Abort of pending Tx Msgs.   */
                      iMX6_CAN_MCR_IDAM_FMT_A  |                /* Set Filter ID Format to Format A.                    */
                      iMX6_CAN_MCR_MAXMB(iMX6_CAN_MB_TOTAL));   /* Update MAXMB with Number of Rx/Tx Buffers to Config. */
                                                                
    p_reg->CTRL1 &= ~(iMX6_CAN_CTRL1_BOFF_MSK  |                /* Disable Bus Off Interrupt. No Check or Correction.   */
                      iMX6_CAN_CTRL1_LPB       |                /* Disable Loop Back Operation Mode. Only Normal Mode.  */
                      iMX6_CAN_CTRL1_TWRN_MSK  |                /* Disable Tx Warning Interrupt. See Note (2).          */
                      iMX6_CAN_CTRL1_RWRN_MSK  |                /* Disable Rx Warning Interrupt. See Note (2).          */
                      iMX6_CAN_CTRL1_LOM);                      /* Listen Only Mode is Deactivated.                     */
    
    p_reg->CTRL1 |=   iMX6_CAN_CTRL1_LBUF;                      /* Lowest Mailbox Number buff is Tx'd First. Per Specs. */
    
                                                                /* ---------------- CONFIGURE BAUD RATE --------------- */
    baud.BaudRate         = CAN_DEFAULT_BAUDRATE;               /* Set Default Baud Rate Settings                       */
    baud.SamplePoint      = CAN_DEFAULT_SP;
    baud.ReSynchJumpWidth = CAN_DEFAULT_RJW;

    drv_err = iMX6_CAN_CalcTimingReg(&baud);                    /* Calculate Bit Timing Register Values                 */
    if (drv_err == DEF_FAIL) {
        iMX6_DrvErr = iMX6_CAN_ERR_INIT;                        /* Bit Timing Error, Return Error                       */
        return (result);
    }
                                                                /* Clear Previous Timing Options. See Note (3).         */
    DEF_BIT_FIELD_WR(p_reg->CTRL1, 0u, iMX6_CAN_CTRL1_TIMING_MASK);
    
                                                                /*  - UPDATE BAUD CONFIG -                              */
    p_reg->CTRL1 |= (iMX6_CAN_CTRL1_PRESDIV(baud.PrescalerDiv)| /* Set Bit Timing Baud Rate Prescaler.                  */
                     iMX6_CAN_CTRL1_RJW(baud.SJW)             | /* Set ReSync Jump Width.                               */
                     iMX6_CAN_CTRL1_PSEG1(baud.PhaseBufSeg1)  | /* Set Time Phase Buffer Segment 1 & 2.                 */
                     iMX6_CAN_CTRL1_PSEG2(baud.PhaseBufSeg2)  | /* Set Propagation Segment in Bit Time below.           */
                     iMX6_CAN_CTRL1_PROP_SEG(baud.PropagationSeg));

                                                                /* ---------------- INTERRUPT SETTINGS ---------------- */
#if ((CANBUS_RX_HANDLER_EN > 0u) || \
     (CANBUS_TX_HANDLER_EN > 0u) || \
     (CANBUS_NS_HANDLER_EN > 0u))
    iMX6_CAN_IntSetting(para_id);                               /* Configure Interrupt Settings                         */
#endif
    
    p_reg->ESR1   = 0u;                                         /* Disable / Clear all Interrupt Registers, including.. */
    p_reg->IMASK2 = 0u;                                         /* ... Error Conditions and Interrupts found in ESR1.   */
    p_reg->IMASK1 = 0u;
    p_reg->IFLAG2 = 0u;
    p_reg->IFLAG1 = 0u;
                                                                /* -------------- Rx FIFO MAILBOX CONFIG -------------- */
    p_reg->RX14MASK = 0u;                                       /* Set Global Mask Filters to "Don't Care"              */
    p_reg->RX15MASK = 0u;
    p_reg->RXFGMASK = 0u;
    p_reg->RXMGMASK = 0u;
    
    p_reg->CTRL2 |= (iMX6_CAN_CTRL2_RFEN(0u) |                  /* Define Rx FIFO Filters available Based on Tbl 27-61. */
                     iMX6_CAN_CTRL2_TASD(0u));

    p_reg->CTRL2 &= ~iMX6_CAN_CTRL2_MRP;                        /* Rx Matching Starts from Rx FIFO & Continues on MB's. */
 
    for (i = 0u; i < iMX6_CAN_RX_FIFO_ID_FILTER_MIN_LIMIT; i++) {
        p_mb->ID_FILTER[i] = 0u;                                /* Clear Rx FIFO ID Filter Table Element(s).            */
    }
    
                                                                /* ----------------- Tx MAILBOX CONFIG ---------------- */
    for (i = 0u; i < iMX6_CAN_TX_MB_LIMIT; i++) {               /* Clear Tx Mailboxes, set 'Inactive' Code.             */
        p_mb->TX[i].ID    = 0u;
        p_mb->TX[i].DATA1 = 0u;
        p_mb->TX[i].DATA2 = 0u;
        p_mb->TX[i].DLC   = iMX6_CAN_Tx_PUT_CODE(iMX6_CAN_TX_CODE_INACTIVE);
    }
                                                                /* ----------------- ENABLE INTERRUPTS ---------------- */
    p_reg->IMASK1 = iMX6_CAN_RX_TX_ISR_TOT_BITs;                /* Enable Buffer Bit(s) 0 -> 13. See Note (4).          */
    p_reg->IFLAG1 = 0u;                                         /* Clear any Pending Active Interrupts.                 */
    
                                                                /* --------- CAN OPERATION MODE CONFIGURATION --------- */
    drv_err = iMX6_CAN_SetMode(para_id, iMX6_CAN_MODE_NORMAL);  /* Set CAN Module to Normal Operation Mode.             */
    if (drv_err == DEF_FAIL) {
        iMX6_DrvErr = iMX6_CAN_ERR_INIT;                        /* CAN Configuration Mode Timeout, Return Error         */
        return (result);
    }
    
    result = iMX6_CAN_ERR_NONE;                                 /* Set Function Result: No Error                        */
    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                           iMX6_CAN_Open()
*
* Description : Unlocks the CAN device, i.e. Read/Write/IoCtl Functions will take effect.
*
* Argument(s) : dev_id      Bus Node Name, used to interface with the CAN Bus Layer.
*
*               dev_name    Driver Device Name, used to interface with the Low-Level Device Driver.
*                           Possible Values for Device Name:
*                                   iMX6_CAN_BUS_1     [CAN Bus 1]
*                                   iMX6_CAN_BUS_2     [CAN Bus 2]
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

CPU_INT16S  iMX6_CAN_Open (CPU_INT16S  dev_id,
                           CPU_INT32U  dev_name,
                           CPU_INT16U  mode)
{
    CPU_INT16S  result;
    CPU_SR_ALLOC();

    
    result = -1;                                                /* Initializing Variable(s)                             */

    if (dev_name >= iMX6_CAN_N_DEV) {                           /* Check if Device Name is out of Range                 */
        iMX6_DrvErr = iMX6_CAN_ERR_BUS;
        return (result);
    }   

    if (mode != DEV_RW) {                                       /* Check if Mode is not Supported                       */
        iMX6_DrvErr = iMX6_CAN_ERR_MODE;
        return (result);
    }

    CPU_CRITICAL_ENTER();

    if (iMX6_DevData[dev_name].Use == DEF_NO) {                 /* Check if CAN Device is Unused                        */
        iMX6_DevData[dev_name].Use = DEF_YES;                   /* Mark CAN Device as Used                              */

#if ((CANBUS_RX_HANDLER_EN > 0u) || \
     (CANBUS_TX_HANDLER_EN > 0u) || \
     (CANBUS_NS_HANDLER_EN > 0u))
        iMX6_CAN_SetDevIds((CPU_INT08U)dev_id,                  /* Set Device IDs for the ISRs                          */
                           (CPU_INT08U)dev_name);
#else
        (void)&dev_id;                                          /* Prevent Compiler Warning                             */
#endif
        
        result = (CPU_INT16S)dev_name;                          /* OK, Device is Opened                                 */
    } else {
        iMX6_DrvErr = iMX6_CAN_ERR_OPEN;
    }
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                          iMX6_CAN_Close()
*
* Description : Locks the CAN device, i.e. Read/Write/IoCtl Functions will not take effect.
*
* Argument(s) : para_id     Parameter Identifier, returned by iMX6_CAN_Open().
*
* Return(s)   : Error code:  0 = No Error.
*                           -1 = Error Occurred.
*
* Caller(s)   : CanCfg in 'can_cfg.c'
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT16S  iMX6_CAN_Close (CPU_INT16S  para_id)
{
    CPU_INT16S  result;
    CPU_SR_ALLOC();

    
    result = -1;                                                /* Initializing Variable(s)                             */

    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= (CPU_INT08S)iMX6_CAN_N_DEV)) {
        iMX6_DrvErr = iMX6_CAN_ERR_BUS;
        return (result);
    }

    CPU_CRITICAL_ENTER();
    
    if (iMX6_DevData[para_id].Use != DEF_NO) {                  /* Check if CAN Device is Used                          */
        iMX6_DevData[para_id].Use = DEF_NO;                     /* Mark CAN Device as Unused                            */
        result = iMX6_CAN_ERR_NONE;                             /* OK, Device is Closed                                 */
    } else {
        iMX6_DrvErr = iMX6_CAN_ERR_CLOSE;
    }
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                          iMX6_CAN_IoCtl()
*
* Description : Performs Special Action on the Opened Device. The Function Code 'func' defines 
*               what the caller wants to do. Description of Function Codes are defined in the header.
*
* Argument(s) : para_id     Parameter Identifier, returned by iMX6_CAN_Open().
*
*               func        Function Code.
*
*               p_arg       Argument List, Specific to the Function Code. Possible Function Codes are:
*                                       IO_iMX6_CAN_GET_IDENT
*                                       IO_iMX6_CAN_GET_ERRNO
*                                       IO_iMX6_CAN_GET_DRVNAME
*                                       IO_iMX6_CAN_SET_BAUDRATE
*                                       IO_iMX6_CAN_START
*                                       IO_iMX6_CAN_CONFIG
*                                       IO_iMX6_CAN_RX_STANDARD
*                                       IO_iMX6_CAN_RX_EXTENDED
*                                       IO_iMX6_CAN_TX_READY
*                                       IO_iMX6_CAN_GET_NODE_STATUS
*                                       IO_iMX6_CAN_SET_RX_FILTER
*                                       IO_iMX6_CAN_IO_FUNC_N
*
* Return(s)   : Error code:  0 = No Error.
*                           -1 = Error Occurred.
*
* Caller(s)   : CanCfg in 'can_cfg.c'
*
* Note(s)     : (1) Rx Filtration will be through the Rx FIFO Structure ID Filter Table Element buffer
*                   list. Any ID Filter Table Element greater than the maximum quantity of ID Filters
*                   available will result in an error and return with no update of the filter value.
*
*               (2) The CTRL1 Timing Mask encompases the PRESDIV, RJW, PSEG1, PSEG2, and PROP_SEG options
*                   of the FLEXCANx_CTRL1 Register. This will "clear" any previous configuration prior to
*                   updating the CAN Module with the updated Baud Rate configuration without affecting
*                   other bits/options found in this register.
*********************************************************************************************************
*/

CPU_INT16S  iMX6_CAN_IoCtl (CPU_INT16S   para_id,
                            CPU_INT16U   func,
                            void        *p_arg)
{
    iMX6_CAN_REG        *p_reg;
    iMX6_CAN_MB_STRUCT  *p_mb;
    iMX6_CAN_BAUD        baud;
    CPU_INT08U           tx_i;
    CPU_INT08U           tx_code;
    CPU_BOOLEAN          tx_ok;
    CPU_INT08U           id_element_nbr;
    CPU_INT32U           id_filter;
    CPU_BOOLEAN          can_err;
    CPU_INT16S           result;
    CPU_SR_ALLOC();


    result  = -1;                                               /* Initialize Variable(s)                               */
    can_err = DEF_OK;
    
    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= (CPU_INT08S)iMX6_CAN_N_DEV)) {
        iMX6_DrvErr = iMX6_CAN_ERR_BUS;
        return (result);
    }

    if (iMX6_DevData[para_id].Use != DEF_YES) {                 /* Check if CAN Device is Opened                        */
        iMX6_DrvErr = iMX6_CAN_ERR_OPEN;
        return (result);
    }

    p_reg = iMX6_DevData[para_id].RegPtr;                       /* Set Base Address for CAN Device(s)                   */
    p_mb  = iMX6_DevData[para_id].MB_Ptr;                       /* Set Rx FIFO Structure Mailbox Base Address.          */
    
    CPU_CRITICAL_ENTER();

    switch (func) {                                             /*                SELECT: FUNCTION CODE                 */
        case IO_iMX6_CAN_GET_IDENT:                             /* -------------------- GET IDENT --------------------- */
             *(CPU_INT32U*)p_arg = iMX6_DrvId;                  /* Return Driver Identification Code                    */
             result = iMX6_CAN_ERR_NONE;                        /* Indicate Successful Function Execution               */
             break;


        case IO_iMX6_CAN_GET_ERRNO:                             /* ------------------ GET ERRORCODE ------------------- */
             *(CPU_INT16U*)p_arg = iMX6_DrvErr;                 /* Return Last Detected Error Code                      */
             result = iMX6_CAN_ERR_NONE;                        /* Indicate Successful Function Execution               */
             break;


        case IO_iMX6_CAN_GET_DRVNAME:                           /* ----------------- GET DRIVER NAME ------------------ */
                                                                /* Return Human Readable Driver Name                    */
             *(CPU_INT08U**)p_arg = (CPU_INT08U*)iMX6_CAN_NAME;
             result = iMX6_CAN_ERR_NONE;                        /* Indicate Successful Function Execution               */
             break;


        case IO_iMX6_CAN_SET_BAUDRATE:                          /* ------------------ SET BAUD RATE ------------------- */
             baud.BaudRate         = *((CPU_INT32U *)p_arg);
             baud.SamplePoint      = CAN_DEFAULT_SP;
             baud.ReSynchJumpWidth = CAN_DEFAULT_RJW;

             can_err = iMX6_CAN_CalcTimingReg(&baud);           /* Calculate Bit Timing Register Values                 */
             if (can_err == DEF_FAIL) {
                 iMX6_DrvErr = iMX6_CAN_ERR_FUNC;               /* Bit Timing Error, Return Error                       */
                 break;
             }
                                                                /* Enter CAN Freeze / Config Mode.                      */
             can_err = iMX6_CAN_SetMode(para_id, iMX6_CAN_MODE_FREEZE);
             if (can_err == DEF_FAIL) {
                 iMX6_DrvErr = iMX6_CAN_ERR_FUNC;               /* CAN Freeze Mode Timeout, Return Error                */
                 break;
             }
             
                                                                /* Clear Previous Timing Options. See Note (2).         */
             DEF_BIT_FIELD_WR(p_reg->CTRL1, 0u, iMX6_CAN_CTRL1_TIMING_MASK);
             
                                                                /* Update Baud Rate Settings with new Configuration.    */
             p_reg->CTRL1|= (iMX6_CAN_CTRL1_PRESDIV(baud.PrescalerDiv) |
                             iMX6_CAN_CTRL1_RJW(baud.SJW)              |
                             iMX6_CAN_CTRL1_PSEG1(baud.PhaseBufSeg1)   |
                             iMX6_CAN_CTRL1_PSEG2(baud.PhaseBufSeg2)   |
                             iMX6_CAN_CTRL1_PROP_SEG(baud.PropagationSeg));

                                                                /* Revert to CAN Normal Mode.                           */
             can_err = iMX6_CAN_SetMode(para_id, iMX6_CAN_MODE_NORMAL);
             break;


        case IO_iMX6_CAN_START:                                 /* ------------- START CAN COMMUNICATION -------------- */
                                                                /* Set CAN to Normal Mode. Start CAN communication.     */
             can_err = iMX6_CAN_SetMode(para_id, iMX6_CAN_MODE_NORMAL);
             break;


        case IO_iMX6_CAN_CONFIG:                                /* ----------------- SET CAN TO CONFIG ---------------- */
                                                                /* Enter CAN Freeze / Config Operating Mode             */
             can_err = iMX6_CAN_SetMode(para_id, iMX6_CAN_MODE_FREEZE);
             break;


        case IO_iMX6_CAN_TX_READY:                              /* -------------------- TX READY ---------------------- */
             tx_i  = 0u;                                        /* Init Var(s).                                         */
             tx_ok = DEF_NO;
             
             while (tx_i < iMX6_CAN_TX_MB_LIMIT) {              /* Check if Tx Buffer has an Open Tx Mailbox.           */
                 tx_code = iMX6_CAN_Tx_GET_CODE(p_mb->TX[tx_i].DLC);
                 if (tx_code == iMX6_CAN_TX_CODE_INACTIVE) {
                     tx_ok = DEF_YES;                           /* Mailbox [i] is Ready to Transmit.                    */
                     break;
                 }
                 
                 tx_i++;                                        /* Increment Tx MB check counter.                       */
             }
             
             if (tx_ok == DEF_YES) {
                 *((CPU_INT08U *)p_arg) = 1u;                   /* At least 1 Tx Mailbox is Inactive, Ok to Transmit.   */
             } else {
                 *((CPU_INT08U *)p_arg) = 0u;                   /* NO Tx Mailbox is Inactive, Not OK to Transmit.       */
             }
             
             result = iMX6_CAN_ERR_NONE;                        /* Indicate Successful Function Execution               */
             break;


        case IO_iMX6_CAN_GET_NODE_STATUS:                       /* ---------------- GET NODE STATUS ------------------- */
             *((CPU_INT08U *)p_arg) = 0u;                       /* Set the Initial Argument Ptr to Zero. No Node Check. */

             result = iMX6_CAN_ERR_NONE;                        /* Indicate Successful Function Execution               */
             break;
            

        case IO_iMX6_CAN_SET_RX_FILTER:                         /* ---------------- SET RX FILTER --------------------- */
             id_element_nbr = ((CPU_INT08U *)p_arg)[0u];        /* Get Rx FIFO ID Filter Table Element Number Desired.  */
             id_filter      = ((CPU_INT32U *)p_arg)[1u];        /* Get ID Filter Desired for Table Element Number.      */

             if (id_element_nbr >= iMX6_CAN_RX_FIFO_ID_FILTER_MIN_LIMIT) {
                 can_err     = DEF_FAIL;
                 iMX6_DrvErr = iMX6_CAN_ERR_ARG;                /* Error Checking: Incorrect ID Filter Table Nbr.       */
                 break;
             }
             
             p_mb->ID_FILTER[id_element_nbr] = id_filter;       /* Set ID Table Element with desired ID Filter.         */
             break;


        case IO_iMX6_CAN_IO_FUNC_N:
                                                                /* Set the Size of IO Function Number for return.       */
            *((CPU_INT08U *)p_arg) = IO_iMX6_CAN_IO_FUNC_N + 1u;

             result = iMX6_CAN_ERR_NONE;                        /* Indicate Successful Function Execution               */
             break;

             
        case IO_iMX6_CAN_RX_STANDARD:                           /* --------------- SET RX STANDARD -------------------- */
        case IO_iMX6_CAN_RX_EXTENDED:                           /* --------------- SET RX EXTENDED -------------------- */
                /* 
                    Rx FIFO is being used. This does not apply. Rx FIFO Region for Setting Rx Standard/Extended bit,
                    (AKA IDE Bit) is Read-Only.
                */
        default:                                                /* -------------- UNKNOWN FUNCTION CODE --------------- */
             break;
    }

    if (can_err == DEF_FAIL) {
        result      = iMX6_CAN_ERR_FUNC;                        /* Error occurred in function, Return with Error.       */
        iMX6_DrvErr = iMX6_CAN_ERR_FUNC;
    } else {
        result = iMX6_CAN_ERR_NONE;                             /* Indicate Successful Function Execution               */
    }

    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                           iMX6_CAN_Read()
*
* Description : Read a received CAN Frame from a Message Buffer. The Buffer must have space for only
*               one CAN Frame.
*
* Argument(s) : para_id     Parameter Identifier, returned by iMX6_CAN_Open().
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
* Note(s)     : (1) A 'Middle Man' approach is used when reading the Rx FIFO Mailbox Registers to ensure
*                   that data integrity is preserved when reading multiple parts of the ID & DLC Registers.
*                   The following format will then be used to split the Data Bytes from the Data Registers
*                   and be placed in the Frame's Data buffer based on the qty of data bytes in the CAN
*                   message (DLC value). Following this format:
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
*
*               (3) According to the iMX6 Reference Manual for the CAN Recieve Process (Section 27.6.4),
*                   for Rx FIFO Mailboxes, it is recommended to clear the Rx FIFO Interrupt - IFLAG1[BIT 5I]
*                   after reading the DLC, ID, DATA1, & DATA2 Registers of the Rx FIFO Structure as clearing
*                   the interrupt bit releases the MB and allows the CPU to read the next Rx FIFO entry.
*                   Thus, it will be cleared after reading the Rx FIFO Message in the BSP ISR Handler.
*********************************************************************************************************
*/

CPU_INT16S  iMX6_CAN_Read (CPU_INT16S   para_id,
                           CPU_INT08U  *buf,
                           CPU_INT16U   size)
{
    iMX6_CAN_REG        *p_reg;
    iMX6_CAN_MB_STRUCT  *p_mb;
    iMX6_CAN_FRM        *p_frm;
    iMX6_CAN_REG_FRM     rx_fifo;
    CPU_INT08U           cnt;
    CPU_INT16S           result;
    CPU_SR_ALLOC();
    
    
    result = -1;                                                /* Initializing Variable(s)                             */
                                                        
    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= (CPU_INT08S)iMX6_CAN_N_DEV)) {
        iMX6_DrvErr = iMX6_CAN_ERR_BUS;
        return (result);
    }
    
    if (size != sizeof(iMX6_CAN_FRM)) {                         /* Check if Size is Plausible                           */
        iMX6_DrvErr = iMX6_CAN_ERR_NO_DATA;
        return (result);
    }
    
    if (buf == (void *)0) {                                     /* Check if Buffer Pointer is Invalid                   */
        iMX6_DrvErr = iMX6_CAN_ERR_ARG;
        return (result);
    }
    
    if (iMX6_DevData[para_id].Use != DEF_YES) {                 /* Check if CAN Device is Opened                        */
        iMX6_DrvErr = iMX6_CAN_ERR_OPEN;
        return (result);
    }
    
    p_reg = iMX6_DevData[para_id].RegPtr;                       /* Set Base Address for CAN Device(s)                   */
    p_mb  = iMX6_DevData[para_id].MB_Ptr;                       /* Set Rx FIFO Structure Mailbox Base Address.          */
    p_frm = (iMX6_CAN_FRM *)buf;
     
    CPU_CRITICAL_ENTER();
                                                                /* ---------------- READ Rx'D FIFO MSG ---------------- */
    if (DEF_BIT_IS_SET(p_reg->IFLAG1, iMX6_CAN_IFLAG1_RX_FIFO_RD_RDY) == DEF_YES) {

        rx_fifo.ID    = p_mb->RX_ID;                            /* Read New Message. See Note (1).                      */
        rx_fifo.Data1 = p_mb->RX_DATA1;
        rx_fifo.Data2 = p_mb->RX_DATA2;
        rx_fifo.DLC   = p_mb->RX_DLC;
                                                                /* ---------------- Rx FIFO ID REGISTER --------------- */
                                                                /*  - EXTENDED FRAME ID -                               */
        if (DEF_BIT_IS_SET(rx_fifo.DLC, iMX6_CAN_MB_DLC_IDE) == DEF_YES) {

                                                                /* Copy Both Standard & Extended IDs. See Note (2).     */
            p_frm->Identifier = iMX6_CAN_Rx_ID_BOTH(rx_fifo.ID);

                                                                /* Set Extended ID Flag on Rx Frame.                    */
            DEF_BIT_SET(p_frm->Identifier, iMX6_CAN_FRM_IDE_FLAG);

                                                                /* Check for Remote Frame Flag on Extended Frames.      */
            if (DEF_BIT_IS_SET(rx_fifo.DLC, iMX6_CAN_MB_DLC_RTR) == DEF_YES) {
                DEF_BIT_SET(p_frm->Identifier, iMX6_CAN_FRM_RTR_FLAG);
            }

        } else {                                                /*  - STANDARD FRAME ID -                               */
                                                                /* Copy only Standard ID.                               */
            p_frm->Identifier = iMX6_CAN_Rx_ID_STD(rx_fifo.ID);

                                                                /* Check for Remote Frame Flag on Standard Frames.      */
            if (DEF_BIT_IS_SET(rx_fifo.ID, iMX6_CAN_MB_DLC_SRR) == DEF_YES) {
                DEF_BIT_SET(p_frm->Identifier, iMX6_CAN_FRM_RTR_FLAG);
            }
        }
                                                                /* --------------- Rx FIFO DLC REGISTER --------------- */
        p_frm->DLC = iMX6_CAN_Rx_DLC_DLC(rx_fifo.DLC);          /* Read and Store DLC Amount onto Frame.                */

                                                                /* ---------- Rx FIFO DATA1 & DATA2 REGISTERs --------- */
        for (cnt = 0u; cnt < p_frm->DLC; cnt++) {               /* Read Data Bytes based on DLC Value.                  */
            if (cnt < iMX6_CAN_DLC_DATA_SPLIT) {                /* MACRO splits Data based on cnt value See Note (1).   */
                p_frm->Data[cnt] = iMX6_CAN_Rx_DATA_BYTES(rx_fifo.Data1, cnt);
            } else {
                p_frm->Data[cnt] = iMX6_CAN_Rx_DATA_BYTES(rx_fifo.Data2, (cnt - iMX6_CAN_DLC_DATA_SPLIT));
            }
        }

        result = size;

    } else {                                                    /* No Message in Rx FIFO Mailbox Array.                 */
        iMX6_DrvErr = iMX6_CAN_ERR_NO_DATA;
    }

    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                          iMX6_CAN_Write()
*
* Description : Write a CAN Frame to a Message Buffer. The Buffer must contain only one CAN Frame
*               which will be written to a predefined Message Buffer.
*
* Argument(s) : para_id     Parameter Identifier, returned by iMX6_CAN_Open().
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
* Note(s)     : (1) A 'Middle Man' approach is used when writing to the Mailboxes to ensure that data
*                   integrity is preserved when writing to multiple parts of the ID & DLC Registers.
*                   The following format will then be used to split the Data Bytes from the Frame's Data
*                   buffers and be placed in the Mailbox Data Registers based on the qty of data bytes
*                   in the CAN message (DLC value). Following this format:
*                           p_frm->Data[8u]   : { DB0 , DB1 , DB2 , DB3 , DB4 , DB5 , DB6 , DB7}
*                             Bit(s)          : 31                          0
*                                  Data 1 Reg : [  DB0  | DB1  | DB2  | DB3  ]
*                                  Data 2 Reg : [  DB4  | DB5  | DB6  | DB7  ]
*
*               (2) Based on the Transmit Process of the iMX6 Reference Manual (Section 27.6.2) Once the
*                   Control & Status Word (AKA DLC) Register is written, with the DLC, the Mailbox will
*                   then activate and will participate into the Arbitration Process. Thus, the DLC Reg.
*                   will be written last.
*********************************************************************************************************
*/

CPU_INT16S  iMX6_CAN_Write (CPU_INT16S   para_id,
                            CPU_INT08U  *buf,
                            CPU_INT16U   size)
{
    iMX6_CAN_MB_STRUCT  *p_mb;
    iMX6_CAN_FRM        *p_frm;
    iMX6_CAN_REG_FRM     tx_fifo;
    CPU_INT08U           cnt;
    CPU_INT08U           tx_mb;
    CPU_BOOLEAN          tx_found;
    CPU_INT16S           result;
    CPU_SR_ALLOC();

    
    result        = -1;                                         /* Initializing Variable(s)                             */
    cnt           =  0u;
    tx_mb         =  0u;
    tx_found      =  DEF_NO;
    
    tx_fifo.ID    =  0u;                                        /* Pre-Clr 'Middle-Man' Registers. See Note (1).        */
    tx_fifo.DLC   =  0u;
    tx_fifo.Data1 =  0u;
    tx_fifo.Data2 =  0u;

    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= (CPU_INT08S)iMX6_CAN_N_DEV)) {
        iMX6_DrvErr = iMX6_CAN_ERR_BUS;
        return (result);
    }
    
    if (size != sizeof(iMX6_CAN_FRM)) {                         /* Check if Size is Plausible                           */
        iMX6_DrvErr = iMX6_CAN_ERR_NO_DATA;
        return (result);
    }
    
    if (buf == (void *)0) {                                     /* Check if Buffer Pointer is Invalid                   */
        iMX6_DrvErr = iMX6_CAN_ERR_ARG;
        return (result);
    }
    
    if (iMX6_DevData[para_id].Use != DEF_YES) {                 /* Check if CAN Device is Opened                        */
        iMX6_DrvErr = iMX6_CAN_ERR_OPEN;
        return (result);      
    }

    p_mb  =  iMX6_DevData[para_id].MB_Ptr;                      /* Set Rx FIFO Structure Mailbox Base Address.          */
    p_frm = (iMX6_CAN_FRM *)buf;

    CPU_CRITICAL_ENTER();
    
    for (tx_mb = 0u; tx_mb < iMX6_CAN_TX_MB_LIMIT; tx_mb++) {   /* Check for the first INACTIVE Tx MB. See Note (2).    */
        if (iMX6_CAN_Tx_GET_CODE(p_mb->TX[tx_mb].DLC) == iMX6_CAN_TX_CODE_INACTIVE) {
            tx_found = DEF_YES;                                 /* A TX MB is Rdy to Transmit.                          */
            break;
        }
    }
                                                                /* ---------------- WRITE Tx'D FIFO MSG --------------- */
    if (tx_found == DEF_YES) {                                  /* If Tx MB is OK/Ready to Transmit. Code = INACTIVE.   */
        
                                                                /* ----------------- Tx MB ID REGISTER ---------------- */
        if ((DEF_BIT_IS_SET(p_frm->Identifier, iMX6_CAN_FRM_IDE_FLAG) == DEF_YES) ||
            (p_frm->Identifier > iMX6_CAN_SID_MASK)) {          /*  - EXTENDED FRAME ID -                               */
                                                                /* Set Extended ID in Tx MB ID Register.                */
            tx_fifo.ID  = iMX6_CAN_Tx_ID_EXT(p_frm->Identifier);
                                                                /* Set Standard ID as well from CAN Frame.              */
            tx_fifo.ID |= iMX6_CAN_Tx_ID_STD(p_frm->Identifier >> 18u);

            DEF_BIT_SET(tx_fifo.ID, iMX6_CAN_MB_DLC_IDE);       /* Set Extended Identifier Flag.                        */

                                                                /* If Frame RTR Flag set, Set Extended Frame RTR Flag.  */
            if (DEF_BIT_IS_SET(p_frm->Identifier, iMX6_CAN_FRM_RTR_FLAG) == DEF_YES) {
                DEF_BIT_SET(tx_fifo.ID, iMX6_CAN_MB_DLC_RTR);
            }
        } else {                                                /*  - STANDARD FRAME ID -                               */
                                                                /* Clears Tx MB (and Extended ID Flag & Msg ID).        */
            tx_fifo.ID = iMX6_CAN_Tx_ID_STD(p_frm->Identifier);

                                                                /* If Frame RTR Flag set, Set Standard Frame RTR Flag.  */
            if (DEF_BIT_IS_SET(p_frm->Identifier, iMX6_CAN_FRM_RTR_FLAG) == DEF_YES) {
                DEF_BIT_SET(tx_fifo.ID, iMX6_CAN_MB_DLC_SRR);
            }
        }
                                                                /* ----------- Tx MB DATA1 & DATA2 REGISTERs ---------- */
        for (cnt = 0u; cnt < p_frm->DLC; cnt++) {               /* Write Data Bytes based on DLC Qty.                   */
            if (cnt < iMX6_CAN_DLC_DATA_SPLIT) {                /* MACRO saves splits Data based on cnt value.          */
                tx_fifo.Data1 |= iMX6_CAN_Tx_DATA_BYTES(p_frm->Data[cnt], cnt);
            } else {
                tx_fifo.Data2 |= iMX6_CAN_Tx_DATA_BYTES(p_frm->Data[cnt], (cnt - iMX6_CAN_DLC_DATA_SPLIT));
            }
        }
                                                                /* ---------------- Tx MB DLC REGISTER ---------------- */
        tx_fifo.DLC = iMX6_CAN_Tx_DLC_DLC(p_frm->DLC);          /* Set Frame DLC in Tx MB DLC Register.                 */
        
        tx_fifo.DLC |= iMX6_CAN_Tx_PUT_CODE(iMX6_CAN_TX_CODE_DATA); /* Placed MB with 'DATA' code to see if it starts tx. */
        
        p_mb->TX[tx_mb].ID    = tx_fifo.ID;
        p_mb->TX[tx_mb].DATA1 = tx_fifo.Data1;
        p_mb->TX[tx_mb].DATA2 = tx_fifo.Data2;
        p_mb->TX[tx_mb].DLC   = tx_fifo.DLC;

        result = size;

    } else {                                                    /* Tx FIFO is Full                                      */
        iMX6_DrvErr = iMX6_CAN_ERR_BUSY;                        /* All Tx Mailboxes are Busy                            */
    }
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                         iMX6_CAN_SetMode()
*
* Description : Sets the CAN Mode.
*
* Argument(s) : para_id     Desired CAN Module ID.
*
*               mode        Desired Mode to set CAN.
*
* Return(s)   : none.
*
* Caller(s)   : iMX6_CAN_Init().
*
* Note(s)     : (1) This bit is self-clearing. If a '1' is written to this bit, the CAN Controller config
*                   register(s), including the SRR register, are Reset. The controller comes up in 'CONFIG'
*                   mode, thus once 'Reset' has been set, the polling for the 'Config' bit in the Status
*                   Register will be done.
*********************************************************************************************************
*/

static  CPU_BOOLEAN  iMX6_CAN_SetMode (CPU_INT32U     para_id,
                                       iMX6_CAN_MODE  mode)
{
    iMX6_CAN_REG  *p_reg;
    CPU_REG16      reg16;
    CPU_INT32U     i;
    CPU_BOOLEAN    can_err;
    
    
    p_reg = iMX6_DevData[para_id].RegPtr;                       /* Set Base Address for CAN Device(s)                   */
    i     = 0u;                                                 /* Initialize Variable(s)                               */
    
    switch (mode) {
        case iMX6_CAN_MODE_FREEZE:                              /* -------------------- FREEZE MODE ------------------- */
             DEF_BIT_SET(p_reg->MCR, iMX6_CAN_MCR_FRZ);         /* Set Bit to Enable to Enter Freeze Mode.              */
             DEF_BIT_SET(p_reg->MCR, iMX6_CAN_MCR_HALT);        /* Enters Freeze Mode, since FRZ bit has been asserted. */

             while (DEF_BIT_IS_CLR(p_reg->MCR, iMX6_CAN_MCR_FRZ_ACK) == DEF_YES) {
                 i++;                                           /* Wait for "Freeze ACK" Bit to be Set in Control Reg.  */

                 if (i > CAN_TIMEOUT_ERR_VAL) {                 /* Check for Timeout Error.                             */
                     return (DEF_FAIL);
                 }
             }
             break;


        case iMX6_CAN_MODE_RESET:                               /* -------------------- RESET MODE -------------------- */
             can_err = iMX6_CAN_BSP_Start(para_id);             /* Restart the Desired CAN Module                       */
             if (can_err != DEF_OK) {
                 iMX6_DrvErr = iMX6_CAN_ERR_FUNC;               /* CAN_BSP_START Timed Out, Return Error                */
                 return (DEF_FAIL);
             }
             
             DEF_BIT_SET(p_reg->MCR, iMX6_CAN_MCR_SOFT_RST);    /* Reset the CAN Registers. Soft Reset.                 */
             
             while (DEF_BIT_IS_SET(p_reg->MCR, iMX6_CAN_MCR_SOFT_RST) == DEF_YES) {
                 i++;                                           /* Wait for "Soft Reset" Bit is Cleared.                */

                 if (i > CAN_TIMEOUT_ERR_VAL) {                 /* Check for Timeout Error.                             */
                     return (DEF_FAIL);
                 }
             }
             break;


        case iMX6_CAN_MODE_NORMAL:                              /* -------------------- NORMAL MODE ------------------- */
             DEF_BIT_CLR(p_reg->MCR, iMX6_CAN_MCR_FRZ);         /* Exit Freeze Mode. Negate the Freeze Mode Enable.     */
             DEF_BIT_CLR(p_reg->MCR, iMX6_CAN_MCR_HALT);        /* Exit Freeze Mode. No Freeze Mode Request Set.        */
             
             DEF_BIT_CLR(p_reg->MCR, iMX6_CAN_MCR_MDIS);        /* Enable CAN Module.                                   */

             while (DEF_BIT_IS_SET(p_reg->MCR, iMX6_CAN_MCR_NOT_RDY) == DEF_YES) {
                 i++;                                           /* Wait for "Not Ready" Bit to be Clr in Control Reg.   */

                 if (i > CAN_TIMEOUT_ERR_VAL) {                 /* Check for Timeout Error.                             */
                     return (DEF_FAIL);
                 }
             }
             break;


        default:                                                /* --------------- UNKNOWN DEFAULT MODE --------------- */
             break;
    }
    
    return (DEF_OK);
}
