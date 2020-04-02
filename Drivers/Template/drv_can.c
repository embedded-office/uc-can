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
*                                              Template
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
                                                                /* Unique Driver Identification Code                    */
static  const  CPU_INT32U           <Drv_Name>_DrvId = 0x00000000u;

static         <Drv_Name>_CAN_ERR   <Drv_Name>_DrvErr;          /* Holds Detailed Error Code if Detected                */

                                                                /* Array Holds Driver Runtime Data                      */
static         <Drv_Name>_CAN_DATA  <Drv_Name>_DevData[<Drv_Name>_CAN_N_DEV];


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

static  CPU_BOOLEAN  <Drv_Name>_CAN_SetMode(CPU_INT32U           para_id,
                                            <Drv_Name>_CAN_MODE  mode);


/*
*********************************************************************************************************
*                                        <Drv_Name>_CAN_Init()
*
* Description : Initializes the CAN Driver with the given Device Name.
*
* Argument(s) : para_id     Device ID.
*
* Return(s)   : Error code:  0 = No Error.
*                           -1 = Error Occurred.
*
* Caller(s)   : CanBusEnable().
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT16S  <Drv_Name>_CAN_Init (CPU_INT32U  para_id)
{
    <Drv_Name>_CAN_REG   *p_reg;
    <Drv_Name>_CAN_BAUD   baud;
    CPU_BOOLEAN           drv_err;
    CPU_INT16S            result;
    
    
    result = -1;                                                /* Initialize Variable(s)                               */

    switch (para_id) {                                          /* ------------ INITIALIZE CAN BUS SETTINGS ----------- */
        case <Drv_Name>_CAN_BUS_0:
             <Drv_Name>_DevData[<Drv_Name>_CAN_BUS_0].RegPtr = (<Drv_Name>_CAN_REG *) /* $$$ - CAN 0 ADDR - $$$ */
             break;
             
        case <Drv_Name>_CAN_BUS_1:
             <Drv_Name>_DevData[<Drv_Name>_CAN_BUS_1].RegPtr = (<Drv_Name>_CAN_REG *) /* $$$ - CAN 1 ADDR - $$$ */
             break;
                
        default:
             <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_INIT;       /* Return Error if CAN Device is out of Range.          */
             return (result);
    }
    
    <Drv_Name>_DevData[para_id].Use = DEF_NO;                   /* Set Proper Can Device to UNUSED Status               */

    p_reg = <Drv_Name>_DevData[para_id].RegPtr;                 /* Set Base Address for specified CAN Device.           */
    
                                                                /* No Error with Mailboxes, Proceed with Init           */
    <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_NONE;                /* Reset Driver Error                                   */
    
                                                                /* ---------------- CONFIGURE CAN MODE ---------------- */
    drv_err = <Drv_Name>_CAN_PinSetting(para_id);               /* Configure Pin & Clk Settings for CAN Device.         */
    if (drv_err == DEF_FAIL) {
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_INIT;            /* Pin & Clk Settings Timeout, Return Error             */
        return (result);
    }
    
    /* $$$ - Enter Config Mode - $$$ */

                                                                /* ---------------- CONFIGURE BAUD RATE --------------- */
    baud.BaudRate         = CAN_DEFAULT_BAUDRATE;               /* Set Default Baud Rate Settings                       */
    baud.SamplePoint      = CAN_DEFAULT_SP;
    baud.ReSynchJumpWidth = CAN_DEFAULT_RJW;

    drv_err = <Drv_Name>_CAN_CalcTimingReg(&baud);              /* Calculate Bit Timing Register Values                 */
    if (drv_err == DEF_FAIL) {
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_INIT;            /* Bit Timing Error, Return Error                       */
        return (result);
    }

    /* $$$ - Load new Baud Rate values - $$$ */
                                                                /* ------------- CAN DRIVER CONFIGURATION ------------- */
    /* $$$ - Continue to Configure CAN Driver - $$$ */

                                                                /* --------- CAN OPERATION MODE CONFIGURATION --------- */
    /* $$$ - Enter Start Mode - $$$ */

    result = <Drv_Name>_CAN_ERR_NONE;                           /* Set Function Result: No Error                        */
    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                        <Drv_Name>_CAN_Open()
*
* Description : Unlocks the CAN device, i.e. Read/Write/IoCtl Functions will take effect.
*
* Argument(s) : dev_id      Bus Node Name, used to interface with the CAN Bus Layer.
*
*               dev_name    Driver Device Name, used to interface with the Low-Level Device Driver.
*                           Possible Values for Device Name:
*                                   <Drv_Name>_CAN_BUS_0     [CAN Bus 0]
*                                   <Drv_Name>_CAN_BUS_1     [CAN Bus 1]
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

CPU_INT16S  <Drv_Name>_CAN_Open (CPU_INT16S  dev_id,
                                 CPU_INT32U  dev_name,
                                 CPU_INT16U  mode)
{
    CPU_INT16S  result;
    CPU_SR_ALLOC();

    
    result = -1;                                                /* Initializing Variable(s)                             */

    if (dev_name >= <Drv_Name>_CAN_N_DEV) {                     /* Check if Device Name is out of Range                 */
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_BUS;
        return (result);
    }   

    if (mode != DEV_RW) {                                       /* Check if Mode is not Supported                       */
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_MODE;
        return (result);
    }

    CPU_CRITICAL_ENTER();

    if (<Drv_Name>_DevData[dev_name].Use == DEF_NO) {           /* Check if CAN Device is Unused                        */
        <Drv_Name>_DevData[dev_name].Use = DEF_YES;             /* Mark CAN Device as Used                              */

#if ((CANBUS_RX_HANDLER_EN > 0u) || \
     (CANBUS_TX_HANDLER_EN > 0u) || \
     (CANBUS_NS_HANDLER_EN > 0u))
        <Drv_Name>_CAN_SetDevIds((CPU_INT08U)dev_id,            /* Set Device IDs for the ISRs                          */
                                 (CPU_INT08U)dev_name);
#else
        (void)&dev_id;                                          /* Prevent Compiler Warning                             */
#endif
        
        result = (CPU_INT16S)dev_name;                          /* OK, Device is Opened                                 */
    } else {
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_OPEN;
    }
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                       <Drv_Name>_CAN_Close()
*
* Description : Locks the CAN device, i.e. Read/Write/IoCtl Functions will not take effect.
*
* Argument(s) : para_id     Parameter Identifier, returned by <Drv_Name>_CAN_Open().
*
* Return(s)   : Error code:  0 = No Error.
*                           -1 = Error Occurred.
*
* Caller(s)   : CanCfg in 'can_cfg.c'
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT16S  <Drv_Name>_CAN_Close (CPU_INT16S  para_id)
{
    CPU_INT16S  result;
    CPU_SR_ALLOC();

    
    result = -1;                                                /* Initializing Variable(s)                             */

    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= (CPU_INT08S)<Drv_Name>_CAN_N_DEV)) {
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_BUS;
        return (result);
    }

    CPU_CRITICAL_ENTER();
    
    if (<Drv_Name>_DevData[para_id].Use != DEF_NO) {            /* Check if CAN Device is Used                          */
        <Drv_Name>_DevData[para_id].Use = DEF_NO;               /* Mark CAN Device as Unused                            */
        result = <Drv_Name>_CAN_ERR_NONE;                       /* OK, Device is Closed                                 */
    } else {
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_CLOSE;
    }
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                        <Drv_Name>_CAN_IoCtl()
*
* Description : Performs Special Action on the Opened Device. The Function Code 'func' defines 
*               what the caller wants to do. Description of Function Codes are defined in the header.
*
* Argument(s) : para_id     Parameter Identifier, returned by <Drv_Name>_CAN_Open().
*
*               func        Function Code.
*
*               p_arg       Argument List, Specific to the Function Code. Possible Function Codes are:
*                                       IO_<Drv_Name>_CAN_GET_IDENT
*                                       IO_<Drv_Name>_CAN_GET_ERRNO
*                                       IO_<Drv_Name>_CAN_GET_DRVNAME
*                                       IO_<Drv_Name>_CAN_SET_BAUDRATE
*                                       IO_<Drv_Name>_CAN_START
*                                       IO_<Drv_Name>_CAN_STOP
*                                       IO_<Drv_Name>_CAN_RX_STANDARD
*                                       IO_<Drv_Name>_CAN_RX_EXTENDED
*                                       IO_<Drv_Name>_CAN_TX_READY
*                                       IO_<Drv_Name>_CAN_GET_NODE_STATUS
*                                       IO_<Drv_Name>_CAN_SET_RX_FILTER
*                                       IO_<Drv_Name>_CAN_IO_FUNC_N
*
* Return(s)   : Error code:  0 = No Error.
*                           -1 = Error Occurred.
*
* Caller(s)   : CanCfg in 'can_cfg.c'
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT16S  <Drv_Name>_CAN_IoCtl (CPU_INT16S   para_id,
                                  CPU_INT16U   func,
                                  void        *p_arg)
{
    <Drv_Name>_CAN_REG   *p_reg;
    <Drv_Name>_CAN_BAUD   baud;
    CPU_BOOLEAN           can_err;
    CPU_INT16S            result;
    CPU_SR_ALLOC();


    result  = -1;                                               /* Initialize Variable(s)                               */
    can_err = DEF_OK;
    
    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= (CPU_INT08S)<Drv_Name>_CAN_N_DEV)) {
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_BUS;
        return (result);
    }

    if (<Drv_Name>_DevData[para_id].Use != DEF_YES) {           /* Check if CAN Device is Opened                        */
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_OPEN;
        return (result);
    }
    
    p_reg = <Drv_Name>_DevData[para_id].RegPtr;                 /* Set Base Address for CAN Device(s)                   */
    
    CPU_CRITICAL_ENTER();

    switch (func) {                                             /*                SELECT: FUNCTION CODE                 */
        case IO_<Drv_Name>_CAN_GET_IDENT:                       /* --------------------- GET IDENT -------------------- */
             *(CPU_INT32U*)p_arg = <Drv_Name>_DrvId;            /* Return Driver Identification Code                    */
             result = <Drv_Name>_CAN_ERR_NONE;                  /* Indicate Successful Function Execution               */
             break;


        case IO_<Drv_Name>_CAN_GET_ERRNO:                       /* ------------------- GET ERRORCODE ------------------ */
             *(CPU_INT16U*)p_arg = <Drv_Name>_DrvErr;           /* Return Last Detected Error Code                      */
             result = <Drv_Name>_CAN_ERR_NONE;                  /* Indicate Successful Function Execution               */
             break;


        case IO_<Drv_Name>_CAN_GET_DRVNAME:                     /* ------------------ GET DRIVER NAME ----------------- */
                                                                /* Return Human Readable Driver Name                    */
             *(CPU_INT08U**)p_arg = (CPU_INT08U*)<Drv_Name>_CAN_NAME;
             result = <Drv_Name>_CAN_ERR_NONE;                  /* Indicate Successful Function Execution               */
             break;


        case IO_<Drv_Name>_CAN_SET_BAUDRATE:                    /* ------------------- SET BAUD RATE ------------------ */

             baud.BaudRate         = *((CPU_INT32U *)p_arg);
             baud.SamplePoint      = CAN_DEFAULT_SP;
             baud.ReSynchJumpWidth = CAN_DEFAULT_RJW;

             can_err = <Drv_Name>_CAN_CalcTimingReg(&baud);     /* Calculate Bit Timing Register Values                 */
             if (can_err == DEF_FAIL) {
                 <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_FUNC;   /* Bit Timing Error, Return Error                       */
                 return (result);
             }
             
             /* $$$ - Enter Config Mode & Load new Baud Rate values - $$$ */
             break;


        case IO_<Drv_Name>_CAN_START:                           /* -------------- START CAN COMMUNICATION ------------- */

             break;


        case IO_<Drv_Name>_CAN_STOP:                            /* ------------------ SET CAN TO STOP ----------------- */

             break;


        case IO_<Drv_Name>_CAN_RX_STANDARD:                     /* ------------------ SET RX STANDARD ----------------- */

             break;


        case IO_<Drv_Name>_CAN_RX_EXTENDED:                     /* ------------------ SET RX EXTENDED ----------------- */

             break;


        case IO_<Drv_Name>_CAN_TX_READY:                        /* --------------------- TX READY --------------------- */
             if (/* $$$ - Tx is Ready - $$$ */) {
                 *((CPU_INT08U *)p_arg) = 1u;                   /* Tx is     Ready, OK to Transmit.                     */
             } else {
                 *((CPU_INT08U *)p_arg) = 0u;                   /* Tx is NOT Ready, Do NOT Transmit.                    */
             }

             result = <Drv_Name>_CAN_ERR_NONE;                  /* Indicate Successful Function Execution               */
             break;


        case IO_<Drv_Name>_CAN_GET_NODE_STATUS:                 /* ------------------ GET NODE STATUS ----------------- */
             *((CPU_INT08U *)p_arg) = 0u;                       /* Set the Initial Argument Pointer to Zero             */

                                                                /* Check CAN Node Status: Bus Off, Error Active, etc.   */
                                                                
             result = <Drv_Name>_CAN_ERR_NONE;                  /* Indicate Successful Function Execution               */
             break;
            

        case IO_<Drv_Name>_CAN_SET_RX_FILTER:                   /* ------------------- SET RX FILTER ------------------ */

             break;


        case IO_<Drv_Name>_CAN_IO_FUNC_N:
                                                                /* Set the Size of IO Function Number for return.       */
            *((CPU_INT08U *)p_arg) = IO_<Drv_Name>_CAN_IO_FUNC_N + 1u;

             result = <Drv_Name>_CAN_ERR_NONE;                  /* Indicate Successful Function Execution               */
             break;


        default:                                                /* --------------- UNKNOWN FUNCTION CODE -------------- */
             break;
    }

    if (can_err == DEF_FAIL) {
        result            = <Drv_Name>_CAN_ERR_FUNC;            /* Error occurred in function, Return with Error.       */
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_FUNC;
    } else {
        result = <Drv_Name>_CAN_ERR_NONE;                       /* Indicate Successful Function Execution               */
    }

    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                        <Drv_Name>_CAN_Read()
*
* Description : Read a received CAN Frame from a Message Buffer. The Buffer must have space for only
*               one CAN Frame.
*
* Argument(s) : para_id     Parameter Identifier, returned by <Drv_Name>_CAN_Open().
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
* Note(s)     : (1) When Using Extended IDs for Rx'd Messages, its required to save both the Standard
*                   Section and Extended Section of the ID and placed both in the Rx Frame in the
*                   following Format:
*                               Bits:  31     30    29                   18                   0
*                       uC/CAN Frame: [ 0u | RTR | IDE |    Standard ID    |    Extended ID    ]
*********************************************************************************************************
*/

CPU_INT16S  <Drv_Name>_CAN_Read (CPU_INT16S   para_id,
                                 CPU_INT08U  *buf,
                                 CPU_INT16U   size)
{
    <Drv_Name>_CAN_REG  *p_reg;
    <Drv_Name>_CAN_FRM  *p_frm;
    CPU_INT16S           result;
    CPU_SR_ALLOC();
    
    
    result = -1;                                                /* Initializing Variable(s)                             */
                                                        
    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= (CPU_INT08S)<Drv_Name>_CAN_N_DEV)) {
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_BUS;
        return (result);
    }
    
    if (size != sizeof(<Drv_Name>_CAN_FRM)) {                   /* Check if Size is Plausible                           */
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_NO_DATA;
        return (result);
    }
    
    if (buf == (void *)0) {                                     /* Check if Buffer Pointer is Invalid                   */
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_ARG;
        return (result);
    }
    
    if (<Drv_Name>_DevData[para_id].Use != DEF_YES) {           /* Check if CAN Device is Opened                        */
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_OPEN;
        return (result);
    }
    
    p_reg = <Drv_Name>_DevData[para_id].RegPtr;                 /* Set Base Address for CAN Device(s)                   */
    p_frm = (<Drv_Name>_CAN_FRM *)buf;
    
    CPU_CRITICAL_ENTER();

                                                                /* ------------------- READ Rx'D MSG ------------------ */
    /* $$$ - Read Rx'd Message - $$$ */
    
    result = size;                                              /* If everything is good, return the size.              */

    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                       <Drv_Name>_CAN_Write()
*
* Description : Write a CAN Frame to a Message Buffer. The Buffer must contain only one CAN Frame
*               which will be written to a predefined Message Buffer.
*
* Argument(s) : para_id     Parameter Identifier, returned by <Drv_Name>_CAN_Open().
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
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT16S  <Drv_Name>_CAN_Write (CPU_INT16S   para_id,
                                  CPU_INT08U  *buf,
                                  CPU_INT16U   size)
{
    <Drv_Name>_CAN_REG  *p_reg;
    <Drv_Name>_CAN_FRM  *p_frm;
    CPU_INT16S           result;
    CPU_SR_ALLOC();

    
    result = -1;                                                /* Initializing Variable(s)                             */

    if ((para_id <  0) ||                                       /* Check if Parameter ID is out of Range                */
        (para_id >= (CPU_INT08S)<Drv_Name>_CAN_N_DEV)) {
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_BUS;
        return (result);
    }
    
    if (size != sizeof(<Drv_Name>_CAN_FRM)) {                   /* Check if Size is Plausible                           */
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_NO_DATA;
        return (result);
    }
    
    if (buf == (void *)0) {                                     /* Check if Buffer Pointer is Invalid                   */
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_ARG;
        return (result);
    }
    
    if (<Drv_Name>_DevData[para_id].Use != DEF_YES) {           /* Check if CAN Device is Opened                        */
        <Drv_Name>_DrvErr = <Drv_Name>_CAN_ERR_OPEN;
        return (result);      
    }

    p_reg =  <Drv_Name>_DevData[para_id].RegPtr;                /* Set Base Address for CAN Device(s)                   */
    p_frm = (<Drv_Name>_CAN_FRM *)buf;
    
    CPU_CRITICAL_ENTER();
                                                                /* ------------------ WRITE Tx'D MSG ------------------ */
    /* $$$ - Write Message to be Transmitted - $$$ */

    result = size;                                              /* If everything is good, return the size.              */
    
    CPU_CRITICAL_EXIT();

    return (result);                                            /* Return Function Result                               */
}


/*
*********************************************************************************************************
*                                      <Drv_Name>_CAN_SetMode()
*
* Description : Sets the CAN Mode.
*
* Argument(s) : para_id     Desired CAN Module ID.
*
*               mode        Desired Mode to set CAN.
*
* Return(s)   : none.
*
* Caller(s)   : <Drv_Name>_CAN_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_BOOLEAN  <Drv_Name>_CAN_SetMode (CPU_INT32U           para_id,
                                     <Drv_Name>_CAN_MODE  mode)
{
    <Drv_Name>_CAN_REG  *p_reg;

    
    p_reg = <Drv_Name>_DevData[para_id].RegPtr;                 /* Set Base Address for CAN Device(s)                   */
    
    switch (mode) {
        case <Drv_Name>_CAN_MODE_<Mode>:                        /* ------------------ CONFIGURE MODE ------------------ */

             break;


        default:                                                /* --------------- UNKNOWN DEFAULT MODE --------------- */
             break;
    }
    
    return (DEF_OK);
}


/*
*********************************************************************************************************
*                                      <Drv_Name>_CAN_ErrCheck()
*
* Description : Corrects Errors found in the Status & Error Register(s).
*
* Argument(s) : para_id     Desired CAN Module ID.
*
* Return(s)   : none.
*
* Caller(s)   : <Drv_Name>_CAN_NSHandler().
*               <Drv_Name>_CAN_IoCtl().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  <Drv_Name>_CAN_ErrCheck (CPU_INT32U  para_id)
{
    <Drv_Name>_CAN_REG  *p_reg;
    CPU_INT32U       baud_rate;


    if (para_id < <Drv_Name>_CAN_N_DEV) {                       /* Check if Parameter ID is out of Range, w/o return    */
        p_reg     = <Drv_Name>_DevData[para_id].RegPtr;         /* Set Base Address for CAN Device(s)                   */
        baud_rate = CAN_DEFAULT_BAUDRATE;                       /* Set Restart Baudrate to Default Baudrate             */

        /* $$$ - Add Error Checking & Correction - $$$ */
    }
}
