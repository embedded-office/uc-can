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
#include "drv_can.h"                                  /* driver declarations                      */
#include "drv_def.h"                                  /* driver layer declarations                */
#include "drv_can_reg.h"                              /* register declarations                    */


/*
***************************************************************************************************
*                            CONSTANTS
***************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER IDENTIFICATION
* \ingroup  V850E2_Fx4_CAN
*
*           This constant variable holds the unique driver identification code.
*/
/*------------------------------------------------------------------------------------------------*/
static const CPU_INT32U DrvIdent = 0x243F2901;

/*
***************************************************************************************************
*                            GLOBAL DATA
***************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER ERRORCODE
* \ingroup  V850E2_Fx4_CAN
*
*           This variable holds the detailed errorcode, if an error is detected.
*/
/*------------------------------------------------------------------------------------------------*/
static CPU_INT16U DrvError;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER RUNTIME DATA
* \ingroup  V850E2_Fx4_CAN
*
*           Global can device data.
*/
/*------------------------------------------------------------------------------------------------*/
static V850E2_Fx4_CAN_DATA DevData[V850E2_Fx4_CAN_N_DEV];

/*
***************************************************************************************************
*                            FUNCTIONs
***************************************************************************************************
*/

/*************************************************************************************************/
/*!
* \brief                      CAN INITIALISATION
*
*           Initializes the V850E2_Fx4 CAN controller selected by argument.
*
*
* \param    arg               CAN bus device name
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*************************************************************************************************/
CPU_INT16S V850E2_Fx4CANInit(CPU_INT32U arg)
{
    CPU_INT16S            result = -1;                /* Local: Function result                   */
    CPU_INT16U            val    = 0;                 /* Local: help variable                     */
    CPU_INT32U            i;                          /* Local: loop variable                     */
    V850E2_MASK_CONTROL  *pmask;                      /* Local: mask register                     */
    V850E2_MB_16BIT      *mb16;                       /* Local: message box 16 bit format         */
    V850E2_MB_8BIT       *mb8;                        /* Local: message box 8 bit format          */    
    V850E2_GLOB_MOD_REG  *reg;                        /* Local: global and module register        */
    V850E2_Fx4_CAN_DATA  *can;                        /* Local: Pointer to can device             */
                                                      /*------------------------------------------*/
#if V850E2_Fx4_CAN_ARG_CHK_CFG > 0                    /* Checking arguments (if enabled)          */
    if (arg >= V850E2_Fx4_CAN_N_DEV) {                /* can dev out of range?                    */
        DrvError = V850E2_Fx4_CAN_INIT_ERR;
        return(result);
    }
#endif                                                /*------------------------------------------*/
    DrvError = V850E2_Fx4_CAN_NO_ERR;                 /* reset to 0                               */
    
    V850E2_Fx4InitTiming();                           /* configure the domain clocks              */
                                                      /*------------------------------------------*/
    if (arg == V850E2_Fx4_CAN_BUS_0) {                /* Check which can bus to initialise        */
        DevData[arg].BaseAddrGlobRegs  = (void *) FCN0_CAN_BASE;
        DevData[arg].BaseAddrMaskRegs  = (void *)(FCN0_CAN_BASE + FCNxCMMKCTL01W);
        DevData[arg].BaseAddrMB16Regs  = (void *)(FCN0_CAN_BASE + FCNxM000DAT0H);
        DevData[arg].BaseAddrMB8Regs   = (void *)(FCN0_CAN_BASE + FCNxM000DAT0B);
        DevData[arg].MaxNumBuf         = FCN0_NUMBER_MB;
    } else if (arg == V850E2_Fx4_CAN_BUS_1) {
        DevData[arg].BaseAddrGlobRegs  = (void *) FCN1_CAN_BASE;
        DevData[arg].BaseAddrMaskRegs  = (void *)(FCN1_CAN_BASE + FCNxCMMKCTL01W);
        DevData[arg].BaseAddrMB16Regs  = (void *)(FCN1_CAN_BASE + FCNxM000DAT0H);
        DevData[arg].BaseAddrMB8Regs   = (void *)(FCN1_CAN_BASE + FCNxM000DAT0B);
        DevData[arg].MaxNumBuf         = FCN1_NUMBER_MB;
    } else if (arg == V850E2_Fx4_CAN_BUS_2) {
        DevData[arg].BaseAddrGlobRegs  = (void *) FCN2_CAN_BASE;
        DevData[arg].BaseAddrMaskRegs  = (void *)(FCN2_CAN_BASE + FCNxCMMKCTL01W);
        DevData[arg].BaseAddrMB16Regs  = (void *)(FCN2_CAN_BASE + FCNxM000DAT0H);
        DevData[arg].BaseAddrMB8Regs   = (void *)(FCN2_CAN_BASE + FCNxM000DAT0B);
        DevData[arg].MaxNumBuf         = FCN2_NUMBER_MB;
    } else if (arg == V850E2_Fx4_CAN_BUS_3) {
        DevData[arg].BaseAddrGlobRegs  = (void *) FCN3_CAN_BASE;
        DevData[arg].BaseAddrMaskRegs  = (void *)(FCN3_CAN_BASE + FCNxCMMKCTL01W);
        DevData[arg].BaseAddrMB16Regs  = (void *)(FCN3_CAN_BASE + FCNxM000DAT0H);
        DevData[arg].BaseAddrMB8Regs   = (void *)(FCN3_CAN_BASE + FCNxM000DAT0B);
        DevData[arg].MaxNumBuf         = FCN3_NUMBER_MB;
    } else if (arg == V850E2_Fx4_CAN_BUS_4) {
        DevData[arg].BaseAddrGlobRegs  = (void *) FCN4_CAN_BASE;
        DevData[arg].BaseAddrMaskRegs  = (void *)(FCN4_CAN_BASE + FCNxCMMKCTL01W);
        DevData[arg].BaseAddrMB16Regs  = (void *)(FCN4_CAN_BASE + FCNxM000DAT0H);
        DevData[arg].BaseAddrMB8Regs   = (void *)(FCN4_CAN_BASE + FCNxM000DAT0B);
        DevData[arg].MaxNumBuf         = FCN4_NUMBER_MB;
    } else if (arg == V850E2_Fx4_CAN_BUS_DIAG) {
        DevData[arg].BaseAddrGlobRegs  = (void *) DCN0_CAN_BASE;
        DevData[arg].BaseAddrMaskRegs  = (void *)(DCN0_CAN_BASE + FCNxCMMKCTL01W);
        DevData[arg].BaseAddrMB16Regs  = (void *)(DCN0_CAN_BASE + FCNxM000DAT0H);
        DevData[arg].BaseAddrMB8Regs   = (void *)(DCN0_CAN_BASE + FCNxM000DAT0B);
        DevData[arg].MaxNumBuf         = DCN0_NUMBER_MB;
    }

    can = &DevData[arg];                              /* set pointer to can device                */
    reg = (V850E2_GLOB_MOD_REG *) can->BaseAddrGlobRegs;
    can->Use         = 0;                             /* Set device to unused                     */        
    can->OverflowCnt = 0;
    V850E2_Fx4InitPorts (arg);                        /* set the pins                             */
    
                                                      /*------------------------------------------*/
    reg->FCNnGMCLCTL = 0x1000;                        /* perform soft reset                       */
    i = 0;
    while ((reg->FCNnGMCLCTL & 0x0010) != 0){         /* check Soft reset execution status bit    */
        i++;
    }
                                                      /*------------------------------------------*/
    if ((reg->FCNnGMCLCTL & 0x0020) == 0) {           /* check Message buffer RAM read error      */
                                                      /* detect bit, (0) is no error              */
                                                      /* set fcan clock selection                 */
        reg->FCNnGMCSPRE = V850E2_Fx4_CLOCK_SELECTION;
                                                      /*------------------------------------------*/
        reg->FCNnGMCLCTL = 0x0100;                    /* set Global operation mode bit            */
        i = 0;
        while ((reg->FCNnGMCLCTL & 0x01) == 0) {      /* wait until this mode is reached          */
            reg->FCNnGMCLCTL = 0x0100;                /* enable FCN module                        */
            i++;
        }
                                                      /*------------------------------------------*/
                                                      /* Initialize parameter struct of the       */
        DevData[arg].Baudrate        = V850E2_Fx4_DEF_BAUDRATE;
        DevData[arg].SamplePoint     = V850E2_Fx4_DEF_SP;
        DevData[arg].ResynchJumpWith = V850E2_Fx4_DEF_RJW;
                                                      /*------------------------------------------*/
                                                      /* Calculate the values for the timing      */
                                                      /* register with the parameter setting      */
        result  = V850E2_Fx4CalcTimingReg (&DevData[arg]);
        if (result != V850E2_Fx4_CAN_NO_ERR) {
            DrvError = V850E2_Fx4_CAN_INIT_ERR;
            result   = V850E2_Fx4_CAN_INIT_ERR;
        } else {
            DrvError = V850E2_Fx4_CAN_NO_ERR;
            result   = V850E2_Fx4_CAN_NO_ERR;                
            DevData[arg].Initialized     = 1;         /* current device                           */        
        }
        
        reg->FCNnCMBRPRS = DevData[arg].PRESDIV;
        reg->FCNnCMBTCTL = (DevData[arg].RJW   << 12) |/* Set the bit rate register               */
                           (DevData[arg].PSEG1) | 
                           (DevData[arg].PSEG2 << 8);
                                                      /*------------------------------------------*/

#if V850E2_Fx4_CAN_RX_INTERRUPT_EN > 0
       val = 0x0200;                                  /* enable rx interrupt                      */
#endif
#if V850E2_Fx4_CAN_TX_INTERRUPT_EN > 0
       val |= 0x0100;                                 /* enable tx interrupt                      */
#endif
#if V850E2_Fx4_CAN_NS_INTERRUPT_EN > 0
       val |= 0x0400;                                 /* enable error status interrupt            */
#endif
                                                      /* install interrupt handler                */
#if ((V850E2_Fx4_CAN_TX_INTERRUPT_EN > 0) || \
     (V850E2_Fx4_CAN_RX_INTERRUPT_EN > 0) || \
     (V850E2_Fx4_CAN_NS_INTERRUPT_EN > 0))
       reg->FCNnCMIECTL = val;                        /* enable selected interrupts               */
       V850E2_Fx4_BSP_InstallIRQ(arg);
#endif            
                                                      /*------------------------------------------*/
                                                      /* set mask control register                */            
        pmask = (V850E2_MASK_CONTROL *)can->BaseAddrMaskRegs;
        pmask->FCnNCMMKCTL01 = 0x1FFFFFFF;            /* set all to 1 = masked                    */
        pmask->FCnNCMMKCTL03 = 0x1FFFFFFF;
        pmask->FCnNCMMKCTL05 = 0x1FFFFFFF;
        pmask->FCnNCMMKCTL07 = 0x1FFFFFFF;
        pmask->FCnNCMMKCTL09 = 0x1FFFFFFF;
        pmask->FCnNCMMKCTL11 = 0x1FFFFFFF;
        pmask->FCnNCMMKCTL13 = 0x1FFFFFFF;
        pmask->FCnNCMMKCTL15 = 0x1FFFFFFF;
                                                      /*------------------------------------------*/
                                                      /* Initialize message buffers               */
        mb16 = (V850E2_MB_16BIT *) can->BaseAddrMB16Regs;
        mb8  = (V850E2_MB_8BIT *)  can->BaseAddrMB8Regs;
                    
        for (i=0; i<can->MaxNumBuf; i++) {
            mb16->FCNnMmCTL = 0x0;
            mb16->FCNnMmCTL = 0x000F;                 /* clear IENF, RDYF, TRQF, DTNF             */
            mb16++;  
        }
        for (i=0; i<V850E2_Fx4_TX_MB; i++) {
            mb8->FCNnMmSTRB = 1;                      /* tx buffer: no overwrite, use data frames */
            mb8++;
        }
        for (i=V850E2_Fx4_TX_MB; i<can->MaxNumBuf; i++) {
            mb8->FCNnMmSTRB = 0x11;                   /* rx buffer: no overwrite, use data frames */
            mb8++;                                    /* use mask 1, i.e. receive all             */
        }
        mb16 = (V850E2_MB_16BIT *) can->BaseAddrMB16Regs;
        for (i=0;i<can->MaxNumBuf;i++) {
            mb16->FCNnMmCTL = 0x0;
            mb16->FCNnMmCTL = 0x0900;                 /* set mb ready & interrupt enable bit      */
            mb16++;  
        }                                             /*------------------------------------------*/
                                                      /* Set module control register              */
        reg->FCNnCMCLCTL = 0x0100;                    /* set to normal operation mode             */
    }
                                                      /*------------------------------------------*/
    return result;                                    /* return function result                   */
}


/*************************************************************************************************/
/*!
* \brief                      OPEN THE CAN DEVICE
*
*           Unlocks the device, i.e. IoCtl/Read/Write-function will take effect.
*
*
* \param devId   the CAN device name - use only when irqs are enabled
* \param devName the CAN device name
* \param mode the mode in which CAN devices will be used
* \return the parameter identifier for further access or -1 if an error occurs
*/
/*************************************************************************************************/
CPU_INT16S V850E2_Fx4CANOpen(CPU_INT16S devId, CPU_INT32U devName, CPU_INT16U mode)
{
    V850E2_Fx4_CAN_DATA *can;                         /* Local: Pointer to can device             */
    CPU_INT16S           result = -1;                 /* Local: Function result                   */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/
#if V850E2_Fx4_CAN_ARG_CHK_CFG > 0
    if ((CPU_INT08U)devName >= V850E2_Fx4_CAN_N_DEV) {/* check that device name is in range       */
        DrvError= V850E2_Fx4_CAN_BUS_ERR;
        return (result);                              /* return function result                   */
    }
    if (mode != DEV_RW) {                             /* mode not supported?                      */
        DrvError = V850E2_Fx4_CAN_MODE_ERR;
        return (result);
    }
#endif
    can = &DevData[devName];                          /* set pointer to can device                */
    CPU_CRITICAL_ENTER();
    if (can->Use == 0) {                              /* check, that can device is unused         */
        can->Use = 1;                                 /* mark can device as used                  */
#if ((V850E2_Fx4_CAN_RX_INTERRUPT_EN > 0) || \
     (V850E2_Fx4_CAN_TX_INTERRUPT_EN > 0) || \
     (V850E2_Fx4_CAN_NS_INTERRUPT_EN > 0))
                                                      /* store the received Node Id for the irqs  */
        V850E2_Fx4_BSP_SetDevIds ((CPU_INT08U) devId,
                                 (CPU_INT08U) devName);
#else
        devId = devId;                                /* prevent compiler warning                 */
#endif
        result   = (CPU_INT08U) devName;              /* Okay, device is opened                   */
    } else {
        DrvError = V850E2_Fx4_CAN_OPEN_ERR;
    }
    CPU_CRITICAL_EXIT();

    return (result);                                  /* return function result                   */
}


/*************************************************************************************************/
/*!
* \brief                      CLOSE THE CAN DEVICE
*
*           Locks the device, i.e. IoCtl/Read/Write-function will have no effect.
*
*
* \param paraId the parameter identifier, returned by V850E2_Fx4CANOpen()
* \return errorcode (0 if ok -1 if an error occured)
*/
/*************************************************************************************************/
CPU_INT16S V850E2_Fx4CANClose(CPU_INT16S paraId)
{
    V850E2_Fx4_CAN_DATA *can;                         /* Local: Pointer to can device             */
    CPU_INT16S           result = -1;                 /* Local: Function result                   */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/
#if V850E2_Fx4_CAN_ARG_CHK_CFG > 0
    if ((paraId >= V850E2_Fx4_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range      */
        DrvError = V850E2_Fx4_CAN_BUS_ERR;
        return (result);                              /* return function result                   */
    }
#endif
    can = &DevData[paraId];                           /* Set pointer to can device                */
    CPU_CRITICAL_ENTER();
    if (can->Use != 0) {                              /* check, that can device is used           */
        can->Use  = 0;                                /* mark can device as unused                */
        result    = V850E2_Fx4_CAN_NO_ERR;            /* Indicate sucessfull function execution   */
    } else {
        DrvError  = V850E2_Fx4_CAN_CLOSE_ERR;
    }
    CPU_CRITICAL_EXIT();

    return (result);                                  /* return function result                   */
}

/*************************************************************************************************/
/*!
* \brief                      CAN I/O CONTROL
*
*           This function performs a special action on the opened device. The functioncode func
*           defines what the caller want to do. Description of functioncodes as defined in headerfile
*
*
* \param paraId parameter identifier, returned by V850E2_Fx4CANOpen()
* \param func function code
* \param arg argument list, specific to the function code
* \return errorcode (0 if ok -1 if an error occured)
*/
/*************************************************************************************************/
CPU_INT16S V850E2_Fx4CANIoCtl(CPU_INT16S paraId, CPU_INT16U func, void *argp)
{
    CPU_INT16S            result      = -1;           /* Local: Function result                   */
    CPU_INT08U            val;                        /* Local: register value                    */
    CPU_INT32U            i;                          /* Local: loop variable                     */
    CPU_INT32U            mask;                       /* Local: mask for filter                   */
    CPU_INT32U            canId;                      /* Local: id for filter                     */
    CPU_INT08U            mb;                         /* Local: message box  number               */
    V850E2_MB_16BIT      *mb16;                       /* Local: message box 16 bit format         */
    V850E2_MB_8BIT       *mb8;                        /* Local: message box 8 bit format          */    
    V850E2_MASK_CONTROL  *pmask;                      /* Local: mask register                     */
    V850E2_GLOB_MOD_REG  *reg;                        /* Local: global and module register        */    
    V850E2_Fx4_CAN_DATA  *can;                        /* Local: Pointer to can device             */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/
    can = &DevData[paraId];                           /* set pointer to can device                */

#if V850E2_Fx4_CAN_ARG_CHK_CFG > 0
    if ((paraId >= V850E2_Fx4_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range      */
        DrvError = V850E2_Fx4_CAN_BUS_ERR;
        return (result);                              /* return function result                   */
    }
    if (can->Use != 1) {                              /* check, that can device is opened         */
        DrvError  = V850E2_Fx4_CAN_OPEN_ERR;
        return (result);                              /* return function result                   */
    }
#endif
    CPU_CRITICAL_ENTER();
    
    mb16  = (V850E2_MB_16BIT *)     can->BaseAddrMB16Regs;
    mb8   = (V850E2_MB_8BIT *)      can->BaseAddrMB8Regs;
    pmask = (V850E2_MASK_CONTROL *) can->BaseAddrMaskRegs;
    reg   = (V850E2_GLOB_MOD_REG *) can->BaseAddrGlobRegs;
    
                                                      /*------------------------------------------*/
    switch (func) {                                   /* select: function code                    */
                                                      /*------------------------------------------*/
        case IO_V850E2_Fx4_CAN_GET_IDENT:             /* GET IDENT                                */
            (*(CPU_INT32U*)argp) = DrvIdent;          /* return driver ident code                 */
            result = V850E2_Fx4_CAN_NO_ERR;           /* Indicate sucessfull function execution   */
            break;
                                                      /*------------------------------------------*/
        case IO_V850E2_Fx4_CAN_GET_ERRNO:             /* GET ERRORCODE                            */
            (*(CPU_INT16U*)argp) = DrvError;          /* return last detected errorcode           */
            result = V850E2_Fx4_CAN_NO_ERR;           /* Indicate sucessfull function execution   */
            break;
                                                      /*------------------------------------------*/
        case IO_V850E2_Fx4_CAN_GET_DRVNAME:           /* GET DRIVER NAME                          */
            (*(CPU_INT08U**)argp) = (CPU_INT08U*)
                                   V850E2_Fx4_CAN_NAME;
                                                      /* return human readable driver name        */
            result = V850E2_Fx4_CAN_NO_ERR;           /* Indicate sucessfull function execution   */
            break;

        case IO_V850E2_Fx4_CAN_SET_BAUDRATE:          /* Function: Set can bus baudrate           */
            can->Baudrate = *(CPU_INT32U *)argp;      /* Set baudrate parameter to given argument */
                                                      /* and calculate the values for the timing  */
                                                      /* register with the parameter setting      */
            result = V850E2_Fx4CalcTimingReg(can);
            if (result != V850E2_Fx4_CAN_NO_ERR) {
                DrvError = V850E2_Fx4_CAN_ARG_ERR;    /* indicate error on function result.       */
            } else {                                  /* set can timing registers if timing       */
                                                      /* calculation success.                     */
                
                                                      /* Set module control register              */
                reg->FCNnCMCLCTL = 0x0003;            /* set to init operation mode               */
                                                      /* wait until init mode is reached          */
                i = 0;
                while ((reg->FCNnCMCLCTL & 0x0003) != 0) {
                    i++;
                };        
                reg->FCNnCMBRPRS =  can->PRESDIV;
                reg->FCNnCMBTCTL = (can->RJW   << 12) |/* Set the bit rate register            */
                                   (can->PSEG1) | 
                                   (can->PSEG2 << 8);
                reg->FCNnCMCLCTL = 0x0100;            /* set to normal operation mode             */                
                result = V850E2_Fx4_CAN_NO_ERR;       /* Indicate sucessfull function execution   */
            }
            break;                                    /*------------------------------------------*/
        case IO_V850E2_Fx4_CAN_TX_READY:
            
            mb = 99;
            for (i=0; i<V850E2_Fx4_TX_MB;i++) {       /* find free tx message buffer              */
                if ((mb16->FCNnMmCTL & 0x0002) == 0) { /* .. check TRQF                           */
                    mb = i;
                    break;
                }      
                mb16++;
            }
            if (mb == 99) {
                *((CPU_INT08U *)argp) = 0;            /* Transmit Channel is not available        */
            } else {
                *((CPU_INT08U *)argp) = 1;            /* Transmit Channel is available            */
            }
            result = V850E2_Fx4_CAN_NO_ERR;           /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/
        case IO_V850E2_Fx4_CAN_START:                 /* START CAN COMMUNICATION                  */
                                                      /* Set module control register              */
            reg->FCNnCMCLCTL = 0x0100;                /* set to normal operation mode             */
            result = V850E2_Fx4_CAN_NO_ERR;           /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/
        case IO_V850E2_Fx4_CAN_STOP:                  /* STOP CAN COMMUNICATION                   */
            
                                                      /* Set module control register              */
            reg->FCNnCMCLCTL = 0x0003;                /* set to init operation mode               */
                                                      /* wait until init mode is reached          */
            i = 0;
            while ((reg->FCNnCMCLCTL & 0x0003) != 0) {
                i++;
            };        
            result = V850E2_Fx4_CAN_NO_ERR;           /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/
        case IO_V850E2_Fx4_CAN_RX_STANDARD:
            
            mb16 += V850E2_Fx4_TX_MB;            
            for (i=V850E2_Fx4_TX_MB; i<can->MaxNumBuf; i++) {
                mb16->FCNnMmCTL = 0x0001;             /* clear RDFY bit to make buffer writeable  */
                while (mb16->FCNnMmCTL & 1) {};       /* wait until the RDFY is cleared           */                
                mb16->FCNnMmMID1 &= 0x7FFF;           /* clear SSIE bit                           */
                mb16->FCNnMmCTL = 0x0100;             /* set RDFY bit to make buffer operational  */
                mb16++;
            }
            
            result = V850E2_Fx4_CAN_NO_ERR;           /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/
        case IO_V850E2_Fx4_CAN_RX_EXTENDED: 

            mb16 += V850E2_Fx4_TX_MB;            
            for (i=V850E2_Fx4_TX_MB; i<can->MaxNumBuf; i++) {
                mb16->FCNnMmCTL = 0x0001;             /* clear RDFY bit to make buffer writeable  */
                while (mb16->FCNnMmCTL & 1) {};       /* wait until the RDFY is cleared           */
                mb16->FCNnMmMID1 |= 0x8000;           /* set SSIE bit                             */
                mb16->FCNnMmCTL = 0x0100;             /* set RDFY bit to make buffer operational  */
                mb16++;
            }
            
            result = V850E2_Fx4_CAN_NO_ERR;           /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/
        case IO_V850E2_Fx4_CAN_GET_NODE_STATUS:       /* GET NODE STATUS                          */

            *((CPU_INT08U *)argp) = 0;                /* bus active                               */

            val = reg->FCNnCMINSTR;                   /* get module info register                 */
            if (((val & 0x0C) == 0x0C) ||             /* TX Error Passive Bit                     */
                ((val & 0x03) == 0x03)) {             /* RX Error Passive Bit                     */
                *((CPU_INT08U *)argp) = 1;
            }
            if ((val & 0x10) != 0) {                  /* Bus Off Bit                              */
                *((CPU_INT08U *)argp) = 2;
            }        
            result = V850E2_Fx4_CAN_NO_ERR;           /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/

        case IO_V850E2_Fx4_CAN_SET_RX_FILTER_1:       /* SET RX FILTER 1                          */
            mask  = ((CPU_INT32U*)argp)[0];
            canId = ((CPU_INT32U*)argp)[1];
            

            if (canId > 0x7FF) {                      /* depending on the CAN ID                  */
                                                      /* use bit 0..28                            */
                                                      /* set ext canId + ide bit and mask         */
                pmask->FCnNCMMKCTL03 = ~mask;         /* set the mask                             */
                mb8  += V850E2_Fx4_TX_MB;
                mb16 += V850E2_Fx4_TX_MB;            
                for (i=V850E2_Fx4_TX_MB; i<can->MaxNumBuf; i++) {
                    mb16->FCNnMmCTL = 0x0001;         /* clear RDFY bit to make buffer writeable  */
                    while (mb16->FCNnMmCTL & 1) {};   /* wait until the RDFY is cleared           */

                    mb8->FCNnMmSTRB   = 0x19;         /* use mask 2                               */
                    mb16->FCNnMmMID1  = (CPU_INT16U) ((canId & 0x1FFF) >> 16);
                    mb16->FCNnMmMID1 |= 0x8000;       /* set SSIE bit                             */        
                    mb16->FCNnMmMID0  = (CPU_INT16U)  (canId & 0xFFFF);
 
                    mb16->FCNnMmCTL = 0x0100;         /* set RDFY bit to make buffer operational  */
                    mb16++;
                    mb8++;
                }
                
            } else {
                                                      /* use bit 28..18                           */
                                                      /* set std canId and mask                   */
                pmask->FCnNCMMKCTL03 = (~mask << 18); /* set the mask                             */
                mb8  += V850E2_Fx4_TX_MB;
                mb16 += V850E2_Fx4_TX_MB;            
                for (i=V850E2_Fx4_TX_MB; i<can->MaxNumBuf; i++) {
                    mb16->FCNnMmCTL = 0x0001;         /* clear RDFY bit to make buffer writeable  */
                    while (mb16->FCNnMmCTL & 1) {};   /* wait until the RDFY is cleared           */

                    mb8->FCNnMmSTRB   = 0x19;         /* use mask 2                               */
                    mb16->FCNnMmMID1  = (CPU_INT16U) ((canId & 0x7FF) << 2);
                    mb16->FCNnMmMID0  = 0;
 
                    mb16->FCNnMmCTL   = 0x0100;       /* set RDFY bit to make buffer operational  */
                    mb16++;
                    mb8++;                            
                }
            }
            break;
                                                      /*------------------------------------------*/
        default:
            break;
    }
    CPU_CRITICAL_EXIT();

    if (result == -1) {
        DrvError = V850E2_Fx4_CAN_FUNC_ERR;
    }
    return(result);                                   /* Return function result                   */
}

/*************************************************************************************************/
/*!
* \brief                      CAN READ DATA
*
*           Read a received can frame from a message buffer. The buffer must have space for only
*           one can frame.
*
*
* \param paraId parameter identifier, returned by V850E2_Fx4CANOpen()
* \param buffer Pointer to can frame
* \param size   Length of can frame memory
* \return      (size of can frame if ok, errorcode if an error occured)
*/
/*************************************************************************************************/
CPU_INT16S V850E2_Fx4CANRead (CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size)
{
    V850E2_Fx4_CANFRM    *frm;                        /* Local: Pointer to can frame              */
    CPU_INT32U            i;                          /* Local: loop variable                     */
    CPU_INT16S            result = -1;                /* Local: return value                      */
    CPU_INT08U            nbr    =  0;                /* Local: receive buffer index              */
    CPU_INT16U            hist;                       /* Local: receive history                   */
    V850E2_MB_16BIT      *mb16;                       /* Local: message box 16 bit format         */
    V850E2_MB_8BIT       *mb8;                        /* Local: message box 8 bit format          */
    V850E2_GLOB_MOD_REG  *reg;                        /* Local: global and module register        */    
    V850E2_Fx4_CAN_DATA  *can;                        /* Local: Pointer to can device             */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/
    can = &DevData[paraId];                           /* set pointer to can device                */

#if V850E2_Fx4_CAN_ARG_CHK_CFG > 0

    if ((paraId >= V850E2_Fx4_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range       */
        DrvError = V850E2_Fx4_CAN_BUS_ERR;
        return (result);
    }
    if (size != sizeof(V850E2_Fx4_CANFRM)) {           /* check that size is plausible             */
        DrvError = V850E2_Fx4_CAN_NO_DATA_ERR;
        return (result);
    }
    if (buffer == (void *)0) {                        /* invalid buffer pointer                   */
        DrvError = V850E2_Fx4_CAN_ARG_ERR;
        return (result);
    }
    if (can->Use != 1) {                              /* check, that can device is opened         */
        DrvError = V850E2_Fx4_CAN_OPEN_ERR;
        return (result);
    }
#endif
    
    frm   = (V850E2_Fx4_CANFRM *)buffer;               /* Set pointer to can frame                 */
    reg   = (V850E2_GLOB_MOD_REG *) can->BaseAddrGlobRegs;

    CPU_CRITICAL_ENTER();    
                                                      /*------------------------------------------*/
    hist  = reg->FCNnCMRGRX;                          /* get receive history                      */
    
    if ((hist & 0x02) == 0) {                         /* check if something is in the history     */
        nbr = hist >> 8;                              /* get receive history list pointer         */
    } else {
        nbr = 99;                                     /* set a invalid number                     */
    }
                                                      /*------------------------------------------*/
    if ((nbr >= V850E2_Fx4_TX_MB) &&                  /* range check buffer number                */
        (nbr < can->MaxNumBuf)) {
              
        mb16  = (V850E2_MB_16BIT *)     can->BaseAddrMB16Regs;
        mb8   = (V850E2_MB_8BIT *)      can->BaseAddrMB8Regs;
                
        mb16 += nbr;
        mb8  += nbr;
                                                      /*------------------------------------------*/
        if ((mb16->FCNnMmCTL & 0x0004) != 0) {        /* check for new data                       */
            
                                                      /* Get the Id                               */
            if ((mb16->FCNnMmMID1 & 0x8000) == 0) {   /* is std id                                */
                frm->Identifier = (mb16->FCNnMmMID1 >> 2) & 0x03FF;
            } else {
                frm->Identifier  = (CPU_INT32U) mb16->FCNnMmMID0;
                frm->Identifier |= (CPU_INT32U)(mb16->FCNnMmMID1 & 0x1FFF) << 16;
                frm->Identifier |= V850E2_Fx4_FF_FRAME_BIT;
            }
            if ((mb8->FCNnMmSTRB & 0x04) != 0) {      /* check remote frame                       */
                frm->Identifier |= V850E2_Fx4_RTR_FRAME_BIT;
            }
            frm->DLC = mb8->FCNnMmDTLGB & 0x0F;       /* Get DLC                                  */
              
            for (i=0; i<frm->DLC; i++) {              /* Get the data                             */
                frm->Data[i] = mb8->FCNnMmDATA[i].DATA;
            }
                                                      /*------------------------------------------*/
            mb16->FCNnMmCTL = 0x0000;                 /* clear DTNF bit                           */
            mb16->FCNnMmCTL = 0x0004;
    
            DrvError = V850E2_Fx4_CAN_NO_ERR;
            result = size;
        } else {
            DrvError = V850E2_Fx4_CAN_NO_DATA_ERR;
        }
    } else {
        DrvError = V850E2_Fx4_CAN_NO_DATA_ERR;
    }
                                                      /*------------------------------------------*/
    CPU_CRITICAL_EXIT();
    return result;                                    /* Return function result                   */
}

/*************************************************************************************************/
/*!
* \brief                      CAN WRITE DATA
*
*           Write a can frame to a message buffer. The buffer must contain only one can frame,
*           which will be written to a predefined message buffer.
*
*
* \param paraId parameter identifier, returned by V850E2_Fx4CANOpen()
* \param buffer Pointer to can frame
* \param size   Length of can frame memory
* \return errorcode (0 if ok -1 if an error occured)
*/
/*************************************************************************************************/
CPU_INT16S V850E2_Fx4CANWrite (CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size)
{
    V850E2_Fx4_CANFRM    *frm;                        /* Local: Pointer to can frame              */
    CPU_INT16S            result = -1;                /* Local: return value                      */
    CPU_INT08U            mb;                         /* Local: transmit mailbox number           */
    CPU_INT32U            i;                          /* Local: loop variable                     */
    V850E2_MB_16BIT      *mb16;                       /* Local: message box 16 bit format         */
    V850E2_MB_8BIT       *mb8;                        /* Local: message box 8 bit format          */
    V850E2_Fx4_CAN_DATA  *can;                        /* Local: Pointer to can device             */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/
    can = &DevData[paraId];                           /* set pointer to can device                */
#if V850E2_Fx4_CAN_ARG_CHK_CFG > 0

    if ((paraId >= V850E2_Fx4_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range      */
        DrvError = V850E2_Fx4_CAN_BUS_ERR;
        return (result);
    }
    if (size != sizeof(V850E2_Fx4_CANFRM)) {          /* check that size is plausible             */
        DrvError = V850E2_Fx4_CAN_NO_DATA_ERR;
        return (result);
    }
    if (buffer == (void *)0) {                        /* invalid buffer pointer                   */
        DrvError = V850E2_Fx4_CAN_ARG_ERR;
        return (result);
    }
    if (can->Use != 1) {                              /* check, that can device is opened         */
        DrvError = V850E2_Fx4_CAN_OPEN_ERR;
        return (result);
    }
#endif
    CPU_CRITICAL_ENTER();
    frm = (V850E2_Fx4_CANFRM *)buffer;                 /* Set pointer to can frame                */
        
    mb16  = (V850E2_MB_16BIT *)     can->BaseAddrMB16Regs;
    mb8   = (V850E2_MB_8BIT *)      can->BaseAddrMB8Regs;
    
    mb = 99;
    for (i=0; i<V850E2_Fx4_TX_MB;i++) {               /* find free tx message buffer              */
        if ((mb16->FCNnMmCTL & 0x0002) == 0) {     /* .. check TRQF                            */
            mb = i;
            break;
        }      
        mb16++;
        mb8++;
    }
    if (mb < V850E2_Fx4_TX_MB) {                      /* if free tx message buffer was found      */
        
        mb16->FCNnMmCTL = 0x0000;                     /* clear RDFY bit to make buffer writeable  */
        mb16->FCNnMmCTL = 0x0001;
        while (mb16->FCNnMmCTL & 1) {};               /* wait until the RDFY is cleared           */
        
        if (frm->Identifier <= 0x7FF) {
             mb16->FCNnMmMID0  = 0x0;
             mb16->FCNnMmMID1  = frm->Identifier << 2;
        } else {
             mb16->FCNnMmMID0  = (CPU_INT16U) frm->Identifier;
             mb16->FCNnMmMID1  = (CPU_INT16U) ((frm->Identifier >> 16) & 0x1FFF);
             mb16->FCNnMmMID1 |= 0x8000;
        }
                                                      /* Set up the DLC                           */
        mb8->FCNnMmDTLGB = frm->DLC;
                                                      /* Set up the data field                    */
        for (i=0; i<frm->DLC; i++) {
            mb8->FCNnMmDATA[i].DATA = frm->Data[i];
        }
        mb16->FCNnMmCTL = 0x0100;                     /* set RDFY bit to make buffer operational  */                                                      
        mb16->FCNnMmCTL = 0x0200;                     /* Request transmission                     */

        result   = size;
    } else {
        CPU_CRITICAL_EXIT();
        DrvError = V850E2_Fx4_CAN_BUSY_ERR;
        return result;
    }

    CPU_CRITICAL_EXIT();
                                                      /*------------------------------------------*/
    return result;                                    /* Return function result                   */
}



/*! } */

