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
#include "drv_can.h"                                  /* driver declarations                      */
#include "drv_def.h"                                  /* driver layer declarations                */
#include "can_bsp.h"                                  /* user definable definitions               */


/*
****************************************************************************************************
*                                             DEFINES
****************************************************************************************************
*/

/*
****************************************************************************************************
*                                              MACROS
****************************************************************************************************
*/

/*
****************************************************************************************************
*                                            LOCAL DATA
****************************************************************************************************
*/

/*
****************************************************************************************************
*                                           GLOBAL DATA
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER ERRORCODE
* \ingroup  ADSPBF537_CAN
*
*           This variable holds the detailed errorcode, if an error is detected.
*/
/*------------------------------------------------------------------------------------------------*/
static CPU_INT16U DrvError;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER IDENTIFICATION
* \ingroup  ADSPBF537_CAN
*
*           This constant variable holds the unique driver identification code.
*/
/*------------------------------------------------------------------------------------------------*/
static const CPU_INT32U DrvIdent = 0x243F2B01;   /* version 2.4.0  ID 2B01 */ 


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER RUNTIME DATA
* \ingroup  ADSPBF537_CAN
*
*           Can table.
*/
/*------------------------------------------------------------------------------------------------*/
static CAN_DATA CanTbl[ADSPBF537_CAN_N_DEV];

/*
****************************************************************************************************
*                                       LOCAL FUNCTIONS
****************************************************************************************************
*/


/*
****************************************************************************************************
*                                            FUNCTIONS
****************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN INITIALISATION
* \ingroup  ADSPBF537_CAN
*
*           Initializes the CAN module.
*
* \param    arg               identifies can device
*
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S ADSPBF537CANInit (CPU_INT32U arg)
{
    CPU_INT16S     result = ADSPBF537_CAN_NO_ERR;     /* Local: Function result                   */
    CPU_INT32U     i;                                 /* Local: loop variable                     */
    CPU_INT32U     mask;                              /* Local: mask variable                     */
    CAN_ADSPBF537 *ADSPBF537;                         /* Local: pointer to CAN device             */
                                                      /*------------------------------------------*/
#if ADSPBF537_CAN_ARG_CHK_CFG > 0
    if ((CPU_INT08U)arg >= ADSPBF537_CAN_N_DEV) {     /* check that device name is in range       */
        DrvError= ADSPBF537_CAN_INIT_ERR;
        return(-1);                                   /* return function result                   */
    }
#endif

    CanTbl[arg].BaseAddr = ADSPBF537_CAN0_BASE_ADDR;

    DrvError= ADSPBF537_CAN_NO_ERR;                   /* set to defined value                     */
    CanTbl[arg].Use = 0;
    ADSPBF537 = (CAN_ADSPBF537 *) CanTbl[arg].BaseAddr;
                                                      /*------------------------------------------*/
    ADSPBF537CANPinSettings(arg);
                                                      /*------------------------------------------*/
    ADSPBF537->CONTROL = CAN_CONTROL_CCR;             /* CAN Configuration Mode Request           */
    while ((ADSPBF537->STATUS & CAN_STATUS_CCA) == 0) {};

                                                      /*------------------------------------------*/
                                                      /* Initialize baud rate parameter           */
    CanTbl[arg].Baudrate        = ADSPBF537CAN_DEF_BAUD_RATE;
    CanTbl[arg].SamplePoint     = ADSPBF537CAN_DEF_SP;
    CanTbl[arg].ResynchJumpWith = ADSPBF537CAN_DEF_RJW;

    result = ADSPBF537CAN_CalcTimingReg (&CanTbl[arg]);
    if (result != ADSPBF537_CAN_NO_ERR) {
        DrvError = ADSPBF537_CAN_INIT_ERR;
        result   = ADSPBF537_CAN_INIT_ERR;
    } else {

                                                      /* Set the bit rate register                */
        ADSPBF537->CLOCK = (CPU_INT16U)(CanTbl[arg].PRESDIV);

        ADSPBF537->TIMING = (CPU_INT16U)(CanTbl[arg].RJW   << 8)  |
                            (CanTbl[arg].PSEG1)  |
                            (CanTbl[arg].PSEG2 << 4);
        ssync();
    }

                                                      /*------------------------------------------*/
                                                      /* Setup complete messagebox area           */
    for (i=0; i<32; i++) {
        ADSPBF537->MB[i].DATA0      = 0;
        ADSPBF537->MB[i].DATA1      = 0;
        ADSPBF537->MB[i].DATA2      = 0;
        ADSPBF537->MB[i].DATA3      = 0;
        ADSPBF537->MB[i].LENGTH     = 8;
        ADSPBF537->MB[i].TIMESTAMP  = 0;
        ADSPBF537->MB[i].ID0        = 0;
        ADSPBF537->MB[i].ID1        = 0;
    } 
    mask = 0;
    for (i=0; i<ADSPBF537_CAN_SIZE_RX_MB+1; i++) {
        mask |= (1 << i);
        ADSPBF537->MB[i].ID0        = 0x0000;
        ADSPBF537->MB[i].ID1        = CAN_MB_ID1_AME; /* Set AME (Acceptance Masks Enable) bit    */
        ADSPBF537->MB_ACC[i].AM_x_L = 0xFFFF;         /* Set Acceptance Masks to accept all       */
        ADSPBF537->MB_ACC[i].AM_x_H = 0x3FFF;
    }
    ADSPBF537->OPSS1 = 0xFFFF;                        /* enable overwrite protection for rx mbox  */
                                                      /* Setup Mailbox Direction Register         */
    ADSPBF537->MD1 = (mask & 0x0000FFFF);
    ADSPBF537->MD2 = (mask >> 16);

                                                      /* Enable the  Mailboxes                    */
    ADSPBF537->MC1 = 0x0000FFFF;                      /* mailboxes 0-15                           */
    ADSPBF537->MC2 = 0x0000FFFF;                      /* mailboxes 16-31                          */
        
                                                      /*------------------------------------------*/
#if ADSPBF537_CAN_INTERRUPT_EN > 0                    /* INTERRUPT Settings                       */
    ADSPBF537->MBIM1  = 0xFFFF;                       /* enable interrupts for all mboxes         */
    ADSPBF537->MBIM2  = 0xFFFF;
    ADSPBF537->MBTIF2 = 0xFFFF;                       /* clear all pending interrupts             */
    ADSPBF537->MBRIF1 = 0xFFFF;
                                                      /* Enable error interrupts                  */
    ADSPBF537->GIM    = (CAN_GIM_EPIM |
                         CAN_GIM_BOIM); 
    ADSPBF537->GIS    = 0x000F;                       /* Clear error interrupts                   */
    ADSPBF537CANIrqSettings (arg);
#endif

    return(result);                                   /* return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      OPEN THE CAN BUS
* \ingroup  ADSPBF537_CAN
*
*           Open the CAN Bus
*
* \param    devId             unused
* \param    devName           the bus id
* \param    mode              unused
*
* \return   parameter identifier for further access or -1 if an error occurs
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S ADSPBF537CANOpen(CPU_INT16S devId, CPU_INT32U devName, CPU_INT16U mode)
{
    CPU_INT16S result = -1;                           /* Local: Function result                   */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/

    mode  = mode;
#if ADSPBF537_CAN_ARG_CHK_CFG > 0
    if ((CPU_INT08U)devName >= ADSPBF537_CAN_N_DEV) {  /* check that device name is in range       */
        DrvError= ADSPBF537_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
    if (mode != DEV_RW) {                             /* mode not supported?                      */
        DrvError = ADSPBF537_CAN_MODE_ERR;
        return(result);
    }
#endif

#if (ADSPBF537_CAN_INTERRUPT_EN > 0)
                                                      /* store the received Node Id for the irqs  */
    ADSPBF537_BSP_SetDevIds ((CPU_INT08U) devId,
                         (CPU_INT08U) devName);
#else
    devId = devId;                                    /* prevent compiler warning                 */
#endif

    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    if (CanTbl[devName].Use == 0) {                   /* Check, that device is not in use         */
        CanTbl[devName].Use = 1;                      /* mark can device as used                  */
        result = devName;                             /* Okay, device is opened                   */
    } else {
        DrvError= ADSPBF537_CAN_OPEN_ERR;             /* not opened - set open error              */
    }
                                                      /*------------------------------------------*/
    
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return(result);                                   /* return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CLOSE THE CAN BUS
* \ingroup  ADSPBF537_CAN
*
*           Close the CAN bus
*
* \param    paraId            unused
*
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S ADSPBF537CANClose(CPU_INT16S paraId)
{
    CPU_INT16S result = -1;                           /* Local: Function result                   */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/

#if ADSPBF537_CAN_ARG_CHK_CFG > 0
                                                      /* check that paraId is in range            */
    if ((paraId >= ADSPBF537_CAN_N_DEV) || (paraId < 0)) {
        DrvError= ADSPBF537_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
#endif
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */                                                      /*------------------------------------------*/
    if (CanTbl[paraId].Use == 1) {                    /* Check, that device is in use             */
        CanTbl[paraId].Use = 0;                       /* mark can device as unused                */
        result = 0;                                   /* Okay, device is closed                   */
    } else {
        DrvError= ADSPBF537_CAN_CLOSE_ERR;             /* not opened - set close error             */
    }
                                                      /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return(result);                                   /* return function result                   */
}


/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN I/O CONTROL
* \ingroup  ADSPBF537_CAN
*
*           This function performs a special action on the opened device. The functioncode 'func'
*           defines what the caller want to do. Description of functioncodes as defined in
*           headerfile.
*
* \param    paraId            unused
* \param    func              function code
* \param    arg               argument list, specific to the function code
*
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S ADSPBF537CANIoCtl (CPU_INT16S paraId, CPU_INT16U func, void* argp)
{
    CPU_INT32U      i;                                /* Local: counter variable                  */
    CPU_INT32U      msg_obj;                          /* Local: message object                    */
    CPU_INT32U      msg_if = 0;                       /* Local: message interface                 */
    CPU_INT32U      canId;                            /* Local: CAN Identifier                    */
    CPU_INT32U      mask;                             /* Local: Mask for identifier caluclations  */
    CPU_INT16U      reg;                              /* Local: register value                    */
    CPU_INT16S      result  = -1;                     /* Local: Function result                   */
    CAN_ADSPBF537  *ADSPBF537;                        /* Local: pointer to CAN device             */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/
    ADSPBF537 = (CAN_ADSPBF537    *)CanTbl[paraId].BaseAddr;


#if ADSPBF537_CAN_ARG_CHK_CFG > 0
    if ((paraId >= ADSPBF537_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range       */
        DrvError= ADSPBF537_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
    if (CanTbl[paraId].Use != 1) {                    /* check, that can device is opened         */
        DrvError= ADSPBF537_CAN_OPEN_ERR;
        return(result);                               /* return function result                   */
    }
#endif

    result = ADSPBF537_CAN_NO_ERR;
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */
    switch (func) {                                   /* select: function code                    */
        case IO_ADSPBF537_CAN_GET_IDENT:              /* GET IDENT                                */
            (*(CPU_INT32U*)argp) = DrvIdent;          /* return driver ident code                 */
            break;
                                                      /*------------------------------------------*/
        case IO_ADSPBF537_CAN_GET_ERRNO:              /* GET ERRORCODE                            */
            (*(CPU_INT16U*)argp) = DrvError;          /* return last detected errorcode           */
            break;
                                                      /*------------------------------------------*/
        case IO_ADSPBF537_CAN_GET_DRVNAME:            /* GET DRIVER NAME                          */
            (*(CPU_INT08U**)argp) = (CPU_INT08U*)     /* return human readable driver name        */
                                   ADSPBF537_CAN_NAME;
            break;
                                                      /*------------------------------------------*/
        case IO_ADSPBF537_CAN_SET_BAUDRATE:           /* Function: Set can bus baudrate           */
                                                      /*------------------------------------------*/

            reg = ADSPBF537->CONTROL;                 /* store register value                     */
            ADSPBF537->CONTROL = CAN_CONTROL_CCR;     /* CAN Configuration Mode Request           */
                                                      /* wait untile Config mode is established   */
            while ((ADSPBF537->STATUS & CAN_STATUS_CCA) == 0) {};

                                                      /* Initialize baud rate parameter           */
            CanTbl[paraId].Baudrate = *(CPU_INT32U *)argp;

            result = ADSPBF537CAN_CalcTimingReg (&CanTbl[paraId]);
            if (result != ADSPBF537_CAN_NO_ERR) {
                DrvError = ADSPBF537_CAN_INIT_ERR;
                result   = ADSPBF537_CAN_INIT_ERR;
            } else {

                                                      /* Set the bit rate register                */
                ADSPBF537->CLOCK = (CPU_INT16U)(CanTbl[paraId].PRESDIV);

                ADSPBF537->TIMING = (CPU_INT16U)(CanTbl[paraId].RJW   << 8)  |
                                    (CanTbl[paraId].PSEG1)  |
                                    (CanTbl[paraId].PSEG2 << 4);
                ssync();
            }
            ADSPBF537->CONTROL = reg;                 /* restore rgister value                    */
            
            break;                                    /*------------------------------------------*/
        case IO_ADSPBF537_CAN_START:                  /* Function: Start can bus communication    */
                                                      /*------------------------------------------*/
            ADSPBF537->CONTROL = 0;                   /* CAN normal operation mode                */
            while ((ADSPBF537->STATUS & CAN_STATUS_CCA) != 0) {};

#if ADSPBF537_CAN_INTERRUPT_EN > 0
            ADSPBF537->MBIM1  = 0xFFFF;               /* anable interrupts for all mboxes         */
            ADSPBF537->MBIM2  = 0xFFFF;
            ADSPBF537->MBTIF2 = 0xFFFF;               /* clear all pending interrupts             */
            ADSPBF537->MBRIF1 = 0xFFFF;            
#endif
          break;                                      /*------------------------------------------*/
        case IO_ADSPBF537_CAN_STOP:                   /* Function: Stop can bus communication     */
                                                      /*------------------------------------------*/
            ADSPBF537->CONTROL = CAN_CONTROL_CCR;     /* CAN Configuration Mode Request           */
                                                      /* wait untile Config mode is established   */
            while ((ADSPBF537->STATUS & CAN_STATUS_CCA) == 0) {};

            ADSPBF537->MBTIF2 = 0xFFFF;               /* clear all pending interrupts             */
            ADSPBF537->MBRIF1 = 0xFFFF;
            
            ADSPBF537->MBIM1 = 0x0;                   /* disable interrupts for all mboxes        */
            ADSPBF537->MBIM2 = 0x0;
            
            
            break;                                    /*------------------------------------------*/

        case IO_ADSPBF537_CAN_RX_STANDARD:            /* Function: receive standard format ids    */
                                                      /*------------------------------------------*/
                                                      
            ADSPBF537->CONTROL = CAN_CONTROL_CCR;     /* CAN Configuration Mode Request           */
                                                      /* wait untile Config mode is established   */
            while ((ADSPBF537->STATUS & CAN_STATUS_CCA) == 0) {};
                                                      
            for (i=0; i<ADSPBF537_CAN_SIZE_RX_MB+1; i++) {
                                                      /* set AME bit to check dedicated bits      */
                ADSPBF537->MB[i].ID1  = CAN_MB_ID1_AME;
                                                      /* clear IDE to receive only standard IDs   */
                ADSPBF537->MB[i].ID1 &= (CPU_INT16U) ~CAN_MB_ID1_IDE;
                                                      /* clear AMIDE bit to get IDE bit checked   */
                ADSPBF537->MB_ACC[i].AM_x_H &= (CPU_INT16U)~CAN_AMxx_AMIDE;
            }
            
            ADSPBF537->CONTROL = 0;                   /* CAN normal operation mode                */
            while ((ADSPBF537->STATUS & CAN_STATUS_CCA) != 0) {};

            break;                                    /*------------------------------------------*/
        case IO_ADSPBF537_CAN_RX_EXTENDED:            /* Function: receive extended format ids    */
                                                      /*------------------------------------------*/
                                                      
            ADSPBF537->CONTROL = CAN_CONTROL_CCR;     /* CAN Configuration Mode Request           */
                                                      /* wait untile Config mode is established   */
            while ((ADSPBF537->STATUS & CAN_STATUS_CCA) == 0) {};
                                                      
            for (i=0; i<ADSPBF537_CAN_SIZE_RX_MB+1; i++) {
                                                      /* set AME bit to check dedicated bits      */
                ADSPBF537->MB[i].ID1  = CAN_MB_ID1_AME;
                                                      /* set IDE to receive only extended IDs     */
                ADSPBF537->MB[i].ID1 |= CAN_MB_ID1_IDE;
                                                      /* clear AMIDE bit to get IDE bit checked   */
                ADSPBF537->MB_ACC[i].AM_x_H &= (CPU_INT16U)~CAN_AMxx_AMIDE;
            }
            
            ADSPBF537->CONTROL = 0;                   /* CAN normal operation mode                */
            while ((ADSPBF537->STATUS & CAN_STATUS_CCA) != 0) {};
            
            break;                                    /*------------------------------------------*/
        case IO_ADSPBF537_CAN_TX_READY:

                                                      /* check availability of tx buffer          */            
            if ((ADSPBF537->TRS2) == 0xFFFF) {
                *((CPU_INT08U *)argp) = 0;            /* Transmit Channel is not available        */
            } else {
                *((CPU_INT08U *)argp) = 1;            /* Transmit Channel is available            */
            }
            
    
            break;                                    /*------------------------------------------*/
        case IO_ADSPBF537_CAN_GET_NODE_STATUS:        /* Function: Get Node Status                */
                                                      /*------------------------------------------*/
            reg = ADSPBF537->STATUS;
            
            if ((reg & CAN_STATUS_EP) != 0) {         /* Error Status Bit                         */
                *((CPU_INT08U *)argp) = 1;
            }
            if ((reg & CAN_STATUS_EBO) != 0) {        /* Bus Off Bit                              */
                *((CPU_INT08U *)argp) = 2;
            }
                                                      /* not set - bus active                     */
            if ((reg & (CAN_STATUS_EP | CAN_STATUS_EBO)) == 0) {
                *((CPU_INT08U *)argp) = 0;
            }            
            
            break;                                    /*------------------------------------------*/
         case IO_ADSPBF537_CAN_SET_RX_FILTER:         /* SET RX FILTER                            */
                                                      /*------------------------------------------*/
            mask  = ~((CPU_INT32U*)argp)[0];
            canId = ((CPU_INT32U*)argp)[1];
            
            ADSPBF537->CONTROL = CAN_CONTROL_CCR;     /* CAN Configuration Mode Request           */
                                                      /* wait untile Config mode is established   */
            while ((ADSPBF537->STATUS & CAN_STATUS_CCA) == 0) {};
                                                      
            for (i=0; i<ADSPBF537_CAN_SIZE_RX_MB+1; i++) {
                                                      /* Set CAN Identifier and mask              */
                if (canId > 0x7FF) {                  /* if Id is extended Id                     */
                    ADSPBF537->MB[i].ID0 = (CPU_INT16U)(canId & 0xFFFF);
                    ADSPBF537->MB[i].ID1 = (CPU_INT16U)((canId & 0x1FFF0000) >> 16);
                    ADSPBF537->MB[i].ID1 |= CAN_MB_ID1_IDE;
                

                    ADSPBF537->MB_ACC[i].AM_x_L = (CPU_INT16U)(mask & 0xFFFF);
                    ADSPBF537->MB_ACC[i].AM_x_H = (CPU_INT16U)((mask & 0x1FFF0000) >> 16);
                    ADSPBF537->MB_ACC[i].AM_x_H |= CAN_AMxx_AMIDE;

                } else {                              /* standard ID                              */
                    ADSPBF537->MB[i].ID1 = (CPU_INT16U) (canId << 2);

                    ADSPBF537->MB_ACC[i].AM_x_H  = (CPU_INT16U) (mask << 2);
                }
            }
            
            ADSPBF537->CONTROL = 0;                   /* CAN normal operation mode                */
            while ((ADSPBF537->STATUS & CAN_STATUS_CCA) != 0) {};
           

            break;                                    /*------------------------------------------*/
        default:
            DrvError = ADSPBF537_CAN_FUNC_ERR;
            result = -1;
            break;

    }                                                 /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return result;                                    /* return function result                   */
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN READ DATA
* \ingroup  ADSPBF537_CAN
*
*           Read a received message from a message buffer after checking, that this
*           device is opened.
*
* \param    paraId            unused, but needed for common interface
* \param    buffer            byte array for received data
* \param    size              length of byte array
*
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S ADSPBF537CANRead (CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size)
{
    ADSPBF537_CANFRM *frm;                            /* Local: Pointer to can frame              */
    CPU_INT16S        result = -1;                    /* Local: Result of function                */
    CPU_INT32U        i;                              /* Local: loop variable                     */
    CPU_INT32U        mb_status;                      /* Local: message box status                */
    CPU_INT32U        mask;                           /* Local: bit mask                          */
    CPU_INT32U        bit_pos = 0;                    /* Local: bit position                      */
    CPU_INT32U        mbID;                           /* Local: message interface                 */
    
    CAN_ADSPBF537    *ADSPBF537;                      /* Local: pointer to CAN device             */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/
    ADSPBF537 = (CAN_ADSPBF537    *)CanTbl[paraId].BaseAddr;

#if ADSPBF537_CAN_ARG_CHK_CFG > 0

    if ((paraId >= ADSPBF537_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range        */
        DrvError= ADSPBF537_CAN_BUS_ERR;
        return(result);
    }
    if (size != sizeof(ADSPBF537_CANFRM)) {            /* check that size is plausible             */
        DrvError = ADSPBF537_CAN_NO_DATA_ERR;
        return(result);
    }
    if (buffer == 0) {                                /* invalid buffer pointer                   */
        DrvError= ADSPBF537_CAN_ARG_ERR;
        return(result);
    }
    if (CanTbl[paraId].Use != 1) {                    /* check, that can device is opened         */
        DrvError= ADSPBF537_CAN_OPEN_ERR;
        return(result);
    }
#endif
    CPU_CRITICAL_ENTER();                             /* enter critical section                   */

                                                      /*------------------------------------------*/
    mb_status = ADSPBF537->RMP1;                      /* search for pending message box           */
    mask      = 1;
    while (((mb_status & mask) == 0) && 
            (bit_pos < 16)){
        mask <<= 1;
        bit_pos++;
    }
    if (bit_pos == 16) {                              /* check if message was found               */
        DrvError = ADSPBF537_CAN_NO_DATA_ERR;
        CPU_CRITICAL_EXIT();                          /* exit critical section                    */
        return ADSPBF537_CAN_NO_DATA_ERR;    
    }

    mbID = bit_pos;

    frm = (ADSPBF537_CANFRM *)buffer;                 /* Set pointer to can frame                 */

    
                                                      /*------------------------------------------*/
                                                      /* get standard Identifier                  */
    if ((ADSPBF537->MB[mbID].ID1  & CAN_MB_ID1_IDE) == 0) { 
        frm->Identifier = (ADSPBF537->MB[mbID].ID1 >> 2) & 0x07FF;
    } else {
                                                      /* if IDE bit is set add extended ID bits   */
        frm->Identifier = ADSPBF537->MB[mbID].ID0;
        frm->Identifier |= ((ADSPBF537->MB[mbID].ID1 & 0x1FFF) << 16);
                                                      /* mark extended frame                      */
        frm->Identifier |= ADSPBF537_CAN_FF_FRAME_BIT;
    }
                                                      /*------------------------------------------*/
                                                      /* get RTR bit                              */
    if ((ADSPBF537->MB[mbID].ID1 & CAN_MB_ID1_RTR) != 0) {
        frm->Identifier |= ADSPBF537_CAN_RTR_FRAME_BIT;        
    }

    frm->DLC = ADSPBF537->MB[mbID].LENGTH & 0x0F;

    frm->Data[0] = (CPU_INT08U)(ADSPBF537->MB[mbID].DATA3 >> 8);
    frm->Data[1] = (CPU_INT08U) ADSPBF537->MB[mbID].DATA3;
    frm->Data[2] = (CPU_INT08U)(ADSPBF537->MB[mbID].DATA2 >> 8);
    frm->Data[3] = (CPU_INT08U) ADSPBF537->MB[mbID].DATA2;
    frm->Data[4] = (CPU_INT08U)(ADSPBF537->MB[mbID].DATA1 >> 8);
    frm->Data[5] = (CPU_INT08U) ADSPBF537->MB[mbID].DATA1;
    frm->Data[6] = (CPU_INT08U)(ADSPBF537->MB[mbID].DATA0 >> 8);
    frm->Data[7] = (CPU_INT08U) ADSPBF537->MB[mbID].DATA0;

    ADSPBF537->RMP1 = mask;                           /* write 1 to bit to clear bit              */
        
    result = size;                                    /* set successfull result                   */
        
                                                      /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return(result);                                   /* Return function result                   */
}




/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      CAN WRITE DATA
* \ingroup  ADSPBF537_CAN
*
*           Write a message to a message buffer after checking, that this
*           device is opened.
*
* \param    paraId            unused, but needed for common interface
* \param    buffer            byte array for transmitting data
* \param    size              length of byte array
*
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*------------------------------------------------------------------------------------------------*/
CPU_INT16S ADSPBF537CANWrite (CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size)
{
    ADSPBF537_CANFRM *frm;                            /* Local: Pointer to can frame              */
    CPU_INT16S        result = -1;                    /* Local: Result of function                */
    CPU_INT32U        cnt    = 0;                     /* Local: Result of function                */
    CPU_INT32U        mbID;                           /* Local: message box number                */
    CPU_INT32U        bit_pos = 0;                    /* Local: bit position                      */
    CPU_INT32U        mask;                           /* Local: bit mask                          */
    CPU_INT32U        mb_status;                      /* Local: message box status                */
    CAN_ADSPBF537    *ADSPBF537;                      /* Local: pointer to CAN device             */
    CPU_SR_ALLOC();                                   /* Allocate storage for CPU status reg.     */
                                                      /*------------------------------------------*/
    ADSPBF537 = (CAN_ADSPBF537    *)CanTbl[paraId].BaseAddr;

#if ADSPBF537_CAN_ARG_CHK_CFG > 0

    if ((paraId >= ADSPBF537_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range        */
        DrvError= ADSPBF537_CAN_BUS_ERR;
        return(result);
    }
    if (size != sizeof(ADSPBF537_CANFRM)) {            /* check that size is plausible             */
        DrvError = ADSPBF537_CAN_NO_DATA_ERR;
        return(result);
    }
    if (buffer == 0) {                                /* invalid buffer pointer                   */
        DrvError= ADSPBF537_CAN_ARG_ERR;
        return(result);
    }
    if (CanTbl[paraId].Use != 1) {                    /* check, that can device is opened         */
        DrvError= ADSPBF537_CAN_OPEN_ERR;
        return(result);
    }
#endif

    CPU_CRITICAL_ENTER();                             /* enter critical section                   */

                                                      /* check availability of tx buffer          */
    mb_status = ADSPBF537->TRS2;
    mask      = 1;
    while (((mb_status & mask) != 0) &&               /* ready when set to 0                      */
            (bit_pos < 16)) {                                 
        mask <<= 1;
        bit_pos++;
    }
    
    if (bit_pos == 16) {                              /* check if m box is available              */
        DrvError = ADSPBF537_CAN_BUSY_ERR;
        CPU_CRITICAL_EXIT();                          /* exit critical section                    */
        return ADSPBF537_CAN_BUSY_ERR;    
    }
    mbID = bit_pos + ADSPBF537_CAN_SIZE_RX_MB + 1;    /* Select TX mbox ID                        */
    
                                                      
    frm = (ADSPBF537_CANFRM *)buffer;                 /* Set pointer to can frame                 */    

                                                      /* settings for standard or extended id     */
    if (frm->Identifier > 0x7FF) {                    /* if Id is extended Id                     */
        ADSPBF537->MB[mbID].ID0 = (CPU_INT16U)(frm->Identifier & 0xFFFF);
        ADSPBF537->MB[mbID].ID1 = (CPU_INT16U)((frm->Identifier & 0x1FFF0000) >> 16);
        ADSPBF537->MB[mbID].ID1 |= CAN_MB_ID1_IDE;

    } else {                                          /* standard ID                              */
        ADSPBF537->MB[mbID].ID1 = (frm->Identifier << 2);
    }

    ADSPBF537->MB[mbID].LENGTH = frm->DLC;            /* set data length code                     */

                                                      /* copy data to msg object via interface reg */
    ADSPBF537->MB[mbID].DATA3 = ((CPU_INT16U)frm->Data[0]<<8) | frm->Data[1];
    ADSPBF537->MB[mbID].DATA2 = ((CPU_INT16U)frm->Data[2]<<8) | frm->Data[3];
    ADSPBF537->MB[mbID].DATA1 = ((CPU_INT16U)frm->Data[4]<<8) | frm->Data[5];
    ADSPBF537->MB[mbID].DATA0 = ((CPU_INT16U)frm->Data[6]<<8) | frm->Data[7];

                                                      /* execute request to update msg object     */
    ADSPBF537->TRS2 |= mask;
                                                      
    result = size;                                    /* set successfull result                   */
                                                      /*------------------------------------------*/
    CPU_CRITICAL_EXIT();                              /* exit critical section                    */
    return(result);                                   /* Return function result                   */
}

/*! } */

