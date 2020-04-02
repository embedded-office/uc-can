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
#include "can_bus.h"


/*
***************************************************************************************************
*                            CONSTANTS
***************************************************************************************************
*/


#if LPC24XX_CAN_FILTER_EN > 0
  #if LPC24XX_CAN_STD_FILTER_SIZE > 0
extern const CPU_INT32U StdFilter_Tbl[];
  #endif

  #if LPC24XX_CAN_STD_GROUP_FILTER_SIZE > 0
extern const CPU_INT32U StdGroupFilter_Tbl[];
  #endif

  #if LPC24XX_CAN_EXT_FILTER_SIZE > 0
extern const CPU_INT32U ExtFilter_Tbl[];
  #endif

  #if LPC24XX_CAN_EXT_GROUP_FILTER_SIZE > 0
extern const CPU_INT32U ExtGroupFilter_Tbl[];
  #endif

volatile CPU_INT32U *AcceptanceTblPtr;                /* Pointer to can acceptance filter           */

#endif /* can acceptance filter */

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER IDENTIFICATION
* \ingroup  LPC24XX_CAN
*
*           This constant variable holds the unique driver identification code.
*/
/*------------------------------------------------------------------------------------------------*/
static const CPU_INT32U DrvIdent = 0x243F1521;

/*
***************************************************************************************************
*                            GLOBAL DATA
***************************************************************************************************
*/

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER ERRORCODE
* \ingroup  LPC24XX_CAN
*
*           This variable holds the detailed errorcode, if an error is detected.
*/
/*------------------------------------------------------------------------------------------------*/
static CPU_INT16U DrvError;

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief                      DRIVER RUNTIME DATA
* \ingroup  LPC24XX_CAN
*
*           Global can device data.
*/
/*------------------------------------------------------------------------------------------------*/
static LPC24XX_CAN_DATA DevData[LPC24XX_CAN_N_DEV];

/*
***************************************************************************************************
*                            FUNCTIONs
***************************************************************************************************
*/

/*************************************************************************************************/
/*!
* \brief                      CAN INITIALISATION
*
*           Initializes the LPC24XX CAN controller selected by argument.
*
*
* \param    arg               CAN bus device name
* \return   errorcode         (0 if ok -1 if an error occured)
*/
/*************************************************************************************************/
CPU_INT16S LPC24XXCANInit(CPU_INT32U arg)
{
    CPU_INT16S       result = -1;                     /* Local: Function result                   */
    LPC24XX_CAN_BAUD baud;                            /* Local: Baud rate configuration           */
    CPU_INT32U       val;                             /* Local: help variable                     */

#if LPC24XX_CAN_ARG_CHK_CFG > 0                       /* Checking arguments (if enabled)          */
    if (arg >= LPC24XX_CAN_N_DEV) {                   /* can dev out of range?                    */
        DrvError = LPC24XX_CAN_INIT_ERR;
        return(result);
    }
#endif                                                /*------------------------------------------*/

    DrvError = LPC24XX_CAN_NO_ERR;                    /* reset to 0                               */
    LPC24XX_CAN_AFMR = 0x00000002L;                   /* Acceptance Filter Mode Register =        */
                                                      /*       filter off, receive all            */
    AcceptanceTblPtr = LPC24XX_CAN_ACCEPTANCE_FILTER; /* initialize acceptance table pointer      */

                                                      /*------------------------------------------*/
    if (arg == LPC24XX_CAN_BUS_0) {                   /* Check which can bus to initialise        */
        DevData[LPC24XX_CAN_BUS_0].Use = 0;           /* Set device to unused                     */

        LPC24XX_CAN_PCONP |= 0x00002000L;             /* Switch power to on on CAN 1              */
        LPC24XX_PinSetting(arg);
                                                      /* Set bit 18, select CAN instead of GPIO   */
        LPC24XX_CAN_C1MOD = 1;                        /* Set Reset Mode                           */
        LPC24XX_CAN_C1IER = 0;                        /* Disable All Interrupts                   */
        LPC24XX_CAN_C1GSR = 0;                        /* Clear Status register                    */

        baud.Baudrate        = 1000000L;              /* Default Settings 1MBaud, 75% Sample Point*/
        baud.SamplePoint     = LPC24XX_CAN_DEF_SP;    /* Re-synch jump width 12,5 %               */
        baud.ResynchJumpWith = LPC24XX_CAN_DEF_RJW;
                                                      /*------------------------------------------*/
                                                      /* Calculate the values for the timing      */
        LPC24XX_CalcTimingReg (&baud);                /* register with the parameter setting      */
                                                      /*------------------------------------------*/

        val = (baud.PRESDIV)     |
              (baud.RJW   << 14) |
              (baud.PSEG1 << 16) |
              (baud.PSEG2 << 20);

        LPC24XX_CAN_C1BTR = val;                      /* Set bit timing                           */

        result = LPC24XX_CAN_NO_ERR;

                                                      /*------------------------------------------*/
    } else {
        DevData[LPC24XX_CAN_BUS_1].Use = 0;           /* Set device to unused                     */

        LPC24XX_CAN_PCONP |= 0x00004000L;             /* Switch power to on on CAN 2              */
        LPC24XX_PinSetting(arg);
        LPC24XX_CAN_C2MOD = 1;                        /* Go into Reset mode                       */
        LPC24XX_CAN_C2IER = 0;                        /* Disable All Interrupts                   */
        LPC24XX_CAN_C2GSR = 0;                        /* Clear Status register                    */

        baud.Baudrate        = 1000000L;              /* Default Settings 1MBaud, 75% Sample Point*/
        baud.SamplePoint     = LPC24XX_CAN_DEF_SP;    /* Re-synch jump width 12,5 %               */
        baud.ResynchJumpWith = LPC24XX_CAN_DEF_RJW;
                                                      /*------------------------------------------*/
                                                      /* Calculate the values for the timing      */
        LPC24XX_CalcTimingReg (&baud);                /* register with the parameter setting      */
                                                      /*------------------------------------------*/

        val = (baud.PRESDIV)     |
              (baud.RJW   << 14) |
              (baud.PSEG1 << 16) |
              (baud.PSEG2 << 20);

        LPC24XX_CAN_C2BTR = val;                      /* Set bit timing                           */

        result = LPC24XX_CAN_NO_ERR;
    }

#if (LPC24XX_CAN_RX_INTERRUPT_EN > 0) || \
    (LPC24XX_CAN_TX_INTERRUPT_EN > 0) || \
    (LPC24XX_CAN_NS_INTERRUPT_EN > 0)
    LPC24XX_CANISR_Setting (arg);
#endif

    return result;                                    /* return function result                   */
}


/*************************************************************************************************/
/*!
* \brief                      OPEN THE CAN DEVICE
*
*           Unlocks the device, i.e. IoCtl/Read/Write-function will take effect.
*
*
* \param devId     bus node name which must be used by the interrupt routine to access the
*                  can bus layer.
* \param devName   the CAN device name
* \param mode the mode in which CAN devices will be used
* \return the parameter identifier for further access or -1 if an error occurs
*/
/*************************************************************************************************/
CPU_INT16S LPC24XXCANOpen(CPU_INT16S devId, CPU_INT32U devName, CPU_INT16U mode)
{
    LPC24XX_CAN_DATA *can;                            /* Local: Pointer to can device             */
    CPU_INT16S result = -1;                           /* Local: Function result                   */
    CPU_SR_ALLOC();
                                                      /*------------------------------------------*/
#if LPC24XX_CAN_ARG_CHK_CFG > 0
    if ((CPU_INT08U)devName >= LPC24XX_CAN_N_DEV) {   /* check that device name is in range       */
        DrvError= LPC24XX_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
    if (mode != DEV_RW) {                             /* mode not supported?                      */
        DrvError = LPC24XX_CAN_MODE_ERR;
        return(result);
    }
#endif
    can = &DevData[devName];                          /* set pointer to can device                */
    CPU_CRITICAL_ENTER();
    if (can->Use == 0) {                              /* check, that can device is unused         */
        can->Use = 1;                                 /* mark can device as used                  */
#if ((LPC24XX_CAN_RX_INTERRUPT_EN > 0) || \
     (LPC24XX_CAN_TX_INTERRUPT_EN > 0) || \
     (LPC24XX_CAN_NS_INTERRUPT_EN > 0))
                                                      /* store the received Node Id for the irqs  */
    LPC24XX_BSP_SetDevIds ((CPU_INT08U) devId,
                           (CPU_INT08U) devName);
#else
    devId = devId;                                    /* prevent compiler warning                 */
#endif

        result = (CPU_INT08U)devName;                 /* Okay, device is opened                   */
    } else {
        DrvError= LPC24XX_CAN_OPEN_ERR;
    }
    CPU_CRITICAL_EXIT();

    return(result);                                   /* return function result                   */
}


/*************************************************************************************************/
/*!
* \brief                      CLOSE THE CAN DEVICE
*
*           Locks the device, i.e. IoCtl/Read/Write-function will have no effect.
*
*
* \param paraId the parameter identifier, returned by LPC24XXCANOpen()
* \return errorcode (0 if ok -1 if an error occured)
*/
/*************************************************************************************************/
CPU_INT16S LPC24XXCANClose(CPU_INT16S paraId)
{
    LPC24XX_CAN_DATA *can;                            /* Local: Pointer to can device             */
    CPU_INT16S result = -1;                           /* Local: Function result                   */
    CPU_SR_ALLOC();
                                                      /*------------------------------------------*/
#if LPC24XX_CAN_ARG_CHK_CFG > 0
    if ((paraId >= LPC24XX_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range         */
        DrvError= LPC24XX_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
#endif
    can = &DevData[paraId];                           /* Set pointer to can device                */
    CPU_CRITICAL_ENTER();
    if (can->Use != 0) {                              /* check, that can device is used           */
        can->Use = 0;                                 /* mark can device as unused                */
        result = LPC24XX_CAN_NO_ERR;                  /* Indicate sucessfull function execution   */
    } else {
        DrvError= LPC24XX_CAN_CLOSE_ERR;
    }
    CPU_CRITICAL_EXIT();

    return(result);                                   /* return function result                   */
}

/*************************************************************************************************/
/*!
* \brief                      CAN I/O CONTROL
*
*           This function performs a special action on the opened device. The functioncode func
*           defines what the caller want to do. Description of functioncodes as defined in headerfile
*
*
* \param paraId parameter identifier, returned by LPC24XXCANOpen()
* \param func function code
* \param arg argument list, specific to the function code
* \return errorcode (0 if ok -1 if an error occured)
*/
/*************************************************************************************************/
CPU_INT16S LPC24XXCANIoCtl(CPU_INT16S paraId, CPU_INT16U func, void *argp)
{
    LPC24XX_CAN_DATA *can;                            /* Local: Pointer to can device             */
    CPU_INT16S result = -1;                           /* Local: Function result                   */
    CPU_INT16S i;                                     /* Local: loop variable                     */
    CPU_INT32U status_reg;                            /* Local: register value                    */
    LPC24XX_CAN_BAUD baud;                            /* Local: Baud rate configuration           */
    CPU_INT32U       val;                             /* Local: help variable                     */
    CPU_SR_ALLOC();
                                                      /*------------------------------------------*/
#if LPC24XX_CAN_ARG_CHK_CFG > 0
    if ((paraId >= LPC24XX_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range         */
        DrvError= LPC24XX_CAN_BUS_ERR;
        return(result);                               /* return function result                   */
    }
    can = &DevData[paraId];                           /* set pointer to can device                */
    if (can->Use != 1) {                              /* check, that can device is opened         */
        DrvError= LPC24XX_CAN_OPEN_ERR;
        return(result);                               /* return function result                   */
    }
#endif
    CPU_CRITICAL_ENTER();
                                                      /*------------------------------------------*/
    switch (func) {                                   /* select: function code                    */
                                                      /*------------------------------------------*/
        case IO_LPC24XX_CAN_GET_IDENT:                /* GET IDENT                                */
            (*(CPU_INT32U*)argp) = DrvIdent;          /* return driver ident code                 */
            result = LPC24XX_CAN_NO_ERR;              /* Indicate sucessfull function execution   */
            break;
                                                      /*------------------------------------------*/
        case IO_LPC24XX_CAN_GET_ERRNO:                /* GET ERRORCODE                            */
            (*(CPU_INT16U*)argp) = DrvError;          /* return last detected errorcode           */
            result = LPC24XX_CAN_NO_ERR;              /* Indicate sucessfull function execution   */
            break;
                                                      /*------------------------------------------*/
        case IO_LPC24XX_CAN_GET_DRVNAME:              /* GET DRIVER NAME                          */
            (*(CPU_INT08U**)argp) = (CPU_INT08U*)
                                   LPC24XX_CAN_NAME;
                                                      /* return human readable driver name        */
            result = LPC24XX_CAN_NO_ERR;              /* Indicate sucessfull function execution   */
            break;

        case IO_LPC24XX_CAN_SET_BAUDRATE:             /* Function: Set can bus baudrate           */

            val = (*((CPU_INT32U *)argp));            /* get the baudrate                         */
            switch (paraId) {
                case LPC24XX_CAN_BUS_0 :
                    LPC24XX_CAN_C1MOD = 1L;           /* Go into Reset mode                       */

                    baud.Baudrate        = val;
                    baud.SamplePoint     = LPC24XX_CAN_DEF_SP;
                    baud.ResynchJumpWith = LPC24XX_CAN_DEF_RJW;

                    LPC24XX_CalcTimingReg (&baud);

                    val = (baud.PRESDIV)     |
                          (baud.RJW   << 14) |
                          (baud.PSEG1 << 16) |
                          (baud.PSEG2 << 20);

                    LPC24XX_CAN_C1BTR = val;          /* Set bit timing                           */

                    LPC24XX_CAN_C1MOD = 0L;           /* Operating Mode                           */
                    result =
                        LPC24XX_CAN_NO_ERR;           /* Indicate sucessfull function execution   */
                    break;
                case LPC24XX_CAN_BUS_1 :
                    LPC24XX_CAN_C2MOD = 1L;           /* Go into Reset mode                       */
                    baud.Baudrate        = val;
                    baud.SamplePoint     = LPC24XX_CAN_DEF_SP;
                    baud.ResynchJumpWith = LPC24XX_CAN_DEF_RJW;

                    LPC24XX_CalcTimingReg (&baud);

                    val = (baud.PRESDIV)     |
                          (baud.RJW   << 14) |
                          (baud.PSEG1 << 16) |
                          (baud.PSEG2 << 20);

                    LPC24XX_CAN_C2BTR = val;          /* Set bit timing                           */
                    LPC24XX_CAN_C2MOD = 0L;           /* Operating Mode                           */
                    result =
                        LPC24XX_CAN_NO_ERR;           /* Indicate sucessfull function execution   */
                    break;
            }
            break;                                    /*------------------------------------------*/
        case IO_LPC24XX_CAN_TX_READY:
            switch (paraId) {
                case LPC24XX_CAN_BUS_0:
                    if ((LPC24XX_CAN_C1SR & 0x04L) == 0x0L) {
                        *((CPU_INT08U *)argp) = 0;    /* Transmit Channel is not available        */
                    } else {
                        *((CPU_INT08U *)argp) = 1;    /* Transmit Channel is available            */
                    }
                    break;
                case LPC24XX_CAN_BUS_1 :
                    if ((LPC24XX_CAN_C2SR & 0x04L) == 0x0L) {
                        *((CPU_INT08U *)argp) = 0;    /* Transmit Channel is not available        */
                    } else {
                        *((CPU_INT08U *)argp) = 1;    /* Transmit Channel is available            */
                    }
                    break;
            }
            result = LPC24XX_CAN_NO_ERR;              /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/
        case IO_LPC24XX_CAN_START:                    /* START CAN COMMUNICATION                  */
            switch (paraId) {
                case LPC24XX_CAN_BUS_0 :
                    LPC24XX_CAN_C1MOD = 0x0;          /* Enter Normal Operating Mode              */
                    break;
                case LPC24XX_CAN_BUS_1 :
                    LPC24XX_CAN_C2MOD = 0x0;          /* Enter Normal Operating Mode              */
                    break;
            }
            result = LPC24XX_CAN_NO_ERR;              /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/
        case IO_LPC24XX_CAN_STOP:                     /* STOP CAN COMMUNICATION                   */
            switch (paraId) {
                case LPC24XX_CAN_BUS_0 :
                    LPC24XX_CAN_C1MOD = 0x1;          /* Go into Reset mode                       */
                    break;
                case LPC24XX_CAN_BUS_1 :
                    LPC24XX_CAN_C2MOD = 0x1;          /* Go into Reset mode                       */
                    break;
            }
            result = LPC24XX_CAN_NO_ERR;              /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/
        case IO_LPC24XX_CAN_RX_STANDARD:
            result = LPC24XX_CAN_NO_ERR;              /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/
        case IO_LPC24XX_CAN_RX_EXTENDED:
            result = LPC24XX_CAN_NO_ERR;              /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/
        case IO_LPC24XX_CAN_GET_NODE_STATUS:          /* GET NODE STATUS                          */
            switch (paraId) {
                case LPC24XX_CAN_BUS_0 :
                    status_reg =
                        LPC24XX_CAN_C1GSR;            /* read ICR to clear ns interrupt           */
                    break;
                case LPC24XX_CAN_BUS_1 :
                    status_reg =
                        LPC24XX_CAN_C2GSR;            /* read ICR to clear ns interrupt           */
                    break;
            }
            if ((status_reg & 0x20) != 0) {           /* Bit 6 Error Status Bit                   */
                *((CPU_INT08U *)argp) = 1;
            }
            if ((status_reg & 0x40) != 0) {           /* Bit 7 Bus Off Bit                        */
                *((CPU_INT08U *)argp) = 2;
            }
            if ((status_reg & 0x60) == 0) {           /* Bit 6/7 not set - bus active             */
                *((CPU_INT08U *)argp) = 0;
            }
            result = LPC24XX_CAN_NO_ERR;              /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/
#if LPC24XX_CAN_STD_FILTER_SIZE > 0
        case IO_LPC24XX_CAN_SET_STD_FILTER:           /* SET STANDARD FILTER                      */
            LPC24XX_CAN_AFMR       =   0x00000001L;   /* Disable the Acceptance filters to allow  */
                                                      /* setup of the table                       */
            for (i = 0; i < LPC24XX_CAN_STD_FILTER_SIZE; i++) {
                *AcceptanceTblPtr++ = StdFilter_Tbl[i];
            }
            LPC24XX_CAN_SFF_SA       = 0x0L;          /* Set start address of Standard table      */
            LPC24XX_CAN_SFF_GRP_SA   =                /* Set start address of Standard group table */
                 (LPC24XX_CAN_STD_FILTER_SIZE * 4);
            LPC24XX_CAN_EFF_SA       =                /* Set start address of Extended table      */
                 LPC24XX_CAN_SFF_GRP_SA +
                 (LPC24XX_CAN_STD_GROUP_FILTER_SIZE * 4);
            LPC24XX_CAN_EFF_GRP_SA   =                /* Set start address of Extended group table */
                 LPC24XX_CAN_EFF_SA +
                 (LPC24XX_CAN_EXT_FILTER_SIZE * 4);
            LPC24XX_CAN_END_OF_TABLE =                /* Set end of table address                 */
                 LPC24XX_CAN_EFF_GRP_SA +
                 (LPC24XX_CAN_EXT_GROUP_FILTER_SIZE * 4);
            LPC24XX_CAN_AFMR         = 0x0L;          /* Enable Acceptance filters                */
            result = LPC24XX_CAN_NO_ERR;              /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/
#endif
#if LPC24XX_CAN_STD_GROUP_FILTER_SIZE > 0
         case IO_LPC24XX_CAN_SET_STD_GROUP_FILTER:    /* SET STANDARD GROUP FILTER                */
            LPC24XX_CAN_AFMR       =   0x01L;         /* Disable the Acceptance filters to allow  */
                                                      /* setup of the table                       */
            for (i = 0; i < LPC24XX_CAN_STD_GROUP_FILTER_SIZE; i++) {
                *AcceptanceTblPtr++ = StdGroupFilter_Tbl[i];
            }
            LPC24XX_CAN_SFF_SA       = 0x0L;          /* Set start address of Standard table      */
            LPC24XX_CAN_SFF_GRP_SA   =                /* Set start address of Standard group table */
                 LPC24XX_CAN_STD_FILTER_SIZE * 4;
            LPC24XX_CAN_EFF_SA       =                /* Set start address of Extended table      */
                 LPC24XX_CAN_SFF_GRP_SA +
                 (LPC24XX_CAN_STD_GROUP_FILTER_SIZE * 4);
            LPC24XX_CAN_EFF_GRP_SA   =                /* Set start address of Extended group table */
                 LPC24XX_CAN_EFF_SA +
                 (LPC24XX_CAN_EXT_FILTER_SIZE * 4);
            LPC24XX_CAN_END_OF_TABLE =                /* Set end of table address                 */
                 LPC24XX_CAN_EFF_GRP_SA +
                 (LPC24XX_CAN_EXT_GROUP_FILTER_SIZE * 4);
            LPC24XX_CAN_AFMR         = 0x0L;          /* Enable Acceptance filters                */
            result = LPC24XX_CAN_NO_ERR;              /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/
#endif
#if LPC24XX_CAN_EXT_FILTER_SIZE > 0
        case IO_LPC24XX_CAN_SET_EXT_FILTER:           /* SET EXTENDED FILTER                      */
            LPC24XX_CAN_AFMR       =   0x01L;         /* Disable the Acceptance filters to allow  */
                                                      /* setup of the table                       */
            for (i = 0; i < LPC24XX_CAN_EXT_FILTER_SIZE; i++) {
                *AcceptanceTblPtr++ = ExtFilter_Tbl[i];
            }
            LPC24XX_CAN_SFF_SA       = 0x0L;          /* Set start address of Standard table      */
            LPC24XX_CAN_SFF_GRP_SA   =                /* Set start address of Standard group table */
                 LPC24XX_CAN_STD_FILTER_SIZE * 4;
            LPC24XX_CAN_EFF_SA       =                /* Set start address of Extended table      */
                 LPC24XX_CAN_SFF_GRP_SA +
                 (LPC24XX_CAN_STD_GROUP_FILTER_SIZE * 4);
            LPC24XX_CAN_EFF_GRP_SA   =                /* Set start address of Extended group table */
                 LPC24XX_CAN_EFF_SA +
                 (LPC24XX_CAN_EXT_FILTER_SIZE * 4);
            LPC24XX_CAN_END_OF_TABLE =                /* Set end of table address                 */
                LPC24XX_CAN_EFF_GRP_SA +
                 (LPC24XX_CAN_EXT_GROUP_FILTER_SIZE * 4);
            LPC24XX_CAN_AFMR         = 0x0L;          /* Enable Acceptance filters                */
            result = LPC24XX_CAN_NO_ERR;              /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/
#endif
#if LPC24XX_CAN_EXT_GROUP_FILTER_SIZE > 0
        case IO_LPC24XX_CAN_SET_EXT_GROUP_FILTER:     /* SET EXTENDED GROUP FILTER                */
            LPC24XX_CAN_AFMR       =   0x01L;         /* Disable the Acceptance filters to allow  */
                                                      /* setup of the table                       */
            for (i = 0; i < LPC24XX_CAN_EXT_GROUP_FILTER_SIZE; i++) {
                *AcceptanceTblPtr++ = ExtGroupFilter_Tbl[i];
            }
            LPC24XX_CAN_SFF_SA       = 0x0L;          /* Set start address of Standard table      */
            LPC24XX_CAN_SFF_GRP_SA   =                /* Set start address of Standard group table */
                 LPC24XX_CAN_STD_FILTER_SIZE * 4;
            LPC24XX_CAN_EFF_SA       =                /* Set start address of Extended table      */
                 LPC24XX_CAN_SFF_GRP_SA +
                 (LPC24XX_CAN_STD_GROUP_FILTER_SIZE * 4);
            LPC24XX_CAN_EFF_GRP_SA   =                /* Set start address of Extended group table */
                 LPC24XX_CAN_EFF_SA +
                 (LPC24XX_CAN_EXT_FILTER_SIZE * 4);
            LPC24XX_CAN_END_OF_TABLE =                /* Set end of table address                 */
                 LPC24XX_CAN_EFF_GRP_SA +
                 (LPC24XX_CAN_EXT_GROUP_FILTER_SIZE * 4);
            LPC24XX_CAN_AFMR         = 0x0L;          /* Enable Acceptance filters                */
            result = LPC24XX_CAN_NO_ERR;              /* Indicate sucessfull function execution   */
            break;                                    /*------------------------------------------*/
#endif
        default:
            break;
    }
    CPU_CRITICAL_EXIT();

    if (result == -1) {
        DrvError = LPC24XX_CAN_FUNC_ERR;
    }
    return(result);                                    /* Return function result                   */
}

/*************************************************************************************************/
/*!
* \brief                      CAN READ DATA
*
*           Read a received can frame from a message buffer. The buffer must have space for only
*           one can frame.
*
*
* \param paraId parameter identifier, returned by LPC24XXCANOpen()
* \param buffer Pointer to can frame
* \param size   Length of can frame memory
* \return errorcode (0 if ok -1 if an error occured)
*/
/*************************************************************************************************/
CPU_INT16S LPC24XXCANRead (CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size)
{
    LPC24XX_CANFRM *frm;                              /* Local: Pointer to can frame              */
    CPU_INT32U      Word;                             /* Local: A word for extended IO access     */
    CPU_INT32U     *pData;                            /* Local: A word for extended IO access     */
    CPU_INT16S      result = -1;                      /* Local: return value                      */
    CPU_SR_ALLOC();
                                                      /*------------------------------------------*/
#if LPC24XX_CAN_ARG_CHK_CFG > 0
    LPC24XX_CAN_DATA *can;                            /* Local: Pointer to can device             */

    if ((paraId >= LPC24XX_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range         */
        DrvError= LPC24XX_CAN_BUS_ERR;
        return(result);
    }
    if (size != sizeof(LPC24XX_CANFRM)) {             /* check that size is plausible             */
        DrvError = LPC24XX_CAN_NO_DATA_ERR;
        return(result);
    }
    if (buffer == (void *)0) {                        /* invalid buffer pointer                   */
        DrvError= LPC24XX_CAN_ARG_ERR;
        return(result);
    }
    can = &DevData[paraId];                           /* set pointer to can device                */
    if (can->Use != 1) {                              /* check, that can device is opened         */
        DrvError= LPC24XX_CAN_OPEN_ERR;
        return(result);
    }
#endif
    CPU_CRITICAL_ENTER();
    switch (paraId) {
        case LPC24XX_CAN_BUS_0:
            if ((LPC24XX_CAN_C1GSR & 1) == 1) {       /* data is available                        */
                frm = (LPC24XX_CANFRM *)buffer;
                frm->Identifier = LPC24XX_CAN_C1RID;  /* Set identifier                           */
                Word = LPC24XX_CAN_C1RFS;
                                                      /* mark remote/data frame                   */
                if ((Word & LPC24XX_CAN_RTR_MASK) != 0) {
                    frm->Identifier |= LPC24XX_CAN_RTR_FRAME_BIT;
                }
                                                      /* mark std/extended frame                  */
                if ((Word & LPC24XX_CAN_FF_MASK) != 0) {
                    frm->Identifier |= LPC24XX_CAN_FF_FRAME_BIT;
                }
                Word = LPC24XX_CAN_C1RFS & LPC24XX_CAN_DLC_MASK;
                frm->DLC = Word >> 16;                /* set DLC                                  */
                pData = (CPU_INT32U *)&frm->Data[0];  /* get data bytes 0-3 of rx buffer          */
                *pData = LPC24XX_CAN_C1RDA;
                pData = (CPU_INT32U *)&frm->Data[4];  /* get data bytes 4-7 of rx buffer          */
                *pData = LPC24XX_CAN_C1RDB;
                result = size;
            } else {
                DrvError = LPC24XX_CAN_NO_DATA_ERR;
            }
            break;
                                                      /*------------------------------------------*/
        case LPC24XX_CAN_BUS_1:
            if ((LPC24XX_CAN_C2GSR & 1) == 1) {       /* data is available                        */
                frm = (LPC24XX_CANFRM *)buffer;
                frm->Identifier = LPC24XX_CAN_C2RID;  /* Set identifier                           */
                Word = LPC24XX_CAN_C2RFS;
                                                      /* mark remote/data frame                   */
                if ((Word & LPC24XX_CAN_RTR_MASK) != 0) {
                    frm->Identifier |= LPC24XX_CAN_RTR_FRAME_BIT;
                }
                                                      /* mark std/extended frame                  */
                if ((Word & LPC24XX_CAN_FF_MASK) != 0) {
                    frm->Identifier |= LPC24XX_CAN_FF_FRAME_BIT;
                }
                Word = LPC24XX_CAN_C2RFS & LPC24XX_CAN_DLC_MASK;
                frm->DLC = Word >> 16;                /* set DLC                                  */
                pData = (CPU_INT32U *)&frm->Data[0];  /* get data bytes 0-3 of rx buffer          */
                *pData = LPC24XX_CAN_C2RDA;
                pData = (CPU_INT32U *)&frm->Data[4];  /* get data bytes 4-7 of rx buffer          */
                *pData = LPC24XX_CAN_C2RDB;
                result = size;
            } else {
                DrvError = LPC24XX_CAN_NO_DATA_ERR;
            }
            break;
        default:
            break;
    }
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
* \param paraId parameter identifier, returned by LPC24XXCANOpen()
* \param buffer Pointer to can frame
* \param size   Length of can frame memory
* \return errorcode (0 if ok -1 if an error occured)
*/
/*************************************************************************************************/
CPU_INT16S LPC24XXCANWrite (CPU_INT16S paraId, CPU_INT08U *buffer, CPU_INT16U size)
{
    LPC24XX_CANFRM *frm;                              /* Local: Pointer to can frame              */
    CPU_INT32U Word;                                  /* Local: A word for extended IO access     */
    CPU_INT16S result = -1;
    CPU_SR_ALLOC();
                                                      /*------------------------------------------*/
#if LPC24XX_CAN_ARG_CHK_CFG > 0
    LPC24XX_CAN_DATA *can;                            /* Local: Pointer to can device             */

    if ((paraId >= LPC24XX_CAN_N_DEV) || (paraId < 0)) { /* check that paraId is in range         */
        DrvError= LPC24XX_CAN_BUS_ERR;
        return(result);
    }
    if (size != sizeof(LPC24XX_CANFRM)) {             /* check that size is plausible             */
        DrvError = LPC24XX_CAN_NO_DATA_ERR;
        return(result);
    }
    if (buffer == (void *)0) {                        /* invalid buffer pointer                   */
        DrvError= LPC24XX_CAN_ARG_ERR;
        return(result);
    }
    can = &DevData[paraId];                           /* set pointer to can device                */
    if (can->Use != 1) {                              /* check, that can device is opened         */
        DrvError= LPC24XX_CAN_OPEN_ERR;
        return(result);
    }
#endif
    CPU_CRITICAL_ENTER();
    frm = (LPC24XX_CANFRM *)buffer;                   /* Set pointer to can frame                 */

    if (frm->Identifier > 0x7FF) {
        Word = 0x80000000L;                           /* identifier extended format               */
    } else {
        Word = 0x00000000L;                           /* identifier standard format               */
    }
    Word |= (frm->DLC << 16);

    switch (paraId) {
        case LPC24XX_CAN_BUS_0:
            if ((LPC24XX_CAN_C1SR & 0x04L) == 0x0L) { /* Transmit Channel is not available        */
                LPC24XX_CAN_C1CMR = 0x02L;            /* Abort Transmission                       */
                DrvError = LPC24XX_CAN_BUSY_ERR;
                CPU_CRITICAL_EXIT();
                return result;
            }
            LPC24XX_CAN_C1TFI1 = Word;                /* Write DLC, RTR (0) and FF                */
            LPC24XX_CAN_C1TID1 = frm->Identifier;
            LPC24XX_CAN_C1TDA1 = *(CPU_INT32U*) frm->Data;
            LPC24XX_CAN_C1TDB1 = *(CPU_INT32U*) &frm->Data[4];
            LPC24XX_CAN_C1CMR = 0x21L;                /* Transmit Buf 1                           */
            result = size;
            break;
        case LPC24XX_CAN_BUS_1:
            if ((LPC24XX_CAN_C2SR & 0x04L) == 0x0L) { /* Transmit Channel is not available        */
                LPC24XX_CAN_C2CMR = 0x02L;            /* Abort Transmission                       */
                DrvError = LPC24XX_CAN_BUSY_ERR;
                CPU_CRITICAL_EXIT();
                return result;
            }
            LPC24XX_CAN_C2TFI1 = Word;                /* Write DLC, RTR (0) and FF                */
            LPC24XX_CAN_C2TID1 = frm->Identifier;
            LPC24XX_CAN_C2TDA1 = *(CPU_INT32U*) frm->Data;
            LPC24XX_CAN_C2TDB1 = *(CPU_INT32U*) &frm->Data[4];
            LPC24XX_CAN_C2CMR = 0x21L;                /* Transmit Buf 1                           */
            result = size;
            break;
        default:
            break;
    }
    CPU_CRITICAL_EXIT();
                                                      /*------------------------------------------*/
    return result;                                    /* Return function result                   */
}



/*! } */

