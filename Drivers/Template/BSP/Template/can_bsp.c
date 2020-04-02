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
*                                         CAN BSP DRIVER CODE
*
*                                              Template
*
* Filename : can_bsp.c
* Version  : V2.42.01
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              INCLUDES
*********************************************************************************************************
*/

#include  "drv_can.h"
#include  "drv_def.h"
#include  "can_bsp.h"

#if ((CANBUS_RX_HANDLER_EN > 0u) || \
     (CANBUS_TX_HANDLER_EN > 0u) || \
     (CANBUS_NS_HANDLER_EN > 0u))
#include  "can_bus.h"
#endif


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/


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
                                                                /* Array Holds Initialized Node ID of Module            */
static  CPU_INT08U  <Drv_Name>_CAN_DevIds[<Drv_Name>_CAN_N_DEV];


/*
*********************************************************************************************************
*                                             GLOBAL DATA
*********************************************************************************************************
*/


/*
*********************************************************************************************************
                                               CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                              FUNCTIONS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                   <Drv_Name>_CAN_BSP_IntVectSet()
*
* Description : This function is used to assign ISR handlers for CAN functionality.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  <Drv_Name>_CAN_BSP_IntVectSet (void)
{

#if ((CANBUS_RX_HANDLER_EN > 0u) || \
     (CANBUS_TX_HANDLER_EN > 0u) || \
     (CANBUS_NS_HANDLER_EN > 0u))

#endif
}


/*
*********************************************************************************************************
*                                     <Drv_Name>_CAN_PinSetting()
*
* Description : This Function provides all the necessary Pin Settings for the required CAN Device.
*
* Argument(s) : para_id     Selects the CAN Device [<Drv_Name>_CAN_BUS_0, <Drv_Name>_CAN_BUS_1].
*
* Return(s)   : Returns error value if improper CAN Device used, if not returns 'DEF_OK'.
*
* Caller(s)   : <Drv_Name>_CAN_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_BOOLEAN  <Drv_Name>_CAN_PinSetting (CPU_INT08U  para_id)
{
    CPU_BOOLEAN  bsp_err;


    bsp_err = DEF_OK;                                           /* Init Var(s).                                         */

    return (bsp_err);
}


/*
*********************************************************************************************************
*                                   <Drv_Name>_CAN_CalcTimingReg()
*
* Description : Calculates the Timing Register Values according to the given Baudrate Settings.
*
* Argument(s) : data        Pointer to the Baudrate Settings.
*
* Return(s)   : Returns error value of 'DEF_FAIL' if Timeout occurred, if not returns 'DEF_OK'.
*
* Caller(s)   : <Drv_Name>_CAN_Init().
*               <Drv_Name>_CAN_IoCtl().
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_BOOLEAN  <Drv_Name>_CAN_CalcTimingReg (<Drv_Name>_CAN_BAUD  *data)
{

    return (DEF_OK);
}


/*
*********************************************************************************************************
*                                     <Drv_Name>_CAN_SetDevIds()
*
* Description : This Function sets the Device IDs for the ISRs.
*
* Argument(s) : dev_id      CAN Device ID.
*
*               dev_name    CAN Device Name.
*
* Return(s)   : none.
*
* Caller(s)   : <Drv_Name>_CAN_Open().
*
* Note(s)     : none. 
*********************************************************************************************************
*/

#if ((CANBUS_RX_HANDLER_EN > 0u) || \
     (CANBUS_TX_HANDLER_EN > 0u) || \
     (CANBUS_NS_HANDLER_EN > 0u))
void  <Drv_Name>_CAN_SetDevIds (CPU_INT08U  dev_id,
                                CPU_INT08U  dev_name)
{
    <Drv_Name>_CAN_DevIds[dev_name] = dev_id;
}
#endif


/*
*********************************************************************************************************
*                                    <Drv_Name>_CANx_ISR_Handler()
*
* Description : This holds the Generic CAN Interrupt Handler(s) for a given CAN Channel. This can be
*               configured differently based on the IVT table requirements for CAN Rx, Tx, and NS handling.
*               It is assumed that only one CAN ISR handler is needed. Additional ISR handlers must be
*               added based on the quantity of CAN channels (One ISR Handler per channel).
*
* Argument(s) : cpu_id      Function pointer ID from CPU.
*
* Return(s)   : none.
*
* Caller(s)   : <Drv_Name>_CAN_BSP_IntVectSet().
*
* Note(s)     : none.
*********************************************************************************************************
*/

#if ((CANBUS_RX_HANDLER_EN > 0u) || \
     (CANBUS_TX_HANDLER_EN > 0u) || \
     (CANBUS_NS_HANDLER_EN > 0u))
void  <Drv_Name>_CAN0_ISR_Handler (CPU_INT32U  cpu_id)
{

}

#endif
