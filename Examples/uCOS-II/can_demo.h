/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*               This file is provided as an example on how to use Micrium products.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only. This file can be modified as
*               required to meet the end-product requirements.
*
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                         uC/CAN EXAMPLE DEMO
*
* Filename : can_demo.h
* Version  : V2.42.01
*********************************************************************************************************
*/

#ifndef _CAN_DEMO_H_
#define _CAN_DEMO_H_


/*
*********************************************************************************************************
*                                               INLCUDES
*********************************************************************************************************
*/

#include  "cpu.h"
#include  "app_cfg.h"
#include  "lib_def.h"

#include  "can_bus.h"
#include  "can_frm.h"
#include  "can_msg.h"
#include  "can_sig.h"
#include  "can_err.h"


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void  App_CAN_Startup(void);
void  StatusChange   (void          *Signal,
                      CANSIG_VAL_T  *NewVal,
                      CPU_INT32U     CallbackId);
  
  
/*
*********************************************************************************************************
*                                         CONFIGURATION ERRORS
*********************************************************************************************************
*/
                                                                /* -------------- RX TASK PRIO & STK SIZE ------------- */
#ifndef  APP_CFG_CAN_RX_TASK_PRIO
#error  "APP_CFG_CAN_RX_TASK_PRIO                                     not #define'd in 'app_cfg.h'"
#endif

#ifndef  APP_CFG_CAN_RX_TASK_STK_SIZE
#error  "APP_CFG_CAN_RX_TASK_STK_SIZE                                 not #define'd in 'app_cfg.h'"
#endif

                                                                /* -------------- TX TASK PRIO & STK SIZE ------------- */
#ifndef  APP_CFG_CAN_TX_TASK_PRIO
#error  "APP_CFG_CAN_TX_TASK_PRIO                                     not #define'd in 'app_cfg.h'"
#endif

#ifndef  APP_CFG_CAN_TX_TASK_STK_SIZE
#error  "APP_CFG_CAN_TX_TASK_STK_SIZE                                 not #define'd in 'app_cfg.h'"
#endif

                   
/*
*********************************************************************************************************
*                                                 END
*********************************************************************************************************
*/

#endif  /* #ifndef _CAN_DEMO_H_ */
