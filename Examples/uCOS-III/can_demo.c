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
* Filename : can_demo.c
* Version  : V2.42.01
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                               INCLUDES
*********************************************************************************************************
*/

#include  "can_demo.h"


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/
                                                                /* CAN Demo Definitions.                                */
#define  APP_CAN_RX_TX_DEMO                 0u
#define  APP_CAN_ECHO_DEMO                  1u
                                                                /* --------------- uC/CAN DEMO SELECTION -------------- */
                                                                /* Select the CAN demo to run.                          */
#define  APP_CAN_DEMO_SELECT                APP_CAN_ECHO_DEMO


/*
*********************************************************************************************************
*                                            EXTERNAL DATA
*********************************************************************************************************
*/

extern  CANBUS_PARA  CanCfg;
extern  CANSIG_PARA  CanSig[];
extern  CANMSG_PARA  CanMsg[];


/*
*********************************************************************************************************
*                                              FUNCTIONS
*********************************************************************************************************
*/

#if (APP_CAN_DEMO_SELECT == APP_CAN_ECHO_DEMO)
static  void  Echo_Task  (void  *argp);
#endif

#if (APP_CAN_DEMO_SELECT == APP_CAN_RX_TX_DEMO)
static  void  Rx_Task    (void  *argp);

static  void  Tx_Task    (void  *argp);

static  void  Send_Status(void);
#endif


/*
*********************************************************************************************************
*                                              LOCAL DATA
*********************************************************************************************************
*/

#if (APP_CAN_DEMO_SELECT == APP_CAN_RX_TX_DEMO)
static  OS_TCB      App_CAN_RxTaskTCB;
static  CPU_STK     App_CAN_RxTaskStk[APP_CFG_CAN_RX_TASK_STK_SIZE];

static  OS_TCB      App_CAN_TxTaskTCB;
static  CPU_STK     App_CAN_TxTaskStk[APP_CFG_CAN_TX_TASK_STK_SIZE];

        CPU_INT08U  DemoCnt = 0u;
#endif

#if (APP_CAN_DEMO_SELECT == APP_CAN_ECHO_DEMO)
static  OS_TCB      App_CAN_EchoTaskTCB;
static  CPU_STK     App_CAN_EchoTaskStk[APP_CFG_CAN_RX_TASK_STK_SIZE];
#endif


/*
*********************************************************************************************************
*                                      CAN DEMO STARTUP FUNCTION
*
* Description : This function MUST be called by the Application to start the uC/CAN Module & Demo(s).
*
* Arguments   : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  App_CAN_Startup (void)
{
    CPU_INT16S    can_err;
    CANMSG_PARA  *m;
#if (CANSIG_STATIC_CONFIG == 0u)
    CANSIG_PARA  *s;
#endif
#if ((APP_CAN_DEMO_SELECT == APP_CAN_RX_TX_DEMO) || \
     (APP_CAN_DEMO_SELECT == APP_CAN_ECHO_DEMO))
    OS_ERR        os_err;
#endif

                                                                /* ------------------ HARDWARE SETUP ------------------ */
    /* $$$ - Add ISR Handlers Here - $$$ */

                                                                /* ------------------- uC/CAN SETUP ------------------- */
    CanSigInit(0L);                                             /* Initialize CAN Signals.                              */

#if (CANSIG_STATIC_CONFIG == 0u)
    s = CanSig;
    while (s < &CanSig[S_MAX]) {                                /* Create CAN Signals                                   */
        can_err = CanSigCreate(s);
        if (can_err < 0) {
           while (1);                                           /* Failure Handling Here.                               */
        }
        s++;
    }
#endif

    CanMsgInit(0L);                                             /* Initialize CAN Messages.                             */

    m = (CANMSG_PARA *)CanMsg;

    while (m < &CanMsg[CANMSG_N]) {                             /* Create CAN Messages.                                 */
        can_err = CanMsgCreate(m);
        if (can_err < 0) {
           while (1);                                           /* Failure Handling Here.                               */
        }
        m++;
    }

    CanBusInit(0L);                                             /* Initialize CAN Objects & Bus Layer.                  */
    can_err = CanBusEnable((CANBUS_PARA *)&CanCfg);             /* Enable CAN Device according to Configuration.        */

    if (can_err != CAN_ERR_NONE) {
        while (1);                                              /* Failure Handling Here.                               */
    }
    
                                                                /* --------------- uC/OS-III DEMO TASKs --------------- */
#if (APP_CAN_DEMO_SELECT == APP_CAN_RX_TX_DEMO)
    OSTaskCreate(&App_CAN_RxTaskTCB,                            /* Create CAN Rx Task.                                  */
                 "CAN Rx Demo",
                  Rx_Task,
                  0u,
                  APP_CFG_CAN_RX_TASK_PRIO,
                 &App_CAN_RxTaskStk[0u],
                 (APP_CFG_CAN_RX_TASK_STK_SIZE / 10u),
                  APP_CFG_CAN_RX_TASK_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);

    OSTaskCreate(&App_CAN_TxTaskTCB,                            /* Create CAN Rx Task.                                  */
                 "CAN Tx Demo",
                  Tx_Task,
                  0u,
                  APP_CFG_CAN_TX_TASK_PRIO,
                 &App_CAN_TxTaskStk[0u],
                 (APP_CFG_CAN_TX_TASK_STK_SIZE / 10u),
                  APP_CFG_CAN_TX_TASK_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);
#endif                                                          /* End APP_CAN_DEMO_SELECT == APP_CAN_RX_TX_DEMO        */

#if (APP_CAN_DEMO_SELECT == APP_CAN_ECHO_DEMO)
    OSTaskCreate(&App_CAN_EchoTaskTCB,                          /* Create CAN Echo Task.                                */
                 "CAN Echo Demo",
                  Echo_Task,
                  0u,
                  APP_CFG_CAN_RX_TASK_PRIO,
                 &App_CAN_EchoTaskStk[0u],
                 (APP_CFG_CAN_RX_TASK_STK_SIZE / 10u),
                  APP_CFG_CAN_RX_TASK_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);
#endif
}


/*
*********************************************************************************************************
*                                              ECHO TASK
*
* Description : This function Echoes a Received frame back to the sender. The frame Identifier is
*               changed to 0x100 before Echoing.
*
* Arguments   : argp     Function Argument Pointer.
*
* Return(s)   : none.
*
* Caller(s)   : App_CAN_Startup().
*
* Note(s)     : none.
*********************************************************************************************************
*/
#if (APP_CAN_DEMO_SELECT == APP_CAN_ECHO_DEMO)
void  Echo_Task (void  *argp)
{
    CANFRM      frm;
    CPU_INT08U  busId;

   
    (void)&argp;                                                /* Suppress Compiler Warning.                           */
   
    busId = 0u;                                                 /* CAN Device Number/ CAN Controller Number.            */
    CanBusIoCtl(busId,                                          /* Rx Timeout is set to wait forever for a new Frame.   */
                CANBUS_SET_RX_TIMEOUT,
                0);

    while (DEF_ON) {                                            /* Endless while loop.                                  */
        CanBusRead(         busId,                              /* Wait for new CAN Frame to Arrive.                    */
                   (void *)&frm,
                            sizeof(CANFRM));

        frm.Identifier = 0x100L;                                /* Change Frame ID.                                     */

        CanBusWrite(         busId,                             /* Echo back same data with updated Frame ID.           */
                    (void *)&frm,
                             sizeof(CANFRM));
    }
}
#endif


/*
*********************************************************************************************************
*                                               RX TASK
*
* Description : This function Receives a frame and writes it to the Message layer. It's unused in this
*               Example but can be exchanged with the current 'task' running in App_CAN_Startup().
*
* Arguments   : argp     Function Argument Pointer.
*
* Return(s)   : none.
*
* Caller(s)   : App_CAN_Startup().
*
* Note(s)     : none.
*********************************************************************************************************
*/
#if (APP_CAN_DEMO_SELECT == APP_CAN_RX_TX_DEMO)
void  Rx_Task (void  *argp)
{
    CANFRM      frm;
    CPU_INT16S  msg;
    CPU_INT08U  busId;

   
    (void)&argp;                                                /* Suppress Compiler Warning.                           */
   
    busId = 0u;                                                 /* CAN Device Number/ CAN Controller Number.            */
    CanBusIoCtl(busId,                                          /* Rx Timeout is set to wait forever for a new Frame.   */
                CANBUS_SET_RX_TIMEOUT,
                0);
                
    while (DEF_ON) {                                            /* Endless while loop.                                  */
        CanBusRead(         busId,                              /* Wait for new CAN Frame to Arrive.                    */
                   (void *)&frm,
                            sizeof(CANFRM));

        msg = CanMsgOpen(busId,                                 /* Try to open the Received Frame on Message layer.     */
                         frm.Identifier,
                         0);
        if (msg >= 0) {                                         /* If it could be opened, write the Frame on Msg layer. */
            CanMsgWrite(         msg,
                        (void *)&frm,
                                 sizeof(CANFRM));
        }
    }
}


/*
*********************************************************************************************************
*                                               TX TASK
*
* Description : This function updates a signal and writes the corresponding message to the CAN bus.
*
* Arguments   : argp     Function Argument Pointer.
*
* Return(s)   : none.
*
* Caller(s)   : App_CAN_Startup().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  Tx_Task (void  *argp)
{
    OS_ERR  os_err;
    
    
    (void)&argp;                                                /* Suppress Compiler Warning.                           */
    
    while (DEF_ON) {                                            /* Endless while loop.                                  */
        DemoCnt++;
        CanSigWrite( S_CPULOAD,                                 /* Set an Incrementing Counter.                         */
                    &DemoCnt,
                     1);
        
        if (DemoCnt >= DEF_OCTET_MASK) {
            DemoCnt = 0u;                                       /* Reset Demo Count to 0.                               */
        }
                   
        Send_Status();                                          /* Send Signal Status on CAN bus                        */
        
        OSTimeDlyHMSM(0u, 0u, 0u, 200u,
                      OS_OPT_TIME_HMSM_STRICT, &os_err);
    }
}


/*
*********************************************************************************************************
*                                          SEND STATUS FRAME
*
* Description : This function reads a CAN frame with ID 0x150 from the message layer and writes it
*               to the CAN bus.
*
* Arguments   : none.
*
* Return(s)   : none.
*
* Caller(s)   : Tx_Task().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  Send_Status (void)
{
    CANFRM      frm;
    CPU_INT16S  msg;
    CPU_INT08U  busId;
    
    
    busId = 0u;                                                 /* CAN Device Number/ CAN Controller Number.            */
    msg   = CanMsgOpen(busId,                                   /* Open frame with ID 0x123 on Message Layer.           */
                       0x123u,
                       0);
   if (msg >= 0) {
        CanMsgRead(         msg,                                /* Read frame from Message Layer.                       */
                   (void *)&frm,
                            sizeof(CANFRM));
                          
        CanBusWrite(         busId,                             /* Write Frame to CAN Bus Layer.                        */
                    (void *)&frm,
                             sizeof(CANFRM));
   }
}
#endif


/*
*********************************************************************************************************
*                                     SIGNAL CALLBACK FUNCTION
*
* Description : This function is in the format of a signal callback function. It is mainly empty and can
*               be used as a template.
*
* Arguments   : Signal      Pointer to signal
*               NewVal      Pointer to new value to signal
*               CallbackId  Identifies the callback function as one of the following
*                               CANSIG_CALLBACK_WRITE_ID
*                               CANSIG_CALLBACK_READ_ID
*
* Return(s)   : none.
*
* Caller(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  StatusChange (void         *Signal,
                   CANSIG_VAL_T  *NewVal,
                   CPU_INT32U     CallbackId)
{
    (void)&Signal;                                              /* Suppress Compiler Warning.                           */

    if (CallbackId == CANSIG_CALLBACK_READ_ID) {
        if (*NewVal == 1) {
            /* $$$ - Start Load-Task - $$$ */
        } else {
            /* $$$ - Stop Load-Task - $$$ */
        }
    }
}


/*
*********************************************************************************************************
*                                                 END
*********************************************************************************************************
*/
