// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2022 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END

/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <string.h>
#include "app.h"
#include "definitions.h"
#include "app_ble.h"
#include "ble_trspc/ble_trspc.h"
#include "app_timer/app_timer.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

#define APP_RNBD_REMOTE_CMD_DISABLE         0x46
#define APP_RNBD_REMOTE_CMD_ENABLE          0x59


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
bool b_button_debounce=false;


static void APP_BLE_Sensor_Button_Callback(uintptr_t context)
{
    APP_Msg_T appMsg;
    static uint8_t press_cnt=0;
    press_cnt++;
    if(press_cnt>15)
    {
        press_cnt=0;
        APP_TIMER_SetTimer(APP_TIMER_ID_0,APP_TIMER_1S,false);
    }
    if(!b_button_debounce)
    {
//        appMsg.msgId = APP_MSG_BLE_ENT_RMT_MODE2;
        appMsg.msgId = APP_MSG_RMT_BLE_IO_ON;
        OSAL_QUEUE_SendISR(&appData.appQueue, &appMsg);
        b_button_debounce = true;
    }  
    else
    {
//        appMsg.msgId = APP_MSG_BLE_ENT_RMT_MODE3;
        appMsg.msgId = APP_MSG_RMT_BLE_IO_OFF;
        OSAL_QUEUE_SendISR(&appData.appQueue, &appMsg);
        b_button_debounce = false;
    }
    if(press_cnt==15)
    {
        appMsg.msgId = APP_MSG_BLE_EXT_RMT;
        OSAL_QUEUE_SendISR(&appData.appQueue, &appMsg);
        b_button_debounce = false;
    }
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;


    appData.appQueue = xQueueCreate( 64, sizeof(APP_Msg_T) );
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    APP_Msg_T    appMsg[1];
    APP_Msg_T   *p_appMsg;
    p_appMsg=appMsg;

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
            //appData.appQueue = xQueueCreate( 10, sizeof(APP_Msg_T) );
            APP_BleStackInit();
            if (appInitialized)
            {
                printf("Scanning Started\r\n");
                BLE_GAP_SetScanningEnable(true, BLE_GAP_SCAN_FD_DISABLE, BLE_GAP_SCAN_MODE_OBSERVER, 1200);   //mmr
                EIC_CallbackRegister(EIC_PIN_0,APP_BLE_Sensor_Button_Callback,0);
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            if (OSAL_QUEUE_Receive(&appData.appQueue, &appMsg, OSAL_WAIT_FOREVER))
            {

                if(p_appMsg->msgId==APP_MSG_BLE_STACK_EVT)
                {
                    // Pass BLE Stack Event Message to User Application for handling
                    APP_BleStackEvtHandler((STACK_Event_T *)p_appMsg->msgData);
                }
                else if(p_appMsg->msgId==APP_MSG_BLE_STACK_LOG)
                {
                    // Pass BLE LOG Event Message to User Application for handling
                    APP_BleStackLogHandler((BT_SYS_LogEvent_T *)p_appMsg->msgData);
                }
                else if(p_appMsg->msgId==APP_MSG_RMT_BLE_IO_ON)
                {
                    extern uint16_t conn_hdl;
                    uint8_t rmt_data[14];
                    APP_Msg_T appMsg;
                    sprintf((char *)rmt_data, "|O,0001,0001\r\n", conn_hdl);
                    uint16_t result=BLE_TRSPC_SendData(conn_hdl,14,&rmt_data);
                    if(result==0)
                    {
                        printf("IO ON send:%d\r\n",result);
//                        appMsg.msgId = APP_MSG_BLE_EXT_RMT;
//                        OSAL_QUEUE_SendISR(&appData.appQueue, &appMsg);
                    }
                    
                }
                else if(p_appMsg->msgId==APP_MSG_RMT_BLE_IO_OFF)
                {
                    extern uint16_t conn_hdl;
                    APP_Msg_T appMsg;
                    uint8_t rmt_data[14];
                    sprintf((char *)rmt_data, "|O,0001,0000\r\n", conn_hdl);
                    uint16_t result=BLE_TRSPC_SendData(conn_hdl,14,&rmt_data);
                    printf("IO OFF send:%d\r\n",result);
                    if(result==0)
                    {
                        printf("IO ON send:%d\r\n",result);
//                        appMsg.msgId = APP_MSG_BLE_EXT_RMT;
//                        OSAL_QUEUE_SendISR(&appData.appQueue, &appMsg);
                    }
                }
                else if(p_appMsg->msgId==APP_MSG_BLE_SEND_DATA)
                {
                    extern uint16_t conn_hdl;
                    uint8_t rmt_data[3];
                    sprintf((char *)rmt_data, "D\r\n", conn_hdl);
                    uint16_t result=BLE_TRSPC_SendData(conn_hdl,3,&rmt_data);
                    printf("data send:%d\r\n",result);
                }
                else if(p_appMsg->msgId == APP_MSG_BLE_RECIEVE_CHAT_EVT)      //APP_TIMER_SCAN_DELAY
                {
                    uint8_t data_len;
                    uint8_t data[150];
                    BLE_TRSPC_GetDataLength(((p_appMsg->msgData[0]<<8)|p_appMsg->msgData[1]), &data_len);
                    BLE_TRSPC_GetData(((p_appMsg->msgData[0]<<8)|p_appMsg->msgData[1]), &data);
                    printf("data len:%d\r\n",data_len);
                    if (data_len < sizeof(data)) 
                    {
                        data[data_len] = '\0';
                    } 
                    else 
                    {
                        data[sizeof(data) - 1] = '\0';
                    }
                    printf("RNBD Data:\r\n%s\n", data);
                }
                else if(p_appMsg->msgId==APP_MSG_BLE_ENT_RMT_MODE)
                {
                    extern uint16_t conn_hdl;
                    uint8_t rmt_cmd[4]={0x30,0x30,0x30,0x30};
                    uint16_t result=BLE_TRSPC_SendVendorCommand(conn_hdl,APP_RNBD_REMOTE_CMD_ENABLE,4,&rmt_cmd);
                    if (result == MBA_RES_SUCCESS)
                    {   
                        APP_Msg_T appMsg;
                        appMsg.msgId = APP_MSG_BLE_SEND_DATA;
                        OSAL_QUEUE_Send(&appData.appQueue, &appMsg, 0);
                        printf("Success:%d\r\n",result);
                    }
                }
                else if(p_appMsg->msgId==APP_MSG_BLE_EXT_RMT)
                {
                    extern uint16_t conn_hdl;
                    printf("Exit rmt\r\n");
                    uint8_t rmt_cmd[4]={0x30,0x30,0x30,0x30};
                    uint16_t result=BLE_TRSPC_SendVendorCommand(conn_hdl,APP_RNBD_REMOTE_CMD_DISABLE,4,&rmt_cmd);
                    if (result == MBA_RES_SUCCESS)
                    {   
                        APP_Msg_T appMsg;
                        appMsg.msgId = APP_MSG_BLE_SEND_DATA;
                        OSAL_QUEUE_Send(&appData.appQueue, &appMsg, 0);
                        printf("Success:%d\r\n",result);
                    }
                }
//                else if(p_appMsg->msgId==APP_MSG_BLE_EXT_RMT_MODE)
//                {
//                    extern uint16_t conn_hdl;
//                    uint8_t rmt_cmd[4]={0x30,0x30,0x30,0x30};
//                    uint16_t result=BLE_TRSPC_SendVendorCommand(conn_hdl,APP_RNBD_REMOTE_CMD_ENABLE,4,&rmt_cmd);
//                    if (result == MBA_RES_SUCCESS)
//                    {   
//                        APP_Msg_T appMsg;
//                        appMsg.msgId = APP_MSG_BLE_SEND_DATA;
//                        OSAL_QUEUE_Send(&appData.appQueue, &appMsg, 0);
//                        printf("Success:%d\r\n",result);
//                    }
//                }
                else if(p_appMsg->msgId == APP_MSG_BLE_SCAN_EVT)
                {
                    uint16_t connStatus;
                    BLE_GAP_EvtAdvReport_T addrDevAddr;
                    BLE_GAP_CreateConnParams_T createConnParam_t;
                    memcpy(&addrDevAddr, &p_appMsg->msgData, sizeof(BLE_GAP_EvtAdvReport_T));

                    createConnParam_t.scanInterval = 0x3C; // 37.5 ms 
                    createConnParam_t.scanWindow = 0x1E; // 18.75 ms
                    createConnParam_t.filterPolicy = BLE_GAP_SCAN_FP_ACCEPT_ALL;
                    createConnParam_t.peerAddr.addrType = addrDevAddr.addr.addrType;
                    memcpy(createConnParam_t.peerAddr.addr, addrDevAddr.addr.addr, GAP_MAX_BD_ADDRESS_LEN);
                    createConnParam_t.connParams.intervalMin = 0x50; // 20ms
                    createConnParam_t.connParams.intervalMax = 0x50; // 20ms
                    createConnParam_t.connParams.latency = 0;
                    createConnParam_t.connParams.supervisionTimeout = 0x48; // 720ms
                    connStatus = BLE_GAP_CreateConnection(&createConnParam_t);
                    if(connStatus == MBA_RES_SUCCESS)
                    {
                        printf("Connecting to BLE Device: RSSI:%ddBm, \r\n", addrDevAddr.rssi);
                    }
                    else if(connStatus==0x10C)
                    {
                        APP_Msg_T appMsg;
                        appMsg.msgId = APP_MSG_BLE_SCAN_EVT;
                        memcpy(appMsg.msgData, &p_appMsg->msgData, sizeof(BLE_GAP_EvtAdvReport_T));
                        OSAL_QUEUE_Send(&appData.appQueue, &appMsg, 0);
                    }
                    else
                    {
                        printf("Connecting to BLE Device: - Failed: 0x%X\r\n", connStatus);
                    } 
                }
            }
            break;
        }

        /* TODO: implement your application state machine.*/


        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}


/*******************************************************************************
 End of File
 */
