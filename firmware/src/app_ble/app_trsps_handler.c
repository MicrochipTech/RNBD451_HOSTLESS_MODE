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

/*******************************************************************************
  Application BLE Profile Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_trsps_handler.c

  Summary:
    This file contains the Application BLE functions for this project.

  Description:
    This file contains the Application BLE functions for this project.
 *******************************************************************************/


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "app_trsps_handler.h"
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variables
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************

void APP_TrspsEvtHandler(BLE_TRSPS_Event_T *p_event)
{
    switch(p_event->eventId)
    {
        case BLE_TRSPS_EVT_CTRL_STATUS:
        {
            /* TODO: implement your application code.*/
        }
        break;
        
        case BLE_TRSPS_EVT_TX_STATUS:
        {
            /* TODO: implement your application code.*/
        }
        break;

        case BLE_TRSPS_EVT_CBFC_ENABLED:
        {
            /* TODO: implement your application code.*/
        }
        break;
        
        case BLE_TRSPS_EVT_CBFC_CREDIT:
        {
            /* TODO: implement your application code.*/
        }
        break;
        
        case BLE_TRSPS_EVT_RECEIVE_DATA:
        {
            /* TODO: implement your application code.*/
//            printf("TRPS BLE Data received \r\n");
//            uint8_t data_len;
//            BLE_TRSPC_GetDataLength(p_event->eventField.onReceiveData.connHandle, &data_len);
//            if(data_len>150)
//            {
//                data_len=150;
//            }
//            APP_Msg_T appMsg;
//            appMsg.msgData[0]=(uint8_t)(p_event->eventField.onReceiveData.connHandle>>8);
//            appMsg.msgData[1]=(uint8_t)(p_event->eventField.onReceiveData.connHandle & 0xFF);
//            appMsg.msgData[2]=data_len;
//            BLE_TRSPC_GetData(p_event->eventField.onReceiveData.connHandle, &appMsg.msgData[3]);
//            appMsg.msgId = APP_MSG_BLE_RECIEVE_CHAT_EVT;
//            OSAL_QUEUE_Send(&appData.appQueue, &appMsg, 0);
        }
        break;
        
        case BLE_TRSPS_EVT_VENDOR_CMD:
        {
            /* TODO: implement your application code.*/
        }
        break;

        case BLE_TRSPS_EVT_ERR_UNSPECIFIED:
        {
            /* TODO: implement your application code.*/
        }
        break;

        case BLE_TRSPS_EVT_ERR_NO_MEM:
        {
            /* TODO: implement your application code.*/
        }
        break;

        default:
        break;
    }
}