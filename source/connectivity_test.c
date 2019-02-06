/*!
* The Clear BSD License
* Copyright 2016-2017 NXP
* All rights reserved.
* 
* \file
*
* This is a source file for the main application.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the
* disclaimer below) provided that the following conditions are met:
* 
* * Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
* 
* * Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
* 
* * Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
* 
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
* GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
/* Drv */
#include "LED.h"
#include "Keyboard.h"

/* Fwk */
#include "MemManager.h"
#include "TimersManager.h"
#include "RNG_Interface.h"
#include "Messaging.h"
#include "SecLib.h"
#include "Panic.h"
#include "fsl_xcvr.h"
#include "fsl_os_abstraction.h"

/* KSDK */
#include "board.h"

#ifdef cPWR_UsePowerDownMode
#if (cPWR_UsePowerDownMode)
#include "PWR_Interface.h"
#endif
#endif

#ifdef FSL_RTOS_FREE_RTOS
#include "FreeRTOSConfig.h"
#endif

#include "genfsk_interface.h"
#include "gen_fsk_interaction.h"

#include "connectivity_test.h"

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define gAppNumberOfTests_d (3)
#define App_NotifySelf() OSA_EventSet(mAppThreadEvt, gCtEvtSelfEvent_c)

#ifdef FSL_RTOS_FREE_RTOS
#if (configUSE_IDLE_HOOK)
#define mAppIdleHook_c 1
#endif
#endif

#ifndef mAppIdleHook_c
#define mAppIdleHook_c 0
#endif


/************************************************************************************
*************************************************************************************
* Private definitions
*************************************************************************************
************************************************************************************/
typedef bool_t ( * pCtTestFunction)(ct_event_t evt, void* pAssocData);

typedef enum uart_state_tag {
	stateNone = 0,
	stateReadingDestID,
	stateReadingLED,
	stateReadingString
} uart_state_tag_t;
/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
/*Application main*/
static void App_Thread (uint32_t param); 
/*Application event handler*/
static void App_HandleEvents(osaEventFlags_t flags);
/*Function that reads latest byte from Serial Manager*/
static void App_UpdateUartData(uint8_t* pData);
/*Application Init*/
static void App_InitApp();
/*Generic FSK RX callback*/
static void App_GenFskReceiveCallback(uint8_t *pBuffer, 
                                      uint16_t bufferLength, 
                                      uint64_t timestamp, 
                                      uint8_t rssi,
                                      uint8_t crcValid);
/*Generic FSK Notification callback*/
static void App_GenFskEventNotificationCallback(genfskEvent_t event, 
                                                genfskEventStatus_t eventStatus);

static void App_NotifyAppThread(void);
/*Serial Manager UART RX callback*/
static void App_SerialCallback(void* param);
/*Timer callback*/
static void App_TimerCallback(void* param);
/*Used for testing by manually specifying data over the Serial*/
static void App_ResolveUartRx();
/* Sends the latest accelerometer data over the airwaves */
static void App_ReceiveAccelData();
/* Callback that allows for specifying arbitrary events */
static void App_CallArbitraryEvent(ct_event_t);

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
static uint8_t platformInitialized = 0;

/*event used by the application thread*/
static osaEventId_t mAppThreadEvt;

/*variable to store key pressed by user*/
static uint8_t mAppUartData = 0;

/*structure to store information regarding latest received packet*/
static ct_rx_indication_t mAppRxLatestPacket;

/*latest generic fsk event status*/
static genfskEventStatus_t mAppGenfskStatus;

/* extern Serial instance Id */
extern uint8_t mAppSerId;
/* extern Timer Id */
extern uint8_t mAppTmrId;
/*extern GENFSK instance id*/
extern uint8_t mAppGenfskId;
/*extern MCU reset api*/
extern void ResetMCU(void);

/*! *********************************************************************************
* \brief  This is the first task created by the OS. This task will initialize 
*         the system
*
* \param[in]  param
*
********************************************************************************** */
void main_task(uint32_t param)
{  
    if (!platformInitialized)
    {
        platformInitialized = 1;
        

        hardware_init();
        
        /* Framework init */
        MEM_Init();
        TMR_Init();
        //initialize Serial Manager
        SerialManager_Init();
        LED_Init();
        SecLib_Init();
        
        GENFSK_Init();
        
        /* GENFSK LL Init with default register config */
        GENFSK_AllocInstance(&mAppGenfskId, NULL, NULL, NULL);
        
        /*create app thread event*/
        mAppThreadEvt = OSA_EventCreate(TRUE);
        
        /*Generate GenFSK callbacks*/
        App_InitApp();

        /*start serial flashing using all LEDs*/
        LED_StartSerialFlash(LED1);
        
        /*initialize the application interface id*/
        Serial_InitInterface(&mAppSerId, 
                             APP_SERIAL_INTERFACE_TYPE, 
                             APP_SERIAL_INTERFACE_INSTANCE);
        /*set baudrate to 115200*/
        Serial_SetBaudRate(mAppSerId, 
                           APP_SERIAL_INTERFACE_SPEED);
        /*set Serial Manager receive callback*/
        Serial_SetRxCallBack(mAppSerId, App_SerialCallback, NULL);
        
        /*allocate a timer*/
        mAppTmrId = TMR_AllocateTimer();

    	Serial_Print(mAppSerId, "\r\nReady to send...\r\n", 1);
    }
    
    /* Call application task */
    App_Thread( param );
}

/*! *********************************************************************************
* \brief  This function represents the Application task. 
*         This task reuses the stack allocated for the MainThread.
*         This function is called to process all events for the task. Events 
*         include timers, messages and any other user defined events.
* \param[in]  argument
*
********************************************************************************** */

void App_Thread (uint32_t param)
{
    osaEventFlags_t mAppThreadEvtFlags = 0;

    //Maybe setup Timer here with TMR_... functions
    //Could be used for periodic polling of accelerometer


    while(1)
    {
    	IT_Rx(0);
    	//Wait indefinitely for an event to occur. Then do stuff based on the received event
        (void)OSA_EventWait(mAppThreadEvt, gCtEvtEventsAll_c, FALSE, osaWaitForever_c ,&mAppThreadEvtFlags);
        App_HandleEvents(mAppThreadEvtFlags);/*handle app events*/
    }

}

/*! *********************************************************************************
* \brief  The application event handler 
*         This function is called each time there is an OS event for the AppThread
* \param[in]  flags The OS event flags specific to the Connectivity Test App.
*
********************************************************************************** */
void App_HandleEvents(osaEventFlags_t flags)
{
    if(flags & EvtRxDone_c) {
    	Serial_Print(mAppSerId, "RxDone\r\n", 1);
    	IT_HandlePacket();
    }
    if(flags & EvtTxDone_c) {
    	Serial_Print(mAppSerId, "TxDone\r\n", 1);
    }
    if(flags & EvtRxFailed_c) {
    	Serial_Print(mAppSerId, "RxFailed\r\n", 1);
    }
    if(flags & EvtSeqTimeout_c) {
    	Serial_Print(mAppSerId, "SeqTimeout\r\n", 1);
    }

    if(flags & gCtEvtUart_c) {
    	App_UpdateUartData(&mAppUartData);
    	App_ResolveUartRx();
    }

    if (flags & gCtEvtAccelRequest_c) {
    	App_ReceiveAccelData();
    }

    if(flags & gCtEvtTimerExpired_c) {
    	Serial_Print(mAppSerId, "TimerExpired\r\n", 1);
    }
    if(flags & gCtEvtSelfEvent_c) {
    	Serial_Print(mAppSerId, "SelfEvent\r\n", 1);
    }

}

static void App_ReceiveAccelData(){

}

static void App_ResolveUartRx(){

	static uint8_t ledMessage[5] = {gLedOpcode_c, LedUnchanged, LedUnchanged, LedUnchanged, LedUnchanged};
	static uart_state_tag_t state = stateNone;
	static uint8_t destID = broadcastID;

	//Do checks before entering state machine
	if (mAppUartData == 'r') { //Reset the state machine
		Serial_Print(mAppSerId, "Resetting state\r\n", true);
		state = stateNone;
		return;
	} else if (mAppUartData == 'd'){ //Print number of devices
		Serial_PrintDec(mAppSerId, IT_GetNumberOfDevices());
		Serial_Print(mAppSerId, "\r\n", true);
	} else if (mAppUartData == 'a'){
		uint8_t message[1] = {gAccelDataRequestOpcode_c};
		IT_TxWithPayload(message, 1, broadcastID);
	}

	//Message Builder state machine
	switch (state) {
	case stateNone:
		switch (mAppUartData){
		case '0':
			state = stateReadingDestID;
			break;

		case '1':
			state = stateReadingLED;
			break;

		case '2':
			state = stateReadingString;
			break;

		default:
			break;
		}
		break;
	case stateReadingDestID:
		destID = mAppUartData;
		state = stateNone;
		break;
	case stateReadingLED:
		;
		static uint8_t LEDnum = 0;
		LEDnum++;

		switch(mAppUartData){
		case '0':
			ledMessage[LEDnum] = LedOn;
			break;
		case '1':
			ledMessage[LEDnum] = LedOff;
			break;
		case '2':
			ledMessage[LEDnum] = LedToggle;
			break;
		case '3':
			ledMessage[LEDnum] = LedUnchanged;
			break;
		default:
			break;
		}

		//Received all LED data
		if (LEDnum == 4) {
			state = stateNone;
			LEDnum = 0;
			IT_TxWithPayload(ledMessage, 5, destID);
			break;
		}

		break;
	case stateReadingString:
		break;
	default:
		break;
	}
}

/*! *********************************************************************************
* \brief  This function is called each time SerialManager notifies the application
*         task that a byte was received.
*         The function checks if there are additional bytes in the SerialMgr  
*         queue and simulates a new SM event if there is more data.
* \param[in]  pData Pointer to where to store byte read.
*
********************************************************************************** */
static void App_UpdateUartData(uint8_t* pData)
{
    uint16_t u16SerBytesCount = 0;
    if(gSerial_Success_c == Serial_GetByteFromRxBuffer(mAppSerId, pData, &u16SerBytesCount))
    {
        Serial_RxBufferByteCount(mAppSerId, &u16SerBytesCount);
        if(u16SerBytesCount)
        {
            (void)OSA_EventSet(mAppThreadEvt, gCtEvtUart_c);
        }
    } 
}

/*! *********************************************************************************
* \brief  Application initialization. It installs the main menu callbacks and
*         calls the Connectivity Test for Generic FSK init.
*
********************************************************************************** */
static void App_InitApp()
{   

   /*register callbacks for the generic fsk LL */
   GENFSK_RegisterCallbacks(mAppGenfskId,
                            App_GenFskReceiveCallback, 
                            App_GenFskEventNotificationCallback);

   /*init and provide means to notify the app thread from connectivity tests*/
   IT_GenFskInit(App_NotifyAppThread, App_TimerCallback, App_CallArbitraryEvent);
}

/*! *********************************************************************************
* \brief  This function represents the Generic FSK receive callback. 
*         This function is called each time the Generic FSK Link Layer receives a 
*         valid packet
* \param[in]  pBuffer Pointer to receive buffer as byte array
* \param[in]  timestamp Generic FSK timestamp for received packet
* \param[in]  rssi The RSSI measured during the reception of the packet
*
********************************************************************************** */
static void App_GenFskReceiveCallback(uint8_t *pBuffer, 
                                      uint16_t bufferLength, 
                                      uint64_t timestamp, 
                                      uint8_t rssi,
                                      uint8_t crcValid)
{
   mAppRxLatestPacket.pBuffer      = pBuffer;
   mAppRxLatestPacket.bufferLength = bufferLength;
   mAppRxLatestPacket.timestamp    = timestamp;
   mAppRxLatestPacket.rssi         = rssi;
   mAppRxLatestPacket.crcValid     = crcValid;
   
   /*send event to app thread*/
   OSA_EventSet(mAppThreadEvt, EvtRxDone_c);
}

/*! *********************************************************************************
* \brief  This function represents the Generic FSK event notification callback. 
*         This function is called each time the Generic FSK Link Layer has 
*         a notification for the upper layer
* \param[in]  event The event that generated the notification
* \param[in]  eventStatus status of the event
*
********************************************************************************** */
static void App_GenFskEventNotificationCallback(genfskEvent_t event, 
                                                genfskEventStatus_t eventStatus)
{
   if(event & gGenfskTxEvent)
   {
       mAppGenfskStatus = eventStatus;
       /*send event done*/
       OSA_EventSet(mAppThreadEvt, EvtTxDone_c);
   }
   if(event & gGenfskRxEvent)
   {
       if(eventStatus == gGenfskTimeout)
       {
           OSA_EventSet(mAppThreadEvt, EvtSeqTimeout_c);
       }
       else
       {
           OSA_EventSet(mAppThreadEvt, EvtRxFailed_c);
       }
   }
   /*not handling other events in this application*/
}

static void App_SerialCallback(void* param)
{
    OSA_EventSet(mAppThreadEvt, gCtEvtUart_c);
}

static void App_NotifyAppThread(void)
{
    App_NotifySelf();
}

static void App_TimerCallback(void* param)
{
    OSA_EventSet(mAppThreadEvt, gCtEvtTimerExpired_c);
}

static void App_CallArbitraryEvent(ct_event_t eventFlag){
	OSA_EventSet(mAppThreadEvt, eventFlag);
}

