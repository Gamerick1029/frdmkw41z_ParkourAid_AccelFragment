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
#ifndef __GEN_FSK_TESTS_H__
#define __GEN_FSK_TESTS_H__

/*! *********************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
********************************************************************************** */
typedef enum it_led_codes
{
  LedOn			= 0x1,
  LedOff		= 0x2,
  LedToggle		= 0x4,
  LedUnchanged	= 0x8
}it_led_t;

/* Opcodes for received messages */
typedef enum gMessage_Opcode {
	gStringOpcode_c = 0,		//Payload is a string message
	gLedOpcode_c,				//Payload is a sequence of LED commands
	gAccelDataOpcode_c,			//Payload is a new package of accelerometer data
	gAccelDataRequestOpcode_c,	//Empty Payload. A request for accelerometer data
	gIDregReqOpcode_c,			//Empty payload. New Node attempting to register with server
	gIDregAckOpcode_c,			//Sent from server to new Node with a payload containing their ID
	gMessageAckOpcode_c			//Sent from Node to server when a message has been acknowledged
}gMessage_Opcode_t;

typedef enum ct_event_tag
{
  EvtRxDone_c       = 0x00000001U,
  EvtTxDone_c       = 0x00000002U,
  EvtSeqTimeout_c   = 0x00000004U,
  EvtRxFailed_c     = 0x00000008U,
  
  gCtEvtTimerExpired_c = 0x00000010U,
  gCtEvtUart_c         = 0x00000020U,
  gCtEvtAccelRequest_c = 0x00000040U,
  gCtEvtSelfEvent_c    = 0x00000080U,
  
  gCtEvtWakeUp_c       = 0x00000100U,
  
  gCtEvtMaxEvent_c     = 0x00000200U,
  gCtEvtEventsAll_c    = 0x000003FFU
}ct_event_t;

typedef enum ct_param_type_tag{
    gParamTypeNumber_c = 0,
    gParamTypeString_c,
    gParamTypeBool_c,
    gParamTypeMaxType_c
}ct_param_type_t;

typedef struct ct_config_params_tag
{
    ct_param_type_t paramType;
    uint8_t paramName[20];
    union
    {
        uint32_t decValue;
        uint8_t stringValue[4];
        bool_t  boolValue;
    }
    paramValue;
}ct_config_params_t;

typedef struct ct_rx_indication_tag
{
    uint64_t timestamp;
    uint8_t *pBuffer;
    uint16_t bufferLength; 
    uint8_t rssi;
    uint8_t crcValid;
}ct_rx_indication_t;

typedef void (* pHookAppNotification) ( void );
typedef void (* pTmrHookNotification) (void*);
typedef void (* pArbitraryHookNotification) (ct_event_t);
/*! *********************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
********************************************************************************** */
#define gModeRx_c (1)
#define gModeTx_c (2)
#define gDefaultMode_c gModeRx_c

/*tx power*/
#define gGenFskMaxTxPowerLevel_c     (0x20)
#define gGenFskMinTxPowerLevel_c     (0x00)
#define gGenFskDefaultTxPowerLevel_c (0x10)

/*channel*/
#define gGenFskMaxChannel_c     (0x7F)
#define gGenFskMinChannel_c     (0x00)
#define gGenFskDefaultChannel_c (0x2A)
                                        
/*network address*/
#define gGenFskDefaultSyncAddress_c  (0x11002299)
#define gGenFskDefaultSyncAddrSize_c (0x03) /*bytes = size + 1*/

/*the following field sizes must be multiple of 8 bit*/
#define gGenFskDefaultH0FieldSize_c     (8)
#define gGenFskDefaultLengthFieldSize_c (6)
#define gGenFskDefaultH1FieldSize_c     (2)
#define gGenFskDefaultHeaderSizeBytes_c ((gGenFskDefaultH0FieldSize_c + \
                                         gGenFskDefaultLengthFieldSize_c + \
                                             gGenFskDefaultH1FieldSize_c) >> 3)                             
#if gGenFskDefaultLengthFieldSize_c < 3
#error "For this application the length field size should not be less than 3"
#endif

/*payload length*/
#define gGenFskMaxPayloadLen_c ((1 << gGenFskDefaultLengthFieldSize_c) - 1)

/*test opcode + 2byte packet index + 2byte number of packets for PER test*/
#if ((cPWR_UsePowerDownMode) && (cPWR_GENFSK_LL_Enable))
#define gGenFskMinPayloadLen_c (10) 
#else
#define gGenFskMinPayloadLen_c (6) 
#endif /* ((cPWR_UsePowerDownMode) && (cPWR_GENFSK_LL_Enable)) */
#define gGenFskDefaultPayloadLen_c (gGenFskMinPayloadLen_c)

#define gGenFskDefaultMaxBufferSize_c (gGenFskDefaultSyncAddrSize_c + 1 + \
                                       gGenFskDefaultHeaderSizeBytes_c  + \
                                           gGenFskMaxPayloadLen_c)

/*H0 and H1 config*/
#define gGenFskDefaultH0Value_c        (0x0000)
#define gGenFskDefaultH0Mask_c         ((1 << gGenFskDefaultH0FieldSize_c) - 1)

#define gGenFskDefaultH1Value_c        (0x0000)
#define gGenFskDefaultH1Mask_c         ((1 << gGenFskDefaultH1FieldSize_c) - 1)

/*ID config*/
#define serverID 		0
#define clientDefaultID	254
#define broadcastID 	255
/*! *********************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Public function Definitions
*************************************************************************************
********************************************************************************** */
void IT_GenFskInit(pHookAppNotification pFunc, pTmrHookNotification pTmrFunc, pArbitraryHookNotification pArbFunc);
void IT_TxWithPayload(uint8_t *payload, uint8_t payloadLen, uint8_t destinationID);
void IT_Rx(uint8_t waitPeriod);
void IT_HandlePacket();
bool_t IT_RegisterDevice();
uint8_t IT_GetNumberOfDevices();

#endif
