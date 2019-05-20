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

#include "MemManager.h"
#include "RNG_Interface.h"
#include "TimersManager.h"
#if ((cPWR_UsePowerDownMode) && (cPWR_GENFSK_LL_Enable))
#include "PWR_Interface.h"
#endif /* ((cPWR_UsePowerDownMode) && (cPWR_GENFSK_LL_Enable)) */

#include "genfsk_interface.h"
#include "gen_fsk_interaction.h"
#include "xcvr_test_fsk.h"
#include "SerialManager.h"
#include "LED.h"

/*! *********************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */
/*serial interface id*/
uint8_t mAppSerId;
/*timers manager app timer id*/
uint8_t mAppTmrId;
/*GENFSK instance id*/
uint8_t mAppGenfskId;

/*! *********************************************************************************
*************************************************************************************
* Private type declaration
*************************************************************************************
************************************************************************************/
typedef enum g_message_byte_loc{
	gByteDestinationID_c,
    gBytePayloadStart_c
}g_message_byte_loc_t;


/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
void IT_SetTxPayload(uint8_t *payload, uint8_t payloadLen);
void IT_TransmitBuffer(uint8_t txBuffer);
void IT_ServerProcessMessage();
void IT_NodeProcessMessage();
/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
/* buffers for interaction with Generic FSK */
static uint8_t* gRxBuffer;
static uint8_t* gTxBuffer;

/* Generic FSK packets to get formatted data*/
static GENFSK_packet_t gRxPacket;
static GENFSK_packet_t gTxPacket;

static uint8_t* payloadBuilder;

/*hook to notify app thread*/
static pHookAppNotification pNotifyAppThread = NULL;
/*hook to notify app thread*/
static pTmrHookNotification pTmrCallback = NULL;

static pArbitraryHookNotification pSendEvent = NULL;

/*packet configuration*/
static GENFSK_packet_config_t pktConfig = 
{
    .preambleSizeBytes = 0, /*1 byte of preamble*/
    .packetType = gGenfskFormattedPacket,
    .lengthSizeBits = gGenFskDefaultLengthFieldSize_c,
    .lengthBitOrder = gGenfskLengthBitLsbFirst,
    /*sync address bytes = size + 1*/
    .syncAddrSizeBytes = gGenFskDefaultSyncAddrSize_c,
    .lengthAdjBytes = 3, /*length field not including CRC so adjust by crc len*/
    .h0SizeBits = gGenFskDefaultH0FieldSize_c,
    .h1SizeBits = gGenFskDefaultH1FieldSize_c,
    .h0Match = gGenFskDefaultH0Value_c, /*match field containing zeros*/
    .h0Mask = gGenFskDefaultH0Mask_c,
    .h1Match = gGenFskDefaultH1Value_c,
    .h1Mask = gGenFskDefaultH1Mask_c
};

/*CRC configuration*/
static GENFSK_crc_config_t crcConfig =
{
    .crcEnable = gGenfskCrcEnable,
    .crcSize = 3,
    .crcStartByte = 4,
    .crcRefIn = gGenfskCrcInputNoRef,
    .crcRefOut = gGenfskCrcOutputNoRef,
    .crcByteOrder = gGenfskCrcLSByteFirst,
    .crcSeed = 0x00555555,
    .crcPoly = 0x0000065B,
    .crcXorOut = 0
};

/*whitener configuration*/
static GENFSK_whitener_config_t whitenConfig = 
{
    .whitenEnable = gGenfskWhitenEnable,
    .whitenStart = gWhitenStartWhiteningAtH0,
    .whitenEnd = gWhitenEndAtEndOfCrc,
    .whitenB4Crc = gCrcB4Whiten,
    .whitenPolyType = gGaloisPolyType,
    .whitenRefIn = gGenfskWhitenInputNoRef,
    .whitenPayloadReinit = gGenfskWhitenNoPayloadReinit,
    .whitenSize = 7,
    .whitenInit = 0x53,
    .whitenPoly = 0x11, /*x^7 + x^4 + 1! x^7 is never set*/
    .whitenSizeThr = 0,
    .manchesterEn = gGenfskManchesterDisable,
    .manchesterStart = gGenfskManchesterStartAtPayload,
    .manchesterInv = gGenfskManchesterNoInv,
};

/*radio configuration*/
static GENFSK_radio_config_t radioConfig = 
{
    .radioMode = gGenfskGfskBt0p5h0p5,
    .dataRate = gGenfskDR1Mbps
};

/*bit processing configuration*/

/*network / sync address configuration*/
static GENFSK_nwk_addr_match_t ntwkAddr = 
{
    .nwkAddrSizeBytes = gGenFskDefaultSyncAddrSize_c,
    .nwkAddrThrBits = 0,
    .nwkAddr = gGenFskDefaultSyncAddress_c,
}; 

/* LED code translation */
static uint8_t LEDS[5] = {
	0x00,
	LED1,
	LED2,
	LED3,
	LED4,
};

/* This boards (hopefully) unique ID */
static uint8_t deviceID = serverID;

static uint8_t nextID;
/**********************************************************************************/
void IT_GenFskInit(pHookAppNotification pFunc, pTmrHookNotification pTmrFunc, pArbitraryHookNotification pArbFunc)
{

    /*configure hook*/
    pNotifyAppThread = pFunc;
    
    /*configure timer callback*/
    pTmrCallback = pTmrFunc;
    
    /* Configure arbitrary event callback */
    pSendEvent = pArbFunc;

    /* allocate once to use for the entire application */
    gRxBuffer  = MEM_BufferAlloc(gGenFskDefaultMaxBufferSize_c + 
                                 crcConfig.crcSize);
    gTxBuffer  = MEM_BufferAlloc(gGenFskDefaultMaxBufferSize_c);
    
    gRxPacket.payload = (uint8_t*)MEM_BufferAlloc(gGenFskMaxPayloadLen_c  + 
                                                       crcConfig.crcSize);
    gTxPacket.payload = (uint8_t*)MEM_BufferAlloc(gGenFskMaxPayloadLen_c);
    
    payloadBuilder = MEM_BufferAlloc(gGenFskMaxPayloadLen_c);

    /*prepare the part of the tx packet that is common for all tests*/
    gTxPacket.addr = gGenFskDefaultSyncAddress_c;
    gTxPacket.header.h0Field = gGenFskDefaultH0Value_c;
    gTxPacket.header.h1Field = gGenFskDefaultH1Value_c;
    gTxPacket.header.lengthField = gGenFskDefaultPayloadLen_c;
    
    /*set bitrate*/
    GENFSK_RadioConfig(mAppGenfskId, &radioConfig);
    /*set packet config*/
    GENFSK_SetPacketConfig(mAppGenfskId, &pktConfig);
    /*set whitener config*/
    GENFSK_SetWhitenerConfig(mAppGenfskId, &whitenConfig);
    /*set crc config*/
    GENFSK_SetCrcConfig(mAppGenfskId, &crcConfig);
    
    /*set network address at location 0 and enable it*/
    GENFSK_SetNetworkAddress(mAppGenfskId, 0, &ntwkAddr);
    GENFSK_EnableNetworkAddress(mAppGenfskId, 0);
    
    /*set tx power level*/
    GENFSK_SetTxPowerLevel(mAppGenfskId, gGenFskDefaultTxPowerLevel_c);
    /*set channel: Freq = 2360MHz + ChannNumber*1MHz*/
    GENFSK_SetChannelNumber(mAppGenfskId, gGenFskDefaultChannel_c);

    nextID = 1;
}

void IT_TxWithPayload(uint8_t *payload, uint8_t payloadLen, uint8_t destinationID){
	//Set up message MetaData
	payloadBuilder[gByteDestinationID_c] = destinationID;
	FLib_MemCpy(&payloadBuilder[gBytePayloadStart_c], payload, payloadLen);

	IT_SetTxPayload(payloadBuilder, gBytePayloadStart_c+payloadLen);

	GENFSK_PacketToByteArray(mAppGenfskId, &gTxPacket, gTxBuffer);

	uint8_t buffLen = gTxPacket.header.lengthField+
	                    (gGenFskDefaultHeaderSizeBytes_c)+
	                        (gGenFskDefaultSyncAddrSize_c + 1);

	GENFSK_AbortAll();
	GENFSK_StartTx(mAppGenfskId, gTxBuffer, buffLen, 0);
}

void IT_SetTxPayload(uint8_t *payload, uint8_t payloadLen){
	FLib_MemSet(gTxPacket.payload, 0, payloadLen);
	FLib_MemCpy(gTxPacket.payload, payload, payloadLen);
	gTxPacket.header.lengthField = payloadLen;
}

void IT_Rx(uint8_t waitPeriod){
	GENFSK_StartRx(mAppGenfskId, gRxBuffer, gGenFskDefaultMaxBufferSize_c + crcConfig.crcSize, 0, waitPeriod);
}

/*
 *
 * Current maximum message size = 64 bytes (including opcode) where 1 byte is system reserved
 * Any messages longer than that will not be received at the other end
 *
 * Format of a string message:
 * 		1 byte for gStringOpcode
 * 		Rest of payload for a null-terminated string
 * 		Varies in length
 *
 * Format of an LED message:
 * 		1 byte for gLedOpcode
 * 		1 byte per LED (4 LEDs in total) using the it_led_t type to determine the action
 * 		Payload precisely 5 bytes in length (1 for opcode, 1 for each LED)
 */
void IT_HandlePacket(){
	GENFSK_ByteArrayToPacket(mAppGenfskId, gRxBuffer, &gRxPacket);

	//Check destination ID
			if (gRxPacket.payload[gByteDestinationID_c] != broadcastID &&
				gRxPacket.payload[gByteDestinationID_c] != deviceID){
				return;
			}

			// Check payload type
			switch (gRxPacket.payload[gBytePayloadStart_c]){
			case gAccelDataOpcode_c:
				;
				uint8_t message[5] = {
						4,
						gAccelDataOpcode_c,
						gRxPacket.payload[gBytePayloadStart_c + 1],
						gRxPacket.payload[gBytePayloadStart_c + 2],
						gRxPacket.payload[gBytePayloadStart_c + 3]
				};
				Serial_SyncWrite(mAppSerId, message, 5);
			case gIDregReqOpcode_c:
				//TODO: Device registering
				break;
			case gMessageAckOpcode_c:
				//TODO: Message Acknowledgements -> Robust communications
				break;
			default:
				Serial_Print(mAppSerId, "Couldn't determine packet type", 1);
			}
}

uint8_t IT_GetNumberOfDevices(){
	return nextID-1;
}





