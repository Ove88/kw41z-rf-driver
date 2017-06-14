/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyStateMachine.c
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/



/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"
#include "fsl_device_registers.h"

#include "PhyInterface.h"
#include "Phy.h"
#include "FunctionLib.h"

/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#define mPhyMaxIdleRxDuration_c         (0xF00000) /* [sym] */
#define mPhyOverhead_d                  (30)       /* [sym] */
#define mPhyMaxFrameDuration_d (gPhySHRDuration_c + (1 + gMaxPHYPacketSize_c) * gPhySymbolsPerOctet_c + gPhyTurnaroundTime_c + 54)
//#define mPhyMinRxDuration_d mPhyMaxFrameDuration_d
#define mPhyMinRxDuration_d (gPhyTurnaroundTime_c + 22)

/*! *********************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */
static phyStatus_t Phy24Task(Phy_PhyLocalStruct_t *pPhyStruct, macToPdDataMessage_t *pMsgIn );

static phyStatus_t Phy_HandlePdDataReq( Phy_PhyLocalStruct_t *pPhyData, macToPdDataMessage_t * pMsg );

static void Phy_EnterIdle( Phy_PhyLocalStruct_t *pPhyData );

static void PLME_SendMessage(Phy_PhyLocalStruct_t *pPhyStruct, phyMessageId_t msgType);

static void PD_SendMessage(Phy_PhyLocalStruct_t *pPhyStruct, phyMessageId_t msgType);

static void Phy_SendLatePD( uint32_t param );
static void Phy_SendLatePLME( uint32_t param );


/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */
Phy_PhyLocalStruct_t phyLocal;
static pdDataToMacMessage_t rxData;
static uint8_t pPsdu[127];

//uint8_t mXcvrDisallowSleep = 0;
const uint8_t gPhyPoolId = gPhyPoolId_d;


/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
* \brief  This function creates the PHY task
*
********************************************************************************** */
void Phy_Init(void)
{
    PhyHwInit();
    PhyTime_TimerInit(NULL);

    phyLocal.flags = gPhyFlagDeferTx_c;

    rxData.msgData.dataInd.pPsdu = &pPsdu;
    phyLocal.rxParams.pRxData = &rxData;   

    PhyIsrPassRxParams( NULL );
    PhyPlmeSetPwrState( gPhyDefaultIdlePwrMode_c );
}

/*! *********************************************************************************
* \brief  This function binds a MAC instance to a PHY instance
*
* \param[in]  instanceId The instance of the MAC
*
* \return  The instance of the PHY.
*
********************************************************************************** */
instanceId_t BindToPHY( instanceId_t macInstance )
{
    return 0;
}

/*! *********************************************************************************
* \brief  This function registers the MAC PD and PLME SAP handlers
*
* \param[in]  pPD_MAC_SapHandler   Pointer to the MAC PD handler function
* \param[in]  pPLME_MAC_SapHandler Pointer to the MAC PLME handler function
* \param[in]  instanceId           The instance of the PHY
*
* \return  The status of the operation.
*
********************************************************************************** */
void Phy_RegisterSapHandlers( PD_MAC_SapHandler_t pPD_MAC_SapHandler,
                              PLME_MAC_SapHandler_t pPLME_MAC_SapHandler,
                              instanceId_t instanceId )
{
    instanceId = instanceId;
    phyLocal.PD_MAC_SapHandler = pPD_MAC_SapHandler;
    phyLocal.PLME_MAC_SapHandler = pPLME_MAC_SapHandler;
}

/*! *********************************************************************************
* \brief  This function represents the PHY's task
*
* \param[in]  taskParam The instance of the PHY
*
********************************************************************************** */
static phyStatus_t Phy24Task(Phy_PhyLocalStruct_t *pPhyStruct, macToPdDataMessage_t *pMsgIn)
{
    uint8_t state;
    phyStatus_t status;

    ProtectFromXcvrInterrupt();
    
    status = gPhySuccess_c;
    state = PhyGetSeqState();
    
    /* Check if PHY is busy */
    if( (state != gIdle_c) && (state != gRX_c) )
    {
        status = gPhyBusy_c;
    }
    else
    {
        pPhyStruct->currentMacInstance = pMsgIn->macInstance;
    }
    
    if( gPhyBusy_c == status )
    {
       return;
    }
    
    if( gRX_c == state )
    {
        state = gIdle_c;
        PhyPlmeForceTrxOffRequest();
    }

    if( status == gPhySuccess_c )
    {
        pPhyStruct->flags &= ~(gPhyFlagIdleRx_c);
        
        switch( pMsgIn->msgType )
        {
        case gPdDataReq_c:
            status = Phy_HandlePdDataReq( pPhyStruct, pMsgIn );
            break;
        case gPlmeCcaReq_c:
            status = PhyPlmeCcaEdRequest(gPhyCCAMode1_c, gPhyContCcaDisabled);
            break;
        case gPlmeEdReq_c:
            status = PhyPlmeCcaEdRequest(gPhyEnergyDetectMode_c, gPhyContCcaDisabled);
            break;
        default:
            status = gPhyInvalidPrimitive_c;
            break;
        }
    }
    
    /* Check status */
    if( gPhySuccess_c != status )
    {
        switch( pMsgIn->msgType )
        {
        case gPdDataReq_c:
            PD_SendMessage(pPhyStruct, gPdDataCnf_c);
            break;
            /* Fallthorough */
        case gPlmeCcaReq_c:
            pPhyStruct->channelParams.channelStatus = gPhyChannelBusy_c;
            PLME_SendMessage(pPhyStruct, gPlmeCcaCnf_c);
            break;
        case gPlmeEdReq_c:
            pPhyStruct->channelParams.energyLeveldB = 0;
            PLME_SendMessage(pPhyStruct, gPlmeEdCnf_c);
            break;
        default:
            PLME_SendMessage(pPhyStruct, gPlmeTimeoutInd_c);
            break;
        }
    } 
    UnprotectFromXcvrInterrupt();
    
    /* Check if PHY can enter Idle state */
    if( gIdle_c == PhyGetSeqState() )
    {
        Phy_EnterIdle( pPhyStruct );
    }
    
    return status;
}

/*! *********************************************************************************
* \brief  This is the PD SAP message handler
*
* \param[in]  pMsg Pointer to the PD request message
* \param[in]  instanceId The instance of the PHY
*
* \return  The status of the operation.
*
********************************************************************************** */
phyStatus_t MAC_PD_SapHandler(macToPdDataMessage_t *pMsg, instanceId_t phyInstance)
{
    phyStatus_t result = gPhySuccess_c;
    uint8_t baseIndex = 0;

    phyInstance = phyInstance;

    if( NULL == pMsg )
    {
        result = gPhyInvalidParameter_c;
    }
    else
    {       
        switch( pMsg->msgType )
        {
        case gPdIndQueueInsertReq_c:
            result = PhyPp_IndirectQueueInsert(baseIndex + pMsg->msgData.indQueueInsertReq.index,
                                               pMsg->msgData.indQueueInsertReq.checksum,
                                               phyInstance);
            break;
            
        case gPdIndQueueRemoveReq_c:
            result = PhyPp_RemoveFromIndirect(baseIndex + pMsg->msgData.indQueueRemoveReq.index,
                                              phyInstance);
            break;
            
        case gPdDataReq_c:

            result = Phy24Task(&phyLocal, pMsg);
            break;
            
        default:
            result = gPhyInvalidPrimitive_c;
            break;
        }
    }
    
    return result;
}

/*! *********************************************************************************
* \brief  This is the PLME SAP message handler
*
* \param[in]  pMsg Pointer to the PLME request message
* \param[in]  instanceId The instance of the PHY
*
* \return  phyStatus_t The status of the operation.
*
********************************************************************************** */
phyStatus_t MAC_PLME_SapHandler(macToPlmeMessage_t * pMsg, instanceId_t phyInstance)
{
    Phy_PhyLocalStruct_t *pPhyStruct = &phyLocal;
    phyStatus_t result = gPhySuccess_c;
    uint8_t phyRegSet = 0;

    if( NULL == pMsg )
    {
        result = gPhyInvalidParameter_c;
    }
    else
    {
        switch( pMsg->msgType )
        {
        case gPlmeEdReq_c:
        case gPlmeCcaReq_c:

            Phy24Task( &phyLocal, pMsg);
            break;
            
        case gPlmeSetReq_c:

            result = PhyPlmeSetPIBRequest(pMsg->msgData.setReq.PibAttribute, pMsg->msgData.setReq.PibAttributeValue, phyRegSet, phyInstance);
            break;
            
        case gPlmeGetReq_c:

            result = PhyPlmeGetPIBRequest( pMsg->msgData.getReq.PibAttribute, pMsg->msgData.getReq.pPibAttributeValue, phyRegSet, phyInstance);
            break;
            
        case gPlmeSetTRxStateReq_c:
            if(gPhySetRxOn_c == pMsg->msgData.setTRxStateReq.state)
            {
                /* Compensate Rx warmup time */
                if( pMsg->msgData.setTRxStateReq.startTime != gPhySeqStartAsap_c )
                {
                    pMsg->msgData.setTRxStateReq.startTime -= ((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_SHIFT) >> 4; /* /16 */
                }

                pMsg->msgData.setTRxStateReq.rxDuration += ((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_SHIFT) >> 4; /* /16 */
                

                if( PhyIsIdleRx(phyInstance) )
                {
                    PhyPlmeForceTrxOffRequest();
                }
                else 
                {
                    if( gIdle_c != PhyGetSeqState() )
                    {
                        result = gPhyBusy_c;
                        break;
                    }
                }

                pPhyStruct->flags &= ~(gPhyFlagIdleRx_c);
                
                Phy_SetSequenceTiming(pMsg->msgData.setTRxStateReq.startTime,
                                      pMsg->msgData.setTRxStateReq.rxDuration);
                
                result = PhyPlmeRxRequest(pMsg->msgData.setTRxStateReq.slottedMode, (phyRxParams_t *) &pPhyStruct->rxParams);
                break;
            }
            else
            {
                if (gPhyForceTRxOff_c == pMsg->msgData.setTRxStateReq.state)
                {

                    pPhyStruct->flags &= ~(gPhyFlagIdleRx_c);
                    PhyPlmeForceTrxOffRequest();
                }
            }
            break;
            
        default:
            result = gPhyInvalidPrimitive_c;
            break;
        }
    }
    return result;
}

/*! *********************************************************************************
* \brief  This function programs a new TX sequence
*
* \param[in]  pMsg Pointer to the PD request message
* \param[in]  pPhyData pointer to PHY data
*
* \return  The status of the operation.
*
********************************************************************************** */
static phyStatus_t Phy_HandlePdDataReq( Phy_PhyLocalStruct_t *pPhyData, macToPdDataMessage_t * pMsg )
{
    phyStatus_t status = gPhySuccess_c;
    phyTime_t startTime = gPhySeqStartAsap_c;
    phyTime_t t;
    
    if( NULL == pMsg->msgData.dataReq.pPsdu )
    {
        status = gPhyInvalidParameter_c;
    }
    else
    {
        ProtectFromXcvrInterrupt();
        
        /* Compensate XCVR Tx warmup time */
        if( pMsg->msgData.dataReq.startTime != gPhySeqStartAsap_c )
        {
            startTime = pMsg->msgData.dataReq.startTime - (((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_SHIFT) >> 4); /* /16 */
            PhyTimeSetEventTrigger( startTime );
        }

        status = PhyPdDataRequest(&pMsg->msgData.dataReq , &pPhyData->rxParams, &pPhyData->txParams);
        
        t = PhyTime_GetTimestamp();
        
        if( t > startTime )
        {
            status = gPhyTRxOff_c;
        }
        else 
        {
            if( pMsg->msgData.dataReq.txDuration != 0xFFFFFFFFU )
            {
                if( startTime != gPhySeqStartAsap_c )
                {
                    t = startTime + pMsg->msgData.dataReq.txDuration;
                }
                else
                {
                    t += pMsg->msgData.dataReq.txDuration;
                }
                /* Compensate PHY overhead */
                t += ((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_SHIFT) >> 4; /* /16 */
                t += 10;
                PhyTimeSetEventTimeout( &t );
            }
        }
        
        UnprotectFromXcvrInterrupt();
        
        if( gPhySuccess_c != status )
        {
            PhyPlmeForceTrxOffRequest();
        }
    }
    return status;
}

/*! *********************************************************************************
* \brief  This function sets the start time and the timeout value for a sequence.
*
* \param[in]  startTime The absolute start time for the sequence.
*             If startTime is gPhySeqStartAsap_c, the start timer is disabled.
* \param[in]  seqDuration The duration of the sequence.
*             If seqDuration is 0xFFFFFFFF, the timeout is disabled.
*
********************************************************************************** */
void Phy_SetSequenceTiming(phyTime_t startTime, uint32_t seqDuration)
{
    phyTime_t endTime;

    OSA_InterruptDisable();

    if( gPhySeqStartAsap_c == startTime )
    {
        PhyTimeReadClock( &endTime );
    }
    else
    {
        PhyTimeSetEventTrigger( startTime );
        endTime = startTime & gPhyTimeMask_c;
    }

    if( 0xFFFFFFFFU != seqDuration )
    {
        endTime += seqDuration;
        endTime = endTime & gPhyTimeMask_c;

        PhyTimeSetEventTimeout( &(endTime) );
    }

    OSA_InterruptEnable();
}

/*! *********************************************************************************
* \brief  This function starts the IdleRX if the PhyRxOnWhenIdle PIB is set
*
* \param[in]  pPhyData pointer to PHY data
*
********************************************************************************** */
static void Phy_EnterIdle( Phy_PhyLocalStruct_t *pPhyData )
{
    if( (pPhyData->flags & gPhyFlagRxOnWhenIdle_c))
    {
        uint32_t t = mPhyMaxIdleRxDuration_c;
        
        pPhyData->flags |= gPhyFlagIdleRx_c;
        Phy_SetSequenceTiming( gPhySeqStartAsap_c, t );
        PhyPlmeRxRequest( gPhyUnslottedMode_c, &pPhyData->rxParams );  
    }
    else
    {
        pPhyData->flags &= ~(gPhyFlagIdleRx_c);

        // if( mXcvrDisallowSleep && (gIdle_c == PhyGetSeqState()) )
        // {
        //     mXcvrDisallowSleep = 0;
        //     PWR_AllowXcvrToSleep();
        // }
    }
}

/*! *********************************************************************************
* \brief  This function sets the value of the maxFrameWaitTime PIB
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  maxTime The maxFrameWaitTime value
*
********************************************************************************** */
void PhyPlmeSetFrameWaitTime( uint32_t maxTime, instanceId_t instanceId )
{
    phyLocal.maxFrameWaitTime = maxTime;
}

/*! *********************************************************************************
* \brief  This function sets the state of the PhyRxOnWhenIdle PIB
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  state The PhyRxOnWhenIdle value
*
********************************************************************************** */
void PhyPlmeSetRxOnWhenIdle( bool_t state, instanceId_t instanceId )
{
    uint8_t radioState = PhyGetSeqState();

    if( state )
    {
        phyLocal.flags |= gPhyFlagRxOnWhenIdle_c;

    }
    else
    {
        phyLocal.flags &= ~gPhyFlagRxOnWhenIdle_c;
        if( (phyLocal.flags & gPhyFlagIdleRx_c) && (radioState == gRX_c) )
        {
            PhyPlmeForceTrxOffRequest();
        }
    }

    if( gIdle_c == PhyGetSeqState() )
    {
        Phy_EnterIdle( &phyLocal );
    }
}

/*! *********************************************************************************
* \brief  This function starts the IdleRX if the PhyRxOnWhenIdle PIB is set
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
bool_t PhyIsIdleRx( instanceId_t instanceId )
{
    bool_t status = FALSE;
    uint8_t state = PhyGetSeqState();

    if( (phyLocal.flags & gPhyFlagIdleRx_c) && (gRX_c == state) )
    {
        status = TRUE;
    }

    return status;
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a TX operation completed successfully.
*         If the received ACK has FP=1, then the radio will enter RX state for
*         maxFrameWaitTime duration.
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  framePending The value of the framePending bit for the received ACK
*
********************************************************************************** */
void Radio_Phy_PdDataConfirm(instanceId_t instanceId, bool_t framePending)
{
    PhyTimeDisableEventTimeout();

    if( framePending )
    {
        phyLocal.flags |= gPhyFlagRxFP_c;
        if( phyLocal.maxFrameWaitTime > 0 )
        {
            /* Restart Rx asap if an ACK with FP=1 is received */
            phyLocal.flags &= ~(gPhyFlagIdleRx_c);
            Phy_SetSequenceTiming( gPhySeqStartAsap_c, phyLocal.maxFrameWaitTime );
            PhyPlmeRxRequest( gPhyUnslottedMode_c, &phyLocal.rxParams );
        }
    }
    else
    {
        phyLocal.flags &= ~gPhyFlagRxFP_c;
    }

    PD_SendMessage(&phyLocal, gPdDataCnf_c);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that new data has been received
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_PdDataIndication(instanceId_t instanceId)
{
    PhyTimeDisableEventTimeout();

    PD_SendMessage(&phyLocal, gPdDataInd_c);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that timer1 compare match occured
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_TimeWaitTimeoutIndication(instanceId_t instanceId)
{
    PhyTime_ISR();
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a CCA sequence has finished
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  phyChannelStatus The status of the channel: Idle/Busy
*
* \return  None.
*
********************************************************************************** */
void Radio_Phy_PlmeCcaConfirm(phyStatus_t phyChannelStatus, instanceId_t instanceId)
{
    PhyTimeDisableEventTimeout();

    phyLocal.channelParams.channelStatus = phyChannelStatus;

    PLME_SendMessage(&phyLocal, gPlmeCcaCnf_c);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a ED sequence has finished
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  energyLevel The enetgy level on the channel.
* \param[in]  energyLeveldB The energy level in DB
*
********************************************************************************** */
void Radio_Phy_PlmeEdConfirm(uint8_t energyLeveldB, instanceId_t instanceId)
{
    PhyTimeDisableEventTimeout();

    phyLocal.channelParams.energyLeveldB = energyLeveldB;

    PLME_SendMessage(&phyLocal, gPlmeEdCnf_c);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that the programmed sequence has timed out
*         The Radio is forced to Idle.
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_TimeRxTimeoutIndication(instanceId_t instanceId)
{
    if( (phyLocal.flags & gPhyFlagIdleRx_c) != gPhyFlagIdleRx_c )
    {
        PLME_SendMessage(&phyLocal, gPlmeTimeoutInd_c);
    }
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that the programmed sequence has started
*
* \param[in]  instanceId The instance of the PHY
*
* \return  None.
*
********************************************************************************** */
void Radio_Phy_TimeStartEventIndication(instanceId_t instanceId)
{
#ifdef MAC_PHY_DEBUG
    PLME_SendMessage(&phyLocal[instanceId], gPlme_StartEventInd_c);
#endif
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that the specified Rx watermark has been reached.
*         Also, if there is not enough time to receive the entire packet, the
*         RX timeout will be extended.
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  frameLen the length of the PSDU
*
********************************************************************************** */
void Radio_Phy_PlmeRxWatermark(instanceId_t instanceId, uint32_t frameLength)
{
    /* In DualMode operation, sequence timeouts are strict, and cannot be extended. */
    if( (phyLocal.flags & gPhyFlagDeferTx_c) == gPhyFlagDeferTx_c )
    {
        phyTime_t currentTime, t;

        OSA_InterruptDisable();

        /* Read currentTime and Timeout values [sym] */
        PhyTimeReadClock(&currentTime);
        /* Convert to symbols and add IFS and ACK duration */
        frameLength = frameLength * 2 + 12 + 22 + 2;

        t = (PhyTimeGetEventTimeout() - currentTime) & gPhyTimeMask_c;

        if( t > 1 )
        {
            /* Disable TMR3 compare */
            ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_TMR3CMP_EN_MASK;
            /* Write new TMR3 compare value */
            currentTime += frameLength;
            ZLL->T3CMP = currentTime;
            /* Enable TMR3 compare */
            ZLL->PHY_CTRL |= ZLL_PHY_CTRL_TMR3CMP_EN_MASK;
        }

        OSA_InterruptEnable();
    }

#ifdef MAC_PHY_DEBUG
    PLME_SendMessage(&phyLocal[instanceId], gPlme_RxSfdDetectInd_c);
#endif
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a Sync Loss occured (PLL unlock)
*         The Radio is forced to Idle.
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_PlmeSyncLossIndication(instanceId_t instanceId)
{
    PhyPlmeForceTrxOffRequest();
#ifdef MAC_PHY_DEBUG
    PLME_SendMessage(&phyLocal[instanceId], gPlme_SyncLossInd_c);
#endif
    Radio_Phy_TimeRxTimeoutIndication(instanceId);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a Filter Fail occured
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_PlmeFilterFailRx(instanceId_t instanceId)
{
#ifdef MAC_PHY_DEBUG
    PLME_SendMessage(&phyLocal[instanceId], gPlme_FilterFailInd_c);
#endif
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that an unexpected Transceiver Reset
*          occured and force the TRX to Off
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_UnexpectedTransceiverReset(instanceId_t instanceId)
{
    PhyPlmeForceTrxOffRequest();
#ifdef MAC_PHY_DEBUG
    PLME_SendMessage(&phyLocal[instanceId], gPlme_UnexpectedRadioResetInd_c);
#endif
    Radio_Phy_TimeRxTimeoutIndication(instanceId);
}

/*! *********************************************************************************
* \brief  Senf a PLME message to upper layer
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  msgType    The type of message to be sent
*
********************************************************************************** */
static void PLME_SendMessage(Phy_PhyLocalStruct_t *pPhyStruct, phyMessageId_t msgType)
{
    static plmeToMacMessage_t  staticMsg;
    plmeToMacMessage_t * pMsg = &staticMsg;
    
    if( pPhyStruct->PLME_MAC_SapHandler )
    {
          
        if(NULL == pMsg)
        {
            pMsg = (plmeToMacMessage_t *)Phy_BufferAlloc/*Forever*/(sizeof(plmeToMacMessage_t));
        }
              
        pMsg->msgType = msgType;
        
        switch(msgType)
        {
        case gPlmeCcaCnf_c:
            pMsg->msgData.ccaCnf.status = pPhyStruct->channelParams.channelStatus;
            break;
            
        case gPlmeEdCnf_c:
            pMsg->msgData.edCnf.status        = gPhySuccess_c;
            pMsg->msgData.edCnf.energyLeveldB = pPhyStruct->channelParams.energyLeveldB;
            pMsg->msgData.edCnf.energyLevel   = Phy_GetEnergyLevel(pPhyStruct->channelParams.energyLeveldB);
            break;
            
        default:
            /* No aditional info needs to be filled */
            break;
        }
            
            pPhyStruct->PLME_MAC_SapHandler(pMsg, pPhyStruct->currentMacInstance);
            //Phy_BufferFree(pMsg);
    }
}

/*! *********************************************************************************
* \brief  Senf a PD message to upper layer
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  msgType    The type of message to be sent
*
********************************************************************************** */
static void PD_SendMessage(Phy_PhyLocalStruct_t *pPhyStruct, phyMessageId_t msgType)
{
    static pdDataToMacMessage_t  staticMsg;
    pdDataToMacMessage_t *pMsg;
    if( pPhyStruct->PD_MAC_SapHandler )
    {
        if( msgType == gPdDataInd_c )
        {
            uint32_t temp;
            uint16_t len = pPhyStruct->rxParams.psduLength - 2; /* Excluding FCS (2 bytes) */
            
            pMsg = pPhyStruct->rxParams.pRxData;
            //pPhyStruct->rxParams.pRxData = NULL;
            
            FLib_MemCpy((uint8_t *)(pMsg->msgData.dataInd.pPsdu), (void*)ZLL->PKT_BUFFER_RX, len);
            
            pMsg->msgType                         = gPdDataInd_c;
            pMsg->msgData.dataInd.ppduLinkQuality = pPhyStruct->rxParams.linkQuality;
            pMsg->msgData.dataInd.psduLength      = len;
            
            pMsg->msgData.dataInd.timeStamp       = PhyTime_GetTimestamp();      /* current timestamp (64bit) */
            temp = (uint32_t)(pMsg->msgData.dataInd.timeStamp & gPhyTimeMask_c); /* convert to 24bit */
            pMsg->msgData.dataInd.timeStamp -= (temp - pPhyStruct->rxParams.timeStamp) & gPhyTimeMask_c;

            pPhyStruct->PD_MAC_SapHandler(pMsg, pPhyStruct->currentMacInstance);
        }
        else
        {
            phyStatus_t status;
            pMsg = &staticMsg;
            
            if( (pPhyStruct->flags & gPhyFlagRxFP_c) == gPhyFlagRxFP_c )
            {
                pPhyStruct->flags &= ~(gPhyFlagRxFP_c);
                status = gPhyFramePending_c;
            }
            else
            {
                status = gPhySuccess_c;
            }
            
                       
            if(NULL == pMsg)
            {                
                pMsg = (pdDataToMacMessage_t *)Phy_BufferAlloc/*Forever*/(sizeof(phyMessageHeader_t) + sizeof(pdDataCnf_t));
            }
                      
            pMsg->msgType = gPdDataCnf_c;
            pMsg->msgData.dataCnf.status = status;
            pPhyStruct->PD_MAC_SapHandler(pMsg, pPhyStruct->currentMacInstance);
            //Phy_BufferFree(pMsg);
        }
    }
}


static void Phy_SendLatePLME( uint32_t param )
{
    PLME_SendMessage(&phyLocal, (phyMessageId_t)param);
}

static void Phy_SendLatePD( uint32_t param )
{
    PD_SendMessage(&phyLocal, (phyMessageId_t)param);
}

void Radio_Phy_Notify(void)
{
    /* Check if PHY can enter Idle state */
    if( gIdle_c == PhyGetSeqState() )
    {
        Phy_EnterIdle( &phyLocal );
    }
}