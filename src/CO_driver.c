/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster
 * @copyright   2004 - 2020 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "targetSpecific.h" /* Include HAL interfaces generated by cube MX. */
#include "301/CO_driver.h"

// Static variable allocations
// Header for CAN messages. This header gets copied to the CAN module when a transaction is done
static CAN_TxHeaderTypeDef TxHeader;

// Header for RX CAN message. This header gets set when reading the oldest message in the fifo
static CAN_RxHeaderTypeDef RxHeader;

// This variable must be used AFTER calling CO_CANmodule_init
static CAN_HandleTypeDef *CanHandle = NULL;

// pointer to CO_CanModule used in CubeMX CAN Rx interrupt routine*/
static CO_CANmodule_t* CanFifo_Callback_CanModule_p = NULL;

// Private function declarations
static inline void prepareTxHeader(CAN_TxHeaderTypeDef *TxHeader, CO_CANtx_t *buffer);

// Declare functions here so we can use them
void CO_CANInterruptRx(CO_CANmodule_t *CANmodule);
void CO_CANInterruptTx(CO_CANmodule_t *CANmodule);

// Custom functions for STM32 implementation of CANOpen
static inline void prepareTxHeader(CAN_TxHeaderTypeDef *TxHeader, CO_CANtx_t *buffer) {
	/* Map buffer data to the HAL CAN tx header data*/
	TxHeader->ExtId = 0u;
	TxHeader->IDE = 0;

    // Number of bytes in the messages is the 12th and 13th bit of the identifer
    //TxHeader->DLC = (buffer->ident >> 12U);

	TxHeader->DLC = buffer->DLC;
	TxHeader->StdId = ( buffer->ident >> 2 );

    // Need to remove the bits 12 and 13 as those hold the number of bytes
    // in the message
    //TxHeader->StdId = (buffer->ident & 0x7FF);
	TxHeader->RTR = ( buffer->ident & 0x2 );
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if(CanFifo_Callback_CanModule_p != NULL)
	{
		CO_CANInterruptRx(CanFifo_Callback_CanModule_p);
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if(CanFifo_Callback_CanModule_p != NULL)
	{
		CO_CANInterruptRx(CanFifo_Callback_CanModule_p);
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    if(CanFifo_Callback_CanModule_p != NULL) {
        CO_CANInterruptTx(CanFifo_Callback_CanModule_p);
    }
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    if(CanFifo_Callback_CanModule_p != NULL) {
        CO_CANInterruptTx(CanFifo_Callback_CanModule_p);
    }
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    if(CanFifo_Callback_CanModule_p != NULL) {
        CO_CANInterruptTx(CanFifo_Callback_CanModule_p);
    }
}

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr){
    /* Put CAN module in configuration mode */
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule){
    /* Put CAN module in normal mode */
    if(((CAN_HandleTypeDef *)(CANmodule->CANptr))->Instance == CAN1) {
        if(HAL_CAN_Start(CANmodule->CANptr) == HAL_OK) {
            if(HAL_CAN_ActivateNotification(CanHandle,
                CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY |
                CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR_WARNING)
                != HAL_OK) {
                /* Notification Error */
                CANmodule->errinfo = CO_ERROR_TIMEOUT;
            }
        } else {
            CANmodule->errinfo = CO_ERROR_TIMEOUT;
        }

        CANmodule->CANnormal = true;
        CANmodule->errinfo = CO_ERROR_NO;
    } else {
        CANmodule->errinfo = CO_ERROR_INVALID_STATE;
    }
    
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        void *CANptr,
		CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
    uint16_t i;

    /* verify arguments */
    if(CANmodule==NULL || rxArray==NULL || txArray==NULL){
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    // Save local pointer to the CO_CAN object so it can be used for interrupts
    CanFifo_Callback_CanModule_p = CANmodule;

    /* Configure object variables */
	CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    //CANmodule->useCANrxFilters = (rxSize <= 32U) ? true : false;/* microcontroller dependent */
    CANmodule->useCANrxFilters = false;
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    for(i=0U; i<rxSize; i++){
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for(i=0U; i<txSize; i++){
        txArray[i].bufferFull = false;
    }

    /* Configure CAN module registers */
    // Configuration is done with HAL
    CO_CANmodule_disable(CANmodule);

	// Set the HAL CAN variable to memory address we specified
	CanHandle = CANmodule->CANptr;

	HAL_CAN_MspDeInit(CanHandle);
	HAL_CAN_MspInit(CanHandle); /* NVIC and GPIO */

	CanHandle->Instance = CANx;
	CanHandle->Init.Mode = CAN_MODE_NORMAL;
	CanHandle->Init.SyncJumpWidth = CAN_SJW_1TQ;
	CanHandle->Init.TimeTriggeredMode = DISABLE;
	CanHandle->Init.AutoBusOff = DISABLE;
	CanHandle->Init.AutoWakeUp = DISABLE;
	CanHandle->Init.AutoRetransmission = ENABLE;
	CanHandle->Init.ReceiveFifoLocked = DISABLE;
	CanHandle->Init.TransmitFifoPriority = DISABLE;

    /* Configure CAN timing */
    // Hard coded in rn
    CanHandle->Init.TimeSeg1 = CAN_BS1_13TQ;
    CanHandle->Init.TimeSeg2 = CAN_BS2_2TQ;

    switch(CANbitRate) {
        case 500:
            CanHandle->Init.Prescaler = 4;
            break;
        case 1000:
            CanHandle->Init.Prescaler = 2;
            break;
        default:
         return CO_ERROR_ILLEGAL_BAUDRATE;
    }  

    if(HAL_CAN_Init(CanHandle) != HAL_OK)
    {
        /* Initialization Error */
        return CO_ERROR_TIMEOUT;
    }

    // Reset the bxCan module incase we didn't do a full power off restart
    HAL_CAN_ResetError(CanHandle);

    /* Configure CAN module hardware filters */
    if(CANmodule->useCANrxFilters){
        /* CAN module filters are used, they will be configured with */
        /* CO_CANrxBufferInit() functions, called by separate CANopen */
        /* init functions. */
        /* Configure all masks so, that received message must match filter */
    }
    else{
        /* CAN module filters are not used, all messages with standard 11-bit */
        /* identifier will be received */
        /* Configure mask 0 so, that all messages with standard identifier are accepted */
        // No hardware filters are used
        CAN_FilterTypeDef  FilterConfig;
        FilterConfig.FilterBank = 0;
        FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
        FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
        FilterConfig.FilterIdHigh = 0x0000;
        FilterConfig.FilterIdLow = 0x0000;
        FilterConfig.FilterMaskIdHigh = 0x0000;
        FilterConfig.FilterMaskIdLow = 0x0000;
        FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        FilterConfig.FilterActivation = ENABLE;
        FilterConfig.SlaveStartFilterBank = 14;
        
        if(HAL_CAN_ConfigFilter(CanHandle, &FilterConfig) != HAL_OK)
        {
            /* Filter configuration Error */
            return CO_ERROR_TIMEOUT;
        }
    }
    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule){
    /* turn off the module */
	/* handled by HAL*/
    if(((CAN_HandleTypeDef *)(CANmodule->CANptr))->Instance == CAN1) {
        HAL_CAN_DeactivateNotification(CanHandle,
			CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
	    HAL_CAN_Stop(CanHandle);
    }
	
}


/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*CANrx_callback)(void *object, void *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if((CANmodule!=NULL) && (object!=NULL) && (CANrx_callback!=NULL) && (index < CANmodule->rxSize)){
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
        // Implementation comes from STM32 MX Driver for CANOpen
        buffer->ident = (ident & 0x07FF) << 2;
		if (rtr)
		{
			buffer->ident |= 0x02;
		}
		buffer->mask = (mask & 0x07FF) << 2;
		buffer->mask |= 0x02;

        /* Set CAN hardware module filter and mask. */
        if(CANmodule->useCANrxFilters){
            // TODO: Add hardware filters
            
        } else {

        }
    }
    else{
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if((CANmodule != NULL) && (index < CANmodule->txSize)){
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer.*/
        // Implementation comes from STM32 MX Driver for CANOpen
		buffer->ident &= 0x7FF;
		buffer->ident = ident << 2;
		if (rtr) buffer->ident |= 0x02;

		buffer->DLC = noOfBytes;
		buffer->bufferFull = false;
		buffer->syncFlag = syncFlag;
    }

    return buffer;
}


/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer){
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if(buffer->bufferFull){
        if(!CANmodule->firstCANtxMessage){
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    CO_LOCK_CAN_SEND();
    /* if CAN TX buffer is free, copy message to it */
    if(1 && CANmodule->CANtxCount == 0 && 
      (HAL_CAN_GetTxMailboxesFreeLevel(CanHandle) > 0 )){
        CANmodule->bufferInhibitFlag = buffer->syncFlag;

        // Temp variable to hold which mail box the transmitted message was put into
        uint32_t TxMailboxNum;

        // Convert CANOpen buffer to HAL header
        prepareTxHeader(&TxHeader, buffer);

        /* copy message and txRequest */
        if(HAL_CAN_AddTxMessage(CanHandle,&TxHeader,&buffer->data[0],&TxMailboxNum)	!= HAL_OK)
		{
			err = CO_ERROR_TIMEOUT;
		}
    }
    /* if no buffer is free, message will be sent by interrupt */
    else{
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
    }
    CO_UNLOCK_CAN_SEND();

    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule){
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND();
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */

    // TODO: Add PDO removal if used
    // if ((HAL_CAN_IsTxMessagePending(CANmodule->CANbaseAddress) && (CANmodule->bufferInhibitFlag))
    // {
    // HAL_CAN_AbortTxRequest(CANmodule->);
    // }


    if(/*messageIsOnCanBuffer && */CANmodule->bufferInhibitFlag){
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if(CANmodule->CANtxCount != 0U){
        uint16_t i;
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for(i = CANmodule->txSize; i > 0U; i--){
            if(buffer->bufferFull){
                if(buffer->syncFlag){
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND();


    if(tpdoDeleted != 0U){
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}


/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
    * different way to determine errors. */
static uint16_t rxErrors=0, txErrors=0, overflow=0;

void CO_CANmodule_process(CO_CANmodule_t *CANmodule) {
    uint32_t err;

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;

    if (CANmodule->errOld != err) {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (txErrors >= 256U) {
            /* bus off */
            status |= CO_CAN_ERRTX_BUS_OFF;
        }
        else {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF |
                                CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE |
                                CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

            /* rx bus warning or passive */
            if (rxErrors >= 128) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRRX_WARNING;
            }

            /* tx bus warning or passive */
            if (txErrors >= 128) {
                status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRTX_WARNING;
            }

            /* if not tx passive clear also overflow */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0) {
                status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        if (overflow != 0) {
            /* CAN RX bus overflow */
            status |= CO_CAN_ERRRX_OVERFLOW;
        }

        CANmodule->CANerrorStatus = status;
    }
}


/******************************************************************************/
void CO_CANInterruptRx(CO_CANmodule_t *CANmodule) {
    CO_CANrxMsg_t rcvMsg;      /* pointer to received message in CAN module */
    uint16_t index;             /* index of received message */
    uint32_t rcvMsgIdent;       /* identifier of the received message */
    CO_CANrx_t *buffer = NULL;  /* receive message buffer from CO_CANmodule_t object. */
    bool_t msgMatched = false;

    memset(&rcvMsg,0,sizeof(CO_CANrxMsg_t));
    /* get message from module here */

    HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, rcvMsg.data);

    // Extract data from HAL RxHeader to CANOpen variables
    rcvMsg.DLC = RxHeader.DLC;
    rcvMsg.ident = RxHeader.StdId;

    rcvMsgIdent = rcvMsg.ident << 2;
    if(CANmodule->useCANrxFilters){
        /* CAN module filters are used. Message with known 11-bit identifier has */
        /* been received */
        index = 0;  /* get index of the received message here. Or something similar */
        if(index < CANmodule->rxSize){
            buffer = &CANmodule->rxArray[index];
            /* verify also RTR */
            if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U){
                msgMatched = true;
            }
        }
    }
    else{
        /* CAN module filters are not used, message with any standard 11-bit identifier */
        /* has been received. Search rxArray form CANmodule for the same CAN-ID. */
        buffer = &CANmodule->rxArray[0];
        for(index = CANmodule->rxSize; index > 0U; index--){
            if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U){
                msgMatched = true;
                break;
            }
            buffer++;
        }
    }

    /* Call specific function, which will process the message */
    if(msgMatched && (buffer != NULL) && (buffer->CANrx_callback != NULL)){
        buffer->CANrx_callback(buffer->object, (void*) &rcvMsg);
    }

    /* Clear interrupt flag */
    // HAL clears the flag for us
}

void CO_CANInterruptTx(CO_CANmodule_t *CANmodule){
    /* Clear interrupt flag */

    /* First CAN message (bootup) was sent successfully */
    CANmodule->firstCANtxMessage = false;
    /* clear flag from previous message */
    CANmodule->bufferInhibitFlag = false;
    /* Are there any new messages waiting to be send */
    if(CANmodule->CANtxCount > 0U){
        uint16_t i;             /* index of transmitting message */

        /* first buffer */
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        /* search through whole array of pointers to transmit message buffers. */
        for(i = CANmodule->txSize; i > 0U; i--){
            /* if message buffer is full, send it. */
            if(buffer->bufferFull){
                buffer->bufferFull = false;
                CANmodule->CANtxCount--;

                /* Copy message to CAN buffer */
                CANmodule->bufferInhibitFlag = buffer->syncFlag;
                
                uint32_t TxMailboxNum;

                prepareTxHeader(&TxHeader, buffer);
                if(HAL_CAN_AddTxMessage(CanHandle,
                        &TxHeader,
                        &buffer->data[0],
                        &TxMailboxNum) != HAL_OK){
                    ;//do nothing
                }
                else
                {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                }
                break;                      /* exit for loop */
            }
            buffer++;
        }/* end of for loop */

        /* Clear counter if no more messages */
        if(i == 0U){
            CANmodule->CANtxCount = 0U;
        }
    }
}