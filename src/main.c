/**
  ******************************************************************************
  * @file    UART/UART_Printf/Src/main.c
  * @author  MCD Application Team
  * @brief   This example shows how to retarget the C library printf function
  *          to the UART.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "targetCommon.h"
#include "config.h"
#include "Uart.h"
#include "DataCollection.h"
#include "stdio.h"
#include "stdbool.h"


/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_Printf
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef debugUartHandle;
const uint32_t debugUartBaudRate = 115200;


CAN_HandleTypeDef     CanHandle;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;
uint32_t              RxMailbox;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static void setupDebugUart(UART_HandleTypeDef *huart, uint32_t buadRate);
static HAL_StatusTypeDef CAN_Polling(void);

bool collectData();
bool collectDataUART();

/* Private functions ---------------------------------------------------------*/

int setup(void) {
  /* STM32F103xB HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 64 MHz */
  SystemClock_Config();

  /* Initialize BSP Led for LED2 */
  BSP_LED_Init(LED2);

  setupDebugUart(&debugUartHandle,debugUartBaudRate);

  return 0;
  }

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  // Run the set up code 
  setup();
  

  // How to enable millis counter...

  char *msg = "Starting\r\n";

  UART_putString(&debugUartHandle, msg); 

  UART_putString(&debugUartHandle, "Temp\r\n");

  bool newData = false;
  uint32_t lastDataCollectTime = 0U;

  CAN_Polling();

  /* Main Loop */
  // while (1)
  // {
  //   // Get new sensor Data
  //   // Periodic task that runs every few milliseconds
  //   if(lastDataCollectTime > HAL_GetTick() + DATA_COLLECTION_RATE) {
  //     collectDataUART();
      
  //     //collectData();

  //     //filterData()

  //     newData = true;
  //   }

  //   // Send sensor data if requested

  //   // Validate data is within range

  // }
}

/*
 * PURPOSE
 *      This function is used to be a substitute for using the real collect data function. This
 *      function should be used with a program to send sensor data over UART. The UART being use
 *      should be set up ahead of time and be a global UART
 * PARAMETERS
 *      None
 * RETURNS
 *      bool - whether data was collected without error
 */
bool collectDataUART() {
    // Collect data from UART
    uint8_t dataBuff[sizeof(struct ShockSensorData)];

    int status = HAL_UART_Receive(&debugUartHandle,&dataBuff,
                                  sizeof(struct ShockSensorData),0xffff);

    // Copy the received data to the buffer
    memcpy(&sensorDataBuffer,dataBuff,sizeof(struct ShockSensorData));

    return (status != HAL_OK);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int _write(int fd, char * ptr, int len)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&debugUartHandle, (uint8_t *)ptr, len, 0xFFFF); 

  return len;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Configure PLL ------------------------------------------------------*/
  /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
  /* Enable HSI and activate PLL with HSi_DIV2 as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
  oscinitstruct.HSEState        = RCC_HSE_OFF;
  oscinitstruct.LSEState        = RCC_LSE_OFF;
  oscinitstruct.HSIState        = RCC_HSI_ON;
  oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI_DIV2;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 on */
  
  while (1)
  {
    BSP_LED_On(LED2);
    HAL_Delay(500);
    BSP_LED_Off(LED2);
    HAL_Delay(500);
  }
}

static void setupDebugUart(UART_HandleTypeDef *huart, uint32_t buadRate) {
  debugUartHandle.Instance        = USARTx;

  debugUartHandle.Init.BaudRate   = debugUartBaudRate;
  debugUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  debugUartHandle.Init.StopBits   = UART_STOPBITS_1;
  debugUartHandle.Init.Parity     = UART_PARITY_NONE;
  debugUartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  debugUartHandle.Init.Mode       = UART_MODE_TX_RX;
  if (HAL_UART_Init(&debugUartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
  * @brief  Configures the CAN, transmit and receive by polling
  * @param  None
  * @retval PASSED if the reception is well done, FAILED in other case
  */
HAL_StatusTypeDef CAN_Polling(void)
{
  CAN_FilterTypeDef  sFilterConfig;
  
  /*##-1- Configure the CAN peripheral #######################################*/
  CanHandle.Instance = CANx;
    
  CanHandle.Init.TimeTriggeredMode = DISABLE;
  CanHandle.Init.AutoBusOff = DISABLE;
  CanHandle.Init.AutoWakeUp = DISABLE;
  CanHandle.Init.AutoRetransmission = DISABLE;
  CanHandle.Init.ReceiveFifoLocked = DISABLE;
  CanHandle.Init.TransmitFifoPriority = DISABLE;
  CanHandle.Init.Mode = CAN_MODE_NORMAL;
  CanHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
  CanHandle.Init.TimeSeg1 = CAN_BS1_13TQ;
  CanHandle.Init.TimeSeg2 = CAN_BS2_6TQ;
  CanHandle.Init.Prescaler = 4;

  // Used this website to get config values
  // http://www.bittiming.can-wiki.info/
  // Used a APB2 clock of 32 MHz and a CAN baud rate of 500kbps
  
  if(HAL_CAN_Init(&CanHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the CAN Filter ###########################################*/
  // sFilterConfig.FilterBank = 0;
  // sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  // sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  // sFilterConfig.FilterIdHigh = 0x0000;
  // sFilterConfig.FilterIdLow = 0x0000;
  // sFilterConfig.FilterMaskIdHigh = 0x0000;
  // sFilterConfig.FilterMaskIdLow = 0x0000;
  // sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  // sFilterConfig.FilterActivation = ENABLE;
  // sFilterConfig.SlaveStartFilterBank = 14;
  
  // if(HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
  // {
  //   /* Filter configuration Error */
  //   Error_Handler();
  // }

  /*##-3- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(&CanHandle) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  // Wait for main controller to send a request
  UART_putString(&debugUartHandle,"Waiting for message\r\n");
  while(HAL_CAN_GetRxFifoFillLevel(&CanHandle,CAN_RX_FIFO0) == 0);

  if(HAL_CAN_GetRxMessage(&CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

  if((RxHeader.StdId != 0x11)                     ||
     (RxHeader.RTR != CAN_RTR_DATA)               ||
     (RxHeader.IDE != CAN_ID_STD)                 ||
     (RxHeader.DLC != 2))
  {
    /* Rx message Error */
    return HAL_ERROR;
  }

  UART_putString(&debugUartHandle,"Got CAN message\r\n");
  UART_putString(&debugUartHandle,RxData);

  // Send requested data
  TxHeader.StdId = 0x11;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 2;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxData[0] = 10U;
  
  /* Request transmission */
  if(HAL_CAN_AddTxMessage(&CanHandle, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
  }
  
  /* Wait transmission complete */
  while(HAL_CAN_GetTxMailboxesFreeLevel(&CanHandle) != 3) {}
  

  return HAL_OK; /* Test Passed */
}

