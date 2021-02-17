/*
 * CANopen main program file.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        main_generic.c
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


#include "targetSpecific.h"
#include "targetCommon.h"
//#include "config.h"
#include "Uart.h"
//#include "DataCollection.h"
#include "stdio.h"
#include "stdbool.h"
#include "CANopen.h"


#define TMR_TASK_INTERVAL   (1000)          /* Interval of tmrTask thread in microseconds */
#define INCREMENT_1MS(var)  (var++)         /* Increment 1ms variable in tmrTask */

#define log_printf(macropar_message, ...) \
        printf(macropar_message, ##__VA_ARGS__)


/* Global variables and objects */
volatile uint16_t   CO_timer1ms = 0U;   /* variable increments each millisecond */
uint8_t LED_red, LED_green;

/* UART handler declaration */
UART_HandleTypeDef debugUartHandle;
const uint32_t debugUartBaudRate = 115200;

CAN_HandleTypeDef     CanHandle;
TIM_HandleTypeDef msTimer = {.Instance = TIM4};
bool nmtChanged = false;

// LED values and pins
#define GREEN_LED_PIN D8
#define RED_LED_PIN D9
volatile uint32_t ledBlinkRate = 1000;

/* Local function definitons */
int setup(void);
void SystemClock_Config(void);
static void Error_Handler(void);
static void setupDebugUart(UART_HandleTypeDef *huart, uint32_t buadRate);
void NMT_Changed_Callback(CO_NMT_internalState_t state);

/* setup **********************************************************************/
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

    setupDebugUart(&debugUartHandle,debugUartBaudRate);


    /* Initialize BSP Led for LED2 */
    BSP_LED_Init(LED2);
    BspGpioInitOutput(GREEN_LED_PIN);
    BspGpioInitOutput(RED_LED_PIN);

    return 0;
}


/* main ***********************************************************************/
int main (void){
  CO_ReturnError_t err;
  CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
  uint32_t heapMemoryUsed;
  void *CANmoduleAddress = &CanHandle; /* CAN module address */
  uint8_t activeNodeId = 0x20; /* Copied from CO_pendingNodeId in the communication reset section */
  uint16_t pendingBitRate = 500;  /* read from dip switches or nonvolatile memory, configurable by LSS slave */

  /* Configure microcontroller. */
  setup();

  /* Allocate memory but these are statically allocated so no malloc */
  err = CO_new(&heapMemoryUsed);
  if (err != CO_ERROR_NO) {
      log_printf("Error: Can't allocate memory\r\n");
      return 0;
  }
  else {
      log_printf("Allocated %d bytes for CANopen objects\r\n", heapMemoryUsed);
  }

  /* initialize EEPROM */


  /* increase variable each startup. Variable is stored in EEPROM. */
  //OD_powerOnCounter++;

  //log_printf("CANopenNode - Reset application, count = %d\r\n", OD_powerOnCounter);


  while(reset != CO_RESET_APP){
/* CANopen communication reset - initialize CANopen objects *******************/
    uint16_t timer1msPrevious;

    log_printf("CANopenNode - Reset communication...\r\n");

    /* disable CAN and CAN interrupts */

    /* initialize CANopen */
    err = CO_CANinit(CANmoduleAddress, pendingBitRate);
    if (err != CO_ERROR_NO) {
        log_printf("Error: CAN initialization failed: %d\r\n", err);
        return 0;
    }
    // err = CO_LSSinit(&pendingNodeId, &pendingBitRate);
    // if(err != CO_ERROR_NO) {
    //     log_printf("Error: LSS slave initialization failed: %d\r\n", err);
    //     return 0;
    // }
    err = CO_CANopenInit(activeNodeId);
    if(err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        log_printf("Error: CANopen initialization failed: %d\r\n", err);
        return 0;
    }

    // // Need to start CAN module after setting filters
    // if (HAL_CAN_Start(&CanHandle) != HAL_OK)
    // {
    //   /* Start Error */
    //   Error_Handler();
    // }

    // Reset timer just inscase the registers were not reset
    HAL_TIM_Base_Stop_IT(&msTimer);
    HAL_TIM_Base_DeInit(&msTimer);
    

    //HAL_TIM_ConfigClockSource()
    /* Configure Timer interrupt function for execution every 1 millisecond */
    // Using timer 4
    // Timer 4 input clock is APB1
    // As of now APB1 is 32Mhz
    // The Timer 4 clock input is multiplied by 2 so 64 Mhz.
    __TIM4_CLK_ENABLE();
    
    // Input = 64 Mhz
    // Interal divider = 1
    // Prescaler = 64
    // Clock rate = (Input)/(Interal divider * prescaler)
    //            = (64 MHz)/(64) = 1 Mhz

    // Prescaler and period need to be subtracted by 1 to count at the right rate
    msTimer.Init.Prescaler = 64-1;
    msTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
    msTimer.Init.Period = 1000-1;
    msTimer.Init.AutoReloadPreload = 0;
    msTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    msTimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    // Initalize timer device
    HAL_TIM_Base_Init(&msTimer);

    

    // Enable interrupts for timer
    HAL_TIM_Base_Start_IT(&msTimer);

    /* Configure CAN transmit and receive interrupt */
    // As of now we are not using interrupts

    // /* Configure CANopen callbacks, etc */
    // if(!CO->nodeIdUnconfigured) {

    // }

    // Set up NMT call back function to print out messages when state has changed
    CO_NMT_initCallbackChanged(CO->NMT, NMT_Changed_Callback);

    /* start CAN */
    CO_CANsetNormalMode(CO->CANmodule[0]);

    reset = CO_RESET_NOT;
    timer1msPrevious = CO_timer1ms;

    log_printf("CANopenNode - Running...\r\n");
    fflush(stdout);

    uint32_t lastLedBlinkTime = HAL_GetTick();

    while(reset == CO_RESET_NOT){
/* loop for normal program execution ******************************************/
        uint16_t timer1msCopy, timer1msDiff;

        timer1msCopy = CO_timer1ms;
        timer1msDiff = timer1msCopy - timer1msPrevious;
        timer1msPrevious = timer1msCopy;


        /* CANopen process */
        reset = CO_process(CO, (uint32_t)timer1msDiff*1000, NULL);

        // Handle LED Updates
        LED_red = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
        LED_green = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);

        BspGpioWrite(GREEN_LED_PIN,LED_green);
        BspGpioWrite(RED_LED_PIN,LED_red);

        /* Nonblocking application code may go here. */
        if(HAL_GetTick() - lastLedBlinkTime > ledBlinkRate) {
          lastLedBlinkTime = HAL_GetTick();
          BSP_LED_Toggle(LED2);
        }
        /* Process EEPROM */

        /* optional sleep for short time */
    }
  }


/* program exit ***************************************************************/
    /* stop threads */


    /* delete objects from memory */
    CO_delete((void*) &CanHandle);

    log_printf("CANopenNode finished\r\n");

    /* reset */
    return 0;
}


/* timer thread executes in constant intervals ********************************/
void tmrTask_thread(void){
  INCREMENT_1MS(CO_timer1ms);
  if(CO->CANmodule[0]->CANnormal) {
      bool_t syncWas;

      /* Process Sync */
      syncWas = CO_process_SYNC(CO, TMR_TASK_INTERVAL, NULL);

      /* Read inputs */
      CO_process_RPDO(CO, syncWas);

      /* Further I/O or nonblocking application code may go here. */

      /* Write outputs */
      CO_process_TPDO(CO, syncWas, TMR_TASK_INTERVAL, NULL);

      /* verify timer overflow */
      if(0) {
          CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0U);
      }
  }
}


/* CAN interrupt function executes on received CAN message ********************/
void /* interrupt */ CO_CAN1InterruptHandler(void){
  nmtChanged = true;
    /* clear interrupt flag */
}

void NMT_Changed_Callback(CO_NMT_internalState_t state) {
  switch (CO->NMT->operatingState)
  {
  case CO_NMT_INITIALIZING:
    ledBlinkRate = 2000;
    break;
  case CO_NMT_STOPPED:
    ledBlinkRate = 100;
    break;
  case CO_NMT_PRE_OPERATIONAL:
    ledBlinkRate = 1000;
    break;
  case CO_NMT_OPERATIONAL:
    ledBlinkRate = 500;
    break;
  default:
    ledBlinkRate = 100;
    break;
  }
}

// Micro controller specific function calls
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int _write(int fd, char * ptr, int len){
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
void SystemClock_Config(void){
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
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void){
  /* Turn LED2 on */
  
  while (1)
  {
    BSP_LED_On(LED2);
    HAL_Delay(500);
    BSP_LED_Off(LED2);
    HAL_Delay(500);
  }
}

void TIM4_IRQHandler(void) {
    HAL_TIM_IRQHandler(&msTimer);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM4) {
        tmrTask_thread();
    }
    
    
}
