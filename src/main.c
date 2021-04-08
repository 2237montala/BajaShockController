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
#include "config.h"
#include "Uart.h"
#include "DataCollection.h"
#include "stdio.h"
#include "stdbool.h"
#include "CANopen.h"

// Includes for acclerometer
#include "I2C.h"
#include "LIS3DH.h"

#define TMR_TASK_INTERVAL   (1000)          /* Interval of tmrTask thread in microseconds */
#define INCREMENT_1MS(var)  (var++)         /* Increment 1ms variable in tmrTask */

#define log_printf(macropar_message, ...) \
        printf(macropar_message, ##__VA_ARGS__)


/* Global variables and objects */
volatile uint16_t   CO_timer1ms = 0U;   /* variable increments each millisecond */
uint8_t LED_red, LED_green;

/* UART handler declaration */
UART_HandleTypeDef debugUartHandle;

CAN_HandleTypeDef     CanHandle;
TIM_HandleTypeDef coThreadTimer = {.Instance = TIM4};
bool nmtChanged = false;

// LED values and pins

volatile uint32_t ledBlinkRate = 1000;

/* Local function definitons */
int setupMicro(void);
void SystemClock_Config(void);
static void Error_Handler(void);
static void setupDebugUart(UART_HandleTypeDef *huart, uint32_t buadRate);
void NMT_Changed_Callback(CO_NMT_internalState_t state);
void printSync(void *object);
void systemSoftwareReset();
HAL_StatusTypeDef configCOThreadTimer(TIM_HandleTypeDef *timer, uint32_t abp2Clock, uint32_t inputDivider);
void stopCOThreadTimer(TIM_HandleTypeDef *timer);
void startCOThreadTimer(TIM_HandleTypeDef *timer);
void setSensorDataToCoTdpoData();

/* setup **********************************************************************/
int setupMicro(void) {
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

    #ifdef DEBUG_UART_ON
      setupDebugUart(&debugUartHandle,DEBUG_UART_BAUD_RATE);
    #endif

    /* Initialize BSP Led for LED2 */
    BSP_LED_Init(LED2);
    BspGpioInitOutput(GREEN_LED_PIN);
    BspGpioInitOutput(RED_LED_PIN);
    BspGpioInit(NODE_ID_BIT_ONE_PIN,INPUT_PULLUP);
    BspGpioInit(NODE_ID_BIT_TWO_PIN,INPUT_PULLUP);

    #ifdef DEBUG_GPIO_ON
      BspGpioInitOutput(DEBUG_GPIO_PIN);
    #endif

    // Set up I2C
    I2cInit(I2Cx,I2C_HIGH_SPEED,I2C_DUTYCYCLE_2,HAL_I2C_MODE_MASTER,0x00);
    HAL_Delay(50);

    return 0;
}

bool setupSensors(void) {
  // Start accelerometer
  bool temp = Lis3dhInit(LIS3DH_DEFAULT_ADDRESS,LIS3DH_DEFAULT_WAI);
  if(!temp) return false;

  // Set range of accelerometer
  switch(ACCELEROMETER_G_RANGE) {
    case ACCLEROMETER_2G_RANGE:
      temp = Lis3dhSetRange(ACCLEROMETER_2G_RANGE);
      break;

    case ACCLEROMETER_4G_RANGE:
      temp = Lis3dhSetRange(ACCLEROMETER_4G_RANGE);
      break;

    case ACCLEROMETER_8G_RANGE:
      temp = Lis3dhSetRange(ACCLEROMETER_8G_RANGE);
      break;

    case ACCLEROMETER_16G_RANGE:
      temp = Lis3dhSetRange(ACCLEROMETER_16G_RANGE);
      break;

    default:
      temp = Lis3dhSetRange(ACCELEROMETER_DEFAULT_RANGE);
      break;
  }
  if(!temp) return false;

  return temp;
}

uint8_t setupNodeId() {
  uint8_t tempNodeId = NODE_ID_BASE;
  // Read the pins for their bit status
  if(BspGpioRead(NODE_ID_BIT_ONE_PIN) == GPIO_PIN_SET) {
    tempNodeId += 0x1;
  }
  if(BspGpioRead(NODE_ID_BIT_TWO_PIN) == GPIO_PIN_SET) {
    tempNodeId += 0x2;
  }
  return tempNodeId;
}

/* main ***********************************************************************/
int main (void){
  CO_ReturnError_t err;
  CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
  uint32_t heapMemoryUsed;
  void *CANmoduleAddress = &CanHandle; /* CAN module address */
  uint8_t currNodeId = 0;

  /* Configure microcontroller. */
  setupMicro();

  // Set up sensors
  if(!setupSensors()) {
    log_printf("Error setting up sensors\r\n");
    Error_Handler();
  }

  // TODO: check if sensor values are within expected range
  // Check sensor value for errors

  // Calculate the node's CAN id based on on the dip switch position
  // Connect to ground is a 0
  currNodeId = setupNodeId();

  /* Allocate memory but these are statically allocated so no malloc */
  err = CO_new(&heapMemoryUsed);
  if (err != CO_ERROR_NO) {
      #ifdef DEBUG_UART_ON
        log_printf("Error: Can't allocate memory\r\n");
      #endif
      return 0;
  }
  else {
    #ifdef DEBUG_UART_ON
      log_printf("Allocated %lu bytes for CANopen objects\r\n", heapMemoryUsed);
    #endif
  }

  while(reset != CO_RESET_APP){
/* CANopen communication reset - initialize CANopen objects *******************/
    uint16_t timer1msPrevious;

    #ifdef DEBUG_UART_ON
      log_printf("CANopenNode - Reset communication...\r\n");
    #endif

    /* disable CAN and CAN interrupts */

    /* initialize CANopen */
    err = CO_CANinit(CANmoduleAddress, CAN_BAUD_RATE);
    if (err != CO_ERROR_NO) {
        #ifdef DEBUG_UART_ON
          log_printf("Error: CAN initialization failed: %d\r\n", err);
        #endif
        return 0;
    }

    err = CO_CANopenInit(currNodeId);
    if(err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
      #ifdef DEBUG_UART_ON
        log_printf("Error: CANopen initialization failed: %d\r\n", err);
      #endif
      return 0;
    }

    #ifdef DEBUG_UART_ON
      printf("Starting with node id: 0x%x\r\n",currNodeId);
    #endif

    // Reset timer just inscase the registers were not reset
    stopCOThreadTimer(&coThreadTimer);

    /* Configure Timer interrupt function for execution every 1 millisecond */
    // Using timer 4
    // Timer 4 input clock is APB1
    // As of now APB1 is 32Mhz
    // The Timer 4 clock input is multiplied by 2 so 64 Mhz.    
    // Input = 64 Mhz
    if(configCOThreadTimer(&coThreadTimer,64,TIM_CLOCKDIVISION_DIV1) == HAL_ERROR) {
      Error_Handler();
    };

    // Start CO thread timer
    startCOThreadTimer(&coThreadTimer);

    // Set up NMT call back function to print out messages when state has changed
    CO_NMT_initCallbackChanged(CO->NMT, NMT_Changed_Callback);

    /* start CAN */
    CO_CANsetNormalMode(CO->CANmodule[0]);

    reset = CO_RESET_NOT;
    timer1msPrevious = CO_timer1ms;

    #ifdef DEBUG_UART_ON
    log_printf("CANopenNode - Running...\r\n");
    fflush(stdout);
    #endif

    uint32_t lastLedBlinkTime = HAL_GetTick();
    uint32_t lastDataCollectTime = lastLedBlinkTime;
    //uint32_t lastBitTime = lastLedBlinkTime;
    uint32_t loopMsValue = 0;

    while(reset == CO_RESET_NOT){
    /* loop for normal program execution ******************************************/
        #ifdef DEBUG_GPIO_ON
          BspGpioWrite(DEBUG_GPIO_PIN,GPIO_PIN_SET);
        #endif
        // Toggle an gpio to do some timing

        uint16_t timer1msCopy, timer1msDiff;

        timer1msCopy = CO_timer1ms;
        timer1msDiff = timer1msCopy - timer1msPrevious;
        timer1msPrevious = timer1msCopy;

        loopMsValue = HAL_GetTick();

        /* CANopen process */
        reset = CO_process(CO, (uint32_t)timer1msDiff*1000, NULL);
        if(reset == CO_RESET_APP) {
          // Do a software reset
          systemSoftwareReset();
        }

        // Handle LED Updates
        LED_red = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
        LED_green = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);

        BspGpioWrite(GREEN_LED_PIN,LED_green);
        BspGpioWrite(RED_LED_PIN,LED_red);

        /* Nonblocking application code may go here. */
        if(loopMsValue - lastLedBlinkTime > ledBlinkRate) {
          lastLedBlinkTime = HAL_GetTick();
          BSP_LED_Toggle(LED2);
        }

        // Collect new sensor data if we passed our interval
        if(loopMsValue - lastDataCollectTime > DATA_COLLECTION_RATE) {
          lastDataCollectTime = HAL_GetTick();
          collectData();
        }
        
        #ifdef DEBUG_GPIO_ON
          BspGpioWrite(DEBUG_GPIO_PIN,GPIO_PIN_RESET);
        #endif

        /* optional sleep for short time */
    }
  }

  // We shouldn't get here
  // this means a reset quit was sent
  BspGpioWrite(GREEN_LED_PIN,1);
  BspGpioWrite(RED_LED_PIN,1);

/* program exit ***************************************************************/
  /* stop threads */


  /* delete objects from memory */
  CO_delete((void*) &CanHandle);

  #ifdef DEBUG_UART_ON
    log_printf("CANopenNode finished\r\n");
  #endif

  /* reset */
  return 0;
}


/* timer thread executes in constant intervals ********************************/
void tmrTask_thread(void){
  BspGpioWrite(DEBUG_GPIO_PIN,1);
  INCREMENT_1MS(CO_timer1ms);
  if(CO->CANmodule[0]->CANnormal) {
      bool_t syncWas;

      /* Process Sync */
      syncWas = CO_process_SYNC(CO, TMR_TASK_INTERVAL, NULL);

      /* Read inputs */
      CO_process_RPDO(CO, syncWas);

      /* Further I/O or nonblocking application code may go here. */
      // If we got a sync then our implementation means we are sending the data
      // tpdos now so filter them and update the object directory
      if(syncWas) {
        filterData();
        setSensorDataToCoTdpoData();
        //UART_putStringNL(&debugUartHandle,"send");
      }

      /* Write outputs */
      CO_process_TPDO(CO, syncWas, TMR_TASK_INTERVAL, NULL);

      /* verify timer overflow */
      if(0) {
          CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0U);
      }
  }
  BspGpioWrite(DEBUG_GPIO_PIN,0);
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

  #ifdef DEBUG_UART_ON
    printf("Entering state %d\r\n",state);
  #endif
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
  /* PLL configuration: PLLCLK = (HSE / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
  /* Enable HSI and activate PLL with HSi_DIV2 as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.LSEState        = RCC_LSE_OFF;
  oscinitstruct.HSIState        = RCC_HSI_OFF;
  oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV2;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
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

  /* Output clock on MCO1 pin(PA8) for debugging purpose */
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);
}

static void setupDebugUart(UART_HandleTypeDef *huart, uint32_t buadRate) {
  debugUartHandle.Instance        = USARTx;

  debugUartHandle.Init.BaudRate   = buadRate;
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


// This function is called when any timer hits its period value
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == coThreadTimer.Instance) {
        // Run the next iteration of the CO interrupt
        tmrTask_thread();
    }
}

// Configure a timer to run at a 1ms interrupt rate. The goal is to make the clock input 1MHz
HAL_StatusTypeDef configCOThreadTimer(TIM_HandleTypeDef *timer, uint32_t abp2Clock, uint32_t inputDivider) {   
    if(IS_TIM_CLOCKDIVISION_DIV(inputDivider) == false) {
      return HAL_ERROR;
    }

    if(abp2Clock == 0) {
      return HAL_ERROR;
    }

    // Convert input divider to its numerical representation
    switch ((inputDivider))
    {
    case TIM_CLOCKDIVISION_DIV1:
      inputDivider = 1;
      break;
    case TIM_CLOCKDIVISION_DIV2:
      inputDivider = 2;
      break;
    case TIM_CLOCKDIVISION_DIV4:
      inputDivider = 4;
      break;
    default:
      inputDivider = 1;
      break;
    }


    // Input = 64 Mhz
    // Interal divider = 1
    // Prescaler = 64
    // Clock rate = (Input)/(Interal divider * prescaler)
    //            = (64 MHz)/(64) = 1 Mhz

    uint32_t prescalar = abp2Clock / inputDivider;
    if(prescalar < 0) {
      return HAL_ERROR;
    }

    // Prescaler and period need to be subtracted by 1 to count at the right rate
    timer->Init.Prescaler = prescalar-1;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = 1000-1;
    timer->Init.AutoReloadPreload = 0;
    timer->Init.ClockDivision = inputDivider;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    return HAL_OK;
}

// Stops the timer passed into it
void stopCOThreadTimer(TIM_HandleTypeDef *timer) {
  // Stop timer
  HAL_TIM_Base_Stop_IT(timer);

  // Prevent interrupts from firing
  HAL_TIM_Base_DeInit(timer);
}

// Starts the timer passed into it
void startCOThreadTimer(TIM_HandleTypeDef *timer) {
  // Initalize timer device
  HAL_TIM_Base_Init(timer);

  // Enable interrupts for timer
  HAL_TIM_Base_Start_IT(timer);
}

void systemSoftwareReset() {
  NVIC_SystemReset();
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
  #ifdef DEBUG_UART_ON
    printf("CAN TEC Error: %lu\r\n",(hcan->Instance->ESR & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos);
    printf("CAN REC Error: %lu\r\n",(hcan->Instance->ESR & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos);
    printf("CAN error: 0x%lx\r\n",hcan->ErrorCode);
  #endif
}

// Since CanOpen uses the object dictionary for sending data we need to update the
// internal arrays. Calling this function will set the CANOpen TDPO data structures
// equal to the newest filtered sensor data
void setSensorDataToCoTdpoData() {

  // Copy accelerations over
  memcpy(OD_readShockAccel,getMostRecentFilteredSensorData()->accels,(sizeof(REAL32) * ODL_readShockAccel_arrayLength));

}
