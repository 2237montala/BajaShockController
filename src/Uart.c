#include "Uart.h"
#include "string.h"

int UART_Init(UART_HandleTypeDef *UartHandle) {
    return HAL_UART_Init(UartHandle);
}

int UART_putData(UART_HandleTypeDef *huart,uint8_t ptr, int len) {
    return HAL_UART_Transmit(huart, ptr, len, 0xFFFF); 
}

int UART_putString(UART_HandleTypeDef *huart,char *str) {
    return HAL_UART_Transmit(huart, (uint8_t *)str, strlen(str), 0xFFFF); 
}

int UART_putStringNL(UART_HandleTypeDef *huart,char *str) {
    uint32_t bytesSent = 0;
    bytesSent += HAL_UART_Transmit(huart, (uint8_t *)str, strlen(str), 0xFFFF); 

    #if (ADD_CR == 1)
        bytesSent += HAL_UART_Transmit(huart, (uint8_t *)"\r\n", strlen(UART_NEW_LINE), 0xFFFF); 
    #else
        HAL_UART_Transmit(huart, (uint8_t *)"\n", strlen("\n"), 0xFFFF); 
    #endif
    
    return bytesSent;
}