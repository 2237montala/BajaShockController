#include "I2C.h"

/*
 * HEADER NAME : I2C.c
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 03/09/2021
 * DESCRIPTION :
 */

// Create private access variable to the I2c device
// Only support one i2c device right now cause that is all I need
I2C_HandleTypeDef I2cHandle;
HAL_I2C_ModeTypeDef deviceMode;

bool I2cInit(I2C_TypeDef *instance, uint32_t i2cSpeed, uint32_t dutyCycleMode,
             HAL_I2C_ModeTypeDef i2cType, uint8_t i2cAddress) {
    // Copy the users i2c config to the hardware
    I2cHandle.Instance = instance;
    I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    I2cHandle.Init.ClockSpeed      = i2cSpeed;
    I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2cHandle.Init.DutyCycle       = dutyCycleMode;
    I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    // Set address of device if we are going to be in slave mode
    if(i2cType == HAL_I2C_MODE_SLAVE) {
        I2cHandle.Init.OwnAddress1     = i2cAddress;
        I2cHandle.Init.OwnAddress2     = 0x7F;
    } else {
        I2cHandle.Init.OwnAddress1     = 0x7F;
        I2cHandle.Init.OwnAddress2     = 0x7F;
    }
    deviceMode = i2cType;

    if(HAL_I2C_Init(&I2cHandle) != HAL_OK) {
        /* Initialization Error */
        return false;  
    }

    // Need to enable master acknowledgement as it is not included in the HAL
    // __HAL_I2C_DISABLE(&I2cHandle);

    // MODIFY_REG((&I2cHandle)->Instance->CR1,I2C_CR1_ACK,I2C_CR1_ACK);

    // __HAL_I2C_ENABLE(&I2cHandle);

    //Reset DR register
    (&I2cHandle)->Instance->DR = 0;
    uint8_t temp = (&I2cHandle)->Instance->DR;

    return true;
}

bool I2cIsReady(void) {
    return I2cHandle.State != HAL_I2C_STATE_RESET;
}

bool I2cIsDeviceReady(uint8_t deviceID) {
    HAL_StatusTypeDef temp = HAL_I2C_IsDeviceReady(&I2cHandle,deviceID,5,I2C_OPERATION_TIMEOUT);
    return temp == HAL_OK;
}

bool I2cRead(uint8_t deviceID, uint8_t *buffer, uint8_t len) {
    // HAL_StatusTypeDef status =  HAL_I2C_Master_Receive(&I2cHandle,deviceID,buffer,len,I2C_OPERATION_TIMEOUT);
    // return status == HAL_OK;
    return HAL_I2C_Master_Receive(&I2cHandle,deviceID,buffer,len,I2C_OPERATION_TIMEOUT) == HAL_OK;
}

bool I2cReadByte(uint8_t deviceID, uint8_t *value) {
    return HAL_I2C_Master_Receive(&I2cHandle,deviceID,value,sizeof(uint8_t),I2C_OPERATION_TIMEOUT) == HAL_OK;
}

bool I2cReadTwoBytes(uint8_t deviceID, uint16_t *value) {
    return HAL_I2C_Master_Receive(&I2cHandle,deviceID,(uint8_t *)value,sizeof(uint16_t),I2C_OPERATION_TIMEOUT) == HAL_OK;
}

bool I2cWriteByte(uint8_t deviceID, uint8_t data) {
    return HAL_I2C_Master_Transmit(&I2cHandle,deviceID,&data,sizeof(uint8_t),I2C_OPERATION_TIMEOUT) == HAL_OK;
}

bool I2cWriteTwoBytes(uint8_t deviceID, uint16_t data) {
    return HAL_I2C_Master_Transmit(&I2cHandle,deviceID,(uint8_t *)(&data),sizeof(uint16_t),I2C_OPERATION_TIMEOUT) == HAL_OK;
}

bool I2cWrite(uint8_t deviceID, uint8_t *buffer, uint8_t len) {
    return HAL_I2C_Master_Transmit(&I2cHandle,deviceID,buffer,len,I2C_OPERATION_TIMEOUT) == HAL_OK;
}

bool I2cWriteThenReadByte(uint8_t deviceID, uint16_t deviceRegAddr,uint8_t *value) {
    return I2cWriteThenRead(deviceID,deviceRegAddr,value,sizeof(uint8_t));
    // bool status = I2cWriteThenRead(deviceID,deviceRegAddr,value,sizeof(uint8_t));
    // return status;
}

bool I2cWriteThenReadTwoBytes(uint8_t deviceID, uint16_t deviceRegAddr, uint16_t *value) {
    return I2cWriteThenRead(deviceID,deviceRegAddr,(uint8_t *)value,sizeof(uint16_t));
}

bool I2cWriteThenRead(uint8_t deviceID, uint16_t deviceRegAddr, uint8_t *buffer, uint8_t len) {
    if(buffer != NULL && len > 0)
    {
        // Request a device register read
        bool success = I2cWriteByte(deviceID,deviceRegAddr);

        // Only continue if we didn't have an error
        if(success) {
            // Request data from the requested register
            return I2cRead(deviceID,buffer,len);
        } else {
            return success;
        }
    } else
        return false;
}

uint32_t I2cGetError() {
    return HAL_I2C_GetError(&I2cHandle);
}

void I2cClockOutPreviousData(uint32_t numberOfClockCycles) {
    // If we reset the device while it was sending data it will still
    // have data in the output register. Need to clock out that extra data
    // or the device will not respond
    
    // Deactive I2C
    HAL_I2C_DeInit(&I2cHandle);

    // Change i2c pins to regular gpio pins
    GPIO_InitTypeDef  GPIO_InitStruct;
    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* I2C TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = I2Cx_SCL_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);
        
    /* I2C RX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = I2Cx_SDA_PIN;
    HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);

    // Set the pins to their default state
    // Since these are open drain if a device on the bus is pulling them low
    // our pins will also be low
    HAL_GPIO_WritePin(I2Cx_SCL_GPIO_PORT,I2Cx_SCL_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2Cx_SDA_GPIO_PORT,I2Cx_SDA_PIN,GPIO_PIN_SET);


    // Toggle clock pin until data line goes high or we hit our cycle count
    bool dataHigh = false;
    uint32_t cycles = 0;
    while(!dataHigh && cycles < numberOfClockCycles) {
        HAL_GPIO_TogglePin(I2Cx_SCL_GPIO_PORT,I2Cx_SCL_PIN);
        HAL_Delay(1);
        HAL_GPIO_TogglePin(I2Cx_SCL_GPIO_PORT,I2Cx_SCL_PIN);

        // Check if data has gone high (idle state)
        if(HAL_GPIO_ReadPin(I2Cx_SDA_GPIO_PORT,I2Cx_SDA_PIN) == GPIO_PIN_SET) {
            dataHigh = true;
        }
        cycles++;
    }

    HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
    HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);

    // Re initialize the I2C module
    HAL_I2C_Init(&I2cHandle);
}