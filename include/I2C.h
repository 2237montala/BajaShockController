#pragma once

// Get the hal library for the specific micro
#include "Bsp.h"
#include <stdbool.h>
#include "targetSpecific.h"
/*
 * HEADER NAME : I2C.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 03/09/2021
 * DESCRIPTION : This header defines I2C functions that can be used by any I2C device. This is 
 *               not a complete driver and only function that I need have been added.
 */

enum RequestType {ADDRBIT8_HIGH_TOREAD, 
                  ADDRBIT8_HIGH_TOWRITE};

#define I2C_OPERATION_TIMEOUT 100

#define I2C_HIGH_SPEED 400000
#define I2C_LOW_SPEED 100000

bool I2cInit(I2C_TypeDef *instance, uint32_t i2cSpeed, uint32_t dutyCycleMode,
             HAL_I2C_ModeTypeDef i2cType, uint8_t i2cAddress);

bool I2cIsReady(void);

bool I2cIsDeviceReady(uint8_t deviceID);

bool I2cRead(uint8_t deviceID, uint8_t *buffer, uint8_t len);
bool I2cReadByte(uint8_t deviceID, uint8_t *value);
bool I2cReadTwoBytes(uint8_t deviceID, uint16_t *value);

bool I2cWriteByte(uint8_t deviceID, uint8_t data);
bool I2cWriteTwoBytes(uint8_t deviceID, uint16_t data);
bool I2cWrite(uint8_t deviceID, uint8_t *buffer, uint8_t len);

bool I2cWriteThenReadTwoBytes(uint8_t deviceID, uint16_t deviceRegAddr, uint16_t *value);
bool I2cWriteThenReadByte(uint8_t deviceID, uint16_t deviceRegAddr,uint8_t *value);


bool I2cWriteThenRead(uint8_t deviceID, uint16_t deviceRegAddr, uint8_t *buffer, uint8_t len);

uint32_t I2cGetError();

void I2cClockOutPreviousData(uint32_t numberOfClockCycles);

uint8_t width(void);