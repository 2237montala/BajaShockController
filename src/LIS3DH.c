#include "LIS3DH.h"
#include "targetSpecific.h"

I2C_HandleTypeDef *I2cHandle;
uint8_t i2cAddr;
uint8_t wai;

bool begin(I2C_HandleTypeDef *newI2CHandle,  uint8_t addr, uint8_t nWAI) {
    // Store I2C handle to local version
    // We assume here that the I2C interface has been set up
    I2cHandle = newI2CHandle;

    // Save local versoin of constructer params
    i2cAddr = addr;
    wai = nWAI;

    // Check if I2C device is ready to use
    if(I2cHandle->State != HAL_I2C_STATE_READY) {
        return false;
    }

    // Send out addr to see if the sensor responds
    // Try 5 times before giving up
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(I2cHandle,addr,5,100);

    if(status != HAL_OK) {
        return false;
    }

    // Get device ID
    if(getDeviceID() != wai) {
        // No LIS3DH connected
        return false;
    }
}

uint8_t getDeviceID(void);
bool haveNewData(void);
bool enableDRDY(bool enable_drdy, uint8_t int_pin);

void read(void);
int16_t readADC(uint8_t a);

void setRange(lis3dh_range_t range);
lis3dh_range_t getRange(void);