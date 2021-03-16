#include "LIS3DH.h"
#include "targetSpecific.h"

uint8_t i2cAddr;
uint8_t wai;

bool Lis3dhBegin(uint8_t addr, uint8_t nWAI) {
    // Save local versoin of constructer params
    i2cAddr = addr;
    wai = nWAI;

    // Check if I2C device is ready to use
    if(!I2cIsReady()) {
        return false;
    }

    // Send out addr to see if the sensor responds
    // Try 5 times before giving up

    if(!I2cIsDeviceReady(addr)) {
        return false;
    }

    // Get device ID
    if(getDeviceID() != wai) {
        // No LIS3DH connected
        return false;
    }

    // Enable reading on all axes in normal mode
    uint8_t regConfig = (LIS3DH_AXIS_X | LIS3DH_AXIS_Y | LIS3DH_AXIS_Z | LIS3DH_NORMAL_MODE);
    uint8_t data[2] = {LIS3DH_REG_CTRL1,regConfig};
    if(!I2cWrite(addr,data,sizeof(data))) {
        return false;
    }

    // Set sampling rate
    if(!Lis3dhSetDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ)) {
        return false;
    }

    //
}

uint8_t getDeviceID(void) {
    uint8_t deviceId = 0;
    I2cWriteThenReadByte(i2cAddr,LIS3DH_REG_WHOAMI,&deviceId);
    return deviceId;
}
bool haveNewData(void);
bool enableDRDY(bool enable_drdy, uint8_t int_pin);

void read(void);
int16_t readADC(uint8_t a);

void setRange(lis3dh_range_t range);
lis3dh_range_t getRange(void);

bool Lis3dhSetDataRate(lis3dh_dataRate_t dataRate) {
    bool hasError = false;
    // Get current data rate register because it also contrains the current axes config
    uint8_t regValue = 0;
    if(!I2cWriteThenReadByte(i2cAddr,LIS3DH_REG_CTRL1, &regValue)) {
        hasError = true;
    } else {
        // Do bitwise math on the requested data rate and the current axes config
        regValue |= (dataRate << LIS3DH_DATA_RATE_POS);

        // Write the new register back
        uint16_t cmd = (LIS3DH_REG_CTRL1 << 8) | regValue;
        hasError = !I2cWriteTwoBytes(i2cAddr,cmd);
    }
    return hasError;
}