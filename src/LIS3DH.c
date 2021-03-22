#include "LIS3DH.h"
#include "targetSpecific.h"

uint8_t i2cAddr;
uint8_t wai;
lis3dh_range_t setGRange;

uint32_t numBitsInRawAccel = 8 * sizeof(uint16_t);
uint32_t accelerationMaxValue = 0;
uint32_t accelerationZeroValue = 0;

uint32_t xRawOffset,yRawOffset,zRawOffset = 0;

static bool getRegister(uint8_t registerAddress, uint8_t *regValue);

static bool writeRegister(uint8_t registerAddress, uint8_t regValue);

static void convertToGs(float *valueInG, uint32_t valueInRaw, uint32_t lsb_factor, uint32_t scalingFactorToGs);

bool Lis3dhInit(uint8_t addr, uint8_t nWAI) {
    // Save local versoin of constructer params
    // Need to shift address over by 1 byte per the I2C standard
    // LSB is used to hold whether we are reading (0) or writing (1)
    i2cAddr = addr << 1;
    wai = nWAI;
    setGRange = LIS3DH_RANGE_2_G;

    // Check if I2C device is ready to use
    if(!I2cIsReady()) {
        return false;
    }

    // Send out addr to see if the sensor responds
    // Try 5 times before giving up
    bool status = I2cIsDeviceReady(i2cAddr);
    if(status == false) {
        int x = 0;
        x += 1;
        return false;
    }

    // Get device ID
    if(Lis3dhGetDeviceID() != wai) {
        // No LIS3DH connected
        return false;
    }

    // Enable reading on all axes in normal mode
    uint8_t regConfig = (LIS3DH_AXIS_X | LIS3DH_AXIS_Y | LIS3DH_AXIS_Z | LIS3DH_NORMAL_MODE);
    if(!writeRegister(LIS3DH_REG_CTRL1,regConfig)) {
        return false;
    }

    //uint8_t temp;
    // getRegister(LIS3DH_REG_CTRL1,&temp);

    // Set sampling rate
    if(!Lis3dhSetDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ)) {
        return false;
    }

    // getRegister(LIS3DH_REG_CTRL4,&temp);

    // Set normal resolution mode and block update mode
    // Normal mode is 10bit
    if(!writeRegister(LIS3DH_REG_CTRL4,0x80)) {
        return false;
    }

    // Set up the zero value which is half the range of the raw range
    accelerationZeroValue = (1 << (numBitsInRawAccel-1));

    accelerationMaxValue = (1 << numBitsInRawAccel);

    //getRegister(LIS3DH_REG_CTRL4,&temp);

    return true;
}

void Lis3dhSetOffsets(uint32_t newXRawOffset,uint32_t newYRawOffset,uint32_t newZRawOffset) {
    xRawOffset = newXRawOffset;
    yRawOffset = newYRawOffset;
    zRawOffset = newZRawOffset;
}

uint8_t Lis3dhGetDeviceID(void) {
    uint8_t deviceId = 0;
    I2cWriteThenReadByte(i2cAddr,LIS3DH_REG_WHOAMI,&deviceId);
    return deviceId;
}

bool Lis3dhRead(struct Lis3dhDataStruct *dataSample) {
    bool success = true;

    // Only continue if dataSample is valid
    if(dataSample != NULL) {
        // Process, request read of acceleration registers, read in data
        // Enable auto increment of address by setting MSb of address 1
        uint8_t buffer[6];
        //LIS3DH_REG_OUT_X_L
        success = I2cWriteThenRead(i2cAddr,LIS3DH_REG_OUT_X_L | 0x80, buffer, sizeof(buffer));
        if(success) {
            // Convert the individual bytes from sensor into 16 bit accelerations
            dataSample->xRaw = buffer[0] | ((uint16_t) buffer[1] << 8);
            dataSample->yRaw = buffer[2] | ((uint16_t) buffer[3] << 8);
            dataSample->zRaw = buffer[4] | ((uint16_t) buffer[5] << 8);

            // Convert raw data to g's
            uint8_t lsb_value = 1;
            switch(setGRange) {
            case LIS3DH_RANGE_2_G:
                lsb_value = 4;
                break;
            case LIS3DH_RANGE_4_G:
                lsb_value = 8;
                break;
            case LIS3DH_RANGE_8_G:
                lsb_value = 16;
                break;
            case LIS3DH_RANGE_16_G:
                lsb_value = 48;
                break;
            default:
                break;
            }

            convertToGs(&(dataSample->xGs),dataSample->xRaw,lsb_value,LIS3DH_LSB16_TO_KILO_LSB10);
            convertToGs(&(dataSample->yGs),dataSample->yRaw,lsb_value,LIS3DH_LSB16_TO_KILO_LSB10);
            convertToGs(&(dataSample->zGs),dataSample->zRaw,lsb_value,LIS3DH_LSB16_TO_KILO_LSB10);

        }
    }

    return success;
}

bool Lis3dhSetRange(lis3dh_range_t range) {
    bool success = false;
    // Get current status of REG CTRL4
    uint8_t regValue = 0;
    success = getRegister(LIS3DH_REG_CTRL4,&regValue);
    if(success) {
        // Add requested range to the current register value
        // Set range bits to 0
        regValue &= (LIS3DH_RANGE_MASK ^ 0xFF);

        regValue |= range << LIS3DH_RANGE_POS;

        // Write new value
        success = writeRegister(LIS3DH_REG_CTRL4,regValue);
    }

    // Update the global variable for the range
    setGRange = (success ? range : setGRange);

    return success;
}
lis3dh_range_t Lis3dhGetRange(void) {
    return setGRange;
}

bool Lis3dhSetDataRate(lis3dh_dataRate_t dataRate) {
    bool success = false;
    // Get current data rate register because it also contrains the current axes config
    uint8_t regValue = 0;
    if(!getRegister(LIS3DH_REG_CTRL1,&regValue)) {
        success = false;
    } else {
        // Do bitwise math on the requested data rate and the current axes config
        regValue |= (dataRate << LIS3DH_DATA_RATE_POS);

        // Write the new register back
        success = writeRegister(LIS3DH_REG_CTRL1,regValue);
        //success = I2cWriteTwoBytes(i2cAddr,cmd);
    }
    return success;
}

uint8_t Lis3dhGetAddress() {
    return i2cAddr >> 1;
}

//------------------------------------------------------------------------------
// Static methods
static bool getRegister(uint8_t registerAddress, uint8_t *regValue) {
    // Get the current register value from the sensor
    //return I2cWriteThenReadByte(i2cAddr,registerAddress, regValue);
    volatile bool status = I2cWriteThenReadByte(i2cAddr,registerAddress, regValue);
    return status;
}

static bool writeRegister(uint8_t registerAddress, uint8_t regValue) {
    //uint16_t cmd = (registerAddress << 8) | regValue;
    uint8_t cmd[2] = {registerAddress,regValue};
    return I2cWrite(i2cAddr,cmd,sizeof(cmd));
    //return I2cWriteTwoBytes(i2cAddr,cmd) == HAL_OK;
}

static void convertToGs(float *valueInG, uint32_t valueInRaw, uint32_t lsb_factor, uint32_t scalingFactorToGs) {
    if(valueInRaw > accelerationZeroValue) {
        valueInRaw = accelerationMaxValue - valueInRaw;
        valueInRaw *= lsb_factor;
        *valueInG = (float)valueInRaw/scalingFactorToGs;
    } else {
        valueInRaw *= lsb_factor;
        *valueInG = (float)valueInRaw/scalingFactorToGs;
        *valueInG *= -1;
    }
}