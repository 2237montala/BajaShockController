#include "DataCollection.h"

float32_t tempData;

struct ShockSensorRawData newDataSample;

void initializeCollectData() {

    // Zero out all the data structures
    memset(sensorDataBuffer,0x0,sizeof(sensorDataBuffer));
    memset(&newDataSample,0x0,sizeof(newDataSample));

}

/*
 * PURPOSE
 *      This function samples the sensors connected to the device and saves their raw
 *      output to a structure for later usage.
 * PARAMETERS
 *      None
 * RETURNS
 *      bool - whether data was collected without error
 */
bool collectRawData(void) {
    
    newDataSample.gs[X_INDEX] += 1;
    newDataSample.gs[Y_INDEX] += 1;
    newDataSample.gs[Z_INDEX] += 1;
    newDataSample.encoderTicks += 1;

    return true;
}


bool collectData() {
    bool succeed = false;

    //Collect raw data
    succeed = collectRawData();
    if(!succeed) {
        return false;
    }

    // Convert g values to accel
    succeed = convertShockGsToAccel(sensorDataBuffer[0].accels,NUMBER_OF_AXIS);
    if(!succeed) {
        return false;
    }

    // Convert linear enocder ticks to new position
    succeed = convertShockTicksToPosition(&(sensorDataBuffer[0].linearPos));
    if(!succeed) {
        return false;
    }

    // Filter the data
    
    return true;
}

// Takes in the location to save the converted raw sensor data
bool convertShockGsToAccel(float32_t *accelArray, uint32_t numberOfAxis) {
    if(numberOfAxis < NUMBER_OF_AXIS) {
        for(int i = 0; i < numberOfAxis; i++) {
            accelArray[i] = convertGsToAccel(newDataSample.gs[i]);
        }
        return true;
    }
    return false;
}

bool convertShockTicksToPosition(float32_t *linearPos) {
    *linearPos = (float32_t) newDataSample.encoderTicks;
    return true;
}

float convertGsToAccel(uint32_t g) {
    return (float32_t) g;
}

float filterData(float data);

