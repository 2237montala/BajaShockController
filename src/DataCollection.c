#include "DataCollection.h"

float32_t tempData;

// Create a local structure to hold the newest raw data sample
struct ShockSensorRawData newDataSample;

ShockSensorDataStruct newConvertedDataSample;

ShockSensorDataStruct lastestFilteredData;

// Create a fifo for holding the data
_fff_declare(ShockSensorDataStruct,ShockSenorDataFifo,DATA_BUFFER_LEN);

void initializeCollectData() {

    // Zero out all the data structures
    memset(sensorDataBuffer,0x0,sizeof(sensorDataBuffer));
    memset(&newDataSample,0x0,sizeof(newDataSample));
    memset(&newConvertedDataSample,0x0,sizeof(newConvertedDataSample));
    memset(&lastestFilteredData,0x0,sizeof(lastestFilteredData));

    // Initialize the fifo
    _fff_init(ShockSenorDataFifo);
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

    // If fifo is full then pop off an element
    if(_fff_is_full(ShockSenorDataFifo)) {
        _fff_remove(ShockSenorDataFifo,1);
    }

    // Add new sample to the fifo
    _fff_write_lite(ShockSenorDataFifo,newConvertedDataSample);
    
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

void filterData() {
    // Simple running average over all the fifo
    for(int i = 0; i < DATA_BUFFER_LEN; i++) {
        // Sum accels
        for(int q = 0; q < NUMBER_OF_AXIS; q++) {
            lastestFilteredData.accels[q] += _fff_peek(ShockSenorDataFifo,i).accels[q]; 
        }

        // Sum linear position
        lastestFilteredData.linearPos += _fff_peek(ShockSenorDataFifo,i).linearPos;
    }
}

ShockSensorDataStruct* getMostRecentSensorData() {
    return &(_fff_peek(ShockSenorDataFifo,0));
}

ShockSensorDataStruct* getMostRecentFilteredSensorData() {
    return &lastestFilteredData;
}
