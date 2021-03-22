#include "DataCollection.h"

float32_t tempData;

// Create a local structure to hold the newest raw data sample
struct ShockSensorRawData newDataSample;

ShockSensorDataStruct newConvertedDataSample;

ShockSensorDataStruct lastestFilteredData;

// Create a fifo for holding the data
_fff_declare(ShockSensorDataStruct,ShockSenorDataFifo,DATA_BUFFER_LEN);

// Static function declarations
static bool collectDataFromSensors();

void initializeCollectData() {

    // Zero out all the data structures
    //memset(sensorDataBuffer,0x0,sizeof(sensorDataBuffer));
    memset(&newDataSample,0x0,sizeof(newDataSample));
    memset(&newConvertedDataSample,0x0,sizeof(newConvertedDataSample));
    memset(&lastestFilteredData,0x0,sizeof(lastestFilteredData));

    // Initialize the fifo
    _fff_init(ShockSenorDataFifo);
}

static bool collectDataFromSensors() {
    // Temporary struct for holding accelerometer data
    static struct Lis3dhDataStruct accelSensorData;
    static uint32_t currLinearPosTicks;
    bool succeed = false;

    // Read accelerometer
    succeed = Lis3dhRead(&accelSensorData);

    // Copy over the sensor data to the fifo struct
    newConvertedDataSample.accels[X_INDEX] = accelSensorData.xGs;
    newConvertedDataSample.accels[Y_INDEX] = accelSensorData.yGs;
    newConvertedDataSample.accels[Z_INDEX] = accelSensorData.zGs;

    // Get which index the user wants to check for free fall
    float32_t accelToCheck = 0;
    switch(AXIS_FOR_FREE_FALL) {
        case X_INDEX:
            accelToCheck = newConvertedDataSample.accels[X_INDEX];
            break;
        case Y_INDEX:
            accelToCheck = newConvertedDataSample.accels[Y_INDEX];
            break;
        case Z_INDEX:
            accelToCheck = newConvertedDataSample.accels[Z_INDEX];
            break;
        default:
            accelToCheck = INFINITY;
            break;  
    }

    if(accelToCheck != INFINITY && accelToCheck < FREE_FALL_G_THRESHOLD) {
        // We could be in free fall so we should check
        // TODO: Implement true free fall detection
        newConvertedDataSample.inFreefall = true;
    }

    // Read in encoder ticks
    currLinearPosTicks++; // Temp sensor reading 

    // Set encoder ticks to data struct
    succeed = convertShockTicksToPosition(&(newConvertedDataSample.linearPos),currLinearPosTicks);

    return succeed;
}

bool collectData() {
    bool succeed = false;

    // Collect data
    succeed = collectDataFromSensors();
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

bool convertShockTicksToPosition(float32_t *linearPos, uint32_t rawTicks) {
    *linearPos = (float32_t) rawTicks;
    return true;
}


void filterData() {
    // Clear the old data out
    memset(&lastestFilteredData,0x0,sizeof(lastestFilteredData));

    // Simple running average over all the fifo
    for(int i = 0; i < DATA_BUFFER_LEN; i++) {
        // Sum accels
        for(int q = 0; q < NUMBER_OF_AXIS; q++) {
            lastestFilteredData.accels[q] += _fff_peek(ShockSenorDataFifo,i).accels[q]; 
        }

        // Sum linear position
        lastestFilteredData.linearPos += _fff_peek(ShockSenorDataFifo,i).linearPos;
    }

    for(int i = 0; i < NUMBER_OF_AXIS; i++) {
        lastestFilteredData.accels[i] /= DATA_BUFFER_LEN;
    }

    lastestFilteredData.linearPos /= DATA_BUFFER_LEN;

    // TODO: do something with free fall here
    // Maybe count the number of continous free fall values
}

ShockSensorDataStruct* getMostRecentSensorData() {
    return &(_fff_peek(ShockSenorDataFifo,0));
}

ShockSensorDataStruct* getMostRecentFilteredSensorData() {
    return &lastestFilteredData;
}
