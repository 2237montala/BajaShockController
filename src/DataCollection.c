#include "DataCollection.h"

float32_t tempData;

// Create a local structure to hold the newest raw data sample
struct ShockSensorRawData newDataSample;

ShockSensorDataStruct newConvertedDataSample;

ShockSensorDataStruct lastestFilteredData;

// Create a fifo for holding the data
_fff_declare(ShockSensorDataStruct,ShockSensorDataFifo,DATA_BUFFER_LEN);

// Static function declarations
static bool collectDataFromSensors();

void initializeCollectData() {

    // Zero out all the data structures
    //memset(sensorDataBuffer,0x0,sizeof(sensorDataBuffer));
    memset(&newDataSample,0x0,sizeof(newDataSample));
    memset(&newConvertedDataSample,0x0,sizeof(newConvertedDataSample));
    memset(&lastestFilteredData,0x0,sizeof(lastestFilteredData));

    // Initialize the fifo
    _fff_init(ShockSensorDataFifo);
}

static bool collectDataFromSensors() {
    // Temporary struct for holding accelerometer data
    static struct Lis3dhDataStruct accelSensorData;
    static uint32_t currLinearPosTicks;
    static uint32_t samplesInFreefall = 0;
    static uint32_t samplesNotInFreefall = 0;
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

    // Check if the sample was within the free fall threshold
    if(accelToCheck != INFINITY && accelToCheck > FREE_FALL_G_THRESHOLD_NEG && accelToCheck < FREE_FALL_G_THRESHOLD_POS) {
        // We could be in free fall so we should check
        // TODO: Implement true free fall detection
        samplesInFreefall++;
        if(samplesInFreefall > SAMPLES_BEFORE_VALID_FREE_FALL) {
            newConvertedDataSample.inFreefall = true;
        }

        
    } else if (samplesInFreefall > 0 && samplesNotInFreefall < SAMPLES_BEFORE_VALID_FREE_FALL) {
        // The sample wasn't in the free fall threshold but we might still be in free fall
        // This helps remove jitter in the free fall state
        samplesNotInFreefall++;
    } else {
        // Last SAMPLES_BEFORE_VALID_FREE_FALL samples were not in free fall so reset our state
        // to not being in free fall
        newConvertedDataSample.inFreefall = false;
        samplesNotInFreefall = 0;
        samplesInFreefall = 0;
    }
    

    // TODO: Add correct encoder tick calculating
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
    if(_fff_is_full(ShockSensorDataFifo)) {
        _fff_remove(ShockSensorDataFifo,1);
    }

    // Add new sample to the fifo
    _fff_write_lite(ShockSensorDataFifo,newConvertedDataSample);
    
    return true;
}

bool convertShockTicksToPosition(float32_t *linearPos, uint32_t rawTicks) {
    *linearPos = (float32_t) rawTicks;
    return true;
}


void filterData() {
    // Clear the old data out
    memset(&lastestFilteredData,0x0,sizeof(lastestFilteredData));
    
    // Holds the number of free falls events between a time interval
    // bool consecutiveFreefall = true;
    // static uint32_t totalValidFreefalls = 0;
    // uint32_t currValidFreefalls = 0;

    // TODO : Change the average value to be the number of samples in the fifo
    // Simple running average over all the fifo
    for(int i = 0; i < _fff_mem_level(ShockSensorDataFifo); i++) {
        // Sum accels
        for(int q = 0; q < NUMBER_OF_AXIS; q++) {
            lastestFilteredData.accels[q] += _fff_peek(ShockSensorDataFifo,i).accels[q]; 
        }

        // Sum linear position
        lastestFilteredData.linearPos += _fff_peek(ShockSensorDataFifo,i).linearPos;        
    }

    for(int i = 0; i < _fff_mem_level(ShockSensorDataFifo); i++) {
        lastestFilteredData.accels[i] /= DATA_BUFFER_LEN;
    }

    lastestFilteredData.linearPos /= DATA_BUFFER_LEN;
    
    // Set free fall state which was averaged out in the last samples
    // so no filtering has to be done here. If the system is changing free fall states
    // then this variable will be a few ms old. At most 5ms
    lastestFilteredData.inFreefall = _fff_peek(ShockSensorDataFifo,0).inFreefall;
}

ShockSensorDataStruct* getMostRecentSensorData() {
    return &(_fff_peek(ShockSensorDataFifo,0));
}

ShockSensorDataStruct* getMostRecentFilteredSensorData() {
    return &lastestFilteredData;
}
