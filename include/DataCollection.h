#pragma once
/*
 * HEADER NAME : DataCollection.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/21/2020
 * DESCRIPTION :
 */

#include "config.h"
#include "stdbool.h"
#include "fifofast.h"
#include "LIS3DH.h"

#define NUMBER_OF_AXIS 3
#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2

// Raw sensor data structure
struct ShockSensorRawData {
    uint32_t gs[NUMBER_OF_AXIS];
    uint32_t encoderTicks;
};

// Sensor Structure
// This needs to be typedef'd for use in the fifo
typedef struct ShockSensorData{
    float32_t accels[NUMBER_OF_AXIS];
    float32_t linearPos;
    uint8_t inFreefall;
} ShockSensorDataStruct;

//ShockSensorDataStruct sensorDataBuffer[DATA_BUFFER_LEN];

bool collectRawData(void);

bool collectData(void);

bool convertShockGsToAccel(float32_t *accelArray, uint32_t numberOfAxis);

bool convertShockTicksToPosition(float32_t *linearPos, uint32_t rawTicks);

struct ShockSensorData convertRawShockData(struct ShockSensorRawData rawDataStruct);

float convertGsToAccel(uint32_t g);

void filterData(void);

ShockSensorDataStruct* getMostRecentSensorData();

ShockSensorDataStruct* getMostRecentFilteredSensorData();

