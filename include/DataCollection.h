#pragma once
/*
 * HEADER NAME : DataCollection.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/21/2020
 * DESCRIPTION :
 */

#include "config.h"
#include "stdbool.h"

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
struct ShockSensorData {
    float32_t accels[NUMBER_OF_AXIS];
    float32_t linearPos;
};

struct ShockSensorData sensorDataBuffer[DATA_BUFFER_LEN];

bool collectRawData(void);

bool collectData(void);

bool convertShockGsToAccel(float32_t *accelArray, uint32_t numberOfAxis);

bool convertShockTicksToPosition(float32_t *linearPos);

struct ShockSensorData convertRawShockData(struct ShockSensorRawData rawDataStruct);

float convertGsToAccel(uint32_t g);

struct ShockSensorData filterShockSensorData(struct ShockSensorData dataStruct);

float filterData(float data);

