#pragma once
/*
 * HEADER NAME : DataCollection.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/21/2020
 * DESCRIPTION :
 */

#include "config.h"
#include "stdbool.h"

// Raw sensor data structure
struct ShockSensorRawData {
    float gx;
    float gy;
    float gz;
    uint32_t encoderTicks;
};

// Sensor Structure
struct ShockSensorData {
    float accelX;
    float accelY;
    float accelZ;
    float linearPos;
};

struct ShockSensorData sensorDataBuffer[DATA_BUFFER_LEN];

bool collectData(void);

struct ShockSensorData convertRawShockData(struct ShockSensorRawData rawDataStruct);

float convertGsToAccel(float g);

struct ShockSensorData filterShockSensorData(struct ShockSensorData dataStruct);

float filterData(float data);

