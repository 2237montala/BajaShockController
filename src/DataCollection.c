#include "DataCollection.h"


/*
 * PURPOSE
 *      This function is used to be a substitute for using the real collect data function. This
 *      function should be used with a program to send sensor data over UART. The UART being use
 *      should be set up ahead of time and be a global UART
 * PARAMETERS
 *      None
 * RETURNS
 *      bool - whether data was collected without error
 */
bool collectData() {
    return false;

}


struct ShockSensorData convertRawShockData(struct ShockSensorRawData rawDataStruct);

float convertGsToAccel(float g);

struct ShockSensorData filterShockSensorData(struct ShockSensorData dataStruct);

float filterData(float data);

