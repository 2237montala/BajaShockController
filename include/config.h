#pragma once

/*
 * HEADER NAME : config.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/21/2020
 * DESCRIPTION : This file contains constants or configuration values for the Shock Controller.
 *               These values will be global so watch out...
 */

#include "arm_math.h"

#define SOFTWARE_TESTING 0

#if (SOFTWARE_TESTING == 1) 
// How much time must pass between data samples
// This value is in milliseconds
#define DATA_COLLECTION_RATE 5

#define DATA_BUFFER_LEN 1

#else
// How much time must pass between data samples
// This value is in milliseconds
#define DATA_COLLECTION_RATE 1

// Sets how long the buffer will be for averaging samples
// This should also be larger than the data request rate
// This should be a multiple of 2
#define DATA_BUFFER_LEN 32
#endif

// Sets the lower bound of the free fall detection threshold
// The lower this value the easier we will be in free fall
// Free fall is detected when the sensor reads about 0g on the 
// axis perpendicular with the ground
#define FREE_FALL_G_THRESHOLD_NEG -0.1f
#define FREE_FALL_G_THRESHOLD_POS  0.1f

// Defines which axis we will check for free fall acceleration
// Options are: X_INDEX, Y_INDEX, Z_INDEX
#define AXIS_FOR_FREE_FALL Z_INDEX

// Defines how many samples must be in free fall to consider the car in free fall
// This value is the number of samples so the length of time this will take
// is DATA_COLLECTION_RATE * number of samples
#define TIME_BEFORE_VALID_FREE_FALL 0

#define TIME_BEFORE_FREE_FALL_RESET 0

// CANOpen Settings -------------------------------------------------------------------------------
// The base Node ID must be lower than 0x7F. The lower the number the higher it's priority on the 
// CAN bus.
#define NODE_ID_BASE 0x20
#define NODE_ID_BIT_ONE_PIN D5
#define NODE_ID_BIT_TWO_PIN D6


// Baud rate in bps
#define CAN_BAUD_RATE 500

// Arduino Digital Pin names for the status indicators
#define GREEN_LED_PIN D8
#define RED_LED_PIN D9


// Debug settings ---------------------------------------------------------------------------------
// Enables or disables toggling pin every CO processing loop
// Comment out to disable
#define DEBUG_GPIO_ON

// This digital pin is toggled on during the CanOpen interrupt
#define DEBUG_GPIO_PIN D4

// Enable or disables printing of debug messages
// Comment out to disable
#define DEBUG_UART_ON

// Baud rate for debug uart
// If equal to 0 then there will be no debug messages
#define DEBUG_UART_BAUD_RATE 115200

// Accelerometer Settings -------------------------------------------------------------------------
// Possible options for accelerometer g range
// These needs to be constants
#define ACCLEROMETER_2G_RANGE LIS3DH_RANGE_2_G
#define ACCLEROMETER_4G_RANGE LIS3DH_RANGE_4_G
#define ACCLEROMETER_8G_RANGE LIS3DH_RANGE_8_G
#define ACCLEROMETER_16G_RANGE LIS3DH_RANGE_16_G
#define ACCELEROMETER_DEFAULT_RANGE ACCLEROMETER_2G_RANGE

// Sets the max g the sensor will read
#define ACCELEROMETER_G_RANGE ACCLEROMETER_4G_RANGE