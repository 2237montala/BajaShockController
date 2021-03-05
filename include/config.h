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

// This digital pin is toggled on during the CanOpen interrupt
#define DEBUG_GPIO_PIN D4

// Baud rate for debug uart
// If equal to 0 then there will be no debug messages
#define DEBUG_UART_BAUD_RATE 115200