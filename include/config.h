#pragma once

/*
 * HEADER NAME : config.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/21/2020
 * DESCRIPTION : This file contains constants or configuration values for the Shock Controller.
 *               These values will be global so watch out...
 */

#include "arm_math.h"

#define SOFTWARE_TESTING 1

#if (SOFTWARE_TESTING == 1) 
// How much time must pass between data samples
// This value is in milliseconds
#define DATA_COLLECTION_RATE 5

#define DATA_BUFFER_LEN 1

#else
// How much time must pass between data samples
// This value is in milliseconds
#define DATA_COLLECTION_RATE 5

#define DATA_BUFFER_LEN 5
#endif


