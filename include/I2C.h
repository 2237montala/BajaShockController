#pragma once

// Get the hal library for the specific micro
#include "Bsp.h"
#include <stdbool.h>
/*
 * HEADER NAME : I2C.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 03/09/2021
 * DESCRIPTION : This header defines I2C functions that can be used by any I2C device. This is 
 *               not a complete driver and only function that I need have been added.
 */

enum RequestType {ADDRBIT8_HIGH_TOREAD, 
                  ADDRBIT8_HIGH_TOWRITE};


bool I2cInit();

bool I2cBegin();


bool read(uint8_t *buffer, uint8_t len);
bool read(uint8_t *value);
bool read(uint16_t *value);
uint32_t read(void);
uint32_t readCached(void);
bool write(uint8_t *buffer, uint8_t len);
bool write(uint32_t value, uint8_t numbytes = 0);

uint8_t width(void);