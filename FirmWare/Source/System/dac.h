
#ifndef __DAC_H
#define __DAC_H

#include <stdio.h>
#include <stdint.h>
#include <lpc_types.h>

#include "device.h"

typedef struct {
    uint8_t *ptr;
    uint32_t size;
 } SoundData;

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void DACInit( void );

#endif
