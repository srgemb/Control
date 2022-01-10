
#ifndef __RESERV_H
#define __RESERV_H

#include <stdint.h>
#include <stdbool.h>

#include "device.h"

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void ReservInit( void );
void ReservCmnd( RelayRes id, RelayCmnd cmnd );
void ExtOut( RelayOut id, RelayCmnd cmnd );

#endif
