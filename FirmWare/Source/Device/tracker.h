
#ifndef __TRACKER_H
#define __TRACKER_H

#include <stdbool.h>

#include "dev_param.h"
#include "eeprom.h"

//*************************************************************************************************
//Функции управления
//*************************************************************************************************
void TrackerInit( void );
void TRCMode( PowerAct mode, EepromMode restore );
TrackerPosErr TrackerSetPos( TrackerAct pos, TrackerPos param, float value );
Status TrackerCmd( uint16_t cmnd, uint16_t param1, uint16_t param2 );

//*************************************************************************************************
//Функции статуса
//*************************************************************************************************
void TrcRestPos( void );

#endif
