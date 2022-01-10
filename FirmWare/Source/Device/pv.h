
#ifndef __PV_H
#define __PV_H

#include <stdint.h>
#include <stdbool.h>

#include "eeprom.h"
#include "dev_param.h"

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void PvInit( void );
PvError PvControl( PvCtrl ctrl, EepromMode restore );
void PvSetMode( PvMode mode, EepromMode restore );

//*************************************************************************************************
// Функции статуса/состояния
//*************************************************************************************************
PvStatus PvGetStat( void );
PvMode PvGetMode( void );

#endif 
