
#ifndef __CHARGER_H
#define __CHARGER_H

#include <stdbool.h>

#include "device.h"
#include "dev_param.h"
#include "eeprom.h"

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void ChargeInit( void );
ChargeError Charger( ChargeMode charge, EepromMode restore );

//*************************************************************************************************
// Функции статуса/состояния
//*************************************************************************************************
ChargeMode ChargeGetMode( void );
void EventLogPB( char *text );

#endif
