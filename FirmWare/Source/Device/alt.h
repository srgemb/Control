
#ifndef __ALT_H
#define __ALT_H

#include "dev_param.h"

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void AltInit( void );
AltError AltPowerAC( void );
AltError AltPowerDC( void );

//*************************************************************************************************
// Функции статуса/состояния
//*************************************************************************************************
AltPower AltPowerStat( void );

#endif
