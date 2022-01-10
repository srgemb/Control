
#ifndef __INFORMING_H
#define __INFORMING_H

#include <stdint.h>
#include <stdbool.h>

#include "device.h"
#include "dev_param.h"

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void VoiceInit( void );
ModBusError SetVolume( Volume volume );
ModBusError Informing( VoiceId id_info, char *name_info );

#endif
