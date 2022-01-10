
#ifndef __SOUND_H
#define __SOUND_H

#include <stdio.h>
#include <stdint.h>
#include <lpc_types.h>

#include "device.h"

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
Status SoundPlay( SoundId sound, char *name );
SoundId VoiceToSound( VoiceId id_voice );

#endif
