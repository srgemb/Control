
#ifndef __RING_UART_H
#define __RING_UART_H

#include <stdint.h>
#include <stdbool.h>

//*************************************************************************************************
// Р¤СѓРЅРєС†РёРё СѓРїСЂР°РІР»РµРЅРёСЏ
//*************************************************************************************************
void RingClear( void );
void RingAddStr( char *str );
void RingAddStrLen( char *str, uint16_t len );

//*************************************************************************************************
// Р¤СѓРЅРєС†РёРё СЃС‚Р°С‚СѓСЃР°/СЃРѕСЃС‚РѕСЏРЅРёСЏ
//*************************************************************************************************
bool RingGetChar( char *ch );
void RingCheckFree( void );
bool RingGetAdd( uint16_t size );
uint16_t RingGetSize( void );

#endif
