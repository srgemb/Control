
#ifndef __UART_H
#define __UART_H

#include <stdint.h>
#include <stdbool.h>

#define KEY_CR                  0x0D        //код '\r'
#define KEY_LF                  0x0A        //код '\n'
#define KEY_ESC                 0x1B        //код ESC
#define KEY_BS                  0x08        //код Backspace

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void UartInit( void );
void UartSendStr( char *buff );
void UartRecvClear( void );

//*************************************************************************************************
// Функции статуса/состояния
//*************************************************************************************************
char *UartBuffer( void );
void UartSendStart( void );

#endif