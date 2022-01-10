
#include <lpc177x_8x.h>

#ifndef __VT100_H
#define __VT100_H

#define VT100_ATTR_OFF      0               //выключение курсора
#define VT100_BOLD          1               //"жирный" шрифт вкл
#define VT100_USCORE        4               //
#define VT100_BLINK         5               //мигание курсора вкл
#define VT100_REVERSE       7               //инверсное отображение вкл
#define VT100_BOLD_OFF      21              //"жирный" шрифт выкл
#define VT100_USCORE_OFF    24              //
#define VT100_BLINK_OFF     25              //мигание курсора выкл
#define VT100_REVERSE_OFF   27              //инверсное отображение выкл

//*************************************************************************************************
//Функции управления
//*************************************************************************************************
void vt100Init( void );
void vt100ClearScreen( void );
void vt100SetAttr( uint8_t attr );
void vt100SetCursorMode( uint8_t visible );
void vt100SetCursorPos( uint8_t line, uint8_t col );

#endif

