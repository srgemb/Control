
#ifndef __RTC_H
#define __RTC_H

#include "lpc177x_8x_rtc.h"
#include "lpc_types.h"
#include "stdbool.h"

#include "device.h"
#include "dev_data.h"
#include "dev_param.h"

//результат проверки времени в заданном интервале
typedef enum {
    TIME_NOT_INTERVAL,                      //время не входит в заданный интервал      
    TIME_IN_INTERVAL                        //время входит в заданный интервал
 } TimeInterval;

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void RTCInit( void );
void RTCStart( void );
void RTCSet( RTC *datetime );
Status RTCSetTime( char *param );
Status RTCSetDate( char *param );

//*************************************************************************************************
// Функции статуса/состояния
//*************************************************************************************************
char *RTCGetTime( char *endstr );
char *RTCGetDate( char *endstr );
char *RTCGetDateTime( char *endstr );
TimeInterval TimeCheck( float time_beg, float time, float time_end );
float GetTimeDecimal( void );
float SecToDecimal( uint32_t value );
float TimeToDecimal( uint8_t hour, uint8_t minute, uint8_t seconds );
uint16_t SecToIntMinutes( uint16_t value );
char *RTCGetLog( void );
char *RTCFileName( void );
char *RTCFileShort( void );
Status CheckDate( char *value, uint8_t *day, uint8_t *month, uint16_t *year );

#endif
