
//*************************************************************************************************
//
// Функционал управления часами реального времени
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <time.h>

#include "cmsis_os2.h"

#include "dev_data.h"
#include "ports.h"
#include "outinfo.h"
#include "modbus.h"
#include "batmon.h"
#include "charger.h"
#include "system.h"
#include "rtc.h"
#include "spa_calc.h"
#include "scheduler.h"
#include "pv.h"
#include "tracker.h"
#include "informing.h"
#include "priority.h"
#include "hmi_can.h"
#include "events.h"

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
RTC rtc;
static osEventFlagsId_t rtc_event = NULL;
static char date_time[40], str_date[40], str_time[20], year_mon[20], date_log[20], log_str[30];
static char * const dows_short[] = { "Вск", "Пон", "Втр", "Срд", "Чтв", "Птн", "Суб", NULL };

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t rtc_attr = {
    .name = "Rtc",
    .stack_size = 256,
    .priority = osPriorityNormal
 };
         
static const osEventFlagsAttr_t evn_attr = { .name = "Rtc" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static uint16_t DayOfWeek( int day, int month, int year );
static uint16_t DayOfYear( int day, int month, int year );
static void TaskRtc( void *pvParameters );

//*************************************************************************************************
// Инициализация часов
//*************************************************************************************************
void RTCInit( void ) {

    //создаем флаг события
    rtc_event = osEventFlagsNew( &evn_attr );
    //создаем задачу
    osThreadNew( TaskRtc, NULL, &rtc_attr );
    //инициализация часов
    RTC_Init( LPC_RTC );
    //включение обработки прерывания
    NVIC_SetPriority( RTC_IRQn, NVIC_EncodePriority( NVIC_GetPriorityGrouping(), PRIORITY_RTC, 0 ) );
    NVIC_EnableIRQ( RTC_IRQn );
    //вкл часов
    RTC_Cmd( LPC_RTC, ENABLE );
    RTC_CntIncrIntConfig( LPC_RTC, RTC_TIMETYPE_SECOND, ENABLE );
 }

//*************************************************************************************************
// Обработка секундного прерывания часов
//*************************************************************************************************
void RTC_IRQHandler( void ) {

    if ( RTC_GetIntPending( LPC_RTC, RTC_INT_COUNTER_INCREASE ) == SET )
        RTC_ClearIntPending( LPC_RTC, RTC_INT_COUNTER_INCREASE );
    //передаем событие в задачу TaskRtc()
    osEventFlagsSet( rtc_event, EVN_RTC_IRQ );
 }

//*************************************************************************************************
// Задача обработки сообщения от прерывания RTC
//*************************************************************************************************
static void TaskRtc( void *pvParameters ) {

    uint32_t msg;
    RTC_TIME_Type Time;

    for ( ;; ) {
        //ждем события от часов
        osEventFlagsWait( rtc_event, EVN_RTC_IRQ, osFlagsWaitAll, osWaitForever );
        //текущее время, сохраним в структуре RTC
        RTC_GetFullTime( LPC_RTC, &Time );
        rtc.sec = Time.SEC;
        rtc.min = Time.MIN;
        rtc.hour = Time.HOUR;
        rtc.dow = Time.DOW;
        rtc.day = Time.DOM;
        rtc.month = Time.MONTH;
        rtc.year = Time.YEAR;
        
        //SEGGER_RTT_WriteString( 0, "SEGGER Real-Time-Terminal Sample\r\n" );
        
        //передача событий внешним задачам с интервалом 1 секунд
        if ( alt_event != NULL )
            osEventFlagsSet( alt_event, EVN_RTC_SECONDS );          //отсчет программных таймеров
        if ( gen_event != NULL )
            osEventFlagsSet( gen_event, EVN_RTC_SECONDS );          //отсчет программных таймеров
        if ( batmon_event != NULL )
            osEventFlagsSet( batmon_event, EVN_RTC_SECONDS );       //передача данных монитора АКБ
        if ( mppt_event != NULL )
            osEventFlagsSet( mppt_event, EVN_RTC_SECONDS );         //передача данных контроллера MPPT
        if ( job_event != NULL )
            osEventFlagsSet( job_event, EVN_RTC_SECONDS );          //обработка заданий планировщика
        if ( out_event != NULL )
            osEventFlagsSet( out_event, EVN_RTC_SECONDS );          //вывод данных устройств по шаблону на консоль
        if ( charge_event != NULL )
            osEventFlagsSet( charge_event, EVN_RTC_SECONDS );       //передача данных, контроль тока заряда
        if ( info_event != NULL )
            osEventFlagsSet( info_event, EVN_RTC_SECONDS );         //запрос состояния голосового информатора 
        if ( inv1_event != NULL )                                   //проверка ручного режима вкл/выкл инвертора TS-1000-224
            osEventFlagsSet( inv1_event, EVN_RTC_SECONDS );         //передача данных в HMI
        if ( inv2_event != NULL )                                   //проверка ручного режима вкл/выкл инвертора TS-3000-224
            osEventFlagsSet( inv2_event, EVN_RTC_SECONDS );         //передача данных в HMI
        msg = ID_DEV_RTC;
        if ( hmi_msg != NULL )                                      //передача данных часов в HMI
            osMessageQueuePut( hmi_msg, &msg, 0, 0 );
        msg = ID_DEV_VOICE;
        if ( hmi_msg != NULL )                                      //передача состояния голосового информатора
            osMessageQueuePut( hmi_msg, &msg, 0, 0 );
        if ( !Time.SEC ) {
            //передача событий задачам управления с интервалом 1 минута
            if ( trc_event != NULL )
                osEventFlagsSet( trc_event, EVN_RTC_1MINUTES );     //управление контроллером трекера
            if ( soc_event != NULL )
                osEventFlagsSet( soc_event, EVN_RTC_1MINUTES );     //Контролирует минимальный уровень заряда АКБ (SOC)
           }
        if ( !Time.SEC && !( Time.MIN % 5 ) ) {
            //передача событий задачам управления с интервалом 5 минут
            if ( trc_event != NULL )
                osEventFlagsSet( trc_event, EVN_RTC_5MINUTES );     //управление контроллером трекера
            if ( spa_event != NULL )
                osEventFlagsSet( spa_event, EVN_RTC_5MINUTES );     //расчет положения солнца
            if ( pv_event != NULL )
                osEventFlagsSet( pv_event,  EVN_RTC_5MINUTES );     //подк/откл солнечных панелей
            if ( info_event != NULL )
                osEventFlagsSet( info_event, EVN_RTC_5MINUTES );    //установка громкости голосового информатора 
           }
       }
 }

//*************************************************************************************************
// Возвращает время в формате HH:MI:SS
// char *endstr - строка добавляемая к результату
// return       - строка времени в формате HH:MI:SS + добавляемая строка
//*************************************************************************************************
char *RTCGetTime( char *endstr ) {

    static RTC_TIME_Type Time;
    
    RTC_GetFullTime( LPC_RTC, &Time );
    if ( endstr != NULL && strlen( endstr ) )
        sprintf( str_time, "%02d:%02d:%02d%s", Time.HOUR, Time.MIN, Time.SEC, endstr );  
    else sprintf( str_time, "%02d:%02d:%02d", Time.HOUR, Time.MIN, Time.SEC );  
    return str_time;
 }

//*************************************************************************************************
// Возвращает дату в формате DD.MM.YYYY
// char *endstr - строка добавляемая к результату
// return       - строка даты в формате DD.MM.YYYY + добавляемая строка
//*************************************************************************************************
char *RTCGetDate( char *endstr ) {

    static RTC_TIME_Type Time;
    
    RTC_GetFullTime( LPC_RTC, &Time );
    if ( endstr != NULL && strlen( endstr ) )
        sprintf( str_date, "%02d.%02d.%04d%s", Time.DOM, Time.MONTH, Time.YEAR, endstr );  
    else sprintf( str_date, "%02d.%02d.%04d", Time.DOM, Time.MONTH, Time.YEAR );  
    return str_date;
 }

//*************************************************************************************************
// Возвращает дату в формате YYYYMMDD для формирования имени файла протокола
// return - дата в формате YYYYMMDD
//*************************************************************************************************
char *RTCFileName( void ) {

    static RTC_TIME_Type Time;
    
    RTC_GetFullTime( LPC_RTC, &Time );
    sprintf( date_log, "%04d%02d%02d", Time.YEAR, Time.MONTH, Time.DOM );  
    return date_log;
 }

//*************************************************************************************************
// Возвращает дату в формате YYYYMM для формирования имени файла протокола
// return - дата в формате YYYYMM
//*************************************************************************************************
char *RTCFileShort( void ) {

    static RTC_TIME_Type Time;
    
    RTC_GetFullTime( LPC_RTC, &Time );
    sprintf( year_mon, "%04d%02d", Time.YEAR, Time.MONTH );  
    return year_mon;
 }

//*************************************************************************************************
// Возвращает дату/время в полном формате DD.MM.YYYY Week HH:MI:SS
// char *endstr - строка добавляемая к результату
// return       - дата/время в полном формате DD.MM.YYYY Week HH:MI:SS + добавляемая строка
//*************************************************************************************************
char *RTCGetDateTime( char *endstr ) {

    static RTC_TIME_Type Time;
    
    RTC_GetFullTime( LPC_RTC, &Time );
    if ( endstr != NULL && strlen( endstr ) )
        sprintf( date_time, "%02d.%02d.%04d %s %02d:%02d:%02d %s", Time.DOM, Time.MONTH, Time.YEAR, dows_short[Time.DOW], Time.HOUR, Time.MIN, Time.SEC, endstr );
    else sprintf( date_time, "%02d.%02d.%04d %s %02d:%02d:%02d", Time.DOM, Time.MONTH, Time.YEAR, dows_short[Time.DOW], Time.HOUR, Time.MIN, Time.SEC );
    return date_time;
 }

//*************************************************************************************************
// Возвращает дату/время в формате DD.MM.YYYY HH.MI.SS
// return - дата в формате DD.MM.YYYY HH.MI.SS
//*************************************************************************************************
char *RTCGetLog( void ) {

    static RTC_TIME_Type Time;
    
    RTC_GetFullTime( LPC_RTC, &Time );
    sprintf( log_str, "%02d.%02d.%04d %02d:%02d:%02d", Time.DOM, Time.MONTH, Time.YEAR, Time.HOUR, Time.MIN, Time.SEC );
    return log_str;
 }

//*************************************************************************************************
// Установка времени, перед установкой выполняется проверка устанавливаемого значения
// char *param - строка с устанавливаемыми значениями
// return      - ERROR/SUCCESS - ошибка в параметрах/время установлено 
//*************************************************************************************************
Status RTCSetTime( char *param ) {

    uint8_t hour, min, sec, idx, chk = 0;
    char *mask = NULL, mask1[] = "NN:NN", mask2[] = "NN:NN:NN";

    //тип формата
    if ( strlen( param ) == 5 )
        mask = mask1;
    if ( strlen( param ) == 8 )
        mask = mask2;
    if ( mask == NULL )
        return ERROR;
    //проверка формата
    for ( idx = 0; idx < strlen( mask ); idx++ ) {
        if ( mask[idx] == 'N' && isdigit( *(param+idx) ) )
            chk++;
        if ( mask[idx] == ':' && ispunct( *(param+idx) ) )
            chk++;
       } 
    if ( chk != strlen( mask ) )
        return ERROR;
    //проверка значений
    hour = atoi( param );
    min = atoi( param + 3 );
    if ( strlen( param ) == 8 )
        sec = atoi( param + 6 );
    else sec = 0;
    if ( hour > 23 || min > 59 || sec > 59 )
        return ERROR;
    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_HOUR, hour );
    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_MINUTE, min );
    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_SECOND, sec );
    return SUCCESS;
 }

//*************************************************************************************************
// Установка даты
// Входной формат DD.MM.YYYY 1-31.1-12.2000-2099
//*************************************************************************************************
Status RTCSetDate( char *param ) {
 
    uint8_t day, month;
    uint16_t year;
        
    if ( !CheckDate( param, &day, &month, &year ) )
        return ERROR;
    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_DAYOFMONTH, day );
    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_MONTH, month );
    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_YEAR, year );
    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_DAYOFWEEK, DayOfWeek( day, month, year ) );
    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_DAYOFYEAR, DayOfYear( day, month, year ) );
    return SUCCESS;
 }

//*************************************************************************************************
// Установка дата/время из внешнего источника
//*************************************************************************************************
void RTCSet( RTC *datetime ) {

    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_HOUR, datetime->hour );
    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_MINUTE, datetime->min );
    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_SECOND, datetime->sec );
    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_DAYOFMONTH, datetime->day );
    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_MONTH, datetime->month );
    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_YEAR, datetime->year );
    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_DAYOFWEEK, DayOfWeek( datetime->day, datetime->month, datetime->year ) );
    RTC_SetTime( LPC_RTC, RTC_TIMETYPE_DAYOFYEAR, DayOfYear( datetime->day, datetime->month, datetime->year ) );
 }

//*************************************************************************************************
// Возвращает текущее время в формате десятичных долей часа: H.xxxxxx
// return - результат в десятичных долях часа (12.36591)
//*************************************************************************************************
float GetTimeDecimal( void ) {

    float result;
    static RTC_TIME_Type Time;
    
    RTC_GetFullTime( LPC_RTC, &Time );
    result = (float)Time.SEC / 60.0;
    result = ( result + (float)Time.MIN ) / 60.0;
    result += (float)Time.HOUR;
    return result;
 }

//*************************************************************************************************
// Перевод времени (часы, минуты, секунды) в десятичные доли часа
// http://deep125.narod.ru/astra_calc/p7.htm
// uint8_t hour    - значение часов
// uint8_t minute  - значение минут
// uint8_t seconds - значение секунд
// return          - результат в десятичных долях часа (12.36591)
//*************************************************************************************************
float TimeToDecimal( uint8_t hour, uint8_t minute, uint8_t seconds ) {

    float result;
    
    result = (float)seconds / 60.0;
    result = ( result + (float)minute ) / 60.0;
    result += (float)hour;
    return result;
 }

//*************************************************************************************************
// Перевод целых секунд в десятичные доли часа
// uint32_t value - значение в секундах
// return         - результат в десятичных долях часа (12.36591)
//*************************************************************************************************
float SecToDecimal( uint32_t value ) {

    uint8_t hour, min, sec;
    
    //переведем секунды сначала в часы/минуты/секунды
    hour = value / 3600; 
    sec = value % 3600;
    min = sec / 60;
    sec = sec % 60;
    return TimeToDecimal( hour, min, sec );
 }

//*************************************************************************************************
// Возвращает целое число секунд из исходного значения в секундах в момент, когда секунды = 0
// uint16_t value - значение секундах
// return = 0     - целое значение минут не наступило
//          N     - значение в минутах
//*************************************************************************************************
uint16_t SecToIntMinutes( uint16_t value ) {

    uint16_t min, sec;
    
    min = value / 60;
    sec = value % 60;
    if ( min && !sec )
        return min;
    return 0;
 }

//*************************************************************************************************
// Проверяет вхождения времени во временной интервал
// float time1         - начальное время (в десятичных долях часа)
// float time          - проверяемое время (в десятичных долях часа)
// float time2         - конечное время (в десятичных долях часа)
// return TimeInterval - время в интервале/не в интервале
//*************************************************************************************************
TimeInterval TimeCheck( float time_beg, float time, float time_end ) {

    if ( time < time_beg )
        return TIME_NOT_INTERVAL;
    if ( time > time_end )
        return TIME_NOT_INTERVAL;
    return TIME_IN_INTERVAL;
 }

//*************************************************************************************************
// Расчет дня недели по дате
// Все деления целочисленные (остаток отбрасывается).
// return: 0 — воскресенье, 1 — понедельник и т.д.
//*************************************************************************************************
static uint16_t DayOfWeek( int day, int month, int year ) {

    int a, y, m;
    
    a = (14 - month) / 12;
    y = year - a;
    m = month + 12 * a - 2;
    return (7000 + (day + y + y / 4 - y / 100 + y / 400 + (31 * m) / 12)) % 7;
}

//*************************************************************************************************
// Расчет номера дня (1-365) года по дате
// return - номер дня
//*************************************************************************************************
static uint16_t DayOfYear( int day, int month, int year ) {

    uint16_t i, leap;
    uint16_t day_tab[2][13] = { 0,31,28,31,30,31,30,31,31,30,31,30,31,
                                0,31,29,31,30,31,30,31,31,30,31,30,31 };
    
    leap = ( year%400 ) == 0 || ( ( year%4 ) == 0 && ( year%100 ) != 0 );
    for ( i = 1; i < month; i++ ) 
        day += day_tab[leap][i];
    return day;
 }

//*************************************************************************************************
// Проверяет формат даты в "value" по маске DD.MM.YYYY (01-31.01-12.2000-2099)
// return = ERROR   - формат соответствует маске, данные заносятся в переменные: day,month,year
//        = SUCCESS - формат не соответствует маске
//*************************************************************************************************
Status CheckDate( char *value, uint8_t *day, uint8_t *month, uint16_t *year ) {

    uint8_t idx, ch_day, ch_mon, chk = 0;
    uint16_t ch_year;
    char mask[] = "NN.NN.NNNN";

    //проверка формата
    for ( idx = 0; idx < strlen( mask ); idx++ ) {
        if ( mask[idx] == 'N' && isdigit( *(value+idx) ) )
            chk++;
        if ( mask[idx] == '.' && ispunct( *(value+idx) ) )
            chk++;
       } 
    if ( chk != strlen( mask ) )
        return ERROR; //не соответствие формату
    //проверка значений
    ch_day = atoi( value );
    ch_mon = atoi( value + 3 );
    ch_year = atoi( value + 6 );
    if ( ch_day < 1 || ch_day > 31 || ch_mon < 1 || ch_mon > 12 || ch_year < 2000 || ch_year > 2099 )
        return ERROR; //недопустимые значения
    //проверка прошла
    *day = ch_day;
    *month = ch_mon; 
    *year = ch_year;
    return SUCCESS;
 }
