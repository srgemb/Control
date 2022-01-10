
//*************************************************************************************************
//
// Отображение в консоли шаблона экрана с параметрами устройств
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "device.h"
#include "dev_param.h"

#include "command.h"
#include "outinfo.h"
#include "batmon.h"
#include "mppt.h"
#include "inverter.h"
#include "pv.h"
#include "alt.h"
#include "gen.h"
#include "command.h"
#include "uart.h"
#include "eeprom.h"
#include "vt100.h"
#include "charger.h"
#include "rtc.h"
#include "tracker.h"
#include "spa_calc.h"
#include "informing.h"
#include "message.h"
#include "events.h"

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osEventFlagsId_t out_event = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define MAX_CNT_PARAM_OUT   4               //максимальное кол-во параметров в макроподстановке
#define MAX_LEN_PARAM_OUT   15              //максимальное длинна значения одного параметра в макроподстановке

#define TEMPL_MAX_STR       60              //максимальное кол-во строк в шаблоне

#define MAX_MACRO           130             //максимальное кол-во макроподстановок в шаблоне

#define UNIT_OFFSET         10              //смещение курсора от начала строки для 
                                            //вывода единиц измерения параметра 

#define SCREEN_SIZE         7168            //размер буфера экрана

//индексы для массива параметров вывода par_out
typedef enum {
    PAR_IND_DEV,                            //ID устройства (Batmon, MPPT, TS ...)
    PAR_IND_PARAM,                          //ID параметра
    PAR_IND_STR,                            //строка вывода
    PAR_IND_POS                             //позиция вывода значения
 } MacroIndex;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static uint8_t str_cnt = 0;                                 //кол-во строк в шаблоне
static TemplateOut templ_mode = TEMPLATE_OFF;               //вкл/выкл режим вывода экрана с параметрами

static uint8_t ind_len[TEMPL_MAX_STR];                      //массив содержит абсолютную длинну строки 
                                                            //шаблона экрана, индекс - номер строки
static uint8_t par_out[MAX_MACRO][4];                       //параметры вывода данных состояния на консоль, 
                                                            //уст-во, параметр, строка, столбец
static char param[MAX_CNT_PARAM_OUT][MAX_LEN_PARAM_OUT];    //массив с разобранными значениями  
                                                            //параметров макроподстановок
static char screen[SCREEN_SIZE];                            //буфер экрана

static const osThreadAttr_t out_attr = {
    .name = "Outinfo", 
    .stack_size = 832,
    .priority = osPriorityNormal,
 };

static const osEventFlagsAttr_t evn_out = { .name = "Outinfo" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void OutData( void );
static void OutDataOn( void );
static void OutDataOff( void );
static void CrtParamOut( void );
static void ScreenOut( void );
static uint8_t ParseMacro( char *src, uint16_t len, char *par );
static void TaskOut( void *pvParameters );

//*************************************************************************************************
// Инициализация экрана
//*************************************************************************************************
void ScreenInit( void ) {

    vt100Init();
    vt100ClearScreen();
    vt100SetCursorMode( 1 ); //курсор включен
    vt100SetCursorPos( 1, 1 );
    //создаем флаг события
    out_event = osEventFlagsNew( &evn_out );
    //создаем задачу управления выводом параметров уст-в
    osThreadNew( TaskOut, NULL, &out_attr );
 }

//*************************************************************************************************
// Задача вывода шаблона на экран по секундному событию от RTC
//*************************************************************************************************
static void TaskOut( void *pvParameters ) {

    for ( ;; ) {
        //ждем события
        osEventFlagsWait( out_event, EVN_RTC_SECONDS, osFlagsWaitAll, osWaitForever );
        if ( templ_mode == TEMPLATE_OUT )
            OutData();
       }
 }

//*************************************************************************************************
// Загрузка шаблона экрана, заменяем макроподстановку значениями и сохраняет во временном буфере
//*************************************************************************************************
void ScreenLoad( void ) {

    FILE *scr;
    char str[256];
    uint16_t ind, ind_ch = 0, ind_str = 0;

    memset( screen, 0x00, sizeof( screen ) );
    //обнулим массив размера строк
    for ( ind = 0; ind < TEMPL_MAX_STR; ind++ )
        ind_len[ind] = 0;
    //загрузка шаблона экрана
    sprintf( str, Message( CONS_MSG_LOAD_SCREEN ), config.scr_file ); 
    ConsoleSend( str, CONS_NORMAL );
    //открытие файла
    scr = fopen( config.scr_file, "r" );
    if ( scr != NULL ) {
        //читаем файл
        ind = 0;
        while( !feof( scr ) ) {
            memset( str, 0x00, sizeof( str ) );
            if ( fgets( str, sizeof( str ), scr ) != NULL ) {
                ind_ch++;
                ind_ch += strlen( str );
                ind_len[ind_str++] = strlen( str ); //новая строка, запишем кол-во символов
                strcat( screen, str );
                //контроль на превышение размерности массивов
                if ( ind_str >= TEMPL_MAX_STR || ind_ch >= SCREEN_SIZE ) {
                    ConsoleSend( Message( CONS_ERR_BUFF_FULL ), CONS_NORMAL );
                    break;
                   } 
                str_cnt = ind_str; //сохраним кол-во строк в шаблоне
               }
           }
        fclose( scr );
        CrtParamOut(); //замена макроподстановок фактическими значениями
        ConsoleSend( Message( CONS_MSG_OK ), CONS_NORMAL );
        return;
       }
    if ( scr == NULL ) {
        ConsoleSend( Message( CONS_MSG_ERR_FOPEN ), CONS_NORMAL );
       } 
 }

//*************************************************************************************************
// Заполняет массив par_out данными из параметров макроподстановок для вывода в консоль
// индексы уст-в и параметров, позиции вывода в строке
//*************************************************************************************************
void CrtParamOut( void ) {

    char *buff, *macro_beg, *macro_end, *rem;
    uint16_t i, istr, imac, str_beg, cpar, cmac;
    
    buff = screen;
    //обнулим массив параметров
    for ( i = 0; i < MAX_MACRO; i++ )
        par_out[i][0] = par_out[i][1] = par_out[i][2] = par_out[i][3] = 0;
    //заполняем массив параметров вывода, проходим по строкам
    for ( istr = 0, str_beg = 0, cmac = 0; istr < TEMPL_MAX_STR; istr++ ) {
        if ( !ind_len[istr] )
            break; //строки в массиве закончились
        macro_beg = NULL;
        macro_end = NULL;
        //поиск макроподстановок в строке
        for ( imac = 0; imac < ind_len[istr]; imac++ ) {
            //ищем начало макроподстановки
            if ( *( buff + str_beg + imac ) == '{' )
                macro_beg = buff + str_beg + imac;
            //ищем окончание макроподстановки
            if ( *( buff + str_beg + imac ) == '}' )
                macro_end = buff + str_beg + imac;
            if ( macro_beg != NULL && macro_end != NULL && macro_beg < macro_end ) {
                //макро-подстановка найдена, разберем значения параметров
                cpar = ParseMacro( macro_beg + 1, macro_end - macro_beg - 1, *param );
                if ( cpar ) {
                    //есть параметры, разберем
                    par_out[cmac][PAR_IND_DEV] = DevGetInd( param[PAR_IND_DEV] );
                    par_out[cmac][PAR_IND_PARAM] = ParamGetInd( DevGetInd( param[PAR_IND_DEV] ), param[PAR_IND_PARAM] );
                    if ( atoi( param[PAR_IND_STR] ) )
                        par_out[cmac][PAR_IND_STR] = atoi( param[PAR_IND_STR] );    //номер строки из макроподстановки
                    else par_out[cmac][PAR_IND_STR] = istr + 1;                     //фактический номер строки
                    par_out[cmac][PAR_IND_POS] = atoi( param[PAR_IND_POS] );
                    //затрем макрос пробелами
                    memset( macro_beg, ' ', macro_end - macro_beg + 1 );
                    //на место макроподстановки запишем наименование параметра
                    rem = NULL;
                    if ( par_out[cmac][PAR_IND_PARAM] )
                        rem = ParamGetForm( (Device)par_out[cmac][PAR_IND_DEV], par_out[cmac][PAR_IND_PARAM] - 1, PARAM_DESC );
                    if ( rem != NULL )
                        memcpy( macro_beg, rem, strlen( rem ) );
                    cmac++; //индекс для сохранения макроподстановки
                   }
                //разбор завершен
                macro_beg = NULL;
                macro_end = NULL;
               }
           } 
        str_beg += ind_len[istr]; //индекс начала новой строки
       }
 }

//*************************************************************************************************
// Вывод шаблона экрана из временного буфера на консоль
//*************************************************************************************************
static void ScreenOut( void ) {

    vt100Init();
    vt100ClearScreen();
    vt100SetCursorPos( 1, 1 );
    UartSendStr( screen );
 }

//*************************************************************************************************
// Переключить режим отображения
//*************************************************************************************************
void ChangeModeOut( void ) {

    if ( !strlen( screen ) ) 
        return; //шаблон не загружен
    if ( templ_mode == TEMPLATE_OUT ) {
        templ_mode = TEMPLATE_OFF;
        OutDataOff(); //восстановим курсор и его положение
       }
    else OutDataOn(); //включить вывод шаблона
 }

//*************************************************************************************************
// Включить режим отображения данных по шаблону
//*************************************************************************************************
static void OutDataOn( void ) {

    ScreenOut(); //вывод шаблона без макроподстановок
    templ_mode = TEMPLATE_OUT;
 }

//*************************************************************************************************
// Выключить режим отображения данных по шаблону, переход в командный режим
//*************************************************************************************************
static void OutDataOff( void ) {

    templ_mode = TEMPLATE_OFF;
    vt100SetCursorPos( str_cnt + 1, 1 );
    vt100SetCursorMode( 1 ); //курсор включен
    ConsoleSend( Message( CONS_MSG_PROMPT ), CONS_NORMAL );
 }

//*************************************************************************************************
// Возвращает текущий режим консоли
// return TemplateOut - режим вывода шаблона вкл/выкл
//*************************************************************************************************
TemplateOut OutDataStat( void ) {

    return templ_mode;
 }

//*************************************************************************************************
// Вывод данных устройств, позиционирование вывода определены в массиве "par_out"
//*************************************************************************************************
static void OutData( void ) {

    char *unit;
    uint8_t idat;
    
    vt100SetCursorMode( 0 ); //курсор выключен
    for ( idat = 0; idat < MAX_MACRO; idat++ ) {
        if ( !par_out[idat][PAR_IND_DEV] && !par_out[idat][PAR_IND_PARAM] )
            break; //пропускаем, ID параметра = "0"
        //установка курсора для вывода значения параметра
        vt100SetCursorPos( par_out[idat][PAR_IND_STR], par_out[idat][PAR_IND_POS] );
        //вывод данных
        UartSendStr( ParamGetForm( (Device)par_out[idat][PAR_IND_DEV], par_out[idat][PAR_IND_PARAM]-1, PARAM_VALUE ) );
        //проверим наличие единиц измерения для параметра
        unit = ParamGetForm( (Device)par_out[idat][PAR_IND_DEV], par_out[idat][PAR_IND_PARAM]-1, PARAM_UNIT );
        if ( unit != NULL && strlen( unit ) ) {
            //установка курсора для вывода единиц измерения параметра со смещением
            vt100SetCursorPos( par_out[idat][PAR_IND_STR], par_out[idat][PAR_IND_POS] + UNIT_OFFSET );
            //вывод данных
            UartSendStr( unit );
           } 
       }
 }

//*************************************************************************************************
// Разбор параметров в макроподстановке {dev,par,n,n}
// char *src    - строка с параметрами
// uint16_t len - длинна строки для разбора
// char *par    - указатель на массив параметров param[][]
// return       - количество параметров, в т.ч. команда
//*************************************************************************************************
static uint8_t ParseMacro( char *src, uint16_t len, char *par ) {

    uint16_t row, i = 0;
    char *str, str_pars[MAX_CNT_PARAM_OUT*MAX_LEN_PARAM_OUT];
   
    //обнулим предыдущие параметры
    memset( str_pars, 0x00, MAX_CNT_PARAM_OUT*MAX_LEN_PARAM_OUT );
    for ( row = 0; row < MAX_CNT_PARAM_OUT; row++ )
        memset( par + row * MAX_LEN_PARAM_OUT, 0x00, MAX_LEN_PARAM_OUT );
    strncpy( str_pars, src, len );
    //разбор параметров
    str = strtok( str_pars, "," );
    while ( str != NULL && str[0] != ' ' ) {
        strcpy( par + ( i * MAX_LEN_PARAM_OUT ), str );
        i++; //параметр найден
        str = strtok( NULL, "," );
        if ( i > MAX_CNT_PARAM_OUT )
            return 0; //превышение кол-ва параметров
       }
    return i;
 }
