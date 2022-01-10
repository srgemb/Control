
//*************************************************************************************************
//
// Управление коммутацией солнечных панелей
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "lpc177x_8x_gpio.h"
#include "gpio_lpc17xx.h"

#include "rl_fs.h"
#include "cmsis_os2.h"

#include "device.h"
#include "dev_data.h"

#include "pv.h"
#include "ports.h"
#include "eeprom.h"
#include "outinfo.h"
#include "rtc.h"
#include "mppt.h"
#include "sdcard.h"
#include "spa_calc.h"
#include "informing.h"
#include "message.h"
#include "command.h"
#include "events.h"

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osEventFlagsId_t pv_event = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define TIME_CHK_PV         300             //задержка на проверку вкл/выкл контактора панелей (msec)

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static osTimerId_t timer_pv1, timer_pv2, timer_pv3;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t pv_attr = {
    .name = "Pv",
    .stack_size = 896,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn_attr = { .name = "Pv" };
static const osTimerAttr_t timer1_attr = { .name = "PvOn" };
static const osTimerAttr_t timer2_attr = { .name = "PvOff" };
static const osTimerAttr_t timer3_attr = { .name = "PvCheck" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void EventLog( char *text );
static Status PvCtrlMain( void );
static void TaskPv( void *pvParameters );
static void Timer1Callback( void *arg );
static void Timer2Callback( void *arg );
static void Timer3Callback( void *arg );

//*************************************************************************************************
// Инициализация портов и восстановление последнего состояния реле
//*************************************************************************************************
void PvInit( void ) {

    GPIO_SetDir( PV_PORT1, PVK_ON, GPIO_DIR_OUTPUT );
    GPIO_SetDir( PV_PORT2, PVK_OFF, GPIO_DIR_OUTPUT );
    GPIO_SetDir( PV_PORT1, PV_MODE, GPIO_DIR_OUTPUT );
    //восстанавливаем предыдущее состояние
    PvSetMode( (PvMode)EepromLoad( EEP_PV_MODE ), EEPROM_RESTORE );
    pv_event = osEventFlagsNew( &evn_attr );
    //таймеры длительности импульса управления реле
    timer_pv1 = osTimerNew( Timer1Callback, osTimerOnce, NULL, &timer1_attr );
    timer_pv2 = osTimerNew( Timer2Callback, osTimerOnce, NULL, &timer2_attr );
    //таймер задержки проверки результата вкл/выкл контактора
    timer_pv3 = osTimerNew( Timer3Callback, osTimerOnce, NULL, &timer3_attr );
    //создаем задачу управления коммутацией панелей
    osThreadNew( TaskPv, NULL, &pv_attr );
 }

//*************************************************************************************************
// Задача обработки событий управления солнечными панелями
//*************************************************************************************************
static void TaskPv( void *pvParameters ) {

    PvCtrl mode;
    uint32_t event;
    
    for ( ;; ) {
        event = osEventFlagsWait( pv_event, EVN_PV_MASK, osFlagsWaitAny, osWaitForever );
        if ( event & EVN_RTC_5MINUTES )
            PvCtrlMain(); //подкл/откл солнечных панелей к контроллеру MPPT, интервал 5 минут 
        if ( event & EVN_PV_CHECK ) {
            //проверка включения/выключения контактора
            mode = (PvCtrl)EepromLoad( EEP_PV_CONN );
            if ( mode == PV_CTRL_ON ) {
                //проверка на включение
                if ( PvGetStat() == PV_STAT_ON ) {
                    EventLog( MessageLog( ID_DEV_PV, LOG_MSG_PV_ON ) );
                    ConsoleSend( Message( CONS_MSG_CRLF ), CONS_SELECTED );
                    ConsoleSend( MessageLog( ID_DEV_PV, LOG_MSG_PV_ON ), CONS_SELECTED );
                    ConsoleSend( Message( CONS_MSG_CRLF ), CONS_SELECTED );
                    ConsoleSend( Message( CONS_MSG_PROMPT ), CONS_SELECTED );
                    Informing( VOICE_PV_ON, NULL );
                   }
                 else {
                    EventLog( ErrorDescr( ID_DEV_PV, 0, LOG_MSG_PV_ON_ERR ) );
                    ConsoleSend( Message( CONS_MSG_CRLF ), CONS_SELECTED );
                    ConsoleSend( MessageLog( ID_DEV_PV, LOG_MSG_PV_ON_ERR ), CONS_SELECTED );
                    ConsoleSend( Message( CONS_MSG_CRLF ), CONS_SELECTED );
                    ConsoleSend( Message( CONS_MSG_PROMPT ), CONS_SELECTED );
                   }
               }
            if ( mode == PV_CTRL_OFF ) {
                //проверка на выключение
                if ( PvGetStat() == PV_STAT_OFF ) {
                    EventLog( MessageLog( ID_DEV_PV, LOG_MSG_PV_OFF ) );
                    ConsoleSend( Message( CONS_MSG_CRLF ), CONS_SELECTED );
                    ConsoleSend( MessageLog( ID_DEV_PV, LOG_MSG_PV_OFF ), CONS_SELECTED );
                    ConsoleSend( Message( CONS_MSG_CRLF ), CONS_SELECTED );
                    ConsoleSend( Message( CONS_MSG_PROMPT ), CONS_SELECTED );
                    Informing( VOICE_PV_OFF, NULL );
                   }
                 else {
                    EventLog( MessageLog( ID_DEV_PV, LOG_MSG_PV_OFF_ERR ) );
                    ConsoleSend( Message( CONS_MSG_CRLF ), CONS_SELECTED );
                    ConsoleSend( MessageLog( ID_DEV_PV, LOG_MSG_PV_OFF_ERR ), CONS_SELECTED );
                    ConsoleSend( Message( CONS_MSG_CRLF ), CONS_SELECTED );
                    ConsoleSend( Message( CONS_MSG_PROMPT ), CONS_SELECTED );
                   }
               }
           }
       }
 }

//*************************************************************************************************
// Функция обратного вызова таймера, включение панелей
//*************************************************************************************************
static void Timer1Callback( void *arg ) {

    GPIO_PinWrite( PV_PORT1, PVK_ON, RELAY_OFF );
    osTimerStart( timer_pv3, TIME_CHK_PV );
 }

//*************************************************************************************************
// Функция обратного вызова таймера, отключение панелей
//*************************************************************************************************
static void Timer2Callback( void *arg ) {

    GPIO_PinWrite( PV_PORT2, PVK_OFF, RELAY_OFF );
    osTimerStart( timer_pv3, TIME_CHK_PV );
 }

//*************************************************************************************************
// Функция обратного вызова таймера, контроль работы контактора
//*************************************************************************************************
static void Timer3Callback( void *arg ) {

    osEventFlagsSet( pv_event, EVN_PV_CHECK );
 }

//*************************************************************************************************
// Управляет подкл/откл солнечных панелей к контроллеру MPPT при нахождении текущего 
// времени в интервале текущей продолжительности дня, т.е. подключение панелей после восхода, 
// отключение панелей после захода солнца
// return = ERROR   - контроллер MPPT не подключен
//          SUCCESS - функция выполнена
//*************************************************************************************************
static Status PvCtrlMain( void ) {

    float time;
    
    //текущее время 
    time = GetTimeDecimal();
    //проверка текущего времени в интервале восход - заход
    if ( TimeCheck( sunpos.sunrise, time, sunpos.sunset ) == TIME_NOT_INTERVAL && PvGetStat() == PV_STAT_ON )
        PvControl( PV_CTRL_OFF, EEPROM_SAVE ); //отключаем панели
    //проверка подключения MPPT
    if ( MpptCheckPwr() == MPPT_POWER_OFF )
        return ERROR; //контроллер заряда выключен
    if ( TimeCheck( sunpos.sunrise, time, sunpos.sunset ) == TIME_IN_INTERVAL && PvGetStat() == PV_STAT_OFF )
        PvControl( PV_CTRL_ON, EEPROM_SAVE ); //подключаем панели
    return SUCCESS;
 }

//*************************************************************************************************
// Управление подключением/отключением солнечных панелей к контроллеру MPPT
// PvCtrl mode        - подключение/отключение панелей 
// EepromMode restore - запись/восстановление состояния режима из EEPROM
// return ErrorCode   - результат выполнения
//*************************************************************************************************
PvError PvControl( PvCtrl ctrl, EepromMode restore ) {

    if ( !GetDataPort( MPPT_ON ) )
        return PV_ERR_NOBAT; //контроллер не подключен к АКБ
    if ( ctrl == PV_CTRL_ON && PvGetStat() == PV_STAT_OFF ) {
        //включим контактор если он еще не вкл.
        GPIO_PinWrite( PV_PORT1, PVK_ON, RELAY_ON );
        //запуск таймера на отключение реле
        osTimerStart( timer_pv1, RELAY_PULSE1 );
       } 
    if ( ctrl == PV_CTRL_OFF && PvGetStat() == PV_STAT_ON ) {
        //выключим контактор если он еще не выкл.
        GPIO_PinWrite( PV_PORT2, PVK_OFF, RELAY_ON );
        //запуск таймера на отключение реле
        osTimerStart( timer_pv2, RELAY_PULSE1 );
       }
    if ( restore == EEPROM_SAVE ) {
        EepromUpdate( EEP_PV_CONN, ctrl );
        EepromSave();
       } 
    return PV_ERR_OK;
 }

//*************************************************************************************************
// Устанавливает режим соединения панелей
// PvMode mode        - режим панелей: паралл/послед
// EepromMode restore - запись/восстановление состояния режима из EEPROM, 
//*************************************************************************************************
void PvSetMode( PvMode mode, EepromMode restore ) {

    //вкл параллельное соединение панелей
    if ( mode == PV_MODE_PAR ) {
        GPIO_PinWrite( PV_PORT1, PV_MODE, RELAY_OFF );
        if ( restore == EEPROM_SAVE )
            EventLog( MessageLog( ID_DEV_PV, LOG_MSG_PV_PAR ) );
       }
    //вкл последовательное соединение панелей
    if ( mode == PV_MODE_SER ) {
        GPIO_PinWrite( PV_PORT1, PV_MODE, RELAY_ON );
        if ( restore == EEPROM_SAVE )
            EventLog( MessageLog( ID_DEV_PV, LOG_MSG_PV_SER ) );
       }
    if ( restore == EEPROM_SAVE ) {
        EepromUpdate( EEP_PV_MODE, mode );
        EepromSave();
       } 
 }

//*************************************************************************************************
// Возвращает режим коммутации панелей
// Return PvMode - режим коммутации панелей
//*************************************************************************************************
PvMode PvGetMode( void ) {

    //возвращаем статус соединения панелей
    if ( GPIO_PinRead( PV_PORT1, PV_MODE ) )
        return PV_MODE_SER;
    else return PV_MODE_PAR;
 }

//*************************************************************************************************
// Возвращает статус контактора панелей вкл/выкл
// return PvStatus - статус контактора панелей 
//*************************************************************************************************
PvStatus PvGetStat( void ) {

    if ( GetDataPort( MPPT_PV_ON ) )
        return PV_STAT_ON;
    else return PV_STAT_OFF;
 }

//*************************************************************************************************
// Ведение протокола режимов коммутации панелей
//*************************************************************************************************
static void EventLog( char *text ) {

    char name[40];
    FILE *pv_log;

    if ( !config.log_enable_pv )
        return; //логирование выключено
    if ( SDStatus() == ERROR )
        return; //карты нет
    //формируем имя файла
    if ( !config.mode_logging )
        sprintf( name, "\\mppt\\pv_%s.log", RTCFileName() );
    else sprintf( name, "\\mppt\\%s\\pv_%s.log", RTCFileShort(), RTCFileName() );
    pv_log = fopen( name, "a" );
    if ( pv_log == NULL )
        return;
    //запись строки
    fprintf( pv_log, "%s %s\r\n", RTCGetLog(), text );
    fclose( pv_log );
 }
