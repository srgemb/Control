
//*************************************************************************************************
//
// Управление блоком АВР (автоматический ввод резерва)
//
//*************************************************************************************************

#include <stdio.h>
#include <string.h>

#include "rl_fs.h"
#include "cmsis_os2.h"

#include "device.h"
#include "dev_data.h"
#include "dev_param.h"

#include "alt.h" 
#include "rtc.h" 
#include "ports.h"
#include "eeprom.h"
#include "config.h"
#include "sdcard.h"
#include "batmon.h"
#include "charger.h"
#include "informing.h"
#include "inverter.h" 
#include "message.h" 
#include "system.h" 
#include "events.h" 

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
ALT alt;
osEventFlagsId_t alt_event;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define TIMER_STOP              0           //таймер выключен

#define TIMEOUT_WAIT_EVENT      500         //время ожидания события (msec)

#define ALT_K5DELAY             3000        //задержка отключения контактора K5 в блоке АВР 
                                            //после отключения контакторов К2,К3 (msec)
typedef enum {
    ALT_AC_UNDEF,                           //состояние сети не определено
    ALT_AC_OK,                              //основная сеть есть
    ALT_AC_NO                               //основной сети нет
 } AltACStat;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static AltPower prev_alt;
static bool flg_chk_timer = false;
static AltACStat alt_chk_ac = ALT_AC_UNDEF;
static osTimerId_t timer_k5delay, timer_k5rest;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t alt_attr = {
    .name = "Alt",
    .stack_size = 1024,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn_attr = { .name = "Alt" };
static const osTimerAttr_t timer1_attr = { .name = "AltK5Delay" };
static const osTimerAttr_t timer2_attr = { .name = "AltK5Rest" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void AltCheckPower( void );
static void CheckTimer( void );
static void EventLog( char *msg );
static void TaskAlt( void *pvParameters );
static void Timer1Callback( void *arg );
static void Timer2Callback( void *arg );

//*************************************************************************************************
// Инициализация портов, восстановление предыдущего состояния
//*************************************************************************************************
void AltInit( void ) {

    //настройка портов
    GPIO_SetDir( ALT_PORT, ALT_OFF, GPIO_DIR_OUTPUT );
    GPIO_SetDir( ALT_PORT, ALT_AC_OFF, GPIO_DIR_OUTPUT );
    //состояние основной сети при вкл контроллера
    if ( GetDataPort( ALT_CONNECT ) ) {
        if ( GetDataPort( ALT_AC_MAIN ) )
            alt_chk_ac = ALT_AC_OK;     //основная сеть есть
        else alt_chk_ac = ALT_AC_NO;    //основной сети нет
       }
    else alt_chk_ac = ALT_AC_UNDEF;
    if ( EepromLoad( EEP_ALT_AC_OFF ) ) {
        if ( AltPowerDC() != ALT_ERR_OK ) {
            //восстановить состояние не удалось, обнулим параметр
            EepromUpdate( EEP_ALT_AC_OFF, RELAY_OFF );
            EepromSave();
           }
       }
    alt.timer_delay = TIMER_STOP;
    //создаем задачу и таймеры
    alt_event = osEventFlagsNew( &evn_attr );
    timer_k5delay = osTimerNew( Timer1Callback, osTimerOnce, NULL, &timer1_attr );
    timer_k5rest = osTimerNew( Timer2Callback, osTimerOnce, NULL, &timer2_attr );
    //создаем задачу сканирования портов ввода
    osThreadNew( TaskAlt, NULL, &alt_attr );
 }

//*************************************************************************************************
// Задача управления блоком АВР
//*************************************************************************************************
static void TaskAlt( void *pvParameters ) {

    uint32_t send;
    int32_t event;

    for ( ;; ) {
        //выполнение по событию или с интервалом TIMEOUT_WAIT_EVENT
        event = osEventFlagsWait( alt_event, EVN_ALT_MASK, osFlagsWaitAny, TIMEOUT_WAIT_EVENT );
        if ( event == osErrorTimeout ) {
            //контроль таймера вкл/выкл инверторов при откл/восстановлении
            //оснновной сети, только в режима MODE_SYSTEM_BACKUP
            CheckTimer();
            //отслеживаем состояние основной сети, отключение/восстановление
            AltCheckPower(); 
            //заполняем состояние блока АВР
            alt.connect = GetDataPort( ALT_CONNECT ) ? LINK_CONN_OK : LINK_CONN_NO;
            alt.gen_on = GetDataPort( ALT_GEN_OK ) ? POWER_AC_ON : POWER_AC_OFF;
            alt.main_ac = GetDataPort( ALT_AC_MAIN ) ? POWER_AC_ON : POWER_AC_OFF;
            alt.power = AltPowerStat();
            send = ID_DEV_ALT; //передача данных в HMI
            osMessageQueuePut( hmi_msg, &send, 0, 0 ); 
           }
        else {
            if ( event & EVN_RTC_SECONDS ) {
                //отсчет секундного таймера
                if ( alt.timer_delay )
                    alt.timer_delay--;
               }
           }
       }
 }

//*************************************************************************************************
// Функция обратного вызова таймера - включение реле (прерывание питания контактора K5)
//*************************************************************************************************
static void Timer1Callback( void *arg ) {

    //прерываем питание контактора K5 в блоке АВР, 
    //т.е. подключаем нагрузку к инверторам
    GPIO_PinWrite( ALT_PORT, ALT_OFF, RELAY_ON );
    //запустим таймер продолжительности выключенного состояния
    osTimerStart( timer_k5rest, RELAY_PULSE1 );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - выключения реле
//*************************************************************************************************
static void Timer2Callback( void *arg ) {

    //восстановление состояния реле управляющего контактором К5
    GPIO_PinWrite( ALT_PORT, ALT_OFF, RELAY_OFF );
 }

//*************************************************************************************************
// Отслеживает включение/отключение основной сети AC
// Вызов из TaskAlt() с интервалом 0,5 сек
//*************************************************************************************************
static void AltCheckPower( void ) {

    AltPower stat_alt;
    
    if ( !GetDataPort( ALT_CONNECT ) ) {
        alt_chk_ac = ALT_AC_UNDEF;
        return; //блок АВР не подключен
       }
    else {
        //установим исходное состояние сети если это еще не выполнено
        if ( alt_chk_ac == ALT_AC_UNDEF ) {
            prev_alt = AltPowerStat();
            if ( GetDataPort( ALT_AC_MAIN ) )
                alt_chk_ac = ALT_AC_OK;   //основная сеть есть
            else alt_chk_ac = ALT_AC_NO;  //основной сети нет
           }
        //отслеживаем изменение источников напряжения
        stat_alt = AltPowerStat();
        if ( stat_alt != prev_alt ) {
            prev_alt = stat_alt;
            if ( stat_alt == ALT_POWER_DC )
                Informing( VOICE_POWER_TS, NULL );
            if ( stat_alt == ALT_POWER_GEN )
                Informing( VOICE_POWER_GEN, NULL );
            if ( stat_alt == ALT_POWER_AC )
                Informing( VOICE_POWER_AC, NULL );
           }
       }
    //отслеживание откл/восстановление основной сети
    if ( GetDataPort( ALT_AC_MAIN ) && alt_chk_ac == ALT_AC_NO ) {
        //сеть AC восстановлена
        alt_chk_ac = ALT_AC_OK; 
        osEventFlagsSet( gen_event, EVN_ALT_AC_REST );
        EventLog( MessageLog( ID_DEV_ALT, LOG_MSG_AC_RESTORE ) );
        if ( flg_chk_timer == true ) {
            //таймер был запущен ранее, выключим запуск инверторов
            flg_chk_timer = false;
            alt.timer_delay = TIMER_STOP;
           }
        else {
            //таймер еще не запускали
            alt.timer_delay = config.delay_stop_inv;
            flg_chk_timer = true; //разрешим контроль таймера задержки
           }
        Informing( VOICE_AC_ON, NULL );
        //восстановим контроль заряда после восстановления основной сети
        //Charger( (ChargeMode)EepromLoad( EEP_BP_CHARGE ), EEPROM_RESTORE );
        return;
       } 
    if ( !GetDataPort( ALT_AC_MAIN ) && alt_chk_ac == ALT_AC_OK ) {
        //сеть AC отключена
        alt_chk_ac = ALT_AC_NO; 
        osEventFlagsSet( gen_event, EVN_ALT_AC_LOST );
        //задержка на завершения переходных процессов перед отключением 
        //контактора K5 в блоке АВР перевод нагрузки на инверторы
        osTimerStart( timer_k5delay, ALT_K5DELAY );
        EventLog( MessageLog( ID_DEV_ALT, LOG_MSG_AC_DISCON ) );
        if ( flg_chk_timer == true ) {
            //таймер был запущен ранее, блокируем выключение инверторов
            flg_chk_timer = false;
            alt.timer_delay = TIMER_STOP;
           }
        else {
            //таймер еще не запускали
            alt.timer_delay = config.delay_start_inv;
            flg_chk_timer = true; //разрешим контроль таймера задержки
           }
        Informing( VOICE_AC_OFF, NULL );
        return;
       } 
 }

//*************************************************************************************************
// Переключение нагрузки к основной сети (генератору)
// return ErrorCode - результат выполнения
//*************************************************************************************************
AltError AltPowerAC( void ) {

    if ( !GetDataPort( ALT_CONNECT ) )
        return ALT_ERR_NO_CONN;
    if ( GPIO_PinRead( ALT_PORT, ALT_AC_OFF ) ) {
        //реле Р2 блока АВР включено, выключаем, 
        //переход на основную сеть
        GPIO_PinWrite( ALT_PORT, ALT_AC_OFF, RELAY_OFF );
        EepromUpdate( EEP_ALT_AC_OFF, RELAY_OFF );
        EepromSave();
        EventLog( MessageLog( ID_DEV_ALT, LOG_MSG_PWR_AC ) );
        if ( GetDataPort( ALT_AC_MAIN ) )
            Informing( VOICE_POWER_AC, NULL );
        else if ( GetDataPort( ALT_AC_GEN ) )
            Informing( VOICE_POWER_GEN, NULL );
       }
    return ALT_ERR_OK;
 }

//*************************************************************************************************
// Переключение нагрузки на инвертора, которые должны быть предварительно включены
// return ErrorCode - результат выполнения
//*************************************************************************************************
AltError AltPowerDC( void ) {

    if ( !GetDataPort( ALT_CONNECT ) )
        return ALT_ERR_NO_CONN; //блок АВР не подключен
    if ( InvBatConn( ID_DEV_INV1 ) == BAT_DC_OFF && InvBatConn( ID_DEV_INV2 ) == BAT_DC_OFF )
        return ALT_ERR_NO_INV;   //инверторы не подключены
    if ( AltPowerStat() == ALT_POWER_DC )
        return ALT_ERR_OK; //переключение на DC уже выполнено
    //отключаем контакторы К2, К3 в блоке АВР, через реле Р2
    GPIO_PinWrite( ALT_PORT, ALT_AC_OFF, RELAY_ON );
    //сохраним состояние
    EepromUpdate( EEP_ALT_AC_OFF, RELAY_ON );
    EepromSave();
    EventLog( MessageLog( ID_DEV_ALT, LOG_MSG_PWR_INV ) );
    //задержка для завершения переходных процессов перед отключением 
    //контактора K5 в блоке АВР, перевод нагрузки на инверторы
    osTimerStart( timer_k5delay, ALT_K5DELAY );
    return ALT_ERR_OK;
 }

//*************************************************************************************************
// Контроль таймера задержки вкл/выкл инверторов при отключении/восстановлении основной сети
// Проверка выполняется только для режима MODE_SYSTEM_BACKUP и если не было принудительного 
// перехода на инверторы при наличии основной сети
// Вызов из TaskAlt() с интервалом 0,5 сек
//*************************************************************************************************
static void CheckTimer( void ) {

    if ( config.mode_sys == SYSTEM_MODE_TEST || config.mode_sys == SYSTEM_MODE_MIXED )
        return;
    if ( GPIO_PinRead( ALT_PORT, ALT_AC_OFF ) )
        return; //принудительный переход на инверторы 
    if ( flg_chk_timer == false )
        return; //таймеры не включались
    if ( alt.timer_delay ) {
        if ( !GetDataPort( ALT_AC_MAIN | ALT_AC_GEN ) && SecToIntMinutes( alt.timer_delay )  )
            Informing( VOICE_TS_START, NULL );
        return; //отсчет таймеров не завершен
       }
    //время таймера истекло, пора что-то делать
    if ( GetDataPort( ALT_AC_MAIN | ALT_AC_GEN ) ) {
        //сеть восстановлена, выключаем инверторы
        if ( InvBatConn( ID_DEV_INV1 ) == BAT_DC_ON )
            InvCtrl( ID_DEV_INV1, INV_CTRL_OFF );
        if ( InvBatConn( ID_DEV_INV2 ) == BAT_DC_ON )
            InvCtrl( ID_DEV_INV2, INV_CTRL_OFF );
        //после работы инверторов проверим уровень SOC 
        //и при необходимости включим подзарядку
        BatSocCheck(); 
       }
    else {
        //нет сети ни генератора, включаем инверторы
        InvCtrl( ID_DEV_INV1, INV_CTRL_ON );
        InvCtrl( ID_DEV_INV2, INV_CTRL_ON );
       }
    flg_chk_timer = false;
 }

//*************************************************************************************************
// Возвращает информацию о источнике питания нагрузки: основная сеть/генератор/инверторы
// return AltPower - источник питания нагрузки
//*************************************************************************************************
AltPower AltPowerStat( void ) {

    if ( !GetDataPort( ALT_CONNECT ) )
        return ALT_POWER_UNKN;  //блок АВР не подключен
    if ( GPIO_PinRead( ALT_PORT, ALT_AC_OFF ) )
        return ALT_POWER_DC;    //принудительный переход на инверторы
    if ( GetDataPort( ALT_AC_GEN ) )
        return ALT_POWER_GEN;   //питание от генератора
    if ( GetDataPort( ALT_AC_MAIN ) )
        return ALT_POWER_AC;    //питание от основной сети
    return ALT_POWER_DC;
 }

//*************************************************************************************************
// Запись в протокол текстовой строки
// char *msg - текстовая строка добавляемая в файл протокола
//*************************************************************************************************
static void EventLog( char *msg ) {

    char name[80];
    FILE *log;
    
    if ( SDStatus() == ERROR )
        return; //карты нет
    if ( !config.log_enable_alt )
        return; //логирование выключено
    //формируем имя файла
    if ( !config.mode_logging )
        sprintf( name, "\\alt\\alt_%s.log", RTCFileName() );
    else sprintf( name, "\\alt\\%s\\alt_%s.log", RTCFileShort(), RTCFileName() );
    log = fopen( name, "a" );
    if ( log == NULL )
        return;
    //запись в протокол текстовой строки
    fprintf( log, "%s %s\r\n", RTCGetLog(), msg );
    fclose( log );
 }
