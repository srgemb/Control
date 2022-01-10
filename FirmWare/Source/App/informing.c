
//*************************************************************************************************
//
// Управление голосовыми сообщениями (воспроизведение внешним модулем)
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "rl_fs.h"
#include "cmsis_os2.h"

#include "device.h"
#include "dev_param.h"
#include "config.h"

#include "main.h"
#include "inverter.h"
#include "sound.h"
#include "rtc.h"
#include "ports.h"
#include "batmon.h"
#include "charger.h"
#include "modbus.h"
#include "sdcard.h"
#include "outinfo.h"
#include "command.h"
#include "informing.h"
#include "modbus_def.h"
#include "events.h"
#include "voice_ext.h"

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
VOICE voice;
osEventFlagsId_t info_event = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
typedef struct {               
    VoiceId id_info;                        //ID сообщения
    char    name_info[20];                  //имя сообщения
} INFO;

static const INFO info[] = {
    VOICE_INFO,           "info",           //Та-та-та (сигнал начала информации)        |A| | | |
    VOICE_LOW_CHARGE,     "low_charge",     //Низкий заряд батареи %                     | |И| | |
    VOICE_LEVEL_CHARGE,   "level_charge",   //Уровень заряда батареи %                   | |И|П| |
    VOICE_POWER_INV,      "power_inv",      //Нагрузка инверторов % и %                  | |И| | |
    VOICE_TTGO,           "ttgo",           //Продолжительность работы % часов % минут   | |И| | |
    VOICE_CHARGE,         "charge",         //Плановая подзарядка                        | | | |Р|
    VOICE_BAT_FULL,       "bat_full",       //Батарея заряжена                           |A| | | |
    VOICE_POWER_AC,       "power_ac",       //Питание нагрузки от основной сети          |A| | | |
    VOICE_POWER_TS,       "power_ts",       //Питание нагрузки от инверторов             |A| | | |
    VOICE_POWER_GEN,      "power_gen",      //Питание нагрузки от генератора             |A| | | |
    VOICE_AC_OFF,         "ac_off",         //Основная сеть отключена                    |A| | | |
    VOICE_AC_ON,          "ac_on",          //Основная сеть восстановлена                |A| | | |
    VOICE_POWER_CTRL,     "power_ctrl",     //Нет питания контроллера                    | | | | |
    VOICE_BAT_NOCONN,     "bat_noconn",     //Батарея не подключена                      |А| | | |
    VOICE_CHARGE_END,     "charge_end",     //Подзарядка завершена                       |A| | | |
    VOICE_GEN_RUN,        "gen_run",        //Генератор запущен                          |A| | | |
    VOICE_GEN_OFF,        "gen_off",        //Генератор выключен                         |A| | | |
    VOICE_GEN_DISCON,     "gen_discon",     //Генератор отключился                       |A| | | |
    VOICE_GEN_CHECK,      "gen_check",      //Проверьте генератор                        |A| | | |
    VOICE_PV_ON,          "pv_on",          //Солнечные панели подключены                |A| | | |
    VOICE_PV_OFF,         "pv_off",         //Солнечные панели отключены                 |A| | | |
    VOICE_TRC_ON,         "trc_on",         //Управление трекером включено               |A| | | |
    VOICE_TRC_OFF,        "trc_off",        //Управление трекером выключено              |A| | | |
    VOICE_TEST,           "test",           //Плановое тестирование                      | | | |Р|
    VOICE_TS_START,       "ts_start",       //Инверторы будут запущены через ... минут   |A| | | |
    VOICE_VOICE,          "voice",          //Голосовой информатор                       | | | |Р|
    VOICE_INFO,           ""                //окончание списка
 };

// Примечания:
// A - Автоматический вывод голосового сообщения при наступлении события
// И - Периодический вывод голосового сообщения при работе инверторов
// П - Периодический вывод голосового сообщения при подзарядке от PB-1000-224
// Р - Вывод только по команде из планировщика или консоли 

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static uint8_t level_vol = 1;
static osTimerId_t timer1_info, timer2_info, timer3_info;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t info_attr = {
    .name = "Informing", 
    .stack_size = 896,
    .priority = osPriorityNormal
 };

static const osTimerAttr_t timer1_attr = { .name = "InfoBat" };
static const osTimerAttr_t timer2_attr = { .name = "InfoInv" };
static const osTimerAttr_t timer3_attr = { .name = "InfoTtg" };
static const osEventFlagsAttr_t evn_attr = { .name = "Informing" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void DevVoice( void );
static Volume GetVolume( void );
static void MainSetVolume( void );
static void EventLog( uint16_t *data, bool limit );
static void TaskInforming( void *pvParameters );
static void Timer1Callback( void *arg );
static void Timer2Callback( void *arg );
static void Timer3Callback( void *arg );

//*************************************************************************************************
// Инициализация
//*************************************************************************************************
void VoiceInit( void ) {

    //очередь сообщений
    info_event = osEventFlagsNew( &evn_attr );
    //таймеры интервалов информирования
    timer1_info = osTimerNew( Timer1Callback, osTimerPeriodic, NULL, &timer1_attr );
    timer2_info = osTimerNew( Timer2Callback, osTimerPeriodic, NULL, &timer2_attr );
    timer3_info = osTimerNew( Timer3Callback, osTimerPeriodic, NULL, &timer3_attr );
    //создаем задачу
    osThreadNew( TaskInforming, NULL, &info_attr );
 }

//*************************************************************************************************
// Задача управления голосовыми сообщениями
//*************************************************************************************************
static void TaskInforming( void *pvParameters ) {

    uint32_t event;
    
    for ( ;; ) {
        //стартуем таймеры
        if ( !osTimerIsRunning( timer1_info ) )
            osTimerStart( timer1_info, _TIME_INFO_BAT * MIN_TO_TICK );
        if ( !osTimerIsRunning( timer2_info ) )
            osTimerStart( timer2_info, _TIME_INFO_INV * MIN_TO_TICK );
        if ( !osTimerIsRunning( timer3_info ) )
            osTimerStart( timer3_info, _TIME_INFO_TTGO * MIN_TO_TICK );
        event = osEventFlagsWait( info_event, EVN_INFO_MASK, osFlagsWaitAny, osWaitForever );
        if ( event & EVN_RTC_SECONDS )
            DevVoice(); //запрос состояния голосового информатора
        if ( event & EVN_RTC_5MINUTES )
            MainSetVolume(); //установка громкости
        if ( event & EVN_INFO_BAT ) {
            //подзарядка от PB-1000-224 сообщение об уровне заряда
            if ( charger.charge_mode != CHARGE_OFF )
                Informing( VOICE_LEVEL_CHARGE, NULL ); //подзарядка включена
           }
        if ( event & EVN_INFO_INV ) {
            //один из инверторов включен, сообщение о мощности нагрузки
            if ( inv1.mode == INV_MODE_ON || inv2.mode == INV_MODE_ON )
                Informing( VOICE_POWER_INV, NULL );
           }
        if ( event & EVN_INFO_TTG ) {
            //сообщение о продолжительности работы и уровне заряда
            if ( inv1.mode == INV_MODE_ON || inv2.mode == INV_MODE_ON ) {
                //один из инверторов включен
                Informing( VOICE_LEVEL_CHARGE, NULL );
                Informing( VOICE_TTGO, NULL );
               }
           }
       }
 }

//*************************************************************************************************
// Функция обратного вызова таймера, информирование о состоянии уровня заряда
//*************************************************************************************************
static void Timer1Callback( void *arg ) {

    osEventFlagsSet( info_event, EVN_INFO_BAT );
 }

//*************************************************************************************************
// Функция обратного вызова таймера, информирование о нагрузке инверторов
//*************************************************************************************************
static void Timer2Callback( void *arg ) {

    osEventFlagsSet( info_event, EVN_INFO_INV );
 }

//*************************************************************************************************
// Функция обратного вызова таймера, информирование о продолжительности работы
//*************************************************************************************************
static void Timer3Callback( void *arg ) {

    osEventFlagsSet( info_event, EVN_INFO_TTG );
 }

//*************************************************************************************************
// Формируем список воспроизведения для голосового информатора
// VoiceId id_info    - ID сообщения, если ID > 0 - параметр name_info игнорируется
// char *name_info    - имя сообщения, если ID = 0
// return ModBusError - результат выполнения
//*************************************************************************************************
ModBusError Informing( VoiceId id_info, char *name_info ) {

    uint8_t i, cnt_rpt;
    bool limit = false;
    uint16_t voice_data[EXVOI_REG_WR_MAX];
    uint16_t len = sizeof( voice_data );
    ModBusError status = MBUS_ERROR_PARAM;
    MBUS_REQUEST reqst = { MB_ID_DEV_VOICE, FUNC_WR_MULT_REG, EXVOI_REG_WR_CMD, 4, &voice_data, &len };

    if ( !id_info && strlen( name_info ) < 3 && name_info == NULL )
        return MBUS_ERROR_PARAM;
    if ( id_info ) {
        //проверка номера сообщения
        for ( i = 0; info[i].id_info; i++ ) {
            if ( id_info == info[i].id_info )
                break;
           }
       }
    else {
        //проверка имени сообщения
        for ( i = 0; info[i].id_info; i++ ) {
            if ( strcasecmp( name_info, info[i].name_info ) )
                continue;
            break;
           }
       }
    if ( !info[i].id_info ) //проверка на "конец списка"
        return MBUS_ERROR_PARAM;
    if ( voice.link == LINK_CONN_NO ) {
        //уст-ва нет в сети, просигналим через внутренний динамик
        SoundPlay( VoiceToSound( info[i].id_info ), NULL );
        return MBUS_CONNECT_LOST;
       }
    //сообщение идентифицировано, формируем запрос MODBUS
    voice_data[EXVOI_REG_WR_CMD] = EXVOI_CMD_PLAY;   //воспроизведение
    voice_data[EXVOI_REG_WR_VOLUME] = level_vol;     //уровень громкости указан без команды установки
    voice_data[EXVOI_REG_WR_PAR1] = VOICE_INFO;      //предварительное сообщений
    voice_data[EXVOI_REG_WR_PAR2] = info[i].id_info; //основное сообщение
    //определение значения параметров для голосового информатора
    if ( info[i].id_info == VOICE_LOW_CHARGE || info[i].id_info == VOICE_LEVEL_CHARGE ) {
        reqst.cnt_reg = 5; //количество регистров данных для уровня зарядки
        //проверка на допустимые значения SOC <= 100%
        if ( batmon.soc > 100 )
            limit = true;
        voice_data[EXVOI_REG_WR_PAR3] = EXVOI_PAR_PERCENT | (uint16_t)batmon.soc;
       }
    if ( info[i].id_info == VOICE_POWER_INV ) { 
        reqst.cnt_reg = 6; //количество регистров данных для мощности нагрузки
        //проверка на допустимые значения, нагрузка инверторов <= 100%
        if ( inv1.power_perc > 100 || inv2.power_perc > 100 )
            limit = true;
        voice_data[EXVOI_REG_WR_PAR3] = EXVOI_PAR_PERCENT | (uint16_t)inv1.power_perc;
        voice_data[EXVOI_REG_WR_PAR4] = EXVOI_PAR_PERCENT | (uint16_t)inv2.power_perc;
       }
    if ( info[i].id_info == VOICE_TTGO ) {
        reqst.cnt_reg = 6; //количество регистров данных для продолжительности работы
        //проверка на допустимые значения продолжительности работы <= 100%
        if ( BatMonTTG( BATMON_TTG_HOUR ) > 100 )
            limit = true;
        if ( BatMonTTG( BATMON_TTG_HOUR ) ) //значение "0" часов не озвучиваем
            voice_data[EXVOI_REG_WR_PAR3] = EXVOI_PAR_HOURS | (uint16_t)BatMonTTG( BATMON_TTG_HOUR );
        voice_data[EXVOI_REG_WR_PAR4] = EXVOI_PAR_MINUTES | (uint16_t)BatMonTTG( BATMON_TTG_HOUR );
       }
    if ( info[i].id_info == VOICE_TS_START ) {
        reqst.cnt_reg = 5; //количество регистров данных для времени вкл инверторов
        voice_data[EXVOI_REG_WR_PAR3] = EXVOI_PAR_MINUTES | SecToIntMinutes( alt.timer_delay );
       }
    EventLog( voice_data, limit );
    if ( limit == false ) //воспроизвести сообщение если значение не превышает допустимые пределы
        cnt_rpt = CNT_REPEAT_REQST;
        do {
            reqst.ptr_lendata = &len;
            status = ModBusRequest( &reqst );
           } while ( cnt_rpt-- && ( status == MBUS_ANSWER_CRC || status == MBUS_ANSWER_TIMEOUT ) );
    return status;
 }

//*************************************************************************************************
// Запрос состояния голосового информатора
// Выполняется с интервалом 1 секунда
//*************************************************************************************************
static void DevVoice( void ) {

    uint8_t cnt_rpt;
    ModBusError status;
    uint16_t reg_data[2];
    uint16_t len = sizeof( reg_data );
    MBUS_REQUEST reqst = { MB_ID_DEV_VOICE, FUNC_RD_HOLD_REG, EXVOI_REG_RD_STAT, 2, &reg_data, &len };

    cnt_rpt = CNT_REPEAT_REQST;
    do {
        len = sizeof( reg_data );
        status = ModBusRequest( &reqst );
       } while ( cnt_rpt-- && ( status == MBUS_ANSWER_CRC || status == MBUS_ANSWER_TIMEOUT ) );
    if ( status == MBUS_ANSWER_OK ) {
        //голосовой информатор подключен
        voice.link = LINK_CONN_OK;
        voice.stat = reg_data[EXVOI_REG_RD_STAT];
        voice.volume = (Volume)reg_data[EXVOI_REG_RD_VOLUME];
       }
    else {
        //голосовой информатор не отвечает
        voice.link = LINK_CONN_NO;
        voice.stat = 0;
        voice.volume = VOLUME0;
       }
 }

//*************************************************************************************************
// Установка громкости воспроизведения сообщений голосовым информатором
// Выполняется с интервалом 5 минут
// Volume volume      - уровень громкости: 0...9 
// return ModBusError - результат выполнения
//*************************************************************************************************
ModBusError SetVolume( Volume volume ) {

    uint8_t cnt_rpt;
    uint16_t reg_data[2];
    ModBusError status;
    uint16_t len = sizeof( reg_data );
    MBUS_REQUEST reqst = { MB_ID_DEV_VOICE, FUNC_WR_MULT_REG, EXVOI_REG_WR_CMD, 2, &reg_data, &len };

    if ( voice.link == LINK_CONN_NO )
        return MBUS_CONNECT_LOST; //уст-ва нет в сети
    if ( volume > VOLUME9 )
        return MBUS_ERROR_PARAM;
    reg_data[EXVOI_REG_WR_CMD] = EXVOI_CMD_VOLUME;      //команда установки громкости
    reg_data[EXVOI_REG_WR_VOLUME] = level_vol = volume; //значение уровня громкости
    //отправка команды установки громкости
    cnt_rpt = CNT_REPEAT_REQST;
    do {
        len = sizeof( reg_data );
        status = ModBusRequest( &reqst );
       } while ( cnt_rpt-- && ( status == MBUS_ANSWER_CRC || status == MBUS_ANSWER_TIMEOUT ) );
    return status;
 }

//*************************************************************************************************
// Установка громкости воспроизведения сообщений голосовым информатором по часовым интервалам
//*************************************************************************************************
static void MainSetVolume( void ) {

    Volume volume;
    
    volume = GetVolume();
    if ( voice.volume != volume )
        SetVolume( volume );
 }

//*************************************************************************************************
// Определение уровня громкости по часовым интервалам
// result - код уровня громкости
// с 00:00:00 - 0%  = 0
// с 08:00:00 - 10% = 1
// с 09:00:00 - 40% = 4
// с 15:00:00 - 30% = 3
// с 20:00:00 - 10% = 1
// с 22:00:00 - 00% = 0
//*************************************************************************************************
static Volume GetVolume( void ) {

    Volume volume;
    static RTC_TIME_Type Time;
    
    RTC_GetFullTime( LPC_RTC, &Time );
    if ( Time.HOUR < 8 )
        volume = VOLUME0;
    if ( Time.HOUR >= 8 && Time.HOUR < 9 )
        volume = VOLUME1;
    if ( Time.HOUR >= 9 && Time.HOUR < 15 )
        volume = VOLUME4;
    if ( Time.HOUR >= 15 && Time.HOUR < 20 )
        volume = VOLUME3;
    if ( Time.HOUR >= 20 && Time.HOUR < 22 )
        volume = VOLUME1;
    if ( Time.HOUR >= 22 )
        volume = VOLUME1;
    return volume;
 }

//*************************************************************************************************
// Запись команд управления голосовым информатором в файл протокола
// uint16_t *data - адрес блока данных с параметрами
// bool limit     - признак выхода значения параметра за пределы допустимых значений
//*************************************************************************************************
static void EventLog( uint16_t *data, bool limit ) {

    char name[40];
    FILE *log;
    
    if ( SDStatus() == ERROR )
        return; //карты нет
    if ( !config.mode_logging )
        sprintf( name, "\\voice\\voice_%s.log", RTCFileName() );
    else sprintf( name, "\\voice\\%s\\voice_%s.log", RTCFileShort(), RTCFileName() );
    log = fopen( name, "a" );
    if ( log == NULL )
        return; //файл не открылся
    fprintf( log, "%s LINK = %s, CMD = 0x%04X, PAR = %u %u %u 0x%04X/%u 0x%04X/%u %s\r\n", RTCGetLog(), voice.link ? "YES":"NO", 
            data[EXVOI_REG_WR_CMD], data[EXVOI_REG_WR_VOLUME], data[EXVOI_REG_WR_PAR1], data[EXVOI_REG_WR_PAR2], 
            data[EXVOI_REG_WR_PAR3] & EXVOI_PAR_TYPE, data[EXVOI_REG_WR_PAR3] & EXVOI_PAR_VALUE,
            data[EXVOI_REG_WR_PAR4] & EXVOI_PAR_TYPE, data[EXVOI_REG_WR_PAR4] & EXVOI_PAR_VALUE, limit ? "LIMIT PARAM":"" ); 
    fclose( log );
 }
