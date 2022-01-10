
//*************************************************************************************************
//
// Управление контроллером трекера по протоколу MODBUS
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>

#include "cmsis_os2.h"

#include "dev_data.h"
#include "dev_param.h"
#include "device.h"

#include "main.h"
#include "pv.h"
#include "rtc.h"
#include "ports.h"
#include "mppt.h"
#include "eeprom.h"
#include "sdcard.h"
#include "tracker.h"
#include "trc_calc.h"
#include "can_def.h"
#include "command.h"
#include "modbus.h"
#include "modbus_def.h"
#include "spa_calc.h"
#include "informing.h"
#include "tracker_ext.h"
#include "message.h"
#include "events.h"

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
TRACKER tracker;
osEventFlagsId_t trc_event = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define TIMEOUT_WAIT_EVENT_TRC  500         //время ожидания события (msec)

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static osTimerId_t timer_on, timer_off, timer_log;

//тип логирования данных
typedef enum {
    EVENT_LOG,
    ERROR_LOG
 } EventType;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t trc_attr = {
    .name = "Tracker", 
    .stack_size = 864,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn_attr = { .name = "Tracker" };
static const osTimerAttr_t timer1_attr = { .name = "TrcSunON" };
static const osTimerAttr_t timer2_attr = { .name = "TrcSunOFF" };
static const osTimerAttr_t timer3_attr = { .name = "TrcLog" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void TrackerStatus( void );
static void TracerCtrlAct( void );
static void TracerCtrlMain( void );
static void TracerCtrlPos( void );
static void TrcSavePos( void );
static void EventLog( char *text, EventType evn );
static void TrcLogging( void );
static void Timer1Callback( void *arg );
static void Timer2Callback( void *arg );
static void Timer3Callback( void *arg );
static void TaskTracker( void *pvParameters );
static void TrcCmdLog( MBUS_REQUEST *reqst );

//*************************************************************************************************
// Инициализация портов
//*************************************************************************************************
void TrackerInit( void ) {

    GPIO_SetDir( TRC_PORT, TRC_ON, GPIO_DIR_OUTPUT );
    //восстановим режим реле питания актуаторов
    TRCMode( (PowerAct)EepromLoad( EEP_TRC_ON ), EEPROM_RESTORE );
    //запрос статуса контроллера
    TrackerStatus();
    //очередь сообщений
    trc_event = osEventFlagsNew( &evn_attr );
    timer_on = osTimerNew( Timer1Callback, osTimerOnce, NULL, &timer1_attr );
    timer_off = osTimerNew( Timer2Callback, osTimerOnce, NULL, &timer2_attr );
    timer_log = osTimerNew( Timer3Callback, osTimerOnce, NULL, &timer3_attr );
    //создаем задачу
    osThreadNew( TaskTracker, NULL, &trc_attr );
 }

//*************************************************************************************************
// Задача управления контроллером трекера
//*************************************************************************************************
static void TaskTracker( void *pvParameters ) {
    
    uint32_t send;
    int32_t event;
    
    for ( ;; ) {
        //запуск таймера интервальной записи данных
        if ( !osTimerIsRunning( timer_log ) )
            osTimerStart( timer_log, config.datlog_upd_trc * SEC_TO_TICK );
        //ждем события но не более TIMEOUT_WAIT_EVENT_TRC
        event = osEventFlagsWait( trc_event, EVN_TRC_MASK, osFlagsWaitAny, TIMEOUT_WAIT_EVENT_TRC );
        if ( event == osErrorTimeout ) {
            TrackerStatus();     //чтение состояния контроллера трекера
            //TracerCtrlMain(); //управления режимом работы контроллера трекера: сенсор/командный
            send = ID_DEV_TRC;   //передача данных в HMI
            osMessageQueuePut( hmi_msg, &send, 0, 0 );
           }
        else {
            if ( event & EVN_RTC_5MINUTES ) {
                //управления работой реле питания актуаторов трекера
                TracerCtrlAct();
                //управления режимом работы контроллера трекера: сенсор/командный
                TracerCtrlMain();
               }
            if ( event & EVN_TRC_SUN_POS )
                TracerCtrlPos(); //позиционирование трекера в командном режиме по данным SPA
            if ( event & EVN_RTC_1MINUTES )
                TrcSavePos();   //сохраним в файле данные позиционирования
            if ( event & EVN_TRC_SUN_ON ) {
                //сенсор освещался больше времени TIME_SENSOR_ON
                if ( tracker.stat & EXT_CMND_MODE ) {
                    //таймер завершил отсчет, можно переходить на сенсор
                    EventLog( MessageLog( ID_DEV_TRC, LOG_MSG_TRC_SUN ), EVENT_LOG );
                    TrackerCmd( EXTRC_CMD_OFF, 0, 0 );
                   }
               }
            if ( event & EVN_TRC_SUN_OFF ) {
                //сенсор не освещался больше времени TIME_SENSOR_OFF
                if ( !( tracker.stat & EXT_CMND_MODE ) ) {
                    //включим командный режим
                    EventLog( MessageLog( ID_DEV_TRC, LOG_MSG_TRC_CMD ), EVENT_LOG );
                    TrackerCmd( EXTRC_CMD_ON, 0, 0 );
                   }
               }
            if ( event & EVN_TRC_LOG ) {
                TrcLogging(); //интервальное сохранение данных
                osTimerStart( timer_log, config.datlog_upd_trc * SEC_TO_TICK );
               }
           }
       }
 }

//*************************************************************************************************
// Функция обратного вызова таймера - пропала освещенность
//*************************************************************************************************
static void Timer1Callback( void *arg ) {

    osEventFlagsSet( trc_event, EVN_TRC_SUN_ON );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - освещенность пропала
//*************************************************************************************************
static void Timer2Callback( void *arg ) {

    osEventFlagsSet( trc_event, EVN_TRC_SUN_OFF );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - интервальное логирование
//*************************************************************************************************
static void Timer3Callback( void *arg ) {

    osEventFlagsSet( trc_event, EVN_TRC_LOG );
 }

//*************************************************************************************************
// Запрос состояния контроллера трекера
//*************************************************************************************************
static void TrackerStatus( void ) {

    uint8_t cnt_rpt;
    ModBusError status;
    uint16_t trc_data[EXTRC_REG_RD_MAX];
    uint16_t len = sizeof( trc_data );
    MBUS_REQUEST reqst = { MB_ID_DEV_TRACKER, FUNC_RD_HOLD_REG, EXTRC_REG_RD_STAT, EXTRC_REG_RD_MAX, &trc_data, &len };

    tracker.pwr_trc = GetDataPort( TRC_PWR ) ? POWER_ON : POWER_OFF;
    tracker.pwr_act = GPIO_PinRead( TRC_PORT, TRC_ON ) ? TRC_ACT_ON : TRC_ACT_OFF;
    tracker.pwr_fuse = GetDataPort( TRC_FUSE ) ? PROTECT_WORK : PROTECT_OFF;
    cnt_rpt = CNT_REPEAT_REQST;
    do {
        len = sizeof( trc_data );
        status = ModBusRequest( &reqst );
       } while ( cnt_rpt-- && ( status == MBUS_ANSWER_CRC || status == MBUS_ANSWER_TIMEOUT ) );
    if ( status != MBUS_ANSWER_OK ) {
        tracker.link = LINK_CONN_NO;
        tracker.stat = 0;
        tracker.time_on = 0;
        tracker.act_pos_vert = 0;
        tracker.act_pos_horz = 0;
        tracker.act_vert_eep = 0;
        tracker.act_horz_eep = 0;
       }
    else {
        tracker.link = LINK_CONN_OK;
        tracker.stat = trc_data[EXTRC_REG_RD_STAT];
        tracker.time_on = trc_data[EXTRC_REG_RD_TIMEON];
        tracker.act_pos_vert = trc_data[EXTRC_REG_RD_VERT];
        tracker.act_pos_horz = trc_data[EXTRC_REG_RD_HORZ];
        tracker.act_vert_eep = trc_data[EXTRC_REG_RD_VEEP];
        tracker.act_horz_eep = trc_data[EXTRC_REG_RD_HEEP];
       }
 }

//*************************************************************************************************
// Управления работой реле питания актуаторов трекера
// Вызывается из TaskTracker() с интервалом 5 минут
//*************************************************************************************************
static void TracerCtrlAct( void ) {

    float time;

    if ( tracker.pwr_trc == POWER_OFF || tracker.pwr_fuse == PROTECT_WORK )
        return; //нет питания трекера или сработала защита
    if ( MpptCheckPwr() == MPPT_POWER_OFF )
        return; //контроллер заряда MPPT выключен
    //текущее время
    time = GetTimeDecimal();
    //после восхода - до захода солнца - включаем реле
    if ( time > sunpos.sunrise && time < sunpos.sunset && tracker.pwr_act == TRC_ACT_OFF )
        TRCMode( TRC_ACT_ON, EEPROM_SAVE );   //включаем реле управления актуаторами
    //после полуночи - до восхода солнца - выключаем реле
    if ( time > 0 && time < sunpos.sunrise && tracker.pwr_act == TRC_ACT_ON )
        TRCMode( TRC_ACT_OFF, EEPROM_SAVE );  //выключаем реле управления актуаторами
 }

//*************************************************************************************************
// Позиционирование трекера в командном режиме по данным SPA
//*************************************************************************************************
static void TracerCtrlPos( void ) {

    if ( tracker.link == LINK_CONN_OK && ( tracker.stat & EXT_CMND_MODE ) )
        TrackerCmd( EXTRC_VERT | EXTRC_HORZ, StemVert( sunpos.zenith ), StemHorz( sunpos.azimuth ) );
 }

//*************************************************************************************************
// Управления режимом работы контроллера трекера: сенсор/командный
// Управление выполняется при включенном питании трекера, включенном контроллере MPPT
//*************************************************************************************************
static void TracerCtrlMain( void ) {

    float time;

    if ( tracker.pwr_trc == POWER_OFF || tracker.pwr_fuse == PROTECT_WORK )
        return; //нет питания трекера или сработала защита
    if ( MpptCheckPwr() == MPPT_POWER_OFF )
        return; //контроллер заряда MPPT выключен
    //текущее время
    time = GetTimeDecimal();
    //проверка диапазона времени
    if ( TimeCheck( sunpos.sunrise, time, sunpos.sunset ) ) {
        //наступило дневное время суток, проверка подключения трекера
        if ( tracker.link != true )
            return; //трекер не подключен
        //проверим наличие освещенности на сенсоре через TIME_BEGIN_TRACK после восхода
        if ( ( time - sunpos.sunrise ) > TimeToDecimal( 0, 0, TIME_BEGIN_TRACK ) ) {
            //контролируем наличие освещенности сенсора
            if ( tracker.stat & EXT_SOLAR_ALL ) {
                //освещенность восстановлена
                osTimerStop( timer_off );
                //запуск таймера задержки переключения режима "командный" -> "сенсор"
                if ( !osTimerIsRunning( timer_on ) )
                    osTimerStart( timer_on, TIME_SENSOR_ON * SEC_TO_TICK );
               }
            else {
                //освещенность пропала
                osTimerStop( timer_on );
                //запуск таймера задержки переключения режима "сенсор" -> "командный"
                if ( !osTimerIsRunning( timer_off ) )
                    osTimerStart( timer_off, TIME_SENSOR_OFF * SEC_TO_TICK );
               }
           }
       }
    else {
        //наступило ночное время суток
        if ( tracker.link == LINK_CONN_NO )
            return; //трекер не подключен
        //выключим командный режим
        if ( tracker.stat & EXT_CMND_MODE ) {
            //выключим командный режим
            EventLog( MessageLog( ID_DEV_TRC, LOG_MSG_TRC_SUN ), EVENT_LOG );
            TrackerCmd( EXTRC_CMD_OFF, 0, 0 );
           }
       }
 }

//*************************************************************************************************
// Управление реле питания актуаторов трекера
// PowerAct mode      - режим: вкл/выкл
// EepromMode restore - запись/чтения состояния в EEPROM
//*************************************************************************************************
void TRCMode( PowerAct mode, EepromMode restore ) {

    //вкл
    if ( mode == TRC_ACT_ON ) {
        GPIO_PinWrite( TRC_PORT, TRC_ON, RELAY_ON );
        //информируем только при записи
        if ( restore == EEPROM_SAVE ) {
            Informing( VOICE_TRC_ON, NULL );
            EventLog( MessageLog( ID_DEV_TRC, LOG_MSG_ACT_ON ), EVENT_LOG );
           }
       } 
    //выкл
    if ( mode == TRC_ACT_OFF ) {
        GPIO_PinWrite( TRC_PORT, TRC_ON, RELAY_OFF );
        //информируем только при записи
        if ( restore == EEPROM_SAVE ) {
            Informing( VOICE_TRC_OFF, NULL );
            EventLog( MessageLog( ID_DEV_TRC, LOG_MSG_ACT_OFF ), EVENT_LOG );
           }
       } 
 }

//*************************************************************************************************
// Управление позиционированием трекера
// TrackerAct type_act   - тип позиционирования верт./горз.
// TrackerPos type_value - тип параметра град./мм
// float value           - значение для позиционирования (град./мм), значения углов 
//                         передаются фактические т.е. посчитанные по SPA
// return TrackerPosErr  - результат выполнения команды
//*************************************************************************************************
TrackerPosErr TrackerSetPos( TrackerAct type_act, TrackerPos type_value, float value ) {

    char str[80];
    
    //проверим исходные параметры, длинна в (мм) для всех 0 - 900
    if ( type_act == TRC_POS_LENGTH && ( value < 0 || value > 900 ) )
        return TRC_ERR_VALUE;
    //проверим исходные параметры, для вертикали, градусы: 0.0 - 90.0
    if ( type_act == TRC_POS_VERTICAL && type_value == TRC_POS_DEGREE && ( value < 0 || value > 90 ) )
        return TRC_ERR_VALUE;
    //проверим исходные параметры, для горизонтали, градусы: 54.51 - 305.20
    if ( type_act == TRC_POS_HORIZONTAL && type_value == TRC_POS_DEGREE && ( value < MIN_ANGLE_AZIMUT || value > MAX_ANGLE_AZIMUT ) )
        return TRC_ERR_VALUE;
    
    //вывод обработанных значений
    if ( type_act == TRC_POS_VERTICAL ) {
        if ( type_value == TRC_POS_DEGREE )
            sprintf( str, "VERT = %.1f° = %d mm\r\n", value, StemVert( value ) );
        else sprintf( str, "VERT = %d mm = %.1f°\r\n", (uint16_t)value, AngleVert( (uint16_t)value ) );
       }
    if ( type_act == TRC_POS_HORIZONTAL ) {
        if ( type_value == TRC_POS_DEGREE )
            sprintf( str, "HORZ = %.1f° = %d mm\r\n", value, StemHorz( value ) );
        else sprintf( str, "HORZ = %d mm = %.1f°\r\n", (uint16_t)value, AngleHorz( (uint16_t)value ) );
       }
    ConsoleSend( str, CONS_SELECTED );
    
    if ( type_act == TRC_POS_VERTICAL && type_value == TRC_POS_LENGTH ) {
        //вертикальное позиционирование по (мм)
        if ( TrackerCmd( EXTRC_VERT, (uint16_t)value, 0 ) == SUCCESS )
            return TRC_SUCCESS;
        else return TRC_ERR_SEND;
       }
    if ( type_act == TRC_POS_VERTICAL && type_value == TRC_POS_DEGREE ) {
        //вертикальное позиционирование по (град)
        if ( TrackerCmd( EXTRC_VERT, StemVert( value ), 0 ) == SUCCESS )
            return TRC_SUCCESS;
        else return TRC_ERR_SEND;
       }
    if ( type_act == TRC_POS_HORIZONTAL && type_value == TRC_POS_LENGTH ) {
        //горизонтальное позиционирование по (мм)
        if ( TrackerCmd( EXTRC_HORZ, 0, (uint16_t)value ) == SUCCESS )
            return TRC_SUCCESS;
        else return TRC_ERR_SEND;
       }
    if ( type_act == TRC_POS_HORIZONTAL && type_value == TRC_POS_DEGREE ) {
        //горизонтальное позиционирование по (град)
        if ( TrackerCmd( EXTRC_HORZ, 0, StemHorz( value ) ) == SUCCESS ) 
            return TRC_SUCCESS;
        else return TRC_ERR_SEND;
       }
    return TRC_ERR_PARAM;
 }

//*************************************************************************************************
// Отправка команды в контроллер трекера
// uint16_t cmnd - код команды для контроллера трекера, допускается набор команд
// uint16_t param1 - значение параметра 1
// uint16_t param2 - значение параметра 2
// result ERROR    - трекер не подключен
//        SUCCESS  - команда передана
//*************************************************************************************************
Status TrackerCmd( uint16_t cmnd, uint16_t param1, uint16_t param2 ) {

    uint8_t cnt_rpt;
    uint32_t send, msg;
    ModBusError status;
    uint16_t trc_data[EXTRC_REG_WR_MAX];
    uint16_t len = sizeof( trc_data );
    MBUS_REQUEST reqst = { MB_ID_DEV_TRACKER, FUNC_WR_MULT_REG, EXTRC_REG_WR_CMD, EXTRC_REG_WR_MAX, &trc_data, &len };

    if ( tracker.link == LINK_CONN_NO )
        return ERROR;
    reqst.cnt_reg = 1; //передача только команды
    trc_data[EXTRC_REG_WR_CMD] = cmnd;
    trc_data[EXTRC_REG_WR_VERT] = param1;
    trc_data[EXTRC_REG_WR_HORZ] = param2;
    //определим кол-во регистров для передачи
    if ( cmnd & EXTRC_VERT || cmnd & EXTRC_VERT_VAL )
        reqst.cnt_reg = 2; //только вертикальный актуатор
    if ( cmnd & EXTRC_HORZ || cmnd & EXTRC_HORZ_VAL )
        reqst.cnt_reg = 3; //т.к. значение горизонтального актуатора передается всегда в 3-м регистре
    TrcCmdLog( &reqst );
    cnt_rpt = CNT_REPEAT_REQST;
    do {
        len = sizeof( trc_data );
        status = ModBusRequest( &reqst );
       } while ( cnt_rpt-- && ( status == MBUS_ANSWER_CRC || status == MBUS_ANSWER_TIMEOUT ) );
    if ( status == MBUS_ANSWER_OK ) {
        send = ID_DEV_TRC;   //передача данных в HMI
        osMessageQueuePut( hmi_msg, &send, 0, 0 );
        //передаем в модуль HMI событие выполнения команды
        msg = CAN_DEV_ID( ID_DEV_LOG ) | CAN_PARAM_ID( ID_DEV_TRC );
        osMessageQueuePut( hmi_msg, &msg, 0, 0 );
        return SUCCESS;
       }
    else {
        EventLog( ModBusErrDesc( status ), ERROR_LOG );
        return ERROR;
       }
 }

//*************************************************************************************************
// Восстановить из файла trc_position.data значений позиционирования актуаторов трекера
//*************************************************************************************************
void TrcRestPos( void ) {

    char name[64];
    FILE *trc_opn;
    char date[12], time[12];
    int vrt, hrz;

    if ( SDStatus() == ERROR )
        return; //карты нет
    if ( !tracker.link ) 
        return; //трекер не подключен по интерфейсу RS-485
    sprintf( name, "\\trc\\trc_position.data" );
    trc_opn = fopen( name, "r" );
    if ( trc_opn == NULL )
        return; //файл не открылся
    fscanf( trc_opn, "%s %s %d %d", date, time, &vrt, &hrz ); 
    fclose( trc_opn );
    sprintf( name, "%s %s %u %u\r\n", date, time, vrt, hrz );
    ConsoleSend( name, CONS_NORMAL );
    //проверка достоверности значений положения актуаторов
    if ( !tracker.act_pos_vert && !( tracker.stat & EXT_TERM_VCLOSE ) )
        return; //актуатор находится в открытом состоянии, но значение положения = "0"
    if ( !tracker.act_pos_horz && !( tracker.stat & EXT_TERM_HCLOSE ) )
        return; //актуатор находится в открытом состоянии, но значение положения = "0"
    //установим значения позиционирования в контроллере трекера
    ;//TrackerCmd( EXTRC_VERT_VAL | EXTRC_HORZ_VAL, vrt, hrz );
 }

//*************************************************************************************************
// Сохранение в файле trc_position.data текущих значений позиционирования актуаторов трекера
//*************************************************************************************************
static void TrcSavePos( void ) {

    char name[64];
    FILE *trc_save;

    if ( SDStatus() == ERROR )
        return; //карты нет
    if ( !tracker.link ) 
        return; //трекер не подключен по интерфейсу RS-485
    //проверка достоверности значений положения актуаторов
    if ( !tracker.act_pos_vert && !( tracker.stat & EXT_TERM_VCLOSE ) )
        return; //актуатор находится в открытом состоянии, но значение положения = "0"
    if ( !tracker.act_pos_horz && !( tracker.stat & EXT_TERM_HCLOSE ) )
        return; //актуатор находится в открытом состоянии, но значение положения = "0"
    sprintf( name, "\\trc\\trc_position.data" );
    trc_save = fopen( name, "w" );
    if ( trc_save == NULL )
        return; //файл не открылся
    fprintf( trc_save, "%s %03d %03d\r\n", RTCGetLog(), tracker.act_pos_vert, tracker.act_pos_horz ); 
    fclose( trc_save );
 }

//*************************************************************************************************
// Логирование состояние трекера в файл: trc_YYYYMMDD.log
//*************************************************************************************************
static void TrcLogging( void ) {

    char name[64];
    FILE *trc_log;

    if ( !config.log_enable_trc )
        return; //логирование выключено
    if ( SDStatus() == ERROR )
        return; //карты нет
    if ( !tracker.link ) 
        return; //трекер не подключен по интерфейсу RS-485
    if ( !config.mode_logging )
        sprintf( name, "\\trc\\trc_%s.log", RTCFileName() );
    else sprintf( name, "\\trc\\%s\\trc_%s.log", RTCFileShort(), RTCFileName() );
    //пишем в конец файла
    trc_log = fopen( name, "a" );
    if ( trc_log == NULL )
        return; //файл не открылся
    fprintf( trc_log, "%s STAT: 0x%04X %s VER=%03d/%.1f° HRZ=%03d/%.1f°\r\n", RTCGetLog(), tracker.stat, 
        ParamGetDesc( ID_DEV_TRC, TRC_MODE ), tracker.act_pos_vert, 
        AngleVert( tracker.act_pos_vert ), tracker.act_pos_horz, AngleHorz( tracker.act_pos_horz ) ); 
    fclose( trc_log );
 }

//*************************************************************************************************
// Логирование команд управления трекером в файл: trc_YYYYMMDD.log
// MBUS_REQUEST *reqst - cтруктура передачи запроса по MODBUS 
//*************************************************************************************************
static void TrcCmdLog( MBUS_REQUEST *reqst ) {

    uint8_t i;
    uint16_t *reg;
    char name[64];
    FILE *trc_log;

    if ( !config.log_enable_trc )
        return; //логирование выключено
    if ( SDStatus() == ERROR )
        return; //карты нет
    if ( !config.mode_logging )
        sprintf( name, "\\trc\\trc_%s.log", RTCFileName() );
    else sprintf( name, "\\trc\\%s\\trc_%s.log", RTCFileShort(), RTCFileName() );
    //пишем в конец файла
    trc_log = fopen( name, "a" );
    if ( trc_log == NULL )
        return; //файл не открылся
    fprintf( trc_log, "%s ", RTCGetLog() );
    fprintf( trc_log, "ADDR=0x%04X REGS=0x%04X DATA=", reqst->addr_reg, reqst->cnt_reg );
    reg = (uint16_t *)reqst->ptr_data;
    for ( i = 0; i < reqst->cnt_reg; i++, reg++ )
        fprintf( trc_log, "0x%04X ", *reg );
    fprintf( trc_log, "\r\n" ); 
    fclose( trc_log );
 }

//*************************************************************************************************
// Ведение протокола управления режимами в файл: trc_YYYYMMDD.log
// char *text - текстовая строка добавляемая в файл протокола
//*************************************************************************************************
static void EventLog( char *text, EventType evn ) {

    char name[64];
    FILE *trc_log;

    if ( !config.log_enable_trc )
        return; //логирование выключено
    if ( SDStatus() == ERROR )
        return; //карты нет
    //определение имени файла
    if ( !config.mode_logging )
        sprintf( name, "\\trc\\trc_%s.log", RTCFileName() );
    else sprintf( name, "\\trc\\%s\\trc_%s.log", RTCFileShort(), RTCFileName() );
    trc_log = fopen( name, "a" );
    if ( trc_log == NULL )
        return;
    //запись строки
    if ( evn == EVENT_LOG )
        fprintf( trc_log, "%s EVENT: %s\r\n", RTCGetLog(), text );
    if ( evn == ERROR_LOG )
        fprintf( trc_log, "%s ERROR: %s\r\n", RTCGetLog(), text );
    fclose( trc_log );
 }
