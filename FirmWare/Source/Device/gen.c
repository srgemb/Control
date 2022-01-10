
//*************************************************************************************************
//
// Управление генератором FUBAG TI-3003
// Подключение локальное (непосредственно к контроллеру) или удаленное по RS-485 (MODBUS)
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

#include "main.h"
#include "gen.h"
#include "rtc.h"
#include "alt.h"
#include "can_def.h"
#include "outinfo.h"
#include "command.h"
#include "message.h"
#include "sdcard.h"
#include "ports.h"
#include "rs485.h"
#include "eeprom.h"
#include "informing.h"
#include "dev_data.h"
#include "modbus.h"
#include "modbus_def.h"
#include "gen_ext.h"
#include "events.h"

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osEventFlagsId_t gen_event = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define TIMER_NO_WAIT           0           //таймер ожидания не включаем
#define TIMER_EXEC_PROC         1           //таймер задержки вызова GenStartProc() из GenStatusCheck()
#define DELAY_START             3           //задержка перед первым запуском (сек)
#define DELAY_CHECK_RUN         4           //ожидание сигнала запуска генератора (сек) 
#define DELAY_CHECK_CON         2           //ожидание после включения схемы контроля генератора (сек) 
#define TIME_OFF_AFTER_WORK     300         //время отключение схемы контроля генератора после завершения работы (сек)
#define TIME_GEN_LINK_OK        5000        //время обнуления данных при отсутствии связи по MODBUS с генератором (мсек)

#define TIMER_SEC_MAX           65535       //максимальное значение инкрементного секундного таймера
#define TIMER_INC_START         0
#define TIMER_INC_STOP          TIMER_SEC_MAX
#define TIMER_DEC_STOP          0

#define TIMEOUT_WAIT_EVENT_GEN  500         //время ожидания события (msec)

//Маски выборки для генератора подключенного удаленно по MODBUS
#define GEN_RMT_ONLY_MODE       0x000F      //маска для выборки режима работы
#define GEN_RMT_ONLY_STAT       0x000F      //маска для выборки режима состояния
#define GEN_RMT_CYCLE1          0xF000      //текущая попытка запуск
#define GEN_RMT_CYCLE2          0x0F00      //общее кол-во попыток запуска


//циклограмма запуска генератора
typedef enum {
    GEN_STEP_STOP,                          //ничего не делаем
    GEN_STEP_NEXT,                          //переход на следующий шаг
    GEN_STEP_BEGIN,                         //начало циклограммы, проверка подключения и отсутствие ошибок
    GEN_STEP_ON,                            //включение генератора
    GEN_STEP_STARTER_ON,                    //запуск стартера
    GEN_STEP_STARTER_OFF,                   //ожидание отсчета таймера работы стартера
    GEN_STEP_CHECK_RUN                      //проверка запуска генератор
 } GenStartStep;

//источник запуска/выключения генератора
typedef enum {
    GEN_CMD_STOP,                           //ручная остановка генератора
    GEN_CMD_START,                          //ручной запуск генератора
    GEN_CMD_AC_LOST,                        //основная сеть отключена
    GEN_CMD_SLEEP_END,                      //запуск генератора после отдыха
    GEN_CMD_TEST_START,                     //запуск генератора для тестирования
    GEN_CMD_CHECK_STOP,                     //генератор уже выключен, сбросим флаги, таймеры и т.д.
    GEN_CMD_TIME_STOP,                      //выключение из режима "тест", "ручной" по окончании времени работы
    GEN_CMD_AC_REST,                        //сеть восстановлена
    GEN_CMD_TEST_STOP                       //выключение тестового режима
 } GenCmdSrc;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
GEN gen_loc, gen_rmt, *gen_ptr = NULL; 
static bool gen_run = false, flg_event = false; 
static bool timer_lost_chk = false, timer_rest_chk = false;
static uint8_t last_start = 0, prev_auto = 0;
static uint8_t cycle_run = 0, auto_once = 0;
static uint16_t rmt_mode = 0, rmt_stat = 0, rmt_cycle1 = 0;
static GenStartStep step_start = GEN_STEP_STOP;
static osTimerId_t timer1, timer2;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t gen_attr = {
    .name = "Generator", 
    .stack_size = 1024,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn_attr = { .name = "Gen" };
static const osTimerAttr_t timer1_attr = { .name = "GenLocalOff" };
static const osTimerAttr_t timer2_attr = { .name = "Gen2" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void GenStartInit( GenCmdSrc source );
static void GenStopPrc( GenCmdSrc source );
static void TimerCount( void );
static Status GenStatus( uint32_t status );
static void GenRemoteStat( void );
static void GenStartProc( void );
static void GenWorkSleep( void );
static void GenCheckTimer( void );
static void GenStatusCheck( void );
static void GenAcLost( void );
static void GenAcRest( void );
static void NextStep( GenStartStep step, uint32_t time );
static void SaveDate( void );
static void EventLog( void );
static void ConsoleLog( void );
static void Timer1Callback( void *arg );
static void Timer2Callback( void *arg );
static void TaskGen( void *pvParameters );

//*************************************************************************************************
// Инициализация портов управления
//*************************************************************************************************
void GenInit( void ) {

    gen_ptr = &gen_loc;
    GPIO_SetDir( GEN_PORT, GEN_CHK,   GPIO_DIR_OUTPUT );
    GPIO_SetDir( GEN_PORT, GEN_ON,    GPIO_DIR_OUTPUT );
    GPIO_SetDir( GEN_PORT, GEN_START, GPIO_DIR_OUTPUT );

    GPIO_PinWrite( GEN_PORT, GEN_CHK,   RELAY_OFF );
    GPIO_PinWrite( GEN_PORT, GEN_ON,    RELAY_OFF );
    GPIO_PinWrite( GEN_PORT, GEN_START, RELAY_OFF );

    gen_loc.mode = GEN_MODE_NULL;
    gen_loc.stat = GEN_STAT_NULL;
    gen_loc.cycle1 = 0;
    gen_loc.cycle2 = config.gen_cnt_start;

    gen_loc.timer_run_inc = TIMER_INC_STOP;     //продолжительность работы
    gen_loc.timer_run_dec = TIMER_DEC_STOP;     //остаток времени работы генератора
    gen_loc.timer_sleep = TIMER_DEC_STOP;       //таймер отдыха генератора
    gen_loc.timer_lost_acmain = TIMER_DEC_STOP; //таймер задержки включения генератора после отключения сети
    gen_loc.timer_rest_acmain = TIMER_DEC_STOP; //таймер задержки отключения генератора после восстановления сети

    //включим схему контроля генератора
    GPIO_PinWrite( GEN_PORT, GEN_CHK, RELAY_ON );
    //очередь сообщений
    gen_event = osEventFlagsNew( &evn_attr );
    timer1 = osTimerNew( Timer1Callback, osTimerOnce, NULL, &timer1_attr );
    timer2 = osTimerNew( Timer2Callback, osTimerOnce, NULL, &timer2_attr );
    //создаем задачу
    osThreadNew( TaskGen, NULL, &gen_attr );
 }

//*************************************************************************************************
// Задача управления контроллером трекера
//*************************************************************************************************
static void TaskGen( void *pvParameters ) {
    
    int32_t event;
    uint32_t send, msg;
    
    for ( ;; ) {
        //запуск таймера схемы контроля генератора
        if ( !osTimerIsRunning( timer1 ) )
            osTimerStart( timer1, TIME_OFF_AFTER_WORK * SEC_TO_TICK );
        //ждем события, не более TIMEOUT_WAIT_EVENT_GEN
        event = osEventFlagsWait( gen_event, EVN_GEN_MASK, osFlagsWaitAny, TIMEOUT_WAIT_EVENT_GEN );
        if ( event == osErrorTimeout ) {
            GenRemoteStat();   //проверка подключения генератора по MODBUS
            send = ID_DEV_GEN; //передача данных в HMI
            osMessageQueuePut( hmi_msg, &send, 0, 0 );
           }
        else {
            GenStatusCheck(); //проверка состояния генератора
            GenWorkSleep();   //управление цикличностью работы генератора
            GenCheckTimer();  //контроль таймеров
            //обработка сообщений
            if ( event & EVN_RTC_SECONDS )
                TimerCount(); //отсчет секундных таймеров
            if ( event & EVN_GEN_CHECK_OFF ) {
                GPIO_PinWrite( GEN_PORT, GEN_CHK, RELAY_OFF );
                //выключаем схему контроля
                gen_run = false;
                GPIO_PinWrite( GEN_PORT, GEN_CHK, RELAY_OFF );
                if ( !gen_loc.timer_sleep )
                    gen_loc.mode = GEN_MODE_NULL; //таймер отдыха выключен, сбросим режим
               }
            if ( event & EVN_GEN_CYCLE_NEXT )
                GenStartProc(); //циклограмма запуска генератора
            if ( event & EVN_ALT_AC_LOST )
                GenAcLost();    //основная сеть отключена
            if ( event & EVN_ALT_AC_REST )
                GenAcRest();    //основная сеть восстановлена
            if ( event & EVN_GEN_LOG ) {
                //передача данных в HMI (актуализация состояния генератора)
                send = ID_DEV_GEN;
                osMessageQueuePut( hmi_msg, &send, 0, 0 );
                //передача события в HMI о изменении состояния генератора
                msg = CAN_DEV_ID( ID_DEV_LOG ) | CAN_PARAM_ID( ID_DEV_GEN );
                osMessageQueuePut( hmi_msg, &msg, 0, 0 );
                //логирование состояния генератора
                EventLog();
               }
            if ( event & EVN_GEN_CONSOLE )
                ConsoleLog();   //вывод результата запуска генератора в консоль
           }
       }
 }

//*************************************************************************************************
// Функция обратного вызова таймера - отключение схемы контроля генератора
//*************************************************************************************************
static void Timer1Callback( void *arg ) {

    osEventFlagsSet( gen_event, EVN_GEN_CHECK_OFF );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - переход к следующему шагу циклограммы
//*************************************************************************************************
static void Timer2Callback( void *arg ) {

    osEventFlagsSet( gen_event, EVN_GEN_CYCLE_NEXT );
 }

//*************************************************************************************************
// Запуск генератора
//*************************************************************************************************
void GenStart( void ) {

    uint8_t cnt_rpt;
    ModBusError status;
    uint16_t regs_data = EXGEN_CMD_START;
    uint16_t len = sizeof( regs_data );
    MBUS_REQUEST reqst = { MB_ID_DEV_GEN, FUNC_WR_MULT_REG, EXGEN_REG_WR_CMD, 1, &regs_data, &len };
    
    if ( gen_rmt.remote ) {
        //запуск генератора (контроллер подключен по MODBUS)
        cnt_rpt = CNT_REPEAT_REQST;
        do {
            len = sizeof( regs_data );
            status = ModBusRequest( &reqst );
           } while ( cnt_rpt-- && ( status == MBUS_ANSWER_CRC || status == MBUS_ANSWER_TIMEOUT ) );
        return;
       }
    if ( step_start != GEN_STEP_STOP )
        return; //запуск уже начат
    if ( gen_loc.timer_sleep )
        return; //запуск блокирован, генератор отдыхает
    GenStartInit( GEN_CMD_START ); 
 }

//*************************************************************************************************
// Тестовый режим запуска: запуск - N минут работы - выкл
//*************************************************************************************************
void GenTest( void ) {

    uint8_t cnt_rpt;
    ModBusError status;
    uint16_t regt_data = EXGEN_CMD_TEST;
    uint16_t len = sizeof( regt_data );
    MBUS_REQUEST reqst = { MB_ID_DEV_GEN, FUNC_WR_MULT_REG, EXGEN_REG_WR_CMD, 1, &regt_data, &len };
    
    if ( gen_rmt.remote ) {
        //тестовый запуск генератора (контроллер подключен по MODBUS)
        cnt_rpt = CNT_REPEAT_REQST;
        do {
            len = sizeof( regt_data );
            status = ModBusRequest( &reqst );
           } while ( cnt_rpt-- && ( status == MBUS_ANSWER_CRC || status == MBUS_ANSWER_TIMEOUT ) );
        return;
       }
    if ( step_start != GEN_STEP_STOP )
        return; //запуск уже выполнен
    if ( gen_loc.timer_sleep )
        return; //запуск блокирован, генератор отдыхает
    GenStartInit( GEN_CMD_TEST_START ); //запускаем
 }

//*************************************************************************************************
// Останов генератора
//*************************************************************************************************
void GenStop( void ) {

    uint8_t cnt_rpt;
    ModBusError status;
    uint16_t regp_data = EXGEN_CMD_STOP;
    uint16_t len = sizeof( regp_data );
    MBUS_REQUEST reqst = { MB_ID_DEV_GEN, FUNC_WR_MULT_REG, EXGEN_REG_WR_CMD, 1, &regp_data, &len };
    
    if ( gen_rmt.remote ) {
        //выключение генератора (контроллер подключен по MODBUS)
        cnt_rpt = CNT_REPEAT_REQST;
        do {
            len = sizeof( regp_data );
            status = ModBusRequest( &reqst );
           } while ( cnt_rpt-- && ( status == MBUS_ANSWER_CRC || status == MBUS_ANSWER_TIMEOUT ) );
        return;
       }
    auto_once = 0;
    GenStopPrc( GEN_CMD_STOP );
 }

//*************************************************************************************************
// Чтение состояние генератора подключенного по MODBUS
// Вызов из TaskGen() с интервалом 500 ms
// регистр EXGEN_REG_RD_MODE - режим работы
//                    0x000X - режим
//                    0x0X00 - источник сброса
//                    0x8000 - автоматический режим
// регистр EXGEN_REG_RD_STAT - текущее состояние режима (в т.ч. ошибки)
//                    0x00XX - статус
//                    0x0X00 - кол-во циклов запуска
//                    0xX000 - номер цикла запуска
//*************************************************************************************************
static void GenRemoteStat( void ) {

    char *ptr_mode, *ptr_stat;
    uint8_t cnt_rpt;
    ModBusError status;
    uint16_t gen_data[EXGEN_REG_RD_MAX_BASE];
    uint16_t len = sizeof( gen_data );
    MBUS_REQUEST reqst = { MB_ID_DEV_GEN, FUNC_RD_HOLD_REG, EXGEN_REG_RD_MODE, EXGEN_REG_RD_MAX_BASE, &gen_data, &len };
    
    cnt_rpt = CNT_REPEAT_REQST;
    do {
        len = sizeof( gen_data );
        status = ModBusRequest( &reqst );
       } while ( cnt_rpt-- && ( status == MBUS_ANSWER_CRC || status == MBUS_ANSWER_TIMEOUT ) );
    if ( status != MBUS_ANSWER_OK ) {
        //генератор не подключен по MODBUS шине - подключен локально
        //источник данных для передачи по CAN шине берем из "gen_loc"
        gen_ptr = &gen_loc;
       }
    else {
        //генератор подключен по MODBUS шине
        //источник данных для передачи по CAN шине берем из "gen_rmt"
        gen_ptr = &gen_rmt;
        gen_rmt.remote = LINK_CONN_OK;
        gen_rmt.mode = (GenMode)( gen_data[EXGEN_REG_RD_MODE] & GEN_RMT_ONLY_MODE );
        gen_rmt.stat = (GenStat)( gen_data[EXGEN_REG_RD_STAT] & GEN_RMT_ONLY_STAT );
        gen_rmt.error = (GenError)gen_data[EXGEN_REG_RD_ERROR];
        gen_rmt.auto_mode = gen_data[EXGEN_REG_RD_MODE] & EXGEN_MODE_AUTO ? GEN_AUTO : GEN_MANUAL;
        gen_rmt.timer_run_inc = gen_data[EXGEN_REG_RD_RUN];
        gen_rmt.timer_run_dec = gen_data[EXGEN_REG_RD_RUND];
        gen_rmt.timer_lost_acmain = gen_data[EXGEN_REG_RD_LOST];
        gen_rmt.timer_rest_acmain = gen_data[EXGEN_REG_RD_REST];
        gen_rmt.timer_sleep = gen_data[EXGEN_REG_RD_SLEEP];
        gen_rmt.cycle1 = ( gen_data[EXGEN_REG_RD_STAT] & GEN_RMT_CYCLE1 ) >> 12;
        gen_rmt.cycle2 = ( gen_data[EXGEN_REG_RD_STAT] & GEN_RMT_CYCLE2 ) >> 8;
        ptr_mode = ParamGetDesc( ID_DEV_GEN, GEN_PAR_MODE );
        ptr_stat = ParamGetDesc( ID_DEV_GEN, GEN_PAR_STAT );
        if ( rmt_mode && rmt_stat )
            flg_event = true;
        if ( ptr_mode != NULL && ptr_stat != NULL && ( gen_rmt.mode != rmt_mode || gen_rmt.stat != rmt_stat || gen_rmt.cycle1 != rmt_cycle1 ) ) {
            //изменение состояния удаленного контроллера
            rmt_mode = gen_rmt.mode;
            rmt_stat = gen_rmt.stat;
            rmt_cycle1 = gen_rmt.cycle1;
            //отправим сообщение в HMI
            if ( flg_event )
                osEventFlagsSet( gen_event, EVN_GEN_LOG );
           }
       }
 }

//*************************************************************************************************
// Инициализация запуска генератора
// GenCmdSrc source - источник запуска генератора 
//*************************************************************************************************
static void GenStartInit( GenCmdSrc source ) {

    gen_loc.error = GEN_ERR_OK; //сброс кодов ошибок
    auto_once = 0;
    gen_loc.stat = GEN_STAT_NULL;
    gen_loc.mode = GEN_MODE_START;
    last_start = source;
    //установить время отключения схемы контроля
    osTimerStart( timer1, TIME_OFF_AFTER_WORK * SEC_TO_TICK );
    //включаем схему контроля генератора
    GPIO_PinWrite( GEN_PORT, GEN_CHK, RELAY_ON );
    //запускаем циклограмму запуска генератора
    NextStep( GEN_STEP_BEGIN, DELAY_CHECK_CON );
    gen_run = false;  //флаг работы генератора
    cycle_run = 0 ; //блокируем проверку таймера отдыха
    gen_loc.timer_sleep = TIMER_DEC_STOP;
    //в случае, если запуск выполнен принудительно до окончания 
    //времени таймера задержки запуска, выключим таймер 
    gen_loc.timer_lost_acmain = TIMER_DEC_STOP;
    osEventFlagsSet( gen_event, EVN_GEN_LOG );
 }

//*************************************************************************************************
// Перевод в режим "генератор выключен"
// GenCmdSrc source - код источника остановки генератора
//*************************************************************************************************
static void GenStopPrc( GenCmdSrc source ) {

    uint8_t cnt_rpt;
    ModBusError status;
    uint32_t prev_stat, prev_relay;
    uint16_t reg_data = EXGEN_CMD_STOP;
    uint16_t len = sizeof( reg_data );
    MBUS_REQUEST reqst = { MB_ID_DEV_GEN, FUNC_WR_MULT_REG, EXGEN_REG_WR_CMD, 1, &reg_data, &len };
    
    if ( gen_rmt.remote ) {
        //выключение генератора (контроллер подключен по MODBUS)
        cnt_rpt = CNT_REPEAT_REQST;
        do {
            len = sizeof( reg_data );
            status = ModBusRequest( &reqst );
           } while ( cnt_rpt-- && ( status == MBUS_ANSWER_CRC || status == MBUS_ANSWER_TIMEOUT ) );
        Informing( VOICE_GEN_OFF, NULL );
        osEventFlagsSet( gen_event, EVN_GEN_LOG );
        return;
       }
    gen_loc.mode = GEN_MODE_OFF;
    //сохраним статус генератора до процедуры выключения
    prev_stat = GetDataPort( GEN_ALL_STAT | ALT_ALL_STAT );
    //выключаем процедуру запуска 
    step_start = GEN_STEP_STOP;
    //состояние реле до выключения GEN_ON_CHK
    prev_relay = GPIO_PinRead( GEN_PORT, GEN_ON );
    GPIO_PinWrite( GEN_PORT, GEN_ON, RELAY_OFF );
    GPIO_PinWrite( GEN_PORT, GEN_START, RELAY_OFF );
    //остановим таймеры продолжительности работы
    gen_loc.timer_run_inc = TIMER_INC_STOP;
    gen_loc.timer_run_dec = TIMER_DEC_STOP;
    //остановим таймер отдыха, задержки выключения
    gen_loc.timer_sleep = TIMER_DEC_STOP;
    gen_loc.timer_rest_acmain = TIMER_DEC_STOP;
    //фиксируем статус при прерывании автозапуска в режиме "отдых"
    if ( source == GEN_CMD_STOP && cycle_run ) {
        //выключаем автозапуск после отдыха 
        cycle_run = 0;
        gen_loc.stat = GEN_STAT_AUTO_BREAK;
        gen_loc.error = GEN_ERR_OK;
        osEventFlagsSet( gen_event, EVN_GEN_LOG );
        gen_loc.mode = GEN_MODE_NULL;
       }
    //фиксируем статус при прерывании запуска
    if ( GetDataPort( GEN_CONN ) && source == GEN_CMD_STOP && step_start && prev_relay ) {
        gen_loc.stat = GEN_STAT_START_BREAK;
        gen_loc.error = GEN_ERR_OK;
        osEventFlagsSet( gen_event, EVN_GEN_LOG );
       }
    //фиксируем статус при завершении тестового запуска
    if ( GetDataPort( GEN_CONN ) && source == GEN_CMD_TIME_STOP && last_start == GEN_CMD_TEST_START && prev_relay ) {
        gen_loc.stat = GEN_STAT_TEST_END;
        gen_loc.error = GEN_ERR_OK;
        osEventFlagsSet( gen_event, EVN_GEN_LOG );
       } 
    //фиксируем статус при штатном выключении генератора
    if ( GetDataPort( GEN_CONN ) && source == GEN_CMD_STOP && prev_relay ) {
        gen_loc.stat = GEN_STAT_MANUAL_BREAK;
        gen_loc.error = GEN_ERR_OK;
        osEventFlagsSet( gen_event, EVN_GEN_LOG );
       } 
    //фиксируем статус при выключении генератора по окончанию времени работы
    if ( GetDataPort( GEN_CONN ) && ( prev_stat | GEN_RUN | ALT_GEN_OK ) && source == GEN_CMD_TIME_STOP && last_start != GEN_CMD_TEST_START ) {
        gen_loc.mode = GEN_MODE_SLEEP;
        gen_loc.stat = GEN_STAT_TIME_OUT;
        gen_loc.error = GEN_ERR_OK;
        osEventFlagsSet( gen_event, EVN_GEN_LOG );
       }
    Informing( VOICE_GEN_OFF, NULL );
 }

//*************************************************************************************************
// Управление запуском генератора (выполнение циклограммы)
//*************************************************************************************************
void GenStartProc( void ) {

    //проверка подключения генератора
    if ( step_start == GEN_STEP_BEGIN ) {
        if ( GenStatus( GetDataPort( GEN_ALL_STAT ) ) ) {
            //генератор подключен, ошибок нет, идем дальше
            NextStep( GEN_STEP_NEXT, TIMER_NO_WAIT );
           }
        else {
            //есть ошибки
            osEventFlagsSet( gen_event, EVN_GEN_LOG );
            NextStep( GEN_STEP_STOP, TIMER_NO_WAIT );
            gen_loc.mode = GEN_MODE_NULL;
            //вывод результата в консоль
            osEventFlagsSet( gen_event, EVN_GEN_CONSOLE );
            return;
           }
       } 
    //включаем генератор
    if ( step_start == GEN_STEP_ON ) {
        gen_loc.cycle1 = 0;
        GPIO_PinWrite( GEN_PORT, GEN_ON, RELAY_ON );
        //запускаем таймер, пока типа бензин протечет
        NextStep( GEN_STEP_NEXT, DELAY_START );
        gen_loc.stat = GEN_STAT_STEP_START;
        return;
       }
    //запуск
    if ( step_start == GEN_STEP_STARTER_ON ) {
        osEventFlagsSet( gen_event, EVN_GEN_LOG );
        //проверка подключения и наличие ошибок
        if ( !GenStatus( GetDataPort( GEN_ALL_STAT ) ) ) {
            //генератор не подключен или есть ошибки
            //местами переменные не переставлять
            gen_loc.stat = GEN_STAT_START_ERROR;
            GPIO_PinWrite( GEN_PORT, GEN_ON, RELAY_OFF );
            osEventFlagsSet( gen_event, EVN_GEN_LOG );
            gen_loc.mode = GEN_MODE_NULL;
            NextStep( GEN_STEP_STOP, TIMER_NO_WAIT );
            //вывод результата в консоль
            osEventFlagsSet( gen_event, EVN_GEN_CONSOLE );
            return;
           }
        //включаем стартер
        GPIO_PinWrite( GEN_PORT, GEN_START, RELAY_ON );
        //запускаем таймер работы стартера
        NextStep( GEN_STEP_NEXT, config.gen_time_start[gen_loc.cycle1] );
        return;
       } 
    //ожидание отсчета таймера работы стартера
    if ( step_start == GEN_STEP_STARTER_OFF ) {
        //выключаем стартер
        GPIO_PinWrite( GEN_PORT, GEN_START, RELAY_OFF );
        //ожидание сигнала работы генератора
        NextStep( GEN_STEP_NEXT, config.gen_delay_chk_run );
        return;
       }
    //проверка запуска генератор
    if ( step_start == GEN_STEP_CHECK_RUN ) {
        //проверим сигнал запуска генератора
        if ( GetDataPort( GEN_RUN | ALT_GEN_OK ) ) {
            //генератор запустился
            gen_loc.mode = GEN_MODE_RUN;
            gen_loc.stat = GEN_STAT_NULL;
            //запускаем прямой отсчет продолжительности работы
            gen_loc.timer_run_inc = TIMER_INC_START;
            //запускаем обратный отсчет продолжительности работы
            if ( last_start == GEN_CMD_TEST_START )
                gen_loc.timer_run_dec = config.gen_time_test;
            else gen_loc.timer_run_dec = config.gen_time_run;
            SaveDate(); //сохраним дату включения генератора
            osEventFlagsSet( gen_event, EVN_GEN_LOG );
            Informing( VOICE_GEN_RUN, NULL );
            NextStep( GEN_STEP_STOP, TIMER_NO_WAIT );
            //вывод результата в консоль
            osEventFlagsSet( gen_event, EVN_GEN_CONSOLE );
            return;
           }              
        else {
            //время вышло, запуска нет, один цикл запуска завершился неудачей
            gen_loc.cycle1++;
            gen_loc.stat = GEN_STAT_STEP_START;
            //проверим предельное значение кол-ва запусков
            if ( gen_loc.cycle1 > ( config.gen_cnt_start - 1 ) ) {
                //все попытки запуска исчерпаны
                NextStep( GEN_STEP_STOP, TIMER_NO_WAIT );
                //обновим состояние генератора (ошибки)
                if ( GenStatus( GetDataPort( GEN_ALL_STAT ) ) ) {
                    //генератор подключен, но не запустился
                    gen_loc.error = GEN_ERR_START;
                    gen_loc.stat = GEN_STAT_START_ERROR;
                    Informing( VOICE_GEN_CHECK, NULL );
                   }
                GPIO_PinWrite( GEN_PORT, GEN_ON, RELAY_OFF );
                osEventFlagsSet( gen_event, EVN_GEN_LOG );
                gen_loc.mode = GEN_MODE_NULL;
                //вывод результата в консоль
                osEventFlagsSet( gen_event, EVN_GEN_CONSOLE );
                return;
               } 
            else {
                //отсчет завершен, не запустился, повторяем запуск
                NextStep( GEN_STEP_STARTER_ON, config.gen_before_start );
               } 
           } 
       }    
 }

//*************************************************************************************************
// Отслеживает состояние генератора: работу, отключение
// По таймеру отключает схему контроля. 
//*************************************************************************************************
static void GenStatusCheck( void ) {

    uint32_t prev_stat;

    gen_loc.cycle2 = config.gen_cnt_start;
    //проверяем только если: вкл.генератор + вкл.схема контроля
    if ( GPIO_PinRead( GEN_PORT, GEN_ON ) && GPIO_PinRead( GEN_PORT, GEN_CHK ) ) {
        //проверим, работает ли генератор: ген.подкл. + ген.работает 
        if ( GetDataPort( GEN_CONN ) && GetDataPort( GEN_RUN | ALT_GEN_OK ) ) {
            //генератор работает
            if ( gen_run == false ) {
                //при предыдущей проверке не работал
                gen_run = true;
                //выключим стартер
                GPIO_PinWrite( GEN_PORT, GEN_START, RELAY_OFF );
                //для завершения циклограммы запуска вызов GenStartProc()
                NextStep( GEN_STEP_CHECK_RUN, TIMER_EXEC_PROC );
                //запускаем прямой отсчет продолжительности работы
                gen_loc.timer_run_inc = TIMER_INC_START;
                //запускаем обратный отсчет продолжительности работы
                if ( last_start == GEN_CMD_TEST_START ) {
                    gen_loc.mode = GEN_MODE_TEST;
                    gen_loc.timer_run_dec = config.gen_time_test;
                   }
                else {
                    gen_loc.mode = GEN_MODE_RUN;
                    gen_loc.timer_run_dec = config.gen_time_run;
                   }
                gen_loc.stat = GEN_STAT_NULL;
                gen_loc.error = GEN_ERR_OK;
                SaveDate();
                osEventFlagsSet( gen_event, EVN_GEN_LOG );
              }
           } 
        else {
            //генератор не работает
            if ( gen_run == true ) {
                //при предыдущей проверке работал
                prev_stat = GetDataPort( GEN_ALL_STAT );
                //генератор уже выключен, сбросим флаги, таймеры и т.д.
                GenStopPrc( GEN_CMD_CHECK_STOP );
                gen_loc.mode = GEN_MODE_OFF;
                //проверим ошибки генератора
                if ( GenStatus( prev_stat ) ) {
                    //генератор подключен, ошибок нет, но выключился
                    gen_loc.error = GEN_ERR_BREAK;
                    gen_loc.stat = GEN_STAT_GEN_OFF;
                    Informing( VOICE_GEN_DISCON, NULL );
                   }
                osEventFlagsSet( gen_event, EVN_GEN_LOG );
               }                  
           }
       }  
    //генератор включен, перезапустим таймер отключения схемы контроля
    if ( GPIO_PinRead( GEN_PORT, GEN_ON ) )
        osTimerStart( timer1, TIME_OFF_AFTER_WORK * SEC_TO_TICK );
    //обновим данные в структуре GEN
    gen_loc.connect = GetDataPort( GEN_CONN ) ? LINK_CONN_OK : LINK_CONN_NO;
 }

//*************************************************************************************************
// Вывод в консоль результата запуска генератора 
//*************************************************************************************************
static void ConsoleLog( void ) {

    char msg[160];

    if ( gen_loc.error )
        sprintf( msg, "\r\nFUBAG TI-3003 %s\r\n", ErrorDescr( ID_DEV_GEN, gen_loc.error, 0 ) );
    else sprintf( msg, "\r\nFUBAG TI-3003 %s %s\r\n", ParamGetDesc( ID_DEV_GEN, GEN_PAR_MODE ), ParamGetDesc( ID_DEV_GEN, GEN_PAR_STAT ) );
    ConsoleSend( msg, CONS_SELECTED );
    ConsoleSend( Message( CONS_MSG_CRLF ), CONS_SELECTED );
    ConsoleSend( Message( CONS_MSG_PROMPT ), CONS_SELECTED );
 }

//*************************************************************************************************
// Контроль режима цикличности "работа-отдых"
// Выключение при окончании времени непрерывной работы и запуск по окончании времени отдыха
//*************************************************************************************************
void GenWorkSleep( void ) {

    //проверим, работает ли генератор: ген.вкл. + ген.подкл. + ген.работает
    //на время запуска не проверяем 
    if ( !step_start && GPIO_PinRead( GEN_PORT, GEN_ON ) && GetDataPort( GEN_CONN ) && GetDataPort( GEN_RUN | ALT_GEN_OK ) ) {
        //проверим остаток времени непрерывной работы
        if ( !gen_loc.timer_run_dec ) {
            //время работы закончилось, выключаем
            GenStopPrc( GEN_CMD_TIME_STOP );
            if ( last_start != GEN_CMD_TEST_START ) {
                //запустим таймер отдыха, кроме режима тестирования
                gen_loc.timer_sleep = config.gen_time_sleep;
                cycle_run = 1; //фиксируем шаг запуска таймера для последующих проверок  
               }
           } 
       }
    //проверим таймер отдыха
    if ( cycle_run == 1 && !gen_loc.timer_sleep ) {
        cycle_run = 0;
        //блокируем запуск генератора после отдыха в ручном режиме если был 
        //включен режим авто-выключение генератора при первом восстановлении сети
        if ( !config.gen_auto_mode && auto_once && GetDataPort( ALT_AC_MAIN ) && ( last_start == GEN_CMD_START || last_start == GEN_CMD_SLEEP_END ) ) {
            auto_once = 0;
            GenStopPrc( GEN_CMD_AC_REST );  //останавливаем
            return;
           }    
        //перезапуск генератора после отдыха при:
        //1. при выключенном автоматическом режиме и ручном запуске
        //инициируем следующий запуск как после отдыха
        if ( !config.gen_auto_mode && ( last_start == GEN_CMD_START || last_start == GEN_CMD_SLEEP_END ) ) {
            GenStartInit( GEN_CMD_SLEEP_END );
            return;
           }    
        //2. при включенном автоматическом режиме и отсутствии основной сети
        //следующий запуск после отдыха, при автоматическом режиме - всегда AC_LOST
        if ( config.gen_auto_mode && !GetDataPort( ALT_AC_MAIN ) && last_start == GEN_CMD_AC_LOST ) {
            GenStartInit( GEN_CMD_AC_LOST ); 
            return;
           }    
       } 
 }

//*************************************************************************************************
// Отслеживание состояния таймеров восстановления и отключения основной сети
//*************************************************************************************************
static void GenCheckTimer( void ) {

    //обработка выключения режима "автозапуск" при работающих таймерах: TIMER_LOST_ACMAIN, TIMER_REST_ACMAIN
    if ( prev_auto && !config.gen_auto_mode && ( gen_loc.timer_rest_acmain || gen_loc.timer_lost_acmain ) ) {
        prev_auto = 0; //блокируем повторный вход сюда
        //при переводе режима "автоматический" в "ручной" в момент 
        //отключении основной сети, выключаем таймер задержки запуска и блокируем запуск генератора.
        //таймер задержки выключения продолжает работать и выключит генератор
        timer_lost_chk = false;
        if ( gen_loc.timer_lost_acmain ) {
            //таймер запуска генератора работает, выключаем, генератор не запускаем
            gen_loc.timer_lost_acmain = TIMER_DEC_STOP;
           } 
       }
    //обработка таймера запуска генератора при отключении основной сети
    if ( config.gen_auto_mode && !gen_loc.timer_lost_acmain && timer_lost_chk == true ) {
        timer_lost_chk = false;
        GenStartInit( GEN_CMD_AC_LOST ); //запускаем генератор
       }
    //обработка таймеров выключения генератора при восстановлении основной сети
    //генератор выключается и при выключении режима "автоматический" после запуска 
    if ( !gen_loc.timer_rest_acmain && timer_rest_chk == true ) {
        timer_rest_chk = false;
        auto_once = 0; //авто-выключение генератора при первом восстановлении сети - выкл.
        GenStopPrc( GEN_CMD_AC_REST );  //выключаем генератор
       }
 }

//*************************************************************************************************
// Проверка подключения генератора и наличия ошибок 
// uint32_t status  - состояние входов генератора
// return = SUCCESS - генератор подключен, ошибок нет
//        = ERROR   - генератор не подключен или есть ошибки
// в gen_loc.stat       - содержит номер сообщения состояния
// в gen_loc.error      - содержит код ошибки
//*************************************************************************************************
static Status GenStatus( uint32_t status ) {

    if ( !( status & GEN_CONN ) ) {
        gen_loc.stat = GEN_STAT_NO_CONN;     //Не подключен
        gen_loc.error = GEN_ERR_NO_CONNECT;
        return ERROR;
       } 
    if ( status & GEN_LOW_BAT ) {
        gen_loc.stat = GEN_STAT_BAT_LOW;     //Низкий заряд батареи генератора
        gen_loc.error = GEN_ERR_BATLOW;
        return ERROR;
       }
    if ( ( status & GEN_FUEL_LOW ) == GEN_FUEL_LOW ) {
        gen_loc.stat = GEN_STAT_FUEL_LOW;    //Низкий уровень топлива
        gen_loc.error = GEN_ERR_FUEL;
        return ERROR;
       }
    if ( status & GEN_OIL ) {
        gen_loc.stat = GEN_STAT_OIL_LOW;     //Низкий уровень масла
        gen_loc.error = GEN_ERR_OIL;
        return ERROR;
       } 
    if ( status & GEN_OVR ) {
        gen_loc.stat = GEN_STAT_OVERLOAD;    //Перегрузка инвертора
        gen_loc.error = GEN_ERR_OVR;
        return ERROR;
       } 
    return SUCCESS; 
 }

//*************************************************************************************************
// Возвращает состояние выполнения циклограмма запуска генератора
// return = true  - циклограмма выполняется
//        = false - циклограмма не запущена
//*************************************************************************************************
bool GenCycleStart( void ) {

    if ( step_start != GEN_STEP_STOP )
        return true;
    else return false;
 }

//*************************************************************************************************
// Обработка сигнала отключения основной сети
//*************************************************************************************************
static void GenAcLost( void ) {

    if ( config.gen_auto_mode || auto_once || !gen_loc.timer_sleep ) {
        prev_auto = 1; //флаг изменения состояния основной сети
        // обработка только для автоматического режима и не во время отдыха генератора
        if ( !GetDataPort( GEN_RUN | ALT_GEN_OK ) ) {
            //генератор выключен, запустим таймер задержки включения генератора
            timer_lost_chk = true; //разрешение проверки таймера
            gen_loc.timer_lost_acmain = config.gen_delay_start;
           } 
        if ( GPIO_PinRead( GEN_PORT, GEN_ON ) && GetDataPort( GEN_RUN | ALT_GEN_OK ) ) {
            //генератор работает, остановим таймер задержки выключения генератора
            timer_rest_chk = false;
            gen_loc.timer_rest_acmain = TIMER_DEC_STOP;
           }
       }  
 }

//*************************************************************************************************
// Обработка сигнала восстановления основной сети
//*************************************************************************************************
static void GenAcRest( void ) {

    if ( config.gen_auto_mode || auto_once || !gen_loc.timer_sleep ) {
        prev_auto = 1; //флаг изменения состояния основной сети
        // обработка только для автоматического режима и не во время отдыха генератора
        if ( !GetDataPort( GEN_RUN | ALT_GEN_OK ) ) {
            //генератор не работает, выключаем таймер задержки запуска генератора
            timer_lost_chk = false;
            gen_loc.timer_lost_acmain = TIMER_DEC_STOP;
           } 
        if ( GPIO_PinRead( GEN_PORT, GEN_ON ) && GetDataPort( GEN_RUN | ALT_GEN_OK ) ) {
            //генератор работает, запустим таймер задержки выключения генератора
            timer_rest_chk = true;
            gen_loc.timer_rest_acmain = config.gen_delay_stop;
           }
       }  
 }

//*************************************************************************************************
// Разрешим авто-выключение генератора при первом восстановлении сети
//*************************************************************************************************
void GenAutoOnce( void ) {

    auto_once = 1;
 }

//*************************************************************************************************
// Логирование команд и состояния генератора в протокол
//*************************************************************************************************
static void EventLog( void ) {

    FILE *log;
    char name[40], prnval[40];
    
    if ( !config.log_enable_gen )
        return; //логирование выключено
    if ( SDStatus() == ERROR )
        return; //карты нет
    if ( !config.mode_logging )
        sprintf( name, "\\gen\\gen_%s.log", RTCFileName() );
    else sprintf( name, "\\gen\\%s\\gen_%s.log", RTCFileShort(), RTCFileName() );
    log = fopen( name, "a" );
    if ( log == NULL )
        return;
    if ( gen_ptr->stat == GEN_STAT_STEP_START ) //только для одного параметра подставляем значения
        sprintf( prnval, ParamGetDesc( ID_DEV_GEN, GEN_PAR_STAT ), gen_ptr->cycle1 + 1, gen_ptr->cycle2 ); 
    else sprintf( prnval, "%s", ParamGetDesc( ID_DEV_GEN, GEN_PAR_STAT ) );
    if ( gen_loc.error )
        fprintf( log, "%s %s %s\r\n", RTCGetLog(), ParamGetDesc( ID_DEV_GEN, GEN_PAR_MODE ), ErrorDescr( ID_DEV_GEN, gen_ptr->error, 0 ) );
    else fprintf( log, "%s %s %s\r\n", RTCGetLog(), ParamGetDesc( ID_DEV_GEN, GEN_PAR_MODE ), prnval );
    fclose( log );
 }

//*************************************************************************************************
// Сохраним дату запуска генератора
//*************************************************************************************************
static void SaveDate( void ) {

    static RTC_TIME_Type Time;
    
    RTC_GetFullTime( LPC_RTC, &Time );
    config.gen_last_run.day = Time.DOM;
    config.gen_last_run.month = Time.MONTH;
    config.gen_last_run.year = Time.YEAR;
    ConfigSave();
 }

//*************************************************************************************************
// Формирует следующий шаг циклограммы запуска генератора с заданной задержкой
// GenStepStart step - тип шага выполнения циклограммы
//                     при step = GEN_STEP_NEXT - выполняется авто инкремент значения шага
// uint32_t time     - время задержки выполнения в секундах, таймер запускается 
//                     только для значений > 0
//*************************************************************************************************
static void NextStep( GenStartStep step, uint32_t time ) {

    if ( step == GEN_STEP_STOP ) {
        //останавливаем цикл запуска
        step_start = step;
        osTimerStop( timer2 );
        return;
       }
    if ( step == GEN_STEP_NEXT ) {
        //следующий шаг формируется инкрементом
        if ( step_start < GEN_STEP_CHECK_RUN ) {
            //следующий шаг циклограммы
            step_start++;
            if ( time )
                osTimerStart( timer2, time * SEC_TO_TICK );
           }
        else {
            //останавливаем цикл запуска
            step_start = GEN_STEP_STOP;
            osTimerStop( timer2 );
           }
       }
    else {
        //следующий шаг указан в явном виде
        step_start = step;
        if ( time )
            osTimerStart( timer2, time * SEC_TO_TICK );
       }
 }

//*************************************************************************************************
// Отсчет секундных таймеров
//*************************************************************************************************
static void TimerCount( void ) {

    //отсчет, инкрементный таймер
    //продолжительность работы
    if ( gen_loc.timer_run_inc < TIMER_SEC_MAX )
        gen_loc.timer_run_inc++;
    //отсчет, декрементные таймеры
    //остаток времени работы генератора
    if ( gen_loc.timer_run_dec )
        gen_loc.timer_run_dec--;
    //таймер отдыха генератора
    if ( gen_loc.timer_sleep )
        gen_loc.timer_sleep--;
    //таймер задержки включения генератора после отключения сети
    if ( gen_loc.timer_lost_acmain )
        gen_loc.timer_lost_acmain--;
    //таймер задержки отключения генератора после восстановления сети
    if ( gen_loc.timer_rest_acmain )
        gen_loc.timer_rest_acmain--;
 }
