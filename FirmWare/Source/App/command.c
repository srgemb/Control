
//*************************************************************************************************
//
// Обмен данными с консолью (терминальной программой)
// Обработка введенных команд
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "rl_fs.h"
#include "cmsis_os2.h"

#include "uart_lpc17xx.h"
#include "driver_usart.h"
#include "lpc177x_8x_eeprom.h"
#include "lpc177x_8x_clkpwr.h"

#include "device.h"
#include "dev_param.h"

#include "version.h"
#include "vt100.h"
#include "rtc.h"
#include "command.h"
#include "ports.h"
#include "uart.h"
#include "charger.h"
#include "batmon.h"
#include "reserv.h"
#include "eeprom.h"
#include "outinfo.h"
#include "mppt.h"
#include "inverter.h"
#include "pv.h"
#include "alt.h"
#include "rtc.h"
#include "spa_calc.h"
#include "rs485.h"
#include "sound.h"
#include "gen.h"
#include "scheduler.h"
#include "parse.h"
#include "modbus.h"
#include "modbus_def.h"
#include "sdcard.h"
#include "message.h"
#include "informing.h"
#include "tracker.h"
#include "tracker_ext.h"
#include "events.h"

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osEventFlagsId_t command_event = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define MB_MAX_DATA_REG         10          //максимальное кол-во 16 битных регистров для чтения/записи

#define CONS_RECV_BUFF          384         //размер приемного буфера
#define CONS_SEND_BUFF          8192        //размер передающего буфера

#define EXEC_JOBS_ENABLE        true        //разрешение выполнения команды из планировщика

typedef enum {
    LOG_NEW_STR,                            //логирование без добавления даты времени выполнения
    LOG_NEW_CMND                            //логирование новой команды, добавляется дата время выполнения
 } LogModeCmd;

//структура хранения перечня команд
typedef struct {
    char    name_cmd[20];                           //имя команды
    void    (*func)( uint8_t cnt_par, Source src ); //указатель на функцию выполнения
    bool    exec_job;                               //разрешено выполнять из планировщика
} COMMAND;

//расшифровка статуса задач
static char * const state_name[] = {
    "Inactive",
    "Ready",
    "Running",
    "Blocked",
    "Terminated",
    "Error"
 };

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static Source cmd_src;
FILE *cmd_file, *stream = NULL;
static char job_buffer[CONS_RECV_BUFF];
static osMutexId_t mutex_param;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t console_attr = {
    .name = "Command",
    .stack_size = 2048,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn_attr = { .name = "Command" };
static const osMutexAttr_t mutex1_attr = { .name = "Param", .attr_bits = osMutexPrioInherit };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void FileClose( void );
static void ExecuteCmd( void );
static void TaskConsole( void *pvParameters );
static void ExecLog( char *str, LogModeCmd mode );
static char *Version( uint32_t version, char *str );
static char *TaskStateDesc( osThreadState_t state );

static void CmdHelp( uint8_t cnt_par, Source src );
static void CmdTime( uint8_t cnt_par, Source src );
static void CmdDate( uint8_t cnt_par, Source src );
static void CmdDateTime( uint8_t cnt_par, Source src );
static void CmdStatAll( uint8_t cnt_par, Source src );
static void CmdEeprom( uint8_t cnt_par, Source src );
static void CmdConfig( uint8_t cnt_par, Source src );
static void CmdReset( uint8_t cnt_par, Source src );
static void CmdCls( uint8_t cnt_par, Source src );
static void CmdSystem( uint8_t cnt_par, Source src );
static void CmdScr( uint8_t cnt_par, Source src );

static void CmdCharge( uint8_t cnt_par, Source src );
static void CmdInvCtrl( uint8_t cnt_par, Source src );
static void CmdAlt( uint8_t cnt_par, Source src );
static void CmdBatMon( uint8_t cnt_par, Source src );
static void CmdPvMode( uint8_t cnt_par, Source src );
static void CmdTrcMode( uint8_t cnt_par, Source src );
static void CmdSpa( uint8_t cnt_par, Source src );
static void CmdReserv( uint8_t cnt_par, Source src );
static void CmdExtOut( uint8_t cnt_par, Source src );
static void CmdMppt( uint8_t cnt_par, Source src );
static void CmdGen( uint8_t cnt_par, Source src );

static void CmdModbus( uint8_t cnt_par, Source src );
static void CmdModbusLog( uint8_t cnt_par, Source src );
static void CmdModbusErr( uint8_t cnt_par, Source src );

static void CmdMount( uint8_t cnt_par, Source src );
static void CmdUnmount( uint8_t cnt_par, Source src );
static void CmdDir( uint8_t cnt_par, Source src );
static void CmdType( uint8_t cnt_par, Source src );
static void CmdHex( uint8_t cnt_par, Source src );
static void CmdDelete( uint8_t cnt_par, Source src );
static void CmdDirDelete( uint8_t cnt_par, Source src );
static void CmdRename( uint8_t cnt_par, Source src );
static void CmdFile( uint8_t cnt_par, Source src );
static void CmdCid( uint8_t cnt_par, Source src );
static void CmdTask( uint8_t cnt_par, Source src );

static void CmdVoice( uint8_t cnt_par, Source src );
static void CmdVolume( uint8_t cnt_par, Source src );
static void CmdSound( uint8_t cnt_par, Source src );

static void CmdJobs( uint8_t cnt_par, Source src );
static void CmdCmd( uint8_t cnt_par, Source src );
static void CmdHmiStat( uint8_t cnt_par, Source src );

static void CmdSoc( uint8_t cnt_par, Source src );

//Перечень команд и прав доступа для выполнения
static const COMMAND cmd[] = {
    //------------------------------------------
    //имя       функция         разрешено       
    //команды   вызова          выполнять       
    //                          из задания      
    //------------------------------------------
    "time",     CmdTime,       0,
    "date",     CmdDate,       0,
    "dtime",    CmdDateTime,   0,
    "mount",    CmdMount,      0,
    "unmount",  CmdUnmount,    0,
    "config",   CmdConfig,     0,
    "scr",      CmdScr,        0,
    "dir",      CmdDir,        0,
    "type",     CmdType,       0,
    "hex",      CmdHex,        0,
    "del",      CmdDelete,     0,
    "dirdel",   CmdDirDelete,  0,
    "ren",      CmdRename,     0,
    "file",     CmdFile,       0,
    "cid",      CmdCid,        0,
    "task",     CmdTask,       0,
    "eeprom",   CmdEeprom,     0,
    "statall",  CmdStatAll,    0,
    "hmi",      CmdHmiStat,    0,
    "cls",      CmdCls,        0,
    "cmd",      CmdCmd,        EXEC_JOBS_ENABLE,
    "jobs",     CmdJobs,       0,
    "batmon",   CmdBatMon,     0,
    "inv",      CmdInvCtrl,    EXEC_JOBS_ENABLE,
    "alt",      CmdAlt,        EXEC_JOBS_ENABLE,
    "pv",       CmdPvMode,     EXEC_JOBS_ENABLE,
    "mppt",     CmdMppt,       0,
    "charge",   CmdCharge,     EXEC_JOBS_ENABLE,
    "trc",      CmdTrcMode,    EXEC_JOBS_ENABLE,
    "gen",      CmdGen,        EXEC_JOBS_ENABLE,
    "spa",      CmdSpa,        0,
    "modbus",   CmdModbus,     EXEC_JOBS_ENABLE,
    "moderr",   CmdModbusErr,  0,
    "modlog",   CmdModbusLog,  0,
    "voice",    CmdVoice,      EXEC_JOBS_ENABLE,
    "volume",   CmdVolume,     EXEC_JOBS_ENABLE,
    "sound",    CmdSound,      EXEC_JOBS_ENABLE,
    "res",      CmdReserv,     EXEC_JOBS_ENABLE,
    "ext",      CmdExtOut,     EXEC_JOBS_ENABLE,
    "reset",    CmdReset,      EXEC_JOBS_ENABLE,
    "system",   CmdSystem,     0,
    "?",        CmdHelp,       0,
    "soc",        CmdSoc,       0
 };

//*************************************************************************************************
// Инициализация консоли
//*************************************************************************************************
void CommandInit( void ) {

    command_event = osEventFlagsNew( &evn_attr );
    mutex_param = osMutexNew( &mutex1_attr );
    //создаем задачу выполнения команд
    osThreadNew( TaskConsole, NULL, &console_attr );
    Message( CONS_MSG_PROMPT );
 }

//*************************************************************************************************
// Задача обработки сообщений обмена по UART
//*************************************************************************************************
static void TaskConsole( void *pvParameters ) {

    uint32_t event;
    uint8_t i, cnt_par;

    for ( ;; ) {
        //ждем события
        event = osEventFlagsWait( command_event, EVN_COMMAND_MASK, osFlagsWaitAny, osWaitForever );
        if ( event == EVN_COMMAND_CR ) {
            //нажата клавиша Enter
            ConsoleSend( Message( CONS_MSG_CRLF ), CONS_NORMAL );
            if ( stream != NULL ) {
                //включен режим записи в файл, все данные пишем в файл
                //без выполнения команд, до получения кода: "ESC"
                fprintf( stream, "%s\r\n", UartBuffer() );
                UartRecvClear();
                ConsoleSend( Message( CONS_MSG_PROMPT2 ), CONS_NORMAL );
                continue;
               }
            if ( !strlen( UartBuffer() ) ) {
                //команда не введена
                UartRecvClear();
                ConsoleSend( Message( CONS_MSG_PROMPT ), CONS_NORMAL );
                continue;
               }
            //установим семафор ожидания разбора параметров
            osMutexAcquire( mutex_param, osWaitForever );
            //разбор параметров команды
            cnt_par = ParseCommand( UartBuffer() );
            //очистим приемный буфер
            UartRecvClear();
            //проверка и выполнение команды
            for ( i = 0; i < SIZE_ARRAY( cmd ); i++ ) {
                if ( strcasecmp( (const char *)&cmd[i].name_cmd, GetParamVal( IND_PAR_CMND ) ) )
                    continue;
                cmd[i].func( cnt_par, CONS_NORMAL ); //выполнение команды
                break;
               }
            if ( i == SIZE_ARRAY( cmd ) )
                ConsoleSend( Message( CONS_MSG_ERR_CMND ), CONS_NORMAL );
            else {
                if ( stream == NULL ) {
                    ConsoleSend( Message( CONS_MSG_CRLF ), CONS_NORMAL );
                    ConsoleSend( Message( CONS_MSG_PROMPT ), CONS_NORMAL );
                   }
               }
            osMutexRelease( mutex_param );
           }
        if ( event == EVN_COMMAND_CMD )
           ExecuteCmd(); //выполнение пакетного файла
        if ( event == EVN_COMMAND_ESC ) {
            //нажата клавиша Esc
            if ( stream == NULL )
                ChangeModeOut(); //меняем режим "командный" <-> "вывод данных"
            else FileClose();    //закрываем файл открытый командой "FILE"
           }
       }
 }

//*************************************************************************************************
// Вывод строки в консоль
// char *buff     - указатель на строку для вывода,
// uint8_t source - источник вызова (режим вывода в консоль)
//*************************************************************************************************
void ConsoleSend( char *buff, Source source ) {

    if ( buff == NULL )
        return;
    if ( OutDataStat() == TEMPLATE_OUT || source == CONS_SHADOW ) {
        //вывод шаблона, выводим только в файл протокола
        ExecLog( buff, LOG_NEW_STR );
        return;
       }
    if ( OutDataStat() == TEMPLATE_OUT && source == CONS_SELECTED ) {
        //вывод шаблона или вывод в фоновом режиме, выводим только в файл протокола
        ExecLog( buff, LOG_NEW_STR );
        return;
       }
    UartSendStr( buff );
 }

//*************************************************************************************************
// Выполнение команды из планировщика, вызов из Scheduler()
// char *job_cmnd - строка команды для выполнения
// return JobExec - код возврата при выполнении из планировщика
//*************************************************************************************************
JobExec ExecuteJob( char *job_cmnd ) {

    JobExec result;
    uint8_t i, cnt_par;

    if ( job_cmnd == NULL || !strlen( job_cmnd ) )
        return JOB_NO_COMMAND;
    //копируем команду во временный буфер
    memset( job_buffer, 0x00, sizeof( job_buffer ) );
    strcpy( job_buffer, job_cmnd );
    ExecLog( job_cmnd, LOG_NEW_CMND );
    //установим мьютекс ожидания разбора параметров
    osMutexAcquire( mutex_param, osWaitForever );
    //разбор параметров команды
    cnt_par = ParseCommand( job_cmnd );
    //проверка и выполнение команды
    for ( i = 0; i < SIZE_ARRAY( cmd ); i++ ) {
        if ( strcasecmp( (const char *)&cmd[i].name_cmd, GetParamVal( IND_PAR_CMND ) ) )
            continue;
        if ( cmd[i].exec_job == EXEC_JOBS_ENABLE ) {
            //выполнение команды
            cmd[i].func( cnt_par, CONS_SHADOW );
            result = JOB_OK;
           }
        else result = JOB_NO_ACCESS;
        break;
       }
    //освободим мьютекс
    osMutexRelease( mutex_param );
    if ( i == SIZE_ARRAY( cmd ) )
        result = JOB_NOT_FOUND;
    if ( result == JOB_NOT_FOUND )
        ConsoleSend( Message( CONS_MSG_ERR_CMND ), CONS_SHADOW );
    if ( result == JOB_NO_ACCESS )
        ConsoleSend( Message( CONS_MSG_ERR_NOJOB ), CONS_SHADOW );
    return result;
 }

//*************************************************************************************************
// Выполнение пакетного файла с командами, вложенное выполнение файлов не допускается
// Для выполнения необходимо открыть файл в текстовом режиме, дескриптор разместить в cmd_file
//*************************************************************************************************
static void ExecuteCmd( void ) {

    uint8_t i, cnt_par;
    char *rds, cmd_buffer[CONS_RECV_BUFF];

    if ( cmd_file == NULL )
        return; //файл не открыт
    //читаем одну строку из пакетного файла
    memset( cmd_buffer, 0x00, sizeof( cmd_buffer ) );
    rds = fgets( cmd_buffer, CONS_RECV_BUFF, cmd_file );
    if ( rds == NULL ) {
        //файл закончился
        fclose( cmd_file );
        cmd_file = NULL;
        ConsoleSend( Message( CONS_MSG_OK ), cmd_src );
        return;
       }
    ClearCrLf( cmd_buffer );
    if ( !strlen( cmd_buffer ) )
        return; //команды нет
    if ( cmd_buffer[0] == ';' || cmd_buffer[0] == '#' || cmd_buffer[0] == ' ' )
        return; //коментарий, пропускаем строку
    ExecLog( cmd_buffer, LOG_NEW_CMND );
    osMutexAcquire( mutex_param, osWaitForever );
    //разбор параметров команды
    cnt_par = ParseCommand( cmd_buffer );
    //проверка и выполнение команды
    for ( i = 0; i < SIZE_ARRAY( cmd ); i++ ) {
        if ( !strcasecmp( (const char *)&cmd[i].name_cmd, "cmd" ) )
            continue; //вложенное выполнение команд - пропускаем
        if ( strcasecmp( (const char *)&cmd[i].name_cmd, GetParamVal( IND_PAR_CMND ) ) )
            continue;
        //выполнение команды
        cmd[i].func( cnt_par, cmd_src );
        break;
       }
    osMutexRelease( mutex_param );
    if ( i == SIZE_ARRAY( cmd ) )
        ConsoleSend( Message( CONS_MSG_ERR_CMND ), cmd_src );
 }

//*************************************************************************************************
// Вывод перечня доступных команд
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdHelp( uint8_t cnt_par, Source src ) {

    ConsoleSend( MessageHelp(), src );
 }

//*************************************************************************************************
// Запуск выполнения пакетного файла с командами
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdCmd( uint8_t cnt_par, Source src ) {

    if ( SDStatus() == ERROR ) {
        ConsoleSend( MessageSd( MSG_SD_NO ), src );
        return;
       }
    //проверим параметры вызова
    if ( cnt_par != 2 && !strlen( GetParamVal( IND_PARAM1 ) ) ) {
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
        return;
       }
    //наличие файла
    cmd_file = fopen( GetParamVal( IND_PARAM1 ), "r" );
    if ( cmd_file == NULL ) {
        ConsoleSend( Message( CONS_MSG_ERR_FOPEN ), src );
        return;
       }
    cmd_src = src; //режим вывода результата консоль/фоновый
    osEventFlagsSet( command_event, EVN_COMMAND_CMD );
 }

//*************************************************************************************************
// Вывод/установка времени
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdTime( uint8_t cnt_par, Source src ) {

    if ( cnt_par == 2 ) {
        if ( RTCSetTime( GetParamVal( IND_PARAM1 ) ) != SUCCESS )
            ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
       }
    ConsoleSend( RTCGetTime( Message( CONS_MSG_CRLF ) ), src );
 }

//*************************************************************************************************
// Вывод/установка даты
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdDate( uint8_t cnt_par, Source src ) {

    if ( cnt_par == 2 ) {
        if ( RTCSetDate( GetParamVal( IND_PARAM1 ) ) != SUCCESS )
            ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
       }
    ConsoleSend( RTCGetDate( Message( CONS_MSG_CRLF ) ), src );
 }

//*************************************************************************************************
// Вывод даты/времени в полном формате
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdDateTime( uint8_t cnt_par, Source src ) {

    ConsoleSend( RTCGetDateTime( Message( CONS_MSG_CRLF ) ), src );
 }

//*************************************************************************************************
// Монтирование SD карты
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdMount( uint8_t cnt_par, Source src ) {

    SDMount();
 }

//*************************************************************************************************
// Размонтирование SD карты
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdUnmount( uint8_t cnt_par, Source src ) {

    SDUnMount();
 }

//*************************************************************************************************
// Управление параметрами настройки
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdConfig( uint8_t cnt_par, Source src ) {

    uint32_t msg;
    ConfigParam id_par;
    ConfigValSet cfg_set;

    if ( cnt_par == 1 ) {
        for ( id_par = CFG_SCR_FILE; id_par < DevParamCnt( ID_CONFIG, CNT_FULL ); id_par++ ) {
            ConsoleSend( ParamGetForm( ID_CONFIG, id_par, (ParamMode)( PARAM_NUMB | PARAM_DESC | PARAM_DOT | PARAM_VALUE | PARAM_UNIT ) ), src );
            ConsoleSend( Message( CONS_MSG_CRLF ), src );
           }
        return;
       }
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "save" ) ) {
        //сохранение в EEPROM
        ConfigSave();
        ConsoleSend( Message( CONS_MSG_OK ), src );
        return;
       }
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "load" ) ) {
        //загрузка значений параметров
        ConfigLoad();
        //просмотр параметров
        for ( id_par = CFG_SCR_FILE; id_par < DevParamCnt( ID_CONFIG, CNT_FULL ); id_par++ ) {
            ConsoleSend( ParamGetForm( ID_CONFIG, id_par, (ParamMode)( PARAM_NUMB | PARAM_DESC | PARAM_DOT | PARAM_VALUE | PARAM_UNIT ) ), src );
            ConsoleSend( Message( CONS_MSG_CRLF ), src );
           }
        return;
       }
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "clr" ) ) {
        //очистка всех параметров
        ConfigClear();
        ConsoleSend( Message( CONS_MSG_OK ), src );
        return;
       }
    if ( cnt_par == 3 ) {
        //установка значения параметра
        id_par = (ConfigParam)atoi( GetParamVal( IND_PARAM1 ) );
        StrToConfigVal( id_par, GetParamVal( IND_PARAM2 ), &cfg_set );
        if ( ConfigChkVal( id_par, cfg_set ) == SUCCESS ) {
            //для параметров scr_file, job_file, job_test не выводим результат сообщения
            if ( id_par > 3 )
                ConsoleSend( Message( CONS_MSG_OK ), src );
            //значение параметра изменилось, передадим в модуль HMI новое значение
            ConfigSet( id_par, &cfg_set );
            msg = ID_CONFIG;
            osMessageQueuePut( hmi_msg, &msg, 0, 0 );
            return;
           }
       }
    ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
 }

//*************************************************************************************************
// Перезагрузить шаблон экрана из файла и выполнить разбор макроподстановок
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdScr( uint8_t cnt_par, Source src ) {

    if ( SDStatus() == ERROR ) {
        ConsoleSend( MessageSd( MSG_SD_NO ), src );
        return;
       }
    ScreenLoad();
 }

//*************************************************************************************************
// Вывод содержимого SDCard
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdDir( uint8_t cnt_par, Source src ) {

    if ( SDStatus() == ERROR ) {
        ConsoleSend( MessageSd( MSG_SD_NO ), src );
        return;
       }
    SDDir( GetParamVal( IND_PARAM1 ) );
 }

//*************************************************************************************************
// Вывод файла на консоль
// По умолчанию выводиться только последняя страница файла
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdType( uint8_t cnt_par, Source src ) {

    if ( SDStatus() == ERROR ) {
        ConsoleSend( MessageSd( MSG_SD_NO ), src );
        return;
       }
    if ( !strlen( GetParamVal( IND_PARAM1 ) ) ) {
        ConsoleSend( Message( CONS_MSG_ERR_NONAME ), src );
        return;
       }
    FileType( GetParamVal( IND_PARAM1 ) );
}

//*************************************************************************************************
// Вывод файла на консоль в формате HEX
// По умолчанию выводиться только последняя страница файла
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdHex( uint8_t cnt_par, Source src ) {

    if ( SDStatus() == ERROR ) {
        ConsoleSend( MessageSd( MSG_SD_NO ), src );
        return;
       }
    if ( !strlen( GetParamVal( IND_PARAM1 ) ) ) {
        ConsoleSend( Message( CONS_MSG_ERR_NONAME ), src );
        return;
       }
    FileHex( GetParamVal( IND_PARAM1 ) );
}

//*************************************************************************************************
// Удаление файла
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdDelete( uint8_t cnt_par, Source src ) {

    if ( SDStatus() == ERROR ) {
        ConsoleSend( MessageSd( MSG_SD_NO ), src );
        return;
       }
    if ( GetParamVal( IND_PARAM1 ) == NULL ) {
        ConsoleSend( Message( CONS_MSG_ERR_NOPARAM ), src );
        return;
       }
    FileDelete( GetParamVal( IND_PARAM1 ) );
 }

//*************************************************************************************************
// Удаление файла
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdDirDelete( uint8_t cnt_par, Source src ) {

    if ( SDStatus() == ERROR ) {
        ConsoleSend( MessageSd( MSG_SD_NO ), src );
        return;
       }
    if ( GetParamVal( IND_PARAM1 ) == NULL ) {
        ConsoleSend( Message( CONS_MSG_ERR_NOPARAM ), src );
        return;
       }
    DirDelete( GetParamVal( IND_PARAM1 ) );
 }

//*************************************************************************************************
// Переименование файла
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdRename( uint8_t cnt_par, Source src ) {

    if ( SDStatus() == ERROR ) {
        ConsoleSend( MessageSd( MSG_SD_NO ), src );
        return;
       }
    if ( GetParamVal( IND_PARAM1 ) == NULL || GetParamVal( IND_PARAM2 ) == NULL ) {
        ConsoleSend( Message( CONS_MSG_ERR_NOPARAM ), src );
        return;
       }
    FileRename( GetParamVal( IND_PARAM1 ), GetParamVal( IND_PARAM2 ) );
 }

//*************************************************************************************************
// Вывод информации о SD карте
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdCid( uint8_t cnt_par, Source src ) {

    if ( SDStatus() == ERROR ) {
        ConsoleSend( MessageSd( MSG_SD_NO ), src );
        return;
       }
    SDCid();
 }

//*************************************************************************************************
// Вывод дампа памяти EEPROM по 1 блоку (64 байта)
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdEeprom( uint8_t cnt_par, Source src ) {

    char str[256], tmp[10];
    uint8_t dat[EEPROM_PAGE_SIZE];
    uint16_t hexi, hexb = 0, dati, page;

    if ( ( cnt_par == 1 || cnt_par == 2 ) && strcasecmp( GetParamVal( IND_PARAM1 ), "clr" ) ) {
        //вывод дампа памяти EEPROM
        page = atoi( GetParamVal( IND_PARAM1 ) );
        if ( page > 63 ) {
            ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
            return;
           }
        //инициализация массива
        memset( str, 0, sizeof( str ) );
        sprintf( tmp, "PAGE: %d\r\n", page );
        strcat( str, tmp );
        //читаем 1 блок
        EEPROM_Read( 0, page, (uint8_t*)&dat, MODE_8_BIT, EEPROM_PAGE_SIZE );
        for ( hexi = 0, dati = 0; hexi < 4; hexi++ ) {
            //вывод адреса
            sprintf( tmp, "%04X: ", page * EEPROM_PAGE_SIZE + hexi * 16 );
            strcat( str, tmp );
            //вывод строки по 16 байт
            for ( hexb = 0; hexb < 16; hexb++ ) {
                sprintf( tmp, "%02X ", dat[dati++] );
                strcat( str, tmp );
               }
            strcat( str, Message( CONS_MSG_CRLF ) );
           }
        ConsoleSend( str, src );
        return;
       }
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "clr" ) ) {
        //очистка блока памяти в EEPROM
        page = atoi( GetParamVal( IND_PARAM2 ) );
        if ( page < EEPROM_PAGE_NUM ) {
            EepromClear( page );
            ConsoleSend( Message( CONS_MSG_OK ), src );
            return;
           }
       }
    ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
 }

//*************************************************************************************************
// Вывод состояния всех входов
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdStatAll( uint8_t cnt_par, Source src ) {

    char str[80];
    uint32_t stat;

    stat = GetDataPort( STAT_ALL );
    //общее состояние
    sprintf( str, "CPU MODE = 0x%02X\r\n+24V BAT = %d\r\n+12V BAT = %d\r\n BAT CON = %d\r\nSTAT ALL = 0x%08X\r\n",
            CPUMode(), Fuse24Vdc(), StatCtrl(), BatConn(), stat );
    ConsoleSend( str, src );
    sprintf( str, " SD CARD = %s%s\r\n", SDDetect() ? "INSERT " : "", SDMountStat() ? "MOUNT" : "" );
    ConsoleSend( str, src );
    //статусы входов генератора
    sprintf( str, "     GEN = %s%s%s%s%s%s\r\n", ( stat & GEN_CONN ) ? "CONNECT " : "", ( stat & GEN_RUN ) ? "RUN " : "",
            ( stat & GEN_OVR ) ? "OVR " : "", ( stat & GEN_OIL ) ? "OIL " : "", ( stat & GEN_LOW_BAT ) ? "LOW_BAT " : "",
            ( stat & GEN_FUEL_LOW ) ? "FUEL_LOW" : "" );
    ConsoleSend( str, src );
    //реле монитора батареи
    sprintf( str, "  BATMON = %s\r\n", ( stat & BATMON_KEY ) ? "BATMON_KEY" : "" );
    ConsoleSend( str, src );
    //статусы зарядного уст-ва
    sprintf( str, " CHARGER = %s%s%s\r\n", ( stat & CHARGE_AC_OK ) ? "AC_OK " : "", ( stat & CHARGE_DEV_OK ) ? "DEV_OK " : "",
            ( stat & CHARGE_BANK_OK ) ? "BANK_OK" : "" );
    ConsoleSend( str, src );
    //солнечный контроллер заряда
    sprintf( str, " MPPT PV = %s%s%s%s%s\r\n", ( stat & MPPT_ON ) ? "ON " : "", ( stat & MPPT_LINK ) ? "LINK " : "",
            ( stat & MPPT_PV_ON ) ? "PV_ON " : "", ( stat & MPPT_K1 ) ? "K1 " : "", ( stat & MPPT_K2 ) ? "K2" : "" );
    ConsoleSend( str, src );
    //контроль автомата солнечного трекера
    sprintf( str, "     TRC = %s%s\r\n", ( stat & TRC_FUSE ) ? "ON " : "", ( stat & TRC_PWR ) ? "ON" : "" );
    ConsoleSend( str, src );
    //контроль подключения инверторов и нагрузки
    sprintf( str, "      TS = %s%s%s%s\r\n", ( stat & TS1000_CHK ) ? "TS1000_CHK " : "", ( stat & TS1000_LOC ) ? "TS1000_LOC " : "",
            ( stat & TS3000_CHK ) ? "TS3000_CHK " : "", ( stat & TS3000_LOC ) ? "TS3000_LOC" : "" );
    ConsoleSend( str, src );
    //резервные входы
    sprintf( str, " RES_CHK = %s%s%s%s\r\n", ( stat & RES_CHK1 ) ? "CHK1 " : "", ( stat & RES_CHK2 ) ? "CHK2 " : "",
            ( stat & RES_CHK3 ) ? "CHK3 " : "", ( stat & RES_CHK4 ) ? "CHK4" : "" );
    ConsoleSend( str, src );
    //контрольные входы трекера
    sprintf( str, " TRC_EXT = %s%s%s%s\r\n", ( stat & TRC_CH_RT ) ? "RT " : "", ( stat & TRC_CH_LF ) ? "LF " : "",
            ( stat & TRC_CH_DN ) ? "DN " : "", ( stat & TRC_CH_UP ) ? "UP" : "" );
    ConsoleSend( str, src );
    //состояние блока АВР
    sprintf( str, "     ALT = %s%s%s%s\r\n", ( stat & ALT_CONNECT ) ? "CONNECT " : "", ( stat & ALT_AC_MAIN ) ? "AC_MAIN " : "",
            ( stat & ALT_AC_GEN ) ? "AC_GEN " : "", ( stat & ALT_GEN_OK ) ? "GEN_OK" : "" );
    ConsoleSend( str, src );
 }

//*************************************************************************************************
// Обработка файла заданий планировщика, просмотр, добавление, удаление заданий
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdJobs( uint8_t cnt_par, Source src ) {

    uint8_t add, all = 0, mode = JOBS_VIEW;
    char cmd_job[CONS_RECV_BUFF];

    if ( SDStatus() == ERROR ) {
        ConsoleSend( MessageSd( MSG_SD_NO ), src );
        return;
       }
    //просмотр заданий
    if ( cnt_par == 1 ) {
        JobsEdit( JOBS_VIEW, 0, NULL, 0 );
        return;
       }
    //вывод заданий готовых в выполнению (без параметров запуска)
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "run" ) ) {
        JobsEdit( JOBS_RUN, 0, NULL, 0 );
        return;
       }
    //перезгрузка заданий из файла
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "load" ) ) {
        LoadJobs();
        return;
       }
    if ( cnt_par > 2 ) {
        if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "add" ) )
            mode = JOBS_ADD;
        if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "del" ) )
            mode = JOBS_DEL;
        if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "on" ) )
            mode = JOBS_ON;
        if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "off" ) )
            mode = JOBS_OFF;
        if ( !strcasecmp( GetParamVal( IND_PARAM2 ), "all" ) )
            all = 1;
        if ( mode == JOBS_ADD ) {
            //добавление строку с заданием, т.к. строка тоже "разбита"
            //по параметрам, соберем их вместе в одну строку
            memset( cmd_job, 0x00, sizeof( cmd_job ) );
            for ( add = IND_PARAM2; add < MAX_CNT_PARAM; add++ ) {
                //соберем параметры в одну строку
                if ( strlen( GetParamVal( (CmndParam)add ) ) ) {
                    strcat( cmd_job, GetParamVal( (CmndParam)add ) );
                    strcat( cmd_job, " " );
                   }
                else break;
               }
           }
        JobsEdit( (JobsCmnd)mode, atoi( GetParamVal( IND_PARAM2 ) ), cmd_job, all );
        return;
       }
    ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
 }

//*************************************************************************************************
// Вывод состояния монитора батареи
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdBatMon( uint8_t cnt_par, Source src ) {

    uint8_t ind;

    for ( ind = 0; ind < DevParamCnt( ID_DEV_BATMON, CNT_FULL ); ind++ ) {
        ConsoleSend( ParamGetForm( ID_DEV_BATMON, ind, (ParamMode)( PARAM_DESC | PARAM_DOT | PARAM_VALUE | PARAM_UNIT ) ), src );
        ConsoleSend( Message( CONS_MSG_CRLF ), src );
       }
 }

//*************************************************************************************************
// Управление инверторами
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdInvCtrl( uint8_t cnt_par, Source src ) {

    Device inv;
    char *result;
    uint8_t ind, ts;

    if ( cnt_par == 1 ) {
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
        return;
       }
    ts = atoi( GetParamVal( IND_PARAM1 ) ); //номер инвертора
    if ( ts == 1 )
        inv = ID_DEV_INV1;
    if ( ts == 2 )
        inv = ID_DEV_INV2;
    if ( cnt_par == 2 ) {
        //только вывод данных
        if ( inv == ID_DEV_INV1 || inv == ID_DEV_INV2 )  {
            for ( ind = 0; ind < DevParamCnt( inv, CNT_FULL ); ind++ ) {
                result = ParamGetForm( inv, ind, (ParamMode)( PARAM_DESC | PARAM_DOT | PARAM_VALUE | PARAM_UNIT ) );
                if ( result == NULL )
                    continue;
                ConsoleSend( result, src );
                ConsoleSend( Message( CONS_MSG_CRLF ), src );
               }
            return;
           }
        else {
            ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
            return;
           }
       }
    if ( ( strcasecmp( GetParamVal( IND_PARAM2 ), "on" ) && strcasecmp( GetParamVal( IND_PARAM2 ), "off" ) ) || ( ts != 1 && ts != 2 ) ) {
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
        return;
       }
    if ( !strcasecmp( GetParamVal( IND_PARAM2 ), "on" ) )
        InvCtrl( inv, INV_CTRL_ON );
    if ( !strcasecmp( GetParamVal( IND_PARAM2 ), "off" ) )
        InvCtrl( inv, INV_CTRL_OFF );
    ConsoleSend( Message( CONS_MSG_OK ), src );
 }

//*************************************************************************************************
// Перевод нагрузки на основную сеть (генератор) или инвертора
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdAlt( uint8_t cnt_par, Source src ) {

    char msg[80];
    int ind, ac, dc;
    uint8_t error;

    if ( cnt_par == 1 ) {
        for ( ind = 0; ind < DevParamCnt( ID_DEV_ALT, CNT_FULL ); ind++ ) {
            ConsoleSend( ParamGetForm( ID_DEV_ALT, ind, (ParamMode)( PARAM_DESC | PARAM_DOT | PARAM_VALUE | PARAM_UNIT ) ), src );
            ConsoleSend( Message( CONS_MSG_CRLF ), src );
           }
        return;
       }
    if ( cnt_par == 2 ) {
        ac = strcasecmp( GetParamVal( IND_PARAM1 ), "ac" );
        dc = strcasecmp( GetParamVal( IND_PARAM1 ), "dc" );
        if ( ac && dc ) {
            ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
            return;
           }
        if ( !ac )
            error = AltPowerAC();
        if ( !dc )
            error = AltPowerDC();
        if ( error ) {
            sprintf( msg, Message( CONS_MSG_ERR_DESC ), ErrorDescr( ID_DEV_ALT, 0, error ) );
            ConsoleSend( msg, src );
           }
        else ConsoleSend( Message( CONS_MSG_OK ), src );
        return;
       }
    ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
 }

//*************************************************************************************************
// Управление вкл/выкл и режимом солнечных панелей
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdPvMode( uint8_t cnt_par, Source src ) {

    char msg[80];
    int on, off, prl, ser;
    uint8_t ind, error = 0;

    if ( cnt_par != 1 && cnt_par != 2 ) {
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
        return;
       }
    if ( cnt_par == 1 ) {
        //параметра нет, вывод состояния
        for ( ind = 0; ind < DevParamCnt( ID_DEV_PV, CNT_FULL ); ind++ ) {
            ConsoleSend( ParamGetForm( ID_DEV_PV, ind, (ParamMode)( PARAM_DESC | PARAM_DOT | PARAM_VALUE | PARAM_UNIT ) ), src );
            ConsoleSend( Message( CONS_MSG_CRLF ), src );
           } 
        return;
       } 
    if ( cnt_par == 2 ) {
        //выполняем команды
        on = strcasecmp( GetParamVal( IND_PARAM1 ), "on" );
        off = strcasecmp( GetParamVal( IND_PARAM1 ), "off" );
        prl = strcasecmp( GetParamVal( IND_PARAM1 ), "par" );
        ser = strcasecmp( GetParamVal( IND_PARAM1 ), "ser" );
        if ( on && off && prl && ser ) {
            ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
            return;
           } 
        if ( !on )
            error = PvControl( PV_CTRL_ON, EEPROM_SAVE );
        if ( !off )
            error = PvControl( PV_CTRL_OFF, EEPROM_SAVE );
        if ( !prl )
            PvSetMode( PV_MODE_PAR, EEPROM_SAVE );
        if ( !ser )
            PvSetMode( PV_MODE_SER, EEPROM_SAVE );
        if ( error ) {
            sprintf( msg, Message( CONS_MSG_ERR_DESC ), ErrorDescr( ID_DEV_PV, 0, error ) );
            ConsoleSend( msg, src );
            return;
           }
        ConsoleSend( Message( CONS_MSG_OK ), src );
       } 
 }
 
//*************************************************************************************************
// Вывод параметров MPPT контроллера
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdMppt( uint8_t cnt_par, Source src ) {

    uint8_t ind;

    for ( ind = 0; ind < DevParamCnt( ID_DEV_MPPT, CNT_FULL ); ind++ ) {
        ConsoleSend( ParamGetForm( ID_DEV_MPPT, ind, (ParamMode)( PARAM_DESC | PARAM_DOT | PARAM_VALUE | PARAM_UNIT ) ), src );
        ConsoleSend( Message( CONS_MSG_CRLF ), src );
       } 
 }

//*************************************************************************************************
// Управление/состояние зарядкой АКБ от сети AC
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdCharge( uint8_t cnt_par, Source src ) {

    char str[50];
    ChargeMode chrgmode;
    uint8_t error, ind, mode, last_chrg; 
    
    if ( cnt_par == 1 ) {
        for ( ind = 0; ind < DevParamCnt( ID_DEV_CHARGER, CNT_FULL ); ind++ ) {
            ConsoleSend( ParamGetForm( ID_DEV_CHARGER, ind, (ParamMode)( PARAM_DESC | PARAM_DOT | PARAM_VALUE | PARAM_UNIT ) ), src );
            ConsoleSend( Message( CONS_MSG_CRLF ), src );
           } 
        return;
       }
    if ( cnt_par == 2 || cnt_par == 3 ) {
        //режим заряда 0/2/3/8
        mode = atoi( GetParamVal( IND_PARAM1 ) );
        if ( mode != 0 && mode != 2 && mode != 3 && mode != 8 ) {
            ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
            return;
           } 
        //запуск заряда если кол-во дней от полного заряда > указанного в параметре (>=10)
        //значение "дней от полного заряда" берем из BATMON
        last_chrg = atoi( GetParamVal( IND_PARAM2 ) );
        if ( last_chrg && batmon.link ) {
            if ( mode && batmon.h9 < last_chrg ) {
                sprintf( str, Message( CONS_MSG_CHARGE_DAY ), batmon.h9 );
                ConsoleSend( str, src );
                return;
               }
           }
        if ( mode == 0 )
            chrgmode = CHARGE_OFF;
        if ( mode == 2 )
            chrgmode = CHARGE_MODE2;
        if ( mode == 3 )
            chrgmode = CHARGE_MODE3;
        if ( mode == 8 )
            chrgmode = CHARGE_MODE8;
        error = Charger( chrgmode, EEPROM_SAVE );
        if ( error ) {
            sprintf( str, Message( CONS_MSG_ERR_DESC ), ErrorDescr( ID_DEV_CHARGER, 0, error ) );
            ConsoleSend( str, src );
           }
        else ConsoleSend( Message( CONS_MSG_OK ), src );
       } 
    else ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
 }
 
//*************************************************************************************************
// Управление трекером
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdTrcMode( uint8_t cnt_par, Source src ) {

    uint8_t ind, res;
    
    if ( cnt_par != 1 && cnt_par != 2 && cnt_par != 3 ) {
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
        return;
       }
    if ( cnt_par == 1 ) {
        //параметров нет, вывод статуса
        for ( ind = 0; ind < DevParamCnt( ID_DEV_TRC, CNT_FULL ); ind++ ) {
            ConsoleSend( ParamGetForm( ID_DEV_TRC, ind, (ParamMode)( PARAM_DESC | PARAM_DOT | PARAM_VALUE | PARAM_UNIT ) ), src );
            ConsoleSend( Message( CONS_MSG_CRLF ), src );
           } 
        return;
       } 
    if ( cnt_par == 2 ) {
        if ( tracker.link == LINK_CONN_NO ) {
            ConsoleSend( Message( CONS_MSG_ERR_NOLINK ), src );
            return;
           }
        //выполняем команды вкл/выкл реле питания актуаторов
        if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "on" ) ) {
            TRCMode( TRC_ACT_ON, EEPROM_SAVE );
            ConsoleSend( Message( CONS_MSG_OK ), src );
            return;
           }
        if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "off" ) ) {
            TRCMode( TRC_ACT_OFF, EEPROM_SAVE );
            ConsoleSend( Message( CONS_MSG_OK ), src );
            return;
           }
        if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "stop" ) ) {
            //прервать позиционирование
            if ( TrackerCmd( EXTRC_STOP, 0, 0 ) == SUCCESS )
                ConsoleSend( Message( CONS_MSG_OK ), src );
            else ConsoleSend( Message( CONS_MSG_ERR_NOLINK ), src );
            return;
           }
        if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "cmd" ) ) {
            //переход в режим позиционирования только по командам
            if ( TrackerCmd( EXTRC_CMD_ON, 0, 0 ) == SUCCESS )
                ConsoleSend( Message( CONS_MSG_OK ), src );
            else ConsoleSend( Message( CONS_MSG_ERR_NOLINK ), src );
            return;
           }
        if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "int" ) ) {
            //переход в режим позиционирования по сенсору
            if ( TrackerCmd( EXTRC_CMD_OFF, 0, 0 ) == SUCCESS )
                ConsoleSend( Message( CONS_MSG_OK ), src );
            else ConsoleSend( Message( CONS_MSG_ERR_NOLINK ), src );
            return;
           }
        if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "init" ) ) {
            //позиционирование актуаторов в начальное положения (закрыты)
            if ( TrackerCmd( EXTRC_VERT | EXTRC_HORZ, 0, 0 ) == SUCCESS )
                ConsoleSend( Message( CONS_MSG_OK ), src );
            else ConsoleSend( Message( CONS_MSG_ERR_NOLINK ), src );
            return;
           }
        if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "save" ) ) {
            //сохранить текущие значения позиционирования в EEPROM контроллера трекера
            if ( TrackerCmd( EXTRC_SEEP, 0, 0 ) == SUCCESS )
                ConsoleSend( Message( CONS_MSG_OK ), src );
            else ConsoleSend( Message( CONS_MSG_ERR_NOLINK ), src );
            return;
           }
        if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "rest" ) ) {
            TrcRestPos();
            //восстановить значения позиционирования из EEPROM контроллера трекера
            /*if ( TrackerCmd( EXTRC_REEP, 0, 0 ) == SUCCESS )
                ConsoleSend( Message( CONS_MSG_OK ), src );
            else ConsoleSend( Message( CONS_MSG_ERR_NOLINK ), src );*/
            return;
           }
        if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "reset" ) ) {
            //перезапуск контроллера трекера
            if ( TrackerCmd( EXTRC_RESET, 0, 0 ) == SUCCESS )
                ConsoleSend( Message( CONS_MSG_OK ), src );
            else ConsoleSend( Message( CONS_MSG_ERR_NOLINK ), src );
            return;
           }
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
       } 
    if ( cnt_par == 3 ) {
        if ( tracker.link == LINK_CONN_NO ) {
            ConsoleSend( Message( CONS_MSG_ERR_NOLINK ), src );
            return;
           }
        //позиционирование по параметрам
        if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "ver" ) && strlen( GetParamVal( IND_PARAM2 ) ) ) {
            //определим тип значения параметра позиционирования
            if ( strchr( GetParamVal( IND_PARAM2 ), '.' ) == NULL )
                res = TrackerSetPos( TRC_POS_VERTICAL, TRC_POS_LENGTH, atof( GetParamVal( IND_PARAM2 ) ) );  //значение в мм
            else res = TrackerSetPos( TRC_POS_VERTICAL, TRC_POS_DEGREE, atof( GetParamVal( IND_PARAM2 ) ) ); //значение в град.
            if ( res == TRC_SUCCESS ) {
                ConsoleSend( Message( CONS_MSG_OK ), src );
                return;
               }
            if ( res == TRC_ERR_VALUE ) {
                ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
                return;
               }
            if ( res == TRC_ERR_SEND ) {
                ConsoleSend( Message( CONS_MSG_ERR_SEND ), src );
                return;
               }
           }
        if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "hor" ) && strlen( GetParamVal( IND_PARAM2 ) ) ) {
            //определим тип значения параметра позиционирования
            if ( strchr( GetParamVal( IND_PARAM2 ), '.' ) == NULL )
                res = TrackerSetPos( TRC_POS_HORIZONTAL, TRC_POS_LENGTH, atof( GetParamVal( IND_PARAM2 ) ) );  //значение в мм
            else res = TrackerSetPos( TRC_POS_HORIZONTAL, TRC_POS_DEGREE, atof( GetParamVal( IND_PARAM2 ) ) ); //значение в град.
            if ( res == TRC_SUCCESS ) {
                ConsoleSend( Message( CONS_MSG_OK ), src );
                return;
               }
            if ( res == TRC_ERR_VALUE ) {
                ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
                return;
               }
            if ( res == TRC_ERR_SEND ) {
                ConsoleSend( Message( CONS_MSG_ERR_SEND ), src );
                return;
               }
           }
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
       } 
 }

//*************************************************************************************************
// Управление генератором
// Запуск/останов/вывод состояния генератора
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdGen( uint8_t cnt_par, Source src ) {

    uint8_t ind;
    
    if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "start" ) ) {
        GenStart();
        ConsoleSend( Message( CONS_MSG_OK ), src );
        return;
       } 
    if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "stop" ) ) {
        GenStop();
        ConsoleSend( Message( CONS_MSG_OK ), src );
        return;
       } 
    if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "test" ) ) {
        GenTest();
        ConsoleSend( Message( CONS_MSG_OK ), src );
        return;
       } 
    if ( cnt_par == 1 ) {
        //вывод состояния генератора
        for ( ind = 0; ind < DevParamCnt( ID_DEV_GEN, CNT_FULL ); ind++ ) {
            ConsoleSend( ParamGetForm( ID_DEV_GEN, ind, (ParamMode)( PARAM_DESC | PARAM_DOT | PARAM_VALUE ) ), src );
            ConsoleSend( Message( CONS_MSG_CRLF ), src );
           }
        return;
       }
    ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
 }

//*************************************************************************************************
// Вывод исходных и расчетных значений данных положения солнца
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdSpa( uint8_t cnt_par, Source src ) {

    uint8_t ind;
    
    //вывод данных
    for ( ind = 0; ind < DevParamCnt( ID_DEV_SPA, CNT_FULL ); ind++ ) {
        ConsoleSend( ParamGetForm( ID_DEV_SPA, ind, (ParamMode)( PARAM_DESC | PARAM_DOT | PARAM_VALUE | PARAM_UNIT ) ), src );
        ConsoleSend( Message( CONS_MSG_CRLF ), src );
       } 
 }

//*************************************************************************************************
// Воспроизведение голосового сообщения по номеру или по имени
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdVoice( uint8_t cnt_par, Source src ) {

    uint8_t ind;

    if ( !strlen( GetParamVal( IND_PARAM1 ) ) ) {
        //параметров нет, вывод состояния голосового информатора
        for ( ind = 0; ind < DevParamCnt( ID_DEV_VOICE, CNT_FULL ); ind++ ) {
            ConsoleSend( ParamGetForm( ID_DEV_VOICE, ind, (ParamMode)( PARAM_DESC | PARAM_DOT | PARAM_VALUE ) ), src );
            ConsoleSend( Message( CONS_MSG_CRLF ), src );
           }
        return;
       }
    ConsoleSend( ModBusErrDesc( Informing( (VoiceId)atoi( GetParamVal( IND_PARAM1 ) ), GetParamVal( IND_PARAM1 ) ) ), src );
    ConsoleSend( Message( CONS_MSG_CRLF ), src );
 }

//*************************************************************************************************
// Установка уровня громкости голосового сообщения
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdVolume( uint8_t cnt_par, Source src ) {

    if ( cnt_par != 2 ) {
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
        return;
       }
    ConsoleSend( ModBusErrDesc( SetVolume( (Volume)atoi( GetParamVal( IND_PARAM1 ) ) ) ), src );
    ConsoleSend( Message( CONS_MSG_CRLF ), src );
 }

//*************************************************************************************************
// Воспроизведение звукового сообщения по имени
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdSound( uint8_t cnt_par, Source src ) {

    uint8_t id, result;
    
    if ( cnt_par != 2 ) {
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
        return;
       }
    if ( !strlen( GetParamVal( IND_PARAM1 ) ) ) {
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
        return; //не указано имя сообщения
       }
    id = atoi( GetParamVal( IND_PARAM1 ) );
    if ( id )
        result = SoundPlay( (SoundId)id, NULL );                       //воспроизведение по номеру фрагмента
    else result = SoundPlay( SND_NOTHING, GetParamVal( IND_PARAM1 ) ); //воспроизведение по имени фрагмента
    if ( result == ERROR )
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
    else ConsoleSend( Message( CONS_MSG_OK ), src );
 }

//*************************************************************************************************
// Управление дополнительными реле
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdReserv( uint8_t cnt_par, Source src ) {

    int id, on, off, puls;
    
    if ( cnt_par != 3 ) {
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
        return;
       }
    //выполняем команды
    id = atoi( GetParamVal( IND_PARAM1 ) );
    on = strcasecmp( GetParamVal( IND_PARAM2 ), "on" );
    off = strcasecmp( GetParamVal( IND_PARAM2 ), "off" );
    puls = strcasecmp( GetParamVal( IND_PARAM2 ), "pulse" );
    if ( on && off && puls ) {
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
        return;
       } 
    if ( !on )
        ReservCmnd( (RelayRes)id, RELAY_ON );
    if ( !off )
        ReservCmnd( (RelayRes)id, RELAY_OFF );
    if ( !puls )
        ReservCmnd( (RelayRes)id, RELAY_PULSE );
    ConsoleSend( Message( CONS_MSG_OK ), src );
 }

//*************************************************************************************************
// Управление дополнителными выходами
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdExtOut( uint8_t cnt_par, Source src ) {

    RelayOut id_out; 
    RelayCmnd mode;
    bool chk1 = false, chk2 = false;
    
    if ( cnt_par != 3 ) {
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
        return;
       }
    //разбор параметров команды
    if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "lf" ) ) {
        chk1 = true;
        id_out = EXT_OUT_LF;
       }
    if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "rt" ) ) {
        chk1 = true;
        id_out = EXT_OUT_RT;
       }
    if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "up" ) ) {
        chk1 = true;
        id_out = EXT_OUT_UP;
       }
    if ( !strcasecmp( GetParamVal( IND_PARAM1 ), "dn" ) ) {
        chk1 = true;
        id_out = EXT_OUT_DN;
       }
    if ( !strcasecmp( GetParamVal( IND_PARAM2 ), "on" ) ) {
        chk2 = true;
        mode = RELAY_ON;
       }
    if ( !strcasecmp( GetParamVal( IND_PARAM2 ), "off" ) ) {
        chk2 = true;
        mode = RELAY_OFF;
       }
    if ( !strcasecmp( GetParamVal( IND_PARAM2 ), "pulse" ) ) {
        chk2 = true;
        mode = RELAY_PULSE;
       }
    //выполнение команды
    if ( chk1 == true && chk2 == true ) {
        ExtOut( id_out, mode );
        ConsoleSend( Message( CONS_MSG_OK ), src );
        return;
       }
 }

//*************************************************************************************************
// Перезапуск контроллера
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdReset( uint8_t cnt_par, Source src ) {

    NVIC_SystemReset();
 }

//*************************************************************************************************
// Вывод кол-ва ошибок обмена данными с HMI
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdHmiStat( uint8_t cnt_par, Source src ) {

    uint8_t ind;

    for ( ind = 0; ind < DevParamCnt( ID_DEV_HMI, CNT_FULL ); ind++ ) {
        ConsoleSend( ParamGetForm( ID_DEV_HMI, ind, (ParamMode)( PARAM_DESC | PARAM_DOT | PARAM_VALUE ) ), src );
        ConsoleSend( Message( CONS_MSG_CRLF ), src );
       }
 }

//*************************************************************************************************
// Отправляет команду по протоколу MODBUS
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
// Формат команды: modbus dev func reg cnt data1 [data2 ... data8 ]
//*************************************************************************************************
// Параметры команды:
// dev      - ID устройства
// func     - ID функции (0x05, 0x06, 0x0F, 0x10)
// reg      - адрес первого регистра
// cnt      - кол-во регистров
// data ... - данные для записи (не более 10 параметров)
//*************************************************************************************************
static void CmdModbus( uint8_t cnt_par, Source src ) {
    
    int val;
    uint8_t i;

    ModBusError stat;
    uint8_t *ptr_reg8, cnt_rpt;
    uint16_t reg_data[MB_MAX_DATA_REG];
    uint16_t *ptr_reg16, len = sizeof( reg_data );
    MBUS_REQUEST reqst = { .ptr_data = &reg_data, .ptr_lendata = &len };

    if ( cnt_par < 5 ) {
        ConsoleSend( Message( CONS_MSG_ERR_NOTENPAR ), src );
        return;
       }
    //читаем значения параметров
    //ID уст-ва
    sscanf( GetParamVal( IND_PARAM1 ), "%x", &val );
    reqst.dev_addr = (uint8_t)val;
    //код функции
    sscanf( GetParamVal( IND_PARAM2 ), "%x", &val );
    reqst.function = (uint8_t)val;
    //адрес первого регистра
    sscanf( GetParamVal( IND_PARAM3 ), "%x", &val );
    reqst.addr_reg = (uint16_t)val;
    //кол-во регистров
    sscanf( GetParamVal( IND_PARAM4 ), "%x", &val );
    reqst.cnt_reg = (uint16_t)val;
    //проверим входящие значения
    if ( !reqst.dev_addr || ModBusFunc( reqst.function ) == MBUS_FUNC_ERR || !reqst.cnt_reg || reqst.cnt_reg > MB_MAX_DATA_REG ) {
        ConsoleSend( Message( CONS_MSG_ERR_VALUE ), src );
        return;
       }
    if ( cnt_par >= 6 ) {
        //читаем значения регистров
        if ( ModBusAddr( reqst.function ) == MBUS_REG_16BIT ) {
            ptr_reg16 = (uint16_t *)reg_data; //16-битная адресация
            for ( i = 0; i < reqst.cnt_reg; i++, ptr_reg16++ ) {
                sscanf( GetParamVal( (CmndParam)( IND_PARAM5 + i ) ), "%x", &val );
                *( ptr_reg16 ) = val;
               }
           }
        else {
            ptr_reg8 = (uint8_t *)reg_data; //8-битная адресация
            for ( i = 0; i < CALC_BYTE( reqst.cnt_reg ); i++, ptr_reg8++ ) {
                sscanf( GetParamVal( (CmndParam)( IND_PARAM5 + i ) ), "%x", &val );
                *( ptr_reg8 ) = val;
               }
           }
       }
    //вывод расшифровки введенных параметров
    ConsoleSend( ModBusDecode( &reqst, DATA_LOG_REQUEST, DATA_LOG_NO_FILE ), src );
    ConsoleSend( Message( CONS_MSG_CRLF ), src );
    //отправка запроса
    cnt_rpt = CNT_REPEAT_REQST;
    do {
        len = sizeof( reg_data );
        stat = ModBusRequest( &reqst );
       } while ( cnt_rpt-- && ( stat == MBUS_ANSWER_CRC || stat == MBUS_ANSWER_TIMEOUT ) );
    if ( stat != MBUS_ANSWER_OK ) {
        //Вывод расшифровки ошибки
        ConsoleSend( ModBusErrDesc( stat ), src );
        ConsoleSend( Message( CONS_MSG_CRLF ), src );
        return;
       }
    ConsoleSend( ModBusDecode( &reqst, DATA_LOG_ANSWER, DATA_LOG_NO_FILE ), src );
    ConsoleSend( Message( CONS_MSG_OK ), src );
 }

//*************************************************************************************************
// Вывод кол-ва ошибок протокола MODBUS
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdModbusErr( uint8_t cnt_par, Source src ) {

    uint8_t i;
    char str[160];

    if ( cnt_par == 1 ) {
        ConsoleSend( Message( CONS_MSG_MODBUS_ERR ), src );
        ConsoleSend( Message( CONS_MSG_HEADER ), src );
        for ( i = 1; i < ModBusErrCnt( MBUS_ANSWER_OK ); i++ ) {
            sprintf( str, "%s\r\n", ModBusErrCntDesc( (ModBusError)i, str ) );
            ConsoleSend( str, src );
           }
        ConsoleSend( Message( CONS_MSG_HEADER ), src );
        //вывод кол-ва отправленных пакетов
        ConsoleSend( ModBusErrCntDesc( MBUS_ANSWER_OK, str ), src );
        ConsoleSend( Message( CONS_MSG_OK ), src );
       }
    if ( cnt_par == 2 && atoi( GetParamVal( IND_PARAM1 ) ) == 0 ) {
        ModBusErrClr();
        ConsoleSend( Message( CONS_MSG_OK ), src );
       }
 }

//*************************************************************************************************
// Вкл/выкл режима логирования обмена данными по MODBUS
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdModbusLog( uint8_t cnt_par, Source src ) {

    if ( cnt_par != 2 ) {
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
        return;
       }
    if ( ModbusLog( (Mode)atoi( GetParamVal( IND_PARAM1 ) ) ) == SUCCESS )
        ConsoleSend( Message( CONS_MSG_OK ), src );
    else ConsoleSend( Message( CONS_MSG_ERR_FOPEN ), src );
 }

//*************************************************************************************************
// Открывает файл для записи, все данные введенные из консоли записываются в файл
// Окончание записи файла (закрытие) по коду KEY_ESC
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdFile( uint8_t cnt_par, Source src ) {

    if ( SDStatus() == ERROR ) {
        ConsoleSend( MessageSd( MSG_SD_NO ), src );
        return;
       }
    if ( cnt_par != 2 ) {
        ConsoleSend( Message( CONS_MSG_ERR_PARAM ), src );
        return;
       }
    stream = fopen( GetParamVal( IND_PARAM1 ), "w" );
    if ( stream == NULL ) {
        ConsoleSend( Message( CONS_MSG_ERR_FOPEN ), src );
        return;
       }
    ConsoleSend( Message( CONS_MSG_OK ), src );
    ConsoleSend( Message( CONS_MSG_PROMPT2 ), src );
 }

//*************************************************************************************************
// Вывод значений частоты ядра/шины/версии
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdSystem( uint8_t cnt_par, Source src ) {
    
    osVersion_t osv;
    osStatus_t status;
    char msg[80], val[32], infobuf[32];
    
    sprintf( msg, Message( CONS_MSG_APP_VER ), version );
    ConsoleSend( msg, src );
    sprintf( msg, Message( CONS_MSG_BUILD_VER ), compiler_date, compiler_time );
    ConsoleSend( msg, src );
    FormatDot( CLKPWR_GetCLK( CLKPWR_CLKTYPE_CPU ), val );
    sprintf( msg, Message( CONS_MSG_CPU_CLOCK ), val );
    ConsoleSend( msg, src );
    FormatDot( CLKPWR_GetCLK( CLKPWR_CLKTYPE_PER ), val );
    sprintf( msg, Message( CONS_MSG_CLOCK_PER ), val );
    ConsoleSend( msg, src );
    FormatDot( osKernelGetTickFreq(), val );
    sprintf( msg, Message( CONS_MSG_CLOCK_KERNEL ), val );
    ConsoleSend( msg, src );
    FormatDot( osKernelGetSysTimerFreq(), val );
    sprintf( msg, Message( CONS_MSG_CLOCK_SYSTIMER ), val );
    ConsoleSend( msg, src );
    status = osKernelGetInfo( &osv, infobuf, sizeof( infobuf ) );
    if ( status == osOK ) {
        sprintf( msg, Message( CONS_MSG_KERNEL_INFO ), infobuf );
        ConsoleSend( msg, src );
        sprintf( msg, Message( CONS_MSG_KERNEL_VER ), Version( osv.kernel, val ) );
        ConsoleSend( msg, src );
        sprintf( msg, Message( CONS_MSG_KERNEL_API ), Version( osv.api, val ) );
        ConsoleSend( msg, src );
       }
 }

//*************************************************************************************************
//*************************************************************************************************
static void CmdSoc( uint8_t cnt_par, Source src ) {

    batmon.soc = atoi( GetParamVal( IND_PARAM1 ) );
    ConsoleSend( Message( CONS_MSG_OK ), CONS_NORMAL );
 }

//*********************************************************************************************
// Вывод статистики по задачам
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*********************************************************************************************
static void CmdTask( uint8_t cnt_par, Source src ) {

    uint8_t i;
    char str[80];
    const char *name;
    osThreadState_t state;
    osPriority_t priority;
    osThreadId_t th_id[40];
    uint32_t cnt_task, stack_space, stack_size; 

    //вывод шапки параметров
    ConsoleSend( Message( CONS_MSG_TASK_HDR1 ), src );
    ConsoleSend( Message( CONS_MSG_TASK_HDR2 ), src );
    //заполним весь массив th_id значением NULL
    memset( th_id, 0x00, sizeof( th_id ) );
    cnt_task = osThreadGetCount();
    cnt_task = osThreadEnumerate( &th_id[0], sizeof( th_id )/sizeof( th_id[0] ) );
    for ( i = 0; i < cnt_task; i++ ) {
        state = osThreadGetState( th_id[i] );
        priority = osThreadGetPriority( th_id[i] );
        stack_size = osThreadGetStackSize( th_id[i] );
        stack_space = osThreadGetStackSpace( th_id[i] );
        name = osThreadGetName( th_id[i] );
        if ( name != NULL && strlen( name ) )
            sprintf( str, "%2u %-16s    %2u    %-10s %5u %5u\r\n", i + 1, name, priority, TaskStateDesc( state ), stack_size, stack_space );
        else sprintf( str, "%2u ID = %-11u    %2u    %-10s %5u %5u\r\n", i + 1, (uint32_t)th_id[i], priority, TaskStateDesc( state ), stack_size, stack_space );
        ConsoleSend( str, src );
       }
    ConsoleSend( Message( CONS_MSG_TASK_HDR2 ), src );
 }

//*************************************************************************************************
// Очищаем экран консоли
// uint8_t cnt_par - кол-во параметров включая команду
// Source src      - режим вывода информации в консоль
//*************************************************************************************************
static void CmdCls( uint8_t cnt_par, Source src ) {

    vt100ClearScreen();
    vt100SetCursorMode( 1 ); //курсор включен
    vt100SetCursorPos( 1, 1 );
 }

//*********************************************************************************************
// Возвращает расшифровку статуса задачи
// osThreadState_t state - код статуса
//*********************************************************************************************
static char *TaskStateDesc( osThreadState_t state ) {

    if ( state == osThreadInactive )
        return state_name[0];
    if ( state == osThreadReady )
        return state_name[1];
    if ( state == osThreadRunning )
        return state_name[2];
    if ( state == osThreadBlocked )
        return state_name[3];
    if ( state == osThreadTerminated )
        return state_name[4];
    if ( state == osThreadError )
        return state_name[5];
    return NULL;
 }

//*************************************************************************************************
// Закрываем файл открытый командой "FILE"
//*************************************************************************************************
static void FileClose( void ) {

    if ( SDStatus() == ERROR ) {
        ConsoleSend( MessageSd( MSG_SD_NO ), CONS_NORMAL );
        return;
       }
    if ( stream == NULL )
        return;
    //запись строки введенной перед нажатием "ESC"
    if ( strlen( UartBuffer() ) )
        fprintf( stream, "%s\r\n", UartBuffer() );
    fclose( stream );
    stream = NULL;
    UartRecvClear();
    ConsoleSend( Message( CONS_MSG_OK ), CONS_NORMAL );
 }

//*************************************************************************************************
// Вывод в файл логирования команд и результатов их выполнения из планировщика
// или команд длительного исполнения: управления генератором, инверторами
//*************************************************************************************************
static void ExecLog( char *str, LogModeCmd mode ) {

    FILE *log;
    char name[80];

    if ( str == NULL || !strlen( str ) )
        return; //данных нет
    if ( SDStatus() == ERROR )
        return; //карты нет
    sprintf( name, "\\execute\\cmd_%s.log", RTCFileName() );
    log = fopen( name, "a" );
    if ( log != NULL ) {
        //запишем в лог файл
        if ( mode == LOG_NEW_CMND )
            fprintf( log, "%s > %s\r\n", RTCGetLog(), str );
        else fprintf( log, "%s", str );
        fclose( log );
       }
 }

//*************************************************************************************************
// Удаляет в исходной строке коды CR и LF '\r\n'
// char *str - указатель на исходную строку
// result    - указатель на исходную строку с удаленными CR и LF '\r\n'
//*************************************************************************************************
char *ClearCrLf( char *str ) {

    char *temp;
    
    temp = str;
    while ( *temp ) {
        if ( *temp == KEY_CR || *temp == KEY_LF )
            *temp = '\0';
        temp++;
       }
    return str;
 }

//*********************************************************************************************
// Расшифровка версии в текстовый буфер
// uint32_t version - значение версии в формате: mmnnnrrrr dec
// char *str        - указатель на строку для размещения результата
// return           - указатель на строку с расшифровкой в формате major.minor.rev
//*********************************************************************************************
static char *Version( uint32_t version, char *str ) {

    uint32_t major, minor, rev;
    
    rev = version%10000;
    version /= 10000;
    minor = version%1000;
    major = version/1000;
    sprintf( str, "%d.%d.%d", major, minor, rev );
    return str; 
 }
