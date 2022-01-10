
//*************************************************************************************************
//
// Планировщик заданий
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "rl_fs.h"
#include "cmsis_os2.h"

#include "dev_data.h"

#include "rtc.h"
#include "command.h"
#include "sdcard.h"
#include "eeprom.h"
#include "config.h"
#include "scheduler.h"
#include "scheduler_def.h"
#include "bitstring.h"
#include "command.h"
#include "message.h"
#include "events.h"

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osEventFlagsId_t job_event = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
const static char parsing_log[] = "job_parsing.log";

static char const *dows_name[]  = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun", NULL };
static char const *month_name[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec", NULL };

//Расшифровка ошибок параметров задания
static char * const err_desc[] = {
    "OK",
    "Ошибка в секундах",
    "Ошибка в минутах",
    "Ошибка в часах",
    "Ошибка в днях месяца",
    "Ошибка в месяце",
    "Ошибка в дне недели",
    "Ошибка в команде",
    "Ошибка в периодичности"
 };

//Коды ошибок параметров задания
typedef enum {
    ERR_JOB_NONE, 
    ERR_JOB_SECONDS, 
    ERR_JOB_MINUTE, 
    ERR_JOB_HOUR, 
    ERR_JOB_DOM, 
    ERR_JOB_MONTH, 
    ERR_JOB_DOW, 
    ERR_JOB_CMD, 
    ERR_JOB_TIMESPEC
 } ErrCode;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static osThreadId_t task_exec, task_jobs;
static JOB jobs[MAX_JOBS]; //список заданий загруженный из файла
static char buffer[SIZE_BUFFER], job_name[40];
static char temp[SIZE_BUFFER], str[SIZE_BUFFER];
static osMessageQueueId_t exec_msg = NULL;
static uint8_t idx_read, ind_job;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t shd_attr = {
    .name = "Scheduler",
    .stack_size = 384,
    .priority = osPriorityNormal
 };

static const osThreadAttr_t job_attr = {
    .name = "JobsExec",
    .stack_size = 1024,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn_shd = { .name = "Scheduler" };
static const osMessageQueueAttr_t que_attr = { .name = "JobsExec" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static ErrCode JobAdd( char *str );
static void JobsClear( void );
static char GetList( bitstr_t *bits, uint8_t low, uint8_t high, char const *names[], char ch );
static uint8_t SetElement( bitstr_t *bits, uint8_t low, uint8_t high, uint8_t number );
static char GetNumber( uint8_t *numptr, uint8_t low, char const *names[], char ch );
static char GetRange( bitstr_t *bits, uint8_t low, uint8_t high, char const *names[], char ch );
static uint8_t GetString( char *string, uint8_t size, char *terms );
static void JobClear( JOB *JClr );
static void JobParam( JOB *Jobp, char *source );
static char GetChar( void );
static void UngetChar( void );
static void SkipComments( void );
static int StrCaseCmp( const char *left, char *right );
static const char *JobGetError( uint8_t error );
static void CommentSpace( char *buffer );
static void LeadSpace( char *buffer );
static void TaskJob( void *pvParameters );
static void TaskExec( void *pvParameters );
static void JobCheck( void );
static void JobRunReboot( void );

//*************************************************************************************************
// Инициализация модуля заданий
//*************************************************************************************************
void JobInit( void ) {

    //задания загружаются автоматически при каждом монтировании SD карты SDMount()
    job_event = osEventFlagsNew( &evn_shd );
    exec_msg = osMessageQueueNew( 128, sizeof( uint8_t ), &que_attr );
    //создаем задачу управления заданиями
    task_jobs = osThreadNew( TaskJob, NULL, &shd_attr );
    task_exec = osThreadNew( TaskExec, NULL, &job_attr );
    JobRunReboot(); //задания для выполнения в момент загрузки системы
 }

//*************************************************************************************************
// Задача управления заданиями
//*************************************************************************************************
static void TaskJob( void *pvParameters ) {

    for ( ;; ) {
        //ждем события от RTC
        osEventFlagsWait( job_event, EVN_RTC_SECONDS, osFlagsWaitAny, osWaitForever );
        JobCheck(); //проверка заданий, помещение в очередь на исполнение
       }
 }

//*************************************************************************************************
// Задача исполнения заданий планировщика
//*************************************************************************************************
static void TaskExec( void *pvParameters ) {

    FILE *log;
    char name[40];
    uint8_t id_job;
    JobExec result;
    osStatus_t status;
    
    for ( ;; ) {
        //ждем события
        status = osMessageQueueGet( exec_msg, &id_job, NULL, osWaitForever );
        if ( status == osOK ) {
            if ( id_job < SIZE_ARRAY( jobs ) ) {
                memset( buffer, 0x00, sizeof( buffer ) );
                strcpy( buffer, jobs[id_job].command );
                sprintf( name, "\\execute\\job_%s.log", RTCFileName() );
                log = fopen( name, "a" );
                if ( log != NULL )
                    fprintf( log, "%s %s ... ", RTCGetDateTime( NULL ), buffer );
                //выполняем задание
                result = ExecuteJob( buffer );
                if ( log != NULL ) {
                    if ( result == JOB_OK )         //команда из планировщика выполнена
                        fprintf( log, "%s", Message( CONS_MSG_OK ) );
                    if ( result == JOB_NO_COMMAND ) //команда не указана
                        fprintf( log, "%s", Message( CONS_MSG_ERR_CMND ) );
                    if ( result == JOB_NO_ACCESS )  //команду нельзя выполнять из планировщика
                        fprintf( log, "%s", Message( CONS_MSG_ERR_NOJOB ) );
                    if ( result == JOB_NOT_FOUND )  //команда не найдена в списке
                        fprintf( log, "%s", Message( CONS_MSG_ERR_CMND ) );
                   }
               }
            if ( log != NULL )
                fclose( log );
           }
       }
 }

//*************************************************************************************************
// Загрузка заданий из файла, имя файла указано в config.job_test и config.job_file
// Загружаются только включенные задания
//*************************************************************************************************
void LoadJobs( void ) {

    FILE *file;
    uint16_t row = 0, res_add;
    
    //выбор файла заданий в зависимости от режима
    if ( config.mode_sys == SYSTEM_MODE_TEST )
        strcpy( job_name, config.job_test );
    else strcpy( job_name, config.job_file );
    sprintf( str, Message( CONS_MSG_FILE_JOBS ), job_name );
    ConsoleSend( str, CONS_NORMAL );
    file = fopen( job_name, "r" );
    if ( file == NULL ) {
        ConsoleSend( Message( CONS_MSG_ERR_FOPEN ), CONS_NORMAL );
        return;
       }
    //останавливаем исполнение заданий
    osThreadSuspend( task_exec );
    osThreadSuspend( task_jobs );
    //сброс очереди заданий
    osMessageQueueReset( exec_msg );
    //обнулим массив заданий
    JobsClear();
    while ( !feof( file ) ) {
        if ( fgets( str, sizeof( str ), file ) == NULL )
            continue;
        LeadSpace( str ); //убираем ведущие пробелы
        if ( str[0] == ';' || str[0] == '#' || !strlen( str ) )
            continue;   //строка коментарий, задание выключено, пропускаем
        //уберем коментарий в строке с заданием
        CommentSpace( str );
        //добавляем задание
        res_add = JobAdd( str );
        ClearCrLf( str );
        sprintf( temp, "\r\nJOB: %s ... %s", str, JobGetError( res_add ) );
        ConsoleSend( temp, CONS_NORMAL );
        row++;
        //проверка лимита строк
        if ( row > MAX_JOBS ) {
            ConsoleSend( Message( CONS_MSG_ERR_LIMITSTR ), CONS_NORMAL );
            break;
           }
       }
    fclose( file );
    //восстанавливаем исполнение заданий
    osThreadResume( task_exec );
    osThreadResume( task_jobs );
    ConsoleSend( Message( CONS_MSG_CRLF ), CONS_NORMAL );
 }

//*************************************************************************************************
// Обработка общего списка заданий, в случае совпадения параметров запуска, 
// задание помещается в очередь на исполнение. 
// Выполняется из TaskJob() 1 раз в секунду.
//*************************************************************************************************
static void JobCheck( void ) {

    JOB *jobp;
    RTC_TIME_Type Time;
    uint8_t id_job, cmp, seconds, minute, hour, day, month, dow;
    
    RTC_GetFullTime( LPC_RTC, &Time );
    seconds = Time.SEC;
    minute = Time.MIN;
    hour = Time.HOUR;
    day = Time.DOM - FIRST_DAY;
    //0..11 -> 1..12
    month = Time.MONTH - FIRST_MONTH;
    dow = Time.DOW;

    for ( id_job = 0; id_job < MAX_JOBS; id_job++ ) {
        //идем по всем заданиям в списке
        jobp = &jobs[id_job];
        if ( !( jobp->flags & JOB_PRESENT ) )
            continue; //задание не готово к выполнению
        /*if ( BitTest( Jobp->seconds, seconds ) && 
             BitTest( Jobp->minute, minute ) && \
             BitTest( Jobp->hour, hour ) && \
             BitTest( Jobp->month, month ) && ( 
             ( ( Jobp->flags & DAY_STAR ) || ( Jobp->flags & DOW_STAR ) ) ? ( BitTest( Jobp->dow, dow ) && \
             BitTest( Jobp->day, day ) ) : ( BitTest( Jobp->dow, dow ) || BitTest( Jobp->day, day ) ) ) ) {*/
        cmp = 0;
        if ( BitTest( jobp->seconds, seconds ) )
            cmp++;
        if ( BitTest( jobp->minute, minute ) )
            cmp++;
        if ( BitTest( jobp->hour, hour ) )
            cmp++;
        if ( BitTest( jobp->month, month ) )
            cmp++;
        if ( ( ( jobp->flags & DAY_STAR ) || ( jobp->flags & DOW_STAR ) ) ? ( BitTest( jobp->dow, dow ) && \
             BitTest( jobp->day, day ) ) : ( BitTest( jobp->dow, dow ) || BitTest( jobp->day, day ) ) )
             cmp++;
        if ( cmp == 5 ) {
            //все параметры выполнения совпали, команда задание помещается в список на выполнение
            osMessageQueuePut( exec_msg, &id_job, 0, osWaitForever );
           }
       }
 }

//*************************************************************************************************
// Обработка команд для файла заданий: добавление/удаление/вкл/выключение задания
// JobsCmnd mode    - режим редактирования
// uint8_t numb_str - номер строки для которой выполняется команда
// char *job_text   - строка с добавляемым заданием
// uint8_t flg_all  - признак выполнения команд: JOBS_ON JOBS_OFF для всех строк файла
//*************************************************************************************************
void JobsEdit( JobsCmnd cmnd, uint8_t numb_str, char *job_text, uint8_t flg_all ) {

    uint8_t ind;
    uint16_t rows;
    bool flg_edit = false;
    FILE *job_file, *job_new;
    char str[10], cmd_job[SIZE_BUFFER];

    if ( cmnd == JOBS_ADD ) {
        //добавление строки задания
        if ( !strlen( job_text ) ) {
            ConsoleSend( Message( CONS_MSG_ERR_JOB ), CONS_NORMAL );
            return;
           }
        job_file = fopen( job_name, "a" );
        if ( job_file == NULL ) {
            ConsoleSend( Message( CONS_MSG_ERR_FOPEN ), CONS_NORMAL );
            return;
           }
        fprintf( job_file, "%s\r\n", job_text );
        fclose( job_file );
        flg_edit = true;  //файл изменился, установим признак перезагрузки заданий
        cmnd = JOBS_VIEW; //выведем снова задания на экран
       }
    if ( cmnd == JOBS_DEL || cmnd == JOBS_ON || cmnd == JOBS_OFF ) {
        //удаление, включение, выключение задания
        job_file = fopen( job_name, "r" );
        if ( job_file == NULL ) {
            ConsoleSend( Message( CONS_MSG_FILE ), CONS_NORMAL );
            ConsoleSend( job_name, CONS_NORMAL );
            ConsoleSend( Message( CONS_MSG_NOT_OPEN ), CONS_NORMAL );
            return;
           }
        //откроем временный файл
        job_new = fopen( "jobs~", "w" );
        if ( job_new == NULL ) {
            if ( job_file != NULL )
                fclose( job_file );
            ConsoleSend( Message( CONS_MSG_ERR_JOB_OPEN ), CONS_NORMAL );
            return;
           }
        rows = 0;
        while ( !feof( job_file ) ) {
            //построчное чтение 
            memset( cmd_job, 0x00, sizeof( cmd_job ) );
            if ( fgets( cmd_job, sizeof( cmd_job ), job_file ) != NULL ) {
                rows++;
                //удаление строки
                if ( cmnd == JOBS_DEL && rows == numb_str )
                    continue; //удаляем строку
                //включение задания для одной строки
                if ( cmnd == JOBS_ON && !flg_all && rows == numb_str && cmd_job[0] == '#' )
                    memmove( cmd_job, cmd_job + 1, strlen( cmd_job ) + 1 );
                //выключение задания для одной строки
                if ( cmnd == JOBS_OFF && !flg_all && rows == numb_str && cmd_job[0] != '#' ) {
                    memmove( cmd_job + 1, cmd_job, strlen( cmd_job ) );
                    cmd_job[0] = '#'; //добавим признак выключения задания
                   } 
                //включение задания для всех строк
                if ( cmnd == JOBS_ON && flg_all && cmd_job[0] == '#' )
                    memmove( cmd_job, cmd_job + 1, strlen( cmd_job ) + 1 );
                //выключение задания для всех строк
                if ( cmnd == JOBS_OFF && flg_all && ( cmd_job[0] == '*' || isdigit( cmd_job[0] ) ) ) {
                    memmove( cmd_job + 1, cmd_job, strlen( cmd_job ) );
                    cmd_job[0] = '#'; //добавим признак выключения задания
                   } 
                //сохраним в файл скорректированную строку
                fputs( cmd_job, job_new );
               }
           }
        fclose( job_file );
        fclose( job_new );
        //удаление старого файлов
        if ( fdelete( job_name, NULL ) != fsOK ) {
            ConsoleSend( Message( CONS_MSG_FILE ), CONS_NORMAL );
            ConsoleSend( job_name, CONS_NORMAL );
            ConsoleSend( Message( CONS_MSG_NOT_DELETED ), CONS_NORMAL );
            return;
           }
        //переименуем временный файл в постоянный
        if ( frename( "jobs~", job_name ) != fsOK ) {
            ConsoleSend( Message( CONS_MSG_ERR_JOB_RENAME ), CONS_NORMAL );
            return;
           }
        flg_edit = true;  //файл изменился, установим признак перезагрузки заданий
        cmnd = JOBS_VIEW; //выведем снова задания на экран
       }
    if ( cmnd == JOBS_VIEW ) {
        //вывод содержимого файла с заданиями
        job_file = fopen( job_name, "r" );               
        if ( job_file == NULL ) {
            ConsoleSend( Message( CONS_MSG_ERR_FOPEN ), CONS_NORMAL );
            return;
           }
        ConsoleSend( Message( CONS_MSG_HEADER ), CONS_NORMAL );
        rows = 0;
        while ( !feof( job_file ) ) {
            //построчное чтение 
            if ( fgets( cmd_job, sizeof( cmd_job ), job_file ) != NULL ) {
                //т.к. включение и выключение заданий выполняется по номерам строк 
                //из исходного файла, выводим все строки в т.ч. пустые
                rows++; //счетчик строк
                sprintf( str, "%3d: ", rows );
                ConsoleSend( str, CONS_NORMAL );
                ConsoleSend( cmd_job, CONS_NORMAL );
               }
            }
        fclose( job_file );
        ConsoleSend( Message( CONS_MSG_HEADER ), CONS_NORMAL );
       }
    if ( cmnd == JOBS_RUN ) {
        //вывод заданий готовых в выполнению (без параметров запуска)
        ConsoleSend( Message( CONS_MSG_HEADER ), CONS_NORMAL );
        for ( ind = 0; ind < MAX_JOBS; ind++ ) {
            if ( strlen( jobs[ind].command ) ) {
                ConsoleSend( jobs[ind].command, CONS_NORMAL );
                ConsoleSend( Message( CONS_MSG_CRLF ), CONS_NORMAL );
               }
           }
        ConsoleSend( Message( CONS_MSG_HEADER ), CONS_NORMAL );
       }
    if ( flg_edit == true )
        LoadJobs(); //выполнили редактирование, перезагрузим обновленный список заданий
 }

//*************************************************************************************************
// Помещает в очередь задания для выполнения в момент загрузки системы
//*************************************************************************************************
static void JobRunReboot( void ) {

    uint8_t id_job;
    JOB *jobp;

    for ( id_job = 0; id_job < MAX_JOBS; id_job++ ) {
        jobp = &jobs[id_job];
        if ( jobp->flags & WHEN_REBOOT )
            osMessageQueuePut( exec_msg, &id_job, 0, osWaitForever );
       }
 }

//*************************************************************************************************
// Добавляем задание в список задач при условии корректности параметров задания
// char *str      - исходная строка с заданием типа "*/10 * * * * * command"
// return ErrCode - результат проверки на корректность параметров
//*************************************************************************************************
static ErrCode JobAdd( char *str ) {

    JOB *jobp;
    char ch, cmd[MAX_LEN_JOBCMMND];

    if ( str == NULL || !strlen( str ) )
        return ERR_JOB_NONE; //строка задания пустая
    jobp = &jobs[ind_job];
    //сохраним строку во временный буфер
    idx_read = 0;
    memset( buffer, 0x00, sizeof( buffer ) );
    strcpy( buffer, str );
    SkipComments();
    ch = GetChar();
    if ( ch == STR_END )
        return ERR_JOB_NONE;
    if ( ch == '@' ) {
        //признак фиксированных параметров задания
        ch = GetString( cmd, MAX_LEN_JOBCMMND, " \t\r\n" );
        if ( !strcasecmp( "reboot", cmd ) ) {
            jobp->flags |= WHEN_REBOOT; //запуск при загрузке
        } else if ( !strcasecmp( "yearly", cmd ) || !strcasecmp( "annually", cmd )) {
            //ежегодно
            BitSet( jobp->seconds, 0 );
            BitSet( jobp->minute, 0 );
            BitSet( jobp->hour, 0 );
            BitSet( jobp->day, 0 );
            BitSet( jobp->month, 0 );
            NBitSet( jobp->dow, 0, ( LAST_DOW - FIRST_DOW + 1 ) );
        } else if ( !strcasecmp( "monthly", cmd ) ) {
            //ежемесячно
            BitSet( jobp->seconds, 0 );
            BitSet( jobp->minute, 0 );
            BitSet( jobp->hour, 0 );
            BitSet( jobp->day, 0 );
            NBitSet( jobp->month, 0, ( LAST_MONTH - FIRST_MONTH + 1 ) );
            NBitSet( jobp->dow, 0, ( LAST_DOW - FIRST_DOW + 1 ) );
        } else if ( !strcasecmp( "weekly", cmd ) ) {
            //еженедельно
            BitSet( jobp->seconds, 0 );
            BitSet( jobp->minute, 0 );
            BitSet( jobp->hour, 0 );
            NBitSet( jobp->day, 0, ( LAST_DAY - FIRST_DAY + 1 ) );
            NBitSet( jobp->month, 0, ( LAST_MONTH - FIRST_MONTH+1) );
            BitSet( jobp->dow, 0 );
        } else if ( !strcasecmp( "daily", cmd ) || !strcasecmp( "midnight", cmd ) ) {
            //ежедневная полночь
            BitSet( jobp->seconds, 0 );
            BitSet( jobp->minute, 0 );
            BitSet( jobp->hour, 0 );
            NBitSet( jobp->day, 0, ( LAST_DAY - FIRST_DAY + 1 ) );
            NBitSet( jobp->month, 0, ( LAST_MONTH - FIRST_MONTH + 1 ) );
            NBitSet( jobp->dow, 0, ( LAST_DOW - FIRST_DOW + 1 ) );
        } else if ( !strcasecmp( "hourly", cmd ) ) {
            //почасовой
            BitSet( jobp->seconds, 0 );
            BitSet( jobp->minute, 0 );
            BitSet( jobp->hour, ( LAST_HOUR - FIRST_HOUR + 1 ) );
            NBitSet( jobp->day, 0, ( LAST_DAY - FIRST_DAY + 1 ) );
            NBitSet( jobp->month, 0, ( LAST_MONTH - FIRST_MONTH + 1 ) );
            NBitSet( jobp->dow, 0, ( LAST_DOW - FIRST_DOW + 1 ) );
        } else return ERR_JOB_TIMESPEC;
    } else {
        //обработка значений "секунды"
        ch = GetList( jobp->seconds, FIRST_SECONDS, LAST_SECONDS, PPC_NULL, ch );
        if ( ch == STR_END )
            return ERR_JOB_SECONDS;
        //обработка значений "минуты"
        ch = GetList( jobp->minute, FIRST_MINUTE, LAST_MINUTE, PPC_NULL, ch );
        if ( ch == STR_END )
            return ERR_JOB_MINUTE;
        //обработка значений "часы"
        ch = GetList( jobp->hour, FIRST_HOUR, LAST_HOUR, PPC_NULL, ch );
        if ( ch == STR_END )
            return ERR_JOB_HOUR;
        //обработка значений "число месяца"
        if ( ch == '*' )
            jobp->flags |= DAY_STAR;
        ch = GetList( jobp->day, FIRST_DAY, LAST_DAY, PPC_NULL, ch );
        if ( ch == STR_END )
            return ERR_JOB_DOM;
        //обработка значений "месяц"
        ch = GetList( jobp->month, FIRST_MONTH, LAST_MONTH, month_name, ch );
        if ( ch == STR_END )
            return ERR_JOB_MONTH;
        //обработка значений "день недели"
        if ( ch == '*' )
            jobp->flags |= DOW_STAR;
        ch = GetList( jobp->dow, FIRST_DOW, LAST_DOW, dows_name, ch );
        if ( ch == STR_END )
            return ERR_JOB_DOW;
       }
    //make sundays equivilent
    if ( BitTest( jobp->dow, 0 ) || BitTest( jobp->dow, 7 ) ) {
        BitSet( jobp->dow, 0 );
        BitSet( jobp->dow, 7 );
       }
    //сейчас индекс указывает на первый символ команды, уменьшим на 1
    UngetChar();
    //возвращает строку команды
    ch = GetString( cmd, MAX_LEN_JOBCMMND, "\r\n" );
    if ( ch == STR_END )
        return ERR_JOB_CMD;
    //сохраним текст команды в массив
    strncpy( jobp->command, cmd, MAX_LEN_JOBCMMND );
    //добавление задания завершено
    jobp->flags |= JOB_PRESENT;
    //вывод параметров задания в файл
    JobParam( jobp, str );
    //следующее задание
    ind_job++;
    return ERR_JOB_NONE;
 }

//*************************************************************************************************
// Очищаем весь список задач и очередь задач
//*************************************************************************************************
static void JobsClear( void ) {

    uint8_t ind;

    ind_job = 0;
    for ( ind = 0; ind < MAX_JOBS; ind++ )
        JobClear( &jobs[ind] );
    fdelete( parsing_log, NULL );
 }

//*************************************************************************************************
// Очищаем параметры одного задания
// JOB *JClr - указатель на структуру с заданием
//*************************************************************************************************
static void JobClear( JOB *JClr ) {

    NBitClear( JClr->seconds, 0, ( LAST_SECONDS - FIRST_SECONDS + 1 ) );
    NBitClear( JClr->minute,  0, ( LAST_MINUTE - FIRST_MINUTE + 1 ) );
    NBitClear( JClr->hour,    0, ( LAST_HOUR - FIRST_HOUR + 1 ) );
    NBitClear( JClr->day,     0, ( LAST_DAY - FIRST_DAY + 1 ) );
    NBitClear( JClr->month,   0, ( LAST_MONTH - FIRST_MONTH + 1 ) );
    NBitClear( JClr->dow,     0, ( LAST_DOW - FIRST_DOW + 1 ) );
    memset( JClr->command,    0, MAX_LEN_JOBCMMND );
    JClr->flags = 0;
 }

//*************************************************************************************************
// Заполнить биты в соответствии с параметрами
// bitstr_t *bits      - указатель на массив битов
// uint8_t low         - минимальное значение
// uint8_t high        - максимальное значение
// const char *names[] - массив допустимых имен
// char ch             - символ для обработки
// return              - следующий обрабатываемый символ
//*************************************************************************************************
static char GetList( bitstr_t *bits, uint8_t low, uint8_t high, const char *names[], char ch ) {

    bool done;

    //очищаем все биты в массиве bits
    NBitClear( bits, 0, ( high-low + 1 ) );
    done = false;
    while ( !done ) {
        ch = GetRange( bits, low, high, names, ch );
        if ( ch == ',' )
            ch = GetChar();
        else
            done = true;
       }
    SkipNonblanks( ch );
    SkipBlanks( ch );
    return ch;
 }

//*************************************************************************************************
// Проверка наличия диапазона или одиночного значения
// bitstr_t *bits      - указатель на массив битов
// uint8_t low         - минимальное значение
// uint8_t high        - максимальное значение
// const char *names[] - массив допустимых имен
// return              - последний обработанный символ или STR_END 
//*************************************************************************************************
static char GetRange( bitstr_t *bits, uint8_t low, uint8_t high, const char *names[], char ch ) {

    uint8_t i, num1, num2, num3;

    if ( ch == '*' ) {
        //'*' весь диапазон значений
        num1 = low;
        num2 = high;
        ch = GetChar();
        if ( ch == STR_END )
            return STR_END;
    } else {
        if ( STR_END == ( ch = GetNumber( &num1, low, names, ch ) ) )
            return STR_END;
        if ( ch != '-' ) {
            //одиночное число/номер
            if ( STR_END == SetElement( bits, low, high, num1 ) )
                return STR_END;
            return ch;
       } else {
            //есть диапазон значений
            ch = GetChar();
            if ( ch == STR_END )
                return STR_END;
            //получить номер, следующий за тире
            ch = GetNumber( &num2, low, names, ch );
            if ( ch == STR_END )
                return STR_END;
           }
       }
    //проверим размер шага
    if ( ch == '/' ) {
        //есть разделитель
        ch = GetChar();
        if ( ch == STR_END )
            return STR_END;
        //получить размер шага
        ch = GetNumber( &num3, 0, NULL, ch );
        if ( ch == STR_END )
            return STR_END;
        } else {
        //шаг по умолчанию = 1
        num3 = 1;
       }
    //диапазон, установить все элементы от num1 до num2, с шагом num3.
    for ( i = num1; i <= num2; i += num3 )
    if ( STR_END == SetElement( bits, low, high, i ) )
        return STR_END;
    return ch;
 }

//*************************************************************************************************
// Возвращает цифровое значение параметра, в т.ч. по символьному значению
// uint8_t *numptr     - указатель на значение
// uint8_t low         - минимальное значение
// const char *names[] - указатель на массив имен (день недели, месяц)
// char ch             - первый символ для обработки
// return              - последний обработанный символ или STR_END 
//*************************************************************************************************
static char GetNumber( uint8_t *numptr, uint8_t low, const char *names[], char ch ) {

    char temp[MAX_TEMPSTR], *pc;
    uint8_t len, i, all_digits;

    //соберем перечисление параметров в массив
    pc = temp;
    len = 0;
    all_digits = true;
    while ( isalnum( ch ) ) {
        if ( ++len >= MAX_TEMPSTR )
            return STR_END;
        *pc++ = ch;
        if ( !isdigit( ch ) )
            all_digits = false;
        ch = GetChar();
       }
    *pc = '\0';
    //пробуем найти в списке имен
    if ( names ) {
        for ( i = 0; names[i] != NULL; i++ ) {
            if ( !StrCaseCmp( names[i], temp ) ) {
                *numptr = i+low;
                return ch;
               }
           }
       }
    //нет указанного списка имен, или есть один
    if ( all_digits ) {
        //все остальные цифры
        *numptr = atoi( temp );
        return ch;
       }
    return STR_END;
 }

//*************************************************************************************************
// Установка бита в массиве bits
// bitstr_t *bits - указатель на массив битов 
// uint8_t low    - минимальное допустимое значение
// uint8_t high   - максимальное допустимое значение
// uint8_t number - номер бита для установки
// return = true  - бит установлен
//        = 0     - номер бита за пределами допустимых значений   
//*************************************************************************************************
static uint8_t SetElement( bitstr_t *bits, uint8_t low, uint8_t high, uint8_t number ) {

    if ( number < low || number > high )
        return STR_END;
    BitSet( bits, ( number - low ) );
    return true;
 }

//*************************************************************************************************
// Убираем в начале строки лишние символы, оставляем только допустимые
//*************************************************************************************************
static void SkipComments( void ) {

    char ch;

    while ( STR_END != ( ch = GetChar() ) ) {
        //пропускаем символы пробела и табуляции
        while ( ch == ' ' || ch == '\t' )
            ch = GetChar();
        if ( ch == STR_END )
            break;
        //значащий символ найден, выходим
        if ( ch != '\n' && ch != '\r' && ch != '#' && ch != ';' )
            break;
        //ch должен быть символом новой строки или комментарием как первый непустой символ в строке
        while ( ch != '\n' && ch != '\r' && ch != STR_END )
            ch = GetChar();
        //ch теперь является новой строкой строки, которую мы будем игнорировать
       }
    if ( ch != STR_END )
        UngetChar(); //вернем символ в буфер
 }

//*************************************************************************************************
// Возвращает следующий символ из буфера, после каждого чтения idx_read + 1
//*************************************************************************************************
static char GetChar( void ) {

    if ( buffer[idx_read] )
        return buffer[idx_read++];
    return STR_END;
 }

//*************************************************************************************************
// "Возвращает" символ в буфер, фактически уменьшаем индекс
//*************************************************************************************************
static void UngetChar( void ) {

    idx_read--;
 }

//*************************************************************************************************
// Сравнение двух строк без учета регистра
// const char *left - первая строка
// char *right      - вторая строка
// return = 0       - строки равны
//        > 0       - строки не равны
//*************************************************************************************************
static int StrCaseCmp( const char *left, char *right ) {

    while ( *left && ( MkLower( *left ) == MkLower( *right ) ) ) {
        left++;
        right++;
       }
    return MkLower( *left ) - MkLower( *right );
 }

//*************************************************************************************************
// Копирует строку в промежуточный буфер
// char *string - указатель на промежуточный буфер для копирования
// uint8_t size - размер исходной строки
// char *terms  - массив разделителей исключаемых при копировании
// return       - последний скопированный символ
//*************************************************************************************************
static uint8_t GetString( char *string, uint8_t size, char *terms ) {

    char ch;

    while ( STR_END != ( ch = GetChar() ) && !strchr( terms, ch ) ) {
        if ( size > 1 ) {
            *string++ = ch;
            size--;
           }
      }
    if ( size > 0 )
        *string = '\0';
    return ch;
 }

//*************************************************************************************************
// Расшифровка параметров задания, вывод масок выполнения в файл
// JOB *Jobp    - структура задания
// char *source - исходная строка задания
//*************************************************************************************************
static void JobParam( JOB *Jobp, char *source ) {

    uint8_t i;
    FILE *pars_log;

    //запись результата разбора команды
    pars_log = fopen( parsing_log, "a" );
    if ( pars_log == NULL )
        return;
    fprintf( pars_log, "%s", Message( CONS_MSG_JOB_HDR1 ) );
    fprintf( pars_log, "   Job: %s\r\n", source );
    fprintf( pars_log, "%s", Message( CONS_MSG_JOB_HDR2 ) );
    fprintf( pars_log, "%s", Message( CONS_MSG_JOB_HDR3 ) );
    fprintf( pars_log, "%s", Message( CONS_MSG_JOB_HDR4 ) );
    //маска выполнения для секунд
    fprintf( pars_log, "Second: " );
    for ( i = 0; i < SECONDS_COUNT; i++ ) {
        if ( BitTest( Jobp->seconds, i ) )
            fprintf( pars_log, "1" );
        else fprintf( pars_log, "0" );
       }
    fprintf( pars_log, "%s", Message( CONS_MSG_CRLF ) );
    //маска выполнения для минут
    fprintf( pars_log, "Minute: " );
    for ( i = 0; i < MINUTE_COUNT; i++ ) {
        if ( BitTest( Jobp->minute, i ) )
            fprintf( pars_log, "1" );
        else fprintf( pars_log, "0" );
       }
    fprintf( pars_log, "%s", Message( CONS_MSG_CRLF ) );
    //маска выполнения для часов
    fprintf( pars_log, "  Hour: " );
    for ( i = 0; i < HOUR_COUNT; i++ ) {
        if ( BitTest( Jobp->hour, i ) )
            fprintf( pars_log, "1" );
        else fprintf( pars_log, "0" );
       }
    fprintf( pars_log, "%s", Message( CONS_MSG_CRLF ) );
    //маска выполнения для даты
    fprintf( pars_log, "   Day: " );
    for ( i = 0; i < DAY_COUNT; i++ ) {
        if ( BitTest( Jobp->day, i ) )
            fprintf( pars_log, "1" );
        else fprintf( pars_log, "0" );
       }
    fprintf( pars_log, "%s", Message( CONS_MSG_CRLF ) );
    //маска выполнения для месяца
    fprintf( pars_log, " Month: " );
    for ( i = 0; i < MONTH_COUNT; i++ ) {
        if ( BitTest( Jobp->month, i ) )
            fprintf( pars_log, "1" );
        else fprintf( pars_log, "0" );
       }
    fprintf( pars_log, "%s", Message( CONS_MSG_CRLF ) );
    //маска выполнения для для недели
    fprintf( pars_log, "   Dow: " );
    for ( i = 0; i < DOW_COUNT; i++ ) {
        if ( BitTest( Jobp->dow, i ) )
            fprintf( pars_log, "1" );
        else fprintf( pars_log, "0" );
       }
    fprintf( pars_log, "\r\n  Cmnd: %s\r\n\r\n", Jobp->command );
    fclose( pars_log );
 }

//*************************************************************************************************
// Удаление комментария и пробелов от знака комментария ";" до первого значащего символа
// char *buffer - указатель на исходную строку
//*************************************************************************************************
static void CommentSpace( char *buffer ) {

    char *del;
    
    del = strchr( buffer, ';' );
    if ( del == NULL )
        return;
    do {
        *( del ) = '\0';
        del--;
       } while ( *del == 0x20 );
    strcat( buffer, "\r\n" );
 }

//*************************************************************************************************
// Удаление ведущих пробелов в строке
// char *buffer - указатель на исходную строку
//*************************************************************************************************
static void LeadSpace( char *buffer ) {

    char ch = '\0', *ptr;
    
    if ( *buffer > 0x20 )
        return;
    ptr = buffer;
    while ( *ptr++ ) {
        if ( *ptr <= 0x20 && !ch )
            continue;
        ch = 1;
        *buffer++ = *ptr++;
       }
    *buffer = '\0';
 }

//*************************************************************************************************
// Возвращает текст ошибки при разборе параметров задания
//*************************************************************************************************
static const char *JobGetError( uint8_t error ) {

    return err_desc[error];
 }
