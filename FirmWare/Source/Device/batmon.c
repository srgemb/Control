
//*************************************************************************************************
//
// Обработка данных монитора АКБ BMV-600S
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "cmsis_os2.h"

#include "uart_lpc17xx.h"
#include "driver_usart.h"
#include "lpc177x_8x.h"

#include "device.h"
#include "dev_param.h"
#include "dev_data.h"

#include "main.h"
#include "rtc.h"
#include "alt.h"
#include "gen.h"
#include "inverter.h"
#include "config.h"
#include "mppt.h"
#include "batmon.h"
#include "eeprom.h"
#include "charger.h"
#include "sdcard.h"
#include "message.h"
#include "ports.h"
#include "informing.h"
#include "priority.h"
#include "gen_ext.h"
#include "events.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern ARM_DRIVER_USART Driver_USART1;

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
BATMON batmon;
osEventFlagsId_t batmon_event = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define BATMON_RECV_BUFF    250             //размер приемного буфера для приема данных от BMV-600S
#define TIME_PAUSE          200             //длительность паузы в приеме для проверки принятых данных (msec)
#define TIME_PAUSE_MAX      1500            //максимальное время в пределах которого ожидается прием данных
                                            //от BMV-600S, если данных нет - нет связи с монитором (msec)
//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static ARM_DRIVER_USART *USARTdrv;
static uint8_t recv_ind = 0;
static char recv_ch, recv_buffer[BATMON_RECV_BUFF];
static osTimerId_t timer_bmon1, timer_bmon2, timer_bmon3;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t batmon_attr = {
    .name = "Batmon", 
    .stack_size = 1024,
    .priority = osPriorityNormal
 };
 
static const osEventFlagsAttr_t evn_attr = { .name = "Batmon" };
static const osTimerAttr_t timer1_attr = { .name = "BmonPause1" };
static const osTimerAttr_t timer2_attr = { .name = "BmonPause2" };
static const osTimerAttr_t timer3_attr = { .name = "BmonLog" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void CallBackUsart( uint32_t event );
static void RecvClear( void );
static uint8_t DataParse( char *data );
static void DataClear( void );
static void SaveLog( void );
static void DayLog( void );

static void Timer1Callback( void *arg );
static void Timer2Callback( void *arg );
static void Timer3Callback( void *arg );
static void TaskBatmon( void *pvParameters );

//*************************************************************************************************
// Инициализация параметров и портов
//*************************************************************************************************
void BatMonInit( void ) {

    RecvClear();
    DataClear();
    //очередь сообщений
    batmon_event = osEventFlagsNew( &evn_attr );
    //таймер паузы между пакетами
    timer_bmon1 = osTimerNew( Timer1Callback, osTimerOnce, NULL, &timer1_attr );
    //таймер отсутствия связи
    timer_bmon2 = osTimerNew( Timer2Callback, osTimerOnce, NULL, &timer2_attr );
    //таймер интервальной записи данных
    timer_bmon3 = osTimerNew( Timer3Callback, osTimerOnce, NULL, &timer3_attr );
    //создаем задачу
    osThreadNew( TaskBatmon, NULL, &batmon_attr );
    //настройка последовательного порта
    USARTdrv = &Driver_USART1;
    USARTdrv->Initialize( &CallBackUsart );
    NVIC_SetPriority( UART1_IRQn, NVIC_EncodePriority( NVIC_GetPriorityGrouping(), PRIORITY_USART, SUB_PRIORITY_USART1 ) );
    USARTdrv->PowerControl( ARM_POWER_FULL );
    USARTdrv->Control( ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | ARM_USART_PARITY_NONE | 
                       ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE, 19200 );
    USARTdrv->Control( ARM_USART_CONTROL_RX, 1 );    
    USARTdrv->Receive( &recv_ch, 1 );
 }

//*************************************************************************************************
// Обработчик событий последовательного порта
//*************************************************************************************************
static void CallBackUsart( uint32_t event ) {

    if ( event & ARM_USART_EVENT_RECEIVE_COMPLETE ) {
        //принят один байт
        recv_ind += USARTdrv->GetRxCount();
        if ( recv_ind >= sizeof( recv_buffer ) )
            RecvClear(); //буфер переполнен, сбросим буфер
        else recv_buffer[recv_ind-1] = toupper( recv_ch ); //сохраним байт в буфере
        //повторная инициализация приема
        USARTdrv->Receive( &recv_ch, 1 );
        //событие для перезапуска таймеров ожидания паузы между пакетами данных
        osEventFlagsSet( batmon_event, EVN_BATMON_RECV );
       } 
 }

//*************************************************************************************************
// Задача обработки событий
//*************************************************************************************************
static void TaskBatmon( void *pvParameters ) {
    
    uint32_t send, event;
    
    for ( ;; ) {
        //запуск таймера интервальной записи данных
        if ( !osTimerIsRunning( timer_bmon3 ) )
            osTimerStart( timer_bmon3, config.datlog_upd_bmon * SEC_TO_TICK );
        //ждем события
        event = osEventFlagsWait( batmon_event, EVN_BATMON_MASK, osFlagsWaitAny, osWaitForever );
        if ( event & EVN_BATMON_RECV ) {
            //данные получены, перезапуск таймеров ожидания паузы между пакетами данных
            osTimerStart( timer_bmon1, TIME_PAUSE );
            osTimerStart( timer_bmon2, TIME_PAUSE_MAX );
           }
        if ( event & EVN_BATMON_PAUSE1 ) {
            //пауза между пакетами данных
            //разбор параметров пакета данных
            if ( DataParse( recv_buffer ) )
                batmon.link = LINK_CONN_OK;
            else batmon.link = LINK_CONN_NO;
            RecvClear();
            send = ID_DEV_BATMON; //передача данных в HMI
            osMessageQueuePut( hmi_msg, &send, 0, 0 );
           }
        if ( event & EVN_BATMON_PAUSE2 ) {
            //вышло время ожидания пакета данных
            RecvClear();
            DataClear();
           }
        if ( event & EVN_RTC_SECONDS ) {
            DayLog(); //сохранение суточных данных
            if ( batmon.link == LINK_CONN_NO ) {
                send = ID_DEV_BATMON; //передача данных в HMI
                osMessageQueuePut( hmi_msg, &send, 0, 0 );
               }
           }
        if ( event & EVN_BATMON_LOG ) {
            SaveLog(); //интервальное сохранение данных
            osTimerStart( timer_bmon3, config.datlog_upd_bmon * SEC_TO_TICK );
           }
       }
 }

//*************************************************************************************************
// Функция обратного вызова таймера - пауза между пакетами данных
//*************************************************************************************************
static void Timer1Callback( void *arg ) {

    osEventFlagsSet( batmon_event, EVN_BATMON_PAUSE1 );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - отсутствие обмена данными
//*************************************************************************************************
static void Timer2Callback( void *arg ) {

    osEventFlagsSet( batmon_event, EVN_BATMON_PAUSE2 );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - интервальная запись данных
//*************************************************************************************************
static void Timer3Callback( void *arg ) {

    osEventFlagsSet( batmon_event, EVN_BATMON_LOG );
 }

//*************************************************************************************************
// Очистка буфера приема
//*************************************************************************************************
static void RecvClear( void ) {

    recv_ind = 0;
    memset( recv_buffer, 0, sizeof( recv_buffer ) );
 }

//*************************************************************************************************
// Разбор принятых данных и заполнение переменных значениями
// возвращает кол-во обработанных параметров
// char *data - исходная строка с данными
// result     - кол-во найденных параметров
//*************************************************************************************************
static uint8_t DataParse( char *data ) {

    uint8_t ind, par_len, val_len, parse_cnt = 0;
    char *find, *end;
    char param[10], value[40]; //значения параметра в строковом виде

    //обработка данных
    for ( ind = 0; ind < DevParamCnt( ID_DEV_BATMON, CNT_FULL ); ind++ ) {
        //формируем имя параметра для поиска и проверки
        sprintf( param, "\r\n%s\t", ParamGetName( ID_DEV_BATMON, ind ) );
        //найдем параметр по имени
        find = strstr( data, param );
        if ( find == NULL )
            continue; //параметра нет
        //параметр найден
        parse_cnt++;
        //размер имени параметра
        par_len = strlen( param );
        //пропускаем имя параметра с служебными символами
        find += par_len; 
        //адрес окончания строки
        end = strstr( find, "\r\n" );
        if ( end == NULL )
            continue;
        //сохраним исходное значение параметра, 
        val_len = end - find;
        memset( value, 0x00, sizeof( value ) );
        strncpy( value, find, val_len );
        //заполняем параметры структуры фактическими значениями
        if ( ind == MON_VOLTAGE )
            batmon.voltage = atof( value )/1000;     //напряжение АКБ (mV->V)
        if ( ind == MON_CURRENT )
            batmon.current = atof( value )/1000;     //ток АКБ (mA->A)
        if ( ind == MON_CONSUMENERGY )
            batmon.cons_energy = atof( value )/1000; //израсходованная энергия от АКБ (mAh->Ah)
        if ( ind == MON_SOC )
            batmon.soc = atof( value )/10;           //состояние заряда АКБ
        if ( ind == MON_TTG ) {
            batmon.ttg = atoi( value );              //продолжительность работы
            if ( batmon.ttg < 0 )
                batmon.ttg = 0;
           }
        if ( ind == MON_ALARM )
           batmon.alarm = ( strcasecmp( value, "ON" ) == NULL ? 1 : 0 ); //ON/OFF состояние сигнала
        if ( ind == MON_RELAY )
           batmon.relay = ( strcasecmp( value, "ON" ) == NULL ? 1 : 0 ); //ON/OFF состояние реле
        if ( ind == MON_ALARMMODE )
           batmon.alarm_mode = atoi( value );  //состояние сигнализации
        if ( ind == MON_MODEL )
           strcpy( batmon.model, value );      //модель монитора
        if ( ind == MON_VERSION )
           strcpy( batmon.version, value );    //версия прошивки
        if ( ind == MON_H1 )
           batmon.h1 = atof( value )/1000;     //глубина самого глубокого разряда (mAh->Ah)
        if ( ind == MON_H2 )
           batmon.h2 = atof( value )/1000;     //глубина последнего разряда (mAh->Ah)
        if ( ind == MON_H3 )
           batmon.h3 = atof( value )/1000;     //глубина среднего разряда (mAh->Ah)
        if ( ind == MON_H4 )
           batmon.h4 = atoi( value );          //число циклов заряда
        if ( ind == MON_H5 )
           batmon.h5 = atoi( value );          //число полных разрядов
        if ( ind == MON_H6 )
           batmon.h6 = atof( value )/1000;     //совокупное значение Ah полученное от АКБ (mAh->Ah)
        if ( ind == MON_H7 )
           batmon.h7 = atof( value )/1000;     //минимальное напряжение АКБ (mV->V)
        if ( ind == MON_H8 )
           batmon.h8 = atof( value )/1000;     //максимальное напряжение АКБ (mV->V)
        if ( ind == MON_H9 )
           batmon.h9 = atoi( value )/60/60/24; //число дней с момента последнего полного заряда
        if ( ind == MON_H10 )
           batmon.h10 = atoi( value );         //Кол-во автоматических синхронизаций
        if ( ind == MON_H11 )
           batmon.h11 = atoi( value );         //Кол-во аварийных сигналов низкого напряжения
        if ( ind == MON_H12 )
           batmon.h12 = atoi( value );         //Кол-во аварийных сигналов высокого напряжения
       }
    return parse_cnt;
 }

//*************************************************************************************************
// Обнуление данных монитора АКБ
//*************************************************************************************************
static void DataClear( void ) {

    memset( (uint8_t *)&batmon, 0x00, sizeof( batmon ) );
 }

//*************************************************************************************************
// Добавляет в протокол "bm_yyyymmdd.csv" данные монитора АКБ
// Логирование ведется при включенном параметре "log_enable_bmon" 
// с интервалом времени по параметру "datlog_upd_bmon"
//*************************************************************************************************
static void SaveLog( void ) {

    uint32_t pos;
    char name[80];
    FILE *bm_log;

    if ( batmon.link == LINK_CONN_NO )
        return; //данных нет
    if ( !config.log_enable_bmon )
        return; //логирование выключено
    if ( SDStatus() == ERROR )
        return; //карты нет
    //открываем файл
    if ( !config.mode_logging )
        sprintf( name, "\\batmon\\bm_%s.csv", RTCFileName() );
    else sprintf( name, "\\batmon\\%s\\bm_%s.csv", RTCFileShort(), RTCFileName() );
    bm_log = fopen( name, "a" );
    if ( bm_log == NULL )
        return; //файл не открылся
    pos = ftell( bm_log );
    //запишем наименование полей
    if ( !pos )
        fprintf( bm_log, "Date;Time;Bat_V(V);Bat_I(A);Energy from BAT(Ah);SOC(%%);TTGo;Total energy from BAT(Ah);Alarm;Relay;Last discharge(Ah);Medium discharge(Ah)\r\n" );
    //запишем данные
    fprintf( bm_log, "%s;%s;%.2f;%+.2f;%+.2f;%.1f;%3d:%02d;%.2f;%u;%u;%5.2f;%5.2f\r\n", RTCGetDate( NULL ), RTCGetTime( NULL ), 
            batmon.voltage, batmon.current, batmon.cons_energy, batmon.soc, BatMonTTG( BATMON_TTG_HOUR ), BatMonTTG( BATMON_TTG_MIN ), batmon.h6,
            batmon.alarm, batmon.relay, batmon.h2, batmon.h3 ); 
    fclose( bm_log );
 }

//*************************************************************************************************
// Добавляет в протокол "bm_yyyymm.csv" суточные данные монитора АКБ
//*************************************************************************************************
static void DayLog( void ) {

    uint32_t pos;
    char name[80];
    FILE *time_log;
    RTC_TIME_Type Time;

    if ( !config.log_enable_bmon )
        return; //логирование выключено
    if ( SDStatus() == ERROR )
        return; //карты нет
    RTC_GetFullTime( LPC_RTC, &Time );
    if ( Time.HOUR != 23 || Time.MIN != 59 || Time.SEC != 59 )
        return;
    //формируем имя файла
    sprintf( name, "\\batmon\\bm_%s.csv", RTCFileShort() );
    time_log = fopen( name, "a" );
    if ( time_log == NULL )
        return; //файл не открылся
    pos = ftell( time_log );
    if ( !pos )
        fprintf( time_log, "Date;Bat_V(V);Energy(Ah);SOC(%%);H6(Ah)\r\n" ); //запишем наименование полей
    //запишем данные
    fprintf( time_log, "%s;%.2f;%+.2f;%.1f;%.2f\r\n", RTCGetDate( NULL ), batmon.voltage, batmon.cons_energy, batmon.soc, batmon.h6 ); 
    fclose( time_log );
 }

//*************************************************************************************************
// Возвращает значения параметра TTG, продолжительность работы в часах/минутах
// BatMonTTGTime type - тип результата: часы/минуты 
// return uint16_t    - значение параметра
//*************************************************************************************************
uint16_t BatMonTTG( BatMonTTGTime type ) {

    if ( type == BATMON_TTG_MIN )
        return batmon.ttg%60;
    if ( type == BATMON_TTG_HOUR )
        return (uint16_t)batmon.ttg/60;
    return 0;
 }
