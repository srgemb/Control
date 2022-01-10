
//*************************************************************************************************
//
// Обработка данных контроллера заряда MPPT ProSolar SS-50C
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <lpc_types.h>

#include "cmsis_os2.h"

#include "uart_lpc17xx.h"
#include "driver_usart.h"

#include "device.h"
#include "dev_data.h"

#include "main.h"
#include "outinfo.h"
#include "ports.h"
#include "eeprom.h"
#include "sdcard.h"
#include "message.h"
#include "command.h"
#include "pv.h"
#include "rtc.h"
#include "mppt.h"
#include "informing.h"
#include "priority.h"
#include "events.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern ARM_DRIVER_USART Driver_USART4;

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
MPPT mppt;
osEventFlagsId_t mppt_event = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define MPPT_RECV_BUFF      64              //размер буфера приема данных от MPPT контроллера
#define TIME_PAUSE          500             //длительность паузы в приеме для проверки принятых данных (msec)
#define TIME_PAUSE_MAX      2000            //максимальное время в пределах которого ожидается прием данных
                                            //от MPPT, если данных нет - нет связи с контроллером (msec)

static const uint8_t header[] = { 0x73, 0x00, 0xFF, 0x35 }; //заголовок пакета данных
static const uint8_t tail[] = { 0x06, 0x0A, 0x49 };         //хвост пакета данных

#pragma pack( push, 1 )

//структура для пакета данных контроллера MPPT
//последовательность и размерность полей менять нельзя
typedef struct {
    uint8_t     header[4];                  //заголовок пакета
    uint16_t    u03_out_voltage;            //(V) выходное напряжение
    uint16_t    u01_in_voltage;             //(V) напряжение панелей
    uint16_t    u04_out_current;            //(A) выходной ток
    int16_t     u13_bat_current;            //(A) ток АКБ
    uint8_t     u12_soc;                    //(%) уровень заряда АКБ
    uint16_t    u15_bat_temp;               //(С) температура АКБ
    uint16_t    u11_mppt_temp;              //(С) температура контроллера
    uint8_t     unused2[6];
    uint16_t    time_charge;                //счетчик времени заряда (сколько идет заряд)
    uint8_t     unused3;
    uint8_t     u08_charge_mode;            //режим зарядки
    uint16_t    u14_bat_capasity;           //(AHr) емкость АКБ
    uint8_t     unused4[8];
    uint32_t    u05_energy2;                //(AHr) собранная энергия сегодня
    uint32_t    u05_energy1;                //(WHr) собранная энергия сегодня
    uint16_t    u07_time_flt;               //(min) время контроллера в режиме "поддержки" сегодня
    uint16_t    u17_serial;                 //серийный номер контроллера
    uint8_t     tail[3];                    //хвост пакета
    uint8_t     crc;                        //контрольная сумма
 } PACK;

#pragma pack( pop )

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static PACK pack;
static uint8_t recv_ind = 0;
static ARM_DRIVER_USART *USARTdrv;
static uint8_t recv_ch, recv_buffer[MPPT_RECV_BUFF];
static osTimerId_t timer_mppt1, timer_mppt2, timer_mppt3;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t mppt_attr = {
    .name = "Mppt", 
    .stack_size = 1280,
    .priority = osPriorityNormal 
 };
 
static const osEventFlagsAttr_t evn_attr = { .name = "Mppt" };
static const osTimerAttr_t timer1_attr = { .name = "MpptPause1" };
static const osTimerAttr_t timer2_attr = { .name = "MpptPause2" };
static const osTimerAttr_t timer3_attr = { .name = "MpptLog" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void CallBackUsart( uint32_t event );
static Status ParseData( uint8_t *data );
static void RecvClear( void );
static void DataClear( void );
static void SaveLog( void );
static MpptConn MpptCheckConn( void );
static void Timer1Callback( void *arg );
static void Timer2Callback( void *arg );
static void Timer3Callback( void *arg );
static void TaskMppt( void *pvParameters );

//*************************************************************************************************
// Инициализация параметров и портов 
// USART4/IRQ
//*************************************************************************************************
void MpptInit( void ) {

    DataClear();
    RecvClear();
    //очередь сообщений
    mppt_event = osEventFlagsNew( &evn_attr );
    //таймер паузы между пакетами
    timer_mppt1 = osTimerNew( Timer1Callback, osTimerOnce, NULL, &timer1_attr );
    //таймер отсутствия связи
    timer_mppt2 = osTimerNew( Timer2Callback, osTimerOnce, NULL, &timer2_attr );
    //таймер интервальной записи данных
    timer_mppt3 = osTimerNew( Timer3Callback, osTimerOnce, NULL, &timer3_attr );
    //создаем задачу
    osThreadNew( TaskMppt, NULL, &mppt_attr );
    //настройка портов
    GPIO_SetDir( MPPT_PORT, MPPT_CTRL, GPIO_DIR_OUTPUT );
    GPIO_PinWrite( MPPT_PORT, MPPT_CTRL, RS485_RECV );
    //настройка последовательного порта
    USARTdrv = &Driver_USART4;
    USARTdrv->Initialize( &CallBackUsart );
    NVIC_SetPriority( UART4_IRQn, NVIC_EncodePriority( NVIC_GetPriorityGrouping(), PRIORITY_USART, SUB_PRIORITY_USART4 ) );
    USARTdrv->PowerControl( ARM_POWER_FULL );
    USARTdrv->Control( ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | ARM_USART_PARITY_NONE | 
                       ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE, 9600 );
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
        else recv_buffer[recv_ind-1] = recv_ch; //сохраним байт в буфере
        //повторная инициализация приема
        USARTdrv->Receive( &recv_ch, 1 );
        //событие для перезапуска таймера ожидания паузы между пакетами данных
        osEventFlagsSet( mppt_event, EVN_MPPT_RECV );
       }
 }

//*************************************************************************************************
// Задача обработки событий
//*************************************************************************************************
static void TaskMppt( void *pvParameters ) {
    
    uint32_t send, event;
    
    for ( ;; ) {
        //запуск таймера интервальной записи данных
        if ( !osTimerIsRunning( timer_mppt3 ) )
            osTimerStart( timer_mppt3, config.datlog_upd_mppt * SEC_TO_TICK );
        //ждем события
        event = osEventFlagsWait( mppt_event, EVN_MPPT_MASK, osFlagsWaitAny, osWaitForever );
        if ( event & EVN_MPPT_RECV ) {
            //данные получены, перезапуск таймера ожидания паузы между пакетами данных
            osTimerStart( timer_mppt1, TIME_PAUSE );
            osTimerStart( timer_mppt2, TIME_PAUSE_MAX );
           }
        if ( event & EVN_MPPT_PAUSE1 ) {
            //пауза между пакетами данных
            //разбор параметров пакета данных, проверим размер принятого пакета              
            if ( recv_ind == sizeof( pack ) ) {
                if ( ParseData( recv_buffer ) == ERROR )
                    DataClear();
                else mppt.link = LINK_CONN_OK;
                //разбор данных завершен
                RecvClear();
               }
            else RecvClear();
            send = ID_DEV_MPPT; //передача данных в HMI
            osMessageQueuePut( hmi_msg, &send, 0, 0 );
           }
        if ( event & EVN_MPPT_PAUSE2 ) {
            //вышло время ожидания пакета данных
            RecvClear();
            DataClear();
           }
        if ( event & EVN_RTC_SECONDS ) {
            //состояние подключения контроллера MPPT
            mppt.power = MpptCheckPwr();
            mppt.connect = MpptCheckConn();
            mppt.pv_stat = PvGetStat();
            mppt.pv_mode = PvGetMode();
            if ( mppt.link == LINK_CONN_NO ) {
                send = ID_DEV_MPPT; //передача данных в HMI
                osMessageQueuePut( hmi_msg, &send, 0, 0 ); 
               }
           }
        if ( event & EVN_MPPT_LOG ) {
            SaveLog(); //запись интервальных данных
            osTimerStart( timer_mppt3, config.datlog_upd_mppt * SEC_TO_TICK );
           }
       }
 }

//*************************************************************************************************
// Функция обратного вызова таймера - паузы между пакетами данных
//*************************************************************************************************
static void Timer1Callback( void *arg ) {

    osEventFlagsSet( mppt_event, EVN_MPPT_PAUSE1 );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - отсутствия обмена данными
//*************************************************************************************************
static void Timer2Callback( void *arg ) {

    osEventFlagsSet( mppt_event, EVN_MPPT_PAUSE2 );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - интервальная запись данных
//*************************************************************************************************
static void Timer3Callback( void *arg ) {

    osEventFlagsSet( mppt_event, EVN_MPPT_LOG );
 }

//*************************************************************************************************
// Очистка буфера приема
//*************************************************************************************************
static void RecvClear( void ) {

    recv_ind = 0;
    memset( recv_buffer, 0x00, sizeof( recv_buffer ) );
 }

//*************************************************************************************************
// Возвращает статус подключения контроллера MPPT к АКБ
// return MpptPower - состояние подключения
//*************************************************************************************************
MpptPower MpptCheckPwr( void ) {

    if ( GetDataPort( MPPT_ON ) )
        return MPPT_POWER_ON;
    else return MPPT_POWER_OFF;
 }

//*************************************************************************************************
// Проверка подключения кабеля связи контроллера MPPT к управляющему контроллеру
// return MpptConn - состояние подключения
//*************************************************************************************************
static MpptConn MpptCheckConn( void ) {

    if ( GetDataPort( MPPT_CONN ) )
        return MPPT_CONN_ON;
    else return MPPT_CONN_OFF;
 }

//*************************************************************************************************
// Проверка и разбор пакет данных контроллера MPPT
// char *data    - указатель на буфер данных
// return Status - результат проверки пакета данных
//*************************************************************************************************
static Status ParseData( uint8_t *data ) {

    memcpy( (uint8_t *)&pack, data, sizeof( pack ) );
    if ( memcmp( (uint8_t *)&pack.header, header, sizeof( pack.header ) ) != 0 || memcmp( (uint8_t *)&pack.tail, tail, sizeof( pack.tail ) ) != 0 )
        return ERROR;
    mppt.u01_in_voltage = ( (float)pack.u01_in_voltage ) / 10;
    mppt.u03_out_voltage = ( (float)pack.u03_out_voltage ) / 10;
    mppt.u04_out_current = ( (float)pack.u04_out_current ) / 10;
    //ток панелей рассчитываем
    mppt.u02_in_current = ( mppt.u03_out_voltage * mppt.u04_out_current ) / mppt.u01_in_voltage;
    mppt.u05_energy1 = pack.u05_energy1;
    mppt.u05_energy2 = pack.u05_energy2;
    mppt.u07_time_flt = pack.u07_time_flt;
    mppt.u08_charge_mode = (MpptCharge)pack.u08_charge_mode;
    mppt.u11_mppt_temp = ( (float)pack.u11_mppt_temp ) / 10;
    if ( pack.u12_soc > 100 )
        mppt.u12_soc = 0;
    else mppt.u12_soc = pack.u12_soc;
    mppt.u13_bat_current = ( (float)pack.u13_bat_current ) / 10;
    mppt.u14_bat_capasity = pack.u14_bat_capasity;
    mppt.u15_bat_temp = ((float)pack.u15_bat_temp ) / 10;
    mppt.u17_serial = pack.u17_serial;
    mppt.time_charge = pack.time_charge;
    return SUCCESS;
 }

//*************************************************************************************************
// Обнуление данных контроллера MPPT
//*************************************************************************************************
static void DataClear( void ) {

    memset( (uint8_t *)&pack, 0x00, sizeof( pack ) );
    mppt.u01_in_voltage = 0;
    mppt.u02_in_current = 0;
    mppt.u03_out_voltage = 0;
    mppt.u04_out_current = 0;
    mppt.u05_energy1 = 0;
    mppt.u05_energy2 = 0;
    mppt.u07_time_flt = 0;
    mppt.u08_charge_mode = MPPT_CHARGE_OFF;
    mppt.u11_mppt_temp = 0;
    mppt.u12_soc = 0;
    mppt.u13_bat_current = 0;
    mppt.u14_bat_capasity = 0;
    mppt.u15_bat_temp = 0;
    mppt.u17_serial = 0;
    mppt.time_charge = 0;
    mppt.link = LINK_CONN_NO;
 }

//*************************************************************************************************
// Добавляет в протокол "mppt_yyyymmdd.csv" данные контроллера заряда MPPT
//*************************************************************************************************
static void SaveLog( void ) {

    uint8_t *data;
    uint32_t ind, pos;
    char name_csv[80], name_hex[80];
    FILE *mppt_log;

    if ( mppt.link == LINK_CONN_NO )
        return; //данных нет
    if ( !config.log_enable_mppt )
        return; //логирование выключено
    if ( SDStatus() == ERROR )
        return; //карты нет
    //открываем файл для записи данных
    if ( !config.mode_logging ) {
        sprintf( name_csv, "\\mppt\\mppt_%s.csv", RTCFileName() );
        sprintf( name_hex, "\\mppt\\mppt_%s.hex", RTCFileName() );
       }
    else {
        sprintf( name_csv, "\\mppt\\%s\\mppt_%s.csv", RTCFileShort(), RTCFileName() );
        sprintf( name_hex, "\\mppt\\%s\\mppt_%s.hex", RTCFileShort(), RTCFileName() );
       }
    mppt_log = fopen( name_csv, "a" );
    if ( mppt_log == NULL )
        return; //файл не открылся
    pos = ftell( mppt_log );
    if ( !pos )
        fprintf( mppt_log, "Date;Time;PV_V(V);PV_I(A);OUT_V(V);OUT_I(A);WHr;AHr;Float;ModeCharge;SOC(%%);Bat_I(A);PVOn;PVMode\r\n" ); //запишем наименование полей
    //запишем данные
    fprintf( mppt_log, "%s;%s;%.1f;%.1f;%.1f;%.1f;%d;%d;%d;%s;%03d;%+.1f;%s;%s\r\n", RTCGetDate( NULL ), RTCGetTime( NULL ),
        mppt.u01_in_voltage, mppt.u02_in_current, mppt.u03_out_voltage, mppt.u04_out_current, mppt.u05_energy1,
        mppt.u05_energy2, mppt.u07_time_flt, ParamGetDesc( ID_DEV_MPPT, MPPT_CHARGE_MODE ),
        mppt.u12_soc, mppt.u13_bat_current, ParamGetDesc( ID_DEV_MPPT, MPPT_PVON ), ParamGetDesc( ID_DEV_MPPT, MPPT_PVMODE ) ); 
    fclose( mppt_log );
    //запись всех данных пакета MPPT в HEX формате
    mppt_log = fopen( name_hex, "a" );
    if ( mppt_log == NULL )
        return; //файл не открылся
    pos = ftell( mppt_log );
    if ( !pos ) {
        //запишем шапку для данных
        fputs( Message( CONS_MSG_MPPT_HEADER1 ), mppt_log );
        fputs( Message( CONS_MSG_MPPT_HEADER2 ), mppt_log );
        fputs( Message( CONS_MSG_MPPT_HEADER3 ), mppt_log );
        fputs( Message( CONS_MSG_MPPT_HEADER4 ), mppt_log );
        fputs( Message( CONS_MSG_MPPT_HEADER5 ), mppt_log );
       }
    //запишем данные в HEX формате
    fprintf( mppt_log, "%s %s ", RTCGetDate( NULL ), RTCGetTime( NULL ) );
    data = (uint8_t *)&pack;
    for ( ind = 0; ind < sizeof( pack ); ind++ )
        fprintf( mppt_log, "%02X ", *( data + ind ) );
    fputs( Message( CONS_MSG_CRLF ), mppt_log );
    fclose( mppt_log );
 }
