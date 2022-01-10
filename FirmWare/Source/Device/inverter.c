
//*************************************************************************************************
//
// Управление инверторами TS-1000-224, TS-3000-224
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "driver_usart.h"

#include "device.h"
#include "dev_data.h"

#include "main.h"
#include "rtc.h"
#include "ports.h"
#include "eeprom.h"
#include "sound.h"
#include "sdcard.h"
#include "command.h"
#include "informing.h"
#include "priority.h"
#include "inverter.h"
#include "inverter_def.h"
#include "message.h"
#include "events.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern ARM_DRIVER_USART Driver_USART2, Driver_USART3;

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osEventFlagsId_t inv1_event = NULL, inv2_event = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define TS_SEND_BUFF            40          //размер передающего буфера
#define TS_RECV_BUFF            200         //размер приемного буфера

#define TIME_SEND_COMMNAD       500         //интервал отправки команд инвертору (msec)
#define TIMEOUT_ANSWER          300         //время ожидания ответа инвертора
#define TIME_CHK_CONTACTOR      500         //задержка перед проверкой включения контактора питания (msec)
#define TIME_WAIT_ANSWER        2000        //время ожидания ответа инвертора (msec)
#define TIME_WAIT_POWER_ON      4000        //ожидание включения инвертора (msec)
#define TIME_WAIT_POWER_OFF     5000        //ожидание выключения инвертора (msec)
#define TIME_NO_WAIT            0           //таймер задержки выполнения команды не включаем
#define TIME_NEXT_STEP          5           //таймер запускается 5 тактов ОС для вызова следующего шага

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
INVERTER inv1, inv2;
static ARM_DRIVER_USART *UsartInv1, *UsartInv2;

static InvCtrlCmnd control1 = INV_CTRL_OFF;         //команда вкл/выкл инвертора TS-1000-224
static InvCtrlCmnd control2 = INV_CTRL_OFF;         //команда вкл/выкл инвертора TS-3000-224
static InvCommand last_cmnd1 = INV_CMD_NULL;        //последняя команда отправленная инвертору TS-1000-224
static InvCommand last_cmnd2 = INV_CMD_NULL;        //последняя команда отправленная инвертору TS-3000-224

static bool manually1 = false, manually2 = false;   //контроль ручного вкл/выкл инверторов

static uint8_t recv1_ind, recv2_ind;
static char recv1_ch, send1_buffer[TS_SEND_BUFF], recv1_buffer[TS_RECV_BUFF]; 
static char recv2_ch, send2_buffer[TS_SEND_BUFF], recv2_buffer[TS_RECV_BUFF]; 

static osTimerId_t timer_conn, timer_step1, timer_step2;
static osTimerId_t timer_timeout1, timer_timeout2;
static osTimerId_t timer_status1, timer_status2, timer_log;
static osTimerId_t timer_rel1on, timer_rel1off, timer_rel2on, timer_rel2off;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t inv1_attr = {
    .name = "Invertor1", 
    .stack_size = 1152,
    .priority = osPriorityNormal
 };

static const osThreadAttr_t inv2_attr = {
    .name = "Invertor2", 
    .stack_size = 1152,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn1_attr = { .name = "Inv1" };
static const osEventFlagsAttr_t evn2_attr = { .name = "Inv2" };
static const osTimerAttr_t timer0_attr = { .name = "InvDcConn" };
static const osTimerAttr_t timer1_attr = { .name = "InvStep1" };
static const osTimerAttr_t timer2_attr = { .name = "InvStep2" };
static const osTimerAttr_t timer3_attr = { .name = "InvStatus1" };
static const osTimerAttr_t timer4_attr = { .name = "InvStatus2" };
static const osTimerAttr_t timer5_attr = { .name = "InvLog" };
static const osTimerAttr_t timer6_attr = { .name = "Inv1RelOn" };
static const osTimerAttr_t timer7_attr = { .name = "Inv1RelOff" };
static const osTimerAttr_t timer8_attr = { .name = "Inv2RelOn" };
static const osTimerAttr_t timer9_attr = { .name = "Inv2RelOff" };
static const osTimerAttr_t timer10_attr = { .name = "Inv1Timeout" };
static const osTimerAttr_t timer11_attr = { .name = "Inv2Timeout" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void ClrData( Device dev, uint8_t mask  );
static TypePack TypeData( char *data );
static InvDcStatus InvDcConn( Device dev );
static void InvGetData( Device dev, ParamInv param, char *data );
static uint8_t ParseData( Device dev, char *data, const uint8_t *mask_parse, uint8_t cnt_param );
static void InvSendCmnd( Device dev, InvCommand cmd );
static void InvSend( Device dev, const char *buff );
static void InvCheckManual( Device dev );
static void SendClear( Device dev );
static void RecvClear( Device dev );

void CallBackInv1( uint32_t event );
static void Inv1CycleOn( void );
static void Inv1CycleOff( void );
static void Inv1ResulOut( void );
static void Inv1CheckAnswer( void );
static void Inv1NextStep( InvCtrlCycle step, uint32_t time, InvCtrlError step_result );
static void EventLog1( char *text, InvCtrlError error );
static void InvSaveLog( void );

void CallBackInv2( uint32_t event );
static void Inv2CycleOn( void );
static void Inv2CycleOff( void );
static void Inv2ResulOut( void );
static void Inv2CheckAnswer( void );
static void Inv2NextStep( InvCtrlCycle step, uint32_t time, InvCtrlError step_result );
static void EventLog2( char *text, InvCtrlError error );

static void Timer0Callback( void *arg );
static void Timer1Callback( void *arg );
static void Timer2Callback( void *arg );
static void Timer3Callback( void *arg );
static void Timer4Callback( void *arg );
static void Timer5Callback( void *arg );
static void Timer6Callback( void *arg );
static void Timer7Callback( void *arg );
static void Timer8Callback( void *arg );
static void Timer9Callback( void *arg );
static void Timer10Callback( void *arg );
static void Timer11Callback( void *arg );
static void TaskInv1( void *pvParameters );
static void TaskInv2( void *pvParameters );

//*************************************************************************************************
// Инициализация портов управления инверторами
// управление реле только в импульсном режиме
// USART2/IRQ7  USART3/IRQ8
//*************************************************************************************************
void InvInit( void ) {

    //настройка портов управления реле
    GPIO_SetDir( TS_CTRL_PORT, TS1000_ON, GPIO_DIR_OUTPUT );
    GPIO_SetDir( TS_CTRL_PORT, TS3000_ON, GPIO_DIR_OUTPUT );
    GPIO_SetDir( TS_CTRL_PORT, TS1000_OFF, GPIO_DIR_OUTPUT );
    GPIO_SetDir( TS_CTRL_PORT, TS3000_OFF, GPIO_DIR_OUTPUT );
    //чистим буферы обмена
    SendClear( ID_DEV_INV1 );
    SendClear( ID_DEV_INV2 );
    RecvClear( ID_DEV_INV1 );
    RecvClear( ID_DEV_INV2 );
    //обнулим данные инверторов
    ClrData( ID_DEV_INV1, CLR_ALL ); 
    ClrData( ID_DEV_INV2, CLR_ALL ); 
    //очередь сообщений
    inv1_event = osEventFlagsNew( &evn1_attr );
    inv2_event = osEventFlagsNew( &evn2_attr );
    //обновление состояния подключения инверторов
    timer_conn = osTimerNew( Timer0Callback, osTimerPeriodic, NULL, &timer0_attr );
    //таймеры выполнения циклограммы управления 
    timer_step1 = osTimerNew( Timer1Callback, osTimerOnce, NULL, &timer1_attr );
    timer_step2 = osTimerNew( Timer2Callback, osTimerOnce, NULL, &timer2_attr );
    //таймер запроса состояния инверторов
    timer_status1 = osTimerNew( Timer3Callback, osTimerOnce, NULL, &timer3_attr );
    timer_status2 = osTimerNew( Timer4Callback, osTimerOnce, NULL, &timer4_attr );
    //таймер интервальной записи данных
    timer_log = osTimerNew( Timer5Callback, osTimerOnce, NULL, &timer5_attr );
    //таймеры импульсного управления контакторами
    timer_rel1on = osTimerNew( Timer6Callback, osTimerOnce, NULL, &timer6_attr );
    timer_rel1off = osTimerNew( Timer7Callback, osTimerOnce, NULL, &timer7_attr );
    timer_rel2on = osTimerNew( Timer8Callback, osTimerOnce, NULL, &timer8_attr );
    timer_rel2off = osTimerNew( Timer9Callback, osTimerOnce, NULL, &timer9_attr );
    //таймер ожидания ответа инвертора
    timer_timeout1 = osTimerNew( Timer10Callback, osTimerOnce, NULL, &timer10_attr );
    timer_timeout2 = osTimerNew( Timer11Callback, osTimerOnce, NULL, &timer11_attr );
    //создаем задачи
    osThreadNew( TaskInv1, NULL, &inv1_attr );
    osThreadNew( TaskInv2, NULL, &inv2_attr );
    //последовательный порт инвертора TS-1000-224
    UsartInv1 = &Driver_USART2;
    UsartInv1->Initialize( &CallBackInv1 );
    NVIC_SetPriority( UART2_IRQn, NVIC_EncodePriority( NVIC_GetPriorityGrouping(), PRIORITY_USART, SUB_PRIORITY_USART2 ) );
    UsartInv1->PowerControl( ARM_POWER_FULL );
    UsartInv1->Control( ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | ARM_USART_PARITY_NONE | 
                       ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE, 9600 );
    UsartInv1->Control( ARM_USART_CONTROL_TX, 1 );
    UsartInv1->Control( ARM_USART_CONTROL_RX, 1 );    
    UsartInv1->Receive( &recv1_ch, 1 );
    //последовательный порт инвертора TS-3000-224
    UsartInv2 = &Driver_USART3;
    UsartInv2->Initialize( &CallBackInv2 );
    NVIC_SetPriority( UART3_IRQn, NVIC_EncodePriority( NVIC_GetPriorityGrouping(), PRIORITY_USART, SUB_PRIORITY_USART3 ) );
    UsartInv2->PowerControl( ARM_POWER_FULL );
    UsartInv2->Control( ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | ARM_USART_PARITY_NONE | 
                       ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE, 9600 );
    UsartInv2->Control( ARM_USART_CONTROL_TX, 1 );
    UsartInv2->Control( ARM_USART_CONTROL_RX, 1 );    
    UsartInv2->Receive( &recv2_ch, 1 );
    osEventFlagsSet( inv1_event, EVN_INV_STARTUP );
    osEventFlagsSet( inv2_event, EVN_INV_STARTUP );
    osTimerStart( timer_conn, 250 );
 }

//*************************************************************************************************
// Задача управления инвертором TS-1000-224
//*************************************************************************************************
static void TaskInv1( void *pvParameters ) {
    
    uint32_t send, event;
    
    for ( ;; ) {
        //запуск таймера интервальной записи данных
        if ( !osTimerIsRunning( timer_log ) )
            osTimerStart( timer_log, config.datlog_upd_inv * SEC_TO_TICK );
        //ждем события
        event = osEventFlagsWait( inv1_event, EVN_INV_MASK, osFlagsWaitAny, osWaitForever );
        //проверка подключения инвертора при запуске задачи
        if ( event & EVN_INV_STARTUP && InvBatConn( ID_DEV_INV1 ) == BAT_DC_ON ) {
            //АКБ, схема управления, контактор включен, отправим команду чтения состояния
            control1 = INV_CTRL_ON;
            Inv1NextStep( INV_STEP_READ_CONFIG, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
           }
        if ( event & EVN_INV_CYCLE_NEXT ) {
            //выполнение циклограммы вкл/выкл инверторов
            if ( inv1.cycle_step != INV_STEP_STOP && control1 == INV_CTRL_ON )            
                Inv1CycleOn(); 
            if ( inv1.cycle_step != INV_STEP_STOP && control1 == INV_CTRL_OFF )            
                Inv1CycleOff(); 
           }
        //Вывод результата выполнения циклограммы
        if ( event & EVN_INV_CONSOLE )
            Inv1ResulOut();
        //запрос состояния инвертора
        if ( event & EVN_INV_STATUS ) {
            InvSendCmnd( ID_DEV_INV1, last_cmnd1 );
            send = ID_DEV_INV1; //передача данных в HMI
            osMessageQueuePut( hmi_msg, &send, 0, 0 );
           }
        //обработка ответа инвертора
        if ( event & EVN_INV_RECV )
            Inv1CheckAnswer();
        //проверка вкл/выкл инвертора в ручном режиме
        if ( event & EVN_RTC_SECONDS ) {
            InvCheckManual( ID_DEV_INV1 );
            if ( !osTimerIsRunning( timer_status1 ) ) {
                send = ID_DEV_INV1; //передача данных в HMI
                osMessageQueuePut( hmi_msg, &send, 0, 0 );
               }
           }
        //интервальное сохранение данных
        if ( event & EVN_INV_LOG ) {
            InvSaveLog(); 
            osTimerStart( timer_log, config.datlog_upd_inv * SEC_TO_TICK );
           }
       }
 }

//*************************************************************************************************
// Задача управления инвертором TS-3000-224
//*************************************************************************************************
static void TaskInv2( void *pvParameters ) {
    
    uint32_t send, event;
    
    for ( ;; ) {
        //ждем события
        event = osEventFlagsWait( inv2_event, EVN_INV_MASK, osFlagsWaitAny, osWaitForever );
        //проверка подключения инвертора при запуске задачи
        if ( event & EVN_INV_STARTUP && InvBatConn( ID_DEV_INV2 ) == BAT_DC_ON ) {
            //АКБ, схема управления, контактор включен, отправим команду чтения состояния
            control2 = INV_CTRL_ON;
            Inv2NextStep( INV_STEP_READ_CONFIG, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
           }
        //выполнение циклограммы вкл/выкл инверторов
        if ( event & EVN_INV_CYCLE_NEXT ) {
            if ( inv2.cycle_step != INV_STEP_STOP && control2 == INV_CTRL_ON )            
                Inv2CycleOn(); 
            if ( inv2.cycle_step != INV_STEP_STOP && control2 == INV_CTRL_OFF )            
                Inv2CycleOff(); 
           }
        //Вывод результата выполнения циклограммы
        if ( event & EVN_INV_CONSOLE )
            Inv2ResulOut();
        //запрос состояния инвертора
        if ( event & EVN_INV_STATUS ) {
            InvSendCmnd( ID_DEV_INV2, last_cmnd2 );
            send = ID_DEV_INV2; //передача данных в HMI
            osMessageQueuePut( hmi_msg, &send, 0, 0 );
           }
        //обработка ответа инвертора
        if ( event & EVN_INV_RECV )
            Inv2CheckAnswer();
        //проверка вкл/выкл инвертора в ручном режиме
        if ( event & EVN_RTC_SECONDS )
            InvCheckManual( ID_DEV_INV2 );
            if ( !osTimerIsRunning( timer_status2 ) ) {
                send = ID_DEV_INV2; //передача данных в HMI
                osMessageQueuePut( hmi_msg, &send, 0, 0 );
               }
       }
 }

//*************************************************************************************************
// Функция обратного вызова таймера - обновление состояния DC_CONN для TS-1000-224/TS-3000-224
//*************************************************************************************************
static void Timer0Callback( void *arg ) {

    inv1.dc_conn = InvDcConn( ID_DEV_INV1 );
    inv2.dc_conn = InvDcConn( ID_DEV_INV2 );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - выполнение шага циклограммы для TS-1000-224
//*************************************************************************************************
static void Timer1Callback( void *arg ) {

    osEventFlagsSet( inv1_event, EVN_INV_CYCLE_NEXT );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - выполнение шага циклограммы для TS-3000-224
//*************************************************************************************************
static void Timer2Callback( void *arg ) {

    osEventFlagsSet( inv2_event, EVN_INV_CYCLE_NEXT );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - запроса состояния инвертора TS-1000-224
//*************************************************************************************************
static void Timer3Callback( void *arg ) {

    osEventFlagsSet( inv1_event, EVN_INV_STATUS );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - запроса состояния инвертора TS-3000-224
//*************************************************************************************************
static void Timer4Callback( void *arg ) {

    osEventFlagsSet( inv2_event, EVN_INV_STATUS );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - интервальное логирование состояния инверторов
//*************************************************************************************************
static void Timer5Callback( void *arg ) {

    osEventFlagsSet( inv1_event, EVN_INV_LOG );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - выключение реле "включения контактора" TS-1000-224
//*************************************************************************************************
static void Timer6Callback( void *arg ) {

    GPIO_PinWrite( TS_CTRL_PORT, TS1000_ON, RELAY_OFF );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - выключение реле "выключения контактора" TS-1000-224
//*************************************************************************************************
static void Timer7Callback( void *arg ) {

    GPIO_PinWrite( TS_CTRL_PORT, TS1000_OFF, RELAY_OFF );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - выключение реле "включения контактора" TS-3000-224
//*************************************************************************************************
static void Timer8Callback( void *arg ) {

    GPIO_PinWrite( TS_CTRL_PORT, TS3000_ON, RELAY_OFF );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - выключение реле "выключения контактора" TS-3000-224
//*************************************************************************************************
static void Timer9Callback( void *arg ) {

    GPIO_PinWrite( TS_CTRL_PORT, TS3000_OFF, RELAY_OFF );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - время ожидания ответа на команду инвертора TS-3000-224
//*************************************************************************************************
static void Timer10Callback( void *arg ) {

    //снимаем соответствующий признак актуальности данных при отсутствии ответа
    if ( last_cmnd1 & INV_CMD_CONFIG )
        inv1.act_config = 0;
    if ( last_cmnd1 & INV_CMD_STAT )
        inv1.act_status = 0;
 }

//*************************************************************************************************
// Функция обратного вызова таймера - время ожидания ответа на команду инвертора TS-3000-224
//*************************************************************************************************
static void Timer11Callback( void *arg ) {

    //снимаем соответствующий признак актуальности данных при отсутствии ответа
    if ( last_cmnd2 & INV_CMD_CONFIG )
        inv2.act_config = 0;
    if ( last_cmnd2 & INV_CMD_STAT )
        inv2.act_status = 0;
 }

//*************************************************************************************************
// Обработчик событий последовательного порта TS-1000-224
//*************************************************************************************************
void CallBackInv1( uint32_t event ) {

    if ( event & ARM_USART_EVENT_RECEIVE_COMPLETE ) {
        if ( recv1_ch > 0x1F || recv1_ch == '\r' ) {
            //принят один байт, сохраним только значащие символы
            recv1_ind += UsartInv1->GetRxCount();
            if ( recv1_ind <= TS_RECV_BUFF )
                recv1_buffer[recv1_ind-1] = toupper( recv1_ch ); //сохраним байт в буфере
            else RecvClear( ID_DEV_INV1 ); //буфер переполнен, сбросим буфер
            if ( recv1_ind > 0 && recv1_buffer[recv1_ind-1] == '\r' ) {
                //ответ получен
                recv1_buffer[recv1_ind-1] = '\0';
                osEventFlagsSet( inv1_event, EVN_INV_RECV );
               }
           }
        //повторная инициализация приема
        UsartInv1->Receive( &recv1_ch, 1 );
       }
    if ( event & ARM_USART_EVENT_SEND_COMPLETE )
        SendClear( ID_DEV_INV1 ); //передача завершена
 }

//*************************************************************************************************
// Обработчик событий последовательного порта TS-3000-224
//*************************************************************************************************
void CallBackInv2( uint32_t event ) {

    if ( event & ARM_USART_EVENT_RECEIVE_COMPLETE ) {
        if ( recv2_ch > 0x1F || recv2_ch == '\r' ) {
            //принят один байт, сохраним только значащие символы
            recv2_ind += UsartInv2->GetRxCount();
            if ( recv2_ind <= TS_RECV_BUFF )
                recv2_buffer[recv2_ind-1] = toupper( recv2_ch ); //сохраним байт в буфере
            else RecvClear( ID_DEV_INV2 ); //буфер переполнен, сбросим буфер
            if ( recv2_ind > 0 && recv2_buffer[recv2_ind-1] == '\r' ) {
                //ответ получен
                recv2_buffer[recv2_ind-1] = '\0';
                osEventFlagsSet( inv2_event, EVN_INV_RECV );
               }
          }
        //повторная инициализация приема
        UsartInv2->Receive( &recv2_ch, 1 );
       }
    if ( event & ARM_USART_EVENT_SEND_COMPLETE )
        SendClear( ID_DEV_INV2 ); //передача завершена
 }

//*************************************************************************************************
// Управление вкл/выкл инверторов
// Device dev       - ID инвертора
// InvCtrlCmnd mode - команда вкл/выкл
//*************************************************************************************************
void InvCtrl( Device dev, InvCtrlCmnd mode ) {

    if ( dev == ID_DEV_INV1 && inv1.cycle_step == INV_STEP_STOP ) {
        //начинаем циклограмму если она еще не запущена
        control1 = mode;
        if ( mode == INV_CTRL_ON ) {
            manually1 = true; //разрешим контроль ручного выключения инвертора
            Inv1NextStep( INV_STEP_CHK_BAT, TIME_NEXT_STEP, INV_ERR_CTRL_OK ); 
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV1_TURN_ON ), INV_ERR_CTRL_OK );
           }
        if ( mode == INV_CTRL_OFF ) {
            manually1 = false; //разрешим контроль ручного включения инвертора
            Inv1NextStep( INV_STEP_CHK_BAT, TIME_NEXT_STEP, INV_ERR_CTRL_OK ); 
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV1_SHDN ), INV_ERR_CTRL_OK );
           }
       }
    if ( dev == ID_DEV_INV2 && inv2.cycle_step == INV_STEP_STOP ) {
        //начинаем циклограмму если она еще не запущена
        control2 = mode;
        if ( mode == INV_CTRL_ON ) {
            manually2 = true; //разрешим контроль ручного выключения инвертора
            Inv2NextStep( INV_STEP_CHK_BAT, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV2_TURN_ON ), INV_ERR_CTRL_OK );
           }
        if ( mode == INV_CTRL_OFF ) {
            manually2 = false; //разрешим контроль ручного включения инвертора
            Inv2NextStep( INV_STEP_CHK_BAT, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV2_SHDN ), INV_ERR_CTRL_OK );
           }
       }
 }

//*************************************************************************************************
// Проверяет вкл/выкл инверторов в ручном режиме.
// Для вкл/выкл отправки команды чтения статуса инвертора, вызов из TaskInv1/2
// Device dev - ID инвертора
//*************************************************************************************************
static void InvCheckManual( Device dev ) {

    if ( dev == ID_DEV_INV1 ) {
        if ( InvBatConn( ID_DEV_INV1 ) == BAT_DC_ON && !last_cmnd1 && inv1.cycle_step == INV_STEP_STOP && manually1 == false ) {
            //АКБ, схема управления, контактор включен, циклограмма не запущена - отправим команду чтения состояния
            manually1 = true;
            InvSendCmnd( ID_DEV_INV1, (InvCommand)( INV_CMD_CONFIG | INV_CMD_STAT ) );
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV1_MAN_ON ), INV_ERR_CTRL_OK );
           } 
        if ( InvBatConn( ID_DEV_INV1 ) == BAT_DC_OFF && last_cmnd1 && inv1.cycle_step == INV_STEP_STOP && manually1 == true ) {
            //АКБ, схема управления, контактор выключен, циклограмма не запущена - отправим команду выключения чтения состояния
            manually1 = false;
            //останавливаем отправку команд циклических команд
            InvSendCmnd( ID_DEV_INV1, INV_CMD_NULL );
            ClrData( ID_DEV_INV1, CLR_DATA | CLR_STATUS );
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV1_MAN_OFF ), INV_ERR_CTRL_OK );
           }
       }
    if ( dev == ID_DEV_INV2 ) {
        if ( InvBatConn( ID_DEV_INV2 ) == BAT_DC_ON && !last_cmnd2 && inv2.cycle_step == INV_STEP_STOP && manually2 == false ) {
            //АКБ, схема управления, контактор включен, циклограмма не запущена - отправим команду чтения состояния
            manually2 = true;
            InvSendCmnd( ID_DEV_INV2, (InvCommand)( INV_CMD_CONFIG | INV_CMD_STAT ) );
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV2_MAN_ON ), INV_ERR_CTRL_OK );
           } 
        if ( InvBatConn( ID_DEV_INV2 ) == BAT_DC_OFF && last_cmnd2 && inv2.cycle_step == INV_STEP_STOP && manually2 == true ) {
            //АКБ, схема управления, контактор выключен, циклограмма не запущена - отправим команду выключения чтения состояния
            manually2 = false;
            //останавливаем отправку команд циклических команд
            InvSendCmnd( ID_DEV_INV2, INV_CMD_NULL );
            ClrData( ID_DEV_INV2, CLR_DATA | CLR_STATUS );
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV2_MAN_OFF ), INV_ERR_CTRL_OK );
           }
       }
 }

//*************************************************************************************************
// Циклограмма включения инвертора TS-1000-224
//*************************************************************************************************
static void Inv1CycleOn( void ) {

    if ( inv1.cycle_step == INV_STEP_CHK_BAT ) {
        //проверка подключения АКБ
        if ( !BatConn() ) {
            EventLog1( NULL, INV_ERR_CTRL_NOBAT );
            Inv1NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_NOBAT );
            osEventFlagsSet( inv1_event, EVN_INV_CONSOLE );
            Informing( VOICE_BAT_NOCONN, NULL );
            return; //АКБ не включена
           }
        //следующий шаг 
        Inv1NextStep( INV_STEP_CHK_CTRL, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
        return;
       } 
    if ( inv1.cycle_step == INV_STEP_CHK_CTRL ) {
        //проверка включения схемы управления контакторами
        if ( !StatCtrl() ) {
            //управление контакторами выключено, но возможно контакторы 
            //остались включенными, пробуем включить инвертор программно
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV_CTRL_DC_OFF ), INV_ERR_CTRL_OK );
            //включаем программно
            InvSendCmnd( ID_DEV_INV1, INV_CMD_ON );
            //следующий шаг, с задержкой на TIME_WAIT_POWER_ON секунд время вкл инвертора
            Inv1NextStep( INV_STEP_READ_CONFIG, TIME_WAIT_POWER_ON, INV_ERR_CTRL_OK );
            return;
           }
         else {
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV_CTRL_DC_ON ), INV_ERR_CTRL_OK );
            //схема управления вкл, следующий шаг - включения контакторов
            Inv1NextStep( INV_STEP_ON_CONTACTOR, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
           }
         return;
       }
    if ( inv1.cycle_step == INV_STEP_ON_CONTACTOR ) {
        //схема управления контакторами включена, проверим вкл контактора
        if ( !GetDataPort( TS1000_CHK ) ) {
            //включаем контактор
            GPIO_PinWrite( TS_CTRL_PORT, TS1000_ON, RELAY_ON );
            osTimerStart( timer_rel1on, RELAY_PULSE1 );
            //задержка перед проверкой включения контактора
            Inv1NextStep( INV_STEP_CHK_CONTACTOR, TIME_CHK_CONTACTOR, INV_ERR_CTRL_OK );
           }
        else {
            //контактор включен, проверка статуса инвертора "выключен удаленно"
            Inv1NextStep( INV_STEP_CHK_RMT_OFF, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
           }
        return;
       }
    if ( inv1.cycle_step == INV_STEP_CHK_CONTACTOR ) {
        //проверка включение контактора
        if ( !GetDataPort( TS1000_CHK ) ) {
            //контактор не включился, выходим с ошибкой
            EventLog1( NULL, INV_ERR_CTRL_NOSIGNAL );
            Inv1NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_NOSIGNAL );
            osEventFlagsSet( inv1_event, EVN_INV_CONSOLE );
           } 
        else {
            //контактор включен
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV_DC_ON ), INV_ERR_CTRL_OK );
            //следующий шаг, с задержкой на TIME_WAIT_POWER_ON секунд время вкл инвертора
            Inv1NextStep( INV_STEP_READ_CONFIG, TIME_WAIT_POWER_ON, INV_ERR_CTRL_OK );
           }
        return;
       }
    if ( inv1.cycle_step == INV_STEP_CHK_RMT_OFF ) {
        //проверка статуса инвертора "выключен удаленно"
        if ( inv1.mode == INV_MODE_RMT_OFF ) {
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV_RMTOFF_SWON ), INV_ERR_CTRL_OK );
            //включаем программно
            InvSendCmnd( ID_DEV_INV1, INV_CMD_ON );
            //следующий шаг - чтение конфигурации с задержкой TIME_WAIT_POWER_ON
            Inv1NextStep( INV_STEP_READ_CONFIG, TIME_WAIT_POWER_ON, INV_ERR_CTRL_OK );
           }
        else {
            //следующий шаг, с задержкой на TIME_WAIT_POWER_ON секунд время вкл инвертора
            Inv1NextStep( INV_STEP_READ_CONFIG, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
           }
        return;
       }
    if ( inv1.cycle_step == INV_STEP_READ_CONFIG ) {
        //запрос конфигурации инвертора
        InvSendCmnd( ID_DEV_INV1, INV_CMD_CONFIG );
        EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV_RD_CONF ), INV_ERR_CTRL_OK );
        //следующий шаг - чтение статуса инвертора, с задержкой на TIME_WAIT_ANSWER секунд
        Inv1NextStep( INV_STEP_READ_STATUS, TIME_WAIT_ANSWER, INV_ERR_CTRL_OK );
        return;
       }
    if ( inv1.cycle_step == INV_STEP_READ_STATUS ) {
        //цикл включения завершен, запускаем циклическое чтение статуса инвертора
        InvSendCmnd( ID_DEV_INV1, INV_CMD_STAT );
        EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV_RD_STAT ), INV_ERR_CTRL_OK );
        //следующий шаг - проверка статуса инвертора, с задержкой на TIME_WAIT_ANSWER секунд
        Inv1NextStep( INV_STEP_CHK_STATUS, TIME_WAIT_ANSWER, INV_ERR_CTRL_OK );
        return;
       }
    if ( inv1.cycle_step == INV_STEP_CHK_STATUS ) {
        //проверим результат включения инвертора
        if ( inv1.mode == INV_MODE_NO_LINK ) {
            //инвертор не отвечает
            EventLog1( NULL, INV_ERR_CTRL_ANSWER );
            Inv1NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_ANSWER );
            osEventFlagsSet( inv1_event, EVN_INV_CONSOLE );
            return;
           }
        if ( inv1.mode == INV_MODE_OFF ) {
            //выключен клавишей питания
            EventLog1( NULL, INV_ERR_CTRL_POWER_OFF );
            Inv1NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_POWER_OFF );
            osEventFlagsSet( inv1_event, EVN_INV_CONSOLE );
            return;
           }
        if ( inv1.mode == INV_MODE_ON || inv1.mode == INV_MODE_SAVE ) {
            //инвертор включен, ничего не делаем
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV1_ON ), INV_ERR_CTRL_OK );
            Inv1NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_OK );
            osEventFlagsSet( inv1_event, EVN_INV_CONSOLE );
            return;
           }
        Inv1NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_OK );
        return;
       }
 }

//*************************************************************************************************
// Циклограмма выключения инвертора TS-1000-224
//*************************************************************************************************
static void Inv1CycleOff( void ) {

    if ( inv1.cycle_step == INV_STEP_CHK_BAT ) {
        //проверка подключения АКБ
        if ( !BatConn() ) {
            //АКБ выключена
            Inv1NextStep( INV_STEP_STOP, TIME_NEXT_STEP, INV_ERR_CTRL_NOBAT );
            osEventFlagsSet( inv1_event, EVN_INV_CONSOLE );
           }
        //следующий шаг - проверка статуса инвертора, программное выключение
        else Inv1NextStep( INV_STEP_CHK_STATUS, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
        return;
       } 
    if ( inv1.cycle_step == INV_STEP_CHK_STATUS ) {
        //программное выключение инвертора
        if ( inv1.mode == INV_MODE_NO_LINK ) {
            //инвертор не отвечает
            EventLog1( NULL, INV_ERR_CTRL_ANSWER );
            //пробуем выключить программно
            InvSendCmnd( ID_DEV_INV1, INV_CMD_OFF );
            ClrData( ID_DEV_INV1, CLR_DATA );
            inv1.mode = INV_MODE_RMT_OFF;
            //следующий шаг - проверка управления контакторами
            Inv1NextStep( INV_STEP_CHK_CTRL, TIME_WAIT_POWER_OFF, INV_ERR_CTRL_ANSWER );
            return;
           }
        if ( inv1.mode == INV_MODE_OFF ) {
            //выключен клавишей питания
            EventLog1( NULL, INV_ERR_CTRL_POWER_OFF );
            //следующий шаг - проверка управления контакторами
            Inv1NextStep( INV_STEP_CHK_CTRL, TIME_NEXT_STEP, INV_ERR_CTRL_POWER_OFF );
            return;
           }
        if ( inv1.mode == INV_MODE_ON || inv1.mode == INV_MODE_SAVE ) {
            //инвертор включен или в режиме "save mode"
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV_OFF_CMD ), INV_ERR_CTRL_OK );
            //пробуем выключить программно
            InvSendCmnd( ID_DEV_INV1, INV_CMD_OFF );
            ClrData( ID_DEV_INV1, CLR_DATA );
            inv1.mode = INV_MODE_RMT_OFF;
            //следующий шаг - проверка управления контакторами
            Inv1NextStep( INV_STEP_CHK_CTRL, TIME_WAIT_POWER_OFF, INV_ERR_CTRL_OK );
            return;
           }
        if ( inv1.mode == INV_MODE_RMT_OFF ) {
            //уже выключен удаленно
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV_OFF_RMT ), INV_ERR_CTRL_OK );
            //следующий шаг - проверка управления контакторами
            Inv1NextStep( INV_STEP_CHK_CTRL, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
            return;
           }
       }
    if ( inv1.cycle_step == INV_STEP_CHK_CTRL ) {
        //проверка управления контакторами 
        if ( !StatCtrl() ) {
            //управление контакторами выключено - завершаем циклограмму
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV_CTRL_DC_OFF ), INV_ERR_CTRL_NOCTRL );
            Inv1NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_NOCTRL );
           }
         else {
            //управления контакторами включено, следующий шаг - проверка отсутствия нагрузки
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV_CTRL_DC_ON ), INV_ERR_CTRL_OK );
            Inv1NextStep( INV_STEP_CHK_POWER, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
           }
        return;
       }
    if ( inv1.cycle_step == INV_STEP_CHK_POWER ) {
        //проверка выключения инвертора по отсутствию нагрузки
        if ( GetDataPort( TS1000_LOC ) ) {
            //есть нагрузка, выключать нельзя
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV_POWER_AC ), INV_ERR_CTRL_POWER );
            Inv1NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_POWER );
            osEventFlagsSet( inv1_event, EVN_INV_CONSOLE );
           }
        else {
            //следующий шаг - выключаем контактор
            InvSendCmnd( ID_DEV_INV1, INV_CMD_NULL );
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV_DC_SHDN ), INV_ERR_CTRL_OK );
            Inv1NextStep( INV_STEP_OFF_CONTACTOR, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
           }
        return;
       }
    if ( inv1.cycle_step == INV_STEP_OFF_CONTACTOR ) {
        //выключаем контактор
        EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV_DC_SHDN ), INV_ERR_CTRL_OK );
        GPIO_PinWrite( TS_CTRL_PORT, TS1000_OFF, RELAY_ON );
        osTimerStart( timer_rel1off, RELAY_PULSE1 );
        //следующий шаг - проверка выключения контактора
        Inv1NextStep( INV_STEP_CHK_CONTACTOR, TIME_CHK_CONTACTOR, INV_ERR_CTRL_OK );
        return;
       }
    if ( inv1.cycle_step == INV_STEP_CHK_CONTACTOR ) {
        //проверим выключение контактора
        if ( GetDataPort( TS1000_CHK ) ) {
            //контактор не выключился
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV_DC_OFF_ERR ), INV_ERR_CTRL_NOSIGNAL );
            Inv1NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_NOSIGNAL );
            osEventFlagsSet( inv1_event, EVN_INV_CONSOLE );
            SoundPlay( SND_HDFAIL, NULL );
            SoundPlay( SND_HDFAIL, NULL );
            SoundPlay( SND_HDFAIL, NULL );
           }
        else {
            //контактор выключен
            inv1.mode = INV_MODE_OFF;
            EventLog1( MessageLog( ID_DEV_INV1, LOG_MSG_INV1_OFF ), INV_ERR_CTRL_OK );
            Inv1NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_OK );
            osEventFlagsSet( inv1_event, EVN_INV_CONSOLE );
           }
        return;
      }
 }

//*************************************************************************************************
// Циклограмма управления включением инвертора TS-3000-224
//*************************************************************************************************
static void Inv2CycleOn( void ) {

    if ( inv2.cycle_step == INV_STEP_CHK_BAT ) {
        //проверка подключения АКБ
        if ( !BatConn() ) {
            EventLog2( NULL, INV_ERR_CTRL_NOBAT );
            Inv2NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_NOBAT );
            osEventFlagsSet( inv2_event, EVN_INV_CONSOLE );
            Informing( VOICE_BAT_NOCONN, NULL );
            return; //АКБ не включена
           }
        //следующий шаг 
        Inv2NextStep( INV_STEP_CHK_CTRL, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
        return;
       } 
    if ( inv2.cycle_step == INV_STEP_CHK_CTRL ) {
        //проверка включения схемы управления контакторами
        if ( !StatCtrl() ) {
            //управление контакторами выключено, но возможно контакторы 
            //остались включенными, пробуем включить инвертор програмнно
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV_CTRL_DC_OFF ), INV_ERR_CTRL_OK );
            //включаем программно
            InvSendCmnd( ID_DEV_INV2, INV_CMD_ON );
            //следующий шаг, с задержкой на TIME_WAIT_POWER_ON секунд время вкл инвертора
            Inv2NextStep( INV_STEP_READ_CONFIG, TIME_WAIT_POWER_ON, INV_ERR_CTRL_OK );
            return;
           }
         else {
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV_CTRL_DC_ON ), INV_ERR_CTRL_OK );
            //схема управления вкл, следующий шаг - включения контакторов
            Inv2NextStep( INV_STEP_ON_CONTACTOR, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
           }
         return;
       }
    if ( inv2.cycle_step == INV_STEP_ON_CONTACTOR ) {
        //схема управления контакторами включена, проверим вкл контактора
        if ( !GetDataPort( TS3000_CHK ) ) {
            //включаем контактор
            GPIO_PinWrite( TS_CTRL_PORT, TS3000_ON, RELAY_ON );
            osTimerStart( timer_rel2on, RELAY_PULSE1 );
            //задержка перед проверкой включения контактора
            Inv2NextStep( INV_STEP_CHK_CONTACTOR, TIME_CHK_CONTACTOR, INV_ERR_CTRL_OK );
           }
        else {
            //контактор включен, проверка статуса инвертора "выключен удаленно"
            Inv2NextStep( INV_STEP_CHK_RMT_OFF, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
           }
        return;
       }
    if ( inv2.cycle_step == INV_STEP_CHK_CONTACTOR ) {
        //проверка включение контактора
        if ( !GetDataPort( TS3000_CHK ) ) {
            //контактор не включился, выходим с ошибкой
            EventLog2( NULL, INV_ERR_CTRL_NOSIGNAL );
            Inv2NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_NOSIGNAL );
            osEventFlagsSet( inv2_event, EVN_INV_CONSOLE );
           } 
        else {
            //контактор включен
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV_DC_ON ), INV_ERR_CTRL_OK );
            //следующий шаг, с задержкой на TIME_WAIT_POWER_ON секунд время вкл инвертора
            Inv2NextStep( INV_STEP_READ_CONFIG, TIME_WAIT_POWER_ON, INV_ERR_CTRL_OK );
           }
        return;
       }
    if ( inv2.cycle_step == INV_STEP_CHK_RMT_OFF ) {
        //проверка статуса инвертора "выключен удаленно"
        if ( inv2.mode == INV_MODE_RMT_OFF ) {
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV_RMTOFF_SWON ), INV_ERR_CTRL_OK );
            //включаем программно
            InvSendCmnd( ID_DEV_INV2, INV_CMD_ON );
            //следующий шаг - чтение конфигурации с задержкой TIME_WAIT_POWER_ON
            Inv2NextStep( INV_STEP_READ_CONFIG, TIME_WAIT_POWER_ON, INV_ERR_CTRL_OK );
           }
        else {
            //следующий шаг, с задержкой на TIME_WAIT_POWER_ON секунд время вкл инвертора
            Inv2NextStep( INV_STEP_READ_CONFIG, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
           }
        return;
       }
    if ( inv2.cycle_step == INV_STEP_READ_CONFIG ) {
        //запрос конфигурации инвертора
        InvSendCmnd( ID_DEV_INV2, INV_CMD_CONFIG );
        EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV_RD_CONF ), INV_ERR_CTRL_OK );
        //следующий шаг - чтение статуса инвертора, с задержкой на TIME_WAIT_ANSWER секунд
        Inv2NextStep( INV_STEP_READ_STATUS, TIME_WAIT_ANSWER, INV_ERR_CTRL_OK );
        return;
       }
    if ( inv2.cycle_step == INV_STEP_READ_STATUS ) {
        //цикл включения завершен, запускаем циклическое чтение статуса инвертора
        InvSendCmnd( ID_DEV_INV2, INV_CMD_STAT );
        EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV_RD_STAT ), INV_ERR_CTRL_OK );
        //следующий шаг - проверка статуса инвертора, с задержкой на TIME_WAIT_ANSWER секунд
        Inv2NextStep( INV_STEP_CHK_STATUS, TIME_WAIT_ANSWER, INV_ERR_CTRL_OK );
        return;
       }
    if ( inv2.cycle_step == INV_STEP_CHK_STATUS ) {
        //проверим результат включения инвертора
        if ( inv2.mode == INV_MODE_NO_LINK ) {
            //инвертор не отвечает
            EventLog2( NULL, INV_ERR_CTRL_ANSWER );
            Inv2NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_ANSWER );
            osEventFlagsSet( inv2_event, EVN_INV_CONSOLE );
            return;
           }
        if ( inv2.mode == INV_MODE_OFF ) {
            //выключен клавишей питания
            EventLog2( NULL, INV_ERR_CTRL_POWER_OFF );
            Inv2NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_POWER_OFF );
            osEventFlagsSet( inv2_event, EVN_INV_CONSOLE );
            return;
           }
        if ( inv2.mode == INV_MODE_ON || inv2.mode == INV_MODE_SAVE ) {
            //инвертор включен, ничего не делаем
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV2_ON ), INV_ERR_CTRL_OK );
            Inv2NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_OK );
            osEventFlagsSet( inv2_event, EVN_INV_CONSOLE );
            return;
           }
        Inv2NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_OK );
        return;
       }
 }

//*************************************************************************************************
// Циклограмма управления выключением инвертора TS-3000-224
//*************************************************************************************************
static void Inv2CycleOff( void ) {

    if ( inv2.cycle_step == INV_STEP_CHK_BAT ) {
        //проверка подключения АКБ
        if ( !BatConn() ) {
            //АКБ выключена
            Inv2NextStep( INV_STEP_STOP, TIME_NEXT_STEP, INV_ERR_CTRL_NOBAT );
            osEventFlagsSet( inv2_event, EVN_INV_CONSOLE );
           }
        //следующий шаг - проверка статуса инвертора, программное выключение
        else Inv2NextStep( INV_STEP_CHK_STATUS, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
        return;
       } 
    if ( inv2.cycle_step == INV_STEP_CHK_STATUS ) {
        //программное выключение инвертора
        if ( inv2.mode == INV_MODE_NO_LINK ) {
            //инвертор не отвечает
            EventLog2( NULL, INV_ERR_CTRL_ANSWER );
            //пробуем выключить программно
            InvSendCmnd( ID_DEV_INV2, INV_CMD_OFF );
            ClrData( ID_DEV_INV2, CLR_DATA );
            inv2.mode = INV_MODE_RMT_OFF;
            //следующий шаг - проверка управления контакторами
            Inv2NextStep( INV_STEP_CHK_CTRL, TIME_WAIT_POWER_OFF, INV_ERR_CTRL_ANSWER );
            return;
           }
        if ( inv2.mode == INV_MODE_OFF ) {
            //выключен клавишей питания
            EventLog2( NULL, INV_ERR_CTRL_POWER_OFF );
            //следующий шаг - проверка управления контакторами
            Inv2NextStep( INV_STEP_CHK_CTRL, TIME_NEXT_STEP, INV_ERR_CTRL_POWER_OFF );
            return;
           }
        if ( inv2.mode == INV_MODE_ON || inv2.mode == INV_MODE_SAVE ) {
            //инвертор включен или в режиме "save mode"
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV_OFF_CMD ), INV_ERR_CTRL_OK );
            //пробуем выключить программно
            InvSendCmnd( ID_DEV_INV2, INV_CMD_OFF );
            ClrData( ID_DEV_INV2, CLR_DATA );
            inv2.mode = INV_MODE_RMT_OFF;
            //следующий шаг - проверка управления контакторами
            Inv2NextStep( INV_STEP_CHK_CTRL, TIME_WAIT_POWER_OFF, INV_ERR_CTRL_OK );
            return;
           }
        if ( inv2.mode == INV_MODE_RMT_OFF ) {
            //уже выключен удаленно
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV_OFF_RMT ), INV_ERR_CTRL_OK );
            //следующий шаг - проверка управления контакторами
            Inv2NextStep( INV_STEP_CHK_CTRL, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
            return;
           }
       }
    if ( inv2.cycle_step == INV_STEP_CHK_CTRL ) {
        //проверка управления контакторами 
        if ( !StatCtrl() ) {
            //управление контакторами выключено - завершаем циклограмму
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV_CTRL_DC_OFF ), INV_ERR_CTRL_NOCTRL );
            Inv2NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_NOCTRL );
           }
         else {
            //управления контакторами включено, следующий шаг - проверка отсутствия нагрузки
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV_CTRL_DC_ON ), INV_ERR_CTRL_OK );
            Inv2NextStep( INV_STEP_CHK_POWER, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
           }
        return;
       }
    if ( inv2.cycle_step == INV_STEP_CHK_POWER ) {
        //проверка выключения инвертора по отсутствию нагрузки
        if ( GetDataPort( TS3000_LOC ) ) {
            //есть нагрузка, выключать нельзя
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV_POWER_AC ), INV_ERR_CTRL_POWER );
            Inv2NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_POWER );
            osEventFlagsSet( inv2_event, EVN_INV_CONSOLE );
           }
        else {
            //следующий шаг - выключаем контактор
            InvSendCmnd( ID_DEV_INV2, INV_CMD_NULL );
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV_DC_SHDN ), INV_ERR_CTRL_OK );
            Inv2NextStep( INV_STEP_OFF_CONTACTOR, TIME_NEXT_STEP, INV_ERR_CTRL_OK );
           }
        return;
       }
    if ( inv2.cycle_step == INV_STEP_OFF_CONTACTOR ) {
        //выключаем контактор
        EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV_DC_SHDN ), INV_ERR_CTRL_OK );
        GPIO_PinWrite( TS_CTRL_PORT, TS3000_OFF, RELAY_ON );
        osTimerStart( timer_rel2off, RELAY_PULSE1 );
        //следующий шаг - проверка выключения контактора
        Inv2NextStep( INV_STEP_CHK_CONTACTOR, TIME_CHK_CONTACTOR, INV_ERR_CTRL_OK );
        return;
       }
    if ( inv2.cycle_step == INV_STEP_CHK_CONTACTOR ) {
        //проверим выключение контактора
        if ( GetDataPort( TS3000_CHK ) ) {
            //контактор не выключился
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV_DC_OFF_ERR ), INV_ERR_CTRL_NOSIGNAL );
            Inv2NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_NOSIGNAL );
            osEventFlagsSet( inv2_event, EVN_INV_CONSOLE );
            SoundPlay( SND_HDFAIL, NULL );
            SoundPlay( SND_HDFAIL, NULL );
            SoundPlay( SND_HDFAIL, NULL );
           }
        else {
            //контактор выключен
            inv2.mode = INV_MODE_OFF;
            EventLog2( MessageLog( ID_DEV_INV2, LOG_MSG_INV2_OFF ), INV_ERR_CTRL_OK );
            Inv2NextStep( INV_STEP_STOP, TIME_NO_WAIT, INV_ERR_CTRL_OK );
            osEventFlagsSet( inv2_event, EVN_INV_CONSOLE );
           }
        return;
      }
 }

//*************************************************************************************************
// Формирует следующий шаг циклограммы с заданной задержкой для инвертора TS-1000-224
// InvCycleStep step        - тип шага выполнения циклограммы
// uint32_t time            - время задержки выполнения в msec, таймер запускается 
//                            только для значений > 0
// InvCtrlError step_result - код выполнения шага циклограммы (код ошибки)
//*************************************************************************************************
static void Inv1NextStep( InvCtrlCycle step, uint32_t time, InvCtrlError step_result ) {

    inv1.ctrl_error = step_result;
    if ( step == INV_STEP_STOP ) {
        //останавливаем цикл
        inv1.cycle_step = step;
        osTimerStop( timer_step1 );
        return;
       }
    //следующий шаг указан в явном виде
    inv1.cycle_step = step;
    if ( time != TIME_NEXT_STEP && time )
        osTimerStart( timer_step1, time );
    //для вызова следующего шага из задачи, запустим таймер
    //с минимальным временем исполнения
    else osTimerStart( timer_step1, TIME_NEXT_STEP );
 }

//*************************************************************************************************
// Формирует следующий шаг циклограммы с заданной задержкой для инвертора TS-3000-224
// InvCycleStep step        - тип шага выполнения циклограммы
// uint32_t time            - время задержки выполнения в msec, таймер запускается 
//                            только для значений > 0
// InvCtrlError step_result - код выполнения шага циклограммы (код ошибки)
//*************************************************************************************************
static void Inv2NextStep( InvCtrlCycle step, uint32_t time, InvCtrlError step_result ) {

    inv2.ctrl_error = step_result;
    if ( step == INV_STEP_STOP ) {
        //останавливаем цикл
        inv2.cycle_step = step;
        osTimerStop( timer_step2 );
        return;
       }
    inv2.cycle_step = step;
    if ( time != TIME_NEXT_STEP && time )
        osTimerStart( timer_step2, time );
    //для вызова следующего шага из задачи, запустим таймер
    //с минимальным временем исполнения
    else osTimerStart( timer_step2, TIME_NEXT_STEP );
 }

//*************************************************************************************************
// Отправка команд инвертору
// Device dev     - ID инвертора
// InvCommand cmd - команда вкл/выкл/состояние/конфигурация
//*************************************************************************************************
static void InvSendCmnd( Device dev, InvCommand cmd ) {

    //инвертор TS-1000-224
    if ( dev == ID_DEV_INV1 ) {
        RecvClear( ID_DEV_INV1 );
        SendClear( ID_DEV_INV1 );
        if ( cmd & INV_CMD_CONFIG || cmd & INV_CMD_STAT )
            last_cmnd1 |= cmd; //только для циклических команд
        else last_cmnd1 = cmd;
        if ( last_cmnd1 & INV_CMD_CONFIG || last_cmnd1 & INV_CMD_STAT ) {
            //запускаем таймер ожидания ответа
            osTimerStop( timer_timeout1 ); 
            osTimerStart( timer_timeout1, TIMEOUT_ANSWER );
            //отправка циклической команды 
            if ( last_cmnd1 & INV_CMD_CONFIG ) {
                InvSend( dev, inv_comnd[INV_CMD_CONFIG] );
                //запускаем таймер циклической отправки
                osTimerStart( timer_status1, TIME_SEND_COMMNAD );
                return;
               }
            if ( last_cmnd1 & INV_CMD_STAT ) {
                InvSend( dev, inv_comnd[INV_CMD_STAT] );
                //запускаем таймер циклической отправки
                osTimerStart( timer_status1, TIME_SEND_COMMNAD );
                return;
               }
           } 
        else {
            //остановка цикличной отправки команды
            osTimerStop( timer_status1 );
            osTimerStop( timer_timeout1 );
            //отправка команды 
            if ( cmd != INV_CMD_NULL )
                InvSend( dev, inv_comnd[cmd] );
           }
       } 
    //инвертор TS-3000-224
    if ( dev == ID_DEV_INV2 ) {
        RecvClear( ID_DEV_INV2 );
        SendClear( ID_DEV_INV2 );
        if ( cmd & INV_CMD_CONFIG || cmd & INV_CMD_STAT )
            last_cmnd2 |= cmd; //только для циклических команд
        else last_cmnd2 = cmd;
        if ( last_cmnd2 & INV_CMD_CONFIG || last_cmnd2 & INV_CMD_STAT ) {
            //запускаем таймер ожидания ответа
            osTimerStop( timer_timeout2 ); 
            osTimerStart( timer_timeout2, TIMEOUT_ANSWER );
            //отправка циклической команды 
            if ( last_cmnd2 & INV_CMD_CONFIG ) {
                InvSend( dev, inv_comnd[INV_CMD_CONFIG] );
                //запускаем таймер циклической отправки
                osTimerStart( timer_status2, TIME_SEND_COMMNAD );
                return;
               }
            if ( last_cmnd2 & INV_CMD_STAT ) {
                InvSend( dev, inv_comnd[INV_CMD_STAT] );
                //запускаем таймер циклической отправки
                osTimerStart( timer_status2, TIME_SEND_COMMNAD );
                return;
               }
           } 
        else {
            //остановка цикличной отправки команды
            osTimerStop( timer_status2 );
            osTimerStop( timer_timeout2 );
            //отправка команды 
            if ( cmd != INV_CMD_NULL )
                InvSend( dev, inv_comnd[cmd] );
           }
       } 
 }

//*************************************************************************************************
// Оработка ответа инвертора TS-1000-224
//*************************************************************************************************
static void Inv1CheckAnswer( void ) {

    TypePack pack;
    
    //определим тип принятых данных и обработаем
    pack = TypeData( recv1_buffer );
    if ( pack == INV_PACK_EEPROM ) 
        if ( ParseData( ID_DEV_INV1, recv1_buffer, eeprom_msk, sizeof( eeprom_msk ) ) ) {
            //установим признак актуальности данных
            inv1.act_config = 1;
            //ответ получен, остановим таймер ожидания ответа
            osTimerStop( timer_timeout1 );
            //сбросим бит цикличности команды чтения конфигурации инвертора
            last_cmnd1 &= ~INV_CMD_CONFIG;
           }
    if ( pack == INV_PACK_STATUS )
        if ( ParseData( ID_DEV_INV1, recv1_buffer, status_msk, sizeof( status_msk ) ) ) {
            //установим признак актуальности данных
            inv1.act_status = 1;
            //ответ получен, остановим таймер ожидания ответа
            osTimerStop( timer_timeout1 );
           }
    RecvClear( ID_DEV_INV1 );
 }

//*************************************************************************************************
// Обработка ответа инвертора TS-1000-224
//*************************************************************************************************
static void Inv2CheckAnswer( void ) {

    TypePack pack;

    //определим тип принятых данных и обработаем
    pack = TypeData( recv2_buffer );
    if ( pack == INV_PACK_EEPROM ) 
        if ( ParseData( ID_DEV_INV2, recv2_buffer, eeprom_msk, sizeof( eeprom_msk ) ) ) {
            //установим признак актуальности данных
            inv2.act_config = 1;
            //ответ получен, остановим таймер ожидания ответа
            osTimerStop( timer_timeout2 );
            //сбросим бит цикличности команды
            last_cmnd2 &= ~INV_CMD_CONFIG;
           }
    if ( pack == INV_PACK_STATUS )
        if ( ParseData( ID_DEV_INV2, recv2_buffer, status_msk, sizeof( status_msk ) ) ) {
            //установим признак актуальности данных
            inv2.act_status = 1;
            //ответ получен, остановим таймер ожидания ответа
            osTimerStop( timer_timeout2 );
           }
    RecvClear( ID_DEV_INV2 );
 } 

//*************************************************************************************************
// Определение типа данных в ответе инвертора: EEPROM/статус
// char *data      - указатель на набор данных
// return TypePack - тип пакета данных от инвертора
//*************************************************************************************************
static TypePack TypeData( char *data ) {

    if ( !strlen( data ) )
        return INV_PACK_NODATA;
    //данные статуса
    if ( *data == 'C' && strlen( data ) == 1 )
        return INV_PACK_SHUTDOWN;
    //данные статуса
    if ( *data == '(' && *( data + strlen( data ) - 1 ) == ')' ) {
        //уберем лишние данные: "()"
        *(data + strlen( data ) - 1) = '\0';
        memmove( data, data + 1, strlen( data ) );
        return INV_PACK_STATUS;
       } 
    //данные EEPROM
    if ( *data == '#' && strstr( data, "MEANWELL" ) != NULL ) {
        //уберем лишние данные в начале пакета: "#"
        memmove( data, data + 1, strlen( data ) );
        return INV_PACK_EEPROM;
       } 
    //данные при запуске инвертора
    if ( strstr( data, "DOWNLOAD-1" ) != NULL || strstr( data, "F/W" ) != NULL || strstr( data, "S/N" ) != NULL )
        return INV_PACK_STARTUP;
    return INV_PACK_NODATA;
 }

//*************************************************************************************************
// Передача команды инвертору
// Device dev - ID устройства
// char *buff - указатель на строку команды
//*************************************************************************************************
static void InvSend( Device dev, const char *buff ) {

    if ( buff == NULL )
        return;
    if ( dev == ID_DEV_INV1 ) {
        SendClear( ID_DEV_INV1 );
        strcpy( send1_buffer, buff );
        UsartInv1->Send( send1_buffer, strlen( send1_buffer ) );
       }
    if ( dev == ID_DEV_INV2 ) {
        SendClear( ID_DEV_INV2 );
        strcpy( send2_buffer, buff );
        UsartInv2->Send( send2_buffer, strlen( send2_buffer ) );
       }
 }

//*************************************************************************************************
// Обнуление буфера передачи TS-1000-224/TS-3000-224
// Device dev - ID инвертора
//*************************************************************************************************
static void SendClear( Device dev ) {

    if ( dev == ID_DEV_INV1 )
        memset( send1_buffer, 0x00, sizeof( send1_buffer ) );
    if ( dev == ID_DEV_INV2 )
        memset( send2_buffer, 0x00, sizeof( send2_buffer ) );
 }

//*************************************************************************************************
// Обнуление буфера приема TS-1000-224/TS-3000-224
// Device dev - ID инвертора
//*************************************************************************************************
static void RecvClear( Device dev ) {

    if ( dev == ID_DEV_INV1 ) {
        recv1_ind = 0;
        memset( recv1_buffer, 0x00, sizeof( recv1_buffer ) );
       }
    if ( dev == ID_DEV_INV2 ) {
        recv2_ind = 0;
        memset( recv2_buffer, 0x00, sizeof( recv2_buffer ) );
       }
 }

//*************************************************************************************************
// Статус подключения DC питания инверторов с учетом включенного управления контакторами
// Device dev       - ID инвертора
// return BatStatus - состояние подключения инвертора к АКБ
//        BAT_DC_ON - АКБ подключена + контактор вкл
//*************************************************************************************************
BatStatus InvBatConn( Device dev ) {

    if ( StatCtrl() == false )
        return BAT_DC_UNKNOWN;
    if ( dev == ID_DEV_INV1 ) {
        if ( BatConn() && GetDataPort( TS1000_CHK ) )
            return BAT_DC_ON;
        else return BAT_DC_OFF;
       }
    if ( dev == ID_DEV_INV2 ) {
        if ( BatConn() && GetDataPort( TS3000_CHK ) )
            return BAT_DC_ON;
        else return BAT_DC_OFF;
       }
    return BAT_DC_UNKNOWN;
 }

//*************************************************************************************************
// Статус подключения DC питания инверторов без учета включенного управления контакторами
// Device dev         - ID инвертора
// return InvDcStatus - состояние подключения инвертора к АКБ
//        INV_DC_ON   - АКБ подключена + контактор вкл
//*************************************************************************************************
static InvDcStatus InvDcConn( Device dev ) {

    if ( dev == ID_DEV_INV1 ) {
        if ( BatConn() == true && GetDataPort( TS1000_CHK ) )
            return INV_DC_ON;
        else return INV_DC_OFF;
       }
    if ( dev == ID_DEV_INV2 ) {
        if ( BatConn() == true && GetDataPort( TS3000_CHK ) )
            return INV_DC_ON;
        else return INV_DC_OFF;
       }
    return INV_DC_OFF;
 }

//*************************************************************************************************
// Разбор параметров ответа инвертора, данные загружаются в структуры типа INVERTER
// Device dev          - ID инвертора
// char *data          - указатель на массив с данными ответа
// uint8_t *mask_parse - массив масок разбора в зависимости от типа пакета
// uint8_t cnt_param   - кол-во элементов в маске разбора
//*************************************************************************************************
static uint8_t ParseData( Device dev, char *data, const uint8_t *mask_parse, uint8_t cnt_param ) {

    uint8_t param = 0;
    char *str, *token, *saveptr;

    str = data;
    while ( 1 ) {
        //разбор параметров
        token = strtok_r( str, " ", &saveptr );
        if ( token == NULL )
            break;
        str = saveptr;
        if ( mask_parse[param] != INV_DATA_NOTUSED )
            InvGetData( dev, (ParamInv)mask_parse[param], token );
        param++; //следующий параметр
        if ( param > cnt_param )
            return 0; //превышение кол-ва параметров
       }
    return param + 1;
 }

//*************************************************************************************************
// Обнуление данных инвертора
// Device dev   - ID инвертора
// uint8_t mask - маска очистки данных
//*************************************************************************************************
static void ClrData( Device dev, uint8_t mask  ) {

    INVERTER *inv = NULL;
    
    if ( dev == ID_DEV_INV1 )
        inv = &inv1;
    if ( dev == ID_DEV_INV2 )
        inv = &inv2;
    if ( inv == NULL )
        return;
    if ( mask & CLR_CONFIG ) {
        //параметры EEPROM
        inv->act_config = 0;
        inv->cfg_eql_volt = 0;
        inv->cfg_flt_volt = 0;
        inv->cfg_alarm_volt = 0;
        inv->cfg_shdn_volt = 0;
        memset( inv->vendor, 0x00, sizeof( inv->vendor ) );
        memset( inv->version, 0x00, sizeof( inv->version ) );
        memset( inv->model, 0x00, sizeof( inv->model ) );
       }
    if ( mask & CLR_DATA ) {
        //параметры статуса
        inv->act_status = 0;
        inv->ac_out = 0;
        inv->ac_power = 0;
        inv->dc_in = 0;
        inv->bat_perc = 0;
        inv->temperature = 0;
        inv->unused = 0;
        inv->ac_freq = 0;
        inv->work_time = 0;
        inv->power_perc = 0;
        inv->power_watt = 0;
        inv->mode = INV_MODE_NO_LINK;
       }
    if ( mask & CLR_STATUS ) {
        inv->bit_status = 0;
        inv->dev_error = 0;
        inv->ctrl_error = INV_ERR_CTRL_OK;
       } 
    if ( dev == ID_DEV_INV1 )
        inv->dc_conn = InvDcConn( ID_DEV_INV1 );
    if ( dev == ID_DEV_INV2 )
        inv->dc_conn = InvDcConn( ID_DEV_INV2 );
 }

//*************************************************************************************************
// Обработка ответа инвертора и заполнение структуры INVERTER данными
// Device dev     - ID инвертора
// ParamInv param - ID параметра
// char *data     - указатель на данные одного параметра
//*************************************************************************************************
static void InvGetData( Device dev, ParamInv param, char *data ) {

    uint8_t i;
    INVERTER *inv = NULL;
    uint32_t bit_mask = TS_MSK_INVMODE;
    
    if ( dev == ID_DEV_INV1 )
        inv = &inv1;
    if ( dev == ID_DEV_INV2 )
        inv = &inv2;
    if ( inv == NULL )
        return;
    //параметры EEPROM
    if ( param == INV_CFG_EQL_VOLT )
        inv->cfg_eql_volt = atof( data );
    if ( param == INV_CFG_FLT_VOLT )
        inv->cfg_flt_volt = atof( data );
    if ( param == INV_CFG_ALARM_VOLT )
        inv->cfg_alarm_volt = atof( data );
    if ( param == INV_CFG_SHDN_VOLT )
        inv->cfg_shdn_volt = atof( data );
    if ( param == INV_VENDOR )
        strcpy( inv->vendor, data );
    if ( param == INV_VERSION )
        strcpy( inv->version, data );
    if ( param == INV_MODEL )
        strcpy( inv->model, data );
    //параметры статуса
    if ( param == INV_AC_OUT )
        inv->ac_out = atoi( data );
    if ( param == INV_AC_POWER )
        inv->ac_power = atoi( data );
    if ( param == INV_DC_IN )
        inv->dc_in = atof( data );
    if ( param == INV_BAT_PERC )
        inv->bat_perc = atoi( data );
    if ( param == INV_TEMPERATURE )
        inv->temperature = atof( data );
    if ( param == INV_UNUSED )
        inv->unused = atoi( data );
    if ( param == INV_AC_FREQ )
        inv->ac_freq = atof( data );
    if ( param == INV_WORK_TIME )
        inv->work_time = atoi( data );
    if ( param == INV_POWER_PERC ) {
        inv->power_perc = atoi( data );
        //рассчитаем фактическую мощность нагрузки
        if ( dev == ID_DEV_INV1 )
            inv->power_watt = (uint16_t)( 1000 * (float)( (float)inv->power_perc/100 ) );
        else inv->power_watt = (uint16_t)( 3000 * (float)( (float)inv->power_perc/100 ) );
       } 
    if ( param == INV_STATUS && strlen( data ) == TS_MSK_MAX_SIZE ) {
        inv->bit_status = 0;
        inv->dev_error = 0;
        //разбор битового параметра
        for ( i = 0; i < strlen( data ); i++, bit_mask >>= 1 ) {
            if ( *(data+i) & 0x01 )
                inv->bit_status |= bit_mask;
           }     
        //выделим коды ошибок для отображения в консоли, отображается
        //только одна ошибка, для отображения нескольких ошибок 
        //надо выполнить дополнительную обработку нескольких бит
        if ( inv->bit_status & TS_MSK_EEPROM_ERR )
            inv->dev_error = TS_ERR_EEPROM;
        if ( inv->bit_status & TS_MSK_BAT_UUP )
            inv->dev_error = TS_ERR_BAT;
        if ( inv->bit_status & TS_MSK_BAT_LOW )
            inv->dev_error = TS_ERR_BATLOW;
        if ( inv->bit_status & TS_MSK_OVR_100_115 )
            inv->dev_error = TS_ERR_OVR100;
        if ( inv->bit_status & TS_MSK_OVR_115_150 )
            inv->dev_error = TS_ERR_OVR115;
        if ( inv->bit_status & TS_MSK_OVR_150 )
            inv->dev_error = TS_ERR_OVR150;
        if ( inv->bit_status & TS_MSK_OVERHEAT )
            inv->dev_error = TS_ERR_OVRHEAT;
        if ( inv->bit_status & TS_MSK_ERROR )
            inv->dev_error = TS_ERR_ERROR;
        //выделяем состояние инвертора
        if ( inv->bit_status & TS_MSK_INVMODE )
            inv->mode = INV_MODE_ON;
        if ( inv->bit_status & TS_MSK_SAVING )
            inv->mode = INV_MODE_SAVE;
        //состояние "выкл удаленно" приоритетно, 
        //т.к. может совмещаться с состоянием "выключен"
        if ( inv->bit_status & TS_MSK_POWER_OFF )
            inv->mode = INV_MODE_OFF;
        if ( inv->bit_status & TS_MSK_RMT_SHDN )
            inv->mode = INV_MODE_RMT_OFF;
       } 
    //статус подключения к АКБ не входит в маску разбора параметров
    if ( dev == ID_DEV_INV1 )
        inv->dc_conn = InvDcConn( ID_DEV_INV1 );
    if ( dev == ID_DEV_INV2 )
        inv->dc_conn = InvDcConn( ID_DEV_INV2 );
 }

//*************************************************************************************************
// Вывод результата выполнения циклограммы управления инвертором TS-1000-224
//*************************************************************************************************
static void Inv1ResulOut( void ) {

    char msg[80];

    //вывод сообщения об ошибке в консоль, если есть
    if ( inv1.ctrl_error )
        sprintf( msg, "\r\n%s %s\r\n", MessageLog( ID_DEV_INV1, LOG_MSG_INV1 ), ErrorDescr( ID_DEV_INV1, 0, inv1.ctrl_error ) );
    else sprintf( msg, "\r\n%s\r\n", MessageLog( ID_DEV_INV1, LOG_MSG_INV1_OK ) );
    ConsoleSend( msg, CONS_SELECTED );
 }
 
//*************************************************************************************************
// Вывод результата выполнения циклограммы управления инвертором TS-3000-224
//*************************************************************************************************
static void Inv2ResulOut( void ) {

    char msg[80];

    //вывод сообщения об ошибке в консоль, если есть
    if ( inv2.ctrl_error )
        sprintf( msg, "\r\n%s %s\r\n", MessageLog( ID_DEV_INV2, LOG_MSG_INV2 ), ErrorDescr( ID_DEV_INV2, 0, inv2.ctrl_error ) );
    else sprintf( msg, "\r\n%s\r\n", MessageLog( ID_DEV_INV2, LOG_MSG_INV2_OK ) );
    ConsoleSend( msg, CONS_SELECTED );
 }

//*************************************************************************************************
// Запись в протокол состояние вкл/выкл для TS-1000-224
// Записывает в протокол сообщение и описание ошибки (если указан код ошибки)
// char *text     - текстовая строка добавляемая в файл протокола
// uint32_t error - код ошибки
// Если error = 0 - запишем только текст сообщения
// Если error > 0 - соответствует маске ERR_MSK_TS запишем текст сообщения об ошибке, при этом
//                  значение в поле text игнорируется  
//*************************************************************************************************
static void EventLog1( char *text, InvCtrlError error ) {

    FILE *log;
    char name[40];
    
    if ( SDStatus() == ERROR )
        return; //карты нет
    if ( !config.log_enable_inv )
        return;
    if ( !config.mode_logging )
        sprintf( name, "\\inv\\inv1_%s.log", RTCFileName() );
    else sprintf( name, "\\inv\\%s\\inv1_%s.log", RTCFileShort(), RTCFileName() );
    log = fopen( name, "a" );
    if ( log == NULL )
        return;
    if ( error ) {
        //указан код ошибки
        fprintf( log, "%s Шаг: %d код ошибки: %u %s\r\n", RTCGetLog(), inv1.cycle_step, error, ErrorDescr( ID_DEV_INV1, 0, error ) );
        fclose( log );
        return;
       }
    //запись только текста сообщения
    fprintf( log, "%s %s\r\n", RTCGetLog(), text );
    fclose( log );
 }

//*************************************************************************************************
// Запись в протокол состояние вкл/выкл для TS-3000-224
// Записывает в протокол сообщение и описание ошибки (если указан код ошибки)
// char *text     - текст сообщения
// uint32_t error - код ошибки
// Если error = 0 - запишем только текст сообщения
// Если error > 0 и < кодов ошибок - запишем текст сообщения и шестн. расшифровку кода
//*************************************************************************************************
static void EventLog2( char *text, InvCtrlError error ) {

    FILE *log;
    char name[40];
    
    if ( SDStatus() == ERROR )
        return; //карты нет
    if ( !config.log_enable_inv )
        return;
    //открываем файл
    if ( !config.mode_logging )
        sprintf( name, "\\inv\\inv2_%s.log", RTCFileName() );
    else sprintf( name, "\\inv\\%s\\inv2_%s.log", RTCFileShort(), RTCFileName() );
    log = fopen( name, "a" );
    if ( log == NULL )
        return;
    if ( error ) {
        //указан код ошибки
        fprintf( log, "%s Шаг: %d код ошибки: %u %s\r\n", RTCGetLog(), inv2.cycle_step, error, ErrorDescr( ID_DEV_INV1, 0, error ) );
        fclose( log );
        return;
       }
    //запись только текста сообщения
    fprintf( log, "%s %s\r\n", RTCGetLog(), text );
    fclose( log );
 }

//*************************************************************************************************
// Пишет в протокол "inv_yyyymmdd.csv" данные по обоим инверторам
// Вызов из TSCheckAnswer после разбор данных
//*************************************************************************************************
static void InvSaveLog( void ) {

    uint32_t pos;
    char name[40];
    FILE *ts_log;

    if ( !config.log_enable_inv )
        return; //логирование выключено
    if ( SDStatus() == ERROR )
        return; //карты нет
    if ( inv1.mode != INV_MODE_ON && inv2.mode != INV_MODE_ON )
        return; //оба инвертора выключены
    //открываем файл
    if ( !config.mode_logging )
        sprintf( name, "\\inv\\inv_%s.csv", RTCFileName() );
    else sprintf( name, "\\inv\\%s\\inv_%s.csv", RTCFileShort(), RTCFileName() );
    ts_log = fopen( name, "a" );
    if ( ts_log == NULL )
        return; //файл не открылся
    pos = ftell( ts_log );
    if ( !pos )
        fputs( "Date;Time;Pwr1(%);Pwr1(W);Temp1(C);Conn1;Mode1;Error1;Pwr3(%);Pwr3(W);Temp3(C);Conn3;Mode3;Error3;\r\n", ts_log ); //запишем наименование полей
    //запишем данные
    fprintf( ts_log, "%s;%s;%d;%d;%4.1f;%s;%s;%s;%d;%d;%4.1f;%s;%s;%s\r\n", 
        RTCGetDate( NULL ), RTCGetTime( NULL ), 
        inv1.power_perc, inv1.power_watt, inv1.temperature, 
        inv1.dc_conn == INV_CTRL_ON ? "Да " : "Нет", 
        ParamGetDesc( ID_DEV_INV1, INV_MODE ),
        ErrorDescr( ID_DEV_INV1, inv1.dev_error, 0 ),
        inv2.power_perc, inv2.power_watt, inv2.temperature, 
        inv2.dc_conn == INV_CTRL_ON ? "Да " : "Нет", 
        ParamGetDesc( ID_DEV_INV2, INV_MODE ),
        ErrorDescr( ID_DEV_INV2, inv2.dev_error, 0 ) ); 
    fclose( ts_log );
 }
