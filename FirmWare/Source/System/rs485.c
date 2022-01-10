
//*************************************************************************************************
//
// Управление внешним интерфейсом RS485 (MAX3100)
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "ssp_lpc17xx.h"
#include "lpc177x_8x_gpio.h"

#include "rtc.h"
#include "rs485.h"
#include "max3100.h"
#include "ports.h"
#include "command.h"
#include "modbus.h"
#include "priority.h"
#include "events.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern ARM_DRIVER_SPI Driver_SPI1;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define BUFFER_SIZE         256             //размер приемного/передающего буфера

#define DELAY_RTS           600U            //задержка выкл сигнала RTS после начала передачи
                                            //последнего байта (usec)

#define IRQ_485_PIN         18              //номер бита прерывания от MAX3100
#define IRQ_485_MASK        (1 << 18)       //маска бита прерывания от MAX3100


//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
ARM_DRIVER_SPI *SpiDrv1;

static bool flg_rts = false;
static uint16_t len_send, recv_ind, send_ind, pack_len = 0;
static uint8_t recv_buffer[BUFFER_SIZE];
static uint8_t send_buffer[BUFFER_SIZE];
static osEventFlagsId_t rs485_event;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t rs485_attr = {
    .name = "Rs485",
    .stack_size = 512,
    .priority = osPriorityHigh7
 };

static const osEventFlagsAttr_t evn_attr = { .name = "Rs485" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void ClearSend( void );
static void SendConfig( uint16_t data );
static void Mode485Recv( void );
static void IRQ_RS485( void );
static uint16_t RecvData( void );
static void SendData( uint8_t data );
static void TaskRs485( void *pvParameters );

//*************************************************************************************************
// Инициализация SPI для обмена с MAX3100
// SPI1 - 1MHz
//*************************************************************************************************
void RS485Init( void ) {

    ClearSend();
    ClearRecv();
    //очередь сообщений
    rs485_event = osEventFlagsNew( &evn_attr );
    //создаем задачу
    osThreadNew( TaskRs485, NULL, &rs485_attr );
    //инициализация SPI интерфейса для MAX3100
    SpiDrv1 = &Driver_SPI1;
    SpiDrv1->Initialize( NULL );
    NVIC_SetPriority( SSP1_IRQn, NVIC_EncodePriority( NVIC_GetPriorityGrouping(), PRIORITY_SSP, SUB_PRIORITY_SSP1 ) );
    SpiDrv1->PowerControl( ARM_POWER_FULL );
    SpiDrv1->Control( ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_MSB_LSB | ARM_SPI_SS_MASTER_SW | ARM_SPI_DATA_BITS( 16 ), 1000000 );
    SpiDrv1->Control( ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE );
    //настройка входа прерывания
    GPIO_SetDir( PORT0, IRQ_485_PIN, GPIO_DIR_INPUT );
    GPIO_IntCmd( PORT0, IRQ_485_MASK, 1 );  //Falling edge - по спаду
    //настройка прерывания
    NVIC_SetPriority( GPIO_IRQn, NVIC_EncodePriority( NVIC_GetPriorityGrouping(), PRIORITY_GPIO, 0 ) );
    NVIC_EnableIRQ( GPIO_IRQn );
    //включаем режим приема MAX3100, RTS=0
    Mode485Recv();                  
    //конфигурирование MAX3100, UART=19200 bps(bod*sec) 
    //19200/10 = 1920 byte*sec, 1 byte = 1/1920 = 520 us
    //Завершение передачи - отсутствие передачи данных в течении 1,5
    //времени передачи одного байта данных 520 * 1.5 = 780 us
    SendConfig( BAUD_19200 | IRQ_TM | IRQ_RM | FIFO_ENABLE );
 }

//*************************************************************************************************
// Обработка внешнего прерывания IRQ_485 от MAX3100
//*************************************************************************************************
void GPIO_IRQHandler( void ) {

    if ( GPIO_GetIntStatus( PORT0, IRQ_485_PIN, 1 ) == ENABLE ) {
        //обработка прерывания сигнала IRQ от MAX3100
        GPIO_ClearInt( PORT0, IRQ_485_MASK );
        //передаем событие
        osEventFlagsSet( rs485_event, EVN_RS485_IRQ );
       }
 }

//*************************************************************************************************
// Задача обработки прерывания от MAX3100
//*************************************************************************************************
static void TaskRs485( void *pvParameters ) {

    uint32_t event;
    
    for ( ;; ) {
        //ждем события от прерывания
        event = osEventFlagsWait( rs485_event, EVN_RS485_MASK, osFlagsWaitAny, osWaitForever );
        if ( event == EVN_RS485_IRQ )
            IRQ_RS485();
       }
 }

//*************************************************************************************************
// Обработка внешнего прерывания IRQ_485 от MAX3100
//*************************************************************************************************
static void IRQ_RS485( void ) {

    uint16_t stat;
    uint32_t msg, tick, timeout;

    stat = RecvData();
    timeout = DELAY_RTS * ( osKernelGetSysTimerFreq() / 960000U );
    //проверим наличие флагов прерывания MAX3100
    if ( stat & IRQ_MASK_T ) {
        //передача одного байта завершена, прерывание формируется сразу после передачи 
        //байта в буфер передатчика, т.е. в начале передачи. Переключение сигнала RTS 
        //(переход на прием) надо выполнить с задержкой DELAY_RTS после появления IRQ
        if ( len_send ) {
            len_send--; //есть еще данные для передачи
            SendData( send_buffer[send_ind++] ); //передача следующего байта
           } 
        else if ( flg_rts == true ) {
            //снимем флаг flg_rts, для исключения повторного входа сюда, т.к. 
            //IRQ_MASK_T (буфер передатчика пуст) остается активным после завершения передачи 
            flg_rts = false;
            osKernelLock(); //начало критической секция кода
            //задержка на переключение сигнала RTS после передачи последнего байта
            tick = osKernelGetSysTimerCount();
            while ( ( osKernelGetSysTimerCount() - tick ) < timeout );
            Mode485Recv(); //RTS=0 - переход на прием
            osKernelUnlock(); //окончание критической секция кода
           }
       }
    if ( stat & IRQ_MASK_R ) {
        //прием одного байта, сброс прерывания
        if ( recv_ind < BUFFER_SIZE )
            recv_buffer[recv_ind++] = (uint8_t)( 0x00FF & stat );
        else ClearRecv(); //переполнение буфера
        if ( recv_ind == CNT_RECV_CHECK ) {
            pack_len = PackSize( recv_buffer ); //расчет размера пакета с ответом
            msg = MSG_MODBUS_CHECK; //перезапуск таймера TIMEOUT_ANSWER ответа
            osMessageQueuePut( modbus_queue, &msg, 0, 0 ); 
           }
        if ( recv_ind == pack_len ) {
            msg = MSG_MODBUS_RECV; //пакет получен полностью
            osMessageQueuePut( modbus_queue, &msg, 0, 0 );  
           }
       }
 }

//*************************************************************************************************
// Очистка буфера передачи
//*************************************************************************************************
static void ClearSend( void ) {

    send_ind = 0;
    len_send = 0;
    memset( send_buffer, 0x00, sizeof( send_buffer ) );
 }

//*************************************************************************************************
// Очистка буфера приема
//*************************************************************************************************
void ClearRecv( void ) {

    recv_ind = 0;
    memset( recv_buffer, 0x00, sizeof( recv_buffer ) );
 }

//*************************************************************************************************
// Запись конфигурации в MAX3100
// uint16_t data - параметры конфигурации
//*************************************************************************************************
static void SendConfig( uint16_t data ) {
    
    uint16_t cmnd;

    cmnd = CMD_WR_CONFIG | data;            
    SpiDrv1->Control( ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE );
    SpiDrv1->Send( &cmnd, 1 );
    SpiDrv1->Control( ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE );
 }

//*************************************************************************************************
// Включить режим приема данных в MAX31000
// RTS=0 - режим приема данных через порт RS-485
//*************************************************************************************************
static void Mode485Recv( void ) {

    uint16_t cmnd;
    
    //включить режим приема, TE=1, RTS=1
    cmnd = CMD_WR_DATA | DATA_TE | DATA_RTS;
    SpiDrv1->Control( ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE );
    SpiDrv1->Send( &cmnd, 1 );
    SpiDrv1->Control( ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE );
 }

//*************************************************************************************************
// Передача 1 байта данных в MAX3100
// uint8_t data - данные для передачи
//*************************************************************************************************
static void SendData( uint8_t data ) {
    
    uint16_t cmnd;

    //включаем режим передачи, TE=0, RTS=0
    cmnd = CMD_WR_DATA | (uint16_t)data;            
    SpiDrv1->Control( ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE );
    SpiDrv1->Send( &cmnd, 1 );
    SpiDrv1->Control( ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE );
 }

//*************************************************************************************************
// Прием 1 байта данных из MAX3100
// result - состояние и принятые данные 
//*************************************************************************************************
static uint16_t RecvData( void ) {

    uint16_t cmnd, recv;
    
    cmnd = CMD_RD_DATA;
    SpiDrv1->Control( ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE );
    SpiDrv1->Transfer( &cmnd, &recv, 1 );
    SpiDrv1->Control( ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE );
    return recv;
 }

//*************************************************************************************************
// Запуск передачи пакета данных через порт RS-485
// uint8_t *data  - адрес блока данных для передачи
// uint16_t len   - размер передаваемых данных для двоичного режима
// RS485Mode mode - режим передачи данных текстовый/двоичный
//*************************************************************************************************
RS485StatSend RS485Send( uint8_t *data, uint16_t len, RS485Mode mode ) {

    if ( data == NULL )
        return RS485_SEND_ERR;          //не указан размер данных для передачи
    if ( mode == RS485_SEND_RTU && !len )
        return RS485_SEND_ERR;          //не указан размер данных для передачи
    if ( mode == RS485_SEND_ASCII )
        len = strlen( (char *)data );   //только для текстового режима
    if ( len >= BUFFER_SIZE )
        return RS485_SEND_FULL;         //превышение размера буфера
    //включаем режим передачи
    ClearSend();
    ClearRecv();
    memcpy( send_buffer, data, len );    //буфер пустой, передаем "0" в кол-ве len
    flg_rts = true;                      //признак запуска передачи  
    len_send = len-1;                    //первый байт уже передали
    SendData( send_buffer[send_ind++] ); //инициируем начало передачи данных
    return RS485_SEND_OK;
 }

//*************************************************************************************************
// Возвращает указатель на приемный буфер порта RS-485
// uint8_t *len - адрес переменной для сохранения размера принятого блока данных
// result       - адрес массива с данными или NULL
//                len = 0 - данных нет
//                len > 0 - кол-во принятых данных
//*************************************************************************************************
uint8_t *RS485Recv( uint16_t *len ) {

    if ( !recv_ind ) {
        *len = 0;
        return NULL;
       } 
    *len = recv_ind;
    return recv_buffer;
 }
