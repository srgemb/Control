
//*************************************************************************************************
//
// Обмен данными по UART
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

#include "priority.h"
#include "uart.h"
#include "ring_uart.h"
#include "events.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern ARM_DRIVER_USART Driver_USART0;

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osSemaphoreId_t uart_semaphore = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define RECV_BUFF               200         //размер приемного буфера

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static uint16_t recv_ind;
static ARM_DRIVER_USART *USARTdrv;
static char recv_ch, recv_buffer[RECV_BUFF];

static osEventFlagsId_t uart_event = NULL;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t uart_attr = {
    .name = "Uart", 
    .stack_size = 256,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn_attr = { .name = "UArt" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void CallBackUart( uint32_t event );
static void TaskUart( void *pvParameters );

//*************************************************************************************************
// Инициализация консоли UART0/IRQ5/115200
//*************************************************************************************************
void UartInit( void ) {

    UartRecvClear();
    //очередь сообщений
    uart_event = osEventFlagsNew( &evn_attr );
    //создаем задачу управления обменом по UART
    osThreadNew( TaskUart, NULL, &uart_attr );
    //настройка порта
    USARTdrv = &Driver_USART0;
    USARTdrv->Initialize( &CallBackUart );
    NVIC_SetPriority( UART0_IRQn, NVIC_EncodePriority( NVIC_GetPriorityGrouping(), PRIORITY_USART, SUB_PRIORITY_USART0 ) );
    USARTdrv->PowerControl( ARM_POWER_FULL );
    USARTdrv->Control( ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | ARM_USART_PARITY_NONE | 
                       ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE, 115200 );
    USARTdrv->Control( ARM_USART_CONTROL_TX, 1 );
    USARTdrv->Control( ARM_USART_CONTROL_RX, 1 );    
    USARTdrv->Receive( &recv_ch, 1 );
 }

//*************************************************************************************************
// Задача обработки сообщений обмена по UART
//*************************************************************************************************
static void TaskUart( void *pvParameters ) {

    uint32_t event;
    
    for ( ;; ) {
        //ждем события
        event = osEventFlagsWait( uart_event, EVN_UART_BUSY, osFlagsWaitAny, osWaitForever );
        if ( event & EVN_UART_BUSY )
            UartSendStart(); //запускаем отправку
       }
 }

//*************************************************************************************************
// Обработчик событий последовательного порта
//*************************************************************************************************
static void CallBackUart( uint32_t event ) {

    char ch;
    
    if ( event & ARM_USART_EVENT_RECEIVE_COMPLETE ) {
        //принят один байт
        if ( recv_ch != KEY_ESC ) {
            if ( recv_ch == KEY_BS ) {
                //получен BS, стираем последний введенный символ
                if ( recv_ind )
                    recv_buffer[--recv_ind] = '\0';
                USARTdrv->Receive( &recv_ch, 1 );
                return;
               }
            recv_ind += USARTdrv->GetRxCount();
            if ( recv_ind <= RECV_BUFF )
                recv_buffer[recv_ind-1] = recv_ch; //сохраним байт в буфере
            else UartRecvClear(); //буфер переполнен, сбросим буфер
            //повторная инициализация приема
            USARTdrv->Receive( &recv_ch, 1 );
            if ( recv_ind > 0 && recv_buffer[recv_ind-1] == KEY_CR ) {
                recv_buffer[recv_ind-1] = '\0'; //уберем код CR
                //нажали клавишу Enter, передаем событие в задачу "Command"
                osEventFlagsSet( command_event, EVN_COMMAND_CR );
               } 
           }
        else {
            //нажали клавишу ESC, передаем событие в задачу "Command"
            osEventFlagsSet( command_event, EVN_COMMAND_ESC );
            //повторная инициализация приема
            USARTdrv->Receive( &recv_ch, 1 );
           } 
        }
    if ( event & ARM_USART_EVENT_SEND_COMPLETE ) {
        //передача завершена
        if ( RingGetChar( &ch ) ) {
            USARTdrv->Send( &ch, 1 );
            RingCheckFree(); //снимем семафор если в буфере много свободного места
           }
       }
 }
 
//*************************************************************************************************
// Вывод строки в UART. Строка предварительно помещается в кольцевой буфер.
// char *str - указатель на строку для передач
//*************************************************************************************************
void UartSendStr( char *str ) {

    uint16_t ind_str = 0, buf_len, str_len, tmp_len;
    
    //вывод в последовательный порт
    if ( str == NULL || !strlen( str ) )
        return;
    //проверка на превышение размера буфера
    str_len = strlen( str );
    buf_len = RingGetSize();
    if ( str_len > buf_len ) {
        //длина строки больше размера кольцевого буфера, 
        //отправлять будем по частям, по 1/2 от размера буфера
        tmp_len = buf_len / 2;
        while ( true ) {
            //запрос доступного места в кольцевом буфере
            if ( RingGetAdd( tmp_len ) ) {
                RingAddStrLen( str + ind_str, tmp_len ); //место есть
                if ( tmp_len < ( buf_len / 2 ) )
                    break; //передали последний блок
                ind_str += tmp_len;
                if ( ( str_len - tmp_len ) > tmp_len )
                    str_len -= tmp_len;             //остаток для передачи
                else tmp_len = str_len - tmp_len;   //размер последнего блока
               }
            else { 
                //ждем пока освободится семафор буфера
                osSemaphoreAcquire( uart_semaphore, osWaitForever );
               }
           }
        return;
       }
    //запрос доступного места в кольцевом буфере
    if ( RingGetAdd( strlen( str ) ) )
        RingAddStr( str ); //место есть
    else { 
        //ждем пока освободится семафор буфера
        osSemaphoreAcquire( uart_semaphore, osWaitForever );
       }
 }
//*************************************************************************************************
// Возвращает адрес приемного буфера 
// return char* - указатель на буфер
//*************************************************************************************************
char *UartBuffer( void ) {

    return recv_buffer;
 }

//*************************************************************************************************
// Очистка буфера приема
//*************************************************************************************************
void UartRecvClear( void ) {

    recv_ind = 0;
    memset( recv_buffer, 0, sizeof( recv_buffer ) );
 }

//*************************************************************************************************
// Запуск вывода из кольцевого буфера в UART, если UART во время вызова функции занят - отправим 
// сообщение в задачу TaskUart() для запуска передачи
//*************************************************************************************************
void UartSendStart( void ) {

    char ch;
    ARM_USART_STATUS stat;
    
    stat = USARTdrv->GetStatus();
    if ( stat.tx_busy ) {
        //UART занят, установим сигнал EVN_UART_BUSY
        osEventFlagsSet( uart_event, EVN_UART_BUSY );
        return;
       }
    if ( RingGetChar( &ch ) )
        USARTdrv->Send( &ch, 1 );
 }

