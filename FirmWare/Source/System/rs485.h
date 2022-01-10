
#ifndef __RS485_H
#define __RS485_H

#include <stdbool.h>
#include <stdint.h>

#include "device.h"

typedef enum {
    RS485_SEND_RTU,                         //режим двоичный передачи 
    RS485_SEND_ASCII                        //режим текстовый передачи
 } RS485Mode;

//Коды возврата функции "RS485Send"
typedef enum {
    RS485_SEND_OK,                          //Функция выполнена
    RS485_SEND_NO,                          //Передача предыдущего пакета не завершена
    RS485_SEND_ERR,                         //Неверные параметры вызова функции
    RS485_SEND_FULL                         //Передаваемый пакет больше буфера
 } RS485StatSend;

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void RS485Init( void );
void ClearRecv( void );
RS485StatSend RS485Send( uint8_t *data, uint16_t len, RS485Mode mode );
uint8_t *RS485Recv( uint16_t *len );

#endif
