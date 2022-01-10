
#ifndef __MODBUS_H
#define __MODBUS_H

#include <stdbool.h>
#include <lpc177x_8x.h>
#include <lpc_types.h>

#include "device.h"
#include "dev_data.h"

#define CNT_RECV_CHECK          3       //кол-во принятых байт для предварительной проверки пакета

#define CNT_REPEAT_REQST        3       //кол-во попыток отправки запроса если уст-во не отвечает

//пересчет кол-ва бит в кол-во байт
#define CALC_BYTE( bits )       ( bits%8 ? (bits/8)+1 : bits/8 )

// Тип данных для логирования
typedef enum {
    DATA_LOG_REQUEST,                   //запрос (автоматическая запись в файл)
    DATA_LOG_ANSWER                     //ответ (автоматическая запись в файл)
 } DataLogMode;

typedef enum {
    DATA_LOG_NO_FILE,                   //без записи в файл
    DATA_LOG_FILE                       //автоматическая запись в файл
 } DataLogFile;

#pragma pack( push, 1 )                 //выравнивание структуры по границе 1 байта

//*************************************************************************************************
// Структура передачи запроса по MODBUS
//*************************************************************************************************
typedef struct {
    uint8_t  dev_addr;                  //Адрес устройства
    uint8_t  function;                  //Функциональный код
    uint16_t addr_reg;                  //Адрес первого регистра (HI/LO байт)
    uint16_t cnt_reg;                   //Количество регистров (HI/LO байт)
    void     *ptr_data;                 //Указатель на буфер для размещения передаваемых данных 
                                        //регистров, после приема - указатель на принятые данные
    uint16_t *ptr_lendata;              //Указатель на переменную размера буфера для приема данных
} MBUS_REQUEST;                         //после приема - кол-во принятых байт без служебной информации

#pragma pack( pop )

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void ModBusInit( void );
void ModBusDebug( void  );
Status ModbusLog( Mode mode );
void ModBusErrClr( void );

//*************************************************************************************************
// Функции статуса/состояния
//*************************************************************************************************
uint16_t PackSize( uint8_t *data );
uint32_t ModBusErrCnt( ModBusError error );
char *ModBusErrDesc( ModBusError error );
char *ModBusErrCntDesc( ModBusError err_ind, char *str );
ModbusFunc ModBusFunc( uint8_t func );
ModbusAddrReg ModBusAddr( uint8_t func );
ModBusError ModBusRequest( MBUS_REQUEST *reqst );
char *ModBusDecode( MBUS_REQUEST *reqst, DataLogMode mode, DataLogFile log );

#endif
