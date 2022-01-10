
//*************************************************************************************************
//
// Управление протоколом MODBUS
//
//*************************************************************************************************

#include <lpc177x_8x.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "ports.h"
#include "command.h"    //////////

#include "device.h"
#include "dev_param.h"

#include "rs485.h"
#include "modbus.h"
#include "events.h"
#include "crc16.h"
#include "sdcard.h"

#include "modbus_def.h"
#include "tracker_ext.h"
#include "voice_ext.h"
#include "gen_ext.h"

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osMessageQueueId_t modbus_queue = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define RS485_MODE_SEND         1
#define RS485_MODE_RECV         0

#define DEV_ERROR_CNT           8           //кол-во устройств по которым ведется учет ошибок

#define MAX_REGS_OUT_LOG        10          //максимальное кол-во регистров выводимых в лог файл
                                            //при увеличении значения необходимо увеличить
                                            //размер буфера str_log[128]

#define TIMEOUT_ANSWER          100         //время ожидания ответа от уст-ва (msec)

#define MB_ANSWER_DEV           0           //индекс ID уст-ва ответа
#define MB_ANSWER_FUNC          1           //индекс кода функции ответа
#define MB_ANSWER_COUNT         2           //индекс кол-ва байт данных ответа (без CRC)
#define MB_ANSWER_ERROR         2           //индекс кода ошибки
#define MB_ANSWER_DATA_RD       3           //индекс начала данных в пакете (ответ на чтение регистров)
#define MB_ANSWER_DATA_WR       2           //индекс начала данных в пакете
#define MB_ANSWER_WRREG         4           //размер данных ответа при записи регистров (без служебной информации)

#define SIZE_CRC                2           //кол-во байт для хранения КС
#define SIZE_PACK_HEADER        3           //кол-во байт заголовка пакета (код уст-ва + код фун-и + кол-во байт данных)
#define SIZE_PACK_ERROR         5           //кол-во байт ответа с ошибкой
#define SIZE_PACK_0506          8           //кол-во байт ответа функций 05,06
#define SIZE_PACK_0F10          8           //кол-во байт ответа функций 0F,10

#define IND_BYTE_CMD01_04       2           //позиция в пакете кол-ва байт данных для функций 01,02,03,04

//расшифровка ошибок протокола MODBUS
static char * const modbus_error_descr[] = {
    "OK",                                           //MBUS_ANSWER_OK
    "Function code not supported",                  //MBUS_ERROR_FUNC
    "Data address not available",                   //MBUS_ERROR_ADDR
    "Invalid value in data field",                  //MBUS_ERROR_DATA
    "Unrecoverable error",                          //MBUS_ERROR_DEV
    "It takes time to process the request",         //MBUS_ERROR_ACKWAIT
    "Busy processing command",                      //MBUS_ERROR_BUSY
    "The slave cannot execute the function",        //MBUS_ERROR_NOACK
    "Memory Parity error",                          //MBUS_ERROR_MEMCRC
    "Errors in function call parameters",           //MBUS_ERROR_PARAM
    "Received packet checksum error",               //MBUS_ANSWER_CRC
    "The slave is not responding",                  //MBUS_ANSWER_TIMEOUT
    "Communication with the device is lost"         //MBUS_CONNECT_LOST
 };

#pragma pack( push, 1 )                 //выравнивание структуры по границе 1 байта

//Структура регистров для запроса чтения (01,02,03,04)
typedef struct {
    uint8_t  dev_addr;                  //Адрес устройства
    uint8_t  function;                  //Функциональный код
    uint16_t addr_reg;                  //Адрес первого регистра HI/LO байт
    uint16_t cnt_reg;                   //Количество регистров HI/LO байт
    uint16_t crc;                       //Контрольная сумма CRC
 } MBUS_REQ_REG;

 //Структура регистров для записи (05,06) дискретная/16-битная
 typedef struct {
    uint8_t  dev_addr;                  //Адрес устройства
    uint8_t  function;                  //Функциональный код
    uint16_t addr_reg;                  //Адрес регистра HI/LO байт
    uint16_t reg_val;                   //Значение HI/LO байт
    uint16_t crc;                       //Контрольная сумма CRC
   } MBUS_WRT_REG;

//Структура для записи значений в несколько регистров (0F,10)
typedef struct {
    uint8_t  dev_addr;                  //Адрес устройства
    uint8_t  function;                  //Функциональный код
    uint16_t addr_reg;                  //Адрес первого регистра HI/LO байт
    uint16_t cnt_reg;                   //Количество регистров HI/LO байт
    uint8_t  cnt_byte;                  //Количество байт данных регистров
    //далее идут данные и КС
 } MBUS_WRT_REGS;

#pragma pack( pop )

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
FILE *modbus_log = NULL;

static char str_log[128];
static uint8_t send_buff[256];
static uint32_t send_total = 0, error_cnt[SIZE_ARRAY( modbus_error_descr )]; //счетчики ошибок протокола
static uint32_t error_dev[DEV_ERROR_CNT][SIZE_ARRAY( modbus_error_descr )];  //счетчики ошибок протокола по устройствам

static osMutexId_t mutex_mbus;
static osTimerId_t timer_mbus;
static osSemaphoreId_t mbus_semaphore;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t mbus_attr = {
    .name = "Modbus", 
    .stack_size = 512,
    .priority = osPriorityHigh
 };
 
static const osTimerAttr_t timer_attr = { .name = "Modbus" };
static const osSemaphoreAttr_t sem_attr = { .name = "Modbus" };
static const osMutexAttr_t mutex_attr = { .name = "Modbus", .attr_bits = osMutexPrioInherit };
static const osMessageQueueAttr_t que_attr = { .name = "Modbus" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void DataLog( uint8_t *data, uint16_t len, DataLogMode mode );
static uint8_t CreateFrame( MBUS_REQUEST *reqst );
static ModBusError AnswerData( uint8_t *data, uint8_t pack_len );

static void Timer1Callback( void *arg );
static void TaskModbus( void *pvParameters );

//*************************************************************************************************
// Инициализация протокола
//*************************************************************************************************
void ModBusInit( void ) {

    ModBusErrClr();
    //таймер ожидания ответа
    timer_mbus = osTimerNew( Timer1Callback, osTimerOnce, NULL, &timer_attr );
    //мьютех блокировки порта RS485
    mutex_mbus = osMutexNew( &mutex_attr );
    //семафор ожидания ответа
    mbus_semaphore = osSemaphoreNew( 1, 0, &sem_attr );
    //очередь сообщений
    modbus_queue = osMessageQueueNew( 8, sizeof( uint32_t ), &que_attr );
    //создаем задачу управления обменом по MODBUS
    osThreadNew( TaskModbus, NULL, &mbus_attr );
 }

//*************************************************************************************************
// Задача управления обменом по протоколу MODBUS
//*************************************************************************************************
static void TaskModbus( void *pvParameters ) {

    uint32_t msg;
    osStatus_t status;
    uint16_t len_pack;

    for ( ;; ) {
        status = osMessageQueueGet( modbus_queue, &msg, NULL, osWaitForever );
        if ( status == osOK ) {
            //отправка сообщения
            if ( msg & MSG_MODBUS_SEND ) {
                //выделим размер передаваемого блока
                len_pack = msg & MSG_MODBUS_MASK_DATA;
                RS485Send( send_buff, len_pack, RS485_SEND_RTU );
                osTimerStart( timer_mbus, TIMEOUT_ANSWER );
               }
            //проверка заголовка пакета
            if ( msg == MSG_MODBUS_CHECK )
               osTimerStart( timer_mbus, TIMEOUT_ANSWER );
            //весь пакет получен
            if ( msg == MSG_MODBUS_RECV ) {
                osTimerStop( timer_mbus );
                osSemaphoreRelease( mbus_semaphore );
               }
            //вышло время ожидания ответа
            if ( msg == MSG_MODBUS_TIMEOUT ) {
                ClearRecv();
                osSemaphoreRelease( mbus_semaphore );
               }
          }
      }
 }

//*************************************************************************************************
// Функция возвращает рассчитанный размер пакета ожидаемого к приему от уст-ва
// uint8_t *data   - указатель на буфер с данными ответа, для расчета необходимо первые 3 байта
// return uint16_t - размер пакета
//*************************************************************************************************
uint16_t PackSize( uint8_t *data ) {

   uint8_t func;
   
   if ( *( data + MB_ANSWER_FUNC ) & FUNC_ANSWER_ERROR )
       return SIZE_PACK_ERROR; //размер пакета с ошибкой
   else {
       //расчет размера пакета
       func = *( data + MB_ANSWER_FUNC );
       if ( func == FUNC_RD_COIL_STAT || func == FUNC_RD_DISC_INP || func == FUNC_RD_HOLD_REG || func == FUNC_RD_INP_REG )
           return SIZE_PACK_HEADER + *( data + IND_BYTE_CMD01_04 ) + SIZE_CRC;
       if ( func == FUNC_WR_SING_COIL || func == FUNC_WR_SING_REG )
           return SIZE_PACK_0506;
       if ( func == FUNC_WR_MULT_COIL || func == FUNC_WR_MULT_REG )
           return SIZE_PACK_0F10;
      }
    return 0;
 }

//*************************************************************************************************
// Функция обратного вызова таймера - пауза между пакетами данных
//*************************************************************************************************
static void Timer1Callback( void *arg ) {

    uint32_t msg;
    
    msg = MSG_MODBUS_TIMEOUT;
    osMessageQueuePut( modbus_queue, &msg, 0, 0 );
 }

//*************************************************************************************************
// Отправка команды по MODBUS
// MBUS_REQUEST *reqst - указатель на структуры с параметрами запроса
// return ModBusError  - результат выполнения запроса
//*************************************************************************************************
ModBusError ModBusRequest( MBUS_REQUEST *reqst ) {

    uint32_t msg;
    uint8_t *recv_buff;
    uint16_t len_pack, recv_ind, len_data = 0, data_ind;
    ModBusError status = MBUS_ERROR_PARAM;

    if ( reqst->ptr_data == NULL || reqst->ptr_lendata == NULL || *reqst->ptr_lendata == 0 || ModBusFunc( reqst->function ) == MBUS_FUNC_UNKNOW )
        return MBUS_ERROR_PARAM;
    //устанавливаем блокировку
    osMutexAcquire( mutex_mbus, osWaitForever );
    //логирование данных запроса с расшифровкой
    ModBusDecode( reqst, DATA_LOG_REQUEST, DATA_LOG_FILE );
    //формируем пакет для передачи
    len_pack = CreateFrame( reqst );
    //логирование данных
    DataLog( send_buff, len_pack, DATA_LOG_REQUEST );
    if ( len_pack ) {
        //установим семафор ожидания ответа уст-ва
        osSemaphoreAcquire( mbus_semaphore, 0 );
        send_total++;
        //отправка запроса с указанием размера пакета
        msg = MSG_MODBUS_SEND | len_pack;
        osMessageQueuePut( modbus_queue, &msg, 0, osWaitForever );
        //ждем ответа от устройства
        osSemaphoreAcquire( mbus_semaphore, osWaitForever );
        //получаем адрес приемного буфера и размер принятого ответа
        recv_buff = RS485Recv( &recv_ind );
        //логирование HEX данных ответа
        DataLog( recv_buff, recv_ind, DATA_LOG_ANSWER );
        //обработка ответа, после обработки в 16-битных данных байты будут переставлены местами
        status = AnswerData( recv_buff, recv_ind );
        if ( status == MBUS_ANSWER_OK ) {
            //только чтение регистров
            if ( ModBusFunc( reqst->function ) == MBUS_REGS_READ ) {
                data_ind = MB_ANSWER_DATA_RD;         //смещение начала данных
                len_data = recv_buff[MB_ANSWER_COUNT];//размер фактически принятых данных
               }
            //запись одного/нескольких регистров
            if ( ModBusFunc( reqst->function ) == MBUS_REG1_WRITE || ModBusFunc( reqst->function ) == MBUS_REGS_WRITE ) {
                data_ind = MB_ANSWER_DATA_WR; //смещение начала данных ответа
                len_data = MB_ANSWER_WRREG;   //размер принятых данных
               }
            //скопируем принятые данные ответа в MBUS_REQUEST->ptr_data
            if ( len_data <= *reqst->ptr_lendata ) {
                //размер принятых данных меньше размера выделенной памяти, копируем все
                *reqst->ptr_lendata = len_data;
                memcpy( (uint8_t *)reqst->ptr_data, recv_buff + data_ind, len_data );
               }
            //размер принятых данных больше размера выделенной памяти, копируем только часть
            else memcpy( (uint8_t *)reqst->ptr_data, recv_buff + data_ind, *reqst->ptr_lendata );

            /*if ( *reqst->ptr_lendata == 0 ) {
                len_data++;
                len_data--;
               } */

            //логирование данных ответа с расшифровкой
            ModBusDecode( reqst, DATA_LOG_ANSWER, DATA_LOG_FILE );
           }
        else {
            *reqst->ptr_lendata = 0;
            //подсчет ошибок обмена
            if ( status < SIZE_ARRAY( error_cnt ) ) {
                error_cnt[status]++;
                if ( ( reqst->dev_addr - 1 ) < DEV_ERROR_CNT ) 
                    error_dev[reqst->dev_addr-1][status]++;
               }
           }
        osDelay( 5 ); //пауза между пакетами
       }
    //снимаем блокировку
    osMutexRelease( mutex_mbus );
    return status;
 }

//*************************************************************************************************
// Формирование пакета протокола MODBUS
// MBUS_REQUEST *reqst - указатель на структуры с параметрами запроса
// return              - размер пакета в байтах для передачи
//*************************************************************************************************
static uint8_t CreateFrame( MBUS_REQUEST *reqst ) {

    uint16_t *src, *dst, value;
    uint8_t idx, data_len;
    MBUS_REQ_REG mbus_req;
    MBUS_WRT_REG mbus_reg;
    MBUS_WRT_REGS mbus_regs;

    //только чтение регистров
    if ( ModBusFunc( reqst->function ) == MBUS_REGS_READ ) {
        mbus_req.dev_addr = reqst->dev_addr;
        mbus_req.function = reqst->function;
        //поменяем байты местами для переменных uint16_t, т.к. сначала передаем старший байт
        //в текущей модели LITTLE-ENDIAN младший байт хранится первым
        mbus_req.addr_reg = __REVSH( reqst->addr_reg );
        mbus_req.cnt_reg = __REVSH( reqst->cnt_reg );
        //расчет контрольной суммы данных по уже переставленным байтам
        //Контрольная сумма передается в фрейме младшим байтом вперед, 
        //т.е. в формате LSB|MSB, т.е. байты местами не меняем !!!
        mbus_req.crc = CalcCRC16( (uint8_t *)&mbus_req, sizeof( mbus_req ) - sizeof( uint16_t ) );
        memset( send_buff, 0x00, sizeof( send_buff ) );
        memcpy( send_buff, &mbus_req, sizeof( MBUS_REQ_REG ) );
        return sizeof( MBUS_REQ_REG );
       }
    //запись одного регистра (05, 06) 8/16-ми битная адресация
    if ( ModBusFunc( reqst->function ) == MBUS_REG1_WRITE ) {
        mbus_reg.dev_addr = reqst->dev_addr;
        mbus_reg.function = reqst->function;
        //поменяем байты местами для переменных uint16_t, т.к. сначала передаем старший байт
        //в текущей модели LITTLE-ENDIAN младший байт хранится первым
        mbus_reg.addr_reg = __REVSH( reqst->addr_reg );
        //для функций с 16-битной адресацией меняем байты местами
        if ( ModBusAddr( mbus_reg.function ) == MBUS_REG_16BIT )
            mbus_reg.reg_val = __REVSH( *( (uint16_t *)reqst->ptr_data ) );
        //расчет контрольной суммы
        mbus_reg.crc = CalcCRC16( (uint8_t *)&mbus_reg, sizeof( mbus_reg ) - 2 );
        memset( send_buff, 0x00, sizeof( send_buff ) );
        memcpy( send_buff, &mbus_reg, sizeof( MBUS_WRT_REG ) );
        return sizeof( MBUS_WRT_REG );
       }
    //запись нескольких битов/регистров (0F,10)
    if ( ModBusFunc( reqst->function ) == MBUS_REGS_WRITE ) {
        mbus_regs.dev_addr = reqst->dev_addr;
        mbus_regs.function = reqst->function;
        //поменяем байты местами для переменных uint16_t, т.к. сначала передаем старший байт
        //в текущей модели LITTLE-ENDIAN младший байт хранится первым
        mbus_regs.addr_reg = __REVSH( reqst->addr_reg );
        mbus_regs.cnt_reg = __REVSH( reqst->cnt_reg );
        data_len = sizeof( MBUS_WRT_REGS );
        //расчет кол-ва байт по типу регистров
        if ( ModBusAddr( mbus_regs.function ) == MBUS_REG_16BIT )
            mbus_regs.cnt_byte = reqst->cnt_reg * sizeof( uint16_t );
        else mbus_regs.cnt_byte = CALC_BYTE( reqst->cnt_reg );
        memset( send_buff, 0x00, sizeof( send_buff ) );
        //копируем в буфер заголовок пакета
        memcpy( send_buff, &mbus_regs, data_len );
        //копируем в буфер данные регистров
        //для функций с 16-битной адресацией меняем байты местами
        if ( ModBusAddr( mbus_regs.function ) == MBUS_REG_16BIT ) {
            //копируем как uint16_t
            src = (uint16_t *)reqst->ptr_data;
            dst = (uint16_t *)( send_buff + data_len );
            for ( idx = 0; idx < reqst->cnt_reg; idx++, src++, dst++ )
                *dst = __REVSH( *src );
            data_len += idx * sizeof( uint16_t );
           }
        else {
            //копируем как uint8_t
            memcpy( send_buff + data_len, (uint8_t *)reqst->ptr_data, mbus_regs.cnt_byte );
            data_len += mbus_regs.cnt_byte;
           }
        //расчет, сохранение КС
        value = CalcCRC16( send_buff, data_len );
        memcpy( send_buff + data_len, (uint8_t *)&value, sizeof( uint16_t ) );
        return data_len + sizeof( uint16_t );
       }
    return 0;
 }

//*************************************************************************************************
// Обработка ответа, проверка всего пакета с ответом
// uint8_t *data        - адрес буфера с принятыми данными
// uint8_t len          - размер пакета (байт)
// return = ModBusError - результат проверки принятого пакета
//*************************************************************************************************
static ModBusError AnswerData( uint8_t *data, uint8_t pack_len ) {

    uint8_t func, len_data, idx, cnt_word;
    uint16_t crc_calc, crc_data, offset, *ptr_uint16;

    if ( data == NULL || !pack_len )
        return MBUS_ANSWER_TIMEOUT;
    //проверим КС всего пакета
    crc_calc = CalcCRC16( data, pack_len - sizeof( uint16_t ) );
    crc_data = *((uint16_t*)( data + pack_len - sizeof( uint16_t ) ));
    if ( crc_calc != crc_data )
        return MBUS_ANSWER_CRC; //КС не совпали
    //проверим флаг ошибки в пакете
    if ( data[MB_ANSWER_FUNC] & FUNC_ANSWER_ERROR )
        return (ModBusError)*( data + MB_ANSWER_ERROR );
    func = *( data + MB_ANSWER_FUNC );
    if ( ModBusFunc( func ) == MBUS_REG1_WRITE || ModBusFunc( func ) == MBUS_REGS_WRITE ) {
        offset = MB_ANSWER_DATA_WR; //смещение для данных ответа (запись регистров)
        len_data = MB_ANSWER_WRREG;
       }
    else {
        offset = MB_ANSWER_DATA_RD; //смещение для данных ответа (чтение регистров)
        len_data = *( data + MB_ANSWER_COUNT ); //переменное кол-во байт ответа
       }
    //меняем байты местами
    cnt_word = len_data / sizeof( uint16_t );
    ptr_uint16 = (uint16_t *)( data + offset );
    for ( idx = 0; idx < cnt_word; idx++, ptr_uint16++ )
        *ptr_uint16 = __REVSH( *ptr_uint16 );
    return MBUS_ANSWER_OK;
 }

//*************************************************************************************************
// По коду функции возвращет тип операции: чтение/запись регистров
// uint8_t func      - код функции
// return ModbusFunc - тип операции: чтение/запись регистров
//*************************************************************************************************
ModbusFunc ModBusFunc( uint8_t func ) {

    //функции чтения нескольких регистров
    if ( func == FUNC_RD_COIL_STAT || func == FUNC_RD_DISC_INP || func == FUNC_RD_HOLD_REG || \
         func == FUNC_RD_INP_REG || func == FUNC_RD_EXCP_STAT || func == FUNC_RD_DIAGNOSTIC )
        return MBUS_REGS_READ;
    //функции записи одного регистра
    if ( func == FUNC_WR_SING_COIL || func == FUNC_WR_SING_REG || func == FUNC_WR_MASK_REG )
        return MBUS_REG1_WRITE;
    //функции записи нескольких регистров
    if ( func == FUNC_WR_MULT_COIL || func == FUNC_WR_MULT_REG || func == FUNC_WR_FILE_REC )
        return MBUS_REGS_WRITE;
    return MBUS_FUNC_UNKNOW;
 }

//*************************************************************************************************
// По коду функции возвращет тип адресации регистров 8/16-битная
// uint8_t func         - код функции
// return ModbusAddrReg - тип адресации регистров
//*************************************************************************************************
ModbusAddrReg ModBusAddr( uint8_t func ) {

    //функции с 8-битной адресаций
    if ( func == FUNC_RD_COIL_STAT || func == FUNC_RD_DISC_INP || func == FUNC_WR_SING_COIL || \
         func == FUNC_WR_MULT_COIL || func == FUNC_RD_EXCP_STAT )
        return MBUS_REG_8BIT;
    //функции с 16-битной адресаций
    if ( func == FUNC_RD_HOLD_REG || func == FUNC_RD_INP_REG || func == FUNC_WR_SING_REG || \
         func == FUNC_WR_MULT_REG || func == FUNC_WR_MASK_REG || func == FUNC_RD_FIFO_QUE || \
         func == FUNC_RD_FILE_REC || func == FUNC_WR_FILE_REC ||  func == FUNC_RD_EVENT_CNT || \
          func == FUNC_RD_DIAGNOSTIC || func == FUNC_RD_EVENT_LOG )
        return MBUS_REG_16BIT;
    return MBUS_REG_OTHER;
 }

//*************************************************************************************************
// Вкл/Выкл логирования данных запрос/ответ
// Mode mode        - вкл/выкл логирование запросов ответов уст-в в сети MODBUS
// return = SUCCESS - логирование включено
//        = ERROR   - логирование выключено
//*************************************************************************************************
Status ModbusLog( Mode mode ) {

    if ( SDStatus() == ERROR )
        return ERROR; //SD карты нет
    if ( mode ) {
        //открываем файл
        modbus_log = fopen( "modbus_data.log", "a" );
        if ( modbus_log != NULL )
            return SUCCESS;
        else return ERROR;
       }
    else {
        if ( modbus_log != NULL ) {
            fclose( modbus_log );
            modbus_log = NULL;
           }
        return SUCCESS;
       }
 }

//*************************************************************************************************
// Возвращает указатель на строку расшифровки результата выполнения запроса по протоколу MODBUS
// ModBusError error - код ошибки обработки принятого ответа от уст-ва
// return            - расшифровка кода ошибки
//*************************************************************************************************
char *ModBusErrDesc( ModBusError error ) {

    if ( error >= SIZE_ARRAY( modbus_error_descr ) )
        return NULL;
    return modbus_error_descr[error];
 }

//*************************************************************************************************
// Обнуляет счетчики ошибок MODBUS
//*************************************************************************************************
void ModBusErrClr( void ) {

    send_total = 0;
    memset( (uint8_t *)&error_cnt, 0x00, sizeof( error_cnt ) );
    memset( (uint8_t *)&error_dev, 0x00, sizeof( error_dev ) );
 }

//*************************************************************************************************
// Возвращает значение счетчика ошибок MODBUS
// ModBusError err_ind - индекс счетчика ошибок, 
// Для err_ind = MBUS_ANSWER_OK возвращается кол-во счетчиков
// return = N          - значение счетчика ошибок
//*************************************************************************************************
uint32_t ModBusErrCnt( ModBusError err_ind ) {

    if ( err_ind == MBUS_ANSWER_OK )
        return SIZE_ARRAY( error_cnt );
    if ( err_ind >= SIZE_ARRAY( error_cnt ) )
        return 0;
    return error_cnt[err_ind];
 }

//*************************************************************************************************
// Возвращает расшифровку и значения счетчиков ошибок MODBUS
// ModBusError err_ind - индекс счетчика ошибок, 
// Для err_ind = MBUS_ANSWER_OK возвращается общее кол-во отправленных пакетов
// return = N          - значение счетчика ошибок
//*************************************************************************************************
char *ModBusErrCntDesc( ModBusError err_ind, char *str ) {

    uint8_t i;
    char *ptr;
    
    if ( err_ind >= SIZE_ARRAY( error_cnt ) )
        return NULL;
    if ( err_ind == MBUS_ANSWER_OK ) {
        sprintf( str, "Total packages sent: %u\r\n", send_total );
        return str;
       }
    ptr = str;
    ptr += sprintf( ptr, "%s", ModBusErrDesc( err_ind ) );
    //дополним расшифровку ошибки справа знаком "." до 40 символов
    ptr += AddDot( str, 40 );
    ptr += sprintf( ptr, "%6u ", error_cnt[err_ind] );
    for ( i = 0; i < DEV_ERROR_CNT; i++ )
        ptr += sprintf( ptr, "%6u ", error_dev[i][err_ind] );
    return str;
 }

//*************************************************************************************************
// Запись в протокол запросов и ответов протокола MODBUS
// uint8_t *data      - указатель на данные
// uint16_t len       - кол-во байт данных
// DataLogMode mode   - тип источника данных: запрос/ответ
//*************************************************************************************************
static void DataLog( uint8_t *data, uint16_t len, DataLogMode mode ) {

    uint8_t i;
    char *ptr, str[180];
    
    if ( SDStatus() == ERROR )
        return; //SD карты нет
    if ( modbus_log == NULL )
        return; //файл не открыт
    //формируем строку с значения в формате 0xNN
    if ( mode == DATA_LOG_REQUEST )
        fprintf( modbus_log, "REQ: " );
    if ( mode == DATA_LOG_ANSWER )
        fprintf( modbus_log, "ANS: " );
    if ( !len )
        fprintf( modbus_log, "%s", ModBusErrDesc( MBUS_ANSWER_TIMEOUT ) );
    ptr = str;
    memset( str, 0x00, sizeof( str ) );
    for ( i = 0; i < len; i++ )
        ptr += sprintf( ptr, "%02X ", *( data + i ) );
    fprintf( modbus_log, "%s\r\n", str );
 }

//*************************************************************************************************
// Формирует в буфере расшифровку запроса и ответа данных MODBUS протокола
// При кол-ве регистров > 10, выводиться только 10 регистров
// Логирование в файл выполняется автоматически при включении логирования
// MBUS_REQUEST *reqst - указатель на структуру с данными запроса/ответа
// DataLogMode mode    - тип информации: запрос/ответ
// return              - указатель на (str_log[]) расшифрованные данные
//*************************************************************************************************
char *ModBusDecode( MBUS_REQUEST *reqst, DataLogMode mode, DataLogFile log ) {

    char *ptr;
    uint8_t i, *ptr_reg8;
    uint16_t cnt, *ptr_reg16;
    
    ptr = str_log;
    memset( str_log, 0x00, sizeof( str_log ) );
    //запрос
    if ( mode == DATA_LOG_REQUEST ) {
        //вывод информации по введенным параметрам
        ptr += sprintf( ptr, "\r\nDEV=0x%02X ", reqst->dev_addr );
        ptr += sprintf( ptr, "FUNC=0x%02X ", reqst->function );
        ptr += sprintf( ptr, "ADDR=0x%04X ", reqst->addr_reg );
        ptr += sprintf( ptr, "REGS=0x%04X ", reqst->cnt_reg );
        //данные регистров, если есть
        if ( ( ModBusFunc( reqst->function ) == MBUS_REG1_WRITE || ModBusFunc( reqst->function ) == MBUS_REGS_WRITE ) ) {
            ptr += sprintf( ptr, "DATA=" );
            //кол-во регистров для вывода
            cnt = reqst->cnt_reg;
            if ( cnt > MAX_REGS_OUT_LOG )
                cnt = MAX_REGS_OUT_LOG;
            if ( ModBusAddr( reqst->function ) == MBUS_REG_16BIT ) {
                ptr_reg16 = (uint16_t *)reqst->ptr_data; //16-битная адресация
                for ( i = 0; i < cnt; i++, ptr_reg16++ )
                    ptr += sprintf( ptr, "0x%04X ", *ptr_reg16 );
               }
            else {
                ptr_reg8 = (uint8_t *)reqst->ptr_data; //8-битная адресация
                for ( i = 0; i < CALC_BYTE( cnt ); i++, ptr_reg8++ )
                    ptr += sprintf( ptr, "0x%02X ", *ptr_reg8 );
               }
            if ( reqst->cnt_reg > 10 )
                ptr += sprintf( ptr, "..." );
           }
       }
    //ответ
    if ( ( ModBusFunc( reqst->function ) == MBUS_REG1_WRITE || ModBusFunc( reqst->function ) == MBUS_REGS_WRITE ) && mode == DATA_LOG_ANSWER ) {
        //только при записи регистров
        ptr_reg16 = (uint16_t *)reqst->ptr_data;
        ptr += sprintf( ptr, "ADDR=0x%04X ", *ptr_reg16++ );
        ptr += sprintf( ptr, "REGS=0x%04X", *ptr_reg16 );
       }
    if ( ModBusFunc( reqst->function ) == MBUS_REGS_READ && mode == DATA_LOG_ANSWER ) {
        //только при чтении регистров и ответа
        ptr += sprintf( ptr, "DATA=" );
        //кол-во регистров для вывода
        cnt = *reqst->ptr_lendata;
        if ( ModBusAddr( reqst->function ) == MBUS_REG_16BIT )
            cnt /= sizeof( uint16_t );
        if ( cnt > MAX_REGS_OUT_LOG )
            cnt = MAX_REGS_OUT_LOG;
        if ( ModBusAddr( reqst->function ) == MBUS_REG_16BIT ) {
            ptr_reg16 = (uint16_t *)reqst->ptr_data; //16-битная адресация
            for ( i = 0; i < cnt; i++, ptr_reg16++ )
                ptr += sprintf( ptr, "0x%04X ", *ptr_reg16 );
           }
        else {
            ptr_reg8 = (uint8_t *)reqst->ptr_data; //8-битная адресация
            for ( i = 0; i < cnt; i++, ptr_reg8++ )
                ptr += sprintf( ptr, "0x%02X ", *ptr_reg8 );
           }
        if ( reqst->cnt_reg > 10 )
            ptr += sprintf( ptr, "..." );
      }
    //автоматическое логирование в файл
    if ( SDStatus() == SUCCESS && modbus_log != NULL && log == DATA_LOG_FILE )
        fprintf( modbus_log, "%s\r\n", str_log );
    return str_log;
 }
