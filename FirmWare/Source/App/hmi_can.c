
//*************************************************************************************************
//
// Управление обменом данными c HMI контроллером по CAN шине
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "can_lpc17xx.h"

#include "device.h"
#include "dev_data.h"
#include "dev_param.h"
#include "can_data.h"
#include "can_def.h"

#include "priority.h"
#include "command.h"
#include "hmi_can.h"

#include "charger.h"
#include "message.h"
#include "reserv.h"
#include "outinfo.h"
#include "inverter.h"
#include "sdcard.h"
#include "pv.h"
#include "alt.h"
#include "rtc.h"
#include "gen.h"
#include "sound.h"
#include "modbus.h"
#include "modbus_def.h"
#include "informing.h"
#include "tracker.h"
#include "tracker_ext.h"
#include "events.h"

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osMessageQueueId_t hmi_msg = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define CAN_BITRATE_NOMINAL     125000      //скорость обмена (125 kbit/s)

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern ARM_DRIVER_CAN Driver_CAN2;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
ARM_DRIVER_CAN *CanDrv;

static char str_val[16];
static uint8_t rx_data[8], *can_mbus_addr;
static uint32_t rx_obj_idx, tx_obj_idx;
static int32_t send_stat = 0, send_pack = 0, error_send = 0;
static uint16_t len_mbus_data, can_mbus_rest;

static MBUS_REQUEST reqst;
static CAN_MODBUS can_modbus;
static ARM_CAN_MSG_INFO rx_msg_info, tx_msg_info;
static osMessageQueueId_t cmd_msg = NULL;
static osSemaphoreId_t can_semaph;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t hmi_attr = {
    .name = "HmiLink", 
    .stack_size = 512,
    .priority = osPriorityNormal
 };

static const osThreadAttr_t cmd_attr = {
    .name = "HmiCmd", 
    .stack_size = 1280,
    .priority = osPriorityNormal
 };

static const osMessageQueueAttr_t que1_attr = { .name = "HmiLink" };
static const osMessageQueueAttr_t que2_attr = { .name = "HmiCmd" };
static const osSemaphoreAttr_t sem_attr = { .name = "HmiLink" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void CANMbusAnswer( void );
static void TaskHmi( void *pvParameters );
static void TaskCmd( void *pvParameters );
void CAN_SignalObjectEvent( uint32_t obj_idx, uint32_t event );
static void ExecCommand( MSGQUEUE_CAN *que_cmd );
static void CmdSaveLog( MSGQUEUE_CAN *que_cmd );
static void ConfigSaveLog( ConfigParam id_par, ConfigValSet cfg_set, Status check );

//*************************************************************************************************
// Инициализация CAN2 для обмена данными с внешним контроллером "Human-machine interface"
//*************************************************************************************************
void CANInit( void ) {

    int32_t status;
    uint32_t i, clock;
    ARM_CAN_CAPABILITIES can_cap;
    ARM_CAN_OBJ_CAPABILITIES can_obj_cap;
    
    //семафор ожидания завершения передачи одного фрэйма
    can_semaph = osSemaphoreNew( 1, 0, &sem_attr );
    //очередь сообщений
    hmi_msg = osMessageQueueNew( 128, sizeof( uint32_t ), &que1_attr );
    cmd_msg = osMessageQueueNew( 64, sizeof( MSGQUEUE_CAN ), &que2_attr );
    //создаем задачу
    osThreadNew( TaskHmi, NULL, &hmi_attr );
    osThreadNew( TaskCmd, NULL, &cmd_attr );
    //инициализация CAN
    CanDrv = &Driver_CAN2;
    can_cap = CanDrv->GetCapabilities();
    CanDrv->Initialize( NULL, CAN_SignalObjectEvent );
    CanDrv->PowerControl( ARM_POWER_FULL );
    CanDrv->SetMode( ARM_CAN_MODE_INITIALIZATION );
    //настройка скорости
    clock = CanDrv->GetClock();
    if ( ( clock % ( 8U * CAN_BITRATE_NOMINAL ) ) == 0U ) {                               
        //If CAN base clock is divisible by 8 * nominal bitrate without remainder
        //Set nominal bitrate
        //Set nominal bitrate to configured constant value
        //Set propagation segment to 5 time quanta
        //Set phase segment 1 to 1 time quantum (sample point at 87.5% of bit time)
        //Set phase segment 2 to 1 time quantum (total bit is 8 time quanta long)
        //Resynchronization jump width is same as phase segment 2
        status = CanDrv->SetBitrate( ARM_CAN_BITRATE_NOMINAL, CAN_BITRATE_NOMINAL, ARM_CAN_BIT_PROP_SEG( 5U ) | ARM_CAN_BIT_PHASE_SEG1( 1U ) | ARM_CAN_BIT_PHASE_SEG2( 1U ) | ARM_CAN_BIT_SJW( 1U ) );
    } else if ( ( clock % ( 10U * CAN_BITRATE_NOMINAL ) ) == 0U ) {                     
        //If CAN base clock is divisible by 10 * nominal bitrate without remainder
        //Set nominal bitrate
        //Set nominal bitrate to configured constant value
        //Set propagation segment to 7 time quanta
        //Set phase segment 1 to 1 time quantum (sample point at 90% of bit time)
        //Set phase segment 2 to 1 time quantum (total bit is 10 time quanta long)
        //Resynchronization jump width is same as phase segment 2
        status = CanDrv->SetBitrate( ARM_CAN_BITRATE_NOMINAL, CAN_BITRATE_NOMINAL, ARM_CAN_BIT_PROP_SEG( 7U ) | ARM_CAN_BIT_PHASE_SEG1( 1U ) | ARM_CAN_BIT_PHASE_SEG2( 1U ) | ARM_CAN_BIT_SJW( 1U ) );                
    } else if ( ( clock % ( 12U * CAN_BITRATE_NOMINAL ) ) == 0U ) {                       
        //If CAN base clock is divisible by 12 * nominal bitrate without remainder
        //Set nominal bitrate
        //Set nominal bitrate to configured constant value
        //Set propagation segment to 7 time quanta
        //Set phase segment 1 to 2 time quantum (sample point at 83.3% of bit time)
        //Set phase segment 2 to 2 time quantum (total bit is 12 time quanta long)
        //Resynchronization jump width is same as phase segment 2
        status = CanDrv->SetBitrate( ARM_CAN_BITRATE_NOMINAL, CAN_BITRATE_NOMINAL, ARM_CAN_BIT_PROP_SEG( 7U ) | ARM_CAN_BIT_PHASE_SEG1( 2U ) | ARM_CAN_BIT_PHASE_SEG2( 2U ) | ARM_CAN_BIT_SJW( 2U ) );                
       }
    if ( status != ARM_DRIVER_OK ) 
        return; //инициализация не выполнена
    //определение ID объектов сообщений
    rx_obj_idx = 0xFFFFFFFFU;
    tx_obj_idx = 0xFFFFFFFFU;
    for ( i = 0; i < can_cap.num_objects; i++ ) {
        //поиск первого доступного объекта для приема
        can_obj_cap = CanDrv->ObjectGetCapabilities( i );
        if ( ( rx_obj_idx == 0xFFFFFFFFU ) && ( can_obj_cap.rx == 1 ) )
            rx_obj_idx = i;
        //поиск первого доступного объекта для передачи
        else if ( ( tx_obj_idx == 0xFFFFFFFFU ) && ( can_obj_cap.tx == 1 ) ) { 
                tx_obj_idx = i; 
                break; 
               }
       }
    if ( ( rx_obj_idx == 0xFFFFFFFFU ) || ( tx_obj_idx == 0xFFFFFFFFU ) )
        return; //доступных объектов нет
    //конфигурация найденых объектов прием/передача/фильт
    CanDrv->ObjectConfigure( tx_obj_idx, ARM_CAN_OBJ_TX );
    CanDrv->ObjectConfigure( rx_obj_idx, ARM_CAN_OBJ_RX );
    //для установки фильтра с учетом 29-битного идентификаторов необходимо добавить атрибут ARM_CAN_ID_IDE_Msk 
    CanDrv->ObjectSetFilter( rx_obj_idx, ARM_CAN_FILTER_ID_RANGE_ADD, 0 | ARM_CAN_ID_IDE_Msk, CAN_FILTER_RANGE );
    CanDrv->SetMode( ARM_CAN_MODE_NORMAL ); 
    //настройка прерывания
    NVIC_SetPriority( CAN_IRQn, NVIC_EncodePriority( NVIC_GetPriorityGrouping(), PRIORITY_SSP, SUB_PRIORITY_CAN ) );
 }

//*************************************************************************************************
// Задача управления передачи данных контроллеру HMI по протоколу CAN
//*************************************************************************************************
static void TaskHmi( void *pvParameters ) {

    uint32_t msg;
    osStatus_t status;
    
    for ( ;; ) {
        status = osMessageQueueGet( hmi_msg, &msg, NULL, osWaitForever );
        if ( status == osOK ) {
            if ( msg == ID_DEV_MODBUS_ANS )
                CANMbusAnswer(); //передача ответа на запрос по MODBUS
            else DevDataSend( msg ); //передача данных по уст-м
           }
       }
 }

//*************************************************************************************************
// Задача выполнения команд полученных по CAN шине от модуля HMI
//*************************************************************************************************
static void TaskCmd( void *pvParameters ) {

    osStatus_t status;
    MSGQUEUE_CAN que_cmd;
    
    for ( ;; ) {
        status = osMessageQueueGet( cmd_msg, &que_cmd, NULL, osWaitForever );
        if ( status == osOK )
            ExecCommand( &que_cmd );
       }
 }

//*************************************************************************************************
// Обработка событий CAN шины
//*************************************************************************************************
void CAN_SignalObjectEvent( uint32_t obj_idx, uint32_t event ) {

    uint32_t send, id;
    MSGQUEUE_CAN que_cmd;
    
    if ( event == ARM_CAN_EVENT_RECEIVE ) {
        if ( obj_idx == rx_obj_idx ) {
            //принятые данные помещаем в очередь на исполнение
            CanDrv->MessageRead( rx_obj_idx, &rx_msg_info, rx_data, sizeof( rx_data ) );
            id = CAN_GET_DEV_ID( ( rx_msg_info.id & CAN_MASK_DEV_ID ) );
            if ( id == ID_CONFIG && rx_msg_info.rtr == 1 ) {
                //запрос на передачу параметров настроек в HMI
                send = ID_CONFIG; 
                osMessageQueuePut( hmi_msg, &send, 0, 0 );
                return;
               }
            if ( id ) {
                //команды для выполнения
                que_cmd.dev_id = (Device)CAN_GET_DEV_ID( ( rx_msg_info.id & CAN_MASK_DEV_ID ) );
                que_cmd.param_id = (ConfigParam)CAN_GET_PARAM_ID( ( rx_msg_info.id & CAN_MASK_PARAM_ID ) );
                que_cmd.sub_pack_id = CAN_GET_PACK_ID( ( rx_msg_info.id & CAN_MASK_PACK_ID ) );
                que_cmd.len_data = rx_msg_info.dlc;
                //помещаем в очередь
                memcpy( (uint8_t *)que_cmd.data, (uint8_t *)rx_data, que_cmd.len_data );
                osMessageQueuePut( cmd_msg, &que_cmd, 0, 0 );
               }
           }
       }
    if ( event == ARM_CAN_EVENT_SEND_COMPLETE ) {
        if ( obj_idx == tx_obj_idx ) {
            //снимаем семафор ожидания для отправки следующего блока данных
            osSemaphoreRelease( can_semaph );
           }
       }
 }

//*************************************************************************************************
// Запуск передачи одного сообщения по CAN шине
// uint32_t can_id   - ID сообщения
// uint8_t *ptr_data - указатель на блок данных
// uint8_t len_data  - размер блока данных для передачи
//*************************************************************************************************
void CANSendFrame( uint32_t can_id, uint8_t *ptr_data, uint8_t len_data ) {

    //установим семафор ожидания завершения передачи
    osSemaphoreAcquire( can_semaph, 0 );
    memset( &tx_msg_info, 0x00, sizeof( ARM_CAN_MSG_INFO ) );
    tx_msg_info.id = ARM_CAN_EXTENDED_ID( can_id );
    send_stat = CanDrv->MessageSend( tx_obj_idx, &tx_msg_info, ptr_data, len_data );
    send_pack++;
    if ( send_stat < 0 )
        error_send++;
    //ждем семафор - завершение передачи предыдущего блока
    osSemaphoreAcquire( can_semaph, osWaitForever );
 }

//*************************************************************************************************
// Обработка команды полученной по CAN шине
// MSGQUEUE_CMD *que_cmd - указатель на команду в очереди сообщений
//*************************************************************************************************
static void ExecCommand( MSGQUEUE_CAN *que_cmd ) {

    uint32_t msg;
    CAN_PV *can_pv;
    uint8_t cnt_rpt;
    CAN_INFO *can_info;
    CAN_RELAY *can_rel;
    CAN_EXT *can_ext;
    RTC *datetime;
    CAN_TRC *can_trc;
    ConfigValSet cfg_set;

    //Установка часов/календаря
    if ( que_cmd->dev_id == ID_DEV_RTC && !que_cmd->param_id ) {
        datetime = (RTC *)que_cmd->data;
        RTCSet( datetime );
       }
    //Управление коммутацией солнечных панелей
    if ( que_cmd->dev_id == ID_DEV_PV && !que_cmd->param_id ) {
        can_pv = (CAN_PV *)que_cmd->data;
        if ( can_pv->ctrl & CAN_PV_CTRL )
            PvControl( can_pv->ctrl, EEPROM_SAVE );
        if ( can_pv->ctrl & CAN_PV_MODE )
            PvSetMode( can_pv->mode, EEPROM_SAVE );
       }
    //Управление контроллером заряда PB-1000-224
    if ( que_cmd->dev_id == ID_DEV_CHARGER && !que_cmd->param_id )
        Charger( (ChargeMode)*que_cmd->data, EEPROM_SAVE );
    //Управление инверторами TS-1000-224, TS-3000-224
    if ( que_cmd->dev_id == ID_DEV_INV1 && !que_cmd->param_id )
        InvCtrl( ID_DEV_INV1, (InvCtrlCmnd)*que_cmd->data );
    if ( que_cmd->dev_id == ID_DEV_INV2 && !que_cmd->param_id )
        InvCtrl( ID_DEV_INV2, (InvCtrlCmnd)*que_cmd->data );
    //Управление блоком АВР
    if ( que_cmd->dev_id == ID_DEV_ALT && !que_cmd->param_id ) {
        if ( (CanCtrlAlt)*que_cmd->data == CAN_ALT_DC )
            AltPowerDC();
        if ( (CanCtrlAlt)*que_cmd->data == CAN_ALT_AC )
            AltPowerAC();
       }
    //Управление генератором
    if ( que_cmd->dev_id == ID_DEV_GEN && !que_cmd->param_id ) {
        if ( (CanCtrlGen)*que_cmd->data == CAN_GEN_STOP )
            GenStop();
        if ( (CanCtrlGen)*que_cmd->data == CAN_GEN_START )
            GenStart();
        if ( (CanCtrlGen)*que_cmd->data == CAN_GEN_TEST )
            GenTest();
       }
    //Управление контроллером солнечного трекера 
    if ( que_cmd->dev_id == ID_DEV_TRC && !que_cmd->param_id ) {
        can_trc = (CAN_TRC *)que_cmd->data;
        //Управление реле питания актуаторов трекера
        if ( can_trc->ctrl == CAN_TRC_POWER )
            TRCMode( can_trc->power_act, EEPROM_SAVE );
        //Управление позиционированием трекера
        if ( can_trc->ctrl == CAN_TRC_POS )
            TrackerSetPos( can_trc->type_act, can_trc->type_value, can_trc->value );
        //Прервать позиционирование
        if ( can_trc->ctrl == CAN_TRC_STOP )
            TrackerCmd( EXTRC_STOP, 0, 0 );
        //Переход в командный режим
        if ( can_trc->ctrl == CAN_TRC_CMD )
            TrackerCmd( EXTRC_CMD_ON, 0, 0 );
        //Позиционирование по солнечному сенсору
        if ( can_trc->ctrl == CAN_TRC_INT )
            TrackerCmd( EXTRC_CMD_OFF, 0, 0 );
        //Инициалзация контроллера трекера
        if ( can_trc->ctrl == CAN_TRC_INIT )
            TrackerCmd( EXTRC_VERT | EXTRC_HORZ, 0, 0 );
        //Сохранить текущие значения позиционирования в EEPROM контроллера трекера
        if ( can_trc->ctrl == CAN_TRC_SAVE )
            TrackerCmd( EXTRC_SEEP, 0, 0 );
        //Восстановить значения позиционирования из EEPROM контроллера трекера
        if ( can_trc->ctrl == CAN_TRC_REST )
            TrackerCmd( EXTRC_REEP, 0, 0 );
        //Перезапуск контроллера трекера
        if ( can_trc->ctrl == CAN_TRC_RESET )
            TrackerCmd( EXTRC_RESET, 0, 0 );
       }
    //Управление голосовым/звуковым информатором
    if ( que_cmd->dev_id == ID_DEV_VOICE && !que_cmd->param_id ) {
        can_info = (CAN_INFO *)que_cmd->data;
        if ( can_info->ctrl & CAN_VOICE )
            Informing( can_info->voice, NULL );
        if ( can_info->ctrl & CAN_VOLUME )
            SetVolume( can_info->volume );
        if ( can_info->ctrl & CAN_SOUND )
            SoundPlay( can_info->sound, NULL );
       }
    //Управление дополнительными реле
    if ( que_cmd->dev_id == ID_DEV_RESERV && !que_cmd->param_id ) {
        can_rel = (CAN_RELAY *)que_cmd->data;
        ReservCmnd( can_rel->relay, can_rel->mode );
       }
    //Управление дополнительными выходами
    if ( que_cmd->dev_id == ID_DEV_EXTOUT && !que_cmd->param_id ) {
        can_ext = (CAN_EXT *)que_cmd->data;
        ExtOut( can_ext->relay, can_ext->mode );
       }
    //Обмен CAN -> MODBUS
    //Прием первого пакета с данными
    if ( que_cmd->dev_id == ID_DEV_MODBUS_REQ && !que_cmd->sub_pack_id ) {
        //первый пакет с MODBUS
        can_mbus_addr = (uint8_t *)&can_modbus;
        memset( (uint8_t *)&can_modbus, 0x00, sizeof( CAN_MODBUS ) );
        //для первого пакета размер данных всегда равен 8
        memcpy( (uint8_t *)&can_modbus, que_cmd->data, CAN_DATA_MAX );
        if ( can_modbus.length )
            can_mbus_rest = can_modbus.length - que_cmd->len_data; //остаток байтов для приема
        if ( !can_mbus_rest )
            que_cmd->sub_pack_id = 1; //только один пакет, переходим на отправку запроса
        else can_mbus_addr += que_cmd->len_data; //смещение для следующего блока
       }
    //Прием следующих пакетов с данными
    if ( que_cmd->dev_id == ID_DEV_MODBUS_REQ && que_cmd->sub_pack_id ) {
        if ( can_mbus_rest ) {
            //принят следующий пакет
            memcpy( can_mbus_addr, que_cmd->data, que_cmd->len_data );
            can_mbus_rest -= que_cmd->len_data; //остаток байт для приема
            can_mbus_addr += que_cmd->len_data; //смещение для следующего блока
           }
        if ( !can_mbus_rest ) {
            //все пакеты приняты, формируем запрос
            reqst.dev_addr = can_modbus.dev_addr;
            reqst.function = can_modbus.function;
            reqst.addr_reg = can_modbus.addr_reg;
            reqst.cnt_reg = can_modbus.cnt_reg;
            reqst.ptr_data = can_modbus.mbus_data;
            reqst.ptr_lendata = &len_mbus_data;
            cnt_rpt = CNT_REPEAT_REQST;
            do {
                len_mbus_data = sizeof( can_modbus.mbus_data );
                can_modbus.answer = ModBusRequest( &reqst );
               } while ( cnt_rpt-- && ( can_modbus.answer == MBUS_ANSWER_CRC || can_modbus.answer == MBUS_ANSWER_TIMEOUT ) );
            //к размеру принятых данных добавим минимальный размер пакета с ответом
            can_modbus.length = len_mbus_data + CAN_DATA_MBUS_MIN;
            //отправка ответа на запрос MODBUS
            msg = ID_DEV_MODBUS_ANS;
            osMessageQueuePut( hmi_msg, &msg, 0, 0 );               
           }
       }
    //сохраним команду в лог файле
    CmdSaveLog( que_cmd );
    //Установка значения параметра настройки
    if ( que_cmd->dev_id == ID_CONFIG ) {
        memset( &cfg_set, 0x00, sizeof( cfg_set ) );
        memcpy( &cfg_set, que_cmd->data, que_cmd->len_data );
        //обработка значения параметра содержащегося в двух пакетах
        if ( ( que_cmd->param_id == CFG_SCR_FILE || que_cmd->param_id == CFG_JOB_FILE || que_cmd->param_id == CFG_JOB_TEST ) && que_cmd->sub_pack_id == 0 ) {
            //первый пакет
            memset( str_val, 0x00, sizeof( str_val ) );
            memcpy( str_val, cfg_set.uint8_array, sizeof( cfg_set.uint8_array ) );
            return;
           }
        if ( ( que_cmd->param_id == CFG_SCR_FILE || que_cmd->param_id == CFG_JOB_FILE || que_cmd->param_id == CFG_JOB_TEST ) && que_cmd->sub_pack_id == 1 ) {
            //второй пакет
            memcpy( str_val + 8, cfg_set.uint8_array, sizeof( cfg_set.uint8_array ) );
            cfg_set.ptr = str_val;
            //проверка данных на корректность
            if ( ConfigChkVal( que_cmd->param_id, cfg_set ) == SUCCESS ) {
                ConfigSaveLog( que_cmd->param_id, cfg_set, SUCCESS );
                //сохранение параметров: CFG_SCR_FILE/CFG_JOB_FILE/CFG_JOB_TEST
                ConfigSet( que_cmd->param_id, &cfg_set );
               }
            else ConfigSaveLog( que_cmd->param_id, cfg_set, ERROR );
            return;
           }
        if ( !que_cmd->sub_pack_id && que_cmd->len_data ) {
            //сохранение значения параметров кроме: CFG_SCR_FILE/CFG_JOB_FILE/CFG_JOB_TEST
            if ( ConfigChkVal( que_cmd->param_id, cfg_set ) == SUCCESS ) {
                ConfigSaveLog( que_cmd->param_id, cfg_set, SUCCESS );
                //сохранение параметров
                ConfigSet( que_cmd->param_id, &cfg_set );
               }
            else ConfigSaveLog( que_cmd->param_id, cfg_set, ERROR );
           }
        if ( que_cmd->sub_pack_id == CAN_CONFIG_SAVE )
            ConfigSave(); //сохраним параметры в EEPROM
       }
 }

//*************************************************************************************************
// Отправка ответа MODBUS в контроллер HMI
//*************************************************************************************************
static void CANMbusAnswer( void ) {

    uint8_t idx = 0, *offset, rest, data_len;

    if ( can_modbus.answer == MBUS_ANSWER_OK ) {
        rest = can_modbus.length;
        offset = (uint8_t *)&can_modbus;
        do {
            //определим размер данных в одном пакете
            if ( rest > CAN_DATA_MAX )
                data_len = CAN_DATA_MAX;
            else data_len = rest;
            //отправка пакета
            CANSendFrame( CAN_DEV_ID( ID_DEV_MODBUS_ANS ) | CAN_PACK_SUB( idx++ ), offset, data_len );
            //следующий блок данных для передачи
            offset += data_len;
            //остаток байт для передачи
            rest -= data_len;
           } while ( rest );
       }
    else {
        //ответ с ошибкой, отправим только код ошибки, адрес, кол-во регистров
        can_modbus.length = CAN_DATA_MBUS_MIN;
        CANSendFrame( CAN_DEV_ID( ID_DEV_MODBUS_ANS ), (uint8_t *)&can_modbus, can_modbus.length );
       }
 }

//*************************************************************************************************
// Возвращает значение параметра (состояние связи по CAN шине)
// ParamHmi id_param - ID параметра
// return ValueParam - значение параметра
//*************************************************************************************************
ValueParam HmiGetValue( ParamHmi id_param ) {

    ValueParam value;
    ARM_CAN_STATUS can_status;
    
    value.uint32 = NULL;
    can_status = CanDrv->GetStatus();
    if ( id_param == HMI_LINK ) {
        if ( error_send || can_status.tx_error_count || can_status.rx_error_count )
            value.uint32 = LINK_CONN_NO;
        else value.uint32 = LINK_CONN_OK;
       }
    if ( id_param == HMI_SEND )
        value.uint32 = send_pack;
    if ( id_param == HMI_ERROR_SEND )
        value.uint32 = error_send;
    if ( id_param == HMI_ERROR_TX )
        value.uint8 = can_status.tx_error_count;
    if ( id_param == HMI_ERROR_RX )
        value.uint8 = can_status.rx_error_count;
    return value;
 }

//*************************************************************************************************
// Добавляет в протокол "cmd_yyyymmdd.log" команды управления от HMI
//*************************************************************************************************
static void CmdSaveLog( MSGQUEUE_CAN *que_cmd ) {

    uint8_t i;
    FILE *cmd_log;
    char name[40];

    if ( SDStatus() == ERROR )
        return; //карты нет
    //открываем файл
    sprintf( name, "\\hmi\\cmd_%s.log", RTCFileName() );
    cmd_log = fopen( name, "a" );
    if ( cmd_log == NULL )
        return; //файл не открылся
    //запишем данные
    fprintf( cmd_log, "%s ID=0x%03X %s PAR=0x%02X %s SUB=0x%02X LEN=%u DATA=", RTCGetLog(), que_cmd->dev_id, DevName( que_cmd->dev_id ), 
             que_cmd->param_id, ConfigName( que_cmd->param_id ), que_cmd->sub_pack_id, que_cmd->len_data );
    for ( i = 0; i < que_cmd->len_data; i++ )
        fprintf( cmd_log, "0x%02X ", *( que_cmd->data + i ) );
    fprintf( cmd_log, "\r\n" );
    fclose( cmd_log );
 }

//*************************************************************************************************
// Добавляем в протокол "cfg_yyyymmdd.log" изменяемые значения параметров
//*************************************************************************************************
static void ConfigSaveLog( ConfigParam id_par, ConfigValSet cfg_set, Status check ) {

    FILE *cfg_log;
    const DevParam *dev_ptr;
    char name[40], buff[40], old[40];

    if ( SDStatus() == ERROR )
        return; //карты нет
    //открываем файл
    sprintf( name, "\\hmi\\cfg_%s.log", RTCFileName() );
    cfg_log = fopen( name, "a" );
    if ( cfg_log == NULL )
        return; //файл не открылся
    dev_ptr = DevParamPtr( ID_CONFIG );
    if ( dev_ptr[id_par].subtype == STRING )
        sprintf( buff, "%s", cfg_set.ptr );
    if ( dev_ptr[id_par].subtype == NUMBER || dev_ptr[id_par].subtype == SYS_MODE || 
         dev_ptr[id_par].subtype == MODE_DIR || dev_ptr[id_par].subtype == BOOL1 || dev_ptr[id_par].subtype == BOOL7 )
        sprintf( buff, "%d", cfg_set.uint16 );
    if ( dev_ptr[id_par].subtype == FLOAT )
        sprintf( buff, "%.6f", cfg_set.dbl );
    if ( dev_ptr[id_par].subtype == SDATE )
        sprintf( buff, "%02u.%02u.%04u", cfg_set.date.day, cfg_set.date.month, cfg_set.date.year );
    if ( dev_ptr[id_par].subtype == TIMESTART )  
        sprintf( buff, "%d-%d-%d-%d-%d-%d-%d-%d", cfg_set.uint8_array[0], cfg_set.uint8_array[1], cfg_set.uint8_array[2], cfg_set.uint8_array[3],
        cfg_set.uint8_array[4], cfg_set.uint8_array[5], cfg_set.uint8_array[6], cfg_set.uint8_array[7] );
    strcpy( old, ParamGetForm( ID_CONFIG, id_par, PARAM_VALUE ) );
    if ( check == SUCCESS )
        fprintf( cfg_log, "%s %s: новый: %s, текущий: %s\r\n", RTCGetLog(), ParamGetForm( ID_CONFIG, id_par, PARAM_DESC ), buff, old );
    else fprintf( cfg_log, "%s %s: %s - %s", RTCGetLog(), ParamGetForm( ID_CONFIG, id_par, PARAM_DESC ), buff, Message( CONS_MSG_ERR_PARAM ) );
    fclose( cfg_log );
 }
