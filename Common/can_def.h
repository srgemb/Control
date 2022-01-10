
//*************************************************************************************************
//
// Описание структур, масок для обмена данными по CAN шине
//
//*************************************************************************************************

#ifndef __CAN_DEF_H
#define __CAN_DEF_H

#include <stdint.h>
#include <stdbool.h>

#include "dev_data.h"
#include "message.h"

//структура сообщения с данными
typedef struct {
    Device      dev_id;                     //ID устройства
    ConfigParam param_id;                   //ID параметра
    uint8_t     sub_pack_id;                //ID дополнительного пакета
    uint8_t     len_data;                   //размер данных
    uint8_t     data[8];                    //данные
} MSGQUEUE_CAN;

//структура сообщения с данными
typedef struct {
    Device      dev_id;                     //ID устройства
    LogMessId   id_mess;                    //ID сообщения
} MSGQUEUE_LOG;

//маски атрибуты в пакете данных
#define CAN_POS_DEV_ID          16                              //смещение для ID уст-ва
#define CAN_POS_PARAM_ID        8                               //смещение для ID параметра
#define CAN_POS_PACK_ID         0                               //смещение для ID пакета

#define CAN_MASK_DEV_ID         ( 0xFFU << CAN_POS_DEV_ID )     //маска ID уст-ва
#define CAN_MASK_PARAM_ID       ( 0xFFU << CAN_POS_PARAM_ID )   //маска ID параметра
#define CAN_MASK_PACK_ID        ( 0xFFU << CAN_POS_PACK_ID )    //маска ID пакета
#define CAN_MASK_MESS_ID        ( 0xFFU << CAN_POS_PACK_ID )    //маска ID пакета

#define CAN_GET_DEV_ID( id )    ( id >> CAN_POS_DEV_ID )        //ID уст-ва
#define CAN_GET_PARAM_ID( id )  ( id >> CAN_POS_PARAM_ID )      //ID параметра
#define CAN_GET_PACK_ID( id )   ( id >> CAN_POS_PACK_ID )       //ID суб-пакета

#define CAN_DEV_ID( id )        ( id << CAN_POS_DEV_ID )        //ID уст-ва со смещением
#define CAN_PARAM_ID( id )      ( id << CAN_POS_PARAM_ID )      //ID параметра со смещением
#define CAN_PACK_SUB( id )      ( id << CAN_POS_PACK_ID )       //ID суб-пакета со смещением

#define CAN_FILTER_MESS         0x1FFFFF00UL                    //значение фильтра для выделения DEV_ID

#define CAN_FILTER_RANGE        0x1FFFFFFFUL                    //максимальное значение фильтра

#define CAN_CONFIG_SAVE         0xFF                            //сохранить параметры в EEPROM

#endif

