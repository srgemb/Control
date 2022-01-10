
#ifndef __INVETER_DEF_H
#define __INVETER_DEF_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "device.h"
#include "dev_param.h"

//*************************************************************************************************
// Команды и ответы инвертора
//*************************************************************************************************
// Команда включения инвертора
// C100000000000000+cr
// Ответ:
// C<0>F/W s/n: 3FTS-1000107abcRRRR<0>
// Download-<0>3
// Download-<0>2
// Download-<0>1
//
// Статус инвертора
// Q+cr = (000 025 24.5 025 40.0 519 50.0 000 000 0000000001000000001)
//
// Конфигурация инвертора (EEPROM)
// I+cr = #xxx28.5 26.5 22.5 21.00 22.0 MEANWELL LOC-123456789 REV:1.07 TS-1000-224 06/23/2007
//
// Команда включения Saving mode
// "W" 02 01 04 "28.5 26.5 22.5 21.0 22.0 MEANWELL LOC-1234567890 REV:1.07 TS-1000-224 " 00 " 06/23/2007" 01 02
//
// Команда выключения Saving mode
// "W" 02 00 04 "28.5 26.5 22.5 21.0 22.0 MEANWELL LOC-1234567890 REV:1.07 TS-1000-224 " 00 " 06/23/2007" 01 02
//
// Команда выключения инвертора
// Q+cr
// Ответ:
// C

#define TS_MSK_INVMODE      (1<<18)         //Режим инвертора
#define TS_MSK_BYPASS       (1<<17)         //Режим обхода (для моделей TN)
#define TS_MSK_AC_ON        (1<<16)         //Включено внешнее питание (для моделей TN)
#define TS_MSK_AC_CHARGE    (1<<15)         //Включена зарядка от внешнего источника (для моделей TN)
#define TS_MSK_SOLAR_ON     (1<<14)         //Включена солнечная панель (для моделей TN)
#define TS_MSK_SAVING       (1<<13)         //Дежурный режим 
#define TS_MSK_BAT_LOW      (1<<12)         //Низкий уровень заряда АКБ
#define TS_MSK_BAT_UUP      (1<<11)         //Батарея разряжена
#define TS_MSK_BAT_OVP      (1<<10)         //Батарея защищена от перенапряжения (для моделей TN)
#define TS_MSK_RMT_SHDN     (1<<9)          //Инвертор выключен удаленно
#define TS_MSK_OVR_100_115  (1<<8)          //Перегрузка по выходу 100% ~ 115%
#define TS_MSK_OVR_115_150  (1<<7)          //Перегрузка по выходу 115% ~ 150%
#define TS_MSK_OVR_150      (1<<6)          //Перегрузка по выходу больше 150%
#define TS_MSK_OVERHEAT     (1<<5)          //Перегрев
#define TS_MSK_INV_OVP      (1<<4)          //Инвертор защищен от перенапряжения
#define TS_MSK_INV_UVP      (1<<3)          //Инвертор защищен от низкого входного напряжение
#define TS_MSK_ERROR        (1<<2)          //Ошибка инвертора
#define TS_MSK_EEPROM_ERR   (1<<1)          //Ошибка EEPROM
#define TS_MSK_POWER_OFF    (1<<0)          //Инвертор выключен

//группы масок состояния инвертора
#define TS_MSK_GLB_MODE    ( TS_MSK_INVMODE | TS_MSK_SAVING | TS_MSK_RMT_SHDN | TS_MSK_POWER_OFF )
#define TS_MSK_GLB_ERROR   ( TS_MSK_BAT_LOW | TS_MSK_BAT_UUP | TS_MSK_OVR_100_115 | TS_MSK_OVR_115_150 | \
                             TS_MSK_OVR_150 | TS_MSK_OVERHEAT | TS_MSK_ERROR TS_MSK_EEPROM_ERR )

//тип данных от инвертора
typedef enum {
    INV_PACK_NODATA,                        //данных нет
    INV_PACK_STARTUP,                       //информация при включении инвертора
    INV_PACK_SHUTDOWN,                      //выключение 
    INV_PACK_EEPROM,                        //конфигурация
    INV_PACK_STATUS                         //текущий статус
 } TypePack;

#define INV_DATA_NOTUSED    255             //признак пропуска параметра при разборе данных пакета

#define CLR_CONFIG          0x01            //очистить только данные EEPROM
#define CLR_DATA            0x02            //очистить только данные
#define CLR_STATUS          0x04            //очистить только биты статуса
#define CLR_ALL             CLR_CONFIG | CLR_DATA | CLR_STATUS //очистить все данные

//битовые маски состяния инвертора
#define TS_MSK_MAX_SIZE     19              //максимальный размер строки с битовой маской

//команды управления инвертором (битовые)
typedef enum {
    INV_CMD_NULL,                           //нет команды
    INV_CMD_STAT,                           //запрос статуса инвертора (циклическая)
    INV_CMD_CONFIG,                         //конфигурация инвертора (циклическая до первого ответа)
    INV_CMD_ON = 4,                         //программное включение инвертора
    INV_CMD_OFF = 8                         //программное выключение инвертора
 } InvCommand;

//команды инвертора
static char * const inv_comnd[] = {
    NULL,                                   //нет команды
    "Q\r",                                  //команда запроса статуса
    "I\r",                                  //команда запроса конфигурация
    NULL,                                   //нет команды
    "C010000000000000\r",                   //команды включения инвертора
    NULL,                                   //нет команды
    NULL,                                   //нет команды
    NULL,                                   //нет команды
    "C100000000000000\r"                    //команда выключения инвертора
 };

//Параметры EEPROM инвертора, номер индекса - порядковый номер
//значение по индексу - номер параметра
const static uint8_t eeprom_msk[] = { 
    INV_CFG_EQL_VOLT,                       //напряжение выравнивания
    INV_CFG_FLT_VOLT,                       //напряжение поддержания
    INV_CFG_ALARM_VOLT,                     //пониженное напряжение
    INV_CFG_SHDN_VOLT,                      //напряжение выключения
    INV_DATA_NOTUSED, 
    INV_VENDOR,                             //производитель
    INV_DATA_NOTUSED, 
    INV_VERSION,                            //версия
    INV_MODEL,                              //модель
    INV_DATA_NOTUSED, 
    INV_DATA_NOTUSED, 
    INV_DATA_NOTUSED, 
    INV_DATA_NOTUSED 
 }; 

//параметры для разбора данных статуса, номер индекса - порядковый номер
//значение по индексу - номер параметра
const static uint8_t status_msk[] = { 
    INV_AC_OUT,                             //напряжение на выходе
    INV_AC_POWER,                           //выходная мощность в % (0-25-50-75-100)
    INV_DC_IN,                              //напряжение АКБ
    INV_BAT_PERC,                           //уровень зарядки АКБ % (0-25-50-75-100)
    INV_TEMPERATURE,                        //температура инвертора
    INV_UNUSED, 
    INV_AC_FREQ,                            //частота напряжения на выходе
    INV_WORK_TIME,                          //прогнозируемое время работы min
    INV_POWER_PERC,                         //потребляемая мощность на выходе 0-100%
    INV_STATUS                              //биты состояния инвертора
 }; 

#endif 
