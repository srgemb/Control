
#ifndef __EEPROM_H
#define __EEPROM_H

#include <stdbool.h>

#include <lpc_types.h>

#include "device.h"
#include "dev_param.h"

//режим сохранения/восстановления параметра
typedef enum {
    EEPROM_SAVE,                            //сохранить значение
    EEPROM_RESTORE                          //восстановить значение
 } EepromMode;

//индексы параметров состояния выходов управления
typedef enum {
    EEP_BP_CHARGE,                          //режим зарядка выкл/2/3/8
    EEP_TRC_ON,                             //трекер вкл
    EEP_PV_CONN,                            //режим подключения солнечных панелей
    EEP_PV_MODE,                            //режим соединения солнечных панелей
    EEP_ALT_AC_OFF,                         //режим работы нагрузки от инверторов
    EEP_RESERV1,                            //резервное реле 1
    EEP_RESERV2,                            //резервное реле 2
    EEP_RESERV3,                            //резервное реле 3
    EEP_RESERV4                             //резервное реле 4
 } EepromParam;

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void EepromInit( void );
void EepromSave( void );
void EepromClear( uint8_t page );
uint8_t EepromLoad( EepromParam id_param );
void EepromUpdate( EepromParam id_param, uint8_t value );
void ConfigClear( void );
void ConfigLoad( void );
void ConfigSet( ConfigParam id_par, ConfigValSet *value );
void ConfigSave( void );

#endif
