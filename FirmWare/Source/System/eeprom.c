
//*************************************************************************************************
//
// Управление хранением параметров в EEPROM
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "lpc177x_8x_eeprom.h"

#include "dev_data.h"

#include "eeprom.h"
#include "command.h"
#include "config.h"
#include "rtc.h"
#include "scheduler.h"
#include "outinfo.h"

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define EEPROM_PAGE_PIN         0           //номер блока для хранения состояния выходов управления
#define EEPROM_PAGE_CFG         1           //номер блока для хранения настроек

//кол-во блоков для хранения настроек
#define CFG_BLOCK               (uint16_t)( ( sizeof( CONFIG ) / EEPROM_PAGE_SIZE ) + 1 )

//размер промежуточного блока для обработки структуры хранения настроек
#define CFG_SIZE                EEPROM_PAGE_SIZE * CFG_BLOCK

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
CONFIG config;                              //структура для хранения настроек

static uint8_t ee_flg = false;              //признак изменения параметров и сохранение 
                                            //еще не выполнено
static uint8_t cfg_buff[CFG_SIZE];          //буфер параметров настройки
static uint8_t ee_value[EEPROM_PAGE_SIZE];  //буфер параметров выходов управления

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void ConfigSaveFile( void );

//*************************************************************************************************
// Инициализация EEPROM памяти, чтение состояния выходов и настроек
//*************************************************************************************************
void EepromInit( void ) {

    uint8_t cfg_buff[CFG_SIZE];
    
    EEPROM_Init();
    EEPROM_PowerDown( DISABLE );
    //загрузим состояние выходов управления, 1 блок
    EEPROM_Read( 0, EEPROM_PAGE_PIN, (uint8_t*)&ee_value, MODE_8_BIT, EEPROM_PAGE_SIZE );
    //загрузим настройки
    ConfigClear();
    EEPROM_Read( 0, EEPROM_PAGE_CFG, (uint8_t*)&cfg_buff, MODE_8_BIT, CFG_SIZE );
    memcpy( (uint8_t *)&config, cfg_buff, sizeof( CONFIG ) ); 
 }

//*************************************************************************************************
// Обновить значение переменной в промежуточном буфере
// EepromParam id_param - ID переменной
// uint8_t value        - новое значение переменной
//*************************************************************************************************
void EepromUpdate( EepromParam id_param, uint8_t value ) {

    ee_flg = true; //признак наличия обновленных данных
    ee_value[id_param] = value;
 }

//*************************************************************************************************
// Восстановить блок данных из EEPROM в промежуточный буфер
// EepromParam id_param - номер параметра
//*************************************************************************************************
uint8_t EepromLoad( EepromParam id_param ) {

    //перечитаем значения из EEPROM только если не установлен признак обновления данных
    if ( ee_flg == false )
        EEPROM_Read( 0, EEPROM_PAGE_PIN, (uint8_t*)&ee_value, MODE_8_BIT, EEPROM_PAGE_SIZE );
    return ee_value[id_param];
 }

//*************************************************************************************************
// Записать данные из промежуточного буфера в память
//*************************************************************************************************
void EepromSave( void ) {

    EEPROM_Write( 0, EEPROM_PAGE_PIN, (uint8_t*)&ee_value, MODE_8_BIT, EEPROM_PAGE_SIZE );
    ee_flg = false;
 }

//*************************************************************************************************
// Очистить блок данных и записать в EEPROM
// uint8_t page - номер страницы (блока) 0-63
//*************************************************************************************************
void EepromClear( uint8_t page ) {

    if ( page < EEPROM_PAGE_NUM ) {
        memset( ee_value, 0x00, sizeof( ee_value ) );
        EEPROM_Write( 0, page, (uint8_t*)&ee_value, MODE_8_BIT, EEPROM_PAGE_SIZE );
       }
 }

//*************************************************************************************************
// Обнуление значений всех настроек в RAM
//*************************************************************************************************
void ConfigClear( void ) {

    memset( cfg_buff, 0x00, sizeof( cfg_buff ) ); 
    memcpy( &config, cfg_buff, sizeof( CONFIG ) ); 
 }

//*************************************************************************************************
// Сохранить значения настроек в EEPROM
//*************************************************************************************************
void ConfigSave( void ) {

    memset( cfg_buff, 0x00, sizeof( cfg_buff ) ); 
    memcpy( cfg_buff, &config, sizeof( CONFIG ) ); 
    EEPROM_Write( 0, EEPROM_PAGE_CFG, (uint8_t*)&cfg_buff, MODE_8_BIT, CFG_SIZE );
    ConfigSaveFile();
 }

//*************************************************************************************************
// Сохранить значения настроек в двоичный файл config_eeprom.bin
//*************************************************************************************************
static void ConfigSaveFile( void ) {

    FILE *bin;
    
    bin = fopen( "config_eeprom.bin", "wb" );
    if ( bin == NULL )
        return;
    fwrite( &config, sizeof( uint8_t ), sizeof( CONFIG ), bin );
    fclose( bin );
 }

//*************************************************************************************************
// Установка значения параметра
// ConfigParam id_par  - ID параметра
// ConfigValSet *value - значение параметра
//*************************************************************************************************
void ConfigSet( ConfigParam id_par, ConfigValSet *value ) {

    if ( value == NULL )
        return;
    //файл экрана
    if ( id_par == CFG_SCR_FILE ) {
        //проверим изменение параметра
        if ( strcasecmp( value->ptr, config.scr_file ) ) {
            strcpy( config.scr_file, value->ptr );
            ScreenLoad();
           }
       }
    //имя файла задания для режима "Резервный", "Смешанный"
    if ( id_par == CFG_JOB_FILE ) {
        //проверим изменение параметра
        if ( strcasecmp( value->ptr, config.job_file ) ) {
            strcpy( config.job_file, value->ptr );
            LoadJobs();
           }
       }
    //имя файла задания для режима "Тестовый"
    if ( id_par == CFG_JOB_TEST ) {
        //проверим изменение параметра
        if ( strcasecmp( value->ptr, config.job_test ) ) {
            strcpy( config.job_test, value->ptr );
            LoadJobs();
           }
       }
    //режим работы системы
    if ( id_par == CFG_MODE_SYS )
        config.mode_sys = value->uint8;
    //Минимальный ток отключения зарядного уст-ва BP-1000-224
    if ( id_par == CFG_PB_CURRENT_STOP )
        config.pb_current_stop = value->uint8;
    //дата последного включения подзарядки от основной сети
    if ( id_par == CFG_LAST_CHARGE )
        config.last_charge = value->date;
    //период записи данных зарядного уст-ва
    if ( id_par == CFG_DATLOG_UPD_CHARGE )
        config.datlog_upd_chrg = value->uint16;
    //период записи данных солнечного контроллера заряда
    if ( id_par == CFG_DATLOG_UPD_MPPT )
        config.datlog_upd_mppt = value->uint16;
    //период записи данных монитора батареи
    if ( id_par == CFG_DATLOG_UPD_BMON )
        config.datlog_upd_bmon = value->uint16;
    //период записи данных инверторов
    if ( id_par == CFG_DATLOG_UPD_INV )
        config.datlog_upd_inv = value->uint16;
    //период записи данных трекера
    if ( id_par == CFG_DATLOG_UPD_TRC )
        config.datlog_upd_trc = value->uint16;
    //ведение лог-файлов, логирование команд управление солнечными панелями
    if ( id_par == CFG_LOG_ENABLE_PV )
        config.log_enable_pv = value->uint8;
    //ведение лог-файлов, логирование команд зарядного уст-ва
    if ( id_par == CFG_LOG_ENABLE_CHARGE )
        config.log_enable_chrg = value->uint8;
    //ведение лог-файлов, логирование команд/данных вкл/выкл солнечных панелей
    if ( id_par == CFG_LOG_ENABLE_MPPT )
        config.log_enable_mppt = value->uint8;
    //ведение лог-файлов, логирование команд/данных управление инверторов
    if ( id_par == CFG_LOG_ENABLE_INV )
        config.log_enable_inv = value->uint8;
    //ведение лог-файлов, логирование данных монитора батареи
    if ( id_par == CFG_LOG_ENABLE_BMON )
        config.log_enable_bmon = value->uint8;
    //ведение лог-файлов, логирование команд генератора
    if ( id_par == CFG_LOG_ENABLE_GEN )
        config.log_enable_gen = value->uint8;
    //ведение лог-файлов, логирование команд блока АВР
    if ( id_par == CFG_LOG_ENABLE_ALT )
        config.log_enable_alt = value->uint8;
    //ведение лог-файлов, логирование команд трекера
    if ( id_par == CFG_LOG_ENABLE_TRC )
        config.log_enable_trc = value->uint8;
    //генератор, задержка запуска генератора после отключения основной сети (сек)
    if ( id_par == CFG_GEN_DELAY_START )
        config.gen_delay_start = value->uint16;
    //генератор, задержка выключения генератора после восстановления основной сети (сек)
    if ( id_par == CFG_GEN_DELAY_STOP )
        config.gen_delay_stop = value->uint16;
    //генератор, ожидание сигнала запуска генератора (сек)
    if ( id_par == CFG_GEN_DELAY_CHK_RUN )
        config.gen_delay_chk_run = value->uint8;
    //генератор, пауза между запусками (сек) фактическое время = delay_check_run + before_start
    if ( id_par == CFG_GEN_BEFORE_START )
        config.gen_before_start = value->uint8;
    //генератор, кол-во попыток запуска генератора (макс - 8)
    if ( id_par == CFG_GEN_CNT_START )
        config.gen_cnt_start = value->uint8;
    //генератор, продолжительность запуска для каждой попытки (макс - 8)
    if ( id_par == CFG_GEN_TIME_START )
        memcpy( config.gen_time_start, value->uint8_array, sizeof( config.gen_time_start ) );
    //генератор, максимальная продолжительность работы генератора (сек)
    if ( id_par == CFG_GEN_TIME_RUN )
        config.gen_time_run = value->uint16;
    //генератор, продолжительность паузы между длительными работами (сек)
    if ( id_par == CFG_GEN_TIME_SLEEP )
        config.gen_time_sleep = value->uint16;
    //генератор, время тестирования генератора (сек)
    if ( id_par == CFG_GEN_TIME_TEST )
        config.gen_time_test = value->uint16;
    //генератор, ручной/автоматический режим запуска при отключении сети
    if ( id_par == CFG_GEN_AUTO_MODE )
        config.gen_auto_mode = value->uint8;
    //генератор, дата последнего запуска
    if ( id_par == CFG_GEN_LAST_RUN )
        config.gen_last_run = value->date;
    //SPA, часовой пояс
    if ( id_par == CFG_SPA_TIMEZONE )
        config.spa_timezone = value->uint8;
    //SPA, Широта наблюдателя
    if ( id_par == CFG_SPA_LATITUDE )
        config.spa_latitude = value->dbl;
    //SPA, Долгота наблюдателя
    if ( id_par == CFG_SPA_LONGITUDE )
        config.spa_longitude = value->dbl;
    //SPA, Высота наблюдателя
    if ( id_par == CFG_SPA_ELEVATION )
        config.spa_elevation = value->uint16;
    //SPA, Среднегодовое местное давление
    if ( id_par == CFG_SPA_PRESSURE )
        config.spa_pressure = value->uint16;
    //SPA, Среднегодовая местная температура
    if ( id_par == CFG_SPA_TEMPERATURE )
        config.spa_temperature = value->uint8;
    //SPA, Наклон поверхности (измеряется от горизонтальной плоскости)
    if ( id_par == CFG_SPA_SLOPE )
        config.spa_slope = value->uint16;
    //SPA, Вращение поверхности азимут (измеряется с юга на проекции нормали 
    if ( id_par == CFG_SPA_AZM_ROTATION )
        config.spa_azm_rotation = value->uint16;
    //таймер задержки вкл инверторов при отключении основной сети
    if ( id_par == CFG_DELAY_START_INV )
        config.delay_start_inv = value->uint16;
    //таймер задержки выкл инверторов после восстановлении основной сети
    if ( id_par == CFG_DELAY_STOP_INV )
        config.delay_stop_inv = value->uint16;
    //режим логирования файлов 0/1 - [каталог\файл]/[\каталог\YYYYMM\файл]
    if ( id_par == CFG_MODE_LOGGING )
        config.mode_logging = value->uint8;
 }

//*************************************************************************************************
// Загрузить параметры настроек по умолчанию
//*************************************************************************************************
void ConfigLoad( void ) {

    strcpy( config.scr_file, "screen.scr" );
    strcpy( config.job_file, "main.job" );
    strcpy( config.job_test, "test.job" );
    config.mode_sys          = 0;
    config.pb_current_stop   = 4;
    config.delay_start_inv   = 180;
    config.delay_stop_inv    = 180;
    //временные интервалы логирования
    config.datlog_upd_chrg   = 120;
    config.datlog_upd_mppt   = 120;
    config.datlog_upd_bmon   = 300;
    config.datlog_upd_inv    = 30;
    //разрешения логирования событий
    config.log_enable_pv     = 1;
    config.log_enable_chrg   = 1;
    config.log_enable_mppt   = 1;
    config.log_enable_inv    = 1;
    config.log_enable_bmon   = 1;
    config.log_enable_gen    = 1;
    config.log_enable_alt    = 1;
    config.log_enable_trc    = 1;
    //параметры генератора
    config.gen_delay_start   = 60;
    config.gen_delay_stop    = 60;
    config.gen_cnt_start     = 5;
    config.gen_delay_chk_run = 4;
    config.gen_before_start  = 1;
    config.gen_time_run      = 18000;
    config.gen_time_sleep    = 1800;
    config.gen_time_test     = 600;
    config.gen_auto_mode     = 0;
    config.gen_time_start    = { 1,2,3,3,3,3,3,3 };
    //параметры расчета положения солнца
    config.spa_timezone      = 3;
    config.spa_latitude      = 45.035470;
    config.spa_longitude     = 38.975313;
    config.spa_elevation     = 27;
    config.spa_pressure      = 760;
    config.spa_temperature   = 12;
    config.spa_slope         = 0;
    config.spa_azm_rotation  = 0;
 }
