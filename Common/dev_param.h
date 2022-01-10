
#ifndef __DEV_PARAM_H
#define __DEV_PARAM_H

#include <stdint.h>
#include <stdbool.h>

#include <lpc_types.h>

#include "device.h"
#include "dev_data.h"

#define BUFFER_PARAM        160             //размер буфера для формирования результата по одному параметру

#define PARAM1_ALIGNMENT    33              //граница заполнения названия параметра символом "..." справа
                                            //для всех параметров кроме ConfigParam
#define PARAM2_ALIGNMENT    70              //граница заполнения названия параметра символом "..." справа
                                            //для параметров ConfigParam

#define SIZE_ARRAY( array ) ( sizeof( array )/sizeof( array[0] ) )

//*************************************************************************************************
// Подтип преобразования значения параметра
//*************************************************************************************************
typedef enum {
    NOTYPE,
    BOOL1,                                  //логическое "Да/Нет"
    BOOL2,                                  //логическое "Вкл/Выкл"
    BOOL3,                                  //логическое "Исправен/Неисправен"
    BOOL4,                                  //логическое "Завершена/Заряд"
    BOOL5,                                  //логическое "Паралл/Пар+Посл"
    BOOL_BRK,                               //логическое "Авар/ОК" для автомата защиты
    BOOL7,                                  //логическое "Автоматический/Ручной"
    BOOL8,                                  //логическое "OK/Нет"
    BOOL_FUSE,                              //логическое "Авар/ОК" для предохранителя
    NUMBER,                                 //число целое
    NUMSIGN,                                //число целое со знаком
    FLOAT,                                  //число float
    DOUBLE,                                 //число double
    DATES,                                  //дата dd.mm.yyyy
    STRING,                                 //строка
    STRINT,                                 //строка с целыми числами "123569", каждая цифра - отдельное значение
    SDATE,                                  //дата (строка)
    TIME_FULL,                              //преобразование в uint16 -> HH:MM:SS (вызов функции)
    TIME_1SHORT,                            //преобразование в uint32 -> HH:MM (вызов функции)
    TIME_2SHORT,                            //преобразование в uint16 -> MM:SS (вызов функции)
    TIME_3SHORT,                            //преобразование в uint32 -> HHH:MM (вызов функции)
    TIME_SUN,                               //преобразование в float -> HH:MM (вызов функции)
    PWR_STAT,                               //источник питания нагрузки (вызов функции)
    MPPT_MODE,                              //режим контроллера MPPT (вызов функции)
    CHRGE_MODE,                             //режим заряда PB-100-224 (вызов функции)
    GEN_MODE,                               //режим генератора (вызов функции)
    GEN_STAT,                               //статус генератора (вызов функции)
    GEN_ERROR,                              //ошибки генератора (вызов функции)
    INVR_MODE,                              //режим работы инвертора (вызов функции)
    INVR_ERR_CTRL,                          //ошибки управления инвертором (вызов функции)
    INVR_ERROR,                             //ошибки инвертора (вызов функции)
    SPA_ERRDS,                              //ошибки при проверки исходных данных (вызов функции)
    TRAC_MODE,                              //режим работы трекера (вызов функции)
    TRAC_MODE2,                             //режим работы трекера (вызов функции) (англ)
    SYS_MODE,                               //режим работы системы
    MODE_DIR,                               //режим логирования файлов
    TIMESTART                               //длительность запуска генератора для каждой попытки
 } SubType;

//*************************************************************************************************
// Структура описания параметров уст-в и их типов
//*************************************************************************************************
typedef struct {
    char * const name;                      //имя параметра
    char * const frm;                       //формат вывода значения параметра
    char * const units;                     //единицы измерения параметра
    SubType      subtype;                   //подтип параметра (формат вывода определен через функцию)
    bool         view_hmi;                  //отображение параметра в модуле HMI
    char * const comment;                   //описание параметра
} DevParam;

//*************************************************************************************************
// Структура описания предельных значений параметров настройки
//*************************************************************************************************
typedef struct {
    ConfigParam param;                      //ID параметра
    bool        check;                      //признак выполнения проверки
    SubType     type_var;                   //тип данных
    uint8_t     size_data;                  //размер данных
    int32_t     min_value;                  //минимальное значение
    int32_t     max_value;                  //максимальное значение
} ConfigCheck;

//*************************************************************************************************
// Варианты отображения параметра (логическое комбинирование по ИЛИ)
//*************************************************************************************************
typedef enum {
    PARAM_NUMB  = 0x01,                     //номер параметра
    PARAM_DESC  = 0x02,                     //описание параметра
    PARAM_DOT   = 0x04,                     //дополнение описания параметра справа "..."
    PARAM_VALUE = 0x08,                     //значение параметра
    PARAM_UNIT  = 0x10                      //единицы измерения параметра
 } ParamMode;

//*************************************************************************************************
// Тип значения параметров уст-в
//*************************************************************************************************
typedef union {
    int8_t      int8;
    uint8_t     uint8;
    uint16_t    uint16;
    uint32_t    uint32;
    float       flt;
    void        *ptr;
    DATE        date;
 } ValueParam;

//*************************************************************************************************
// Тип значения параметра настроек управляющего контроллера
//*************************************************************************************************
typedef union {
    int8_t      int8;
    uint8_t     uint8;
    uint16_t    uint16;
    uint16_t    int16;
    int32_t     int32;
    uint32_t    uint32;
    uint8_t     uint8_array[8];
    double      dbl;
    DATE        date;
    char        *ptr;
 } ConfigValSet;

//*************************************************************************************************
// Функции статуса/состояния
//*************************************************************************************************
Device DevGetInd( char *name );
char *DevName( Device dev );
char *ConfigName( ConfigParam param );
uint8_t DevParamCnt( Device dev, CountType type );
uint8_t ParamGetInd( Device dev, char *name );
const DevParam *DevParamPtr( Device dev );
char *ParamGetName( Device dev, uint32_t param );
ValueParam ParamGetVal( Device dev, uint32_t param );
char *ParamGetForm( Device dev, uint32_t param, ParamMode mode );
char *ParamGetDesc( Device dev, uint32_t param );
uint8_t AddDot( char *src, uint8_t aligment );
char *ErrorDescr( Device dev, uint8_t err_dev, uint8_t err_ctrl );
void StrToConfigVal( ConfigParam id_par, char *value, ConfigValSet *cfg_set );
Status ConfigChkVal( ConfigParam id_par, ConfigValSet cfg_set );
void ConfigLimit( ConfigParam id_par, int32_t *min, int32_t *max );
uint8_t ConfigParSize( ConfigParam id_par );
uint8_t DevParamRelat( Device dev, uint32_t param );

ValueParam PortsGetValue( ParamPort id_param );
ValueParam BatmonGetValue( ParamBatMon id_param );
ValueParam MpptGetValue( ParamMppt id_param );
ValueParam PvGetValue( ParamPv id_param );
ValueParam ChargeGetValue( ParamCharger id_param );
ValueParam AltGetValue( ParamAlt id_param );
ValueParam TrackerGetValue( ParamTracker id_param );
ValueParam InvGetValue( Device dev, ParamInv id_param );
ValueParam SpaGetValue( ParamSunPos id_param );
ValueParam VoiceGetValue( ParamVoice id_param );
ValueParam ConfigValue( ConfigParam id_param );
ValueParam RtcGetValue( ParamRtc id_param );
ValueParam GenGetValue( ParamGen id_param );

#endif
