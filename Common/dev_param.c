
//*************************************************************************************************
//
// Описание параметров устройств и форматы вывода данных 
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "device.h"
#include "dev_param.h"
#include "dev_data.h"

#ifdef CONFIG_CONTROL
#include "hmi_can.h"
#endif
#ifdef CONFIG_HMIDEV
#include "can.h"
#endif

#include "rtc.h"
#include "trc_calc.h"
#include "tracker_ext.h"
#include "events.h"

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
static char const result_ok[]     = "OK";
static char const result_undef[]  = "...";
static char * const bool_desc1[]  = { "Нет", "Да " };
static char * const bool_desc2[]  = { "Выкл", "Вкл " };
static char * const bool_desc3[]  = { "Завершена", "Заряд    " };
static char * const bool_desc4[]  = { "Неисправен", "Исправен   " };
static char * const bool_desc5[]  = { "Паралл  ", "Пар+Посл" };
static char * const bool_desc6[]  = { "ОК  ", "Авар" }; //для автомата защиты
static char * const bool_desc7[]  = { "Ручной        ", "Автоматический" };
static char * const bool_desc8[]  = { "Нет", "OK " };
static char * const bool_desc9[]  = { "Каталог\\Файл", "каталог\\YYYYMM\\файл" };
static char * const bool_desc10[] = { "Авар", "ОК  " }; //для предохранителя

//*************************************************************************************************
// Имена устройств в т.ч. по которым выполняется вывод информации через макроподстановки
//*************************************************************************************************
static char * const dev_name[] = { 
    NULL,
    "PORTS",                                //порты управления/состояния
    "RTC",                                  //часы реального времени
    "BAT",                                  //монитор АКБ BMV-600S
    "MPPT",                                 //контроллер заряда Prosolar SunStar MPPT SS-50C
    "PV",                                   //управление подключением солнечных панелей
    "CHARGE",                               //контроллер заряда PB-1000-24
    "TS1",                                  //инвертор TS-1000-224
    "TS3",                                  //инвертор TS-3000-224
    "ALT",                                  //блок АВР (автоматический ввод резерва)
    "GEN",                                  //контроллер генератора
    "TRC",                                  //контроллер солнечного трекера
    "SPA",                                  //положение солнца
    "VOICE",                                //голосовой информатор
    "HMI",                                  //модуль HMI
    "RESERV",                               //управление доп. реле
    "EXTOUT",                               //управление доп. выходами
    "MODBUS_REQ",                           //запрос данных по протоколу MODBUS
    "MODBUS_ANS",                           //ответ на запрос по протоколу MODBUS
    "CONFIG"                                //параметры настроек
 };

//*************************************************************************************************
// Параметры для блока АВР (автоматический ввод резерва)
//*************************************************************************************************
static const DevParam DeviceAlt[] = {
    { "CONN",           "%s",           "",         BOOL1,          true,       "Блок АВР подключен" },
    { "AC_MAIN",        "%s",           "",         BOOL1,          true,       "Наличие основной сети" },
    { "GEN_ON",         "%s",           "",         BOOL1,          true,       "Генератор включен" },
    { "PWR",            "%s",           "",         PWR_STAT,       true,       "Питание нагрузки от" },
    { "DELAY_TS",       "%s ",          "м:с",      TIME_2SHORT,    true,       "Время до вкл/выкл инверторов" },
    { NULL,             NULL,           NULL,       NOTYPE,         false,      NULL }
 };

//Источник питания нагрузки
static char * const pwr_src[] = { 
    "             ", 
    "Основной сети", 
    "Генератора   ", 
    "Инверторов   " 
 };

//расшифровка ошибок АВР
static char * const alt_err_descr[] = {
    "",
    "Блок АВР не подключен",
    "Инверторы не подключены",
    "На блоке АВР нет основной сети или сети генератора"
 };

//*************************************************************************************************
// Параметры контроллера заряда MPPT
//*************************************************************************************************
static const DevParam DeviceMppt[] = {
    { "PV_V",           "%4.1f ",       "V",        FLOAT,          true,       "Напряжение панелей" },
    { "PV_I",           "%4.1f ",       "A",        FLOAT,          true,       "Ток панелей" },
    { "V_OUT",          "%4.1f ",       "V",        FLOAT,          true,       "Выходное напряжение" },
    { "I_OUT",          "%4.1f ",       "A",        FLOAT,          true,       "Выходной ток" },
    { "ENERGY1",        "%5d ",         "W*h",      NUMBER,         true,       "Собранная энергия сегодня" },
    { "ENERGY2",        "%4d ",         "W*h",      NUMBER,         true,       "Собранная энергия сегодня" },
    { "TIME_FLT",       "%s ",          "ч:м",      TIME_1SHORT,    true,       "Время в режиме \"поддержки\"" },
    { "MODE",           "%s",           "",         MPPT_MODE,      true,       "Режим заряда" },
    { "TEMP_MPPT",      "%4.1f ",       "C",        FLOAT,          true,       "Температура контроллера" },
    { "SOC",            "%3d ",         "%%",       NUMBER,         true,       "Уровень заряда АКБ (SOC)" },
    { "I_BAT",          "%+6.1f ",      "A",        FLOAT,          true,       "Ток АКБ" },
    { "BAT_AH",         "%3d ",         "A*h",      NUMBER,         true,       "Остаточная емкость АКБ" },
    { "TEMP_BAT",       "%4.1f ",       "C",        FLOAT,          false,      "Температура АКБ" },
    { "SERIAL",         "%5d",          "",         STRING,         false,      "Серийный номер контроллера" },
    { "TIME_CHARGE",    "%s ",          "ч:м",      TIME_1SHORT,    true,       "Время заряда АКБ" },
    { "MPPT_ON",        "%s",           "",         BOOL1,          true,       "Контроллер включен" },
    { "LINK",           "%s",           "",         BOOL1,          true,       "Кабель связи подключен" },
    { "PV",             "%s",           "",         BOOL2,          true,       "Солнечные панели" },
    { "PVMODE",         "%s",           "",         BOOL5,          true,       "Режим панелей" },
    { "DATAUPD",        "%s",           "",         BOOL8,          true,       "Обмен данными" },
    { NULL,             NULL,           NULL,       NOTYPE,         true,       NULL }
 };

//режимы заряда контроллера MPPT
static char * const mppt_mode[] = {
    "Выключен    ",
    "MPPT        ",
    "Быстрый     ",
    "Насыщение   ",
    "Поддержка   ",
    "Начало      ",
    "Остановка   ",
    "Выравнивание" 
 };

//
static char * const pv_err_descr[] = {
    "OK",
    "Нет сигнала подтверждения вкл/выкл",
    "АКБ не подключен" 
 };

//*************************************************************************************************
// Параметры контроллера заряда PB-1000-224
//*************************************************************************************************
static const DevParam DeviceCharger[] = {
    { "CONN",           "%s",           "",         BOOL1,          true,       "Подключен к сети" },
    { "STAT",           "%s",           "",         BOOL4,          true,       "Исправность блока" },
    { "MODE",           "%s",           "",         CHRGE_MODE,     true,       "Режим заряда" },
    { "STAT_BNK",       "%s",           "",         BOOL3,          true,       "Статус заряда" },
    { "I",              "%3.1f ",       "A",        FLOAT,          true,       "Ток заряда" },
    { NULL,             NULL,           NULL,       NOTYPE,         false,      NULL }
 };

//декодирование режима зарядки
static char * const charge_mode[] = {
    "Вык    ",
    "Режим-2",
    "Режим-3",
    "Режим-8" 
 };

//расшифровка ошибок контроллера заряда
static char * const charge_err_descr[] = {
    "",
    "Нет сети AC.",
    "АКБ не подключена.",
    "Ошибка зарядного уст-ва.",
    "Неверный режим заряда."
 };

//*************************************************************************************************
// Параметры монитора АКБ BMV-600S
//*************************************************************************************************
static const DevParam DeviceBatmon[] = {
    { "V",              "%5.2f ",       "V",        FLOAT,          true,       "Напряжение АКБ" },
    { "I",              "%+5.2f ",      "A",        FLOAT,          true,       "Ток АКБ" },
    { "CE",             "%+7.2f ",      "Ah",       FLOAT,          true,       "Израсходованная энергия от АКБ" },
    { "SOC",            "%3.1f ",       "%%",       FLOAT,          true,       "Уровень заряда АКБ (SOC)" },
    { "TTG",            "%s ",          "ч:м",      TIME_3SHORT,    true,       "Продолжительность работы (TTG)" },
    { "ALARM",          "%1d",          "",         NUMBER,         true,       "Состояние звукового сигнала" },
    { "RELAY",          "%1d",          "",         NUMBER,         true,       "Состояние реле" },
    { "AR",             "0x%02X",       "",         NUMBER,         true,       "Состояние сигнализации" },
    { "BMV",            "%s",           "",         STRING,         false,      "Модель монитора" },
    { "FW",             "%s",           "",         STRING,         false,      "Версия прошивки" },
    { "H1",             "%5.2f ",       "Ah",       FLOAT,          true,       "Самый глубокий разряд" },
    { "H2",             "%5.2f ",       "Ah",       FLOAT,          true,       "Глубина последнего разряда" },
    { "H3",             "%5.2f ",       "Ah",       FLOAT,          true,       "Глубина среднего разряда" },
    { "H4",             "%3d",          "",         NUMBER,         true,       "Число циклов разряд/заряд" },
    { "H5",             "%3d",          "",         NUMBER,         true,       "Число полных разрядов" },
    { "H6",             "%5.2f ",       "Ah",       FLOAT,          true,       "Число Ah полученных от АКБ" },
    { "H7",             "%5.2f ",       "V",        FLOAT,          true,       "Минимальное напряжение АКБ" },
    { "H8",             "%5.2f ",       "V",        FLOAT,          true,       "Максимальное напряжение АКБ" },
    { "H9",             "%3d",          "",         NUMBER,         true,       "Число дней от полного заряда" },
    { "H10",            "%3d",          "",         NUMBER,         false,      "Автомат. синхронизаций BMV" },
    { "H11",            "%3d",          "",         NUMBER,         true,       "Сигналов низкого напряжения" },
    { "H12",            "%3d",          "",         NUMBER,         false,      "Сигналов высокого напряжения" },
    { "LINK",           "%s",           "",         BOOL1,          true,       "Связь установлена" },
    { NULL,             NULL,           NULL,       NOTYPE,         true,       NULL }
 };

//*************************************************************************************************
// Параметры солнечных панелей
//*************************************************************************************************
static const DevParam DevicePv[] = {
    { "PV",             "%s",           "",         BOOL2,          true,       "Солнечные панели" },
    { "PVMODE",         "%s",           "",         BOOL5,          true,       "Режим панелей" },
    { NULL,             NULL,           NULL,       NOTYPE,         false,      NULL }
 };

//*************************************************************************************************
// Параметры трекера
//*************************************************************************************************
static const DevParam DeviceTracker[] = {
    { "TRCLINK",        "%s",           "",         BOOL1,          false,      "Связь с контроллером" },
    { "PWRTRC",         "%s",           "",         BOOL2,          true,       "Питание трекера" },
    { "PWRBRK",         "%s",           "",         BOOL_BRK,       true,       "Защита питания трекера" },
    { "PWRACT",         "%s",           "",         BOOL2,          true,       "Питание актуаторов" },
    { "MODE",           "%s",           "",         TRAC_MODE,      true,       "Режим работы" },
    { "MODE2",          "%s",           "",         TRAC_MODE2,     false,      "Режим работы" },
    { "FUSE",           "%s",           "",         BOOL_FUSE,      true,       "Предохранитель трекера" },
    { "SENSOR",         "%s",           "",         STRING,         true,       "Сенсор" },
    { "VERT",           "%s ",          "мм/°",     STRING,         true,       "Вертик.положение" },
    { "HORZ",           "%s ",          "мм/°",     STRING,         true,       "Гориз.положение" },
    { "VEEP",           "%5d ",         "имп",      NUMBER,         true,       "Вертик.положение EEPROM" },
    { "HEEP",           "%5d ",         "имп",      NUMBER,         true,       "Гориз.положение EEPROM" },
    { "LIMSW",          "%s",           "",         STRING,         true,       "Концев.выкл." },
    { "TIMEON",         "%s ",          "ч:м:с",    TIME_FULL,      true,       "Время работы" },
    { NULL,             NULL,           NULL,       NOTYPE,         false,      NULL }
 };

//режим трекера
static char * const trc_mode[] = {
    "Нет связи",
    "Командный",
    "Автомат  ",
    "Ветер    "
 };

static char * const trc_mode2[] = {
    "No connection",
    "Command",
    "Auto",
    "Wind",
 };

//*************************************************************************************************
// Параметры положения солнца
//*************************************************************************************************
static const DevParam DeviceSunPos[] = {
    { "SUN",            "%s ",          "час:мин",  TIME_SUN,       true,       "Восход" },
    { "RISE",           "%s ",          "час:мин",  TIME_SUN,       true,       "Заход" },
    { "TZONE",          "%2d ",         "",         NUMBER,         false,      "Часовой пояс" },
    { "LATIT",          "%.6f ",        "с.ш.",     FLOAT,          false,      "Широта" },
    { "LONGI",          "%.6f ",        "в.д.",     FLOAT,          false,      "Долгота" },
    { "ELEV",           "%4d ",         "м",        NUMBER,         false,      "Высота" },
    { "PRESS",          "%3d ",         "мм.рс.",   NUMBER,         false,      "Давление" },
    { "TEMP",           "%3.2f ",       "С",        FLOAT,          false,      "Температура" },
    { "SLOPE",          "%3.2f ",       "°",        FLOAT,          false,      "Наклон поверхности" },
    { "AZMR",           "%3.2f ",       "°",        FLOAT,          false,      "Вращ. поверх. азимута" },
    { "ZENIT",          "%3.2f ",       "°",        FLOAT,          true,       "Зенитный угол" },
    { "AZIMUT",         "%3.2f ",       "°",        FLOAT,          true,       "Азимутальный угол" },
    { "DURAT",          "%s ",          "час:мин",  TIME_SUN,       true,       "Продолжительность дня" },
    { "SUN1",           "%s",           "",         TIME_SUN,       false,      "Восход:" },
    { "RISE1",          "%s",           "",         TIME_SUN,       false,      "Заход:" },
    { "DURAT1",         "%s",           "",         TIME_SUN,       false,      "Продолжительность дня:" },
    { "ZENIT1",         "%3.1f° ",      "",         FLOAT,          false,      "Зенитный угол:" },
    { "AZIMUT1",        "%3.1f° ",      "",         FLOAT,          false,      "Азимутальный угол:" },
    { "SPAERR",         "%s",           "",         SPA_ERRDS,      false,      "Проверка данных:" },
    { NULL,             NULL,           NULL,       NOTYPE,         false,      NULL }
 };

//*************************************************************************************************
//расшировка ошибок проверки исходных данных для расчета
//*************************************************************************************************
static char * const spa_error[] = {
    "OK",
    "ошибка в годе (< -2000 * > 6000)",
    "ошибка в месяце (< 1 * > 12)",
    "ошибка в дне (< 1 * > 31)",
    "ошибка в часах (< 0 * > 24)",
    "ошибка в минутах (< 0 * > 59)",
    "ошибка в секундах (< 0 * >= 60)",
    "ошибка в разнице между временем вращения земли и земным временем (> 8000)",
    "ошибка в часовом поясе (> 18)",
    "ошибка в долготе (> 180)",
    "ошибка в широте (> 90)",
    "ошибка в высоте наблюдения (< -6500000)",
    "ошибка в давлении (< 0 * > 5000)",
    "ошибка в температуре (<= -273 * > 6000)",
    "ошибка в наклоне к поверхности (> 360)",
    "вращение поверхности (> 360)",
    "ошибка в параметре атмосферной рефракции (> 5)",
    "ошибка в разнице секунд между UTC и UT (<= -1 * >= 1)"
 };

//*************************************************************************************************
// Параметры голосового информатора
//*************************************************************************************************
static const DevParam DeviceVoice[] = {
    { "VLINK",          "%s",           "",         BOOL1,          true,       "Подключен" },
    { "VSTAT",          "0x%04X",       "",         NUMBER,         true,       "Статус" },
    { "VVOL",           "%d",           "",         NUMBER,         true,       "Уровень громкости" },
    { NULL,             NULL,           NULL,       NOTYPE,         false,      NULL }
 };

//*************************************************************************************************
// Параметры для отображения времени/даты
//*************************************************************************************************
static const DevParam DeviceRtc[] = {
    { "DATE",           "%s",           "",         STRING,         false,      "" },
    { "TIME",           "%s",           "",         STRING,         false,      "" },
    { "FULL",           "%s",           "",         STRING,         false,      "" },
    { "FULLDW",         "%s",           "",         STRING,         false,      "" },
    { NULL,             NULL,           NULL,       NOTYPE,         false,      NULL }
 };
 
//*************************************************************************************************
// Параметры инвертора
//*************************************************************************************************
static const DevParam DeviceInv[] = {
    { "EQV",            "%4.1f ",       "V",        FLOAT,          false,       "Напряжение выравнивания" },
    { "FLV",            "%4.1f ",       "V",        FLOAT,          false,       "Напряжение поддержания" },
    { "ALRV",           "%4.1f ",       "V",        FLOAT,          false,       "Пониженное напряжение" },
    { "SHDNV",          "%4.1f ",       "V",        FLOAT,          false,       "Напряжение выключения" },
    { "VENDOR",         "%s",           "",         STRING,         false,       "Производитель" },
    { "VERS",           "%s",           "",         STRING,         false,       "Версия" },
    { "MODEL",          "%s",           "",         STRING,         false,       "Модель" },
    { "ACOUT",          "%3d ",         "V",        NUMBER,         true,        "Напряжение на выходе" },
    { "ACPWR",          "%3d ",         "%%",       NUMBER,         false,       "Уровень выходной мощности" },
    { "DCIN",           "%4.1f ",       "V",        FLOAT,          true,        "Напряжение АКБ" },
    { "BATPRC",         "%3d ",         "%%",       FLOAT,          false,       "Уровень заряда АКБ" },
    { "TEMP",           "%4.1f ",       "C",        FLOAT,          true,        "Температура инвертора" },
    { "UNUSED",         "%d",           "",         NUMBER,         false,       "" },
    { "FREQ",           "%4.1f ",       "Hz",       FLOAT,          false,       "Частота на выходе" },
    { "TTG",            "%3d ",         "мин",      NUMBER,         false,       "Прогнозируемое время работы" },
    { "PERC",           "%3d ",         "%%",       NUMBER,         true,        "Потребляемая мощность" },
    { "WATT",           "%4d ",         "W",        NUMBER,         true,        "Мощность на выходе" },
    { "STAT",           "0x%06X",       "",         NUMBER,         false,       "Cостояние инвертора" },
    { "CONN",           "%s",           "",         BOOL1,          true,        "Инвертор подключен" },
    { "MODE",           "%s",           "",         INVR_MODE,      true,        "Режим инвертора" },
    { "INV_ERR",        "%s",           "",         INVR_ERROR,     true,        "Ошибки инвертора" },
    { "CTRL_ERR",       "%s",           "",         INVR_ERR_CTRL,  true,        "Ошибки управления инвертором" },
    { NULL,             NULL,           NULL,       NOTYPE,         false,       NULL }
 };

//описание режима работы
static char * const inv_mode[] = {
    "        ",
    "Выключен",
    "Включен ",
    "Дежурный",
    "Вык дист" 
 };

//описание ошибок инвертора
static char * const inv_err_descr[] = {
    "             ",                                                //TS_ERR_OK
    "EEPROM",                                                       //TS_ERR_EEPROM
    "BAT",                                                          //TS_ERR_BAT
    "BAT LOW",                                                      //TS_ERR_BATLOW
    "OVR-100%%~115%%",                                              //TS_ERR_OVR100
    "OVR-115%%~150%%",                                              //TS_ERR_OVR115
    "OVR>150%",                                                     //TS_ERR_OVR150
    "OVERHEAT",                                                     //TS_ERR_OVRHEAT
    "ERROR",                                                        //TS_ERR_ERROR
 };

//расшифровка ошибок инвертора
static char * const inv_err_ctrl[] = {
    "",                                                             //INV_ERR_CTRL_OK
    "Параметры вызова функции заданы неправильно",                  //INV_ERR_CTRL_PARAM
    "АКБ не подключена",                                            //INV_ERR_CTRL_NOBAT
    "Выключение инвертор от АКБ блокировано, есть нагрузка",        //INV_ERR_CTRL_POWER
    "Нет сигнала подтверждения вкл/выкл контактора",                //INV_ERR_CTRL_NOSIGNAL
    "Нет сигнала включения схемы управления контакторами",          //INV_ERR_CTRL_NOCTRL
    "Нет ответа от инвертора",                                      //INV_ERR_CTRL_ANSWER
    "нет подтверждения программного выключения инвертора",          //INV_ERR_CTRL_NO_OFF
    "Выключена клавиша питания"                                     //INV_ERR_CTRL_POWER_OFF
   };

//*************************************************************************************************
// Параметры генератора
//*************************************************************************************************
static const DevParam DeviceGen[] = {
    { "REMOTE",         "%s",           "",         BOOL1,          false,      "Подключен удаленно", },
    { "CONN",           "%s",           "",         BOOL1,          true,       "Подключен", },
    { "MODE",           "%s",           "",         GEN_MODE,       true,       "Режим работы генератора", },
    { "STAT",           "%s",           "",         GEN_STAT,       true,       "Состояние генератора", },
    { "TM_RUN",         "%s ",          "ч:м:с",    TIME_FULL,      true,       "Время работы", },
    { "TM_END",         "%s ",          "ч:м:с",    TIME_FULL,      true,       "Время работы до выключения", },
    { "START_STOP",     "%s ",          "ч:м:с",    TIME_FULL,      true,       "Время до запуска/выключения", },
    { "SLEEP",          "%s ",          "ч:м:с",    TIME_FULL,      true,       "Время отдыха", },
    { "AUTO",           "%s",           "",         BOOL7,          true,       "Режим работы автоматики", },
    { "GENALT",         "%s",           "",         BOOL8,          true,       "Напряжения генератора на АВР" },
    { "GEN_ERR",        "%s",           "",         GEN_ERROR,      true,       "Ошибки генератора" },
    { NULL,             NULL,           NULL,       NOTYPE,         false,      NULL }
 };

//режимы работы генератора
static char * const gen_mode[] = {
    "            ",                                                 //GEN_MODE_NULL
    "Запуск      ",                                                 //GEN_MODE_START
    "Запущен     ",                                                 //GEN_MODE_RUN
    "Отдых       ",                                                 //GEN_MODE_SLEEP
    "Выключен    ",                                                 //GEN_MODE_OFF
    "Тестирование"                                                  //GEN_MODE_TEST
 };

//состояние генератора
static char * const gen_stat[] = {                  
    "                  ",                                           //GEN_STAT_NULL
    "Не подключен      ",                                           //GEN_STAT_NO_CONN
    "Разряд батареи    ",                                           //GEN_STAT_BAT_LOW
    "Уровень топлива   ",                                           //GEN_STAT_FUEL_LOW
    "Уровень масла     ",                                           //GEN_STAT_OIL_LOW
    "Перегрузка инверт.",                                           //GEN_STAT_OVERLOAD
    "Автозапуск прерван",                                           //GEN_STAT_AUTO_BREAK
    "Ген. выключился   ",                                           //GEN_STAT_GEN_OFF
    "Выкл. принудит-но ",                                           //GEN_STAT_MANUAL_BREAK
    "Запуск прерван    ",                                           //GEN_STAT_START_BREAK
    "Запуск %02u из %02u   ",                                       //GEN_STAT_STEP_START
    "Время раб. истекло",                                           //GEN_STAT_TIME_OUT
    "Тест завершен     ",                                           //GEN_STAT_TEST_END
    "Ошибка запуска    "                                            //GEN_STAT_START_ERROR
 };

//расшифровка ошибок генератора
static char * const gen_err_descr[] = {                 
    "",                                                             //GEN_ERR_OK
    "Генератор не подключен",                                       //GEN_ERR_NO_CONNECT
    "Низкий заряд АКБ генератора",                                  //GEN_ERR_BATLOW
    "Низкий уровень топлива",                                       //GEN_ERR_FUEL
    "Низкий уровень масла",                                         //GEN_ERR_OIL
    "Перегрузка инвертора генератора",                              //GEN_ERR_OVR
    "Генератор выключился",                                         //GEN_ERR_BREAK
    "Генератор не запустился"                                       //GEN_ERR_START
 };

//*************************************************************************************************
// Параметры состояния связи с модулем HMI
//*************************************************************************************************
static const DevParam DeviceHmi[] = {
    { "LINK",                   "%s",       "",          BOOL1,      false,     "Связь с HMI:" },
    { "SEND",                   "%u",       "",          NUMBER,     false,     "Передано пакетов" },
    { "ERROR",                  "%u",       "",          NUMBER,     false,     "Ошибок передачи" },
    { "ERROR_TX",               "%u",       "",          NUMBER,     false,     "Счетчик ошибок передачи" },
    { "ERROR_RX",               "%u",       "",          NUMBER,     false,     "Счетчик ошибок приема" },
    { NULL,                     NULL,       NULL,        NOTYPE,     false,     NULL }
 };

//*************************************************************************************************
// Параметры настроек
//*************************************************************************************************
static const DevParam Config[] = {
    { "CFG_SCR_FILE",           "%s",       "",          STRING,     false,     "Файл экрана" },
    { "CFG_JOB_FILE",           "%s",       "",          STRING,     false,     "Файл заданий (резервный/смешанный режим)" },
    { "CFG_JOB_TEST",           "%s",       "",          STRING,     false,     "Файл заданий (тестовый режим)" },
    { "CFG_MODE_SYS",           "%s",       "",          SYS_MODE,   false,     "Режим работы системы" },
    { "CFG_MODE_LOGGING",       "%s",       "",          MODE_DIR,   false,     "Режим логирования файлов" },
    { "CFG_LAST_CHARGE",        "%s",       "",          SDATE,      false,     "Дата последнего включения подзарядки от основной сети" },
    { "CFG_LOG_ENABLE_PV",      "%s",       "",          BOOL1,      false,     "Логирование команд управление солнечными панелями" },
    { "CFG_LOG_ENABLE_CHARGE",  "%s",       "",          BOOL1,      false,     "Логирование команд зарядного уст-ва" },
    { "CFG_LOG_ENABLE_MPPT",    "%s",       "",          BOOL1,      false,     "Логирование команд/данных вкл/выкл солнечных панелей" },
    { "CFG_LOG_ENABLE_TS",      "%s",       "",          BOOL1,      false,     "Логирование команд/данных управление инверторов" },
    { "CFG_LOG_ENABLE_BMON",    "%s",       "",          BOOL1,      false,     "Логирование данных монитора батареи" },
    { "CFG_LOG_ENABLE_GEN",     "%s",       "",          BOOL1,      false,     "Логирование команд генератора" },
    { "CFG_LOG_ENABLE_ALT",     "%s",       "",          BOOL1,      false,     "Логирование команд блока АВР" },
    { "CFG_LOG_ENABLE_TRC",     "%s",       "",          BOOL1,      false,     "Логирование команд трекера" },
    { "CFG_DATLOG_UPD_PB",      "%u ",      "сек",       NUMBER,     false,     "Период записи данных зарядного уст-ва" },
    { "CFG_DATLOG_UPD_MPPT",    "%u ",      "сек",       NUMBER,     false,     "Период записи данных солнечного контроллера заряда" },
    { "CFG_DATLOG_UPD_BMON",    "%u ",      "сек",       NUMBER,     false,     "Период записи данных монитора батареи" },
    { "CFG_DATLOG_UPD_TS",      "%u ",      "сек",       NUMBER,     false,     "Период записи данных инверторов" },
    { "CFG_DATLOG_UPD_TRC",     "%u ",      "сек",       NUMBER,     false,     "Период записи данных трекера" },
    { "CFG_GEN_DELAY_START",    "%u ",      "сек",       NUMBER,     false,     "Задержка запуска генератора после отключения основной сети" },
    { "CFG_GEN_DELAY_STOP",     "%u ",      "сек",       NUMBER,     false,     "Задержка выключения генератора после восстановления основной сети" },
    { "CFG_GEN_DELAY_CHK_RUN",  "%u ",      "сек",       NUMBER,     false,     "Ожидание сигнала запуска генератора" },
    { "CFG_GEN_BEFORE_START",   "%u ",      "сек",       NUMBER,     false,     "Пауза между запусками" },
    { "CFG_GEN_CNT_START",      "%u ",      "",          NUMBER,     false,     "Кол-во попыток запуска генератора" },
    { "CFG_GEN_TIME_START",     "%s",       "",          TIMESTART,  false,     "Длительность запуска для каждой попытки" },
    { "CFG_GEN_TIME_RUN",       "%u ",      "сек",       NUMBER,     false,     "Максимальная продолжительность работы генератора" },
    { "CFG_GEN_TIME_SLEEP",     "%u ",      "сек",       NUMBER,     false,     "Продолжительность паузы между длительными работами" },
    { "CFG_GEN_TIME_TEST",      "%u ",      "сек",       NUMBER,     false,     "Продолжительность тестирования генератора" },
    { "CFG_GEN_AUTO_MODE",      "%s",       "",          BOOL7,      false,     "Ручной/автоматический режим запуска при отключении сети" },
    { "CFG_GEN_LAST_RUN",       "%s",       "",          SDATE,      false,     "Дата последнего включения генератора" },
    { "CFG_SPA_TIMEZONE",       "%u",       "",          NUMBER,     false,     "Часовой пояс" },
    { "CFG_SPA_LATITUDE",       "%.6f",     "",          FLOAT,      false,     "Широта места" },
    { "CFG_SPA_LONGITUDE",      "%.6f",     "",          FLOAT,      false,     "Долгота места" },
    { "CFG_SPA_ELEVATION",      "%u ",      "м",         NUMBER,     false,     "Высота места" },
    { "CFG_SPA_PRESSURE",       "%u ",      "мм.рт.ст.", NUMBER,     false,     "Среднегодовое местное давление" },
    { "CFG_SPA_TEMPERATURE",    "%d ",      "C°",        NUMBER,     false,     "Среднегодовая местная температура" },
    { "CFG_SPA_SLOPE",          "%u ",      "°",         NUMBER,     false,     "Наклон поверхности" },
    { "CFG_SPA_AZM_ROTATION",   "%u ",      "°",         NUMBER,     false,     "Вращение поверхности азимута" },
    { "CFG_PB_CURRENT_STOP",    "%u ",      "A",         NUMBER,     false,     "Минимальный ток заряда для выключения PB-1000-224" },
    { "CFG_DELAY_START_INV",    "%u ",      "сек",       NUMBER,     false,     "Задержка вкл инверторов при отключении основной сети" },
    { "CFG_DELAY_STOP_INV",     "%u ",      "сек",       NUMBER,     false,     "Задержки выкл инверторов после восстановлении основной сети" },
    { NULL,                     NULL,       NULL,        NOTYPE,     false,     NULL }
 };

//описание режима работы системы
static char * const sys_mode[] = {
    "Тестовый",
    "Резервный",
    "Смешанный"
 };

//параметры для проверки допустимых значений настроек
static const ConfigCheck ConfCheck[] = {
    { CFG_SCR_FILE,             true,   STRING,     sizeof( config.scr_file ),  0,      sizeof( config.scr_file ) },
    { CFG_JOB_FILE,             true,   STRING,     sizeof( config.job_file ),  0,      sizeof( config.job_file ) },
    { CFG_JOB_TEST,             true,   STRING,     sizeof( config.job_test ),  0,      sizeof( config.job_test ) },
    { CFG_MODE_SYS,             true,   NUMBER,     sizeof( uint8_t ),          0,      2 },
    { CFG_MODE_LOGGING,         true,   NUMBER,     sizeof( uint8_t ),          0,      1 },
    { CFG_LAST_CHARGE,          true,   DATES,      sizeof( DATE ),             0,      0 },
    { CFG_LOG_ENABLE_PV,        true,   NUMBER,     sizeof( uint8_t ),          0,      1 },
    { CFG_LOG_ENABLE_CHARGE,    true,   NUMBER,     sizeof( uint8_t ),          0,      1 },
    { CFG_LOG_ENABLE_MPPT,      true,   NUMBER,     sizeof( uint8_t ),          0,      1 },
    { CFG_LOG_ENABLE_INV,       true,   NUMBER,     sizeof( uint8_t ),          0,      1 },
    { CFG_LOG_ENABLE_BMON,      true,   NUMBER,     sizeof( uint8_t ),          0,      1 },
    { CFG_LOG_ENABLE_GEN,       true,   NUMBER,     sizeof( uint8_t ),          0,      1 },
    { CFG_LOG_ENABLE_ALT,       true,   NUMBER,     sizeof( uint8_t ),          0,      1 },
    { CFG_LOG_ENABLE_TRC,       true,   NUMBER,     sizeof( uint8_t ),          0,      1 },
    { CFG_DATLOG_UPD_CHARGE,    true,   NUMBER,     sizeof( uint16_t ),         1,      900 },
    { CFG_DATLOG_UPD_MPPT,      true,   NUMBER,     sizeof( uint16_t ),         1,      900 },
    { CFG_DATLOG_UPD_BMON,      true,   NUMBER,     sizeof( uint16_t ),         1,      900 },
    { CFG_DATLOG_UPD_INV,       true,   NUMBER,     sizeof( uint16_t ),         1,      900 },
    { CFG_DATLOG_UPD_TRC,       true,   NUMBER,     sizeof( uint16_t ),         1,      900 },
    { CFG_GEN_DELAY_START,      true,   NUMBER,     sizeof( uint16_t ),         15,     7200 },
    { CFG_GEN_DELAY_STOP,       true,   NUMBER,     sizeof( uint16_t ),         15,     7200 },
    { CFG_GEN_DELAY_CHK_RUN,    true,   NUMBER,     sizeof( uint16_t ),         1,      6 },
    { CFG_GEN_BEFORE_START,     true,   NUMBER,     sizeof( uint16_t ),         1,      15 },
    { CFG_GEN_CNT_START,        true,   NUMBER,     sizeof( uint8_t ),          1,      8 },
    { CFG_GEN_TIME_START,       true,   STRINT,     sizeof( uint8_t ) * 8,      1,      6 },
    { CFG_GEN_TIME_RUN,         true,   NUMBER,     sizeof( uint16_t),          5,      18000 },
    #ifdef DEBUG_VERSION
    { CFG_GEN_TIME_SLEEP,       true,   NUMBER,     sizeof( uint16_t ),         60,     7200 },
    #else
    { CFG_GEN_TIME_SLEEP,       true,   NUMBER,     sizeof( uint16_t ),         1800,   7200 },
    #endif
    { CFG_GEN_TIME_TEST,        true,   NUMBER,     sizeof( uint16_t ),         60,     300 },
    { CFG_GEN_AUTO_MODE,        true,   NUMBER,     sizeof( uint8_t ),          0,      1 },
    { CFG_GEN_LAST_RUN,         true,   DATES,      sizeof( DATE ),             0,      0 },
    { CFG_SPA_TIMEZONE,         true,   NUMBER,     sizeof( uint8_t ),          0,      18 },
    { CFG_SPA_LATITUDE,         true,   DOUBLE,     sizeof( double ),           0,      90 },
    { CFG_SPA_LONGITUDE,        true,   DOUBLE,     sizeof( double ),           0,      180 },
    { CFG_SPA_ELEVATION,        true,   NUMSIGN,    sizeof( uint16_t ),         -32768, 32768 },
    { CFG_SPA_PRESSURE,         true,   NUMBER,     sizeof( uint16_t ),         0,      5000 },
    { CFG_SPA_TEMPERATURE,      true,   NUMBER,     sizeof( uint8_t ),          0,      255 },
    { CFG_SPA_SLOPE,            true,   NUMBER,     sizeof( uint16_t ),         0,      360 },
    { CFG_SPA_AZM_ROTATION,     true,   NUMBER,     sizeof( uint16_t ),         0,      360 },
    { CFG_PB_CURRENT_STOP,      true,   NUMBER,     sizeof( uint8_t ),          3,      10 },
    { CFG_DELAY_START_INV,      true,   NUMBER,     sizeof( uint16_t ),         15,     7200 },
    { CFG_DELAY_STOP_INV,       true,   NUMBER,     sizeof( uint16_t ),         15,     7200 }
 };

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static char format[BUFFER_PARAM];           //буфер для записи описания параметра + формат + единицы измерения
static char result[BUFFER_PARAM];           //буфер для записи описания параметра + значение + единицы измерения
static uint8_t par_cnt[32][2] = { 0 };
static char time_str1[12], time_str2[12], time_str3[12];
static char time_str4[12], time_str5[12], time_str6[12], date_str[20];

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static uint8_t ParamCnt( const DevParam *dp, CountType type );

static char *TimeValue( uint16_t value );
static char *Time1Value( uint32_t value );
static char *Time2Value( uint16_t value );
static char *Time3Value( uint32_t value );
static char *Time4Value( float value );
static char *DescBool1( uint32_t value );
static char *DescBool2( uint32_t value );
static char *DescBool3( uint32_t value );
static char *DescBool4( uint32_t value );
static char *DescBool5( uint32_t value );
static char *DescBool6( uint32_t value );
static char *DescBool7( uint32_t value );
static char *DescBool8( uint32_t value );
static char *DescBool9( uint32_t value );
static char *PwrSource( uint32_t value );
static char *MpptMode( uint32_t value );
static char *ChrgeMode( uint32_t value );
static char *GenModeDesc( uint32_t value );
static char *GenStatDesc( uint32_t value );
static char *InvModeDesc( uint32_t value );
static char *TracModeDesc( uint32_t value );
static char *TracModeDesc2( uint32_t value );
static char *SpaErrorDesc( uint32_t value );
static char *SysModeDesc( uint32_t value );
static char *DirModeDesc( uint32_t value );
static char *DateToStr( DATE date );
static char *TimeStart( uint8_t *ptr );
static TrackerStat TrackerMode( uint16_t stat );

//*************************************************************************************************
// Возвращает ID устройства по имени уст-ва, имена уст-в хранятся в: dev_name[]
// char *name           - имя уст-ва
// return = ID_DEV_NULL - уст-во не обнаружено или ID уст-ва
//*************************************************************************************************
Device DevGetInd( char *name ) {

    Device ind;
    
    for ( ind = ID_DEV_NULL; ind < SIZE_ARRAY( dev_name ); ind++ ) {
        //найдем параметр по имени
        if ( !strcasecmp( name, dev_name[ind] ) )
            return ind;
       }
    return ID_DEV_NULL;
 }

//*************************************************************************************************
// Возвращает имя уст-ва в т.ч. для ID_CONFIG возвращает имя параметра
// Device dev - ID уст-ва
// return     - указатель на имя уст-ва
//*************************************************************************************************
char *DevName( Device dev ) {

    if ( dev < SIZE_ARRAY( dev_name ) )
        return dev_name[dev];
    else return NULL;
 }

//*************************************************************************************************
// Возвращает имя параметра настройки для ID_CONFIG
// ConfigParam - ID параметра
// return      - указатель на имя уст-ва
//*************************************************************************************************
char *ConfigName( ConfigParam param ) {

    if ( param < SIZE_ARRAY( Config ) )
        return Config[param].name;
    return NULL;
 }

//*************************************************************************************************
// Возвращает кол-во доступных параметров уст-ва для указанного ID уст-ва
// Device dev     - ID уст-ва
// CountType type - тип подсчета параметров
// return         - кол-во доступных параметров уст-ва
//*************************************************************************************************
uint8_t DevParamCnt( Device dev, CountType type ) {

    if ( dev >= SIZE_ARRAY( dev_name ) )
        return 0;
    if ( par_cnt[dev][CNT_FULL] )
        return par_cnt[dev][type];
    //расчет кол-ва параметров выполняется один раз при первом вызове
    par_cnt[dev][CNT_FULL] = ParamCnt( DevParamPtr( dev ), CNT_FULL );
    par_cnt[dev][CNT_HMI] = ParamCnt( DevParamPtr( dev ), CNT_HMI );
    return par_cnt[dev][type];
 }

//*************************************************************************************************
// Возвращает относительный ID параметра уст-ва
// Device dev     - ID уст-ва
// uint32_t param - относительный ID параметра (вывод в формах модуля HMI)
// return         - абсолютный ID параметра
//*************************************************************************************************
uint8_t DevParamRelat( Device dev, uint32_t param ) {

    uint8_t ind, rel;
    const DevParam *dp;
    
    if ( dev >= SIZE_ARRAY( dev_name ) )
        return 0;
    dp = DevParamPtr( dev );
    for ( ind = 0, rel = 0; dp[ind].comment != NULL; ind++ ) {
        if ( dp[ind].view_hmi == true && rel == param )
            return ind;
        if ( dp[ind].view_hmi == true )
            rel++;
       }
    return 0;
 }

//*************************************************************************************************
// Возвращает указатель на массив параметров уст-ва для указанного ID уст-ва
// Device dev      - ID уст-ва
// return DevParam - указатель на массив параметров уст-ва
//*************************************************************************************************
const DevParam *DevParamPtr( Device dev ) {

    if ( dev == ID_DEV_ALT )
        return DeviceAlt;
    if ( dev == ID_DEV_MPPT )
        return DeviceMppt;
    if ( dev == ID_DEV_CHARGER )
        return DeviceCharger;
    if ( dev == ID_DEV_BATMON )
        return DeviceBatmon;
    if ( dev == ID_DEV_PV )
        return DevicePv;
    if ( dev == ID_DEV_VOICE )
        return DeviceVoice;
    if ( dev == ID_DEV_SPA )
        return DeviceSunPos;
    if ( dev == ID_DEV_TRC )
        return DeviceTracker;
    if ( dev == ID_DEV_RTC )
        return DeviceRtc;
    if ( dev == ID_DEV_GEN )
        return DeviceGen;
    if ( dev == ID_DEV_INV1 )
        return DeviceInv;
    if ( dev == ID_DEV_INV2 )
        return DeviceInv;
    if ( dev == ID_DEV_HMI )
        return DeviceHmi;
    if ( dev == ID_CONFIG )
        return Config;
    return NULL;
 }

//*************************************************************************************************
// Возвращает кол-во доступных параметров уст-ва
// const DevParam *dp - указатель на структуру описания параметров
// CountType type     - тип подсчета параметров: полный/выборочный
// return = 0         - параметров нет
//        > 0         - кол-во параметров
//*************************************************************************************************
static uint8_t ParamCnt( const DevParam *dp, CountType type ) {

    uint8_t ind1, ind2 = 0;

    if ( type == CNT_FULL )
        for ( ind1 = 0; dp[ind1].comment != NULL; ind1++ );
    else {
        for ( ind1 = 0; dp[ind1].comment != NULL; ind1++ )
            if ( dp[ind1].view_hmi == true )
                ind2++;
       }
    if ( type == CNT_FULL )
        return ind1;
    else return ind2;
 }

//*************************************************************************************************
// Возвращает индекс параметра по имени 
// Device dev - ID уст-ва
// char *name - имя параметра
// return = 0 - параметра нет
//        > 0 - индекс параметра, фактическое значение индекса: return-1
//*************************************************************************************************
uint8_t ParamGetInd( Device dev, char *name ) {

    uint8_t ind;
    const DevParam *dp;

    dp = DevParamPtr( dev );
    for ( ind = 0; ind < DevParamCnt( dev, CNT_FULL ); ind++ ) {
        if ( !strcasecmp( dp[ind].name, name ) )
            return ++ind;
       }
    return 0;
 } 

//*************************************************************************************************
// Возвращает строку с именем параметра уст-ва по индексу параметра
// Device dev     - ID уст-ва
// uint8_t param  - ID параметра
//*************************************************************************************************
char *ParamGetName( Device dev, uint32_t param ) {

    const DevParam *dp;

    dp = DevParamPtr( dev );
    if ( param >= DevParamCnt( dev, CNT_FULL ) )
        return NULL;
    else return dp[param].name;
 }
 
//*************************************************************************************************
// Возвращает значение параметра уст-ва
// Device dev     - ID уст-ва
// uint8_t param  - ID параметра
//*************************************************************************************************
ValueParam ParamGetVal( Device dev, uint32_t param ) {

    ValueParam value;
    
    value.uint32 = 0;
    //значение параметра
    if ( dev == ID_DEV_PORTS )
        value = PortsGetValue( (ParamPort)param );
    if ( dev == ID_DEV_ALT )
        value = AltGetValue( (ParamAlt)param );
    if ( dev == ID_DEV_MPPT )
        value = MpptGetValue( (ParamMppt)param );
    if ( dev == ID_DEV_CHARGER )
        value = ChargeGetValue( (ParamCharger)param );
    if ( dev == ID_DEV_BATMON )
        value = BatmonGetValue( (ParamBatMon)param );
    if ( dev == ID_DEV_PV )
        value = PvGetValue( (ParamPv)param );
    if ( dev == ID_DEV_VOICE )
        value = VoiceGetValue( (ParamVoice)param );
    if ( dev == ID_DEV_SPA )
        value = SpaGetValue( (ParamSunPos)param );
    if ( dev == ID_DEV_TRC )
        value = TrackerGetValue( (ParamTracker)param );
    if ( dev == ID_DEV_RTC )
        value = RtcGetValue( (ParamRtc)param );
    if ( dev == ID_DEV_GEN )
        value = GenGetValue( (ParamGen)param );
    if ( dev == ID_DEV_INV1 )
        value = InvGetValue( ID_DEV_INV1, (ParamInv)param );
    if ( dev == ID_DEV_INV2 )
        value = InvGetValue( ID_DEV_INV2, (ParamInv)param );
    if ( dev == ID_CONFIG )
        value = ConfigValue( (ConfigParam)param );
    if ( dev == ID_DEV_HMI ) {
        #ifdef CONFIG_CONTROL
        value = HmiGetValue( (ParamHmi)param );
        #endif
        #ifdef CONFIG_HMIDEV
        value = LinkGetValue( (ParamHmi)param );
        #endif
       }
    if ( dev == ID_CONFIG )
        value = ConfigValue( (ConfigParam)param );
    return value;
 }

//*************************************************************************************************
// Возвращает отформатированную строку с значением параметра 
// параметра уст-ва в соответствии с маской отображения
// Device dev     - ID уст-ва
// uint8_t param  - ID параметра
// ParamMode mode - маска отображения
//*************************************************************************************************
char *ParamGetForm( Device dev, uint32_t param, ParamMode mode ) {

    ValueParam value;
    const DevParam *dpar;
    
    if ( !mode )
        return NULL;
    //указатель на список параметров
    dpar = DevParamPtr( dev );
    if ( dpar == NULL )
        return NULL;
    //проверка на превышение кол-ва параметров
    if ( param >= DevParamCnt( dev, CNT_FULL ) )
        return NULL;
    memset( format, 0x00, sizeof( format ) );
    memset( result, 0x00, sizeof( result ) );
    //номер параметра
    if ( mode & PARAM_NUMB )
        sprintf( format, "%2u ", param );
    //описание параметра
    if ( mode & PARAM_DESC ) {
        if ( strlen( dpar[param].comment ) )
            strcat( format, dpar[param].comment );
        else return NULL;
       }
    //выравнивание справа символами "..."
    if ( mode & PARAM_DOT ) {
        if ( dev == ID_CONFIG )
            AddDot( format, PARAM2_ALIGNMENT );
        else AddDot( format, PARAM1_ALIGNMENT );
       }
    //формат вывода значения параметра
    if ( mode & PARAM_VALUE )
        strcat( format, dpar[param].frm );
    //единицы измерения параметра
    if ( mode & PARAM_UNIT && strlen( dpar[param].units ) )
        strcat( format, dpar[param].units );
    //значение параметра
    value = ParamGetVal( dev, param );
    //вывод логических значений
    if ( mode & PARAM_VALUE && dpar[param].subtype == BOOL1 )
        sprintf( result, format, DescBool1( value.uint8 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == BOOL2 )
        sprintf( result, format, DescBool2( value.uint8 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == BOOL3 )
        sprintf( result, format, DescBool3( value.uint8 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == BOOL4 )
        sprintf( result, format, DescBool4( value.uint8 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == BOOL5 )
        sprintf( result, format, DescBool5( value.uint8 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == BOOL_BRK )
        sprintf( result, format, DescBool6( value.uint8 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == BOOL7 )
        sprintf( result, format, DescBool7( value.uint8 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == BOOL8 )
        sprintf( result, format, DescBool8( value.uint8 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == BOOL_FUSE )
        sprintf( result, format, DescBool8( value.uint8 ) );
    //вывод стандартных параметров
    if ( mode & PARAM_VALUE && dpar[param].subtype == NUMBER )
        sprintf( result, format, value.uint32 );
    if ( mode & PARAM_VALUE && dpar[param].subtype == FLOAT )
        sprintf( result, format, value.flt );
    if ( mode & PARAM_VALUE && dpar[param].subtype == STRING )
        sprintf( result, format, value.ptr );
    //вывод времени
    if ( mode & PARAM_VALUE && dpar[param].subtype == TIME_FULL )
        sprintf( result, format, TimeValue( value.uint16 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == TIME_1SHORT )
        sprintf( result, format, Time1Value( value.uint32 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == TIME_2SHORT )
        sprintf( result, format, Time2Value( value.uint16 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == TIME_3SHORT )
        sprintf( result, format, Time3Value( value.uint32 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == TIME_SUN )
        sprintf( result, format, Time4Value( value.flt ) );
    //вывод статусов и режимов
    if ( mode & PARAM_VALUE && dpar[param].subtype == PWR_STAT )
        sprintf( result, format, PwrSource( value.uint32 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == MPPT_MODE )
        sprintf( result, format, MpptMode( value.uint32 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == CHRGE_MODE )
        sprintf( result, format, ChrgeMode( value.uint32 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == GEN_MODE )
        sprintf( result, format, GenModeDesc( value.uint32 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == GEN_ERROR )
        sprintf( result, format, ErrorDescr( dev, value.uint8, 0 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == GEN_STAT ) {
        if ( value.uint32 == GEN_STAT_STEP_START )
            sprintf( result, GenStatDesc( value.uint32 ), ParamGetVal( ID_DEV_GEN, GEN_PAR_CYCLE1 ).uint8, ParamGetVal( ID_DEV_GEN, GEN_PAR_CYCLE2 ).uint8 );
        else sprintf( result, format, GenStatDesc( value.uint32 ) );
       }
    if ( mode & PARAM_VALUE && dpar[param].subtype == INVR_MODE )
        sprintf( result, format, InvModeDesc( value.uint32 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == INVR_ERR_CTRL )
        sprintf( result, format, ErrorDescr( dev, 0, value.uint8 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == INVR_ERROR )
        sprintf( result, format, ErrorDescr( dev, value.uint8, 0 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == TRAC_MODE )
        sprintf( result, format, TracModeDesc( value.uint32 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == TRAC_MODE2 )
        sprintf( result, format, TracModeDesc2( value.uint32 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == SPA_ERRDS )
        sprintf( result, format, SpaErrorDesc( value.uint8 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == SYS_MODE )
        sprintf( result, format, SysModeDesc( value.uint8 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == SDATE )
        sprintf( result, format, DateToStr( value.date ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == MODE_DIR )
        sprintf( result, format, DirModeDesc( value.uint32 ) );
    if ( mode & PARAM_VALUE && dpar[param].subtype == TIMESTART )
        sprintf( result, format, TimeStart( (uint8_t *)value.ptr ) );
    //возвращаем результат
    if ( strlen( result ) )
        return result;
    if ( strlen( format ) )
        return format;
    return NULL;
 } 

//*************************************************************************************************
// Возвращает расшифровку значения параметра, только для полей имеющих текстовую расшифровку
// Например: значения 0/1 = "Нет"/"Да" и т.д.
// Device dev    - ID уст-ва
// uint8_t param - ID параметра
// return        - указатель на строку с результатом
//*************************************************************************************************
char *ParamGetDesc( Device dev, uint32_t param ) {

    char *ptr = NULL;
    ValueParam value;
    const DevParam *dpar;

    //указатель на список параметров
    dpar = DevParamPtr( dev );
    if ( dpar == NULL )
        return NULL;
    //проверка на превышение кол-ва параметров
    if ( param >= DevParamCnt( dev, CNT_FULL ) )
        return NULL;
    //значение параметра уст-ва
    value = ParamGetVal( dev, param );
    //вывод расшифровки логических значений статусов и режимов
    if ( dpar[param].subtype == BOOL1 )
        ptr = DescBool1( value.uint8 );
    if ( dpar[param].subtype == BOOL2 )
        ptr = DescBool2( value.uint8 );
    if ( dpar[param].subtype == BOOL3 )
        ptr = DescBool3( value.uint8 );
    if ( dpar[param].subtype == BOOL4 )
        ptr = DescBool4( value.uint8 );
    if ( dpar[param].subtype == BOOL5 )
        ptr = DescBool5( value.uint8 );
    if ( dpar[param].subtype == BOOL_BRK )
        ptr = DescBool6( value.uint8 );
    if ( dpar[param].subtype == BOOL7 )
        ptr = DescBool7( value.uint8 );
    if ( dpar[param].subtype == BOOL8 )
        ptr = DescBool8( value.uint8 );
    if ( dpar[param].subtype == BOOL_FUSE )
        ptr = DescBool9( value.uint8 );
    //вывод расшифровки статусов и режимов
    if ( dpar[param].subtype == PWR_STAT )
        ptr = PwrSource( value.uint32 );
    if ( dpar[param].subtype == MPPT_MODE )
        ptr = MpptMode( value.uint32 );
    if ( dpar[param].subtype == CHRGE_MODE )
        ptr = ChrgeMode( value.uint32 );
    if ( dpar[param].subtype == GEN_MODE )
        ptr = GenModeDesc( value.uint8 );
    if ( dpar[param].subtype == GEN_STAT )
        ptr = GenStatDesc( value.uint8 );
    if ( dpar[param].subtype == GEN_ERROR )
        ptr = ErrorDescr( dev, value.uint8, 0 );
    if ( dpar[param].subtype == INVR_MODE )
        ptr = InvModeDesc( value.uint32 );
    if ( dpar[param].subtype == INVR_ERR_CTRL )
        ptr = ErrorDescr( dev, 0, value.uint8 );
    if ( dpar[param].subtype == INVR_ERROR )
        ptr = ErrorDescr( dev, value.uint8, 0 );
    if ( dpar[param].subtype == TRAC_MODE )
        ptr = TracModeDesc( value.uint32 );
    if ( dpar[param].subtype == TRAC_MODE2 )
        ptr = TracModeDesc2( value.uint32 );
    if ( dpar[param].subtype == SPA_ERRDS )
        ptr = SpaErrorDesc( value.uint32 );
    if ( dpar[param].subtype == SYS_MODE )
        ptr = SysModeDesc( value.uint32 );
    if ( dpar[param].subtype == MODE_DIR )
        ptr = DirModeDesc( value.uint32 );
    if ( dpar[param].subtype == SDATE )
        ptr = DateToStr( value.date );
    if ( dpar[param].subtype == TIMESTART )
        ptr = TimeStart( (uint8_t *)value.ptr );
    return ptr;
 }

//*************************************************************************************************
// Возвращает текст ошибки по коду для указанного уст-ва
// Device dev       - ID уст-ва
// uint8_t err_dev  - код ошибки уст-ва
// uint8_t err_ctrl - код ошибки управления уст-м
// return           - адрес строки с расшифровкой кода ошибки
//*************************************************************************************************
char *ErrorDescr( Device dev, uint8_t err_dev, uint8_t err_ctrl ) {

    if ( !err_dev && !err_ctrl )
        return (char *)result_ok;
    if ( dev == ID_DEV_CHARGER ) {
        //расшифровка кодов ошибок управления контроллером заряда
        if ( err_ctrl < SIZE_ARRAY( charge_err_descr ) )
            return charge_err_descr[err_ctrl];
        else return NULL;
       }
    if ( ( dev == ID_DEV_INV1 || dev == ID_DEV_INV2 ) && err_dev ) {
        //расшифровка кодов ошибок инвертора
        if ( err_dev < SIZE_ARRAY( inv_err_descr ) )
            return inv_err_descr[err_dev];
        else return NULL;
       }
    if ( ( dev == ID_DEV_INV1 || dev == ID_DEV_INV2 ) && err_ctrl ) {
        //расшифровка кодов ошибок управления инверторами
        if ( err_ctrl < SIZE_ARRAY( inv_err_ctrl ) )
            return inv_err_ctrl[err_ctrl];
        else return NULL;
       }
    if ( dev == ID_DEV_ALT ) {
        //расшифровка кодов ошибок управления блоком АВР
        if ( err_ctrl < SIZE_ARRAY( alt_err_descr ) )
            return alt_err_descr[err_ctrl];
        else return NULL;
       }
    if ( dev == ID_DEV_PV ) {
        //расшифровка кодов ошибок управления блоком АВР
        if ( err_ctrl < SIZE_ARRAY( pv_err_descr ) )
            return pv_err_descr[err_ctrl];
        else return NULL;
       }
    if ( dev == ID_DEV_GEN ) {
        //расшифровка кодов ошибок управления генератором
        if ( err_dev < SIZE_ARRAY( gen_err_descr ) )
            return gen_err_descr[err_dev];
        else return NULL;
       }
    return (char *)result_undef;
 }

//*************************************************************************************************
// Дополняет строку справа знаками '.', граница заполнения задана в PARAM_ALIGNMENT
// char *src - указатель на исходную строку
// return    - кол-во добавленных символов в т.ч. доп. пробел
//*************************************************************************************************
uint8_t AddDot( char *src, uint8_t aligment ) {

    uint8_t add, len;

    strcat( src, " " );
    len = strlen( src );
    for ( add = 0; len < aligment; add++, len++ )
        *(src + len) = '.';
    *(src + len) = ' ';
    return add + 2;
 }

//*************************************************************************************************
// Перевод секундных значений в полный формат: HH:MM:SS
// uint16_t value - значение таймера (в секундах)  
// return         - указатель на строку с результатом
//*************************************************************************************************
static char *TimeValue( uint16_t value ) {

    uint16_t hour, min, sec;
    
    hour = value / 3600; 
    sec = value % 3600;
    min = sec / 60;
    sec = sec % 60;
    sprintf( time_str1, "%02d:%02d:%02d", hour, min, sec );
    return time_str1;
 }
 
//*************************************************************************************************
// Перевод секундных значений в формат: HH:MM
// Если value > 59:59 (59*60+59) = 3599, выводим "**:**"
// uint16_t value - значение таймера (в секундах)  
// return         - указатель на строку с результатом
//*************************************************************************************************
static char *Time1Value( uint32_t value ) {

    uint16_t hour, min;
    
    if ( value > 3599 ) {
        sprintf( time_str2, "**:**" );
        return time_str2;
       }
    hour = value / 60;
    min = value % 60;
    sprintf( time_str2, "%02d:%02d", hour, min );
    return time_str2;
 }

//*************************************************************************************************
// Перевод секундных значений в формат: MM:SS
// Если value > 59:59 (59*60+59) = 3599, выводим "**:**"
// uint16_t value - значение таймера (в секундах)  
// return         - указатель на строку с результатом
//*************************************************************************************************
static char *Time2Value( uint16_t value ) {

    uint16_t min, sec;
    
    if ( value > 3599 ) {
        sprintf( time_str3, "**:**" );
        return time_str3;
       }
    sec = value % 3600;
    min = sec / 60;
    sec = sec % 60;
    sprintf( time_str3, "%02d:%02d", min, sec );
    return time_str3;
 }

//*************************************************************************************************
// Перевод секундных значений в формат: HHH:MM
// uint16_t value - значение таймера (в секундах)  
// return         - указатель на строку с результатом
//*************************************************************************************************
static char *Time3Value( uint32_t value ) {

    uint16_t hour, min;
    
    hour = value / 60;
    min = value % 60;
    sprintf( time_str4, "%03d:%02d", hour, min );
    return time_str4;
 }

//*************************************************************************************************
// Перевод секундных значений в формат: HH:MM
// float value - значение в секундах
// return      - указатель на строку с результатом
//*************************************************************************************************
static char *Time4Value( float value ) {

    float min;
    
    min = 60.0 * ( value - (int)value );
    sprintf( time_str5, "%02d:%02d", (int)value, (int)min );
    return time_str5;
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку логического значения "Да/Нет"
// uint32_t value - логическое значение
// return         - указатель на строку
//*************************************************************************************************
static char *DescBool1( uint32_t value ) {

    if ( value )
        return bool_desc1[1];
    else return bool_desc1[0];
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку логического значения "Вкл/Выкл"
// uint32_t value - логическое значение
// return         - указатель на строку
//*************************************************************************************************
static char *DescBool2( uint32_t value ) {

    if ( value )
        return bool_desc2[1];
    else return bool_desc2[0];
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку логического значения "Завершена/Заряд"
// uint32_t value - логическое значение
// return         - указатель на строку
//*************************************************************************************************
static char *DescBool3( uint32_t value ) {

    if ( value )
        return bool_desc3[0];
    else return bool_desc3[1];
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку логического значения "Исправен/Неисправен"
// uint32_t value - логическое значение
// return         - указатель на строку
//*************************************************************************************************
static char *DescBool4( uint32_t value ) {

    if ( value )
        return bool_desc4[1];
    else return bool_desc4[0];
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку логического значения "Паралл/Пар+Посл"
// uint32_t value - логическое значение
// return         - указатель на строку
//*************************************************************************************************
static char *DescBool5( uint32_t value ) {

    if ( value )
        return bool_desc5[1];
    else return bool_desc5[0];
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку логического значения "Авар/ОК"
// uint32_t value - логическое значение
// return         - указатель на строку
//*************************************************************************************************
static char *DescBool6( uint32_t value ) {

    if ( value )
        return bool_desc6[1];
    else return bool_desc6[0];
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку логического значения "Автоматический/Ручной"
// uint32_t value - логическое значение
// return         - указатель на строку
//*************************************************************************************************
static char *DescBool7( uint32_t value ) {

    if ( value )
        return bool_desc7[1];
    else return bool_desc7[0];
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку логического значения "OK/Нет"
// uint32_t value - код режима зарядки работы
// return         - указатель на строку
//*************************************************************************************************
static char *DescBool8( uint32_t value ) {

    if ( value )
        return bool_desc8[1];
    else return bool_desc8[0];
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку логического значения для предохранителя "Авар/ОК"
// uint32_t value - код режима зарядки работы
// return         - указатель на строку
//*************************************************************************************************
static char *DescBool9( uint32_t value ) {

    if ( value )
        return bool_desc10[1];
    else return bool_desc10[0];
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку источника питания нагрузки
// uint32_t value - логическое значение
// return         - указатель на строку
//*************************************************************************************************
static char *PwrSource( uint32_t value ) {

    if ( value < SIZE_ARRAY( pwr_src ) )
        return pwr_src[value];
    else return NULL;
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку состояния контроллера MPPT
// uint32_t value - логическое значение
// return         - указатель на строку с результатом
//*************************************************************************************************
static char *MpptMode( uint32_t value ) {

    if ( value < SIZE_ARRAY( mppt_mode ) )
        return mppt_mode[value];
    else return NULL;
 }

//*************************************************************************************************
// Возвращает номер режима зарядки "0/2/3/8"
// uint32_t value - код режима работы
// return         - указатель на строку с результатом
//*************************************************************************************************
static char *ChrgeMode( uint32_t value ) {

    if ( value < SIZE_ARRAY( charge_mode ) )
        return charge_mode[value];
    else return NULL;
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку режима работы генератора
// uint32_t value - код режима работы
// return         - указатель на строку с результатом
//*************************************************************************************************
static char *GenModeDesc( uint32_t value ) {

    if ( value < SIZE_ARRAY( gen_mode ) )
        return gen_mode[value];
    else return NULL;
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку режима работы генератора
// uint32_t value - код режима работы
// return         - указатель на строку с результатом
//*************************************************************************************************
static char *GenStatDesc( uint32_t value ) {

    if ( value < SIZE_ARRAY( gen_stat ) )
        return gen_stat[value];
    else return NULL;
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку режима работы инвертора
// uint32_t value - код режима работы
// return         - указатель на строку с результатом
//*************************************************************************************************
static char *InvModeDesc( uint32_t value ) {

    if ( value < SIZE_ARRAY( inv_mode ) )
        return inv_mode[value];
    else return NULL;
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку режима работы трекера
// uint32_t value - код режима работы
//*************************************************************************************************
static char *TracModeDesc( uint32_t value ) {

    if ( value < SIZE_ARRAY( trc_mode ) )
        return trc_mode[value];
    else return NULL;
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку (англ) режима работы трекера 
// uint32_t value - код режима работы
//*************************************************************************************************
static char *TracModeDesc2( uint32_t value ) {

    if ( value < SIZE_ARRAY( trc_mode2 ) )
        return trc_mode2[value];
    else return NULL;
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку ошибок проверки параметров для расчета SPA
// uint32_t value - код ошибки
// return         - указатель на строку с результатом
//*************************************************************************************************
static char *SpaErrorDesc( uint32_t value ) {

    if ( value < SIZE_ARRAY( spa_error ) )
        return spa_error[value];
    else return NULL;
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку режима работы системы
// uint32_t value - код режима работы
// return         - указатель на строку с результатом
//*************************************************************************************************
static char *SysModeDesc( uint32_t value ) {

    if ( value < SIZE_ARRAY( sys_mode ) )
        return sys_mode[value];
    else return NULL;
 }

//*************************************************************************************************
// Возвращает текстовую расшифровку режима логирования файлов
// uint32_t value - код типа логирования
// return         - указатель на строку с результатом
//*************************************************************************************************
static char *DirModeDesc( uint32_t value ) {

    if ( value < SIZE_ARRAY( bool_desc9 ) )
        return bool_desc9[value];
    else return NULL;
 }

//*************************************************************************************************
// Возвращает дату из переменной типа "DATE" как текстовую строку
// DATE value - дата
// return     - указатель на строку с результатом
//*************************************************************************************************
static char *DateToStr( DATE date ) {

    sprintf( date_str, "%02u.%02u.%04u", date.day, date.month, date.year );
    return date_str;
 }

//*************************************************************************************************
// Возвращает продолжительность запуска генератора для каждой попытки 
// uint8_t *ptr - указатель на массив 
// return     - указатель на строку с результатом
//*************************************************************************************************
static char *TimeStart( uint8_t *ptr_arr ) {

    uint8_t i;
    char *ptr_str;

    ptr_str = time_str6;
    for ( i = 0; i < 8; i++ ) {
        if ( i < 7 )
            ptr_str += sprintf( ptr_str, "%u,", *( ptr_arr + i ) );
        else ptr_str += sprintf( ptr_str, "%u", *( ptr_arr + i ) );
       }
    return time_str6;
 }

//*************************************************************************************************
// Преобразование значения параметра типа "строка" в значение ConfigValSet
// ConfigParam id_par    - ID параметра
// char *value           - указатель на строку со значением параметра 
// ConfigValSet *cfg_set - Указатель на переменную ConfigValSet для размещения значения параметра
//*************************************************************************************************
void StrToConfigVal( ConfigParam id_par, char *value, ConfigValSet *cfg_set ) {

    DATE date;
    uint8_t idx, ids, val;
    
    if ( id_par >= SIZE_ARRAY( ConfCheck ) ) {
        cfg_set->uint32 = 0;
        return;
       }
    memset( (uint8_t *)cfg_set, 0x00, sizeof( ConfigValSet ) );
    //указатель на строку
    if ( ConfCheck[id_par].type_var == STRING )
        cfg_set->ptr = value;
    //целое без знака
    if ( ConfCheck[id_par].type_var == NUMBER )
        cfg_set->uint32 = atoi( value );
    //целое со знаком
    if ( ConfCheck[id_par].type_var == NUMSIGN )
        cfg_set->int32 = atoi( value );
    //с плавающей точкой
    if ( ConfCheck[id_par].type_var == DOUBLE )
        cfg_set->dbl = atof( value );
    //дата с проверкой
    if ( ConfCheck[id_par].type_var == DATES ) {
        if ( CheckDate( value, &date.day, &date.month, &date.year ) == SUCCESS )
            cfg_set->date = date;
        else {
            cfg_set->date.day = 0;
            cfg_set->date.month = 0;
            cfg_set->date.year = 0;
           }
       }
    //строка с целыми без знака
    if ( ConfCheck[id_par].type_var == STRINT ) {
        //преобразование каждой цифры в строке как одно число
        for ( idx = 0, ids = 0; idx < strlen( value ) && ids < sizeof( cfg_set->uint8_array ); idx++ ) {
            val = *( value + idx );
            if ( isdigit( val ) ) {
                cfg_set->uint8_array[ids] = val - 0x30;
                ids++;
               }
           }
       }
 }

//*************************************************************************************************
// Проверка значения параметра настройки на допустимые значения 
// ConfigParam id_par   - ID параметра
// ConfigValSet cfg_set - значение параметра
// return = SUCCESS     - данные допустимы
//          ERROR       - данные не допустимы
//*************************************************************************************************
Status ConfigChkVal( ConfigParam id_par, ConfigValSet cfg_set ) {

    uint8_t i;
    uint32_t min_val, max_val;
    
    if ( id_par >= SIZE_ARRAY( ConfCheck ) )
        return ERROR;
    min_val = ConfCheck[id_par].min_value;
    max_val = ConfCheck[id_par].max_value;
    if ( ConfCheck[id_par].check == false )
        return SUCCESS; //проверка не требуется
    if ( ConfCheck[id_par].param != id_par )
        return ERROR; //внутренний номер в структуре не совпал с внешним номером
    //тип "строка"
    if ( ConfCheck[id_par].type_var == STRING && strlen( cfg_set.ptr ) < ConfCheck[id_par].max_value )
        return SUCCESS;
    //тип целое без знака
    if ( ConfCheck[id_par].type_var == NUMBER && cfg_set.uint32 >= min_val && cfg_set.uint32 <= max_val )
        return SUCCESS;
    //тип целое со знаком
    if ( ConfCheck[id_par].type_var == NUMSIGN && cfg_set.int32 >= (int32_t)min_val && cfg_set.int32 <= (int32_t)max_val )
        return SUCCESS;
    //тип с плавающей точкой
    if ( ConfCheck[id_par].type_var == DOUBLE && cfg_set.dbl >= (double)min_val && cfg_set.dbl <= (double)max_val )
        return SUCCESS;
    //тип дата
    if ( ConfCheck[id_par].type_var == DATES && cfg_set.date.day && cfg_set.date.month && cfg_set.date.year )
        return SUCCESS;
    if ( ConfCheck[id_par].type_var == STRINT ) {
        //проверка каждой цифры в массиве
        for ( i = 0; i < config.gen_cnt_start && i < sizeof( cfg_set.uint8_array ); i++ ) {
            if ( cfg_set.uint8_array[i] >= min_val && cfg_set.uint8_array[i] <= max_val )
                continue;
            else return ERROR;
           }
        return SUCCESS;
       }
    return ERROR;
 }

//*************************************************************************************************
// Возвращает минимальное и максимальное допустимое значения параметра
// ConfigParam id_par - ID параметра
// uint32_t *min      - указатель на переменную с минимальным допустимым значением параметра
// uint32_t *max      - указатель на переменную с максимальным допустимым значением параметра
//*************************************************************************************************
void ConfigLimit( ConfigParam id_par, int32_t *min, int32_t *max ) {

    *min = 0;
    *max = 0;
    if ( id_par >= SIZE_ARRAY( ConfCheck ) )
        return;
    if ( ConfCheck[id_par].param != id_par )
        return; //внутренний номер в структуре не совпал с внешним номером
    if ( ConfCheck[id_par].check == false )
        return; //проверка не требуется
    //допустимые значения
    *min = ConfCheck[id_par].min_value;
    *max = ConfCheck[id_par].max_value;
    return;
 }

//*************************************************************************************************
// Возвращает размер переменной указанного параметра
// ConfigParam id_par - ID параметра
// return             - размер параметра в байтах
//*************************************************************************************************
uint8_t ConfigParSize( ConfigParam id_par ) {

    if ( id_par < SIZE_ARRAY( ConfCheck ) )
        return ConfCheck[id_par].size_data;
    return 0;
 }

//*************************************************************************************************
// Возвращает значения параметров портов
// ParamPort id_param - ID параметра
// return ValueParam  - значение параметра
//*************************************************************************************************
ValueParam PortsGetValue( ParamPort id_param ) {

    ValueParam value;

    value.uint32 = NULL;
    if ( id_param == BAT_CONNECT )
        value.uint8 = ports.bat_connect;
    if ( id_param == CPU_MODE )
        value.uint8 = ports.cpu_mode;
    if ( id_param == FUSE_24VDC )
        value.uint8 = ports.fuse_24vdc;
    if ( id_param == STAT_CTRL )
        value.uint8 = ports.stat_ctrl;
    if ( id_param == PORTS_ALL )
        value.uint32 = ports.state;
    return value;
 }

//*************************************************************************************************
// Возвращает значения параметров монитора АКБ
// ParamBatMon id_param - ID параметра
// return ValueParam    - значение параметра
//*************************************************************************************************
ValueParam BatmonGetValue( ParamBatMon id_param ) {

    ValueParam value;
    
    value.uint32 = NULL;
    if ( id_param == MON_VOLTAGE )
        value.flt = batmon.voltage;
    if ( id_param == MON_CURRENT )
        value.flt = batmon.current;
    if ( id_param == MON_CONSUMENERGY )
        value.flt = batmon.cons_energy;
    if ( id_param == MON_SOC )
        value.flt = batmon.soc;
    if ( id_param == MON_TTG )
        value.uint16 = batmon.ttg;
    if ( id_param == MON_ALARM )
        value.uint8 = batmon.alarm;
    if ( id_param == MON_RELAY )
        value.uint8 = batmon.relay;
    if ( id_param == MON_ALARMMODE )
        value.uint8 = batmon.alarm_mode;
    if ( id_param == MON_MODEL )
        value.ptr = batmon.model;
    if ( id_param == MON_VERSION )
        value.ptr = batmon.version;
    if ( id_param == MON_H1 )
        value.flt = batmon.h1;
    if ( id_param == MON_H2 )
        value.flt = batmon.h2;
    if ( id_param == MON_H3 )
        value.flt = batmon.h3;
    if ( id_param == MON_H4 )
        value.uint16 = batmon.h4;
    if ( id_param == MON_H5 )
        value.uint16 = batmon.h5;
    if ( id_param == MON_H6 )
        value.flt = batmon.h6;
    if ( id_param == MON_H7 )
        value.flt = batmon.h7;
    if ( id_param == MON_H8 )
        value.flt = batmon.h8;
    if ( id_param == MON_H9 )
        value.uint16 = batmon.h9;
    if ( id_param == MON_H10 )
        value.uint16 = batmon.h10;
    if ( id_param == MON_H11 )
        value.uint16 = batmon.h11;
    if ( id_param == MON_H12 )
        value.uint16 = batmon.h12;
    if ( id_param == MON_LINK )
        value.uint8 = batmon.link;
    return value;
 }

//*************************************************************************************************
// Возвращает значения параметров солнечного контроллера заряда
// ParamMppt id_param - ID параметра
// return ParamMppt   - значение параметра
//*************************************************************************************************
ValueParam MpptGetValue( ParamMppt id_param ) {

    ValueParam value;
    
    value.uint32 = NULL;
    if ( id_param == MPPT_IN_VOLTAGE )
        value.flt = mppt.u01_in_voltage;
    if ( id_param == MPPT_IN_CURRENT )
        value.flt = mppt.u02_in_current;
    if ( id_param == MPPT_OUT_VOLTAGE )
        value.flt = mppt.u03_out_voltage;
    if ( id_param == MPPT_OUT_CURRENT )
        value.flt = mppt.u04_out_current;
    if ( id_param == MPPT_ENERGY1 )
        value.uint32 = mppt.u05_energy1;
    if ( id_param == MPPT_ENERGY2 )
        value.uint32 = mppt.u05_energy2;
    if ( id_param == MPPT_TIME_FLOAT )
        value.uint16 = mppt.u07_time_flt;
    if ( id_param == MPPT_CHARGE_MODE )
        value.uint8 = mppt.u08_charge_mode;
    if ( id_param == MPPT_MPPT_TEMP )
        value.flt = mppt.u11_mppt_temp;
    if ( id_param == MPPT_SOC )
        value.uint8 = mppt.u12_soc;
    if ( id_param == MPPT_BAT_CURRENT )
        value.flt = mppt.u13_bat_current;
    if ( id_param == MPPT_BAT_CAPASITY )
        value.uint16 = mppt.u14_bat_capasity;
    if ( id_param == MPPT_BAT_TEMP )
        value.flt = mppt.u15_bat_temp;
    if ( id_param == MPPT_SERIAL )
        value.uint16 = mppt.u17_serial;
    if ( id_param == MPPT_TIMECHARGING )
        value.uint16 = mppt.time_charge;
    if ( id_param == MPPT_POWER )
        value.uint8 = mppt.power;
    if ( id_param == MPPT_CONNECT )
        value.uint8 = mppt.connect;
    if ( id_param == MPPT_PVON )
        value.uint8 = mppt.pv_stat;
    if ( id_param == MPPT_PVMODE )
        value.uint8 = mppt.pv_mode;
    if ( id_param == MPPT_LINK )
        value.uint8 = mppt.link;
    return value;
 }

//*************************************************************************************************
// Возвращает значения параметров состояния солнечных панелей
// ParamPv id_param  - ID параметра
// return ValueParam - значение параметра
//*************************************************************************************************
ValueParam PvGetValue( ParamPv id_param ) {

    ValueParam value;
    
    value.uint32 = NULL;
    if ( id_param == PV_PVON )
        value.uint8 = mppt.pv_stat;
    if ( id_param == PV_PVMODE )
        value.uint8 = mppt.pv_mode;
    return value;
 }

//*************************************************************************************************
// Возвращает значения параметров контроллера заряда
// ParamCharger id_param - ID параметра
// return ValueParam     - значение параметра
//*************************************************************************************************
ValueParam ChargeGetValue( ParamCharger id_param ) {

    ValueParam value;
    
    value.uint32 = NULL;
    if ( id_param == CHARGE_CONN_AC )
        value.uint8 = charger.connect_ac;
    if ( id_param == CHARGE_DEV_STAT )
        value.uint8 = charger.device_ok;
    if ( id_param == CHARGE_BANK_STAT )
        value.uint8 = charger.charge_end;
    if ( id_param == CHARGE_MODE )
        value.uint8 = charger.charge_mode;
    if ( id_param == CHARGE_CURRENT )
        value.flt = charger.current;
    return value;
 }

//*************************************************************************************************
// Возвращает значения параметров блока АВР
// ParamAlt id_param - ID параметра
// return ValueParam - значение параметра
//*************************************************************************************************
ValueParam AltGetValue( ParamAlt id_param ) {

    ValueParam value;
    
    value.uint32 = NULL;
    if ( id_param == ALT_CONN )
        value.uint8 = alt.connect;
    if ( id_param == ALT_MAIN_AC )
        value.uint8 = alt.main_ac;
    if ( id_param == ALT_GEN_ON )
        value.uint8 = alt.gen_on;
    if ( id_param == ALT_POWER_SRC )
        value.uint8 = alt.power;
    if ( id_param == ALT_DELAY_TS )
        value.uint16 = alt.timer_delay;
    return value;
 }

//*************************************************************************************************
// Возвращает значения параметров инвертора
// Device dev        - ID инвертора
// ParamInv id_param - ID параметра
// return ValueParam - значение параметра
//*************************************************************************************************
ValueParam InvGetValue( Device dev, ParamInv id_param ) {

    ValueParam value;
    
    value.uint32 = NULL;
    INVERTER *inv = NULL;
    
    //выбор инвертора
    if ( dev == ID_DEV_INV1 )
        inv = &inv1;
    if ( dev == ID_DEV_INV2 )
        inv = &inv2;
    //выбор данных
    if ( id_param == INV_CFG_EQL_VOLT )
        value.flt = inv->cfg_eql_volt;
    if ( id_param == INV_CFG_FLT_VOLT )
        value.flt = inv->cfg_flt_volt;
    if ( id_param == INV_CFG_ALARM_VOLT )
        value.flt = inv->cfg_alarm_volt;
    if ( id_param == INV_CFG_SHDN_VOLT )
        value.flt = inv->cfg_shdn_volt;
    if ( id_param == INV_VENDOR )
        value.ptr = inv->vendor;
    if ( id_param == INV_VERSION )
        value.ptr = inv->version;
    if ( id_param == INV_MODEL )
        value.ptr = inv->model;
    if ( id_param == INV_AC_OUT )
        value.uint16 = inv->ac_out;
    if ( id_param == INV_AC_POWER )
        value.uint8 = inv->ac_power;
    if ( id_param == INV_DC_IN )
        value.flt = inv->dc_in;
    if ( id_param == INV_BAT_PERC )
        value.uint8 = inv->bat_perc;
    if ( id_param == INV_TEMPERATURE )
        value.flt = inv->temperature;
    if ( id_param == INV_UNUSED )
        value.uint16 = inv->unused;
    if ( id_param == INV_AC_FREQ )
        value.flt = inv->ac_freq;
    if ( id_param == INV_WORK_TIME )
        value.uint16 = inv->work_time;
    if ( id_param == INV_POWER_PERC )
        value.uint8 = inv->power_perc;
    if ( id_param == INV_POWER_WATT )
        value.uint16 = inv->power_watt;
    if ( id_param == INV_STATUS )
        value.uint32 = inv->bit_status;
    if ( id_param == INV_DC_CONNECT )
        value.uint8 = inv->dc_conn;
    if ( id_param == INV_MODE )
        value.uint8 = inv->mode;
    if ( id_param == INV_ERROR )
        value.uint8 = inv->dev_error;
    if ( id_param == INV_CTRL_ERR )
        value.uint8 = inv->ctrl_error;
    return value;
 }

//*************************************************************************************************
// Возвращает значения параметров контроллера трекера
// ParamTracker id_param - ID параметра
// return ValueParam     - значение параметра
//*************************************************************************************************
ValueParam TrackerGetValue( ParamTracker id_param ) {

    ValueParam value;

    value.uint32 = NULL;
    if ( id_param == TRC_LINK )
        value.uint8 = tracker.link;
    if ( id_param == TRC_POWER )
        value.uint8 = tracker.pwr_trc;
    if ( id_param == TRC_POWER_BREAK )
        value.uint8 = tracker.pwr_fuse;
    if ( id_param == TRC_ACT )
        value.uint8 = tracker.pwr_act;
    if ( id_param == TRC_MODE || id_param == TRC_MODE2 )
        value.uint8 = TrackerMode( tracker.stat );
    if ( id_param == TRC_FUSE_OK )
        value.uint8 = ( tracker.stat & EXT_V24_OK ) ? POWER_ON : POWER_OFF;
    if ( id_param == TRC_SENSOR )
        value.ptr = TrackerSensor( tracker.stat );
    if ( id_param == TRC_VERT )
        value.ptr = PosAngle( tracker.act_pos_vert, TRC_POS_VERTICAL );
    if ( id_param == TRC_HORZ )
        value.ptr = PosAngle( tracker.act_pos_horz, TRC_POS_HORIZONTAL );
    if ( id_param == TRC_VEEP )
        value.uint16 = tracker.act_vert_eep;
    if ( id_param == TRC_HEEP )
        value.uint16 = tracker.act_horz_eep;
    if ( id_param == TRC_LIMSW )
        value.ptr = TrackerLimSw( tracker.stat );
    if ( id_param == TRC_TIMEON )
        value.uint16 = tracker.time_on;
    if ( id_param == TRC_VERT_MM )
        value.uint16 = tracker.act_pos_vert;
    if ( id_param == TRC_HORZ_MM )
        value.uint16 = tracker.act_pos_horz;
    return value;
 }

//*************************************************************************************************
// Возвращает режим работы трекера
// uint16_t stat      - статус трекера
// return TrackerStat - результат командный/автоматический
//*************************************************************************************************
static TrackerStat TrackerMode( uint16_t stat ) {

    //статус трекера
    if ( stat & EXT_CMND_MODE )
        return TRC_STAT_COMMAND;
    if ( stat & EXT_WIND_WARNING )
        return TRC_STAT_WIND;
    if ( !( stat & ( EXT_CMND_MODE | EXT_WIND_WARNING ) ) )
        return TRC_STAT_SENSOR;
    return TRC_STAT_NOLINK;
 }

//*************************************************************************************************
// Возвращает значения параметров положения солнца
// ParamSunPos id_param - ID параметра
// return ValueParam    - значение параметра
//*************************************************************************************************
ValueParam SpaGetValue( ParamSunPos id_param ) {

    ValueParam value;

    value.uint32 = NULL;
    if ( id_param == SPA_SUNRISE || id_param == SPA_SUNRISE1 )
        value.flt = sunpos.sunrise;
    if ( id_param == SPA_SUNSET || id_param == SPA_SUNSET1 )
        value.flt = sunpos.sunset;
    if ( id_param == SPA_TIMEZONE )
        value.int8 = (int8_t)config.spa_timezone;
    if ( id_param == SPA_LONGITUDE )
        value.flt = config.spa_longitude;
    if ( id_param == SPA_LATITUDE )
        value.flt = config.spa_latitude;
    if ( id_param == SPA_ELEVATION )
        value.uint16 = (uint16_t)config.spa_elevation;
    if ( id_param == SPA_PRESSURE )
        value.uint16 = (uint16_t)config.spa_pressure;
    if ( id_param == SPA_TEMPERATURE )
        value.uint8 = (uint8_t)config.spa_temperature;
    if ( id_param == SPA_SLOPE )
        value.uint16 = (uint16_t)config.spa_slope;
    if ( id_param == SPA_AZM_ROTATION )
        value.uint16 = (uint16_t)config.spa_azm_rotation;
    if ( id_param == SPA_ZENIT || id_param == SPA_ZENIT1 )
        value.flt = sunpos.zenith;
    if ( id_param == SPA_AZIMUT || id_param == SPA_AZIMUT1 )
        value.flt = sunpos.azimuth;
    if ( id_param == SPA_DURATION || id_param == SPA_DURATION1 )
        value.flt = sunpos.duration;
    if ( id_param == SPA_ERROR )
        value.uint8 = sunpos.error;
    return value;
 }

//*************************************************************************************************
// Возвращает значения параметров голосового информатора
// ParamMppt id_param - ID параметра
// return ValueParam  - значение параметра
//*************************************************************************************************
ValueParam VoiceGetValue( ParamVoice id_param ) {

    ValueParam value;
    
    value.uint32 = NULL;
    if ( id_param == VOICE_PAR_CONN )
        value.uint8 = voice.link;
    if ( id_param == VOICE_PAR_STAT )
        value.uint8 = voice.stat;
    if ( id_param == VOICE_PAR_VOL )
        value.uint8 = voice.volume;
    return value;
 }

//*************************************************************************************************
// Возвращает значения параметров настроек управляющего контроллера
// ConfigParam id_param - ID параметра
// return ValueParam    - значение параметра
//*************************************************************************************************
ValueParam ConfigValue( ConfigParam id_param ) {

    ValueParam value;
    
    value.uint32 = NULL;
    if ( id_param == CFG_SCR_FILE )                     //имя файла экрана с макроподстановками    
        value.ptr = config.scr_file;
    if ( id_param == CFG_JOB_FILE )                     //имя файла загружаемых заданий для планировщика
        value.ptr = config.job_file;
    if ( id_param == CFG_JOB_TEST )                     //имя файла загружаемых заданий для планировщика
        value.ptr = config.job_test;
    if ( id_param == CFG_MODE_SYS )                     //режим работы системы
        value.uint8 = config.mode_sys;
    if ( id_param == CFG_MODE_LOGGING )                 //режим логирования файлов 0/1 - [каталог\файл]/[\каталог\YYYYMM\файл]
        value.uint8 = config.mode_logging;
    if ( id_param == CFG_LAST_CHARGE )                  //дата последнего включения подзарядки от основной сети
        value.date = config.last_charge;
    if ( id_param == CFG_LOG_ENABLE_PV )                //вкл логирование команд управление солнечными панелями
        value.uint8 = config.log_enable_pv;
    if ( id_param == CFG_LOG_ENABLE_CHARGE )            //вкл логирование команд зарядного уст-ва
        value.uint8 = config.log_enable_chrg;
    if ( id_param == CFG_LOG_ENABLE_MPPT )              //вкл логирование команд/данных вкл/выкл солнечных панелей
        value.uint8 = config.log_enable_mppt;
    if ( id_param == CFG_LOG_ENABLE_INV )               //вкл логирование команд/данных управление инверторов
        value.uint8 = config.log_enable_inv;
    if ( id_param == CFG_LOG_ENABLE_BMON )              //вкл логирование данных монитора батареи
        value.uint8 = config.log_enable_bmon;
    if ( id_param == CFG_LOG_ENABLE_GEN )               //вкл логирование команд генератора
        value.uint8 = config.log_enable_gen;
    if ( id_param == CFG_LOG_ENABLE_ALT )               //вкл логирование команд блока АВР
        value.uint8 = config.log_enable_alt;
    if ( id_param == CFG_LOG_ENABLE_TRC )               //вкл логирование команд трекера
        value.uint8 = config.log_enable_trc;
    if ( id_param == CFG_DATLOG_UPD_CHARGE )            //период записи данных зарядного уст-ва
        value.uint16 = config.datlog_upd_chrg;
    if ( id_param == CFG_DATLOG_UPD_MPPT )              //период записи данных солнечного контроллера заряда
        value.uint16 = config.datlog_upd_mppt;
    if ( id_param == CFG_DATLOG_UPD_BMON )              //период записи данных монитора батареи
        value.uint16 = config.datlog_upd_bmon;
    if ( id_param == CFG_DATLOG_UPD_INV )               //период записи данных инверторов
        value.uint16 = config.datlog_upd_inv;
    if ( id_param == CFG_DATLOG_UPD_TRC )               //период записи данных трекера
        value.uint16 = config.datlog_upd_trc;
    if ( id_param == CFG_GEN_DELAY_START )              //задержка запуска генератора после отключения основной сети (сек)
        value.uint16 = config.gen_delay_start;
    if ( id_param == CFG_GEN_DELAY_STOP )               //задержка выключения генератора после восстановления основной сети (сек)
        value.uint16 = config.gen_delay_stop;
    if ( id_param == CFG_GEN_DELAY_CHK_RUN )            //ожидание сигнала запуска генератора (сек)
        value.uint16 = config.gen_delay_chk_run;
    if ( id_param == CFG_GEN_BEFORE_START )             //пауза между запусками (сек) фактическое время = delay_check_run + before_start
        value.uint8 = config.gen_before_start;
    if ( id_param == CFG_GEN_CNT_START )                //кол-во попыток запуска генератора (макс - 8)
        value.uint16 = config.gen_cnt_start;
    if ( id_param == CFG_GEN_TIME_START )               //продолжительность запуска для каждой попытки (макс - 8)
        value.ptr = (uint8_t *)&config.gen_time_start;
    if ( id_param == CFG_GEN_TIME_RUN )                 //#21600 максимальная продолжительность работы генератора (сек)
        value.uint16 = config.gen_time_run;
    if ( id_param == CFG_GEN_TIME_SLEEP )               //#7200 продолжительность паузы между длительными работами (сек)
        value.uint16 = config.gen_time_sleep;
    if ( id_param == CFG_GEN_TIME_TEST )                //#600 продолжительность тестирования генератора (сек)
        value.uint16 = config.gen_time_test;
    if ( id_param == CFG_GEN_AUTO_MODE )                //ручной/автоматический режим запуска при отключении сети
        value.uint8 = config.gen_auto_mode;
    if ( id_param == CFG_GEN_LAST_RUN )                 //дата последнего включения генератора
        value.date = config.gen_last_run;
    if ( id_param == CFG_SPA_TIMEZONE )                 //часовой пояс
        value.uint8 = config.spa_timezone;
    if ( id_param == CFG_SPA_LATITUDE )                 //Широта наблюдателя
        value.flt = config.spa_latitude;
    if ( id_param == CFG_SPA_LONGITUDE )                //Долгота наблюдателя
        value.flt = config.spa_longitude;
    if ( id_param == CFG_SPA_ELEVATION )                //Высота наблюдателя
        value.uint16 = config.spa_elevation;
    if ( id_param == CFG_SPA_PRESSURE )                 //Среднегодовое местное давление
        value.uint16 = config.spa_pressure;
    if ( id_param == CFG_SPA_TEMPERATURE )              //Среднегодовая местная температура
        value.uint8 = config.spa_temperature;
    if ( id_param == CFG_SPA_SLOPE )                    //Наклон поверхности (измеряется от горизонтальной плоскости)
        value.uint16 = config.spa_slope;
    if ( id_param == CFG_SPA_AZM_ROTATION )             //Вращение поверхности азимута (измеряется с юга на проекции нормали
        value.uint16 = config.spa_azm_rotation;
    if ( id_param == CFG_PB_CURRENT_STOP )              //минимальный ток при котором происходит выключение зарядки от PB-1000-224
        value.uint16 = config.pb_current_stop;
    if ( id_param == CFG_DELAY_START_INV )               //таймер задержки вкл инверторов при отключении основной сети
        value.uint16 = config.delay_start_inv;
    if ( id_param == CFG_DELAY_STOP_INV )                //таймер задержки выкл инверторов после восстановлении основной сети
        value.uint16 = config.delay_stop_inv;
    return value;
 }

//*************************************************************************************************
// Возвращает значения часов (RTC)
// ParamRtc id_param - ID параметра
// return ValueParam - значение параметра
//*************************************************************************************************
ValueParam RtcGetValue( ParamRtc id_param ) {

    ValueParam value;
    
    value.uint32 = NULL;
    if ( id_param == RTC_DATE )
        value.ptr = RTCGetDate( NULL );
    if ( id_param == RTC_TIME )
        value.ptr = RTCGetTime( NULL );
    if ( id_param == RTC_FULL )
        value.ptr = RTCGetLog();
    if ( id_param == RTC_FULL_DW )
    #ifdef CONFIG_CONTROL
        value.ptr = RTCGetDateTime( NULL );
    #else
        value.ptr = RTCGetDateTime( NULL, DOW_RUS_LANG );
    #endif
    return value;
 }

//*************************************************************************************************
// Возвращает значения параметров контроллера генератора
// ParamGen id_param - ID параметра
// return ValueParam - значение параметра
//*************************************************************************************************
ValueParam GenGetValue( ParamGen id_param ) {

    ValueParam value;
    
    value.uint32 = NULL;
    if ( id_param == GEN_PAR_REMOTE )
        value.uint8 = gen_ptr->remote;
    if ( id_param == GEN_PAR_CONN ) {
        if ( gen_ptr->remote )
            value.uint8 = LINK_CONN_OK;
        else value.uint8 = gen_ptr->connect;
       }
    if ( id_param == GEN_PAR_MODE )
        value.uint8 = gen_ptr->mode;
    if ( id_param == GEN_PAR_STAT )
        value.uint8 = gen_ptr->stat;
    if ( id_param == GEN_PAR_TM_RUN ) {
        if ( gen_ptr->mode == GEN_MODE_RUN || gen_ptr->mode == GEN_MODE_TEST )
            value.uint16 = gen_ptr->timer_run_inc;
        else value.uint16 = 0;
       }
    if ( id_param == GEN_PAR_TM_END ) {
        if ( gen_ptr->mode == GEN_MODE_RUN || gen_ptr->mode == GEN_MODE_TEST )
            value.uint16 = gen_ptr->timer_run_dec;
        else value.uint16 = 0;
       }
    if ( id_param == GEN_PAR_START_STOP ) {
        if ( gen_ptr->timer_lost_acmain )
            value.uint16 = gen_ptr->timer_lost_acmain;
        if ( gen_ptr->timer_rest_acmain )
            value.uint16 = gen_ptr->timer_rest_acmain;
       }
    if ( id_param == GEN_PAR_SLEEP )
        value.uint16 = gen_ptr->timer_sleep;
    if ( id_param == GEN_PAR_AUTO )
        value.uint8 = gen_ptr->auto_mode;
    if ( id_param == GEN_PAR_ALT )
        value.uint8 = AltGetValue( ALT_GEN_ON ).uint8;
    if ( id_param == GEN_PAR_ERROR )
        value.uint8 = gen_ptr->error;
    if ( id_param == GEN_PAR_CYCLE1 )
        value.uint8 = gen_ptr->cycle1 + 1;
    if ( id_param == GEN_PAR_CYCLE2 )
        value.uint8 = gen_ptr->cycle2;
    return value;
 }
