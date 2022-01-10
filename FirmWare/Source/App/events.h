
//*************************************************************************************************
//
// Константы сообщений
//
//*************************************************************************************************

#ifndef __EVENTS_H
#define __EVENTS_H

#include "cmsis_os2.h"

//*************************************************************************************************
// ID объектов RTOS
//*************************************************************************************************
extern osMessageQueueId_t sound_msg, hmi_msg, modbus_queue;

extern osEventFlagsId_t spa_event, pv_event, job_event, alt_event, out_event, uart_event;
extern osEventFlagsId_t mppt_event, batmon_event, info_event, inv1_event, inv2_event;
extern osEventFlagsId_t command_event, soc_event, charge_event, trc_event, gen_event;

//*************************************************************************************************
// Флаги событий
//*************************************************************************************************
#define EVN_RTC_IRQ             0x10000000  //прерывание от RTC
#define EVN_RTC_SECONDS         0x01000000  //секундный интервал от RTC
#define EVN_RTC_1MINUTES        0x02000000  //минутный интервал от RTC
#define EVN_RTC_5MINUTES        0x04000000  //5-минутный интервал от RTC

//*************************************************************************************************
//события обрабатываемые в задаче "Batmon"
#define EVN_BATMON_RECV         0x00000001  //прием данных по UART
#define EVN_BATMON_PAUSE1       0x00000002  //таймер паузы между пакетами данных
#define EVN_BATMON_PAUSE2       0x00000004  //таймер отсутствия обмена данными
#define EVN_BATMON_LOG          0x00000008  //таймер интервальной записи данных
#define EVN_BATMON_MASK         EVN_RTC_SECONDS | EVN_BATMON_RECV | EVN_BATMON_PAUSE1 | EVN_BATMON_PAUSE2 | EVN_BATMON_LOG

//*************************************************************************************************
//события обрабатываемые в задаче "Mppt"
#define EVN_MPPT_RECV           0x00000010  //прием данных по UART
#define EVN_MPPT_PAUSE1         0x00000020  //таймер паузы между пакетами данных
#define EVN_MPPT_PAUSE2         0x00000040  //таймер отсутствия обмена данными
#define EVN_MPPT_LOG            0x00000080  //таймер интервальной записи данных
#define EVN_MPPT_MASK           EVN_RTC_SECONDS | EVN_MPPT_RECV | EVN_MPPT_PAUSE1 | EVN_MPPT_PAUSE2 | EVN_MPPT_LOG

//*************************************************************************************************
//события обрабатываемые в задаче "Charger"
#define EVN_CHARGER_CHK         0x00000100  //таймер задержки проверки контроллера
#define EVN_CHARGER_CURR        0x00000200  //таймер задержки стабилизации тока
#define EVN_CHARGER_LOG         0x00000400  //таймер интервальной записи данных
#define EVN_CHARGER_OFF         0x00000800  //событие на отключение контроллера заряда
#define EVN_CHARGER_MASK        EVN_RTC_SECONDS | EVN_CHARGER_CHK | EVN_CHARGER_CURR | EVN_CHARGER_LOG | EVN_CHARGER_OFF

//*************************************************************************************************
//события обрабатываемые в задаче "Pv"
#define EVN_PV_CHECK            0x00001000  //событие проверки включения контактора
#define EVN_PV_MASK             EVN_RTC_5MINUTES | EVN_PV_CHECK

//*************************************************************************************************
//события обрабатываемые в задаче "Alt"
#define EVN_ALT_TIMER           0x00000001  //восстановление состояния реле управляющего контактором К5
#define EVN_ALT_AC_LOST         0x00000002  //основная сеть отключена
#define EVN_ALT_AC_REST         0x00000004  //основная сеть восстановлена
#define EVN_ALT_MASK            EVN_ALT_TIMER  

//*************************************************************************************************
//события обрабатываемые в задаче "Uart"
#define EVN_UART_BUSY           0x00000002  //событие завершения передачи по UART

//*************************************************************************************************
//события обрабатываемые в задаче "Command"
#define EVN_COMMAND_CR          0x00000001  //событие нажатия клавиши Enter 
#define EVN_COMMAND_ESC         0x00000002  //событие нажатия клавиши Esc
#define EVN_COMMAND_CMD         0x00000004  //событие выполнения пакетного файла
#define EVN_COMMAND_MASK        EVN_COMMAND_CR | EVN_COMMAND_ESC | EVN_COMMAND_CMD  

//*************************************************************************************************
//события обрабатываемые в задаче "Rs485"
#define EVN_RS485_IRQ           0x00000001  //событие прерывания от MAX3100
#define EVN_RS485_MASK          EVN_RS485_IRQ

//*************************************************************************************************
//события обрабатываемые в задаче "Informing"
#define EVN_INFO_BAT            0x00000001  //событие сообщения о уровне заряда
#define EVN_INFO_INV            0x00000002  //событие сообщения о мощности нагрузки
#define EVN_INFO_TTG            0x00000004  //событие сообщения о продолжительности работы
#define EVN_INFO_MASK           EVN_RTC_SECONDS | EVN_RTC_5MINUTES | EVN_INFO_BAT | EVN_INFO_INV | EVN_INFO_TTG

//*************************************************************************************************
//события обрабатываемые в задаче "Soc"
#define EVN_SOC_CHARGE          0x00000001  //включаем подзарядку от PB-1000-224
#define EVN_SOC_MASK            EVN_RTC_1MINUTES | EVN_SOC_CHARGE  

//*************************************************************************************************
//события обрабатываемые в задаче "Modbus"
#define MSG_MODBUS_SEND         0x00010000  //передача пакета, младший два байт содержит размер 
                                            //передаваемого блока
#define MSG_MODBUS_RECV         0x00020000  //весь пакет от уст-ва получен
#define MSG_MODBUS_CHECK        0x00040000  //предварительная проверка принимаемого пакета данных
#define MSG_MODBUS_TIMEOUT      0x00080000  //вышло время ожидания ответа
#define MSG_MODBUS_MASK_DATA    0x0000FFFF  //маска для выделения размера передаваемого блока

//*************************************************************************************************
//события обрабатываемые в задаче "Tracker"
#define EVN_TRC_SUN_ON          0x00000001  //освещенность восстановлена
#define EVN_TRC_SUN_OFF         0x00000002  //пропала освещенность
#define EVN_TRC_SUN_POS         0x00000004  //позиционирование по данным SPA
#define EVN_TRC_LOG             0x00000008  //интервальное логирование
#define EVN_TRC_MASK            EVN_RTC_1MINUTES | EVN_RTC_5MINUTES | EVN_TRC_SUN_ON | EVN_TRC_SUN_OFF | EVN_TRC_SUN_POS | EVN_TRC_LOG

//*************************************************************************************************
//события обрабатываемые в задаче "Generator"
#define EVN_GEN_LOG             0x00010000  //логирование состояния генератора
#define EVN_GEN_CONSOLE         0x00020000  //вывод в консоль результата выполнения команды
#define EVN_GEN_CHECK_OFF       0x00040000  //выключение схемы контроля
#define EVN_GEN_CYCLE_NEXT      0x00080000  //следующий шаг циклограммы
#define EVN_GEN_MASK            EVN_RTC_SECONDS | EVN_GEN_LOG | EVN_GEN_CONSOLE | EVN_GEN_CHECK_OFF | EVN_GEN_CYCLE_NEXT | EVN_ALT_AC_LOST | EVN_ALT_AC_REST

//*************************************************************************************************
//события обрабатываемые в задаче "Invertor"
#define EVN_INV_LOG             0x00010000  //логирование состояния генератора
#define EVN_INV_CONSOLE         0x00020000  //вывод в консоль результата выполнения циклограммы
#define EVN_INV_STATUS          0x00040000  //чтение данных инвертора
#define EVN_INV_CYCLE_NEXT      0x00080000  //следущий шаг циклограммы
#define EVN_INV_STARTUP         0x00100000  //запрос статуса если инверторы уже включены
#define EVN_INV_RECV            0x00200000  //приняты данные от инвертора
#define EVN_INV_MASK            EVN_RTC_SECONDS | EVN_INV_LOG | EVN_INV_CONSOLE | EVN_INV_STATUS | EVN_INV_CYCLE_NEXT | EVN_INV_STARTUP | EVN_INV_RECV

#endif
