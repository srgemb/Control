
#ifndef __PORTS_H
#define __PORTS_H

#include <stdbool.h>
#include <lpc177x_8x.h>

#include "gpio_lpc17xx.h"
#include "lpc177x_8x_timer.h"

//*************************************************************************************************
// Глобальные переменные
//*************************************************************************************************
#define LED_ON              1               //включить светодиод
#define LED_OFF             0               //выключить светодиод

#define RS485_RECV          0               //режим приема для MPPT
#define RS485_SEND          1               //режим передачи для MPPT

//*************************************************************************************************
// Выходы управления
//*************************************************************************************************
//порты
#define PORT0               0
#define PORT1               1
#define PORT2               2
#define PORT3               3
#define PORT4               4
#define PORT5               5

//выходы управления генератором (PORT2)
#define CHECK_PORT          PORT2
#define CHECK_LED           2

//выходы управления генератором (PORT1)
#define GEN_PORT            PORT1
#define GEN_ON              9
#define GEN_CHK             10
#define GEN_START           31

//выходы управления контакторами инверторов (PORT1)
#define TS_CTRL_PORT        PORT1
#define TS1000_ON           18
#define TS1000_OFF          19
#define TS3000_ON           20
#define TS3000_OFF          21

//выходы управления АВР (PORT1)
#define ALT_PORT            PORT1
#define ALT_AC_OFF          17
#define ALT_OFF             16

//выходы управления PB-1000 (PORT1)
#define AC_CHARGE_PORT      PORT1
#define AC_CHARGE_OFF       30
//выходы управления PB-1000 (PORT4)
#define AC_CHARGE_MODE      PORT4
#define AC_CHARGE_MODE2     5
#define AC_CHARGE_MODE8     4

//выход управления трекером (PORT1)
#define TRC_PORT            PORT1
#define TRC_ON              24

//выходы управления солнечными панелями (PORT1)
#define PV_PORT1            PORT1
#define PVK_ON              22              //включить панели
#define PV_MODE             23              //параллельное/последовательное соединение
//выходы управления солнечными панелями (PORT4)
#define PV_PORT2            PORT4      
#define PVK_OFF             0               //выключить панели

//выход управления RS-485 MPPT (PORT3)
#define MPPT_PORT           PORT3      
#define MPPT_CTRL           25              //прием/передача данных RS-485 MPPT

//выходы дополнительного управления (PORT1)
#define RES_PORT            PORT1
#define RESERV1             15
#define RESERV2             14
#define RESERV3             25
//выходы дополнительного управления (PORT4)
#define RES_PORT4           PORT4
#define RESERV4             1

//выходы дополнительного управления (PORT1)
#define EXT_PORT            PORT1
#define EXT_UP              27
#define EXT_DN              26
#define EXT_LF              29
#define EXT_RT              28

//входы дополнительные
#define CPU_MODE1_IN        13
#define CPU_MODE2_IN        12

//*************************************************************************************************
// Маски статусов (входов)
//*************************************************************************************************
#define STAT_ALL            0xFFFFFFFF

//статусы входов генератора
#define GEN_CONN            0x00040000      //генератор подключен
#define GEN_RUN             0x02000000      //генератор запущен
#define GEN_OVR             0x00080000      //перегрузка генератора
#define GEN_OIL             0x00100000      //низкий уровень масла
#define GEN_LOW_BAT         0x00200000      //низкий заряд батареи
#define GEN_FUEL_LOW        ( GEN_OVR | GEN_OIL ) 
#define GEN_ALL_STAT        ( GEN_CONN | GEN_RUN | GEN_OVR | GEN_OIL | GEN_LOW_BAT )

//реле монитора батареи
#define BATMON_KEY          0x80000000      //реле монитора батареи

//статусы зарядного уст-ва
#define CHARGE_AC_OK        0x20000000      //есть основная сеть
#define CHARGE_BANK_OK      0x40000000      //зарядка завершена
#define CHARGE_DEV_OK       0x00000004      //уст-во исправно
#define CHARGE_ALL_STAT     ( CHARGE_AC_OK | CHARGE_BANK_OK | CHARGE_OK )

//реле солнечного контроллера заряда
#define MPPT_K1             0x10000000      //реле 1 контроллера
#define MPPT_K2             0x08000000      //реле 2 контроллера 
#define MPPT_CONN           0x04000000      //порт RS485 подключен
#define MPPT_ON             0x00400000      //контроллер вкл
#define MPPT_PV_ON          0x00000002      //панели подключены
#define MPPT_ALL_STAT       ( MPPT_K1 | MPPT_K2 | MPPT_CONN | MPPT_ON | MPPT_PV_ON )

//контроль автомата солнечного трекера
#define TRC_FUSE            0x00004000      //автомат защиты трекера
#define TRC_PWR             0x00002000      //питание трекера вкл

//контроль подключения инверторов и нагрузки
#define TS3000_CHK          0x00000400      //контактор инвертора TS-3000-224 включен
#define TS1000_CHK          0x00000200      //контактор инвертора TS-1000-224 включен
#define TS1000_LOC          0x00000010      //инвертора TS-1000-224 - заблокирован, есть нагрузка
#define TS3000_LOC          0x00000008      //инвертора TS-3000-224 - заблокирован, есть нагрузка
#define TS_ALL_STAT         ( TS3000_CHK | TS1000_CHK | TS1000_LOC | TS3000_LOC )

//резервные входы
#define RES_CHK1            0x00000020
#define RES_CHK2            0x00000040
#define RES_CHK3            0x00000080
#define RES_CHK4            0x00000100
#define RES_ALL_STAT        ( RES_CHK1 | RES_CHK2 | RES_CHK3 | RES_CHK4 )

//контрольные входы
#define TRC_CH_RT           0x00020000
#define TRC_CH_LF           0x00010000
#define TRC_CH_DN           0x00008000
#define TRC_CH_UP           0x00000800
#define TRC_ALL_STAT        ( TRC_CH_UP | TRC_CH_DN | TRC_CH_LF | TRC_CH_RT )

//состояние блока АВР
#define ALT_GEN_OK          0x01000000      //генератор включен
#define ALT_AC_GEN          0x00000001      //нагрузка подключена к генератору
#define ALT_AC_MAIN         0x00800000      //нагрузка подключена к основной сети
#define ALT_CONNECT         0x00001000      //блок АВР подключен
#define ALT_ALL_STAT        ( ALT_GEN_OK | ALT_AC_GEN | ALT_AC_MAIN | ALT_CONNECT ) 

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void PortsInit( void );

//*************************************************************************************************
// Функции статуса/состояния
//*************************************************************************************************
uint8_t CPUMode( void );
uint8_t ExtInt( void );
uint32_t GetDataPort( uint32_t mask );
bool Fuse24Vdc( void );
//bool StatUZFS( void );
bool BatConn( void );
bool StatCtrl( void );
bool SDDetect( void );

#endif 
