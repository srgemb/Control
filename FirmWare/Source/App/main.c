
#include <stdio.h>
#include <stdbool.h>

#include "rl_fs.h"
#include "cmsis_os2.h"
#include "RTE_Components.h"

#include "gpio_lpc17xx.h"
#include "lpc177x_8x_wwdt.h"

#ifdef RTE_Compiler_EventRecorder
#include "EventRecorder.h"
#endif

/*#include "segger_rtt.h"
#include "segger_sysview.h"
#include "segger_sysview_conf.h"*/

#include "config.h"
#include "ports.h"
#include "gen.h"
#include "alt.h"
#include "mppt.h"
#include "main.h"
#include "rtc.h"
#include "command.h"
#include "charger.h"
#include "batmon.h"
#include "system.h"
#include "message.h"
#include "inverter.h"
#include "pv.h"
#include "eeprom.h"
#include "uart.h"
#include "reserv.h"
#include "tracker.h"
#include "outinfo.h"
#include "dac.h"
#include "spa_calc.h"
#include "rs485.h"
#include "sdcard.h"
#include "hmi_can.h"
#include "scheduler.h"
#include "modbus.h"
#include "message.h"
#include "informing.h"

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
uint32_t reset;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t init_attr = {
    .name = "Init", 
    .stack_size = 2048,
    .priority = osPriorityNormal };
    
//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void ResetLog( void );
static void WDTInit( void );
static void TaskInit( void *pvParameters );

//*************************************************************************************************
// MAIN ....
//*************************************************************************************************
int main ( void ) {
    
    reset = LPC_SC->RSID;
    
    NVIC_SetPriorityGrouping( 3 );

    #if defined( DEBUG_VERSION ) && defined( RTE_Compiler_EventRecorder )
    EventRecorderInitialize( EventRecordAll, 1U );
    EventRecorderEnable( EventRecordError, 0xF0U, 0xF8U ); //RTOS Events
    EventRecorderEnable( EventRecordAll, 0xF2U, 0xF2U );   //Thread Events
    #endif

    osKernelInitialize();
    //создаем задачу инициализации
    osThreadNew( TaskInit, NULL, &init_attr );
    //стартуем OS
    if ( osKernelGetState() == osKernelReady )
        osKernelStart();
    for ( ;; );
 }

//*************************************************************************************************
// Задача инициализации 
//*************************************************************************************************
static void TaskInit( void *pvParameters ) {

    WDTInit();          //включим WDT
    EepromInit();       //инициализация EEPROM, загрузка параметров настроек
    PortsInit();        //порты управления/состояния
    ReservInit();       //порты управления дополнительными реле/выходами

    RTCInit();          //часы реального времени RTC
    UartInit();         //UART консоли
    ScreenInit();       //инициализация экран консоли
    CommandInit();      //командный интерфейс
    SDMount();          //монтирование SD карты
    ResetLog();         //логирование источника сброса контроллера
    CANInit();          //инициализация CAN интерфейса
    RS485Init();        //интерфейс RS-485 (MODBUS) SSP1
    ModBusInit();       //управление MODBUS

    AltInit();          //управление блоком АВР
    PvInit();           //управление коммутацией солнечных панелей
    ChargeInit();       //управление контроллером заряда PB-1000-224 SSP0
    BatMonInit();       //монитор АКБ BMV-600S (UART)
    MpptInit();         //связь с контроллером заряда MPPT (UART/RS485)
    DACInit();          //ЦАП (звуковые сообщения)
    JobInit();          //планировщик заданий
    SPAInit();          //расчет текущих значений положения солнца
    VoiceInit();        //управление голосовым информатором
    TrackerInit();      //управление трекером
    GenInit();          //управление генератором (локально/MODBUS)
    SocInit();          //контроль уровня SOC, режимов заряд/разряд
    InvInit();          //управление инверторами TS-1000-224, TS-3000-224
    
    ConsoleSend( Message( CONS_MSG_PROMPT ), CONS_NORMAL );
    osThreadTerminate( osThreadGetId() );
 }
 
//*************************************************************************************************
// Сохраним в файл флаги источника сброса контроллера
//*************************************************************************************************
static void ResetLog( void ) {

    FILE *log;

    if ( SDStatus() == ERROR )
        return; //карты нет
    log = fopen( "startup.log", "a" );
    if ( log == NULL )
        return;
    fprintf( log, "%s ", RTCGetDateTime( NULL ) );
    if ( reset & RESET_POR )
        fprintf( log, "POR " );      //сброс при включении
    if ( reset & RESET_EXTR )
        fprintf( log, "EXTR " );     //сброс по кнопке
    if ( reset & RESET_WDTR )
        fprintf( log, "WDTR " );     //сброс от WDT
    if ( reset & RESET_BODR )
        fprintf( log, "BODR " );     //сброс при понижении напряжения питания
    if ( reset & RESET_SYSRESET )
        fprintf( log, "SYSRESET " ); //системный сброс
    if ( reset & RESET_LOCKUP )
        fprintf( log, "LOCKUP" );    //сброс из-за «блокировки»
    fprintf( log, "\r\n" );
    fclose( log );
 }

//*************************************************************************************************
// Инициализация WDT для режимов MODE1 != OFF MODE2 != OFF
//*************************************************************************************************
static void WDTInit( void ) {

    #ifdef DEBUG_VERSION
    if ( !CPUMode() )
        return;
    #endif
    WWDT_Init( _WDT_TIMEOUT );
    WWDT_Enable( ENABLE );
    WWDT_SetMode( WWDT_RESET_MODE, ENABLE );
    WWDT_Start( _WDT_TIMEOUT );
    WWDT_Feed();
 }

//*************************************************************************************************
// Перезапуск WDT
//*************************************************************************************************
void WDTReset( void ) {

    #ifdef DEBUG_VERSION
    if ( !CPUMode() )
        return;
    #endif
    WWDT_FeedStdSeq();
 }

