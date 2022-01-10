
//*************************************************************************************************
//
// Контроль уровня заряда АКБ (SOC), управление подзарядкой
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "device.h"
#include "dev_data.h"
#include "dev_param.h"

#include "main.h"
#include "alt.h"
#include "gen.h"
#include "ports.h"
#include "system.h"
#include "mppt.h"
#include "inverter.h"
#include "command.h"
#include "charger.h"
#include "informing.h"
#include "message.h"
#include "config.h"
#include "events.h"

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osEventFlagsId_t soc_event = NULL;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static osTimerId_t timer_charger;
static bool flg_gen = false, flg_soc = false;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t soc_attr = {
    .name = "Soc", 
    .stack_size = 1024,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn_attr = { .name = "Soc" };
static const osTimerAttr_t timer1_attr = { .name = "Soc" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void SocCheck( void );
static void CheckGen( void );
static void Timer1Callback( void *arg );
static void TaskSoc( void *pvParameters );

//*************************************************************************************************
// Инициализация
//*************************************************************************************************
void SocInit( void ) {

    //таймер задержки контроля 
    timer_charger = osTimerNew( Timer1Callback, osTimerOnce, NULL, &timer1_attr );
    //очередь сообщений
    soc_event = osEventFlagsNew( &evn_attr );
    //создаем задачу
    osThreadNew( TaskSoc, NULL, &soc_attr );
 }

//*************************************************************************************************
// Задача обработки событий контроля уровня заряда АКБ (SOC)
//*************************************************************************************************
static void TaskSoc( void *pvParameters ) {
    
    uint32_t event;
    
    for ( ;; ) {
        //ждем события
        event = osEventFlagsWait( soc_event, EVN_SOC_MASK, osFlagsWaitAny, osWaitForever );
        if ( event & EVN_RTC_1MINUTES ) {
            //интервал выполнения 1 минут
            SocCheck(); //контроль минимального уровня заряда АКБ (SOC)
            CheckGen(); //проверка запуска генератора
           }
        if ( event & EVN_SOC_CHARGE )
            Charger( CHARGE_MODE3, EEPROM_SAVE ); //включаем подзарядку от PB-1000-224
       }
 }

//*************************************************************************************************
// Функция обратного вызова таймера - задержка на включения контроллера заряда PB-1000-224
// в случае восстановления сети или запуска генератора
//*************************************************************************************************
static void Timer1Callback( void *arg ) {

    osEventFlagsSet( timer_charger, EVN_SOC_CHARGE );
 }

//*************************************************************************************************
// Контролирует минимальный уровень заряда АКБ (SOC).
// При низком уровне SOC, пороговый уровень устанавливается в BMV-600S параметром "DF" для включения
// реле в мониторе АКБ, если реле не включилось, проверяем порог SOC < _BMV_MIN_SOC (40%)
// и переводим нагрузку на основную сеть, если сети нет запускаем генератор. 
// Если подзарядка будет выключена, по тем или иным причинам, по флагу flg_soc подзарядка 
// будет восстанавливаться пока уровень SOC не будет достигнут значения _BMV_NORM_SOC
// Голосовое сообщение "VOICE_LOW_CHARGE" выдается пока не включится подзарядка от PB-1000-224 или MPPT
//*************************************************************************************************
static void SocCheck( void ) {

    if ( batmon.link == LINK_CONN_OK && batmon.soc >= ( (float)_BMV_NORM_SOC ) && flg_soc == true )
        flg_soc = false; //сбросим флаг контроля уровня SOC, т.к. АКБ уже заряжена
    if ( config.mode_sys == SYSTEM_MODE_TEST )
        return; //в режиме TEST не проверяем
    //проверим значение SOC на минимальное значение по данным BMV или сигнального реле BMV 
    if ( ( batmon.link == true && ( batmon.soc < ( (float)_BMV_MIN_SOC ) || GetDataPort( BATMON_KEY ) ) ) || flg_soc == true ) {
        flg_soc = true; //флаг контроля подзарядки при превышение уровня _BMV_MIN_SOC
        if ( MpptCheckPwr() == MPPT_POWER_ON && mppt.u08_charge_mode != MPPT_CHARGE_OFF )
            return; //MPPT включен и идет подзарядка от него, PB-1000-224 не подключаем
        if ( ChargeGetMode() != CHARGE_OFF )
            return; //включен PB-1000-224 идет подзарядка от него
        //голосовое сообщение выдается пока не включится подзарядка от PB-1000-224 или MPPT
        Informing( VOICE_LOW_CHARGE, NULL ); 
        if ( GetDataPort( ALT_AC_MAIN | ALT_AC_GEN ) ) {
            //есть основная сеть или включен генератор, переводим нагрузку на них
            AltPowerAC();
            //запуск таймера ожидания подключения к сети PB-1000-224
            osTimerStart( timer_charger, _TIME_WAIT_PB_AC * SEC_TO_TICK );
            if ( GetDataPort( ALT_AC_MAIN ) )
                EventLogPB( MessageLog( ID_DEV_CHARGER, LOG_MSG_SOC_BAT_CHRG_AC ) );
            if ( GetDataPort( ALT_AC_GEN ) )
                EventLogPB( MessageLog( ID_DEV_CHARGER, LOG_MSG_SOC_BAT_CHRG_GEN ) );
            //отключаем инверторы
            if ( InvBatConn( ID_DEV_INV1 ) == BAT_DC_ON )
                InvCtrl( ID_DEV_INV1, INV_CTRL_OFF );
            if ( InvBatConn( ID_DEV_INV2 ) == BAT_DC_ON )
                InvCtrl( ID_DEV_INV2, INV_CTRL_OFF );
           }
        else {
            //основной сети нет, пробуем запустить генератор
            GenStart();     
            flg_gen = true; //включим проверку запуска генератора
            EventLogPB( MessageLog( ID_DEV_GEN, LOG_MSG_SOC_GEN_START ) );
           }
       }
 }

//*************************************************************************************************
// Проверяем уровень заряда после восстановления сети и выключения инверторов, выполняется 
// только для режима MODE_SYSTEM_BACKUP, вызывается из функции CheckTimer() (alt.c)
//*************************************************************************************************
void BatSocCheck( void ) {

    if ( batmon.link == true && batmon.soc < ( (float)_BMV_CHARGE_SOC ) ) {
        //определим источник подзарядки AC/GEN - MPPT
        if ( MpptCheckPwr() == MPPT_POWER_ON && mppt.u08_charge_mode != MPPT_CHARGE_OFF )
            return; //MPPT включен и идет подзарядка от него, PB-1000-224 не подключаем
        //MPPT выключен или подзарядки от него нет
        if ( GetDataPort( ALT_AC_MAIN | ALT_AC_GEN ) ) {
            //запуск таймера ожидания подключения к сети контроллера заряда PB-1000-224
            osTimerStart( timer_charger, _TIME_WAIT_PB_AC * SEC_TO_TICK );
            if ( GetDataPort( ALT_AC_MAIN ) )
                EventLogPB( MessageLog( ID_DEV_CHARGER, LOG_MSG_SOC_CHRG_AC ) );
            if ( GetDataPort( ALT_AC_GEN ) )
                EventLogPB( MessageLog( ID_DEV_CHARGER, LOG_MSG_SOC_CHRG_GEN ) );
           }
       }
 }

//*************************************************************************************************
// Проверяем запуск генератора, при включении включаем подзарядку контроллером заряда PB-1000-224
//*************************************************************************************************
static void CheckGen( void ) {

    if ( flg_gen == false )
        return; //проверка выключена
    if ( GenCycleStart() == true )
        return; //циклограмма запуска генератора не завершена
    //проверим запуск генератора
    if ( gen_ptr->mode == GEN_MODE_RUN ) {
        //запуск таймера ожидания подключения к сети контроллера заряда PB-1000-224
        osTimerStart( timer_charger, _TIME_WAIT_PB_AC * SEC_TO_TICK );
        AltPowerAC();   //генератор запущен, переводим нагрузку на него
        //разрешим авто-выключение генератора при первом восстановлении сети
        GenAutoOnce();
       }
    flg_gen = false; //выключаем проверку
 }
