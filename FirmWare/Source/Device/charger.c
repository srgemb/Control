
//*************************************************************************************************
//
// Управление контроллером заряда PB-1000-224
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "cmsis_os2.h"

#include "ssp_lpc17xx.h"

#include "device.h"
#include "dev_data.h"

#include "main.h"
#include "rtc.h"
#include "ports.h"
#include "eeprom.h"
#include "outinfo.h"
#include "charger.h"
#include "command.h"
#include "sdcard.h"
#include "config.h"
#include "informing.h"
#include "priority.h"
#include "message.h"
#include "events.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern ARM_DRIVER_SPI Driver_SPI0;

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
CHARGER charger;
osEventFlagsId_t charge_event;

//*************************************************************************************************
// Локальные константы для расчета тока на шунте
//*************************************************************************************************
#define VREF                2.485           //опорное напряжение (V)
#define ADC_BIT             (1 << 12)       //разрядность АЦП 12 бит (4096)
#define I_SHUNT             50.000          //максимальный ток шунта (А)
#define V_DROP              0.0750          //падение напряжения на шунте (V)
#define GAIN_AMPLF          15.889          //коэффициент усиления операционного усилителя

//*************************************************************************************************
// Временные параметры контроля работы PB-1000-224
//*************************************************************************************************
#define TIME_CHECK_DEV      5               //время задержки проверки сигнала исправности PB-1000-224 (sec)
#define TIME_STAB_CURR      10              //время продолжительности стабилизации тока заряда PB-1000-224 (sec)
#define TIME_CHECK_CURR     900             //продолжительность заряда после снижения тока ниже 
                                            //значения в режиме "Отключение по току" (sec)

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static ARM_DRIVER_SPI *SpiDrv0;
static bool enable_calc = false;
static float current;
static osTimerId_t timer1_charger, timer2_charger, timer3_charger, timer4_charger;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t charger_attr = {
    .name = "Charger", 
    .stack_size = 1024,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn_attr = { .name = "Charger" };
static const osTimerAttr_t timer1_attr = { .name = "ChrgCheck" };
static const osTimerAttr_t timer2_attr = { .name = "ChrgStabCurr" };
static const osTimerAttr_t timer3_attr = { .name = "ChrgOff" };
static const osTimerAttr_t timer4_attr = { .name = "ChrgLog" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void ChargeSetMode( ChargeMode mode );
static void EventLog( char *text, ChargeError error );
static void SaveLog( void );
static void SaveDate( void );
static float ChargeCurrent( void );
static void TaskCharger( void *pvParameters );
static void CheckCharge( void );
static void CalcCurrent( void );
static void Timer1Callback( void *arg );
static void Timer2Callback( void *arg );
static void Timer3Callback( void *arg );
static void Timer4Callback( void *arg );

//*************************************************************************************************
// Инициализация портов
//*************************************************************************************************
void ChargeInit( void ) {

    //режим портов "выход"
    GPIO_SetDir( AC_CHARGE_PORT, AC_CHARGE_OFF,   GPIO_DIR_OUTPUT );
    GPIO_SetDir( AC_CHARGE_MODE, AC_CHARGE_MODE2, GPIO_DIR_OUTPUT );
    GPIO_SetDir( AC_CHARGE_MODE, AC_CHARGE_MODE8, GPIO_DIR_OUTPUT );
    //очередь сообщений
    charge_event = osEventFlagsNew( &evn_attr );
    //таймер задержки проверки исправности уст-ва
    timer1_charger = osTimerNew( Timer1Callback, osTimerOnce, NULL, &timer1_attr );
    //таймер стабилизации тока зарядки после включения
    timer2_charger = osTimerNew( Timer2Callback, osTimerOnce, NULL, &timer2_attr );
    //таймер выключения по току зарядки
    timer3_charger = osTimerNew( Timer3Callback, osTimerOnce, NULL, &timer3_attr );
    //таймер интервальной записи данных
    timer4_charger = osTimerNew( Timer4Callback, osTimerOnce, NULL, &timer4_attr );
    //создаем задачу управления зарядкой PB-1000-224
    osThreadNew( TaskCharger, NULL, &charger_attr );
    //инициализация SPI0 интерфейса АЦП ADS1286
    SpiDrv0 = &Driver_SPI0;
    SpiDrv0->Initialize( NULL );
    NVIC_SetPriority( SSP0_IRQn, NVIC_EncodePriority( NVIC_GetPriorityGrouping(), PRIORITY_SSP, SUB_PRIORITY_SSP0 ) );
    SpiDrv0->PowerControl( ARM_POWER_FULL );
    SpiDrv0->Control( ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_MSB_LSB | ARM_SPI_SS_MASTER_SW | ARM_SPI_DATA_BITS( 15 ), 10000 );
    SpiDrv0->Control( ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE );
    //восстанавливаем предыдущее состояние режима заряда
    Charger( (ChargeMode)EepromLoad( EEP_BP_CHARGE ), EEPROM_RESTORE );
 }

//*************************************************************************************************
// Задача управления работой контроллера заряда PB-1000-224
//*************************************************************************************************
static void TaskCharger( void *pvParameters ) {

    uint32_t send, event;

    for ( ;; ) {
        //запуск таймера логирования данных
        if ( !osTimerIsRunning( timer4_charger ) )
            osTimerStart( timer4_charger, config.datlog_upd_chrg * SEC_TO_TICK );
        //ждем события
        event = osEventFlagsWait( charge_event, EVN_CHARGER_MASK, osFlagsWaitAny, osWaitForever );
        if ( event & EVN_RTC_SECONDS ) {
            //заполняем структуру данными состояния контроллера заряда
            charger.connect_ac = !GetDataPort( CHARGE_AC_OK ) ? POWER_AC_OFF : POWER_AC_ON;
            charger.device_ok = GetDataPort( CHARGE_DEV_OK ) ? DEVICE_OK : DEVICE_FAULT;
            charger.charge_mode = ChargeGetMode();
            charger.charge_end = GetDataPort( CHARGE_BANK_OK ) ? CHARGE_FULL : CHARGE_PROGSS;
            charger.current = ChargeCurrent();
            CalcCurrent();  //расчет тока зарядки
            CheckCharge();  //проверяем завершение зарядки
            send = ID_DEV_CHARGER; //передача данных в HMI
            osMessageQueuePut( hmi_msg, &send, 0, 0 );
           }
        if ( event & EVN_CHARGER_CHK ) {
            if ( !GPIO_PinRead( AC_CHARGE_PORT, AC_CHARGE_OFF ) ) {
                //проверка наличие сигнала: "Уст-во исправно"
                if ( !GetDataPort( CHARGE_DEV_OK ) ) {
                    //зарядное уст-во выдало ошибку
                    Charger( CHARGE_OFF, EEPROM_SAVE );
                    ConsoleSend( Message( CONS_MSG_CRLF ), CONS_SELECTED );
                    ConsoleSend( ErrorDescr( ID_DEV_CHARGER, 0, CHARGE_ERR_DEVICE ), CONS_SELECTED );
                    ConsoleSend( Message( CONS_MSG_CRLF ), CONS_SELECTED );
                    EventLog( NULL, CHARGE_ERR_DEVICE );
                    charger.error = CHARGE_ERR_DEVICE;
                  }
                else osTimerStart( timer2_charger, TIME_STAB_CURR * SEC_TO_TICK );
               }
            charger.charge_exec = COMMAND_END;
           }
        if ( event & EVN_CHARGER_CURR ) {
            enable_calc = true; //разрешаем измерение тока шунта
           }
        if ( event & EVN_CHARGER_OFF ) {
            //зарядка завершена
            Charger( CHARGE_OFF, EEPROM_SAVE );
            SaveDate();
            EventLog( MessageLog( ID_DEV_CHARGER, LOG_MSG_CHRG_END1 ), CHARGE_ERR_OK );
            Informing( VOICE_CHARGE_END, NULL );
           }
        if ( event & EVN_CHARGER_LOG ) {
            osTimerStart( timer4_charger, config.datlog_upd_chrg * SEC_TO_TICK );
            SaveLog(); //логирование данных
           }
       }
 }

//*************************************************************************************************
// Функция обратного вызова таймера - проверки готовности уст-ва
//*************************************************************************************************
static void Timer1Callback( void *arg ) {

    osEventFlagsSet( charge_event, EVN_CHARGER_CHK );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - стабилизации тока заряда
//*************************************************************************************************
static void Timer2Callback( void *arg ) {

    osEventFlagsSet( charge_event, EVN_CHARGER_CURR );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - выключение зарядки
//*************************************************************************************************
static void Timer3Callback( void *arg ) {

    osEventFlagsSet( charge_event, EVN_CHARGER_OFF );
 }

//*************************************************************************************************
// Функция обратного вызова таймера - интервальная запись данных
//*************************************************************************************************
static void Timer4Callback( void *arg ) {

    osEventFlagsSet( charge_event, EVN_CHARGER_LOG );
 }

//*************************************************************************************************
// Включение/выключения режима заряда
// Менять режим заряда 2/3/8 нельзя в процессе заряда нельзя, т.е. сначала выкл, 
// потом задать новый режим заряда
// ChargeMode charge  - режим заряда
// EepromMode restore - сохранение/восстановление параметра в/из EEPROM
// При восстановлении заряда после перезапуска контроллера и отключенной АКБ контроллер заряда
// PB-1000-224 не переходит в режим заряда, для восстановления режима необходимо выкл/вкл
// контроллер PB-1000-224 по сети AC
//*************************************************************************************************
ChargeError Charger( ChargeMode charge, EepromMode restore ) {

    char *log = NULL;
    ChargeMode chrg_mode;
    
    charger.charge_exec = COMMAND_END;
    charger.error = CHARGE_ERR_OK;
    if ( restore == EEPROM_RESTORE ) {
        //восстанавливаем состояние до перезагрузки
        chrg_mode = (ChargeMode)EepromLoad( EEP_BP_CHARGE );
        if ( chrg_mode != CHARGE_OFF ) {
            enable_calc = true; //сразу включим измерение тока
            log = MessageLog( ID_DEV_NULL, LOG_MSG_CHRG_RSTART );
           }
       }
    else chrg_mode = charge;
    //восстанавливаем управление
    if ( log == NULL )
        log = MessageLog( ID_DEV_CHARGER, LOG_MSG_CHRG_START );
    if ( chrg_mode == CHARGE_OFF ) {
        //выключаем зарядку
        ChargeSetMode( chrg_mode );
        GPIO_PinWrite( AC_CHARGE_PORT, AC_CHARGE_OFF, RELAY_ON );   //выключаем
        if ( restore == EEPROM_SAVE ) {
            //сохраним текущее состояние "Выключено"
            EepromUpdate( EEP_BP_CHARGE, chrg_mode );
            EepromSave();
           }
        charger.error = CHARGE_ERR_OK;
        charger.charge_exec = COMMAND_END;
        return CHARGE_ERR_OK;
       } 
    else {
        //включаем зарядку, сброс ошибки
        charger.error = CHARGE_ERR_OK;
        charger.charge_exec = COMMAND_END;
        //проверка подключения АКБ
        if ( !BatConn() ) {
            GPIO_PinWrite( AC_CHARGE_PORT, AC_CHARGE_OFF, RELAY_ON ); //выключаем
            charger.error = CHARGE_ERR_NOBAT;
            EventLog( log, CHARGE_ERR_NOBAT );
            return CHARGE_ERR_NOBAT;
           } 
        //проверка сети AC
        if ( !GetDataPort( CHARGE_AC_OK ) ) {
            GPIO_PinWrite( AC_CHARGE_PORT, AC_CHARGE_OFF, RELAY_ON ); //выключаем
            charger.error = CHARGE_ERR_NOAC;
            EventLog( log, CHARGE_ERR_NOAC );
            return CHARGE_ERR_NOAC;
           } 
        //установка режима
        charger.charge_exec = COMMAND_EXEC;
        ChargeSetMode( chrg_mode );
        GPIO_PinWrite( AC_CHARGE_PORT, AC_CHARGE_OFF, RELAY_OFF ); //включаем
        //сохраним текущее состояние
        EepromUpdate( EEP_BP_CHARGE, chrg_mode );
        EepromSave();
        //включаем таймер ожидания сигнала готовности контроллера заряда
        osTimerStart( timer1_charger, TIME_CHECK_DEV * SEC_TO_TICK );
        EventLog( log, CHARGE_ERR_OK );
        return CHARGE_ERR_OK;
       } 
 } 

//*************************************************************************************************
// Управление реле режимом заряда контроллера заряда PB-1000-224
// ChargeMode mode - режим заряда
//*************************************************************************************************
static void ChargeSetMode( ChargeMode mode ) {

    if ( mode == CHARGE_MODE2 ) {
        GPIO_PinWrite( AC_CHARGE_MODE, AC_CHARGE_MODE2, RELAY_ON );
        GPIO_PinWrite( AC_CHARGE_MODE, AC_CHARGE_MODE8, RELAY_OFF );
       } 
    if ( mode == CHARGE_MODE3 || mode == CHARGE_OFF ) {
        GPIO_PinWrite( AC_CHARGE_MODE, AC_CHARGE_MODE2, RELAY_OFF );
        GPIO_PinWrite( AC_CHARGE_MODE, AC_CHARGE_MODE8, RELAY_OFF );
       } 
    if ( mode == CHARGE_MODE8 ) {
        GPIO_PinWrite( AC_CHARGE_MODE, AC_CHARGE_MODE2, RELAY_OFF );
        GPIO_PinWrite( AC_CHARGE_MODE, AC_CHARGE_MODE8, RELAY_ON );
       } 
 }

//*************************************************************************************************
// Возвращает текущий режим зарядки контроллера PB-1000-224
// return ChargeMode = 0/2/3/8 - выкл/2/3/8
//*************************************************************************************************
ChargeMode ChargeGetMode( void ) {

    if ( !GPIO_PinRead( AC_CHARGE_PORT, AC_CHARGE_OFF ) )
        return (ChargeMode)EepromLoad( EEP_BP_CHARGE ); //идет зарядка, читаем режим заряда
    else return CHARGE_OFF;
 }

//*************************************************************************************************
// Возвращает ток заряда АКБ
// return float - ток заряда
//*************************************************************************************************
static float ChargeCurrent( void ) {

    if ( GPIO_PinRead( AC_CHARGE_PORT, AC_CHARGE_OFF ) )
        return 0; //заряда нет
    return current;
 }

//*************************************************************************************************
// При завершении заряда выключаем PB-1000-224
// Если glb_par.bp_current_stop = 0, зарядка выключается по сигналу CHARGE_BANK_OK
// Если glb_par.bp_current_stop > 0, зарядка выключается при достижении заданного значения
// Разрешение выполнения по секундному прерыванию RTC
//*************************************************************************************************
static void CheckCharge( void ) {

    uint8_t curr_charge;
    
    //проверка режима работы
    if ( GPIO_PinRead( AC_CHARGE_PORT, AC_CHARGE_OFF ) || !GetDataPort( CHARGE_AC_OK ) )
        return; //зарядка выключена, нет сети
    //определим режим отключения заряда: по току или по сигналу от зарядного уст-ва
    if ( config.pb_current_stop ) {
        //отключение по току в параметре config.bp_current_stop
        curr_charge = (uint8_t)ChargeCurrent();
        if ( GetDataPort( CHARGE_BANK_OK ) && curr_charge < config.pb_current_stop ) {
            if ( !osTimerIsRunning( timer3_charger ) )
                osTimerStart( timer3_charger, TIME_CHECK_CURR * SEC_TO_TICK ); //включаем таймер ожидания завершения зарядки
           }
       }
    else {
        //отключение по сигналу "BANK-A OK"
        if ( GetDataPort( CHARGE_BANK_OK ) ) {
            //Зарядка завершена, выключаем
            Charger( CHARGE_OFF, EEPROM_SAVE );
            SaveDate();
            EventLog( MessageLog( ID_DEV_CHARGER, LOG_MSG_CHRG_END2 ), CHARGE_ERR_OK );
            Informing( VOICE_CHARGE_END, NULL );
           }
       }
 }

//*************************************************************************************************
// Чтение-расчет тока заряда АКБ
//*************************************************************************************************
// VReal = ADC*(VRef/4096)
// VRef - ИОН
// 4096 - разрядность
// ADC0 - полученный результат со входа АЦП.
// X * ( VRef / 4096 )
//*************************************************************************************************
static void CalcCurrent( void ) {

    uint16_t value;
    float voltage;

    //проверка режима работы
    if ( GPIO_PinRead( AC_CHARGE_PORT, AC_CHARGE_OFF ) || !GetDataPort( CHARGE_AC_OK ) ) {
        //зарядка выключена, нет сети, неисправен блок заряда
        current = 0;
        enable_calc = false;
        return; 
       } 
    if ( enable_calc == false )
        return;        //запуск измерений не разрешен
    //чтение данных АЦП
    SpiDrv0->Control( ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE );
    SpiDrv0->Receive( &value, 1 );
    if ( ( value & 0x6000 ) == 0x6000 ) {
        //данные достоверны когда биты старшей тетрады содержат "1", иначе пропускаем
        value &= 0x0FFE; //уберем старшие биты, т.к. они не несут информации
        voltage = (((float)VREF/(float)ADC_BIT) * (float)value);
        //накапливаем данные для усреднения
        current = ((float)I_SHUNT/(float)V_DROP)*(voltage/GAIN_AMPLF);
       } 
    SpiDrv0->Control( ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE );
 }

//*************************************************************************************************
// Запись команд в протокол из внешних модулей
// char *text - текст сообщения
//*************************************************************************************************
void EventLogPB( char *text ) {

    EventLog( text, CHARGE_ERR_OK );
 }

//*************************************************************************************************
// Добавляет в протокол "pb_yyyymmdd.csv" данные заряда
//*************************************************************************************************
static void SaveLog( void ) {

    uint32_t pos;
    char name[80];
    FILE *bp_log;

    if ( !config.log_enable_chrg )
        return; //логирование выключено
    if ( SDStatus() == ERROR )
        return; //карты нет
    if ( ChargeGetMode() == CHARGE_OFF )
        return; //зарядка выкл
    if ( !config.mode_logging )
        sprintf( name, "\\charger\\pb_%s.csv", RTCFileName() );
    else sprintf( name, "\\charger\\%s\\pb_%s.csv", RTCFileShort(), RTCFileName() );
    bp_log = fopen( name, "a" );
    if ( bp_log == NULL )
        return; //файл не открылся
    pos = ftell( bp_log );
    if ( !pos )
        fprintf( bp_log, "Date;Time;AC;Dev;Mode;Stat;SOC(%%);I(A);V\r\n" ); //запишем наименование полей
    //запишем данные
    fprintf( bp_log, "%s;%s;%s;%s;%d;%s;%.1f;%4.1f;%.2f\r\n", RTCGetDate( NULL ), RTCGetTime( NULL ),
        ParamGetDesc( ID_DEV_CHARGER, CHARGE_CONN_AC ),
        ParamGetDesc( ID_DEV_CHARGER, CHARGE_DEV_STAT ),
        ChargeGetMode(), 
        ParamGetDesc( ID_DEV_CHARGER, CHARGE_BANK_STAT ),
        batmon.soc, ChargeCurrent(), batmon.voltage );
    fclose( bp_log );
 }

//*************************************************************************************************
// Запись команд в протокол
// char *text     - текстовая строка добавляемая в файл протокола
// uint32_t error - код ошибки добавляемый в файл протокола
//*************************************************************************************************
static void EventLog( char *text, ChargeError error ) {

    char name[80];
    FILE *log;
    
    if ( SDStatus() == ERROR )
        return; //карты нет
    if ( text == NULL )
        return;
    if ( !config.mode_logging )
        sprintf( name, "\\charger\\pb_%s.log", RTCFileName() );
    else sprintf( name, "\\charger\\%s\\pb_%s.log", RTCFileShort(), RTCFileName() );
    log = fopen( name, "a" );
    if ( log == NULL )
        return;
    if ( error )
        fprintf( log, "%s %s %s\r\n", RTCGetLog(), text, ErrorDescr( ID_DEV_CHARGER, 0, error ) );
    else fprintf( log, "%s %s\r\n", RTCGetLog(), text );
    fclose( log );
 }

//*************************************************************************************************
// Сохраним дату последнего цикла заряда
//*************************************************************************************************
static void SaveDate( void ) {

    static RTC_TIME_Type Time;
    
    RTC_GetFullTime( LPC_RTC, &Time );
    config.last_charge.day = Time.DOM;
    config.last_charge.month = Time.MONTH;
    config.last_charge.year = Time.YEAR;
    ConfigSave();
 }
