
//*************************************************************************************************
// Solar position algorithm: http://www.nrel.gov/midc/spa/
// Расчет времени и углов восхода захода солнца, продолжительности дня
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "device.h"

#include "rtc.h"
#include "spa.h"
#include "spa_calc.h"
#include "outinfo.h"
#include "eeprom.h"
#include "minini.h"
#include "events.h"

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
SUNPOS sunpos;
osEventFlagsId_t spa_event = NULL;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static spa_data spa;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t spa_attr = {
    .name = "SpaCalc",
    .stack_size = 1664,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn_spa = { .name = "SpaCalc" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void TaskSpa( void *pvParameters );

//*************************************************************************************************
// Инициализация
//*************************************************************************************************
void SPAInit( void ) {

    spa_event = osEventFlagsNew( &evn_spa );
    //создаем задачу 
    osThreadNew( TaskSpa, NULL, &spa_attr );
    //первичный расчет положения солнца
    osEventFlagsSet( spa_event, EVN_RTC_5MINUTES );
 }

//*************************************************************************************************
// Задача расчета положения солнца, продолжительности дня
// Выполняется с интервалом 5 минут по событию от RTC
//*************************************************************************************************
static void TaskSpa( void *pvParameters ) {

    uint32_t send;
    
    for ( ;; ) {
        osEventFlagsWait( spa_event, EVN_RTC_5MINUTES, osFlagsWaitAll, osWaitForever );
        SPACalc();
        //Передача команды позиционирования трекеру
        osEventFlagsSet( trc_event, EVN_TRC_SUN_POS );
        //передача данных в HMI
        send = ID_DEV_SPA; 
        osMessageQueuePut( hmi_msg, &send, 0, 0 );
       }
 }

//*************************************************************************************************
// Расчет времени восхода, заката, продолжительности дня, углов азимута и зенита
// для текущей даты и времени
//
// Широта — угол между местным направлением зенита и плоскостью экватора, отсчитываемый от 0° до 90° 
// в обе стороны от экватора. Географическую широту точек, лежащих в северном полушарии, 
// (северную широту) принято считать положительной, широту точек в южном полушарии — отрицательной.
//
// Долгота — двугранный угол между плоскостью меридиана, проходящего через данную точку, и 
// плоскостью начального нулевого меридиана, от которого ведётся отсчёт долготы. 
// Долготу от 0° до 180° к востоку от нулевого меридиана называют восточной, к западу — западной. 
// Восточные долготы принято считать положительными, западные — отрицательными.
//
// На картах Google и картах Яндекс вначале широта, затем долгота
//*************************************************************************************************
void SPACalc( void ) {

    RTC_TIME_Type Time;

    RTC_GetFullTime( LPC_RTC, &Time );
    spa.day = Time.DOM;
    spa.month = Time.MONTH;
    spa.year = Time.YEAR;
    spa.hour = Time.HOUR;
    spa.minute = Time.MIN;
    spa.second = Time.SEC;
    //постоянные значения для расчета
    spa.delta_ut1     = 0;
    spa.delta_t       = 67;
    spa.atmos_refract = 0.5667;
    spa.function      = SPA_ALL;
    //параметры места наблюдения из структуры параметров
    spa.timezone = (double)config.spa_timezone;         //http://www.timeanddate.com/time/map/
    spa.latitude = config.spa_latitude;                 //широта наблюдателя
    spa.longitude = config.spa_longitude;               //долгота наблюдателя
    spa.elevation = (double)config.spa_elevation;       //высота наблюдателя
    spa.pressure = (double)config.spa_pressure;         //среднегодовое местное давление
    spa.temperature = (double)config.spa_temperature;   //среднегодовая местная температура
    spa.slope = (double)config.spa_slope;               //наклон поверхности (измеряется от горизонтальной плоскости)
    spa.azm_rotation = (double)config.spa_azm_rotation; //вращение поверхности азимут (измеряется с юга на проекции нормали
                                                        //к поверхности на горизонтальной плоскости, отрицательный восток)
    //выполняем расчет данных
    sunpos.error = (SpaValid)spa_calculate( &spa );
    if ( sunpos.error == SPA_ERROR_OK ) {
        sunpos.sunrise = spa.sunrise;
        sunpos.sunset = spa.sunset;
        //проверка вхождения текущего времени в интервал продолжительности дня
        if ( TimeCheck( spa.sunrise, TimeToDecimal( spa.hour, spa.minute, spa.second ), spa.sunset ) ) {
            sunpos.zenith = spa.zenith;   //зенитный угол - угол между направлением на солнце и нормали к земле
            sunpos.azimuth = spa.azimuth; //азимутальный угол
           }
        else {
            sunpos.zenith = 0;
            sunpos.azimuth = 0;
           }
        sunpos.duration = sunpos.sunset - sunpos.sunrise;
       }
    else {
        //расчет не выполнен, ошибки в параметрах, обнулим данные
        sunpos.sunrise = spa.sunrise = 0;
        sunpos.sunset = spa.sunset = 0;
        sunpos.zenith = spa.zenith = 0;
        sunpos.azimuth = spa.azimuth = 0;
        sunpos.duration = 0;
       }
 }
