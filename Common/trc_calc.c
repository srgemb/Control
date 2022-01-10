
//*************************************************************************************************
//
// Функционал для расчета углов положения трекера
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "device.h"

#include "trc_calc.h"
#include "tracker_ext.h"

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
//сигналы солнечного сенсора
typedef enum {
    SENSOR_UP,
    SENSOR_DN,
    SENSOR_LF,
    SENSOR_RT
 } Sensor;

//сигналы концевых выключателей
typedef enum {
    LIMSWTCH_VCLS,
    LIMSWTCH_VOPN,
    LIMSWTCH_HCLS,
    LIMSWTCH_HOPN
 } LimSwtch;

//признак сигнала
typedef enum {
    SIGNAL_OFF,
    SIGNAL_ON
 } Signal;

//состояние сенсора
static char * const trc_sensor[][2] = {
    { "  :", "UP:" },
    { "  :", "DN:" },
    { "  :", "LF:" },
    { "   ", "RT"  }
 };

//состояние концевиков
static char * const trc_limsw[][2] = {
    { "     :", "V-CLS:" },
    { "     :", "V-OPN:" },
    { "      ", "H-CLS"  },
    { "      ", "H-OPN"  }
 };

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static char pos_angl[30], str_limsw[20], str_sensor[20];

//*************************************************************************************************
// Расчет угла наклона панелей по длине штока актуатора с учетом корректировки начального угла
// uint16_t len_stem - длина штока актуатора (0-900) мм
// return float      - угол наклона в градусах
//*************************************************************************************************
float AngleVert( uint16_t len_stem ) {

    float angle;

    if ( len_stem > MAX_VALUE_VERT )
        return 0;
    //угол наклона панелей к горизонту в радианах
    angle = acos( ( pow( VERT_A, 2 ) + pow( VERT_B, 2 ) - pow( (float)len_stem, 2 ) ) / ( 2 * VERT_A * VERT_B ) );
    //переводим в градусы, расчет угла на солнце
    return ( angle * RAD_TO_DEGREE ) + VERT_ANGLE_CORRECT;
 }

//*************************************************************************************************
// Расчет угла поворота панелей по азимуту для указанной длине штока актуатора
// uint16_t len_stem - длина штока (0-900) мм
// return float      - угол азимута
//*************************************************************************************************
float AngleHorz( uint16_t len_stem ) {

    float angle;

    if ( len_stem > MAX_VALUE_HORZ )
        len_stem = 0;
    //угол поворота панелей
    angle = acos( ( pow( HORZ_A, 2 ) + pow( HORZ_B, 2 ) - pow( (float)len_stem, 2 ) ) / ( 2 * HORZ_A * HORZ_B ) );
    //переводим в градусы и в фактический угол азимута
    return ( ( angle * RAD_TO_DEGREE ) * 2 ) + MIN_ANGLE_AZIMUT;
 }

//*************************************************************************************************
// Расчет длинны штока актуатора по углу наклона панелей на солнце
// float angle - угол наклона в градусах, допустимые значения 0-90 градусов
// return      - длинна штока (целое число) мм
//*************************************************************************************************
uint16_t StemVert( float angle ) {

    float angl;
    uint16_t stem;

    //корректировка угла
    angle -= VERT_ANGLE_CORRECT;
    if ( angle < 0 )
        return MIN_VALUE_VERT;
    if ( angle > 90 )
        return MAX_VALUE_VERT;
    //переводим угол
    angl = cos( angle * DEGREE_TO_RAD );
    stem = sqrt( pow( VERT_A, 2 ) + pow( VERT_B, 2 ) - ( 2 * VERT_A * VERT_B * angl ) );
    if ( stem > MAX_VALUE_VERT )
        return MAX_VALUE_VERT;
    return stem;
 }

//*************************************************************************************************
// Расчет длинны штока актуатора (поворот панелей) по азимуту SPA
// Фактический угол поворота панелей = ( "азимут SPA" - "минимальное значение азимута на восходе" )/2
// float angle - угол поворота в градусах, допустимые значения: MIN_ANGLE_AZIMUT - MAX_ANGLE_AZIMUT
// return      - длинна штока (целое число) мм
//*************************************************************************************************
uint16_t StemHorz( float angle ) {

    float angl;
    uint16_t stem;

    if ( angle <= MIN_ANGLE_AZIMUT )
        return MIN_VALUE_HORZ; //угол позиционирования меньше минимального угла
    if ( angle >= MAX_ANGLE_AZIMUT )
        return MAX_VALUE_HORZ; //угол позиционирования больше максимального угла
    //расчет фактического угла поворота панелей
    angle = ( angle - MIN_ANGLE_AZIMUT )/2;
    //переводим угол
    angl = cos( angle * DEGREE_TO_RAD );
    stem = sqrt( pow( VERT_A, 2 ) + pow( VERT_B, 2 ) - ( 2 * VERT_A * VERT_B * angl ) );
    if ( stem > 900 )
        return MAX_VALUE_HORZ;
    return stem;
 }

//*************************************************************************************************
// Возвращает вертикальную/горизонтальную позицию и угол положения панелей трекера
// uint16_t pos   - значение положения трекера в мм
// TrackerPos trc - тип значения горизонтальное/вертикальное
// return         - указатель на строку с результатом
//*************************************************************************************************
char *PosAngle( uint16_t pos, TrackerAct trc ) {
 
    memset( pos_angl, 0x00, sizeof( pos_angl ) );
    if ( trc == TRC_POS_VERTICAL )
        sprintf( pos_angl, "%03u/%.2f  ", pos, AngleVert( pos ) );
    if ( trc == TRC_POS_HORIZONTAL )
        sprintf( pos_angl, "%03u/%.2f  ", pos, AngleHorz( pos ) );
    return pos_angl;
 }
 
//*************************************************************************************************
// Возвращает расшифровку состояние солнечного сенсора
// uint16_t data - биты состояния трекера
//*************************************************************************************************
char *TrackerSensor( uint16_t data ) {

    memset( str_sensor, 0x00, sizeof( str_sensor ) );
    if ( data & EXT_SOLAR_UP )
        strcat( str_sensor, trc_sensor[SENSOR_UP][SIGNAL_ON] );
    else strcat( str_sensor, trc_sensor[SENSOR_UP][SIGNAL_OFF] );
    if ( data & EXT_SOLAR_DN )
        strcat( str_sensor, trc_sensor[SENSOR_DN][SIGNAL_ON] );
    else strcat( str_sensor, trc_sensor[SENSOR_DN][SIGNAL_OFF] );
    if ( data & EXT_SOLAR_LF )
        strcat( str_sensor, trc_sensor[SENSOR_LF][SIGNAL_ON] );
    else strcat( str_sensor, trc_sensor[SENSOR_LF][SIGNAL_OFF] );
    if ( data & EXT_SOLAR_RT )
        strcat( str_sensor, trc_sensor[SENSOR_RT][SIGNAL_ON] );
    else strcat( str_sensor, trc_sensor[SENSOR_RT][SIGNAL_OFF] );
    return str_sensor;
 }

//*************************************************************************************************
// Возвращает расшифровку состояние концевых выключателей актуаторов
// uint16_t data - биты состояния трекера
//*************************************************************************************************
char *TrackerLimSw( uint16_t data ) {

    memset( str_limsw, 0x00, sizeof( str_limsw ) );
    if ( data & ( EXT_TERM_VCLOSE | EXT_TERM_VOPEN ) ) {
        if ( data & EXT_TERM_VCLOSE ) 
            strcat( str_limsw, trc_limsw[LIMSWTCH_VCLS][SIGNAL_ON] );
        if ( data & EXT_TERM_VOPEN ) 
            strcat( str_limsw, trc_limsw[LIMSWTCH_VOPN][SIGNAL_ON] );
       }
    else strcat( str_limsw, trc_limsw[LIMSWTCH_VCLS][SIGNAL_OFF] );
    if ( data & ( EXT_TERM_HCLOSE | EXT_TERM_HOPEN ) ) {
        if ( data & EXT_TERM_HCLOSE ) 
            strcat( str_limsw, trc_limsw[LIMSWTCH_HCLS][SIGNAL_ON] );
        if ( data & EXT_TERM_HOPEN ) 
            strcat( str_limsw, trc_limsw[LIMSWTCH_HOPN][SIGNAL_ON] );
       }
    else strcat( str_limsw, trc_limsw[LIMSWTCH_VCLS][SIGNAL_OFF] );
    return str_limsw;
 }


