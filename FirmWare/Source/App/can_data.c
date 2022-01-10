
//*************************************************************************************************
//
// Формирование данных для передачи по CAN шине
//
//*************************************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "device.h"
#include "dev_data.h"
#include "dev_param.h"

#include "hmi_can.h"
#include "can_data.h"
#include "can_def.h"

#include "command.h"
#include "events.h"

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void CanDataBatmon( uint8_t sub_id );
static void CanDataMppt( uint8_t sub_id );
static void CanDataCharger( uint8_t sub_id );
static void CanDataInv1( uint8_t sub_id );
static void CanDataInv2( uint8_t sub_id );
static void CanDataGen( uint8_t sub_id );
static void CanDataTrc( uint8_t sub_id );
static void CanDataSpa( uint8_t sub_id );
static void CanDataConfig( uint8_t sub_id );

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static CAN_BATMON1     can_batmon1;
static CAN_BATMON2     can_batmon2;
static CAN_BATMON3     can_batmon3;
static CAN_BATMON4     can_batmon4;
static CAN_BATMON5     can_batmon5;
static CAN_BATMON6     can_batmon6;
static CAN_BATMON7     can_batmon7;

static CAN_MPPT1       can_mppt1;
static CAN_MPPT2       can_mppt2;
static CAN_MPPT3       can_mppt3;
static CAN_MPPT4       can_mppt4;
static CAN_MPPT5       can_mppt5;
static CAN_MPPT6       can_mppt6;
    
static CAN_INV1        can_1inv1, can_2inv1;
static CAN_INV2        can_1inv2, can_2inv2;

static CAN_CHARGER1    can_charger1;

static CAN_GEN1        can_gen1;
static CAN_GEN2        can_gen2;

static CAN_TRC1        can_trc1;
static CAN_TRC2        can_trc2;

static CAN_SPA1        can_spa1;
static CAN_SPA2        can_spa2;
static CAN_SPA3        can_spa3;
 
static CAN_CONFIG1     can_config1;
static CAN_CONFIG2     can_config2;
static CAN_CONFIG3     can_config3;
static CAN_CONFIG4     can_config4;
static CAN_CONFIG5     can_config5;
static CAN_CONFIG6     can_config6;
static CAN_CONFIG7     can_config7;
static CAN_CONFIG8     can_config8;
static CAN_CONFIG9     can_config9;
static CAN_CONFIG10    can_config10;
static CAN_CONFIG11    can_config11;
static CAN_CONFIG12    can_config12;
static CAN_CONFIG13    can_config13;
static CAN_CONFIG14    can_config14;
static CAN_CONFIG15    can_config15;

//Структура описания передаваемых данных по CAN шине
typedef struct {
    Device dev_id;                  //ID устр-ва
    uint32_t pack_id;               //номер пакета
    void (*func)( uint8_t sub_id ); //функция заполняющая структуры данными
    uint8_t *ptr_data;              //указатель на структуру данных
    uint8_t len_data;               //размер блока данных
 } CAN_DATA;

//Описание передаваемых данных по CAN шине
static const CAN_DATA can_data[] = {
    //---------------------------------------------------------------------------------------
    //ID уст-ва   номер     функция         структура данных            размер данных
    //            пакета    данных
    //---------------------------------------------------------------------------------------
    //данные портов
    ID_DEV_PORTS,   1,      NULL,           (uint8_t *)&ports,          sizeof( ports ),
    //данные RTC
    ID_DEV_RTC,     1,      NULL,           (uint8_t *)&rtc,            sizeof( rtc ),
    //данные монитора АКБ
    ID_DEV_BATMON,  1,      CanDataBatmon,  (uint8_t *)&can_batmon1,    sizeof( can_batmon1 ),
    ID_DEV_BATMON,  2,      CanDataBatmon,  (uint8_t *)&can_batmon2,    sizeof( can_batmon2 ),
    ID_DEV_BATMON,  3,      CanDataBatmon,  (uint8_t *)&can_batmon3,    sizeof( can_batmon3 ),
    ID_DEV_BATMON,  4,      CanDataBatmon,  (uint8_t *)&can_batmon4,    sizeof( can_batmon4 ),
    ID_DEV_BATMON,  5,      CanDataBatmon,  (uint8_t *)&can_batmon5,    sizeof( can_batmon5 ),
    ID_DEV_BATMON,  6,      CanDataBatmon,  (uint8_t *)&can_batmon6,    sizeof( can_batmon6 ),
    ID_DEV_BATMON,  7,      CanDataBatmon,  (uint8_t *)&can_batmon7,    sizeof( can_batmon7 ),
    //данные контроллера заряда MPPT
    ID_DEV_MPPT,    1,      CanDataMppt,    (uint8_t *)&can_mppt1,      sizeof( can_mppt1 ),
    ID_DEV_MPPT,    2,      CanDataMppt,    (uint8_t *)&can_mppt2,      sizeof( can_mppt2 ),
    ID_DEV_MPPT,    3,      CanDataMppt,    (uint8_t *)&can_mppt3,      sizeof( can_mppt3 ),
    ID_DEV_MPPT,    4,      CanDataMppt,    (uint8_t *)&can_mppt4,      sizeof( can_mppt4 ),
    ID_DEV_MPPT,    5,      CanDataMppt,    (uint8_t *)&can_mppt5,      sizeof( can_mppt5 ),
    ID_DEV_MPPT,    6,      CanDataMppt,    (uint8_t *)&can_mppt6,      sizeof( can_mppt6 ),
    //данные контроллера заряда PB-1000-224
    ID_DEV_CHARGER, 1,      CanDataCharger, (uint8_t *)&can_charger1,   sizeof( can_charger1 ),
    //данные инвертора TS-1000-224
    ID_DEV_INV1,    1,      CanDataInv1,    (uint8_t *)&can_1inv1,      sizeof( can_1inv1 ),
    ID_DEV_INV1,    2,      CanDataInv1,    (uint8_t *)&can_1inv2,      sizeof( can_1inv2 ),
    //данные инвертора TS-3000-224
    ID_DEV_INV2,    1,      CanDataInv2,    (uint8_t *)&can_2inv1,      sizeof( can_2inv1 ),
    ID_DEV_INV2,    2,      CanDataInv2,    (uint8_t *)&can_2inv2,      sizeof( can_2inv2 ),
    //данные блока АВР
    ID_DEV_ALT,     1,      NULL,           (uint8_t *)&alt,            sizeof( alt ),
    //данные генератора
    ID_DEV_GEN,     1,      CanDataGen,     (uint8_t *)&can_gen1,       sizeof( can_gen1 ),
    ID_DEV_GEN,     2,      CanDataGen,     (uint8_t *)&can_gen2,       sizeof( can_gen2 ),
    //данные контроллера трекера
    ID_DEV_TRC,     1,      CanDataTrc,     (uint8_t *)&can_trc1,       sizeof( can_trc1 ),
    ID_DEV_TRC,     2,      CanDataTrc,     (uint8_t *)&can_trc2,       sizeof( can_trc2 ),
    //данные положения солнца
    ID_DEV_SPA,     1,      CanDataSpa,     (uint8_t *)&can_spa1,       sizeof( can_spa1 ),
    ID_DEV_SPA,     2,      CanDataSpa,     (uint8_t *)&can_spa2,       sizeof( can_spa2 ),
    ID_DEV_SPA,     3,      CanDataSpa,     (uint8_t *)&can_spa3,       sizeof( can_spa3 ),
    //данные голосового информатора
    ID_DEV_VOICE,   1,      NULL,           (uint8_t *)&voice,          sizeof( voice ),
    //параметры конфигурации
    ID_CONFIG,      1,      CanDataConfig,  (uint8_t *)&can_config1,    sizeof( can_config1 ),
    ID_CONFIG,      2,      CanDataConfig,  (uint8_t *)&can_config2,    sizeof( can_config2 ),
    ID_CONFIG,      3,      CanDataConfig,  (uint8_t *)&can_config3,    sizeof( can_config3 ),
    ID_CONFIG,      4,      CanDataConfig,  (uint8_t *)&can_config4,    sizeof( can_config4 ),
    ID_CONFIG,      5,      CanDataConfig,  (uint8_t *)&can_config5,    sizeof( can_config5 ),
    ID_CONFIG,      6,      CanDataConfig,  (uint8_t *)&can_config6,    sizeof( can_config6 ),
    ID_CONFIG,      7,      CanDataConfig,  (uint8_t *)&can_config7,    sizeof( can_config7 ),
    ID_CONFIG,      8,      CanDataConfig,  (uint8_t *)&can_config8,    sizeof( can_config8 ),
    ID_CONFIG,      9,      CanDataConfig,  (uint8_t *)&can_config9,    sizeof( can_config9 ),
    ID_CONFIG,      10,     CanDataConfig,  (uint8_t *)&can_config10,   sizeof( can_config10 ),
    ID_CONFIG,      11,     CanDataConfig,  (uint8_t *)&can_config11,   sizeof( can_config11 ),
    ID_CONFIG,      12,     CanDataConfig,  (uint8_t *)&can_config12,   sizeof( can_config12 ),
    ID_CONFIG,      13,     CanDataConfig,  (uint8_t *)&can_config13,   sizeof( can_config13 ),
    ID_CONFIG,      14,     CanDataConfig,  (uint8_t *)&can_config14,   sizeof( can_config14 ),
    ID_CONFIG,      15,     CanDataConfig,  (uint8_t *)&can_config15,   sizeof( can_config15 )
 };

//*************************************************************************************************
// Передача данных/событий уст-ва по CAN шине
// uint32_t dev_id - ID уст-ва + PARAM_ID по которому будет выполняться передача данных
//*************************************************************************************************
void DevDataSend( uint32_t dev_id ) {

    uint32_t can_id;
    uint8_t i, id_mess;
    
    //передача событий
    if ( CAN_GET_DEV_ID( dev_id ) == ID_DEV_LOG ) {
        //ID сообщения (без ID события)
        can_id = dev_id & CAN_FILTER_MESS;
        //ID события
        id_mess = (uint8_t)( (uint32_t)CAN_MASK_MESS_ID & (uint32_t)dev_id );
        //передача пакета данных, для ID_DEV_GEN размер данных = 0
        if ( CAN_GET_PARAM_ID( ( dev_id & CAN_MASK_PARAM_ID ) ) == ID_DEV_GEN ) 
            CANSendFrame( can_id, (uint8_t *)&id_mess, 0 );
        else CANSendFrame( can_id, (uint8_t *)&id_mess, sizeof( id_mess ) );
        return;
       }
    //передача данных
    for ( i = 0; i < SIZE_ARRAY( can_data ); i++ ) {
        if ( can_data[i].dev_id != dev_id )
            continue;
        if ( can_data[i].func != 0 )
            can_data[i].func( can_data[i].pack_id ); //вызов функции - формируем данные
        //параметры пакета
        can_id = CAN_DEV_ID( can_data[i].dev_id ) | CAN_PACK_SUB( can_data[i].pack_id );
        //передача пакета данных
        CANSendFrame( can_id, can_data[i].ptr_data, can_data[i].len_data );
       }
 }

//*************************************************************************************************
// Заполняет структуры данными монитора АКБ
// uint8_t sub_id - ID блока данных
//*************************************************************************************************
static void CanDataBatmon( uint8_t sub_id ) {

    if ( sub_id == 1 ) {
        can_batmon1.link = batmon.link;
        can_batmon1.alarm = batmon.alarm;
        can_batmon1.relay = batmon.relay;
        can_batmon1.alarm_mode = batmon.alarm_mode;
        can_batmon1.ttg = batmon.ttg;
        can_batmon1.h4 = batmon.h4;
        can_batmon1.h5 = batmon.h5;
       }
    if ( sub_id == 2 ) {
        can_batmon2.voltage = batmon.voltage;
        can_batmon2.current = batmon.current;
       }
    if ( sub_id == 3 ) {
        can_batmon3.cons_energy = batmon.cons_energy;
        can_batmon3.soc = batmon.soc;
       }
    if ( sub_id == 4 ) {
        can_batmon4.h1 = batmon.h1;
        can_batmon4.h2 = batmon.h2;
       }
    if ( sub_id == 5 ) {
        can_batmon5.h3 = batmon.h3;
        can_batmon5.h6 = batmon.h6;
       }
    if ( sub_id == 6 ) {
        can_batmon6.h7 = batmon.h7;
        can_batmon6.h8 = batmon.h8;
       }
    if ( sub_id == 7 ) {
        can_batmon7.h9 = batmon.h9;
        can_batmon7.h11 = batmon.h11;
       }
 }

//*************************************************************************************************
// Заполняет структуры данными контроллера MPPT
// uint8_t sub_id - ID блока данных
//*************************************************************************************************
static void CanDataMppt( uint8_t sub_id ) {

    if ( sub_id == 1 ) {
        can_mppt1.power = mppt.power;
        can_mppt1.connect = mppt.connect;
        can_mppt1.link = mppt.link;
        can_mppt1.pv_stat = mppt.pv_stat;
        can_mppt1.pv_mode = mppt.pv_mode;
        can_mppt1.u08_charge_mode = mppt.u08_charge_mode;
        can_mppt1.u12_soc = mppt.u12_soc;
        can_mppt1.u07_time_flt = mppt.u07_time_flt;
        can_mppt1.time_charge = mppt.time_charge;
       }
    if ( sub_id == 2 ) {
        can_mppt2.u01_in_voltage = mppt.u01_in_voltage;
        can_mppt2.u02_in_current = mppt.u02_in_current;
       }
    if ( sub_id == 3 ) {
        can_mppt3.u03_out_voltage = mppt.u03_out_voltage;
        can_mppt3.u04_out_current = mppt.u04_out_current;
       }
    if ( sub_id == 4 )
        can_mppt4.u13_bat_current = mppt.u13_bat_current;
    if ( sub_id == 5 ) {
        can_mppt5.u05_energy1 = mppt.u05_energy1;
        can_mppt5.u05_energy2 = mppt.u05_energy2;
       }
    if ( sub_id == 6 ) {
        can_mppt6.u15_bat_temp = mppt.u15_bat_temp;
        can_mppt6.u11_mppt_temp = mppt.u11_mppt_temp;
       }
 }

//*************************************************************************************************
// Заполняет структуру данными контроллера заряда PB-1000-224
// uint8_t sub_id - ID блока данных
//*************************************************************************************************
static void CanDataCharger( uint8_t sub_id ) {

    if ( sub_id == 1 ) {
        can_charger1.connect_ac = charger.connect_ac;
        can_charger1.device_ok = charger.device_ok;
        can_charger1.charge_end = charger.charge_end;
        can_charger1.charge_mode = charger.charge_mode;
        can_charger1.charge_exec = charger.charge_exec;
        can_charger1.current = charger.current;
        can_charger1.error = charger.error;
       }
 }

//*************************************************************************************************
// Заполняет структуры данными инвертора TS-1000-224
// uint8_t sub_id - ID блока данных
//*************************************************************************************************
static void CanDataInv1( uint8_t sub_id ) {

    if ( sub_id == 1 ) {
        can_1inv1.dc_conn = inv1.dc_conn;
        can_1inv1.mode = inv1.mode;
        can_1inv1.cycle_step = inv1.cycle_step;
        can_1inv1.ctrl_error = inv1.ctrl_error;
        can_1inv1.dev_error = inv1.dev_error;
        can_1inv1.ac_out = inv1.ac_out;
        can_1inv1.power_watt = inv1.power_watt;
        can_1inv1.power_perc = inv1.power_perc;
       }
    if ( sub_id == 2 ) {
        can_1inv2.dc_in = inv1.dc_in;
        can_1inv2.temperature = inv1.temperature;
       }
 }

//*************************************************************************************************
// Заполняет структуры данными инвертора TS-3000-224
// uint8_t sub_id - ID блока данных
//*************************************************************************************************
static void CanDataInv2( uint8_t sub_id ) {

    if ( sub_id == 1 ) {
        can_2inv1.dc_conn = inv2.dc_conn;
        can_2inv1.mode = inv2.mode;
        can_2inv1.cycle_step = inv2.cycle_step;
        can_2inv1.ctrl_error = inv2.ctrl_error;
        can_2inv1.dev_error = inv2.dev_error;
        can_2inv1.ac_out = inv2.ac_out;
        can_2inv1.power_watt = inv2.power_watt;
        can_2inv1.power_perc = inv2.power_perc;
       }
    if ( sub_id == 2 ) {
        can_2inv2.dc_in = inv2.dc_in;
        can_2inv2.temperature = inv2.temperature;
       }
 }

//*************************************************************************************************
// Заполняет структуры данными контроллера генератора
// uint8_t sub_id - ID блока данных
//*************************************************************************************************
static void CanDataGen( uint8_t sub_id ) {

    if ( sub_id == 1 ) {
        can_gen1.remote = gen_ptr->remote;
        can_gen1.connect = gen_ptr->connect;
        can_gen1.mode = gen_ptr->mode;
        can_gen1.stat = gen_ptr->stat;
        can_gen1.cycle1 = gen_ptr->cycle1;
        can_gen1.cycle2 = gen_ptr->cycle2;
        can_gen1.error = gen_ptr->error;
        can_gen1.timer_run_inc = gen_ptr->timer_run_inc;
        can_gen1.timer_run_dec = gen_ptr->timer_run_dec;
       }
    if ( sub_id == 2 ) {
        can_gen2.timer_lost_acmain = gen_ptr->timer_lost_acmain;
        can_gen2.timer_rest_acmain = gen_ptr->timer_rest_acmain;
        can_gen2.timer_sleep = gen_ptr->timer_sleep;
       }
 }

//*************************************************************************************************
// Заполняет структуры данными трекера
// uint8_t sub_id - ID блока данных
//*************************************************************************************************
static void CanDataTrc( uint8_t sub_id ) {

    if ( sub_id == 1 ) {
        can_trc1.link = tracker.link;
        can_trc1.pwr_trc = tracker.pwr_trc;
        can_trc1.pwr_act = tracker.pwr_act;
        can_trc1.pwr_fuse = tracker.pwr_fuse;
        can_trc1.stat = tracker.stat;
        can_trc1.time_on = tracker.time_on;
       }
    if ( sub_id == 2 ) {
        can_trc2.act_pos_vert = tracker.act_pos_vert;
        can_trc2.act_pos_horz = tracker.act_pos_horz;
        can_trc2.act_vert_eep = tracker.act_vert_eep;
        can_trc2.act_horz_eep = tracker.act_horz_eep;
       }
 }

//*************************************************************************************************
// Заполняет структуры данными положения солнца
// uint8_t sub_id - ID блока данных
//*************************************************************************************************
static void CanDataSpa( uint8_t sub_id ) {

    if ( sub_id == 1 ) {
        can_spa1.sunrise = sunpos.sunrise;
        can_spa1.sunset = sunpos.sunset;
       }
    if ( sub_id == 2 ) {
        can_spa2.zenith = sunpos.zenith;
        can_spa2.azimuth = sunpos.azimuth;
       }
    if ( sub_id == 3 ) {
        can_spa3.duration = sunpos.duration;
        can_spa3.error = sunpos.error;
       }
 }

//*************************************************************************************************
// Заполняет структуры параметрами настроек
// uint8_t sub_id - ID блока данных
//*************************************************************************************************
static void CanDataConfig( uint8_t sub_id ) {

    if ( sub_id == 1 )
        memcpy( (uint8_t *)can_config1.scr_file, (uint8_t *)config.scr_file, 8 );
    if ( sub_id == 2 )
        memcpy( (uint8_t *)can_config2.scr_file, (uint8_t *)config.scr_file + 8, 8 );
    if ( sub_id == 3 )
        memcpy( (uint8_t *)can_config3.job_file, (uint8_t *)config.job_file, 8 );
    if ( sub_id == 4 )
        memcpy( (uint8_t *)can_config4.job_file, (uint8_t *)config.job_file + 8, 8 );
    if ( sub_id == 5 )
        memcpy( (uint8_t *)can_config5.job_test, (uint8_t *)config.job_test, 8 );
    if ( sub_id == 6 )
        memcpy( (uint8_t *)can_config6.job_test, (uint8_t *)config.job_test + 8, 8 );
    if ( sub_id == 7 ) {
        can_config7.mode_sys = config.mode_sys;
        can_config7.log_enable_pv = config.log_enable_pv;
        can_config7.log_enable_chrg = config.log_enable_chrg;
        can_config7.log_enable_mppt = config.log_enable_mppt;
        can_config7.log_enable_inv = config.log_enable_inv;
        can_config7.log_enable_bmon = config.log_enable_bmon;
        can_config7.log_enable_gen = config.log_enable_gen;
        can_config7.log_enable_alt = config.log_enable_alt;
        can_config7.log_enable_trc = config.log_enable_trc;
        can_config7.mode_logging = config.mode_logging;
        can_config7.gen_auto_mode = config.gen_auto_mode;
        can_config7.log_enable1 = 0;
        can_config7.log_enable2 = 0;
        can_config7.pb_current_stop = config.pb_current_stop;
        can_config7.delay_start_inv = config.delay_start_inv;
        can_config7.delay_stop_inv = config.delay_stop_inv;
       }
    if ( sub_id == 8 ) {
        can_config8.last_charge = config.last_charge;
        can_config8.datlog_upd_pb = config.datlog_upd_chrg;
        can_config8.datlog_upd_mppt = config.datlog_upd_mppt;
       }
    if ( sub_id == 9 ) {
        can_config9.datlog_upd_bmon = config.datlog_upd_bmon;
        can_config9.datlog_upd_ts = config.datlog_upd_inv;
        can_config9.datlog_upd_trc = config.datlog_upd_trc;
        can_config9.gen_time_run = config.gen_time_run;
       }
    if ( sub_id == 10 ) {
        can_config10.gen_delay_start = config.gen_delay_start;
        can_config10.gen_delay_stop = config.gen_delay_stop;
        can_config10.gen_cnt_start = config.gen_cnt_start;
        can_config10.gen_delay_chk_run = config.gen_delay_chk_run;
        can_config10.gen_before_start = config.gen_before_start;
        can_config10.gen_time_sleep = config.gen_time_sleep;
       }
    if ( sub_id == 11 )
        memcpy( (uint8_t *)can_config11.gen_time_start, (uint8_t *)config.gen_time_start, sizeof( config.gen_time_start ) );
    if ( sub_id == 12 ) {
        can_config12.gen_time_test = config.gen_time_test;
        can_config12.gen_last_run = config.gen_last_run;
        can_config12.spa_timezone = config.spa_timezone;
        can_config12.spa_temperature = config.spa_temperature;
       }
    if ( sub_id == 13 ) {
        can_config13.spa_elevation = config.spa_elevation;
        can_config13.spa_pressure = config.spa_pressure;
        can_config13.spa_slope = config.spa_slope;
        can_config13.spa_azm_rotation = config.spa_azm_rotation;
       }
    if ( sub_id == 14 )
        can_config14.spa_latitude = config.spa_latitude;
    if ( sub_id == 15 )
        can_config15.spa_longitude = config.spa_longitude;
 }
