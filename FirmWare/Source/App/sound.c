
//*************************************************************************************************
//
// Управление ЦАП (воспроизведение звуковых сообщений)
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>
#include <lpc_types.h>

#include "cmsis_os2.h"

#include "dev_param.h"

#include "dac.h"
#include "sound.h"
#include "events.h"

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static SoundData sound_data;

//*************************************************************************************************
// Функция и звуковые данные находиться в отдельном блоке 
// FLASH памяти: 0x00053000, размер 0x0002D000 (180K)
//*************************************************************************************************
extern uint8_t *GetWaveData( char *name, uint32_t *size );

// Соответствие голосовых и звуковых сообщений.
// Имя сообщения       Содержание                               Звуковое сообщение
// ---------------------------------------------------------------------------------
//  1 info             Та-та-та (сигнал начала информации)      
//  2 low_charge       Низкий заряд батареи %                       2 batlow
//  3 level_charge     Уровень заряда батареи %                 
//  4 power_inv        Нагрузка инверторов % и %                
//  5 ttgo             Продолжительность работы % часов % минут 
//  6 charge           Плановая подзарядка                      
//  7 bat_full         Батарея заряжена                             1 batcrt
//  8 power_ac         Питание нагрузки от основной сети        
//  9 power_ts         Питание нагрузки от инверторов           
// 10 power_gen        Питание нагрузки от генератора              12 print
// 11 ac_off           Основная сеть отключена                     13 shutdn
// 12 ac_on            Основная сеть восстановлена                 14 startu
// 13 power_ctrl       Нет питания контроллера                      5 excl
// 14 bat_noconn       Батарея не подключена                        4 error
// 15 charge_end       Подзарядка завершена                         1 batcrt
// 16 gen_run          Генератор запущен                           11 notify
// 17 gen_off          Генератор выключен                           4 error
// 18 gen_discon       Генератор отключился                         3 crtstp
// 19 gen_check        Проверьте генератор                          4 error
// 20 pv_on            Солнечные панели подключены                  6 hdfail
// 21 pv_off           Солнечные панели одключены                   7 hdinst
// 22 trc_on           Управление трекером включено                 9 logoff
// 23 trc_off          Управление трекером выключено               10 logon
// 24 test             Плановое тестирование                    
// 25 inv_start        Инверторы будут запущены через ... минут
// 26 voice            Голосовой информатор

//*************************************************************************************************
//Таблица соответствия голосового сообщения к звуковому
static SoundId inform_song[] = { 
    SND_NOTHING,                            //VOICE_NULL
    SND_NOTHING,                            //VOICE_INFO          //Предварительное сообщение
    SND_BATLOW,                             //VOICE_LOW_CHARGE    //Низкий заряд батареи %
    SND_NOTHING,                            //VOICE_LEVEL_CHARGE  //Уровень заряда батареи %
    SND_NOTHING,                            //VOICE_POWER_INV     //Нагрузка инверторов % и %
    SND_NOTHING,                            //VOICE_TTGO          //Продолжительность работы ... часов (минут)
    SND_NOTHING,                            //VOICE_CHARGE        //Плановая подзарядка
    SND_BATCRT,                             //VOICE_BAT_FULL      //Батарея заряжена
    SND_NOTHING,                            //VOICE_POWER_AC      //Питание нагрузки от основной сети
    SND_NOTHING,                            //VOICE_POWER_TS      //Питание нагрузки от инверторов
    SND_PRINT,                              //VOICE_POWER_GEN     //Питание нагрузки от генератора
    SND_SHUTDN,                             //VOICE_AC_OFF        //Основная сеть отключена
    SND_STARTU,                             //VOICE_AC_ON         //Основная сеть восстановлена
    SND_EXCL,                               //VOICE_POWER_CTRL    //Нет питания контроллера
    SND_ERROR,                              //VOICE_BAT_NOCONN    //Батарея не подключена
    SND_BATCRT,                             //VOICE_CHARGE_END    //Подзарядка завершена
    SND_NOTIFY,                             //VOICE_GEN_RUN       //Генератор запущен
    SND_ERROR,                              //VOICE_GEN_OFF       //Генератор выключен
    SND_CRTSTP,                             //VOICE_GEN_DISCON    //Генератор отключился
    SND_ERROR,                              //VOICE_GEN_CHECK     //Проверьте генератор
    SND_HDFAIL,                             //VOICE_PV_ON         //Солнечные панели подключены
    SND_HDINST,                             //VOICE_PV_OFF        //Солнечные панели подключены
    SND_LOGOFF,                             //VOICE_TRC_ON        //Управление трекером включено
    SND_LOGON,                              //VOICE_TRC_OFF       //Управление трекером выключено
    SND_NOTHING,                            //VOICE_TEST          //Плановое тестирование
    SND_NOTHING,                            //VOICE_TS_START      //Инверторы будут запущены через ... минут
    SND_NOTHING                             //VOICE_VOICE         //Голосовой информатор
 };

//*************************************************************************************************
// Инициализация таймера, DAC, задачи
//*************************************************************************************************
//*************************************************************************************************
// Запуск воспроизведения звукового сообщения
// Sound id_sound   - ID сообщения, если ID > 0 - параметр name_info игнорируется
// char *name       - имя сообщения
// return = SUCCESS - параметры приняты
//        = ERROR   - неправильные параметры, нет звукового фрагмента
//*************************************************************************************************
Status SoundPlay( SoundId id_sound, char *name ) {

    uint32_t size = 0;
    uint8_t *ptr = NULL;
    
    if ( !id_sound && name == NULL )
        return ERROR;
    if ( id_sound == SND_BATCRT )                   //Батарея заряжена
        ptr = GetWaveData( "batcrt", &size );
    if ( id_sound == SND_BATLOW )                   //Низкий заряд батареи
        ptr = GetWaveData( "batlow", &size );
    if ( id_sound == SND_CRTSTP )                   //Генератор отключился
        ptr = GetWaveData( "crtstp", &size );
    if ( id_sound == SND_ERROR )                    //Генератор выключен
        ptr = GetWaveData( "error", &size );
    if ( id_sound == SND_EXCL )                     //Нет питания контроллера
        ptr = GetWaveData( "excl", &size );
    if ( id_sound == SND_HDFAIL )                   //Солнечные панели подключены
        ptr = GetWaveData( "hdfail", &size );
    if ( id_sound == SND_HDINST )                   //Солнечные панели подключены
        ptr = GetWaveData( "hdinst", &size );
    if ( id_sound == SND_HDREM )
        ptr = GetWaveData( "hdrem", &size );
    if ( id_sound == SND_LOGOFF )                   //Управление трекером включено
        ptr = GetWaveData( "logoff", &size );
    if ( id_sound == SND_LOGON )                    //Управление трекером выключено
        ptr = GetWaveData( "logon", &size );
    if ( id_sound == SND_NOTIFY )                   //Генератор запущен
        ptr = GetWaveData( "notify", &size );
    if ( id_sound == SND_PRINT )                    //Питание нагрузки от генератора
        ptr = GetWaveData( "print", &size );
    if ( id_sound == SND_SHUTDN )                   //Основная сеть отключена
        ptr = GetWaveData( "shutdn", &size );
    if ( id_sound == SND_STARTU )                   //Основная сеть восстановлена
        ptr = GetWaveData( "startu", &size );
    if ( !id_sound )
        ptr = GetWaveData( name, &size );
    if ( ptr == NULL || !size )
        return ERROR;
    if ( ptr != NULL ) {
        sound_data.ptr = ptr;
        sound_data.size = size;
        osMessageQueuePut( sound_msg, &sound_data, 0, osWaitForever );
       }
    return SUCCESS;
 }

//*************************************************************************************************
// Соответствие номера голосового сообщения и звукового сообщения
// Voice id_sound - ID голосового сообщения
// result SoundId - ID звукового сообщения
//*************************************************************************************************
SoundId VoiceToSound( VoiceId id_voice ) {

    if ( id_voice < SIZE_ARRAY( inform_song ) )
        return inform_song[id_voice];
    return SND_NOTHING;
 }
