
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

#include "lpc177x_8x_dac.h"

#include "dac.h"
#include "ports.h"
#include "sound.h"
#include "priority.h"
#include "events.h"

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osMessageQueueId_t sound_msg = NULL;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static bool flg_play = false;
static uint32_t size = 0, ind_data = 0;
static uint8_t *sound = NULL;
static osSemaphoreId_t sound_sem;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t sound_attr = {
    .name = "Sound", 
    .stack_size = 256,
    .priority = osPriorityNormal
 };

static const osSemaphoreAttr_t sem_attr = { .name = "Sound" };
static const osMessageQueueAttr_t msg_attr = { .name = "Sound" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void DACPlay( void );
static void TaskSound( void *pvParameters );

//*************************************************************************************************
// Инициализация таймера, DAC, задачи
//*************************************************************************************************
void DACInit( void ) {

    TIM_TIMERCFG_Type TimerCfg2;
    TIM_MATCHCFG_Type TimerMath2;
    DAC_CONVERTER_CFG_Type dac_cfg;

    //очередь сообщений, семафор
    sound_sem = osSemaphoreNew( 1, 0, &sem_attr );
    sound_msg = osMessageQueueNew( 16, sizeof( SoundData ), &msg_attr );
    //создаем задачу управления
    osThreadNew( TaskSound, NULL, &sound_attr );
    //инициализация таймера "2", прерывание 125 usec, F = 4 KHz, T = 250 usec
    TimerCfg2.PrescaleOption = TIM_PRESCALE_USVAL;
    TimerCfg2.PrescaleValue = 25;
    //разрешение прерывания для таймера
    TimerMath2.MatchChannel = 0;
    TimerMath2.MatchValue = 4;
    TimerMath2.IntOnMatch = true;
    TimerMath2.StopOnMatch = DISABLE;
    TimerMath2.ResetOnMatch = ENABLE;
    //инициализация таймера
    TIM_Init( LPC_TIM2, TIM_TIMER_MODE, &TimerCfg2 );
    //установка параметров
    TIM_ConfigMatch( LPC_TIM2, &TimerMath2 );
    //Настройка прерывания
    NVIC_SetPriority( TIMER2_IRQn, NVIC_EncodePriority( NVIC_GetPriorityGrouping(), PRIORITY_TIMER, SUB_PRIORITY_TIMER2 ) );
    NVIC_EnableIRQ( TIMER2_IRQn );
    //включаем таймер
    TIM_Cmd( LPC_TIM2, ENABLE );
    //инициализация DAC
    dac_cfg.DBLBUF_ENA = 1;
    dac_cfg.DMA_ENA = 1;
    DAC_Init( 0 );
    DAC_ConfigDAConverterControl( 0, &dac_cfg );
 }

//*************************************************************************************************
// Обработка прерывания от таймера T2
//*************************************************************************************************
void TIMER2_IRQHandler( void ) {

    if ( TIM_GetIntStatus( LPC_TIM2, TIM_MR0_INT ) == SET )
        TIM_ClearIntPending( LPC_TIM2, TIM_MR0_INT );
    DACPlay();
 }

//*************************************************************************************************
// Задача обработки очереди воспроизведения звуковых сообщений
//*************************************************************************************************
static void TaskSound( void *pvParameters ) {

    osStatus_t status;
    SoundData sound_data;
    
    for ( ;; ) {
        status = osMessageQueueGet( sound_msg, &sound_data, 0, osWaitForever );
        if ( status == osOK ) {
            //данные звукового фрагмента сообщения
            sound = sound_data.ptr;
            size = sound_data.size;
            flg_play = true;
            osSemaphoreAcquire( sound_sem, osWaitForever );
           }
       }
 }

//*************************************************************************************************
// Вывод данных в ЦАП (воспроизведение звука) по таймеру 4 kHz
// Разрешение выполнения по флагу flg_play и наличию данных
//*************************************************************************************************
static void DACPlay( void ) {

    if ( flg_play == false )
        return; //нет команды воспроизведения
    if ( sound == NULL || !size ) {
        flg_play = false;
        return; //параметры воспроизведения не указаны
       }
    //воспроизведение
    DAC_UpdateValue( 0, *( sound + ind_data ) );
    ind_data++;
    if ( ind_data >= size ) {
        //воспроизведение завершено
        sound = NULL;
        ind_data = 0;
        flg_play = false;
        osSemaphoreRelease( sound_sem );
       }
 }
