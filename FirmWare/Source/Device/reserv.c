
//*************************************************************************************************
//
// Управление дополнительными выходами
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "cmsis_os2.h"

#include "device.h"

#include "ports.h"
#include "eeprom.h"
#include "reserv.h"
#include "events.h"

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static osTimerId_t timer_imp1, timer_imp2, timer_imp3, timer_imp4;
static osTimerId_t timer_imp5, timer_imp6, timer_imp7, timer_imp8;

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void Timer1Callback( void *arg );
static void Timer2Callback( void *arg );
static void Timer3Callback( void *arg );
static void Timer4Callback( void *arg );
static void Timer5Callback( void *arg );
static void Timer6Callback( void *arg );
static void Timer7Callback( void *arg );
static void Timer8Callback( void *arg );

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static const osTimerAttr_t imp1_attr = { .name = "Reserv1" };
static const osTimerAttr_t imp2_attr = { .name = "Reserv2" };
static const osTimerAttr_t imp3_attr = { .name = "Reserv3" };
static const osTimerAttr_t imp4_attr = { .name = "Reserv4" };

static const osTimerAttr_t imp5_attr = { .name = "ExtUp" };
static const osTimerAttr_t imp6_attr = { .name = "ExtDn" };
static const osTimerAttr_t imp7_attr = { .name = "ExtLf" };
static const osTimerAttr_t imp8_attr = { .name = "ExtRt" };

//*************************************************************************************************
// Инициализация портов
//*************************************************************************************************
void ReservInit( void ) {

    //режим портов "выход"
    GPIO_SetDir( RES_PORT,  RESERV1, GPIO_DIR_OUTPUT );
    GPIO_SetDir( RES_PORT,  RESERV2, GPIO_DIR_OUTPUT );
    GPIO_SetDir( RES_PORT,  RESERV3, GPIO_DIR_OUTPUT );
    GPIO_SetDir( RES_PORT4, RESERV4, GPIO_DIR_OUTPUT );
    //режим портов "выход"
    GPIO_SetDir( TRC_PORT, EXT_UP, GPIO_DIR_OUTPUT );
    GPIO_SetDir( TRC_PORT, EXT_LF, GPIO_DIR_OUTPUT );
    GPIO_SetDir( TRC_PORT, EXT_DN, GPIO_DIR_OUTPUT );
    GPIO_SetDir( TRC_PORT, EXT_RT, GPIO_DIR_OUTPUT );
    //восстановить предыдущее состояние
    ReservCmnd( RELAY_RES1, (RelayCmnd)EepromLoad( EEP_RESERV1 ) );
    ReservCmnd( RELAY_RES2, (RelayCmnd)EepromLoad( EEP_RESERV2 ) );
    ReservCmnd( RELAY_RES3, (RelayCmnd)EepromLoad( EEP_RESERV3 ) );
    ReservCmnd( RELAY_RES4, (RelayCmnd)EepromLoad( EEP_RESERV4 ) );
    //таймер длительности включения реле
    timer_imp1 = osTimerNew( Timer1Callback, osTimerOnce, NULL, &imp1_attr );
    timer_imp2 = osTimerNew( Timer2Callback, osTimerOnce, NULL, &imp2_attr );
    timer_imp3 = osTimerNew( Timer3Callback, osTimerOnce, NULL, &imp3_attr );
    timer_imp4 = osTimerNew( Timer4Callback, osTimerOnce, NULL, &imp4_attr );
    timer_imp5 = osTimerNew( Timer5Callback, osTimerOnce, NULL, &imp5_attr ); 
    timer_imp6 = osTimerNew( Timer6Callback, osTimerOnce, NULL, &imp6_attr ); 
    timer_imp7 = osTimerNew( Timer7Callback, osTimerOnce, NULL, &imp7_attr ); 
    timer_imp8 = osTimerNew( Timer8Callback, osTimerOnce, NULL, &imp8_attr );
 }

//*************************************************************************************************
// Управление дополнительными реле
// RelayRes id_rel - ID дополнительного реле
// RelayCmnd cmnd  - команда управления
//*************************************************************************************************
void ReservCmnd( RelayRes id_rel, RelayCmnd cmnd ) {

    //включение
    if ( id_rel == RELAY_RES1 && cmnd == RELAY_ON ) {
        GPIO_PinWrite( RES_PORT, RESERV1, RELAY_ON );
        EepromUpdate( EEP_RESERV1, RELAY_ON );
       } 
    if ( id_rel == RELAY_RES2 && cmnd == RELAY_ON ) {
        GPIO_PinWrite( RES_PORT, RESERV2, RELAY_ON );
        EepromUpdate( EEP_RESERV2, RELAY_ON );
       } 
    if ( id_rel == RELAY_RES3 && cmnd == RELAY_ON ) {
        GPIO_PinWrite( RES_PORT, RESERV3, RELAY_ON );
        EepromUpdate( EEP_RESERV3, RELAY_ON );
       } 
    if ( id_rel == RELAY_RES4 && cmnd == RELAY_ON ) {
        GPIO_PinWrite( RES_PORT4, RESERV4, RELAY_ON );
        EepromUpdate( EEP_RESERV4, RELAY_ON );
       } 
    //выключение
    if ( id_rel == RELAY_RES1 && cmnd == RELAY_OFF ) {
        GPIO_PinWrite( RES_PORT, RESERV1, RELAY_OFF );
        EepromUpdate( EEP_RESERV1, RELAY_OFF );
        osTimerStop( timer_imp1 );
       } 
    if ( id_rel == RELAY_RES2 && cmnd == RELAY_OFF ) {
        GPIO_PinWrite( RES_PORT, RESERV2, RELAY_OFF );
        EepromUpdate( EEP_RESERV2, RELAY_OFF );
        osTimerStop( timer_imp2 );
       } 
    if ( id_rel == RELAY_RES3 && cmnd == RELAY_OFF ) {
        GPIO_PinWrite( RES_PORT, RESERV3, RELAY_OFF );
        EepromUpdate( EEP_RESERV3, RELAY_OFF );
        osTimerStop( timer_imp3 );
       } 
    if ( id_rel == RELAY_RES4 && cmnd == RELAY_OFF ) {
        GPIO_PinWrite( RES_PORT4, RESERV4, RELAY_OFF );
        EepromUpdate( EEP_RESERV4, RELAY_OFF );
        osTimerStop( timer_imp4 );
       }
    //импульсное включение
    if ( id_rel == RELAY_RES1 && cmnd == RELAY_PULSE ) {
        GPIO_PinWrite( RES_PORT, RESERV1, RELAY_ON );
        osTimerStart( timer_imp1, RELAY_PULSE1 );
        EepromUpdate( EEP_RESERV1, RELAY_OFF );
       } 
    if ( id_rel == RELAY_RES2 && cmnd == RELAY_PULSE ) {
        GPIO_PinWrite( RES_PORT, RESERV2, RELAY_ON );
        osTimerStart( timer_imp2, RELAY_PULSE1 );
        EepromUpdate( EEP_RESERV2, RELAY_OFF );
       } 
    if ( id_rel == RELAY_RES3 && cmnd == RELAY_PULSE ) {
        GPIO_PinWrite( RES_PORT, RESERV3, RELAY_ON );
        osTimerStart( timer_imp3, RELAY_PULSE1 );
        EepromUpdate( EEP_RESERV3, RELAY_OFF );
       } 
    if ( id_rel == RELAY_RES4 && cmnd == RELAY_PULSE ) {
        GPIO_PinWrite( RES_PORT4, RESERV4, RELAY_ON );
        osTimerStart( timer_imp4, RELAY_PULSE1 );
        EepromUpdate( EEP_RESERV4, RELAY_OFF );
       }
    EepromSave();
 }

//*************************************************************************************************
// Управление дополнительными выходами UP/DN/LF/RT
// RelayOut id_out - ID дополнительного выхода
// RelayCmnd cmnd  - команда управления
//*************************************************************************************************
void ExtOut( RelayOut id_out, RelayCmnd cmnd ) {

    //включение
    if ( id_out == EXT_OUT_UP && cmnd == RELAY_ON )
        GPIO_PinWrite( EXT_PORT, EXT_UP, RELAY_ON );
    if ( id_out == EXT_OUT_DN && cmnd == RELAY_ON )
        GPIO_PinWrite( EXT_PORT, EXT_DN, RELAY_ON );
    if ( id_out == EXT_OUT_LF && cmnd == RELAY_ON )
        GPIO_PinWrite( EXT_PORT, EXT_LF, RELAY_ON );
    if ( id_out == EXT_OUT_RT && cmnd == RELAY_ON )
        GPIO_PinWrite( EXT_PORT, EXT_RT, RELAY_ON );
    //выключение
    if ( id_out == EXT_OUT_UP && cmnd == RELAY_OFF ) {
        GPIO_PinWrite( EXT_PORT, EXT_UP, RELAY_OFF );
        osTimerStop( timer_imp5 );
       }
    if ( id_out == EXT_OUT_DN && cmnd == RELAY_OFF ) {
        GPIO_PinWrite( EXT_PORT, EXT_DN, RELAY_OFF );
        osTimerStop( timer_imp6 );
       }
    if ( id_out == EXT_OUT_LF && cmnd == RELAY_OFF ) {
        GPIO_PinWrite( EXT_PORT, EXT_LF, RELAY_OFF );
        osTimerStop( timer_imp7 );
       }
    if ( id_out == EXT_OUT_RT && cmnd == RELAY_OFF ) {
        GPIO_PinWrite( EXT_PORT, EXT_RT, RELAY_OFF );
        osTimerStop( timer_imp8 );
       }
    //импульсное включение
    if ( id_out == EXT_OUT_UP && cmnd == RELAY_PULSE ) {
        GPIO_PinWrite( EXT_PORT, EXT_UP, RELAY_ON );
        osTimerStart( timer_imp5, RELAY_PULSE1 );
       }
    if ( id_out == EXT_OUT_DN && cmnd == RELAY_PULSE ) {
        GPIO_PinWrite( EXT_PORT, EXT_DN, RELAY_ON );
        osTimerStart( timer_imp6, RELAY_PULSE1 );
       }
    if ( id_out == EXT_OUT_LF && cmnd == RELAY_PULSE ) {
        GPIO_PinWrite( EXT_PORT, EXT_LF, RELAY_ON );
        osTimerStart( timer_imp7, RELAY_PULSE1 );
       }
    if ( id_out == EXT_OUT_RT && cmnd == RELAY_PULSE ) {
        GPIO_PinWrite( EXT_PORT, EXT_RT, RELAY_ON );
        osTimerStart( timer_imp8, RELAY_PULSE1 );
       }
 }

//*************************************************************************************************
// Функция обратного вызова таймера выключения реле RESERV1
//*************************************************************************************************
static void Timer1Callback( void *arg ) {
    
    GPIO_PinWrite( RES_PORT, RESERV1, RELAY_OFF );
 }

//*************************************************************************************************
// Функция обратного вызова таймера выключения реле RESERV2
//*************************************************************************************************
static void Timer2Callback( void *arg ) {

    GPIO_PinWrite( RES_PORT, RESERV2, RELAY_OFF );
 }

//*************************************************************************************************
// Функция обратного вызова таймера выключения реле RESERV3
//*************************************************************************************************
static void Timer3Callback( void *arg ) {

    GPIO_PinWrite( RES_PORT, RESERV3, RELAY_OFF );
 }

//*************************************************************************************************
// Функция обратного вызова таймера выключения реле RESERV4
//*************************************************************************************************
static void Timer4Callback( void *arg ) {

    GPIO_PinWrite( RES_PORT4, RESERV4, RELAY_OFF );
 }

//*************************************************************************************************
// Функция обратного вызова таймера выключения выхода EXT_UP
//*************************************************************************************************
static void Timer5Callback( void *arg ) {

    GPIO_PinWrite( EXT_PORT, EXT_UP, RELAY_OFF );
 }

//*************************************************************************************************
// Функция обратного вызова таймера выключения выхода EXT_DN
//*************************************************************************************************
static void Timer6Callback( void *arg ) {

    GPIO_PinWrite( EXT_PORT, EXT_DN, RELAY_OFF );
 }

//*************************************************************************************************
// Функция обратного вызова таймера выключения выхода EXT_LF
//*************************************************************************************************
static void Timer7Callback( void *arg ) {

    GPIO_PinWrite( EXT_PORT, EXT_LF, RELAY_OFF );
 }

//*************************************************************************************************
// Функция обратного вызова таймера выключения выхода EXT_RT
//*************************************************************************************************
static void Timer8Callback( void *arg ) {

    GPIO_PinWrite( EXT_PORT, EXT_RT, RELAY_OFF );
 }
