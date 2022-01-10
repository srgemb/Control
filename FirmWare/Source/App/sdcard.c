
//*************************************************************************************************
//
// Функционал работы с SD картой
//
//*************************************************************************************************

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <lpc_types.h>

#include "rl_fs.h"
#include "cmsis_os2.h"

#include "mci_lpc177x_8x.h"

#include "command.h"
#include "sdcard.h"
#include "charger.h"
#include "mppt.h"
#include "uart.h"
#include "scheduler.h"
#include "batmon.h"
#include "eeprom.h"
#include "parse.h"
#include "outinfo.h"
#include "ports.h"
#include "gen.h"
#include "spa_calc.h"
#include "inverter.h"
#include "main.h"
#include "message.h"

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define SD_DRIVE            "M0:"

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static Status sd_mount = ERROR;

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void CheckMkDir( void );
static char *LowerCase( char *str );
static void DumpHex( uint32_t addr, uint8_t *data, uint16_t cnt );

//*************************************************************************************************
// 1. Монтирование карты и файловой системы.
// 2. Создание каталогов.
// 3. Загрузка шаблона экрана.
// 4. Загрузка заданий планировщика.
// return = ERROR  - ошибка монтирования или отсутствие карты
//          SUCCES - монтирование карты выполнено
//*************************************************************************************************
Status SDMount( void ) {

    fsStatus fstat;
    uint32_t ser_num;
    char msg[80], label[12];

    if ( !SDDetect() ) {
        sd_mount = ERROR;
        ConsoleSend( MessageSd( MSG_SD_NO ), CONS_NORMAL );
        return ERROR;
       }
    GPIO_PinWrite( RTE_SD_PWR_PORT, RTE_SD_PWR_PIN, RTE_SD_PWR_ACTIVE );
    //инициализация файловой системы
    ConsoleSend( MessageSd( MSG_SD_FSYS_INIT ), CONS_NORMAL );
    fstat = finit( SD_DRIVE );
    if ( fstat == fsOK ) {
        fstat = fmount( SD_DRIVE );
        if ( fstat == fsOK ) 
            ConsoleSend( MessageSd( MSG_SD_MOUNT ), CONS_NORMAL );
        else {
            if ( fstat == fsNoFileSystem )
                ConsoleSend( MessageSd( MSG_SD_FSYS_NOSYS ), CONS_NORMAL );
            else {
                sprintf( msg, MessageSd( MSG_SD_ERR_MOUNT ), fstat );
                ConsoleSend( msg, CONS_NORMAL );
               } 
           } 
       }
    else ConsoleSend( MessageSd( MSG_SD_FSYS_ERR_INIT ), CONS_NORMAL );
    //параметры диска
    if ( fstat == fsOK ) {
        if ( fvol( SD_DRIVE, label, &ser_num ) == fsOK ) {
            if ( strlen( label ) )
                sprintf( msg, MessageSd( MSG_SD_LABEL ), label );
            ConsoleSend( msg, CONS_NORMAL );
            sprintf( msg, MessageSd( MSG_SD_SERIAL ), ser_num );
            ConsoleSend( msg, CONS_NORMAL );
           }
        else ConsoleSend( MessageSd( MSG_SD_ERR_LABEL ), CONS_NORMAL );
       } 
    if ( fstat == fsOK ) {
        sd_mount = SUCCESS;
        //загрузка параметров с SD карты
        CheckMkDir();
        ScreenLoad();
        LoadJobs();
        return SUCCESS;
       }
    else {
        sd_mount = ERROR;
        return ERROR;
       }
 }

//*************************************************************************************************
// Размонтирование карты и файловой системы
//*************************************************************************************************
Status SDUnMount( void ) {

    fsStatus fstat;
    
    if ( !SDDetect() ) {
        sd_mount = ERROR;
        ConsoleSend( MessageSd( MSG_SD_NO ), CONS_NORMAL );
        return ERROR;
       }
    if ( !SDMountStat() ) {
        ConsoleSend( MessageSd( MSG_SD_UNMOUNT_EXEC ), CONS_NORMAL );
        return SUCCESS;
       }
    fstat = funmount( SD_DRIVE );
    if ( fstat == fsOK ) {
        sd_mount = ERROR;
        ConsoleSend( MessageSd( MSG_SD_UNMOUNT_OK ), CONS_NORMAL );
        fstat = funinit( SD_DRIVE );
        if ( fstat == fsOK ) {
            GPIO_PinWrite( RTE_SD_PWR_PORT, RTE_SD_PWR_PIN, !RTE_SD_PWR_ACTIVE );
            ConsoleSend( MessageSd( MSG_SD_FSYS_UNMOUNT ), CONS_NORMAL );
           }
        else ConsoleSend( MessageSd( MSG_SD_FSYS_ERR_UNMOUNT ), CONS_NORMAL );
       }
    else ConsoleSend( MessageSd( MSG_SD_ERR_UNMOUNT ), CONS_NORMAL );
    if ( fstat == fsOK )
        return SUCCESS;
    else return ERROR;
 }

//*************************************************************************************************
// Возвращает статус монтирования карты и файловой системы
// return Status  - состояние монтирования SD карты
//        SUCCESS - SD карты установлена, файловая система смонтирована
//        ERROR   - карты нет или файловая система не смонтирована
//*************************************************************************************************
Status SDMountStat( void ) {

    return sd_mount;
 }

//*************************************************************************************************
// Возвращает статус наличия SD карты и смонтированной файловой системы
// return Status  - состояние монтирования SD карты
//        SUCCESS - SD карты установлена, файловая система смонтирована
//        ERROR   - карты нет или файловая система не смонтирована
//*************************************************************************************************
Status SDStatus( void ) {

    if ( SDDetect() == false )
        return ERROR;   //карты нет
    if ( !SDMountStat() )
        return ERROR;   //ФС не смонтирована
    return SUCCESS;
 }

//*************************************************************************************************
// Вывод информации о SD карте
//*************************************************************************************************
void SDCid( void ) {

    int32_t id;
    char msg[50];
    fsCID_Register cid;

    id = fs_ioc_get_id( SD_DRIVE );
    if ( id >= 0 && ( fs_ioc_lock( id ) == fsOK ) ) {
        if ( fs_ioc_device_ctrl( id, fsDevCtrlCodeGetCID, &cid ) == fsOK ) {
            sprintf( msg, MessageSd( MSG_SD_ID_VENDOR ), cid.MID, cid.MID );
            ConsoleSend( msg, CONS_NORMAL );
            sprintf( msg, MessageSd( MSG_SD_OEMID ), cid.OID >> 8, cid.OID & 0xFF );
            ConsoleSend( msg, CONS_NORMAL );
            sprintf( msg, MessageSd( MSG_SD_NAMEPROD ), cid.PNM[0], cid.PNM[1], cid.PNM[2], cid.PNM[3], cid.PNM[4] );
            ConsoleSend( msg, CONS_NORMAL );
            sprintf( msg, MessageSd( MSG_SD_REVISION ), cid.PRV >> 4, cid.PRV & 0x0F );
            ConsoleSend( msg, CONS_NORMAL );
            sprintf( msg, MessageSd( MSG_SD_SERIAL2 ), cid.PSN );
            ConsoleSend( msg, CONS_NORMAL );
            sprintf( msg, MessageSd( MSG_SD_DATE ), cid.MDT & 0x0F, cid.MDT >> 4 );
            ConsoleSend( msg, CONS_NORMAL );
           }
        else ConsoleSend( MessageSd( MSG_SD_ERR_CID ), CONS_NORMAL );
        fs_ioc_unlock( id );
       }
    else {
        sprintf( msg, MessageSd( MSG_SD_NODRIVER ), SD_DRIVE );
        ConsoleSend( msg, CONS_NORMAL );
       }
 }

//*************************************************************************************************
// Проверка/создание каталогов на SD карте
//*************************************************************************************************
static void CheckMkDir( void ) {

    fmkdir( "\\mppt" );
    fmkdir( "\\batmon" );
    fmkdir( "\\charger" );
    fmkdir( "\\inv" );
    fmkdir( "\\gen" );
    fmkdir( "\\alt" );
    fmkdir( "\\trc" );
    fmkdir( "\\hmi" );
    fmkdir( "\\voice" );
    fmkdir( "\\execute" );
 }

//*************************************************************************************************
// Удаление файла, группы файлов по маске. Маска файлов: filename.*, file*.ext
// char *file_name - указатель на имя файла
// return Status   - результат выполнения
//        SUCCESS  - удаление файла выполнено
//        ERROR    - ошибка удаления файла
//*************************************************************************************************
Status FileDelete( char *fname ) {

    char path[80], full[80], *find;
    fsStatus fstat;
    fsFileInfo info;

    if ( fname == NULL )
        return ERROR; //имя не указано
    //проверим наличие маски в имени файла/каталога
    if ( strchr( fname, '*' ) == NULL ) {
        //удаление одиночного файла
        fstat = fdelete( fname, NULL );
        if ( fstat == fsOK ) {
            ConsoleSend( MessageSd( MSG_FILE_DELETED ), CONS_NORMAL );
            return SUCCESS;
           }
        else {
            if ( fstat == fsFileNotFound )
                ConsoleSend( MessageSd( MSG_FILE_NOT_FOUND ), CONS_NORMAL );
            else ConsoleSend( MessageSd( MSG_FILE_ERR_DELETED ), CONS_NORMAL );
            return ERROR;
           }
       }
    else {
        //удаление файлов по маске
        while ( ffind( fname, &info ) == fsOK ) {
            if ( info.attrib & FS_FAT_ATTR_DIRECTORY )
                continue; //каталоги пропускаем
            else {
                //выделим путь к файлам
                memset( path, 0x00, sizeof( path ) );
                memset( full, 0x00, sizeof( full ) );
                find = strrchr( fname, '\\' );
                if ( find != NULL )
                    strncpy( path, fname, find - fname + 1 );
                //формируем имя удаляемого файла с путем
                strcpy( full, path );
                strcat( full, info.name );
                ConsoleSend( LowerCase( full ), CONS_NORMAL );
                ConsoleSend( " ... ", CONS_NORMAL );
                //удаление одиночного файла
                fstat = fdelete( full, NULL );
                if ( fstat == fsOK ) 
                    ConsoleSend( MessageSd( MSG_FILE_DELETED ), CONS_NORMAL );
                else {
                    if ( fstat == fsFileNotFound )
                        ConsoleSend( MessageSd( MSG_FILE_NOT_FOUND ), CONS_NORMAL );
                    else ConsoleSend( MessageSd( MSG_FILE_ERR_DELETED ), CONS_NORMAL );
                   }
               }
           }
        return SUCCESS;
       }
 }

//*************************************************************************************************
// Удаление каталога
// char *dir_name - имя каталога
// return Status  - результат выполнения
//        SUCCESS - удаление каталога выполнено
//        ERROR   - ошибка удаления каталога
//*************************************************************************************************
Status DirDelete( char *dir_name ) {

    fsStatus fstat;

    if ( dir_name == NULL )
        return ERROR; //имя не указано
    fstat = frmdir( dir_name, NULL );
    if ( fstat == fsOK ) {
        ConsoleSend( MessageSd( MSG_DIR_DEL ), CONS_NORMAL );
        return SUCCESS;
       }
    else {
        if ( fstat == fsFileNotFound )
            ConsoleSend( MessageSd( MSG_DIR_DEL_NFOUND ), CONS_NORMAL );
        else ConsoleSend( MessageSd( MSG_DIR_ERR_DEL ), CONS_NORMAL );
        return ERROR;
       }
 }

//*************************************************************************************************
// Переименование файла
// char *file_name - текущее имя файла
// char *new_name  - новое имя файла
// return Status   - результат выполнения
//        SUCCESS  - удаление каталога выполнено
//        ERROR    - ошибка удаления каталога
//*************************************************************************************************
Status FileRename( char *fname, char *new_name ) {

    if ( frename( fname, new_name ) == fsOK ) {
        ConsoleSend( MessageSd( MSG_FILE_RENAME ), CONS_NORMAL );
        return SUCCESS;
       }
    else {
        ConsoleSend( MessageSd( MSG_FILE_ERR_RENAME ), CONS_NORMAL );
        return ERROR;
       }
 }

//*************************************************************************************************
// Вывод каталога SD карты
// char *mask - путь с маской для вывода содержимого карты
//*************************************************************************************************
void SDDir( char *mask ) {

    uint64_t fsize;
    uint32_t files, dirs, i;
    char temp[32], msg[100], path[32], ch;
    fsFileInfo info;

    if ( !strlen( mask ) )
        strcpy( path, "*.*" ); //параметра нет, добавим маску
    else strcpy( path, mask );
    ConsoleSend( MessageSd( MSG_DIR ), CONS_NORMAL );
    ConsoleSend( path, CONS_NORMAL );
    ConsoleSend( Message( CONS_MSG_CRLF ), CONS_NORMAL );
    ConsoleSend( Message( CONS_MSG_HEADER ), CONS_NORMAL );
    files = 0;
    dirs  = 0;
    fsize = 0;
    info.fileID  = 0;
    while ( ffind( path, &info ) == fsOK ) {
        if ( info.attrib & FS_FAT_ATTR_DIRECTORY ) {
            //вывод атрибутов каталога
            i = 0;
            while ( strlen((const char *)info.name+i ) > 41 ) {
                ch = info.name[i+41];
                info.name[i+41] = 0;
                sprintf( msg, "%-41s", &info.name[i] );
                ConsoleSend( msg, CONS_NORMAL );
                info.name[i+41] = ch;
                i += 41;
               }
            sprintf( msg, MessageSd( MSG_DIR_NAME ), &info.name[i] );
            ConsoleSend( msg, CONS_NORMAL );
            sprintf( msg, " %02d.%02d.%04d  %02d:%02d:%02d\r\n", info.time.day, info.time.mon, info.time.year, info.time.hr, info.time.min, info.time.sec );
            ConsoleSend( msg, CONS_NORMAL );
            dirs++;
           }
        else {
            //вывод атрибутов файла
            FormatDot( info.size, temp );
            i = 0;
            while ( strlen( (const char *)info.name+i ) > 41 ) {
                ch = info.name[i+41];
                info.name[i+41] = 0;
                sprintf( msg, "%-41s", LowerCase( &info.name[i] ) );
                ConsoleSend( msg, CONS_NORMAL );
                info.name[i+41] = ch;
                i += 41;
               }
            sprintf( msg, "%-41s %14s", LowerCase( &info.name[i] ), temp );
            ConsoleSend( msg, CONS_NORMAL );
            sprintf( msg, " %02d.%02d.%04d  %02d:%02d:%02d\r\n", info.time.day, info.time.mon, info.time.year, info.time.hr, info.time.min, info.time.sec );
            ConsoleSend( msg, CONS_NORMAL );
            fsize += info.size;
            files++;
           }
       }
    if ( info.fileID == 0 ) {
        ConsoleSend( Message( CONS_MSG_HEADER ), CONS_NORMAL );
        ConsoleSend( MessageSd( MSG_DIR_NO_FILE ), CONS_NORMAL );
       } 
    else {
        FormatDot( fsize, temp );
        ConsoleSend( Message( CONS_MSG_HEADER ), CONS_NORMAL );
        sprintf( msg, MessageSd( MSG_DIR_CNT_FILES ), files, temp );
        ConsoleSend( msg, CONS_NORMAL );
       }
    FormatDot( ffree( path ), temp );
    if ( dirs ) {
        ConsoleSend( Message( CONS_MSG_HEADER ), CONS_NORMAL );
        sprintf( msg, MessageSd( MSG_DIR_CNT_BYTE ), dirs, temp );
        ConsoleSend( msg, CONS_NORMAL );
       }
    else {
        ConsoleSend( Message( CONS_MSG_HEADER ), CONS_NORMAL );
        sprintf( msg,MessageSd( MSG_DIR_FREE ), temp );
        ConsoleSend( msg, CONS_NORMAL );
       }
 }

//*************************************************************************************************
// Вывод файла на консоль
// char *file_name - имя файла 
//*************************************************************************************************
void FileType( char *fname ) {

    FILE *ftype;
    char str[120];

    if ( fname == NULL || !strlen( fname ) )
        return;
    ftype = fopen( fname, "r" );               
    if ( ftype == NULL ) {
        sprintf( str, MessageSd( MSG_FT_NOT_OPEN ), fname );
        ConsoleSend( str, CONS_NORMAL );
        return;
       }
    //шапка вывода
    sprintf( str, MessageSd( MSG_FT_FPAGE ), fname );
    ConsoleSend( str, CONS_NORMAL );
    ConsoleSend( Message( CONS_MSG_HEADER ), CONS_NORMAL );
    //вывод данных
    while ( fgets( str, sizeof( str ), ftype ) != NULL )
        ConsoleSend( str, CONS_NORMAL ); 
    fclose( ftype );
    //завершение вывода
    ConsoleSend( MessageSd( MSG_FT_EOF ), CONS_NORMAL );
    ConsoleSend( Message( CONS_MSG_HEADER ), CONS_NORMAL );
 }

//*************************************************************************************************
// Вывод файла в формате HEX (дамп)
// char *fname - имя файла 
//*************************************************************************************************
void FileHex( char *fname ) {

    FILE *fhex;
    char str[128];
    uint16_t cnt;
    uint8_t data[16*16];
    uint32_t address = 0;

    if ( fname == NULL || !strlen( fname ) )
        return;
    fhex = fopen( fname, "r" );               
    if ( fhex == NULL ) {
        sprintf( str, MessageSd( MSG_FT_NOT_OPEN ), fname );
        ConsoleSend( str, CONS_NORMAL );
        return;
       }
    //формируем шапку
    sprintf( str, MessageSd( MSG_FT_FPAGE ), fname );
    ConsoleSend( str, CONS_NORMAL );
    ConsoleSend( MessageSd( MSG_FT_HEADER_HEX ), CONS_NORMAL );
    //чтение, вывод данных
    while ( !feof( fhex ) ) {
        cnt = fread( data, sizeof( uint8_t ), sizeof( data ), fhex );
        DumpHex( address, data, cnt );
        address += 256;
       }
    fclose( fhex );
    //завершение вывода
    ConsoleSend( MessageSd( MSG_FT_EOF ), CONS_NORMAL );
    ConsoleSend( Message( CONS_MSG_HEADER ), CONS_NORMAL );
 }

//*************************************************************************************************
// Выводит блок данных в формате HEX дампа
// Вывод выполняется всего блока *data, по одной строке
// uint32_t addr - адрес для вывода в строку
// uint8_t *data - указатель на блок данных для вывода
// uint16_t cnt  - кол-во байт для вывода
//*************************************************************************************************
static void DumpHex( uint32_t addr, uint8_t *data, uint16_t cnt ) {

    uint8_t ch, ichr = 0, chr = 16;
    char *ptr, str[120];
    uint16_t byte, idx, offset = 15, space = 0;
    
    ptr = str;
    for ( byte = 0; byte < cnt; byte++, addr++ ) {
        if ( !( addr & 0x0000000F ) ) { //вывод адреса строки
            ptr += sprintf( ptr, "0x%08X: ", addr ); 
            //остаток байт для вывода
            if ( ( cnt - byte ) < 16 && !space ) {
                //неполная строка, расчет отступа для выравнивания вывода символов
                ichr = chr = idx = cnt - byte;
                offset = chr - 1;
                if ( idx < 8 )
                    space = ( 16 - idx ) * 3 + 2;
                else space = ( 16 - idx ) * 3 + 1;
               }
           }
        //выводим код в формате HEX
        if ( ( byte & 0x0007 ) == 7 ) 
            ptr += sprintf( ptr, "%02X  ", *( data + byte ) );
        else ptr += sprintf( ptr, "%02X ", *( data + byte ) );
        //
        if ( ( byte & 0x000F ) != 0x000F && !ichr )
            continue; //продолжаем вывод, полная строка (16 байт)
        if ( ichr )
            if ( --ichr )
                continue; //продолжаем вывод, остаток строки
        if ( space ) {
            //заполним пробелом для последующего вывода символов
            memset( ptr, 0x20, space );
            ptr += space;
           }
        //выводим символы
        for ( idx = 0; idx < chr; idx++ ) {
            ch = *( data + byte + idx - offset );
            if ( ch >= 32 )
                ptr += sprintf( ptr, "%c", ch );
            else ptr += sprintf( ptr, "." );
           }
        ptr += sprintf( ptr, "\r\n" );
        if ( ( byte & 0x00FF ) == 0x00FF )
            ptr += sprintf( ptr, "\r\n" ); //отделим блок 16*16
        ConsoleSend( str, CONS_NORMAL );
        ptr = str; //новая строка
        memset( str, 0x00, sizeof( str ) );
       }
 }

//*************************************************************************************************
// Форматированный вывод числа с разделителями по группам
// uint64_t value - значение для форматирования
// char *dest     - указатель для размещения результата
//*************************************************************************************************
void FormatDot( uint64_t value, char *dest ) {

    if ( value >= (uint64_t)1e12 ) {
        dest += sprintf( dest, "%d,", (uint32_t)( value/(uint64_t)1e12) );
        value %= (uint64_t)1e12;
        dest += sprintf( dest, "%03d,", (uint32_t)( value/(uint64_t)1e9) );
        value %= (uint64_t)1e9;
        dest += sprintf( dest, "%03d,", (uint32_t)( value/(uint64_t)1e6) );
        value %= (uint64_t)1e6;
        sprintf( dest, "%03d,%03d", (uint32_t)( value/1000),(uint32_t)( value%1000 ) );
        return;
       }
    if ( value >= (uint64_t)1e9 ) {
        dest += sprintf( dest, "%d,", (uint32_t)( value/(uint64_t)1e9) );
        value %= (uint64_t)1e9;
        dest += sprintf( dest, "%03d,", (uint32_t)( value/(uint64_t)1e6) );
        value %= (uint64_t)1e6;
        sprintf( dest, "%03d,%03d", (uint32_t)( value/1000),(uint32_t)( value%1000 ) );
        return;
       }
    if ( value >= (uint64_t)1e6 ) {
        dest += sprintf( dest, "%d,", (uint32_t)( value/(uint64_t)1e6));
        value %= (uint64_t)1e6;
        sprintf( dest,"%03d,%03d", (uint32_t)( value/1000 ),(uint32_t)( value%1000 ) );
        return;
       }
    if ( value >= 1000 ) {
        sprintf( dest, "%d,%03d", (uint32_t)( value/1000 ),(uint32_t)( value%1000 ) );
        return;
       }
    sprintf( dest, "%d", (uint32_t)(value) );
 }

//*************************************************************************************************
// Преобразует строку в нижний регистр
// char *str - исходная строка
// return    - результирующая строка в нижнем регистре
//*************************************************************************************************
static char *LowerCase( char *str ) {

    char *result;
    
    result = str;
    while ( *str != NULL ) {
        if ( isupper( *str ) )
            *str = tolower( *str );
        str++;
       }
    return result;
 }
