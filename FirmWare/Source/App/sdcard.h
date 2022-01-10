
#ifndef __SDCARD_H
#define __SDCARD_H

#include <stdbool.h>
#include <stdio.h>
#include <lpc_types.h>

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
Status SDMount( void );
Status SDUnMount( void );
Status SDMountStat( void );
Status SDStatus( void );
void SDCid( void );
void SDDir( char *mask );
Status FileDelete( char *fname );
Status DirDelete( char *dir_name );
Status FileRename( char *fname, char *new_name );
void FileType( char *fname );
void FileHex( char *fname );

//*************************************************************************************************
// Функции статуса/состояния
//*************************************************************************************************
void FormatDot( uint64_t value, char *dest );

#endif 
