
//*************************************************************************************************
// Константы модуля планировщика заданий
//*************************************************************************************************

#ifndef __SCHEDULER_DEF_H
#define __SCHEDULER_DEF_H

#include <stdint.h>
#include <stdbool.h>

#include "bitstring.h"

#define STR_END                 '\0'        //конец строки

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define SIZE_BUFFER             255         //размер буфера для загрузки одного задания
#define MAX_TEMPSTR             10          //размер временного буфера для обработки чисел
#define MAX_JOBS                70          //максимальное кол-во заданий в файле

#define MAX_LEN_JOBCMMND        80          //максимальная длина команды в задании

#define PPC_NULL                ((const char **)NULL)

#define FIRST_SECONDS           0
#define LAST_SECONDS            59
#define SECONDS_COUNT           (LAST_SECONDS - FIRST_SECONDS + 1)

#define FIRST_MINUTE            0
#define LAST_MINUTE             59
#define MINUTE_COUNT            (LAST_MINUTE - FIRST_MINUTE + 1)

#define FIRST_HOUR              0
#define LAST_HOUR               23
#define HOUR_COUNT              (LAST_HOUR - FIRST_HOUR + 1)

#define FIRST_DAY               1
#define LAST_DAY                31
#define DAY_COUNT               (LAST_DAY - FIRST_DAY + 1)

#define FIRST_MONTH             1
#define LAST_MONTH              12
#define MONTH_COUNT             (LAST_MONTH - FIRST_MONTH + 1)

//DOW: 0 и 7 - оба воскресенья по соображениям совместимости
#define FIRST_DOW               0
#define LAST_DOW                7
#define DOW_COUNT               (LAST_DOW - FIRST_DOW + 1)

//дополнительные флаги для заданий
#define DAY_STAR                0x01        //аналог параметра '*'
#define DOW_STAR                0x02        //аналог параметра '*'
#define WHEN_REBOOT             0x04        //признак запуска задания при включении
#define JOB_PRESENT             0x08        //признак обычного задания, без признака '@'
#define JOB_QUE_RUN             0x10        //признак наличия задачи в очереди
#define JOB_PARAM               0x20        //признак наличия параметра (для вывода в log файл)

#pragma pack( push, 1 )

//структура одного задания
typedef	struct {
    bitstr_t BitDecl( seconds, SECONDS_COUNT ); //маска секунд
    bitstr_t BitDecl( minute,  MINUTE_COUNT );  //маска минут
    bitstr_t BitDecl( hour,    HOUR_COUNT );    //маска часов
    bitstr_t BitDecl( day,     DAY_COUNT );     //маска дней
    bitstr_t BitDecl( month,   MONTH_COUNT );   //маска месяцев
    bitstr_t BitDecl( dow,     DOW_COUNT );     //маска дня недели
    char command[MAX_LEN_JOBCMMND];             //текст команды для выполнения
    uint8_t flags;                              //флаг задания см. "дополнительные флаги для заданий"
} JOB;

//структура очереди заданий
/*typedef struct {
    char command[MAX_LEN_JOBCMMND];         //текст команды для выполнения
    uint8_t flags;
} JOB_QUE;*/

#pragma pack( pop )

#define	SkipBlanks( c ) \
            while ( c == '\t' || c == ' ' ) c = GetChar();

#define	SkipNonblanks( c ) \
            while ( c != '\t' && c != ' ' && c != '\n' && c != STR_END ) c = GetChar();

#define	SkipLine( c ) \
            do { c = GetChar(); } while ( c != '\n' && c != STR_END );

#define MkLower( ch ) ( isupper( ch ) ? tolower( ch ) : ch )
#define MkUpper( ch ) ( islower( ch ) ? toupper( ch ) : ch )

#endif
