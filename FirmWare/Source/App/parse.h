
#ifndef __PARSE_H
#define __PARSE_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_CNT_PARAM       21          //максимальное кол-во параметров, включая команду
#define MAX_LEN_PARAM       16          //максимальный размер (длина) параметра в командной строке

//индексы для получения параметров команды
typedef enum {
    IND_PAR_CMND,                       //команда
    IND_PARAM1,                         //параметр 1
    IND_PARAM2,                         //параметр 2
    IND_PARAM3,                         //параметр 3
    IND_PARAM4,                         //параметр 4
    IND_PARAM5,                         //параметр 5
    IND_PARAM6,                         //параметр 6
    IND_PARAM7,                         //параметр 7
    IND_PARAM8,                         //параметр 8
    IND_PARAM9,                         //параметр 9
    IND_PARAM10,                        //параметр 10
    IND_PARAM11,                        //параметр 11
    IND_PARAM12,                        //параметр 12
    IND_PARAM13,                        //параметр 13
    IND_PARAM14,                        //параметр 14
    IND_PARAM15,                        //параметр 15
    IND_PARAM16,                        //параметр 16
    IND_PARAM17,                        //параметр 17
    IND_PARAM18,                        //параметр 18
    IND_PARAM19,                        //параметр 19
    IND_PARAM20                         //параметр 20
 } CmndParam;

//*************************************************************************************************
// Функции статуса/состояния
//*************************************************************************************************
uint8_t ParseCommand( char *src );
uint8_t GetParamCnt( void );
char *GetParamVal( CmndParam index );
char *GetParamList( void );

#endif
