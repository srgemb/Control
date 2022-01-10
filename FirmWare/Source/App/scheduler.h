
#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

#include "scheduler_def.h"

//Команды управления заданиями
typedef enum {
    JOBS_VIEW,                              //просмотр заданий
    JOBS_ADD,                               //добавление задания
    JOBS_DEL,                               //удаление задания
    JOBS_ON,                                //включение задания
    JOBS_OFF,                               //выключение задания
    JOBS_RUN                                //задания готовые в выполнению (без параметров запуска)
 } JobsCmnd;

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void JobInit( void );
void LoadJobs( void );
void JobsView( void );
void JobsEdit( JobsCmnd mode, uint8_t numb_str, char *job_text, uint8_t flg_all );

#endif
