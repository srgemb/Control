
#include "rtc.h"
#include "rl_fs.h"

//*************************************************************************************************
// Функция возвращает дату-время для файловых функций библиотеки RTX_LIB
//*************************************************************************************************
fsStatus fs_get_time( fsTime *time ) {

    static RTC_TIME_Type Time;
    
    RTC_GetFullTime( LPC_RTC, &Time );
    time->hr = Time.HOUR;
    time->min = Time.MIN;
    time->sec = Time.SEC;
    time->day = Time.DOM;
    time->mon = Time.MONTH;
    time->year = Time.YEAR;
    return fsOK;
 }
