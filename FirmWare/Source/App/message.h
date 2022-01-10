
#ifndef __MESSAGE_H
#define __MESSAGE_H

#include "device.h"
#include "mess_log.h"

//*************************************************************************************************
// ID cообщений консоли
//*************************************************************************************************
typedef enum {
    CONS_MSG_PROMPT,                        //>
    CONS_MSG_PROMPT2,                       //#
    CONS_MSG_OK,                            //OK
    CONS_MSG_CRLF,                          //\r\n
    CONS_MSG_ERR_CMND,                      //Неверная команда.
    CONS_MSG_ERR_NOJOB,                     //Команда не выполняется из планировщика.
    CONS_MSG_ERR_PARAM,                     //Неверный параметр.
    CONS_MSG_ERR_NOPARAM,                   //Параметр(ы) не указан.
    CONS_MSG_ERR_NODEV,                     //Устройства нет в сети.
    CONS_MSG_ERR_SEND,                      //Ошибка передачи.
    CONS_MSG_ERR_NONAME,                    //Имя файла не указано.
    CONS_MSG_ERR_FOPEN,                     //Ошибка открытия файла.
    CONS_MSG_ERR_NOTENPAR,                  //Недостаточно параметров.
    CONS_MSG_ERR_VALUE,                     //Недопустимое значение параметров.
    CONS_MSG_ERR_DESC,                      //Ошибка: %s
    CONS_MSG_ERR_NOLINK,                    //Контроллер не подключен.
    CONS_MSG_LOAD_SCREEN,                   //\rЗагрузка файла экрана: %s ...
    CONS_MSG_FILE_JOBS,                     //\rЗагрузка файла заданий: %s ...
    CONS_MSG_ERR_LIMITSTR,                  //\rПревышение лимита строк.
    CONS_MSG_ERR_JOB,                       //Задание не указано.
    CONS_MSG_ERR_JOB_OPEN,                  //Временный файл: jobs~ не открывается.
    CONS_MSG_ERR_JOB_RENAME,                //Временный файл: jobs~ не переименован.
    CONS_MSG_ERR_JOB_BYTE,                  //!!! Превышение лимита строк (байт) !!!
    CONS_MSG_FILE,                          //Файл:
    CONS_MSG_NOT_DELETED,                   //не удален.
    CONS_MSG_NOT_OPEN,                      //не открывается.
    CONS_ERR_BUFF_FULL,                     //Превышение размера буфера.
    CONS_MSG_CHARGE_DAY,                    //От полного заряда АКБ прошло только: %u дней
    CONS_MSG_JOB_HDR1,                      //====================================================================
    CONS_MSG_JOB_HDR2,                      //        ..........1.........2.........3.........4.........5.........
    CONS_MSG_JOB_HDR3,                      //        012345678901234567890123456789012345678901234567890123456789
    CONS_MSG_JOB_HDR4,                      //--------------------------------------------------------------------
    CONS_MSG_HEADER,                        //--------------------------------- ...
    CONS_MSG_TASK_HDR1,                     //Name thread     Priority  State      Stack Unused
    CONS_MSG_TASK_HDR2,                     //----------------------------------------------------
    CONS_MSG_MPPT_HEADER1,                  //00 01 02 03 04 05 06 07 08 09 ...
    CONS_MSG_MPPT_HEADER2,                  //----------------------------- ...
    CONS_MSG_MPPT_HEADER3,                  //xx xx xx xx ----- ----- ----- ...
    CONS_MSG_MPPT_HEADER4,                  //            00003 00001 00004 ...
    CONS_MSG_MPPT_HEADER5,                  //============================= ...
    CONS_MSG_APP_VER,                       //Application version ......... %s
    CONS_MSG_BUILD_VER,                     //Build version ............... %s %s
    CONS_MSG_CPU_CLOCK,                     //CPU clock ................... %s Hz
    CONS_MSG_CLOCK_PER,                     //Clock periphery ............. %s Hz
    CONS_MSG_CLOCK_KERNEL,                  //Clock of the kernel tick .... %s Hz
    CONS_MSG_CLOCK_SYSTIMER,                //Clock of the system timer ... %s Hz
    CONS_MSG_KERNEL_INFO,                   //Kernel information .......... %s
    CONS_MSG_KERNEL_VER,                    //Kernel version .............. %d.%d.%d
    CONS_MSG_KERNEL_API,                    //Kernel API version .......... %d.%d.%d
    CONS_MSG_MODBUS_ERR                     //Total  Dev01  Dev02  Dev03  Dev04  Dev05  Dev06  Dev07
 } ConsMessage;

//*************************************************************************************************
// ID cообщений функционала работы с SD картой
//*************************************************************************************************
typedef enum {                              
    MSG_SD_NO,                              //SD карта не доступна.
    MSG_SD_MOUNT,                           //SD карта смонтирована.
    MSG_SD_LABEL,                           //Метка SD карты: %s
    MSG_SD_SERIAL,                          //Серийный номер SD карты: %X
    MSG_SD_ERR_LABEL,                       //Ошибка доступа к метке диска.
    //
    MSG_SD_UNMOUNT_OK,                      //Размонтирование SD карты - OK.
    MSG_SD_UNMOUNT_EXEC,                    //Размонтирование SD карты уже выполнено.
    MSG_SD_ERR_MOUNT,                       //Ошибка монтирования SD карты, код: %d
    MSG_SD_ERR_UNMOUNT,                     //Ошибка размонтирования SD карты.
    //
    MSG_SD_FSYS_INIT,                       //Инициализация файловой системы ...
    MSG_SD_FSYS_NOSYS,                      //На SD карте нет файловой системы.
    MSG_SD_FSYS_ERR_INIT,                   //Ошибка инициализации файловой системы.
    MSG_SD_FSYS_UNMOUNT,                    //Размонтирование файловой системы - OK.
    MSG_SD_FSYS_ERR_UNMOUNT,                //Ошибка размонтирование файловой системы.
    //
    MSG_SD_ID_VENDOR,                       // ID производителя: %d (0x%.2X)
    MSG_SD_OEMID,                           //           OEM ID: %c%c
    MSG_SD_NAMEPROD,                        //     Имя продукта: %c%c%c%c%c
    MSG_SD_REVISION,                        // Ревизия продукта: %d.%d
    MSG_SD_SERIAL2,                         //   Серийный номер: 0x%X
    MSG_SD_DATE,                            //Дата производства: %d/%.2d
    MSG_SD_ERR_CID,                         //Ошибка чтения CID регистра.
    MSG_SD_NODRIVER,                        //Драйвер %s не существует.
    //
    MSG_FILE_DELETED,                       //Файл удален.
    MSG_FILE_NOT_FOUND,                     //Файл не найден.
    MSG_FILE_ERR_DELETED,                   //Ошибка удаления файла.
    //
    MSG_DIR_DEL,                            //Каталог удален.
    MSG_DIR_DEL_NFOUND,                     //Каталог не найден.
    MSG_DIR_ERR_DEL,                        //Ошибка удаления Каталога.
    //
    MSG_FILE_RENAME,                        //Файл переименован.
    MSG_FILE_ERR_RENAME,                    //Ошибка переименования файла.
    //
    MSG_DIR,                                //Список файлов M0:
    MSG_DIR_NAME,                           //%-41s    <Каталог>
    MSG_DIR_CNT_FILES,                      //%9d файлов  %21s байт
    MSG_DIR_CNT_BYTE,                       //%9d Каталогов   %21s байт доступно
    MSG_DIR_FREE,                           //%56s байт доступно
    MSG_DIR_NO_FILE,                        //Файлов нет.\r",                                      
    //
    MSG_FT_NOT_OPEN,                        //Файл: %s не открывается.
    MSG_FT_EOF,                             //\r<Конец файла>
    MSG_FT_FPAGE,                           //\rФайл: %s Страница: %d
    MSG_FT_HEADER_HEX                       //-----------00-01-02-03-04-05-06-07...
 } SdMessId;

//*************************************************************************************************
// Функции статуса/состояния
//*************************************************************************************************
char *MessageHelp( void );
char *Message( ConsMessage id_mess );
char *MessageLog( Device dev, LogMessId id_mess );
char *MessageSd( SdMessId id_mess );

#endif

