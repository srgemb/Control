
#ifndef __MAX3100_H
#define __MAX3100_H

#include <stdbool.h>
#include <stdint.h>

#define FOSC_36864                          //частота кварца MAX3100

//*************************************************************************************************
// Команды MAX3100
//*************************************************************************************************
#define CMD_WR_CONFIG       0xC000          //запись конфигурации
#define CMD_RD_CONFIG       0x4000          //чтение конфигурации
#define CMD_WR_DATA         0x8000          //запись данных
#define CMD_RD_DATA         0x0000          //чтение данных

//*************************************************************************************************
// Чтение конфигурации MAX3100
//*************************************************************************************************
#define TEST_MODE           0x0001
#define NORMAL_MODE         0x0000

//*************************************************************************************************
// Параметры конфигурации MAX3100
//*************************************************************************************************
#define FIFO_ENABLE         ( 1 << 13 )     //Enables the receive FIFO when FEN = 0. When FEN = 1, FIFO is disabled
#define SHUTDOWN            ( 1 << 12 )     //Software-Shutdown Bit
#define IRQ_TM              ( 1 << 11 )     //Transmit buffer is empty
#define IRQ_RM              ( 1 << 10 )     //Data available
#define IRQ_PM              ( 1 << 9 )      //Received parity bit = 1
#define IRQ_RAM             ( 1 << 8 )      //Transition on RX when in shutdown
#define IRDA_ENABLE         ( 1 << 7 )      //Enables the IrDA timing mode when IR = 1
#define STOP_BIT            ( 1 << 6 )      //One stop bit will be transmitted when ST = 0. Two stop bits will be transmitted when ST = 1
#define PARITY              ( 1 << 5 )      //Parity-Enable Bit
#define DATA9_BIT           ( 1 << 4 )      //L = 0 results in 8-bit words (9-bit words if PE = 1)

//*************************************************************************************************
// Маски обработки прерываний MAX3100
//*************************************************************************************************
#define IRQ_MASK_T          0x4000          //Transmit-Buffer-Empty Flag. T = 1 means that the transmit buffer is empty 
                                            //and ready to accept another data word.
#define IRQ_MASK_R          0x8000          //Receive Bit or FIFO Not Empty Flag. R = 1 means new data is available 
                                            //to be read from the receive register or FIFO

//*************************************************************************************************
// Параметры передачи данных MAX3100
//*************************************************************************************************
#define DATA_TE             ( 1 << 10 )     //(инверсный) выключить режим передачи данных
#define DATA_RTS            ( 1 << 9 )      //(инверсный) управление RTS, сигнал RTS=1 - передача, RTS=0 - прием
#define DATA_PE             ( 1 << 8 )      //передача бита четности

//*************************************************************************************************
// Маски состояния MAX3100
//*************************************************************************************************
#define STAT_RAFE           ( 1 << 10 )     //Receiver-Activity/Framing-Error Bit
#define STAT_CTS            ( 1 << 9 )      //state of the CTS pin
#define STAT_PR             ( 1 << 8 )      //Receive-Parity Bit

//*************************************************************************************************
// Скорость обмена для разных кварцев MAX3100
//*************************************************************************************************
#ifdef FOSC_36864   //fOSC = 3.6864 MHz (по умолчанию)
    #define BAUD_230400     0
    #define BAUD_115200     1
    #define BAUD_57600      2
    #define BAUD_28800      3
    #define BAUD_14400      4
    #define BAUD_7200       5
    #define BAUD_3600       6
    #define BAUD_1800       7
    #define BAUD_76800      8
    #define BAUD_38400      9
    #define BAUD_19200      10
    #define BAUD_9600       11
    #define BAUD_4800       12
    #define BAUD_2400       13
    #define BAUD_1200       13
    #define BAUD_600        15
#else               //fOSC = 1.8432 MHz
    #define BAUD_115200     0
    #define BAUD_57600      1
    #define BAUD_28800      2
    #define BAUD_14400      3
    #define BAUD_7200       4
    #define BAUD_3600       5
    #define BAUD_1800       6
    #define BAUD_76800      7
    #define BAUD_38400      8
    #define BAUD_19200      9
    #define BAUD_9600       10
    #define BAUD_4800       11
    #define BAUD_2400       12
    #define BAUD_1200       13
    #define BAUD_600        14
    #define BAUD_300        15
#endif

#endif
