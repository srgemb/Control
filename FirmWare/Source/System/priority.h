
#ifndef __PRIORITY_H
#define __PRIORITY_H

//CMSIS NXP
#define PRIORITY_TIMER          1

#define SUB_PRIORITY_TIMER0     0
#define SUB_PRIORITY_TIMER1     1
#define SUB_PRIORITY_TIMER2     2       //прерывание через 125 usec, F = 4 KHz, T = 250 usec

//CMSIS NXP
#define PRIORITY_GPIO           2       //прерывание от HMI и MAX3100 (RS485)

//CMSIS Keil
#define PRIORITY_SSP            3

#define SUB_PRIORITY_SSP2       0       //обмен с HMI           DMA 5/6
#define SUB_PRIORITY_SSP1       1       //MAX3100 (RS485)
#define SUB_PRIORITY_SSP0       2       //ADC ADS1286
#define SUB_PRIORITY_CAN        3       //CAN2

//CMSIS Keil
#define PRIORITY_USART          4

#define SUB_PRIORITY_USART0     0       //консоль               DMA 1/2
#define SUB_PRIORITY_USART1     1       //BMV-600S
#define SUB_PRIORITY_USART2     2       //TS-1000
#define SUB_PRIORITY_USART3     3       //TS-3000
#define SUB_PRIORITY_USART4     4       //MPPT

//CMSIS NXP
#define PRIORITY_RTC            6

#endif

