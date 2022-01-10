
#ifndef __BITSTRING_H
#define __BITSTRING_H

typedef	unsigned char bitstr_t;

//Расчет кол-ва байтов для хранения N битов
#define	BitStrSize( nbits )     ( (((nbits) - 1) >> 3) + 1 )

//Определить номер байта в массиве по номеру бита
#define	BitByte( bit )          ( (bit) >> 3 )

//Размер переменной NAME в байтах для размещения nbits
#define	BitDecl( name, nbits )  (name)[BitStrSize( nbits )]

//Возвращает состояние бита в переменной
#define	BitTest( name, bit )    ( (name)[BitByte(bit)] & BitMask( bit ) )

//Маска для бита в байте
#define	BitMask( bit )          ( 1 << ( (bit)&0x7 ) )

//Установка бита в переменной
#define	BitSet( name, bit )     (name)[BitByte( bit )] |= BitMask( bit )

//Сброс бита в переменной
#define	BitClear( name, bit )   (name)[BitByte( bit )] &= ~BitMask( bit )

//Сбрасываем диапазон битов start ... stop
#define	NBitClear( name, start, stop ) { \
    register bitstr_t *_name = name; \
    register int _start = start, _stop = stop; \
    register int _startbyte = BitByte(_start); \
    register int _stopbyte = BitByte(_stop); \
    if (_startbyte == _stopbyte) { \
        _name[_startbyte] &= ((0xff >> (8 - (_start&0x7))) | (0xff << ((_stop&0x7) + 1))); \
    } else { \
        _name[_startbyte] &= 0xff >> (8 - (_start&0x7)); \
        while (++_startbyte < _stopbyte) \
            _name[_startbyte] = 0; \
        _name[_stopbyte] &= 0xff << ((_stop&0x7) + 1); \
    } \
}

//Устанавливаем диапазон битов start ... stop
#define NBitSet(name, start, stop) { \
    register bitstr_t *_name = name; \
    register int _start = start, _stop = stop; \
    register int _startbyte = BitByte(_start); \
    register int _stopbyte = BitByte(_stop); \
    if (_startbyte == _stopbyte) { \
        _name[_startbyte] |= ((0xff << (_start&0x7)) & (0xff >> (7 - (_stop&0x7)))); \
    } else { \
        _name[_startbyte] |= 0xff << ((_start)&0x7); \
        while (++_startbyte < _stopbyte) \
                _name[_startbyte] = 0xff; \
        _name[_stopbyte] |= 0xff >> (7 - (_stop&0x7)); \
    } \
}

#endif
