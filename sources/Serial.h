#ifndef SERIAL_H_
#define SERIAL_H_

#include "MwiiSSerial.h"

#define UART_NUMBER 1

//#define RX_BUFFER_SIZE 256 // 256 RX buffer is needed for GPS communication (64 or 128 was too short)
#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 128

void    SerialOpen(uint8_t port, uint32_t baud);
uint8_t SerialRead(uint8_t port);
void    SerialWrite(uint8_t port,uint8_t c);
uint8_t SerialAvailable(uint8_t port);
uint8_t SerialUsedTXBuff(uint8_t port);
void    SerialSerialize(uint8_t port,uint8_t a);
void    UartSendData(uint8_t port);

//sserial
#if (GPS)
extern MwiiSSerial ss; // RX, TX
void SSerialOpen(uint32_t baud);
#endif

void SerialWrite16(uint8_t port, int16_t val);

#endif /* SERIAL_H_ */
