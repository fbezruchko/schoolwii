#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "Serial.h"
#include "MwiiSSerial.h"
#include "MultiWii.h"

#if (GPS)
//sserial
  MwiiSSerial ss(3, 4); // RX, TX
#endif

static volatile uint8_t serialHeadRX[UART_NUMBER],serialTailRX[UART_NUMBER];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER],serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];

volatile uint8_t tx_buf_pos = serialTailTX[0];

// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
// *******************************************************

ISR(USART_UDRE_vect) {  // Serial 0 on a PROMINI
  if (serialHeadTX[0] != tx_buf_pos) {
    if (++tx_buf_pos >= TX_BUFFER_SIZE) tx_buf_pos = 0;
    UDR0 = serialBufferTX[tx_buf_pos][0];  // Transmit next byte in the ring
    serialTailTX[0] = tx_buf_pos;
  }
  if (tx_buf_pos == serialHeadTX[0]) UCSR0B &= ~(1<<UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
}

void UartSendData(uint8_t port) {
  UCSR0B |= (1<<UDRIE0);
}

#if (GPS)
//sserial
void SSerialOpen(uint32_t baud) {
  ss.begin(baud);
}
#endif

void SerialOpen(uint8_t port, uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
  switch (port) {
    case 0: UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); break;
  }
}

// we don't care about ring buffer overflow (head->tail) to avoid a test condition : data is lost anyway if it happens
//void store_uart_in_buf(uint8_t data, uint8_t portnum) {
inline void store_uart_in_buf(uint8_t data, uint8_t portnum) {
  uint8_t h = serialHeadRX[portnum];
  serialBufferRX[h++][portnum] = data;
  if (h >= RX_BUFFER_SIZE) h = 0;
  serialHeadRX[portnum] = h;
}

ISR(USART_RX_vect)  { store_uart_in_buf(UDR0, 0); }

uint8_t SerialRead(uint8_t port) {
  uint8_t t = serialTailRX[port];
  uint8_t c = serialBufferRX[t][port];
  if (serialHeadRX[port] != t) {
    if (++t >= RX_BUFFER_SIZE) t = 0;
    serialTailRX[port] = t;
  }
  return c;
}

uint8_t SerialAvailable(uint8_t port) {
  return ((uint8_t)(serialHeadRX[port] - serialTailRX[port]))%RX_BUFFER_SIZE;
}

uint8_t SerialUsedTXBuff(uint8_t port) {
  return ((uint8_t)(serialHeadTX[port] - serialTailTX[port]))%TX_BUFFER_SIZE;
}

void SerialSerialize(uint8_t port,uint8_t a) {
  uint8_t t = serialHeadTX[port];
  if (++t >= TX_BUFFER_SIZE) t = 0;
  serialBufferTX[t][port] = a;
  serialHeadTX[port] = t;
}

void SerialWrite(uint8_t port,uint8_t c){
  SerialSerialize(port,c);UartSendData(port);
}
