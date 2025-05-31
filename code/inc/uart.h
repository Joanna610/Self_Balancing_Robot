#ifndef UART_H
#define UART_H

void SerialInit(unsigned long int fosc, unsigned int baud, short int bits, short int stopBits, short int parity);
void SerialPrintChar(char data);
void SerialPrintInt(int data);
void SerialPrintInt8(uint8_t data);
void SerialPrintDouble(double data);
void SerialPrintInt16(uint16_t data);
void SerialPrint(char *s);
unsigned char SerialRead(void);

#endif 