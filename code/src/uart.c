#include <avr/io.h>
#include "uart.h"
#include <stdio.h>

// inspo https://www.robotyka.net.pl/mikrokontrolery-avr-czesc-10-transmisja-szeregowa-uart/

void SerialInit(unsigned long int fosc, unsigned int baud, short int bits, short int stopBits, short int parity)
{
    unsigned int ubrr = fosc / 16 / baud - 1;

    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;

    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    // Format ramki
    UCSR0C = 0; // wyczyść ustawienia przed nadpisaniem

    switch (parity)
    {
        case 1: // parzystość
            UCSR0C |= (1 << UPM01);
            break;
        case 2: // nieparzystość
            UCSR0C |= (1 << UPM01) | (1 << UPM00);
            break;
        default: // brak parzystości (domyślnie nic nie ustawiamy)
            break;
    }

    if (stopBits == 2)
        UCSR0C |= (1 << USBS0); // 2 bity stopu

    switch (bits)
    {
        case 6:
            UCSR0C |= (1 << UCSZ00);
            break;
        case 7:
            UCSR0C |= (1 << UCSZ01);
            break;
        case 8:
        default:
            UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
            break;
    }
}

void SerialPrintChar(char data)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void SerialPrintInt8(uint8_t data)
{
    char text[4];  // 3 cyfry + null-terminator
    sprintf(text, "%u", data);
    SerialPrint(text);
}

void SerialPrintDouble(double data)
{
    char text[16];  // wystarczy na np. "-123.4567\0"
    sprintf(text, "%f", data);  // zaokrągl do 4 miejsc po przecinku
    SerialPrint(text);
}

void SerialPrintInt16(uint16_t data)
{
    char text[4];  // 3 cyfry + null-terminator
    sprintf(text, "%u", data);
    SerialPrint(text);
}

void SerialPrintInt(int data)
{
    char text[4];  // 3 cyfry + null-terminator
    sprintf(text, "%u", data);
    SerialPrint(text);
}


void SerialPrint( char *s)
{
    while (*s)
    {
        SerialPrintChar(*s);
        s++;
    }
}
