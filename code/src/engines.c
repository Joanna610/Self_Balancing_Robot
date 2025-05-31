#include "engines.h"
#include <stdint.h>
#include <avr/io.h>

// PD6 PB0 PD7
// PB3 PD5 PD4

void initEngines() {
    // Timer1 - Fast PWM, 8-bit, non-inverting, prescaler 64 (dla D9/OC1A)
    TCCR0A |= (1 << COM0A1) | (1 << WGM00) | (1 << WGM01); // Fast PWM na OC0A
    TCCR0B |= (1 << CS01) | (1 << CS00); // prescaler 64

    // Timer2 - Fast PWM, non-inverting, prescaler 64 (dla D3/OC2B)
    TCCR2A |= (1 << COM2B1) | (1 << WGM20) | (1 << WGM21);
    TCCR2B |= (1 << CS22); // prescaler 64

    // Ustaw pozostałe piny jako wyjścia
    DDRB |= (1 << PB0);      // D8 (in1)
    DDRD |= (1 << PD7);      // D7 (in2)
    DDRD |= (1 << PD5);      // D5 (in3)
    DDRD |= (1 << PD4);      // D4 (in4)
    // Set PWM pins as output
    DDRD |= (1 << PD6); // D9 / OC1A
    DDRD |= (1 << PD3); // D3 / OC2B

    // Wyłącz silniki na start
    PORTB &= ~(1 << PB0);    // in1 LOW
    PORTD &= ~(1 << PD7);    // in2 LOW
    PORTD &= ~(1 << PD5);    // in3 LOW
    PORTD &= ~(1 << PD4);    // in4 LOW
}

void forward(int speed) {
    OCR0A = speed; // PWM na D9 (enA)
    OCR2B = speed; // PWM na D3 (enB)

    // Motor A forward
    PORTB |= (1 << PB0);  // in1 HIGH
    PORTD &= ~(1 << PD7); // in2 LOW

    // Motor B forward
    PORTD |= (1 << PD5);  // in3 HIGH
    PORTD &= ~(1 << PD4); // in4 LOW
}

void backward(int speed) {
    OCR0A = speed; // PWM na D9 (enA)
    OCR2B = speed; // PWM na D3 (enB)

    // Motor A forward
    PORTB &= ~(1 << PB0);  // in1 HIGH
    PORTD |= (1 << PD7); // in2 LOW

    // Motor B forward
    PORTD &= ~(1 << PD5);  // in3 HIGH
    PORTD |= (1 << PD4); // in4 LOW
}

void stop() {
    PORTB &= ~(1 << PB0);    // in1 LOW
    PORTD &= ~(1 << PD7);    // in2 LOW
    PORTD &= ~(1 << PD5);    // in3 LOW
    PORTD &= ~(1 << PD4);    // in4 LOW
}
