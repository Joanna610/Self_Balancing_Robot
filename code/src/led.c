#include <avr/io.h>
#include <util/delay.h>
#include "led.h"

void InitLED(void){
    DDRB = DDRB | (1<< DDB5);
}

void TurnOnLED(void){
    PORTB = PORTB | (1<< PORTB5);
}

void TurnOffLED(void){
    PORTB = PORTB & ~(1<< PORTB5);
}