#include<p18f2520.h>
#pragma config OSC = INTIO67
#pragma config PBADEN = OFF, WDT = OFF, LVP = OFF, DEBUG = ON

void initClock(){
    OSCCON = OSCCON | 0b01100000;
    OSCCON = OSCCON & 0b11101111;
}

void initPin(){
    TRISAbits.RA6=0;
    TRISBbits.RB0=1;
    TRISBbits.RB5=0;
}

void temp(int n){
    unsigned int i = 0, j = 0;
    while (i++ < n*10000);
}

void main(void) {
    int n;
    initClock();
    initPin();
    while(1){
        n = 4;
        PORTBbits.RB5 = !PORTBbits.RB5;
        if (PORTBbits.RB0){
            n = 2;
        }
        temp(n);
    }
}
