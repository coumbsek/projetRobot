
#include<p18f2520.h>
#pragma config OSC = INTIO7
#pragma config PBADEN = OFF, WDT = OFF, LVP = OFF, DEBUG = ON
unsigned char led_test; 

void main(void) {
        OSCCON = OSCCON | 0b01100000;
        OSCCON = OSCCON & 0b11101111;
        TRISAbits.RA6=0;
	TRISBbits.RB0=1;
	TRISBbits.RB5=0;
	while(1)
            PORTBbits.RB5 = (PORTBbits.RB0) ? 1 : 0;	
}
	