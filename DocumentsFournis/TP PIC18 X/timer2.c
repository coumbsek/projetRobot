
#include<p18f2520.h>
#pragma config OSC = INTIO67
#pragma config PBADEN = OFF, WDT = OFF, LVP = OFF, DEBUG = ON

#pragma interrupt HighISR

typedef enum boolean {FALSE, TRUE}boolean;
boolean b = FALSE;

void HighISR(void);

#pragma code HighVector=0x08
void IntHighVector(void)
{
    _asm goto HighISR _endasm
}
#pragma code

#pragma interrupt HighISR
void HighISR(void)
{
    if(INTCONbits.TMR0IF){//si timer interrupt
        INTCONbits.TMR0IF = 0;
        PORTBbits.RB5 = !PORTBbits.RB5;
    }
}

void initClock(){
    OSCCON = OSCCON | 0b01100000;
    OSCCON = OSCCON & 0b11101111;
}

void initPin(){
    TRISAbits.RA6=0;
    TRISBbits.RB0=1;
    TRISBbits.RB5=0;
    TRISCbits.RC1=0;//PWMD
    TRISCbits.RC2=0;//PWMG
}

void initTimer0(){
    T0CONbits.TMR0ON = 1;
    T0CONbits.T08BIT = 0;//mod 16 bit
    T0CONbits.T0CS = 0;// mod clock
    T0CONbits.PSA = 0; //active prescale
    T0CONbits.T0PS2 = 1;
    T0CONbits.T0PS1 = 0;
    T0CONbits.T0PS0 = 0;
    INTCONbits.TMR0IE = 1;
}

void initTimer2(){
    T2CONbits.TMR2ON = 1;
    T2CONbits.T2CKPS0 = 1; 
    T2CONbits.T2CKPS1 = 0;
    T2CONbits.T2OUTPS1 = 0;
    PR2 = 249;
    CCPR1L = 50;
    CCP1CONbits.CCP1M3 = 1; //mode pwm sur le pin ccp1
    CCP1CONbits.CCP1M2 = 1;
    CCP1CONbits.DC1B = 0;

}

void initInterruptButtons(){
    INTCON2bits.INTEDG0 = 0; //Int0 sur front descendant i.e. a l'appuie sur le btn ici
    INTCON2bits.INTEDG1 = 1; //Int1 sur front montant
    INTCONbits.INT0IE = 1; //active interuption 0
    INTCON3bits.INT1IE = 1; //active int 3
    INTCON3bits.INT1IP = 0;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
}

void main(void) {
  initClock();
  initPin();
  initTimer2();
  initInterruptButtons();
  while(1);
}


