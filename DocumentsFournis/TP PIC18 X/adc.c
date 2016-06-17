
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
       
    }
}

void initClock(){
    OSCCON = OSCCON | 0b01100000;
    OSCCON = OSCCON & 0b11101111;
}

void initPin(){
    TRISBbits.RB0=1;
    TRISBbits.RB5=0;
}

void initADC(){
    /*
     * Initialisation en mod analogique et selection Vss et vdd
     */
    ADCON1 = 0;
    ADCON1bits.PCFG2 = 1;
    ADCON1bits.PCFG3 = 1;
    /**
     * Choix Tad et Taco et calage a gauche
     */
    ADCON2 = 0;
    ADCON2bits.ACQT1 = 1;
    ADCON2bits.ACQT0 = 1;
    ADCON2bits.ADCS2 = 1;

    ADCON0bits.CHS1 = 1; //AN2 en entrée analogique

    ADCON0bits.ADON = 1;
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
    unsigned char tens;
    initClock();
    initPin();
    initTimer0();
    initInterruptButtons();
    while(1){
        ADCON0bits.GO = 1;
        while(!ADCON0bits.DONE);
        tens = ADRESH;
        if (tens < 254*10/3.2/5){
             PORTBbits.RB5=1;
        }
        else{
            PORTBbits.RB5=0;
        }
    }
}