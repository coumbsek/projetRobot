#include<p18f2520.h>
#include<stdio.h>
#pragma config OSC = INTIO67
#pragma config PBADEN = OFF, WDT = OFF, LVP = OFF, DEBUG = ON

#pragma interrupt HighISR

void HighISR(void);

#pragma code HighVector=0x08
void IntHighVector(void)
{
    _asm goto HighISR _endasm
}
#pragma code

char tab[] = "saisir une vitesse : ";
char i = 1;
char okTrans = 1;
char getchar(char *c){
    return *c;
}

void putchar(char c){
       if (c == '\0'){
           PIE1bits.TXIE = 0;
       }
       else{
           while(!PIR1bits.TXIF);
           TXREG = 'a';
      }
}

#pragma interrupt HighISR
void HighISR(void)
{/*
    if(PIR1bits.TXIF){
        PIR1bits.TXIF = 0;
        okTrans = 1;
        TXREG = c;
        i++;
    }*/
}

void temp(int n){
    unsigned int i = 0, j = 0;
    while (i++ < n*20000);
}

void initClock(){
    OSCCON = OSCCON | 0b01100000;
    OSCCON = OSCCON & 0b11101111;
}

void initPin(){
    TRISAbits.RA6=0;
    TRISBbits.RB0=1;
    TRISBbits.RB5=0;
}

void initBaudRate(){
    SPBRG = 103; // n de la formuble baud raute =  Fosc/[4*(n+1)]
    BAUDCONbits.BRG16 = 1;//mod 16 bits
    TXSTAbits.SYNC = 0; //mod asynchrone
    TXSTAbits.BRGH = 1; //
}

void initSerial(){
    initBaudRate();
    //TRISCbits.RC6 = 1; //broche TX en entrée
    RCSTAbits.SPEN = 1; //serial port enable
    TXSTAbits.TXEN = 1; //validation transmission
    //PIR1bits.TXIF = 0; // effacement inté
    //PIE1bits.TXIE = 1; //validation inté
    TXREG = tab[0];
}

void main(void) {
  initClock();
  initPin();
  initSerial();

  /*
   * Validation générale des inté
   */
  //INTCONbits.PEIE = 1;
  //INTCONbits.GIE = 1;
  while(1){
      putchar('t');
  }
}