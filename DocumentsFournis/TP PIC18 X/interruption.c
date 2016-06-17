#include<p18f2520.h>
#pragma config OSC = INTIO67
#pragma config PBADEN = OFF, WDT = OFF, LVP = OFF, DEBUG = ON

#pragma interrupt HighISR

typedef enum boolean {FALSE, TRUE}boolean;
boolean b = FALSE;
int c = 0;

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
   if(INTCONbits.INT0IF)
   {
      INTCONbits.INT0IF = 0;   //acquitte l'int0
     // PORTBbits.RB5 = 0; //change état bit5 port B
      b = FALSE;
      c++;
   }
   if(INTCON3bits.INT1F){
      INTCON3bits.INT1IF = 0;   //acquitte l'int1
      b = TRUE;
   }
}

void temp(int n){
    unsigned int i = 0, j = 0;
    while (i++ < n*20000);
}

void cli(){
    PORTBbits.RB5 = !PORTBbits.RB5;
    temp(1);
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

void main(void) {
  initClock();

  initPin();
  INTCON2bits.INTEDG0 = 0; //Int sur front descendant i.e. a l'appuie sur le btn ici
  INTCON2bits.INTEDG1 = 1; //Int sur front montant
  INTCONbits.INT0IE = 1; //active interuption 0
  INTCON3bits.INT1IE = 1; //active int 3
  INTCON3bits.INT1IP = 0;
  INTCONbits.PEIE = 1; 
  INTCONbits.GIE = 1; 
  while(1){
      if (b)
          cli();
  }
}
   
