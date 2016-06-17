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

void tempSec(int n){
    unsigned int i = 0;
    while (i++ < n*20000);
}

void tempMiliS(int n){
    unsigned int i = 0;
    while (i++ < n*20);
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

void initI2c(){
    TRISCbits.RC3 = 1; //SCL en entrée
    TRISCbits.RC4 = 1; //SDA en entrée
    SSPCON1bits.SSPEN = 1;
    SSPCON1bits.SSPM3 = 1; //mod maitre
    SSPCON1bits.SSPM2 = 0; //mod maitre
    SSPCON1bits.SSPM1 = 0; //mod maitre
    SSPCON1bits.SSPM0 = 0; //mod maitre
    SSPSTATbits.SMP = 1;
    SSPADD = 9;
}

void initBaudRate(){
    SPBRG = 103; // n de la formuble baud raute =  Fosc/[4*(n+1)]
    BAUDCONbits.BRG16 = 1;//mod 16 bits
    TXSTAbits.SYNC = 0; //mod asynchrone
    TXSTAbits.BRGH = 1; //
}

void initSerial(){
    initBaudRate();
    TRISCbits.RC6 = 1; //broche TX en entrée
    RCSTAbits.SPEN = 1; //serial port enable
    TXSTAbits.TXEN = 1; //validation transmission
    PIR1bits.TXIF = 0; // effacement inté
    PIE1bits.TXIE = 1; //validation inté
    TXREG = tab[0];
}

void waiti2c(){
    while(!PIR1bits.SSPIF);
    PIR1bits.SSPIF = 0;
}
void waitAknowledge(){
    while(SSPCON2bits.ACKSTAT);
}
void transmiti2c(char address, char data){
    SSPCON2bits.SEN = 1;
    waiti2c();
    SSPBUF = address;
    waiti2c();
    waitAknowledge();
    SSPBUF = 0x51;
    waiti2c();
    waitAknowledge();
    SSPCON2bits.PEN = 1;
    waiti2c();
}

void receivei2c(char address, char *data){
    transmiti2c(address, 0x51);
    tempMiliS(70);
    SSPCON2bits.SEN = 1;
    waiti2c();
    SSPBUF = address;
    waiti2c();
    waitAknowledge();
    SSPCON2bits.RCEN = 1;
    waiti2c();
    *data = SSPBUF;
    SSPCON2bits.ACKDT = 1;
    waiti2c();
    SSPCON2bits.PEN = 1;
    waiti2c();
}

void main(void) {
    char data;
    data= 255;
    initClock();
    initPin();
    initI2c();
  
    while(1){
       // transmiti2c(0x40, data--);
      //  tempSec(1);
        receivei2c(0xE0, &data);
        tempSec(1);
    }
}
