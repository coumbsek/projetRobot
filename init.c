#include<p18f2520.h>
#include"init.h"
#include "variablesExt.h"

void init(void){
    /* Initialisation Horloge
     */
    OSCCONbits.IRCF2 = 1;   //Horloge de 8MHz
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;

    /* Initialisation Ports et Pins
     */
    TRISCbits.RC4 = 1;        //Broche SDA en entrée
    TRISCbits.RC3 = 1;        //Broche SCL en entrée

    TRISCbits.RC1 = 1;        //Initialisation PWM droit
    TRISCbits.RC2 = 1;        //Initialisation PWM gauche

    TRISAbits.RA6 = 0;        //Port RA6 en sortie
    TRISAbits.RA7 = 0;        //Port RA7 en sortie
    TRISBbits.RB1 = 0;        //Port RB1 en sortie

    TRISCbits.RC6 = 1;        //Initialisation TX
    TRISCbits.RC7 = 0;        //Initialisation RX

    /* Initialisation des Périphériques
     */
    // Initialisation I2C
    /*
    SSPCON1bits.SSPEN = 1;      //Validation I2C
    SSPCON1bits.SSPM3 = 1;      //mode maître
    SSPCON1bits.SSPM2 = 0;
    SSPCON1bits.SSPM1 = 0;
    SSPCON1bits.SSPM0 = 0;
    SSPSTATbits.SMP = 1;        //Débranche surveillance mode rapide
    SSPADD = 19;*/
    MI2CInit();

    // Initialisation RS232
    SPBRG = 25;                 //Initilisation Baudrate
    TXSTAbits.BRGH = 0;
    TXSTAbits.SYNC = 1;
    BAUDCONbits.BRG16 = 0;
    RCSTAbits.SPEN = 1;         //Validation TX
    TXSTAbits.TXEN = 1;         //Vaidation transmission

    // Initialisation ADC

        //Initialisation en mod analogique et selection Vss et vdd
    ADCON1 = 0;
    ADCON1bits.PCFG2 = 1;
    ADCON1bits.PCFG3 = 1;

        //Choix Tad et Taco et calage a gauche
    ADCON2 = 0;
    ADCON2bits.ACQT1 = 1;
    ADCON2bits.ACQT0 = 1;
    ADCON2bits.ADCS2 = 1;

        //AN2 en entrée analogique
    ADCON0bits.CHS1 = 1; 

    ADCON0bits.ADON = 1;

    // Initialisation PWM
    TRISCbits.RC2 = 0; // ccp1 sortie
    TRISCbits.RC1 = 0; // ccp2 sortie
    TRISAbits.RA6 = 0; // configurer en sortie
    TRISBbits.RB0=1;   // configurer en entree  relie a l interruption INT0 (liaison niveau materiel)
    TRISBbits.RB5=0;   // configurer en sortie led
    TRISBbits.RB1=1;  // int 1 bouton

    T2CONbits.T2CKPS1 = 0;
    T2CONbits.T2CKPS0 = 1; // on met le prescaleur a 4 on a PR2 = 249


    PR2 = 249;  //  choix PWM

    // CCPRxL:DC1B1:DC1B0 = 200 donc CCPRxL = 200 / 4
    CCPR1L = 0;//50 ;                        //50
    CCP1CONbits.DC1B1 = 0;               //0
    CCP1CONbits.DC1B0 = 0;               //0

    CCPR2L = 0;//50 ;                        //50
    CCP2CONbits.DC2B1 = 0;               //0
    CCP2CONbits.DC2B0 = 0;

    T2CONbits.TMR2ON = 1; // active timer

    CCP1CONbits.CCP1M3 = 1; // activation du mode PWM
    CCP1CONbits.CCP1M2 = 1;
    CCP2CONbits.CCP2M3 = 1; // activation du mode PWM
    CCP2CONbits.CCP2M2 = 1;

    T2CONbits.T2OUTPS3 = 1; // gestion postscaleur pour base de temps
    T2CONbits.T2OUTPS2 = 0;  // on divise par 10 pour passer de 1ms a 10ms
    T2CONbits.T2OUTPS1 = 1;  // de periode
    T2CONbits.T2OUTPS0 = 0;

    // Initialisation Interrupt
    INTCON2bits.INTEDG0 = 0;    //Interruption sur front descendant
    INTCONbits.INT0IE = 1;      //interruption 0
    INTCONbits.INT0IF = 0;
    PIE1bits.TMR2IE = 1; //Active l'interruption TIMER2
    INTCONbits.PEIE = 1;        //Validation interruptions
    INTCONbits.GIE = 1;         //Validation interruptions
    c10msTel = 0;
    return ;
}

