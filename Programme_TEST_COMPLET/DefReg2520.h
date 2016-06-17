// définitions persons des noms de broches et de registres
#define RA0 PORTAbits.RA0
#define RA1 PORTAbits.RA1
#define RA2 PORTAbits.RA2
#define RA3 PORTAbits.RA3
#define RA4 PORTAbits.RA4
#define RA5 PORTAbits.RA5
#define RA6 PORTAbits.RA6
#define RA7 PORTAbits.RA7
#define RB0 PORTBbits.RB0
#define RB1 PORTBbits.RB1
#define RB2 PORTBbits.RB2
#define RB3 PORTBbits.RB3
#define RB4 PORTBbits.RB4
#define RB5 PORTBbits.RB5
#define RB6 PORTBbits.RB6
#define RB7 PORTBbits.RB7
#define RC0 PORTCbits.RC0
#define RC1 PORTCbits.RC1
#define RC2 PORTCbits.RC2
#define RC3 PORTCbits.RC3
#define RC4 PORTCbits.RC4
#define RC5 PORTCbits.RC5
#define RC6 PORTCbits.RC6
#define RC7 PORTCbits.RC7
#define RD0 PORTDbits.RD0
#define RD1 PORTDbits.RD1
#define RD2 PORTDbits.RD2
#define RD3 PORTDbits.RD3
#define RD4 PORTDbits.RD4
#define RD5 PORTDbits.RD5
#define RD6 PORTDbits.RD6
#define RD7 PORTDbits.RD7

// zone CAN
#define DONE ADCON0bits.DONE
#define GO   ADCON0bits.GO
#define CHS  ADCON0bits.CHS
#define ADON ADCON0bits.ADON
#define ADCS ADCON2bits.ADCS
#define ACQT ADCON2bits.ACQT
#define ADFM ADCON2bits.ADFM
#define PCFG ADCON1bits.PCFG
#define VCFG ADCON1bits.VCFG
#define ADIF PIR1bits.ADIF
#define ADIE PIE1bits.ADIE

// zone Timer et interruptions
#define T0IF 	INTCONbits.INT0IF
#define T0IE 	INTCONbits.T0IE
#define GIE  	INTCONbits.GIE
#define PEIE	INTCONbits.PEIE
#define RBIE	INTCONbits.RBIE
#define RBIF	INTCONbits.RBIF
#define RBPU	INTCON2bits.RBPU
#define RBIP	INTCON2bits.RBIP
#define INTEDG0	INTCON2bits.INTEDG0
#define INT0IE	INTCONbits.INT0IE
#define INT0IF	INTCONbits.INT0IF
#define RB0front	INTCON2bits.INTEDG0
#define TMR0IP	INTCON2bits.TMR0IP
#define TMR0IF	INTCONbits.TMR0IF
#define TMR0IE	INTCONbits.TMR0IE
#define TMR2IF 	PIR1bits.TMR2IF
#define TMR2IE 	PIE1bits.TMR2IE
#define RCIF	PIR1bits.RCIF
#define T0CS 	T0CONbits.T0CS
#define T0PS 	T0CONbits.T0PS
#define PSA  	T0CONbits.PSA
#define T08BIT  	T0CONbits.T08BIT
#define TMR0ON  	T0CONbits.TMR0ON
#define CCP1M 	CCP1CONbits.CCP1M
#define CCP2M 	CCP2CONbits.CCP2M
#define T2ON 	T2CONbits.T2ON
#define T2CKPS  	T2CONbits.T2CKPS
#define T2OUTPS 	T2CONbits.T2OUTPS
#define DC1B	CCP1CONbits.DC1B
#define DC2B	CCP2CONbits.DC2B
#define TMR1ON	T1CONbits.TMR1ON
#define TMR1IF	PIR1bits.TMR1IF
#define TMR1IE	PIE1bits.TMR1IE
#define TMR1IP	IPR1bits.TMR1IP
#define TMR2IP	IPR1bits.TMR2IP
#define T1SYNC	T1CONbits.T1SYNC
#define T1CKPS	T1CONbits.T1CKPS
#define RD16	T1CONbits.RD16
#define TMR1CS	T1CONbits.TMR1CS

// partie I2C
#define SEN		SSPCON2bits.SEN
#define RSEN		SSPCON2bits.RSEN
#define PEN		SSPCON2bits.PEN
#define RCEN		SSPCON2bits.RCEN
#define ACKDT		SSPCON2bits.ACKDT
#define ACKEN		SSPCON2bits.ACKEN
#define ACKSTAT	SSPCON2bits.ACKSTAT
#define R_W		SSPSTATbits.R_W
#define SMP		SSPSTATbits.SMP
#define BF			SSPSTATbits.BF
#define WCOL		SSPCON1bits.WCOL
#define SSPOV		SSPCON1bits.SSPOV
#define SSPEN		SSPCON1bits.SSPEN
#define CKP		SSPCON1bits.CKP
#define SSPM		SSPCON1bits.SSPM
#define SSPIF		PIR1bits.SSPIF				