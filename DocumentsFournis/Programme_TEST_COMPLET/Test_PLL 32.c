/********************************************************************************
  *		TEST OSCILLATEUR Interne 8MHz * 4PLL = 32MHz							*
  *******************************************************************************
  * Description du test								                            *
  * On fait clignoter la diode de test a 1s                                     *
  *  -  Le timer TMR0 compte la seconde 										*
  *     Horloge Fosc/4 Tosc = 125µs             								*
  *     TMR0 en mode 16bit														*
  *		PRESCALE = 128															*
  *		Au debordement TMRO = 65536 on a 1S													*
  * En sortie :																*
  *  -  Led de test sur RC7														*
  *******************************************************************************/
#include <p18f2520.h>		// définitions des structures du processeur à la sauce JPB



#define Une_Seconde PORTCbits.RC7

// Prototype fonction

void init(void);


/****************************** PROGRAMME PRINCIPAL **************************/
void main(void)
{
    init();             // Séquence initialisation au démarrage des périphérique
    while(1)
    {
	if (INTCONbits.TMR0IF == 1)				// On fait clignoter la led tout les secondes
		{
		INTCONbits.TMR0IF = 0;
		Une_Seconde = ~Une_Seconde;
		}
	Nop();
    }	// fin du while(1)
}		// fin du main()
//----------------------------------------------------------------------------

void init(void)
// Initialise le timer T0 pour le cadencement du test
{
	OSCCON = 0b01110000;     // select 8MHz internal osc. 
	OSCTUNEbits.PLLEN = 1;       // enable 4xPLL ;Fosc = 32MHz

	//  Paramétrage des ports, en lots      
	TRISA = 0b11111111; 	// Entree
 	TRISB = 0b11111111;		// Entree
	TRISC = 0b01111111;		// RC7
	PORTCbits.RC7 = 0;

   	// Timer0 pour débordement toutes les secondes environ
  	TMR0H = 0;          	// raz timer
  	TMR0L = 0;          	// raz timer
	T0CONbits.T08BIT = 0;				// mode 16 bits actif
  	T0CONbits.T0CS = 0;       		// horloge timer 0= 32MHz/4 T(T0) 8MHz = 0,125µs 
  	T0CONbits.PSA = 0;        		// pré-division interne active
	T0CONbits.T0PS0 = 0;				// prédivision par 128
	T0CONbits.T0PS1 = 1;  
	T0CONbits.T0PS2 = 1;
	T0CONbits.TMR0ON = 1;				// Lancement du Timer0
}






