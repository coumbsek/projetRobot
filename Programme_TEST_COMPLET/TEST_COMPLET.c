/********************************************************************************
  *		TEST COMPLET - Pour tester la carte ROBOT en MAP apr�s montage			*
  *******************************************************************************
  * Description sommaire des entr�es et sorties    :                            *
  * En entr�es :                                                                *
  *  -  Deux capteurs distance IR_DROITE, IR_GAUCHE dynamique du signal 		*
  *     analogique capteur IR 0, 3 volts, Cadencement � 1ms environ             *
  *           IR_DROITE sur AN0          IR_GAUCHE sur AN1						*  
  *  -  Image de la tension de la batterie pour surveillance V_BATT 0, 5volts  	*
  *     sur AN2 filtr�e on arr�tera le robot si VBATT moyenn�e atteint 10V      *
  *  -	Un capteur TTL � l'arri�re annonce un obstacle par NLH : RA3			*
  *  -	Deux signaux TTL en  RA4 PA (gauche) et RC0 PB (droite) indiquent par 	*
  * 	un front un d�placement moteur des chenilles de 0.9cm.					*
  *  - Une t�l�commande d�code et m�morise le code de la touche si valide.      *
  *      Un signal IT annonce une r�ception t�l�commande : RB0 actif niveau bas *        
  *      L'octet code touche accessible via bus I2C adresse 0xA2                *
  *  - Un sonar I2C d'une port�e de 1,5m d�tecte les objets avant et cot�s		*
  *  - Une liaison s�rie permet de recevoir des commandes 9600 bauds, 8 bits, 	*
  *		pas parit�, 1 stop														*
  * En sorties :																*
  *  -  Deux commandes moteurs DROITE et GAUCHE signaux TTL actif � 1			*
  *     PWM_GAUCHE 10, 50% sur CCP1           PWM_DROITE 10, 50% sur CCP2		*
  *     DIRECTION_GAUCHE sur RA7                  DIRECTION_DROITE sur RA6 		*
  * Relev�s du moteur employ� dans l'embase : 6V, 0.2A � vide, 0.6A en cas de	*
  *  	patinage, R=4.5Ohm en s�rie avec une inductance de 1.6mH avec 9 tours  	*
  *  	de chenilles (38,7cm) en 15s sous 6V � vide soit 14 m/mn                *
  *  -  Une LED contr�le d'activit� sur RB5 active � 1 indique :                *
  *  -  8 Leds actionn�es par I2C avec PCF8574 Adresse A2-A0 = 000 =0x40+R/W	*
  * 	rendent compte de l'�tat du robot										*
  *  - En option, Un Servo Moteur de 180� permet d'orienter le sonar avec signal*
  *		STEP en RC5 niveau haut de 1ms � 2ms sur une p�riode de 20ms			*
  * Cadencement des t�ches de Contr�le Commandes p�riodique par interruption 	*
  * 	du TIMER0 environ 1ms sert � d�tecter le signal IT de t�l�commande 		*
  *	Toutes les 10 x T(TMR0)=10ms, lecture d'un capteur analogique et du sonar	*
  *	et toutes les 100 x T(TMR0)=100ms remise en cause de la navigation     		*
  * Mise au point et programmation                                             	*
  *******************************************************************************     
  *	Ce programme teste toute la carte sauf l'EEPROM. Apr�s 3 flashs au reset :	*
  * - On affiche un message sur la liaison s�rie et on attend une action clavier*
  *	Selon la touche clavier actionn�e, on effectue le test correspondant :		*
  *	- Touche 0 : Chenillard des leds 											*
  *	- Touche 1 : Moteurs : vitesses opos�es triangulaires >0 puis <0			*
  *	- Touche 2 : Lecture des capteurs IR : Envoie RS232 des valeurs lues		*
  *	- Touche 3 : Lecture sonar : Envoie RS232 de la valeur lue et sur leds		*
  *	- Touche 4 : Commande servo Sonar : Tourne de droite � gauche				*
  *	- Touche 5 : Lecture t�l�commande affich�e sur leds et RS232				*
  *	- Touche 6 : Lecture des capteurs de rotation, affich�s sur leds et RS232	*
  *	- Touche 7 : Lecture de la tension batterie, envoie de la tension sur RS232	*
  *	- Touche A : L'adressage du PCF8574 passe � celui du PCF8574A				*
  *	- Touche N : L'adressage du PCF8574A passe � celui du PCF8574				*
  *	- Touche S : Stoppe le test en cours et affiche le menu                     *
  *	- Touche ? : Affiche le menu                                                *
  *******************************************************************************
  *	Pour cela on va faire fonctionner le PIC 18F2520 � 32MHz par la PLL			*
  *	On va utiliser le Timer 2 pour les 2 PWM des moteurs � environ 2kHz			*
  *	On va utiliser le Timer 0 comme base de temps interne IT toutes les 0,5us	*
  *	On va utiliser le TImer 1 comme rythme de motorisation sonar � 50Hz			*
  *	Une seule IT hard : le signal INT0 son front descendant IT t�l�commande		*
  *******************************************************************************/
#include "p18f2520_2.h"		// d�finitions des structures du processeur � la sauce JPB
					// c'est � dire nom direct des bits et champs num�riques regroup�s
#include "DefReg2520.h"		// d�finitions persos des noms de broches et de registres
#include "En Tete_P2016.h"			// description globale de l'application et d�finitions
#include <stdlib.h>			// pour gestion des chaines de caract�res
#include <i2c.h>			// biblioth�que microchip pour l'I2C

#define MinBat 	150			// la tension mesur�e = Vbatterie - 0.6V est att�nu�e dans un rapport 1/3,2
// le seuil de 10V correspond donc � 9.4/3.2 = 2.93V soit 255*2.93/5 = 150.
#define SIZE    20         	// taille des buffers de liaison s�rie
#define MinHaut 900		// correspond � la dur�e mini 900us pour -90� servomoteur
#define MaxHaut	2100		// correspond � la dur�e maxi 2100us pour +90� servomoteur
#define QuantServo 33		// variation �l�mentaire de dur�e PWM 100�s : 26 positions pour 180�
#define Periode_PWM_servo 20000	// 20ms


unsigned char tempo_LED;

unsigned int heure;					// nombre de d�ciseconde qui tourne en permanence : max = 6553s > 109mn

char NumConv=0, NumEch=0, FinAcq=0;	// �l�ments des CAN
unsigned int DurBas, DurHaut;		// entiers pour fixer Niveaux bas et haut en us de PWM servomoteur sonar
char I2Clibre, I2CStop;
unsigned char Code;			// boutons de la t�l�commande (1 bit parmi les 5 bits faibles) sinon refus			
struct 	{		
	unsigned char OldPA:1;   	// valeur ant�rieure du signal capteur rotation PA
	unsigned char OldPB:1;		// valeur ant�rieure du signal capteur rotation PB
	unsigned char Pas:1;		// indique qu'un pas de mouvement sonar est requis
	unsigned char Ecrit:1;		// indique que l'on doit envoyer un octet � l'affichage
	unsigned char ALire:1;		// Du nouveau sur la RS232 � prendre en compte
	unsigned char Tel:1;		// il faut prendre en compte le code t�l�commande
	unsigned char Sonar:1;		// il est tant de lire le sonar ... et relancer l'�mission
	unsigned char ds:1;			// une d�ciseconde s'est �coul�e .. y faut faire ...
  }CAPT;							// variable d'�tat pour ce test des Entr�es				
struct 	{		
	unsigned char SensM:1;   	// indique le sens des deux moteurs chenilles D et \G
	unsigned char Change:1;		// indique qu'il faut changer les sens des moteurs
	unsigned char TempoLeds:2;	// petite tempo leds chenillards change toutes les 0.4s
	unsigned char Sens:1;		// sens du chenillard, des vitesses moteurs, pas servo ....
	unsigned char Changement:1;	// il faut mettre � jour les leds ou les moteurs ....
  }ACT;							// variable d'�tat pour ce test des Sorties
//        Variables globales de la partie communication s�rie
unsigned char c;				// caract�re recu sur RS232
unsigned char buffer[SIZE];		// buffer de reception 
unsigned char *ptlec;			// pointeur de lecture
unsigned char *ptecr;			// pointeur d'�criture 
unsigned char udata;			// caractere recu depuis l'USART
unsigned char buf[10];			// Pour conversion ASCII et affichage resultat console

// Variables globales Echanges I2C_Telecom
unsigned char Send_I2C_Telecom_ASCII_Buf[8] = {0X31,0X00};
unsigned char Recv_I2C_Telecom_ASCII_Buf[8];

//	Diverses variables globales
unsigned char mode = 0;			// mode du test en cours : 0=Rien ou code ascii du mode re�u en RS232
unsigned char AffI2C = 0x20;	// adresse de base (7 bits) pour l'affichage par PCF8574 normal
unsigned char Aff;				// Octet affich� sur les leds (en logique positive)
signed char PasSonar;			// Num�ro du pas d'orientation sonar
unsigned char Gisement_D;
unsigned int temp;				// Pour calculs

// constantes chaines des messages pour la liaison s�rie
const rom char newline[] = "\r\n";
const rom char separe[] = " - ";
const rom char teta[] = " deg.";
const rom char initial[] =" *****  PIC18f2520 ROBOT ISMIN 1A  v. 2012 ***** 9600 N 8 1\r\n";
const rom char mode0[] = "0 --> Chenillard Leds\r\n";
const rom char mode1[] = "1 --> Vitesses Moteurs\r\n";
const rom char mode2[] = "2 --> Capteurs Dist. IR\r\n";
const rom char mode3[] = "3 --> Distance Sonar\r\n";
const rom char mode4[] = "4 --> Rotation Sonar\r\n";
const rom char mode5[] = "5 --> Lecture Telecommande\r\n";
const rom char mode6[] = "6 --> Capt. Chenilles\r\n";
const rom char mode7[] = "7 --> Tension Batterie\r\n";
const rom char modeN[] = "N --> PCF8574 normal\r\n";
const rom char modeA[] = "A --> PCF8574 A !!\r\n";
const rom char help[] = "? --> HELP\r\n";
const rom char stop[] = "S --> Stop\r\n";
const rom char haut[] = "Touche HAUTE\r\n";
const rom char bas[] = "Touche BASSE\r\n";
const rom char gauche[] = "Touche GAUCHE\r\n";
const rom char droite[] = "Touche DROITE\r\n";
const rom char centre[] = "Touche CENTRE\r\n";
const rom char chenilleD[] = "Rotation D OK\r\n";
const rom char chenilleG[] = "Rotation G OK\r\n";


/* Prototypes des fonctions employ�es */
void init (void);				// initialisation des E/S, timers et variables
void Mesure (void);
void Moyenne (unsigned char * tab, unsigned char val);
// transaction I2C de l'application (lecture de l'octet en entr�e
unsigned char Octet_i2c(unsigned char adresse, unsigned char sens, unsigned char valeur);
void ITHaute (void);				// prototype de la routine d'interruption
unsigned char LitOctet (void);		// retourne le premier caract�re re�u sur SCI
void EcritOctet (unsigned char c);	// �met c sur l'UART
unsigned char *LitChaineUart (unsigned char *s, unsigned char finst);          
                // lit une cha�ne de caract�re sur SCI se terminant par finst
                // la variable finst contient le caract�re attendu en  fin de cha�ne 
int EcritChaine (unsigned char *s);       	// �met la cha�ne s en RAM(finit par 0)
int EcritChaineRom (rom unsigned char *s);  // �met la cha�ne s en ROM(finit par 0)
void putrsUSART (const rom char *data); 	// Ecrit une chaine en ROM vers l'USART sans le caract�re NULL \0
void putsUSART (char *s);                 	// Ecrit une chaine en RAM vers l'USART sans le caract�re NULL \0
void EcritValOctet (unsigned char oct);		// Envoie en s�rie la chaine ascii d'un octet
// SONAR_Ecrit et SONAR_Lit sont utilis�es pour lire et �crire un octet dans 24CXX EEPROM ou SONAR srf02
unsigned int SONAR_Valid (char control, unsigned char address);																		
void SONAR_Ecrit (char control, unsigned char address, char data);
unsigned int SONAR_Lit (char control, unsigned char address); // range dans distance_sonar la distance lue
signed char Read_REC_Telecom(unsigned char control,char *ptr_send,char *ptr_Recv);	// Lecture code touche Telecom
//----------------------------------------------------------------------------

/****************************** PROGRAMME PRINCIPAL **************************/
void main(void)
{
	tempo_LED=4;
    init();    
         		// S�quence initialisation au d�marrage des p�riph�riques
	// On fait clignoter 3 fois la led pour indiquer la fin de l'init
/*	for (i=0; i<6; i++)
	{
		for (j=1; j<20000; j++);	// tempo d'allumage / extinction
		LED = ~LED;
	}*/
    putrsUSART (initial);           // on affiche le message de bienvenue
	putrsUSART (mode0);
	putrsUSART (mode1);
	putrsUSART (mode2);
	putrsUSART (mode3);
	putrsUSART (mode4);
	putrsUSART (mode5);
	putrsUSART (mode6);
	putrsUSART (mode7);
	putrsUSART (modeN);
	putrsUSART (modeA);
	putrsUSART (help);
	putrsUSART (stop);
//	SONAR_Ecrit (control_SONAR, cmd_SONAR, range_cm);		// demande de distance en centimetres

	while(1)            			// T�che de fond    
    { 
		// Si on a re�u un octet, on le lit et on l'affiche
        if (CAPT.ALire)                    	// si un caract�re re�u ? positionn� par IT
        {
        	Valeur = LitOctet();        	// lecture RS232 met � jour les pointeurs
        	putrsUSART (newline);			// on passe � la ligne et
//			EcritOctet(Valeur);            	// �cho du caract�re re�u
			if (ptecr == ptlec) 			// si on a lu tous les caract�res
				CAPT.ALire = NON;			// on l'indique pour ne plus tourner
			// tester le caract�re re�u par �tude de cas
			if ((Valeur >= 0x30) && (Valeur <= 0x37))	// on a actionn� de '0' � '7' ?
				mode = Valeur;				// alors le mode est remis � jour
				switch (Valeur) {			// affichage du mode activ�
					case ('0') : {	putrsUSART (mode0); break;	}
					case ('1') : {	putrsUSART (mode1); break;	}
					case ('2') : {	putrsUSART (mode2); break;	}
					case ('3') : {	putrsUSART (mode3); break;	}
					case ('4') : {	putrsUSART (mode4); break;	}
					case ('5') : {	putrsUSART (mode5); break;	}
					case ('6') : {	putrsUSART (mode6); break;	}
					case ('7') : {	putrsUSART (mode7); break;	}
					case ('?') : {
									putrsUSART (mode0);
									putrsUSART (mode1);
									putrsUSART (mode2);
									putrsUSART (mode3);
									putrsUSART (mode4);
									putrsUSART (mode5);
									putrsUSART (mode6);
									putrsUSART (mode7);
									putrsUSART (modeN);
									putrsUSART (modeA);
									putrsUSART (help);
									break;
					}				
					case ('S') : 			// on arrete le test en cours
							{  
							mode = 'S';
							putrsUSART (stop);
							putrsUSART(newline);
							putrsUSART (mode0);
							putrsUSART (mode1);
							putrsUSART (mode2);
							putrsUSART (mode3);
							putrsUSART (mode4);
							putrsUSART (mode5);
							putrsUSART (mode6);
							putrsUSART (mode7);
							putrsUSART (modeN);
							putrsUSART (modeA);
							putrsUSART (help);
							Octet_i2c (AffI2C, ecriture, 0x00);		// Reset Affichage
							PWMD = 0;								// Reset moteurs
							PWMG = 0;
							TMR1ON = 0;				// On arrete la PWM servomoteur sonar
							T2ON = 0;				// Arret PWM1,2 Moteurs
							PasSonar = 0;
							 break;  
							}
					case ('s') : 			// on arrete le test en cours
							{  
							mode = 's';
							putrsUSART (stop);
							putrsUSART(newline);
							putrsUSART (mode0);
							putrsUSART (mode1);
							putrsUSART (mode2);
							putrsUSART (mode3);
							putrsUSART (mode4);
							putrsUSART (mode5);
							putrsUSART (mode6);
							putrsUSART (mode7);
							putrsUSART (modeN);
							putrsUSART (modeA);
							putrsUSART (help);
							Octet_i2c (AffI2C, ecriture, 0x00);		// Reset Affichage
							PWMD = 0;								// Reset moteurs
							PWMG = 0;
							TMR1ON = 0;				// On arrete la PWM servomoteur sonar
							T2ON = 0;				// Arret PWM1,2 Moteurs
							PasSonar = 0;
							 break;  
							}		
				}
			if ((Valeur == 'N') || (Valeur == 'n')) 	// est-ce un mode I2C avec PCF8574 normal ?
			{
				putrsUSART (modeN);
				AffI2C = 0x20; 				// adr 7 bits PCF8574 : 0 1 0 0 A2A1A0(=000)
			}
			if ((Valeur == 'A') || (Valeur == 'a')) 	// est-ce un mode I2C avec PCF8574 A ?
			{
				putrsUSART (modeA);
				AffI2C = 0x38; 				// adr 7 bits PCF8574A: 0 1 1 1 A2A1A0(=000)
			}
			// si aucune de ces touches clavier ne convient, on ne change rien
		}

		// Routines permanentes, quelque soit le mode de test choisi		
		if (CAPT.ds)				// un �l�ment temporel � prendre en compte
			{	CAPT.ds = NON;			// on indique que l'on a g�r�
				heure += 1;				// le temps passe ...
				CAPT.Pas = OUI;			// �volution de la motorisation sonar

			/*	ACT.TempoLeds ++;		// le temps passe aussi pour le chenillard
				if (ACT.TempoLeds == 0)
					{ 
					ACT.Changement = 1;
					LED = ~LED;			// Clignotement Led Test toute les O,4s
					}
			*/
			}
		Mesure ();					// il est temps d'acqu�rir une nouvelle valeur d'une des voies

		// Maintenant, le main() va r�aliser les routines de chaque mode.

		// Mode '0' : Chenillard � leds
		if (mode == '0')
		{
			INT0IE = NON;			// autorisation d'IT ? (toujours prioritaire)
			IR_OFF = OUI;			// Arret Capteurs IRD,G
			TMR1ON = NON;			// ARRET la PWM servomoteur sonar
			T2ON = NON;				// ARRET PWM1,2 Moteurs
			if (ACT.Changement)		// les 0.4s sont pass�es ?
			{	
				ACT.Changement = 0;	// on devra attendre les 0.4s suivantes
				Octet_i2c (AffI2C, ecriture, Aff);
				if (ACT.Sens == 1)
				{	Aff >>= 1;
					if (Aff == 1) 
						ACT.Sens = 0; // on est au bout � droite, on passe � gauche
				}
				else
				{	Aff <<= 1;
					if (Aff == 0x80)
						ACT.Sens = 1; // on est au bout � gauche, on passe � droite
				}
			}
		}

		// Mode '1' : Moteurs : vitesses opos�es triangulaires > 0 puis < 0
		if (mode == '1')
		{
			INT0IE = NON;			// autorisation d'IT ? (toujours prioritaire)
			IR_OFF = OUI;			// Arret Capteurs IRD,G
			TMR1ON = NON;			// ARRET la PWM servomoteur sonar
			T2ON = OUI;				// On demarre PWM1,2 Moteurs
			if (ACT.Changement)		// les 0.4s sont pass�es ?
			{	
				ACT.Changement = 0;	// on devra attendre les 0.4s suivantes
				if (ACT.Sens == 1)	// si on est en phase acc�l�ration
				{	PWMD += 20;
					PWMG += 20;
					if (PWMD > 128) 
						ACT.Sens = 0; // on est au maxi, on va ralentir
				}
				else	// si on est en phase ralentissement
				{	PWMD -= 20;
					PWMG -= 20;
					if (PWMD <= 20)
					{
						ACT.Sens = 1; // on est au mini, on va acc�l�rer
						DirD = ~DirD; DirG = ~DirD;	// mais dans l'autre sens 
					}
				}
			}
		}

		// Mode '2' : Lecture des capteurs IR : Envoie RS232 des valeurs lues
		if (mode == '2')	// mesure() se fait en permanence
		{
			INT0IE = NON;			// autorisation d'IT ? (toujours prioritaire)
			IR_OFF = NON;			// Marche Capteurs IRD,G
			TMR1ON = NON;			// ARRET la PWM servomoteur sonar
			T2ON = NON;				// ARRET PWM1,2 Moteurs
			if (ACT.Changement)		// les 0.4s sont pass�es ?
			{	
				ACT.Changement = 0;	// on devra attendre les 0.4s suivantes
				Octet_i2c (AffI2C, ecriture, (MesDistD[4]>>1) + (MesDistG[4]>>1));	// Affiche valeur moyenne
				itoa(MesDistD[4], buffer);
				EcritChaine (buffer);
				putrsUSART (separe);
				itoa(MesDistG[4], buffer);
				EcritChaine (buffer);
				putrsUSART (newline);
			}
		}

		// Mode '3' : Lecture sonar : Envoie RS232 de la valeur lue et sur leds
		if (mode == '3')
		{
			if (ACT.Changement)		// les 0.4s sont pass�es ?
			{
				INT0IE = NON;			// autorisation d'IT ? (toujours prioritaire)
				IR_OFF = OUI;			// Arret Capteurs IRD,G
				TMR1ON = NON;			// ARRET la PWM servomoteur sonar
				T2ON = NON;				//	ARRET PWM1,2 Moteurs	
				ACT.Changement = 0;	// on devra attendre les 0.4s suivantes
				// une r�ponse sonar est-elle disponible ? si oui, on l'envoie sur liaison s�rie
				if (CAPT.Sonar == OUI)			// On vient ici toutes les 100ms
				{
					SONAR_Ecrit (control_SONAR, cmd_SONAR, range_cm);		// demande de distance en centimetres 
					CAPT.Sonar = NON;
					while(!CAPT.Sonar);										// Attente r�ponse 100ms
					distance_sonar = SONAR_Lit (control_SONAR, dist_cible);	// Lecture des deux octets distance
					CAPT.Sonar = NON;
					if (distance_sonar > 255)	
						distance_sonar = distance_sonar & 0xFF;		// on ne garde que l'octet faible
					Octet_i2c (AffI2C, ecriture, distance_sonar);	// affichage sur I2C (4bits faibles)
					itoa(distance_sonar,buffer);					// ici on convertit distance en chaine pour affichage
					putsUSART(buffer);							// Ecrit la distance sur RS232
					putrsUSART(newline);
				}
			}
		}

		// Mode '4' : Commande servo Sonar : Tourne de droite � gauche
		if (mode == '4')
		{
			INT0IE = NON;			// autorisation d'IT ? (toujours prioritaire)
			IR_OFF = OUI;			// Arret Capteurs IRD,G
			TMR1ON = OUI;			// ARRET la PWM servomoteur sonar
			T2ON = NON;				//	ARRET PWM1,2 Moteurs
			if (ACT.Changement)		// les 0.4s sont pass�es ?
			{	
				ACT.Changement = 0;	// on devra attendre les 0.4s suivantes
				if (CAPT.Pas)		// on modifie les valeurs de DurBas et DurHaut
				{	CAPT.Pas = NON;	// on enl�ve le drapeau
					if (Gisement_D == OUI) 		// SENS indique le sens d'�volution des dur�es hautes
					{ 	DurHaut += QuantServo;
						DurBas = Periode_PWM_servo - DurHaut; 

						PasSonar +=5;			// on augmente de 5�
						if (DurHaut > MaxHaut) 	// est-on � +90�
						{	DurHaut = MaxHaut;
							Gisement_D = NON;			// on repasse en diminution
						}
					}
					else 
					{	DurHaut -= QuantServo;
						DurBas = Periode_PWM_servo - DurHaut; 
						PasSonar -=5;			// on diminue de 5�
						if (DurHaut < MinHaut )  	// est-on � -90�
						{	DurHaut = MinHaut;
							Gisement_D = OUI;			// On repasse en augmentation
						}
					}
					Octet_i2c (AffI2C, ecriture, PasSonar);	// affichage sur I2C (4bits faibles)
					itoa (PasSonar, buffer);
					EcritChaine (buffer);
					putrsUSART (teta);
					putrsUSART (newline);
				}
			}
		}

		// Mode '5' : Lecture t�l�commande affich�e sur leds et RS232
		if (mode == '5')
		{
			INT0IE = OUI;			// autorisation d'IT ? (toujours prioritaire)
			IR_OFF = OUI;			// Arret Capteurs IRD,G	
			TMR1ON = NON;			// ARRET la PWM servomoteur sonar
			T2ON = NON;				// Arret PWM1,2 Moteurs
			if (CAPT.Tel)	// si la t�l�commande a agit
			{
				CAPT.Tel = 0;							// on a g�r�
				Read_REC_Telecom(Rec_Telecom,Send_I2C_Telecom_ASCII_Buf,Recv_I2C_Telecom_ASCII_Buf);
				Valeur = Recv_I2C_Telecom_ASCII_Buf[1];							// masquage des 4 bits de poids faible
				Octet_i2c (AffI2C, ecriture, Valeur &0x0F );	// affichage sur I2C (4bits faibles)
				switch (Valeur & 0x0F)
				{										// affichage sur RS232 des 4 bits binaires D9 .... D6
					case (Haut) :
					{	putrsUSART (haut); break;	}
					case (Bas) :
					{	putrsUSART (bas); break;	}
					case (Gauche) :
					{	putrsUSART (gauche); break;	}
					case (Droite) :
					{	putrsUSART (droite); break;	}
					case (Centre) :
					{	putrsUSART (centre); break;	}
				}
			}
		}

		// Mode '6' : Lecture des capteurs de rotation, affich�s sur 2 leds poids forts et RS232
		if (mode == '6')
		{
			INT0IE = NON;			// autorisation d'IT ? (toujours prioritaire)
			IR_OFF = OUI;			// Arret Capteurs IRD,G
			// on lit l'�tat des capteurs de rotations et on change le poids fort
			if (PA != CAPT.OldPA) // si du nouveau sur le capteur chenille droite
			{	Valeur = (Valeur ^ 0x80) & 0x80; CAPT.OldPA = PA; CAPT.Ecrit = OUI;putrsUSART(chenilleD);	}
			if (PB != CAPT.OldPB) // si du nouveau sur le capteur chenille gauche
			{	Valeur = (Valeur ^ 0x40) & 0x40;	CAPT.OldPB = PB; CAPT.Ecrit = OUI;putrsUSART(chenilleG);	} 
			// finalement, on �crit le code des leds selon les capteurs
			if (CAPT.Ecrit) 
			{	Octet_i2c (AffI2C, ecriture, Valeur);
				CAPT.Ecrit = NON;
			}
		}

		// Mode '7' : Lecture de la tension batterie, envoie de la tension sur RS232
		if (mode == '7')
		{
			INT0IE = NON;			// autorisation d'IT ? (toujours prioritaire)
			IR_OFF = OUI;			// Arret Capteurs IRD,G
			if (ACT.Changement)		// les 0.4s sont pass�es ?
			{	
				ACT.Changement = 0;	// on devra attendre les 0.4s suivantes
				Octet_i2c (AffI2C, ecriture, MesBatt[4]);
				itoa(MesBatt[4], buffer);
				EcritChaine (buffer);
				putrsUSART(newline);
			}
		}
    }	// fin du while(1)
}		// fin du main()
//----------------------------------------------------------------------------

void init(void)
// Initialise les timers T0 pour le cadencement des mesures et pour la PWM servomoteur
{
	OSCCON = 0b01110000;     // select 8MHz internal osc. 
	OSCTUNEbits.PLLEN = 1;       // enable 4xPLL ;Fosc = 32MHz
	GIE = 0;				// inhibe IT globale en attendant la fin des init

	//  Param�trage des ports, en lots      
	TRISA = 0b00011111; 	// 5 bits bas en entr�es : 3 capteurs, batterie, PA
 	TRISB = 0b11011101;		// tout en entr�e sauf RB1 (IR_OFF) la led en RB5 (TEST)
	TRISC = 0b10011001;		// tout en sorties sauf RC0 = PB, RC7=Rx et I2C, en entr�e
	LED = 1;				// on allume la led d�s maintenant
	RBPU = 1;				// portB avec r�sistances de tirage NLH non employ�es
	IR_OFF = OUI;				// Arret IRD, IRG

	// INT0 doit interrompre pour acqu�rir t�l�commande
	INTEDG0 = 1,			// front Montant actif (allumage de la led)
	INT0IE = NON;			// autorisation d'IT (toujours prioritaire)
  
 	// ADC() Param�trage du convertisseur ANA_NUM
	PCFG = 0xC;			// Seuls les voies AN0, AN1 et AN2 sont analogiques si pas mesure VITMOT D/G
	VCFG = 0;			// Vref+ = VDD; Vref- = Vss
    CHS = 0;			// on commence avec AN0
    ADCS = 6;			// fr�q. conv. bit = Fosc / 64 TAD = 8,2�s
    ADFM = 0;			// format� � gauche, pour n'exploiter que l'octet de poids fort 
	ACQT = 7;			// delai de lecture CAN de 20 x TAD = 2,56�S > 2,4�S
    ADON = 1;			// activation du convertisseur

   	// Timer0() pour IT active par d�bordement toutes les 128�s environ
  	TMR0H = 0;          	// raz timer
  	TMR0L = 0;          	// raz timer
	T08BIT = 1;				// mode 8 bits actif
  	T0CS = 0;       		// horloge timer 0= 32MHz/4 T(T0) 8MHz = 0,125�s 
  	PSA = 0;        		// pr�-division interne active  
	T0PS = 1;				// pr�division par 4 : d�bordement et ITTMR0 toutes les 128�s
 	TMR0IE = 1;				// valide IT sur TMR0
//  TMR0IP = 1;     		// TMR0 en haute priorite (adresse 8)
//	RCONbits.IPEN = 1;  	// validation des niveaux de priorit� des IT
	TMR0ON = 1;					// Lancement du Timer0
	// pour battre au rythme de 1ms, il faudra diviser par 8 !

	// Timer1() Sert au mouvements sonar : PWM de 50Hz 5 � 10 % et stepper (20ms plus tard 100ms)
	// on doit donc d�tecter la retenue de Timer 1 avec rechargement DurBas DurHaut
	RD16 = 1;				// On manipule TMR1 en 16 bits
	TMR1CS = 0;				// Horl interne F = 32/4 = 8MHz
	T1SYNC = 1;				// pas de synchronisation
	T1CKPS = 3;				// pr�division par 8 pour compter la �s 
	TMR1H = 0xFA;			// Mat sonar Gisement 0�
	TMR1L = 0x24;
	TMR1IE = OUI;			// autorisation d'IT en fin de periode du duty cycle PWM Sonar
//	TMR1IP = OUI;			// IT prioritaire
	TMR1ON = 0;				// Arret PWM servomoteur sonar
	
	// Timer2() 32MHz/4 / 4 / 256. P�riode PWM avec les valeurs suivantes on est � 2kHz
	PR2 = 0xFF;		// � ce stade le Timer est OFF on divise par 256 p�riodes du PWM1 et PWM2 
	T2CKPS = 2;   	// Pr�scaler division par 16 et ici TMR2 off 
	T2OUTPS = 0;	// pas de post division une IT � chaque 0.5ms TMR2IF
//	TMR2IP = OUI;	// IT prioritaire

	// PWM DROIT et GAUCHE
	CCP1M = 0x0C;  	// Mode PWM1 
	DC1B = 0;		// Deux Bits poids faibles du DUTY_CYCLE
	PWMD = 0;		// Envoi commande PWMD   
	CCP2M = 0x0C;   // Mode PWM2 
	DC2B = 0;		// Deux Bits poids faibles du DUTY_CYCLE
	PWMG = 0;		// Envoi commande PWMG   
	   
	T2ON = 0;     	// Standby PWM1 et PWM2

	// initialisation de la partie Liaison s�rie asynchrone
	TXSTA = 0b00100100; 	// 8 bits, valid �mission, asynchrone, pas de sync, BRGH=1 High speed                          
	RCSTA = 0b10010000;			// active l'USART, 8bits,  CREN=1 et SPEN=1
	BAUDCON = 0;                // brg16 = 0 pour baud rate sur 8 bits
	SPBRG = 207; 				// baud rate : 32M/(16(n+1)) donc pour 9600 bauds = 207
	udata = 0;               	// on n'a rien re�u

	// initialisation de la partie liaison s�rie en interruption
	ptecr = buffer;				// le buffer de r�ception est d�fini au d�but du tampon
	ptlec = ptecr;				// efface le buffer de reception en pointant lecture idem
	TRISCbits.TRISC7 = 1;		// RX - broche en entr�e
	TRISCbits.TRISC6 = 0;		// TX - broche en sortie
	PIR1bits.TXIF=0;			// IT en emission d�sactiv�e
	PIE1bits.RCIE=1;			// IT en reception activ�e
//	RCONbits.IPEN=1;			// active la priorit� sur les IT
//	IPR1bits.RCIP=1;			// selectionne haute priorit� pour RX (vecteur en 0x8)

	// initialisation de la partie I2C
	TRISCbits.TRISC3 = 1;       // SCL (PORTC,3) en entr�e
 	TRISCbits.TRISC4 = 1;       // SDA (PORTC,4) en entr�e
 	SSPCON1=0b00101000;			// pose WCOL SSPOV SSPEN CKP SSPM3:SSPM0 
 	// efface WCOL et SSPOV, active I2C, I2C mode maitre horloge=FOSC/(4*(SSPADD+1))	 	SMP = 1;					// slew rate inhib� pour vitesses normales(f<400Khz)
 	SSPADD = 127;				// horloge = 32Mhz / 4*(127+1)  = 62,5 KHz

	PEIE = 1;     	// Validation globale des interruptions
  	INTCONbits.GIEH = 1;

	// initialisation des variables
	Tick = 0;Tac=0;	Valeur = 0;  CAPT.ds = NON;	// Valeur peut �tre un octet ou un entier
	STEP = 0;	DirD = 0;	DirG = 0;	CAPT.Tel = NON;
	DurHaut = 1500;	// on commence au milieu
	DurBas = Periode_PWM_servo - DurHaut; 
	CAPT.Sonar = NON;	Aff = 1; ACT.Sens = 0;
	Gisement_D = OUI;
	PWMD = PWMG = 0;	DirD = 0; DirG = 1;	// on commence � l'arr�t avec les sens initiaux
	PasSonar = 0; // on commence l'orientation sonar � 0 pour "droit devant"
}

void Mesure (void)
{	// on acqui�re ici toute la campagne de conversion de mesures par passage successif (12 passages) 
	unsigned char Val;
	if (!DONE)			// on passe alors au canal suivant sauf si campagne finie --> AD OFF
	{
		Val = ADRESH;    	// lecture de l'octet converti
		switch (NumConv) 	// selon NumConv, on range dans le tableau ad�quat le r�sultat
		{
			case 0 : {  Moyenne(MesDistD, Val); 	break;}		// AN0 : CaptD
			case 1 : {  Moyenne(MesDistG, Val); 	break;}    	// AN1 : CaptG
			case 2 : {  Moyenne(MesBatt, Val); 		break;}  	// AN2 : Vbat
		}
        NumConv++;
		if (NumConv > 2)      // si toutes voies converties, 
		{
            NumConv = 0;       // on revient � la premi�re voie
			NumEch++;			// �chantillon nouveau de la nouvelle voie
			if (NumEch > 3)
			{
				NumEch = 0;
				FinAcq = OUI;	// on a eu les 12 mesures utile � la navigation
			}
		}
		CHS = NumConv;  	// on s�lectionne cette voie CAN
		GO = 1;					// lancement conversion de la nouvelle voie choisie
	}
}

void Moyenne (unsigned char * tab, unsigned char val)
// la neuvi�me valeur du tableau pass� en param�tre est affect� de la valeur
// moyenne des 7 valeurs ant�rieures. le second param�tre, valeur r�cente 
// est rang�e en premi�re case. Les cases sont d�plac�es � gauche durant le calcul
{
	char i;
	unsigned int total = 0;
    
	for (i=1; i<4; i++)
	{
        	total += tab[i];
		tab[i-1] = tab[i];
 	}
   	tab[4] = (total + val) >> 2; 	// moyenne en derni�re case
	tab[3] = val;   				// valeur ajout�e en avant derni�re case
}

unsigned char Octet_i2c(unsigned char adresse, unsigned char sens, unsigned char valeur) 
// dialogue avec le destinataire d�fini par adresse
// si SENS = 1 : on lit � l'adresse et on rend la donn�e en valeur
// si SENS = 0 : on �crit l'octet valeur obtenu mais compl�ment� car les Leds sont tir�es au NLH
// On travaille en maitre avec un diviseur d'horloge pour un bit de excatement 10�s soit F = 40kHz
{
	unsigned char t;			// tampon interm�diaire pour octet re�u et pour compte-rendu
	if (sens == lecture)
	{
		while((SSPCON2 & 0x1F) || R_W);  // Attente module idle
 		SEN = 1;			// START
 		while (SEN);		// Attente de fin de condition start (RAZ automatique)
 		SSPBUF = (adresse << 1) + sens; 	// Ecriture de l'adresse avec le bit Lecture 
		while (BF);			// on attend la fin d'�criture de l'adresse
		while (ACKSTAT);		// on attend que l'acquittement ait eu le temps d'�tre mis

		while( SSPCON2 & 0x1F | R_W)
 		RCEN = 1;			// On passe en mode r�ception
		while (!BF);		// On attend la fin de r�ception de l'octet mis par le destinataire
		t = SSPBUF;			// Lecture de l'octet re�u
 		ACKDT = 1;			// pas d'acquittement car on arr�te l� : NON-ACK
 		ACKEN = 1; 			// lancement du cycle acquittement
 		while (ACKEN);		// dont on attend la fin

 		PEN = 1;			// lancement de la condition de STOP
 		while (PEN);		// On attend la fin de temps de stop
		SSPIF = 0;			// effacement du drapeau qui indiquera la fin de stop
 		return (t); 		// t �tant l'octet re�u, on le retourne
	}
	if (sens == ecriture)
	{
		while((SSPCON2 & 0x1F) || R_W);  // Attente module idle
 		SEN = 1;				// START
 		while (SEN);			// Attente de fin de condition start (RAZ automatique)
 		SSPBUF = (adresse << 1) + sens; 	// Ecriture de l'adresse avec le bit Ecriture 
		while (BF);			// Attendre fin ecriture adresse
		while(ACKSTAT);		// Attendre ACK slave

		while((SSPCON2 & 0x1F) || R_W);  // Attente module idle
		SSPBUF = ~valeur;	// on �crit l'octet � �mettre en compl�ment� et c'est parti
		while(BF);			// Attendre buffer vide
		while (ACKSTAT);	// on attend  ACK

		while((SSPCON2 & 0x1F) || R_W);  // Attente module idle
 		PEN = 1;				// STOP
 		while (PEN);			// attente fin du STOP
		SSPIF =0;				//Remise � jour du flag pour IT Stop  
	}
}

//----------------------------------------------------------------------------
// Pose du vecteur d'interruption haute priorit�
#pragma code InterruptVectorHigh = 0x08
void InterruptVectorHigh (void)
{
  	_asm
  		goto ITHaute 			// on y met le saut � la routine IT
  	_endasm
}
//----------------------------------------------------------------------------
// Routine interruption haute priorit�

#pragma code
#pragma interrupt ITHaute
void ITHaute ()					
{	
			
	if (TMR0IF)		// L'IT d�bordement survient environ toutes les 128�s !
  	{    
	                      
    	TMR0IF = 0; 	// si oui, on efface la cause
		Tick++;
		if (Tick == 8) 	// passage � 1ms, p�riode de la PWM servomoteur
		{	Tac ++;	Tick = 0;			}
		if (Tac == 100)			// d�tection des 0.1s de changement de leds
		{ //	LED=~LED;
			tempo_LED--;
			if(	tempo_LED==0){	tempo_LED=4;LED=~LED;	ACT.Changement = 1; }
	
			Tac = 0; CAPT.ds = 1; CAPT.Sonar = OUI;}	// ds = d�ci seconde
  	}

	if (TMR2IF)					// est-ce la PWM qui interrompt � 2kHz ?
		{ 	
		TMR2IF = 0;				// on efface la demande
		// doit mettre � jour la valeur du rapport cyclique selon CaptG
		}
	if (TMR1IF)		// apr�s niveau haut ou bas, il faut reprogrammer la PWM servomoteur
	{
		TMR1ON = 0;
		TMR1IF = 0;				// on efface la demande
		if (STEP) 				// si NLB on charge la dur�e haute
		{	STEP = 0;
			temp = 0xFFFF - DurBas;
			TMR1H = (temp >> 8) & 0xFF;	// nouvelle valeur de TMR1 fin cycle PWM srvo
			TMR1L = temp & 0xFF;		// sur 16 bits
			Nop();
			TMR1ON = 1;
		}
		else 
		{	STEP = 1;
			temp = 0xFFFF - DurHaut;
			TMR1H = (temp >> 8) & 0xFF;	// nouvelle valeur de TMR1 fin cycle PWM srvo
			TMR1L = temp & 0xFF;		// sur 16 bits
			Nop();
			TMR1ON = 1;
		}
	}
	if (RCIF)                // detection IT USART en reception
	{
		RCIF = 0;						// on efface la demande
		c = RCREG;						// on stocke l'octet re�u
		*ptecr++ = c;					// dans le tableau, et on pointe la suite
		if (ptecr >= &buffer[SIZE])		// si on est sorti de l'espace de stockage
			ptecr = buffer;				// on revient (pile tournante)
		CAPT.ALire = OUI;				// on indique qu'il faut lire
	}        
	if (INT0IF)				// on a re�u un ordre de la t�l�commande
	{
		INT0IF = NON;					// �limination de la demande
		CAPT.Tel = OUI;
	}
}
//----------------------------------------------------------------------------

/****************************************************************************
*            Ensemble de routines perso pour g�rer la liaison s�rie     	*
*  d'un PIC18F2520 en mode 9600 baud sans Par, 8 bits, 1 stop        		*
*****************************************************************************/
//        Reception d'un caract�re. boucle en attendant la r�ception.
unsigned char LitOctet (void)
{
	unsigned char c;                                
	while (ptlec == ptecr);     	// attente que les pointeurs diff�rent
	c = *ptlec++;               	// on prend le caract�re suivant
	if (ptlec >= &buffer[SIZE])   	// si on est sortie de la pile, 
		ptlec = buffer;            	// on revient au d�but de la pile tournante
	return (c);                  	// on retourne l'octet nouvellement re�u
}

/*/ Envoie sur RS232 la chaine ascii d'un octet
void EcritValOctet (int oct)
{
	EcritOctet (0x30 + (oct div 100));			// chiffre des centaines en ascii
	EcritOctet (0x30 + ((oct mod 100) div 10));	// chiffre des dizaines en ascii
	EcritOctet (0x30 + (oct mod 10));			// chiffre des unit�s en ascii
}

/* Cette fonction retoure 1 (vrai) si un caract�re est disponible dans le buffer de r�ception
et 0 si non, si les deux pointeurs sont identiques, il n'y a rien dans le buffer*/
char carUSARTdispo(void)
{
	return (ptecr - ptlec);
}

// emission d'un caract�re sur RS232
void EcritOctet (unsigned char c)
{
	while (!TXSTAbits.TRMT);        // on attend que la transmission pr�c�dente soit finie
	TXREG = c;                      // envoi d'un caract�re 
	while (!PIR1bits.TXIF);         // on attend que TXIF passe � 1
}

// Retourne une chaine de caract�re
unsigned char *LitChaineUart (unsigned char *s, unsigned char finst)
{
	unsigned char *r = s;              // tampon m�moire pointant la chaine � pointer
	char c;                                                        
	while ((c = LitOctet()) != finst)  // c est l'octet su sur l'UART jusqu'� �tre le d�limiteur
		*s++ = c;                  		// de fin de chaine, on range � la suite les octets
	*s = 0;                            // on ajoute un caract�re nul pour clore la chaine
	return (r);                        // on retourne l'adresse de la chaine cr�e
}

/* envoie une chaine de caract�re en ROM*/
int EcritChaineRom(rom unsigned char *s)
{
	int n;                          // compteur d'octet � �mettre
	unsigned char a;
	for (n = 0; *s; n++)            // on lit la m�moire jusqu'� trouver 0
	{
		a = *s;
		EcritOctet(a);           	// on �met l'octet trouv�
		s++;
	}        
	return 1;
}

/* envoie une chaine de caract�re en RAM*/
int EcritChaine (unsigned char *s)
{
	int n;
	for (n = 0; *s; n++, s++) 
		EcritOctet (*s);
	return 1;
}

// putrsUSART()  Ecrit une chaine en ROM vers l'USART sans le caract�re NULL \0
void putrsUSART(const rom char *data)
{
	do {                                
		while ( ! (TXSTA & 0x02) );  // on attend que l'uart soit libre
		TXREG = *data;
	} while ( *++data );             // on arr�te si le prochain caract�re point� est nul
}

// putsUSART() Ecrit une chaine en RAM vers l'USART sans le caract�re NULL \0
void putsUSART(char *s)
{
	do {                                
		while ( ! (TXSTA & 0x02) );     // on attend que l'uart soit libre
		TXREG = *s;
	} while ( *++s );                   // on arr�te si le prochain caract�re point� est nul
}
//---------------------------------------------------------------------
// SONAR_Ecrit()
// Ecrit un octet de commande dans le sonar connect� � l'I2C
// � l'adresse indiqu�e 0x00.
//---------------------------------------------------------------------

void SONAR_Ecrit (char controle, unsigned char adresse, char data)
{
	// variables locales
	char erreur, essai;

	erreur = 0;
	essai = 10;						// On essaiera 10 fois d'acc�der au SONAR
	do
	{
		erreur = EEAckPolling (controle);
		essai--;
	} 
	while (erreur && essai > 0); 
	essai = 10;						// R�essai 10 fois MAX
	do								// Tentative d'�criture de l'octet faible de donn�e
	{
		erreur = EEByteWrite (controle, adresse, data);
		essai--;
	} 
	while (erreur && essai > 0);
}

//---------------------------------------------------------------------
// SONAR_Valid()
// Lecture Version srf02 a l'adresse 0x00 sur I2C qi permet d'attendre la fin du ranging
// retour du numero de version sur un char.
//---------------------------------------------------------------------

unsigned int SONAR_Valid (char controle, unsigned char adresse)
{
	int temp;
	char erreur, essai;

	erreur = 0;
	essai = 10;						// On essaiera 10 fois d'acc�der au SONAR
	do
	{
		erreur = EEAckPolling (controle);
		essai--;
	} 
	while (erreur && essai > 0); 
	essai = 10;						// R�essai 10 fois MAX
	do								// Tentative de lecture de l'octet fort de donn�e
	{
		temp = EERandomRead (controle, adresse);
		essai--;
	} 
	while (temp && essai > 0);
	return temp;
}


//---------------------------------------------------------------------
// SONAR_Lit()
// Reads an integer value from SONAR connected to the I2C bus at
// the specified location MSB[0X02], LSB[0x03].
// Return 0x0000 if error or distance = 0x[MSB][LSB] ranging
//---------------------------------------------------------------------

unsigned int SONAR_Lit (char controle, unsigned char adresse)
{
// variables locales
	union 
		{	char b[2];
			int i;
		}	data;
	union 
		{	char b[2];
			int i;
		}	temp;
	char erreur, essai;

	erreur = 0;
	essai = 10;							// On essaiera 10 fois d'acc�der au SONAR
	do
	{
		erreur = EEAckPolling (controle);
		essai--;
	} 
	while (erreur && essai > 0); 
	essai = 10;							// R�essai 10 fois MAX
	do									// Tentative de lecture de l'octet fort de donn�e
	{
		temp.i = EERandomRead (controle, adresse);
		essai--;
	} 
	while (temp.b[1] && essai > 0);
	if (temp.b[1]) data.b[0] = 0xFF;	// Mise du r�sultat de lecture � 0 si erreur
	else data.b[1] = temp.b[0];			// Sinon, mise de l'octet fort de donn�e lue
	essai = 10;							// R�essai 10 fois MAX
	do									// Tentative de lecture de l'octet faible de donn�e
	{
		temp.i = EERandomRead (controle, adresse + 1);
		essai--;
	} 
	while (temp.b[1] && essai > 0);
	if (temp.b[1]) data.b[1] = 0xFF;	// Mise du r�sultat de lecture � 0 si erreur
	else data.b[0] = temp.b[0];			// Sinon, mise de l'octet faible de donn�e lue
	return data.i;
}

/********************************************************************************************
*     Function Name:    Read_REC_Telecom                                                    *
*     Return Value:     signed char -1 si erreur 0 si OK         	                        *
*     Parameters:       char control Adresse physique sur bus I2C (R/W = 0)                 *
*						*ptr_send poiteur sur buffer mode de fonctionnement interface *
*							chaine de deux char '1','\0' mode normal reception		        *
*							chaine de deux char '2','\0' mode test							*
*						*ptr_recv pointeur sur buffer chaine char recus               *
*							termine par '\0'									            *										  *
*     Description:      Effectue la lecture des donnees recues dans le mode                 *
*                       specifique										                    *
*                                                                                           *  
*********************************************************************************************/
signed char Read_REC_Telecom(unsigned char control,char *ptr_send,char *ptr_recv)
	{
	char error = 0;
	char temp;	
  	while ((SSPCON2 & 0x1F) || (R_W));  // module disponible?
	SEN=1;        // initialise START condition
	while(SEN);	 // Attente fin start condition
	temp = SSPBUF;			// Vide SSPBUF
	do
		{
	  	SSPBUF = (control|0x00);		// Ecrit adresse physique R/nW = 0
		if(WCOL)	// Si collision
			error = -1;			// signale erreur collision
		while(BF);	// Attente buffer TX vide
	  	while ((SSPCON2 & 0x1F) || (R_W));  // module disponible?
		if(ACKSTAT)	//si nACK esclave
			error = -2;			// signale l'erreur
		else
			error = 0;			// Pas d'erreur
		if(error == -1)	// Si collision
			{
			temp = SSPBUF;	// on vide SSPBUF
			WCOL = 0;	// reset collision bit WCOL
			}
		}while(error !=0);		// Fin envoie adresse physiqu

	do						// Ecriture du Buffer mode de fonctionnement
		{
		do
			{
	  		SSPBUF = *ptr_send;		// Transmission du mode de fonctionnement
			*ptr_send++;
			if(WCOL)	// Si collision
				error = -1;
			while(BF);	// Attente buffer TX vide
	  		while ((SSPCON2 & 0x1F) || (R_W));  // module disponible ?
			if(ACKSTAT)	//si nACK esclave
				{
				error = -2;			// signale erreur
 				while ((SSPCON2 & 0x1F) || (R_W));  // module disponible ? 
				RSEN=1;        // initialise RE-START condition
				while(RSEN);	 // Attente fin RE_START condition
				temp = SSPBUF;	// Pour vider SSPBUF
				do
					{
	  				SSPBUF = (control|0x00);		// Ecrit adresse physique R/nW = 0
					if(WCOL)	// Si collision
						error = -1;			// signale erreur collision
					while(BF);	// Attente buffer TX vide
				  	while ((SSPCON2 & 0x1F) || (R_W));  // module disponible?
					if(ACKSTAT)	//si nACK esclave
						error = -2;			// signale l'erreur
					else
						error = 0;			// Pas d'erreur
					if(error == -1)	// Si collision
						{
						temp = SSPBUF;	// on vide SSPBUF
						WCOL = 0;	// reset collision bit WCOL
						}
					}while(error !=0);		// Fin envoie adresse physiqu
							}
						else
							error = 0;			// pas d'erreur
						if(error == -1)	// Si collision
							{
							temp = SSPBUF;	// on vide SSPBUF
							WCOL = 0;	// reset collision bit WCOL
							}
						}while(error !=0);
					}while(SSPBUF != '\0');		// Fin envoie Buffer


  	while ((SSPCON2 & 0x1F) || (R_W));  // module disponible ? 
	RSEN=1;        // initialise RE-START condition
	while(RSEN);	 // Attente fin RE_START condition
	temp = SSPBUF;	// Pour vider SSPBUF
	do
		{
  		SSPBUF = (control|0x01);		// Ecrit adresse physique R/nW = 1
		if(WCOL)	// Si collision
			error = -1;			// signale erreur
		while(BF);	// Attente buffer TX vide
  		while ((SSPCON2 & 0x1F) || (R_W));  // module disponible ?
		if(ACKSTAT)	//si nACK esclave
			error = -2;			// signale erreur
		else
			error = 0;			// pas d'erreur
		if(error == -1)			// Si collision
			{
			temp = SSPBUF;	// on vide SSPBUF
			WCOL = 0;	// reset collision bit WCOL
			}
		}while(error !=0);
	do							// Boucle de reception des octets
		{
		RCEN = 1;    	//Commute Maitre I2C en reception pour tous les octets
		while(!BF);		// Attente buffer TX plein
  		*ptr_recv = SSPBUF;	// Archive dans le buffer Recv
		while(RCEN);	// Attente fin RCEN ms � 0 par Hardware
		*ptr_recv++;
			if(SSPBUF != '\0')		// si pas fin reception ?
				{
				ACKDT = 0;	// ACK pour transaction
				ACKEN = 1;	// Envoi ACK
				while(ACKEN);	// Attendre fin ACK
				}
			else					// si dernier octet recu ?
				{
				ACKDT = 1;	// nACK pour transaction
				ACKEN = 1; 	// Envoi nACK fin transaction
				while(ACKEN);	// Attendre fin nACK
				}
		}while (SSPBUF != '\0');	// Fin reception
	PEN = 1;            // envoi STOP condition
    while(PEN);  		// attente fin stop condition  
  	return (error); 				// retour ode de l'erreur	
	}	


