/********************************************************************************
  *		TEST COMPLET - Pour tester la carte ROBOT en MAP après montage			*
  *******************************************************************************
  * Description sommaire des entrées et sorties    :                            *
  * En entrées :                                                                *
  *  -  Deux capteurs distance IR_DROITE, IR_GAUCHE dynamique du signal 		*
  *     analogique capteur IR 0, 3 volts, Cadencement à 1ms environ             *
  *           IR_DROITE sur AN0          IR_GAUCHE sur AN1						*  
  *  -  Image de la tension de la batterie pour surveillance V_BATT 0, 5volts  	*
  *     sur AN2 filtrée on arrètera le robot si VBATT moyennée atteint 10V      *
  *  -	Un capteur TTL à l'arrière annonce un obstacle par NLH : RA3			*
  *  -	Deux signaux TTL en  RA4 PA (gauche) et RC0 PB (droite) indiquent par 	*
  * 	un front un déplacement moteur des chenilles de 0.9cm.					*
  *  - Une télécommande décode et mémorise le code de la touche si valide.      *
  *      Un signal IT annonce une réception télécommande : RB0 actif niveau bas *        
  *      L'octet code touche accessible via bus I2C adresse 0xA2                *
  *  - Un sonar I2C d'une portée de 1,5m détecte les objets avant et cotés		*
  *  - Une liaison série permet de recevoir des commandes 9600 bauds, 8 bits, 	*
  *		pas parité, 1 stop														*
  * En sorties :																*
  *  -  Deux commandes moteurs DROITE et GAUCHE signaux TTL actif à 1			*
  *     PWM_GAUCHE 10, 50% sur CCP1           PWM_DROITE 10, 50% sur CCP2		*
  *     DIRECTION_GAUCHE sur RA7                  DIRECTION_DROITE sur RA6 		*
  * Relevés du moteur employé dans l'embase : 6V, 0.2A à vide, 0.6A en cas de	*
  *  	patinage, R=4.5Ohm en série avec une inductance de 1.6mH avec 9 tours  	*
  *  	de chenilles (38,7cm) en 15s sous 6V à vide soit 14 m/mn                *
  *  -  Une LED contrôle d'activité sur RB5 active à 1 indique :                *
  *  -  8 Leds actionnées par I2C avec PCF8574 Adresse A2-A0 = 000 =0x40+R/W	*
  * 	rendent compte de l'état du robot										*
  *  - En option, Un Servo Moteur de 180° permet d'orienter le sonar avec signal*
  *		STEP en RC5 niveau haut de 1ms à 2ms sur une période de 20ms			*
  * Cadencement des tâches de Contrôle Commandes périodique par interruption 	*
  * 	du TIMER0 environ 1ms sert à détecter le signal IT de télécommande 		*
  *	Toutes les 10 x T(TMR0)=10ms, lecture d'un capteur analogique et du sonar	*
  *	et toutes les 100 x T(TMR0)=100ms remise en cause de la navigation     		*
  * Mise au point et programmation                                             	*
  *******************************************************************************     
  *	Ce programme teste toute la carte sauf l'EEPROM. Après 3 flashs au reset :	*
  * - On affiche un message sur la liaison série et on attend une action clavier*
  *	Selon la touche clavier actionnée, on effectue le test correspondant :		*
  *	- Touche 0 : Chenillard des leds 											*
  *	- Touche 1 : Moteurs : vitesses oposées triangulaires >0 puis <0			*
  *	- Touche 2 : Lecture des capteurs IR : Envoie RS232 des valeurs lues		*
  *	- Touche 3 : Lecture sonar : Envoie RS232 de la valeur lue et sur leds		*
  *	- Touche 4 : Commande servo Sonar : Tourne de droite à gauche				*
  *	- Touche 5 : Lecture télécommande affichée sur leds et RS232				*
  *	- Touche 6 : Lecture des capteurs de rotation, affichés sur leds et RS232	*
  *	- Touche 7 : Lecture de la tension batterie, envoie de la tension sur RS232	*
  *	- Touche A : L'adressage du PCF8574 passe à celui du PCF8574A				*
  *	- Touche N : L'adressage du PCF8574A passe à celui du PCF8574				*
  *	- Touche S : Stoppe le test en cours et affiche le menu                     *
  *	- Touche ? : Affiche le menu                                                *
  *******************************************************************************
  *	Pour cela on va faire fonctionner le PIC 18F2520 à 32MHz par la PLL			*
  *	On va utiliser le Timer 2 pour les 2 PWM des moteurs à environ 2kHz			*
  *	On va utiliser le Timer 0 comme base de temps interne IT toutes les 0,5us	*
  *	On va utiliser le TImer 1 comme rythme de motorisation sonar à 50Hz			*
  *	Une seule IT hard : le signal INT0 son front descendant IT télécommande		*
  *******************************************************************************/
#include "p18f2520_2.h"		// définitions des structures du processeur à la sauce JPB
					// c'est à dire nom direct des bits et champs numériques regroupés
#include "DefReg2520.h"		// définitions persos des noms de broches et de registres
#include "En Tete_P2016.h"			// description globale de l'application et définitions
#include <stdlib.h>			// pour gestion des chaines de caractères
#include <i2c.h>			// bibliothèque microchip pour l'I2C

#define MinBat 	150			// la tension mesurée = Vbatterie - 0.6V est atténuée dans un rapport 1/3,2
// le seuil de 10V correspond donc à 9.4/3.2 = 2.93V soit 255*2.93/5 = 150.
#define SIZE    20         	// taille des buffers de liaison série
#define MinHaut 900		// correspond à la durée mini 900us pour -90° servomoteur
#define MaxHaut	2100		// correspond à la durée maxi 2100us pour +90° servomoteur
#define QuantServo 33		// variation élémentaire de durée PWM 100µs : 26 positions pour 180°
#define Periode_PWM_servo 20000	// 20ms


unsigned char tempo_LED;

unsigned int heure;					// nombre de déciseconde qui tourne en permanence : max = 6553s > 109mn

char NumConv=0, NumEch=0, FinAcq=0;	// éléments des CAN
unsigned int DurBas, DurHaut;		// entiers pour fixer Niveaux bas et haut en us de PWM servomoteur sonar
char I2Clibre, I2CStop;
unsigned char Code;			// boutons de la télécommande (1 bit parmi les 5 bits faibles) sinon refus			
struct 	{		
	unsigned char OldPA:1;   	// valeur antérieure du signal capteur rotation PA
	unsigned char OldPB:1;		// valeur antérieure du signal capteur rotation PB
	unsigned char Pas:1;		// indique qu'un pas de mouvement sonar est requis
	unsigned char Ecrit:1;		// indique que l'on doit envoyer un octet à l'affichage
	unsigned char ALire:1;		// Du nouveau sur la RS232 à prendre en compte
	unsigned char Tel:1;		// il faut prendre en compte le code télécommande
	unsigned char Sonar:1;		// il est tant de lire le sonar ... et relancer l'émission
	unsigned char ds:1;			// une déciseconde s'est écoulée .. y faut faire ...
  }CAPT;							// variable d'état pour ce test des Entrées				
struct 	{		
	unsigned char SensM:1;   	// indique le sens des deux moteurs chenilles D et \G
	unsigned char Change:1;		// indique qu'il faut changer les sens des moteurs
	unsigned char TempoLeds:2;	// petite tempo leds chenillards change toutes les 0.4s
	unsigned char Sens:1;		// sens du chenillard, des vitesses moteurs, pas servo ....
	unsigned char Changement:1;	// il faut mettre à jour les leds ou les moteurs ....
  }ACT;							// variable d'état pour ce test des Sorties
//        Variables globales de la partie communication série
unsigned char c;				// caractère recu sur RS232
unsigned char buffer[SIZE];		// buffer de reception 
unsigned char *ptlec;			// pointeur de lecture
unsigned char *ptecr;			// pointeur d'écriture 
unsigned char udata;			// caractere recu depuis l'USART
unsigned char buf[10];			// Pour conversion ASCII et affichage resultat console

// Variables globales Echanges I2C_Telecom
unsigned char Send_I2C_Telecom_ASCII_Buf[8] = {0X31,0X00};
unsigned char Recv_I2C_Telecom_ASCII_Buf[8];

//	Diverses variables globales
unsigned char mode = 0;			// mode du test en cours : 0=Rien ou code ascii du mode reçu en RS232
unsigned char AffI2C = 0x20;	// adresse de base (7 bits) pour l'affichage par PCF8574 normal
unsigned char Aff;				// Octet affiché sur les leds (en logique positive)
signed char PasSonar;			// Numéro du pas d'orientation sonar
unsigned char Gisement_D;
unsigned int temp;				// Pour calculs

// constantes chaines des messages pour la liaison série
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


/* Prototypes des fonctions employées */
void init (void);				// initialisation des E/S, timers et variables
void Mesure (void);
void Moyenne (unsigned char * tab, unsigned char val);
// transaction I2C de l'application (lecture de l'octet en entrée
unsigned char Octet_i2c(unsigned char adresse, unsigned char sens, unsigned char valeur);
void ITHaute (void);				// prototype de la routine d'interruption
unsigned char LitOctet (void);		// retourne le premier caractère reçu sur SCI
void EcritOctet (unsigned char c);	// émet c sur l'UART
unsigned char *LitChaineUart (unsigned char *s, unsigned char finst);          
                // lit une chaîne de caractère sur SCI se terminant par finst
                // la variable finst contient le caractère attendu en  fin de chaîne 
int EcritChaine (unsigned char *s);       	// émet la chaîne s en RAM(finit par 0)
int EcritChaineRom (rom unsigned char *s);  // émet la chaîne s en ROM(finit par 0)
void putrsUSART (const rom char *data); 	// Ecrit une chaine en ROM vers l'USART sans le caractère NULL \0
void putsUSART (char *s);                 	// Ecrit une chaine en RAM vers l'USART sans le caractère NULL \0
void EcritValOctet (unsigned char oct);		// Envoie en série la chaine ascii d'un octet
// SONAR_Ecrit et SONAR_Lit sont utilisées pour lire et écrire un octet dans 24CXX EEPROM ou SONAR srf02
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
         		// Séquence initialisation au démarrage des périphériques
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

	while(1)            			// Tâche de fond    
    { 
		// Si on a reçu un octet, on le lit et on l'affiche
        if (CAPT.ALire)                    	// si un caractère reçu ? positionné par IT
        {
        	Valeur = LitOctet();        	// lecture RS232 met à jour les pointeurs
        	putrsUSART (newline);			// on passe à la ligne et
//			EcritOctet(Valeur);            	// écho du caractère reçu
			if (ptecr == ptlec) 			// si on a lu tous les caractères
				CAPT.ALire = NON;			// on l'indique pour ne plus tourner
			// tester le caractère reçu par étude de cas
			if ((Valeur >= 0x30) && (Valeur <= 0x37))	// on a actionné de '0' à '7' ?
				mode = Valeur;				// alors le mode est remis à jour
				switch (Valeur) {			// affichage du mode activé
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
		if (CAPT.ds)				// un élément temporel à prendre en compte
			{	CAPT.ds = NON;			// on indique que l'on a géré
				heure += 1;				// le temps passe ...
				CAPT.Pas = OUI;			// évolution de la motorisation sonar

			/*	ACT.TempoLeds ++;		// le temps passe aussi pour le chenillard
				if (ACT.TempoLeds == 0)
					{ 
					ACT.Changement = 1;
					LED = ~LED;			// Clignotement Led Test toute les O,4s
					}
			*/
			}
		Mesure ();					// il est temps d'acquérir une nouvelle valeur d'une des voies

		// Maintenant, le main() va réaliser les routines de chaque mode.

		// Mode '0' : Chenillard à leds
		if (mode == '0')
		{
			INT0IE = NON;			// autorisation d'IT ? (toujours prioritaire)
			IR_OFF = OUI;			// Arret Capteurs IRD,G
			TMR1ON = NON;			// ARRET la PWM servomoteur sonar
			T2ON = NON;				// ARRET PWM1,2 Moteurs
			if (ACT.Changement)		// les 0.4s sont passées ?
			{	
				ACT.Changement = 0;	// on devra attendre les 0.4s suivantes
				Octet_i2c (AffI2C, ecriture, Aff);
				if (ACT.Sens == 1)
				{	Aff >>= 1;
					if (Aff == 1) 
						ACT.Sens = 0; // on est au bout à droite, on passe à gauche
				}
				else
				{	Aff <<= 1;
					if (Aff == 0x80)
						ACT.Sens = 1; // on est au bout à gauche, on passe à droite
				}
			}
		}

		// Mode '1' : Moteurs : vitesses oposées triangulaires > 0 puis < 0
		if (mode == '1')
		{
			INT0IE = NON;			// autorisation d'IT ? (toujours prioritaire)
			IR_OFF = OUI;			// Arret Capteurs IRD,G
			TMR1ON = NON;			// ARRET la PWM servomoteur sonar
			T2ON = OUI;				// On demarre PWM1,2 Moteurs
			if (ACT.Changement)		// les 0.4s sont passées ?
			{	
				ACT.Changement = 0;	// on devra attendre les 0.4s suivantes
				if (ACT.Sens == 1)	// si on est en phase accélération
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
						ACT.Sens = 1; // on est au mini, on va accélérer
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
			if (ACT.Changement)		// les 0.4s sont passées ?
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
			if (ACT.Changement)		// les 0.4s sont passées ?
			{
				INT0IE = NON;			// autorisation d'IT ? (toujours prioritaire)
				IR_OFF = OUI;			// Arret Capteurs IRD,G
				TMR1ON = NON;			// ARRET la PWM servomoteur sonar
				T2ON = NON;				//	ARRET PWM1,2 Moteurs	
				ACT.Changement = 0;	// on devra attendre les 0.4s suivantes
				// une réponse sonar est-elle disponible ? si oui, on l'envoie sur liaison série
				if (CAPT.Sonar == OUI)			// On vient ici toutes les 100ms
				{
					SONAR_Ecrit (control_SONAR, cmd_SONAR, range_cm);		// demande de distance en centimetres 
					CAPT.Sonar = NON;
					while(!CAPT.Sonar);										// Attente réponse 100ms
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

		// Mode '4' : Commande servo Sonar : Tourne de droite à gauche
		if (mode == '4')
		{
			INT0IE = NON;			// autorisation d'IT ? (toujours prioritaire)
			IR_OFF = OUI;			// Arret Capteurs IRD,G
			TMR1ON = OUI;			// ARRET la PWM servomoteur sonar
			T2ON = NON;				//	ARRET PWM1,2 Moteurs
			if (ACT.Changement)		// les 0.4s sont passées ?
			{	
				ACT.Changement = 0;	// on devra attendre les 0.4s suivantes
				if (CAPT.Pas)		// on modifie les valeurs de DurBas et DurHaut
				{	CAPT.Pas = NON;	// on enlève le drapeau
					if (Gisement_D == OUI) 		// SENS indique le sens d'évolution des durées hautes
					{ 	DurHaut += QuantServo;
						DurBas = Periode_PWM_servo - DurHaut; 

						PasSonar +=5;			// on augmente de 5°
						if (DurHaut > MaxHaut) 	// est-on à +90°
						{	DurHaut = MaxHaut;
							Gisement_D = NON;			// on repasse en diminution
						}
					}
					else 
					{	DurHaut -= QuantServo;
						DurBas = Periode_PWM_servo - DurHaut; 
						PasSonar -=5;			// on diminue de 5°
						if (DurHaut < MinHaut )  	// est-on à -90°
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

		// Mode '5' : Lecture télécommande affichée sur leds et RS232
		if (mode == '5')
		{
			INT0IE = OUI;			// autorisation d'IT ? (toujours prioritaire)
			IR_OFF = OUI;			// Arret Capteurs IRD,G	
			TMR1ON = NON;			// ARRET la PWM servomoteur sonar
			T2ON = NON;				// Arret PWM1,2 Moteurs
			if (CAPT.Tel)	// si la télécommande a agit
			{
				CAPT.Tel = 0;							// on a géré
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

		// Mode '6' : Lecture des capteurs de rotation, affichés sur 2 leds poids forts et RS232
		if (mode == '6')
		{
			INT0IE = NON;			// autorisation d'IT ? (toujours prioritaire)
			IR_OFF = OUI;			// Arret Capteurs IRD,G
			// on lit l'état des capteurs de rotations et on change le poids fort
			if (PA != CAPT.OldPA) // si du nouveau sur le capteur chenille droite
			{	Valeur = (Valeur ^ 0x80) & 0x80; CAPT.OldPA = PA; CAPT.Ecrit = OUI;putrsUSART(chenilleD);	}
			if (PB != CAPT.OldPB) // si du nouveau sur le capteur chenille gauche
			{	Valeur = (Valeur ^ 0x40) & 0x40;	CAPT.OldPB = PB; CAPT.Ecrit = OUI;putrsUSART(chenilleG);	} 
			// finalement, on écrit le code des leds selon les capteurs
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
			if (ACT.Changement)		// les 0.4s sont passées ?
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

	//  Paramétrage des ports, en lots      
	TRISA = 0b00011111; 	// 5 bits bas en entrées : 3 capteurs, batterie, PA
 	TRISB = 0b11011101;		// tout en entrée sauf RB1 (IR_OFF) la led en RB5 (TEST)
	TRISC = 0b10011001;		// tout en sorties sauf RC0 = PB, RC7=Rx et I2C, en entrée
	LED = 1;				// on allume la led dès maintenant
	RBPU = 1;				// portB avec résistances de tirage NLH non employées
	IR_OFF = OUI;				// Arret IRD, IRG

	// INT0 doit interrompre pour acquérir télécommande
	INTEDG0 = 1,			// front Montant actif (allumage de la led)
	INT0IE = NON;			// autorisation d'IT (toujours prioritaire)
  
 	// ADC() Paramétrage du convertisseur ANA_NUM
	PCFG = 0xC;			// Seuls les voies AN0, AN1 et AN2 sont analogiques si pas mesure VITMOT D/G
	VCFG = 0;			// Vref+ = VDD; Vref- = Vss
    CHS = 0;			// on commence avec AN0
    ADCS = 6;			// fréq. conv. bit = Fosc / 64 TAD = 8,2µs
    ADFM = 0;			// formaté à gauche, pour n'exploiter que l'octet de poids fort 
	ACQT = 7;			// delai de lecture CAN de 20 x TAD = 2,56µS > 2,4µS
    ADON = 1;			// activation du convertisseur

   	// Timer0() pour IT active par débordement toutes les 128µs environ
  	TMR0H = 0;          	// raz timer
  	TMR0L = 0;          	// raz timer
	T08BIT = 1;				// mode 8 bits actif
  	T0CS = 0;       		// horloge timer 0= 32MHz/4 T(T0) 8MHz = 0,125µs 
  	PSA = 0;        		// pré-division interne active  
	T0PS = 1;				// prédivision par 4 : débordement et ITTMR0 toutes les 128µs
 	TMR0IE = 1;				// valide IT sur TMR0
//  TMR0IP = 1;     		// TMR0 en haute priorite (adresse 8)
//	RCONbits.IPEN = 1;  	// validation des niveaux de priorité des IT
	TMR0ON = 1;					// Lancement du Timer0
	// pour battre au rythme de 1ms, il faudra diviser par 8 !

	// Timer1() Sert au mouvements sonar : PWM de 50Hz 5 à 10 % et stepper (20ms plus tard 100ms)
	// on doit donc détecter la retenue de Timer 1 avec rechargement DurBas DurHaut
	RD16 = 1;				// On manipule TMR1 en 16 bits
	TMR1CS = 0;				// Horl interne F = 32/4 = 8MHz
	T1SYNC = 1;				// pas de synchronisation
	T1CKPS = 3;				// prédivision par 8 pour compter la µs 
	TMR1H = 0xFA;			// Mat sonar Gisement 0°
	TMR1L = 0x24;
	TMR1IE = OUI;			// autorisation d'IT en fin de periode du duty cycle PWM Sonar
//	TMR1IP = OUI;			// IT prioritaire
	TMR1ON = 0;				// Arret PWM servomoteur sonar
	
	// Timer2() 32MHz/4 / 4 / 256. Période PWM avec les valeurs suivantes on est à 2kHz
	PR2 = 0xFF;		// à ce stade le Timer est OFF on divise par 256 périodes du PWM1 et PWM2 
	T2CKPS = 2;   	// Préscaler division par 16 et ici TMR2 off 
	T2OUTPS = 0;	// pas de post division une IT à chaque 0.5ms TMR2IF
//	TMR2IP = OUI;	// IT prioritaire

	// PWM DROIT et GAUCHE
	CCP1M = 0x0C;  	// Mode PWM1 
	DC1B = 0;		// Deux Bits poids faibles du DUTY_CYCLE
	PWMD = 0;		// Envoi commande PWMD   
	CCP2M = 0x0C;   // Mode PWM2 
	DC2B = 0;		// Deux Bits poids faibles du DUTY_CYCLE
	PWMG = 0;		// Envoi commande PWMG   
	   
	T2ON = 0;     	// Standby PWM1 et PWM2

	// initialisation de la partie Liaison série asynchrone
	TXSTA = 0b00100100; 	// 8 bits, valid émission, asynchrone, pas de sync, BRGH=1 High speed                          
	RCSTA = 0b10010000;			// active l'USART, 8bits,  CREN=1 et SPEN=1
	BAUDCON = 0;                // brg16 = 0 pour baud rate sur 8 bits
	SPBRG = 207; 				// baud rate : 32M/(16(n+1)) donc pour 9600 bauds = 207
	udata = 0;               	// on n'a rien reçu

	// initialisation de la partie liaison série en interruption
	ptecr = buffer;				// le buffer de réception est défini au début du tampon
	ptlec = ptecr;				// efface le buffer de reception en pointant lecture idem
	TRISCbits.TRISC7 = 1;		// RX - broche en entrée
	TRISCbits.TRISC6 = 0;		// TX - broche en sortie
	PIR1bits.TXIF=0;			// IT en emission désactivée
	PIE1bits.RCIE=1;			// IT en reception activée
//	RCONbits.IPEN=1;			// active la priorité sur les IT
//	IPR1bits.RCIP=1;			// selectionne haute priorité pour RX (vecteur en 0x8)

	// initialisation de la partie I2C
	TRISCbits.TRISC3 = 1;       // SCL (PORTC,3) en entrée
 	TRISCbits.TRISC4 = 1;       // SDA (PORTC,4) en entrée
 	SSPCON1=0b00101000;			// pose WCOL SSPOV SSPEN CKP SSPM3:SSPM0 
 	// efface WCOL et SSPOV, active I2C, I2C mode maitre horloge=FOSC/(4*(SSPADD+1))	 	SMP = 1;					// slew rate inhibé pour vitesses normales(f<400Khz)
 	SSPADD = 127;				// horloge = 32Mhz / 4*(127+1)  = 62,5 KHz

	PEIE = 1;     	// Validation globale des interruptions
  	INTCONbits.GIEH = 1;

	// initialisation des variables
	Tick = 0;Tac=0;	Valeur = 0;  CAPT.ds = NON;	// Valeur peut être un octet ou un entier
	STEP = 0;	DirD = 0;	DirG = 0;	CAPT.Tel = NON;
	DurHaut = 1500;	// on commence au milieu
	DurBas = Periode_PWM_servo - DurHaut; 
	CAPT.Sonar = NON;	Aff = 1; ACT.Sens = 0;
	Gisement_D = OUI;
	PWMD = PWMG = 0;	DirD = 0; DirG = 1;	// on commence à l'arrêt avec les sens initiaux
	PasSonar = 0; // on commence l'orientation sonar à 0 pour "droit devant"
}

void Mesure (void)
{	// on acquière ici toute la campagne de conversion de mesures par passage successif (12 passages) 
	unsigned char Val;
	if (!DONE)			// on passe alors au canal suivant sauf si campagne finie --> AD OFF
	{
		Val = ADRESH;    	// lecture de l'octet converti
		switch (NumConv) 	// selon NumConv, on range dans le tableau adéquat le résultat
		{
			case 0 : {  Moyenne(MesDistD, Val); 	break;}		// AN0 : CaptD
			case 1 : {  Moyenne(MesDistG, Val); 	break;}    	// AN1 : CaptG
			case 2 : {  Moyenne(MesBatt, Val); 		break;}  	// AN2 : Vbat
		}
        NumConv++;
		if (NumConv > 2)      // si toutes voies converties, 
		{
            NumConv = 0;       // on revient à la première voie
			NumEch++;			// échantillon nouveau de la nouvelle voie
			if (NumEch > 3)
			{
				NumEch = 0;
				FinAcq = OUI;	// on a eu les 12 mesures utile à la navigation
			}
		}
		CHS = NumConv;  	// on sélectionne cette voie CAN
		GO = 1;					// lancement conversion de la nouvelle voie choisie
	}
}

void Moyenne (unsigned char * tab, unsigned char val)
// la neuvième valeur du tableau passé en paramètre est affecté de la valeur
// moyenne des 7 valeurs antérieures. le second paramètre, valeur récente 
// est rangée en première case. Les cases sont déplacées à gauche durant le calcul
{
	char i;
	unsigned int total = 0;
    
	for (i=1; i<4; i++)
	{
        	total += tab[i];
		tab[i-1] = tab[i];
 	}
   	tab[4] = (total + val) >> 2; 	// moyenne en dernière case
	tab[3] = val;   				// valeur ajoutée en avant dernière case
}

unsigned char Octet_i2c(unsigned char adresse, unsigned char sens, unsigned char valeur) 
// dialogue avec le destinataire défini par adresse
// si SENS = 1 : on lit à l'adresse et on rend la donnée en valeur
// si SENS = 0 : on écrit l'octet valeur obtenu mais complémenté car les Leds sont tirées au NLH
// On travaille en maitre avec un diviseur d'horloge pour un bit de excatement 10µs soit F = 40kHz
{
	unsigned char t;			// tampon intermédiaire pour octet reçu et pour compte-rendu
	if (sens == lecture)
	{
		while((SSPCON2 & 0x1F) || R_W);  // Attente module idle
 		SEN = 1;			// START
 		while (SEN);		// Attente de fin de condition start (RAZ automatique)
 		SSPBUF = (adresse << 1) + sens; 	// Ecriture de l'adresse avec le bit Lecture 
		while (BF);			// on attend la fin d'écriture de l'adresse
		while (ACKSTAT);		// on attend que l'acquittement ait eu le temps d'être mis

		while( SSPCON2 & 0x1F | R_W)
 		RCEN = 1;			// On passe en mode réception
		while (!BF);		// On attend la fin de réception de l'octet mis par le destinataire
		t = SSPBUF;			// Lecture de l'octet reçu
 		ACKDT = 1;			// pas d'acquittement car on arrête là : NON-ACK
 		ACKEN = 1; 			// lancement du cycle acquittement
 		while (ACKEN);		// dont on attend la fin

 		PEN = 1;			// lancement de la condition de STOP
 		while (PEN);		// On attend la fin de temps de stop
		SSPIF = 0;			// effacement du drapeau qui indiquera la fin de stop
 		return (t); 		// t étant l'octet reçu, on le retourne
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
		SSPBUF = ~valeur;	// on écrit l'octet à émettre en complémenté et c'est parti
		while(BF);			// Attendre buffer vide
		while (ACKSTAT);	// on attend  ACK

		while((SSPCON2 & 0x1F) || R_W);  // Attente module idle
 		PEN = 1;				// STOP
 		while (PEN);			// attente fin du STOP
		SSPIF =0;				//Remise à jour du flag pour IT Stop  
	}
}

//----------------------------------------------------------------------------
// Pose du vecteur d'interruption haute priorité
#pragma code InterruptVectorHigh = 0x08
void InterruptVectorHigh (void)
{
  	_asm
  		goto ITHaute 			// on y met le saut à la routine IT
  	_endasm
}
//----------------------------------------------------------------------------
// Routine interruption haute priorité

#pragma code
#pragma interrupt ITHaute
void ITHaute ()					
{	
			
	if (TMR0IF)		// L'IT débordement survient environ toutes les 128µs !
  	{    
	                      
    	TMR0IF = 0; 	// si oui, on efface la cause
		Tick++;
		if (Tick == 8) 	// passage à 1ms, période de la PWM servomoteur
		{	Tac ++;	Tick = 0;			}
		if (Tac == 100)			// détection des 0.1s de changement de leds
		{ //	LED=~LED;
			tempo_LED--;
			if(	tempo_LED==0){	tempo_LED=4;LED=~LED;	ACT.Changement = 1; }
	
			Tac = 0; CAPT.ds = 1; CAPT.Sonar = OUI;}	// ds = déci seconde
  	}

	if (TMR2IF)					// est-ce la PWM qui interrompt à 2kHz ?
		{ 	
		TMR2IF = 0;				// on efface la demande
		// doit mettre à jour la valeur du rapport cyclique selon CaptG
		}
	if (TMR1IF)		// après niveau haut ou bas, il faut reprogrammer la PWM servomoteur
	{
		TMR1ON = 0;
		TMR1IF = 0;				// on efface la demande
		if (STEP) 				// si NLB on charge la durée haute
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
		c = RCREG;						// on stocke l'octet reçu
		*ptecr++ = c;					// dans le tableau, et on pointe la suite
		if (ptecr >= &buffer[SIZE])		// si on est sorti de l'espace de stockage
			ptecr = buffer;				// on revient (pile tournante)
		CAPT.ALire = OUI;				// on indique qu'il faut lire
	}        
	if (INT0IF)				// on a reçu un ordre de la télécommande
	{
		INT0IF = NON;					// élimination de la demande
		CAPT.Tel = OUI;
	}
}
//----------------------------------------------------------------------------

/****************************************************************************
*            Ensemble de routines perso pour gérer la liaison série     	*
*  d'un PIC18F2520 en mode 9600 baud sans Par, 8 bits, 1 stop        		*
*****************************************************************************/
//        Reception d'un caractère. boucle en attendant la réception.
unsigned char LitOctet (void)
{
	unsigned char c;                                
	while (ptlec == ptecr);     	// attente que les pointeurs diffèrent
	c = *ptlec++;               	// on prend le caractère suivant
	if (ptlec >= &buffer[SIZE])   	// si on est sortie de la pile, 
		ptlec = buffer;            	// on revient au début de la pile tournante
	return (c);                  	// on retourne l'octet nouvellement reçu
}

/*/ Envoie sur RS232 la chaine ascii d'un octet
void EcritValOctet (int oct)
{
	EcritOctet (0x30 + (oct div 100));			// chiffre des centaines en ascii
	EcritOctet (0x30 + ((oct mod 100) div 10));	// chiffre des dizaines en ascii
	EcritOctet (0x30 + (oct mod 10));			// chiffre des unités en ascii
}

/* Cette fonction retoure 1 (vrai) si un caractère est disponible dans le buffer de réception
et 0 si non, si les deux pointeurs sont identiques, il n'y a rien dans le buffer*/
char carUSARTdispo(void)
{
	return (ptecr - ptlec);
}

// emission d'un caractère sur RS232
void EcritOctet (unsigned char c)
{
	while (!TXSTAbits.TRMT);        // on attend que la transmission précédente soit finie
	TXREG = c;                      // envoi d'un caractère 
	while (!PIR1bits.TXIF);         // on attend que TXIF passe à 1
}

// Retourne une chaine de caractère
unsigned char *LitChaineUart (unsigned char *s, unsigned char finst)
{
	unsigned char *r = s;              // tampon mémoire pointant la chaine à pointer
	char c;                                                        
	while ((c = LitOctet()) != finst)  // c est l'octet su sur l'UART jusqu'à être le délimiteur
		*s++ = c;                  		// de fin de chaine, on range à la suite les octets
	*s = 0;                            // on ajoute un caractère nul pour clore la chaine
	return (r);                        // on retourne l'adresse de la chaine crée
}

/* envoie une chaine de caractère en ROM*/
int EcritChaineRom(rom unsigned char *s)
{
	int n;                          // compteur d'octet à émettre
	unsigned char a;
	for (n = 0; *s; n++)            // on lit la mémoire jusqu'à trouver 0
	{
		a = *s;
		EcritOctet(a);           	// on émet l'octet trouvé
		s++;
	}        
	return 1;
}

/* envoie une chaine de caractère en RAM*/
int EcritChaine (unsigned char *s)
{
	int n;
	for (n = 0; *s; n++, s++) 
		EcritOctet (*s);
	return 1;
}

// putrsUSART()  Ecrit une chaine en ROM vers l'USART sans le caractère NULL \0
void putrsUSART(const rom char *data)
{
	do {                                
		while ( ! (TXSTA & 0x02) );  // on attend que l'uart soit libre
		TXREG = *data;
	} while ( *++data );             // on arrête si le prochain caractère pointé est nul
}

// putsUSART() Ecrit une chaine en RAM vers l'USART sans le caractère NULL \0
void putsUSART(char *s)
{
	do {                                
		while ( ! (TXSTA & 0x02) );     // on attend que l'uart soit libre
		TXREG = *s;
	} while ( *++s );                   // on arrête si le prochain caractère pointé est nul
}
//---------------------------------------------------------------------
// SONAR_Ecrit()
// Ecrit un octet de commande dans le sonar connecté à l'I2C
// à l'adresse indiquée 0x00.
//---------------------------------------------------------------------

void SONAR_Ecrit (char controle, unsigned char adresse, char data)
{
	// variables locales
	char erreur, essai;

	erreur = 0;
	essai = 10;						// On essaiera 10 fois d'accéder au SONAR
	do
	{
		erreur = EEAckPolling (controle);
		essai--;
	} 
	while (erreur && essai > 0); 
	essai = 10;						// Réessai 10 fois MAX
	do								// Tentative d'écriture de l'octet faible de donnée
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
	essai = 10;						// On essaiera 10 fois d'accéder au SONAR
	do
	{
		erreur = EEAckPolling (controle);
		essai--;
	} 
	while (erreur && essai > 0); 
	essai = 10;						// Réessai 10 fois MAX
	do								// Tentative de lecture de l'octet fort de donnée
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
	essai = 10;							// On essaiera 10 fois d'accéder au SONAR
	do
	{
		erreur = EEAckPolling (controle);
		essai--;
	} 
	while (erreur && essai > 0); 
	essai = 10;							// Réessai 10 fois MAX
	do									// Tentative de lecture de l'octet fort de donnée
	{
		temp.i = EERandomRead (controle, adresse);
		essai--;
	} 
	while (temp.b[1] && essai > 0);
	if (temp.b[1]) data.b[0] = 0xFF;	// Mise du résultat de lecture à 0 si erreur
	else data.b[1] = temp.b[0];			// Sinon, mise de l'octet fort de donnée lue
	essai = 10;							// Réessai 10 fois MAX
	do									// Tentative de lecture de l'octet faible de donnée
	{
		temp.i = EERandomRead (controle, adresse + 1);
		essai--;
	} 
	while (temp.b[1] && essai > 0);
	if (temp.b[1]) data.b[1] = 0xFF;	// Mise du résultat de lecture à 0 si erreur
	else data.b[0] = temp.b[0];			// Sinon, mise de l'octet faible de donnée lue
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
		while(RCEN);	// Attente fin RCEN ms à 0 par Hardware
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


