  /**********************************************************************************************
  *                               Module ICP ROBOTIQUE  -  ISMIN 1A                             *
  *                                                                                            	*
  *   		INTRODUCTION EN-TETE du programme ROBOT version P2016								*    
  * Micro controleur : PIC 18F2520 cadenc� en RC interne � 32MHz                     	 		*
  * Language C                                                                           		*
  * Auteur initial J.P. BOURGUET Modifi� P. GENTRIC 11/2013                                           Date Novembre 2012  	*
  * Compilateur CC18 v.3.2 et mise au point avec MPLAB v8.84               		           		*
  * Prototypage sous ISIS VSM V7.10 															*
  * La simulation se fait sous MPLAB avec le fichier TEST-P2015-SIM.DSN   						*
  ***********************************************************************************************/
// Bits de configuration
#pragma config OSC = INTIO67 	// horloge interne , RA6-RA7 en E/S
#pragma config IESO = OFF // Oscillator Switchover mode disabled 
#pragma config FCMEN = OFF // Fail-Safe Clock Monitor disabled 
//#pragma config PWRT = ON		// d�lai de 65ms apr�s mont�e alim avant lib�ration reset logiciel
#pragma config WDT = OFF		// pas de chien de garde
#pragma config LVP = OFF
#pragma config CPD = OFF		// EEPROM interne Non prot�g�e

// D�claration des constantes d'acquisition analogique 
#define MINCAPT		3     		// seuil d�tection �cho capteur IR employ� pour d�tecter maitre
#define MAXCAPT 	153			// seuil de proximit� de 5cm environ 3V soit 3*255/5 = 153
#define MINBAT 		100      	// seuil minima de tension batterie admissible 10V
#define VitMax 		128			// PWM maximale = 50% pour ne pas d�passer les 6V
// Constantes directionnelles
#define av			1	  		// pour positionner DIRx vers l'avant
#define ar			0	  		// pour positionner DIRx vers l'arriere
// D�claration des constantes globales
#define VRAI 		1
#define OUI 		1
#define FAUX		0
#define NON			0
/* D�claration des constantes de cablage  */
#define SERVO		RC5			// signal pilotage ServoMoteur = STEP
#define LED			RB5			// cablage de led t�moin d'activit� 
#define DirD    	RA6     	// cablage des sens de rotation moteur 1 = vers l'Avant
#define DirG    	RA7
#define PWMD		CCPR2L		// valeur des PWM des moteurs CCP2 est reli� au moteur D
#define PWMG		CCPR1L		// valeur des PWM des moteurs CCP1 est reli� au moteur G
#define adrMEM		0x58		// adresse EEPROM 24LC64 ou 24LC256 avant d�calage R/W
#define ITTel		RB0			// entr�e signal IT de r�ception d'une t�l�commande
#define PA			RA4			// entr�e du capteur de rotation du moteur droit
#define PB			RC0			// entr�e capteur de rotation du moteur gauche
#define CaptA		RA3			// entr�e de capteur logique d'�cho ultra son � l'arri�re
#define IR_OFF		RB1			// Pilotage 5V alim. M/A des deux capteurs IRD, IRG actif � 0
//#define SENS		RA5			// futur signal de s�lection de sens de rotation du mat du sonar
#define STEP		RC5			// signal de commande PWM du moteur mat sonar
#define regtemp 	0			// variable temporaire pour I2C
#define lecture 	1			// modes employ�s pour les sens de transaction I2C
#define ecriture 	0
//	Constantes du sonar SRF02
#define control_SONAR  	0xE0	// Adresse I2C srf02us
#define cmd_SONAR      	0x00	// Adresse registre de commande
#define reset_SONAR	   	0x60	// Reinitialisation SONAR
#define ping_SONAR     	0x5C	// Emission d'un ping 8 cycles	 
#define range_cm       	0x51	// Le resultat demand� en centimetres
#define range_us       	0x52	// Le resultat demand� en microsecondes
#define dist_cible     	0x02	// Non signe 0x02=Poids fort, 0x03=Poids faible
//	Constante Rec_Telecom
#define Rec_Telecom 0xA2		//Adresse physique I2C  control Reception A2 = Write, A3 = Read
// D�claration des noms de moteurs pour les commander
#define MotD		1
#define MotG		2
#define PWMmin		15			// dur�e mini de 1ms avec ITTmr0 de 32us
// d�claration des valeurs bits associ�es aux positions des touches de t�l�commande
#define Haut		0b0010		// nouveau code de la touche du haut
#define Bas			0b0100		// nouveau code de la touche du bas
#define Gauche		0b0101		// nouveau code de la touche de gauche
#define Droite		0b0001		// nouveau code de la touche de droite
#define Centre		0b0011		// nouveau code de la touche centrale
// d�finition des �tats du syst�mes pour commander la LED (variable UsageLed)
#define rien		0			// rien encore : led �teinte
#define maitre		1			// un maitre � suivre : allum�e
#define cligno		2			// on est en t�l�commande
#define flash		3			// batterie trop faible
// D�claration des diff�rents modes de navigation retenus
#define Depart  	0      	// au d�part, on va tourner en rond pour trouver son maitre
#define Maitre  	1		// � 1 si il a trouv� son "ma�tre", on allume led, on fonce dessus
#define TPres   	2		// TPres pour dire trop pr�s, on recule du cot� du plus grand signal
#define Recul   	3		// Je dois reculer car je suis trop pr�s, dure 1s et repasse en Init
#define Telecom		4		// on passe en pilotage par radio via I2C
#define arret		5		// �limination de la PWM pendant le basculement de sens d'un moteur

// registre d'�tat pour les �volutions de ce programme
// on utilisera des switch pour la commande de led et l'analyse navigation   
static union {				
  struct 	{		
	unsigned char TimeOut:1;   	// Indique d�bordement TMR0, il est temps d'agir
								// la tache de fond lancera alors les acquisitions capteurs
								// et le calcul de navigation
	unsigned char UsageLed:2;	// 2 bits pour 4 �tats de la led : �teinte pas de maitre,
								// allum� : maitre, cligno : t�l�comande, flash : bat faible
	unsigned char DefAlim:1;	// indique que l'alim n'est pas assez forte
	unsigned char Mode:3;		// Mode de navigation init, maitre, troppres, telecom et arret
	unsigned char ITrepos:1;	// valeur du signal RB0 � la mise sous tension
	unsigned char NumConv:2;	// num�ro de la voie CAN convertie : 0:CaptD; 1:CaptG; 2:Bat
	unsigned char NumEch:3;		// num�ro d'�chantillon de mesure d'une voie de CAN
	unsigned char FinAcq:1;		// bool�en indiquant qu'une camplagne de mesure est termin�e
	unsigned char SonAcq:1;		// bool�en mis � un lors d'une demande de sonar, RAZ apr�s 80ms
	unsigned char :1;			// non utilis�s encore
  };
} ETAT;							// variable d'�tat constitu� pour l'occasion : 2 octets

/* les variables globales du Programme */
unsigned char Vbat;				// variable pour stocker la tension batterie mesur�e
unsigned char Valeur;  			// pour tout calcul interm�diaire et d'autres choses
unsigned int Tick, Tac;			// comptage du nb d'IT TMR0 et les 20ms pour les dur�es 
unsigned int Duree;				// variable temporelle pour recevoir un chronom�tre dans un mode
unsigned char LuI2C;			// variable octet de donn�es lue sur le portI2C
unsigned char OldCde;			// valeur de la pr�c�dente commande touches t�l�commande
unsigned int i, j;
unsigned char VitD, VitG;			// Valeur absolue des consignes de vitesses sur PWMD et PWMG
unsigned char DurPWM;			// valeur du nombre de tick de 128us

/* zones de stockages des donn�es lues sur les CAN : 3 tampons de 4 octets pour moyenner 
   	les distances et charge batterie, et limiter l'effet du bruit. 
	La derni�re case ( [4] ) contiendra la moyenne calcul�e au fil du chargement */
unsigned char MesDistD [5] 	= {0,0,0,0,0};
unsigned char MesDistG [5]	= {0,0,0,0,0};
unsigned char MesBatt [5] 	= {200,200,200,200,0};
// zone de donn�es employ�es pour le sonar
unsigned int distance_sonar;	// distance fournie par le sonar sur deux octets
// tableau des �chos sonar de distances inf�rieures � 2m. pour trouver un minima dans la rotation
unsigned char VueSonar [16] = {200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200};	
