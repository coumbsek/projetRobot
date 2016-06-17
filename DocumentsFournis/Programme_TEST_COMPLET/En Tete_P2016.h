  /**********************************************************************************************
  *                               Module ICP ROBOTIQUE  -  ISMIN 1A                             *
  *                                                                                            	*
  *   		INTRODUCTION EN-TETE du programme ROBOT version P2016								*    
  * Micro controleur : PIC 18F2520 cadencé en RC interne à 32MHz                     	 		*
  * Language C                                                                           		*
  * Auteur initial J.P. BOURGUET Modifié P. GENTRIC 11/2013                                           Date Novembre 2012  	*
  * Compilateur CC18 v.3.2 et mise au point avec MPLAB v8.84               		           		*
  * Prototypage sous ISIS VSM V7.10 															*
  * La simulation se fait sous MPLAB avec le fichier TEST-P2015-SIM.DSN   						*
  ***********************************************************************************************/
// Bits de configuration
#pragma config OSC = INTIO67 	// horloge interne , RA6-RA7 en E/S
#pragma config IESO = OFF // Oscillator Switchover mode disabled 
#pragma config FCMEN = OFF // Fail-Safe Clock Monitor disabled 
//#pragma config PWRT = ON		// délai de 65ms après montée alim avant libération reset logiciel
#pragma config WDT = OFF		// pas de chien de garde
#pragma config LVP = OFF
#pragma config CPD = OFF		// EEPROM interne Non protégée

// Déclaration des constantes d'acquisition analogique 
#define MINCAPT		3     		// seuil détection écho capteur IR employé pour détecter maitre
#define MAXCAPT 	153			// seuil de proximité de 5cm environ 3V soit 3*255/5 = 153
#define MINBAT 		100      	// seuil minima de tension batterie admissible 10V
#define VitMax 		128			// PWM maximale = 50% pour ne pas dépasser les 6V
// Constantes directionnelles
#define av			1	  		// pour positionner DIRx vers l'avant
#define ar			0	  		// pour positionner DIRx vers l'arriere
// Déclaration des constantes globales
#define VRAI 		1
#define OUI 		1
#define FAUX		0
#define NON			0
/* Déclaration des constantes de cablage  */
#define SERVO		RC5			// signal pilotage ServoMoteur = STEP
#define LED			RB5			// cablage de led témoin d'activité 
#define DirD    	RA6     	// cablage des sens de rotation moteur 1 = vers l'Avant
#define DirG    	RA7
#define PWMD		CCPR2L		// valeur des PWM des moteurs CCP2 est relié au moteur D
#define PWMG		CCPR1L		// valeur des PWM des moteurs CCP1 est relié au moteur G
#define adrMEM		0x58		// adresse EEPROM 24LC64 ou 24LC256 avant décalage R/W
#define ITTel		RB0			// entrée signal IT de réception d'une télécommande
#define PA			RA4			// entrée du capteur de rotation du moteur droit
#define PB			RC0			// entrée capteur de rotation du moteur gauche
#define CaptA		RA3			// entrée de capteur logique d'écho ultra son à l'arrière
#define IR_OFF		RB1			// Pilotage 5V alim. M/A des deux capteurs IRD, IRG actif à 0
//#define SENS		RA5			// futur signal de sélection de sens de rotation du mat du sonar
#define STEP		RC5			// signal de commande PWM du moteur mat sonar
#define regtemp 	0			// variable temporaire pour I2C
#define lecture 	1			// modes employés pour les sens de transaction I2C
#define ecriture 	0
//	Constantes du sonar SRF02
#define control_SONAR  	0xE0	// Adresse I2C srf02us
#define cmd_SONAR      	0x00	// Adresse registre de commande
#define reset_SONAR	   	0x60	// Reinitialisation SONAR
#define ping_SONAR     	0x5C	// Emission d'un ping 8 cycles	 
#define range_cm       	0x51	// Le resultat demandé en centimetres
#define range_us       	0x52	// Le resultat demandé en microsecondes
#define dist_cible     	0x02	// Non signe 0x02=Poids fort, 0x03=Poids faible
//	Constante Rec_Telecom
#define Rec_Telecom 0xA2		//Adresse physique I2C  control Reception A2 = Write, A3 = Read
// Déclaration des noms de moteurs pour les commander
#define MotD		1
#define MotG		2
#define PWMmin		15			// durée mini de 1ms avec ITTmr0 de 32us
// déclaration des valeurs bits associées aux positions des touches de télécommande
#define Haut		0b0010		// nouveau code de la touche du haut
#define Bas			0b0100		// nouveau code de la touche du bas
#define Gauche		0b0101		// nouveau code de la touche de gauche
#define Droite		0b0001		// nouveau code de la touche de droite
#define Centre		0b0011		// nouveau code de la touche centrale
// définition des états du systèmes pour commander la LED (variable UsageLed)
#define rien		0			// rien encore : led éteinte
#define maitre		1			// un maitre à suivre : allumée
#define cligno		2			// on est en télécommande
#define flash		3			// batterie trop faible
// Déclaration des différents modes de navigation retenus
#define Depart  	0      	// au départ, on va tourner en rond pour trouver son maitre
#define Maitre  	1		// à 1 si il a trouvé son "maître", on allume led, on fonce dessus
#define TPres   	2		// TPres pour dire trop près, on recule du coté du plus grand signal
#define Recul   	3		// Je dois reculer car je suis trop près, dure 1s et repasse en Init
#define Telecom		4		// on passe en pilotage par radio via I2C
#define arret		5		// élimination de la PWM pendant le basculement de sens d'un moteur

// registre d'état pour les évolutions de ce programme
// on utilisera des switch pour la commande de led et l'analyse navigation   
static union {				
  struct 	{		
	unsigned char TimeOut:1;   	// Indique débordement TMR0, il est temps d'agir
								// la tache de fond lancera alors les acquisitions capteurs
								// et le calcul de navigation
	unsigned char UsageLed:2;	// 2 bits pour 4 états de la led : éteinte pas de maitre,
								// allumé : maitre, cligno : télécomande, flash : bat faible
	unsigned char DefAlim:1;	// indique que l'alim n'est pas assez forte
	unsigned char Mode:3;		// Mode de navigation init, maitre, troppres, telecom et arret
	unsigned char ITrepos:1;	// valeur du signal RB0 à la mise sous tension
	unsigned char NumConv:2;	// numéro de la voie CAN convertie : 0:CaptD; 1:CaptG; 2:Bat
	unsigned char NumEch:3;		// numéro d'échantillon de mesure d'une voie de CAN
	unsigned char FinAcq:1;		// booléen indiquant qu'une camplagne de mesure est terminée
	unsigned char SonAcq:1;		// booléen mis à un lors d'une demande de sonar, RAZ après 80ms
	unsigned char :1;			// non utilisés encore
  };
} ETAT;							// variable d'état constitué pour l'occasion : 2 octets

/* les variables globales du Programme */
unsigned char Vbat;				// variable pour stocker la tension batterie mesurée
unsigned char Valeur;  			// pour tout calcul intermédiaire et d'autres choses
unsigned int Tick, Tac;			// comptage du nb d'IT TMR0 et les 20ms pour les durées 
unsigned int Duree;				// variable temporelle pour recevoir un chronomètre dans un mode
unsigned char LuI2C;			// variable octet de données lue sur le portI2C
unsigned char OldCde;			// valeur de la précédente commande touches télécommande
unsigned int i, j;
unsigned char VitD, VitG;			// Valeur absolue des consignes de vitesses sur PWMD et PWMG
unsigned char DurPWM;			// valeur du nombre de tick de 128us

/* zones de stockages des données lues sur les CAN : 3 tampons de 4 octets pour moyenner 
   	les distances et charge batterie, et limiter l'effet du bruit. 
	La dernière case ( [4] ) contiendra la moyenne calculée au fil du chargement */
unsigned char MesDistD [5] 	= {0,0,0,0,0};
unsigned char MesDistG [5]	= {0,0,0,0,0};
unsigned char MesBatt [5] 	= {200,200,200,200,0};
// zone de données employées pour le sonar
unsigned int distance_sonar;	// distance fournie par le sonar sur deux octets
// tableau des échos sonar de distances inférieures à 2m. pour trouver un minima dans la rotation
unsigned char VueSonar [16] = {200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200};	
