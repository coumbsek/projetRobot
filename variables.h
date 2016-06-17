#include <stdio.h>
#include <stdlib.h>

#define SeuilBat 169
#define v1 50
#define v2 100

typedef struct{
    //avantArriere = 1 => avant
    //avantArriere = 2 => arrière
    //avantArriere = 0 => point mort
    unsigned  avantArriere:2;
    unsigned  vitesse:2;
    //gaucheDroite = 1 => droite
    //gaucheDroite = 2 => gauche
    //gaucheDroite = 0 => ligne droite
    unsigned  gaucheDroite:2;
    unsigned  angle:2;
}etat;

typedef struct{
    etat actuel;
    etat precedent;
}datas;

/* Variables relatives aux interruptions
 *//**/
unsigned char flag_telecom;//*/
unsigned char flag_100ms;//*/
unsigned char flag_2s;//*/
unsigned char c100ms;
unsigned char c10msTel;//*/
unsigned char c2s;
unsigned char flag_IsOn;

/* Variables relatives a la gestion de la batterie
 */
unsigned int mi[] = {0,0,0,0};//[4];
unsigned char mesure;
unsigned char Vbat;

char lectureTelecom[3];
datas donnees;

