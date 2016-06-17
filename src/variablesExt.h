#include <stdio.h>
#include <stdlib.h>

#define SeuilBat 169

typedef struct{
    unsigned  avantArriere:2;
    unsigned  vitesse:2;
    unsigned  gaucheDroite:2;
    unsigned  angle:2;
}etat;

typedef struct{
    etat actuel;
    etat precedent;
}datas;

/* Variables relatives aux interruptions
 *//**/
extern unsigned char flag_telecom;//*/
extern unsigned char flag_100ms;//*/
extern unsigned char flag_2s;//*/
extern unsigned char c100ms;
extern unsigned char c10msTel;//*/
extern unsigned char c2s;
extern unsigned char flag_IsOn;

/* Variables relatives a la gestion de la batterie
 */
extern unsigned char mi[4];
extern unsigned char mesure;
extern unsigned char Vbat;

extern char lectureTelecom;
extern datas donnees;

