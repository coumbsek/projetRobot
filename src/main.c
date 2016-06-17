#include "initASM.h"
#include "variables.h"
#include "MI2C.h"
#include "init.h"
#include "main.h"

#pragma interrupt HighISR
void HighISR(void)
{
    if(PIR1bits.TMR2IF){//si timer interrupt
        PIR1bits.TMR2IF = 0;
        c10msTel++;
        if(c100ms++ > 10) //Incrémente le timer 100ms et vérifie que moins de 100ms se sont écoumées
            flag_100ms = 1;
        if(c2s++ > 200) //Incrémente le timer 2s et vérifie que moins de 2s se sont écoumées
            flag_2s = 1;
    }
    if (INTCONbits.INT0IF){//Si interruption telecommande on positionne le flag telecom a 1 pour prévenir le main qu'il faut actualiser les états.
        INTCONbits.INT0IF = 0;
        flag_telecom = 1;
    }
}
/* Fonction servant à convertir un champs état de la structure datas en char afin de l'afficher sur les D.E.L. du robot par la suite*/
char datasToChart(datas d){
    char c;
    c = 0;
    c |= d.actuel.vitesse;
    c |= d.actuel.avantArriere <<2;
    c |= d.actuel.gaucheDroite <<4;
    c |= d.actuel.angle<<6;
    return c;
}

/* Fonction servant a translater les états, l'état actuel devient le précédent.*/
void saveState(datas *etat){
    etat->precedent.angle           = etat->actuel.angle;
    etat->precedent.avantArriere    = etat->actuel.avantArriere;
    etat->precedent.gaucheDroite    = etat->actuel.gaucheDroite;
    etat->precedent.vitesse         = etat->actuel.vitesse;
}

/* Fonction servant à vérifier que le robot n'est pas en batterie faible. 
S'il atteint le seuil de batterie faible on change l'état du témoins correspondant*/
void gestionBatt(){
    c2s = 0; // reset compteur
    flag_2s = 0; //reset flag
    ADCON0bits.GO = 1; // début conv
    while(!ADCON0bits.DONE); //attend fin conv
    mi[mesure] = ADRESH;
    mesure++;
    mesure = mesure%4;
    if (mi[3] > 0){
        Vbat = (mi[0]+mi[1]+mi[2]+mi[3])/4;
        if (Vbat < SeuilBat){ //Si sous tension de seuil 10V
            PORTBbits.RB5=0;
        }
        else{
            PORTBbits.RB5=1;
        }
    }
}

/*Fonction permettant de lire un code envoyé par la télécommande et en fonction du contexte d'en déduire comment actualiser les états*/
datas actualiseTelecom(datas etat){
    Lire_i2c_Telecom(0xA2, lectureTelecom); //lecture telecommande
    saveState(&etat); //translation des états, l'état précédent est réécrit avec l'état actuel
    switch(lectureTelecom[1]){
        case 0x31: //code touche droite
            if(etat.precedent.gaucheDroite==0 && etat.precedent.angle==0)
            {
                etat.actuel.gaucheDroite=1;
                etat.actuel.angle=1;
            }
            else if(etat.precedent.gaucheDroite==1 )
            {
                etat.actuel.gaucheDroite=1;
                etat.actuel.angle=2;
            }
            else if(etat.precedent.gaucheDroite==2 && etat.precedent.angle==2)
            {
                etat.actuel.gaucheDroite=2;
                etat.actuel.angle=1;
            }
            else if(etat.precedent.gaucheDroite==2 && etat.precedent.angle==1)
            {
                etat.actuel.gaucheDroite=0;
                etat.actuel.angle=0;
            }

            break;
        case 0x32: //code touche avant
            if (etat.precedent.avantArriere==0 
                    && etat.precedent.vitesse==0){
                etat.actuel.vitesse = 1;
                etat.actuel.avantArriere = 1;
            }
            else if (etat.precedent.avantArriere==1 
                    && etat.precedent.vitesse==1){
                etat.actuel.vitesse = 2;
            }
            else if (etat.precedent.avantArriere==2 
                    && etat.precedent.vitesse==2){
                etat.actuel.vitesse = 1;
            }
            else if (etat.precedent.avantArriere==2 
                    && etat.precedent.vitesse==1){
                etat.actuel.vitesse = 0;
                etat.actuel.avantArriere=0;
            }
            break;
        case 0x33: //code touche marche/arrêt
            flag_IsOn = 1-flag_IsOn;
            if (flag_IsOn == 0){
                etat.actuel.vitesse = 0;
                etat.actuel.angle=0;
                etat.actuel.avantArriere = 0;
                etat.actuel.gaucheDroite = 0;
            }

            break;
        case 0x34: //code touche arriere
            if (etat.precedent.avantArriere==0 
                    && etat.precedent.vitesse==0){
                etat.actuel.vitesse = 1;
                etat.actuel.avantArriere = 2;
            }
            else if (etat.precedent.avantArriere==2 
                    && etat.precedent.vitesse==1){
                etat.actuel.vitesse = 2;
            }
            else if (etat.precedent.avantArriere==1 
                    && etat.precedent.vitesse==2){
                etat.actuel.vitesse = 1;
            }
            else if (etat.precedent.avantArriere==1 
                    && etat.precedent.vitesse==1){
                etat.actuel.vitesse = 0;
                etat.actuel.avantArriere=0;
            }
            break;
        case 0x35: //code touche gauche
            if(etat.precedent.gaucheDroite==0 && etat.precedent.angle==0)
            {
                etat.actuel.gaucheDroite=2;
                etat.actuel.angle=1;
            }
            else if(etat.precedent.gaucheDroite==2 )
            {
                etat.actuel.gaucheDroite=2;
                etat.actuel.angle=2;
            }
            else if(etat.precedent.gaucheDroite==1 && etat.precedent.angle==2)
            {
                etat.actuel.gaucheDroite=1;
                etat.actuel.angle=1;
            }
            else if(etat.precedent.gaucheDroite==1 && etat.precedent.angle==1)
            {
                etat.actuel.gaucheDroite=0;
                etat.actuel.angle=0;
            }
            break;
        default:
            break;
    }
    lectureTelecom[1] = 0; //on efface l'ordre reçu de la mémoire
    Write_PCF8574(0x40, ~datasToChart(etat)); //on affiche le nouvel état surles diode de controle
    return etat;
}

void afficherRS232(){
    return;
}

/*Fonction permettant au robot de savoir comment réagir aux ordres reçus en fonction de la variable d'états*/
datas navigue(datas etats){//*

        switch(etats.actuel.avantArriere){
            case 0:
                CCPR1L = 0;
                CCPR2L = 0;
                break;
            case 1:
                PORTAbits.RA6 = 1;
                PORTAbits.RA7 = 1;
                break;
            case 2:
                PORTAbits.RA6 = 0;
                PORTAbits.RA7 = 0;
                break;
            default:
                Write_PCF8574(0x40, ~0xFF);
                break;
        }
        if (etats.actuel.avantArriere!=0){
            switch(etats.actuel.vitesse){
                case 0:
                    CCPR1L = 0;
                    CCPR2L = 0;
                    break;
                case 1:
                    CCPR1L = v1;
                    CCPR2L = v1;
                    break;
                case 2:
                    CCPR1L = v2;
                    CCPR2L = v2;
                    break;
                default:
                    CCPR1L = 0;
                    CCPR2L = 0;
                    break;
            }
            if (etats.actuel.gaucheDroite == 0){
            CCPR1L = CCPR2L;
            }
            else if (etats.actuel.gaucheDroite!=0){
                switch(etats.actuel.angle){
                    case 0:
                        //CCPR1L = 0;
                        CCPR2L = CCPR1L;
                        break;
                    case 2:
                        if (etats.actuel.gaucheDroite == 2){
                            //CCPR1L = CCPR1L/2;
                            CCPR1L = 70;//(CCPR1L)/2;
                            CCPR2L = 150;
                        }
                        else if (etats.actuel.gaucheDroite == 1){
                            CCPR1L = 150;
                            CCPR2L = 70;//(CCPR2L)/2;
                            
                        }
                        break;
                    case 1:
                        if (etats.actuel.gaucheDroite == 2){
                            CCPR1L = 100;//(CCPR1L+50)/4;
                            CCPR2L = 150;
                        }
                        else if (etats.actuel.gaucheDroite == 1){
                            CCPR1L = 150;
                            CCPR2L = 100;//(CCPR2L+50)/4;
                            
                        }
                        break;
                    default:
                        CCPR1L = 0;
                        CCPR2L = 0;
                        break;
                }
            }
        }
    //*/
    return etats;
}

/*Fonction servant à arreter le robot*/
void stop(){
    saveState(&donnees); //on sauvegarde le dernier état connu
    //on réinitialise le nouvel état
    donnees.actuel.angle=0;
    donnees.actuel.avantArriere = 0;
    donnees.actuel.gaucheDroite = 0;
    donnees.actuel.vitesse = 0;
    //on coupe les moteurs
    CCPR1L = 00 ;
    CCPR2L = 00 ;
    return;
}

void main(void)
{
    char c;
    init();
    flag_IsOn = 0;
    donnees.precedent.angle=0;
    donnees.precedent.avantArriere = 0;
    donnees.precedent.gaucheDroite = 0;
    donnees.precedent.vitesse = 0;
    donnees.actuel.angle=0;
    donnees.actuel.avantArriere = 0;
    donnees.actuel.gaucheDroite = 0;
    donnees.actuel.vitesse = 0;
    Write_PCF8574(0x40, ~datasToChart(donnees));
    CCPR1L = 0;
    CCPR2L = 0;
    while(1)
    {
        //Write_PCF8574(0x40, 0xF0);
        afficherRS232();
        if(flag_2s)
        {
           gestionBatt();
        }
        if(flag_telecom)
        {
            flag_telecom = 0;
            if (c10msTel > 5){
                c10msTel = 0;
                donnees = actualiseTelecom(donnees);
            }
        }
        if(flag_IsOn == 1)
        {
            donnees = navigue(donnees);
        }
        else
        {
            stop();
        }
    }
}
