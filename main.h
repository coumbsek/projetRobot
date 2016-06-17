/* 
 * File:   main.h
 * Author: eleves
 *
 * Created on 27 mai 2016, 14:40
 */

#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif

datas actualiseTelecom(datas);
void stop(void);
datas navigue(datas etats);
void afficherRS232(void);
void gestionBatt(void);


#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

