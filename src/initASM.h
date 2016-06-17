#ifndef INITASM_H
#define	INITASM_H

#include<p18f2520.h>
#include<stdio.h>
#pragma config OSC = INTIO67
#pragma config PBADEN = OFF, WDT = OFF, LVP = OFF, DEBUG = ON

#pragma interrupt HighISR

void HighISR(void);

#pragma code HighVector=0x08
void IntHighVector(void)
{
    _asm goto HighISR _endasm
}
#pragma code

#endif	/* INITASM_H */

