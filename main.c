/* 
 * File:   main.c
 * Author: Neil NACARIO
 *
 * Created on October 4, 2015, 3:25 PM
 */

#include <stdio.h>
#include <stdlib.h>
#define _XTAL_FREQ 20000000.0
#include <xc.h>
//#include <pic16f676.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: 
                                // High speed crystal/resonator on RA4/OSC2/CLKOUT and RA5/OSC1/CLKIN)
#pragma config WDTE = ON        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RA3/MCLR pin function select (RA3/MCLR pin function is MCLR)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config CP = OFF         // Code Protection bit (Program Memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)

#define DUTYCYCLE 60 // 1-99 only
#define TIMERHIGH ((256*DUTYCYCLE)/100)
#define TIMERLOW  (256-TIMERHIGH)

#define LMOTOR1 RC1 //Pin 9 of pic16f676
#define LMOTOR2 RC2 //Pin 8 of pic16f676
#define RMOTOR1 RC4 //Pin 6 of pic16f676
#define RMOTOR2 RC5 //Pin 5 of pic16f676

#define SOFT_PWM_OUT RC0 //Pin 10 of pic16f676

#define ANALOGINPUT1 RA0 //Pin 13 of pic16f676 (Left)
#define ANALOGINPUT2 RA1 //Pin 12 of pic16f676 (Right)

#define LMOTOR_FWD()  LMOTOR1=0;LMOTOR2=1;
#define LMOTOR_REV()  LMOTOR1=1;LMOTOR2=0;
#define LMOTOR_STP()  LMOTOR1=0;LMOTOR2=0;
#define RMOTOR_FWD()  RMOTOR1=0;RMOTOR2=1;
#define RMOTOR_REV()  RMOTOR1=1;RMOTOR2=0;
#define RMOTOR_STP()  RMOTOR1=0;RMOTOR2=0;

typedef enum {
  eIS_C1TEST
  ,eIS_C2TEST
} INPUTSTATE;
//-----------------------DATA MEMORY
unsigned char ucPulseState=0;
unsigned char ucInputOne=0;
unsigned char ucInputTwo=0;
INPUTSTATE ComparatorInputState=eIS_C1TEST;

void Initialize(){
  // Input/Output
  TRISA = 0B00000011; // RA0 & RA1 set as input.
  ANSEL = 0B00000011; // Let ANS0 & ANS1 be analog input
  // Comparator
  CMCON = 0B00000101; // Configure comparator mode (101=Multiplexed Input with
                      // Internal Reference and Output)
  VRCON = 0B10101010; // 1010 => 2.08V with VD set to 5.0V
  // Interrupt
  GIE = 1;  // enable Global interrupts
  
  // Setup Timer0 for software PWM
  TMR0 = TIMERLOW;
  ucPulseState=0;
  OPTION_REG = 0B00000011; // For 0B00000011, output freq is at 1.202 kHz using 20Mhz xtal
  T0CS = 0;
  T0IE = 1; //enable TMR0 overflow interrupts
  TRISC = 0;
  PORTC=0;
  // Setup first comparator to test
  CIS = 0;
  ComparatorInputState=eIS_C1TEST;
}

void interrupt OnInterrupt(void)
{
  if (T0IE && T0IF)
  {
    T0IF=0; // set to 0 to allow subsequent interrupts
    if (ucPulseState){
      SOFT_PWM_OUT=0;
      TMR0=TIMERHIGH; //Next timer0 start value
      ucPulseState=0;
    }
    else{
      SOFT_PWM_OUT=1;
      TMR0=TIMERLOW; //Next timer0 start value
      ucPulseState=1;
    }
  }
}

void UpdateDirections()
{
  //InputOne=Left; InputTwo=Right
  if ( 0==ucInputOne&& 0==ucInputTwo ){
    RMOTOR_FWD(); LMOTOR_FWD();
  }
  else if( 0==ucInputOne&& 1==ucInputTwo ){
    RMOTOR_FWD(); LMOTOR_REV();
  }
  else if( 1==ucInputOne&& 0==ucInputTwo ){
    RMOTOR_REV(); LMOTOR_FWD();
  }
  else if( 1==ucInputOne&& 1==ucInputTwo ){
    RMOTOR_STP(); LMOTOR_STP();
  }
}

//! Check comparator inputs
void CheckInputs()
{
  switch (ComparatorInputState){
    case eIS_C1TEST:
      ucInputOne = COUT;//Retrieve comparator output
      CIS = 1;//Change comparator input
      ComparatorInputState = eIS_C2TEST;//next state
      break;
    case eIS_C2TEST:
      ucInputTwo = COUT;//Retrieve comparator output
      CIS = 0;//Change comparator input
      // Decide
      UpdateDirections();
      //fall-through
    default:
      ComparatorInputState = eIS_C1TEST;//next state
  }
}

/*
 * 
 */
int main(int argc, char** argv) {
  Initialize();
  
  while(1){
    __delay_us(1000); // Should not be less than Comparator response time (400ns MAX)
    asm("CLRWDT");
    CheckInputs();
    asm("CLRWDT");
  }
  return (EXIT_SUCCESS);
}

