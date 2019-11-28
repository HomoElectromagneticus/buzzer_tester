/*
 * File:   main.c
 * Author: rschaub
 *
 *                   PIC16F1825
 *                    _______
 *           VDD ---|1      14|--- VSS
 *               ---|2      13|---
 *               ---|3      12|---
 *               ---|4      11|--- potentiometer "input"
 *               ---|5      10|--- 
 *               ---|6       9|--- 
 *       pwm out ---|7_______8|--- "on" light
 * 
 * This program drives a buzzer at specific frequencies to test the loudness
 * for each frequency
 * Created on April 17, 2019, 1:25 PM
 */

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = ON        // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (Low-voltage programming disabled)

#include "buzzer_tester.h"

unsigned int button_state_integral = 0;
unsigned int button_debounce_threshold = 3;
unsigned int button_state_integral_max = 5;
unsigned char button_pushed_flag = 0;
unsigned int display_update_scaler = 0;
unsigned int seconds_scaler = 0;         
unsigned int seconds_divisor = 25;     //set to overflow every tenth of a second
unsigned int seconds_clock = 0;
unsigned char PC_shadow = 0;      //shadow reg for PORTC to avoid R-M-W issues

unsigned int adc_result = 0;        //this is where the ADC value will be stored
char out_state = 0;
int tmp = 0;

void adc_init(void){
    // sets up the ADC
    ADCON0bits.ADON = 0;        //turn ADC off for config
    
    ADCON1bits.ADPREF = 0b00;   //ADC positive reference is set to VDD
    ADCON1bits.ADNREF = 0;      //ADC negative reference is set to VSS
    ADCON0bits.CHS = 0b00010;   //selecting the AN2 analog channel
    ADCON1bits.ADCS = 0b100;    //ADC clock set to FOSC/4
    ADCON1bits.ADFM = 1;        //ADC result is right justified

    ADCON0bits.ADON = 1;        //turn ADC on
}

int ADC_Convert(void){
    ADCON0bits.GO_nDONE = 1;               //start ADC
    while (ADCON0bits.GO_nDONE == 1);      //wait for ADC to finish
    return (ADRESL + (ADRESH *256));
}

void pwm_init(void){
    // starts the PWM output
    // PWM period = [PR2 + 1] * 4 * Tosc * (Timer2 prescale value)
    // PWM period = [255 + 1] * 4 * (1 / 4MHz ) * (1/4)
    // PWM period = 
    // PWM duty cycle = CCPR2L:CCP2CON<5:4> / (4 * (PR2 +1)
    // PWM duty cycle = 0b0011111100 / (4 * ( 255 + 1 ))
    // PWM duty cycle = 50%
    
    TRISCbits.TRISC3 = 1;       //disabling the CCP2 output driver (on pin 7)
    CCP2CONbits.CCP2M = 0b1100; //setting CCP2 for PWM mode
    CCP2CONbits.P2M = 0b00;     //single output, P4A modulated only
    CCP2CONbits.DC2B = 0b11;    //least significant bits for duty cycle
    CCPR4L = 0b1111111111;        //setting the MSBs of the PWM duty cycle
    
    //enable PWM after a new cycle has started
    //while (PIR1bits.TMR2IF == 0);//wait for timer 2 to overflow

    TRISCbits.TRISC3 = 0;       //enable the CCP1 output driver
}

void timer2_init(void) {
    // timer2 is used to drive the output frequency to the buzzer via the PWM
    // driver the clock source is the system clock / 4, which is 1MHz
    T2CONbits.TMR2ON = 0;          //turn off timer2 for configuration
    PIR1bits.TMR2IF = 0;           //reset Timer2 overflow interrupt flag
    
    T2CONbits.T2CKPS = 0b00;       //pre-scaler set to 1:1
    T2CONbits.T2OUTPS = 0b0010;    //post-scaler set to 1:3
    TMR2 = 0;                      //clear timer4
    PR2 = 255;                     //set timer4 match reg to a "nice" value
    
    // the above sets the interrupt freq to ((((1MHz) / 1) / 255) / 3) = ~1.3kHz
    T2CONbits.TMR2ON = 1;          //turn on Timer2
    
    return;
}

void timer4_init(void) {
    // timer4 is used to clock the update of the PWM frequency
    // driver the clock source is the system clock / 4, which is 1MHz
    T4CONbits.TMR4ON = 0;          //turn off timer4 for configuration
    PIR3bits.TMR4IF = 0;           //reset Timer4 overflow interrupt flag
    
    T4CONbits.T4CKPS = 0b11;       //pre-scaler set to 1:64
    T4CONbits.T4OUTPS = 0b0111;    //post-scaler set to 1:8
    TMR4 = 0;                      //clear timer4
    PR4 = 195;                     //set timer4 match reg to a "nice" value
    
    // the above sets the interrupt freq to ((((1MHz) / 64) / 195) / 8) = 10.02Hz
    T4CONbits.TMR4ON = 1;          //turn on Timer4
    
    return;
}

void tmr2_interrupt_handler(void){
    out_state = ~out_state;
    
    PORTCbits.RC3 = out_state;
    
    return;
}

void tmr4_interrupt_handler(void){

    // if the knob is all the way left, let the output frequency be 1kHz. else
    // let the output frequency be 800Hz
    tmp = adc_result / 128;
    switch (tmp) {
        case 0: 
            PR2 = 238;        // 700Hz
            break;
        case 1: 
            PR2 = 208;        // 801Hz
            break;
        case 2: 
            PR2 = 167;        // 998Hz
            break;
        case 3: 
            PR2 = 151;        // 1104Hz
            break;
        case 4: 
            PR2 = 139;        // 1199Hz
            break;
        case 5: 
            PR2 = 111;        // 1501Hz
            break;
        case 6: 
            PR2 = 83;         // 2008Hz
            break;
        case 7: 
            PR2 = 55;         // 3030Hz
            break;
        default: PR2 = 255;         // 654Hz
    }

    return;
}

void main(void) {
    
    // configure the internal program clock to run at 4MHz
    OSCCONbits.SPLLEN = 0b0;    // 4xPLL disabled (also disable by config word)
    OSCCONbits.IRCF = 0b1101;   // HFINTOSC set to 4MHz
    OSCCONbits.SCS = 0b00;      // clock source set by FOSC config word

    // configure the watchdog timer
    WDTCONbits.WDTPS = 0b01011; //set to 2s timer
    
    // configure the inputs and outputs
    TRISAbits.TRISA2 = 1;       //set RA2 (pin 11) as input (for analog input)
    TRISCbits.TRISC0 = 0;       //set RC0 (pin 10) as output
    TRISCbits.TRISC1 = 0;       //set RC1 (pin 9) as output
    TRISCbits.TRISC2 = 0;       //set RC2 (pin 8) as output
    TRISCbits.TRISC3 = 0;       //set RC3 (pin 7) as an output
    ANSELA = 0b00000100;        //set RA2 (pin 11) as an analog input (AN2 channel))
    ANSELC = 0b00000000;        //nothing on port C is an analog input
    
    // turn on interrupts
    PIE1bits.TMR2IE = 1;      //enable timer2 to PR2 match interrupt
    PIE3bits.TMR4IE = 1;      //enable timer4 to PR4 match interrupt
    INTCONbits.PEIE = 1;      //enable peripheral interrupts
    INTCONbits.GIE = 1;       //general interrupts enabled
    
    // configure the timers, the adc, and the pwm
    timer2_init();
    timer4_init();
    adc_init();
    //pwm_init();
    
    PORTCbits.RC2 = 1;          //just to tell the user that the program started
        
    while(1){
        adc_result = ADC_Convert();
        CLRWDT();               //clear the Watchdog Timer to keep the PIC from
                                //resetting. sadly the program won't get here
                                //if the button is pressed...
    }
    return;
}

void interrupt ISR(void){
    // check for timer2 overflow interrupt
    if(PIR1bits.TMR2IF == 1){
        tmr2_interrupt_handler();
        PIR1bits.TMR2IF = 0;                 //reset the interrupt flag
    }
    
    // check for timer4 overflow interrupt
    if(PIR3bits.TMR4IF == 1){
        tmr4_interrupt_handler();
        PIR3bits.TMR4IF = 0;                 //reset the interrupt flag
    }
      
    return;
}