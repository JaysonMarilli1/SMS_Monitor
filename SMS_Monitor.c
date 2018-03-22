/* 
 * File:   SMS_Monitor.c
 * Author: Jayson
 *
 * Created on 18 July 2013, 9:03 AM
 *
 */
/**************************************************************************/
/*                                                                        */
/*  SMS Monitor System for MSS - Microchip PIC16F1827 Microcontrollers    */
/*                Copyright jayson@microcall.co.za                        */
/*                    Version 18 July 2013, 9:03 AM                       */
/*                                                                        */
/**************************************************************************/

// Includes
#include <stdio.h>
#include <stdlib.h>
#include <htc.h>

// Defines
#define _XTAL_FREQ 8000000
#define INPUT1 RB0
#define INPUT2 RB1
#define INPUT3 RB2
#define INPUT4 RB3
#define OnOffSwitch RB4
#define StaySwitch RB5

#define OUTPUT1 RA0
#define OUTPUT2 RA1
#define OUTPUT3 RA2
#define OUTPUT4 RA3
#define statusLED RA4
#define buzzer RA6

// PIC16F1827 Configuration Bit Settings
__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_OFF & MCLRE_ON & CP_OFF &
        CPD_OFF & BOREN_OFF & CLKOUTEN_OFF & IESO_OFF & FCMEN_OFF);
__CONFIG(WRT_OFF & PLLEN_OFF & STVREN_OFF & LVP_OFF);


/* -------------------LATA-----------------
 * Bit#:  -7---6---5---4---3---2---1---0---
 * LED:   -------|OP6|OP5|OP4|OP3|OP2|OP1|-
 *-----------------------------------------
 */

/* -------------------LATB-----------------
 * Bit#:  -7---6---5---4---3---2---1---0---
 * LED:   -------|IP6|IP5|IP4|IP3|IP2|IP1|-
 *-----------------------------------------
 */

enum status {
    ACTIVE = 1, // Inputs 1 - 4 normal state = low so high activates
    INACTIVE = 0 // Low de activates
};

// Functions
void Monitor_ON(void);
void Monitor_OFF(void);
void Monitor_STAY(void);

void init(void);
void InitTimer(void);
void delay_s(unsigned int second); // function for delay in seconds
void Buzzer(unsigned int second); // function to turn on buzzer for x seconds
void Status_LED(int x); // function for status LED. 1 = on. 2 = slow flash. 3 = fast flash

// Global variables
int tick_count = 0;
int off_count = 1;
int tc;
int flashrate;
int state = 0;
int trigger_reset = 500; // resets the triggers 5s after they have been activated. Avoid SMS spam
int enableLOW1 = 1;
int enableLOW2 = 1;
int enableLOW3 = 1;
int enableLOW4 = 1;
int enable1 = 1;
int enable2 = 1;
int enable3 = 1;
int enable4 = 1;
int enable_count1 = 0;
int enable_count2 = 0;
int enable_count3 = 0;
int enable_count4 = 0;

void main(void) {

    // Set start up in known states
    PORTB = 0x30;
    PORTA = 0x00;

    // Initialise the chip
    init();
    InitTimer(); // Initialises timer0 as well as weak pull up

    /*
     * POSSIBLE NEED FOR A DELAY TO LET MOTION SENSOR AND GSM INITIALIZE ?????
     */
    statusLED = ACTIVE;
    delay_s(10);
    statusLED = INACTIVE;

    state = 0; // Set monitor state to OFF on start up
    Monitor_OFF();

    while (1) {
        NOP();

        while (OnOffSwitch == INACTIVE) {
            //                    if(OnOffSwitch == ACTIVE){
            NOP();
            __delay_ms(80);
            if (OnOffSwitch == ACTIVE && state == 1) {
                off_count++;
            }

            if (OnOffSwitch == ACTIVE && state == 0) {
                //                            if(OnOffSwitch && state == 0){
                Monitor_ON();
                state = 1; // Set monitor state to ON
                off_count = 0;
            } else if (OnOffSwitch == ACTIVE && state == 1 && off_count == 2) {
                //                            else if(OnOffSwitch && state == 1){
                Monitor_OFF();
                state = 0; // Set monitor state to OFF
                off_count = 1;
            }
        }
        if (StaySwitch == INACTIVE && state == 0) {
            Monitor_STAY();

            while (INPUT4 == INACTIVE) { // Monitor Input 4 only
                NOP();
                __delay_ms(80);
                while (INPUT4 == ACTIVE) {
                    buzzer = ACTIVE;
                }
                if (StaySwitch == ACTIVE) {
                    buzzer = INACTIVE;
                    Monitor_OFF();
                    state = 0; // Set monitor state to OFF
                    break;
                }
            }
        }


    }
}

void Monitor_ON(void) {

    Status_LED(1);
    Buzzer(1); // Turns on the buzzer for 1 second
    delay_s(30);
    TMR1IE = 1; // Eisable timer1 overflow interrupts scanning inputs 1 to 4
    enable1 = 1;
    enable2 = 1;
    enable3 = 1;
    enable4 = 1;
    //

}

void Monitor_OFF(void) {
    Status_LED(2);
    TMR1IE = 0; // Disable timer1 overflow interrupts. ignores inputs 1 to 4
    Buzzer(1);
    Buzzer(1);
    //
}

void Monitor_STAY(void) {

    Status_LED(3);
    TMR1IE = 0; // Disable timer1 overflow interrupts. ignores inputs 1 to 3
    Buzzer(5);
    //
}

void delay_s(unsigned int second) {
    second = second * 1000;
    while (second > 0) {
        __delay_ms(2);
        second--;
    }
}

void Buzzer(unsigned int second) {
    buzzer = ACTIVE;
    delay_s(second);
    buzzer = INACTIVE;
    __delay_ms(600);
}

void init(void) {
    // Initialises the chip
    OSCCON = 0x7f; // 8 MHz
    ANSELA = 0; // define outputs as digital
    ANSELB = 0; // define inputs as digital
    TRISA = 0x00; // TRIS: 0 = output, 1=input. RA5 is an input only
    TRISB = 0xff; // TRIS: 0 = output, 1=input.
    WPUB = 0x3F; // Enables weak pull up on PORTB. Bits <5:0>
    // CMCON = 0x07; ??????????????????????????
}

void InitTimer(void) {

    OPTION_REG = 0b01010111;
    /*             0-------     Enables weak pull ups
     *             --0-----     Internal instruction cycle clock (Fosc/4)
     *             ---1----     increment on high to low transition on T0CKI pin
     *             ----0---     Prescaler is assigned to timer0
     *             -----111     Sets timer0 prescaler to 1:256
     */
    TMR0IF = 0; // Clear TMR0 flag
    TMR0IE = 0; // Disable timer0 overflow interrupt (for now)

    T1CON = 0b00111001;
    /*        00------     Timer 1 clock source is instruction clock (Fosc/4)
     *        --11----     Sets timer1 prescaler to 1:8
     *        ----1---     Dedicated Timer1 oscillator circuit enabled
     *        -----1--     Do not synchronize external clock input
     *        -------1     Enables timer1
     */
    TMR1IF = 0; // CLear TMR1 flag
    TMR1IE = 0; // Disable timer1 overflow interrupts (for now)

    PEIE = 1; // 1 = enables all active peripheral interrupts
    GIE = 1; //Global interrupt enable

}

void interrupt tc_int(void) {
    if (TMR0IF) {
        // do your stuff here, given that this occurs every 1.28 msec

        TMR0IF = 0; // Clearing the timer0 flag


        if (enable1 == 0) {
            ++enable_count1;
        }
        if (enable2 == 0) {
            ++enable_count2;
        }
        if (enable3 == 0) {
            ++enable_count3;
        }
        if (enable4 == 0) {
            ++enable_count4;
        }

        if (enable_count1 == trigger_reset) {
            enable1 = 1;
            enable_count1 = 0;
        }
        if (enable_count2 == trigger_reset) {
            enable2 = 1;
            enable_count2 = 0;
        }
        if (enable_count3 == trigger_reset) {
            enable3 = 1;
            enable_count3 = 0;
        }
        if (enable_count4 == trigger_reset) {
            enable4 = 1;
            enable_count4 = 0;
        }

        //
        if (INPUT1 == INACTIVE) {
            enableLOW1 = 1;
        }
        if (INPUT2 == INACTIVE) {
            enableLOW2 = 1;
        }
        if (INPUT3 == INACTIVE) {
            enableLOW3 = 1;
        }
        if (INPUT4 == INACTIVE) {
            enableLOW4 = 1;
        }
        //

        ++tick_count;
        if (flashrate == 1) { // If monitor is on the status LED will stay on and not toggle
            statusLED = ACTIVE;
        } else if (tick_count == flashrate) { // Status LED will toggle if monitor off or in stay mode
            statusLED = ~statusLED;
            tick_count = 0;
        }
        return;
    }
    // if you service other int, handle them here
    if (TMR1IF) {
        /* timer1 is used to check the triggers every x ms (1ms)
         * to avoid sms spam i used timer0 and the 'enables'
         * timer0 will only set the enables to true every 50 seconds
         * an additional feature to avoid sms spam is the 'enableLOW'
         * which timer0 will only set to true if the trigger becomes
         * low again. This will avoid scenarios where doors are left open
         */
        TMR1IF = 0; // Clear the timer1 flag
        if (INPUT1 == ACTIVE && enable1 == 1 && enableLOW1 == 1) {
            OUTPUT1 = ACTIVE;
            enable1 = 0;
            enableLOW1 = 0;
            delay_s(2);
            OUTPUT1 = INACTIVE;
        }
        if (INPUT2 == ACTIVE && enable2 == 1 && enableLOW2 == 1) {
            OUTPUT2 = ACTIVE;
            enable2 = 0;
            enableLOW2 = 0;
            delay_s(2);
            OUTPUT2 = INACTIVE;
        }
        if (INPUT3 == ACTIVE && enable3 == 1 && enableLOW3 == 1) {
            OUTPUT3 = ACTIVE;
            enable3 = 0;
            enableLOW3 = 0;
            delay_s(2);
            OUTPUT3 = INACTIVE;
        }
        if (INPUT4 == ACTIVE && enable4 == 1 && enableLOW4 == 1) {
            OUTPUT4 = ACTIVE;
            enable4 = 0;
            enableLOW4 = 0;
            delay_s(2);
            OUTPUT4 = INACTIVE;
        }
        return;
    }

}

void Status_LED(int x) { // function for status LED. 1 = on. 2 = slow flash. 3 = fast flash
    switch (x) {
        case 1:
            //            TMR0IE = 0; // don't allow T0 to interrrupt me
            TMR0IE = 1; // allow T0 to interrrupt me
            //            statusLED = ACTIVE;
            flashrate = 1;
            tick_count = 0;
            break;
        case 2:
            TMR0IE = 1; // allow T0 to interrrupt me
            flashrate = 100; // 100 = slow flash rate
            tick_count = 0;
            break;
        case 3:
            TMR0IE = 1; // allow T0 to interrrupt me
            flashrate = 20; // 20 = fast flash rate
            tick_count = 0;
            break;
    }
}
