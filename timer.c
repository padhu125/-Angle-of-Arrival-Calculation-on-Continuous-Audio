/*
 * timer.c
 *
 *  Created on: 23-Oct-2023
 *      Author: padma
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "timer.h"
#include "tm4c123gh6pm.h"
#include "wait.h"

extern uint16_t index;
extern uint16_t start_index;
extern uint16_t stop_index0;

#define MAX_CHARS 80
#define MAX_FIELDS 5

extern uint32_t global2[512];
extern uint32_t max;
extern bool timerisr_flag;
extern uint32_t i;

#define UDMA_ENACLR_R_25      0x2000000 // Selects Channel 25 ADC1 SS1

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void Holdoff_isr()
{
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
    ADC0_IM_R |= ADC_IM_DCONSS1;
}
void Holdoff(uint16_t Hold)
{
    // Initialize system clock to 40 MHz
    // initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure Timer 1 as the time base
    TIMER2_CTL_R  &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R   = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R  = TIMER_TAMR_TAMR_1_SHOT;          // configure for periodic mode (count down)
    TIMER2_TAILR_R = Hold*4*10000000;                            // 250 us
    TIMER2_IMR_R   = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    //TIMER1_CTL_R  |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R     = 1 << (INT_TIMER2A-16);              // turn-on interrupt 37 (TIMER1A) in NVIC
}

void initTimer1()
{
    // Initialize system clock to 40 MHz
    // initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure Timer 1 as the time base
    TIMER1_CTL_R  &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R   = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R  = TIMER_TAMR_TAMR_1_SHOT;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 4000;                            // 250 us
    TIMER1_IMR_R   = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    //TIMER1_CTL_R  |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R     = 1 << (INT_TIMER1A-16);              // turn-on interrupt 37 (TIMER1A) in NVIC
}

// timer ISR

void timer1Isr()
{

    // DMA Stop Channel and Stop DMA Acquisition

     UDMA_ENACLR_R &= ~(1<<25);

    // stop_index = index;

    // clear interrupt flag end of the timer timer1Isr()

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
    TIMER2_CTL_R |= TIMER_CTL_TAEN;

}


