// ADC0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:

// ADC1 SS1 for DMA

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "adc1.h"

#define ADC_CTL_DITHER          0x00000040

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initAdc1Ss1()
{
    // Enable clocks
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R1;
    _delay_cycles(16);
    ADC1_ACTSS_R     &= ~ADC_ACTSS_ASEN1;                // disable sample sequencer 1 (SS1) for programming
    ADC1_PC_R         = ADC_PC_SR_1M;                        // select 1Msps rate
    ADC1_EMUX_R       = ADC_EMUX_EM1_ALWAYS;               // Always Sample
    ADC1_SSCTL1_R     = ADC_SSCTL1_END3;                 // mark 3rd sample as the end (0,1,2,3)
    //ADC1_ACTSS_R     |= ADC_ACTSS_ASEN1;                 // enable SS1 for operation
}

// Set SS3 input sample average count
void setAdc1Ss1Log2AverageCount(uint8_t log2AverageCount)
{
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN1;                // disable sample sequencer 1 (SS1) for programming
    ADC1_SAC_R = log2AverageCount;                   // sample HW averaging
    if (log2AverageCount == 0)
        ADC1_CTL_R &= ~ADC_CTL_DITHER;               // turn-off dithering if no averaging
    else
        ADC1_CTL_R |= ADC_CTL_DITHER;                // turn-on dithering if averaging
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN1;                 // enable SS1 for operation
}

// Set SS1 analog input
void setAdc1Ss1Mux(uint8_t input)
{
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN1;                // disable sample sequencer 1 (SS1) for programming
    ADC1_SSMUX1_R = input;                           // Set analog input for single sample
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN1;                 // enable SS3 for operation
}

void custom_setAdc1Ss1Mux (uint8_t input1, uint8_t input2, uint8_t input3, uint8_t input4)
{
  ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN1;                                  // disable sample sequencer 1 (SS1) for programming
  ADC1_SSMUX1_R |= (input1) |(input2<<4) |(input3<<8) | (input4<<12); // Set analog input for 4 sample
  ADC1_ACTSS_R |= ADC_ACTSS_ASEN1;                                   // enable SS1 for operation
}

// Request and read one sample from SS3

int16_t readAdc1Ss1()
{
    ADC1_PSSI_R |= ADC_PSSI_SS1;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    while (ADC1_SSFSTAT1_R & ADC_SSFSTAT1_EMPTY);
    return ADC1_SSFIFO1_R;                           // get single result from the FIFO
}
