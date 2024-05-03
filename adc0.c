// ADC0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:

// ADC0 SS1 For Digital Comparator

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "adc0.h"
#include "nvic.h"

extern void DMA_sample_extract_sort(uint16_t *buffer);

extern uint16_t ping_buffer[64]; // Size is 16 bits as ADC Buffer size is 12 bits. 16bit is close to 12bit. PRIMARY BUFFER
extern uint16_t pong_buffer[64];

extern uint16_t index;
extern uint16_t start_index;
extern uint16_t stop_index;

extern uint16_t valid_event_flag;
uint16_t ISR_Hit_Once = 0;

#define ADC_CTL_DITHER            0x00000040

#define ADC_DCCMP0_COMP1_M_Custom (0x500 << 16) // Digital Comparator 0 COMP1 and COMP0 (12bits) 0x005 0x001
#define ADC_DCCMP0_COMP0_M_Custom (0x100 << 0 )    //

#define ADC_DCCMP1_COMP1_M_Custom (0x500 << 16) // Digital Comparator 1 COMP1 and COMP0 (12bits)
#define ADC_DCCMP1_COMP0_M_Custom (0x100 << 0)

#define ADC_DCCMP2_COMP1_M_Custom (0x500 << 16) // Digital Comparator 2 COMP1 and COMP0 (12bits)
#define ADC_DCCMP2_COMP0_M_Custom (0x100 << 0)

#define ADC_DCCMP3_COMP1_M_Custom (0x500 << 16) // Digital Comparator 3 COMP1 and COMP0 (12bits)
#define ADC_DCCMP3_COMP0_M_Custom (0x100 << 0)

// Port E and D masks PE1, PE2, PE3, PD3 (AIN2,AIN1,AIN0,AIN4)

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initAdc0Ss1()
{
    // Enable clocks
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;                               // ADC Module 0 Run Mode Clock Gating Control
    _delay_cycles(16);
    ADC0_ACTSS_R     &= ~ ADC_ACTSS_ASEN1;                               // disable sampler sequencer 1 (SS1) for programming
    ADC0_PC_R        = ADC_PC_SR_1M;                                     // Select 1Msps rate
    ADC0_EMUX_R      = ADC_EMUX_EM1_ALWAYS;                              // SS1 is Always Sampling
    ADC0_SSCTL1_R    = ADC_SSCTL1_END3;                                  // Mark 3rd sample as the end (0,1,2,3)
    ADC0_ACTSS_R    |= ADC_ACTSS_ASEN1;                                  // Enable SS1 for operation
}

// Set SS0 input sample average count
void setAdc0Ss1Log2AverageCount(uint8_t log2AverageCount)
{
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN1;                // disable sample sequencer 1 (SS1) for programming
    ADC0_SAC_R = log2AverageCount;                   // sample HW averaging
    if (log2AverageCount == 0)
        ADC0_CTL_R &= ~ADC_CTL_DITHER;               // turn-off dithering if no averaging
    else
        ADC0_CTL_R |= ADC_CTL_DITHER;                // turn-on dithering if averaging
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN1;                 // enable SS1 for operation
}

// Set SS0 analog input // tweak this to support 4 inputs

void setAdc0Ss1Mux (uint8_t input)
{
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN1;                // disable sample sequencer 1 (SS1) for programming
    ADC0_SSMUX0_R = input;                           // Set analog input for single sample
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN1;                 // enable SS1 for operation
}

void custom_setAdc0Ss1Mux (uint8_t input1, uint8_t input2, uint8_t input3, uint8_t input4)
{
  ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN1;                                  // disable sample sequencer 1 (SS1) for programming
  ADC0_SSMUX1_R |= (input1) |(input2<<4) |(input3<<8) | (input4<<12); // Set analog input for 4 sample Or do 1<<4,2<<8,3<<16,4<<32
  ADC0_ACTSS_R |= ADC_ACTSS_ASEN1;                                   // enable SS1 for operation
}

// Request and read one sample from SS1
int16_t readAdc0Ss1()
{
    ADC0_PSSI_R |= ADC_PSSI_SS1;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS1 is not busy
    while (ADC0_SSFSTAT0_R & ADC_SSFSTAT1_EMPTY);
    return ADC0_SSFIFO1_R;                           // get single result from the FIFO
}

void initAdc0Ss1_DC()
{

   // ADC0 Hardware Oversampling

    ADC0_SAC_R |= ADC_SAC_AVG_64X;

 // Set the ADC's Digital Comparator Range for DC0 to DC3.

      ADC0_DCCMP0_R |= ADC_DCCMP0_COMP1_M_Custom | ADC_DCCMP0_COMP0_M_Custom; // ADC Digital Comparator Range 0 Do the shift-it for both here
      ADC0_DCCMP1_R |= ADC_DCCMP1_COMP1_M_Custom | ADC_DCCMP1_COMP0_M_Custom; // ADC Digital Comparator Range 1
      ADC0_DCCMP2_R |= ADC_DCCMP2_COMP1_M_Custom | ADC_DCCMP2_COMP0_M_Custom; // ADC Digital Comparator Range 2
      ADC0_DCCMP3_R |= ADC_DCCMP3_COMP1_M_Custom | ADC_DCCMP3_COMP0_M_Custom; // ADC Digital Comparator Range 3

 // ADC conversions can either be stored in the ADC Sample Sequence FIFOs or compared using the digital comparator. We do the latter here.

 ADC0_SSOP1_R |=  ADC_SSOP1_S3DCOP   | ADC_SSOP1_S2DCOP  | ADC_SSOP1_S1DCOP  | ADC_SSOP1_S0DCOP;     // Sample 3,2,1,0 Digital Comparator. Skips ADC FIFO & goes to DC directly. ADC_SSOP1_S0DCOP | ADC_SSOP1_S1DCOP ADC_SSOP1_S2DCOP |ADC_SSOP1_S3DCOP
// ADC0_SSDC1_R |=  ADC_SSDC1_S3DCSEL_M|ADC_SSDC1_S3DCSEL_M|ADC_SSDC1_S3DCSEL_M|ADC_SSDC1_S3DCSEL_M;// Sample 3,2,1,0 Digital Comparator Select ADC_SSDC1_S3DCSEL_M

 ADC0_SSDC1_R |=  (0x0<<ADC_SSDC1_S0DCSEL_S)|(0x1<<ADC_SSDC1_S1DCSEL_S)|(0x2<<ADC_SSDC1_S2DCSEL_S);                                                                                                // ADC_SSDC1_S0DCSEL_S

 // Enable the Digital Comparator's (DC 0,1,2,3) Interrupt, Interrupt condition (High band) and Interrupt mode (Always only) using the ADC Digital Comparator Control Register.

 // ADC_DCCTLn_CIE bit enables interrupt.

       ADC0_DCCTL0_R |= ADC_DCCTL0_CIE | ADC_DCCTL0_CIC_HIGH | ADC_DCCTL0_CIM_ONCE;
       ADC0_DCCTL1_R |= ADC_DCCTL1_CIE | ADC_DCCTL1_CIC_HIGH | ADC_DCCTL1_CIM_ONCE;
       ADC0_DCCTL2_R |= ADC_DCCTL2_CIE | ADC_DCCTL2_CIC_HIGH | ADC_DCCTL2_CIM_ONCE;
       ADC0_DCCTL3_R |= ADC_DCCTL3_CIE | ADC_DCCTL3_CIC_HIGH | ADC_DCCTL3_CIM_ONCE;

 // DCONSSx bit of ADC1_IM_R should also be set for interrupt.

 ADC0_IM_R |= ADC_IM_DCONSS1; // Digital Comparator Interrupt on SS1

 enableNvicInterrupt(INT_ADC0SS1);

}

void ADC0_SS1_DC_ISR() // If valid event that is when ADC value is above COMP1, then the interrupt is caused. Clear it.
{
    ADC0_IM_R &= ~ADC_IM_DCONSS1;
    ADC0_DCISC_R |= ADC_DCISC_DCINT0|ADC_DCISC_DCINT1|ADC_DCISC_DCINT2|ADC_DCISC_DCINT3; // Clear the interrupt

    ADC0_DCRIC_R |= ADC_DCRIC_DCINT0|ADC_DCRIC_DCINT1|ADC_DCRIC_DCINT2|ADC_DCRIC_DCINT3; // disable the interrupt

    TIMER1_CTL_R  |= TIMER_CTL_TAEN ;  // Turn on the timer GPTM Timer A Enable for 200us.





}
