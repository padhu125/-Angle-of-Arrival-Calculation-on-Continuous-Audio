// ADC0  Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// ADC0 SS3

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef ADC1_H_
#define ADC1_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initAdc1Ss1();
void setAdc1Ss1Log2AverageCount(uint8_t log2AverageCount);
void setAdc1Ss1Mux(uint8_t input);
void custom_setAdc1Ss1Mux (uint8_t input1, uint8_t input2, uint8_t input3, uint8_t input4);
int16_t readAdc1Ss1();

#endif
