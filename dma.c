/*
 * dma.c
 *
 *  Created on: 16-Apr-2024
 *      Author: padma
 */

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "dma.h"
#include "nvic.h"

extern uint16_t index;
extern uint16_t start_index;
extern uint16_t stop_index;

 extern uint16_t ping_buffer[64]; // Size is 16 bits as ADC Buffer size is 12 bits. 16bit is close to 12bit. PRIMARY BUFFER
 extern uint16_t pong_buffer[64]; // Size is 16 bits as ADC Buffer size is 12 bits. ALTERNATE BUFFER

extern uint16_t ping_buffer_full_status;
extern uint16_t pong_buffer_full_status;

volatile uint32_t *DMA_Array_Address;

#pragma DATA_ALIGN(DMA_Array,1024)
volatile uint8_t  DMA_Array[1024];

#define UDMA_ENASET_R_25      0x2000000 // Selects Channel 25 ADC1 SS1
#define UDMA_PRIOSET_R_25     0x2000000
#define UDMA_ALTCLR_R_25      0x2000000
#define UDMA_USEBURSTCLR_R_25 0x2000000
#define UDMA_REQMASKCLR_R_25  0x2000000
#define UDMA_CHIS_R_25        0x2000000
#define UDMA_CHASGN_R_25      0x2000000
#define UDMA_USEBURSTSET_R_25    0x2000000

#define Primary_Source_End_Ptr_offset        0x190 // Channel 25 only ADC1 SS1
#define Primary_Destination_End_Ptr_offset   0x194
#define Primary_Control_Word_Ptr_offset      0x198

#define Alternate_Source_End_Ptr_offset      0x390 // Channel 25 only ADC1 SS1
#define Alternate_Destination_End_Ptr_offset 0x394
#define Alternate_Control_Word_Ptr_offset    0x398

volatile uint32_t* UDMA_Primary_Source_End_Ptr       ;
volatile uint32_t* UDMA_Primary_Destination_End_Ptr  ;
volatile uint32_t* UDMA_Primary_Control_Word_Ptr     ;
volatile uint32_t* UDMA_Alternate_Source_End_Ptr     ;
volatile uint32_t* UDMA_Alternate_Destination_End_Ptr;
volatile uint32_t* UDMA_Alternate_Control_Word_Ptr   ;

void initDMA()
{
 SYSCTL_RCGCDMA_R |= SYSCTL_RCGCDMA_R0;
  _delay_cycles(10);

 UDMA_CFG_R        |= UDMA_CFG_MASTEN;
 UDMA_CHMAP3_R      |= (1<<4); // UDMA_CHMAP3_CH25SEL_M
 UDMA_PRIOCLR_R     |= (1<<25);
 UDMA_ALTCLR_R      |= (1<<25);
 UDMA_USEBURSTCLR_R |= (1<<25); // UDMA_USEBURSTCLR_CLR_M change it
 UDMA_REQMASKCLR_R  |= (1<<25);
 // UDMA_CHASGN_R       = (1<<4); // UDMA_CHASGN_R_25

 // while(!UDMA_STAT_MASTEN);

 // DMA_Array_Address = ((volatile uint32_t *) (&DMA_Array[0]));

 UDMA_CTLBASE_R    = (uint32_t)(&DMA_Array[0]);

 // 9.3.4 Configuring a peripheral for Ping-Pong Receive - Channel 25 only ADC1 SS1

 // Channel Control Structure Primary and Alternate Configuration. We create the below registers for our application.

   UDMA_Primary_Source_End_Ptr        = ((volatile uint32_t*)(  UDMA_CTLBASE_R + Primary_Source_End_Ptr_offset         ));

   *UDMA_Primary_Source_End_Ptr       = (uint32_t)(&ADC1_SSFIFO1_R); // Primary Source Points to ADC FIFO. This must be fixed.

   UDMA_Primary_Destination_End_Ptr   = ((volatile uint32_t*)(  UDMA_CTLBASE_R + Primary_Destination_End_Ptr_offset    ));

   *UDMA_Primary_Destination_End_Ptr  = (uint32_t) (&ping_buffer[63]); // Primary Destination Points to the last index of ping buffer

   UDMA_Primary_Control_Word_Ptr      = ((volatile uint32_t*) (  UDMA_CTLBASE_R + Primary_Control_Word_Ptr_offset       ));

   *UDMA_Primary_Control_Word_Ptr     = UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_ARBSIZE_2| (63<<UDMA_CHCTL_XFERSIZE_S) | UDMA_CHCTL_XFERMODE_PINGPONG ;

   UDMA_Alternate_Source_End_Ptr      = ((volatile uint32_t*)(  UDMA_CTLBASE_R + Alternate_Source_End_Ptr_offset       ));

   *UDMA_Alternate_Source_End_Ptr     = (uint32_t)(&ADC1_SSFIFO1_R); // Alternate Source Points to ADC FIFO. This must be fixed.

   UDMA_Alternate_Destination_End_Ptr  = (volatile uint32_t*) (uint32_t) (  UDMA_CTLBASE_R + Alternate_Destination_End_Ptr_offset  );

   *UDMA_Alternate_Destination_End_Ptr = (uint32_t) (&pong_buffer[63]); // Alternate Source Points to the last index of pong buffer\

   UDMA_Alternate_Control_Word_Ptr    = ((volatile uint32_t*) (  UDMA_CTLBASE_R + Alternate_Control_Word_Ptr_offset     ));

   *UDMA_Alternate_Control_Word_Ptr   = UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_ARBSIZE_2| (63<<UDMA_CHCTL_XFERSIZE_S) | UDMA_CHCTL_XFERMODE_PINGPONG;



 // adc interrupt, adc mask,nvic

 ADC1_SSCTL1_R |= ADC_SSCTL1_IE3;  // 4th Sample Interrupt Enable
 ADC1_IM_R     |= ADC_IM_MASK1;    // SS1 Interrupt Mask
 enableNvicInterrupt(INT_ADC1SS1); // ADC1 Sequence 1

 // Enable the DMA

 UDMA_ENASET_R |= (1<<25);

}

void DMA_ISR() // Triggers it through ADC only
{

    ADC1_ISC_R  |= ADC_ISC_IN1;      // SS1 Interrupt Status and Clear

 if ((*UDMA_Primary_Control_Word_Ptr & 0x7) == 0) // It means if the Primary Buffer is FULL
 {
  //   ping_buffer_full_status = 1;
  *UDMA_Primary_Control_Word_Ptr      = UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_ARBSIZE_2| (63<<UDMA_CHCTL_XFERSIZE_S) | UDMA_CHCTL_XFERMODE_PINGPONG;

 }

 if((*UDMA_Alternate_Control_Word_Ptr & 0x7) == 0) // It means if the Alternate Buffer is FULL
 {
  //   pong_buffer_full_status = 1;
  *UDMA_Alternate_Control_Word_Ptr    = UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_ARBSIZE_2| (63<<UDMA_CHCTL_XFERSIZE_S) | UDMA_CHCTL_XFERMODE_PINGPONG;
  // Enable the DMA

   UDMA_ENASET_R |= (1<<25);
 }

 UDMA_CHIS_R |= UDMA_CHIS_R_25;   // DMA Channel Interrupt Status - 1 means Corresponding DMA Channel caused an interrupt - Channel 25 for ADC1SS1 - Write 1 to clear

}








