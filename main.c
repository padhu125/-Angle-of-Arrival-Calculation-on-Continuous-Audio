#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "adc0.h"
#include "adc1.h"
#include "gpio.h"
#include "nvic.h"
#include "uart0.h"
#include <string.h>
#include <stdio.h>
#include "dma.h"
#include "timer.h"
#include <inttypes.h>
#include "uart0.h"

#define MAX_CHARS 80
#define MAX_FIELDS 5

typedef struct _USER_DATA
{
 char buffer[MAX_CHARS+1];
 uint8_t fieldCount;
 uint8_t fieldPosition[MAX_FIELDS];
 char fieldType[MAX_FIELDS];
} USER_DATA;

// PortE and D masks PE1 (mid), PE2, PE3, PD3 (AIN2,AIN1,AIN0,AIN4)

#define AIN2_MASK 8
#define AIN1_MASK 4
#define AIN0_MASK 2
#define AIN4_MASK 16

#define UDMA_ENASET_R_25      0x2000000 // Selects Channel 25 ADC1 SS1

uint16_t i = 0;
uint16_t j = 0;
uint16_t k = 0;

uint16_t raw1 = 0;
uint16_t raw2 = 0;
uint16_t raw3 = 0;
uint16_t raw4 = 0;

uint16_t index ;
uint16_t start_index ;
uint16_t stop_index ;

uint16_t ping_buffer[64]; // Size is 16 bits as ADC Buffer size is 12 bits. 16bit is close to 12bit. PRIMARY BUFFER
uint16_t pong_buffer[64]; // Size is 16 bits as ADC Buffer size is 12 bits. ALTERNATE BUFFER

uint32_t mic_pe1_samples[16];
uint32_t mic_pe2_samples[16];
uint32_t mic_pe3_samples[16];
uint32_t mic_pd3_samples[16];

uint16_t valid_event_flag = 0;

#define length                  31
#define n                       16
#define Sample_Size             16
#define Correlation_Result_Size 31
#define Sampling_Rate           1000000
#define Sampling_Period        (1/Sampling_Rate)

// PE1-PE2, PE2-PE3, PE3-PD3, PD3-PE1 PE3-PE2

uint32_t mic_pe1_pe2_xcorr[Correlation_Result_Size];

uint32_t mic_pe2_pe3_xcorr[Correlation_Result_Size]; // Correlation length is 2*n - 1; Here n = 16 samples
uint32_t mic_pe3_pd3_xcorr[Correlation_Result_Size];
uint32_t mic_pd3_pe1_xcorr[Correlation_Result_Size];
uint32_t mic_pd3_pe2_xcorr[Correlation_Result_Size];

// PE1-PE2, PE2-PE3, PE3-PD3, PD3-PE1

uint32_t max_index_pe1_pe2 = 0;        // Assume first element is maximum
uint32_t max_index_pe2_pe3 = 0;        // Assume first element is maximum
uint32_t max_index_pe3_pd3 = 0;        // Assume first element is maximum
uint32_t max_index_pd3_pe1 = 0;        // Assume first element is maximum
uint32_t max_index_pd3_pe2 = 0;

uint32_t max_value_pe2_pe3 = 0; // Initialize with the 1st element // Initialize with the 1st element mic_pe2_pe3_xcorr[0]
uint32_t max_value_pe3_pd3 = 0 ; // Initialize with the 1st element // Initialize with the 1st element mic_pe3_pd3_xcorr[0]
uint32_t max_value_pd3_pe2 = 0 ; // Initialize with the 1st element // Initialize with the 1st element mic_pd3_pe2_xcorr[0]




uint16_t ping_buffer_full_status = 0;
uint16_t pong_buffer_full_status = 0;

float T1=0;
float T2=0;
float T3=0;

float theta_main =0;
float K1 = 0.5;
float K2 = 0.5;
float K3 = 0.5;
float theta_0;

uint16_t THETA_FINAL = 0;
uint16_t THETA_FINAL_1 = 0;
uint16_t THETA_FINAL_2 = 0;
uint16_t THETA_FINAL_3 = 0;

uint16_t Hold = 1;

// Initialize Hardware
void initHw()
{
  // Initialize system clock to 40 MHz
  initSystemClockTo40Mhz();

  enablePort(PORTD);
  enablePort(PORTE);

  // Configure AIN0 as an analog input
  GPIO_PORTE_AFSEL_R |= AIN0_MASK;                 // select alternative functions for AIN0(PE1)
  GPIO_PORTE_DEN_R &= ~AIN0_MASK;                  // turn off digital operation on pin PE1
  GPIO_PORTE_AMSEL_R |= AIN0_MASK;                 // turn on analog operation on pin PE1

  // Configure AIN1 as an analog input
  GPIO_PORTE_AFSEL_R |= AIN1_MASK;                 // select alternative functions for AIN1 (PE2)
  GPIO_PORTE_DEN_R &= ~AIN1_MASK;                  // turn off digital operation on pin PE2
  GPIO_PORTE_AMSEL_R |= AIN1_MASK;                 // turn on analog operation on pin PE2

  // Configure AIN2 as an analog input
  GPIO_PORTE_AFSEL_R |= AIN2_MASK;                 // select alternative functions for AIN2 (PE3)
  GPIO_PORTE_DEN_R &= ~AIN2_MASK;                  // turn off digital operation on pin PE3
  GPIO_PORTE_AMSEL_R |= AIN2_MASK;                 // turn on analog operation on pin PE3

  // Configure AIN4 as an analog input
  GPIO_PORTE_AFSEL_R |= AIN4_MASK;                 // select alternative functions for AIN4 (PD3)
  GPIO_PORTE_DEN_R &= ~AIN4_MASK;                  // turn off digital operation on pin PD3
  GPIO_PORTE_AMSEL_R |= AIN4_MASK;                 // turn on analog operation on pin PD3

}

void PING_BUFFER_FUNCTION()
{
    // DMA SAMPLE EXTRACTION

     for (i=0;i<64;i=i+4)
     {
      mic_pe1_samples[i/4] = ping_buffer[i];
      mic_pe2_samples[i/4] = ping_buffer[i + 1];
      mic_pe3_samples[i/4] = ping_buffer[i + 2];
      mic_pd3_samples[i/4] = ping_buffer[i + 3];
     }

     // FIND CROSS CORRELATION FOR THE PAIRS PE2-PE3, PE3-PD3, PD3-PE2

     for(i=0;i<Correlation_Result_Size;i++)
      {
         mic_pe2_pe3_xcorr[i]=0;      // Initialize cross correlation result
       for(j=0;j<n;j++) // n= 16 no. of samples
       {
        k = j+i;
        if(k<n)
        {
         mic_pe2_pe3_xcorr[i] += mic_pe2_samples[j] * mic_pe3_samples[k];
        }
       }
      }

     for(i=0;i<Correlation_Result_Size;i++)
        {
         mic_pe3_pd3_xcorr[i]=0;      // Initialize cross correlation result
         for(j=0;j<n;j++) // n= 16 no. of samples
         {
          k = j+i;
          if(k<n)
          {
           mic_pe3_pd3_xcorr[i] += mic_pe3_samples[j] * mic_pd3_samples[k];
          }
         }
        }

     for(i=0;i<Correlation_Result_Size;i++)
        {
         mic_pd3_pe2_xcorr[i]=0;      // Initialize cross correlation result
         for(j=0;j<n;j++) // n= 16 no. of samples
         {
          k = j+i;
          if(k<n)
          {
              mic_pd3_pe2_xcorr[i] += mic_pd3_samples[j] * mic_pe3_samples[k];
          }
         }
        }

     // CALCULATE THE MAXIMUM INDEX FROM PE2-PE3, PE3-PD3, PD3-PE2 Combination




        for (i=1;i<length;i++)
        {
         if(mic_pe2_pe3_xcorr[i] > max_value_pe2_pe3)      // Find the new maximum
         {
          max_value_pe2_pe3 = mic_pe2_pe3_xcorr[i];        // New maximum gets stored here
          max_index_pe2_pe3 = i;               // index with max correlation indicates the time shift/delay. Use it to find time stamps by multiplying it with sample period
         }
        }



         for (i=1;i<length;i++)
         {
          if(mic_pe3_pd3_xcorr[i] > max_value_pe3_pd3)      // Find the new maximum
          {
           max_value_pe3_pd3 = mic_pe3_pd3_xcorr[i];        // New maximum gets stored here
           max_index_pe3_pd3 = i;               // index with max correlation indicates the time shift/delay. Use it to find time stamps by multiplying it with sample period
          }
         }



          for (i=1;i<length;i++)
          {
           if(mic_pd3_pe2_xcorr[i] > max_value_pd3_pe2)      // Find the new maximum
           {
            max_value_pd3_pe2 = mic_pd3_pe2_xcorr[i];        // New maximum gets stored here
            max_index_pd3_pe2 = i;               // index with max correlation indicates the time shift/delay. Use it to find time stamps by multiplying it with sample period
           }
          }

          // FIND TIME STAMPS BY MULTIPLYING IT TO THE TIME PERIOD

          T1 =  max_index_pe2_pe3 ;
          T2 = max_index_pe3_pd3 ;
          T3 =  max_index_pd3_pe2 ;

          if(T1 <T2 && T1 < T3)
          {
          THETA_FINAL_1 = (K1 *(T3-T2))  + (K2 * (T3-T2) * (T3-T2) ) ;
          THETA_FINAL = THETA_FINAL_1;
          }

          if (T2 < T1 && T2 < T3)
          {
           THETA_FINAL_2 = -120 + (K1 *(T3-T2))  + (K2 * (T3-T2) * (T3-T2) ) ;
           THETA_FINAL = THETA_FINAL_2;
          }

          if (T3 < T1 && T3 < T2)
          {
           THETA_FINAL_3 = 120 + (K1 *(T3-T2))  + (K2 * (T3-T2) * (T3-T2) ) ;
           THETA_FINAL = THETA_FINAL_3;
          }
}

void PONG_BUFFER_FUNCTION()
{
    // DMA SAMPLE EXTRACTION

     for (i=0;i<64;i=i+4)
     {
      mic_pe1_samples[i/4] = pong_buffer[i];
      mic_pe2_samples[i/4] = pong_buffer[i + 1];
      mic_pe3_samples[i/4] = pong_buffer[i + 2];
      mic_pd3_samples[i/4] = pong_buffer[i + 3];
     }

     // FIND CROSS CORRELATION FOR THE PAIRS PE2-PE3, PE3-PD3, PD3-PE2

     for(i=0;i<Correlation_Result_Size;i++)
      {
         mic_pe2_pe3_xcorr[i]=0;      // Initialize cross correlation result
       for(j=0;j<n;j++) // n= 16 no. of samples
       {
        k = j+i;
        if(k<n)
        {
         mic_pe2_pe3_xcorr[i] += mic_pe2_samples[j] * mic_pe3_samples[k];
        }
       }
      }

     for(i=0;i<Correlation_Result_Size;i++)
        {
         mic_pe3_pd3_xcorr[i]=0;      // Initialize cross correlation result
         for(j=0;j<n;j++) // n= 16 no. of samples
         {
          k = j+i;
          if(k<n)
          {
           mic_pe3_pd3_xcorr[i] += mic_pe3_samples[j] * mic_pd3_samples[k];
          }
         }
        }

     for(i=0;i<Correlation_Result_Size;i++)
        {
         mic_pd3_pe2_xcorr[i]=0;      // Initialize cross correlation result
         for(j=0;j<n;j++) // n= 16 no. of samples
         {
          k = j+i;
          if(k<n)
          {
              mic_pd3_pe2_xcorr[i] += mic_pd3_samples[j] * mic_pe3_samples[k];
          }
         }
        }

     // CALCULATE THE MAXIMUM INDEX FROM PE2-PE3, PE3-PD3, PD3-PE2 Combination


      //uint32_t max_value_pe2_pe3 = 0; // Initialize with the 1st element // Initialize with the 1st element mic_pe2_pe3_xcorr[0]

        for (i=1;i<length;i++)
        {
         if(mic_pe2_pe3_xcorr[i] > max_value_pe2_pe3)      // Find the new maximum
         {
          max_value_pe2_pe3 = mic_pe2_pe3_xcorr[i];        // New maximum gets stored here
          max_index_pe2_pe3 = i;               // index with max correlation indicates the time shift/delay. Use it to find time stamps by multiplying it with sample period
         }
        }

        //uint32_t max_value_pe3_pd3 = 0; // Initialize with the 1st element // Initialize with the 1st element mic_pe3_pd3_xcorr[0]

         for (i=1;i<length;i++)
         {
          if(mic_pe3_pd3_xcorr[i] > max_value_pe3_pd3)      // Find the new maximum
          {
           max_value_pe3_pd3 = mic_pe3_pd3_xcorr[i];        // New maximum gets stored here
           max_index_pe3_pd3 = i;               // index with max correlation indicates the time shift/delay. Use it to find time stamps by multiplying it with sample period
          }
         }

         //uint32_t max_value_pd3_pe2 = 0; // Initialize with the 1st element // Initialize with the 1st element mic_pd3_pe2_xcorr[0]

          for (i=1;i<length;i++)
          {
           if(mic_pd3_pe2_xcorr[i] > max_value_pd3_pe2)      // Find the new maximum
           {
            max_value_pd3_pe2 = mic_pd3_pe2_xcorr[i];        // New maximum gets stored here
            max_index_pd3_pe2 = i;               // index with max correlation indicates the time shift/delay. Use it to find time stamps by multiplying it with sample period
           }
          }

          // FIND TIME STAMPS BY MULTIPLYING IT TO THE TIME PERIOD

          T1 =  max_index_pe2_pe3 ;  // Sampling_Period * (1/1000000) *
          T2 =  max_index_pe3_pd3 ;  //Sampling_Period * (1/1000000) *
          T3 =  max_index_pd3_pe2 ;  //Sampling_Period * (1/1000000) *

          if(T1 < T2 && T1 <T3)
          {
          THETA_FINAL_1 = (K1 *(T3-T2))  + (K2 * (T3-T2) * (T3-T2) ) ;
          THETA_FINAL = THETA_FINAL_1;
          }

          if (T2 < T1 && T2 < T3)
          {
           THETA_FINAL_2 = -120 + (K1 *(T3-T2))  + (K2 * (T3-T2) * (T3-T2) ) ;
           THETA_FINAL = THETA_FINAL_2;
          }

          if (T3 < T1 && T3 < T2)
          {
           THETA_FINAL_3 = 120 + (K1 *(T3-T2))  + (K2 * (T3-T2) * (T3-T2) ) ;
           THETA_FINAL = THETA_FINAL_3;
          }


         // THETA_FINAL = THETA_FINAL + 0;
}

extern void getsUart0(USER_DATA *data);
extern void parseFields(USER_DATA *data);
extern bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments);

/**
 * main.c
 */
int main(void)
{

  int putty_flag=0;

  USER_DATA data;
  initHw();
  char str[80];
  initUart0();
  setUart0BaudRate (115200,40e6);



       // DMA's ADC1SS1

       initAdc1Ss1();
       custom_setAdc1Ss1Mux (2,1,0,4); // custom_setAdc1Ss1Mux (2,1,0,4);
       initDMA();
       UDMA_ENASET_R    |= (1<<25);         // Where or when should we enable the DMA?

     // DC's ADC0SS1

     initTimer1();
     initAdc0Ss1();
     custom_setAdc0Ss1Mux(2,1,0,4);
     Holdoff(Hold);
     initAdc0Ss1_DC();

      while(1)
      {

          PING_BUFFER_FUNCTION();

          PONG_BUFFER_FUNCTION();

       if(kbhitUart0())
       {
        getsUart0(&data);
        parseFields(&data);

        if(isCommand(&data,"Holdoff",1))
        {
            Hold  = (uint16_t) getFieldInteger(&data,1);
            Holdoff(Hold);

        }
        if(isCommand(&data,"aoa",0))
        {
         putty_flag = 1;
        }

       }

       if(putty_flag == 1)
       {
           snprintf(str, sizeof(str),"THETA_FINAL:   %d\n", THETA_FINAL);
           putsUart0(str);
       }

      }

}

