// UART0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"

// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1

#define MAX_CHARS 80
#define MAX_FIELDS 5

// Bit band aliases
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4))) // PF1 Activity LED
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // PF2 Transmit LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4))) // PF3

// PortF masks
#define GREEN_LED_MASK 8
#define RED_LED_MASK   2
#define BLUE_LED_MASK  4

typedef struct _USER_DATA
{
 char buffer[MAX_CHARS+1];
 uint8_t fieldCount;
 uint8_t fieldPosition[MAX_FIELDS];
 char fieldType[MAX_FIELDS];
} USER_DATA;

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initUart0()
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    _delay_cycles(3);

    // Configure UART0 pins
    GPIO_PORTA_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
    GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX; // select UART0 to drive pins PA0 and PA1: default, added for clarity


    // Configure UART0 to 115200 baud (assuming fcyc = 40 MHz), 8N1 format
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
}

// Set baud rate as function of instruction cycle frequency

void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    divisorTimes128 += 1;                               // add 1/128 to allow rounding
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART0_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // turn-on UART0
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart0()
{
    return !(UART0_FR_R & UART_FR_RXFE);
    // RED_LED ^= 1;
}

// This is a function to receive characters from the user interface, processing special
// characters such as backspace and writing the resultant string into the buffer.

void getsUart0(USER_DATA *data)
{
    uint32_t count = 0;           // Initialize a local variable, count, to zero.                                                        :a
    char character;


    while (1)
    {
        character = getcUart0();  // Get a character from UART0                                                                          :b (loop back here)

        if ((character == 8 || character == 127) && count > 0) // If the character is a backspace (ASCII code 8 or 127) and count > 0    :c
        {
            count = count - 1; // decrement the count (erases the character from buffer)
        }
        if (character == 13)  // If the character is a carriage return (ASCII code 13)                                                   :d
        {
            data->buffer[count] = '\0'; // add a null terminator to the end of the string
            return;
        }
        if (character >= 32) // If the character is a space (ASCII code 32) or another printable character                               :e
        {
            data->buffer[count] = character; // store the character in data->buffer[count]
            count = count + 1;               // Increment the count
        }
        if (count == MAX_CHARS)  // if count is MAX / FULL                                                                               :e
        {
            data->buffer[count] = '\0'; // add a null terminator to the end of the string
            return;
        }
    }
}

void parseFields(USER_DATA *data)
{
    uint32_t i = 0; // to traverse array (MAX_CHARS)
    uint32_t j = 0; // to traverse array (MAX_FIELDS)
    bool flag1  = 0;
    bool flag2  = 0;
    data->fieldCount = 0;

    while (data->buffer[i] != '\0')
    {
        if (data->fieldCount == MAX_FIELDS) // If fieldCount = MAX_FIELDS, return from the function
        {
         return;
        }

        else if (((data->buffer[i] >= 'A' && data->buffer[i] <= 'Z') || (data->buffer[i] >= 'a' && data->buffer[i] <= 'z')) ) // ALPHABETS
        {
            if(flag1 == 0) // flag1 makes sure that fieldType, fieldPosition is updated once.
            {

                flag1 = 1;
                data->fieldType[j] = 'a';   // record the type of the field in type array
                data->fieldPosition[j] = i; // record offset of the field in position array
                j++;
                data->fieldCount++;         // Increment field count
            }

        }

        else if ((data->buffer[i] >= '0' && data->buffer[i] <= '9') ) // Numerics
        {
            if(flag2 == 0) // flag2 makes sure that fieldType, fieldPosition is updated once.
            {
                flag2 = 1;
                data->fieldType[j] = 'n';   // record the type of the field in type array
                data->fieldPosition[j] = i; // record offset of the field in position array
                j++;
                data->fieldCount++;         // Increment field count
             }
        }

        else // Anything that is not an alphabet or Numeric is a Delimiters ; Before returning, convert all delimiters in the string to NULL characters
        {
            flag1 = 0;                // reset flags if delimiters are detected.
            flag2 = 0;
            data->buffer[i] = '\0';
        }

        i++;
    }
}

int custom_stringcompare(const char* string1, const char* string2)
{
 while (*string1 && (*string1 == *string2))
 {
  string1++;
  string2++;
 }
  return (*string1 - *string2);
}

char* getFieldString (USER_DATA* data, uint8_t fieldNumber)
{

// In the buffer, at the field position index specified by the fieldNumber, return that string (character).

 return &data->buffer[data->fieldPosition[fieldNumber]];
}

uint32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
  // Returns the number if the string has number. If string has a character, it returns 0.

 return atoi(getFieldString(data,fieldNumber)); // Convert string(character) to integer value from getFieldString using atoi().

}

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{

 if( ! custom_stringcompare (&data->buffer[data->fieldPosition[0]], strCommand) ) // Comparing the strCommand argument and the first field [0] using custom_stringcompare
 {
     if(data->fieldCount >= minArguments) // and checking the number of arguments is greater than or equal to the requested number of minimum arguments
         return true;
 }

 else

     return false;
}
