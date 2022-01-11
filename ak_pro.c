//-----------------------------------------------------------------------------
// COURSE : EMBEDDED MICROCONTROLLERS (EE5314)
// AUTHOR : AKASH VIRENDRA
// UTA ID: 1001841349
// GUIDED BY DR.JASON LOSH
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red onboard LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green onboard LED:
//   PF3 drives an NPN transistor that powers the green LED
// Blue onboard LED:
//   PF2 drives an NPN transistor that powers the blue LED
//Switch1
//   PF4 configured in pull down mode
//RED LED FOR TRANSMISSION
//   PE2 drives an NPN transistor that powers the red LED
//GREEN LED FOR TRANSMISSION
//   PE1 drives an NPN transistor that powers the green LED
//DATA ENABLE
//   PB4 is used
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1 with FIFO disabled
//
//   U1TX (PB1) and U1RX (PB0) are connected to another microcontroller
//   Configured to 38400 baud, 8N1 with FIFO disabled and stick parity enabled
//
//
//          CHANNEL FUNCTIONALITIES
//          CHANNEL 1- SETS THE GREEN LED
//          CHANNEL 2- SETS THE BLUE  LED
//          CHANNEL 3- SETS THE RED LED
//          CHANNEL 4- RETURNS THE intensity VALUE OF THE PUSHBUTTON1
//          CHANNEL 5- RETURNS THE intensity VALUE OF THE GREEN LED
//          CHANNEL 6- RETURNS THE intensity  VALUE OF THE BLUE LED
//          CHANNEL 7- RETURNS THE intensity VALUE OF THE RED LED
//          CHANNEL 8- RGB CONTROL
//          CHANNEL 9- PULSE WAVEFORM
//          CHANNEL 10-SQUARE WAVEFORM
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "tm4c123gh6pm.h"
#include "wait.h"




// Bitband aliases
#define     GREEN  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) //PE1
#define     RED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))//PE2
#define     DEN   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4))) //data enable PB4

#define GREEN_OB PWM1_3_CMPB_R
#define RED_OB   PWM1_2_CMPB_R
#define BLUE_OB  PWM1_3_CMPA_R

//#define     RED_OB  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4))) //PF1
//#define     BLUE_OB (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) //PF2
//#define     GREEN_OB (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4))) //PF3
#define     SWITCH1 (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4))) //PF4

//-----------------------------------------------------------------------------
// MACROS
//-----------------------------------------------------------------------------
#define     RED_OB_MASK 2
#define     BLUE_OB_MASK 4
#define     GREEN_OB_MASK 8
#define     SWITCH1_MASK 16

#define MAX_CHARS 80
#define MAX_BUFF 40 //length of the buffer
#define MAX_FIELDS 20
#define MAX_COUNT 3
#define UART_TX_MASK 2
#define UART_RX_MASK 1
#define ENABLE_PB4  16

// PortF masks
#define GREEN_LED_MASK 2
#define RED_LED_MASK 4


//control commands
#define cmd_set 0x00
#define cmd_piecewise 0x01
#define cmd_pulse 0x02
#define cmd_square 0x03
#define cmd_sawtooth 0x04
#define cmd_triangle 0x05


//data commands
#define cmd_data_request 0x30
#define cmd_data_report 0x31
#define cmd_report_control 0x32

//UI commands
#define cmd_LCD_display 0x40
#define cmd_RGB 0x48
#define cmd_piecewise_RGB 0x49

//serial commands
#define cmd_UART_DATA 0x50
#define cmd_UART_CONTROL 0x51
#define cmd_I2C 0x54


//system commands
#define cmd_ack 0x70
#define cmd_poll_request 0x78
#define cmd_poll_response 0x79
#define cmd_set_address 0x7A
#define cmd_node_control 0x7D
#define cmd_bootload 0x7E
#define cmd_reset 0x7F

//boolean values
#define True 1
#define False 0

//-----------------------------------------------------------------------------
// Structure and variable definitions
//-----------------------------------------------------------------------------
typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

typedef struct _TX485msg
{
    uint8_t dstAdd;
    uint8_t cmd;
    uint8_t chan;
    uint8_t size;
    uint8_t data[10]; //set the array to some fixed amount i.e data[fixed]
    uint8_t seqID; //assigned when sendRS485 is called
                    //seq ID should be icnremnetd whenver theres an interrupt and  remember this is a modulo 256 incremnet
    uint8_t checkSum; //checksum is calculated when sendRS485 is called and put into the queue
    uint8_t timeToTransmit; //default is 0 , transmit when its zero i guess
    uint8_t tx_count;

    bool valid;// mark valid when RS485 is called
}TX485msg;

typedef struct _action
{
    uint8_t a_command;
    uint8_t val1;
    uint8_t val2;
    uint16_t time1;
    uint16_t time2;
    uint16_t cycles;
    uint8_t valid;
}act;

act action;



USER_DATA data;//defining the data variable as a global
bool flag=1;
bool Cs_Valid=0; //for carrier sense
bool Random_Valid=0; //for random
bool Ack_Valid=0; // for acknowledgement
uint8_t sourceAddress=0;
uint8_t destAddress;
uint8_t chanNumber;
uint8_t value[10];
//uint8_t data_v[10];
uint8_t pack_size=7;

TX485msg message[MAX_BUFF];//defining the array of structures represents each message

uint8_t seq=0; //assigning the sequence ID

char tempstring[100];

int8_t msg_in_progress=-1; //tells us which messaage are we currently processing
                            //which says invalid by default which in an indicator of saying there is  no message which is in the process of transmitting
uint8_t msg_in_phase=0; //within the message what is the byte number are we currently processing
                        //this basically tells which part sturcture we should send as in each iteration only one byte is send


uint8_t w_index=0,r_index=0; //write index and read index for the queue buffer
uint8_t msg_active=false;
uint8_t b_f=0; //this is for sendRS485byte() function for the for llop to send data

uint8_t rx_phase=0; //phase of the received data
uint8_t rx_data[50];
uint16_t d; //16 bit for receiving the data

//indexes for buffers for receiving
uint8_t wr=0;       //write index
uint8_t rr=0;       //read index

uint8_t rx_dest;
uint8_t rx_s;
uint8_t rx_seq;
uint8_t rx_cmd;
uint8_t rx_chan;
uint8_t rx_size;
uint8_t rx_d[10];
uint8_t rx_check;
uint8_t data_pd[10];
uint8_t  tx_led_timeout=0;
uint8_t  rx_led_timeout=0;


char str[100];
char u_data[100]; //this is the buffer for sendUImessage()
uint8_t u_tx_index=0;  //uart0 transmit index
uint8_t u_rx_index=0;       //uart0 receive index
uint16_t mbt=500;     //minimum backoff time
uint16_t T=1000;

bool busy=false;
uint8_t test_cs=2;
bool test_done=false;
bool re_tx=0;    //for retransmission

uint8_t val; //for considering the value of the switch

uint16_t p;
uint16_t seed;
bool tx_blink=0;
bool rx_blink=0;

bool sq=0;
bool pul=0;


//-----------------------------------------------------------------------------
// Function Declarations
//-----------------------------------------------------------------------------
void putcUart0(char c);
void putsUart0(char* str);
char getcUart0();
void getsUart0(USER_DATA* data);
void parseFields(USER_DATA* data);
char* getFieldString(USER_DATA* data, uint8_t fieldNumber);
uint16_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber);
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments);
void resetHw();
uint8_t strCompare(char*, const char*);
void sendRS485(uint8_t dstAdd, uint8_t cmd, uint8_t chan, uint8_t size, uint8_t data[], bool ack);
void processComand();
void sendRS485Byte();
void jumpToNextLine();
int power(int,int);
void checkSum();
void makeMessagesInvalid();
void greenLed();//toggling green led
void redLed();//toggle red led
void process_data();
void refreshTheMessage();
void waitMicrosecond(uint32_t);
void setSourceAddress();
//uint8_t checksum_check();
void sendUImessage(char str[]);
int number_of_digits(int);
char* i_to_a(int );
void setRgbColor(uint8_t, uint8_t, uint8_t);
uint16_t parity(uint16_t);
uint8_t random();

//-----------------------------------------------------------------------------
// Initialize Hardware
//-----------------------------------------------------------------------------
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks for UART1, UART0, porta, portb and porte
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1| SYSCTL_RCGCUART_R0; //enable clock for  uart 1 and uart 0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0 | SYSCTL_RCGCGPIO_R1| SYSCTL_RCGCGPIO_R4|SYSCTL_RCGCGPIO_R5; //enable clock for PORT B as UART 1 is is being used PB0 and PB1 and PB4 is used as DEN port
    //port A is uart0 and port E is used for the LED's, PORT F FOR ON BOARD LED AND SWITCH
    SYSCTL_RCGCEEPROM_R |= SYSCTL_RCGCEEPROM_R0;          //clock the EEPROM
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;              // turn-on pwm
    _delay_cycles(3);
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;



    //////configuration of EEPROM
    EEPROM_EESIZE_R |=0X00010000;
    EEPROM_EEBLOCK_R|=0X00;
    EEPROM_EEOFFSET_R|=0X00;



    // Configure  ON BOARD LED AND SWICTH pins on the port F pins
    GPIO_PORTF_DIR_R |= RED_OB_MASK|GREEN_OB_MASK|BLUE_OB_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DIR_R &=~SWITCH1_MASK;
    GPIO_PORTF_DR2R_R |= RED_OB_MASK|GREEN_OB_MASK|BLUE_OB_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_PDR_R |= SWITCH1_MASK;
    GPIO_PORTF_AFSEL_R |= RED_OB_MASK | GREEN_OB_MASK | BLUE_OB_MASK;
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M);
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;
    GPIO_PORTF_DEN_R |=RED_OB_MASK|GREEN_OB_MASK|BLUE_OB_MASK|SWITCH1_MASK;  // enable LEDs


    // Configure LED pins on the port e pins
    GPIO_PORTE_DIR_R |= GREEN_LED_MASK | RED_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTE_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R |= GREEN_LED_MASK | RED_LED_MASK;  // enable LEDs

    //configure PB4 GPI0 pin which is used as DEN


    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= UART_TX_MASK;                   // enable output on UART0 TX pin
    GPIO_PORTA_DIR_R &= ~UART_RX_MASK;                   // enable input on UART0 RX pin
    GPIO_PORTA_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
    GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7, first clear the bits
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX; //then make them as 1 and 1 as the value is 1 to use them as AF
    // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 ;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
    // enable TX, RX, and module
    UART0_IM_R=UART_IM_TXIM;    //UART_IM_RXIM or UART_IM_RXIM                         //interrupt is sent when the TX goes from not empty to empty
    NVIC_EN0_R|=1 << (INT_UART0-16); //enabling the interrupt for UART1
    //configure port B as uart1

    GPIO_PORTB_DIR_R |= UART_TX_MASK;                   // enable output on UART1 TX pin
    GPIO_PORTB_DIR_R &= ~UART_RX_MASK;                   // enable input on UART1 RX pin
    GPIO_PORTB_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART1 pins
    GPIO_PORTB_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PB0, PB1
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_M | GPIO_PCTL_PB0_M); // clear bits 0-7
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX;

    GPIO_PORTB_DIR_R |= ENABLE_PB4; //SET PB4 AS OUTPUT
    GPIO_PORTB_DR2R_R |=ENABLE_PB4;
    GPIO_PORTB_DEN_R |=ENABLE_PB4;

    // Configure UART1 to 38400 baud, 8N1 format
    UART1_CTL_R = 0;                                                                    // turn-off UART0 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                                                     // use system clock (40 MHz)
    UART1_IBRD_R = 65;                                                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART1_FBRD_R = 7;                                                                   // round(fract(r)*64)=45
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_SPS| UART_LCRH_PEN|UART_LCRH_EPS;                     // configure for 8N1 WITH FIFO DISABLED and stick parity selected and parity enabled. paly around with
                                                                                        //EPS TO CHANGE THE VALUE
                            //interrupt is sent when the TX goes from not empty to empty
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN | UART_CTL_EOT;         // enable TX, RX, and module

    UART1_IM_R |=UART_IM_TXIM | UART_IM_RXIM;    //UART_IM_RXIM or UART_IM_RXIM
    NVIC_EN0_R|=1 << (INT_UART1-16); //enabling the interrupt for UART1


    // Configure Timer 1 as the time base
//
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x61A80;                       // set load value to 40e5 for 100 Hz interrupt rate 0x61A80;
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer



    // Configure PWM module 1 to drive RGB LED
    // RED   on M1PWM5 (PF1), M1PWM2b
    // BLUE  on M1PWM6 (PF2), M1PWM3a
    // GREEN on M1PWM7 (PF3), M1PWM3b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_2_CTL_R = 0;                                // turn-off PWM1 generator 2 (drives outs 4 and 5)
    PWM1_3_CTL_R = 0;                                // turn-off PWM1 generator 3 (drives outs 6 and 7)
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
    // output 5 on PWM1, gen 2b, cmpb //whne counting down and when the count value matches the comparator value
    //the pwm signal goes high. when the load value is inserted again, the pwmb signal goes low again
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
    // output 6 on PWM1, gen 3a, cmpa
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
    // output 7 on PWM1, gen 3b, cmpb

    PWM1_2_LOAD_R = 256;                            // set frequency to 40 MHz sys clock / 2 / 256 = 78.125 kHz
    PWM1_3_LOAD_R = 256;

    PWM1_2_CMPB_R = 0;                               // red off (0=always low, 256=always high)
    PWM1_3_CMPB_R = 0;                               // green off
    PWM1_3_CMPA_R = 0;                               // blue off
//    PWM1_INVERT_R = PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;

    PWM1_2_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 2
    PWM1_3_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 3
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
    // enable outputs
}



//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


uint16_t parity(uint16_t x)
{
    while(x)
    {
        p=!p;
        x=x&(x-1);
    }
    return p;
}

uint8_t random()
{
    uint16_t temp=0;
    uint16_t char_poly=0b1000000000000011;
    temp=seed&char_poly;
    seed=(parity(temp)<<15)+(seed>>1);
       return temp+sourceAddress;
}

int number_of_digits(int n)
{
    int dc = 0;

    while(n > 0)
    {
        dc++;
        n /= 10;
    }

    return dc;
}


char* i_to_a(int n)
{
    int dc = 0;

    if(n < 0)
    {
        n = -1*n;
        dc++;
    }

    dc += number_of_digits(n);

    str[dc] = '\0';

    while(n > 0)
    {
        str[dc-1] = n%10 + 48;
        n = n/10;
        dc--;
    }

    if(dc == 1)
        str[0] = '-';

    return str;
}
void u_data_clear()
{
    int i=0;
    while(u_data[i]!='\0')
        u_data[i++]='\0';
}

void uart0Isr()
{
    while(u_tx_index!=u_rx_index)
    {
        while(UART0_FR_R & UART_FR_BUSY);
        UART0_DR_R=u_data[u_rx_index];
        u_rx_index=(u_rx_index+1)%100;
        if(u_tx_index==u_rx_index)
        {
           waitMicrosecond(1000);

            UART0_DR_R=u_data[u_rx_index];
        }
    }
    u_rx_index=0;
    u_tx_index=0;
    u_data_clear();
    UART0_ICR_R|=UART_ICR_TXIC;

}

void sendUImessage(char str[])
{
    while(str[u_tx_index]!='\0')
    {
        u_data[u_tx_index]=str[u_tx_index];
        if((u_tx_index+1)%100!=u_rx_index)
        {
            if(str[u_tx_index+1]=='\0')
                break;
            u_tx_index=(u_tx_index+1)%100;
        }


    }

     if(UART0_FR_R & UART_FR_TXFE)
     {
         while(UART0_FR_R & UART_FR_BUSY);
         UART0_DR_R=u_data[u_rx_index++];
         waitMicrosecond(1000);
     }
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
}

void getsUart0(USER_DATA* data)
{
    unsigned int ct=0;  //count
    char c;
//    goto entry;
//    entry:
//    c[ct++]=getcUart0();
// if(ct>0 && (c[ct-1]==8 || c[ct-1]==128]))
//    {
//      ct--;
//      goto entry;
//    }
    while(ct!=MAX_CHARS)
    {
        c=getcUart0();
        if(c>=32 && c<127)
            data->buffer[ct++]=c;
        else if(c==8 || c==127)
        {
            if(ct>0)
                ct--;
            else
                continue;
        }
        else if(ct==(MAX_CHARS) || (c==10 || c==13))
        {
            data->buffer[ct]='\0';
            break;
        }

    }
}
// Blocking function that writes a string when the UART buffer is not full


void setRgbColor(uint8_t red, uint8_t blue, uint8_t green)
{
    PWM1_2_CMPB_R = red;
    PWM1_3_CMPA_R = blue;
    PWM1_3_CMPB_R = green;
}
void greenLed()
{
    GREEN=1;
    waitMicrosecond(100000);
    GREEN=0;
    waitMicrosecond(100000);
}
void redLed()
{
    RED=1;
    waitMicrosecond(1000000);
    RED=0;
    waitMicrosecond(1000000);
}
void setSourceAddress()
{
    EEPROM_EEOFFSET_R=0x00;
    //write the new address to offset 0
    if (EEPROM_EERDWR_R == 0XFFFFFFFF)
    {
        //EEPROM_EEOFFSET_R=0x00;
        sourceAddress=0x01;
    }
    else
        sourceAddress=EEPROM_EERDWR_R;
}


void resetHw()
{
    NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;// or do  NVIC_APINT_R = 0x05FA0004
}

void jumpToNextLine()
{
    putcUart0('\r');
    putcUart0('\n');
}
void parseFields(USER_DATA* data)
{
    int i=0,j=0;  //i used for normal string array and j used for fieldtype which can be a max of 5
    data->fieldPosition[j]=i;
    data->fieldCount=0;
    (data->fieldCount)++;
    while(data->buffer[i]!='\0')
    {
        if( (data->buffer[i]>=65 && data->buffer[i]<=90) || (data->buffer[i]>=97 && data->buffer[i]<=122))
            data->fieldType[j]='a';
        else if((data->buffer[i]>=48 && data->buffer[i]<=57) || data->buffer[i]==46 || data->buffer[i]==45)
            data->fieldType[j]='n';
        else
        {
            data->buffer[i]='\0';
            data->fieldPosition[j+1]=i+1;
            (data->fieldCount)++;
            j++;
        }

        i++;
        if(j>=MAX_FIELDS)
        {

            flag=0;
            return;
        }
    }
}
char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber<MAX_FIELDS)
    {
        if(data->fieldType[fieldNumber]=='a')
            return &(data->buffer[data->fieldPosition[fieldNumber]]);
        else
        {
            putsUart0("\nthe word in the alphabet is not a alphabet");
            return NULL;
        }
    }
    putsUart0("\ninvalid number of arguments");

    return NULL;
}


void makeMessagesInvalid()
{
    int i;
    for(i=0;i<MAX_BUFF;i++)
    {
        message[i].valid=0;
    }
}

void refreshTheMessage()
{
    int i;
    message[w_index].dstAdd=0;
    message[w_index].cmd=0;
    message[w_index].chan=0;
    message[w_index].size=0;
    message[w_index].seqID=0;
    message[w_index].checkSum=0;
    message[w_index].timeToTransmit=0;
    message[w_index].tx_count=0;
    for(i=0;i<10;i++)
    {
        message[w_index].data[i]=0;
    }
}

int power(int base,int exp)
{
    int i;
    int result=1;
    if(exp==0)
        return 1;
    else
    {
        for(i=1;i<=exp;i++)
            result=result*base;
        return result;
    }
}
uint16_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    int i=data->fieldPosition[fieldNumber];
    int result=0,temp=0,num=i,j,inc=0; //result is to store the final integer value
    //temp is used to store the intermediate value
    //num is like to counter to go till the end of the number and then to convert it
    //j is used for for lopp
    if(fieldNumber<MAX_FIELDS)
    {
        if(data->fieldType[fieldNumber]=='n')
        {
            while(data->buffer[i]!='\0')
            {
             i++;
            }

         for(j=i-1;j>=num;j--)
         {
             temp=(data->buffer[j])-48;
             result=result+temp*power(10,inc++);
         }
         return result;
        }
        else
        {
            putsUart0("not a integer string, cannot convert it");
            return 0;
        }
    }
    putsUart0("\ninvalid number of arguments");
    return 0;
}

uint8_t strCompare(char* str1, const char* str2)
{
    int i=0;
    while(str1[i]!='\0' || str2[i]!='\0')
    {
        if(str1[i]==str2[i])
            i++;
        else
            return 0;
    }
    return 1;
}

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    int i=data->fieldPosition[0];
    char* str=&(data->buffer[data->fieldPosition[0]]);
    int f=1;

        while(str[i]!='\0' && f!=0)
        {
            if(str[i]==strCommand[i])
            {
                i++;
            }
            else
            {
                f=0;
                return 0;
            }
        }
        if(((data->fieldCount)-1)!=minArguments)
            return 0;
        return 1;
}


void sendRS485(uint8_t dstAddress, uint8_t command, uint8_t channel, uint8_t Size, uint8_t Data[], bool ack)
{
    int i=0,sum=0,pack_size=7;

    if(((w_index+1)%MAX_BUFF)==r_index)
    {
        sendUImessage("BUFFER FULL");
        //putsUart0("BUFFER FULL");
        return;
    }
    while(r_index<=w_index && message[w_index].valid!=0)
    {
        w_index=(w_index+1)%MAX_BUFF;
        if(r_index==w_index)
        {
            //sendUImessage("NO VALID MESSAGES IN THE QUEUE");
            putsUart0("NO VALID MESSAGES IN THE QUEUE");
            return;
        }
    }

        if((message[w_index].valid)==0)
        {
            refreshTheMessage();
            message[w_index].dstAdd=dstAddress;
            message[w_index].cmd=command;
            if(ack==1 && (command!=cmd_ack))                 //to prevent ack boom
                message[w_index].cmd=(command|0x80);
            message[w_index].chan=channel;
            message[w_index].size=Size;
            for(i=0;i<Size;i++)
            {
                message[w_index].data[i]=Data[i];
                sum=Data[i]+sum;
            }
            message[w_index].seqID=(seq++)%MAX_BUFF;
            message[w_index].checkSum=~(message[w_index].dstAdd + sourceAddress + message[w_index].cmd + message[w_index].chan + message[w_index].seqID + message[w_index].size + sum);

            message[w_index].timeToTransmit=0;
            message[w_index].tx_count=0;
            message[w_index].valid=1;
            if( message[w_index].size==0)
                pack_size=7;
            else
                pack_size=pack_size + message[w_index].size-1;
            if(UART1_FR_R & UART_FR_TXFE) //then we send rs485() byte and clears TX fifo bit
                sendRS485Byte();
            w_index=(w_index+1)%MAX_BUFF;
        }
        else
        {
            //sendUImessage("ERROR\n\r");
            putsUart0("ERROR\n\r");

        }
}


//uint8_t checksum_check()
//{
//    uint8_t sum,i;
//    if(rx_data[5]==0)
//    {
//        sum=~(rx_data[0]+rx_data[1]+rx_data[2]+rx_data[3]+rx_data[4]+rx_data[5]+rx_data[6]);
//        return sum;
//    }
//    for(i=0;i<rx_data[5];i++)
//        sum=sum+rx_data[6+i];
//    sum=~(rx_data[0]+rx_data[1]+rx_data[2]+rx_data[3]+rx_data[4]+rx_data[5]+sum);
//    return sum;
//}

void process_data()
{
    uint8_t i,sum=0;
    rx_dest=rx_data[0];   //my address
    rx_s=rx_data[1];      //transmitter address
    rx_seq=rx_data[2];
    rx_cmd=rx_data[3];
    rx_chan=rx_data[4];
    rx_size=rx_data[5];
    if(rx_size==0)
    {
        rx_d[0]=0x00;
        rx_check=rx_data[6];
    }
    else
    {
        for(i=6;i<6+rx_size;i++)
        {
            rx_d[i-6]=rx_data[i];
            sum=sum+rx_data[i];
        }
        rx_check=rx_data[i];
    }
    sum=~(rx_dest+rx_s+ rx_seq+rx_cmd+rx_chan+rx_size+sum);

    if(sum==rx_check)
    {
        GREEN=1;
        rx_blink=1;
        rx_led_timeout=200;
        if(rx_cmd & 0x80)
        {
            data_pd[0]=rx_seq;
            sendRS485(rx_s,cmd_ack,0x00,0x01,&data_pd[0],Ack_Valid);

        }
        if(rx_cmd==cmd_set || rx_cmd==0x80)
        {
            if(rx_chan==0x01)  //set green color
            {
                setRgbColor(0,0,rx_d[0]);
            }
            else if(rx_chan==0x02)  //set blue color
            {
                setRgbColor(0,rx_d[0],0);
            }
            else if(rx_chan==0x03)   //set red color
            {
                setRgbColor(rx_d[0],0,0);
            }
        }
        else if(rx_cmd==cmd_data_request || rx_cmd==0xB0) //get
        {
            if(rx_chan==4)    //value of the switch
            {
                data_pd[0]=SWITCH1;
                sendRS485(rx_s,cmd_data_report,rx_chan,0x01,&data_pd[0],Ack_Valid);
            }
            else if(rx_chan==5)      //value of the green led
            {
                data_pd[0]=PWM1_3_CMPB_R;
                sendRS485(rx_s,cmd_data_report,rx_chan,0x01,&data_pd[0],Ack_Valid);
            }
            else if(rx_chan==6)      //value of the blue led
            {
                data_pd[0]=PWM1_3_CMPA_R;
                sendRS485(rx_s,cmd_data_report,rx_chan,0x01,&data_pd[0],Ack_Valid);
            }
            else if(rx_chan==7)      //value of the red led
            {
                data_pd[0]=PWM1_2_CMPB_R;
                sendRS485(rx_s,cmd_data_report,rx_chan,0x01,&data_pd[0],Ack_Valid);
            }
        }
        else if(rx_cmd==cmd_data_report || rx_cmd==0xB1)
        {
//            sendUImessage("the value of is:");
//            sendUImessage(i_to_a(rx_d[0]));
//            sendUImessage("\r\n");
            data_pd[0]=rx_d[0];
            sprintf(tempstring,"value of chan %d is %d\n\r",rx_chan,data_pd[0]);
            putsUart0(tempstring);

        }
        else if(rx_cmd==cmd_poll_response || rx_cmd==0xF9)
        {
//            sendUImessage("poll response received from address:");
//            sendUImessage(i_to_a(rx_s));
//            sendUImessage("\r\n");
            data_pd[0]=rx_s;
            sprintf(tempstring,"poll received from address: %d\n\r",data_pd[0]);
            putsUart0(tempstring);
        }
        else if(rx_cmd==cmd_poll_request || rx_cmd==0xF8)
        {
            putsUart0("polling\n\r");
            data_pd[0]=sourceAddress;
            sendRS485(rx_s,cmd_poll_response,rx_chan,0x01,&data_pd[0],Ack_Valid);
        }
        else if(rx_cmd==cmd_reset)
        {
            putsUart0("resetting device\n\r");
            waitMicrosecond(10000);
            resetHw();
        }
        else if(rx_cmd==cmd_set_address || rx_cmd==0xFA)
        {
            EEPROM_EEOFFSET_R = 0;
            EEPROM_EERDWR_R = rx_d[0];
            sourceAddress=rx_d[0];
//            sendUImessage("address changed to:");
//            sendUImessage(i_to_a(rx_d[0]));
//            sendUImessage("\r\n");
            sprintf(tempstring,"address changed to:%d\n\r",sourceAddress);
            putsUart0(tempstring);
        }
        else if(rx_cmd==cmd_RGB || rx_cmd==0xC8)
        {
            if(rx_chan==0x08)
            {
                data_pd[0]=rx_data[0];
                data_pd[1]=rx_data[1];
                data_pd[2]=rx_data[2];
                setRgbColor(data_pd[0],data_pd[1],data_pd[2]);
            }

        }
        else if(rx_cmd==cmd_ack)
        {
            putsUart0("ack received\n\r");
            message[r_index].valid=false;
            r_index=(r_index+1)%MAX_BUFF;

        }
        else if(rx_cmd==cmd_square || rx_cmd==0x83)
        {
            if(rx_chan==10)
            {
                action.a_command=cmd_square;
                action.val1=rx_data[0];
                action.val2=rx_data[1];
                action.time1=(rx_data[2]*0x100) | rx_data[3];
                action.time2=(rx_data[4]*0x100) | rx_data[5];
                action.cycles=(rx_data[6]*0x100) | rx_data[7];
                RED_OB=action.val1;
                action.valid=1;

            }
        }
        else if(rx_cmd==cmd_pulse || rx_cmd==0x82)
        {
            if(rx_chan==9)
            {
                action.a_command=cmd_pulse;
                action.val1=rx_data[0];
                action.time1=(rx_data[1]*0x100) | rx_data[2];
                GREEN_OB=action.val1;
                action.valid=1;

            }

        }
    }
    else
    {
        sendUImessage("INVALID CHECKSUM \n\r");
        RED=1;

    }
}

void uart1Isr()
{

//IM SET FOR TX EMPTY
    //I THINK YOU HAVE TO USE THE QUEUE HERE. BASED ON WHICH MESSAGE IS INVALID YOU HAVE TO SEND THE MESSAGES IN THAT WAY
    if(msg_in_progress!=-1 && message[r_index].valid==1 )
    { //IF(TXEMPTY), then call sendRS485byte() and and clear TXfifo bit
        if(!(UART1_FR_R & UART_FR_BUSY))
        {
            sendRS485Byte();
            UART1_ICR_R|=UART_ICR_TXIC;
             //CLEARING THE TRANSMIT ineterrupt BIT
        }
    }

    if(!(UART1_FR_R & UART_FR_RXFE)  && (UART1_RIS_R & UART_RIS_RXRIS))
    {
        pack_size=7;
        busy=true;
        d=UART1_DR_R & 0x0FFF;
        if(rx_phase==0 && (d & 0x0200) && ((d&0xFF)==0xFF || ((d & sourceAddress)==sourceAddress)))
        {
            rx_data[rx_phase++]=d;

        }
        else if(rx_phase>0 && !(d & 0x0200))
        {
            rx_data[rx_phase++]=d;

        }
        if(rx_phase==(pack_size+rx_data[5]))  //&&might be there, some other condition
        {
            rx_phase=0;
            process_data();
            busy=false;
        }
        UART1_ICR_R|=UART_ICR_RXIC;  //if this does not work try adding it to the top
    }
    else
        UART1_ICR_R|=UART_ICR_RXIC;
}
void sendRS485Byte()
{
    if(msg_in_progress==-1 || msg_active==true)
    {
        while(message[r_index].valid==0 && r_index<=w_index)
                {
            r_index=(r_index+1)%MAX_BUFF;
            if(r_index==w_index)
                sendUImessage("no valid messages in the queue\n\r");
                }
        if (Cs_Valid && msg_in_phase==0)// this check should be done at the start of the new message                                                           // Checking for carrier sense
        {
            while((UART1_FR_R & UART_FR_RXFE))
            {
                busy=false;
                test_done=false;
                test_cs=2;
                if(test_done)
                {
                    test_done=false;
                    if(!busy)
                        goto write;
                    else
                    {
                        test_done=false;
                        busy=false;
                        test_cs=2;
                    }
                }
            }
        }
        else                 //if cs is off just transmit the data
        {
            write:
            msg_in_progress=r_index;
            DEN = 1;
            msg_active=true;
            if(msg_in_phase==0)
            {
                while(!(UART1_FR_R & UART_FR_TXFE)); //while transmitter not empty keep looping in a while loop. once empty go do the following
                UART1_LCRH_R &= ~UART_LCRH_EPS;
                msg_in_phase++;
                UART1_DR_R = message[r_index].dstAdd;
            }
            else if(msg_in_phase==1)
            {
                while(!(UART1_FR_R & UART_FR_TXFE));
                UART1_LCRH_R |= UART_LCRH_EPS;
                msg_in_phase++;
                UART1_DR_R = sourceAddress;
            }
            else if(msg_in_phase==2)
            {
                while(!(UART1_FR_R & UART_FR_TXFE));
                UART1_LCRH_R |= UART_LCRH_EPS;
                msg_in_phase++;
                UART1_DR_R = message[r_index].seqID;
            }
            else if(msg_in_phase==3)
            {
                while(!(UART1_FR_R & UART_FR_TXFE));
                UART1_LCRH_R |= UART_LCRH_EPS;
                msg_in_phase++;
                UART1_DR_R = message[r_index].cmd;
            }
            else if(msg_in_phase==4)
            {
                while(!(UART1_FR_R & UART_FR_TXFE));
                UART1_LCRH_R |= UART_LCRH_EPS;
                msg_in_phase++;
                UART1_DR_R = message[r_index].chan;
            }
            else if(msg_in_phase==5)
            {
                while(!(UART1_FR_R & UART_FR_TXFE));
                UART1_LCRH_R |= UART_LCRH_EPS;
                msg_in_phase++;
                UART1_DR_R = message[r_index].size;
            }
            else if(msg_in_phase>=6 && msg_in_phase<(pack_size+message[r_index].size))
            {
                while(!(UART1_FR_R & UART_FR_TXFE));
                UART1_LCRH_R |= UART_LCRH_EPS;
                if(message[r_index].size==0 && msg_in_phase==6)
                {
                    msg_in_phase=100;

                    UART1_DR_R=message[r_index].checkSum;
                }
                else if(b_f!=message[r_index].size)
                {

                    msg_in_phase++;
                    UART1_DR_R=message[r_index].data[b_f];
                    b_f++;
                }
                else
                {
                    msg_in_phase++;
                    UART1_DR_R=message[r_index].checkSum;
                }

            }
            else
            {
                sprintf(tempstring,"Queuing Msg %d \n\r",message[r_index].seqID);
                putsUart0(tempstring);
                msg_in_phase=0;
                msg_active=false;
                b_f=0;
                msg_in_progress=-1;

                RED=1;
                tx_blink=false;
                tx_led_timeout=200;
                while(UART1_FR_R & UART_FR_BUSY); //once not busy only then make the DEN=0
                waitMicrosecond(10000);//so that data has left the digital enable
                DEN=0;


                sprintf(tempstring,"Transmitting Msg %d, Attempt %d\n\r",message[r_index].seqID,message[r_index].tx_count);
                putsUart0(tempstring);
                //                sendUImessage(tempstring);

                if(!Ack_Valid || message[r_index].cmd==0x70) //if  you are sending aan ack message
                {
                    if((message[r_index].cmd==0x70))
                    {
                        //sendUImessage("ACK SENT \n\r");
                        putsUart0("ACK SENT \n\r");
                    }
                    message[r_index].valid=0;
                    r_index=(r_index+1)%MAX_BUFF;
                }
                else
                {
                    message[r_index].tx_count++;
                    if(message[r_index].tx_count==MAX_COUNT)
                    {
                        sprintf(tempstring,"message with seq ID:%d, failed to send \n\r",message[r_index].seqID);
                        //sendUImessage(tempstring);
                        putsUart0(tempstring);
                        message[r_index].valid=0;
                        re_tx=0;
                        tx_blink=1;
                        RED=1; //will be cleared when you transmit another message
                        r_index=(r_index+1)%MAX_BUFF;
                    }
                    else
                    {
                        if(!Random_Valid)
                        {
                            re_tx=1;
                            message[r_index].timeToTransmit=mbt+power(2,message[r_index].tx_count)*T;
                        }
                        else
                        {
                            re_tx=1;
                            message[r_index].timeToTransmit=mbt+random()*T;
                        }
                    }
                }

            }
        }
    }
}

void timer1Isr()
{
    if(message[r_index].valid!=0)
    {
        if(test_cs>0 && Cs_Valid)
        {
            test_cs--;
            if(test_cs==0)
            {
                test_done=true;
                busy=false;     //have a doubt in this
            }
        }
    }

    if(!tx_blink)
    {
        if(tx_led_timeout>0)
        {
            tx_led_timeout--;
            if(tx_led_timeout==0)
            {
                tx_blink=true;
                RED=0;
            }
        }
    }
    if(rx_blink)
    {
        if(rx_led_timeout>0)
        {
            rx_led_timeout--;
            if(rx_led_timeout==0)
            {
                rx_blink=0;
                GREEN=0;
            }
        }
    }
    if(message[r_index].valid==1 && re_tx)
    {
        if(message[r_index].timeToTransmit>0)
        {
            message[r_index].timeToTransmit--;
            if(message[r_index].timeToTransmit==0)
            {
                if((UART1_FR_R & UART_FR_TXFE) && !(UART1_RIS_R & UART_RIS_TXRIS)) //dk if this should be here
                    sendRS485Byte();
            }
        }
    }

    if(action.valid)
    {
        if(action.a_command==cmd_pulse)
        {   //after finishing make valid and variable as 0
            if(action.time1>0)
                action.time1--;
            if(action.time1==0)
            {
                GREEN_OB=0;
                action.valid=0;
                pul=0;
            }
        }
        else if(action.a_command==cmd_square)
        {
            if(action.cycles>0)
            {
                if(RED_OB==action.val1)
                {
                    if(action.time1>0)
                        action.time1--;
                    if(action.time1==0)
                    {
                        RED_OB =action.val2;
                        action.time1=(rx_data[2]*0x100) | rx_data[3];
                    }

                }
                if(RED_OB==action.val2)
                {
                    if(action.time2>0)
                        action.time2--;
                    if(action.time2==0)
                    {
                        RED_OB=action.val1;
                        action.time2=(rx_data[4]*0x100) | rx_data[5];
                        if(action.cycles>0)
                            action.cycles--;
                    }
                }

            }
            else
            {
                RED_OB=0;
                action.valid=0;
                sq=0;
            }

        }

    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

void processCommand()
{
    bool valid=false; // this variable is used to check if the command inputed is correct or not
    if(isCommand(&data,"reset",0))
    {
        sprintf(tempstring,"reset issued on our uc\n\r"); //ADD SOURCE ADDRESS
        sendUImessage(tempstring);

        valid=true;
        resetHw();
    }
    else if(isCommand(&data,"reset",1))                  ////resetting target uc
    {
        sendUImessage("RESET initiated on the remote uc\n\r");
        uint8_t destAddress=getFieldInteger(&data,1); //dk what the size of the variable is. might be 16 bit also depending upon the size of the address
        valid=true;
        sendRS485(destAddress,cmd_reset,0x00,0x00,0x00,Ack_Valid);
    }
    else if(isCommand(&data,"cs",1))  //carrier sense
    {
        if(strCompare(getFieldString(&data, 1),"on"))
        {
            sendUImessage("CS enabled\n\r");
            Cs_Valid=true;
            valid=true;
        }
        else if(strCompare(getFieldString(&data, 1),"off"))
        {
            sendUImessage("CS disabled\n\r");
            //jumpToNextLine();
            Cs_Valid=false;
            valid=true;
        }
    }
    else if(isCommand(&data,"random",1))   //random retransmission code
    {
        if(strCompare(getFieldString(&data, 1),"on"))
        {
            sendUImessage("random transmission enabled\n\r");
            //jumpToNextLine();
            Random_Valid=true;
            valid=true;
        }
        else if(strCompare(getFieldString(&data, 1),"off"))
        {
            sendUImessage("random transmission disabled\n\r");
            //jumpToNextLine();
            Random_Valid=false;
            valid=true;
        }
    }
    else if(isCommand(&data,"set",3))
    {
        sendUImessage("setting address, channel and value\n\r");
       // jumpToNextLine();
        destAddress = getFieldInteger(&data, 1);
        chanNumber = getFieldInteger(&data, 2);
        value[0] = getFieldInteger(&data, 3);
        valid=true;
        sendRS485(destAddress,cmd_set,chanNumber,0x01,&value[0],Ack_Valid);
    }
    else if(isCommand(&data,"get",2))
    {
        sendUImessage("requesting the data\n\r");
        //jumpToNextLine();
        destAddress = getFieldInteger(&data, 1);
        chanNumber = getFieldInteger(&data, 2);
        valid=true;
        sendRS485(destAddress,cmd_data_request,chanNumber,0x00,0x00,Ack_Valid);
    }
    else if(isCommand(&data,"poll",0))
    {
        sendUImessage("polling the devices\n\r");
        //jumpToNextLine();
        valid=true;
        sendRS485(0xFF,cmd_poll_request,0x00,0x00,0x00,Ack_Valid);
        //ALSO ADD IF YOU SEND POLL ADDRESS, THEN THE POLL MESSAGE ONLY GOES TO THAT DEVICE

        //isCommand(&data,"poll",1) BUT THIS IS NOT IN THE PROJECT MODULE
    }
    else if(isCommand(&data,"sa",2))
    {
        sendUImessage("setting new address\n\r");
        //jumpToNextLine();
        destAddress = getFieldInteger(&data, 1);
        value[0]=getFieldInteger(&data, 2);
        valid=true;
        sendRS485(destAddress,cmd_set_address,0x00,0x01,&value[0],Ack_Valid);
    }
    else if(isCommand(&data,"ack",1))
    {
        if(strCompare(getFieldString(&data, 1), "on"))
        {
            sendUImessage("ack enabled\n\r");
            //jumpToNextLine();
            Ack_Valid=true;
            valid=true;
        }
        else if(strCompare(getFieldString(&data, 1),"off"))
        {
            sendUImessage("ack disabled\n\r");
           // jumpToNextLine();
            Ack_Valid=false;
            valid=true;
        }
    }
    else if(isCommand(&data,"rgb",5))
    {
        sendUImessage("setting rgb lights\n\r");
        //jumpToNextLine();
        destAddress = getFieldInteger(&data, 1);
        chanNumber = getFieldInteger(&data, 2);
        value[0]=getFieldInteger(&data, 3);
        value[1]=getFieldInteger(&data, 4);
        value[2]=getFieldInteger(&data, 5);
        valid=true;
        sendRS485(destAddress,cmd_RGB,chanNumber,0x03,&value[0],Ack_Valid);
    }
    else if(isCommand(&data,"square",7))
    {
        sendUImessage("square wave generation\n\r");
        destAddress = getFieldInteger(&data, 1);
        chanNumber = getFieldInteger(&data, 2);
        value[0]=getFieldInteger(&data, 3);    //value0
        value[1]=getFieldInteger(&data, 4);     //value1
        uint8_t temp= (getFieldInteger(&data, 5))>>8; //time1
        value[2]=temp;
        value[3]=(getFieldInteger(&data, 5)) & 0xFF;
        uint8_t temp1= (getFieldInteger(&data, 6))>>8; //time2
        value[4]=temp1;
        value[5]=(getFieldInteger(&data, 6)) & 0xFF;
        uint8_t temp2= (getFieldInteger(&data, 7))>>8; //cycles
        value[2]=temp2;
        value[7]=(getFieldInteger(&data, 7)) & 0xFF;
        valid=true;
        sendRS485(destAddress,cmd_square,chanNumber,0x08,&value[0],Ack_Valid);
    }
    else if(isCommand(&data,"pulse",4))
    {
        sendUImessage("setting pulse lights\n\r");
        destAddress = getFieldInteger(&data, 1);
        chanNumber = getFieldInteger(&data, 2);
        value[0]=getFieldInteger(&data, 3);    //value0
        uint8_t temp= (getFieldInteger(&data, 4))>>8; //time1
        value[1]=temp;
        value[2]=(getFieldInteger(&data, 4)) & 0x00FF;
        valid=true;
        sendRS485(destAddress,cmd_pulse,chanNumber,0x03,&value[0],Ack_Valid);
    }
    if (!valid)
    {
        sendUImessage("Invalid command\n\r");
    }
}

int main(void)
{
    initHw();


   sendUImessage("READY\n\r");
   //TESTING THE LED'S
//    greenLed();
//    redLed();
   seed=TIMER1_TAR_R;

    setSourceAddress();
    sprintf(tempstring,"DEVICE: %u\n\r",sourceAddress); //ADD SOURCE ADDRESS
    sendUImessage(tempstring);
    makeMessagesInvalid(); //MAKE ALL THE MESSAGES AS INVALID

    start:
    while(1)
    {
        getsUart0(&data);
        sendUImessage("\n\r");
        parseFields(&data); // Parse fields
        if(flag==0)//if the string is longer than 80 characters
        {
            sendUImessage("string is longer than 80 characters. Try again\n\r");

            goto start;
        }
        processCommand();
    }
}


