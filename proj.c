//Embedded Project
//5314
//Peer to peer communication
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------
// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Hardware configuration:

// Board Red LED: PF1 drives an NPN transistor that powers the red LED
// Board Blue LED: PF2 drives an NPN transistor that powers the blue LED
// Board Green LED: PF3 drives an NPN transistor that powers the green LED
// Board PushButton1: PF4 drives an NPN transistor that powers the blue LED

// HW Red Led :PA7 powers the red LED when the process of sending data is happening on the wire or
//             when data has been already send for maximum number of times
// HW Red Led :PA7 powers the red LED when the process of receiving data is happening on the wire or
//             when data has been received with wrong checksum
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 1,15200 baud, 8N1
//UART1 FOR COMMUNICATION
// UART
// Timer Mode
//   FREQ_IN (WT5CCP0) on PD6
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "tm4c123gh6pm.h"


#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define Board_RED_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define Board_GREEN_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PushButton1     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))


#define Max_Char 80
#define MAX_RETRANSMIT 5
#define Max_Msg 25 // Maximum message size
#define Max_Data 50 //Maximum data size
#define Uart1_D_Enab (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0X40000000)*32 + 6*4)))// Data Enable bit

// Command and their address
#define SET_cmd 0x00
#define GET_cmd 0x20
#define GET_RESPONSE_cmd 0x21
#define SA 0x7A
#define ACK 0x70
#define POLL_REQUEST 0x78
#define POLL_RESPONSE 0x79
#define RESET 0x7F
#define RGB 0X48
#define UART_data 0x50
#define PULSE 0x02

uint8_t MessageNumber=0;
uint8_t AttemptNumber=0;
uint8_t CurrentIndex=0;
uint8_t CurrentPhase=0;
uint8_t RecievePhase=0;
uint8_t Old_RecievePhase=0;
uint8_t Source_Add=36;
uint8_t Retransmission_Power=0;
uint8_t bytesIndex=0;
uint8_t Seq_Id=0;
uint8_t dataBytesLong[Max_Data];
uint8_t FinalData[75];
uint8_t ReceivedData[75];
uint8_t r_ReceivedData[75];
uint8_t Source_Address[Max_Msg];
uint8_t Destination_Address[Max_Msg];
uint8_t Sequence_Id[Max_Msg];
uint8_t Command_Excute[Max_Msg];
uint8_t Channel[Max_Msg];
uint8_t Size_[Max_Msg];
uint8_t Checksum[Max_Msg];
uint8_t Data_Excute[Max_Msg][Max_Data];
uint8_t Retransmission_Count[Max_Msg];
uint16_t Retransmit_timeout[Max_Msg];
bool Valid[Max_Msg];
bool Received_Ack[Max_Msg];
bool inProgress=false;
bool checkCommand_flag=false;
bool  PWM_FLAG=false;//to select high and low section of pwm
uint16_t PWMTime=0;
uint8_t localIndex=0;



char str[Max_Char];
char type[10]={'\0'};
uint8_t position[10]={0};
uint8_t fields=0;
uint8_t count,addr,ch,value_,r_ch,r_value,g_timeout,r_timeout,d_timeout;
bool CS_Enable,ACK_Enable,Random_Enable;


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz, use PWM and divide by 2
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S) | SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A,F,C and D peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD ;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R |= 0x0E;                        // make bit 1,2 and 3 an outputs
    GPIO_PORTF_DR2R_R |= 0x1E;                       // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= 0x1E;                        // enable LED and push button
    GPIO_PORTF_PUR_R = 0x10;                         // enable internal pull-up for push button

    //Configure POARTA
    GPIO_PORTA_DIR_R |= 0xC0;                        // make bit 6 and 7 an outputs
    GPIO_PORTA_DR2R_R |= 0xC0;                       // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= 0xC0;                        // enable LED

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure UART1 pins
    GPIO_PORTC_DIR_R |= 0x40;                        // make bit 6 an outputs
    GPIO_PORTC_DR2R_R |= 0x40;                       // set drive strength to 2mA (not needed since default configuration -- for clarity)
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1, leave other uarts in same status
    GPIO_PORTC_DEN_R |= 0x70;                        // Digital Enable default, added for clarity
    GPIO_PORTC_AFSEL_R |=0x30;                       // Select the function as UART1 for Port C
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_U1RX | GPIO_PCTL_PC5_U1TX; // Enable Tx and Rx of Port C

    // Configure UART1 to 38400 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART1_CTL_R = 0;                                 // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART1_IBRD_R = 65;                               // r = 40 MHz / (N * 38400Hz), where N=16
    UART1_FBRD_R = 7;                                // int(fract(r)*64)=45
    UART1_LCRH_R = UART_LCRH_SPS | UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_PEN; // configure for word length 8,16-level FIFO, Parity Enable, Sticky Parity
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure Timer 1 as the time base
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x9C40;                         // set load value to 40e6 for 1k Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

    //Configure M1PWM5, M1PWM6, M1PWM7 of Port F
    GPIO_PORTF_AFSEL_R |= 0x0E; //Make bit 1, 2 and 3 for alternate select function
    GPIO_PORTF_PCTL_R = GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;//Select PF1,PF2 and PF3 for PWM Generators
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;           // Enable clock gating for Module 1
    __asm("   NOP");                                 //Do nothing
    __asm("   NOP");
    __asm("   NOP");
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R=0;                                // leave reset state
    PWM1_2_CTL_R=0;                                  // turn-off PWM1 generator 2
    PWM1_3_CTL_R=0;                                  // turn-off PWM1 generator 3

    // Count Down
    PWM1_2_GENB_R= PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
    PWM1_3_GENA_R= PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;
    PWM1_3_GENB_R= PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;

    PWM1_2_LOAD_R=1024;                              //51.2 MICROSECOND
    PWM1_3_LOAD_R=1024;                              //set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz

    // invert outputs for duty cycle increases with increasing compare values
    PWM1_INVERT_R = PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;
    PWM1_2_CMPB_R=0;                                //red off
    PWM1_3_CMPA_R=0;                                //green off
    PWM1_3_CMPB_R=0;                                //blue off

    PWM1_2_CTL_R=PWM_1_CTL_ENABLE;                  // turn-on PWM1 generator 2
    PWM1_3_CTL_R=PWM_1_CTL_ENABLE;                  // turn-on PWM1 generator 3

    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;// enable outputs
}

void setRGBColor(uint16_t r, uint16_t g, uint16_t b)
{
    PWM1_2_CMPB_R=r;
    PWM1_3_CMPA_R=b;
    PWM1_3_CMPB_R=g;
}

//Initialize Valid Array to false
void initValid_Array()
{
uint8_t i;
for(i=0;i<Max_Msg;i++)
{
    Valid[i]=0;
}
}
// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
// Blocking function that writes a serial character when the UART buffer is not full

// Delay Function
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

//Put a character on UART0
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART0 buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}
//Put character on UART0
/*uint8_t putIUart0(uint8_t uart1_receive)
{
    while (UART0_FR_R & UART_FR_TXFF);
        UART0_DR_R = uart1_receive;
}*/

// Put in UART1 by considering parity bit(den) and uint8_t data
void sendIntUart1(uint8_t uart1_data, bool den)
{
    Uart1_D_Enab = 1;

    if(den == 1)
        UART1_LCRH_R &= ~UART_LCRH_EPS;
    else
        UART1_LCRH_R |= UART_LCRH_EPS;

    if(!(UART1_FR_R & UART_FR_TXFF))
    UART1_DR_R = uart1_data;
}

// Get the string by using pointer concept from stored str[]
char* getString(uint8_t field)
{
    return &str[position[field]];
}
//Get the number from stored str[]
uint16_t getNum(uint8_t field)
{
   uint8_t i1;
   char* b[10];
   b[0]=&str[position[field]];
   i1=atoi(b[0]);
   return i1;
}

// Check typed string
void check_key_press()
{
    uint8_t i,j,l,f,alpha,num,count_local;
    char str_str[100];

    alpha=0;
    num=0;
    f=0;
    l=0;
    count=0;
            while(count<Max_Char)
                {
                  char c =toupper(getcUart0());// conver every character typed to UPPER CASE
                  count++;                     // to calculate total number of characters
                  i=count-1;
                    if(count>0)
                    {
                        if(c==8)               // check if character is backspace
                        {
                            count--;           //Subtract the count by 1
                            count--;
                        }
                    }
                   if(c==13)                   // check if the written character is space
                        {
                       str[i]='\0';
                       count--;
                       break;
                        }

                   if(c!=8)
                       str[i]=c;

               }

            for(j=0;j<count;j++)
               {                               // A-Z : 65-90 a-z:97-122 numbers 48-57

                 if((str[j]>=65 && str[j]<=90) || (str[j]>=97 && str[j]<=122))
                     {
                     alpha++;
                     localIndex++;            // localIndex has been used for processing of UART DATA command
                     if(alpha==1)
                       {
                        position[f]=j;        // if the 1st hit of alphabet has been encounter then save the index and the type
                         type[f]='a';
                         f++;
                       }
                     }

                else if(str[j]>=48 && str[j]<=57)
                     {
                     num++;
                     localIndex++;

                     if(num==1)
                     {
                         position[f]=j;      // if the 1st hit of numeric has been encounter then save the index and the type
                             type[f]='n';
                             f++;
                     }

                     }


                 else
                 {
                    str[j]='\0';            // if the character written is character other than alphabet or numeric make it null
                    localIndex++;
                    alpha=0;
                    num=0;
                 }
                 fields = f;               // assigning total number of fields

                 // checking if the commands written is UART and the next character is not zero and the fields value is 2(i.e) a Command and Address has been found
                 if((strcmp(&str[position[0]],"UART")==0) && (fields==2) && !(str[localIndex+1]>=48 && str[localIndex+1]<=57))
                 {
                     localIndex++;
                     break;
                 }
                 }


             //Using a checkCommand_flag to prcoess the typed command, if it is ACK / CS / RANDOM / POLL the flag has been assigned as a value of TRUE
            if((strcmp(&str[position[0]],"POLL")==0) | (strcmp(&str[position[0]],"ACK")==0) | (strcmp(&str[position[0]],"CS")==0) | (strcmp(&str[position[0]],"RANDOM")==0))
                checkCommand_flag=true;
            else                                         // Here the position at which the typing error has been made is indicated (SET A 6 7 , the output will be "Error at position 1")
            {
                        for(l=1;l<fields;l++)
                        {

                            if((type[l]=='a'))
                            {
                                sprintf(str_str,"\n\rError at position %d ",l);
                                putsUart0(str_str);
                                checkCommand_flag=false;
                                count_local++;
                            }
                            if(count_local==0)// if not error make the flag as true
                                checkCommand_flag=true;
                        }
            }



}

//Send packet function storing the info right before transmission
void sendPacket(uint8_t dest_add,uint8_t cmd,uint8_t chan_no, uint8_t length, uint8_t dataBytes[])
    {
uint8_t i,j;


    for(i=0;i<Max_Msg;i++)
    {
        if(Valid[i]==0)
        {
            Destination_Address[i]=dest_add;               //destination address
             Source_Address[i]=Source_Add;                 //source address
             Sequence_Id[i]=Seq_Id++;                      //sequence id
             MessageNumber=Sequence_Id[i];                 //for printing sequence id is used asa message number
             if(ACK_Enable == 1)
                 {
                 Command_Excute[i]= cmd | ACK_Enable<<7 ;  // If ack is enable OR the command with 0x80
                 Received_Ack[i]=true;
                 if(cmd == ACK)                            // if the sending command is ack (back) only then don't make the Received_Ack at that current index as a true
                      Received_Ack[i]=false;
                 }
             if(ACK_Enable == 0)
                 {
                 Command_Excute[i]= cmd;
                 Received_Ack[i]=false;
                 }

             Channel[i]=chan_no;                          // Channel number to which data to be sent
             Size_[i]=length;                             // Size of a data to be sent
             Checksum[i]=0;                               //Initialize checksum to 0
             for(j=0;j<length;j++)
             {
                 Data_Excute[i][j]=dataBytes[j];
                 Checksum[i] +=dataBytes[j];              // Add the checksum of all the DATA bytes
             }

             Checksum[i]=(Checksum[i]+dest_add + Source_Add+chan_no+length+Sequence_Id[i]+Command_Excute[i]);// Add the checksum of all the other bytes
             Checksum[i]=~Checksum[i];                    // Take the one's compliment of the calculated checksum
             Retransmission_Count[i]=0;                   //Initialize retransmission count to 0
             Retransmit_timeout[i]=0;                     //Initialize retransmission timeout to 0
             Valid[i]=true;                               //As a new entry of the transmission packet has been made into table make valid of current index to true.
             break;
        }
    }


}
void CommandFunction()                                     // The typed command is processed here
{
    char* a;

    uint8_t size=0;
    bytesIndex=0;                                           // Index for storing the data of packet which has data bytes more than 1

    if(checkCommand_flag)                                   // Checking if the command typed is correct [This flag is set to true in 'check_key_press' function]
    {
        if(strcmp(&str[position[0]],"SET")==0 && fields==4) // SET 36 23 1 Assigning a value of 4 to channel 23 on node 12
     {
         addr=getNum(1);                                    //Get the address from the string stored at position 1 (which will act as a index for str[position[1]])
         ch=getNum(2);                                      //Get the channel from the string stored at position 1 (which will act as a index for str[position[2]])
         value_=getNum(3);                                         //Get the value from the string stored at position 1 (which will act as a index for str[position[3]])
         dataBytesLong[bytesIndex]=value_;                  //store the data received in the array dataBytedLong
         putsUart0("Transmitter : SET command\n\r");
         sendPacket(addr,SET_cmd,ch,1,dataBytesLong);       //send the packet for transission purpose
     }

    else if(strcmp(&str[position[0]],"GET")==0 && fields==3) //GET 36 56 Requesting the data on node 12 of channel 2
         {
             addr=getNum(1);
             ch=getNum(2);
             putsUart0("Transmitter :GET command\n\r");
             sendPacket(addr,GET_cmd,ch,0,0);                // it is similar to set command just the difference is number of data bytes  is zero
         }

    else if(strcmp(&str[position[0]],"SA")==0 && fields==3)  //SA 12 34 Changing the address of node 12 to 34
        {
            addr=getNum(1);
            value_=getNum(2);
            sendPacket(addr,SA,0,1,&value_);
            putsUart0("Transmitter : SET Address command\n\r");
        }

    else if(strcmp(&str[position[0]],"CS")==0 && fields==2)   // CS ON/OFF Enabling CS
     {

         a= getString(1);

        if(strcmp(a,"ON")==0)
            {
            CS_Enable = true;
            putsUart0("CS is enabled\n\r");
            }
        else if(strcmp(a,"OFF")==0)
            {
            CS_Enable = false;
            putsUart0("CS is disabled\n\r");
            }

    }
    else if(strcmp(&str[position[0]],"ACK")==0 && fields==2)   // ACK ON/OFF Enabling ACK
         {
        a= getString(1);
        if(strcmp(a,"ON")==0)
        {
            ACK_Enable = 1;
            putsUart0("ACK is enabled\n\r");
        }
        else if(strcmp(a,"OFF")==0)
        {
            ACK_Enable = 0;
            putsUart0("ACK is disabled\n\r");
        }
         }
    else if(strcmp(&str[position[0]],"RANDOM")==0 && fields==2)   // RANDOM ON/OFF Enabling RANDOM
        {
        a= getString(1);

        if(strcmp(a,"ON")==0)
        {
            Random_Enable = 1;
            putsUart0("Random is enabled\n\r");
        }
        else if(strcmp(a,"OFF")==0)
        {
            Random_Enable = 0;
            putsUart0("Random is disabled\n\r");
        }
        }

    else if((strcmp(&str[position[0]],"UART")==0) && (fields>=2))  // UART 12^785>:'huguhb, Data received at other side should be ^785>:'huguhb
        {
        addr=getNum(1);
        while(str[localIndex]!='\0')
        {
            dataBytesLong[bytesIndex]=str[localIndex];
            localIndex++;
            bytesIndex++;// Count to calculate the size
        }
        putsUart0("Transmitter : UART DATA command\n\r");
        size=bytesIndex;
        sendPacket(addr,UART_data,0,size,dataBytesLong);
    }

    else if(strcmp(&str[position[0]],"POLL")==0 && fields==1)      //POLL, Requesting the address of the each node on the line
    {
        addr=0xFF;                                                 //It's a broadcast message so everyone on the line must receive this message
        ACK_Enable=0;                                              // Poll request it self expects the message in response so ack can be made disabled
        putsUart0("Transmitter : Poll Request command\n\r");
        sendPacket(addr,POLL_REQUEST,0,0,0);
    }
    else if((strcmp(&str[position[0]],"RESET")==0) && fields==2)   //RESET 36, The reset happens on the node 23
        {
            addr=getNum(1);
            putsUart0("Transmitter : RESET command\n\r");
            sendPacket(addr,RESET,0,0,0);
        }
    else if((strcmp(&str[position[0]],"RGB")==0) && fields==6)     // RGB 36 86 45 67 8
            {
                addr=getNum(1);
                ch=getNum(2);
                dataBytesLong[0]=getNum(3);
                dataBytesLong[1]=getNum(4);
                dataBytesLong[2]=getNum(5);
                putsUart0("It's a RGB command\n\r");
                sendPacket(addr,RGB,ch,3,&dataBytesLong[0]);
            }
   /* else if((strcmp(&str[position[0]],"PULSE")==0) && fields==5)
                {
                    addr=getNum(1);
                    ch=getNum(2);
                    value_=getNum(3);
                    PWMTime=getNum(4)
                    dataBytesLong[0]=value_;
                    dataBytesLong[0]=(PWMTime & 0xFF);
                    putsUart0("\n\rIt's a PULSE command");
                    sendPacket(addr,PULSE,ch,3,&dataBytesLong[0]);
                }*/
    else
        putsUart0("The entered command is not valid\n\r");
    }
    else
        putsUart0("The entered command is not valid\n\r");


}

void copyFinalData(uint8_t index)                                  // The final data to be transmitted is copied in array FinalData for the ease
{
    uint8_t i,j;
    j=0;
    FinalData[0]=Destination_Address[index];
    FinalData[1]=Source_Address[index];
    FinalData[2]=Sequence_Id[index];
    FinalData[3]=Command_Excute[index];
    FinalData[4]=Channel[index];
    FinalData[5]=Size_[index];
    for(i=0;i<Size_[index];i++)
    {
        FinalData[6+i]=Data_Excute[index][i];
        j=i+1;
    }
    FinalData[6+j]=Checksum[CurrentIndex];
 }

void ProcessPacket()                                               //A received packet is been processed in this function
{
    uint8_t i,j,size,changed_address;
    uint8_t r_checksum=0;
    char *strprint2[100];

    size=ReceivedData[5];                                          // A size of data has been checked here 1st

    for(i=0;i<ReceivedData[5];i++)                                 // depending of the size of data ,the whole (only) data has been copied into another array r_Received[]
    {
      r_ReceivedData[i]=ReceivedData[6+i];
      r_checksum=r_checksum+r_ReceivedData[i];                     // A checksum has been calculated for all data bytes
    }

    r_checksum=~(r_checksum+ReceivedData[0]+ReceivedData[1]+ReceivedData[2]+ReceivedData[3]+ReceivedData[4]+ReceivedData[5]);
    if(r_checksum==(ReceivedData[6+size]))
    {

    if((ReceivedData[3] & 0x7F)==SET_cmd)                                      // If set command has been received
    {
        sprintf(strprint2,"Receiver : SET Request\n\r",0);
        putsUart0(strprint2);
        r_ch=ReceivedData[4];
        r_value=r_ReceivedData[0];
        if(r_ch==23)                                                           // check if the channel received is the 1 you have assigned
            setRGBColor(0,r_value,0);
        else if(r_ch==32)
            setRGBColor(r_value,0,0);
        else
        {
            sprintf(strprint2,"This channel is not configured\n\r",0);
            putsUart0(strprint2);
        }
    }
    if((ReceivedData[3] & 0x7F)==RGB)//RGB Command
       {
        sprintf(strprint2,"Receiver : RGB reques\n\r",0);
        putsUart0(strprint2);
        r_ch=ReceivedData[4];
        if(r_ch==86)
             setRGBColor(r_ReceivedData[0],r_ReceivedData[1],r_ReceivedData[2]);// set the color to reg , green and blue led by using PWM comparartor

       }

    if((ReceivedData[3] & 0x7F)==GET_cmd)                                       // Get Command
        {
        r_ch=ReceivedData[4];
        addr=ReceivedData[1];
        sprintf(strprint2,"Receiver : GET Request\n\r",0);
        putsUart0(strprint2);
        if(r_ch==56)
        {

            if(PushButton1==1)
                value_=0;
            else
                value_=1;
        }
        else if(r_ch==32)
        {
            if(Board_RED_LED==1)
                value_=1;
            else
                value_=0;
        }
        else if(r_ch==23)
        {
            if(Board_GREEN_LED==1)
                value_=1;
            else
                value_=0;
        }
        else
            putsUart0("This channel is not configured\n\r");
            sendPacket(addr,GET_RESPONSE_cmd,0,1,&value_);
        }

    if((ReceivedData[3] & 0x7F)==GET_RESPONSE_cmd)                     // Get Response
    {
        sprintf(strprint2,"Receiver : GET Response\n\r",0);
        putsUart0(strprint2);
        if(r_ReceivedData[0]==1)
                putsUart0("The value is 1\n\r");
            else if(r_ReceivedData[0]==0)
                putsUart0("The value is 0\n\r");
    }

    if((ReceivedData[3] & 0x7F)==SA)                                  // Set address
        {
        changed_address=ReceivedData[6];
        Source_Add=changed_address;
        sprintf(strprint2,"Receiver : GET Response\n\rSource Address changed to %d\n\r",Source_Add);
        putsUart0(strprint2);
        }

   if(((ReceivedData[3] & 0x80) == 0x80) && ((ReceivedData[3] & 0x7F)!=ACK))// Ack request
    {
       addr=ReceivedData[1];
       value_=ReceivedData[2];
       sprintf(strprint2,"Receiver : ACK Request\n\rAcknowledge has been sent %d\n\r",addr);
       putsUart0(strprint2);
       sendPacket(addr,ACK,0,1,&value_);
    }

    if((ReceivedData[3] & 0x7F)==ACK)                                  // Ack response
    {
         for(j=0;j<Max_Msg;j++)
         {
            if((Valid[j]==true) && (Received_Ack[j]==true) && (Sequence_Id[j]==r_ReceivedData[0]))// Chekc the received ack
                                                                                        //process it by making every control related to the data to false/0
            {
                addr=ReceivedData[1];
                value_=r_ReceivedData[0];
                Valid[j]=false;
                Received_Ack[j]=false;
                Retransmission_Count[j]=0;
                Retransmit_timeout[j]=0;
                sprintf(strprint2,"Receiver : ACK Response\n\rAcknowledge has been received from Address :%d for Seq_Id %d\n\r",addr,value_);
                putsUart0(strprint2);
            }
         }
    }
    if((ReceivedData[3] & 0x7F)==POLL_REQUEST)                          // POLL Request
       {
        addr=ReceivedData[1];
        value_=Source_Add;
        sprintf(strprint2,"Receiver : Poll Request from address %d\n\r",addr);
        putsUart0(strprint2);
        sendPacket(addr,POLL_RESPONSE,0,1,&value_);
       }
    if((ReceivedData[3] & 0x7F)==POLL_RESPONSE)                         // POLL Response
    {
        sprintf(strprint2,"Receiver : Poll Response\n]rThe received address is : %d\n\r",r_ReceivedData[0]);
        putsUart0(strprint2);
    }

    if((ReceivedData[3] & 0x7F)==UART_data)                             // UART Data request
    {
        sprintf(strprint2,"Receiver : UART Data\n\rThe received data on UART1 is :",0);
        putsUart0(strprint2);

        for(j=0;j<size;j++)
        {
            sprintf(strprint2," %c",r_ReceivedData[j]);                 //A character by character data has been printed
            putcUart0(strprint2);
        }

    }

    if((ReceivedData[3] & 0x7F)==RESET)// RESET Command
    {
        sprintf(strprint2,"Receiver : Reset\n\r",0);
        putsUart0(strprint2);
        if((UART1_FR_R & UART_FR_BUSY)==0)
        {
            NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;  // NVIC APINT register reset bits
        }
    }
    }
    else
    {
        GREEN_LED=1;
        g_timeout=0;
    }
}


// Frequency counter service publishing latest frequency measurements every second
void Timer1Isr()
{
    uint16_t i,j,UART_Received_Byte,timeout_Re;
    char strprint3[100];
    BLUE_LED=0;
    timeout_Re=0;
    if(!inProgress)                                        // if not inProgress
    {
        for(i=0;i<Max_Msg;i++)
        {
            if(Valid[i]==true && Retransmit_timeout[i]==0) // checking for any valid entry in the table
            {
                CurrentIndex=i;                            // if found, assign that index to CurrentIndex
                inProgress=true;
                CurrentPhase=0;
                copyFinalData(CurrentIndex);               // Copy all the data to be transmitted into the into FinalData
                sprintf(strprint3,"Queuing Message %d \n\r",MessageNumber);
                putsUart0(strprint3);
                break;
            }
        }
    }


    if(inProgress)
        {
        Uart1_D_Enab = 1;
        if(CurrentPhase == 0)                              //If valid entry is there
        {
            if(((UART1_FR_R & UART_FR_BUSY)==0) && ((CS_Enable == false) || (RecievePhase==0)))//send the destinationo address
            {
                sprintf(strprint3,"Transmitting Message %d Attempt %d \n\r",MessageNumber,AttemptNumber);
                putsUart0(strprint3);
                RED_LED = 1;
                r_timeout=100;
                sendIntUart1(FinalData[CurrentPhase],1);
                CurrentPhase++;
            }
        }
        else
        {
        if((UART1_FR_R & UART_FR_BUSY)==0)                 // should same thing be repeated here?
        {

            while(!(UART1_FR_R & UART_FR_TXFF))            // send other bytes [source address, sequence id , command, channel number,size, data]
               {

                sendIntUart1(FinalData[CurrentPhase],0);
                CurrentPhase++;
                //r_timeout++;
                if(CurrentPhase==7+FinalData[5])           //if the whole packet has been transmitted, i.e.if end of message
                    {
                    inProgress=false;                      //end of message}
                    CurrentPhase=0;
                    if(Received_Ack[CurrentIndex]==false)  // if ack is not required for the specific message, make the Valid[currentindex]= false
                        Valid[CurrentIndex]=false;
                    else
                        {
                        Retransmission_Count[CurrentIndex]++; // if ack was needed, and didn't receive an ack, it will retransmit the message and retransmit counts will be saved.
                        Retransmission_Power=Retransmission_Count[CurrentIndex];
                        if(Retransmission_Count[CurrentIndex]>MAX_RETRANSMIT) // If transmission was done for maximum number of times glow red led
                        {
                            RED_LED=1;
                            r_timeout = 0;
                            AttemptNumber=0;
                            putsUart0("Transmission was done for Max Retransmit Count.\n\r");
                            Valid[CurrentIndex]=false;
                        }
                        else
                        {                                                      // timeout valus changes depednign on the RANDOM_Enable bit
                            timeout_Re=200*pow(2,Retransmission_Power);
                            if(Random_Enable==1)
                                Retransmit_timeout[CurrentIndex]=300+10*timeout_Re;
                            else
                                Retransmit_timeout[CurrentIndex]=300+timeout_Re;
                            AttemptNumber++;
                        }
                        }
                    break;
                    }
            }
        }
        }
        }
    for(j=0;j<Max_Msg;j++)                                                    // decrement Retransmit_timeout
    {
        if(Retransmit_timeout[j]>0)
            Retransmit_timeout[j]--;
    }
    if((UART1_FR_R & UART_FR_BUSY)==0)
        Uart1_D_Enab = 0;



// }
    if(!(CurrentPhase!=0))                                                    // Checking if we are in middle of transmission
    {                                                                         // Received the data depednding on the parity bit status
    while(!(UART1_FR_R & UART_FR_RXFE))
    {
        UART_Received_Byte= UART1_DR_R;
        if((UART1_LCRH_R & UART_LCRH_EPS) == 0)
        {
          if(!(UART_Received_Byte & 0x200))
        {
            RecievePhase=0;
            ReceivedData[RecievePhase]=UART_Received_Byte;
            if(ReceivedData[RecievePhase]==Source_Add | ReceivedData[RecievePhase]==0xFF)
                RecievePhase++;
        }
        else if(RecievePhase!=0)
        ReceivedData[RecievePhase++]=UART_Received_Byte;
        }
        else
        {
        if((UART_Received_Byte & 0x200))
        {
            RecievePhase=0;
            ReceivedData[RecievePhase]=UART_Received_Byte;
            if(ReceivedData[RecievePhase]==Source_Add || ReceivedData[RecievePhase]==0xFF)
              RecievePhase++;
        }
        else if(RecievePhase!=0)
        ReceivedData[RecievePhase++]=UART_Received_Byte;
        }
        Old_RecievePhase=RecievePhase;
    }
    if(Old_RecievePhase!=RecievePhase)
    {
         d_timeout++;
         sprintf(strprint3,"It's in the Receive Deadlock condition\n\r",0);
         putsUart0(strprint3);
        if(d_timeout>100)
            RecievePhase=0;
    }
    else
        d_timeout = 0;
    if(RecievePhase==7+ReceivedData[5])                                       //if the whole packet has been received process it
        {
        GREEN_LED=1;
        g_timeout=200;
        ProcessPacket();
        Old_RecievePhase=0;
        RecievePhase=0;
        }
    }
    if(g_timeout>0)
     {
     g_timeout--;
     if(g_timeout==0)
        GREEN_LED=0;
     }

     if(r_timeout>0)
     {
      r_timeout--;
      if(r_timeout==0)
          RED_LED=0;
     }

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

void initSW()
{
    ACK_Enable=0;
    CS_Enable=0;
    Random_Enable=0;
    r_timeout=0;
    g_timeout=0;
    d_timeout=0;
    Old_RecievePhase=0;
    RecievePhase=0;
}

//Main
void main(void)
{
    initSW();
    // Initialize hardware
    initHw();
    // Initialize Invalid Array
    initValid_Array();
    // Toggle Green LED every second
    GREEN_LED = 1;
    waitMicrosecond(500000);
    GREEN_LED = 0;
    waitMicrosecond(500000);

    putsUart0("\n\rUART0 Initialized");
    putsUart0("\n\r5314 Project\n\r");
    while(1)
  {

    localIndex=0;
    check_key_press();
    putsUart0("\r\n");
    CommandFunction();
  }
}
