// CONFIG
#pragma config FOSC = HS     // Oscillator Selection bits (RC oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#define LeftSensor      PORTBbits.RB1
#define RightSensor     PORTBbits.RB2
#define CenterSensor    PORTBbits.RB0
#define TRIGGER         PORTCbits.RC5
#define ECHO            PORTCbits.RC0
#define IN1             PORTBbits.RB4
#define IN2             PORTBbits.RB5
#define IN3             PORTBbits.RB6
#define IN4             PORTBbits.RB7
#define Baud_rate       9600

#define _XTAL_FREQ      8000000
#include <xc.h>
#include <stdio.h>
#include "lcd.h"

unsigned char flag = 0;
unsigned int  u16time, u16distance;
unsigned char buffer[10];
unsigned char get_value;

void Config(void)
{
    TRISB = 0b00001111;
    TRISC = 0b11000001;
    TRISA = 0;
    TRISD = 0;
//    PORTB = 0;
    PORTD = 0;
}
void PWM1_Init(void)
{
      CCP1M3 = 1;
      CCP1M2 = 1;
      TRISC2 = 0;
      PR2 = 49;  // 500us
      T2CKPS0 = 1;
      T2CKPS1 = 0;
      TMR2ON = 1;
}
void PWM1_Set_Duty(uint16_t DC)
{
    if(DC<1024)
    {
        CCP1Y = DC & 1;
        CCP1X = DC & 2;
        CCPR1L = DC >> 2;
    }
}
void PWM2_Init(void)
{
      CCP2M3 = 1;
      CCP2M2 = 1;
      TRISC1 = 0;
      PR2 = 49;  // 500us
      T2CKPS0 = 1; //1:4
      T2CKPS1 = 0;
      TMR2ON = 1;
}
void PWM2_Set_Duty(uint16_t DC)
{
    if(DC<1024)
    {
        CCP2Y = DC & 1;
        CCP2X = DC & 2;
        CCPR2L = DC >> 2;
    }
}
unsigned int get_val()
{
    //Set timer1
    T1CONbits.T1CKPS1 = 0;
    T1CONbits.T1CKPS0 = 1; //1:1
    T1CONbits.TMR1CS = 0;
    
    //Kich 1 xung lon hon (>10ms) vao trigger
    TRIGGER = 1;
    __delay_ms(20);
    TRIGGER = 0; 
    
    //Lay thoi gian tu chan ECHO
    while(!ECHO);
    TMR1ON = 1;
    while(ECHO);
    TMR1ON = 0;
    
    u16time = TMR1H;
    u16time <<= 8;
    u16time += TMR1L;
    
    return u16time;
}
void Initialize_UART(void)
{
    TRISC6 = 0;
    TRISC7 = 1;
    SPBRG = ((_XTAL_FREQ/16)/Baud_rate) - 1;
    BRGH  = 1;
    SYNC  = 0;
    SPEN  = 1;
    TXEN  = 1;
    CREN  = 1;
    TX9   = 0;
    RX9   = 0;
}
void UART_send_char(char bt)
{
    while(!TXIF);
    TXREG = bt;
}
char UART_get_char()
{
    if(OERR)
    {
        CREN = 0;
        CREN = 1;
    }
    while(!RCIF);
    return RCREG;
}
void UART_send_string(char* st_pt)
{
    while(*st_pt)
        UART_send_char(*st_pt++);
}

void car(unsigned char mode)
{
    if(mode == 1)
    {
        IN1 = 1; IN2 = 0; 
        PWM1_Set_Duty(160);  //Motor1 forward
        IN3 = 1; IN4 = 0; 
        PWM2_Set_Duty(160);  //Motor2 forward
    }
    //Rotate left
    else if(mode == 2)
    {
        IN1 = 0; IN2 = 0;     //Motor1 stop
        PWM2_Set_Duty(200);   //Motor2 forward
        IN3 = 1; IN4 = 0; 
    }
    else if(mode == 3)
    {
        PWM1_Set_Duty(200);   //Motor1 forward
        IN1 = 1; IN2 = 0; 
        IN3 = 0; IN4 = 0;     //Motor2 stop
    }
    else if(mode == 4)
    {
        IN1 = 0; IN2 = 0; 
        IN3 = 0; IN4 = 0; 
    }
    else if(mode == 5)
    {
        IN1 = 0; IN2 = 1; 
        PWM1_Set_Duty(200);
        IN3 = 0; IN4 = 1;
        PWM2_Set_Duty(200);
    }
    else if(mode == 6)
    {
        IN1 = 1; IN2 = 0; 
        PWM1_Set_Duty(160);
        IN3 = 0; IN4 = 1;
        PWM2_Set_Duty(160);
    }
    else if(mode == 7)
    {
        IN1 = 0; IN2 = 0;     //Motor1 stop
        PWM2_Set_Duty(160);   //Motor2 forward
        IN3 = 1; IN4 = 0; 
    }
    else if(mode == 8)
    {
        PWM1_Set_Duty(160);   //Motor1 forward
        IN1 = 1; IN2 = 0; 
        IN3 = 0; IN4 = 0;     //Motor2 stop
    }
}
void control_car(char character)
{
    if(character == 'F')
        car(1);
    else if(character == 'L')
        car(2);
    else if(character == 'R')
        car(3);
    else if(character == 'B')
        car(5);
    else if(character == 'S')
        car(4);
    else if(character == 'X')
    {
        RA5 = 1;
        __delay_ms(100);
        RA5 = 0;
    }    
    else if(character == 'Y')
    {
        RA4 = 1;
        __delay_ms(100);
        RA4 = 0;
    }    
}
unsigned char flag2 = 0;
void main(void) {
    Config();
    PWM1_Init();
    PWM2_Init();
    Initialize_UART();
    RC3 = 1; RA5 = 0;
    car(1);
    __delay_ms(1000);
    while(1)
    {   
        if(flag2 == 0)
        {
            car(1);
            __delay_ms(400);
            flag2 = 1;
        }
        if(flag == 1)
        {
            while(1)
            {
                get_value = UART_get_char(); //Read the char. received via BT
                control_car(get_value); 
            }
        }
        u16distance = get_val();
        unsigned char temp;
        temp = u16distance / 58;

        if((0 == CenterSensor) )
        {
            car(1);
        }
        if((0 == CenterSensor) && (0 == LeftSensor) && (0 == RightSensor))
        {
            car(1);
        }
        if((0 == LeftSensor))
        {
            car(3);
        }
        if(((0 == LeftSensor) && (0 == CenterSensor)))
        {
            car(3);
        }
        if((0 == RightSensor))
        {
            car(2);
        }
        if(((0 == RightSensor) && (0 == CenterSensor)))
        {
            car(2);
        }
        if((1 == CenterSensor) && (1 == LeftSensor) && (1 == RightSensor))
        {
            car(8);
        }
        if(temp < 5)
        {
            __delay_ms(100);
            if(temp < 5)
            {
                flag = 1;
                car(4);
                RA3 = 1;
                __delay_ms(200);
                RA3 = 0; 
            }
        }
        TMR1L = 0; TMR1H = 0;
    }
    return;
}
