# robot-race
This code was created for my Microprocessor Systems class where we had a robot race. 
/*===================================CPEG222====================================
 * Program:      Project 4 Template
 * Authors:     Onyinyechi Okafor & Nora Charles
 * Date:        11/29/2023
 * This is a guide that you can use to write your project 4 code
==============================================================================*/
/*-------------- Board system settings. PLEASE DO NOT MODIFY THIS PART ----------*/
#ifndef _SUPPRESS_PLIB_WARNING          //suppress the plib warning during compiling
#define _SUPPRESS_PLIB_WARNING
#endif
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
/*----------------------------------------------------------------------------*/
#define SYS_FREQ (80000000L) // 80MHz system clock
#define _80Mhz_ (80000000L)
#define LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz 1426
#define LOOPS_NEEDED_TO_DELAY_ONE_MS (LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz * (SYS_FREQ / _80Mhz_))

#define TRUE 1
#define FALSE 0
#define led0 LATAbits.LATA0
#define led1 LATAbits.LATA1
#define led2 LATAbits.LATA2
#define led3 LATAbits.LATA3
#define led4 LATAbits.LATA4
#define led5 LATAbits.LATA5
#define led6 LATAbits.LATA6
#define led7 LATAbits.LATA7
#define sw0 PORTFbits.RF3
#define sw1 PORTFbits.RF5
#define sw6 PORTBbits.RB10
#define sw7 PORTBbits.RB9

//PMOD sensors 
#define LS1 PORTCbits.RC14
#define LS2 PORTDbits.RD0
#define LS3 PORTDbits.RD1
#define LS4 PORTCbits.RC13

//btn
#define BtnC_RAW PORTFbits.RF0
#define BUTTON_DEBOUNCE_DELAY_MS 20


// Libraries
#include <string.h>
#include <xc.h>   //Microchip XC processor header which links to the PIC32MX370512L header
#include <stdio.h>  // need this for sprintf
#include <sys/attribs.h>
#include "config.h" // Basys MX3 configuration header
#include "led.h"
#include "ssd.h"
#include "lcd.h"
#include "swt.h"
//#include "swt.c"
//#include "adc.c"
#include "adc.h"
#include "mic.h" 
//#include "mic.c"
#include "utils.h"
#include "rgbled.h"


int counter = 0;
int timer_on = 1;
char start = TRUE;
unsigned int mic;
int clap = 0;
int checkpoint = 0; 
int check = 0;
int buttonsLocked = FALSE;
char pressedUnlockedBtnC = FALSE;
int left_motor;
int right_motor;
int black_check = 0;
int RED, GRN, BLU = 0;            // Used for setting the RGB led

char sensors;

// Function Declarations
void initializePorts();
void pwmConfig();
void activateServo(char sensors);
void CNConfig();
void config_timer3();
void straight();
void right();
void left();
void hardL();
void hardR();
void slightL();
void slightR();
void reverse();
void stop();
void delay_ms();
void handle_raw_button_presses();
//void black_checkpoint();

int main(void) {
            //PS.. It might be a good idea to put this function in a timer ISR later on.
    initializePorts();
    pwmConfig();
    config_timer3();
    stop();
    clap = 0;
    char oldSensors = 0;
    timer_on = 0;
    while (TRUE) {
        //activateServo();
            //410 is the threshold value for the mic
            mic = MIC_Val();
            
            LCD_WriteStringAtPos("WallE          ", 0, 0); // line 0, position 0
            //char val[10];
            //sprintf(val, "%u", mic);
            
            //LCD_WriteStringAtPos(val, 1, 0); // line 0, position 0 
            if (mic < 400){
                //LCD_WriteStringAtPos("CLAP          ", 0, 0); // line 0, position 0
                //LATA |= 0x01;
                
                clap ++;
//                     handle_raw_button_presses();
//                     timer_on = 0;
//                     stop();
//                          
                if(clap == 1){
                    //LCD_WriteStringAtPos("CLAP 1         ", 0, 0); // line 0, position 0
                      //stop();;
                     //stop();
                     //activateServo();
                     handle_raw_button_presses();
                     //timer_on = 0;
                     stop();
                     
                }
                else {
                    //LCD_WriteStringAtPos("CLAP 2         ", 0, 0); // line 0, position 0
                    //activateServo();
                    timer_on = 1; 
                    while(1) {
                        //straight();
//                        delay_ms(50);
                        sensors = LS4 << 3 | LS3 << 2 | LS2 << 1 | LS1;
                        activateServo(sensors);
                        
                        handle_raw_button_presses();
                        if (black_check >= 2){
                            timer_on = 0;
                            stop();
                            break;
                        }
                    }
                    clap = 1;
                }
                    
            delay_ms(100);
            }
            //else
                //stop();//LCD_WriteStringAtPos("OK          ", 0, 0); // line 0, position 0                 
        //}
}
}




    






// Initialize ports on board
void initializePorts() {
    LED_Init();
    LCD_Init();
    TRISA &= 0xFF00; 
    SSD_Init();
    SWT_Init();
    ADC_Init();
    MIC_Init();
    RGBLED_Init();
    RED = 255;                  //Sets the RGBLED to RED
    GRN = BLU = 0;
    RGBLED_SetValue(RED,GRN,BLU);
    
    
    //switch configuration
    TRISFbits.TRISF3 = 1;//sw0
    TRISFbits.TRISF5 = 1;//sw1
    TRISBbits.TRISB10 = 1; // RB10 (SW6) configured as input
    ANSELBbits.ANSB10 = 0; // RB10 (SW6) disabled analog
    TRISBbits.TRISB9 = 1; // RB9 (SW7) configured as input
    ANSELBbits.ANSB9 = 0; // RB9 (SW7) disabled analog
    
    //IR sensors
    ANSELDbits.ANSD1 = 0;
    TRISCbits.TRISC14 = 1;
    TRISDbits.TRISD1 = 1;
    TRISCbits.TRISC13 = 1;
    TRISDbits.TRISD0 = 1;
    
    //btn c
    TRISFbits.TRISF0 = 1; //configured BTNC  as input
    
    //microphone configuration
//    TRISBbits.TRISB14 = 0; //A_OUT pin digital output pin
//    ANSELBbits.ANSB14 = 0; //A_OUT pin digital output pin
//
//    
//    TRISBbits.TRISB4 = 1; // A_MIC pin configuration used as analog input pin
//    ANSELBbits.ANSB4 = 1; // A_MIC pin configuration
//    DDPCONbits.JTAGEN = 0;
    
    
    //TRISBbits.TRISB2 = 1;//AIC
    //ANSELBbits.ANSB2 = 1;



}
void config_timer3(){
    //PR3 = 10000000; //Period Register.
    //PR3 = (int)(((float)(10000000) / 256) + 0.5);
    PR3 = 24999;
    T3CONbits.TCKPS =  2; //Timer Prescaler 
    T3CONbits.TGATE = 0; // not gated input (the default)
    T3CONbits.TCS = 0; // PBCLK input (the default)
    T3CONbits.ON = 1;  //Turn on Timer
    TMR3 = 0; // Set Timer X to 0
    
    IPC3bits.T3IP = 6;  //    priority
    IPC3bits.T3IS = 3;  //    subpriority
    IFS0bits.T3IF = 0; //    clear interrupt flag
    IEC0bits.T3IE = 1; //    enable interrupt
    
    
}
void pwmConfig() {
    
    // configure Timer X (select the timer, replace X with desired timer number)
    
    PR2 = 24999; //Period Register.
    //PR2 = (int)(((float)(10000000) / (16*5)) + 0.5)*0.2;
    T2CONbits.TCKPS =  3; //Timer Prescaler 
    T2CONbits.TGATE = 0; // not gated input (the default)
    T2CONbits.TCS = 0; // PBCLK input (the default)
    T2CONbits.ON = 1;  //Turn on Timer
    TMR2 = 0; // Set Timer X to 0

    
    
    IPC2bits.T2IP = 6;  //    priority
    IPC2bits.T2IS = 3;  //    subpriority
    IFS0bits.T2IF = 0; //    clear interrupt flag
    IEC0bits.T2IE = 1; //    enable interrupt
    
    
    // Configure Output Compare Module 4
    
    OC4CONbits.OCM = 6;      // PWM mode on OC4; Fault pin is disabled
    OC4CONbits.OCTSEL = 0;   // Select the timer to use as a clock source
    OC4RS = PR2/20;//OC4RS is some fraction of the Period
    OC4R = OC4RS;
    OC4CONbits.ON = 1;       // Start the OC4 module
    
    //Do The same for OC5**************************
    OC5CONbits.OCM = 6;      // PWM mode on OC4; Fault pin is disabled
    OC5CONbits.OCTSEL = 0;   // Select the timer to use as a clock source
    OC5RS = PR2/20;//OC5RS is some fraction of the Period
    OC5R = OC5RS;
    OC5CONbits.ON = 1; 
   
   TRISBbits.TRISB8 = 0; //set servo 0 as output
   TRISAbits.TRISA15 = 1; //set servo 1 as output
   ANSELBbits.ANSB8 = 0; //set servo 0 as digital

   RPB8R = 0x0B; // connect Servo 0 to OC5
   RPA15R = 0x0B;// connect Servo 1 to OC4
    

    //Set up additional timers here if necessary
}
void CNConfig() {
    /* Make sure vector interrupts is disabled prior to configuration */
    macro_disable_interrupts;
    
     //Complete the following configuration of CN interrupts, then uncomment them
    CNCONDbits.ON = 1;   //all port D pins to trigger CN interrupts
    CNEND = 0xF00;      	//configure PORTD pins 8-11 as CN pins
    CNPUD = 0xf00;      	//enable pull-ups on PORTD pins 8-11

    IPC8bits.CNIP = 5;  	// set CN priority to  5
    IPC8bits.CNIS = 3;  	// set CN sub-priority to 3

    IFS1bits.CNDIF = 0;   	//Clear interrupt flag status bit
    IEC1bits.CNDIE = 1  ;   	//Enable CN interrupt on port D
    
    
    int j = PORTD;             //read port to clear mismatch on CN pins
    macro_enable_interrupts();	// re-enable interrupts
}
void SSD_Timer2Setup(){
  //10000000
  //PR2 = (int)(((float)(10000000) / (16*5)) + 0.5)*0.2; //set period register, generates one interrupt every 10 ms
  PR2 = 24999;
  TMR2 = 0;                           //    initialize count to 0
  T2CONbits.TCKPS = 3;                //    1:64 prescale value
  T2CONbits.TGATE = 0;                //    not gated input (the default)
  T2CONbits.TCS = 0;                  //    PCBLK input (the default)
  T2CONbits.ON = 0;                   //    turn on Timer2
  IPC2bits.T2IP = 6;                  //    priority
  IPC2bits.T2IS = 3;                  //    subpriority
  IFS0bits.T2IF = 0;                  //    clear interrupt flag
  IEC0bits.T2IE = 1;                  //    enable interrupt
  macro_enable_interrupts();          //    enable interrupts at CPU
}

void SSD_Timer3Setup(){
  //PR3 = (int)(((float)(10000000) / 256) + 0.5); //set period register, generates one interrupt every 10 ms
  //PR3 = 10000000;
  PR3 = 24999;
  TMR3 = 0;                           //    initialize count to 0
  T3CONbits.TCKPS = 2;                //    1:64 prescale value
  T3CONbits.TGATE = 0;                //    not gated input (the default)
  T3CONbits.TCS = 0;                  //    PCBLK input (the default)
  T3CONbits.ON = 0;//was 0 b4                   //    turn on Timer1
  IPC3bits.T3IP = 6;                  //    priority
  IPC3bits.T3IS = 3;                  //    subpriority
  IFS0bits.T3IF = 0;                  //    clear interrupt flag
  IEC0bits.T3IE = 1;                  //    enable interrupt
  macro_enable_interrupts();          //    enable interrupts at CPU
}
//ISR's are here is you need them. Don't forget to set them up!
void __ISR(_TIMER_2_VECTOR) Timer2ISR(void) {
    IEC0bits.T2IE = 0; // disable interrupt   
    IFS0bits.T2IF = 0; // clear interrupt flag    
    IEC0bits.T2IE = 1; //was 2 before, but changed to 1
}
void delay_ms(int ms)
{
    int i;
    for (i = 0; i < ms * LOOPS_NEEDED_TO_DELAY_ONE_MS; i++)
    {    }
}
void __ISR(_TIMER_3_VECTOR) Timer3ISR(void) {
    IEC0bits.T3IE = 0; // disable interrupt
    if(counter % 10 == 0){
        int converted = counter * 0.1;
        int decimal = converted % 10;
        int ones = (converted / 10) %10;
        int tens = (converted /100) %10;
        //int hundreds = (converted/1000);
        if(tens==0){
           SSD_WriteDigits(decimal, ones, 17, 17, 0, 1, 0, 0);
           }
        else if(ones==0 && tens==0){
           SSD_WriteDigits(decimal,17,17, 17, 0, 1, 0, 0);
           }
        else{
           SSD_WriteDigits(decimal, ones, tens, 17, 0, 1, 0, 0);
        } 
    }  
    
    if(timer_on){
        counter++;
        
        /// Watch the sensors variable
        static int blackCounter = 0;
        static char blackLocked = FALSE;
        if(sensors == 0b0000 ) {
            blackCounter++;
            
            if(blackCounter >= 15) {
                blackLocked = TRUE;
                blackCounter = 0;
                black_check ++;
                //timer_on = 0;
            }
        }
        else {
            blackLocked = FALSE;
            blackCounter = 0;
            //timer_on = 0; 
        }
        //timer_on = 0;    
        //char val[10];
        //sprintf(val,"%d          ", black_check);
        //LCD_WriteStringAtPos(val, 0, 0); // line 0, position 0 

    }
    IFS0bits.T3IF = 0; // clear interrupt flag
    IEC0bits.T3IE = 1; // enable interrupt
}

//void black_checkpoint(){ // checks to see if it passes checkpoint 
//    left_motor = 1;
//    right_motor = 1;   
//    
//    if (counter > 1600){ // counter is our timer value
//        checkpoint ++; // 
//    }
//}

void activateServo(char sensors){   
//    delay_ms(20);
 //oc5 = right motor
 //0c4 = left motor 
  //  stop();

//} 
       switch(sensors) {       
        case 0b0000: {
           LATA &= 0xFF00;
//        stop();
            RED = (255,0,0);
//            black_check++;
 
            

            LCD_WriteStringAtPos("STP", 1, 0); // line 1, position 0
            LCD_WriteStringAtPos("STP", 1, 13); // line 1, position 0
            //stop(); 
            straight();
            

        }
            break;
       
        case 0b0001://slight left
            LATA &= 0xFF0F;
            LATA |= 0x00C0;
            slightL();
            BLU = (0, 0, 255);
            break;
        case 0b0111: // hard L
        case 0b0011:
            LATA &= 0xFF0F;
            LATA |= 0x00C0;
            hardL();
            GRN = (0, 255, 0);
            break;
        case 0b1000: // slight right
            LATA &= 0xFFF0;
            LATA |= 0x000C;
            slightR();
            RED = (255,0,0);
            break;
        case 0b1110: // Hard R
        case 0b1100:
            LATA &= 0xFFF0;
            LATA |= 0x000C;
            hardR();
            GRN = (0, 0, 255);
            break;
        case 0b1111: // reverse
            LATA &= 0xFFF0;
            LATA |= 0x0003;
            
            LATA &= 0xFF0F;
            LATA |= 0x0030;
            GRN = (0, 255, 0);
            LCD_WriteStringAtPos("REV", 1, 0);
            LCD_WriteStringAtPos("REV", 1, 13);
            reverse();//when both motors stop, it turns off the board
            break;
        case 0b1001:
            LATA &= 0xFF0F;
            LATA |= 0x00C0;
            
            LATA &= 0xFFF0;
            LATA |= 0x000C;
            RED = (255,0,0);
//            if (counter > 1600){ // counter is our timer value
//                checkpoint ++; // 
//    }
        LCD_WriteStringAtPos("FWD", 1, 0);
        LCD_WriteStringAtPos("FWD", 1, 13);
            straight();
            break;
        default:
            straight();
            break;
            
       }
    RGBLED_SetValue(RED,GRN,BLU);
}



    


void handle_raw_button_presses(){
   // Clear all button flags initially
{
pressedUnlockedBtnC = FALSE;

if ((BtnC_RAW) && !buttonsLocked)
{
delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
buttonsLocked = TRUE;
pressedUnlockedBtnC = BtnC_RAW;
}
else if (!(BtnC_RAW) && buttonsLocked)
{
delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
buttonsLocked = FALSE;
}
}
}    
//left turns need to be tweaked
 void straight(){
     OC5RS = (int) PR2 / 20;//right motor 1 ms
     OC4RS = (int) PR2 / 10;//left motor 2 ms 
     //black_check++;
     
 }
 void hardR(){
     OC5RS = (int) PR2 / 10;//right motor 2ms
     OC4RS = (int) PR2 / 10;//left motor 1.5ms   
 }
  void hardL(){ //right motor would have to be faster -> works
    OC5RS = (int) PR2 / 20;//right motor 1.0ms -> HARD
    OC4RS = (int) PR2 / 20;//left motor 1.5ms  -> STOP
 }

 void slightL(){//slightly reduce speed of left motor
    //OC5RS = (int) PR2 / 13.2;//right motor 1.5ms
    //OC4RS = (int) PR2 / 14;//left motor 1ms 
    
    OC5RS = (int) PR2 / 20;//right motor 1.5ms
    OC4RS = (int) PR2 / 13.2;//left motor 1ms 
 }
 
 void slightR(){//slightly reduce speed of right motor
    OC5RS = (int) PR2 / 13.2;//right motor 1.5ms
    OC4RS = (int) PR2 / 10;//left motor 1ms (# goes down to speed it up)
 }
 void stop(){
     OC5RS = (int) PR2 / 13.2;//right motor 1.5ms
     OC4RS = (int) PR2 / 13.28;//left motor 1.5ms
     //black_check != 0;
     
     }
 
 void reverse(){
     OC5RS = (int) PR2 / 10;//right motor 2 ms
     OC4RS = (int) PR2 / 20;//left motor 1 ms*/ 
 }
