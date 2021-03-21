/*
 * File:   main.c
 * Author: George Ayman
 *
 * Created on August 9, 2020, 2:03 PM
 */


#include "CONFIG.h"
#include "USB.h"
#include "LCD.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "USB_CONFIG.h"

#define forward 1
#define backward 0

#define StepsPerMillimeterX 160    //el ganzer el so4ayar
#define StepsPerMillimeterY 160    //to be confirmed
#define StepsPerMillimeterZ 160    //to be confirmed

#define moveX 1
#define moveY 2
#define moveZ 4
#define forX 8
#define forY 16
#define forZ 32
#define backX 0b11110111
#define backY 0b11101111
#define backZ 0b11011111


volatile char chop = 0;         //flag indicate on/off chopping time
volatile char transientX = 0;
volatile char transientY = 0;
volatile char transientZ = 0;

volatile signed char currentStepX = 0;
volatile signed char currentStepY = 0;
volatile signed char currentStepZ = 0;

volatile int ready = 22;

float Xpos = 0;
float Ypos = 0;
float Zpos = 0;

char directionX = forward;
char directionY = forward;
char directionZ = forward;

char points[1500];
char currentPoint = 0  ; 
volatile signed int points_first = -1;
signed int points_end = 0;

unsigned char Sindex = 0;
unsigned char findex = 0;
char xbuffer[10];
char ybuffer[10];
char zbuffer[10];
int i;

float X0 = 0;
float Y0 = 0;
float Z0 = 0;

float X1 = 0;
float Y1 = 0;
float Z1 = 0;

unsigned int dx = 0;
unsigned int dy = 0;
unsigned int dz = 0;

float disX,disY,disZ;

int err_1, err_2;
unsigned int dx2, dy2, dz2;
float errX,errY,errZ, tempx,tempy,tempz;
char code = 0;

void TIMER0_INIT(char timerValue)
{
    T08BIT = 1;   //Timer0 8-Bit/16-Bit Control bit, 0 for 16bit , 1 for 8bit
    T0CS = 0;     // Timer0 Clock Source Select bit, 0 for internal cycle clock, 1 for external event
    T0SE = 0;     //Timer0 Source Edge Select bit, 0 for L/H, 1 for H/L
    PSA = 0;      //Timer0 Prescaler Assignment bit,1 off,0 on
    T0CONbits.T0PS = 0b010;   //Timer0 Prescaler Select bits, 0b011 for 1:16
    GIE = 1;
    PEIE = 1;
    TMR0IE = 1;
    TMR0IP = 1;   //high priority interrupt
    TMR0ON = 1;   // Timer0 On/Off Control bit
    TMR0L = timerValue;
} 

void lcdCharToHex(char x)
{
    
    LCD_Clear();
    //LCD_Set_Cursor(1,1);
    __delay_ms(500);
    char temp1 = x & 0x0F;
    char temp2 = (x & 0xF0) >> 4;
    switch (temp2)
    {
        case 10:
            LCD_Write_Char('A');
            break;
        case 11:
            LCD_Write_Char('B');
            break;
        case 12:
            LCD_Write_Char('C');
            break;
        case 13:
            LCD_Write_Char('D');
            break;
        case 14:
            LCD_Write_Char('E');
            break;
        case 15:
            LCD_Write_Char('F');
            break;
        default:
            LCD_Write_Char(temp2+48);
    }
    switch (temp1)
    {
        case 10:
            LCD_Write_Char('A');
            break;
        case 11:
            LCD_Write_Char('B');
            break;
        case 12:
            LCD_Write_Char('C');
            break;
        case 13:
            LCD_Write_Char('D');
            break;
        case 14:
            LCD_Write_Char('E');
            break;
        case 15:
            LCD_Write_Char('F');
            break;
        default:
            LCD_Write_Char(temp1+48);
    }
    __delay_ms(500);
}

void parse()
{
 
    while(buffFlags[rdIndex])
    {
        code = 0;
        X0 = Xpos;
        Y0 = Ypos;
        Z0 = Zpos;
        if (linesBuffer[rdIndex][2] == ' '){
              buffFlags[rdIndex] = 0;
                rdIndex++;
                if(rdIndex == 3 )
                    rdIndex = 0 ;
            continue;
        }
            
            Sindex = 6;
            while(linesBuffer[rdIndex][Sindex]!= ' ')
            {
                 xbuffer[findex++] = linesBuffer[rdIndex][Sindex++];
                 
            }
             xbuffer[findex] = '\0';
            findex=0;
            Sindex+=3;
            while(linesBuffer[rdIndex][Sindex]!= ' ')
            {
                 ybuffer[findex++] = linesBuffer[rdIndex][Sindex++];
            }
            ybuffer[findex] = '\0';
            findex=0;
            Sindex+=3;
            while((linesBuffer[rdIndex][Sindex]!= ' ') && (linesBuffer[rdIndex][Sindex]!= '\r'))
            {
                 
                 zbuffer[findex++] = linesBuffer[rdIndex][Sindex++];
                
            }
            zbuffer[findex] = '\0';
            findex=0;
            
            X1 = (float)atof(xbuffer);
            Y1 = (float)atof(ybuffer);
            Z1 = (float)atof(zbuffer);
     /*
            if(X1 == 0.0500)
                LCD_WR("X = 0.0500");
            else if(X1 == 0.0000)
                LCD_WR("X = 0.0000");
            if(Y1 == 0.0500)
                LCD_WR("Y = 0.0500");
            else if(Y1 == 0.0000)
                LCD_WR("Y = 0.0000");
            if(Z1 == 0.0500)
                LCD_WR("Z = 0.0500");
            else if(Z1 == 0.0000)
                LCD_WR("Z = 0.0000");
      */
            disX = fabs(X1 - X0);
            disY = fabs(Y1 - Y0);
            disZ = fabs(Z1 - Z0);

            tempx = disX * StepsPerMillimeterX;
            tempy = disY * StepsPerMillimeterY;
            tempz = disZ * StepsPerMillimeterZ;
          /*  
            if(tempy == 0.496)
                LCD_WR("tmpy = 0.496");
            else if(tempy == 0.992)
                LCD_WR("tmpy = 0.992");
            else if(tempy == 1.488)
                LCD_WR("tmpy = 1.488");
            */
            dx = (unsigned int)tempx;
            dy = (unsigned int)tempy;
            dz = (unsigned int)tempz;
            
           /*
            if(dx == 8)
                LCD_WR("dx = 8");
            else if(dx == 0)
                LCD_WR("dx = 0");
            if(dy == 8)
                LCD_WR("dy = 8");
            else if(dy == 0)
                LCD_WR("dy = 0");
            if(dz == 8)
                LCD_WR("dz = 8");
            else if(dz == 0)
                LCD_WR("dz = 0");
            */
            
            
            Xpos = X1;
            Ypos = Y1;
            Zpos = Z1;

            directionX = (X1 < X0) ? backward : forward;
            directionY = (Y1 < Y0) ? backward : forward;
            directionZ = (Z1 < Z0) ? backward : forward;
            
            if(directionX)
            {
                errX += (tempx - dx);
                if(errX > 1.0)
                {
                    dx++;
                    errX--;
                }
                else if(errX < -1.0)
                {
                    dx--;
                    errX++;
                }
            }
            else
            {
                errX -= (tempx - dx);
                if(errX > 1.0)
                {
                    dx--;
                    errX--;
                }
                else if(errX < -1.0)
                {
                    dx++;
                    errX++;
                }
            }
            
            
            if(directionY)
            {
                errY += (tempy - dy);
               
            
                if(errY > 1.0)
                {  
                    dy++;
                    errY--;
                }
                else if(errY < -1.0)
                {
                    dy--;
                    errY++;
                }
                
            }
            else
            {
                errY -= (tempy - dy);
                if(errY > 1.0)
                {
                    dy--;
                    errY--;
                }
                else if(errY < -1.0)
                {
                
                    dy++;
                    errY++;
                }
                      
            }
            
            if(directionZ)
            {
                errZ += (tempz - dz);
                if(errZ > 1.0)
                {
                    dz++;
                    errZ--;
                }
                else if(errZ < -1.0)
                {
                    dz--;
                    errZ++;
                }
            }
            else
            {
                errZ -= (tempz - dz);
                if(errZ > 1.0)
                {
                    dz--;
                    errZ--;
                }
                else if(errZ < -1.0)
                {
                    dz++;
                    errZ++;
                }
            }
            

            dx2 = dx << 1;
            dy2 = dy << 1;
            dz2 = dz << 1;

            if ((dx >= dy) && (dx >= dz)) 
            {
 
                err_1 = dy2 - dx;
                err_2 = dz2 - dx;
                for (i = 0; i < dx; i++) {
                    code = 0;
                    if (err_1 > 0) {
                        code |= moveY;
                        if(directionY)
                            code |= forY;
                        else 
                            code &= backY;
                     
                        err_1 -= dx2;
                    }
                    if (err_2 > 0) {
                        code |= moveZ;
                        if(directionZ)
                            code |= forZ;
                        else 
                            code &= backZ;
                        err_2 -= dx2;
                    }
                    err_1 += dy2;
                    err_2 += dz2;
                    code |= moveX;
                    if(directionX)
                        code |= forX;
                    else 
                        code &= backX;
                
                        while(points_end == points_first){}
                        points[points_end++] = code;
                       
                        if(points_end == 1500)
                            points_end = 0;
                }
            } 
            else if ((dy >= dx) && (dy >= dz)) 
            {
                err_1 = dx2 - dy;
                err_2 = dz2 - dy;
                for (i = 0; i < dy; i++) {
                    code = 0;
                    if (err_1 > 0) {
                        code |= moveX;
                        if(directionX)
                            code |= forX;
                        else 
                            code &= backX;
                        err_1 -= dy2;
                    }
                    if (err_2 > 0) {
                        code |= moveZ;
                        if(directionZ)
                            code |= forZ;
                        else 
                            code &= backZ;
                        err_2 -= dy2;
                    }
                    err_1 += dx2;
                    err_2 += dz2;
                    code |= moveY;
                    if(directionY)
                        code |= forY;
                    else 
                        code &= backY;
                    
                    while(points_end == points_first){}
                        points[points_end++] = code;
                       
                        if(points_end == 1500)
                            points_end = 0;
                        
                }
            } 
            else {
                err_1 = dy2 - dz;
                err_2 = dx2 - dz;
                for (i = 0; i < dz; i++) {
                    code = 0;
                    if (err_1 > 0) {
                        code |= moveY;
                        if(directionY)
                            code |= forY;
                        else 
                            code &= backY;
                        err_1 -= dz2;
                    }
                    if (err_2 > 0) {
                        code |= moveX;
                        if(directionX)
                            code |= forX;
                        else 
                            code &= backX;
                        err_2 -= dz2;
                    }
                    err_1 += dy2;
                    err_2 += dx2;
                    code |= moveZ;
                    if(directionZ)
                        code |= forZ;
                    else 
                        code &= backZ;
                    
                    while(points_end == points_first){}
                        points[points_end++] = code;
                       
                        if(points_end == 1500)
                            points_end = 0;
                }
            }
            /*
            for(int iii=0;iii<points_end;iii++)
                lcdCharToHex(points[iii]);
            */
            buffFlags[rdIndex] = 0;
            rdIndex++;
            if(rdIndex == 3 )
                rdIndex = 0 ;
    }
}
 
void __interrupt() ISR()
{
    if(TMR0IF)
    {
        if((points_first+1) != points_end)
        {
            if(ready == 55)
            {
                ready = 0;
                points_first ++;
                if(points_first == 1500)
                    points_first = 0;
                
                    currentPoint = points[points_first] ; 
                if(currentPoint & 0x01)
                {
                    transientX = 1;
                    if(currentPoint & 0x08)
                    {
                        currentStepX++;
                        if (currentStepX == 4)
                            currentStepX = 0 ; 
                    }
                    else
                    {
                        currentStepX = currentStepX - 1;
                        if(currentStepX == -1)
                        currentStepX = 3; 
                    }
                    LATA = (LATA & 0xF0) | (0b00000001 << currentStepX);
                    
                }
                if(currentPoint & 0x02)
                {
                    transientY = 1;
                    if(currentPoint & 0x10)
                    {
                         currentStepY++;
                        if (currentStepY == 4)
                            currentStepY = 0 ; 
                    }
                    else
                    {
                        currentStepY = currentStepY - 1;
                        if(currentStepY == -1)
                        currentStepY = 3;
                    }
                    
                    LATD = (LATD & 0xF0) | (0b00000001 << currentStepY);
                   
                }
                if(currentPoint & 0x04)
                {
                    transientZ = 1;
                    if(currentPoint & 0x20)
                    {
                        currentStepZ++;
                        if (currentStepZ == 4)
                            currentStepZ = 0 ; 
                    }
                    else
                    {
                        currentStepZ = currentStepZ - 1;
                        if(currentStepZ == -1)
                            currentStepZ = 3;
                    }
                    
                    LATD = (LATD & 0x0F) | (0b00010000 << currentStepZ);
                }
            }
            
            if(chop == 0)
            {
                chop = 1;
                
                LATA = (LATA & 0xF0) | (0b00000001 << currentStepX);
                LATD = (LATD & 0xF0) | (0b00000001 << currentStepY);
                LATD = (LATD & 0x0F) | (0b00010000 << currentStepZ);
                //LATA = 0b00000001 << currentStepX;
                T0CONbits.T0PS = 0b010; 
                TMR0L = 237;

            }
            else
            {
                chop = 0;
                ready++;
                if(ready == 19)
                {
                    transientX = 0;
                    transientY = 0;
                    transientZ = 0;
                }
                if(!transientX)
                {
                    LATA &= 0xF0;
                }
                if(!transientY)
                {
                    LATD &= 0xF0;
                }
                if(!transientZ)
                {
                    LATD &= 0x0F;
                }
                //LATA = 0x00;
                T0CONbits.T0PS = 0b010; 
                TMR0L = 242;
        }
        }
        else
        {
            LATA &= 0xF0;
            LATD = 0x00;
        }
      }
         TMR0IF = 0;
    }


void manual_position(){
    LATE0 = 1;
    LATE1 = 1;
    LATE2 = 1;
    while(1){
        
        LATE1 = 0 ; 
        
        if (RC2 == 0 ){
            Delay_Ms(50);
           while (RC2 == 0 ){
           LATA = (LATA & 0xF0) | (0b00000001 << currentStepX);
           Delay_Us(650);
          
           for (char q = 0 ; q < 17;q++){
           LATA &= 0xF0 ;
           Delay_Us(15);
           LATA = (LATA & 0xF0) | (0b00000001 << currentStepX);
           Delay_Us(20);
           }
           
          
           currentStepX++;
           if (currentStepX == 4)
               currentStepX = 0 ; 
           
        }
            
             LATA = (LATA & 0xF0);
             LATD = 0x00; 
             
            
        } else if (RC1 == 0){
            Delay_Ms(50);
            while( RC1 == 0 ){
           LATD = (LATD & 0xF0) | (0b00000001 << currentStepY);
                __delay_us(650);

                for (int q = 0 ; q < 17;q++){
                LATD &= 0xF0 ;
                __delay_us(15);
                LATD = (LATD & 0xF0) | (0b00000001 << currentStepY);
                __delay_us(20);

           }
          
           currentStepY++ ; 
           if (currentStepY == 4)
               currentStepY = 0 ; 
         }
            
            LATA = (LATA & 0xF0);
            LATD = 0x00; 
           
        } else if (RC0 == 0){
            Delay_Ms(50);
            while(RC0 == 0){
            LATD = (LATD & 0x0F) | (0b00010000 << currentStepZ);
                __delay_us(650);

                for (int q = 0 ; q < 17;q++){
                LATD &= 0x0F ;
                __delay_us(15);
                LATD = (LATD & 0x0F) | (0b00010000 << currentStepZ);
                __delay_us(20);

           }
             currentStepZ++ ; 
          if (currentStepZ == 4)
               currentStepZ = 0 ; 
            
            }
            
              LATA = (LATA & 0xF0);
              LATD = 0x00;
        }
        
        LATE1 = 1 ; 
        
        LATE2 = 0 ; 
        
        if (RC2 == 0 ){
            Delay_Ms(50);
            while (RC2 == 0 ){
           LATA = (LATA & 0xF0) | (0b00000001 << currentStepX);
           Delay_Us(650);
           
           
           for (char q = 0 ; q < 17;q++){
           LATA &= 0xF0 ;
           Delay_Us(15);
           LATA = (LATA & 0xF0) | (0b00000001 << currentStepX);
           Delay_Us(20);
           
           }
            
            currentStepX = currentStepX - 1;
            if(currentStepX == -1)
                        currentStepX = 3;       
        }
            
             LATA = (LATA & 0xF0);
             LATD = 0x00; 
            
        }else if (RC1 == 0){
            Delay_Ms(50);
             while(RC1 == 0 ){
             
           LATD = (LATD & 0xF0) | (0b00000001 << currentStepY);
           __delay_us(650);
           
           
          for (int q = 0 ; q < 17;q++){
           LATD &= 0xF0 ;
           __delay_us(15);
           LATD = (LATD & 0xF0) | (0b00000001 << currentStepY);
           __delay_us(20);
           
           }
            
            currentStepY = currentStepY - 1;
            
            if(currentStepY == -1)
                        currentStepY = 3;
         }
            
             LATA = (LATA & 0xF0);
             LATD = 0x00;
            
        }else  if (RC0 == 0){
            Delay_Ms(50);
         while(RC0 == 0){
             
           LATD = (LATD & 0x0F) | (0b00010000 << currentStepZ);
           Delay_Us(650);
             
        for (char q = 0 ; q < 17;q++){
           LATD &= 0x0F ;
           Delay_Us(15);
           LATD = (LATD & 0x0F) | (0b00010000 << currentStepZ);
           Delay_Us(20);
           
           }
         
               currentStepZ = currentStepZ - 1;
                        if(currentStepZ == -1)
                        currentStepZ = 3;
         }
            
             LATA = (LATA & 0xF0);
             LATD = 0x00; 
        } 
         LATE2 = 1 ;
         LATE0 = 0;
         if (RC1 == 0){
             Delay_Ms(50);
             while(RC1 == 0);
             LATE0 = 1;
             break;
         }
         LATE0 = 1 ; 
    }
         
        
    
}
void main(void) {
    
   //OSCCONbits.IRCF = 0b111; 
   // OSCCONbits.SCS = 0b10;
    
   OSCCONbits.SCS =0b00;
    
    TRISC0 = 1 ;
    TRISC1 = 1 ; 
    TRISC2 = 1 ;
    TRISE0 = 0 ;
    TRISE1 = 0 ;
    TRISE2 = 0 ; 
    
    ADCON1bits.PCFG = 0b1111;
    LATE0 = 1 ; 
    LATE1 = 1 ; 
    LATE2 = 1 ; 
    TRISA0 = 0;
    TRISA1 = 0;
    TRISA2 = 0;
    TRISA3 = 0;
    LATA = 0x00;
    TRISD = 0x00;
    LATD = 0x00;
    TRISB1 = 0;
    LATB1 = 1;
    TRISB0 = 0;
    LATBbits.LATB0 = 1;
    __delay_ms(1000);
    LATBbits.LATB0 = 0;
    __delay_ms(1000);
   
    LCD_Init();
    __delay_ms(50);
    LCD_WR("Welcome");
 
    
   USB_Select_File();
   LCD_WR("Move to 0");
   
 
   manual_position();
   LCD_WR("Drawing...");

   
   USB_Req_Bytes_Read(50);

   if(usbStatus == USB_INT_DISK_READ){
		sectorBytesCounter +=USB_Read_Line();
        parse();
                
	}
   
   TIMER0_INIT(0);
   USB_Read_File();
   points_end++;
   if(points_end == 1500)
   {
       points_end = 0;
   }
   points[points_end] = 0x00;
   /////////////////////////////
   LCD_WR("Done");
 
   manual_position(); 
   while(1)
   {    
       /*
        TRISA0 = 0;
        TRISA1 = 0;
        TRISA2 = 0;
        TRISA3 = 0;
        LATA = 0x00;
        TRISD = 0x00;
        LATD = 0x00;
        TRISB1 = 0;
        LATB1 = 1;
        TRISB0 = 0;
        
        chop = 0;         //flag indicate on/off chopping time
        transientX = 0;
        transientY = 0;
        transientZ = 0;

        ready = 22;

        Xpos = 0;
        Ypos = 0;
        Zpos = 0;

        points_first = -1;
        points_end = 0;

        Sindex = 0;
        findex = 0;

        X0 = 0;
        Y0 = 0;
        Z0 = 0;

        X1 = 0;
        Y1 = 0;
        Z1 = 0;
        
        code = 0;
        
        USB_Select_File();
        points_end++;
        if(points_end == 1500)
        {
            points_end = 0;
        }
        points[points_end] = 0x00;
   //LCD_WR("Move to 0");
   LCD_Clear();
    LCD_Set_Cursor(1,1);
    LCD_Write_String("Move to 0");
    __delay_ms(500);
   manual_position();
   //LCD_WR("Drawing...");
   LCD_Clear();
    LCD_Set_Cursor(1,1);
    LCD_Write_String("DRAWING ...");
    __delay_ms(500);
   
   USB_Req_Bytes_Read(50);

   if(usbStatus == USB_INT_DISK_READ){
		sectorBytesCounter +=USB_Read_Line();
        parse();
                
	}
   
   TIMER0_INIT(0);
   USB_Read_File();
   LCD_Clear();
    LCD_Set_Cursor(1,1);
    LCD_Write_String("Done");
    __delay_ms(500);
   manual_position();
   */
   }
    return;
}




