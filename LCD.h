#include <xc.h>
#include "CONFIG.h"
#include "UTIL.h"

#define LCD_EN_Delay 500
#define LCD_DATA_PORT_D TRISB
#define LCD_RS_D TRISB2
#define LCD_EN_D TRISB3
#define RS LATB2
#define EN LATB3
#define D4 LATB4
#define D5 LATB5
#define D6 LATB6
#define D7 LATB7

//==========================================
//-----[ Prototypes For All Functions ]-----
void LCD_Init();
void LCD_Clear();
void LCD_SL();
void LCD_SR();
void LCD_CMD(unsigned char);
void LCD_DATA(unsigned char);
void LCD_Set_Cursor(unsigned char, unsigned char);
void LCD_Write_Char(char);
void LCD_Write_String(char*);
void LCD_WR(const char* str);
void LCD_WRR(const char **str);