/* 
 * File:   USB.h
 * Author: George Ayman
 * 
 *
 * Created on August 9, 2020, 1:01 PM
 */

#ifndef USB_H
#define	USB_H

   
#include <stdint.h>
#include <stdint.h>
#define SECTOR_SIZE 512 


#define BuffSize 50
uint32_t bytesCounter  = 0 ; 
uint32_t sectorBytesCounter = 0 ; 
     
    
char displayName[16] = "/" ; 

struct FileInfo {
    uint8_t  name[8];
    uint8_t  extension[3];
    uint8_t  attributes ; 
    uint8_t  userAttributes;
    uint8_t  deleted ; 
    uint16_t timeCreated;
    uint16_t dateCreated;
    uint16_t ownerIds ;
    uint16_t accessRights ; 
    uint16_t timeCreatedOrUpdated;
    uint16_t dateCreatedOrUpdated ; 
    uint16_t startingClusterNum ;
    uint32_t size;

};



uint8_t interface; 
uint8_t err = 0x00 ;
uint8_t lineByteCounter = 0 ;
uint8_t linesBuffer[3][BuffSize];
uint8_t buffFlags[3] = {0,0,0};
uint8_t wrIndex = 0;
uint8_t rdIndex = 0; 
uint32_t counter = 1;
uint32_t sectorCounter =  0  ; 
uint8_t usbStatus ;  
uint8_t bytesNum = 0 ;
char fileInfoBuffer[32]; 
uint8_t str[10];

struct FileInfo fileInfo ; 

#define fSyncByte 0x57
#define sSyncByte 0xAB
#define UART      1 
#define SPI       2 

#define SDO_TRIS TRISC5
#define SDO  RC5

#define SDI_TRIS TRISC4
#define SDI RC4

#define SCK_TRIS TRISC3 
#define SCK RC3

#define RX_TRIS TRISC7
#define TX_TRIS TRISC6

#define downBtn PORTCbits.RC0
#define downBtn_TRIS TRISCbits.TRISC0

#define okBtn   PORTCbits.RC1
#define okBtn_TRIS TRISCbits.TRISC1

    void    USB_Write(uint8_t data);
    void    USB_Cmd(uint8_t cmd);
    uint8_t USB_Read(void);
    void    USB_Reset(void);
    uint8_t USB_Init(void);
    void    Print_Status(uint8_t status);
    uint8_t USB_Check_Circuit(void);
    uint8_t USB_Set_Mode(uint8_t mode);
    uint8_t USB_Get_Status(void);
    uint8_t USB_Disk_Mount(void);
    uint8_t USB_Disk_Connected(void);
    uint8_t USB_Set_File_Name(const char *filename);
    uint8_t USB_Open_File(const char *filename);
    uint8_t USB_Open_File_Directory(const char *path);
    uint8_t Locate_Byte();
    int     Read_Block(char *buff);
    int     USB_Dir_Info_Read();
    int32_t Read_File_Byte(char *buff, uint16_t length);
    uint8_t USB_File_Enum(void);
    uint8_t ENUM_Init(void);
    uint8_t USB_Req_Bytes_Read(uint8_t numBytes);
    uint8_t USB_Close_File(void);
    int     USB_Read_Fat_Data(void);
    uint8_t USB_Continue_Read(void);
    void    USB_Select_File(void);
    uint8_t    USB_Read_Line();
      void USB_Read_File(void);
   void Display_File_Name(void);
   void parse();

#endif	/* USB_H */

