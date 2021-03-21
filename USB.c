
#include <xc.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "USB_CONFIG.h"
#include "UTIL.h"
#include "USB.h"
#include "LCD.h"



void USB_Write(uint8_t data){
        while(!TRMT);
        TXREG = data; 
 
}

void USB_Cmd(uint8_t cmd){
   
    USB_Write(fSyncByte);
    USB_Write(sSyncByte);  
    USB_Write(cmd);
}

uint8_t USB_Read(){
    uint8_t readByte = 0xFF  ; 
     uint16_t timer = 0 ;
     while(timer < 65535){
    
        if(!RCIF){
        Delay_Us(4);
    
        }else{
    
        readByte =  RCREG;
        break;
    
        }
    }
  return readByte ;
}


void USB_Reset(){
    USB_Cmd(CMD_RESET_ALL);
    Delay_Ms(200);
    
}


uint8_t USB_Init(){
    
    Delay_Ms(40);
 
    RX_TRIS = 1; //enable serial port
    TX_TRIS = 1 ; 
    SPEN = 1;    //Serial port enabled (Configures RB1/RX/DT and RB2/TX/CK pins as serial port pins when bits TRISB<2:1> are set)   
  
    TX9 = 0;   //1 = Selects 9-bit transmission  , 0 = Selects 8-bit transmission
    TXEN = 1;   //1 = Transmit enabled , 0 = Transmit disabled
    SYNC = 0;    // 1 = Synchronous mode , 0 = Asynchronous mode
    BRGH = 1;
    RX9 = 0;     //1 = Selects 9-bit reception , 0 = Selects 8-bit reception
    CREN = 1;    // Asynchronous mode: 1 = Enables continuous receive , 0 = Disables continuous receive
    BRG16 = 1 ; 
    SPBRG = 103;

    uint8_t isReady = 0;
    
    USB_Reset(); 
    for(int i = 0 ; i < 5 ; i++){
       isReady =  USB_Check_Circuit();
       if (isReady) break; 
       Delay_Ms(100);
    }
    
    return isReady;
}

uint8_t USB_Check_Circuit(){
    
 USB_Cmd(CMD_CHECK_EXIST);
 USB_Write(0x03);
 usbStatus = USB_Read();
 return (255 - 0x03) == usbStatus ;
} 

uint8_t USB_Set_Mode(uint8_t mode){
    USB_Cmd(CMD_SET_USB_MODE);
    USB_Write(mode);
    Delay_Ms(20);
    usbStatus = USB_Read();
    return usbStatus == USB_RET_SUCCESS ? 1 : 0  ; 
}

uint8_t USB_Get_Status(){
    USB_Cmd(CMD_GET_STATUS);
    usbStatus = USB_Read();   
    return usbStatus ; 
}

uint8_t USB_Disk_Mount(){
    USB_Cmd(CMD_DISK_MOUNT);
    usbStatus = USB_Read();
    return usbStatus  == USB_INT_SUCCESS ? 1 : 0  ; 
}

uint8_t USB_Disk_Connected(){
    USB_Cmd(CMD_DISK_CONNECT);
    usbStatus = USB_Read();
    return usbStatus ==  USB_INT_SUCCESS ? 1 : 0 ;  
}

uint8_t USB_Set_File_Name(const char* name){
    
	USB_Cmd(CMD_SET_FILE_NAME);
    uint8_t nameLength = strlen(name);
    
	if (nameLength < 14)
	{
		for (uint8_t i = 0; i < nameLength; ++i)
		{
           
			USB_Write(name[i]);
		}
		USB_Write('\0');
        Delay_Ms(10);
		return 1 ;
	}
	else
	{ 
		return 0;
	} 
}

  

void msg(const char* msg){
    LCD_WR(msg);
}

void USB_Print_Status(uint8_t status){
	switch (status){			// To save RAM for small MCUs, comment out both in this
											//  file and CH376_Utility the following functions: 
	case USB_RET_SUCCESS: msg("operation_Successful");//#1. This print_interrupt_status() function. #2. msg() function 
		break;											// #3. print_interrupt_status() call in Interrupt Service Routine.
	case USB_INT_SUCCESS: msg("USB_INT_SUCCESS");
		break;
	case USB_INT_CONNECT: msg("usb Plugged");
		break;
	case USB_INT_DISCONNECT: msg("usb Unplugged");
		break;
	case USB_INT_USB_READY: msg("device_Ready");
		break;
	case USB_INT_DISK_READ: msg("USB_INT_DISK_READ");
		break;
	case USB_INT_DISK_WRITE: msg("disk_Write_Operation");
		break;
	case CMD_RET_ABORT: msg("operation_Failure");
		break;
	case USB_INT_BUF_OVER: msg("buffer_Overflow");
		break;
	case ERR_OPEN_DIR: msg("DIR OPENED");
		break;
	case ERR_MISS_FILE: msg("file_Not_Found");
		break;
	case ERR_FOUND_NAME: msg("err_Found_Name");
		break;
	case ERR_DISK_DISCON: msg("disk_Disconnected");
		break;
	case ERR_LARGE_SECTOR: msg("err_Large_Sector");
		break;
	case ERR_TYPE_ERROR: msg("invalid_Partition_Type");
	break;
	case ERR_BPB_ERROR: msg("partition_Not_Formatted");
	break;
	case ERR_DISK_FULL: msg("disk_Full");
	break;
	case ERR_FDT_OVER: msg("directory_Full");
	break;
	case ERR_FILE_CLOSE: msg("closed_Op");
	break;
	default: msg("unimplemented_Status");
	}

}

uint8_t USB_Open_File(const char *name){
    uint8_t isFileSet = USB_Set_File_Name(name);
 
    if (isFileSet){
        
       USB_Cmd(CMD_FILE_OPEN);
        
    }else {
        
        LCD_WR("NOT SETTTED");
    }
    usbStatus = USB_Read();
    Delay_Ms(100);
    
    return usbStatus == USB_INT_SUCCESS  ? 1 : 0 ; 
    
}

uint8_t Locate_Byte(uint32_t offset){
 
    USB_Cmd(CMD_BYTE_LOCATE);
    
    for(uint8_t shifter = 0 ; shifter < 25 ; shifter += 8 ){
    
        USB_Write((uint8_t)(offset >> shifter));
    }
    
    usbStatus = USB_Read();
    return usbStatus == USB_INT_SUCCESS ? 1 : 0 ; 
}

int Read_Block(char *buff){
        
    USB_Cmd(CMD_RD_USB_DATA0); 
    int len  = USB_Read();
    
    if (len){
        for(uint16_t s =0; s < len; s++){
            buff[s] = USB_Read();
		}
    } else {
        LCD_WR("WHYY");
    }
    return len ; 

}

int USB_Dir_Info_Read(){
    
    USB_Cmd(CMD_DIR_INF0_READ);
    USB_Write(0xFF);
    
    usbStatus = USB_Read();
    int dataLen = USB_Read_Fat_Data();
    return dataLen ;
}

int USB_Read_Fat_Data(){
 int dataLen  = Read_Block(fileInfoBuffer); 
  memcpy(&fileInfo , &fileInfoBuffer , sizeof(fileInfoBuffer));
    return dataLen ;  
}

 uint8_t USB_File_Enum(){
     
     USB_Cmd(CMD_FILE_ENUM_GO);
     usbStatus = USB_Read();
     Delay_Ms(10);
     return usbStatus == USB_INT_DISK_READ ? 1 : 0 ;  
   }
 
 uint8_t USB_Continue_Read(){
     USB_Cmd(CMD_BYTE_RD_GO);
     usbStatus = USB_Read();
     return usbStatus ;
 }
 
 uint8_t USB_Req_Bytes_Read(uint8_t numBytes) {
      USB_Cmd(CMD_BYTE_READ);
      USB_Write(numBytes); 
      USB_Write(0x00);
      usbStatus = USB_Read();
      return usbStatus;
}
 
 uint8_t  USB_Read_Line(){  
    USB_Cmd(CMD_RD_USB_DATA0); 
    int len  = USB_Read();
    if (len){

        for(uint16_t i =0; i < len; i++){
            
              uint8_t recByte = USB_Read();
              bytesCounter++; 
             if (recByte == 'M'){
                  while (recByte != '\n'){
                      recByte = USB_Read();
                      bytesCounter++;
                      i++;
                  }
                  continue;
              }
              
              if (recByte != '\n'){
                linesBuffer[wrIndex][lineByteCounter++] = recByte ; 
              } else {
    
               buffFlags[wrIndex] = 1 ;    
               wrIndex++;
               if (wrIndex == 3)
                   wrIndex = 0 ; 
               lineByteCounter = 0 ;  
              }
		}
       
          
    } else {
        LCD_WR("WHYY");
    }
    
    return len ;
 }
 
  void USB_Read_File(){
      
      uint16_t  byteForRequest = 50 ; 
      
      
      while (bytesCounter < fileInfo.size){	
          
          if(sectorBytesCounter == SECTOR_SIZE){ 
					sectorBytesCounter = 0;
                    usbStatus = USB_Continue_Read();
                    continue;
                    
            } else if ((sectorBytesCounter + byteForRequest) > SECTOR_SIZE){
					byteForRequest = SECTOR_SIZE - sectorBytesCounter;
			}
          
            USB_Req_Bytes_Read(byteForRequest);
                  
            if(usbStatus == USB_INT_DISK_READ){
                
				sectorBytesCounter +=USB_Read_Line();
                // PARSE Comment while Code Below
                parse();
                
			} else if(usbStatus == USB_INT_SUCCESS){ 
             
				break;
                
			}
      }
   
  }
 
 uint8_t USB_Close_File() {
     USB_Cmd(CMD_FILE_CLOSE);
     USB_Write(0x00);
     usbStatus = USB_Read();
     return usbStatus ; 
 }

 void USB_Select_File() {
     
    uint8_t isReady =  USB_Init();
            
    if (isReady){


        uint8_t isModeSet = USB_Set_Mode(MODE_HOST_2);
      
        if (isModeSet){
        
            uint8_t usbConnection = USB_Read();
          
            if(usbConnection == USB_INT_CONNECT){                               
            
                         
                uint8_t isDiskConnectionOk = USB_Disk_Connected();        
               
                if (isDiskConnectionOk){
                
                    uint8_t isMounted = USB_Disk_Mount();
      
                    if (isMounted){
                        LCD_WR("USB mounted");
                       
                           
                       USB_Open_File("/*");
                       
                        
                        USB_File_Enum();
                        USB_Read_Fat_Data();
                        Display_File_Name();
           
                           while(1){
                               
                               LATE0 = 0 ;
                               
                               if (RC2 == 0){
                                   Delay_Ms(50);
                                   while(RC2 == 0);
                                   
                                   USB_File_Enum();
                                    
                                    if (usbStatus  == ERR_MISS_FILE){
                                        USB_Close_File();
                                        USB_Open_File("/*");
                                        USB_Read_Fat_Data();
                                        USB_File_Enum();
                                        USB_Read_Fat_Data();
                                        Display_File_Name();
                                        //LATE0 = 1 ;
                                        continue;
                                    }
                        
                                    USB_Read_Fat_Data();
                                    Display_File_Name();
                                    
                                   
                               }else if (RC1 == 0){
                                   Delay_Ms(50); 
                                   while(RC1 == 0);
                                   
                                   if ((fileInfo.extension[0] == 'M' && 
                                            fileInfo.extension[1] == 'M' && 
                                            fileInfo.extension[2] == 'G')){
                                            USB_Close_File();
                                            strcat(displayName,fileInfo.name);
                                            USB_Open_File(displayName);
                                            USB_Dir_Info_Read();
                                                LATE0  = 1 ;          
                                            LCD_WR("File Selected");
                                            break;
                                        }else{
                                            LCD_WR("File Ext invalid");
                                        }
                               }
                               LATE0 = 1 ; 
                            }
                         
                    } else {
                        LCD_WR("mount Failed");
                    }      
                }else {
            
                    LCD_WR("USB Failed");
                }
                
            } else {
                
                LCD_WR("Connection Failed");
            }
            
        } else {

        LCD_WR("Mode Not Set");

        }
        
    } else {
            
            LCD_WR("Circuit Invalid"); 
        
    }
    LATE0 = 1;
    LATE1 = 1;
    LATE2 = 1;
}
 void Display_File_Name(){
    /*
    uint8_t i = 0;
     while(fileInfo.name[i] != ' '){
         displayName[i+1] = fileInfo.name[i] ; 
         i++;
     }
     displayName[i++] = '.';
     displayName[i++] = fileInfo.extension[0] ; 
     displayName[i++] = fileInfo.extension[1] ; 
     displayName[i++] = fileInfo.extension[2] ;
     displayName[i]  = '\0' ; 
     LCD_WR(displayName);
      */
    
     
     LCD_WR(fileInfo.name);
 }

