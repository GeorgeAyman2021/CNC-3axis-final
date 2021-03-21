/* 
 * File:   USB_CONFIG.h
 * Author: George Ayman
 *
 * Created on August 9, 2020, 1:08 PM
 */

#ifndef USB_CONFIG_H
#define	USB_CONFIG_H

#define FF 0xFF
#define  ATTR_DIRECTORY = 0x10
    
    
    /**/
/* COMMANDS */
#define CMD_GET_IC_VERSION 0x01 //Obtain chip and firmware version number
#define CMD_SET_BAUDRATE 0x02   //Set serial communication baud rate
#define CMD_ENTER_SLEEP 0x03    // Go to low-power and suspending
#define CMD_RESET_ALL 0x05      //Execute hardware reset
#define CMD_CHECK_EXIST 0x06    // Test communication interface and working status
#define CMD_SET_SD0_INT 0x0B    //Set interrupt Mode of SD0 in SPI
#define CMD_GET_FILE_SIZE 0x0C  // Get the current file length
#define CMD_SET_USB_MODE 0x15   //Configure the work mode of USB
#define CMD_GET_STATUS 0x22     // Get interruption status and cancel requirement
#define CMD_RD_USB_DATA0 0x27   // Read data from current interrupt port buffer of USB or receive buffer of host port
#define CMD_WR_USB_DATA 0x2C    //Write data to transfer buffer of USB host
#define CMD_WR_REQ_DATA 0x2D    //Write requested data block to internal appointed buffer
#define CMD_WR_OFS_DATA 0x2E    //Write data block to internal buffer with appointed excursion address
#define CMD_SET_FILE_NAME 0x2F  //Set the file name which will be operated
#define CMD_DISK_CONNECT 0x30   // Check the disk connection status
#define CMD_DISK_MOUNT 0x31     //Initialize disk and test disk ready
#define CMD_FILE_OPEN 0x32      // Open file or catalog, enumerate file and catalog
#define CMD_FILE_ENUM_GO 0x33   // Go on to enumerate file and catalog
#define CMD_FILE_CREATE 0x34    //Create file
#define CMD_FILE_ERASE 0x35     //Delete file
#define CMD_FILE_CLOSE 0x36     // Close the open file or catalog
#define CMD_DIR_INF0_READ 0x37  // Read the catalog information of file
#define CMD_DIR_INF0_SAVE 0x38  // Save catalog information of file
#define CMD_BYTE_LOCATE 0x39    // Move the file pointer take byte as unit
#define CMD_BYTE_READ 0x3A      // Read data block from current location take byte as unit
#define CMD_BYTE_RD_GO 0x3B     //Continue byte read
#define CMD_BYTE_WRITE 0x3C     // Write data block from current location take unit as unit
#define CMD_BYTE_WR_GO 0x3D     //Continue byte write
#define CMD_DISK_CAPACITY 0x3E  // Check disk physical capacity
#define CMD_DISK_QUERY 0x3F     // Check disk space
#define CMD_DIR_CREATE 0x40     // Create catalog and open or open the existed catalog
#define CMD_SEG_LOCATE 0x4A     // Move file pointer from current location take fan as unit
#define CMD_SEC_READ 0x4B       // Read data block from current location take fan as unit
#define CMD_SEC_WRITE 0x4C      // Write data block from current location take fan as unit
#define CMD_DISK_BOC_CMD 0x50   // Execute B0 transfer protocol command to USB Storage
#define CMD_DISK_READ 0x54      // Read physical fan from USB storage device
#define CMD_DISK_RD_GO 0x55     // Go on reading operation of USB storage device
#define CMD_DISK_WRITE 0x56     // Write physical fan to USB storage device
#define CMD_DISK_WR_GO 0x57     // Go on writing operation of USB storage device
    /**/
    /* operation state */
#define CMD_RET_SUCCESS 0x51 // Operation successful
#define CMD_RET_ABORT 0x5F   // Operation failure
 
    /*  STATUS */
#define USB_RET_SUCCESS		0x51
#define USB_INT_SUCCESS 0x14    // Success of SD card or USB transaction or transfer operation
#define USB_INT_CONNECT 0x15    // Detection of USB device attachment
#define USB_INT_DISCONNECT 0x16 // Detection of USB device detachment
#define USB_INT_BUF_OVER 0x17   // Data error or Buffer overflow
#define USB_INT_USB_READY 0x18  //USB device has initialized (appointed USB address)
#define USB_INT_DISK_READ 0x1D  // Read operation of storage device
#define USB_INT_DISK_WRITE 0x1E //Write operation of storage device
#define USB_INT_DISK_ERR 0x1F   // Failure of USB storage device
/**/
 
    

/* error code of SD card or USB-HOST mode */
#define ERR_OPEN_DIR 0x41     // Open directory address is appointed
#define ERR_MISS_FILE 0x42    // File doesn?t be found which address is appointed, maybe the name is error
#define ERR_FOUND_NAME 0x43   // Search suited file name, or open directory according the request, open file in actual
#define ERR_DISK_DISCON 0x82  // Disk doesn?t connect, maybe the disk has cut down
#define ERR_LARGE_SECTOR 0x84 // Fan is too big, only support 512 bytes
#define ERR_TYPE_ERROR 0x92   // Disk partition doesn?t support, re-prartition by tool
#define ERR_BPB_ERROR 0xA1    // Disk doesn?t format, or parameter is errot, re-formate by WINDOWS with default parameter
#define ERR_DISK_FULL 0xB1    // File in disk is full, spare space is too small
#define ERR_FDT_OVER 0xB2     // Many file in directory, no spare directory, clean up the disk, less than 512 in FAT12/FAT16 root directory
#define ERR_FILE_CLOSE 0xB4   // File is closed, re-open file if need

    /**/

#define USB_FILE_OPENED USB_INT_SUCCESS 
#define USB_DIR_OPENED  ERR_OPEN_DIR
#define USB_ENUM_FILE_SUCCESS USB_INT_DISK_READ  

    /* MODES */
	const uint8_t MODE_HOST_0 = 0x05;
	const uint8_t MODE_HOST_1 = 0x07;
	const uint8_t MODE_HOST_2 = 0x06;
	const uint8_t MODE_HOST_SD = 0x03;
	const uint8_t MODE_DEFAULT = 0x00;

#endif	/* USB_CONFIG_H */

