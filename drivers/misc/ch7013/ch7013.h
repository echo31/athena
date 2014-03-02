/*++ 
 
Copyright: 
 
    Copyright (c) 2002 BSQUARE Corporation.  All rights reserved. 
 
Module Name: 
 
    bklight.h     
 
Abstract: 
 
         
Author: 
 
    Anand August 2005 
     
Revision History: 
 
--*/ 
 


struct ch7013_display_mode {
	uint8_t DMR;   //display mode  
	uint8_t FFR;      //flicker filter settings  
	uint8_t CM;  //clock mode 
	uint8_t SAV;  //start of active video register
	uint8_t PO;    //position overflow
        uint8_t BLR;  //black level 
	uint8_t HPR;     //horizontal position  
	uint8_t VPR;  //vertical position  
	uint8_t MNE;   //PLLM and PLLN  
	uint8_t PLLM;
        uint8_t PLLN;
        uint32_t FSCI;
        uint8_t PLLC;   //PLL control 
        uint8_t CIVC;   //turn off CIV 
       
	 
};

//Define in PSC_SMBUS.c 
#define	IOCTL_SMBUS_READDATA		0x80002000  // some arbirary base 
#define	IOCTL_SMBUS_WRITEDATA	        0x80002001 
 
#define REG_DMR		0x00 
#define REG_FFR		0x01 
#define REG_VBW	        0x03 
#define REG_IDF		0x04 
#define REG_CM		0x06 
#define REG_SAV		0x07 
#define REG_PO		0x08 
#define REG_BLR		0x09 
#define REG_HPR		0x0A 
#define REG_VPR		0x0B 
#define REG_SPR		0x0D 
#define REG_PMR		0x0E 
#define REG_CDR		0x10 
#define REG_CE		0x11 
#define REG_MNE		0x13 
#define REG_PLLM 	0x14 
#define REG_PLLN 	0x15 
#define REG_BCO		0x17 
 
#define REG_FSI0	0x18 
#define REG_FSI1	0x19 
#define REG_FSI2	0x1A 
#define REG_FSI3	0x1B 
#define REG_FSI4	0x1C 
#define REG_FSI5	0x1D 
#define REG_FSI6	0x1E 
#define REG_FSI7	0x1F 
 
#define REG_PLLC	0x20 
#define REG_CIVC	0x21 
 
#define REG_CIV1	0x22 
#define REG_CIV2	0x23 
#define REG_CIV3	0x24 
 
#define REG_VID		0x25 
 
#define REG_TR1		0x26 
#define REG_TR2		0x27 
#define REG_TR3		0x28 
#define REG_TR4		0x29 
 
#define REG_AR		0x3F 
 
#define CH7013_ID	0x22 
 
// Struct used to send/revceive data 
//define in PSC_SMBUS.h 
 
