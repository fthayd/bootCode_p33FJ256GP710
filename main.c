/********************************************************************
* FileName:		main.c
* Dependencies:    
* Processor:		dsPIC33F Family
* Hardware:		Explorer 16
* Compiler:		C30 2.01
* Company:		Microchip Technology, Inc.
*
* Software License Agreement
*
* The software supplied herewith by Microchip Technology Incorporated
* (the “Company”) for its PICmicro® Microcontroller is intended and
* supplied to you, the Company’s customer, for use solely and
* exclusively on Microchip PICmicro Microcontroller products. The
* software is owned by the Company and/or its supplier, and is
* protected under applicable copyright laws. All rights are reserved.
* Any use in violation of the foregoing restrictions may subject the
* user to criminal sanctions under applicable laws, as well as to
* civil liability for the breach of the terms and conditions of this
* license.
*
* THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
* IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
* CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
MODIFICATIONS :

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
*********************************************************************/

#include "p33Fxxxx.h"

#define ClrWdt() {__asm__ volatile ("clrwdt");}

//#define INGILIZCE_CINCE_DEVICE
//#define INGILIZCE_TURKCE_DEVICE

//#define MPU_IN	//manyetik pikap giriþli cihaz (TRANS-AMF_MPU)
//#define POSITIVE_TRANSISTOR_OUTS	// Pozitif transistor çýkýþlý cihaz (V02 ve üstü PCB' ler ile çalýþabilmesi için)

//#define	MODUL_HC245_GATE_L		LATDbits.LATD15

//#define	RELAY1_L 		LATEbits.LATE4
//#define Mains_Close_Relay	RELAY1_L

#define SDA_T			TRISGbits.TRISG3	
#define SCL_T			TRISGbits.TRISG2

#define SDA_L				LATGbits.LATG3	
#define SCL_L				LATGbits.LATG2

#define SDA_P				PORTGbits.RG3
#define SCL_P				PORTGbits.RG2

#define eeprom_nop_adet	40

#define EEPROM_R_address	0x0A1	// 0B1010 0001
#define EEPROM_W_address	0x0A0	// 0B1010 0000

#define EEPROM1_R_address0	0x0A1	// 0B1010 0001 "U11 DEVICE"
#define EEPROM1_W_address0	0x0A0	// 0B1010 0000

//#define Breaker_Contact_Type_NO	0

//#define DATA_RECORD		0x00
//#define END_OF_FILE		0x01
//#define EXTENDED_ADDRESS	0x04

#define TIME_OUT_VALUE	0xFF
#define CR	13
#define LF	10

//#define MODEM_COMMAND		0x52	// 'R'
//#define REV_SORGU			0x3A	// ':'
#define REV_SORGU			0x01	// Modbus address
//#define UPDATE_PM_REQ		0x55	// 'U'
//#define PROGRAM_ONE_LINE	0x53	// 'S'

#define COMMAND_NACK     '!'/*0x00*/
#define COMMAND_ACK      '*'/*0x01*/
#define COMMAND_READ_PM  '?'/*0x02*/
#define COMMAND_WRITE_PM 'S'/*0x03*/
#define COMMAND_WRITE_CM 'U'/*0x07*/
#define COMMAND_RESET    'L'/*0x08*/
#define COMMAND_READ_ID  'M'/*0x09*/

#define PM_ROW_SIZE 64 * 8
#define CM_ROW_SIZE 8
#define CONFIG_WORD_SIZE 1

#define PM_ROW_ERASE 		0x4042
#define PM_ROW_WRITE 		0x4001	// program 1 row (64 instruction words)
#define PM_ONE_WORD_WRITE	0x4003	// program a code memory word
#define CONFIG_WORD_WRITE	0X4000

#define FCY   			40000000
//#define BAUDRATE        9600/*115200*/
#define BAUDRATE        19200
#define BRGVAL          ((FCY/BAUDRATE)/16)-1 

#define	POWERLED_L			LATDbits.LATD1 // power ledi

// External Oscillator
_FOSCSEL(FNOSC_PRIPLL);								// Primary (XT, HS, EC) Oscillator with PLL
_FOSC(FCKSM_CSDCMD & OSCIOFNC_OFF  & POSCMD_XT);	// Clock Switching and Fail Safe Clock Monitor is disabled
													// OSC2 Pin Function: OSC2 is Clock Output
													// Primary Oscillator Mode: XT Crystanl
_FWDT(FWDTEN_ON & WINDIS_OFF & WDTPRE_PR128 & WDTPOST_PS512);	//2048msn'lik watchdog enable
//_FWDT(FWDTEN_OFF); 						            // Watchdog Timer Enabled/disabled by user software
													// (LPRC can be disabled by clearing SWDTEN bit in RCON register
//_FPOR(PWRTEN_OFF);  					 			// Turn off the power-up timers.
//_FGS(GCP_OFF);           							// Disable Code Protection

//**********************************************************************
struct Char_Bitmap
{	unsigned  bit0 : 1;
	unsigned  bit1 : 1;
	unsigned  bit2 : 1;
	unsigned  bit3 : 1;
	unsigned  bit4 : 1;
	unsigned  bit5 : 1;
	unsigned  bit6 : 1;
	unsigned  bit7 : 1;
};
//**********************************************************************
struct Int_Bitmap
{	unsigned  bit0  : 1;
	unsigned  bit1  : 1;
	unsigned  bit2  : 1;
	unsigned  bit3  : 1;
	unsigned  bit4  : 1;
	unsigned  bit5  : 1;
	unsigned  bit6  : 1;
	unsigned  bit7  : 1;
	unsigned  bit8  : 1;
	unsigned  bit9  : 1;
	unsigned  bit10 : 1;
	unsigned  bit11 : 1;
	unsigned  bit12 : 1;
	unsigned  bit13 : 1;
	unsigned  bit14 : 1;
	unsigned  bit15 : 1;
};
//**********************************************************************
union Common_2Byte_Variables
{	int INT;
	char BYTE[2];
	unsigned int UINT;
	unsigned char UBYTE[2];
	struct Char_Bitmap BIT[2];
	struct Int_Bitmap BITS;
};
//**********************************************************************

typedef short          Word16;
typedef unsigned short UWord16;
//typedef unsigned int UWord16;
typedef long           Word32;
typedef unsigned long  UWord32;

typedef union tuReg32
{
	UWord32 Val32;

	struct
	{
		UWord16 LW;
		UWord16 HW;
	} Word;

	char Val[4];
//	unsigned char Val[4];
} uReg32;

extern UWord32 ReadLatch(UWord16, UWord16);
void PutChar(/*unsigned */char);
void GetChar(/*unsigned */char *);
/*unsigned */char getdigit(void);
/*unsigned */char getbinary(void);
/*unsigned */char Bin_To_Ascii(/*unsigned */char);
void WriteBuffer(/*unsigned */char *, int);
void ReadPM(/*unsigned */char *, uReg32);
void WritePM(/*unsigned */char *, uReg32);

//unsigned int read_word_from_external_eeprom(unsigned int eeprom_word_address);
//unsigned char read_byte_from_external_eeprom(unsigned int eeprom_address);
void sequential_read_from_ext_eeprom(unsigned long address, unsigned char* register_ptr, unsigned int number_of_byte);
//void write_word_to_external_eeprom(unsigned int eeprom_word_address, unsigned int write_word);
//void write_byte_to_external_eeprom(unsigned int eeprom_address, unsigned char write_byte);
void write_byte_to_ext_eeprom(unsigned long address, unsigned char* register_ptr, unsigned int number_of_byte, unsigned char enable_next_byte);
unsigned char receive_byte_from_eeprom(unsigned char acknowledge);
void transmit_byte_to_eeprom(unsigned char transmit_byte);
void acknowledge_send_to_eeprom(void);
void start_communication_with_eeprom(void);
void stop_communication_with_eeprom(void);
void Wait_nop(unsigned char);

/*unsigned */char Buffer[PM_ROW_SIZE*3 + 1];	//son byte checksum için
//unsigned char CheckSumCalcBuf[40];
//unsigned char FlashBuffer[16];

//#define prgBOOT_MAINCLconty		1893	//BOOT için mains close breaker contact type
//#define prgBOOT_MABREA_HRDSL 	1894	//BOOT için mains breaker hardware selection
#define prgSoftwareUpdate_Reg	1895	//PC' den yazýlým update isteði için

uReg32 ExtendedSourceAddr;

unsigned int MAINCLconty,Breaker_Hard_Reg;
unsigned long VppDelayCtr,TimeOutSayilan_Reg,TimeOutKarsilastirilan_Reg;
unsigned char bytecount,b0byte0,Record_type,checksum,CalcCheckSum,Time_OutFlag_Reg,Min_TimeOutFlag_Reg,LookMinTimeOutFlag_Reg,Relay_reg;
unsigned char LookRtu_TimeOutFlag_Reg;
unsigned char UpdateHexFile_Reg,UpdateHexFile_Reg2;
unsigned char Active_UART_Reg;

int main(void)
{
	unsigned long i;
	/*unsigned */char receive_byte;

	//Using External oscillator at 4MHz
	//The settings below set up the oscillator and PLL for 40 MIPS as
	//follows:
	//            Crystal Frequency  * (DIVISOR+2)
	// Fcy =     ---------------------------------
	//              PLLPOST * (PRESCLR+2) * 4	
	//-----------------------------------------------------
	//							 1			        M
	//			Fcy= Fosc/2 = (------) FIN * (------------)
	//							 2	             N1 * N2
	//	M  = PLL çarpým katsayý   (2...513)
	//	N1 = PLL input  prescaler (2...33)
	//	N2 = PLL output prescaler (2,4,8)
	//-----------------------------------------------------
    PLLFBD = 0x004E;			// M = 80
    CLKDIV = 0x0000;			// N1 = 2,  N2 = 2  Fcy = 40MHz = 40MIPS "Million Instruction Per Second"

	while(OSCCONbits.LOCK!=1) {}; /* Wait for PLL to lock*/
	
	U1BRG = BRGVAL;      /*  BAUD Rate Setting of Uart1  (0XEF for 9600)*/
	U1MODE = 0x8000; 	/* Reset UART to 8-n-1, alt pins, and enable */
	U1STA  = 0x0400; 	/* Reset status register and enable TX */

//	U2BRG = BRGVAL;      /*  BAUD Rate Setting of Uart2  (0XEF for 9600)*/
//	U2MODE = 0x8000; 	/* Reset UART to 8-n-1, alt pins, and enable */
//	U2STA  = 0x0400; 	/* Reset status register and enable TX */

//---------------------------------------------------------------------------
	LATA  = 0xFFFF;	// drive state low
	TRISA = 0xFFFF;// 1111 1111 1111 1111
//---------------------------------------------------------------------------
	LATB  = 0xFFC7; 	// drive state low
	TRISB = 0xFFC7; 	// 1111 1111 1100 0111
//---------------------------------------------------------------------------
	LATC  = 0xFFFF; //drive state low
	TRISC = 0xFFFF; // 1111 1111 1111 1111
//---------------------------------------------------------------------------
	LATD  = 0xFFFD; // drive state low
	TRISD = 0xFFFD;	// 1111 1111 1111 1101 
//---------------------------------------------------------------------------
	LATE  = 0xFFFF; //drive state low
	TRISE = 0xFFFF; // 1111 1111 1111 1111
//---------------------------------------------------------------------------
	LATF  = 0xFFFF; // 1100 1110 0011 1100 
	TRISF = 0xFFD7; // 1111 1111 1101 0111
//---------------------------------------------------------------------------
	LATG  = 0xFFFF; // 0000 1100 0011 1100 
	TRISG = 0xFFF3;	// 1111 1111 1111 0011
	ODCG  = 0x000C; // SDA,SCL is open-drain  
/*---------------------------------------------------------------------------*/	
	LookMinTimeOutFlag_Reg = 0;

// wait at least 2 second 
	for(i = 0; i < 6000000; i++)
	{
		ClrWdt();	//clear watchdog timer (yaklaþýk 2000msn. bekler)
		Nop();
	}
//	PutChar('/');	// sonra sil

	i = 0;
	do{
		sequential_read_from_ext_eeprom((unsigned long)((prgSoftwareUpdate_Reg*2) +1), &UpdateHexFile_Reg, 1);
		sequential_read_from_ext_eeprom((unsigned long)((prgSoftwareUpdate_Reg*2) +1), &UpdateHexFile_Reg2, 1);
	}
	while((UpdateHexFile_Reg != UpdateHexFile_Reg2) && (i < 5));	//5 kere oku		
	
	if((UpdateHexFile_Reg == UpdateHexFile_Reg2) && (UpdateHexFile_Reg != 255))
	{//Blok Otoklav yazýlýmýna (0xC02) dallanýr.
		ResetDevice();
	}

	while(1)
	{
		/*unsigned */char Command;
		POWERLED_L = 1;		//bootta olduðunu gösterir

		ClrWdt();	//clear watchdog timer
		
		GetChar(&Command);

		switch(Command)
		{
			case REV_SORGU:	// 1
			{//Eðer revizyon sorgusunun ([01][04][00][00][00][01][31][CA]) ilk karakteri ise
				POWERLED_L = 0;		//haberleþmede olduðunu gösterir

//				LookMinTimeOutFlag_Reg = 1;
				LookRtu_TimeOutFlag_Reg = 1;
//REVIZYON_SORGU:
				GetChar(&receive_byte);
				if(Time_OutFlag_Reg || Min_TimeOutFlag_Reg)  goto TIME_OUT_CASE;
				if(receive_byte == 0x04)
				{// (komut
					GetChar(&receive_byte);
					if(Time_OutFlag_Reg || Min_TimeOutFlag_Reg)  goto TIME_OUT_CASE;
					if(receive_byte == 0x00)
					{// (Adres MSB)
						GetChar(&receive_byte);
						if(Time_OutFlag_Reg || Min_TimeOutFlag_Reg)  goto TIME_OUT_CASE;
						if(receive_byte == 0x00)
						{// (Adres LSB)
							GetChar(&receive_byte);
							if(Time_OutFlag_Reg || Min_TimeOutFlag_Reg)  goto TIME_OUT_CASE;
							if(receive_byte == 0x00)
							{// (Adet MSB)
								GetChar(&receive_byte);
								if(Time_OutFlag_Reg || Min_TimeOutFlag_Reg)  goto TIME_OUT_CASE;
								if(receive_byte == 0x01)
								{// (Adet LSB)
									GetChar(&receive_byte);
									if(Time_OutFlag_Reg || Min_TimeOutFlag_Reg)  goto TIME_OUT_CASE;
									if(receive_byte == 0x31)
									{// (CRC MSB)
										GetChar(&receive_byte);
										if(Time_OutFlag_Reg || Min_TimeOutFlag_Reg)  goto TIME_OUT_CASE;
										if((unsigned char)receive_byte == 0xCA)
										{// (CRC LSB)
										 // [01] [04] [02] [01] [FF] [F9] gönder
											PutChar(0x01);//Slave ID
											PutChar(0x04);//okuma komutu
											PutChar(0x02);//byte sayýsý	
											PutChar(0x01);//Cihaz tipi ve rev. no MSB
											PutChar(0xFF);//Cihaz tipi ve rev. no LSB	
											PutChar(0xF8);//CRC deðeri MSB
											PutChar(0xE0);//CRC deðeri LSB
										}
									}							
								}
							}						
						}					
					}			
				}
				LookRtu_TimeOutFlag_Reg = 0;
				break;
			}

			case COMMAND_READ_PM:				/*tested*/
			{
				uReg32 SourceAddr;

				POWERLED_L = 0;		//haberleþmede olduðunu gösterir
				LookMinTimeOutFlag_Reg = 1;
				SourceAddr.Val[0] = getbinary();
				if(Time_OutFlag_Reg || Min_TimeOutFlag_Reg)  goto TIME_OUT_CASE;
				SourceAddr.Val[1] = getbinary();
				if(Time_OutFlag_Reg || Min_TimeOutFlag_Reg)  goto TIME_OUT_CASE;
				SourceAddr.Val[2] = getbinary();
				if(Time_OutFlag_Reg || Min_TimeOutFlag_Reg)  goto TIME_OUT_CASE;
				SourceAddr.Val[3]=0;

				ReadPM(Buffer, SourceAddr);

				WriteBuffer(Buffer, PM_ROW_SIZE*3);
				LookMinTimeOutFlag_Reg = 0;
				break;
			}

			case COMMAND_WRITE_PM:				/* tested */
			{
			    uReg32 SourceAddr;
				int    Size;

				POWERLED_L = 0;		//haberleþmede olduðunu gösterir
				LookMinTimeOutFlag_Reg = 1;
				CalcCheckSum = 0;
				CalcCheckSum += COMMAND_WRITE_PM;
				SourceAddr.Val[0] = getbinary();
				if(Time_OutFlag_Reg || Min_TimeOutFlag_Reg)  goto TIME_OUT_CASE;
				CalcCheckSum += SourceAddr.Val[0];
				SourceAddr.Val[1] = getbinary();
				if(Time_OutFlag_Reg || Min_TimeOutFlag_Reg)  goto TIME_OUT_CASE;
				CalcCheckSum += SourceAddr.Val[1];
				SourceAddr.Val[2] = getbinary();
				if(Time_OutFlag_Reg || Min_TimeOutFlag_Reg)  goto TIME_OUT_CASE;
				CalcCheckSum += SourceAddr.Val[2];
				SourceAddr.Val[3]=0;
				
				for(Size = 0; Size < PM_ROW_SIZE*3; Size++)
				{// bufferýn son byte'ý checksum için (1536*2 karakter için)
					Buffer[Size] = getbinary();
					if(Time_OutFlag_Reg || Min_TimeOutFlag_Reg)  goto TIME_OUT_CASE;
				}

				checksum = getbinary();	// haberleþmeden gelen blok sonundaki checksum deðeri
//Checksum calculation
				for(Size = 0; Size < PM_ROW_SIZE*3; Size++)
				{//1536 adet byte üst üste toplanýr.
					CalcCheckSum += Buffer[Size];
				}
				CalcCheckSum = 0x00 - CalcCheckSum;	//2' nin complementi (hesapladýðýmýz checksum deðeri)
//Checksum calculation
				if(CalcCheckSum != checksum)
				{//checksum yanlýþ ise 
					PutChar(COMMAND_NACK);	//checksum yanlýþ olduðu için bu satýr tekrar yazýlacak (sonra ac)
				}
				else
				{// checksum doðruysa
					Erase(SourceAddr.Word.HW,SourceAddr.Word.LW,PM_ROW_ERASE);			
					WritePM(Buffer, SourceAddr);		/*program page */
					PutChar(COMMAND_ACK);				/*Send Acknowledgement */
				}
				LookMinTimeOutFlag_Reg = 0;
 				break;
			}


			case COMMAND_RESET:				/* tested */
			{
				uReg32 SourceAddr;
				int    Size;
				uReg32 Temp;
				unsigned char Zero_Value = 0;

				POWERLED_L = 0;		//haberleþmede olduðunu gösterir
				write_byte_to_ext_eeprom((unsigned long)((prgSoftwareUpdate_Reg*2) +1), &Zero_Value/*0*/, 1, 1);	//yeni göze 0 yaz (iþlem bitti)				
				ResetDevice();
				break;
			}

TIME_OUT_CASE:
			default:
//				PutChar(COMMAND_NACK);
				LookMinTimeOutFlag_Reg = 0;
				LookRtu_TimeOutFlag_Reg = 0;
				break;
		}
	}
}

/******************************************************************************/
void GetChar(/*unsigned */char * ptrChar)
{
	Time_OutFlag_Reg = 0;
	Min_TimeOutFlag_Reg = 0;

	TimeOutKarsilastirilan_Reg = TimeOutSayilan_Reg;

	while(1)
	{
  		ClrWdt();	//clear watchdog timer

		/* check for receive errors */
		if((U1STAbits.FERR == 1) || (U2STAbits.FERR == 1)) 
		{
//			continue;
		}
			
		/* must clear the overrun error to keep uart receiving */
		if(U1STAbits.OERR == 1)
		{
			U1STAbits.OERR = 0;
//			continue;
		}
		if(U2STAbits.OERR == 1)
		{
			U2STAbits.OERR = 0;
//			continue;
		}

		/* get the data */
		if((U1STAbits.URXDA == 1) || (U2STAbits.URXDA == 1))
		{
			TimeOutSayilan_Reg = 0;
			if(U1STAbits.URXDA == 1)
			{
				Active_UART_Reg = 0;	// UART1 kullanýlýyor (default)
				* ptrChar = U1RXREG;
			}
			else
			{
				Active_UART_Reg = 1;	// UART2 kullanýlýyor 
				* ptrChar = U2RXREG;
			}
			break;
		}
		else
		{
			++TimeOutSayilan_Reg;

			if((TimeOutSayilan_Reg - TimeOutKarsilastirilan_Reg) >= 437500)
			{//time out oluþtu (yaklaþýk 250 msn saniye)
				if(LookRtu_TimeOutFlag_Reg)
				{
					PutChar(COMMAND_NACK);
					* ptrChar = TIME_OUT_VALUE;	//dummy
					Min_TimeOutFlag_Reg = 1;
					break;
				}
			}

			if((TimeOutSayilan_Reg - TimeOutKarsilastirilan_Reg) >= 6125000/*3500000*/)
			{//time out oluþtu (yaklaþýk 3,5 saniye)
				if(LookMinTimeOutFlag_Reg)
				{
					PutChar(COMMAND_NACK);
					* ptrChar = TIME_OUT_VALUE;	//dummy
					Min_TimeOutFlag_Reg = 1;
					break;
				}
			}
			if(TimeOutSayilan_Reg >= 138000000)
			{//time out oluþtu (yaklaþýk 80 saniye)
//				PutChar('?');	// sonra sil
				TimeOutSayilan_Reg = 0;
				* ptrChar = TIME_OUT_VALUE;	//dummy
				Time_OutFlag_Reg = 1;
				break;
			}
		}
	}
}

/*-----------------------------------------------------------------------------*/ 
/*-----------------------------------------------------------------------------*/     
/*unsigned */char getdigit(void)
{
	/*unsigned */char receive_digit;
	
	GetChar(&receive_digit);
	if(receive_digit >= 'a' && receive_digit <= 'z')
	{
		receive_digit = (receive_digit - 'a') + 'A';
	}
	receive_digit -= 48;
	if(receive_digit > 9)
		receive_digit -= 7;
	return receive_digit;
}

/*-----------------------------------------------------------------------------*/ 
/*-----------------------------------------------------------------------------*/     
/*unsigned */char getbinary(void)
{
	/*unsigned */char receive_binary;
	
	receive_binary = getdigit();
	receive_binary <<= 4;
	b0byte0 = receive_binary;
	if(Time_OutFlag_Reg == 0 && Min_TimeOutFlag_Reg == 0)  receive_binary = getdigit();
	receive_binary = b0byte0 | receive_binary;
	return receive_binary;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
/*unsigned */char Bin_To_Ascii(/*unsigned */char Ascii)
{
	if(/*(Ascii >= 0x00) &&*/ (Ascii <= 0x09)) return Ascii + 0x30;
	else if((Ascii >= 0x0A) && (Ascii <= 0x0F)) return Ascii + 0x37;
	else return Ascii;
}

/******************************************************************************/
void ReadPM(/*unsigned */char * ptrData, uReg32 SourceAddr)
{
	int    Size;
	uReg32 Temp;

	for(Size = 0; Size < PM_ROW_SIZE; Size++)
	{
		Temp.Val32 = ReadLatch(SourceAddr.Word.HW, SourceAddr.Word.LW);

		ptrData[0] = Temp.Val[2];;
		ptrData[1] = Temp.Val[1];;
		ptrData[2] = Temp.Val[0];;

		ptrData = ptrData + 3;

		SourceAddr.Val32 = SourceAddr.Val32 + 2;
	}
}
/******************************************************************************/

void WriteBuffer(/*unsigned */char * ptrData, int Size)
{
	int DataCount;
	
	for(DataCount = 0; DataCount < Size; DataCount++)
	{
//		PutChar(ptrData[DataCount]);
		PutChar(Bin_To_Ascii(((ptrData[DataCount] & 0xF0) >> 4)));	//high nibble
		PutChar(Bin_To_Ascii((ptrData[DataCount] & 0x0F)));	//low nibble
	}
}
/******************************************************************************/
void PutChar(/*unsigned */char Uchar)
{
	if(Active_UART_Reg == 0)
	{
		while(!U1STAbits.TRMT)  ClrWdt();	//clear watchdog timer (Transmit Shift Register boþ olana kadar beklenir)
		U1TXREG = Uchar;
	}
	else
	{
  		while(!U2STAbits.TRMT)  ClrWdt();	//clear watchdog timer (Transmit Shift Register boþ olana kadar beklenir)
		U2TXREG = Uchar;
	}
	TimeOutSayilan_Reg = 0;	//
}
/******************************************************************************/
void WritePM(/*unsigned */char * ptrData, uReg32 SourceAddr)
{
	int    Size,Size1;
	uReg32 Temp;
	uReg32 TempAddr;
	uReg32 TempData;

	for(Size = 0,Size1=0; Size < PM_ROW_SIZE; Size++)
	{
		Temp.Val[0]=ptrData[Size1+0];	//lsb low
		Temp.Val[1]=ptrData[Size1+1];	//lsb high
		Temp.Val[2]=ptrData[Size1+2];	//msb low
		Temp.Val[3]=0;	//msb high
		Size1+=3;

		WriteLatch(SourceAddr.Word.HW, SourceAddr.Word.LW,Temp.Word.HW,Temp.Word.LW);

		// Device ID errata workaround: Save data at any address that has LSB 0x18
		if((SourceAddr.Val32 & 0x0000001F) == 0x18)
		{
			TempAddr.Val32 = SourceAddr.Val32;
			TempData.Val32 = Temp.Val32;
		}

		if((Size !=0) && (((Size + 1) % 64) == 0))
		{
			// Device ID errata workaround: Reload data at address with LSB of 0x18
			WriteLatch(TempAddr.Word.HW, TempAddr.Word.LW,TempData.Word.HW,TempData.Word.LW);
			WriteMem(PM_ROW_WRITE);
		}

		SourceAddr.Val32 = SourceAddr.Val32 + 2;
	}
}

/******************************************************************************/
/*---------------------------------------------------------------------------*/
/*start-----EEPROM RELATED ROUTINES -----------------------------------------*/
/*---------------------------------------------------------------------------*/
//unsigned int read_word_from_external_eeprom(unsigned int eeprom_word_address)
//{
//	union Common_2Byte_Variables read_eeprom;
//
//	read_eeprom.BYTE[1] = read_byte_from_external_eeprom(eeprom_word_address);
//	read_eeprom.BYTE[0] = read_byte_from_external_eeprom(eeprom_word_address + 1);
//	return(read_eeprom.INT);	
//}
//*******************************************************************************************	
//unsigned char read_byte_from_external_eeprom(unsigned int eeprom_address)
//{
//	unsigned char read_byte;
//
//	start_communication_with_eeprom();								// start
//	transmit_byte_to_eeprom(EEPROM1_W_address0);					// send device,chip and write code
//	transmit_byte_to_eeprom((unsigned char)(eeprom_address >> 8));	// send address high
//	transmit_byte_to_eeprom((unsigned char)(eeprom_address));		// send address low
//
//	start_communication_with_eeprom();								// start
//	transmit_byte_to_eeprom(EEPROM1_R_address0);					// send device,chip and read code
//
//	read_byte = receive_byte_from_eeprom();							// read a byte
//	stop_communication_with_eeprom();								// stop
//	
//	return (read_byte);
//}
//
//*******************************************************************************************
void sequential_read_from_ext_eeprom(unsigned long address, unsigned char* register_ptr, unsigned int number_of_byte)
{
	unsigned char a2_a1_a0;

	do
	{	// gelen adres'ten eeprom'un donaným adresini (A2<3>A1<2>A0<1>) bul
		a2_a1_a0 = (unsigned char)(address >> 14) & 0x0E;
		start_communication_with_eeprom();
		// send device, chip and write code
		transmit_byte_to_eeprom(EEPROM_W_address | a2_a1_a0);
		// send address high and low
		transmit_byte_to_eeprom((unsigned char)(address >> 8) & 0x7F);
		transmit_byte_to_eeprom((unsigned char)(address));
		start_communication_with_eeprom();
		// send device, chip and read code
		transmit_byte_to_eeprom(EEPROM_R_address | a2_a1_a0);
		do
		{	// read a byte and next
			*register_ptr++ = receive_byte_from_eeprom(0);
			// a2a1a0 kontrolü için adresi arttýr
			address++;
		}while(--number_of_byte && (((unsigned int)address & 0x7FFF) != 0));
		// acknowledge için DUMMY read
		receive_byte_from_eeprom(1);
		stop_communication_with_eeprom();
	}while(number_of_byte);
}

//*******************************************************************************************
//void write_word_to_external_eeprom(unsigned int eeprom_word_address, unsigned int write_word)
//{
//	write_byte_to_external_eeprom(eeprom_word_address,(unsigned char)(write_word >> 8));
//	write_byte_to_external_eeprom(eeprom_word_address + 1,(unsigned char)write_word);
//}
//*******************************************************************************************
//void write_byte_to_external_eeprom(unsigned int eeprom_address, unsigned char write_byte)
//{
//	if(write_byte != read_byte_from_external_eeprom(eeprom_address))
//	{	start_communication_with_eeprom();								// start
//		transmit_byte_to_eeprom(EEPROM1_W_address0);
//		transmit_byte_to_eeprom((unsigned char)(eeprom_address >> 8));	// send address high
//		transmit_byte_to_eeprom((unsigned char)(eeprom_address));		// send address low
//		transmit_byte_to_eeprom(write_byte);							// write a byte
//		stop_communication_with_eeprom();								// stop
//	
////		eeprom_write_delay = 0;
////		while(eeprom_write_delay <= 11);								// delay 10ms
//		for (VppDelayCtr = 0; VppDelayCtr < 400000; VppDelayCtr++)
//		{//yaklaþýk 10msn. bekler
//			Nop();
//		}
//	}
//}
/************************************************************************************
FunctionName  :	write_byte_to_ext_eeprom
Descriptions  :	yazma korumalý fakat yavaþ
Parameters    :	address = 
				register_ptr = 
				number_of_byte = 
				enable_next_byte = 
ReturnedValue :	Yok
************************************************************************************/
void write_byte_to_ext_eeprom(unsigned long address, unsigned char* register_ptr, unsigned int number_of_byte, unsigned char enable_next_byte)
{
	unsigned char read_byte, a2_a1_a0;

//	RAM_x.EEPROMWriteCnt++;

	do
	{
		sequential_read_from_ext_eeprom(address, &read_byte, 1);
		if(*register_ptr != read_byte)
		{	// gelen adres'ten eeprom'un donaným adresini (A2<3>A1<2>A0<1>) bul
			a2_a1_a0 = (unsigned char)(address >> 14) & 0x0E;
			start_communication_with_eeprom();
			// send device,chip and write code
			transmit_byte_to_eeprom(EEPROM_W_address | a2_a1_a0);
			// send address high and low
			transmit_byte_to_eeprom((unsigned char)(address >> 8) & 0x7F);
			transmit_byte_to_eeprom((unsigned char)(address));
			// write a byte
			transmit_byte_to_eeprom(*register_ptr);
			stop_communication_with_eeprom();
			// delay 5ms
//			eeprom_write_delay = 0;
//			while(eeprom_write_delay <= 6);
			for (VppDelayCtr = 0; VppDelayCtr < 400000; VppDelayCtr++)
			{//yaklaþýk 10msn. bekler
				Nop();
			}
		}
		// next byte if enable
		if(enable_next_byte)
			register_ptr++;
		// next address
		address++;
	}while(--number_of_byte);
}

//*******************************************************************************************
unsigned char receive_byte_from_eeprom(unsigned char acknowledge)
{
	unsigned char count=8;
	unsigned char receive_byte=0;

	SDA_T = 1;		// SDA port select as input 

	do
	{
		SCL_L = 1;		//EEPROM_scl = 1;
		Wait_nop(eeprom_nop_adet);
		receive_byte <<= 1;
		if(SDA_P)		//EEPROM_sda)
		{
			receive_byte |= 0x01;
		}
		SCL_L = 0;		//EEPROM_scl = 0;
		Wait_nop(eeprom_nop_adet);
	}
	while(--count);

	// Acknowledge isteðine göre SDA 1 veya 0 yapýlýr
	SDA_T = 0;
	if(acknowledge)  SDA_L = 1;
	else  SDA_L = 0;
	acknowledge_send_to_eeprom();
	
	return (receive_byte);
}
//*******************************************************************************************
void transmit_byte_to_eeprom(unsigned char transmit_byte)
{
	unsigned char count=8;
	do
	{
		if(transmit_byte & 0x80)
			SDA_L = 1;	//EEPROM_sda = 1;
		else
			SDA_L = 0;	//EEPROM_sda = 0;
		SCL_L = 1;		//EEPROM_scl = 1;
		transmit_byte <<= 1;
		Wait_nop(eeprom_nop_adet);
		SCL_L = 0;		//EEPROM_scl = 0;
		Wait_nop(eeprom_nop_adet);
	}
	while(--count);
	
	SDA_T = 1;			// SDA port select as input 

	acknowledge_send_to_eeprom();
	
	SDA_T = 0;			// SDA port select as outut 
}
//*******************************************************************************************
// scl must be high while sda goes from high to low transition in order to generate start bit
void start_communication_with_eeprom()
{
	SDA_L = 1; 			//EEPROM_sda = 1;
	Wait_nop(eeprom_nop_adet);
	SCL_L = 1;			//EEPROM_scl = 1;
	Wait_nop(eeprom_nop_adet);
	SDA_L = 0;			//EEPROM_sda = 0;
	Wait_nop(eeprom_nop_adet);
	SCL_L = 0;			//EEPROM_scl = 0;
	Wait_nop(eeprom_nop_adet);
}
//*******************************************************************************************
// sda must go from low to high during scl high state in order to generate stop bit and check bus conditions
void stop_communication_with_eeprom()
{
	SDA_L = 0;			//EEPROM_sda = 0;
	Wait_nop(eeprom_nop_adet);
	SCL_L = 1;			//EEPROM_scl = 1;
	Wait_nop(eeprom_nop_adet);
	SDA_L = 1;			//EEPROM_sda = 1;
	Wait_nop(eeprom_nop_adet);
	SCL_L = 0;			//EEPROM_scl = 0;
	Wait_nop(eeprom_nop_adet);
}

//******************************************************************************************
void acknowledge_send_to_eeprom()
{
	SCL_L = 1;
	Wait_nop(eeprom_nop_adet);

	SCL_L = 0;				//EEPROM_scl = 0;
	Wait_nop(eeprom_nop_adet);
}

//*******************************************************************************************
void Wait_nop(unsigned char adet)
{		
// 24LC256 eeprom entegresi max 400khz(2.5msn) clock frekansýnda haberleþiyor "bu yüzden nop kullanýlýrsa en az 50 adet  gerekiyor."
// MK41T11 RTC entegresi max 100kHz (10msn) clock frekansýnda haberleþiyor.  "bu yüzden nop kullanýlýrsa en az 200 adet  gerekiyor."
// while için herbir döngü 5 komut süresi 5*25ns = 125ns oluyor.
	unsigned int i = 0;
	while(++i<=adet);

//	Nop();				//50*25nsn=1.250usn*2=2.5usn
}


