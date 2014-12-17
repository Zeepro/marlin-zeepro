#ifndef H_RFID_ZIM
#define H_RFID_ZIM

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifdef RFID_ZIM
#include "Marlin.h"

/*

Initialize_Port        = 0 
Read_Device_Mode		= 1
Set_Buzzer_Beep        = 2                                   
Set_Led_Color_OFF      = 3   
Set_Led_Color_ON       = 4 
Set_Antenna_Status_OFF = 5 
Set_Antenna_Status_ON  = 6 
Mifare_Request         = 7                                
Mifare_Anticollision	= 8                               
Mifare_Select			= 9  
Mifare_Hlta            = A                                   
Mifare_Authentication1 = B   
Mifare_Authentication2 = C
Mifare_Read            = D
Mifare_Write           = E
Write_key              = F

*/

byte Initialize_Port[]             ={0x06,0x00,0x00,0x00,0x01,0x01,0x03};
byte Read_Device_Mode[]            ={0x05,0x00,0x00,0x00,0x04,0x01};
byte Set_Buzzer_Beep[]             ={0x06,0x00,0x00,0x00,0x06,0x01, 0xC8};
byte Set_Led_Color_OFF[]           ={0x06,0x00,0x00,0x00,0x07,0x01,0x02};
byte Set_Led_Color_ON[]            ={0x06,0x00,0x00,0x00,0x07,0x01,0x03};

//FC 20140716 New RFID reader
byte Set_Antenna_Status_OFF[]      ={0x06,0x00,0x00,0x00,0x0C,0x01,0x00};
byte Set_Antenna_Status_ON[]       ={0x06,0x00,0x00,0x00,0x0C,0x01,0x01};
//byte Set_Antenna_Status_OFF[]      ={0xAA,0xBB,0x07,0x00,0x00,0x00,0x0D,0x12,0x12,0x06};
//byte Set_Antenna_Status_ON[]      ={0xAA,0xBB,0x07,0x00,0x00,0x00,0x0D,0x12,0x12,0x3F};
//byte Set_Antenna_Status_OFF[]      ={0x07,0x00,0x00,0x00,0x0D,0x12,0x12,0x06};
//byte Set_Antenna_Status_ON[]      ={0x07,0x00,0x00,0x00,0x0D,0x12,0x12,0x3F};

byte Mifare_Request[]              ={0x06,0x00,0x00,0x00,0x01,0x02,0x52};
byte Mifare_Anticollision[]       ={0x05,0x00,0x00,0x00,0x02,0x02};
byte Mifare_Select[]              ={0x09,0x00,0x00,0x00,0x03,0x02,0x00,0x00,0x00,0x00};
byte Mifare_Hlta[]                ={0x05,0x00,0x00,0x00,0x04,0x02};                              
byte Mifare_Authentication1[]     ={};
byte Mifare_Authentication24[]     ={0x0D,0x00,0x00,0x00,0x07,0x02,0x60,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
byte Mifare_Authentication25[]     ={0x0D,0x00,0x00,0x00,0x07,0x02,0x60,0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
byte Mifare_Authentication26[]     ={0x0D,0x00,0x00,0x00,0x07,0x02,0x60,0x02,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
byte Mifare_Authentication27[]     ={0x0D,0x00,0x00,0x00,0x07,0x02,0x60,0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

byte Mifare_Read1[]                ={0x06,0x00,0x00,0x00,0x08,0x02,0x06};  
byte Mifare_Read2[]                ={0x06,0x00,0x00,0x00,0x08,0x02,0x07}; 
byte Mifare_Read3[]                ={0x06,0x00,0x00,0x00,0x08,0x02,0x08}; 
byte Mifare_Read4[]                ={0x06,0x00,0x00,0x00,0x08,0x02,0x09};                             
byte Mifare_Write1[]               ={0x16,0x00,0x00,0x00,0x09,0x02,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
byte Mifare_Write2[]               ={0x16,0x00,0x00,0x00,0x09,0x02,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
byte Mifare_Write3[]               ={0x16,0x00,0x00,0x00,0x09,0x02,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
byte Mifare_Write4[]               ={0x16,0x00,0x00,0x00,0x09,0x02,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
byte Mifare_UltraWrite[]           ={0x0a,0x00,0x00,0x00,0x13,0x02,0x06,0x00,0x00,0x00,0x00};
byte Write_key[]                  ={};
byte Begin[]					  ={0xAA,0xBB};
byte serial_number[4]			  = {0x00,0x00,0x00,0x00};  

byte length_Initialize_Port         = sizeof(Initialize_Port);
byte length_Read_Device_Mode	    = sizeof(Read_Device_Mode);
byte length_Set_Buzzer_Beep			= sizeof(Set_Buzzer_Beep);
byte length_Set_Led_Color_OFF       = sizeof(Set_Led_Color_OFF);
byte length_Set_Led_Color_ON        = sizeof(Set_Led_Color_ON);
byte length_Set_Antenna_Status_OFF  = sizeof(Set_Antenna_Status_OFF);
byte length_Set_Antenna_Status_ON   = sizeof(Set_Antenna_Status_ON);
byte length_Mifare_Request          = sizeof(Mifare_Request);
byte length_Mifare_Anticollision    = sizeof(Mifare_Anticollision);
byte length_Mifare_Select			= sizeof(Mifare_Select);
byte length_Mifare_Hlta				= sizeof(Mifare_Hlta);

byte length_Mifare_Authentication1  = sizeof(Mifare_Authentication1);
byte length_Mifare_Authentication24  = sizeof(Mifare_Authentication24);
byte length_Mifare_Authentication25  = sizeof(Mifare_Authentication25);
byte length_Mifare_Authentication26  = sizeof(Mifare_Authentication26);
byte length_Mifare_Authentication27  = sizeof(Mifare_Authentication27);
byte length_Mifare_Read1             = sizeof(Mifare_Read1);
byte length_Mifare_Read2             = sizeof(Mifare_Read2);
byte length_Mifare_Read3             = sizeof(Mifare_Read3);
byte length_Mifare_Read4             = sizeof(Mifare_Read4);
byte length_Mifare_Write1            = sizeof(Mifare_Write1);
byte length_Mifare_Write2            = sizeof(Mifare_Write2);
byte length_Mifare_Write3            = sizeof(Mifare_Write3);
byte length_Mifare_Write4            = sizeof(Mifare_Write4);
byte length_Write_key               = sizeof(Write_key);
byte length_Begin					= sizeof(Begin);
bool RFID1_16_17_OK= false;
bool RFID2_14_15_OK=false;

void envoie_commande(byte commande[], byte taille, byte numero )
{
	byte Response_mifaire[50]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };

	byte commande_intermediaire[50];
	byte taille_bis = 0;
	byte taille_reception = 0; 
	byte checksum_in = 0x00;  
	byte checksum_out = 0x00;

	// Add serial number when needed

	if(commande[4] == 3 && commande[5] == 2)
		for (int i = 0; i < 4; i++) {
			commande[i + 6] = serial_number[i];
			//MYSERIAL.print((commande[i + 6] >> 4 ) & 0xf, HEX);
			//MYSERIAL.print(commande[i + 6] & 0xf, HEX);
		}

		// Copy command (any 0xAA must be followed by 0x00 - without changing length)
		// (no length verification... shame)

		for (int i = 0; i < taille; i++) {
			commande_intermediaire[taille_bis++] = commande[i];
			if(i > 1) {
				checksum_out ^= commande[i];
				if (commande[i] == 0xAA) {
					commande_intermediaire[taille_bis++] = 0x00;
					checksum_out ^= 0x00;
				}
			}
		}

		if(numero == 2) {

			// Start

			for (int i =0; i< 2; i++) {
				MYSERIAL2.write(Begin[i]);
			}

			// Action      

			for (int i =0; i< taille_bis; i++) {
				MYSERIAL2.write(commande_intermediaire[i]);
			}

			// Checksum

			MYSERIAL2.write(checksum_out);

			// Guess it works like that...
			if(checksum_out == 0xAA)
				MYSERIAL2.write((byte) 0x00);

			// Watchdog
			unsigned long start = millis();

			// Wait for the first byte
			while(MYSERIAL2.available() == 0 && millis() - start < 90) {
				delay(5);
			}

			// Each byte should arrive in 10ms delay
			start = millis();
			while(millis() - start < 10 && taille_reception < 50) {
				if(MYSERIAL2.available()) {
					Response_mifaire[taille_reception++] = MYSERIAL2.read(); 
					start = millis();
				} else
					delay(1);
			}

			//delay(100);

			//while(MYSERIAL2.available()  && millis() - start < 10 && taille_reception < 50)
			//	Response_mifaire[taille_reception++] = MYSERIAL2.read(); 

		}	

		if(numero == 3) {

			// Start

			for (int i =0; i< 2; i++) {
				MYSERIAL3.write(Begin[i]);
			}

			// Action      

			for (int i =0; i< taille_bis; i++) {
				MYSERIAL3.write(commande_intermediaire[i]);
			}

			MYSERIAL3.write(checksum_out);

			// Guess it works like that...

			if(checksum_out == 0xAA)
				MYSERIAL3.write((byte) 0x00);

			// Watchdog
			unsigned long start = millis();

			// Wait for the first byte
			while(MYSERIAL3.available() == 0 && millis() - start < 90) {
				delay(5);
			}

			// Each byte should arrive in 10ms delay
			start = millis();
			while(millis() - start < 10 && taille_reception < 50) {
				if(MYSERIAL3.available()) {
					Response_mifaire[taille_reception++] = MYSERIAL3.read(); 
					start = millis();
				} else
					delay(1);
			}

			//delay(100);

			//while(MYSERIAL3.available() && taille_reception < 50)
			//	Response_mifaire[taille_reception++] = MYSERIAL3.read(); 
		}

		if(taille_reception > 0 ) {

			// Checksum test

			for (int i = 4; i < (Response_mifaire[taille_reception - 2] == 0xAA && Response_mifaire[taille_reception - 1] == 0x00 ? taille_reception - 2 : taille_reception - 1); i++)
				checksum_in ^= Response_mifaire[i];

			//MYSERIAL.print("#R#");
			//for (int i = 0; i < taille_reception; i++) {
			//	MYSERIAL.print((Response_mifaire[i] >> 4 ) & 0xf, HEX);
			//	MYSERIAL.print(Response_mifaire[i] & 0xf, HEX);
			//}

			//MYSERIAL.print("#");
			//MYSERIAL.print((Response_mifaire[Response_mifaire[taille_reception - 2] == 0xAA && Response_mifaire[taille_reception - 1] == 0x00 ? taille_reception - 2 : taille_reception - 1] >> 4 ) & 0xf, HEX);
			//MYSERIAL.print(Response_mifaire[Response_mifaire[taille_reception - 2] == 0xAA && Response_mifaire[taille_reception - 1] == 0x00 ? taille_reception - 2 : taille_reception - 1] & 0xf, HEX);
			//MYSERIAL.print("#");
			//MYSERIAL.print((checksum_in >> 4 ) & 0xf, HEX);
			//MYSERIAL.print(checksum_in & 0xf, HEX);
			//MYSERIAL.print("#");

			if(Response_mifaire[Response_mifaire[taille_reception - 2] == 0xAA && Response_mifaire[taille_reception - 1] == 0x00 ? taille_reception - 2 : taille_reception - 1] == checksum_in) {

				// 0&AA + 0&00 encoding removal

				int j = 0;
				for (int i = 0; i < taille_reception; i++)
				{
					commande_intermediaire[j++] = Response_mifaire[i];
					if((i >= 2) && (Response_mifaire[i] == 0xAA))
						i++;
				}

				// Manage

				if ((commande_intermediaire[0]==0xAA) && (commande_intermediaire[1]==0xBB) && (commande_intermediaire[8]==0)) {

					taille_reception=commande_intermediaire[2];

					switch (commande_intermediaire[7]) {
					case 2:
						switch (commande_intermediaire[6]) {

						case 2:	// Anticollision
							for (int i =9; i< 13; i++){     
								serial_number[i-9]=commande_intermediaire[i];
							}

							break;

						case 8:	// Read
							int bottom_nibble,top_nibble;
							for (int i = 9; i < 25; i++){        
								bottom_nibble = commande_intermediaire[i] & 0xf;
								top_nibble = (commande_intermediaire[i] >> 4 ) & 0xf;
								MYSERIAL.print(top_nibble,HEX);
								MYSERIAL.print(bottom_nibble,HEX);
							}		

							if(numero==2) 
								for (int i = 9; i < 25; i++)
									TAG_RFID_E0[i - 9] = commande_intermediaire[i];

							if(numero == 3)
								for (int i = 9; i < 25; i++)
									TAG_RFID_E1[i - 9] = commande_intermediaire[i];

							break;
						}
					}
				}
			}

		}

		if (numero == 2) 
			MYSERIAL2.flush(); 

		if (numero == 3) 
			MYSERIAL3.flush(); 
}

void RFID_init(void)
{

	int Response_mifaire2[30]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };  
	byte checksum_in2= 0x03;
	//  byte checksum_antenna_off= 0x0D;
	byte checksum_antenna_off= 0x0B;

	int yes1=0;
	int yes2=0;

	//Action
	for (int i =0; i< 2; i++){
		MYSERIAL3.write(Begin[i]);
	}
	for (int i =0; i< length_Initialize_Port; i++){
		MYSERIAL3.write(Initialize_Port[i]);
	}

	MYSERIAL3.write(checksum_in2);
	delay(30);

	for (int i =0; i< 30; i++){
		Response_mifaire2[i] = MYSERIAL3.read(); 
		delay(8); 
	}

	// Manage
	if ((Response_mifaire2[0]==170) && (Response_mifaire2[1]==187) && (Response_mifaire2[8]==0))

		yes2++;

	for (int i =0; i< 30; i++){
		Response_mifaire2[i] = 0;  
	}


	for (int i =0; i< 2; i++){
		MYSERIAL2.write(Begin[i]);
	}
	for (int i =0; i< length_Initialize_Port; i++){
		MYSERIAL2.write(Initialize_Port[i]);
	}

	MYSERIAL2.write(checksum_in2);
	delay(30);

	for (int i =0; i< 30; i++){
		Response_mifaire2[i] = MYSERIAL2.read(); 
		delay(8); 
	}

	// Manage
	if ((Response_mifaire2[0]==170) && (Response_mifaire2[1]==187) && (Response_mifaire2[8]==0))

		yes1++;

	if (yes1 == 1)
	{
		SERIAL_PROTOCOLLN(MSG_RFID1);

		RFID2_14_15_OK = true;		


#ifdef Auto_Desable_RFID


		for (int i =0; i< 2; i++){
			MYSERIAL2.write(Begin[i]);
		}


		// FC 20140716 To avoid calculate checksum
		for (int i =0; i< length_Set_Antenna_Status_OFF; i++){
			MYSERIAL2.write(Set_Antenna_Status_OFF[i]);
		}

		MYSERIAL2.write(checksum_antenna_off);
		//		envoie_commande(Set_Antenna_Status_OFF,length_Set_Antenna_Status_OFF,2); 	

		delay(30);

		for (int i =0; i< 30; i++){
			Response_mifaire2[i] = MYSERIAL2.read(); 
			delay(8); 
		}	

#endif

	}	  
	else
	{
		SERIAL_PROTOCOLLN(MSG_ERR_RFID1);
	}	  

	if (yes2==1)
	{
		SERIAL_PROTOCOLLN(MSG_RFID2);

		RFID1_16_17_OK=true;



#ifdef Auto_Desable_RFID


		for (int i =0; i< 2; i++){
			MYSERIAL3.write(Begin[i]);
		}

		// FC 20140716 To avoid calculate checksum
		for (int i =0; i< length_Set_Antenna_Status_OFF; i++){
			MYSERIAL3.write(Set_Antenna_Status_OFF[i]);
		}

		MYSERIAL3.write(checksum_antenna_off);
		//		envoie_commande(Set_Antenna_Status_OFF,length_Set_Antenna_Status_OFF,3);

		delay(30);

		for (int i =0; i< 30; i++){
			Response_mifaire2[i] = MYSERIAL3.read();
			delay(8);
		}

#endif
	}	  
	else 
	{
		SERIAL_PROTOCOLLN(MSG_ERR_RFID2);
	}	  

	MYSERIAL2.flush();
	MYSERIAL3.flush();


}
#endif
#endif