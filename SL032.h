#ifndef H_SL032
#define H_SL032

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


//Définition des varaibles globales : 

#ifdef RFID_SL032
#include "Marlin.h"
/////////////////////////////////////Fonction pour le RFID/////////////////////////////////////////////////////

//////////////////////////////// Numero des commandes////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 SL032CMD_SelectCard        = 0       
 SL032CMD_LoginSector0      = 1 
 SL032CMD_LoginSector1      = 2
 SL032CMD_ReadBlock1        = 3                                   
 SL032CMD_WriteBlock1       = 4                                
 SL032CMD_ReadValue         = 5 
 SL032CMD_WriteValue        = 6                                
 SL032CMD_IncrementValue    = 7                               
 SL032CMD_DecrementValue    = 8                                         
 SL032CMD_CopyValue         = 9
 SL032CMD_ReadULPage5       = A                                         
 SL032CMD_WriteULPage5      = B 
 SL032CMD_RATSDesFire       = C
 SL032CMD_DesFireGetVersion = D
 SL032CMD_GetAdditionalData = E
 SL032CMD_LightOn           = F
 SL032CMD_LightOff          = G 
 SL032CMD_Halt              = H 

*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////// les commandes////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte SL032CMD_SelectCard[]        ={0xBA,0x02,0x01 };       
byte SL032CMD_LoginSector0[]      ={0xBA,0x0A,0x02,0x00,0xAA,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};  
byte SL032CMD_LoginSector1[]      ={0xBA,0x0A,0x02,0x01,0xAA,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
byte SL032CMD_ReadBlock1[]        ={0xBA,0x03,0x03,0x01};                                     
//byte SL032CMD_WriteBlock1[]       ={0xBA,0x13,0x04,0x01,0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};     
byte SL032CMD_WriteBlock1[]       ={0xBA,0x13,0x04,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};  
byte SL032CMD_ReadValue[]         ={0xBA,0x03,0x05,0x04};  
byte SL032CMD_WriteValue[]        ={0xBA,0x07,0x06,0x04,0x00,0x00,0x00,0x01};                                
byte SL032CMD_IncrementValue[]    ={0xBA,0x07,0x08,0x04,0x00,0x00,0x00,0x20};                                
byte SL032CMD_DecrementValue[]    ={0xBA,0x07,0x09,0x04,0x03,0x00,0x00,0x00};                                          
byte SL032CMD_CopyValue[]         ={0xBA,0x04,0x0A,0x04,0x05};
byte SL032CMD_ReadULPage5[]       ={0xBA,0x03,0x10,0x05};                                          
byte SL032CMD_WriteULPage5[]      ={0xBA,0x07,0x11,0x05,0x11,0x22,0x33,0x44}; 
byte SL032CMD_RATSDesFire[]       ={0xBA,0x02,0x20};
byte SL032CMD_DesFireGetVersion[] ={0xBA,0x03,0x21,0x60}; 
byte SL032CMD_GetAdditionalData[] ={0xBA,0x03,0x21,0xAF}; 
byte SL032CMD_LightOn[]           ={0xBA,0x03,0x40,0x01}; 
byte SL032CMD_LightOff[]           ={0xBA,0x03,0x40,0x00}; 
byte SL032CMD_Halt[]              ={0xBA,0x03,0x50,0x00};  



//////////////////////////////// les tailles des commandes//////////////////////////////////////////////////////////////////////////////////////////////////////////
byte length_SL032CMD_SelectCard            = sizeof(SL032CMD_SelectCard);
byte length_SL032CMD_LoginSector0          = sizeof(SL032CMD_LoginSector0);
byte length_SL032CMD_LoginSector1          = sizeof(SL032CMD_LoginSector1);
byte length_SL032CMD_ReadBlock1            = sizeof(SL032CMD_ReadBlock1);
byte length_SL032CMD_WriteBlock1           = sizeof(SL032CMD_WriteBlock1);
byte length_SL032CMD_ReadValue             = sizeof(SL032CMD_ReadValue);
byte length_SL032CMD_WriteValue            = sizeof(SL032CMD_WriteValue);
byte length_SL032CMD_IncrementValue        = sizeof(SL032CMD_IncrementValue);
byte length_SL032CMD_DecrementValue        = sizeof(SL032CMD_DecrementValue);
byte length_SL032CMD_CopyValue             = sizeof(SL032CMD_CopyValue);
byte length_SL032CMD_ReadULPage5           = sizeof(SL032CMD_ReadULPage5);
byte length_SL032CMD_WriteULPage5          = sizeof(SL032CMD_WriteULPage5);
byte length_SL032CMD_RATSDesFire           = sizeof(SL032CMD_RATSDesFire);
byte length_SL032CMD_DesFireGetVersion     = sizeof(SL032CMD_DesFireGetVersion);
byte length_SL032CMD_GetAdditionalData     = sizeof(SL032CMD_GetAdditionalData);
byte length_SL032CMD_LightOn               = sizeof(SL032CMD_LightOn);
byte length_SL032CMD_LightOff              = sizeof(SL032CMD_LightOff);
byte length_SL032CMD_Halt                  = sizeof(SL032CMD_Halt);


byte calcul_cheksum(byte a[], byte length )
{

  
  byte checksum=0x00;
  
  for (int i=0; i<length;i++)
  {
    checksum=a[i]^checksum;
  }

   return checksum;
}
    
	
	
	
	void calcul_commande(byte commande[], byte taille, byte checksomme )
{

  byte taille_recoie;   
  byte checksum_recevoir=0x00; 
  int Response_mifaire[30]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
  
  //Action
  
   for (int i =0; i< taille; i++){
    MYSERIAL2.write(commande[i]);
    }
    MYSERIAL2.write(checksomme);
    
    delay(30);
	
      for (int i =0; i< 30; i++){
      Response_mifaire[i] = MYSERIAL2.read(); 
     delay(8); 
    }
    
 
    
    
        
    // traitement
    if ((Response_mifaire[0]==189) &&  ( (Response_mifaire[3]==0) || (Response_mifaire[3]==2) ))
    {
      taille_recoie=Response_mifaire[1];
      
  
        for (int i=0; i<taille_recoie+1;i++)
        {
          checksum_recevoir=Response_mifaire[i]^checksum_recevoir;
        }
        
        if (checksum_recevoir!=Response_mifaire[taille_recoie+1])
        {
          MYSERIAL.write(" \n Il y une erreur de checksum \n");
        }
        else
        {
          
         switch (Response_mifaire[2]){
              
                        
            case 1:
            
                       MYSERIAL.write(" \n Le type de la carte est : ");
                       
                       switch (Response_mifaire[8]) {
                         
                         case 1 : 
                                 
                                 MYSERIAL.write("  Mifare 1k, 4 byte UID  \n");
                                 
                         break;
                         
                         
                         case 2 : 
                                 
                                 MYSERIAL.write("  Mifare Pro  \n");
                                 
                         break; 
                         
                         
                         case 3 : 
                                 
                                 MYSERIAL.write("  Mifare UltraLight or NATG203, 7 byte UID  \n");
                                 
                         break;
                         
                         
                         case 4 : 
                                 
                                 MYSERIAL.write("  Mifare 4k, 4 byte UID  \n");
                                 
                         break;
                         
                         
                         case 5 : 
                                 
                                 MYSERIAL.write("  Mifare ProX  \n");
                                 
                         break; 
                         
                         
                         case 6 : 
                                 
                                 MYSERIAL.write("  Mifare DesFire  \n");
                                 
                         break;
                         
                         
                         case 7 : 
                                 
                                 MYSERIAL.write("  Mifare 1k, 7 byte UID   \n");
                                 
                         break; 
                         
                         
                         case 8 : 
                                 
                                 MYSERIAL.write("  Mifare 4k, 7 byte UID  \n");
                                 
                         break;
                         
                         
                         case 10 : 
                                 
                                 MYSERIAL.write("  AUTRE  \n");
                                 
                         break;   
                         
                       }
                       
                       MYSERIAL.write(" \n Serial number is :  ");
                       for (int i =4; i< 8; i++){     
                       MYSERIAL.print(Response_mifaire[i],16);
                       MYSERIAL.write(" ");
                       }
            
            break;
            
            
            case 2:
            
                      MYSERIAL.write(" \n Login to a sector0 \n");
                                  
            break;
           
           
            case 3 :
            
                       MYSERIAL.write(" \n Lecture du block : ");
                       MYSERIAL.print(commande[3],16);
                       MYSERIAL.write(" \n Sa valeur est de : ");
                       for (int i =4; i< 20; i++){     
                        MYSERIAL.print(Response_mifaire[i],16);
                        MYSERIAL.write(" ");
                        }
            
            break;
           
           
            case 4 : 
            
                       MYSERIAL.write(" \n Write a data block \n");
            
            break;
           
           
            case 64 :
            
                      if(commande[3]==0)
                      
                      MYSERIAL.write(" \n Eteindre LED \n");
                      
                      else
            
                       MYSERIAL.write(" \n Allumer LED \n");
            
            break;
           
            }
                    
                    
        }
    }   
      
     else
     {
       MYSERIAL.write(" \n Il y une erreur \n");
     }
     
    MYSERIAL2.flush();  
    delay(800);
}
	


void calcul_commande2(byte commande[], byte taille, byte checksomme )
{

  byte taille_recoie;   
  byte checksum_recevoir=0x00; 
  int Response_mifaire[30]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
  
  //Action
  
   for (int i =0; i< taille; i++){
    MYSERIAL3.write(commande[i]);
    }
    MYSERIAL3.write(checksomme);
    
    delay(30);
	
      for (int i =0; i< 30; i++){
      Response_mifaire[i] = MYSERIAL3.read(); 
     delay(8); 
    }
    
 
    
    
        
    // traitement
    if ((Response_mifaire[0]==189) &&  ( (Response_mifaire[3]==0) || (Response_mifaire[3]==2) ))
    {
      taille_recoie=Response_mifaire[1];
      
  
        for (int i=0; i<taille_recoie+1;i++)
        {
          checksum_recevoir=Response_mifaire[i]^checksum_recevoir;
        }
        
        if (checksum_recevoir!=Response_mifaire[taille_recoie+1])
        {
          MYSERIAL.write(" \n Il y une erreur de checksum \n");
        }
        else
        {
          
         switch (Response_mifaire[2]){
              
                        
            case 1:
            
                       MYSERIAL.write(" \n Le type de la carte est : ");
                       
                       switch (Response_mifaire[8]) {
                         
                         case 1 : 
                                 
                                 MYSERIAL.write("  Mifare 1k, 4 byte UID  \n");
                                 
                         break;
                         
                         
                         case 2 : 
                                 
                                 MYSERIAL.write("  Mifare Pro  \n");
                                 
                         break; 
                         
                         
                         case 3 : 
                                 
                                 MYSERIAL.write("  Mifare UltraLight or NATG203, 7 byte UID  \n");
                                 
                         break;
                         
                         
                         case 4 : 
                                 
                                 MYSERIAL.write("  Mifare 4k, 4 byte UID  \n");
                                 
                         break;
                         
                         
                         case 5 : 
                                 
                                 MYSERIAL.write("  Mifare ProX  \n");
                                 
                         break; 
                         
                         
                         case 6 : 
                                 
                                 MYSERIAL.write("  Mifare DesFire  \n");
                                 
                         break;
                         
                         
                         case 7 : 
                                 
                                 MYSERIAL.write("  Mifare 1k, 7 byte UID   \n");
                                 
                         break; 
                         
                         
                         case 8 : 
                                 
                                 MYSERIAL.write("  Mifare 4k, 7 byte UID  \n");
                                 
                         break;
                         
                         
                         case 10 : 
                                 
                                 MYSERIAL.write("  AUTRE  \n");
                                 
                         break;   
                         
                       }
                       
                       MYSERIAL.write(" \n Serial number is :  ");
                       for (int i =4; i< 8; i++){     
                       MYSERIAL.print(Response_mifaire[i],16);
                       MYSERIAL.write(" ");
                       }
            
            break;
            
            
            case 2:
            
                      MYSERIAL.write(" \n Login to a sector0 \n");
                                  
            break;
           
           
            case 3 :
            
                       MYSERIAL.write(" \n Lecture du block : ");
                       MYSERIAL.print(commande[3],16);
                       MYSERIAL.write(" \n Sa valeur est de : ");
                       for (int i =4; i< 20; i++){     
                        MYSERIAL.print(Response_mifaire[i],16);
                        MYSERIAL.write(" ");
                        }
            
            break;
           
           
            case 4 : 
            
                       MYSERIAL.write(" \n Write a data block \n");
            
            break;
           
           
            case 64 :
            
                      if(commande[3]==0)
                      
                      MYSERIAL.write(" \n Eteindre LED \n");
                      
                      else
            
                       MYSERIAL.write(" \n Allumer LED \n");
            
            break;
           
            }
                    
                    
        }
    }   
      
     else
     {
       MYSERIAL.write(" \n Il y une erreur \n");
     }
     
    MYSERIAL3.flush();  
    delay(800);
}


void RFID_init(void)
{
	
	byte taille_recoie2= length_SL032CMD_LightOn;   
  byte checksum_recevoir2= 0x01;
  int Response_mifaire2[30]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	  int yes1=0;
	  int yes2=0;
  
  //Action
  
   for (int i =0; i< taille_recoie2; i++){
    MYSERIAL3.write(SL032CMD_LightOn[i]);
    }
    MYSERIAL3.write(checksum_recevoir2);
    
    delay(30);
	
      for (int i =0; i< 30; i++){
      Response_mifaire2[i] = MYSERIAL3.read(); 
     delay(8); 
    }
	
	
	// traitement
    if (Response_mifaire2[0]==189)
    
      yes2++;
	  
	  for (int i =0; i< 30; i++){
      Response_mifaire2[i] = 0;  
    }
	  
	  
	  
	   for (int i =0; i< taille_recoie2; i++){
    MYSERIAL2.write(SL032CMD_LightOn[i]);
    }
    MYSERIAL2.write(checksum_recevoir2);
    
    delay(30);
	
      for (int i =0; i< 30; i++){
      Response_mifaire2[i] = MYSERIAL2.read(); 
     delay(8); 
    }
	
	
	
	
	
	// traitement
    if (Response_mifaire2[0]==189)
    
      yes1++;
	  
	  if (yes1==1)
	  {
	  SERIAL_PROTOCOLLN(MSG_RFID1);
		#ifndef RFID1_16_17_OK
		#define RFID1_16_17_OK
		#endif
	  }	  
	  else
	  {
	  SERIAL_PROTOCOLLN(MSG_ERR_RFID1);
	  }	  
	  
	  if (yes2==1)
	  {
	  SERIAL_PROTOCOLLN(MSG_RFID2);
		#ifndef RFID2_14_15_OK
		#define RFID2_14_15_OK
		#endif
	  }	  
	  else 
	  {
	  SERIAL_PROTOCOLLN(MSG_ERR_RFID2);
	  }	  
	  
	  
    
}





#endif






#endif /* guard */