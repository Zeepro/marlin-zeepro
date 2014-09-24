#ifndef H_RFID_ZIM
#define H_RFID_ZIM

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


//Définition des varaibles globales : 

#ifdef RFID_ZIM
#include "Marlin.h"
/////////////////////////////////////Fonction pour le RFID/////////////////////////////////////////////////////


//////////////////////////////// Numero des commandes////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
 
 
*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	
/////////////////////////////////// Liste des commandes //////////////////////////////////////////////
	
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
byte Mifare_Select[]              ={0x09,0x00,0x00,0x00,0x03,0x02};
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
byte Write_key[]                  ={};
byte Begin[]					  ={0xAA,0xBB};
byte serial_number[4]			  = {0x00,0x00,0x00,0x00};  

//////////////////////////////////////////////////////////////////////////////////////////////////////




//////////////////////////////// les tailles des commandes///////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////////////////////////




//////////////////////////////Reponse unique////////////////////////////////////////////////////////




	
	


void envoie_commande(byte commande[], byte taille, byte numero )
{
	 byte Response_mifaire[50]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };

        int a=0;
        byte commande_intermediaire[50];
        byte taille_bis=taille;
        byte taille_recoie; 
        byte checksum_recevoir=0x00;  
		byte checksomme_envoie=0x00;
		
		
		for (int i=2; i<taille;i++)
		{
			 checksomme_envoie=commande[i]^checksomme_envoie;
		}
		  
        
        for (int i=0; i<taille_bis;i++)
          {
            commande_intermediaire[i]=commande[i];
          }
        
        
        
        while(a<50)
        {
           if ((commande_intermediaire[a-1]==170))
            {
                    
            	for (int i=50; i>a;i--)
            	commande_intermediaire[i]=commande_intermediaire[i-1];
            	commande_intermediaire[a]=0;
                taille_bis++;
            }	
          
          
          a++;
        }
		/*
		MYSERIAL.print("\n COMMANDE A ENVOYER\n");
		        for (int i=0; i<50;i++)
          {
            MYSERIAL.print(commande_intermediaire[i], HEX);
			MYSERIAL.print(" ");
          }
		
		MYSERIAL.print("\n FIN COMMANDE A ENVOYER\n");*/
		
         
         if(numero==2)
		 {
				//debut de la communication
				for (int i =0; i< 2; i++)
				{
					MYSERIAL2.write(Begin[i]);
				}
        
          
				//Action      
				
				//MYSERIAL.print("\n DEBUT DE COMMANDE A ENVOYER\n");
				
				for (int i =0; i< taille_bis; i++)
				{
					MYSERIAL2.write(commande_intermediaire[i]);
					//MYSERIAL.print(commande_intermediaire[i], HEX);
				}
				
				//MYSERIAL.print("\n FIN COMMANDE A ENVOYER\n");
				
				//Si ma fonction est de lire ou d'écrir sur un Tag, j'ai d'abords besoin de son Serial_Number 
				if((commande_intermediaire[4]==3)&&(commande_intermediaire[5]==2)&&(serial_number[0]!=0)&&(serial_number[1]!=0)&&(serial_number[2]!=0)&&(serial_number[3]!=0))
				{
					for (int i =0; i< 4; i++)
					{
						MYSERIAL2.write(serial_number[i]);        
						checksomme_envoie=(serial_number[i])^checksomme_envoie;
					}
				}
        		//fin de la communication
				MYSERIAL2.write(checksomme_envoie);  	
				delay(50);
    
				// recevoir
    
				if(MYSERIAL2.available()>0)
				{
					for (int i =0; i< 50; i++)
					{
						Response_mifaire[i] = MYSERIAL2.read(); 
						delay(15); 
					}
				}					
			}	
			
		if(numero==3)
		 {
				//debut de la communication
				for (int i =0; i< 2; i++)
				{
					MYSERIAL3.write(Begin[i]);
				}
        
          
				//Action      
          
				for (int i =0; i< taille_bis; i++)
				{
					MYSERIAL3.write(commande_intermediaire[i]);
				}
        			
				//Si ma fonction est de lire ou d'écrir sur un Tag, j'ai d'abords besoin de son Serial_Number 
				if((commande_intermediaire[4]==3)&&(commande_intermediaire[5]==2)&&(serial_number[0]!=0)&&(serial_number[1]!=0)&&(serial_number[2]!=0)&&(serial_number[3]!=0))
				{
					for (int i =0; i< 4; i++)
					{
						MYSERIAL3.write(serial_number[i]);        
						checksomme_envoie=(serial_number[i])^checksomme_envoie;
					}
				}
        		//fin de la communication
				MYSERIAL3.write(checksomme_envoie);  	
				delay(50);
    
				// recevoir
				if(MYSERIAL3.available()>0)
				{
					for (int i =0; i< 50; i++)
					{
						Response_mifaire[i] = MYSERIAL3.read(); 
						delay(15); 
					}
				}					
			}
			
			
			
        //filtrage
    for (int i=0; i<50;i++)
    {
        commande_intermediaire[i]=Response_mifaire[i];
    }
    
    a=0;
     while(a<50)
    {
       if ((commande_intermediaire[a-1]==170)&&(commande_intermediaire[a]==0))
        {
        	for (int i=a; i<50;i++)
        	commande_intermediaire[i]=commande_intermediaire[i+1];
        	
        }	
      
      
      a++;
    }
    
  
    	
    
    
    
    
    // traitement
    if ((commande_intermediaire[0]==170) && (commande_intermediaire[1]==187) && (commande_intermediaire[8]==0))
    {
      taille_recoie=commande_intermediaire[2];
	  
      
  
        for (int i=4; i<taille_recoie+3;i++)
        {
          checksum_recevoir=commande_intermediaire[i]^checksum_recevoir;
        }
        
        if (checksum_recevoir!=commande_intermediaire[taille_recoie+3])
        {
          MYSERIAL.print(" \n Il y une erreur de checksum \n");
        }
        else
        {
          
         switch (commande_intermediaire[7])
         
         {
           case 1:
           /*
                    switch (commande_intermediaire[6])
                    {
                                            
                                             
                            case 1:
                            
                                      MYSERIAL.println(" \n Initialization \n");
                            
                            break;
                            
                            
                            case 4:
                            
                                       MYSERIAL.println(" \n Reading Device Mode \n");
                            
                            break;
                           
                           
                            case 6:
                            
                                       MYSERIAL.println(" \n Beeping \n");
                            
                            break;
                           
                           
                            case 7: 
                                        if (commande[6]==2)
                                       MYSERIAL.println(" \n LED OFF \n");
                                       else
                                       MYSERIAL.println(" \n LED ON \n");
                            
                            break;
                           
                           
                            case 12:
                            
                                        if (commande[6]==0)
                                       MYSERIAL.println(" \n Antenna OFF \n");
                                       else
                                       MYSERIAL.println(" \n Antenna ON \n");
                            
                            break;
							
                            
                          }
           */
           break;
           
           case 2:
           
                    switch (commande_intermediaire[6])
                    
                    {
                             /*                                           
                              case 1:
                              
                                         MYSERIAL.println(" \n Requesting \n");
                                         if ((commande_intermediaire[9]==68)&&(commande_intermediaire[10]==0))
                                         MYSERIAL.println(" \n Type : UltraLight \n");
                                         if ((commande_intermediaire[9]==4)&&(commande_intermediaire[10]==0))
                                         MYSERIAL.println(" \n Type : Mifare_One_S50 \n");
                                         if ((commande_intermediaire[9]==2)&&(commande_intermediaire[10]==0))
                                         MYSERIAL.println(" \n Type : Mifare_One_S70 \n");
                                         if ((commande_intermediaire[9]==68)&&(commande_intermediaire[10]==3))
                                         MYSERIAL.println(" \n Type : Mifare_DESFire \n");
                                         if ((commande_intermediaire[9]==8)&&(commande_intermediaire[10]==0))
                                         MYSERIAL.println(" \n Type : Mifare_Pro \n");
                                         if ((commande_intermediaire[9]==4)&&(commande_intermediaire[10]==3))
                                         MYSERIAL.println(" \n Type : Mifare_ProX \n");
                                         
                                         
                              
                              break;
                              */
                              
                              case 2:
                              
                                        //MYSERIAL.println(" \n Anticollision \n");
                                        
                                         //MYSERIAL.print(" \n Serial number is :  ");
                                         for (int i =9; i< 13; i++){     
                                         //MYSERIAL.print(commande_intermediaire[i], HEX);
                                         serial_number[i-9]=commande_intermediaire[i];
                                        // MYSERIAL.print(" ");
                                         }
                              
                              break;
                              
                              /*
                              case 3:
                              
                                         MYSERIAL.println(" \n Selecting card \n");
                              
                              break;
                             
                             
                              case 4:
                              
                                         MYSERIAL.println(" \n Hlta \n");
                              
                              break;
                             
                             
                              case 6: 
                              
                                         MYSERIAL.println(" \n Authentication1 \n");
                              
                              break;
                             
                             
                              case 7:
                              
                                         MYSERIAL.println(" \n Authentication2 \n");
                              
                              break;
                             */
                             
                              case 8:
                              
                                        // MYSERIAL.println(" \n Reading the RFID \n");
                                        // MYSERIAL.print(" \n Value : ");
                                         /*for (int i =9; i< 13; i++){     
                                          MYSERIAL.print(commande_intermediaire[i], HEX);
                                          MYSERIAL.print(" ");
                                          }*/
										 int bottom_nibble,top_nibble;
										      for (int i =9; i< 25; i++){        
												bottom_nibble = Response_mifaire[i] & 0xf;
												top_nibble = ( Response_mifaire[i] >> 4 ) & 0xf;
												MYSERIAL.print(top_nibble,HEX);
												MYSERIAL.print(bottom_nibble,HEX);
											  }		
											  
											   if(numero==2)
											 {
												 for (int i=9; i<25;i++)
													{
												TAG_RFID_E0[i-9]=Response_mifaire[i];
													}
	
											 }
	 
											 if(numero==3)
											 {
												 for (int i=9; i<25;i++)
													{
												TAG_RFID_E1[i-9]=Response_mifaire[i];
													 }
											 }											
                              
                              break;
                             
                             /*
                              case 9:
                              
                                         MYSERIAL.println(" \n Writing to the RFID \n");
                              
                              break;
                             
                             
                              case 10:
                              
                                         MYSERIAL.println(" \n Writing the Key \n");
                              
                              break;
							  */
							  default:
							  
							  break;
                    }
                    
           
           break;
		   
		    default:
							  
			break;
           
        }
                    
        }
    }   
      
      else
      {
       // MYSERIAL.println(" \n ERROR \n");
      }
      
          
    //check for response from module
/*
      MYSERIAL.print("\n le RFID vient de vous retourner ceci \n");
      for (int i =0; i< 30; i++)
	  {     
      MYSERIAL.print(Response_mifaire[i], HEX);
      MYSERIAL.print(" ");
		}
    MYSERIAL.print(" \n FIN \n ");*/
	
      
    if (numero= 2) 
    MYSERIAL2.flush(); 
	
	if (numero=3) 
	MYSERIAL3.flush(); 
    delay(8);
  
  
  
}

	


void RFID_init(void)
{
	
  int Response_mifaire2[30]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };  
  byte checksum_recevoir2= 0x03;
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
	
     MYSERIAL3.write(checksum_recevoir2);
    delay(30);
	
      for (int i =0; i< 30; i++){
      Response_mifaire2[i] = MYSERIAL3.read(); 
     delay(8); 
    }
	
	
	// traitement
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
	
     MYSERIAL2.write(checksum_recevoir2);
    delay(30);
	
      for (int i =0; i< 30; i++){
      Response_mifaire2[i] = MYSERIAL2.read(); 
     delay(8); 
    }
	
	
	
	
	
	// traitement
    if ((Response_mifaire2[0]==170) && (Response_mifaire2[1]==187) && (Response_mifaire2[8]==0))
    
      yes1++;
	  
	  if (yes1==1)
	  {
	  SERIAL_PROTOCOLLN(MSG_RFID1);
		
		RFID2_14_15_OK=true;		
		
		
		
		
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






#endif /* guard */