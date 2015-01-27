/* -*- c++ -*- */

/*
Reprap firmware based on Sprinter and grbl.
Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
This firmware is a mashup between Sprinter and grbl.
(https://github.com/kliment/Sprinter)
(https://github.com/simen/grbl/tree)

It has preliminary support for Matthew Roberts advance algorithm 
http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
*/

#include "Marlin.h"
#include "Arduino.h"

#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "pins_arduino.h"
#include "Marlin_RFID_HWSerial2.h"
//#include "SL032.h"
#include "RFID_ZIM.h"

#if DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif

#define VERSION_STRING  "1.1.0.8"

//Stepper Movement Variables

//===========================================================================
//=============================imported variables============================
//===========================================================================


//===========================================================================
//=============================public variables=============================
//===========================================================================
#ifdef SDSUPPORT
CardReader card;
#endif
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float Pause_current_position[NUM_STOP_PARAMETERS ] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
float add_homeing[3]={0,0,0};
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
// Extruder offset, only in XY plane
#if EXTRUDERS > 1
float extruder_offset[2][EXTRUDERS] = { 
#if defined(EXTRUDER_OFFSET_X) && defined(EXTRUDER_OFFSET_Y)
	EXTRUDER_OFFSET_X, EXTRUDER_OFFSET_Y 
#endif
}; 
#endif
float extruders_offset_changement[2]={0,0};
uint8_t active_extruder = 0;
bool unloading_critical_state = false;
bool unloading_command = false;
//bool Commande_cartridge = false;
int fanSpeed=0;

#ifdef FWRETRACT
bool autoretract_enabled=true;
bool retracted=false;
float retract_length=3, retract_feedrate=17*60, retract_zlift=0.8;
float retract_recover_length=0, retract_recover_feedrate=8*60;
#endif

//===========================================================================
//=============================private variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
static float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static bool fromsd[BUFSIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc
static char *name_fichier_supprimer;

const int sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;


// The two variable for the button
static unsigned long Read_button_1 = 0;
static unsigned long Read_button_2 = 0; 





static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME * 1000l;
static unsigned long extruder_inactive_time = DEFAULT_EXTRUDER_DEACTIVE_TIME * 1000l;

static unsigned long max_time_pushing = MAX_TIME_PUSHING*1000;

unsigned long starttime=0;
unsigned long stoptime=0;

static uint8_t tmp_extruder;


bool Stopped=false;



//////////////////////////////     Etat des LED          ///////////////////////////////////////////////

bool Strip_Led_State = false;
bool Top_Led_State = false;
bool Antenna_RFID_State = false;


//////////////////////////////   FIN    ////////////////////////////////////////////////////////////////


///////////////////////// fonction voyage//////////////////////////////////////////////

bool premier_transport = true;

//////////////////////// FIN fonction voyage //////////////////////////////////////////




////////////////////////////////////Fonction pour le RFID/////////////////////////////////////////////////////

// byte checksum_RFID;
// float RFID_data[32]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
byte TAG_RFID_E0[16]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
byte TAG_RFID_E1[16]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

float Distance_Filament_E0=0;

float Distance_Filament_E1=0; 


///////////////////////////Fonction pour les deux thermistance/////////////////

uint8_t numero_extruder=0;
float TMP0_Target = 0;
float TMP1_Target = 0;


////////////////////////Variable pour l'affichage des caract?res//////////////////////
bool MSG_SDCARD = false;
bool MSG_PRONTERFACE = false;




//////////////////////Numero de ligne////////////////////////////////////

float numero_ligne_sd_card=0;
float numero_ligne_pronterface=0;
float numero_fichier_sd=-1;





// Activation interruption
bool Desactiver_interruption = false;


//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

void get_arc_coordinates();
bool setTargetedHotend(int code);

void serial_echopair_P(const char *s_P, float v)
{ serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
{ serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
{ serialprintPGM(s_P); SERIAL_ECHO(v); }

extern "C"{
	extern unsigned int __bss_end;
	extern unsigned int __heap_start;
	extern void *__brkval;

	int freeMemory() {
		int free_memory;

		if((int)__brkval == 0)
			free_memory = ((int)&free_memory) - ((int)&__bss_end);
		else
			free_memory = ((int)&free_memory) - ((int)__brkval);

		return free_memory;
	}
}

//adds an command to the main command buffer
//thats really done in a non-safe way.
//needs overworking someday
void enquecommand(const char *cmd)
{
	if(buflen < BUFSIZE)
	{
		//this is dangerous if a mixing of serial and this happsens
		strcpy(&(cmdbuffer[bufindw][0]),cmd);
		SERIAL_ECHO_START;
		SERIAL_ECHOPGM("enqueing \"");
		SERIAL_ECHO(cmdbuffer[bufindw]);
		SERIAL_ECHOLNPGM("\"");
		bufindw= (bufindw + 1)%BUFSIZE;
		buflen += 1;
	}
}

void enquecommand_P(const char *cmd)
{
	if(buflen < BUFSIZE)
	{
		//this is dangerous if a mixing of serial and this happsens
		strcpy_P(&(cmdbuffer[bufindw][0]),cmd);
		SERIAL_ECHO_START;
		SERIAL_ECHOPGM("enqueing \"");
		SERIAL_ECHO(cmdbuffer[bufindw]);
		SERIAL_ECHOLNPGM("\"");
		bufindw= (bufindw + 1)%BUFSIZE;
		buflen += 1;
	}
}

void setup_killpin()
{
#if( KILL_PIN>-1 )
	pinMode(KILL_PIN,INPUT);
	WRITE(KILL_PIN,HIGH);
#endif
}

void setup_photpin()
{
#ifdef PHOTOGRAPH_PIN
#if (PHOTOGRAPH_PIN > -1)
	SET_OUTPUT(PHOTOGRAPH_PIN);
	WRITE(PHOTOGRAPH_PIN, LOW);
#endif
#endif 
}

void setup_powerhold()
{ 
#ifdef SUICIDE_PIN
#if (SUICIDE_PIN> -1)
	SET_OUTPUT(SUICIDE_PIN);
	WRITE(SUICIDE_PIN, HIGH);
#endif
#endif 
#if (PS_ON_PIN > -1)
	SET_OUTPUT(PS_ON_PIN);
	WRITE(PS_ON_PIN, PS_ON_AWAKE); 
#endif
}

void suicide()
{
#ifdef SUICIDE_PIN
#if (SUICIDE_PIN> -1) 
	SET_OUTPUT(SUICIDE_PIN);
	WRITE(SUICIDE_PIN, LOW);
#endif
#endif
}


void suicide_Zim_Setup()
{

#if (SUICIDE_Zim_PIN> -1) 
	SET_OUTPUT(SUICIDE_Zim_PIN);
	WRITE(SUICIDE_Zim_PIN, LOW);
#endif
}


void suicide_Zim()
{

#if (SUICIDE_Zim_PIN> -1) 
	WRITE(SUICIDE_Zim_PIN, HIGH);
#endif

}



void setup_green_led()
{

#if (Commande_Green> -1) 
	SET_OUTPUT(Commande_Green);
	WRITE(Commande_Green, HIGH);
#endif

}



void declaration_personnal_pin()
{


	// Fan 2  
	pinMode(FAN_v2, OUTPUT);

	// LED bleue
	pinMode(LED_blue, OUTPUT);

	//StripLed Gauche et Droite
	pinMode(LED_Rampes_droite, OUTPUT);
	pinMode(LED_Rampes_gauche, OUTPUT);
	//Endstop_voyage

	//Atmel
	//pinMode(ATMEL_IN_PUSH, INPUT);




#if Endstop_voyage > -1
	SET_INPUT(Endstop_voyage); 
#ifdef ENDSTOPPULLUP_Endstop_voyage
	WRITE(Endstop_voyage,HIGH);
#endif
#endif


#if Endstop_Z_Movement > -1
	SET_INPUT(Endstop_Z_Movement); 
#ifdef ENDSTOPPULLUP_Endstop_Z_Movement
	WRITE(Endstop_Z_Movement,HIGH);
#endif
#endif


#if Endstop_Carre > -1
	SET_INPUT(Endstop_Carre);  
#ifdef ENDSTOPPULLUP_Endstop_Carre
	WRITE(Endstop_Carre,HIGH);
#endif
#endif

}	




void Lecture_offset_E2PROM()
{


	//#ifdef EEPROM_SETTINGS
	int offset_x_e2prom=0;
	int position_offset_x_e2prom=1500;
	EEPROM_READ_VAR(position_offset_x_e2prom, offset_x_e2prom);	
	//	MYSERIAL.print("\n X=");					
	//	MYSERIAL.print(offset_x_e2prom);

	int offset_y_e2prom=0;
	int position_offset_y_e2prom=1600;
	EEPROM_READ_VAR(position_offset_y_e2prom, offset_y_e2prom);	
	//		MYSERIAL.print("\n Y=");					
	//		MYSERIAL.print(offset_y_e2prom);
	//		MYSERIAL.print("\n");


	if((offset_x_e2prom<0))
		extruders_offset_changement[0]=0;
	else
	{

		if(offset_x_e2prom>100)
			extruders_offset_changement[0]=100-offset_x_e2prom;
		else
			extruders_offset_changement[0]=offset_x_e2prom;

	}

	//MYSERIAL.print("\n Avant X=");			
	//MYSERIAL.print(extruder_offset[X_AXIS][1]);

	extruder_offset[X_AXIS][1] = extruder_offset[X_AXIS][1]+(extruders_offset_changement[X_AXIS]/10);

	//MYSERIAL.print("\n nouveau X=");			
	//MYSERIAL.print(extruder_offset[X_AXIS][1]);

	if(offset_y_e2prom<0)
		extruders_offset_changement[1]=0;
	else
	{

		if(offset_y_e2prom>100)
			extruders_offset_changement[1]=100-offset_y_e2prom;
		else
			extruders_offset_changement[1]=offset_y_e2prom; 
	}

	//MYSERIAL.print("\n ancien Y=");
	//MYSERIAL.print(extruder_offset[Y_AXIS][1]);
	extruder_offset[Y_AXIS][1] = extruder_offset[Y_AXIS][1]+(extruders_offset_changement[Y_AXIS]/10);
	//MYSERIAL.print("\n nouveau Y=");
	//MYSERIAL.print(extruder_offset[Y_AXIS][1]);


}	









void setup()
{
	setup_killpin(); 
	setup_powerhold();
	setup_green_led();
	suicide_Zim_Setup();
	MYSERIAL.begin(BAUDRATE);
	//Setup_RFID(); ///////////////////// J'ajoute ma commande au sein du Setup.
	MYSERIAL2.begin(BAUDRATE_RFID);
	MYSERIAL3.begin(BAUDRATE_RFID);

	declaration_personnal_pin();

	SERIAL_PROTOCOLLNPGM("start");
	SERIAL_ECHO_START;

	// Check startup - does nothing if bootloader sets MCUSR to 0
	byte mcu = MCUSR;
	if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
	if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
	if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
	if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
	if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
	MCUSR=0;

	SERIAL_ECHOPGM(MSG_MARLIN);
	SERIAL_ECHOLNPGM(VERSION_STRING);
#ifdef STRING_VERSION_CONFIG_H
#ifdef STRING_CONFIG_H_AUTHOR
	SERIAL_ECHO_START;
	SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
	SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
	SERIAL_ECHOPGM(MSG_AUTHOR);
	SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
	SERIAL_ECHOPGM("Compiled: ");
	SERIAL_ECHOLNPGM(__DATE__);
	SERIAL_ECHOPGM("\n VERSION : ");	  
	SERIAL_ECHOLNPGM(VERSION_STRING);
#endif
#endif
	SERIAL_ECHO_START;
	SERIAL_ECHOPGM(MSG_FREE_MEMORY);
	SERIAL_ECHO(freeMemory());
	SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
	SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
	for(int8_t i = 0; i < BUFSIZE; i++)
	{
		fromsd[i] = false;
	}

	// loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
	Config_RetrieveSettings(); 

	tp_init();    // Initialize temperature loop 
	plan_init();  // Initialize planner;
	watchdog_init();
	st_init();    // Initialize stepper, this enables interrupts!
	setup_photpin();

	lcd_init();

#ifdef CONTROLLERFAN_PIN
	SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
#endif

#ifdef EXTRUDERFAN_PIN
	SET_OUTPUT(EXTRUDERFAN_PIN); //Set pin used for extruder cooling fan
#endif

#ifdef RFID_ZIM
	RFID_init();
	//envoie_commande(Initialize_Port,length_Initialize_Port,2);	
	/*
	envoie_commande(Set_Antenna_Status_OFF,length_Set_Antenna_Status_OFF,2); 	
	//envoie_commande(Initialize_Port,length_Initialize_Port,3);	
	envoie_commande(Set_Antenna_Status_OFF,length_Set_Antenna_Status_OFF,3); 
	Antenna_RFID_State= false;	
	MYSERIAL.print("\n");*/
#endif
	Lecture_offset_E2PROM();
	MYSERIAL.print("END_INITIALISATION\n");
}










void loop()
{

	if(buflen < (BUFSIZE-1))
		get_command();
#ifdef SDSUPPORT
	card.checkautostart(false);
#endif
	if(buflen)
	{
#ifdef SDSUPPORT
		if(card.saving)
		{
			if(strstr_P(cmdbuffer[bufindr], PSTR("M29")) == NULL)
			{
				card.write_command(cmdbuffer[bufindr]);
				if(card.logging)
				{
					process_commands();
				}
				else
				{
					MYSERIAL.print("\n");
					SERIAL_PROTOCOLLNPGM(MSG_OK);
				}
			}
			else
			{
				card.closefile(); 
				SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
				SERIAL_PROTOCOLLNPGM(MSG_OK);
			}
		}
		else
		{
			process_commands();
		}
#else
		process_commands(); 
#endif //SDSUPPORT
		buflen = (buflen-1);
		bufindr = (bufindr + 1)%BUFSIZE;
	}
	//check heater every n milliseconds
	manage_heater();
	Read_Power_Button();
	CheckTemperatureToActivateFan();
	manage_inactivity(); 
	checkHitEndstops();
	lcd_update();
	Fonction_Transport();
	Function_Z_Movement();
	Function_Carre ();

} 


#ifdef RFIDSUPPORT
int cmdPort = -1;
#endif


void get_command() 
{ 
	//Ahjout des commande du RFID

#ifdef RFIDSUPPORT
	while( (MYSERIAL.available() ) > 0  && buflen < BUFSIZE) 
	{
		if (-1 == cmdPort) {
			if (MYSERIAL.available()) {
				cmdPort = 0;
			} 
		}
		if (0 == cmdPort) {
			serial_char = MYSERIAL.read();
		}

		/////////// ici j'ai rajout? les messages////////////////////////////////

		if (MSG_PRONTERFACE== true)
		{
			SERIAL_PROTOCOLPGM("J'ai reu cette lettre : ");	   
			SERIAL_PROTOCOL(serial_char); 	 
			SERIAL_PROTOCOLPGM("\n");
		}	 


		if (serial_char == '\n')
		{	
			numero_ligne_pronterface++; 
			/*if (fichier_sd==true)
			{	
			numero_fichier_sd++; 	
			} 	*/

			if(card.saving)
			{
				numero_fichier_sd++; 
			}
		}




		/////////////////////////////////////////////////////////////////

		if(serial_char == '\n' || 
			serial_char == '\r' || 
			(serial_char == ':' && comment_mode == false) || 
			serial_count >= (MAX_CMD_SIZE - 1) ) 
		{
			cmdPort = -1;
#else


	while( MYSERIAL.available() > 0  && buflen < BUFSIZE) 
	{



		serial_char = MYSERIAL.read();


		/////////// ici j'ai rajout? les messages////////////////////////////////

		if (MSG_PRONTERFACE== true)
		{
			SERIAL_PROTOCOLPGM("J'ai reu cette lettre : ");	  
			SERIAL_PROTOCOL(serial_char); 	 
			SERIAL_PROTOCOLPGM("\n");
		}	 


		if (serial_char == '\n')
		{	
			numero_ligne_pronterface++;
			/* if (fichier_sd==true)
			{	
			numero_fichier_sd++; 	
			} 	*/

			if(card.saving)
			{
				numero_fichier_sd++; 
			}
		}
		/////////////////////////////////////////////////////////////////


		if(serial_char == '\n' || 
			serial_char == '\r' || 
			(serial_char == ':' && comment_mode == false) || 
			serial_count >= (MAX_CMD_SIZE - 1) ) 
		{


#endif



			if(!serial_count)
			{ //if empty line
				comment_mode = false; //for new command
				return;
			}

			cmdbuffer[bufindw][serial_count] = 0; //terminate string 

			if(!comment_mode)

			{

				comment_mode = false; //for new command
				fromsd[bufindw] = false;
				if(strchr(cmdbuffer[bufindw], 'N') != NULL)
				{
					strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
					gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));

					if(gcode_N != gcode_LastN+1 && (strstr_P(cmdbuffer[bufindw], PSTR("M110")) == NULL) )
					{
						SERIAL_ERROR_START;
						SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
						SERIAL_ERRORLN(gcode_LastN);
						//Serial.println(gcode_N);
						FlushSerialRequestResend();
						serial_count = 0;
						return;
					}

					if(strchr(cmdbuffer[bufindw], '*') != NULL)
					{
						byte checksum = 0;
						byte count = 0;
						while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
						strchr_pointer = strchr(cmdbuffer[bufindw], '*');

						if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
							SERIAL_ERROR_START;
							SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
							SERIAL_ERRORLN(gcode_LastN);
							FlushSerialRequestResend();
							serial_count = 0;
							return;
						}
						//if no errors, continue parsing
					}
					else 
					{
						SERIAL_ERROR_START;
						SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
						SERIAL_ERRORLN(gcode_LastN);
						FlushSerialRequestResend();
						serial_count = 0;
						return;
					}

					gcode_LastN = gcode_N;
					//if no errors, continue parsing
				}
				else  // if we don't receive 'N' but still see '*'
				{
					if((strchr(cmdbuffer[bufindw], '*') != NULL))
					{
						SERIAL_ERROR_START;
						SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
						SERIAL_ERRORLN(gcode_LastN);
						serial_count = 0;
						return;
					}
				}

				if((strchr(cmdbuffer[bufindw], 'G') != NULL))
				{
					strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
					switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL))))
					{
					case 0:
					case 1:
					case 2:
					case 3:
						if(Stopped == false)
						{ // If printer is stopped by an error the G[0-3] codes are ignored.
#ifdef SDSUPPORT
							if(card.saving)
								break;
#endif //SDSUPPORT
							SERIAL_PROTOCOLLNPGM(MSG_OK); 
						}
						else
						{
							SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
							LCD_MESSAGEPGM(MSG_STOPPED);
						}
						break;
					default:
						break;
					}

				}
				bufindw = (bufindw + 1)%BUFSIZE;
				buflen += 1;
			}
			serial_count = 0; //clear buffer
		}
		else
		{
			if(serial_char == ';') 
				comment_mode = true;

			if(!comment_mode) 
				cmdbuffer[bufindw][serial_count++] = serial_char;
		}	
	}












#ifdef SDSUPPORT
	if(!card.sdprinting || serial_count!=0){
		return;
	}
	while( !card.eof()  && buflen < BUFSIZE) {
		int16_t n=card.get();
		serial_char = (char)n;

		/////ici j'ai rajout?///////////////////////

		if (MSG_SDCARD== true)
		{
			SERIAL_PROTOCOLPGM("J'ai reu cette lettre : ");	  
			SERIAL_PROTOCOL(serial_char); 	 
			SERIAL_PROTOCOLPGM("\n");
		}

		if (serial_char == '\n') 
		{	
			numero_ligne_sd_card++;	
			digitalWrite(13, HIGH);
			delay(1000);
			digitalWrite(13, LOW);
		} 
		////////////////////////////////////



		if(serial_char == '\n' || 
			serial_char == '\r' || 
			(serial_char == ':' && comment_mode == false) || 
			serial_count >= (MAX_CMD_SIZE - 1)||n==-1) 
		{


			if(card.eof()){
				SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
				stoptime=millis();
				char time[30];
				unsigned long t=(stoptime-starttime)/1000;
				int hours, minutes;
				minutes=(t/60)%60;
				hours=t/60/60;
				sprintf_P(time, PSTR("%i hours %i minutes"),hours, minutes);
				SERIAL_ECHO_START;
				SERIAL_ECHOLN(time);
				lcd_setstatus(time);
				card.printingHasFinished();
				card.checkautostart(true);

			}
			if(!serial_count)
			{
				comment_mode = false; //for new command
				return; //if empty line
			}
			cmdbuffer[bufindw][serial_count] = 0; //terminate string
			//      if(!comment_mode){
			fromsd[bufindw] = true;
			buflen += 1;
			bufindw = (bufindw + 1)%BUFSIZE;
			//      }     
			comment_mode = false; //for new command
			serial_count = 0; //clear buffer
		}
		else
		{
			if(serial_char == ';') comment_mode = true;
			if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
		}
	}

#endif //SDSUPPORT

}


float code_value() 
{ 
	return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL)); 
}

long code_value_long() 
{ 
	return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10)); 
}

long code_value_long32() 
{ 
	return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 32)); 
}

bool code_seen(char code)
{
	strchr_pointer = strchr(cmdbuffer[bufindr], code);
	return (strchr_pointer != NULL);  //Return True if a character was found
}

#define DEFINE_PGM_READ_ANY(type, reader)		\
	static inline type pgm_read_any(const type *p)	\
{ return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG)	\
	static const PROGMEM type array##_P[3] =		\
{ X_##CONFIG, Y_##CONFIG, Z_##CONFIG };		\
	static inline type array(int axis)			\
{ return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

static void axis_is_at_home(int axis) {
	current_position[axis] = base_home_pos(axis) + add_homeing[axis];
	min_pos[axis] =          base_min_pos(axis) + add_homeing[axis];
	max_pos[axis] =          base_max_pos(axis) + add_homeing[axis];
}

static void homeaxis(int axis) {
#define HOMEAXIS_DO(LETTER) \
	((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

	if (axis==X_AXIS ? HOMEAXIS_DO(X) :
		axis==Y_AXIS ? HOMEAXIS_DO(Y) :
		axis==Z_AXIS ? HOMEAXIS_DO(Z) : 
		0) {
			current_position[axis] = 0; 
			plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
			destination[axis] = 1.5 * max_length(axis) * home_dir(axis);
			feedrate = homing_feedrate[axis];
			plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
			st_synchronize(); 

			current_position[axis] = 0;
			plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
			destination[axis] = -home_retract_mm(axis) * home_dir(axis);
			plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
			st_synchronize();

			destination[axis] = 2*home_retract_mm(axis) * home_dir(axis);
			feedrate = homing_feedrate[axis]/2 ; 
			plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
			st_synchronize();

			axis_is_at_home(axis);					
			destination[axis] = current_position[axis];
			feedrate = 0.0;
			endstops_hit_on_purpose();
	}
}
#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)


// ici les fonctions pour finir les choses : 

void ecrir_sur_RFID1()
{
	//#ifdef TEST
	//	return; //do nothing for test
	//#endif //TEST

	if (Distance_Filament_E0>=4) {
		Distance_Filament_E0 = Distance_Filament_E0 - 4;
	}

	if(Distance_Filament_E0>0) {
		byte commande_ecriture_tag_E0[]={0x0a,0x00,0x00,0x00,0x13,0x02,0x06,0x00,0x00,0x00,0x00};
		byte length_commande_ecriture_tag_E0 = sizeof(commande_ecriture_tag_E0);
		byte TAG_E0[16]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

		unsigned long int lUsedFilament = (((unsigned long int) TAG_RFID_E0[8] & 0xf) << 16) + ((unsigned long int) TAG_RFID_E0[9] << 8) + (unsigned long int) TAG_RFID_E0[10] + (unsigned long int) ceil(Distance_Filament_E0);

		TAG_RFID_E0[8] = (TAG_RFID_E0[8]  & 0xf0) + (byte) (lUsedFilament >> 16);
		TAG_RFID_E0[9] = (byte) ((lUsedFilament >> 8) & 0xff);
		TAG_RFID_E0[10] = (byte) (lUsedFilament & 0xff);

		//calcul checksum 
		byte checksum_RFID_E0 = 0;

		for (int i=0; i<15;i++) {
			checksum_RFID_E0 = TAG_RFID_E0[i] ^ checksum_RFID_E0;
		}

		TAG_RFID_E0[15] = checksum_RFID_E0;

		for (int i=0; i<16; i++) {
			TAG_E0[i] = TAG_RFID_E0[i]; 
		}

		////Activer l'antenne///
		if (Antenna_RFID_State== false) {						
			//envoie_commande(Set_Antenna_Status_ON,length_Set_Antenna_Status_ON,2); 						
			Antenna_RFID_State= true;
		}		
		//// fin de l'activation///////////	

		envoie_commande(Mifare_Request,length_Mifare_Request,2); 		
		envoie_commande(Mifare_Anticollision,length_Mifare_Anticollision,2); 
		envoie_commande(Mifare_Select,length_Mifare_Select,2); 
		envoie_commande(Mifare_Read1, length_Mifare_Read1, 2);

		for (int i=0; i<4; i++) {
			commande_ecriture_tag_E0[i+7] = TAG_E0[i];
		}	

		envoie_commande(commande_ecriture_tag_E0, length_commande_ecriture_tag_E0, 2);  		

		commande_ecriture_tag_E0[6]=0x07;

		for (int i=4; i<8;i++) {
			commande_ecriture_tag_E0[i+3]=TAG_E0[i];
		}

		envoie_commande(commande_ecriture_tag_E0, length_commande_ecriture_tag_E0, 2); 

		commande_ecriture_tag_E0[6]=0x08;

		for (int i=8; i<12;i++) {
			commande_ecriture_tag_E0[i-1]=TAG_E0[i];
		}
		envoie_commande(commande_ecriture_tag_E0, length_commande_ecriture_tag_E0, 2); 

		commande_ecriture_tag_E0[6]=0x09;

		for (int i=12; i<16;i++) {
			commande_ecriture_tag_E0[i-5]=TAG_E0[i];
		}

		envoie_commande(commande_ecriture_tag_E0, length_commande_ecriture_tag_E0, 2); 	

		envoie_commande(Mifare_Hlta, length_Mifare_Hlta, 2); 
	}
}

void ecrir_sur_RFID2() {
	//#ifdef TEST
	//	return; //do nothing for test
	//#endif //TEST

	if (Distance_Filament_E1>=4) {
		Distance_Filament_E1 = Distance_Filament_E1 - 4;
	}

	if(Distance_Filament_E1>0) {
		byte commande_ecriture_tag_E1[]={0x0a,0x00,0x00,0x00,0x13,0x02,0x06,0x00,0x00,0x00,0x00};
		byte length_commande_ecriture_tag_E1 = sizeof(commande_ecriture_tag_E1);
		byte TAG_E1[16]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

		unsigned long int lUsedFilament = (((unsigned long int) TAG_RFID_E1[8] & 0xf) << 16) + ((unsigned long int) TAG_RFID_E1[9] << 8) + (unsigned long int) TAG_RFID_E1[10] + (unsigned long int) ceil(Distance_Filament_E1);

		TAG_RFID_E1[8] = (TAG_RFID_E1[8]  & 0xf0) + (byte) (lUsedFilament >> 16);
		TAG_RFID_E1[9] = (byte) ((lUsedFilament >> 8) & 0xff);
		TAG_RFID_E1[10] = (byte) (lUsedFilament & 0xff);

		//calcul checksum 
		byte checksum_RFID_E1 = 0x00;

		for (int i=0; i<15;i++) {
			checksum_RFID_E1=TAG_RFID_E1[i]^checksum_RFID_E1;
		}

		TAG_RFID_E1[15]=checksum_RFID_E1;


		for (int i=0;i<16;i++) {
			TAG_E1[i]=TAG_RFID_E1[i];  
		}

		////Activer l'antenne///
		if (Antenna_RFID_State== false) {						
			//envoie_commande(Set_Antenna_Status_ON,length_Set_Antenna_Status_ON,3); 						
			Antenna_RFID_State= true;
		}		
		//// fin de l'activation///////////			

		envoie_commande(Mifare_Request,length_Mifare_Request,3); 		
		envoie_commande(Mifare_Anticollision,length_Mifare_Anticollision,3); 
		envoie_commande(Mifare_Select,length_Mifare_Select,3); 
		envoie_commande(Mifare_Read1,length_Mifare_Read1,3);

		for (int i=0; i<4;i++) {
			commande_ecriture_tag_E1[i+7]=TAG_E1[i];
		}	

		envoie_commande(commande_ecriture_tag_E1,length_commande_ecriture_tag_E1,3);  

		commande_ecriture_tag_E1[6]=0x07;

		for (int i=4; i<8;i++) {
			commande_ecriture_tag_E1[i+3]=TAG_E1[i];
		}

		envoie_commande(commande_ecriture_tag_E1,length_commande_ecriture_tag_E1, 3); 

		commande_ecriture_tag_E1[6]=0x08;

		for (int i=8; i<12;i++) {
			commande_ecriture_tag_E1[i-1]=TAG_E1[i];
		}

		envoie_commande(commande_ecriture_tag_E1,length_commande_ecriture_tag_E1, 3); 

		commande_ecriture_tag_E1[6]=0x09;

		for (int i=12; i<16;i++) {
			commande_ecriture_tag_E1[i-5]=TAG_E1[i];
		}

		envoie_commande(commande_ecriture_tag_E1,length_commande_ecriture_tag_E1,3); 	

		envoie_commande(Mifare_Hlta,length_Mifare_Hlta,3); 
	}
}

void Message_wait()
{
	MYSERIAL.print("wait\n");
}	

void Reverse_20mm()
{

	// first extruder//

	float Distance_Load=4;
	uint8_t sauvegarde_extruder = active_extruder;
	active_extruder=0;
	relative_mode = true;

	if(Distance_Filament_E0>0)
	{
		destination[3] =  - Distance_Load + (axis_relative_modes[3] || relative_mode)*current_position[3];
		destination[0] = current_position[0];
		destination[1] = current_position[1];
		destination[2] = current_position[2];
		feedrate = feedrate_Extrude_Retract; 
		prepare_move();
		//		delay(1000);
	}

	// second extruder//
	active_extruder=1; 	
	if(Distance_Filament_E1>0)
	{
		destination[3] =  - Distance_Load + (axis_relative_modes[3] || relative_mode)*current_position[3];
		destination[0] = current_position[0];
		destination[1] = current_position[1];
		destination[2] = current_position[2];
		feedrate = feedrate_Extrude_Retract; 
		prepare_move();
	}

	active_extruder=sauvegarde_extruder;
	relative_mode = false; 
	//	delay(1000);

}


void Extrude_20mm()
{

	// first extruder//

	float Distance_Load=4;
	uint8_t sauvegarde_extruder = active_extruder;
	active_extruder=0; 		
	relative_mode = true;

	destination[3] = Distance_Load + (axis_relative_modes[3] || relative_mode)*current_position[3];
	destination[0] = current_position[0];
	destination[1] = current_position[1];
	destination[2] = current_position[2];
	feedrate = feedrate_Extrude_Retract; 
	prepare_move(); 

	// second extruder//
	active_extruder=1; 	

	destination[3] = Distance_Load + (axis_relative_modes[3] || relative_mode)*current_position[3];
	destination[0] = current_position[0];
	destination[1] = current_position[1];
	destination[2] = current_position[2];
	feedrate = feedrate_Extrude_Retract; 
	prepare_move(); 


	active_extruder=sauvegarde_extruder;
	relative_mode = false; 
	//	delay(1000);




}	

void Extrude_20mm_first()
{
	// first extruder//
	float Distance_Load=4;
	uint8_t sauvegarde_extruder = active_extruder;
	active_extruder=0; 		
	relative_mode = true;

	destination[3] = Distance_Load + (axis_relative_modes[3] || relative_mode)*current_position[3];
	destination[0] = current_position[0];
	destination[1] = current_position[1];
	destination[2] = current_position[2];
	feedrate = feedrate_Extrude_Retract;
	prepare_move();

	active_extruder=sauvegarde_extruder;
	relative_mode = false; 
	//	delay(1000);
}

void Extrude_20mm_second()
{
	// second extruder//
	float Distance_Load=4;
	uint8_t sauvegarde_extruder = active_extruder;
	active_extruder=1; 		
	relative_mode = true;

	destination[3] = Distance_Load + (axis_relative_modes[3] || relative_mode)*current_position[3];
	destination[0] = current_position[0];
	destination[1] = current_position[1];
	destination[2] = current_position[2];
	feedrate = feedrate_Extrude_Retract; 
	prepare_move();

	active_extruder=sauvegarde_extruder;
	relative_mode = false; 
	//	delay(1000);
}	





void Reinitialise_E0_E1_Quantity()
{

	Distance_Filament_E0 = 0;
	Distance_Filament_E1 = 0;


}

void Set_temperature_to_0Degree()
{
	////////// Set Temperature to 0? /////////////

	uint8_t sauvegarde_extruder = active_extruder;
	active_extruder=1;
	float Temprature_eteinte = 0;
	setTargetHotend(Temprature_eteinte, active_extruder);
	active_extruder=0;
	setTargetHotend(Temprature_eteinte, active_extruder);
	active_extruder=sauvegarde_extruder;

	////////// Fin Temerature to 0? /////////////

	// reset target temperature (by Peng on 20140708)
	TMP0_Target = 0;
	TMP1_Target = 0;
}


void Moving_X_Y()
{
	///////// Homing ////////////////////////
	HOMEAXIS(X);
	HOMEAXIS(Y);
}

void Moving_X_Y_Z_Voyage()
{
	///////// Homing ////////////////////////
	HOMEAXIS(X);
	HOMEAXIS(Z);

	///////// X ////////////////////////
	float Distance_Load_X=75;	
	float Distance_Load_Y=200;	
	float Distance_Load_Z=200;	
	float Distance_Load_Z_parking=94;
	relative_mode = true;
	float feedrate_X_Y = 2000;	
	float feedrate_Z = 200;

	destination[0] = Distance_Load_X + (axis_relative_modes[0] || relative_mode)*current_position[0];
	destination[1] = current_position[1];
	destination[2] = current_position[2];
	destination[3] = current_position[3];
	feedrate = feedrate_X_Y; 
	prepare_move(); 
	//	delay(1000);

	////// Fin X /////////////////////////

	///////// Y ////////////////////////	

	destination[1] = Distance_Load_Y + (axis_relative_modes[1] || relative_mode)*current_position[1];
	destination[0] = current_position[0];
	destination[2] = current_position[2];
	destination[3] = current_position[3];
	feedrate = feedrate_X_Y; 
	prepare_move(); 
	//	delay(1000);

	////// Fin Y /////////////////////////




	///////// Z ////////////////////////

	destination[2] = Distance_Load_Z + (axis_relative_modes[2] || relative_mode)*current_position[2];
	destination[0] = current_position[0];
	destination[1] = current_position[1];
	destination[3] = current_position[3];
	feedrate = feedrate_Z; 
	prepare_move(); 
	//	delay(1000);

	////// Z 2 /////////////////////////


	destination[2] = -Distance_Load_Z_parking + (axis_relative_modes[2] || relative_mode)*current_position[2];
	destination[0] = current_position[0];
	destination[1] = current_position[1];
	destination[3] = current_position[3];
	feedrate = feedrate_Z; 
	prepare_move(); 
	//	delay(1000);

	////// Fin Z /////////////////////////


	relative_mode = false;

}





void Moving_Z_movement()
{



	///////// Homing ////////////////////////
	HOMEAXIS(Z);




	///////// Z ////////////////////////


	//Movement 1
	float Distance_Z_movement=500;		
	relative_mode = true;
	float feedrate_Z_mouvement = 200;

	destination[2] = Distance_Z_movement + (axis_relative_modes[2] || relative_mode)*current_position[2];
	destination[0] = current_position[0];
	destination[1] = current_position[1];
	destination[3] = current_position[3];
	feedrate = feedrate_Z_mouvement; 
	prepare_move(); 
	//	delay(1000);


	//Movement 2

	destination[2] = -Distance_Z_movement + (axis_relative_modes[2] || relative_mode)*current_position[2];
	destination[0] = current_position[0];
	destination[1] = current_position[1];
	destination[3] = current_position[3];
	feedrate = feedrate_Z_mouvement; 
	prepare_move(); 
	//	delay(1000);

	////// Fin Z /////////////////////////


	relative_mode = false;





}



void Moving_Carre()
{




	///////// Homing ////////////////////////
	HOMEAXIS(X);
	HOMEAXIS(Y);

	//// Faire le grand carr?//////

	///////// X ////////////////////////
	float Distance_X_Movement=300;	
	relative_mode = true;
	float feedrate_X_Y_Movement = 2000;

	destination[0] = Distance_X_Movement + (axis_relative_modes[0] || relative_mode)*current_position[0];
	destination[1] = current_position[1];
	destination[2] = current_position[2];
	destination[3] = current_position[3];
	feedrate = feedrate_X_Y_Movement; 
	prepare_move(); 
	//	delay(1000);

	////// Fin X /////////////////////////




	///////// Y ////////////////////////
	float Distance_Y_Movement=300;	

	destination[1] = Distance_Y_Movement + (axis_relative_modes[1] || relative_mode)*current_position[1];
	destination[0] = current_position[0];
	destination[2] = current_position[2];
	destination[3] = current_position[3];
	feedrate = feedrate_X_Y_Movement; 
	prepare_move(); 
	//	delay(1000);

	////// Fin Y /////////////////////////





	///////// X ////////////////////////

	destination[0] = - Distance_X_Movement + (axis_relative_modes[0] || relative_mode)*current_position[0];
	destination[1] = current_position[1];
	destination[2] = current_position[2];
	destination[3] = current_position[3];
	feedrate = feedrate_X_Y_Movement; 
	prepare_move(); 
	//	delay(1000);

	////// Fin X /////////////////////////




	///////// Y ////////////////////////

	destination[1] = - Distance_Y_Movement + (axis_relative_modes[1] || relative_mode)*current_position[1];
	destination[0] = current_position[0];
	destination[2] = current_position[2];
	destination[3] = current_position[3];
	feedrate = feedrate_X_Y_Movement; 
	prepare_move(); 
	//	delay(1000);

	////// Fin Y /////////////////////////



	///////// Allller au centre///////////////////


	///////// X ////////////////////////
	Distance_X_Movement=40;	

	destination[0] = Distance_X_Movement + (axis_relative_modes[0] || relative_mode)*current_position[0];
	destination[1] = current_position[1];
	destination[2] = current_position[2];
	destination[3] = current_position[3];
	feedrate = feedrate_X_Y_Movement; 
	prepare_move(); 
	//	delay(1000);

	////// Fin X /////////////////////////




	///////// Y ////////////////////////
	Distance_Y_Movement=40;	

	destination[1] = Distance_Y_Movement + (axis_relative_modes[1] || relative_mode)*current_position[1];
	destination[0] = current_position[0];
	destination[2] = current_position[2];
	destination[3] = current_position[3];
	feedrate = feedrate_X_Y_Movement; 
	prepare_move(); 
	//	delay(1000);

	////// Fin Y /////////////////////////





	// commencer le petit point du careau




	///////// X ////////////////////////
	Distance_X_Movement=50;	

	destination[0] = Distance_X_Movement + (axis_relative_modes[0] || relative_mode)*current_position[0];
	destination[1] = current_position[1];
	destination[2] = current_position[2];
	destination[3] = current_position[3];
	feedrate = feedrate_X_Y_Movement; 
	prepare_move(); 
	//	delay(1000);

	////// Fin X ///////////////////////// 




	///////// Y ////////////////////////
	Distance_Y_Movement=50;	

	destination[1] = Distance_Y_Movement + (axis_relative_modes[1] || relative_mode)*current_position[1];
	destination[0] = current_position[0];
	destination[2] = current_position[2];
	destination[3] = current_position[3];
	feedrate = feedrate_X_Y_Movement; 
	prepare_move(); 
	//	delay(1000);

	////// Fin Y /////////////////////////





	///////// X ////////////////////////

	destination[0] = - Distance_X_Movement + (axis_relative_modes[0] || relative_mode)*current_position[0];
	destination[1] = current_position[1];
	destination[2] = current_position[2];
	destination[3] = current_position[3];
	feedrate = feedrate_X_Y_Movement; 
	prepare_move(); 
	//	delay(1000);

	////// Fin X /////////////////////////




	///////// Y ////////////////////////

	destination[1] = - Distance_Y_Movement + (axis_relative_modes[1] || relative_mode)*current_position[1];
	destination[0] = current_position[0];
	destination[2] = current_position[2];
	destination[3] = current_position[3];
	feedrate = feedrate_X_Y_Movement; 
	prepare_move(); 
	//	delay(1000);

	////// Fin Y /////////////////////////







	relative_mode = false;


}



void Moving_Z_1()
{



	///////// Homing ////////////////////////
	float Distance_Load_Z_1=50;	
	relative_mode = true;
	float feedrate_Z_1 = 500;

	destination[2] = Distance_Load_Z_1 + (axis_relative_modes[2] || relative_mode)*current_position[2];
	destination[0] = current_position[0];
	destination[1] = current_position[1];
	destination[3] = current_position[3];
	feedrate = feedrate_Z_1; 
	prepare_move(); 
	//	delay(1000);

	////// Fin Homing /////////////////////////

	relative_mode = false;

}


void Moving_Z_2()
{



	///////// Homing ////////////////////////
	float Distance_Load_Z_2=150;	
	relative_mode = true;
	float feedrate_Z_2 = 500;

	destination[2] = Distance_Load_Z_2 + (axis_relative_modes[2] || relative_mode)*current_position[2];
	destination[0] = current_position[0];
	destination[1] = current_position[1];
	destination[3] = current_position[3];
	feedrate = feedrate_Z_2; 
	prepare_move(); 
	//	delay(1000);

	////// Fin Homing /////////////////////////

	relative_mode = false;

}


void Moving_Z_3()
{



	///////// Homing ////////////////////////
	float Distance_Load_Z_3=1;	
	relative_mode = true;
	float feedrate_Z_3 = 200;

	destination[2] = -Distance_Load_Z_3 + (axis_relative_modes[2] || relative_mode)*current_position[2];
	destination[0] = current_position[0];
	destination[1] = current_position[1];
	destination[3] = current_position[3];
	feedrate = feedrate_Z_3; 
	prepare_move(); 
	//	delay(1000); 

	////// Fin Homing /////////////////////////

	relative_mode = false;

}


void Moving_Z_4()
{



	///////// Homing ////////////////////////
	float Distance_Load_Z_4=11;	
	relative_mode = true;
	float feedrate_Z_4 = 200;

	destination[2] = Distance_Load_Z_4 + (axis_relative_modes[2] || relative_mode)*current_position[2];
	destination[0] = current_position[0];
	destination[1] = current_position[1];
	destination[3] = current_position[3];
	feedrate = feedrate_Z_4; 
	prepare_move(); 
	//	delay(1000);

	////// Fin Homing /////////////////////////

	relative_mode = false;

}



void saving_current_parameters()
{

	Pause_current_position[X_parm]=current_position[X_AXIS];
	Pause_current_position[Y_parm]=current_position[Y_AXIS];
	Pause_current_position[Z_parm]=current_position[Z_AXIS];
	Pause_current_position[T0_parm]=TMP0_Target;
	Pause_current_position[T1_parm]=TMP1_Target;



}	

void return_current_parameters()
{

	SERIAL_PROTOCOLPGM("X:");
	SERIAL_PROTOCOL(Pause_current_position[X_parm]);
	SERIAL_PROTOCOLPGM(", Y:");
	SERIAL_PROTOCOL(Pause_current_position[Y_parm]);
	SERIAL_PROTOCOLPGM(", Z:");
	SERIAL_PROTOCOL(Pause_current_position[Z_parm]);
	SERIAL_PROTOCOLPGM(", T0:");      
	SERIAL_PROTOCOL(Pause_current_position[T0_parm]);
	SERIAL_PROTOCOLPGM(", T1:");      
	SERIAL_PROTOCOL(Pause_current_position[T1_parm]);




}

void Set_current_paramters()
{




	//Moving for X

	relative_mode = false;

	destination[0] = Pause_current_position[X_AXIS] + (axis_relative_modes[0] || relative_mode)*current_position[0];
	destination[1] = current_position[1];
	destination[2] = current_position[2];	
	destination[3] = current_position[3];
	feedrate = 2000;
	prepare_move(); 
	//	delay(1000);

	//Moving for Y

	destination[1] = Pause_current_position[Y_AXIS] + (axis_relative_modes[1] || relative_mode)*current_position[1];
	destination[0] = current_position[0];
	destination[2] = current_position[2];	
	destination[3] = current_position[3];
	feedrate = 2000;
	prepare_move(); 
	//	delay(1000);					

	//Moving for Z

	destination[2] = Pause_current_position[Z_AXIS] + (axis_relative_modes[2] || relative_mode)*current_position[2];
	destination[0] = current_position[0];
	destination[1] = current_position[1];
	destination[3] = current_position[3];
	feedrate = 500;
	prepare_move(); 					

	//	delay(1000);


	// For T0

	uint8_t sauvegarde_extruder = active_extruder;
	active_extruder=0; 							
	setTargetHotend(Pause_current_position[T0_parm], active_extruder);

	// For T1
	active_extruder=1; 							
	setTargetHotend(Pause_current_position[T1_parm], active_extruder);
	active_extruder=sauvegarde_extruder;





}






void Set_current_paramters_2()
{




	//Moving for X

	relative_mode = false;

	destination[0] = Pause_current_position[X_AXIS] + (axis_relative_modes[0] || relative_mode)*current_position[0];
	destination[1] = current_position[1];
	destination[2] = current_position[2];	
	destination[3] = current_position[3];
	feedrate = 2000;
	prepare_move(); 
	//	delay(1000);

	//Moving for Y

	destination[1] = Pause_current_position[Y_AXIS] + (axis_relative_modes[1] || relative_mode)*current_position[1];
	destination[0] = current_position[0];
	destination[2] = current_position[2];	
	destination[3] = current_position[3];
	feedrate = 2000;
	prepare_move(); 
	//	delay(1000);					

	//Moving for Z

	destination[2] = Pause_current_position[Z_AXIS] + (axis_relative_modes[2] || relative_mode)*current_position[2];
	destination[0] = current_position[0];
	destination[1] = current_position[1];
	destination[3] = current_position[3];
	feedrate = 500;
	prepare_move(); 					

	//	delay(1000);







}






void kill_Zim()
{

	MYSERIAL.print("\n Oh noooo, You kill the ZIM :-( \n");
	quickStop(); 

	digitalWrite(Commande_Green, LOW);   // turn the LED on (LOW is the voltage level)
	//  delay(1000); 
	digitalWrite(Commande_Green, HIGH);   // turn the LED on (HIGH is the voltage level)
	//  delay(1000);               // wait for a second
	digitalWrite(Commande_Green, LOW);    // turn the LED off by making the voltage LOW
	//  delay(1000);

	digitalWrite(Commande_Green, HIGH);   // turn the LED on (HIGH is the voltage level)
	//  delay(1000);               // wait for a second
	digitalWrite(Commande_Green, LOW);    // turn the LED off by making the voltage LOW
	//  delay(1000);               // wait for a second
	// digitalWrite(SUICIDE_Zim_PIN, HIGH);    // turn the LED off by making the voltage LOW
	//delay(1000);



	//ecrir_sur_RFID1();


	//ecrir_sur_RFID2();  

	if ((Distance_Filament_E0>0)||(Distance_Filament_E1>0))
	{

		Moving_Z_4();
	}

	Reverse_20mm();	

	//Reinitialise_E0_E1_Quantity();

	//MYSERIAL.print("/n move_Z\n");

	//Moving_X_Y();
	//Set_temperature_to_0Degree();  
	disable_heater(); 
	Moving_Z_3(); 


	while(blocks_queued())
	{
		WRITE(Commande_Green, HIGH);
		delay(250);   
		WRITE(Commande_Green, LOW);
		delay(250);
	}		


	digitalWrite(Commande_Green, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(100);               // wait for a second
	digitalWrite(Commande_Green, LOW);    // turn the LED off by making the voltage LOW
	delay(100);

	digitalWrite(Commande_Green, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(100);               // wait for a second
	digitalWrite(Commande_Green, LOW);    // turn the LED off by making the voltage LOW
	delay(100);               // wait for a second



	cli(); // Stop interrupts
	disable_x();
	disable_y();
	disable_z();
	disable_e0();
	disable_e1();
	disable_e2();

	//if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,INPUT);
	SERIAL_ERROR_START;
	SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
	LCD_ALERTMESSAGEPGM(MSG_KILLED);




	suicide_Zim();
	/*
	while(1) { // Intentionally left empty  
	} // Wait for reset 
	*/

	void(* resetFunc) (void) = 0; 
	resetFunc();  



}


void process_commands()
{
	unsigned long codenum; //throw away variable
	char *starpos = NULL;

	if(code_seen('G'))
	{
		switch((int)code_value())
		{
		case 0: // G0 -> G1
		case 1: // G1
			if(Stopped == false) {
				get_coordinates(); // For X Y Z E F
				prepare_move();
				//ClearToSend();
				return;
			}
			//break;
		case 2: // G2  - CW ARC
			if(Stopped == false) {
				get_arc_coordinates();
				prepare_arc_move(true);
				return;
			}
		case 3: // G3  - CCW ARC
			if(Stopped == false) {
				get_arc_coordinates();
				prepare_arc_move(false);
				return;
			}
		case 4: // G4 dwell
			LCD_MESSAGEPGM(MSG_DWELL);
			codenum = 0;
			if(code_seen('P')) codenum = code_value(); // milliseconds to wait
			if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

			st_synchronize();
			codenum += millis();  // keep track of when we started waiting
			previous_millis_cmd = millis();
			while(millis()  < codenum ){
				manage_heater();
				Read_Power_Button();
				CheckTemperatureToActivateFan();
				manage_inactivity();
				lcd_update();
			}
			break;
		case 5: // Idem G1 then deactivate steppers
			if(Stopped == false) {
				get_coordinates(); // For X Y Z E F
				prepare_move();
				while(blocks_queued()) {
					delay(500);
					Message_wait();
				}
				disable_x(); 
				disable_y(); 
				disable_z(); 
				disable_e0(); 
				disable_e1(); 
				disable_e2();
			}
			break;
#ifdef FWRETRACT  
		case 10: // G10 retract
			if(!retracted) 
			{
				destination[X_AXIS]=current_position[X_AXIS];
				destination[Y_AXIS]=current_position[Y_AXIS];
				destination[Z_AXIS]=current_position[Z_AXIS]; 
				current_position[Z_AXIS]+=-retract_zlift;
				destination[E_AXIS]=current_position[E_AXIS]-retract_length; 
				feedrate=retract_feedrate;
				retracted=true;
				prepare_move();
			}

			break;
		case 11: // G10 retract_recover
			if(!retracted) 
			{
				destination[X_AXIS]=current_position[X_AXIS];
				destination[Y_AXIS]=current_position[Y_AXIS];
				destination[Z_AXIS]=current_position[Z_AXIS]; 

				current_position[Z_AXIS]+=retract_zlift;
				current_position[E_AXIS]+=-retract_recover_length; 
				feedrate=retract_recover_feedrate;
				retracted=false;
				prepare_move();
			}
			break;
#endif //FWRETRACT
		case 28: //G28 Home all Axis one at a time
			saved_feedrate = feedrate;
			saved_feedmultiply = feedmultiply;
			feedmultiply = 100;
			previous_millis_cmd = millis();

			enable_endstops(true);

			for(int8_t i=0; i < NUM_AXIS; i++) {
				destination[i] = current_position[i];
			}
			feedrate = 0.0;
			home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));

#if Z_HOME_DIR > 0                      // If homing away from BED do Z first
			if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
				HOMEAXIS(Z);
			}
#endif

#ifdef QUICK_HOME
			if((home_all_axis)||( code_seen(axis_codes[X_AXIS]) && code_seen(axis_codes[Y_AXIS])) )  //first diagonal move
			{
				current_position[X_AXIS] = 0;current_position[Y_AXIS] = 0;  

				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]); 
				destination[X_AXIS] = 1.5 * X_MAX_LENGTH * X_HOME_DIR;destination[Y_AXIS] = 1.5 * Y_MAX_LENGTH * Y_HOME_DIR;  
				feedrate = homing_feedrate[X_AXIS]; 
				if(homing_feedrate[Y_AXIS]<feedrate)
					feedrate =homing_feedrate[Y_AXIS]; 
				plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
				st_synchronize();

				axis_is_at_home(X_AXIS);
				axis_is_at_home(Y_AXIS);
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
				destination[X_AXIS] = current_position[X_AXIS];
				destination[Y_AXIS] = current_position[Y_AXIS];
				plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
				feedrate = 0.0;
				st_synchronize();
				endstops_hit_on_purpose();
			}
#endif

			if((home_all_axis) || (code_seen(axis_codes[X_AXIS]))) 
			{
				HOMEAXIS(X);
			}

			if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
				HOMEAXIS(Y);
			}

#if Z_HOME_DIR < 0                      // If homing towards BED do Z last
			if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
				HOMEAXIS(Z);
			}
#endif

			if(code_seen(axis_codes[X_AXIS])) 
			{
				if(code_value_long() != 0) {
					current_position[X_AXIS]=code_value()+add_homeing[0];
				}
			}

			if(code_seen(axis_codes[Y_AXIS])) {
				if(code_value_long() != 0) {
					current_position[Y_AXIS]=code_value()+add_homeing[1];
				}
			}

			if(code_seen(axis_codes[Z_AXIS])) {
				if(code_value_long() != 0) {
					current_position[Z_AXIS]=code_value()+add_homeing[2];
				}
			}
			plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

#ifdef ENDSTOPS_ONLY_FOR_HOMING
			enable_endstops(false);
#endif

			feedrate = saved_feedrate;
			feedmultiply = saved_feedmultiply;
			previous_millis_cmd = millis();
			endstops_hit_on_purpose();
			break;
		case 90: // G90
			relative_mode = false;
			break;
		case 91: // G91
			relative_mode = true;
			break;
		case 92: // G92
			if(!code_seen(axis_codes[E_AXIS]))
				st_synchronize();
			for(int8_t i=0; i < NUM_AXIS; i++) {
				if(code_seen(axis_codes[i])) { 
					if(i == E_AXIS) {
						current_position[i] = code_value();  
						plan_set_e_position(current_position[E_AXIS]);
					}
					else {
						current_position[i] = code_value()+add_homeing[i];  
						plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
					}
				}
			}
			break;


			// Par Deafault : On renvoit toujours Unknown Command
		default : 
			{


				SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
				SERIAL_ECHO(cmdbuffer[bufindr]);
				SERIAL_ECHOLNPGM("\"");



			}


		}
	}

	else if(code_seen('M'))
	{
		switch( (int)code_value() ) 
		{
#ifdef ULTIPANEL
		case 0: // M0 - Unconditional stop - Wait for user button press on LCD
		case 1: // M1 - Conditional stop - Wait for user button press on LCD
			{
				LCD_MESSAGEPGM(MSG_USERWAIT);
				codenum = 0;
				if(code_seen('P')) codenum = code_value(); // milliseconds to wait
				if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

				st_synchronize();
				previous_millis_cmd = millis();
				if (codenum > 0){
					codenum += millis();  // keep track of when we started waiting
					while(millis()  < codenum && !LCD_CLICKED){
						manage_heater();
						Read_Power_Button();
						CheckTemperatureToActivateFan();
						manage_inactivity();
						lcd_update();
					}
				}else{
					while(!LCD_CLICKED){
						manage_heater();
						Read_Power_Button();
						CheckTemperatureToActivateFan();
						manage_inactivity();
						lcd_update();
					}
				} 
				LCD_MESSAGEPGM(MSG_RESUMING);
			}
			break;
#endif
		case 17:
			LCD_MESSAGEPGM(MSG_NO_MOVE);
			enable_x(); 
			enable_y(); 
			enable_z(); 
			enable_e0(); 
			enable_e1(); 
			enable_e2(); 
			break;

#ifdef SDSUPPORT
		case 20: // M20 - list SD card
			SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
			card.ls();
			SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
			break;
		case 21: // M21 - init SD card

			card.initsd();
			numero_ligne_sd_card = 0; 

			break;
		case 22: //M22 - release SD card
			card.release();

			break;
		case 23: //M23 - Select file
			starpos = (strchr(strchr_pointer + 4,'*'));
			name_fichier_supprimer = strchr_pointer;

			if(starpos!=NULL)
				*(starpos-1)='\0';
			card.openFile(strchr_pointer + 4,true);
			break;
		case 24: //M24 - Start SD print
			card.startFileprint();
			starttime=millis();
			break;
		case 25: //M25 - Pause SD print
			card.pauseSDPrint();
			break;
		case 26: //M26 - Set SD index
			if(card.cardOK && code_seen('S')) {
				card.setIndex(code_value_long());
			}
			break;
		case 27: //M27 - Get SD status
			card.getStatus();
			break;
		case 28: //M28 - Start SD write
			numero_fichier_sd=-1;
			starpos = (strchr(strchr_pointer + 4,'*'));
			if(starpos != NULL){
				char* npos = strchr(cmdbuffer[bufindr], 'N');
				strchr_pointer = strchr(npos,' ') + 1;
				*(starpos-1) = '\0';
			}
			card.openFile(strchr_pointer+4,false);
			break;
		case 29: //M29 - Stop SD write
			//processed in write to file routine above
			//card,saving = false;
			break;
		case 30: //M30 <filename> Delete File 
			if (card.cardOK){
				card.closefile();
				starpos = (strchr(strchr_pointer + 4,'*'));
				if(starpos != NULL){
					char* npos = strchr(cmdbuffer[bufindr], 'N');
					strchr_pointer = strchr(npos,' ') + 1;
					*(starpos-1) = '\0';
				}
				card.removeFile(strchr_pointer + 4);
			}
			break;
		case 928: //M928 - Start SD write
			starpos = (strchr(strchr_pointer + 5,'*'));
			if(starpos != NULL){
				char* npos = strchr(cmdbuffer[bufindr], 'N');
				strchr_pointer = strchr(npos,' ') + 1;
				*(starpos-1) = '\0';
			}
			card.openLogFile(strchr_pointer+5);
			break;

#endif //SDSUPPORT

		case 31: //M31 take time since the start of the SD print or an M109 command
			{
				stoptime=millis();
				char time[30];
				unsigned long t=(stoptime-starttime)/1000;
				int sec,min;
				min=t/60;
				sec=t%60;
				sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
				SERIAL_ECHO_START;
				SERIAL_ECHOLN(time);
				lcd_setstatus(time);
				autotempShutdown();
			}
			break;
		case 42: //M42 -Change pin status via gcode
			if (code_seen('S'))
			{
				int pin_status = code_value();
				int pin_number = LED_PIN;
				if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
					pin_number = code_value();
				for(int8_t i = 0; i < (int8_t)sizeof(sensitive_pins); i++)
				{
					if (sensitive_pins[i] == pin_number)
					{
						pin_number = -1;
						break;
					}
				}
				if (pin_number > -1)
				{
					pinMode(pin_number, OUTPUT);
					digitalWrite(pin_number, pin_status);
					analogWrite(pin_number, pin_status);
				}
			}
			break;
		case 104: // M104
			if(setTargetedHotend(104)){
				break;
			}
			if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
			if (tmp_extruder == 0)
			{
				TMP0_Target = code_value();
			}		

			if (tmp_extruder == 1)
			{
				TMP1_Target = code_value();
			}	  
			setWatch();
			break;
		case 140: // M140 set bed temp
			if (code_seen('S')) setTargetBed(code_value());
			break;
		case 105 : // M105
			if(setTargetedHotend(105)){
				break;
			}
#if (TEMP_0_PIN > -1)
			SERIAL_PROTOCOLPGM("ok T:");
			SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1); 
			SERIAL_PROTOCOLPGM(" /");
			SERIAL_PROTOCOL_F(degTargetHotend(tmp_extruder),1); 
#if TEMP_BED_PIN > -1
			SERIAL_PROTOCOLPGM(" B:");  
			SERIAL_PROTOCOL_F(degBed(),1);
			SERIAL_PROTOCOLPGM(" /");
			SERIAL_PROTOCOL_F(degTargetBed(),1);
#endif //TEMP_BED_PIN
#else
			SERIAL_ERROR_START;
			SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
#endif

			SERIAL_PROTOCOLPGM(" @:");
			SERIAL_PROTOCOL(getHeaterPower(tmp_extruder));  

			SERIAL_PROTOCOLPGM(" B@:");
			SERIAL_PROTOCOL(getHeaterPower(-1));  

			SERIAL_PROTOCOLLN("");
			return;
			break;
		case 109: 
			{// M109 - Wait for extruder heater to reach target.
				if(setTargetedHotend(109)){
					break;
				}
				LCD_MESSAGEPGM(MSG_HEATING);   
#ifdef AUTOTEMP
				autotemp_enabled=false;
#endif
				if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
#ifdef AUTOTEMP
				if (code_seen('S')) autotemp_min=code_value();
				if (code_seen('B')) autotemp_max=code_value();
				if (code_seen('F')) 
				{
					autotemp_factor=code_value();
					autotemp_enabled=true;
				}
#endif

				if (tmp_extruder == 0)
				{
					TMP0_Target = code_value();
				}		

				if (tmp_extruder == 1)
				{
					TMP1_Target = code_value();
				}	 

				setWatch();
				codenum = millis(); 

				/* See if we are heating up or cooling down */
				bool target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling

#ifdef TEMP_RESIDENCY_TIME
				long residencyStart;
				residencyStart = -1;
				/* continue to loop until we have reached the target temp   
				_and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
				while((residencyStart == -1) ||
					(residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL))) ) {
#else
				while ( target_direction ? (isHeatingHotend(tmp_extruder)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) ) {
#endif //TEMP_RESIDENCY_TIME
					if( (millis() - codenum) > 1000UL )
					{ //Print Temp Reading and remaining time every 1 second while heating up/cooling down
						SERIAL_PROTOCOLPGM("T:");
						SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1); 
						SERIAL_PROTOCOLPGM(" E:");
						SERIAL_PROTOCOL((int)tmp_extruder); 
#ifdef TEMP_RESIDENCY_TIME
						SERIAL_PROTOCOLPGM(" W:");
						if(residencyStart > -1)
						{
							codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
							SERIAL_PROTOCOLLN( codenum );
						}
						else 
						{
							SERIAL_PROTOCOLLN( "?" );
						}
#else
						SERIAL_PROTOCOLLN("");
#endif
						codenum = millis();
					}
					manage_heater();
					Read_Power_Button();
					CheckTemperatureToActivateFan();
					manage_inactivity();
					lcd_update();
#ifdef TEMP_RESIDENCY_TIME
					/* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
					or when current temp falls outside the hysteresis after target temp was reached */
					if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
						(residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
						(residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) ) 
					{
						residencyStart = millis();
					}
#endif //TEMP_RESIDENCY_TIME
				}
				LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
				starttime=millis();
				previous_millis_cmd = millis();
			}
			break;
		case 190: // M190 - Wait for bed heater to reach target.
#if TEMP_BED_PIN > -1
			LCD_MESSAGEPGM(MSG_BED_HEATING);
			if (code_seen('S')) setTargetBed(code_value());
			codenum = millis(); 
			while(isHeatingBed()) 
			{
				if(( millis() - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
				{
					float tt=degHotend(active_extruder);
					SERIAL_PROTOCOLPGM("T:");
					SERIAL_PROTOCOL(tt);
					SERIAL_PROTOCOLPGM(" E:");
					SERIAL_PROTOCOL((int)active_extruder); 
					SERIAL_PROTOCOLPGM(" B:");
					SERIAL_PROTOCOL_F(degBed(),1); 
					SERIAL_PROTOCOLLN(""); 
					codenum = millis(); 
				}
				manage_heater();
				Read_Power_Button();
				CheckTemperatureToActivateFan();
				manage_inactivity();
				lcd_update();
			}
			LCD_MESSAGEPGM(MSG_BED_DONE);
			previous_millis_cmd = millis();
#endif
			break;

#if FAN_PIN > -1
		case 106: //M106 Fan On
			// block M106 when temperature is above 205 degrees
			//		if ((degHotend(0) > 205) || (degHotend(1) > 205)) {
			//			break;
			//		}

			if (code_seen('S')){
				fanSpeed=constrain(code_value(),0,255);
			}
			else {
				fanSpeed=255;			
			}
			break;
		case 107: //M107 Fan Off
			fanSpeed = 0;
			break;
#endif //FAN_PIN

#if (PS_ON_PIN > -1)
		case 80: // M80 - ATX Power On
			SET_OUTPUT(PS_ON_PIN); //GND
			WRITE(PS_ON_PIN, PS_ON_AWAKE);
			break;
#endif

		case 81: // M81 - ATX Power Off

#if defined SUICIDE_PIN && SUICIDE_PIN > -1
			st_synchronize();
			suicide();
#elif (PS_ON_PIN > -1)
			SET_OUTPUT(PS_ON_PIN); 
			WRITE(PS_ON_PIN, PS_ON_ASLEEP);
#endif
			break;

		case 82:
			axis_relative_modes[3] = false;
			break;
		case 83:
			axis_relative_modes[3] = true;
			break;
		case 18: //compatibility
		case 84: // M84
			if(code_seen('S')){ 
				stepper_inactive_time = code_value() * 1000; 
			}
			else
			{ 
				bool all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2]))|| (code_seen(axis_codes[3])));
				if(all_axis)
				{
					st_synchronize();
					disable_e0();
					disable_e1();
					disable_e2();
					finishAndDisableSteppers();
				}
				else
				{
					st_synchronize();
					if(code_seen('X')) disable_x();
					if(code_seen('Y')) disable_y();
					if(code_seen('Z')) disable_z();
#if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
					if(code_seen('E')) {
						disable_e0();
						disable_e1();
						disable_e2();
					}
#endif 
				}
			}
			break;
		case 85: // M85
			code_seen('S');
			max_inactive_time = code_value() * 1000; 
			break;
		case 92: // M92
			for(int8_t i=0; i < NUM_AXIS; i++) 
			{
				if(code_seen(axis_codes[i])) 
				{
					if(i == 3) { // E
						float value = code_value();
						if(value < 20.0) {
							float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
							max_e_jerk *= factor;
							max_feedrate[i] *= factor;
							axis_steps_per_sqr_second[i] *= factor;
						}
						axis_steps_per_unit[i] = value;
					}
					else {
						axis_steps_per_unit[i] = code_value();
					}
				}
			}
			break;
		case 115: // M115
			SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
			break;
		case 117: // M117 display message
			starpos = (strchr(strchr_pointer + 5,'*'));
			if(starpos!=NULL)
				*(starpos-1)='\0';
			lcd_setstatus(strchr_pointer + 5);
			break;
		case 114: // M114
			SERIAL_PROTOCOLPGM("X:");
			SERIAL_PROTOCOL(current_position[X_AXIS]);
			SERIAL_PROTOCOLPGM("Y:");
			SERIAL_PROTOCOL(current_position[Y_AXIS]);
			SERIAL_PROTOCOLPGM("Z:");
			SERIAL_PROTOCOL(current_position[Z_AXIS]);
			SERIAL_PROTOCOLPGM("E:");      
			SERIAL_PROTOCOL(current_position[E_AXIS]);

			SERIAL_PROTOCOLPGM(MSG_COUNT_X);
			SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
			SERIAL_PROTOCOLPGM("Y:");
			SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
			SERIAL_PROTOCOLPGM("Z:");
			SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

			SERIAL_PROTOCOLLN("");
			break;
		case 120: // M120
			enable_endstops(false) ;
			break;
		case 121: // M121
			enable_endstops(true) ;
			break;
		case 119: // M119
			SERIAL_PROTOCOLLN(MSG_M119_REPORT);
#if (X_MIN_PIN > -1)
			SERIAL_PROTOCOLPGM(MSG_X_MIN);
			SERIAL_PROTOCOLLN(((READ(X_MIN_PIN)^X_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if (X_MAX_PIN > -1)
			SERIAL_PROTOCOLPGM(MSG_X_MAX);
			SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if (Y_MIN_PIN > -1)
			SERIAL_PROTOCOLPGM(MSG_Y_MIN);
			SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN)^Y_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if (Y_MAX_PIN > -1)
			SERIAL_PROTOCOLPGM(MSG_Y_MAX);
			SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if (Z_MIN_PIN > -1)
			SERIAL_PROTOCOLPGM(MSG_Z_MIN);
			SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN)^Z_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if (Z_MAX_PIN > -1)
			SERIAL_PROTOCOLPGM(MSG_Z_MAX);
			SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if (PRIVATE_ENDSTOPS1 > -1)
			SERIAL_PROTOCOLPGM("E0: ");
			SERIAL_PROTOCOLLN(((READ(PRIVATE_ENDSTOPS1)^E_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if (PRIVATE_ENDSTOPS2 > -1)
			SERIAL_PROTOCOLPGM("E1: ");
			SERIAL_PROTOCOLLN(((READ(PRIVATE_ENDSTOPS2)^E_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
			break;
			//TODO: update for all axis, use for loop
		case 201: // M201
			for(int8_t i=0; i < NUM_AXIS; i++) 
			{
				if(code_seen(axis_codes[i]))
				{
					max_acceleration_units_per_sq_second[i] = code_value();
				}
			}
			// steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
			reset_acceleration_rates();
			break;
#if 0 // Not used for Sprinter/grbl gen6
		case 202: // M202
			for(int8_t i=0; i < NUM_AXIS; i++) {
				if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
			}
			break;
#endif
		case 203: // M203 max feedrate mm/sec
			for(int8_t i=0; i < NUM_AXIS; i++) {
				if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
			}
			break;
		case 204: // M204 acclereration S normal moves T filmanent only moves
			{
				if(code_seen('S')) acceleration = code_value() ;
				if(code_seen('T')) retract_acceleration = code_value() ;
			}
			break;
		case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
			{
				if(code_seen('S')) minimumfeedrate = code_value();
				if(code_seen('T')) mintravelfeedrate = code_value();
				if(code_seen('B')) minsegmenttime = code_value() ;
				if(code_seen('X')) max_xy_jerk = code_value() ;
				if(code_seen('Z')) max_z_jerk = code_value() ;
				if(code_seen('E')) max_e_jerk = code_value() ;
			}
			break;
		case 206: // M206 additional homeing offset
			for(int8_t i=0; i < 3; i++) 
			{
				if(code_seen(axis_codes[i])) add_homeing[i] = code_value();
			}
			break;
#ifdef FWRETRACT
		case 207: //M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
			{
				if(code_seen('S')) 
				{
					retract_length = code_value() ;
				}
				if(code_seen('F')) 
				{
					retract_feedrate = code_value() ;
				}
				if(code_seen('Z')) 
				{
					retract_zlift = code_value() ;
				}
			}break;
		case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
			{
				if(code_seen('S')) 
				{
					retract_recover_length = code_value() ;
				}
				if(code_seen('F')) 
				{
					retract_recover_feedrate = code_value() ;
				}
			}break;
		case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
			{
				if(code_seen('S')) 
				{
					int t= code_value() ;
					switch(t)
					{
					case 0: autoretract_enabled=false;retracted=false;break;
					case 1: autoretract_enabled=true;retracted=false;break;
					default: 
						SERIAL_ECHO_START;
						SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
						SERIAL_ECHO(cmdbuffer[bufindr]);
						SERIAL_ECHOLNPGM("\"");
					}
				}

			}break;
#endif // FWRETRACT
#if EXTRUDERS > 1
		case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
			{
				if(setTargetedHotend(218)){
					break;
				}
				if(code_seen('X')) 
				{
					extruder_offset[X_AXIS][tmp_extruder] = code_value();
				}
				if(code_seen('Y'))
				{
					extruder_offset[Y_AXIS][tmp_extruder] = code_value();
				}
				SERIAL_ECHO_START;
				SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
				for(tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++) 
				{
					SERIAL_ECHO(" ");
					SERIAL_ECHO(extruder_offset[X_AXIS][tmp_extruder]);
					SERIAL_ECHO(",");
					SERIAL_ECHO(extruder_offset[Y_AXIS][tmp_extruder]);
				}
				SERIAL_ECHOLN("");
			}break;
#endif
		case 220: // M220 S<factor in percent>- set speed factor override percentage
			{
				if(code_seen('S')) 
				{
					feedmultiply = code_value() ;
				}
			}
			break;
		case 221: // M221 S<factor in percent>- set extrude factor override percentage
			{
				if(code_seen('S')) 
				{
					extrudemultiply = code_value() ;
				}
			}
			break;

#if defined(LARGE_FLASH) && LARGE_FLASH == true && defined(BEEPER) && BEEPER > -1
		case 300: // M300
			{
				int beepS = 1;
				int beepP = 1000;
				if(code_seen('S')) beepS = code_value();
				if(code_seen('P')) beepP = code_value();
				tone(BEEPER, beepS);
				delay(beepP);
				noTone(BEEPER);
			}
			break;
#endif // M300

#ifdef PIDTEMP
		case 301: // M301
			{
				if(code_seen('P')) Kp = code_value();
				if(code_seen('I')) Ki = scalePID_i(code_value());
				if(code_seen('D')) Kd = scalePID_d(code_value());

#ifdef PID_ADD_EXTRUSION_RATE
				if(code_seen('C')) Kc = code_value();
#endif

				updatePID();
				SERIAL_PROTOCOL(MSG_OK);
				SERIAL_PROTOCOL(" p:");
				SERIAL_PROTOCOL(Kp);
				SERIAL_PROTOCOL(" i:");
				SERIAL_PROTOCOL(unscalePID_i(Ki));
				SERIAL_PROTOCOL(" d:");
				SERIAL_PROTOCOL(unscalePID_d(Kd));
#ifdef PID_ADD_EXTRUSION_RATE
				SERIAL_PROTOCOL(" c:");
				//Kc does not have scaling applied above, or in resetting defaults
				SERIAL_PROTOCOL(Kc);
#endif
				SERIAL_PROTOCOLLN("");
			}
			break;
#endif //PIDTEMP
#ifdef PIDTEMPBED
		case 304: // M304
			{
				if(code_seen('P')) bedKp = code_value();
				if(code_seen('I')) bedKi = scalePID_i(code_value());
				if(code_seen('D')) bedKd = scalePID_d(code_value());

				updatePID();
				SERIAL_PROTOCOL(MSG_OK);
				SERIAL_PROTOCOL(" p:");
				SERIAL_PROTOCOL(bedKp);
				SERIAL_PROTOCOL(" i:");
				SERIAL_PROTOCOL(unscalePID_i(bedKi));
				SERIAL_PROTOCOL(" d:");
				SERIAL_PROTOCOL(unscalePID_d(bedKd));
				SERIAL_PROTOCOLLN("");
			}
			break;
#endif //PIDTEMP
		case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
			{
#ifdef PHOTOGRAPH_PIN
#if (PHOTOGRAPH_PIN > -1)
				const uint8_t NUM_PULSES=16;
				const float PULSE_LENGTH=0.01524;
				for(int i=0; i < NUM_PULSES; i++) {
					WRITE(PHOTOGRAPH_PIN, HIGH);
					_delay_ms(PULSE_LENGTH);
					WRITE(PHOTOGRAPH_PIN, LOW);
					_delay_ms(PULSE_LENGTH);
				}
				delay(7.33);
				for(int i=0; i < NUM_PULSES; i++) {
					WRITE(PHOTOGRAPH_PIN, HIGH);
					_delay_ms(PULSE_LENGTH);
					WRITE(PHOTOGRAPH_PIN, LOW);
					_delay_ms(PULSE_LENGTH);
				}
#endif
#endif
			}
			break;

		case 302: // allow cold extrudes
			{
				allow_cold_extrudes(true);
			}
			break;
		case 303: // M303 PID autotune
			{
				float temp = 150.0;
				int e=0;
				int c=5;
				if (code_seen('E')) e=code_value();
				if (e<0)
					temp=70;
				if (code_seen('S')) temp=code_value();
				if (code_seen('C')) c=code_value();
				PID_autotune(temp, e, c);
			}
			break;
		case 400: // M400 finish all moves
			{
				st_synchronize();
			}
			break;
		case 500: // M500 Store settings in EEPROM
			{
				Config_StoreSettings();
			}
			break;
		case 501: // M501 Read settings from EEPROM
			{
				Config_RetrieveSettings();
			}
			break;
		case 502: // M502 Revert to default settings
			{
				Config_ResetDefault();
			}
			break;
		case 503: // M503 print settings currently in memory 
			{
				Config_PrintSettings();
			}
			break;
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
		case 540:
			{
				if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
			}
			break;
#endif
#ifdef FILAMENTCHANGEENABLE
		case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
			{
				float target[4];
				float lastpos[4];
				target[X_AXIS]=current_position[X_AXIS];
				target[Y_AXIS]=current_position[Y_AXIS];
				target[Z_AXIS]=current_position[Z_AXIS];
				target[E_AXIS]=current_position[E_AXIS];
				lastpos[X_AXIS]=current_position[X_AXIS];
				lastpos[Y_AXIS]=current_position[Y_AXIS];
				lastpos[Z_AXIS]=current_position[Z_AXIS];
				lastpos[E_AXIS]=current_position[E_AXIS];
				//retract by E
				if(code_seen('E')) 
				{
					target[E_AXIS]+= code_value();
				}
				else
				{
#ifdef FILAMENTCHANGE_FIRSTRETRACT
					target[E_AXIS]+= FILAMENTCHANGE_FIRSTRETRACT ;
#endif
				}
				plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

				//lift Z
				if(code_seen('Z')) 
				{
					target[Z_AXIS]+= code_value();
				}
				else
				{
#ifdef FILAMENTCHANGE_ZADD
					target[Z_AXIS]+= FILAMENTCHANGE_ZADD ;
#endif
				}
				plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

				//move xy
				if(code_seen('X')) 
				{
					target[X_AXIS]+= code_value();
				}
				else
				{
#ifdef FILAMENTCHANGE_XPOS
					target[X_AXIS]= FILAMENTCHANGE_XPOS ;
#endif
				}
				if(code_seen('Y')) 
				{
					target[Y_AXIS]= code_value();
				}
				else
				{
#ifdef FILAMENTCHANGE_YPOS
					target[Y_AXIS]= FILAMENTCHANGE_YPOS ;
#endif
				}

				plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

				if(code_seen('L'))
				{
					target[E_AXIS]+= code_value();
				}
				else
				{
#ifdef FILAMENTCHANGE_FINALRETRACT
					target[E_AXIS]+= FILAMENTCHANGE_FINALRETRACT ;
#endif
				}

				plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

				//finish moves
				st_synchronize();
				//disable extruder steppers so filament can be removed
				disable_e0();
				disable_e1();
				disable_e2();
				delay(100);
				LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);
				uint8_t cnt=0;
				while(!LCD_CLICKED){
					cnt++;
					manage_heater();
					Read_Power_Button();
					CheckTemperatureToActivateFan();
					manage_inactivity();
					lcd_update();

#if BEEPER > -1
					if(cnt==0)
					{
						SET_OUTPUT(BEEPER);

						WRITE(BEEPER,HIGH);
						delay(3);
						WRITE(BEEPER,LOW);
						delay(3);
					}
#endif
				}

				//return to normal
				if(code_seen('L')) 
				{
					target[E_AXIS]+= -code_value();
				}
				else
				{
#ifdef FILAMENTCHANGE_FINALRETRACT
					target[E_AXIS]+=(-1)*FILAMENTCHANGE_FINALRETRACT ;
#endif
				}
				current_position[E_AXIS]=target[E_AXIS]; //the long retract of L is compensated by manual filament feeding
				plan_set_e_position(current_position[E_AXIS]);
				plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //should do nothing
				plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move xy back
				plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move z back
				plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder); //final untretract
			}
			break;
#endif //FILAMENTCHANGEENABLE    
		case 907: // M907 Set digital trimpot motor current using axis codes.
			{
#if DIGIPOTSS_PIN > -1
				for(int i=0;i<=NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_current(i,code_value());
				if(code_seen('B')) digipot_current(4,code_value());
				if(code_seen('S')) for(int i=0;i<=4;i++) digipot_current(i,code_value());
#endif
			}
		case 908: // M908 Control digital trimpot directly.
			{
#if DIGIPOTSS_PIN > -1
				uint8_t channel,current;
				if(code_seen('P')) channel=code_value();
				if(code_seen('S')) current=code_value();
				digitalPotWrite(channel, current);
#endif
			}
			break;
		case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
			{
#if X_MS1_PIN > -1
				if(code_seen('S')) for(int i=0;i<=4;i++) microstep_mode(i,code_value()); 
				for(int i=0;i<=NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
				if(code_seen('B')) microstep_mode(4,code_value());
				microstep_readings();
#endif
			}
			break;
		case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
			{
#if X_MS1_PIN > -1
				if(code_seen('S')) switch((int)code_value())
				{
				case 1:
					for(int i=0;i<=NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,code_value(),-1);
					if(code_seen('B')) microstep_ms(4,code_value(),-1);
					break;
				case 2:
					for(int i=0;i<=NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,code_value());
					if(code_seen('B')) microstep_ms(4,-1,code_value());
					break;
				}
				microstep_readings();
#endif
			}
			break;
		case 999: // M999: Restart after being stopped
			Stopped = false;
			lcd_reset_alert_level();
			gcode_LastN = Stopped_gcode_LastN;
			FlushSerialRequestResend();
			break;


		case 1000:

			{
				card.closefile();
				//starpos = (strchr(name_fichier_supprimer + 4,'*'));

				//card.removeFile(name_fichier_supprimer + 4);




				break;
			}





		case 1200: 
			{// LED t?te Bleu ON
				digitalWrite(LED_blue, HIGH);
				Top_Led_State= true;


				break;
			}	


		case 1201: 
			{// LED t?te Bleu OFF

				digitalWrite(LED_blue, LOW);
				Top_Led_State= false;



				break;
			}


		case 1202: 
			{// Rampes Led sur les cot?s ON
				digitalWrite(LED_Rampes_gauche, HIGH);			
				digitalWrite(LED_Rampes_droite, HIGH);
				Strip_Led_State = true;


				break;
			}

		case 1203: 
			{// Rampes Led sur les cot?s OFF
				digitalWrite(LED_Rampes_gauche, LOW);
				digitalWrite(LED_Rampes_droite, LOW);
				Strip_Led_State = false;


				break;
			}



			//// Temperature extruder 0
		case 1300: 
			{
				numero_extruder=0;
				//MYSERIAL.write("la temperature du premier extruder est  T: ");
				SERIAL_PROTOCOL_F(degHotend(numero_extruder),1); 
				//MYSERIAL.write("\n");



				break;
			}	



			//// Temperature extruder 1
		case 1301: 
			{
				numero_extruder=1;
				// MYSERIAL.write("la temperature du second extruder est  T: ");
				SERIAL_PROTOCOL_F(degHotend(numero_extruder),1); 
				//MYSERIAL.write("\n");


				break;
			}


			/////// Activer ou d?sactiver les messages

		case 1302: 
			{


				MSG_SDCARD = true;
				MYSERIAL.write("Activation des messages re?u de la carte m?moire \n");



				break;
			}
		case 1303: 
			{


				MSG_SDCARD = false;
				MYSERIAL.write("D?sactivation des messages re?u de la carte m?moire \n");


				break;
			}




		case 1304: 
			{


				MSG_PRONTERFACE = true;
				MYSERIAL.write("Activation des messages re?u de de pronterface \n");




				break;
			}
		case 1305: 
			{


				MSG_PRONTERFACE = false;
				MYSERIAL.write("D?sactivation des messages re?u de de pronterface \n");


				break;
			}
			/////////////////////////////////////////////////////////////

			/////// Numero ligne Gcode

		case 1306: 
			{
				MYSERIAL.write("Le nombre de ligne de commande envoy? au processeur par la carte SD est : ");	 
				MYSERIAL.print(numero_ligne_sd_card);
				MYSERIAL.write("\n");



				break;
			}
		case 1307: 
			{

				MYSERIAL.write("R?initialisation du numero de ligne de la carte SD \n");
				numero_ligne_sd_card = 0; 



				break;
			}


		case 1308: 
			{
				numero_ligne_sd_card--;
				numero_extruder=0;
				MYSERIAL.write("la temperature du premier extruder est  T: ");
				SERIAL_PROTOCOL_F(degHotend(numero_extruder),1); 
				MYSERIAL.write("\n");

				numero_extruder=1;
				MYSERIAL.write("la temperature du second extruder est  T: ");
				SERIAL_PROTOCOL_F(degHotend(numero_extruder),1); 		
				MYSERIAL.write("\n");
				MYSERIAL.write("Le nombre de ligne de commande envoy? au processeur est : ");	 
				MYSERIAL.print(numero_ligne_sd_card);
				MYSERIAL.write("\n");

				break;
			}


		case 1309:
			{
				// On va les stocker ? l'adresse 4000

				//#ifdef EEPROM_SETTINGS
				//donn?es ? stocker entre -32,768 to 32,767 :
				int donnees_e_steps=32769;
				int position_esteps=1500;
				EEPROM_WRITE_VAR(position_esteps, donnees_e_steps);
				//#endif

				break;	
			}		


		case 1310:
			{
				// On va lire depuis l'adresse 4000

				//#ifdef EEPROM_SETTINGS
				int donnees_e_steps=0;
				int position_esteps=1500;
				EEPROM_READ_VAR(position_esteps, donnees_e_steps);
				//#endif

				MYSERIAL.print(position_esteps);
				MYSERIAL.print("\t");
				MYSERIAL.print(donnees_e_steps);
				MYSERIAL.println();

				break;
			}





		case 1311: 
			{
				numero_ligne_pronterface--;
				MYSERIAL.write("Le nombre de ligne de commande envoy? au processeur par pronterface est: ");	 
				MYSERIAL.print(numero_ligne_pronterface);
				MYSERIAL.write("\n");



				break;
			}
		case 1312: 
			{

				MYSERIAL.write("R?initialisation du numero de ligne de pronterface \n");
				numero_ligne_pronterface = 0; 



				break;
			}








		case 1400: //verions_marlin
			{
				SERIAL_ECHOLNPGM(VERSION_STRING);



				break;
			}


		case 1401: //affichage la temperture de l'extrudeur courant
			{

				SERIAL_PROTOCOL_F(degHotend(active_extruder),1);  

				break;
			}



		case 1402: //affichage de la temperture des deux connexions
			{

				numero_extruder=0;
				SERIAL_PROTOCOLPGM("TEMP 1:");
				SERIAL_PROTOCOL_F(degHotend(numero_extruder),1);  
				numero_extruder=1;
				SERIAL_PROTOCOLPGM(" - TEMP 2:");
				SERIAL_PROTOCOL_F(degHotend(numero_extruder),1); 



				break;
			}



			//============FIN LED multicouleur==================\\


			// FCO /////////////////
		case 1600:

			{	
				int SD_printing=0;
				if(card.sdprinting==true)
				{
					int Taux_dimpression= 	floor(numero_ligne_sd_card * 100 /numero_fichier_sd);
					if (Taux_dimpression==0)
					{

						MYSERIAL.print("1");
					}
					else
					{
						MYSERIAL.print(Taux_dimpression); 
					}

				}
				else
				{
					MYSERIAL.print(SD_printing);
				}	

				/*MYSERIAL.print("\n");
				MYSERIAL.print(numero_ligne_sd_card);
				MYSERIAL.print("\n");
				MYSERIAL.print(numero_fichier_sd);
				MYSERIAL.print("\n");*/


				break;
			}


		case 1601: 
			{// Serial2 cartridge


				SERIAL_PROTOCOLLN((int)active_extruder);  

				break;
			}	


		case 1602: 
			{// Serial2 read cartridge

				//#ifdef TEST
				//		MYSERIAL.print("5C1200FF000014C0800000646900ED05");

				//#else
				if(RFID2_14_15_OK)
				{

					////Activer l'antenne///
					if (Antenna_RFID_State== false)
					{
						//envoie_commande(Set_Antenna_Status_ON,length_Set_Antenna_Status_ON,2);
						Antenna_RFID_State= true;
					}
					//// fin de l'activation///////////	

					// Ultralight

					//envoie_commande(Initialize_Port,length_Initialize_Port,2);	
					//envoie_commande(Set_Led_Color_ON,length_Set_Led_Color_ON,2); 

					//unsigned long start = millis();

					envoie_commande(Mifare_Request,length_Mifare_Request,2); 		
					envoie_commande(Mifare_Anticollision,length_Mifare_Anticollision,2); 
					envoie_commande(Mifare_Select,length_Mifare_Select,2); 
					//envoie_commande(Mifare_Authentication24,length_Mifare_Authentication24,2); 
					envoie_commande(Mifare_Read1,length_Mifare_Read1,2); 
					envoie_commande(Mifare_Hlta,length_Mifare_Hlta,2); 

					//MYSERIAL.print("#");
					//MYSERIAL.print(millis() - start, DEC);
					//MYSERIAL.print("#");

					MYSERIAL.print("\n"); 
				}
				//#endif

				break;
			}	
		case 1603: 
			{// Serial3 read cartridge 

				//#ifdef TEST
				//		MYSERIAL.print("5C1200FFFFFF14C08000005C6200ED36");
				//#else
				if(RFID1_16_17_OK)
				{
					////Activer l'antenne///
					if (Antenna_RFID_State== false)
					{						
						//envoie_commande(Set_Antenna_Status_ON,length_Set_Antenna_Status_ON,3); 						
						Antenna_RFID_State= true;
					}		
					//// fin de l'activation///////////	


					// Ultralight

					//envoie_commande(Initialize_Port,length_Initialize_Port,3);	
					//envoie_commande(Set_Led_Color_ON,length_Set_Led_Color_ON,3); 
					envoie_commande(Mifare_Request,length_Mifare_Request,3); 		
					envoie_commande(Mifare_Anticollision,length_Mifare_Anticollision,3); 
					envoie_commande(Mifare_Select,length_Mifare_Select,3); 
					//envoie_commande(Mifare_Authentication24,length_Mifare_Authentication24,2); 
					envoie_commande(Mifare_Read1,length_Mifare_Read1,3);
					envoie_commande(Mifare_Hlta,length_Mifare_Hlta,3); 
					MYSERIAL.print("\n");
				}
				//#endif

				break;
			}
		case 1604: 
			{ // loading right cartridge
				uint8_t sauvegarde_extruder = active_extruder;
				float fr_critical = feedrate_Critical_Loading;
				float fr_default = feedrate_Loading;
				
				if (code_seen(CODE_PVA_IN_LOADING_UNLOADING)) {
					fr_critical = feedrate_Critical_Loading_PVA;
					fr_default = feedrate_Loading_PVA;
				}
				
				active_extruder=0;
				allow_cold_extrudes(true);
				//Commande_cartridge=true;
				relative_mode = true;

				//First mouvement
				destination[3] =  Distance_Critical_Loading + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2];
				feedrate = fr_critical;
				prepare_move();

				// Second mouvement
				destination[3] =  Distance_Loading_right + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2];
				feedrate = fr_default;
				prepare_move();

				//Réinitilisation
				Distance_Filament_E0 = 0;

				active_extruder=sauvegarde_extruder;
				relative_mode = false;

				break;
			}

		case 1605: 
			{ // loading left cartridge
				uint8_t sauvegarde_extruder = active_extruder;
				float fr_critical = feedrate_Critical_Loading;
				float fr_default = feedrate_Loading;
				
				if (code_seen(CODE_PVA_IN_LOADING_UNLOADING)) {
					fr_critical = feedrate_Critical_Loading_PVA;
					fr_default = feedrate_Loading_PVA;
				}
				
				active_extruder=1;
				allow_cold_extrudes(true);
				//Commande_cartridge=true;
				relative_mode = true;

				//First mouvement
				destination[3] =  Distance_Critical_Loading + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2];
				feedrate = fr_critical;
				prepare_move();

				// Second mouvement
				destination[3] =  Distance_Loading_left + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2];
				feedrate = fr_default;
				prepare_move();

				//Réinitilisation
				Distance_Filament_E1 = 0;

				active_extruder=sauvegarde_extruder;
				relative_mode = false;

				break;
			}

		case 1606: 
			{ // unloading right cartridge
				uint8_t orignal_extruder = active_extruder;
				float fr_p1 = Unloading_part1_feedrate;
				float fr_p2 = Unloading_part2_feedrate;
				float fr_p3 = Unloading_part3_feedrate;
				
				if (code_seen(CODE_PVA_IN_LOADING_UNLOADING)) {
					fr_p1 = Unloading_part1_feedrate_PVA;
					fr_p2 = Unloading_part2_feedrate_PVA;
					fr_p3 = Unloading_part3_feedrate_PVA;
				}

				active_extruder=0;
				allow_cold_extrudes(true);
				relative_mode = true;
				unloading_command = true;

				destination[3] = -Unloading_part1_length + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2];
				feedrate = fr_p1;
				prepare_move();

				destination[3] = -Unloading_part2_length_right + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2];
				feedrate = fr_p2;
				prepare_move();

				destination[3] = -Unloading_part3_length + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2];
				feedrate = fr_p3;
				prepare_move();

				relative_mode = false;
				active_extruder = orignal_extruder;

				break;
			}

		case 1607: 
			{ // unloading left cartridge
				uint8_t orignal_extruder = active_extruder;
				float fr_p1 = Unloading_part1_feedrate;
				float fr_p2 = Unloading_part2_feedrate;
				float fr_p3 = Unloading_part3_feedrate;
				
				if (code_seen(CODE_PVA_IN_LOADING_UNLOADING)) {
					fr_p1 = Unloading_part1_feedrate_PVA;
					fr_p2 = Unloading_part2_feedrate_PVA;
					fr_p3 = Unloading_part3_feedrate_PVA;
				}

				active_extruder=1;
				allow_cold_extrudes(true);
				relative_mode = true;
				unloading_command = true;

				destination[3] = -Unloading_part1_length + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2]; 
				feedrate = fr_p1;
				prepare_move();

				destination[3] = -Unloading_part2_length_left + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2]; 
				feedrate = fr_p2;
				prepare_move();

				destination[3] = -Unloading_part3_length + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2]; 
				feedrate = fr_p3;
				prepare_move();

				relative_mode = false;
				active_extruder = orignal_extruder;

				break;
			}

		case 1608: 
			{// Serial3 cartridge
				//int FCOOO=0;
#if (PRIVATE_ENDSTOPS1 > -1)
				SERIAL_PROTOCOLLN(((READ(PRIVATE_ENDSTOPS1)^E_ENDSTOPS_INVERTING)?"filament":"no filament"));
#endif





				break;
			}

		case 1609: 
			{// Serial3 cartridge
#if (PRIVATE_ENDSTOPS1 > -1)
				SERIAL_PROTOCOLLN(((READ(PRIVATE_ENDSTOPS2)^E_ENDSTOPS_INVERTING)?"filament":"no filament"));
#endif



				break;
			}


		case 1610 : {	// Read right RFID

			int Tag_ecriture_int[38];
			byte Tag_ecriture_final[16]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};						
			byte commande_ecriture_tag[]={0x0a,0x00,0x00,0x00,0x13,0x02,0x06,0x00,0x00,0x00,0x00};
			byte length_commande_ecriture_tag = sizeof(commande_ecriture_tag);

			for (int i=6; i<38;i++)
			{
				if(bufindw==0)
					Tag_ecriture_int[i] = int(cmdbuffer[3][i]);
				else
					Tag_ecriture_int[i] = int(cmdbuffer[bufindw-1][i]);

				if(Tag_ecriture_int[i]>=48 && Tag_ecriture_int[i]<=57)
					Tag_ecriture_int[i] = Tag_ecriture_int[i]- 48;
				if(Tag_ecriture_int[i]>=65 && Tag_ecriture_int[i]<=70)
					Tag_ecriture_int[i] = Tag_ecriture_int[i] - 55;
			}

			SERIAL_PROTOCOLLN("\n");

			if(RFID2_14_15_OK) {
				for (int i=0; i<32; i=i+2) 
					Tag_ecriture_final[i/2] = byte((Tag_ecriture_int[i+6]*16) + Tag_ecriture_int[i+7]);	

				////Activer l'antenne///
				if (Antenna_RFID_State== false)
				{						
					//envoie_commande(Set_Antenna_Status_ON,length_Set_Antenna_Status_ON,2); 						
					Antenna_RFID_State= true;
				}		
				//// fin de l'activation///////////	

				//unsigned long start = millis();

				envoie_commande(Mifare_Request,length_Mifare_Request,2); 		
				envoie_commande(Mifare_Anticollision,length_Mifare_Anticollision,2); 
				envoie_commande(Mifare_Select,length_Mifare_Select,2); 
				envoie_commande(Mifare_Read1,length_Mifare_Read1,2);

				for (int i=0; i<4;i++)
				{
					commande_ecriture_tag[i+7]=Tag_ecriture_final[i];
				}	

				envoie_commande(commande_ecriture_tag,length_commande_ecriture_tag,2);  

				commande_ecriture_tag[6]=0x07;

				for (int i=4; i<8;i++)
					commande_ecriture_tag[i+3]=Tag_ecriture_final[i];

				envoie_commande(commande_ecriture_tag,length_commande_ecriture_tag,2); 

				commande_ecriture_tag[6]=0x08;

				for (int i=8; i<12;i++)
					commande_ecriture_tag[i-1]=Tag_ecriture_final[i]; 

				envoie_commande(commande_ecriture_tag,length_commande_ecriture_tag,2); 

				commande_ecriture_tag[6]=0x09;

				for (int i=12; i<16;i++)
					commande_ecriture_tag[i-5]=Tag_ecriture_final[i];

				envoie_commande(commande_ecriture_tag,length_commande_ecriture_tag,2); 
				envoie_commande(Mifare_Hlta,length_Mifare_Hlta,2); 
			}
			break;
					}
		case 1611 : {	// Read left RFID

			int Tag_ecriture_int[38];
			byte Tag_ecriture_final[16]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};						
			byte commande_ecriture_tag[]={0x0a,0x00,0x00,0x00,0x13,0x02,0x06,0x00,0x00,0x00,0x00};
			byte length_commande_ecriture_tag = sizeof(commande_ecriture_tag);

			for (int i=6; i<38;i++)
			{
				if(bufindw==0)
					Tag_ecriture_int[i] = int(cmdbuffer[3][i]);
				else
					Tag_ecriture_int[i] = int(cmdbuffer[bufindw-1][i]);

				if(Tag_ecriture_int[i]>=48 && Tag_ecriture_int[i]<=57)
					Tag_ecriture_int[i] = Tag_ecriture_int[i]- 48;
				if(Tag_ecriture_int[i]>=65 && Tag_ecriture_int[i]<=70)
					Tag_ecriture_int[i] = Tag_ecriture_int[i] - 55;
			}

			SERIAL_PROTOCOLLN("\n");

			if(RFID1_16_17_OK) {
				for (int i=0; i<32; i=i+2) 
					Tag_ecriture_final[i/2] = byte((Tag_ecriture_int[i+6]*16) + Tag_ecriture_int[i+7]);	

				////Activer l'antenne///
				if (Antenna_RFID_State== false)
				{						
					//envoie_commande(Set_Antenna_Status_ON,length_Set_Antenna_Status_ON,3); 						
					Antenna_RFID_State= true;
				}		
				//// fin de l'activation///////////	

				envoie_commande(Mifare_Request,length_Mifare_Request,3); 		
				envoie_commande(Mifare_Anticollision,length_Mifare_Anticollision,3); 
				envoie_commande(Mifare_Select,length_Mifare_Select,3); 
				envoie_commande(Mifare_Read1,length_Mifare_Read1,3);

				for (int i=0; i<4;i++)
				{
					commande_ecriture_tag[i+7]=Tag_ecriture_final[i];
				}	

				envoie_commande(commande_ecriture_tag,length_commande_ecriture_tag,3);  

				commande_ecriture_tag[6]=0x07;

				for (int i=4; i<8;i++)
					commande_ecriture_tag[i+3]=Tag_ecriture_final[i];

				envoie_commande(commande_ecriture_tag,length_commande_ecriture_tag,3); 

				commande_ecriture_tag[6]=0x08;

				for (int i=8; i<12;i++)
					commande_ecriture_tag[i-1]=Tag_ecriture_final[i];

				envoie_commande(commande_ecriture_tag,length_commande_ecriture_tag,3); 

				commande_ecriture_tag[6]=0x09;

				for (int i=12; i<16;i++)
					commande_ecriture_tag[i-5]=Tag_ecriture_final[i];

				envoie_commande(commande_ecriture_tag,length_commande_ecriture_tag,3); 
				envoie_commande(Mifare_Hlta,length_Mifare_Hlta,3); 
			}
			break;
					}
		case 1612: // loading Right Filament which takes as parameter the distance of loading
			{

				float Distance_Load=0;
				if(code_seen('D')) {
					Distance_Load = code_value();}


				uint8_t sauvegarde_extruder = active_extruder;
				active_extruder=0; 		
				allow_cold_extrudes(true);
				//Commande_cartridge=true;
				relative_mode = true;

				destination[3] =  Distance_Critical_Loading + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2];
				feedrate = feedrate_Critical_Loading;
				prepare_move();




				// Second mouvement
				destination[3] =  Distance_Load - Distance_Critical_Loading + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2];
				feedrate = feedrate_Loading; 
				prepare_move(); 


				active_extruder=sauvegarde_extruder;
				relative_mode = false;




				break;
			}






		case 1613: 
			{// loading Left Filament which takes as parameter the distance of loading
				float Distance_Load=0;
				if(code_seen('D')) {
					Distance_Load = code_value();}




				uint8_t sauvegarde_extruder = active_extruder;
				active_extruder=1; 		
				allow_cold_extrudes(true);
				//Commande_cartridge=true;
				relative_mode = true;

				destination[3] =  Distance_Critical_Loading + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2];
				feedrate = feedrate_Critical_Loading;
				prepare_move();



				// Second mouvement
				destination[3] =  Distance_Load - Distance_Critical_Loading + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2];
				feedrate = feedrate_Loading;
				prepare_move(); 


				active_extruder=sauvegarde_extruder;
				relative_mode = false;






				break;
			}



		case 1614: 
			{// Get Strip_Led state 			
				if (Strip_Led_State== true)
					MYSERIAL.print("1"); 
				else
					MYSERIAL.print("0"); 


				break;
			}


		case 1615: 
			{// Get Top_Led state 			

				if (Top_Led_State== true)
					MYSERIAL.print("1"); 
				else
					MYSERIAL.print("0"); 

				break;
			}


		case 1616: 
			{// Enable antenna's RFID			


				//envoie_commande(Initialize_Port,length_Initialize_Port,2);	
				//envoie_commande(Set_Antenna_Status_ON,length_Set_Antenna_Status_ON,2); 	
				//envoie_commande(Initialize_Port,length_Initialize_Port,3);	
				//envoie_commande(Set_Antenna_Status_ON,length_Set_Antenna_Status_ON,3); 	
				Antenna_RFID_State= true;
				MYSERIAL.print("\n");

				break;





			}



		case 1617: 
			{// Disable antenna's RFID		


				//envoie_commande(Initialize_Port,length_Initialize_Port,2);	
				envoie_commande(Set_Antenna_Status_OFF,length_Set_Antenna_Status_OFF,2); 	
				//envoie_commande(Initialize_Port,length_Initialize_Port,3);	
				envoie_commande(Set_Antenna_Status_OFF,length_Set_Antenna_Status_OFF,3); 
				Antenna_RFID_State= false;	
				MYSERIAL.print("\n");

				break;


			}


		case 1618: 
			{// Get Antenna_RFID 
				//#ifdef TEST
				//					MYSERIAL.print("1");
				//#else
				if (Antenna_RFID_State== true)
					MYSERIAL.print("1"); 
				else
					MYSERIAL.print("0"); 
				//#endif
				break;
			}





			// Fontions vitesse Peng



		case 1620: 
			{// Obtention de la vitesse courante			
				MYSERIAL.print(feedrate);

				break;
			}



		case 1621: 
			{
				// D?finir la vitesse courante

				if(code_seen('V')) {
					feedrate = code_value();}

				break;

			}



		case 1622: 
			{

				// Etat Cold Extrusion	
				bool Etat = Etat_Cold_Extrusion();

				MYSERIAL.print(Etat);

				break;        

			}



		case 1623: 
			{// Obtenir l'acceleration courante


				MYSERIAL.print(acceleration);

				break;
			}




		case 1624: 
			{// D?finir l'acceleration courante

				if(code_seen('A')) {
					acceleration = code_value();}

				break;
			}


		//case 1625:
		//	{


		//		// loading Right RFID PVA


		//		uint8_t sauvegarde_extruder = active_extruder;
		//		active_extruder=0; 		
		//		allow_cold_extrudes(true);
		//		Commande_cartridge=true;
		//		relative_mode = true;


		//		//First mouvement
		//		destination[3] =  Distance_Critical_Loading + (axis_relative_modes[3] || relative_mode)*current_position[3];
		//		destination[0] = current_position[0];
		//		destination[1] = current_position[1];
		//		destination[2] = current_position[2];
		//		feedrate = 100;
		//		prepare_move();



		//		// Second mouvement
		//		destination[3] =  Distance_Loading_right + (axis_relative_modes[3] || relative_mode)*current_position[3];
		//		destination[0] = current_position[0];
		//		destination[1] = current_position[1];
		//		destination[2] = current_position[2];
		//		feedrate = 500;
		//		prepare_move();


		//		//Réinitilisation
		//		Distance_Filament_E0 = 0;



		//		active_extruder=sauvegarde_extruder;
		//		relative_mode = false;




		//		break;






		//	}


//		case 1626: 
//			{
//				//Unloading RIGHT PVA
//
//				active_extruder=0;
//				allow_cold_extrudes(true);
//				relative_mode = true;
//
//				destination[3] = -Unloading_part1_length + (axis_relative_modes[3] || relative_mode)*current_position[3];
//				destination[0] = current_position[0];
//				destination[1] = current_position[1];
//				destination[2] = current_position[2];
//				feedrate = Unloading_part1_feedrate_PVA;
//				prepare_move();
//
//				Message_wait();
//
//				destination[3] = -Unloading_part2_length_right + (axis_relative_modes[3] || relative_mode)*current_position[3];
//				destination[0] = current_position[0];
//				destination[1] = current_position[1];
//				destination[2] = current_position[2];
//				feedrate = Unloading_part2_feedrate_PVA;
//				prepare_move();
//
//				Message_wait();
//
//				unloading_command = true;
//
//				destination[3] = -Unloading_part3_length + (axis_relative_modes[3] || relative_mode)*current_position[3];
//				destination[0] = current_position[0];
//				destination[1] = current_position[1];
//				destination[2] = current_position[2];
//				feedrate = Unloading_part3_feedrate_PVA;
//				prepare_move();
//
//				relative_mode = false;
//
//				break;
//			}

		//case 1626:
		//	{

		//		// loading left RFID PVA


		//		uint8_t sauvegarde_extruder = active_extruder;
		//		active_extruder=1; 		
		//		allow_cold_extrudes(true);
		//		Commande_cartridge=true;
		//		relative_mode = true;


		//		//First mouvement
		//		destination[3] =  Distance_Critical_Loading + (axis_relative_modes[3] || relative_mode)*current_position[3];
		//		destination[0] = current_position[0];
		//		destination[1] = current_position[1];
		//		destination[2] = current_position[2];
		//		feedrate = 100;
		//		prepare_move();



		//		// Second mouvement
		//		destination[3] =  Distance_Loading_right + (axis_relative_modes[3] || relative_mode)*current_position[3];
		//		destination[0] = current_position[0];
		//		destination[1] = current_position[1];
		//		destination[2] = current_position[2];
		//		feedrate = 500;
		//		prepare_move();


		//		//Réinitilisation
		//		Distance_Filament_E1 = 0;



		//		active_extruder=sauvegarde_extruder;
		//		relative_mode = false;




		//		break;





		//	}	

//		case 1627: 
//			{ 
//				//Unloading LEFT PVA
//
//				//if ((READ(PRIVATE_ENDSTOPS2)^E_ENDSTOPS_INVERTING) == false) {
//				//	break; // block unloading if no filament
//				//}
//
//				active_extruder=1;
//				allow_cold_extrudes(true);
//				relative_mode = true;
//
//				destination[3] = -Unloading_part1_length + (axis_relative_modes[3] || relative_mode)*current_position[3];
//				destination[0] = current_position[0];
//				destination[1] = current_position[1];
//				destination[2] = current_position[2]; 
//				feedrate = Unloading_part1_feedrate_PVA;
//				prepare_move();
//
//				Message_wait();
//
//				destination[3] = -Unloading_part2_length_left + (axis_relative_modes[3] || relative_mode)*current_position[3];
//				destination[0] = current_position[0];
//				destination[1] = current_position[1];
//				destination[2] = current_position[2]; 
//				feedrate = Unloading_part2_feedrate_PVA;  
//				prepare_move();
//
//				Message_wait();
//
//				unloading_command = true;
//
//				destination[3] = -Unloading_part3_length + (axis_relative_modes[3] || relative_mode)*current_position[3];
//				destination[0] = current_position[0];
//				destination[1] = current_position[1];
//				destination[2] = current_position[2]; 
//				feedrate = Unloading_part3_feedrate_PVA;  
//				prepare_move();
//
//				relative_mode = false;
//
//				break;
//			}


		case 1650: 
			{// Extruding 20mm for the first extruder
				float Distance_Load=4;
				uint8_t sauvegarde_extruder = active_extruder;
				float fr_charge = feedrate_Extrude_Retract;
				
				active_extruder=0;
				relative_mode = true;
				if(code_seen(CODE_PVA_IN_LOADING_UNLOADING)) {
					fr_charge = feedrate_Extrude_PVA;
				}

				destination[3] =  Distance_Load + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2];
				feedrate = fr_charge; 
				prepare_move();

				active_extruder=sauvegarde_extruder;
				relative_mode = false;

				break;
			}


		case 1651: 
			{// Extruding 20mm for the second extruder
				float Distance_Load=4;
				uint8_t sauvegarde_extruder = active_extruder;
				float fr_charge = feedrate_Extrude_Retract;
				
				active_extruder=1;
				relative_mode = true;
				if(code_seen(CODE_PVA_IN_LOADING_UNLOADING)) {
					fr_charge = feedrate_Extrude_PVA;
				}

				destination[3] =  Distance_Load + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2];
				feedrate = fr_charge;
				prepare_move();

				active_extruder=sauvegarde_extruder;
				relative_mode = false;

				break;
			}


		case 1652: 
			{// Reversing 20mm for the first extruder, not in use in real
				float Distance_Load=4;
				uint8_t sauvegarde_extruder = active_extruder;
				float fr_retract = feedrate_Extrude_Retract;
				
				active_extruder=0;
				relative_mode = true;
				if(code_seen(CODE_PVA_IN_LOADING_UNLOADING)) {
					fr_retract = feedrate_Extrude_PVA;
				}

				destination[3] = - Distance_Load + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2];
				feedrate = fr_retract;
				prepare_move();

				active_extruder=sauvegarde_extruder;
				relative_mode = false;

				break;
			}

		case 1653: 
			{// Reversing 20mm for the second extruder, not in use in real
				float Distance_Load=4;
				uint8_t sauvegarde_extruder = active_extruder;
				float fr_retract = feedrate_Extrude_Retract;
				
				active_extruder=1;
				relative_mode = true;
				if(code_seen(CODE_PVA_IN_LOADING_UNLOADING)) {
					fr_retract = feedrate_Extrude_PVA;
				}

				destination[3] = - Distance_Load + (axis_relative_modes[3] || relative_mode)*current_position[3];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[2] = current_position[2];
				feedrate = fr_retract; 
				prepare_move();

				active_extruder=sauvegarde_extruder;
				relative_mode = false;

				break;
			}











		case 1660: 
			{// Reversing 20mm for the second extruder
				//#ifdef EEPROM_SETTINGS
				//donn?es ? stocker entre -100 to 100 :


				float offset_x_code_value=0;
				float offset_y_code_value=0;

				int position_offset_x_write_e2prom=1500;
				int position_offset_y_write_e2prom=1600;
				int change_offset_x = 0;
				int change_offset_y = 0;

				if(code_seen('X')||code_seen('x')) {
					offset_x_code_value = code_value();
					change_offset_x = 1;
				}

				if(code_seen('Y')||code_seen('y')) {
					offset_y_code_value = code_value();
					change_offset_y = 1;
				}

				if((offset_x_code_value>100)||(offset_x_code_value<-100)||(offset_y_code_value>100)||(offset_y_code_value<-100)) {
					MYSERIAL.print("\n Error!! Invalid Input!!!!");
				}
				else if (change_offset_x + change_offset_y == 0) {
					MYSERIAL.print("\n Error!! No Input!!!!");
				}
				else
				{
					if (change_offset_x == 1) {
						if(offset_x_code_value<0) {
							offset_x_code_value *=(-1);
							offset_x_code_value += 100;
						}

						int offset_x_write_e2prom=offset_x_code_value;
						EEPROM_WRITE_VAR(position_offset_x_write_e2prom, offset_x_write_e2prom);

						MYSERIAL.print("\n X=");
						MYSERIAL.print(offset_x_write_e2prom);
					}

					if (change_offset_y == 1) {
						if(offset_y_code_value<0) {
							offset_y_code_value *=(-1);
							offset_y_code_value += 100;
						}

						int offset_y_write_e2prom=offset_y_code_value;
						EEPROM_WRITE_VAR(position_offset_y_write_e2prom, offset_y_write_e2prom);

						MYSERIAL.print("\n Y=");
						MYSERIAL.print(offset_y_write_e2prom);
					}

					// return to original values for offset for preparing changing
					float extruder_offset_x_ori[2] = EXTRUDER_OFFSET_X;
					float extruder_offset_y_ori[2] = EXTRUDER_OFFSET_Y;
					extruder_offset[X_AXIS][1] = extruder_offset_x_ori[1];
					extruder_offset[Y_AXIS][1] = extruder_offset_y_ori[1];
					Lecture_offset_E2PROM();
				}

				break;
			}






		case 1661:
			{
				//#ifdef EEPROM_SETTINGS
				int offset_x_e2prom2=0;
				int position_offset_x_e2prom2=1500;
				EEPROM_READ_VAR(position_offset_x_e2prom2, offset_x_e2prom2);	





				if((offset_x_e2prom2<0))
					offset_x_e2prom2=0;
				else
				{

					if(offset_x_e2prom2>100)
						offset_x_e2prom2=100-offset_x_e2prom2;

				}



				MYSERIAL.print(offset_x_e2prom2);


				break;	
			}						


		case 1662:
			{



				int offset_y_e2prom2=0;
				int position_offset_y_e2prom2=1600;
				EEPROM_READ_VAR(position_offset_y_e2prom2, offset_y_e2prom2);	





				if(offset_y_e2prom2<0)
					offset_y_e2prom2=0;
				else
				{

					if(offset_y_e2prom2>100)
						offset_y_e2prom2=100-offset_y_e2prom2;
				}


				MYSERIAL.print(offset_y_e2prom2);


				break;	
			}	


		case 1702: 
			{// Serial2 read cartridge

				////Activer l'antenne///
				if (Antenna_RFID_State== false)
				{						
					//envoie_commande(Set_Antenna_Status_ON,length_Set_Antenna_Status_ON,2); 						
					Antenna_RFID_State= true;
				}		
				//// fin de l'activation///////////	

				// Ultralight

				//envoie_commande(Initialize_Port,length_Initialize_Port,2);	
				//envoie_commande(Set_Led_Color_ON,length_Set_Led_Color_ON,2); 
				//envoie_commande(Mifare_Request,length_Mifare_Request,2); 		
				//envoie_commande(Mifare_Anticollision,length_Mifare_Anticollision,2); 
				//envoie_commande(Mifare_Select,length_Mifare_Select,2); 
				//envoie_commande(Mifare_Authentication24,length_Mifare_Authentication24,2); 
				envoie_commande(Mifare_Read1,length_Mifare_Read1,2); 
				MYSERIAL.print("\n"); 



				break;
			}	
		case 1703: 
			{// Serial3 read cartridge 

				////Activer l'antenne///
				if (Antenna_RFID_State== false)
				{						
					//envoie_commande(Set_Antenna_Status_ON,length_Set_Antenna_Status_ON,3); 						
					Antenna_RFID_State= true;
				}		
				//// fin de l'activation///////////	


				// Ultralight

				//envoie_commande(Initialize_Port,length_Initialize_Port,3);	
				//envoie_commande(Set_Led_Color_ON,length_Set_Led_Color_ON,3); 
				//envoie_commande(Mifare_Request,length_Mifare_Request,3); 		
				//envoie_commande(Mifare_Anticollision,length_Mifare_Anticollision,3); 
				//envoie_commande(Mifare_Select,length_Mifare_Select,3); 
				//envoie_commande(Mifare_Authentication24,length_Mifare_Authentication24,2); 
				envoie_commande(Mifare_Read1,length_Mifare_Read1,3);
				MYSERIAL.print("\n");




				break;
			}


		case 1900: 
			{// Interruption of programme
				quickStop(); 
				Message_wait();			

				/////////////////////ici je rajoute des parties de d?veloppement/////////////////////////

				Moving_Z_1();
				//delay(15000);
				for (int i = 0; i < 15; i++) {
					if (READ(Z_MAX_PIN)^Z_ENDSTOPS_INVERTING) {
						break;
					}
					delay(1000);
				}
				Message_wait();
				ecrir_sur_RFID1();
				ecrir_sur_RFID2();
				Reverse_20mm();
				//					if (Distance_Filament_E0 > 0) delay(10800);
				//					if (Distance_Filament_E1 > 0) delay(10800);
				Message_wait();
				st_synchronize();
				Message_wait();	 
				Reinitialise_E0_E1_Quantity();
				Message_wait();
				Moving_X_Y();	
				Message_wait();	
				Set_temperature_to_0Degree();
				Message_wait();			

				///////////////////////////////////////ici fin /////////////////////////////////////////////////

				disable_e0();
				disable_e1();
				disable_e2();
				fanSpeed = 0;





				break;
			}			

		case 1901: 
			{// Flush
				quickStop2();

				break;
			}	


		case 1902: 
			{// Pause
				// disable the write of RFID to make it stable (so also the reinitialisation)
				saving_current_parameters();
				Message_wait();
				Moving_Z_1();
				//delay(15000);
				for (int i = 0; i < 15; i++) {
					if (READ(Z_MAX_PIN)^Z_ENDSTOPS_INVERTING) {
						break;
					}
					delay(1000);
				}
				Message_wait();
				//ecrir_sur_RFID();
				Message_wait();
				Reverse_20mm();
				//					SERIAL_PROTOCOL_F(Distance_Filament_E0,1);
				//					SERIAL_PROTOCOL_F(Distance_Filament_E1,1);
				//					if (Distance_Filament_E0 > 0) delay(10800);
				//					if (Distance_Filament_E1 > 0) delay(10800);
				Message_wait();
				st_synchronize();
				//					delay(18000);
				Message_wait();
				//Reinitialise_E0_E1_Quantity();
				Message_wait();
				Moving_X_Y();
				Message_wait();
				return_current_parameters();
				Message_wait();
				Set_temperature_to_0Degree();
				Message_wait();

				disable_e0();
				disable_e1();
				disable_e2();

				break;
			}	

		case 1903 : 
			{
				// ranged the alignment and fixed bugs (by Peng on 20140708)
				// - Always extrude 2 filaments even when printing in 1 color
				// - Reset target temperatures in resume
				// - Improve to heat 2 nozzle in the same time when printing in 2 colors

				Set_current_paramters_2();

				uint8_t sauvegarde_extruder = tmp_extruder;
				setTargetHotend(Pause_current_position[T0_parm], 0);
				TMP0_Target = Pause_current_position[T0_parm];
				setTargetHotend(Pause_current_position[T1_parm], 1);
				TMP1_Target = Pause_current_position[T1_parm];

				// For T0
				tmp_extruder=0;

				if (Pause_current_position[T0_parm] > 0)
				{
					LCD_MESSAGEPGM(MSG_HEATING);
#ifdef AUTOTEMP
					autotemp_enabled=false;
#endif
#ifdef AUTOTEMP
					autotemp_min=Pause_current_position[T0_parm];
					if (code_seen('B')) autotemp_max=code_value();
					if (code_seen('F')) 
					{
						autotemp_factor=code_value();
						autotemp_enabled=true;
					}
#endif

					setWatch();
					codenum = millis(); 

					/* See if we are heating up or cooling down */
					bool target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling

#ifdef TEMP_RESIDENCY_TIME
					long residencyStart;
					residencyStart = -1;
					/* continue to loop until we have reached the target temp   
					_and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
					while((residencyStart == -1) ||
						(residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL))) ) {
#else
					while ( target_direction ? (isHeatingHotend(tmp_extruder)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) ) {
#endif //TEMP_RESIDENCY_TIME
						if( (millis() - codenum) > 1000UL )
						{ //Print Temp Reading and remaining time every 1 second while heating up/cooling down
							SERIAL_PROTOCOLPGM("T:");
							SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1); 
							SERIAL_PROTOCOLPGM(" E:");
							SERIAL_PROTOCOL((int)tmp_extruder); 
#ifdef TEMP_RESIDENCY_TIME
							SERIAL_PROTOCOLPGM(" W:");
							if(residencyStart > -1)
							{
								codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
								SERIAL_PROTOCOLLN( codenum );
							}
							else 
							{
								SERIAL_PROTOCOLLN( "?" );
							}
#else
							SERIAL_PROTOCOLLN("");
#endif
							codenum = millis();
						}
						manage_heater();
						Read_Power_Button();
						CheckTemperatureToActivateFan();
						manage_inactivity();
						lcd_update();
#ifdef TEMP_RESIDENCY_TIME
						/* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
						or when current temp falls outside the hysteresis after target temp was reached */
						if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
							(residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
							(residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) ) 
						{
							residencyStart = millis();
						}
#endif //TEMP_RESIDENCY_TIME
					}
					LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
					starttime=millis();
					previous_millis_cmd = millis();

					Extrude_20mm_first();
					delay(1200);
				}
				tmp_extruder = sauvegarde_extruder;



				// For T1
				tmp_extruder=1;

				if (Pause_current_position[T1_parm] > 0)
				{
					LCD_MESSAGEPGM(MSG_HEATING);   
#ifdef AUTOTEMP
					autotemp_enabled=false;
#endif
#ifdef AUTOTEMP
					autotemp_min=Pause_current_position[T1_parm];
					if (code_seen('B')) autotemp_max=code_value();
					if (code_seen('F')) 
					{
						autotemp_factor=code_value();
						autotemp_enabled=true;
					}
#endif

					setWatch();
					codenum = millis(); 

					/* See if we are heating up or cooling down */
					bool target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling

#ifdef TEMP_RESIDENCY_TIME
					long residencyStart;
					residencyStart = -1;
					/* continue to loop until we have reached the target temp   
					_and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
					while((residencyStart == -1) ||
						(residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL))) ) {
#else
					while ( target_direction ? (isHeatingHotend(tmp_extruder)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) ) {
#endif //TEMP_RESIDENCY_TIME
						if( (millis() - codenum) > 1000UL )
						{ //Print Temp Reading and remaining time every 1 second while heating up/cooling down
							SERIAL_PROTOCOLPGM("T:");
							SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1); 
							SERIAL_PROTOCOLPGM(" E:");
							SERIAL_PROTOCOL((int)tmp_extruder); 
#ifdef TEMP_RESIDENCY_TIME
							SERIAL_PROTOCOLPGM(" W:");
							if(residencyStart > -1)
							{
								codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
								SERIAL_PROTOCOLLN( codenum );
							}
							else 
							{
								SERIAL_PROTOCOLLN( "?" );
							}
#else
							SERIAL_PROTOCOLLN("");
#endif
							codenum = millis();
						}
						manage_heater();
						Read_Power_Button();
						CheckTemperatureToActivateFan();
						manage_inactivity();
						lcd_update();
#ifdef TEMP_RESIDENCY_TIME
						/* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
						or when current temp falls outside the hysteresis after target temp was reached */
						if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
							(residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
							(residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) ) 
						{
							residencyStart = millis();
						}
#endif //TEMP_RESIDENCY_TIME
					}
					LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
					starttime=millis();
					previous_millis_cmd = millis();

					Extrude_20mm_second();
					delay(1200);
				}
				tmp_extruder = sauvegarde_extruder;

				break;
			}



		case 1904: 
			{// Motionless end.
				ecrir_sur_RFID1();
				ecrir_sur_RFID2();
				Reverse_20mm();
				//					if (Distance_Filament_E0 > 0) delay(10800);
				//					if (Distance_Filament_E1 > 0) delay(10800);
				Message_wait();
				st_synchronize();
				Message_wait();
				Reinitialise_E0_E1_Quantity();
				Message_wait();
				//	Set_temperature_to_0Degree();
				//	Message_wait();
				disable_e0();
				disable_e1();
				disable_e2();
				fanSpeed = 0;
				break;
			}
		case 1905: 
			{// Plateform rise.



				float Distance_Z_Cartridge=250;	
				relative_mode = true;
				float feedrate_Z_cartridge = 500;

				destination[2] = Distance_Z_Cartridge + (axis_relative_modes[2] || relative_mode)*current_position[2];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[3] = current_position[3];
				feedrate = feedrate_Z_cartridge; 
				prepare_move(); 
				delay(1000);


				//Second mouvement

				Distance_Z_Cartridge=70;
				destination[2] = -Distance_Z_Cartridge + (axis_relative_modes[2] || relative_mode)*current_position[2];
				destination[0] = current_position[0];
				destination[1] = current_position[1];
				destination[3] = current_position[3];
				feedrate = feedrate_Z_cartridge; 
				prepare_move(); 
				delay(1000);


				relative_mode = false;

				break;
			}
		case 2000: 
			{// Begin printing
				Reinitialise_E0_E1_Quantity();
				// If only one extruder is needed
				Set_temperature_to_0Degree();
				// reset extruder to avoid wrong temperature assignment in mono-color model - Peng
				active_extruder=0;
				disable_x();
				disable_y();
				disable_z();
				disable_e0();
				disable_e1();
				disable_e2();

				break;
			}
		case 2001: 
			{// End


				Moving_Z_2();
				//delay(30000);
				for (int i = 0; i < 30; i++) {
					if (READ(Z_MAX_PIN)^Z_ENDSTOPS_INVERTING) {
						break;
					}
					delay(1000); 
				}
				Message_wait();
				ecrir_sur_RFID1(); 
				ecrir_sur_RFID2();
				Reverse_20mm();
				//					if (Distance_Filament_E0 > 0) delay(10800);
				//					if (Distance_Filament_E1 > 0) delay(10800);
				Message_wait();
				st_synchronize();
				Message_wait();
				Reinitialise_E0_E1_Quantity();
				Message_wait();
				Moving_X_Y();
				Message_wait();
				Set_temperature_to_0Degree();
				Message_wait();
				disable_e0();
				disable_e1();
				disable_e2();
				fanSpeed = 0;

				break;
			}
		case 2002: 
			{// kill function

				kill_Zim();

				break;
			}

			/*
			case 2010:
			{// Desactiver l'interruption
			Desactiver_interruption=true;
			}
			case 2011:
			{//Activer l'interruption
			Desactiver_interruption=false;
			}
			*/

		case 9000: 
			{// Voyage function

				Moving_X_Y_Z_Voyage();

				break;
			}
		case 9001: 
			{// Z_Movement function

				Moving_Z_movement(); 

				break;
			}
		case 9002: 
			{// Carr�e_function

				Moving_Carre();

				break; 
			}
		//case 9003: 
		//	{// Carrée_function

		//		//Procedure_test_Zim();

		//		break; 
		//	}
		case 9100:
			{

				premier_transport = true;

				break;
			}					
		//case 9999: // Test debug
		//	{
		//		MYSERIAL.print("#");
		//		MYSERIAL.print(READ(PRIVATE_ENDSTOPS1), BIN);
		//		MYSERIAL.print("#");
		//		MYSERIAL.print(E_ENDSTOPS_INVERTING, BIN);
		//		MYSERIAL.print("#");
		//		MYSERIAL.print(READ(PRIVATE_ENDSTOPS1) ^ E_ENDSTOPS_INVERTING, BIN);
		//		MYSERIAL.print("#");

		//		active_extruder=1;
		//		allow_cold_extrudes(true);
		//		relative_mode = true;

		//		unloading_command = true;

		//		destination[3] = -Unloading_part3_length + (axis_relative_modes[3] || relative_mode)*current_position[3];
		//		destination[0] = current_position[0];
		//		destination[1] = current_position[1];
		//		destination[2] = current_position[2]; 
		//		feedrate = Unloading_part3_feedrate;  
		//		prepare_move();

		//		relative_mode = false;

		//		st_synchronize();
		//		disable_e0();
		//		disable_e1();
		//		disable_e2();
		//		finishAndDisableSteppers();

		//		break;
		//	}					
		default :	// By default
			{
				SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
				SERIAL_ECHO(cmdbuffer[bufindr]);
				SERIAL_ECHOLNPGM("\"");
			}
		}
	}

	else if(code_seen('T')) 
	{
		tmp_extruder = code_value();
		if(tmp_extruder >= EXTRUDERS) {
			SERIAL_ECHO_START;
			SERIAL_ECHO("T");
			SERIAL_ECHO(tmp_extruder);
			SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
		}
		else {
			boolean make_move = false;
			if(code_seen('F')) {
				make_move = true;
				next_feedrate = code_value();
				if(next_feedrate > 0.0) { 
					feedrate = next_feedrate;
				}
			}
#if EXTRUDERS > 1
			if(tmp_extruder != active_extruder) {
				// Save current position to return to after applying extruder offset
				memcpy(destination, current_position, sizeof(destination));
				// Offset extruder (only by XY)
				int i;
				for(i = 0; i < 2; i++) {
					current_position[i] = current_position[i] - 
						extruder_offset[i][active_extruder] +
						extruder_offset[i][tmp_extruder];
				}
				// Set the new active extruder and position
				active_extruder = tmp_extruder;
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
				// Move to the old position if 'F' was in the parameters
				if(make_move && Stopped == false) {
					prepare_move();
				}
			}
#endif
			SERIAL_ECHO_START;
			SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
			SERIAL_PROTOCOLLN((int)active_extruder);
		}
	}

	else
	{
		SERIAL_ECHO_START;
		SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
		SERIAL_ECHO(cmdbuffer[bufindr]);
		SERIAL_ECHOLNPGM("\"");
	}

	ClearToSend();
}

void FlushSerialRequestResend()
{
	//char cmdbuffer[bufindr][100]="Resend:";
	MYSERIAL.flush();
	SERIAL_PROTOCOLPGM(MSG_RESEND);
	SERIAL_PROTOCOLLN(gcode_LastN + 1);
	ClearToSend();
}

void ClearToSend()
{
	previous_millis_cmd = millis();
#ifdef SDSUPPORT
	if(fromsd[bufindr])
		return;
#endif //SDSUPPORT
	MYSERIAL.print("\n");
	SERIAL_PROTOCOLLNPGM(MSG_OK); 
}

void get_coordinates()
{
	bool seen[4]={false,false,false,false};
	for(int8_t i=0; i < NUM_AXIS; i++) {
		if(code_seen(axis_codes[i])) 
		{
			destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
			seen[i]=true;
		}
		else destination[i] = current_position[i]; //Are these else lines really needed?
	}
	if(code_seen('F')) {
		next_feedrate = code_value();
		if(next_feedrate > 0.0) feedrate = next_feedrate;
	}
#ifdef FWRETRACT
	if(autoretract_enabled)
		if( !(seen[X_AXIS] || seen[Y_AXIS] || seen[Z_AXIS]) && seen[E_AXIS])
		{
			float echange=destination[E_AXIS]-current_position[E_AXIS];
			if(echange<-MIN_RETRACT) //retract
			{
				if(!retracted) 
				{ 

					destination[Z_AXIS]+=retract_zlift; //not sure why chaninging current_position negatively does not work.
					//if slicer retracted by echange=-1mm and you want to retract 3mm, corrrectede=-2mm additionally
					float correctede=-echange-retract_length;
					//to generate the additional steps, not the destination is changed, but inversely the current position
					current_position[E_AXIS]+=-correctede; 
					feedrate=retract_feedrate;
					retracted=true;
				}

			}
			else 
				if(echange>MIN_RETRACT) //retract_recover
				{
					if(retracted) 
					{
						//current_position[Z_AXIS]+=-retract_zlift;
						//if slicer retracted_recovered by echange=+1mm and you want to retract_recover 3mm, corrrectede=2mm additionally
						float correctede=-echange+1*retract_length+retract_recover_length; //total unretract=retract_length+retract_recover_length[surplus]
						current_position[E_AXIS]+=correctede; //to generate the additional steps, not the destination is changed, but inversely the current position
						feedrate=retract_recover_feedrate;
						retracted=false;
					}
				}

		}
#endif //FWRETRACT
}

void get_arc_coordinates()
{
#ifdef SF_ARC_FIX
	bool relative_mode_backup = relative_mode;
	relative_mode = true;
#endif
	get_coordinates();
#ifdef SF_ARC_FIX
	relative_mode=relative_mode_backup;
#endif

	if(code_seen('I')) {
		offset[0] = code_value();
	} 
	else {
		offset[0] = 0.0;
	}
	if(code_seen('J')) {
		offset[1] = code_value();
	}
	else {
		offset[1] = 0.0;
	}
}

void clamp_to_software_endstops(float target[3])
{
	if (min_software_endstops) {
		if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
		if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
		if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
	}

	if (max_software_endstops) {
		if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
		if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
		if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
	}
}

void prepare_move()
{
	clamp_to_software_endstops(destination);

	if(active_extruder==0)
	{
		Distance_Filament_E0 = Distance_Filament_E0 + destination[E_AXIS]-current_position[E_AXIS];
	}

	if(active_extruder==1)
	{
		Distance_Filament_E1 = Distance_Filament_E1 + destination[E_AXIS]-current_position[E_AXIS];
	}

	previous_millis_cmd = millis(); 
	// Do not use feedmultiply for E or Z only moves
	if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
	}
	else {
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
	}
	for(int8_t i=0; i < NUM_AXIS; i++) {
		current_position[i] = destination[i];
	}


}

void prepare_arc_move(char isclockwise) {
	float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

	// Trace the arc
	mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

	// As far as the parser is concerned, the position is now == target. In reality the
	// motion control system might still be processing the action and the real tool position
	// in any intermediate location.
	for(int8_t i=0; i < NUM_AXIS; i++) {
		current_position[i] = destination[i];
	}
	previous_millis_cmd = millis();
}

#ifdef CONTROLLERFAN_PIN
unsigned long lastMotor = 0; //Save the time for when a motor was turned on last
unsigned long lastMotorCheck = 0;

void controllerFan()
{
	if ((millis() - lastMotorCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
	{
		lastMotorCheck = millis();

		if(!READ(X_ENABLE_PIN) || !READ(Y_ENABLE_PIN) || !READ(Z_ENABLE_PIN)
#if EXTRUDERS > 2
			|| !READ(E2_ENABLE_PIN)
#endif
#if EXTRUDER > 1
			|| !READ(E2_ENABLE_PIN)
#endif
			|| !READ(E0_ENABLE_PIN)) //If any of the drivers are enabled...    
		{
			lastMotor = millis(); //... set time to NOW so the fan will turn on
		}

		if ((millis() - lastMotor) >= (CONTROLLERFAN_SEC*1000UL) || lastMotor == 0) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...   
		{
			WRITE(CONTROLLERFAN_PIN, LOW); //... turn the fan off
		}
		else
		{
			WRITE(CONTROLLERFAN_PIN, HIGH); //... turn the fan on
		}
	}
}
#endif

#ifdef EXTRUDERFAN_PIN
unsigned long lastExtruderCheck = 0;

void extruderFan()
{
	if ((millis() - lastExtruderCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
	{
		lastExtruderCheck = millis();

		if (degHotend(active_extruder) < EXTRUDERFAN_DEC)
		{
			WRITE(EXTRUDERFAN_PIN, LOW); //... turn the fan off
		}
		else
		{
			WRITE(EXTRUDERFAN_PIN, HIGH); //... turn the fan on
		}
	}
}
#endif


void Fonction_Transport()
{

#ifdef Fonction_transport_Define 

#if (Endstop_voyage > -1)
	bool etat_endstop_voyage=(READ(Endstop_voyage) != Voyage_ENDSTOPS_INVERTING);
	if((premier_transport==true)&&(!etat_endstop_voyage))
	{
		//premier_transport = false;

		MYSERIAL.print("\n Fonction transport \n");		
		Moving_X_Y_Z_Voyage(); 
		delay(10000);

	}

#endif
#endif

}	 




void Function_Z_Movement()
{

#ifdef Fonction_Z_Movement_Define

#if (Endstop_Z_Movement > -1)
	bool etat_endstop_z_movement=(READ(Endstop_Z_Movement) != Z_Movement_ENDSTOPS_INVERTING);
	//SERIAL_PROTOCOLLN(((READ(Endstop_voyage)^Voyage_ENDSTOPS_INVERTING)?"filament":"no filament"));
	if(!etat_endstop_z_movement)
	{	

		MYSERIAL.print("\n Fonction Z \n");
		Moving_Z_movement(); 
		delay(10000);

	}

#endif
#endif

}







void Function_Carre ()
{

#ifdef Fonction_Carre_Define

#if (Endstop_Carre  > -1)
	bool etat_endstop_carre=(READ(Endstop_Carre ) != Carre_ENDSTOPS_INVERTING);
	//SERIAL_PROTOCOLLN(((READ(Endstop_voyage)^Voyage_ENDSTOPS_INVERTING)?"filament":"no filament"));
	if(!etat_endstop_carre)
	{	
		MYSERIAL.print("\n Fonction Carr� \n");
		Moving_Carre(); 

		delay(10000);

	}

#endif
#endif

}






void Read_Power_Button()
{ 
	// read the value of the Kill_button

#ifdef Power_Button



	float voltage_value = VoltageButton();



	if (voltage_value < 37)
	{
		Read_button_1 = 0;	
		Read_button_2 = 0;	
	}	


	if ((voltage_value>37)&&(Read_button_1>0))
	{
		Read_button_2 = millis();

		delay(100);
	}	

	if ((voltage_value>37)&&(Read_button_1==0))
	{
		Read_button_1 = millis();

		Read_button_2 = Read_button_1 + max_time_pushing - 2000;
		delay(100);	
	}





	if( (Read_button_2 - Read_button_1) >  max_time_pushing ) 
		//if( (Read_button_2-Read_button_1) >  4000 ) 
	{


		MYSERIAL.print("\n");
		MYSERIAL.print("Je rentre");
		MYSERIAL.print("\n"); 
		Read_button_1 = 0;	
		Read_button_2 = 0;	
		//delay(1000);






		//if(max_time_pushing) 
		//{
		// Interruption of programme
		quickStop(); 

		// Tuer la Zim!!
		kill_Zim();  
		//}

	}	



#endif





}



void manage_inactivity() 
{ 


	if( (millis() - previous_millis_cmd) >  max_inactive_time ) 
		if(max_inactive_time) 
		{

			//kill(); 
		}	  
		if(stepper_inactive_time)  {
			if( (millis() - previous_millis_cmd) >  stepper_inactive_time ) 
			{
				if(blocks_queued() == false) {
					//SERIAL_ECHOLNPGM("\n LOW POWER SUPPLY\n");

#ifdef LOW_POWER_STANDBY

					if ((degHotend(0)> 180 ) || (degHotend(1)> 180 ))
					{		
						//					ecrir_sur_RFID1();
						//					ecrir_sur_RFID2();	
						Reverse_20mm();					
						Reinitialise_E0_E1_Quantity();	
						Moving_Z_2();
						Moving_X_Y();
						Set_temperature_to_0Degree();
					}

#endif
					disable_x();
					disable_y();
					disable_z();
					disable_e0();
					disable_e1();
					disable_e2();
				}
			}
		} 

		// Deactivate extruder motors

		if(extruder_inactive_time)  {
			if( (millis() - previous_millis_cmd) >  extruder_inactive_time) {
				if(blocks_queued() == false) {
					disable_e0();
					disable_e1();
					disable_e2();
				}
			}
		} 

#if( KILL_PIN>-1 )
		if( 0 == READ(KILL_PIN) )
			kill(); 
#endif
#ifdef CONTROLLERFAN_PIN 
		controllerFan(); //Check if fan should be turned on to cool stepper drivers down
#endif
#ifdef EXTRUDER_RUNOUT_PREVENT
		if( (millis() - previous_millis_cmd) >  EXTRUDER_RUNOUT_SECONDS*1000 ) 
			if(degHotend(active_extruder)>EXTRUDER_RUNOUT_MINTEMP)
			{
				bool oldstatus=READ(E0_ENABLE_PIN);
				enable_e0();
				float oldepos=current_position[E_AXIS];
				float oldedes=destination[E_AXIS];
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], 
					current_position[E_AXIS]+EXTRUDER_RUNOUT_EXTRUDE*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS], 
					EXTRUDER_RUNOUT_SPEED/60.*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS], active_extruder);
				current_position[E_AXIS]=oldepos;
				destination[E_AXIS]=oldedes;
				plan_set_e_position(oldepos);
				previous_millis_cmd=millis();
				st_synchronize();
				WRITE(E0_ENABLE_PIN,oldstatus);
			}
#endif
			check_axes_activity();
}     

void kill()
{
	cli(); // Stop interrupts
	disable_heater();

	disable_x();
	disable_y();
	disable_z();
	disable_e0();
	disable_e1();
	disable_e2();

	if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,INPUT);
	SERIAL_ERROR_START;
	SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
	LCD_ALERTMESSAGEPGM(MSG_KILLED);
	suicide();
	while(1) { /* Intentionally left empty */ } // Wait for reset
}












void Stop()
{
	disable_heater();
	if(Stopped == false) {
		Stopped = true;
		Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
		SERIAL_ERROR_START;
		SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
		LCD_MESSAGEPGM(MSG_STOPPED);
	}
}

bool IsStopped() { return Stopped; };

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
{
	val &= 0x07;
	switch(digitalPinToTimer(pin))
	{

#if defined(TCCR0A)
	case TIMER0A:
	case TIMER0B:
		//         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
		//         TCCR0B |= val;
		break;
#endif

#if defined(TCCR1A)
	case TIMER1A:
	case TIMER1B:
		//         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
		//         TCCR1B |= val;
		break;
#endif

#if defined(TCCR2)
	case TIMER2:
	case TIMER2:
		TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
		TCCR2 |= val;
		break;
#endif

#if defined(TCCR2A)
	case TIMER2A:
	case TIMER2B:
		TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
		TCCR2B |= val;
		break;
#endif

#if defined(TCCR3A)
	case TIMER3A:
	case TIMER3B:
	case TIMER3C:
		TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
		TCCR3B |= val;
		break;
#endif

#if defined(TCCR4A) 
	case TIMER4A:
	case TIMER4B:
	case TIMER4C:
		TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
		TCCR4B |= val;
		break;
#endif

#if defined(TCCR5A) 
	case TIMER5A:
	case TIMER5B:
	case TIMER5C:
		TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
		TCCR5B |= val;
		break;
#endif

	}
}
#endif //FAST_PWM_FAN

bool setTargetedHotend(int code){
	tmp_extruder = active_extruder;
	if(code_seen('T')) {
		tmp_extruder = code_value();
		if(tmp_extruder >= EXTRUDERS) {
			SERIAL_ECHO_START;
			switch(code){
			case 104:
				SERIAL_ECHO(MSG_M104_INVALID_EXTRUDER);
				break;
			case 105:
				SERIAL_ECHO(MSG_M105_INVALID_EXTRUDER);
				break;
			case 109:
				SERIAL_ECHO(MSG_M109_INVALID_EXTRUDER);
				break;
			case 218: 
				SERIAL_ECHO(MSG_M218_INVALID_EXTRUDER);
				break;
			}
			SERIAL_ECHOLN(tmp_extruder);
			return true;
		}
	}
	return false;
}
