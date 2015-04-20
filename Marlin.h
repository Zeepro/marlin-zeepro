// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>


#include "fastio.h"
#include "Configuration.h"
#include "pins.h"
/*
#ifndef AT90USB
#define  HardwareSerial_h // trick to disable the standard HWserial
#endif*/

#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
  //Arduino < 1.0.0 does not define this, so we need to do it ourselfs
# define analogInputToDigitalPin(p) ((p) + A0)
#endif

#include "MarlinSerial.h"
#include "Marlin_RFID_HWSerial2.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "WString.h"

#if MOTHERBOARD == 8  // Teensylu
  #define MYSERIAL Serial
#else
  #ifdef RFIDSUPPORT
  // PNI 20140723 Invert Serial port of two RFID readers
  //#define MYSERIAL2 RFIDSerial
  //#define MYSERIAL3 RFIDSerial2
  #define MYSERIAL2 RFIDSerial2
  #define MYSERIAL3 RFIDSerial
  // PNI End
  #endif
   #define MYSERIAL MSerial
#endif

//this is a unfinsihed attemp to removes a lot of warning messages, see:
// http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=57011
//typedef char prog_char PROGMEM; 
// //#define PSTR    (s )        ((const PROGMEM char *)(s))
// //# define MYPGM(s) (__extension__({static prog_char __c[] = (s); &__c[0];})) 
// //#define MYPGM(s) ((const prog_char *g PROGMEM=s))
#define MYPGM(s) PSTR(s)
//#define MYPGM(s)  (__extension__({static char __c[] __attribute__((__progmem__)) = (s); &__c[0];}))  //This is the normal behaviour
//#define MYPGM(s)  (__extension__({static prog_char __c[]  = (s); &__c[0];})) //this does not work but hides the warnings

// #ifdef RFIDSUPPORT
// #define SERIAL_PROTOCOL(x) { MYSERIAL.print(x); MYSERIAL2.print(x); MYSERIAL3.print(x); };
// #else
#define SERIAL_PROTOCOL(x) MYSERIAL.print(x)
// #endif

// #ifdef RFIDSUPPORT
// #define SERIAL_PROTOCOL_F(x, y) { MYSERIAL.print(x, y); MYSERIAL2.print(x, y); MYSERIAL3.print(x, y); };
// #else
#define SERIAL_PROTOCOL_F(x, y) MYSERIAL.print(x, y)
// #endif

// #ifdef RFIDSUPPORT
// #define SERIAL_PROTOCOLPGM(x) { serialprintPGM(MYPGM(x)); serialprintPGM2(MYPGM(x)); serialprintPGM3(MYPGM(x));};
// #else
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(MYPGM(x))
// #endif

// #ifdef RFIDSUPPORT
// #define SERIAL_PROTOCOLLN(x) { MYSERIAL.print(x); MYSERIAL.write('\n'); MYSERIAL2.print(x); MYSERIAL2.write('\n'); MYSERIAL3.print(x); MYSERIAL3.write('\n');};
// #else
#define SERIAL_PROTOCOLLN(x) { MYSERIAL.print(x); MYSERIAL.write('\n'); }
// #endif

// #ifdef RFIDSUPPORT
// #define SERIAL_PROTOCOLLNPGM(x) { serialprintPGM(MYPGM(x)); serialprintPGM2(MYPGM(x)); serialprintPGM3(MYPGM(x)); MYSERIAL.write('\n'); MYSERIAL2.write('\n'); MYSERIAL3.write('\n');}
// #else
#define SERIAL_PROTOCOLLNPGM(x) { serialprintPGM(MYPGM(x)); MYSERIAL.write('\n'); }
// #endif


const char errormagic[] PROGMEM ="Error:";
const char echomagic[] PROGMEM ="echo:";
// #ifdef RFIDSUPPORT
// #define SERIAL_ERROR_START { serialprintPGM(errormagic); serialprintPGM2(errormagic); serialprintPGM3(errormagic);}
// #else
#define SERIAL_ERROR_START serialprintPGM(errormagic);
// #endif
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

// #ifdef RFIDSUPPORT
// #define SERIAL_ECHO_START { serialprintPGM(echomagic); serialprintPGM2(echomagic); serialprintPGM3(echomagic); }
// #else
#define SERIAL_ECHO_START serialprintPGM(echomagic);
// #endif
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) { SERIAL_ECHOPGM(name); SERIAL_ECHO(value); }

//things to write to serial from Programmemory. saves 400 to 2k of RAM.
#define SerialprintPGM(x) serialprintPGM(MYPGM(x))
FORCE_INLINE void serialprintPGM(const char *str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    MYSERIAL.write(ch);
    ch=pgm_read_byte(++str);
  }
}

/* 
#ifdef RFIDSUPPORT
// things to write to bluetooth from Programmemory. saves 400 to 2k of RAM???
// actually reproduces serialprintPGM that's all
#define SerialprintPGM2(x) serialprintPGM2(MYPGM(x))
FORCE_INLINE void serialprintPGM2(const char *str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    MYSERIAL2.write(ch);
    ch=pgm_read_byte(++str);
  }
}

// second one

#define SerialprintPGM3(x) serialprintPGM3(MYPGM(x))
FORCE_INLINE void serialprintPGM3(const char *str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    MYSERIAL3.write(ch);
    ch=pgm_read_byte(++str);
  }
}
#endif
 */


void get_command();
void process_commands();

void manage_inactivity();
void Read_Power_Button();
void Fonction_Transport();
void Function_Z_Movement();
void Function_Carre ();

#if X_ENABLE_PIN > -1
  #define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#else
  #define enable_x() ;
  #define disable_x() ;
#endif

#if Y_ENABLE_PIN > -1
  #define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
  #define disable_y() WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON)
#else
  #define enable_y() ;
  #define disable_y() ;
#endif

#if Z_ENABLE_PIN > -1
  #ifdef Z_DUAL_STEPPER_DRIVERS
    #define  enable_z() { WRITE(Z_ENABLE_PIN, Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON); }
    #define disable_z() { WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN,!Z_ENABLE_ON); }
  #else
    #define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
    #define disable_z() WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON)
  #endif
#else
  #define enable_z() ;
  #define disable_z() ;
#endif

#if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
  #define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e0()  /* nothing */
  #define disable_e0() /* nothing */
#endif

#if (EXTRUDERS > 1) && defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
  #define enable_e1() WRITE(E1_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e1() WRITE(E1_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e1()  /* nothing */
  #define disable_e1() /* nothing */
#endif

#if (EXTRUDERS > 2) && defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
  #define enable_e2() WRITE(E2_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e2() WRITE(E2_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e2()  /* nothing */
  #define disable_e2() /* nothing */
#endif


enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};
	
enum Stop_Parameters_Enum {X_parm=0, Y_parm=1, Z_parm=2, T0_parm=3, T1_parm=4};


void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
void prepare_move();
void kill();
void kill_Zim();
void Stop();

bool IsStopped();

void enquecommand(const char *cmd); //put an ascii command at the end of the current buffer.
void enquecommand_P(const char *cmd); //put an ascii command at the end of the current buffer, read from flash
void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val);
#endif

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;
extern int extrudemultiply; // Sets extrude multiply factor (in percent)
extern int extruder_multiply[EXTRUDERS]; // sets extrude multiply factor (in percent) for each extruder individually
extern float current_position[NUM_AXIS] ;
extern float Pause_current_position[NUM_STOP_PARAMETERS] ;
extern float add_homeing[3];
extern float min_pos[3];
extern float max_pos[3];
extern int fanSpeed;

#ifdef FWRETRACT
extern bool autoretract_enabled;
extern bool retracted;
extern float retract_length, retract_feedrate, retract_zlift;
extern float retract_recover_length, retract_recover_feedrate;
#endif

extern unsigned long starttime;
extern unsigned long stoptime;

// Handling multiple extruders pins
extern uint8_t active_extruder;
// extern bool unloading_critical_state;
extern bool unloading_command;
extern byte TAG_RFID_E0[16];
extern byte TAG_RFID_E1[16];
// extern bool Desactiver_interruption;

#endif
