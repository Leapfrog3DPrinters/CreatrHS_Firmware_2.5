#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// This configurtion file contains the basic settings.
// Advanced settings can be found in Configuration_adv.h
// BASIC SETTINGS: select your board type, temperature sensor type, axis scaling, and endstop configuration

//User specified version info of THIS file to display in [Pronterface, etc] terminal window during startup.
//Implementation of an idea by Prof Braino to inform user that any changes made
//to THIS file by the user have been successfully uploaded into firmware.
#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__ //Personal revision number for changes to THIS file.
#define STRING_CONFIG_H_AUTHOR "Leapfrog 3D Printers" //Who made the changes.
#define LEAPFROG_FIRMWARE_VERSION "2.5"
#define LEAPFROG_MODEL "CreatrHS"

// This determines the communication speed of the printer
#define BAUDRATE 250000

// In steps that needs to be taken from the second extruder to the first extruder
// See M50 Gcode commando for more info 
#define EXTR2_X_OFFSET 15.00
#define EXTR2_Y_OFFSET 0.00

// The following define selects which electronics board you have. Please choose the one that matches your setup

#ifndef MOTHERBOARD
#define MOTHERBOARD 1
#endif

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================
//
// 1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
// 60 is 100k Thermistor with adepted tabel by Erik Hijdstra
#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 1
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_BED 1

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10	// (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the recidency timer x degC early.

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define BED_MINTEMP 5

//#define USE_BED_PWM // Comment to disable PWM control of the heated bed

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 300 
#define HEATER_1_MAXTEMP 300
#define HEATER_2_MAXTEMP 300
#define BED_MAXTEMP 150

// PID settings:
// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP
#define PID_MAX 255 // limits current to nozzle; 255=full current
#ifdef PIDTEMP
  //#define PID_DEBUG // Sends debug data to the serial port.
  //#define PID_OPENLOOP 1 // Puts PID in open loop. M104 sets the output power in %
  #define PID_INTEGRAL_DRIVE_MAX 255  //limit for the integral term
  #define K1 0.95 //smoothing factor withing the PID
  #define PID_dT ((16.0 * 8.0)/(F_CPU / 64.0 / 256.0)) //sampling period of the

// If you are using a preconfigured hotend then you can use one of the value sets by uncommenting it
// Creatr HS Old
//    #define  DEFAULT_Kp 12.68
//    #define  DEFAULT_Ki 0.64
//    #define  DEFAULT_Kd 62.75
// Creatr HS
#define  DEFAULT_Kp 12.00
#define  DEFAULT_Ki 0.20
#define  DEFAULT_Kd 110.0

#endif // PIDTEMP

#ifdef USE_BED_PWM
  #define BED_PID_THRESHOLD 5 // Temperature error (target-current) at which PID control becomes active. On/off control is used outside this range
  #define BED_K1 0.95 //smoothing factor withing the PID
  #define DEFAULT_BED_Kp 35.00
  #define DEFAULT_BED_Ki 0.6
  #define DEFAULT_BED_Kd 110.0

#endif //USE_BED_PWM

//this prevents dangerous Extruder moves, i.e. if thise temperature is under the limit
//can be software-disabled for whatever purposes by
#define PREVENT_DANGEROUS_EXTRUDE
//if PREVENT_DANGEROUS_EXTRUDE is on, you can still disable (uncomment) very long bits of extrusion separately.
#define PREVENT_LENGTHY_EXTRUDE

#define EXTRUDE_MINTEMP 150
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH+Y_MAX_LENGTH) //prevent extrusion of very large distances.

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// corse Endstop Settings
#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

#ifndef ENDSTOPPULLUPS
  // fine Enstop settings: Individual Pullups. will be ignord if ENDSTOPPULLUPS is defined
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
#endif

#ifdef ENDSTOPPULLUPS
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
#endif


#define ENDSTOPPULLUP_ZMIN
// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
const bool X_ENDSTOPS_INVERTING = false; // set to true to invert the logic of the endstops.
const bool Y_ENDSTOPS_INVERTING = false; // set to true to invert the logic of the endstops.
const bool Z_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
//#define DISABLE_MAX_ENDSTOPS

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false // prevented stripping
#define DISABLE_E true // there are some new funtions in the new version of Merlin

#define INVERT_X_DIR false    // for Creatr HS set to false
#define INVERT_Y_DIR false    // for Creatr HS set to false
#define INVERT_Z_DIR true     // for Creatr HS set to true
#define INVERT_E0_DIR false   // for new creatr HS extruder related to direction of extuder is mounted
#define INVERT_E1_DIR true    // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E2_DIR true   // for direct drive extruder v9 set to true, for geared extruder set to false

// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

#define min_software_endstops false //If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  //If true, axis won't move to coordinates greater than the defined lengths below.

// The position of the homing switches. Use MAX_LENGTH * -0.5 if the center should be 0, 0, 0
#define X_HOME_POS 0
#define Y_HOME_POS 0
#define Z_HOME_POS 0

// Travel limits after homing
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define X_MAX_POS 270
#define Y_MAX_POS 280
#define Z_MAX_POS 180

// Max length 
#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)

//// MOVEMENT SETTINGS
#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E
#define HOMING_FEEDRATE {70*60, 70*60, 20*60, 0}  // set the homing speeds (mm/min)

// default settings
#define DEFAULT_AXIS_STEPS_PER_UNIT   {(100.0/3)*2, (100.0/3)*2, 9600.0/10, 99.30}  //55.465*2 HS Creatr settings for 16 steps all 16 steping
#define DEFAULT_MAX_FEEDRATE          {400, 400, 40, 400}    // (mm/sec) Creatr was 200, 200, 10, 100, 100
#define DEFAULT_MAX_ACCELERATION      {9000,9000,100, 10000}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.

#define DEFAULT_ACCELERATION          3000   // X, Y, Z and E max acceleration in mm/s^2 for prinxting moves
#define DEFAULT_RETRACT_ACCELERATION  3000   // X, Y, Z and E max acceleration in mm/s^2 for r retracts

//
#define DEFAULT_XYJERK                15.0   // (mm/sec)
#define DEFAULT_ZJERK                 0.4     // (mm/sec)
#define DEFAULT_EJERK                 5.0    // (mm/sec)

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
#define EXTRUDER_OFFSET_X {0.0, -EXTR2_X_OFFSET} // (in mm) for each extruder, offset of the hotend on the X axis
#define EXTRUDER_OFFSET_Y {0.0, -EXTR2_Y_OFFSET}  // (in mm) for each extruder, offset of the hotend on the Y axis

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

// EEPROM
// the microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//define this to enable eeprom support
#define EEPROM_SETTINGS
//to disable EEPROM Serial responses and decrease program space by ~1700 byte: comment this out:
// please keep turned on if you can.
#define EEPROM_CHITCHAT

// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
#define FAST_PWM_FAN

#include "Configuration_adv.h"
#include "thermistortables.h"

#endif //__CONFIGURATION_H
