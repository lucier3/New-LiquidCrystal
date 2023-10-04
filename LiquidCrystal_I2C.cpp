/*
  Electronic Level
  mpu-6050-level.ino
  Uses MPU-6050 IMU
  Displays on 128x64 OLED and LED
  
  DroneBot Workshop 2019
  https://dronebotworkshop.com
*/

// Include Wire Library for I2C
#include <Wire.h>

// Include NewLiquidCrystal Library for I2C
#include <LiquidCrystal_I2C.h>

// Define LCD pinout
const int  en = 2, rw = 1, rs = 0, d4 = 4, d5 = 5, d6 = 6, d7 = 7, bl = 3;
 
// Define I2C Address - change if reqiuired
const int i2c_addr = 0x3F;
 
LiquidCrystal_I2C lcd(i2c_addr, en, rw, rs, d4, d5, d6, d7, bl, POSITIVE);

// Level LEDs
int levelLED_neg1 = 9;
int levelLED_neg0 = 10;
int levelLED_level = 11;
int levelLED_pos0 = 12;
int levelLED_pos1 = 13;



//Variables for Gyroscope
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

// Setup timers and temp variables
long loop_timer;
int temp;

// Display counter
int displaycount = 0;

void setup() {

  //Start I2C
  Wire.begin();
  
  // Set display type as 16 char, 2 rows
  lcd.begin(16,2); 
  
  // Set Level LEDs as outputs
  pinMode(levelLED_neg1, OUTPUT);
  pinMode(levelLED_neg0, OUTPUT);
  pinMode(levelLED_level, OUTPUT);
  pinMode(levelLED_pos0, OUTPUT);
  pinMode(levelLED_pos1, OUTPUT);
  
  
  //Setup the registers of the MPU-6050                                                       
  setup_mpu_6050_registers(); 
  
  //Read the raw acc and gyro data from the MPU-6050 1000 times                                          
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  
    read_mpu_6050_data(); 
    //Add the gyro x offset to the gyro_x_cal variable                                            
    gyro_x_cal += gyro_x;
    //Add the gyro y offset to the gyro_y_cal variable                                              
    gyro_y_cal += gyro_y; 
    //Add the gyro z offset to the gyro_z_cal variable                                             
    gyro_z_cal += gyro_z; 
    //Delay 3us to have 250Hz for-loop                                             
    delay(3);                                                          
  }

  // Divide all results by 1000 to get average offset
  gyro_x_cal /= 1000;                                                 
  gyro_y_cal /= 1000;                                                 
  gyro_z_cal /= 1000;
  
  // Start Serial Monitor                                                 
  Serial.begin(115200);
  
  // Init Timer 
  loop_timer = micros();                                               
}

void loop(){

  // Get data from MPU-6050
  read_mpu_6050_data();
     
  //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                
         
  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  
  //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_pitch += gyro_x * 0.0000611;  
  //Calculate the traveled roll angle and add this to the angle_roll variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians                                
  angle_roll += gyro_y * 0.0000611; 
                                     
  //If the IMU has yawed transfer the roll angle to the pitch angle
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  //If the IMU has yawed transfer the pitch angle to the roll angle               
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               
  
  //Accelerometer angle calculations
  
  //Calculate the total accelerometer vector
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z)); 
   
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  //Calculate the pitch angle
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296; 
  //Calculate the roll angle      
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  
  //Accelerometer calibration value for pitch
  angle_pitch_acc -= 0.0;
  //Accelerometer calibration value for roll                                              
  angle_roll_acc -= 0.0;                                               

  if(set_gyro_angles){ 
  
  //If the IMU has been running 
  //Correct the drift of the gyro pitch angle with the accelerometer pitch angle                      
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004; 
    //Correct the drift of the gyro roll angle with the accelerometer roll angle    
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        
  }
  else{ 
    //IMU has just started  
    //Set the gyro pitch angle equal to the accelerometer pitch angle                                                           
    angle_pitch = angle_pitch_acc;
    //Set the gyro roll angle equal to the accelerometer roll angle                                       
    angle_roll = angle_roll_acc;
    //Set the IMU started flag                                       
    set_gyro_angles = true;                                            
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1; 
  //Take 90% of the output roll value and add 10% of the raw roll value 
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1; 
  //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop  
  
  // Print to Serial Monitor   
  //Serial.print(" | Angle  = "); Serial.println(angle_pitch_output);
  
  
  // Increment the display counter
  displaycount = displaycount +1;
  
  if (displaycount > 100) {

  lcd.clear();
  // Print on first row of LCD
  lcd.setCursor(0,0);
  lcd.print("Pitch: ");
  lcd.print(angle_pitch_output);
  lcd.setCursor(0,1);
  lcd.print("Roll: ");
  lcd.print(angle_roll_output);
  
  
  // Check Angle for Level LEDs
  
    if (angle_pitch_output < -2.01) {
    // Turn on Level LED
    digitalWrite(levelLED_neg1, HIGH);
    digitalWrite(levelLED_neg0, LOW);
    digitalWrite(levelLED_level, LOW);
    digitalWrite(levelLED_pos0, LOW);
    digitalWrite(levelLED_pos1, LOW);
    
    } else if ((angle_pitch_output > -2.00) && (angle_pitch_output < -1.01)) {
    // Turn on Level LED
    digitalWrite(levelLED_neg1, LOW);
    digitalWrite(levelLED_neg0, HIGH);
    digitalWrite(levelLED_level, LOW);
    digitalWrite(levelLED_pos0, LOW);
    digitalWrite(levelLED_pos1, LOW);
    
    } else if ((angle_pitch_output < 1.00) && (angle_pitch_output > -1.00)) {
    // Turn on Level LED
    digitalWrite(levelLED_neg1, LOW);
    digitalWrite(levelLED_neg0, LOW);
    digitalWrite(levelLED_level, HIGH);
    digitalWrite(levelLED_pos0, LOW);
    digitalWrite(levelLED_pos1, LOW);
    
    } else if ((angle_pitch_output > 1.01) && (angle_pitch_output < 2.00)) {
    // Turn on Level LED
    digitalWrite(levelLED_neg1, LOW);
    digitalWrite(levelLED_neg0, LOW);
    digitalWrite(levelLED_level, LOW);
    digitalWrite(levelLED_pos0, HIGH);
    digitalWrite(levelLED_pos1, LOW);
    
    } else if (angle_pitch_output > 2.01) {
    // Turn on Level LED
    digitalWrite(levelLED_neg1, LOW);
    digitalWrite(levelLED_neg0, LOW);
    digitalWrite(levelLED_level, LOW);
    digitalWrite(levelLED_pos0, LOW);
    digitalWrite(levelLED_pos1, HIGH);
    
    }
    
  displaycount = 0;
  
  }
  

 while(micros() - loop_timer < 4000); 
 //Reset the loop timer                                
 loop_timer = micros();
  
}

void setup_mpu_6050_registers(){

  //Activate the MPU-6050
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68); 
  //Send the requested starting register                                       
  Wire.write(0x6B);  
  //Set the requested starting register                                                  
  Wire.write(0x00);
  //End the transmission                                                    
  Wire.endTransmission(); 
                                              
  //Configure the accelerometer (+/-8g)
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68); 
  //Send the requested starting register                                       
  Wire.write(0x1C);   
  //Set the requested starting register                                                 
  Wire.write(0x10); 
  //End the transmission                                                   
  Wire.endTransmission(); 
                                              
  //Configure the gyro (500dps full scale)
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register                                        
  Wire.write(0x1B);
  //Set the requested starting register                                                    
  Wire.write(0x08); 
  //End the transmission                                                  
  Wire.endTransmission(); 
                                              
}


void read_mpu_6050_data(){ 

  //Read the raw gyro and accelerometer data

  //Start communicating with the MPU-6050                                          
  Wire.beginTransmission(0x68);  
  //Send the requested starting register                                      
  Wire.write(0x3B);
  //End the transmission                                                    
  Wire.endTransmission(); 
  //Request 14 bytes from the MPU-6050                                  
  Wire.requestFrom(0x68,14);    
  //Wait until all the bytes are received                                       
  while(Wire.available() < 14);
  
  //Following statements left shift 8 bits, then bitwise OR.  
  //Turns two 8-bit values into one 16-bit value                                       
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                 
}// ---------------------------------------------------------------------------
// Created by Francisco Malpartida on 20/08/11.
// Copyright (C) - 2018
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License v3.0
//    along with this program.
//    If not, see <https://www.gnu.org/licenses/gpl-3.0.en.html>.
// 
// ---------------------------------------------------------------------------
//
// Thread Safe: No
// Extendable: Yes
//
// @file LiquidCrystal_I2C.c
// This file implements a basic liquid crystal library that comes as standard
// in the Arduino SDK but using an I2C IO extension board.
// 
// @brief 
// This is a basic implementation of the LiquidCrystal library of the
// Arduino SDK. The original library has been reworked in such a way that 
// this class implements the all methods to command an LCD based
// on the Hitachi HD44780 and compatible chipsets using I2C extension
// backpacks such as the I2CLCDextraIO with the PCF8574* I2C IO Expander ASIC.
//
// The functionality provided by this class and its base class is identical
// to the original functionality of the Arduino LiquidCrystal library.
//
//
//
// @author F. Malpartida - fmalpartida@gmail.com
// ---------------------------------------------------------------------------
#if (ARDUINO <  100)
#include <WProgram.h>
#else
#include <Arduino.h>
#endif
#include <inttypes.h>
#include "I2CIO.h"
#include "LiquidCrystal_I2C.h"

// CONSTANT  definitions
// ---------------------------------------------------------------------------

// flags for backlight control
/*!
 @defined 
 @abstract   LCD_NOBACKLIGHT
 @discussion NO BACKLIGHT MASK
 */
#define LCD_NOBACKLIGHT 0x00

/*!
 @defined 
 @abstract   LCD_BACKLIGHT
 @discussion BACKLIGHT MASK used when backlight is on
 */
#define LCD_BACKLIGHT   0xFF


// Default library configuration parameters used by class constructor with
// only the I2C address field.
// ---------------------------------------------------------------------------
/*!
 @defined 
 @abstract   Enable bit of the LCD
 @discussion Defines the IO of the expander connected to the LCD Enable
 */
#define EN 6  // Enable bit

/*!
 @defined 
 @abstract   Read/Write bit of the LCD
 @discussion Defines the IO of the expander connected to the LCD Rw pin
 */
#define RW 5  // Read/Write bit

/*!
 @defined 
 @abstract   Register bit of the LCD
 @discussion Defines the IO of the expander connected to the LCD Register select pin
 */
#define RS 4  // Register select bit

/*!
 @defined 
 @abstract   LCD dataline allocation this library only supports 4 bit LCD control
 mode.
 @discussion D4, D5, D6, D7 LCD data lines pin mapping of the extender module
 */
#define D4 0
#define D5 1
#define D6 2
#define D7 3


// CONSTRUCTORS
// ---------------------------------------------------------------------------
LiquidCrystal_I2C::LiquidCrystal_I2C( uint8_t lcd_Addr )
{
   config(lcd_Addr, EN, RW, RS, D4, D5, D6, D7);
}


LiquidCrystal_I2C::LiquidCrystal_I2C(uint8_t lcd_Addr, uint8_t backlighPin, 
                                     t_backlightPol pol = POSITIVE)
{
   config(lcd_Addr, EN, RW, RS, D4, D5, D6, D7);
   setBacklightPin(backlighPin, pol);
}

LiquidCrystal_I2C::LiquidCrystal_I2C(uint8_t lcd_Addr, uint8_t En, uint8_t Rw,
                                     uint8_t Rs)
{
   config(lcd_Addr, En, Rw, Rs, D4, D5, D6, D7);
}

LiquidCrystal_I2C::LiquidCrystal_I2C(uint8_t lcd_Addr, uint8_t En, uint8_t Rw,
                                     uint8_t Rs, uint8_t backlighPin, 
                                     t_backlightPol pol = POSITIVE)
{
   config(lcd_Addr, En, Rw, Rs, D4, D5, D6, D7);
   setBacklightPin(backlighPin, pol);
}

LiquidCrystal_I2C::LiquidCrystal_I2C(uint8_t lcd_Addr, uint8_t En, uint8_t Rw,
                                     uint8_t Rs, uint8_t d4, uint8_t d5,
                                     uint8_t d6, uint8_t d7 )
{
   config(lcd_Addr, En, Rw, Rs, d4, d5, d6, d7);
}

LiquidCrystal_I2C::LiquidCrystal_I2C(uint8_t lcd_Addr, uint8_t En, uint8_t Rw,
                                     uint8_t Rs, uint8_t d4, uint8_t d5,
                                     uint8_t d6, uint8_t d7, uint8_t backlighPin, 
                                     t_backlightPol pol = POSITIVE )
{
   config(lcd_Addr, En, Rw, Rs, d4, d5, d6, d7);
   setBacklightPin(backlighPin, pol);
}

// PUBLIC METHODS
// ---------------------------------------------------------------------------

//
// begin
void LiquidCrystal_I2C::begin(uint8_t cols, uint8_t lines, uint8_t dotsize) 
{
   
   init();     // Initialise the I2C expander interface
   LCD::begin ( cols, lines, dotsize );   
}


// User commands - users can expand this section
//----------------------------------------------------------------------------
// Turn the (optional) backlight off/on

//
// setBacklightPin
void LiquidCrystal_I2C::setBacklightPin ( uint8_t value, t_backlightPol pol = POSITIVE )
{
   _backlightPinMask = ( 1 << value );
   _polarity = pol;
   setBacklight(BACKLIGHT_OFF);
}

//
// setBacklight
void LiquidCrystal_I2C::setBacklight( uint8_t value ) 
{
   // Check if backlight is available
   // ----------------------------------------------------
   if ( _backlightPinMask != 0x0 )
   {
      // Check for polarity to configure mask accordingly
      // ----------------------------------------------------------
      if  (((_polarity == POSITIVE) && (value > 0)) || 
           ((_polarity == NEGATIVE ) && ( value == 0 )))
      {
         _backlightStsMask = _backlightPinMask & LCD_BACKLIGHT;
      }
      else 
      {
         _backlightStsMask = _backlightPinMask & LCD_NOBACKLIGHT;
      }
      _i2cio.write( _backlightStsMask );
   }
}


// PRIVATE METHODS
// ---------------------------------------------------------------------------

//
// init
int LiquidCrystal_I2C::init()
{
   int status = 0;
   
   // initialize the backpack IO expander
   // and display functions.
   // ------------------------------------------------------------------------
   if ( _i2cio.begin ( _Addr ) == 1 )
   {
      _i2cio.portMode ( OUTPUT );  // Set the entire IO extender to OUTPUT
      _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
      status = 1;
      _i2cio.write(0);  // Set the entire port to LOW
   }
   return ( status );
}

//
// config
void LiquidCrystal_I2C::config (uint8_t lcd_Addr, uint8_t En, uint8_t Rw, uint8_t Rs, 
                                uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7 )
{
   _Addr = lcd_Addr;
   
   _backlightPinMask = 0;
   _backlightStsMask = LCD_NOBACKLIGHT;
   _polarity = POSITIVE;
   
   _En = ( 1 << En );
   _Rw = ( 1 << Rw );
   _Rs = ( 1 << Rs );
   
   // Initialise pin mapping
   _data_pins[0] = ( 1 << d4 );
   _data_pins[1] = ( 1 << d5 );
   _data_pins[2] = ( 1 << d6 );
   _data_pins[3] = ( 1 << d7 );   
}



// low level data pushing commands
//----------------------------------------------------------------------------

//
// send - write either command or data
void LiquidCrystal_I2C::send(uint8_t value, uint8_t mode) 
{
   // No need to use the delay routines since the time taken to write takes
   // longer that what is needed both for toggling and enable pin an to execute
   // the command.
   
   if ( mode == FOUR_BITS )
   {
      write4bits( (value & 0x0F), COMMAND );
   }
   else 
   {
      write4bits( (value >> 4), mode );
      write4bits( (value & 0x0F), mode);
   }
}

//
// write4bits
void LiquidCrystal_I2C::write4bits ( uint8_t value, uint8_t mode ) 
{
   uint8_t pinMapValue = 0;
   
   // Map the value to LCD pin mapping
   // --------------------------------
   for ( uint8_t i = 0; i < 4; i++ )
   {
      if ( ( value & 0x1 ) == 1 )
      {
         pinMapValue |= _data_pins[i];
      }
      value = ( value >> 1 );
   }
   
   // Is it a command or data
   // -----------------------
   if ( mode == LCD_DATA )
   {
      mode = _Rs;
   }
   
   pinMapValue |= mode | _backlightStsMask;
   pulseEnable ( pinMapValue );
}

//
// pulseEnable
void LiquidCrystal_I2C::pulseEnable (uint8_t data)
{
   _i2cio.write (data | _En);   // En HIGH
   _i2cio.write (data & ~_En);  // En LOW
}
