/*
  Nivel electrónico
  mpu-6050-level.ino
  Utiliza IMU MPU-6050
  Pantallas OLED y LED de 128x64
  
  Taller DroneBot 2019
  https://dronebotworkshop.com
*/

// Incluir biblioteca de cables para I2C
# incluir  < Cable.h >

// Incluir biblioteca NewLiquidCrystal para I2C
# incluir  < LiquidCrystal_I2C.h >

// Definir configuración de pines LCD
constante  int   en = 2 , rw = 1 , rs = 0 , d4 = 4 , d5 = 5 , d6 = 6 , d7 = 7 , bl = 3 ;

// Definir dirección I2C - cambiar si es necesario
constante  int i2c_addr = 0x3F ;

LiquidCrystal_I2C lcd (i2c_addr, en, rw, rs, d4, d5, d6, d7, bl, POSITIVO);

// LED de nivel
int nivelLED_neg1 = 9 ;
int nivelLED_neg0 = 10 ;
int nivelLED_level = 11 ;
int nivelLED_pos0 = 12 ;
int nivelLED_pos1 = 13 ;



// Variables para giroscopio
int gyro_x, gyro_y, gyro_z;
largo gyro_x_cal, gyro_y_cal, gyro_z_cal;
booleano set_gyro_angles;

largo acc_x, acc_y, acc_z, acc_total_vector;
flotador ángulo_roll_acc, ángulo_pitch_acc;

ángulo flotante_pitch , ángulo_roll;
int ángulo_pitch_buffer, ángulo_roll_buffer;
flotador ángulo_pitch_output, ángulo_roll_output;

// Configurar temporizadores y variables temporales
temporizador de bucle largo ;
temperatura interna ;

// Mostrar contador
int displaycount = 0 ;

 configuración nula () {

  // Iniciar I2C
  Cable. comenzar ();

  // Establece el tipo de visualización en 16 caracteres, 2 filas
  LCD. comenzar ( 16 , 2 );

  // Establecer LED de nivel como salidas
  pinMode (nivelLED_neg1, SALIDA);
  pinMode (levelLED_neg0, SALIDA);
  pinMode (nivelLED_nivel, SALIDA);
  pinMode (levelLED_pos0, SALIDA);
  pinMode (nivelLED_pos1, SALIDA);


  // Configurar los registros del MPU-6050                                                       
  setup_mpu_6050_registers ();

  // Lee los datos sin procesar de aceleración y giro del MPU-6050 1000 veces                                          
  para ( int cal_int = 0 ; cal_int < 1000 ; cal_int ++){                  
    read_mpu_6050_data ();
    // Agrega el desplazamiento gyro x a la variable gyro_x_cal                                            
    giro_x_cal += giro_x;
    // Agrega el desplazamiento gyro y a la variable gyro_y_cal                                              
    giro_y_cal += giro_y;
    // Agrega el desplazamiento gyro z a la variable gyro_z_cal                                             
    giro_z_cal += giro_z;
    // Retraso 3us para tener un bucle for de 250 Hz                                             
    retraso ( 3 );                                                          
  }

  // Divide todos los resultados entre 1000 para obtener el desplazamiento promedio
  gyro_x_cal /= 1000 ;                                                 
  gyro_y_cal /= 1000 ;                                                 
  gyro_z_cal /= 1000 ;

  // Iniciar monitor serie                                                 
  De serie. comenzar ( 115200 );

  // Temporizador de inicio
  loop_timer = micros ();                                               
}

 bucle vacío (){

  // Obtener datos de MPU-6050
  read_mpu_6050_data ();

  // Resta los valores de compensación de los valores sin formato del giroscopio
  giro_x -= giro_x_cal;                                                
  giro_y -= giro_y_cal;                                                
  giro_z -= giro_z_cal;                                                

  // Cálculos del ángulo giroscópico. Nota 0,0000611 = 1 / (250 Hz x 65,5)

  // Calcula el ángulo de paso recorrido y agrégalo a la variable angle_pitch
  ángulo_pitch += gyro_x * 0.0000611 ;  
  // Calcula el ángulo de balanceo recorrido y agrégalo a la variable angle_roll
  // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) La función sin de Arduino está en radianes                                
  ángulo_roll += gyro_y * 0.0000611 ;

  // Si la IMU ha guiñado, transfiera el ángulo de balanceo al ángulo de cabeceo
  ángulo_pitch += ángulo_roll * sin (gyro_z * 0.000001066 );
  // Si la IMU ha guiñado, transfiera el ángulo de cabeceo al ángulo de balanceo               
  ángulo_roll -= ángulo_pitch * sin (gyro_z * 0.000001066 );               

  // Cálculos del ángulo del acelerómetro

  // Calcular el vector total del acelerómetro
  acc_total_vector = sqrt ((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));

  // 57.296 = 1 / (3.142 / 180) La función asin de Arduino está en radianes
  // Calcular el ángulo de paso
  ángulo_pitch_acc = asin (( float )acc_y/acc_total_vector)* 57.296 ;
  // Calcular el ángulo de balanceo      
  ángulo_roll_acc = asin (( float )acc_x/acc_total_vector)* - 57.296 ;       

  // Valor de calibración del acelerómetro para el paso
  ángulo_pitch_acc -= 0.0 ;
  // Valor de calibración del acelerómetro para rollo                                              
  ángulo_roll_acc -= 0.0 ;                                               

  si (set_gyro_angles){

  // Si la IMU ha estado funcionando
  // Corregir la desviación del ángulo de paso del giroscopio con el ángulo de paso del acelerómetro                      
    paso_ángulo = paso_ángulo * 0,9996 + paso_ángulo_acc * 0,0004 ;
    // Corregir la deriva del ángulo de giro del giroscopio con el ángulo de giro del acelerómetro    
    ángulo_rollo = ángulo_rollo * 0,9996 + ángulo_roll_acc * 0,0004 ;        
  }
  más {
    // IMU acaba de comenzar  
    // Establece el ángulo de paso del giroscopio igual al ángulo de paso del acelerómetro                                                           
    ángulo_pitch = ángulo_pitch_acc;
    // Establece el ángulo de giro del giroscopio igual al ángulo de giro del acelerómetro                                       
    ángulo_roll = ángulo_roll_acc;
    // Establece el indicador de inicio de IMU                                       
    set_gyro_angles = verdadero ;                                            
  }

  // Para amortiguar los ángulos de cabeceo y balanceo se utiliza un filtro complementario
  // Toma el 90% del valor del tono de salida y agrega el 10% del valor del tono sin procesar
  salida_paso_ángulo = salida_paso_ángulo * 0,9 + paso_ángulo * 0,1 ;
  // Tome el 90% del valor del rollo de salida y agregue el 10% del valor del rollo sin procesar
  salida_giro_ángulo = salida_giro_ángulo * 0,9 + giro_ángulo * 0,1 ;
  // Espere hasta que loop_timer alcance 4000us (250Hz) antes de iniciar el siguiente bucle  

  // Imprimir en el monitor serie   
  // Serial.print(" | Ángulo = "); Serial.println(angle_pitch_output);


  // Incrementar el contador de visualización
  recuento de visualización = recuento de visualización + 1 ;

  si (recuento de visualización > 100 ) {

  LCD. claro ();
  // Imprimir en la primera fila de la pantalla LCD
  LCD. establecerCursor ( 0 , 0 );
  LCD. imprimir ( " Tono: " );
  LCD. imprimir (ángulo_pitch_output);
  LCD. establecerCursor ( 0 , 1 );
  LCD. imprimir ( " Rollo: " );
  LCD. imprimir (angle_roll_output);


  // Comprobar el ángulo de los LED de nivel

    si (ángulo_pitch_output < - 2.01 ) {
    // Enciende el LED de nivel
    escritura digital (nivelLED_neg1, ALTO);
    escritura digital (nivelLED_neg0, BAJO);
    escritura digital (nivelLED_nivel, BAJO);
    escritura digital (nivelLED_pos0, BAJO);
    escritura digital (nivelLED_pos1, BAJO);

    } else  if ((angle_pitch_output > - 2.00 ) && (angle_pitch_output < - 1.01 )) {
    // Enciende el LED de nivel
    escritura digital (nivelLED_neg1, BAJO);
    escritura digital (nivelLED_neg0, ALTO);
    escritura digital (nivelLED_nivel, BAJO);
    escritura digital (nivelLED_pos0, BAJO);
    escritura digital (nivelLED_pos1, BAJO);

    } else  if ((angle_pitch_output < 1,00 ) && (angle_pitch_output > - 1,00 )) {
    // Enciende el LED de nivel
    escritura digital (nivelLED_neg1, BAJO);
    escritura digital (nivelLED_neg0, BAJO);
    escritura digital (nivelLED_nivel, ALTO);
    escritura digital (nivelLED_pos0, BAJO);
    escritura digital (nivelLED_pos1, BAJO);

    } más  si ((angle_pitch_output > 1.01 ) && (angle_pitch_output < 2.00 )) {
    // Enciende el LED de nivel
    escritura digital (nivelLED_neg1, BAJO);
    escritura digital (nivelLED_neg0, BAJO);
    escritura digital (nivelLED_nivel, BAJO);
    escritura digital (nivelLED_pos0, ALTO);
    escritura digital (nivelLED_pos1, BAJO);

    } si no  (angle_pitch_output > 2.01 ) {
    // Enciende el LED de nivel
    escritura digital (nivelLED_neg1, BAJO);
    escritura digital (nivelLED_neg0, BAJO);
    escritura digital (nivelLED_nivel, BAJO);
    escritura digital (nivelLED_pos0, BAJO);
    escritura digital (nivelLED_pos1, ALTO);

    }

  recuento de visualización = 0 ;

  }


 mientras ( micros () - loop_timer < 4000 );
 // Restablecer el temporizador del bucle                                
loop_timer = micros ();

}

anular  setup_mpu_6050_registers (){

  // Activar el MPU-6050

  // Comience a comunicarse con el MPU-6050
  Cable. comenzarTransmisión ( 0x68 );
  // Enviar el registro inicial solicitado                                       
  Cable. escribir ( 0x6B );  
  // Establece el registro inicial solicitado                                                  
  Cable. escribir ( 0x00 );
  // Finaliza la transmisión                                                    
  Cable. finalizar la transmisión ();

  // Configurar el acelerómetro (+/-8g)

  // Comience a comunicarse con el MPU-6050
  Cable. comenzarTransmisión ( 0x68 );
  // Enviar el registro inicial solicitado                                       
  Cable. escribir ( 0x1C );   
  // Establece el registro inicial solicitado                                                 
  Cable. escribir ( 0x10 );
  // Finaliza la transmisión                                                   
  Cable. finalizar la transmisión ();

  // Configurar el giroscopio (500dps escala completa)

  // Comience a comunicarse con el MPU-6050
  Cable. comenzarTransmisión ( 0x68 );
  // Enviar el registro inicial solicitado                                        
  Cable. escribir ( 0x1B );
  // Establece el registro inicial solicitado                                                    
  Cable. escribir ( 0x08 );
  // Finaliza la transmisión                                                  
  Cable. finalizar la transmisión ();

}


anular  read_mpu_6050_data (){

  // Leer los datos sin procesar del giroscopio y del acelerómetro

  // Comience a comunicarse con el MPU-6050                                          
  Cable. comenzarTransmisión ( 0x68 );  
  // Enviar el registro inicial solicitado                                      
  Cable. escribir ( 0x3B );
  // Finaliza la transmisión                                                    
  Cable. finalizar la transmisión ();
  // Solicita 14 bytes del MPU-6050                                  
  Cable. solicitud de ( 0x68 , 14 );    
  // Espera hasta que se reciban todos los bytes                                       
  while (Cable. disponible () < 14 );

  // Las siguientes declaraciones se desplazan a la izquierda 8 bits, luego bit a bit OR.  
  // Convierte dos valores de 8 bits en un valor de 16 bits                                       
  acc_x = Cable. leer ()<< 8 |Cable. leer ();                                  
  acc_y = Cable. leer ()<< 8 |Cable. leer ();                                  
  acc_z = Cable. leer ()<< 8 |Cable. leer ();                                  
  temperatura = Cable. leer ()<< 8 |Cable. leer ();                                   
  gyro_x = Cable. leer ()<< 8 |Cable. leer ();                                 
  gyro_y = Cable. leer ()<< 8 |Cable. leer ();                                 
  gyro_z = Cable. leer ()<< 8 |Cable. leer ();                                 
} // ----------------------------------------------- ----------------------------// ---------------------------------------------------------------------------
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
