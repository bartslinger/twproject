/*
  
This sketch demonstrates polling the inputs of a 74HC165 8-Bit Parallel-Load Shift Register
using the Arduino SPI interface.  You should connect pushbuttons or switches to the inputs
of the 74HC165.  Also connect an LED to Arduino D9 (aka pin 15) for testing.

If you don't use all of the inputs of the 74HC165, make sure to tie them to ground.
  
  
ARDUINO
=======

For pin mappings, see http://arduino.cc/en/Hacking/PinMapping168

                     ----_----
        vcc - [RST] |1      28| [A5]
               [RX] |2   A  27| [A4]
               [TX] |3   T  26| [A3]
               [D2] |4   M  25| [A2]
               [D3] |5   E  24| [A1]
               [D4] |6   G  23| [A0]
        vcc -  [+v] |7   A  22| [GND] - gnd
        gnd - [GND] |8      21| [AREF] - aref
   crystal - [XTL1] |9   3  20| [AVCC] - avcc
   crystal - [XTL2] |10  2  19| [D13] - SPI SCK  (clock) 
               [D5] |11  8  18| [D12] - SPI MISO (master in, slave out)
               [D6] |12     17| [D11] - SPI MOSI (master out, slave in)
               [D7] |13     16| [D10] - SPI SS   (slave select)
               [D8] |14     15| [D9]
                     ---------

Arduino Connections:

  Connect [D13] to the Clock pin of the SN74HC165 (pin 2)
  Connect [D12] to the Qh pin of the SN74HC165 (pin 9)
  Connect [D11] to nothing.  It's not used in this sketch
  Connect [D9]  to an LED for testing
  Connect [D6]  to the SH/LD pin of the SN74HC165 (pin 1)
  

SN74HC165 SHIFT REGISTER
========================

                                     ----_----
 Arduino [D6] (pin 12)  <-  SH/LD - |1      16| - Vcc      -> tied to +5 volts
Arduino [D13] (pin 19)  <-    CLK - |2   7  15| - CLK INH  -> tied to ground
                                E - |3   4  14| - D        
                                F - |4   H  13| - C
                                G - |5   C  12| - B
                                H - |6   1  11| - A
Arduino [D12] (pin 18)  <-     Qh - |7   6  10| - SER      -> unused, but tie to ground
                              gnd - |8   5   9| - Qh'      -> not connected
                                     ---------


Shift register connections:

  Connect inputs A,B,C,D,E,F,G and H to either pushbuttons or switches.
  Connect SH/LD (pin 1) to the Arduino [D6] digital output (pin 12)
  Connect CLK (pin 2) to the Arudino [D13] digital output (pin 19)
  Connect Qh (pin 7) to the Ardino [D12] digital input (pin 18)
  Connect CLK INH (pin 15) to ground
  Connect SER (pin 10) to ground

*/


// ==============================================================================================


// SPI pins
//
// Some of these pin definitions aren't actually ever used because the SPI library assumes these 
// pin configurations and assigns their pinMode for us.  However, I'll define them because SPI_ss 
// (slave select) can be repurposed once the Arduino has been set to master mode.  You cannot 
// reassign these pins and still expect SPI to work.

#define SPI_sck  13    //  Arduino [D13] - (ATMega 328 pin 19) - SPI clock
#define SPI_miso 12    //  Arduino [D12] - (ATMega 328 pin 18) - SPI master in, slave out
#define SPI_mosi 11    //  Arduino [D11] - (ATMega 328 pin 17) - SPI master out, slave in
#define SPI_ss   10    //  Arduino [D10] - (ATMega 328 pin 16) - SPI slave select


// Assorted digital outputs
#define led_output_pin          9 //  Arduino Pin D9 - (ATMega 328 pin 19) -> LED output for testing
#define PLSR_SH_LD_pin          6 //  Arduino Pin D6 - (ATMega 328 pin 12) -> SH/LD pin (pin 1) of 
                                  //                   the SN74HC165 Parallel-Load Shift Register


#include <SPI.h>
#include "Keyboard.h"

void setup(void)
{  
  delay(10000);
    // It is necessary to set the SPI_ss (slave select) as an output before initializing the SPI 
    // library so that the arduino acts as a master, not a slave.  Once the arduino has been set to 
    // master, this pin can be repurposed and used for anything - such as controlling the chip select 
    // on one of the SPI devices.
    
    pinMode(SPI_ss, OUTPUT);  

    // Set up the SPI bus
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);  
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV2);

    // Set pinmodes for the digital outputs
    pinMode(PLSR_SH_LD_pin, OUTPUT);
    pinMode(led_output_pin, OUTPUT);    

    Keyboard.begin();

    // Tell the SN74HC165 Parallel-load shift register to poll the inputs
    digitalWrite(PLSR_SH_LD_pin, LOW);
}


void loop(void)
{
    // Latch the inputs into the shift register
    digitalWrite(PLSR_SH_LD_pin, HIGH);    
    
    // Read in all 8 inputs of the SN74HC165 into a byte
    byte value = SPI.transfer(0x00);
    byte value2 = SPI.transfer(0x00);
    byte value3 = SPI.transfer(0x00);
    byte value4 = SPI.transfer(0x00);
    byte value5 = SPI.transfer(0x00);
    byte value6 = SPI.transfer(0x00);
    

    // Extract the individual input pin values from the byte
    boolean SN74HC165_pin_A = bitRead(value,0);  // get value of pin 11 of the SN74HC165
    boolean SN74HC165_pin_B = bitRead(value,1);  // get value of pin 12 of the SN74HC165
    boolean SN74HC165_pin_C = bitRead(value,2);  // get value of pin 13 of the SN74HC165
    boolean SN74HC165_pin_D = bitRead(value,3);  // get value of pin 14 of the SN74HC165
    boolean SN74HC165_pin_E = bitRead(value,4);  // get value of pin 3 of the SN74HC165
    boolean SN74HC165_pin_F = bitRead(value,5);  // get value of pin 4 of the SN74HC165
    boolean SN74HC165_pin_G = bitRead(value,6);  // get value of pin 5 of the SN74HC165
    boolean SN74HC165_pin_H = bitRead(value,7);  // get value of pin 6 of the SN74HC165
    
    // For testing, set an LED to the value found on input G (pin 5) of the SN74HC165
    digitalWrite(led_output_pin, SN74HC165_pin_G);

    if (value != 255 || value2 != 255 || value3 != 255 || value4 != 255 || value5 != 255 || value6 != 255) {
      for (int i = 0; i < 32+3+16+2; i++) {
        Keyboard.write(8);
      }
      Keyboard.print(value, BIN); 
      Keyboard.print(" ");
      Keyboard.print(value2, BIN);
      Keyboard.print(" ");
      Keyboard.print(value3, BIN);
      Keyboard.print(" ");
      Keyboard.print(value4, BIN);
      Keyboard.print(" ");
      Keyboard.print(value5, BIN);
      Keyboard.print(" ");
      Keyboard.print(value6, BIN);
    }
    delay(300);
//
//    if (!SN74HC165_pin_E) {
//      Keyboard.print(",");
//      delay(300);
//    }
//    if (!SN74HC165_pin_F) {
//      Keyboard.print("P");
//      delay(300);
//    }
//    if (!SN74HC165_pin_G) {
//      Keyboard.print("L");
//      delay(300);
//    }
//    if (!SN74HC165_pin_H) {
//      Keyboard.print(".");
//      delay(300);
//    }
    
    
    // Start polling the inputs again
    digitalWrite(PLSR_SH_LD_pin, LOW);
}
