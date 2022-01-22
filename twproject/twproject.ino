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
//
//const char key_mappings[48] = {
//  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
//  'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
//  'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
//  'Y', 'Z', '1', '2', '3', '4', '5', '6',
//  '7', '8', '9', '0', '?', '!', '@', '#',
//  '$', '%', '^', '&', '*', '(', ')', '-'};

// nr 2 = backspace

const char key_mappings[48] = {
  '-', 0xB0, ':', 0xB2, 'E', 'f', 'G', 0x20,
  ',', 'o', '\'', 'l', '.', 'p', ';', '`',
  'n', 'u', 'j', '8', 'm', 'i', 'k', '9',
  'v', 't', 'g', '6', 'b', 'y', 'h', '7',
  'x', 'e', 'd', '4', 'c', 'r', 'f', '5',
  '1', 'q', 'a', '2', 'z', 'w', 's', '3'
};

const uint8_t PRESSABLE_KEYS = 48;

byte registers[6] = {0, 0, 0, 0, 0, 0};
uint8_t press_counter[48] = {0};

// Tunables
const uint8_t MAX_CNT_VAL = 15;
const uint8_t LOOP_DELAY_MS = 10;

void setup(void)
{
  // Delay so we can upload new code before keyboard is taking over
  delay(3000);

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

void read_shift_registers(void) {
  // Latch the inputs into the shift registers
  digitalWrite(PLSR_SH_LD_pin, HIGH);

  // Read in all 8 inputs of the SN74HC165 into a byte
  for (uint8_t i = 0; i < 6; i++) {
    registers[i] = SPI.transfer(0x00);
  }

  // Unlatch shift registers
  digitalWrite(PLSR_SH_LD_pin, LOW);
}

void handle_key_measurement(uint8_t idx, uint8_t key_value) {

  // Key can have two states, IDLE and PRESSED
  // When press_counter is zero, it is in IDLE state
  // Otherwise, it is in PRESSED state

  // A key stroke is registered when the value is positive while
  // in IDLE state. The state then transitions to PRESSED.
  // The state is only reset to IDLE when the value has been 0
  // for MAX_CNT_VAL times.

  if (press_counter[idx] == 0) {
    // State IDLE
    if (key_value) {
      if (idx == 40) {
        Keyboard.print("3/4");
      } else {
        Keyboard.write(key_mappings[idx]);
      }
      press_counter[idx] = MAX_CNT_VAL;
      // Effectively transitioning to PRESSED state now.
    }
  } else {
    // State PRESSED
    if (key_value) {
      press_counter[idx] = MAX_CNT_VAL;
    } else {
      press_counter[idx] = press_counter[idx] - 1;
      // Effectively transitioning to IDLE when it reaches 0.
    }
  }
}

void loop(void)
{

  read_shift_registers();

  for (uint8_t i = 0; i < PRESSABLE_KEYS; i++) {
    // Get the measurement
    uint8_t b = i % 8;
    uint8_t reg = i / 8;
    uint8_t value = !bitRead(registers[reg], b);
    handle_key_measurement(i, value);
  }

  delay(LOOP_DELAY_MS);

}
