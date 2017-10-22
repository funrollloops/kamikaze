#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif 
#include <Wire.h>

#define LED A0
#define LED2 A1

enum Cmd {
  NONE = 0,
  TELL = 1,
  SEEK = 2,
  FIRE = 3,
};

Cmd cmd = NONE;
uint8_t cmd_pos = 0;
volatile uint16_t led_ms = 2;
struct {
  uint16_t x;
  uint16_t y;
} pos = {0xdead, 0xbeef};


void setup() {
  pinMode(LED, OUTPUT);
  pinMode(LED2, OUTPUT);

  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);  // Enable SPI in slave mode.
  SPCR |= _BV(SPIE);  // Enable SPI interrupts.
}

// Simple protocol implementation: cmd byte followed by spacer/data bytes.
// Unrecognized commands ignored. For simplicity master should send 6-byte
// commands, with zero padding.
ISR (SPI_STC_vect) {
  byte c = SPDR;  // grab byte from SPI Data Register
  switch(cmd) {
    case NONE:
      if (c == TELL || c == SEEK || c == FIRE) {
        cmd = Cmd(c);
        cmd_pos = 0;
      }
      return;
    case TELL:  // Master must send 5 dummy bytes.
      SPDR = reinterpret_cast<volatile byte*>(&pos)[cmd_pos];
      if (cmd_pos >= sizeof(pos) - 1) cmd = NONE;
      break;
    case SEEK:
      reinterpret_cast<volatile byte*>(&pos)[cmd_pos] = c;
      if (cmd_pos >= sizeof(pos) - 1) cmd = NONE;
      break;
    case FIRE:
      reinterpret_cast<volatile byte*>(&led_ms)[cmd_pos] = c;
      if (cmd_pos >= sizeof(led_ms) - 1) cmd = NONE;
      break;
  }
  ++cmd_pos;
}

bool led2=true;
void loop() {
   digitalWrite(LED, led_ms > 0);
   if (led_ms > 0) --led_ms;
   delay(500);
   digitalWrite(LED2, led2=!led2);
}
