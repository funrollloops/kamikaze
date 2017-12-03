#include <avr/cpufunc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdio.h>

#include "stepper_controller.h"
#include "stepper_byj48.h"

#define CLOCK_RATE F_CPU
#define TIMER2_PRESCALER 64
#define TIMER2_INTERRUPTS_PER_SEC 1000
// Compute number of timer ticks between interrupts.
#define TIMER2_INITIAL \
    (UINT8_MAX - CLOCK_RATE/TIMER2_PRESCALER/TIMER2_INTERRUPTS_PER_SEC)

#ifndef A0
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#endif

// Attached peripherals, with configuration.
TimedOutput<PortCOutputPin<PC0>> fire_led;
TimedOutput<PortCOutputPin<PC1>> reset_led;
StepperController<StepperBYJ48<A5, A3, A4, A2>> stepper1;
StepperController<StepperBYJ48<2, 0, 3, 1>> stepper2;

struct __attribute__((packed)) PosPair { int16_t first, second; };

EMPTY_INTERRUPT(BADISR_vect)

ISR(TIMER2_OVF_vect) {
  TCNT2 = TIMER2_INITIAL;
  stepper1.tick();
  stepper2.tick();
  fire_led.tick();
  reset_led.tick();
}

// Simple SPI protocol implementation: cmd byte followed by spacer/data bytes.
// Unrecognized commands ignored. For simplicity master should send 6-byte
// commands, with zero padding.
enum Cmd : uint8_t {
  NONE = 0,
  TELL = 1,
  SEEK = 2,
  FIRE = 3,
};

Cmd cmd = NONE;
uint8_t cmd_pos = 0;
uint16_t buf_u16 = 2;
PosPair buf_pair{0, 0};

ISR (SPI_STC_vect) {
  switch(cmd) {
    case TELL:  // Master must send 5 dummy bytes.
      SPDR = reinterpret_cast<uint8_t*>(&buf_pair)[cmd_pos++];
      if (cmd_pos >= sizeof(buf_pair)) {
        cmd = NONE;
      }
      return;
    case SEEK:
      reinterpret_cast<uint8_t*>(&buf_pair)[cmd_pos++] = SPDR;
      if (cmd_pos >= sizeof(buf_pair)) {
        stepper1.moveTo(buf_pair.first);
        stepper2.moveTo(buf_pair.second);
        cmd = NONE;
      }
      return;
    case FIRE:
      reinterpret_cast<uint8_t*>(&buf_u16)[cmd_pos++] = SPDR;
      if (cmd_pos >= sizeof(buf_u16)) {
        fire_led.set_ticks(buf_u16);
        cmd = NONE;
      }
      return;
    default:
      cmd = Cmd(SPDR);
      cmd_pos = 0;
      switch (cmd) {
        case TELL:
          buf_pair = {stepper1.tell(), stepper2.tell()};
        case SEEK:
        case FIRE:
          break;
        default:
          cmd = NONE;
          SPDR = 0xee;
      }
      return;
  }
}

void setup() {
  cli();  // Disable interrupts.
  // Set clock divisor to 1 to run at full 8MHz.
  CLKPR = 0x80;  // Enable setting clock divisor.
  _NOP();        // Ensure instructions are not re-ordered.
  CLKPR = 0x00;  // Set divisor to 1.

  // Configure TIMER2 to interrupt when TCNT2 reaches UINT8_MAX.
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2B |= _BV(CS22);  // set prescaler to 64.
  TCNT2 = TIMER2_INITIAL;
  TIMSK2 |= _BV(TOIE2);

  // Enable SPI for read and write.
  DDRB |= _BV(4);  // Set MISO pin to output.
  SPCR |= _BV(SPE);  // Enable SPI in slave mode.
  SPCR |= _BV(SPIE);  // Enable SPI interrupts.

  // Put status register into SPDR on reset to help debug.
  SPDR = MCUSR;
  MCUSR = 0;
  sei(); // Enable interrups.

  reset_led.set_ticks(TIMER2_INTERRUPTS_PER_SEC / 10); // 100ms.
}

int main() {
  setup();

  while (true) {
    sleep_mode();
  }
}
