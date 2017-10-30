#include <avr/cpufunc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdio.h>

#include "stepper_controller.h"
#include "stepper_byj48.h"

#define CLOCK_RATE F_CPU
#define TIMER1_PRESCALER 256
#define TIMER1_INTERRUPTS_PER_SEC 1000
// Compute number of timer ticks between interrupts.
#define TIMER1_INITIAL \
    (UINT16_MAX - CLOCK_RATE/TIMER1_PRESCALER/TIMER1_INTERRUPTS_PER_SEC)

// Attached peripherals, with configuration.
TimedOutput<PortCOutputPin<PC0>> fire_led;
TimedOutput<PortCOutputPin<PC1>> reset_led;
StepperController<StepperBYJ48<4, 6, 5, 7>> stepper1;
StepperController<StepperBYJ48<1, 3, 0, 2>> stepper2;

struct __attribute__((packed)) PosPair { int16_t first, second; };

EMPTY_INTERRUPT(BADISR_vect)

ISR(TIMER1_OVF_vect) {
  TCNT1 = TIMER1_INITIAL;
  stepper1.tick();
  stepper2.tick();
  fire_led.tick();
  reset_led.tick();
}

// Simple SPI protocol implementation: cmd byte followed by spacer/data bytes.
// Unrecognized commands ignored. For simplicity master should send 6-byte
// commands, with zero padding.
enum Cmd {
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
  const uint8_t c = SPDR;  // grab byte from SPI Data Register
  switch(cmd) {
    case NONE:
      switch (c) {
        case TELL:
          buf_pair.first = stepper1.tell();
          buf_pair.second = stepper2.tell();
        case SEEK:
        case FIRE:
          cmd = Cmd(c);
          cmd_pos = 0;
          break;
        default:
          SPDR = 0xee;
      }
      return;
    case TELL:  // Master must send 5 dummy bytes.
      SPDR = reinterpret_cast<uint8_t*>(&buf_pair)[cmd_pos];
      if (cmd_pos >= sizeof(buf_pair) - 1) {
        cmd = NONE;
        return;
      }
      break;
    case SEEK:
      reinterpret_cast<uint8_t*>(&buf_pair)[cmd_pos] = c;
      if (cmd_pos >= sizeof(buf_pair) - 1) {
        stepper1.moveTo(buf_pair.first);
        stepper2.moveTo(buf_pair.second);
        cmd = NONE;
        return;
      }
      break;
    case FIRE:
      reinterpret_cast<uint8_t*>(&buf_u16)[cmd_pos] = c;
      if (cmd_pos >= sizeof(buf_u16) - 1) {
        fire_led.set_ticks(buf_u16);
        cmd = NONE;
        return;
      }
      break;
  }
  ++cmd_pos;
}

void setup() {
  cli();  // Disable interrupts.
  // Set clock divisor to 1 to run at full 8MHz.
  CLKPR = 0x80;  // Enable setting clock divisor.
  _NOP();        // Ensure instructions are not re-ordered.
  CLKPR = 0x00;  // Set divisor to 1.
  // Configure TIMER1 to interrupt when TCNT1 reaches UINT16_MAX.
  // TCNT1 will be incremented every 1/256 clock cycles, or 16ns.
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = TIMER1_INITIAL;
  TCCR1B |= (1 << CS12);  // set prescaler to 256.
  TIMSK1 |= (1 << TOIE1);

  // Enable SPI for read and write.
  DDRB |= _BV(4);  // Enable outputting to master.
  SPCR |= _BV(SPE);  // Enable SPI in slave mode.
  SPCR |= _BV(SPIE);  // Enable SPI interrupts.

  SPDR = MCUSR;  // Put status register into SPDR on reset.
  sei(); // Enable interrups.

  reset_led.set_ticks(TIMER1_INTERRUPTS_PER_SEC / 10); // 100ms.
}

int main() {
  setup();

  while (true) {
    sleep_mode();
  }
}
