// Requires Arduinio files have already been included.

#ifndef FUNROLLLOOPS_ARDUINO_OUTPUT_PIN_H_
#define FUNROLLLOOPS_ARDUINO_OUTPUT_PIN_H_

#include <avr/io.h>

// Using Arduino Uno pin numbering.
template <uint8_t kPin> class OutputPin {
public:
  OutputPin() {
    static_assert(kPin < 20, "Arduino numbering only goes to 14 + A0-6");
    if (kPin < 8) DDRD |= _BV(kPin);
    else if (kPin < 14) DDRB |= _BV(kPin - 8);
    else DDRC |= _BV(kPin - 14);
  }

  static void on() {
    if (kPin < 8) PORTD |= _BV(kPin);
    else if (kPin < 14) PORTB |= _BV(kPin - 8);
    else PORTC |= _BV(kPin - 14);
  }

  static void off() {
    if (kPin < 8) PORTD &= ~_BV(kPin);
    else if (kPin < 14) PORTB &= ~_BV(kPin - 8);
    else PORTC &= ~_BV(kPin - 14);
  }

  bool operator=(bool value) {
    value ? on() : off();
    return value;
  }
};

// Using native AVR names.
#define OUTPUT_PORT(suffix) \
  template <int kPin> class Port##suffix##OutputPin { \
    public: \
      Port##suffix##OutputPin() { DDR##suffix |= _BV(kPin); } \
      static void on() { PORT##suffix |= _BV(kPin); } \
      static void off() { PORT##suffix &= ~_BV(kPin); } \
      bool operator=(bool value) { value ? on() : off(); return value; } \
    }

OUTPUT_PORT(B);
OUTPUT_PORT(C);
OUTPUT_PORT(D);
#undef OUTPUT_PORT

template <typename Pin>
class TimedOutput {
 public:
  TimedOutput() { pin_.off(); }

  // To avoid races, tick() and set_ticks() should not called
  // with interrupts enabled.
  void tick() {
    if (ticks_ == 0) return;
    if (--ticks_ == 0) pin_.off();
  }

  void set_ticks(uint16_t ticks) {
    if (ticks > 0) pin_.on();
    ticks_ = ticks;
  }

 private:
  Pin pin_;
  volatile uint16_t ticks_ = 0;
};

#endif
