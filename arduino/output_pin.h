// Requires Arduinio files have already been included.

#ifndef FUNROLLLOOPS_ARDUINO_OUTPUT_PIN_H_
#define FUNROLLLOOPS_ARDUINO_OUTPUT_PIN_H_

#include <avr/io.h>

template <int kPin> class OutputPin {
public:
  OutputPin() { pinMode(kPin, OUTPUT); }
  bool operator=(bool value) { digitalWrite(kPin, value); return value; }
};

#define OUTPUT_PORT(suffix) \
  template <int kPin> class Port##suffix##OutputPin { \
    public: \
      Port##suffix##OutputPin() { DDR##suffix |= _BV(kPin); } \
      void on() { PORT##suffix |= _BV(kPin); } \
      void off() { PORT##suffix &= ~_BV(kPin); } \
      bool operator=(bool value) { value ? on() : off(); return value; } \
    }

OUTPUT_PORT(B);
OUTPUT_PORT(C);
OUTPUT_PORT(D);
#undef OUTPUT_PORT

#endif
