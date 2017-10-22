// Requires Arduinio files have already been included.

#ifndef FUNROLLLOOPS_ARDUINO_OUTPUT_PIN_H_
#define FUNROLLLOOPS_ARDUINO_OUTPUT_PIN_H_

template <int kPin> class OutputPin {
public:
  OutputPin() { pinMode(kPin, OUTPUT); }
  bool operator=(bool value) { digitalWrite(kPin, value); return value; }
};

#endif
