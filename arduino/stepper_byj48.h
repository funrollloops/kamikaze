#ifndef FUNROLLLOOPS_ARDUINO_STEPPER_BYJ48_H_
#define FUNROLLLOOPS_ARDUINO_STEPPER_BYJ48_H_

#include "output_pin.h"

template <int kA1, int kA2, int kB1, int kB2> class StepperBYJ48 {
 public:
  static constexpr auto ON = LOW;
  static constexpr auto OFF = HIGH;

  StepperBYJ48() {
    pinA1 = ON;
    pinA2 = OFF;
    pinB1 = OFF;
    pinB2 = OFF;
  }

  void forward() {
    switch (step_++ & 0x7) {
      case 0: pinB2 = OFF; break;
      case 1: pinB1 = ON; break;
      case 2: pinA1 = OFF; break;
      case 3: pinA2 = ON; break;
      case 4: pinB1 = OFF; break;
      case 5: pinB2 = ON; break;
      case 6: pinA2 = OFF; break;
      case 7: pinA1 = ON; break;
    }
  }

  void back() { // This is <forward> with ON/OFF and pre/post inc/dec swapped.
    switch (--step_ & 0x7) {
      case 0: pinB2 = ON; break;
      case 1: pinB1 = OFF; break;
      case 2: pinA1 = ON; break;
      case 3: pinA2 = OFF; break;
      case 4: pinB1 = ON; break;
      case 5: pinB2 = OFF; break;
      case 6: pinA2 = ON; break;
      case 7: pinA1 = OFF; break;
    }
  }

  void off() {
    pinA1 = pinA2 = pinB1 = pinB2 = ON;
  }

 private:
  OutputPin<kA1> pinA1;
  OutputPin<kA2> pinA2;
  OutputPin<kB1> pinB1;
  OutputPin<kB2> pinB2;
  int step_ = 0;
};

#endif
