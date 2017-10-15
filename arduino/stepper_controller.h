#ifndef FUNROLLLOOPS_ARDUINO_STEPPER_CONTROLLER_H_
#define FUNROLLLOOPS_ARDUINO_STEPPER_CONTROLLER_H_

#include <stdint.h>

template <typename Stepper>
class StepperController {
 public:
   StepperController() { stepper_.off(); }

   void tick() {
     auto pos = pos_;
     if (target_ > pos) {
       stepper_.forward(); pos_ = pos + 1;
     } else if (target_ < pos) {
       stepper_.back(); pos_ = pos - 1;
     } else if (on_) {
       stepper_.off();
       on_ = false;
     }
   }

   void moveTo(int16_t target) { on_ = true; target_ = target; }
   int16_t tell() const { return pos_; }
   int16_t target() const { return target_; }
   void wait() const {
     while (pos_ != target_) delay(1);
   }

 private:
  Stepper stepper_;
  volatile int16_t pos_ = 0;
  int16_t target_ = pos_;
  bool on_ = false;
};

#endif
