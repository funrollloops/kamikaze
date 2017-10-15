#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif 
#include <Wire.h>  // Needed by cmake to generate the pressure sensor deps. (Gross!)

#include <stdio.h>

#include "stepper_controller.h"
#include "stepper_byj48.h"

#define TIMER1_INITIAL (UINT16_MAX - (16ul * 1000 * 1000)/256/(1000))

template <typename Pin>
class TimedOutput {
 public:
  void tick() {
    if (ticks_ == 0) return;
    if (--ticks_ == 0) pin_ = false;
  }

  void set_ticks(uint16_t ticks) {
    if (ticks > 0) pin_ = true;
    ticks_ = ticks;
  }

 private:
  Pin pin_;
  volatile uint16_t ticks_ = 0;
};

TimedOutput<OutputPin<13>> led;
StepperController<StepperBYJ48<4, 6, 5, 7>> stepper1;
StepperController<StepperBYJ48<9, 11, 8, 10>> stepper2;

ISR(TIMER1_OVF_vect) {
  TCNT1 = TIMER1_INITIAL;
  stepper1.tick();
  stepper2.tick();
}

void setup() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = TIMER1_INITIAL;
  TCCR1B |= (1 << CS12);  // set prescaler to 256.
  TIMSK1 |= (1 << TOIE1);
  interrupts();
  Serial.begin(9600);
}

bool received_command = false;

bool parseInt(const char *buf, size_t len, int16_t* value) {
  if (len == 0) return false;
  bool pos = true;
  const char *p = buf;
  if (len > 1 && buf[0] == '-') {
    ++p;
    pos = false;
  }
  *value = 0;
  for (char const *end = buf + len; p != end; ++p) {
    if (*p < '0' || *p > '9') {
      Serial.print("error parsing number ");
      Serial.write((const uint8_t*) buf, len);
      Serial.print('\n');
      return 0;
    }
    *value = *value * 10 + (*p - '0');
  }
  if (!pos) *value = -*value;
  return true;
}

bool exec_command(char *buf, size_t len) {
  struct __attribute__((packed)) PosPair { int16_t first, second; };
  struct __attribute__((packed)) Command {
    char hdr;
    union {
      char str[1];
      PosPair pos_pair;
      uint16_t u16;
    };
  };
  int16_t arg;
  received_command = true;
  if (len < 1) return false;
  const Command *cmd = reinterpret_cast<const Command*>(buf);
  switch (cmd->hdr) {
    case 'h':
      if (parseInt(cmd->str, len - 1, &arg)) stepper1.moveTo(arg);
      else Serial.println(stepper1.tell());
      return true;
    case 'v':
      if (parseInt(cmd->str, len - 1, &arg)) stepper2.moveTo(arg);
      else Serial.println(stepper2.tell());
      return true;
    case 't': {
      PosPair pos{stepper1.tell(), stepper2.tell()};
      Serial.write(reinterpret_cast<const uint8_t*>(&pos), sizeof(pos));
      Serial.print('\n');
      return true;
    }
    case 'm':
      if (len != sizeof(PosPair) + 1) return false;
      stepper1.moveTo(cmd->pos_pair.first);
      stepper2.moveTo(cmd->pos_pair.second);
      return true;
    case 'f':
      if (len != sizeof(uint16_t) + 1) return false;
      led.set_ticks(cmd->u16);
      return true;
  }
  return false;
}

void serialEvent() {
  static char buffer[7];
  static size_t received = 0;
  while (Serial.available()) {
    int byte = Serial.read();
    if (byte == '\n') {
      if (!exec_command(buffer, received)) {
        Serial.print("unknown command: ");
        Serial.write((const uint8_t*) buffer, received);
        Serial.print('\n');
      }
      received = 0;
    } else if (received == sizeof(buffer)) {
      Serial.print("command too long: ");
      Serial.write((const uint8_t*) buffer, received);
      Serial.print('\n');
      received = 0;
    } else {
      buffer[received++] = byte;
    }
  }
}

void loop() { }
