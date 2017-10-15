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

OutputPin<13> led;
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

bool parseInt(char *buf, size_t len, int16_t* value) {
  if (len == 0) return false;
  bool pos = true;
  char *p = buf;
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

void exec_command(char *buf, size_t len) {
  struct PosPair { int16_t first, second; };
  int16_t arg;
  received_command = true;
  if (len < 1) return;
  switch (buf[0]) {
    case 'h': {
      if (parseInt(buf + 1, len - 1, &arg)) stepper1.moveTo(arg);
      else Serial.println(stepper1.tell());
      break;
    }
    case 'v': {
      if (parseInt(buf + 1, len - 1, &arg)) stepper2.moveTo(arg);
      else Serial.println(stepper2.tell());
      break;
    }
    case 't': {
      PosPair pos{stepper1.tell(), stepper2.tell()};
      Serial.write(reinterpret_cast<const uint8_t*>(&pos), sizeof(pos));
      Serial.print('\n');
      break;
    }
    case 'm': {
      if (len == sizeof(PosPair) + 1) {
        auto *arg = reinterpret_cast<const PosPair*>(buf + 1);
        stepper1.moveTo(arg->first);
        stepper2.moveTo(arg->second);
        break;
      }
      // FALLTHROUGH_INTENDED;
    }
    default:
      Serial.print("unknown command: ");
      Serial.write((const uint8_t*) buf, len);
      Serial.print('\n');
  }
}

void serialEvent() {
  static char buffer[6];
  static size_t received = 0;
  while (Serial.available()) {
    int byte = Serial.read();
    if (byte == '\n' || received == sizeof(buffer)) {
      exec_command(buffer, received);
      received = 0;
    } else {
      buffer[received++] = byte;
    }
  }
}


void test_steppers() {
  Serial.write("test loop");
  const uint16_t home = stepper1.tell();
  while(!Serial.available()) {
    led = 0;
    stepper1.moveTo(home - 4096);
    stepper2.moveTo(home + 2048);
    stepper1.wait(); stepper2.wait();
    led = 1;
    stepper1.moveTo(home);
    stepper2.moveTo(home);
    stepper1.wait(); stepper2.wait();
    for (int i = 0; i < 20; ++i) {
      led = i & 1;
      delay(2000 / 20);
    }
  }
  while(true) serialEvent();
}

void loop() {
  //serialEvent();
}
