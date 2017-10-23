#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif 
#include <Wire.h>  // Needed by cmake to generate the pressure sensor deps. (Gross!)

#include <stdio.h>

#include "stepper_controller.h"
#include "stepper_byj48.h"

#define CLOCK_RATE (16ul * 1000* 1000)
#define TIMER1_PRESCALER 256
#define TIMER1_INTERRUPTS_PER_SEC 1000
#define TIMER1_INITIAL \
    (UINT16_MAX - CLOCK_RATE/TIMER1_PRESCALER/TIMER1_INTERRUPTS_PER_SEC)

#define ENABLE_SERIAL 0

template <typename Pin>
class TimedOutput {
 public:
  TimedOutput() { pin_ = false; }

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

TimedOutput<OutputPin<A0>> led;
StepperController<StepperBYJ48<4, 6, 5, 7>> stepper1;
StepperController<StepperBYJ48<1, 3, 0, 2>> stepper2;

struct __attribute__((packed)) PosPair { int16_t first, second; };

ISR(TIMER1_OVF_vect) {
  TCNT1 = TIMER1_INITIAL;
  stepper1.tick();
  stepper2.tick();
  led.tick();
}

void setup() {
  noInterrupts();
  // Configure TIMER1 to interrupt when TCNT1 reaches UINT16_MAX.
  // TCNT1 will be incremented every 1/256 clock cycles, or 16ns.
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = TIMER1_INITIAL;
  TCCR1B |= (1 << CS12);  // set prescaler to 256.
  TIMSK1 |= (1 << TOIE1);

  // Enable SPI for read and write.
  pinMode(MISO, OUTPUT);  // Enable outputting to master.
  SPCR |= _BV(SPE);  // Enable SPI in slave mode.
  SPCR |= _BV(SPIE);  // Enable SPI interrupts.
  interrupts();
#if ENABLE_SERIAL
  Serial.begin(9600);
#endif
}

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
  struct __attribute__((packed)) Command {
    char hdr;
    union {
      char str[6];
      PosPair pos_pair;
      uint16_t u16;
    };
  };
  int16_t arg;
  if (len == 0) return true;  // Ignore empty commands.
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
  static char buffer[8];
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
  const byte c = SPDR;  // grab byte from SPI Data Register
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
      SPDR = reinterpret_cast<volatile byte*>(&buf_pair)[cmd_pos];
      if (cmd_pos >= sizeof(buf_pair) - 1) {
        cmd = NONE;
        return;
      }
      break;
    case SEEK:
      reinterpret_cast<volatile byte*>(&buf_pair)[cmd_pos] = c;
      if (cmd_pos >= sizeof(buf_pair) - 1) {
        stepper1.moveTo(buf_pair.first);
        stepper2.moveTo(buf_pair.second);
        cmd = NONE;
        return;
      }
      break;
    case FIRE:
      reinterpret_cast<volatile byte*>(&buf_u16)[cmd_pos] = c;
      if (cmd_pos >= sizeof(buf_u16) - 1) {
        led.set_ticks(buf_u16);
        cmd = NONE;
        return;
      }
      break;
  }
  ++cmd_pos;
}

void loop() { }
