#ifndef ARDUINO_H_SHIM
#define ARDUINO_H_SHIM
// Minimal Arduino shim for host-side verification of the sketch.
// Not part of the deliverable. Everything routes through a per-station context
// so two simulated boards can run the same sketch in one process.

#include <stdint.h>
#include <stddef.h>
#include <string>
#include <deque>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define LED_BUILTIN 13
#define A0 14

#define PROGMEM
#define PSTR(s) (s)
#define F(s) (s)
static inline uint8_t pgm_read_byte(const void *p) { return *(const uint8_t *)p; }
static inline uint16_t pgm_read_word(const void *p) { return *(const uint16_t *)p; }
static inline uint8_t pgm_read_byte(const char *p) { return (uint8_t)*p; }

struct StationCtx {
  uint32_t ms = 0;
  uint32_t us = 0;
  uint32_t usFrac = 0;
  int adc = 0;
  bool ledTx = false;
  bool ledStatus = false;
  bool buttonDown = false;
  bool keyDown = false;
  bool toneOn = false;
  unsigned toneHz = 0;
  std::deque<char> serialIn;
  std::string serialOut;
};

extern StationCtx *g_ctx;

static inline unsigned long millis() { return g_ctx->ms; }
static inline unsigned long micros() { return g_ctx->us; }
static inline void pinMode(uint8_t, uint8_t) {}
static inline void digitalWrite(uint8_t pin, uint8_t val) {
  if (pin == 3) g_ctx->ledTx = (val == HIGH);
  else if (pin == LED_BUILTIN) g_ctx->ledStatus = (val == HIGH);
}
static inline int digitalRead(uint8_t pin) {
  if (pin == 2) return g_ctx->buttonDown ? LOW : HIGH;
  if (pin == 4) return g_ctx->keyDown ? LOW : HIGH;
  return HIGH;
}
static inline void tone(uint8_t, unsigned f) { g_ctx->toneOn = true; g_ctx->toneHz = f; }
static inline void noTone(uint8_t) { g_ctx->toneOn = false; }
static inline int analogRead(uint8_t) { return g_ctx->adc; }

class SerialShim {
public:
  void begin(unsigned long) {}
  int available() { return (int)g_ctx->serialIn.size(); }
  int read() {
    if (g_ctx->serialIn.empty()) return -1;
    char c = g_ctx->serialIn.front();
    g_ctx->serialIn.pop_front();
    return (int)(unsigned char)c;
  }
  int availableForWrite() { return 64; }
  size_t write(uint8_t b) { g_ctx->serialOut.push_back((char)b); return 1; }
};
extern SerialShim Serial;

#endif
