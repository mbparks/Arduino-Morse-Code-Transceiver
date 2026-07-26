#ifndef EEPROM_H_SHIM
#define EEPROM_H_SHIM
#include <stdint.h>
#include <string.h>
// Host shim: a plain byte array standing in for the EEPROM cells.
struct EEPROMShim {
  uint8_t cells[64];
  EEPROMShim() { memset(cells, 0xFF, sizeof(cells)); }
  uint8_t read(int a) { return (a >= 0 && a < 64) ? cells[a] : 0xFF; }
  void write(int a, uint8_t v) { if (a >= 0 && a < 64) cells[a] = v; }
  void update(int a, uint8_t v) { write(a, v); }
};
#endif
