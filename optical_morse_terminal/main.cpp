// Host verification harness for optical_morse_terminal.ino
// Not part of the deliverable. Compiles the sketch twice, once per namespace,
// so two virtual boards can be pointed at each other.

#define MORSE_HAS_EEPROM 1
#include "Arduino.h"
#include "EEPROM.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <iostream>
#include <cmath>

StationCtx *g_ctx = nullptr;
SerialShim Serial;

namespace A {
static EEPROMShim EEPROM;
#include "../optical_morse_terminal.ino"
}
namespace B {
static EEPROMShim EEPROM;
#include "../optical_morse_terminal.ino"
}

static StationCtx ctxA, ctxB;

// Optical model: a fixed ambient floor, plus the far LED seen across the gap,
// plus whatever the local LED bleeds into the local sensor. A first order lag
// stands in for CdS response time.
static int AMBIENT = 120;
static double rippleAmp = 0.0;   // simulated mains flicker, ADC counts
static double simClockMs = 0.0;
static const int FAR_GAIN = 220;
static double lagA = AMBIENT, lagB = AMBIENT;
static int selfCouplingA = 250;
static int selfCouplingB = 250;

static void feed(StationCtx &c, const char *s) {
  for (const char *p = s; *p; p++) c.serialIn.push_back(*p);
}

static void stepBoth(unsigned long stepUs) {
  // Target levels
  double targetA = AMBIENT + (ctxB.ledTx ? FAR_GAIN : 0) + (ctxA.ledTx ? selfCouplingA : 0);
  double targetB = AMBIENT + (ctxA.ledTx ? FAR_GAIN : 0) + (ctxB.ledTx ? selfCouplingB : 0);
  // tau = 4 ms
  double alpha = (double)stepUs / 4000.0;
  if (alpha > 1.0) alpha = 1.0;
  lagA += (targetA - lagA) * alpha;
  lagB += (targetB - lagB) * alpha;

  simClockMs += (double)stepUs / 1000.0;
  double ripple = rippleAmp * sin(2.0 * 3.14159265 * 120.0 * simClockMs / 1000.0);

  // ms and us are advanced independently so each can be started near its own
  // 32 bit wrap point and roll over during a live message.
  for (StationCtx *c : {&ctxA, &ctxB}) {
    c->us += (uint32_t)stepUs;
    c->usFrac += (uint32_t)stepUs;
    while (c->usFrac >= 1000) { c->usFrac -= 1000; c->ms += 1; }
  }
  ctxA.adc = (int)(lagA + ripple + 0.5);
  ctxB.adc = (int)(lagB + ripple + 0.5);

  g_ctx = &ctxA; A::loop();
  g_ctx = &ctxB; B::loop();
}

static void run(unsigned long ms) {
  for (unsigned long i = 0; i < ms * 4; i++) stepBoth(250);
}

static bool contains(const std::string &hay, const char *needle) {
  return hay.find(needle) != std::string::npos;
}

static int failures = 0;
static void check(bool ok, const char *what) {
  if (!ok) { failures++; printf("HARNESS FAIL: %s\n", what); }
  else printf("HARNESS ok  : %s\n", what);
}

int main(int argc, char **argv) {
  bool nearWrap = (argc > 1 && std::string(argv[1]) == "wrap");
  if (nearWrap) {
    // Park both clocks a few seconds short of their 32 bit wrap points.
    ctxA.ms = ctxB.ms = 0xFFFFFFFFUL - 4000UL;
    ctxA.us = ctxB.us = 0xFFFFFFFFUL - 2000000UL;
    printf("HARNESS: starting with millis() %lu ms from wrap\n",
           (unsigned long)(0xFFFFFFFFUL - ctxA.ms));
  }
  g_ctx = &ctxA; A::setup();
  g_ctx = &ctxB; B::setup();

  run(2000);  // boot settle plus margin
  check(contains(ctxA.serialOut, "SYS: boot_cal"), "A completed boot calibration");
  check(contains(ctxA.serialOut, "receiver=warming"), "A reported warming at boot");

  // ---- built in self test ----
  ctxA.serialOut.clear();
  feed(ctxA, "/loop\n");
  run(200);
  printf("\n--- A /loop output ---\n%s\n", ctxA.serialOut.c_str());
  check(contains(ctxA.serialOut, "result=GREEN"), "self test green");
  check(!contains(ctxA.serialOut, "selftest FAIL"), "no self test failures");

  // ---- calibration on both stations ----
  ctxA.serialOut.clear(); ctxB.serialOut.clear();
  feed(ctxA, "/cal\n");
  run(1600);
  printf("--- A /cal output ---\n%s\n", ctxA.serialOut.c_str());
  check(contains(ctxA.serialOut, "cal_done"), "A calibration succeeded");
  check(contains(ctxA.serialOut, "receiver=armed"), "A receiver armed");

  ctxB.serialOut.clear();
  feed(ctxB, "/cal\n");
  run(1600);
  check(contains(ctxB.serialOut, "cal_done"), "B calibration succeeded");

  // ---- link test: A transmits, B receives ----
  ctxA.serialOut.clear(); ctxB.serialOut.clear();
  feed(ctxA, "/wpm 15\n");
  feed(ctxA, "/echo on\n");
  run(50);
  feed(ctxA, "HELLO WORLD DE N1HNP +\n");
  run(30000);
  printf("--- A console ---\n%s\n", ctxA.serialOut.c_str());
  printf("--- B console ---\n%s\n", ctxB.serialOut.c_str());
  check(contains(ctxB.serialOut, "HELLO WORLD DE N1HNP +"), "B decoded A's message");
  check(contains(ctxB.serialOut, "SYS: rx_end"), "B emitted an end of message line");
  check(!contains(ctxA.serialOut, "RX:"), "A did not decode its own transmission (half duplex gate)");
  check(contains(ctxA.serialOut, "TX+ HELLO WORLD DE N1HNP +"), "A local echo round tripped");
  check(contains(ctxA.serialOut, "TX: done"), "A reported completion");

  // ---- receive at a different speed than the local setting ----
  ctxB.serialOut.clear(); ctxA.serialOut.clear();
  feed(ctxB, "/wpm 8\n");
  run(50);
  feed(ctxA, "/wpm 22\n");
  run(50);
  feed(ctxA, "CQ TEST 123\n");
  run(30000);
  printf("--- B console (speed lock) ---\n%s\n", ctxB.serialOut.c_str());
  check(contains(ctxB.serialOut, "TEST 123"), "B locked onto a faster sender");

  // ---- punctuation and prosigns ----
  ctxB.serialOut.clear();
  feed(ctxA, "OK. FB? / = * #\n");
  run(40000);
  printf("--- B console (punctuation) ---\n%s\n", ctxB.serialOut.c_str());
  check(contains(ctxB.serialOut, "OK. FB? / = * #"), "punctuation and prosigns survive the round trip");

  // ---- unencodable character reported, not swallowed ----
  ctxA.serialOut.clear();
  feed(ctxA, "A~B\n");
  run(8000);
  check(contains(ctxA.serialOut, "ERR: TXDROP"), "unencodable character reported");

  // ---- carrier sense defers a transmission while the channel is busy ----
  ctxA.serialOut.clear(); ctxB.serialOut.clear();
  feed(ctxA, "TEST\n");
  run(600);
  feed(ctxB, "X\n");
  run(200);
  printf("--- B console (defer) ---\n%s\n", ctxB.serialOut.c_str());
  check(contains(ctxB.serialOut, "tx_deferred"), "B deferred while A was sending");
  run(30000);

  // ---- /clear aborts cleanly ----
  ctxA.serialOut.clear();
  feed(ctxA, "PARIS PARIS PARIS\n");
  run(2000);
  feed(ctxA, "/clear\n");
  run(2000);
  check(contains(ctxA.serialOut, "tx_aborted"), "/clear aborted the message");
  run(20000);

  // ---- low contrast calibration is refused ----
  ctxA.serialOut.clear();
  selfCouplingA = 5;                 // sensor can no longer see its own LED
  int savedFar = 0;
  (void)savedFar;
  feed(ctxA, "/cal\n");
  run(1600);
  printf("--- A /cal low contrast ---\n%s\n", ctxA.serialOut.c_str());
  check(contains(ctxA.serialOut, "ERR: LOWCONTRAST"), "low contrast refused");
  check(contains(ctxA.serialOut, "receiver=disarmed"), "receiver disarmed on low contrast");
  selfCouplingA = 250;
  feed(ctxA, "/cal\n");
  run(1600);

  // ---- status and misc commands ----
  ctxA.serialOut.clear();
  feed(ctxA, "/status\n/version\n/help\n/mode full\n/mode sideways\n/wpm 99\n/nope\n");
  run(300);
  printf("--- A console (commands) ---\n%s\n", ctxA.serialOut.c_str());
  check(contains(ctxA.serialOut, "SYS: status"), "/status printed");
  check(contains(ctxA.serialOut, "version=1.1.0"), "/version printed");
  check(contains(ctxA.serialOut, "ERR: BADARG cmd=mode"), "bad mode argument rejected");
  check(contains(ctxA.serialOut, "ERR: BADARG cmd=wpm"), "out of range wpm rejected");
  check(contains(ctxA.serialOut, "ERR: BADCMD"), "unknown command rejected");

  // ---- full duplex mode with true isolation ----
  feed(ctxA, "/mode full\n/wpm 12\n");
  ctxB.serialOut.clear();
  selfCouplingA = 0;                 // simulate a shaded sensor
  run(100);
  feed(ctxA, "SK\n");
  run(20000);
  check(contains(ctxB.serialOut, "SK"), "full duplex mode still transmits correctly");

  // ---- CRLF handling and line overflow ----
  ctxA.serialOut.clear();
  feed(ctxA, "/mode half\r\n");
  run(50);
  check(contains(ctxA.serialOut, "mode=half"), "CRLF terminated line handled once");
  ctxA.serialOut.clear();
  {
    std::string longLine(200, 'E');
    longLine += "\n";
    feed(ctxA, longLine.c_str());
  }
  run(100);
  check(contains(ctxA.serialOut, "ERR: LINEOVF"), "over length line rejected");

  // ---- v1.1 persistence ----
  ctxA.serialOut.clear();
  feed(ctxA, "/wpm 17\n");   run(60);
  feed(ctxA, "/mode full\n"); run(60);
  feed(ctxA, "/status\n");   run(200);
  check(contains(ctxA.serialOut, "unsaved=1"), "changed settings flagged unsaved");
  ctxA.serialOut.clear();
  feed(ctxA, "/save\n");     run(200);
  check(contains(ctxA.serialOut, "SYS: saved"), "/save wrote the block");
  feed(ctxA, "/status\n");   run(300);
  check(contains(ctxA.serialOut, "unsaved=0"), "unsaved flag cleared after save");
  ctxA.serialOut.clear();
  feed(ctxA, "/defaults\n"); run(200);
  check(contains(ctxA.serialOut, "note=not_saved"), "/defaults does not touch EEPROM");
  ctxA.serialOut.clear();
  feed(ctxA, "/load\n");     run(200);
  printf("--- A /load ---\n%s\n", ctxA.serialOut.c_str());
  check(contains(ctxA.serialOut, "eeprom=loaded wpm=17"), "/load restored the saved speed");
  check(contains(ctxA.serialOut, "mode=full"), "/load restored the duplex mode");
  check(contains(ctxA.serialOut, "cal=held"), "/load does not silently arm a stale calibration");
  feed(ctxA, "/mode half\n/wpm 12\n"); run(200);

  // A cold boot must pick the stored block back up.
  {
    StationCtx fresh;
    fresh.ms = ctxA.ms; fresh.us = ctxA.us; fresh.adc = AMBIENT;
    StationCtx saved = ctxA;
    ctxA = fresh;
    g_ctx = &ctxA; A::setup();
    run(2200);
    printf("--- A cold boot ---\n%s\n", ctxA.serialOut.c_str());
    check(contains(ctxA.serialOut, "eeprom=loaded"), "cold boot reloaded stored settings");
    check(contains(ctxA.serialOut, "source=eeprom"), "boot cal used the stored contrast");
    ctxA.serialOut.clear();
    (void)saved;
  }
  feed(ctxA, "/cal\n"); run(1600);

  // ---- v1.2 Farnsworth, sidetone, straight key ----
  ctxA.serialOut.clear(); ctxB.serialOut.clear();
  feed(ctxA, "/wpm 20\n"); run(60);
  feed(ctxA, "/status\n"); run(300);
  check(contains(ctxA.serialOut, "fw=20"), "raising wpm does not latch stale Farnsworth");
  ctxA.serialOut.clear();
  feed(ctxA, "/fw 8\n"); run(200);
  printf("--- A /fw ---\n%s\n", ctxA.serialOut.c_str());
  check(contains(ctxA.serialOut, "gap_unit_ms=296"), "Farnsworth gap unit computed");
  ctxB.serialOut.clear();
  feed(ctxA, "OK\n");
  run(20000);
  printf("--- B under Farnsworth ---\n[%s]\n", ctxB.serialOut.c_str());
  check(contains(ctxB.serialOut, "O") && contains(ctxB.serialOut, "K"), "Farnsworth letters still decode");
  feed(ctxA, "/fw off\n"); run(200);

  ctxA.serialOut.clear();
  feed(ctxA, "/tone on\n/pitch 800\n"); run(300);
  check(contains(ctxA.serialOut, "pitch=800"), "sidetone pitch set");
  feed(ctxA, "E\n");
  bool heardTone = false;
  for (int i = 0; i < 4000; i++) { stepBoth(250); if (ctxA.toneOn) heardTone = true; }
  check(heardTone, "sidetone sounds during a transmitted mark");
  check(!ctxA.toneOn, "sidetone stops when the message ends");
  feed(ctxA, "/tone off\n"); run(200);

  // Send "R" (.-.) on the straight key at the configured 20 wpm, unit 60 ms.
  ctxA.serialOut.clear();
  feed(ctxA, "/wpm 12\n"); run(200);   // unit 100 ms, easier to hand key
  ctxA.keyDown = true;  run(100);
  ctxA.keyDown = false; run(100);
  ctxA.keyDown = true;  run(300);
  ctxA.keyDown = false; run(100);
  ctxA.keyDown = true;  run(100);
  ctxA.keyDown = false; run(3000);
  printf("--- A straight key ---\n%s\n", ctxA.serialOut.c_str());
  check(contains(ctxA.serialOut, "KEY: R"), "straight key decodes what was sent");

  // ---- v1.3 framing end to end ----
  feed(ctxA, "/fw off\n/wpm 20\n"); run(200);
  feed(ctxA, "/call N1HNP\n"); run(120);
  feed(ctxA, "/to CQ\n");      run(120);
  feed(ctxA, "/frame on\n");   run(120);
  ctxB.serialOut.clear(); ctxA.serialOut.clear();
  feed(ctxA, "HELLO WORLD\n");
  run(45000);
  printf("--- B framed receive ---\n%s\n", ctxB.serialOut.c_str());
  check(contains(ctxB.serialOut, "from=N1HNP"), "callsign survived the link");
  check(contains(ctxB.serialOut, "crc=ok"), "CRC validated at the far end");
  check(contains(ctxB.serialOut, "RX: text HELLO WORLD"), "framed payload recovered");
  check(contains(ctxA.serialOut, "TX: framed to=CQ"), "sender reported a framed message");

  // A frame with a deliberately wrong check value, sent as plain text.
  feed(ctxA, "/frame off\n"); run(200);
  ctxB.serialOut.clear();
  feed(ctxA, "= N1HNP CQ 5 = HELLO = 0000 +\n");
  run(60000);
  printf("--- B bad crc ---\n%s\n", ctxB.serialOut.c_str());
  check(contains(ctxB.serialOut, "ERR: BADCRC"), "corrupt frame rejected at the far end");

  // Unframed traffic must still be readable by a framing station.
  feed(ctxB, "/frame on\n/call W1AW\n"); run(300);
  ctxB.serialOut.clear();
  feed(ctxA, "PLAIN\n");
  run(30000);
  check(contains(ctxB.serialOut, "unframed_traffic"), "unframed traffic flagged, not dropped");
  check(contains(ctxB.serialOut, "PLAIN"), "unframed traffic still printed");
  feed(ctxB, "/frame off\n"); run(200);

  ctxA.serialOut.clear();
  feed(ctxA, "/status\n"); run(400);
  check(contains(ctxA.serialOut, "call=N1HNP"), "status carries the callsign");
  check(contains(ctxA.serialOut, "backoff=0"), "backoff counter clears after a clean send");

  // ---- ambient drift and mains flicker ----
  feed(ctxA, "/mode half\n");
  run(100);
  ctxB.serialOut.clear();
  rippleAmp = 25.0;                 // 120 Hz flicker, plus or minus 25 counts
  for (int i = 0; i < 30; i++) {    // ramp the room light up by 300 counts
    AMBIENT = 120 + (i * 10);
    run(800);
  }
  printf("--- B console (drift, idle) ---\n[%s]\n", ctxB.serialOut.c_str());
  check(!contains(ctxB.serialOut, "RX"), "no false keying under drift and flicker");

  ctxB.serialOut.clear();
  feed(ctxA, "SOS TEST\n");
  run(30000);
  printf("--- B console (drift, receiving) ---\n%s\n", ctxB.serialOut.c_str());
  check(contains(ctxB.serialOut, "SOS TEST"), "decodes correctly under drift and flicker");
  rippleAmp = 0.0;
  AMBIENT = 120;

  printf("\nHARNESS SUMMARY: %s (%d failures)\n", failures == 0 ? "GREEN" : "RED", failures);
  return failures == 0 ? 0 : 1;
}
