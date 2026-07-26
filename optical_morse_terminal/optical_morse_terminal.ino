/* ============================================================================
 * OPTICAL MORSE TERMINAL
 * Version 1.3.0
 * License: GPL-3.0
 *
 * ----------------------------------------------------------------------------
 * THEORY OF OPERATION
 * ----------------------------------------------------------------------------
 * One LED is the transmitter, one photoresistor (LDR) in a voltage divider is
 * the receiver, and the USB serial port is the operator. A line typed at the
 * console is pushed into a character ring buffer, walked by a non-blocking
 * state machine that keys the LED with PARIS-convention Morse timing. In
 * parallel, the analog input is sampled on a fixed 1 ms cadence, compared
 * against an adaptive threshold that floats on a slow ambient baseline with
 * separate rise and fall trip points, debounced with a minimum-duration
 * filter, and timestamped at every clean edge. Mark and space intervals are
 * classified against a running estimate of the sender's unit length, so the
 * receiver locks onto a station running at a different speed within a few
 * characters rather than only decoding its own configured WPM. Because the
 * local LED almost certainly illuminates the local LDR, the receiver is gated
 * against the transmitter (see SELF-INTERFERENCE below). Two boards running
 * this sketch, pointed at each other, form a working optical link.
 *
 * ----------------------------------------------------------------------------
 * WIRING
 * ----------------------------------------------------------------------------
 *   Pin    Component                    Orientation / notes
 *   -----  --------------------------   ------------------------------------
 *   D3     TX LED anode (long leg)      Cathode to GND through 220R to 1k.
 *                                       Any visible color. Red and IR-adjacent
 *                                       colors suit CdS cells best, blue and
 *                                       white are the worst match for a CdS
 *                                       spectral response.
 *   A0     LDR / fixed resistor node    See divider below.
 *   D13    Status LED (built in)        Lights while a received mark is
 *                                       being detected. Optional.
 *   D2     Calibration button           Other side to GND. INPUT_PULLUP, so
 *                                       the button is active low. Optional,
 *                                       leave the pin unconnected if unused.
 *   5V     LDR leg 1
 *   GND    Fixed resistor leg 2
 *
 *   Divider:   5V ---- LDR ----+---- Rfixed ---- GND
 *                              |
 *                              A0
 *
 *   With the LDR on the high side, more light lowers the LDR resistance and
 *   raises the voltage at A0, so more light equals a higher ADC count. That is
 *   what SENSOR_ACTIVE_HIGH = true means. If you build the divider the other
 *   way round (fixed resistor on the high side, LDR to GND), set
 *   SENSOR_ACTIVE_HIGH = false and the sketch inverts the reading for you.
 *
 *   Choosing Rfixed: the divider is most sensitive when Rfixed is near the
 *   geometric mean of the LDR resistance in your dark condition and its
 *   resistance under the LED at your intended spacing. Measure both with a
 *   meter, multiply, take the square root, pick the nearest standard value.
 *   A common GL5528 sitting in a normally lit room and looking at an LED a few
 *   centimeters away lands somewhere around 10k, which is why 10k is the usual
 *   starting point. If /cal reports low contrast, the fix is almost always a
 *   different Rfixed, closer spacing, or a shade tube around the sensor.
 *
 * ----------------------------------------------------------------------------
 * SERIAL OUTPUT GRAMMAR
 * ----------------------------------------------------------------------------
 * Every line carries a fixed prefix so a host script can parse the stream
 * without guessing. Lines are terminated with a single \n.
 *
 *   TX: <key>=<value> ...   transmit events, one line per event
 *   TX+ <text>              local echo of transmitted characters, run back
 *                           through the decoder. Opened at message start,
 *                           closed at message end. Enabled by /echo on.
 *   RX: <text>              decoded incoming text, printed incrementally as
 *                           each character resolves. The line stays open until
 *                           a receive idle timeout closes it.
 *   RX+ <text>              continuation of an RX line that had to be closed
 *                           early because another line was emitted.
 *   SYS: <key>=<value> ...  status and informational events
 *   ERR: <CODE> <key>=<value> ...   errors. CODE is a stable uppercase token.
 *   DBG: RAW t=<ms> adc=<n> base=<n> rise=<n> fall=<n> mark=<0|1> gate=<0|1>
 *   DBG: EDGE t=<ms> kind=<MARK|SPACE> dur=<ms> unit=<ms>
 *
 * Stable ERR codes: LINEOVF, TXFULL, TXDROP, BADCMD, BADARG, BUSY, LOWCONTRAST,
 * NOTARMED, RXBREAK, OUTDROP, SAMPLEOVR.
 *
 * ----------------------------------------------------------------------------
 * COMMAND REFERENCE
 * ----------------------------------------------------------------------------
 *   Any line not starting with / is queued for transmission.
 *
 *   /help              list commands
 *   /wpm <5..30>       set transmit speed
 *   /cal               guided ambient calibration (dark, then lit)
 *   /status            wpm, rx wpm, thresholds, baseline, queue, uptime, RAM
 *   /mode <half|full>  duplex mode, see SELF-INTERFERENCE
 *   /echo <on|off>     local echo of transmitted characters as decoded text
 *   /raw <on|off>      stream ADC values and edge timings
 *   /loop              run the built in self test
 *   /clear             flush the transmit queue, abort cleanly at the next
 *                      element boundary
 *   /version           print name and semantic version
 *   /save              write settings and calibration to EEPROM
 *   /load              reload the stored block
 *   /defaults          settings back to compiled defaults, not saved
 *   /fw <wpm|off>      Farnsworth overall speed, never above the character speed
 *   /tone <on|off>     sidetone on the piezo
 *   /pitch <300..1200> sidetone frequency in Hz
 *   /key <on|off>      enable or disable the straight key input
 *   /call <sign>       your callsign, up to 8 sendable characters
 *   /to <sign>         destination callsign, default CQ
 *   /frame <on|off>    send framed, length checked, CRC checked messages
 *
 * ----------------------------------------------------------------------------
 * FRAME FORMAT
 * ----------------------------------------------------------------------------
 *   = <FROM> <TO> <LEN> = <TEXT> = <CRC4> +
 *
 * A frame is ordinary Morse text, so a station that knows nothing about
 * framing still copies something readable rather than gibberish. BT (=) opens
 * the frame and separates the header, the text and the check value. AR (+)
 * closes it. Neither can appear inside TEXT because framed text rejects them.
 * LEN is the decimal character count of TEXT, which catches a truncation that
 * happens to leave a plausible check value. CRC4 is four hex digits of
 * CRC-16/CCITT-FALSE over TEXT.
 *
 * Receiving is always tolerant: a line that does not parse as a frame is
 * printed as plain text and noted, so a 1.3 station and a 1.0 station can
 * still talk, just without any of the guarantees.
 *
 * ----------------------------------------------------------------------------
 * SELF-INTERFERENCE
 * ----------------------------------------------------------------------------
 * The local LED will light the local LDR unless you have physically isolated
 * them. Two mitigations are provided.
 *
 *   MODE_HALF (default, safe): the receiver is gated off for the whole
 *   duration of an outgoing message, plus RX_REARM_GUARD_MS afterwards to let
 *   the LDR decay back to ambient. Nothing that arrives while you are sending
 *   is heard. This is a true half duplex link and it cannot decode its own
 *   transmission because it is not listening at all.
 *
 *   MODE_FULL (requires optical isolation): the receiver is gated only for
 *   TX_GUARD_MS on each side of every LED transition, which covers sensor rise
 *   and fall lag. This assumes a shaded sensor or separate optical paths. If
 *   the local LED is visible to the local LDR in this mode, the sketch will
 *   decode its own transmission and the console will fill with echoes of what
 *   you just sent. The tradeoff: MODE_FULL only listens through the gaps, and
 *   since the guard is 12 ms per transition it starts to eat meaningfully into
 *   the element gaps above roughly 15 WPM.
 *
 * Re-arm: when gating lifts, any partially assembled symbol is discarded and
 * reported as ERR: RXBREAK rather than being silently folded into whatever
 * arrives next. An interrupted RX line is reopened with the RX+ prefix.
 *
 * Carrier sense: if a message is queued while the channel is busy, transmit is
 * deferred until the channel has been idle for one word gap. This is advisory
 * collision avoidance only. It has no way to detect a station that is silent
 * during an inter-character gap, it cannot hear a third station that only one
 * of the two can see, and two stations that begin deferring at the same moment
 * will still collide when the channel clears. There is no backoff.
 *
 * ----------------------------------------------------------------------------
 * KNOWN LIMITATIONS
 * ----------------------------------------------------------------------------
 *  1. Ambient light sensitivity. The adaptive baseline handles slow drift such
 *     as the sun moving across a room. It does not handle a hand passing over
 *     the sensor, a camera flash, or someone flipping the room lights, all of
 *     which will inject spurious elements. Mains flicker at 100/120 Hz is
 *     rejected by the 6 ms debounce, not by any filtering of the signal.
 *  2. LDR response time caps usable speed. A CdS cell takes on the order of
 *     10 to 50 ms to rise and considerably longer to decay. WPM_MAX is 30 for
 *     the arithmetic, but a typical CdS cell will not cleanly resolve a 40 ms
 *     dot. Expect 8 to 15 WPM in practice. A phototransistor or photodiode
 *     front end would lift this ceiling substantially and is out of scope.
 *  3. Detection, not correction. A framed message that fails its length or CRC
 *     check is rejected and reported, but nothing asks for it again. There is
 *     no acknowledgement, no sequence number, and no retransmission, so a lost
 *     message stays lost and the sender never finds out.
 *  4. Addressing is a label, not a filter. Every station in the optical path
 *     still hears and prints every transmission. The TO field says who a frame
 *     is for, and nothing enforces it.
 *  5. Backoff helps two stations, not a crowd. The window doubles and the wait
 *     is drawn at random inside it, which separates two stations reliably. It
 *     does nothing about a station that only one of the two can see, and the
 *     carrier sense underneath it still cannot tell a genuinely idle channel
 *     from a sender pausing between characters.
 *  6. Self-interference gating assumes the transmitter is the only local light
 *     source that matters. In MODE_FULL the assumption of optical isolation is
 *     entirely on the operator to satisfy physically.
 *  7. The receive unit estimator tracks one sender. If two stations at
 *     different speeds transmit in the same session, the estimate will chase
 *     whichever spoke last and the first few characters after a handover may
 *     decode wrong.
 *  8. Persistence is explicit, not automatic. Change a setting, forget to
 *     /save, and the change is gone at the next reset. This is deliberate:
 *     an EEPROM cell tolerates roughly 100000 writes, and saving on every
 *     /wpm would spend that budget on nothing.
 *  9. Farnsworth sending is a one way courtesy. A human copying by ear hears
 *     stretched gaps as thinking time, but a machine decoder using adaptive
 *     spacing thresholds reads them as word gaps, so a far station running
 *     this sketch will copy the right letters with the wrong word breaks. The
 *     receiver does not estimate character and word spacing separately.
 * 10. The straight key classifies against the configured character speed, not
 *     an adaptive estimate. Key faster or slower than /wpm and the KEY: line
 *     will tell you so, which is the point, but it is not a fist reader.
 * 11. Console backpressure is deliberate. Paste a long block of commands and
 *     the sketch stops reading input until the console has drained, which
 *     protects the output stream but can overrun the hardware receive buffer
 *     if the host keeps pushing. Send commands a line at a time.
 * 10. Verified in a host side two station simulation (self test 44 of 44,
 *     link, speed lock, gating, drift, flicker, millis rollover). Not yet
 *     bench verified on hardware, which is what the bring-up procedure is for.
 *
 * ----------------------------------------------------------------------------
 * CHANGELOG
 * ----------------------------------------------------------------------------
 *   1.3.0  Framing, check values and real backoff. Messages can carry a
 *          callsign, a length and a CRC-16, so a corrupt message is now
 *          rejected rather than printed with holes in it, and stations can be
 *          told apart. Deferred transmissions now back off with a doubling
 *          window and a random draw inside it, so two stations that started
 *          waiting together do not resume together. Framing is off by default.
 *   1.2.0  Sidetone, straight key, and Farnsworth spacing. The key drives the
 *          LED and the sidetone directly and decodes what you sent onto a KEY:
 *          line, so the board is a practice oscillator that answers back.
 *          Farnsworth is opt in and never latches: raising /wpm keeps the
 *          overall speed with it unless /fw was asked for explicitly.
 *   1.1.0  EEPROM persistence. Settings and the measured contrast survive a
 *          reset. Saving is explicit, /status carries an unsaved flag, and the
 *          block is protected by a magic word, a layout byte, and a CRC-8.
 *          The stored ambient dark level is deliberately not restored as the
 *          live baseline, only the contrast is.
 *   1.0.0  First release.
 *
 * ----------------------------------------------------------------------------
 * ROADMAP, DELIBERATELY NOT BUILT
 * ----------------------------------------------------------------------------
 *   - Framing with a callsign, length, and CRC so stations can be told apart
 *     and corrupt messages rejected rather than printed with ? holes.
 *   - Exponential backoff on deferred transmit, which would make the carrier
 *     sense worth more than advice.
 *   - Photodiode plus transimpedance front end, and the higher WPM ceiling and
 *     PWM carrier modulation that becomes possible with it.
 *   - Host side terminal script consuming the output grammar.
 *
 * ----------------------------------------------------------------------------
 * RESOURCE USAGE
 * ----------------------------------------------------------------------------
 * NOT YET MEASURED. These are placeholders, not results. Fill them in from the
 * Arduino IDE build output on your first compile, they are part of the record:
 *   ATmega328P flash: ____ bytes of 32256
 *   ATmega328P RAM (static): ____ bytes of 2048
 * Static buffer budget accounted for by hand: TX queue 128, serial input 96,
 * output buffer 256, plus roughly 120 bytes of engine state, so about 600
 * bytes before the Arduino core's own serial buffers. /status reports free RAM
 * at runtime, which is the number that actually matters.
 * ============================================================================
 */

#include <Arduino.h>

/* EEPROM is present on every board this sketch targets. The macro is left
 * overridable so a host side harness can supply its own storage shim. */
#ifndef MORSE_HAS_EEPROM
#  if defined(__AVR__) || defined(ESP32) || defined(ESP8266)
#    define MORSE_HAS_EEPROM 1
#  else
#    define MORSE_HAS_EEPROM 0
#  endif
#endif
#if MORSE_HAS_EEPROM
#  include <EEPROM.h>
#endif

/* ============================================================================
 * CONFIGURATION. Everything tunable lives here and nowhere else.
 * ========================================================================= */

/* ---- Pins and board assumptions ---------------------------------------- */
static const uint8_t  PIN_TX_LED        = 3;
static const uint8_t  PIN_LDR           = A0;
static const uint8_t  PIN_STATUS_LED    = LED_BUILTIN;
static const uint8_t  PIN_CAL_BUTTON    = 2;
static const bool     CAL_BUTTON_FITTED = true;

/* Sidetone piezo. tone() drives it from timer 2 on AVR, which is independent
 * of the transmit LED because that pin is switched with digitalWrite and never
 * with PWM. */
static const uint8_t  PIN_SIDETONE      = 9;
static const uint16_t PITCH_DEFAULT     = 700;
static const uint16_t PITCH_MIN         = 300;
static const uint16_t PITCH_MAX         = 1200;

/* Straight key, other side to GND, active low. The pin is sampled at boot and
 * the key is disabled if it already reads closed, which is what an unwired
 * input looks like when nothing is holding it up. */
static const uint8_t  PIN_KEY           = 4;
static const bool     KEY_FITTED        = true;
static const uint8_t  KEY_DEBOUNCE_MS   = 5;

/* True when more light produces a higher ADC count, which is the case with the
 * LDR on the high side of the divider. See the wiring note in the header. */
static const bool     SENSOR_ACTIVE_HIGH = true;

/* Full scale of analogRead on this core. 1023 on AVR, 4095 on ESP32. */
static const uint16_t ADC_MAX = 1023;

static const uint32_t SERIAL_BAUD = 115200UL;

/* ---- Morse timing ------------------------------------------------------- */
/* PARIS convention: one word is 50 units, so unit_ms = 60000 / (50 * wpm)
 * which reduces to 1200 / wpm. */
static const uint16_t MORSE_UNIT_CONSTANT = 1200;
static const uint8_t  WPM_DEFAULT = 12;
static const uint8_t  WPM_MIN     = 5;    /* 240 ms unit, slow enough for any LDR */
static const uint8_t  WPM_MAX     = 30;   /* 40 ms unit, the arithmetic ceiling */

/* Farnsworth spacing sends the characters themselves at the character speed
 * while stretching the gaps between them to hit a slower overall speed, which
 * is how nearly everyone learns to copy. The standard derivation: a PARIS word
 * is 50 units, of which 31 are inside characters and 19 are the gaps between
 * them. Send the 31 at character speed C, then whatever is left of the word's
 * total duration at overall speed F is divided across the 19 gap units:
 *
 *   gap_unit_ms = (60000 * C - 37200 * F) / (19 * C * F)
 *
 * With F equal to C this reduces exactly to 1200 / C, so standard timing is
 * just the special case and needs no separate code path. */
static const uint32_t FARNSWORTH_WORD_MS   = 60000UL;  /* one word at F, in ms */
static const uint32_t FARNSWORTH_CHAR_MS   = 37200UL;  /* 31 units at 1200/C */
static const uint8_t  FARNSWORTH_GAP_UNITS = 19;

/* ---- Receiver sampling -------------------------------------------------- */
/* Required sampling rate: the shortest element is one dot, which at WPM_MAX is
 * 1200/30 = 40 ms. The stated minimum oversampling factor is 20 samples per
 * dot, so the sample period must be at most 40/20 = 2 ms. We sample at 1 ms,
 * giving 40 samples per dot at 30 WPM and 240 at 5 WPM. analogRead on an
 * ATmega328P takes about 112 us, so the sampler occupies roughly 11 percent of
 * the loop at this cadence. */
static const uint16_t SAMPLE_INTERVAL_US  = 1000;
static const uint8_t  MIN_OVERSAMPLE_PER_DOT = 20;

/* Loop budget: every pass of loop() must complete in well under one sample
 * interval or the sampler starves. Target is 300 us worst case, which leaves
 * 700 us of headroom against the 1000 us cadence. The two things that could
 * break this budget are a long blocking Serial write (avoided by the output
 * ring buffer, which never writes more bytes than availableForWrite reports)
 * and the self test (which reports one assertion per pass). At most one
 * console line is handled per pass for the same reason, and console input is
 * held off entirely while the output ring is more than half full. */

/* Debounce: an accepted edge must hold its new state for this long. This is
 * what rejects mains flicker, whose half cycle is 5.0 ms at 100 Hz and 4.2 ms
 * at 120 Hz. It costs 15 percent of a dot at 30 WPM, which is a second reason
 * WPM_MAX is where it is. */
static const uint8_t  EDGE_DEBOUNCE_MS = 6;

/* Hysteresis, as a percentage of the calibrated contrast above baseline. A
 * space becomes a mark above RISE, a mark becomes a space below FALL. */
static const uint8_t  THRESHOLD_RISE_PCT = 60;
static const uint8_t  THRESHOLD_FALL_PCT = 40;

/* Baseline is an exponential moving average in Q4 fixed point, updated only
 * while the input is below the fall threshold and the receiver is not gated.
 * A shift of 8 gives a time constant of 256 samples, so about 256 ms. Slow
 * enough that it will not chase a legitimate mark, fast enough to follow a
 * room. */
static const uint8_t  BASELINE_SHIFT = 8;

/* Minimum ADC counts between dark and lit for the link to be considered
 * usable. Below this, /cal refuses to arm the receiver. */
static const uint16_t MIN_CONTRAST_COUNTS = 40;

/* Assumed contrast before any /cal has run, so the receiver is usable out of
 * the box on a well built divider. */
static const uint16_t DEFAULT_CONTRAST_COUNTS = 120;

/* Receive unit estimator clamps, in milliseconds. 30 ms is 40 WPM, 400 ms is
 * 3 WPM. Anything outside this is treated as noise, not as a sender. */
static const uint16_t RX_UNIT_MIN_MS = 30;
static const uint16_t RX_UNIT_MAX_MS = 400;

/* An open RX line is closed and summarised after this much silence, expressed
 * in received units. Two word gaps. */
static const uint8_t  RX_LINE_IDLE_UNITS = 14;

/* ---- Self-interference gating ------------------------------------------ */
static const uint8_t  TX_GUARD_MS       = 12;  /* each side of an LED transition, MODE_FULL */
static const uint16_t RX_REARM_GUARD_MS = 60;  /* after a message ends, MODE_HALF */

/* ---- Calibration -------------------------------------------------------- */
static const uint16_t BOOT_SETTLE_MS = 1500;   /* ambient settle before arming */
static const uint16_t CAL_PHASE_MS   = 600;    /* per phase of /cal */

/* ---- Framing ------------------------------------------------------------ */
/* On air a frame is ordinary Morse text, so an unframed station still copies
 * something readable. The shape is:
 *
 *   = <FROM> <TO> <LEN> = <TEXT> = <CRC4> +
 *
 * BT (=) opens the frame and separates the header, the text, and the check.
 * AR (+) closes it. Both are ordinary prosigns that predate this sketch by a
 * century, and neither can appear inside the text because framed text rejects
 * them. LEN is the decimal character count of TEXT and catches a truncation
 * that happens to leave a plausible CRC. CRC4 is four hex digits of
 * CRC-16/CCITT-FALSE over TEXT. */
static const uint8_t  CALLSIGN_MAX     = 8;
static const uint8_t  FRAME_TEXT_MAX   = 60;   /* keeps the whole frame inside the queue */
static const uint16_t CRC16_POLYNOMIAL = 0x1021;
static const uint16_t CRC16_INIT       = 0xFFFF;

/* Backoff. The wait window doubles per attempt from one word gap, and the
 * actual wait is drawn uniformly from inside it, so two stations that started
 * deferring together do not resume together. */
static const uint8_t  BACKOFF_MAX_SHIFT    = 4;   /* window caps at 16 word gaps */
static const uint8_t  BACKOFF_REPORT_AFTER = 4;   /* say something after this many */

/* ---- Buffers. All fixed size, all powers of two where masked. ----------- */
static const uint8_t  TX_QUEUE_SIZE    = 128;  /* must be a power of two */
static const uint8_t  SERIAL_LINE_MAX  = 96;
static const uint16_t OUT_BUF_SIZE     = 256;  /* must be a power of two */
static const uint8_t  MAX_ELEMENTS     = 8;    /* longest supported symbol, the error prosign */
static const uint8_t  RX_LINE_MAX      = 96;   /* received text held for frame parsing */

/* ---- Debug -------------------------------------------------------------- */
static const uint16_t RAW_REPORT_MS = 100;     /* /raw sample line cadence */
static const uint8_t  CAL_BUTTON_DEBOUNCE_MS = 30;
static const uint8_t  SERIAL_BYTES_PER_PASS = 16; /* bounds console work per loop */

/* ---- Persistence -------------------------------------------------------- */
/* Settings live at the bottom of EEPROM. The magic word identifies the block
 * as ours, the layout byte lets a later version recognise and reject a struct
 * it does not understand, and the CRC catches a half finished write. */
static const uint16_t SETTINGS_MAGIC   = 0x4D54;  /* 'MT' */
static const uint8_t  SETTINGS_LAYOUT  = 3;
static const int      SETTINGS_ADDRESS = 0;
static const uint8_t  CRC8_POLYNOMIAL  = 0x07;    /* CRC-8/ATM, init 0x00 */

static const char PROJECT_NAME[] = "OPTICAL MORSE TERMINAL";
static const char PROJECT_VERSION[] = "1.1.0";

/* Marks the end of one operator message inside the transmit queue. Input lines
 * have all CR and LF stripped, so this byte can never appear as payload. */
static const uint8_t MSG_END_SENTINEL = '\n';

/* ============================================================================
 * TYPES
 * ========================================================================= */

enum TxState : uint8_t {
  TX_IDLE,      /* nothing queued, or queued and channel clear on next pass */
  TX_DEFER,     /* queued but carrier sense says wait */
  TX_MARK,      /* LED on for one element */
  TX_INTRA,     /* 1 unit gap between elements of one character */
  TX_CHARGAP,   /* 3 unit gap between characters */
  TX_WORDGAP    /* 7 unit gap between words */
};

enum RxArm : uint8_t {
  RX_DISARMED,  /* no usable calibration, or /cal refused to arm */
  RX_ARMED
};

enum SpaceKind : uint8_t {
  SPACE_ELEMENT,
  SPACE_CHAR,
  SPACE_WORD
};

enum CalState : uint8_t {
  CAL_NONE,
  CAL_BOOT,     /* ambient settle at power on */
  CAL_DARK,     /* /cal phase 1, LED off */
  CAL_LIT       /* /cal phase 2, LED on */
};

enum DuplexMode : uint8_t {
  MODE_HALF,
  MODE_FULL
};

enum SelfTestState : uint8_t {
  ST_IDLE,
  ST_ROUNDTRIP,
  ST_TIMING,
  ST_ROLLOVER,
  ST_RING,
  ST_CLASSIFY,
  ST_PERSIST,
  ST_FRAME,
  ST_SUMMARY
};

/* ============================================================================
 * SMALL UTILITIES
 * ========================================================================= */

/* Rollover safe elapsed test. Unsigned subtraction wraps correctly, so this is
 * valid across the millis() wrap at 49.7 days. Never compare against an
 * absolute start+span sum anywhere in this sketch. */
static inline bool elapsed(uint32_t now, uint32_t start, uint32_t span) {
  return (uint32_t)(now - start) >= span;
}

static inline uint16_t unitMsForWpm(uint8_t wpm) {
  return (uint16_t)(MORSE_UNIT_CONSTANT / wpm);
}

static inline uint8_t wpmForUnitMs(uint16_t unitMs) {
  if (unitMs == 0) return 0;
  return (uint8_t)(MORSE_UNIT_CONSTANT / unitMs);
}

static inline char toUpperAscii(char c) {
  return (c >= 'a' && c <= 'z') ? (char)(c - 32) : c;
}

static int freeRamBytes() {
#if defined(__AVR__)
  extern int __heap_start;
  extern void *__brkval;
  int probe;
  return (int)&probe - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
#elif defined(ESP32) || defined(ESP8266)
  return (int)ESP.getFreeHeap();
#else
  return -1;
#endif
}

/* ============================================================================
 * OUTPUT RING BUFFER
 *
 * Serial.print blocks once the core's 64 byte TX buffer fills, which at 115200
 * baud is about 5.5 ms of stalled loop and a starved sampler. Everything this
 * sketch emits goes through this ring instead, drained only as fast as
 * availableForWrite() says there is room. Nothing in the operating path ever
 * waits on the UART.
 * ========================================================================= */

static char     outBuf[OUT_BUF_SIZE];
static uint16_t outHead = 0;
static uint16_t outTail = 0;
static bool     outDropped = false;

static const uint16_t OUT_MASK = OUT_BUF_SIZE - 1;

static uint16_t outCount() { return (uint16_t)((outHead - outTail) & OUT_MASK); }
static uint16_t outFree()  { return (uint16_t)(OUT_BUF_SIZE - 1 - outCount()); }

static void outPutc(char c) {
  if (outFree() == 0) { outDropped = true; return; }
  outBuf[outHead] = c;
  outHead = (uint16_t)((outHead + 1) & OUT_MASK);
}

static void outStr(const char *s) {
  while (*s) outPutc(*s++);
}

/* PROGMEM string, always used through the OUTP macro below. */
static void outStrP(const char *s) {
  char c;
  while ((c = (char)pgm_read_byte(s++)) != 0) outPutc(c);
}
#define OUTP(literal) outStrP(PSTR(literal))

static void outUL(uint32_t v) {
  char tmp[11];
  uint8_t n = 0;
  if (v == 0) { outPutc('0'); return; }
  while (v > 0 && n < sizeof(tmp)) { tmp[n++] = (char)('0' + (v % 10)); v /= 10; }
  while (n > 0) outPutc(tmp[--n]);
}

static void outI32(int32_t v) {
  if (v < 0) { outPutc('-'); outUL((uint32_t)(-v)); }
  else outUL((uint32_t)v);
}

static void outNL() { outPutc('\n'); }

static void outService() {
  int room = Serial.availableForWrite();
  while (room > 0 && outTail != outHead) {
    Serial.write((uint8_t)outBuf[outTail]);
    outTail = (uint16_t)((outTail + 1) & OUT_MASK);
    room--;
  }
  /* Report the drop only once the ring has room again, otherwise the report
   * itself would be dropped. */
  if (outDropped && outCount() == 0) {
    outDropped = false;
    /* The leading newline terminates whatever line was truncated, so the
     * report does not run into the tail of a half printed line. */
    OUTP("\nERR: OUTDROP reason=console_backlog\n");
  }
}

/* ============================================================================
 * LINE OWNERSHIP
 *
 * RX text and TX echo text are printed incrementally, so they hold an open
 * line. Anything else that emits must close them first or the stream stops
 * being parseable. Reopened RX lines use the RX+ prefix.
 * ========================================================================= */

static bool rxLineOpen = false;
static bool rxLineContinued = false;
static bool txEchoLineOpen = false;
static bool keyLineOpen = false;   /* straight key practice text, KEY: prefix */

static void closeOpenLines() {
  if (rxLineOpen) { outNL(); rxLineOpen = false; rxLineContinued = true; }
  if (txEchoLineOpen) { outNL(); txEchoLineOpen = false; }
  if (keyLineOpen) { outNL(); keyLineOpen = false; }
}

/* ============================================================================
 * CHARACTER RING BUFFER
 *
 * Single threaded, no interrupt touches it, so nothing here needs volatile.
 * Capacity must be a power of two. One slot is always left empty so head equal
 * to tail unambiguously means empty, giving a usable capacity of cap - 1.
 * ========================================================================= */

struct CharRing {
  uint8_t *store;
  uint8_t  cap;
  uint8_t  mask;
  uint8_t  head;
  uint8_t  tail;
  uint16_t droppedCount;

  void init(uint8_t *buffer, uint8_t capacity) {
    store = buffer;
    cap = capacity;
    mask = (uint8_t)(capacity - 1);
    head = 0;
    tail = 0;
    droppedCount = 0;
  }
  uint8_t count() const { return (uint8_t)((head - tail) & mask); }
  uint8_t capacityUsable() const { return (uint8_t)(cap - 1); }
  bool empty() const { return head == tail; }
  bool full() const { return count() == capacityUsable(); }
  bool push(uint8_t v) {
    if (full()) { droppedCount++; return false; }
    store[head] = v;
    head = (uint8_t)((head + 1) & mask);
    return true;
  }
  bool pop(uint8_t *out) {
    if (empty()) return false;
    *out = store[tail];
    tail = (uint8_t)((tail + 1) & mask);
    return true;
  }
  bool peek(uint8_t *out) const {
    if (empty()) return false;
    *out = store[tail];
    return true;
  }
  void clear() { head = 0; tail = 0; }
};

static uint8_t  txQueueStore[TX_QUEUE_SIZE];
static CharRing txQueue;

/* ============================================================================
 * MORSE TABLE
 *
 * Each symbol is stored as a single prefix coded integer. Start at 1, then for
 * each element shift left and OR in 1 for a dash, 0 for a dot. The leading 1 is
 * a sentinel that makes the length recoverable: the element count is the
 * position of the most significant set bit. This makes both directions a
 * single small integer comparison and keeps the whole table in flash.
 *
 * Two parallel arrays rather than an array of structs, because a struct of
 * {char, uint16_t} would need alignment padding to stay safe under
 * pgm_read_word on cores that care about it.
 *
 * Prosigns have no ASCII of their own, so:
 *   '+' is AR  .-.-.   end of message
 *   '='  is BT  -...-   break / double dash
 *   '*' is SK  ...-.-  end of contact   (local convention, documented here)
 *   '#' is HH  ........ error           (local convention, documented here)
 * ========================================================================= */

static const uint8_t MORSE_COUNT = 44;

static const char MORSE_CHARS[MORSE_COUNT] PROGMEM = {
  'A','B','C','D','E','F','G','H','I','J','K','L','M',
  'N','O','P','Q','R','S','T','U','V','W','X','Y','Z',
  '0','1','2','3','4','5','6','7','8','9',
  '.',',','?','/','+','=','*','#'
};

static const uint16_t MORSE_CODES[MORSE_COUNT] PROGMEM = {
  /* A .-    */   5,
  /* B -...  */  24,
  /* C -.-.  */  26,
  /* D -..   */  12,
  /* E .     */   2,
  /* F ..-.  */  18,
  /* G --.   */  14,
  /* H ....  */  16,
  /* I ..    */   4,
  /* J .---  */  23,
  /* K -.-   */  13,
  /* L .-..  */  20,
  /* M --    */   7,
  /* N -.    */   6,
  /* O ---   */  15,
  /* P .--.  */  22,
  /* Q --.-  */  29,
  /* R .-.   */  10,
  /* S ...   */   8,
  /* T -     */   3,
  /* U ..-   */   9,
  /* V ...-  */  17,
  /* W .--   */  11,
  /* X -..-  */  25,
  /* Y -.--  */  27,
  /* Z --..  */  28,
  /* 0 ----- */  63,
  /* 1 .---- */  47,
  /* 2 ..--- */  39,
  /* 3 ...-- */  35,
  /* 4 ....- */  33,
  /* 5 ..... */  32,
  /* 6 -.... */  48,
  /* 7 --... */  56,
  /* 8 ---.. */  60,
  /* 9 ----. */  62,
  /* . .-.-.-*/  85,
  /* , --..--*/ 115,
  /* ? ..--..*/  76,
  /* / -..-. */  50,
  /* + .-.-. */  42,
  /* = -...- */  49,
  /* * ...-.-*/  69,
  /* # ........*/ 256
};

/* Returns 0 when the character has no encoding. 0 is never a valid code
 * because every code carries the sentinel bit. */
static uint16_t morseFindCode(char c) {
  for (uint8_t i = 0; i < MORSE_COUNT; i++) {
    if ((char)pgm_read_byte(&MORSE_CHARS[i]) == c) {
      return (uint16_t)pgm_read_word(&MORSE_CODES[i]);
    }
  }
  return 0;
}

/* Returns 0 when the code matches nothing in the table. */
static char morseFindChar(uint16_t code) {
  for (uint8_t i = 0; i < MORSE_COUNT; i++) {
    if ((uint16_t)pgm_read_word(&MORSE_CODES[i]) == code) {
      return (char)pgm_read_byte(&MORSE_CHARS[i]);
    }
  }
  return 0;
}

/* Expands a prefix code into an element array, MSB first, dropping the
 * sentinel. bits[i] is 1 for a dash and 0 for a dot. */
static void codeToElements(uint16_t code, uint8_t *bits, uint8_t *count) {
  int8_t msb = -1;
  for (int8_t i = 15; i >= 0; i--) {
    if (code & ((uint16_t)1 << i)) { msb = i; break; }
  }
  if (msb <= 0) { *count = 0; return; }
  *count = (uint8_t)msb;
  for (uint8_t i = 0; i < *count; i++) {
    uint8_t shift = (uint8_t)(msb - 1 - i);
    bits[i] = (uint8_t)((code >> shift) & 1);
  }
}

static uint16_t elementsToCode(const uint8_t *bits, uint8_t count) {
  uint16_t code = 1;
  for (uint8_t i = 0; i < count; i++) code = (uint16_t)((code << 1) | (bits[i] & 1));
  return code;
}

/* ============================================================================
 * ELEMENT CLASSIFICATION
 *
 * Pure functions so the self test can hit the decision boundaries directly.
 * Nominal durations are 1 and 3 units for marks, 1, 3 and 7 units for spaces,
 * so the boundaries sit at 2 units and 5 units.
 * ========================================================================= */

static bool markIsDash(uint32_t durationMs, uint16_t unitMs) {
  return durationMs >= (uint32_t)unitMs * 2UL;
}

static SpaceKind classifySpace(uint32_t durationMs, uint16_t unitMs) {
  if (durationMs < (uint32_t)unitMs * 2UL) return SPACE_ELEMENT;
  if (durationMs < (uint32_t)unitMs * 5UL) return SPACE_CHAR;
  return SPACE_WORD;
}

/* ============================================================================
 * RUNTIME STATE
 * ========================================================================= */

/* ---- Operator settings -------------------------------------------------- */
static uint8_t    txWpm       = WPM_DEFAULT;
static uint16_t   txUnitMs    = 0;          /* element unit at character speed */
static uint8_t    txFarnsworthWpm = WPM_DEFAULT;  /* overall speed, never above txWpm */
static bool       farnsworthExplicit = false;     /* the operator asked for it */
static uint16_t   txGapUnitMs = 0;          /* unit used for character and word gaps */
static bool       toneEnabled = false;
static uint16_t   sidetonePitch = PITCH_DEFAULT;
static DuplexMode duplexMode  = MODE_HALF;
static bool       echoEnabled = false;
static bool       rawEnabled  = false;

/* ---- Transmit ----------------------------------------------------------- */
static TxState  txState = TX_IDLE;
static uint32_t txPhaseStart = 0;
static uint32_t txPhaseLen = 0;
static uint32_t txNextChangeMs = 0;   /* when the LED next changes state */
static uint8_t  txElemBits[MAX_ELEMENTS];
static uint8_t  txElemCount = 0;
static uint8_t  txElemIndex = 0;
static uint16_t txCharsSent = 0;
static uint32_t txMessageStartMs = 0;
static uint32_t txEndedMs = 0;
static bool     txLedOn = false;
static uint32_t lastLedChangeMs = 0;
static bool     txAbortRequested = false;
static bool     txDeferReported = false;

/* ---- Receive ------------------------------------------------------------ */
static RxArm    rxArm = RX_DISARMED;
static uint32_t lastSampleUs = 0;
static uint16_t sampleOverruns = 0;
static uint16_t rxLastLevel = 0;
static int32_t  baselineQ4 = 0;             /* ambient dark level, Q4 fixed point */
static uint16_t contrastCounts = DEFAULT_CONTRAST_COUNTS;
static uint16_t riseThreshold = 0;
static uint16_t fallThreshold = 0;
static bool     rxInMark = false;
static bool     rxCandidatePending = false;
static bool     rxCandidateState = false;
static uint32_t rxCandidateStartMs = 0;
static uint32_t rxLastEdgeMs = 0;
static uint16_t rxSymbolCode = 1;
static uint8_t  rxSymbolLen = 0;
static bool     rxSymbolOverflow = false;
static int32_t  rxUnitQ4 = 0;               /* estimated sender unit, Q4 fixed point */
static bool     rxWordPending = false;      /* a word space is owed to the open line */
static uint16_t rxCharsThisLine = 0;
static uint16_t rxErrorsThisLine = 0;
static bool     rxGatedPrev = true;
static uint32_t rawLastReportMs = 0;

/* ---- Calibration -------------------------------------------------------- */
static CalState calState = CAL_NONE;
static uint32_t calPhaseStart = 0;
static uint32_t calSum = 0;
static uint16_t calSamples = 0;
static uint16_t calDarkMean = 0;

/* ---- Framing and backoff ------------------------------------------------ */
static bool     frameEnabled = false;
static char     myCall[CALLSIGN_MAX + 1] = "NOCALL";
static char     toCall[CALLSIGN_MAX + 1] = "CQ";
static char     rxLine[RX_LINE_MAX];
static uint8_t  rxLineLen = 0;
static bool     rxLineTruncated = false;
static uint32_t rngState = 0;
static uint8_t  txBackoffAttempt = 0;
static uint32_t txBackoffStart = 0;
static uint16_t txBackoffWaitMs = 0;
static bool     txBackoffArmed = false;

/* ---- Straight key -------------------------------------------------------- */
static bool     keyFitted = KEY_FITTED;
static bool     keyDown = false;
static bool     keyCandidate = false;
static uint32_t keyCandidateMs = 0;
static uint32_t keyEdgeMs = 0;
static uint16_t keySymbolCode = 1;
static uint8_t  keySymbolLen = 0;
static bool     keySymbolOverflow = false;
static bool     keyWordPending = false;
static bool     sidetoneSounding = false;

/* ---- Persistence -------------------------------------------------------- */
static bool settingsDirty = false;      /* changed since the last save or load */
static bool contrastFromStore = false;  /* contrast came from EEPROM, not a guess */

/* ---- Console ------------------------------------------------------------ */
static char     lineBuf[SERIAL_LINE_MAX];
static uint8_t  lineLen = 0;
static bool     lineOverflow = false;

/* ---- Calibration button ------------------------------------------------- */
static bool     calBtnStable = true;        /* pullup, so true means released */
static bool     calBtnCandidate = true;
static uint32_t calBtnCandidateMs = 0;

/* ---- Self test ---------------------------------------------------------- */
static SelfTestState selfTestState = ST_IDLE;
static uint16_t selfTestPass = 0;
static uint16_t selfTestFail = 0;
static uint16_t stOrdinal = 0;   /* assertion counter within the current group */
static uint16_t stTarget = 0;    /* the one assertion reported this pass */
static bool     stEmitted = false;

/* ============================================================================
 * FORWARD DECLARATIONS
 * ========================================================================= */
static void setTxLed(bool on);
static void recomputeTiming();
static void setSidetone(bool on);
static void keyService(uint32_t now);
static void keyResolveCharacter();
static void keyCloseLine();
static uint16_t crc16(const char *data, uint8_t len);
static uint32_t rngNext();
static void frameHandleLine(uint32_t now);
static void backoffArm(uint32_t now);
static void backoffClear();
static void txService(uint32_t now);
static void txLoadNext(uint32_t now);
static void txAfterCharacter(uint32_t now);
static void txBeginMark(uint32_t now);
static void txBeginGap(uint32_t now, TxState state, uint8_t units);
static void txFinishMessage(uint32_t now);
static void rxService(uint32_t now);
static void rxProcessSample(uint16_t level, uint32_t now);
static void rxHandleMarkEnd(uint32_t durationMs);
static void rxSpaceTimeouts(uint32_t now);
static void rxResolveCharacter();
static void rxEmitChar(char c);
static void rxCloseLine(uint32_t now);
static void rxRearm(uint32_t now, bool reportBreak);
static void rxUpdateThresholds();
static bool gateActive(uint32_t now);
static bool channelBusy(uint32_t now);
static uint16_t rxUnitMs();
static void calService(uint32_t now, uint16_t level);
static void calStart(uint32_t now);
static void calButtonService(uint32_t now);
static void serialService(uint32_t now);
static uint8_t crc8(const uint8_t *data, uint8_t len);
static bool settingsSave();
static bool settingsLoad(bool applyCalibration);
static void settingsDefaults();
static void handleLine(char *line, uint32_t now);
static void queueMessage(const char *text);
static void printHelp();
static void helpService();
static void printStatus(uint32_t now);
static void statusService(uint32_t now);
static void printVersion();
static void selfTestService(uint32_t now);
static void stAssert(bool condition, const char *namePgm);

/* ============================================================================
 * TIMING, SIDETONE AND STRAIGHT KEY
 * ========================================================================= */

/* Recomputes both unit lengths from the current character and overall speeds.
 * Called from anywhere either speed changes so the two can never disagree. */
/* Pure so the self test can hit it directly. c is the character speed, f the
 * overall speed, both in words per minute, f never above c. */
static uint16_t farnsworthGapMs(uint8_t c, uint8_t f) {
  if (c == 0 || f == 0) return 0;
  if (f > c) f = c;
  uint32_t numerator = (FARNSWORTH_WORD_MS * (uint32_t)c) - (FARNSWORTH_CHAR_MS * (uint32_t)f);
  uint32_t denominator = (uint32_t)FARNSWORTH_GAP_UNITS * (uint32_t)c * (uint32_t)f;
  uint32_t gap = numerator / denominator;
  /* Integer division can land a millisecond short of the element unit when the
   * two speeds are equal, which would make the gaps fractionally tight. */
  uint16_t elementUnit = unitMsForWpm(c);
  if (gap < elementUnit) gap = elementUnit;
  return (uint16_t)gap;
}

static void recomputeTiming() {
  if (txWpm < WPM_MIN) txWpm = WPM_MIN;
  if (txWpm > WPM_MAX) txWpm = WPM_MAX;
  /* Farnsworth is opt in. Until the operator asks for it the overall speed
   * tracks the character speed, so raising /wpm cannot leave a stale slower
   * spacing latched on. Once asked for, it is only ever clamped downward. */
  if (!farnsworthExplicit) txFarnsworthWpm = txWpm;
  if (txFarnsworthWpm > txWpm) txFarnsworthWpm = txWpm;
  if (txFarnsworthWpm < WPM_MIN) txFarnsworthWpm = WPM_MIN;
  txUnitMs = unitMsForWpm(txWpm);
  txGapUnitMs = farnsworthGapMs(txWpm, txFarnsworthWpm);
}

static void setSidetone(bool on) {
  if (sidetoneSounding == on) return;
  sidetoneSounding = on;
  if (on && toneEnabled) tone(PIN_SIDETONE, sidetonePitch);
  else noTone(PIN_SIDETONE);
}

/* The straight key is a practice oscillator with a decoder attached. It drives
 * the LED and the sidetone directly, and its own marks and spaces are run
 * through the same classification functions the receiver uses, so what you
 * send comes back as text on a KEY: line. Classification uses the configured
 * character speed rather than an adaptive estimate, because the whole point of
 * practice is to be told what you actually sent, not what you meant. */
static void keyEmitChar(char c) {
  if (rxLineOpen || txEchoLineOpen) closeOpenLines();
  if (!keyLineOpen) { OUTP("KEY: "); keyLineOpen = true; }
  outPutc(c);
}

static void keyCloseLine() {
  if (!keyLineOpen) return;
  closeOpenLines();
  keyWordPending = false;
}

static void keyResolveCharacter() {
  char c = 0;
  if (!keySymbolOverflow) c = morseFindChar(keySymbolCode);
  if (c == 0) c = '?';
  if (keyWordPending) { keyEmitChar(' '); keyWordPending = false; }
  keyEmitChar(c);
  keySymbolCode = 1;
  keySymbolLen = 0;
  keySymbolOverflow = false;
}

static void keyService(uint32_t now) {
  if (!keyFitted) return;

  bool raw = (digitalRead(PIN_KEY) == LOW);
  if (raw != keyCandidate) {
    keyCandidate = raw;
    keyCandidateMs = now;
  } else if (raw != keyDown && elapsed(now, keyCandidateMs, KEY_DEBOUNCE_MS)) {
    uint32_t duration = (uint32_t)(keyCandidateMs - keyEdgeMs);
    if (keyDown) {
      /* A mark just ended. */
      if (keySymbolLen >= MAX_ELEMENTS) {
        keySymbolOverflow = true;
      } else {
        keySymbolCode = (uint16_t)((keySymbolCode << 1) | (markIsDash(duration, txUnitMs) ? 1 : 0));
        keySymbolLen++;
      }
    }
    keyDown = raw;
    keyEdgeMs = keyCandidateMs;
    setTxLed(keyDown);
    setSidetone(keyDown);
  }

  if (keyDown) return;

  uint32_t sinceEdge = (uint32_t)(now - keyEdgeMs);
  if (keySymbolLen > 0 || keySymbolOverflow) {
    if (sinceEdge >= (uint32_t)txUnitMs * 2UL) keyResolveCharacter();
    return;
  }
  if (keyLineOpen) {
    if (!keyWordPending && sinceEdge >= (uint32_t)txGapUnitMs * 5UL) keyWordPending = true;
    if (sinceEdge >= (uint32_t)txGapUnitMs * RX_LINE_IDLE_UNITS) keyCloseLine();
  }
}

/* True while the key is closed or has been closed recently enough that the
 * transmit engine should stay out of the way. */
static bool keyActive(uint32_t now) {
  if (!keyFitted) return false;
  if (keyDown) return true;
  return !elapsed(now, keyEdgeMs, (uint32_t)txGapUnitMs * 7UL);
}

/* ============================================================================
 * TRANSMIT ENGINE
 * ========================================================================= */

static void setTxLed(bool on) {
  if (txLedOn == on) return;
  txLedOn = on;
  digitalWrite(PIN_TX_LED, on ? HIGH : LOW);
  lastLedChangeMs = millis();
}

static void txBeginMark(uint32_t now) {
  uint8_t units = txElemBits[txElemIndex] ? 3 : 1;
  txState = TX_MARK;
  txPhaseStart = now;
  txPhaseLen = (uint32_t)units * txUnitMs;
  txNextChangeMs = now + txPhaseLen;
  setTxLed(true);
  setSidetone(true);
}

/* Elements and the gap between them run at the character speed. The gaps
 * between characters and between words run at the Farnsworth gap unit, which
 * equals the character unit when Farnsworth is off. */
static void txBeginGap(uint32_t now, TxState state, uint8_t units) {
  uint16_t unit = (state == TX_INTRA) ? txUnitMs : txGapUnitMs;
  txState = state;
  txPhaseStart = now;
  txPhaseLen = (uint32_t)units * unit;
  txNextChangeMs = now + txPhaseLen;
  setTxLed(false);
  setSidetone(false);
}

static void txFinishMessage(uint32_t now) {
  setTxLed(false);
  setSidetone(false);
  txState = TX_IDLE;
  txEndedMs = now;
  closeOpenLines();
  OUTP("TX: done chars=");
  outUL(txCharsSent);
  OUTP(" ms=");
  outUL((uint32_t)(now - txMessageStartMs));
  OUTP(" wpm=");
  outUL(txWpm);
  outNL();
  txCharsSent = 0;
  backoffClear();
}

/* Echo one completed character back through the decoder, which proves the
 * encode path and the table agree with each other on live traffic. */
static void txEchoCharacter() {
  if (!echoEnabled) return;
  uint16_t code = elementsToCode(txElemBits, txElemCount);
  char decoded = morseFindChar(code);
  if (decoded == 0) decoded = '?';
  if (rxLineOpen) closeOpenLines();
  if (!txEchoLineOpen) { OUTP("TX+ "); txEchoLineOpen = true; }
  outPutc(decoded);
}

static void txLoadNext(uint32_t now) {
  /* Bounded so a queue full of unencodable bytes cannot stall the loop. */
  for (uint8_t guardCount = 0; guardCount < 8; guardCount++) {
    uint8_t c;
    if (!txQueue.pop(&c)) { txState = TX_IDLE; return; }

    if (c == MSG_END_SENTINEL) { txFinishMessage(now); return; }

    if (c == ' ') {
      /* Leading spaces carry no information, and a word gap is emitted by
       * txAfterCharacter, so a space reaching here is either leading or a
       * duplicate. Either way, skip it. */
      continue;
    }

    uint16_t code = morseFindCode(toUpperAscii((char)c));
    if (code == 0) {
      closeOpenLines();
      OUTP("ERR: TXDROP char=");
      outPutc((c >= 32 && c < 127) ? (char)c : '?');
      OUTP(" code=");
      outUL(c);
      outNL();
      continue;
    }

    codeToElements(code, txElemBits, &txElemCount);
    txElemIndex = 0;
    if (txCharsSent == 0) txMessageStartMs = now;
    txCharsSent++;
    txBeginMark(now);
    return;
  }
  txState = TX_IDLE;
}

/* Decide the gap that follows a completed character by looking at what comes
 * next. Doing it here rather than when the space is popped keeps the word gap
 * at exactly 7 units instead of 3 plus 7. */
static void txAfterCharacter(uint32_t now) {
  txEchoCharacter();

  uint8_t next;
  if (!txQueue.peek(&next)) {
    /* Queue starved mid message, which should not happen because the sentinel
     * is always pushed with the text. Treat as end of message. */
    txFinishMessage(now);
    return;
  }
  if (next == ' ') {
    uint8_t discard;
    while (txQueue.peek(&discard) && discard == ' ') txQueue.pop(&discard);
    /* The echo is meant to be readable text, so the word gap has to show up in
     * it as a space. Without this the echo runs the words together. */
    if (echoEnabled && txEchoLineOpen) outPutc(' ');
    txBeginGap(now, TX_WORDGAP, 7);
    return;
  }
  txBeginGap(now, TX_CHARGAP, 3);
}

static void txService(uint32_t now) {
  if (txAbortRequested && txState != TX_IDLE && txState != TX_DEFER) {
    /* Abort lands on the current element boundary, never mid element, so the
     * far end sees a truncated character rather than a stretched one. */
    if (elapsed(now, txPhaseStart, txPhaseLen)) {
      setTxLed(false);
      setSidetone(false);
      txState = TX_IDLE;
      txEndedMs = now;
      txAbortRequested = false;
      txCharsSent = 0;
      closeOpenLines();
      OUTP("SYS: tx_aborted\n");
    }
    return;
  }

  switch (txState) {
    case TX_IDLE:
      if (txQueue.empty()) return;
      if (keyActive(now)) return;   /* the operator's hand outranks the queue */
      if (channelBusy(now)) {
        txState = TX_DEFER;
        if (!txDeferReported) {
          txDeferReported = true;
          closeOpenLines();
          OUTP("SYS: tx_deferred reason=channel_busy\n");
        }
        return;
      }
      txDeferReported = false;
      txLoadNext(now);
      return;

    case TX_DEFER:
      if (txQueue.empty()) { txState = TX_IDLE; backoffClear(); return; }
      if (keyActive(now)) return;
      if (channelBusy(now)) {
        /* Still occupied, so any wait already drawn is stale. */
        txBackoffArmed = false;
        return;
      }
      if (!txBackoffArmed) { backoffArm(now); return; }
      if (!elapsed(now, txBackoffStart, txBackoffWaitMs)) return;
      txBackoffArmed = false;
      txState = TX_IDLE;
      return;

    case TX_MARK:
      if (!elapsed(now, txPhaseStart, txPhaseLen)) return;
      txElemIndex++;
      if (txElemIndex < txElemCount) {
        txBeginGap(now, TX_INTRA, 1);
      } else {
        txAfterCharacter(now);
      }
      return;

    case TX_INTRA:
      if (!elapsed(now, txPhaseStart, txPhaseLen)) return;
      txBeginMark(now);
      return;

    case TX_CHARGAP:
    case TX_WORDGAP:
      if (!elapsed(now, txPhaseStart, txPhaseLen)) return;
      txLoadNext(now);
      return;
  }
}

/* ============================================================================
 * RECEIVER GATING AND CARRIER SENSE
 * ========================================================================= */

static bool gateActive(uint32_t now) {
  if (calState != CAL_NONE) return true;   /* calibration owns the sensor */

  bool txActive = (txState == TX_MARK || txState == TX_INTRA ||
                   txState == TX_CHARGAP || txState == TX_WORDGAP) || keyDown;

  if (duplexMode == MODE_HALF) {
    if (txActive) return true;
    if (!elapsed(now, txEndedMs, RX_REARM_GUARD_MS)) return true;
    return false;
  }

  /* MODE_FULL: guard only around the transitions themselves. */
  if (!elapsed(now, lastLedChangeMs, TX_GUARD_MS)) return true;
  if (txActive) {
    uint32_t remaining = txNextChangeMs - now;
    if (remaining <= (uint32_t)TX_GUARD_MS) return true;
  }
  return false;
}

/* Advisory carrier sense. Busy means either a mark is being received right
 * now, or the channel has not been quiet for a full word gap. */
static bool channelBusy(uint32_t now) {
  if (rxArm != RX_ARMED) return false;
  if (rxInMark) return true;
  uint32_t quietNeeded = (uint32_t)rxUnitMs() * 7UL;
  return !elapsed(now, rxLastEdgeMs, quietNeeded);
}

/* ============================================================================
 * RECEIVE ENGINE
 * ========================================================================= */

static uint16_t rxUnitMs() {
  int32_t u = rxUnitQ4 >> 4;
  if (u < RX_UNIT_MIN_MS) u = RX_UNIT_MIN_MS;
  if (u > RX_UNIT_MAX_MS) u = RX_UNIT_MAX_MS;
  return (uint16_t)u;
}

static void rxUpdateThresholds() {
  uint16_t base = (uint16_t)(baselineQ4 >> 4);
  uint32_t rise = (uint32_t)base + ((uint32_t)contrastCounts * THRESHOLD_RISE_PCT) / 100UL;
  uint32_t fall = (uint32_t)base + ((uint32_t)contrastCounts * THRESHOLD_FALL_PCT) / 100UL;
  if (rise > ADC_MAX) rise = ADC_MAX;
  if (fall > ADC_MAX) fall = ADC_MAX;
  riseThreshold = (uint16_t)rise;
  fallThreshold = (uint16_t)fall;
}

static void rxRearm(uint32_t now, bool reportBreak) {
  bool hadPartial = (rxSymbolLen > 0) || rxInMark;
  rxInMark = false;
  rxCandidatePending = false;
  rxSymbolCode = 1;
  rxSymbolLen = 0;
  rxSymbolOverflow = false;
  rxLastEdgeMs = now;
  digitalWrite(PIN_STATUS_LED, LOW);
  if (reportBreak && hadPartial) {
    closeOpenLines();
    OUTP("ERR: RXBREAK detail=partial_symbol_discarded\n");
  }
}

static void rxEmitChar(char c) {
  /* The character is both printed live and kept, because framing can only be
   * checked once the whole line has arrived. */
  if (rxLineLen < (uint8_t)(RX_LINE_MAX - 1)) rxLine[rxLineLen++] = c;
  else rxLineTruncated = true;

  if (txEchoLineOpen) closeOpenLines();
  if (!rxLineOpen) {
    outStrP(rxLineContinued ? PSTR("RX+ ") : PSTR("RX: "));
    rxLineOpen = true;
    rxLineContinued = false;
  }
  outPutc(c);
}

static void rxCloseLine(uint32_t now) {
  if (!rxLineOpen && rxCharsThisLine == 0) return;
  closeOpenLines();
  rxLineContinued = false;
  frameHandleLine(now);
  rxLineLen = 0;
  rxLineTruncated = false;
  OUTP("SYS: rx_end chars=");
  outUL(rxCharsThisLine);
  OUTP(" err=");
  outUL(rxErrorsThisLine);
  OUTP(" rxwpm=");
  outUL(wpmForUnitMs(rxUnitMs()));
  OUTP(" unit_ms=");
  outUL(rxUnitMs());
  outNL();
  rxCharsThisLine = 0;
  rxErrorsThisLine = 0;
  rxWordPending = false;
  (void)now;
}

static void rxResolveCharacter() {
  char c = 0;
  if (!rxSymbolOverflow) c = morseFindChar(rxSymbolCode);
  if (c == 0) { c = '?'; rxErrorsThisLine++; }

  if (rxWordPending) { rxEmitChar(' '); rxWordPending = false; }
  rxEmitChar(c);
  rxCharsThisLine++;

  rxSymbolCode = 1;
  rxSymbolLen = 0;
  rxSymbolOverflow = false;
}

/* A mark just ended. Classify it, fold it into the symbol, and let it pull the
 * unit estimate. Dots move the estimate faster than dashes because a dot is a
 * direct measurement of one unit while a dash is a third of a measurement of
 * three, and because dots outnumber dashes in ordinary text. */
static void rxHandleMarkEnd(uint32_t durationMs) {
  uint16_t unit = rxUnitMs();
  bool isDash = markIsDash(durationMs, unit);

  if (!isDash) {
    if (durationMs >= RX_UNIT_MIN_MS && durationMs <= RX_UNIT_MAX_MS) {
      rxUnitQ4 += ((int32_t)(durationMs << 4) - rxUnitQ4) >> 3;
    }
  } else if (durationMs < (uint32_t)unit * 6UL) {
    int32_t impliedQ4 = (int32_t)((durationMs << 4) / 3UL);
    if ((impliedQ4 >> 4) >= RX_UNIT_MIN_MS && (impliedQ4 >> 4) <= RX_UNIT_MAX_MS) {
      rxUnitQ4 += (impliedQ4 - rxUnitQ4) >> 4;
    }
  }
  if ((rxUnitQ4 >> 4) < RX_UNIT_MIN_MS) rxUnitQ4 = (int32_t)RX_UNIT_MIN_MS << 4;
  if ((rxUnitQ4 >> 4) > RX_UNIT_MAX_MS) rxUnitQ4 = (int32_t)RX_UNIT_MAX_MS << 4;

  if (rxSymbolLen >= MAX_ELEMENTS) {
    rxSymbolOverflow = true;
  } else {
    rxSymbolCode = (uint16_t)((rxSymbolCode << 1) | (isDash ? 1 : 0));
    rxSymbolLen++;
  }
}

/* All space classification happens on timeout rather than on the next rising
 * edge, so characters print the moment they are complete instead of waiting
 * for the next mark to arrive. */
static void rxSpaceTimeouts(uint32_t now) {
  if (rxInMark) return;
  uint32_t sinceEdge = (uint32_t)(now - rxLastEdgeMs);
  uint16_t unit = rxUnitMs();

  if (rxSymbolLen > 0 || rxSymbolOverflow) {
    if (sinceEdge >= (uint32_t)unit * 2UL) rxResolveCharacter();
    return;
  }
  if (rxCharsThisLine > 0) {
    if (!rxWordPending && sinceEdge >= (uint32_t)unit * 5UL) rxWordPending = true;
    if (sinceEdge >= (uint32_t)unit * RX_LINE_IDLE_UNITS) rxCloseLine(now);
  }
}

static void rxProcessSample(uint16_t level, uint32_t now) {
  /* Hysteresis: the trip point depends on which state we are already in. */
  bool instant = rxInMark ? (level > fallThreshold) : (level > riseThreshold);

  if (!rxInMark && level <= fallThreshold) {
    baselineQ4 += ((int32_t)(level << 4) - baselineQ4) >> BASELINE_SHIFT;
    rxUpdateThresholds();
  }

  if (instant != rxInMark) {
    if (!rxCandidatePending || rxCandidateState != instant) {
      rxCandidatePending = true;
      rxCandidateState = instant;
      rxCandidateStartMs = now;
    } else if (elapsed(now, rxCandidateStartMs, EDGE_DEBOUNCE_MS)) {
      /* Timestamp the edge where it actually happened, not where the debounce
       * finished, so element durations stay honest. */
      uint32_t edgeMs = rxCandidateStartMs;
      uint32_t duration = (uint32_t)(edgeMs - rxLastEdgeMs);
      rxCandidatePending = false;

      if (rxInMark) {
        rxHandleMarkEnd(duration);
        if (rawEnabled) {
          closeOpenLines();
          OUTP("DBG: EDGE t="); outUL(edgeMs);
          OUTP(" kind=MARK dur="); outUL(duration);
          OUTP(" unit="); outUL(rxUnitMs()); outNL();
        }
      } else if (rawEnabled) {
        closeOpenLines();
        OUTP("DBG: EDGE t="); outUL(edgeMs);
        OUTP(" kind=SPACE dur="); outUL(duration);
        OUTP(" unit="); outUL(rxUnitMs()); outNL();
      }

      rxInMark = instant;
      rxLastEdgeMs = edgeMs;
      digitalWrite(PIN_STATUS_LED, rxInMark ? HIGH : LOW);
    }
  } else {
    rxCandidatePending = false;
  }

  rxSpaceTimeouts(now);
}

static void rxService(uint32_t now) {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastSampleUs) < SAMPLE_INTERVAL_US) return;

  /* Advance by exactly one interval to hold the cadence. If we have fallen
   * more than four intervals behind, something blocked the loop, so resync and
   * count it rather than spinning to catch up. */
  lastSampleUs += SAMPLE_INTERVAL_US;
  if ((uint32_t)(nowUs - lastSampleUs) > (uint32_t)SAMPLE_INTERVAL_US * 4UL) {
    lastSampleUs = nowUs;
    sampleOverruns++;
  }

  int raw = analogRead(PIN_LDR);
  uint16_t level = SENSOR_ACTIVE_HIGH ? (uint16_t)raw : (uint16_t)(ADC_MAX - raw);
  rxLastLevel = level;

  bool gated = gateActive(now);

  if (calState != CAL_NONE) {
    calService(now, level);
  } else if (gated) {
    if (!rxGatedPrev) {
      /* Just entered the gate with the receiver mid symbol. */
      rxRearm(now, true);
    }
  } else {
    if (rxGatedPrev) rxRearm(now, false);
    if (rxArm == RX_ARMED) rxProcessSample(level, now);
  }
  rxGatedPrev = gated;

  if (rawEnabled && elapsed(now, rawLastReportMs, RAW_REPORT_MS)) {
    rawLastReportMs = now;
    closeOpenLines();
    OUTP("DBG: RAW t="); outUL(now);
    OUTP(" adc="); outUL(level);
    OUTP(" base="); outUL((uint32_t)(baselineQ4 >> 4));
    OUTP(" rise="); outUL(riseThreshold);
    OUTP(" fall="); outUL(fallThreshold);
    OUTP(" mark="); outPutc(rxInMark ? '1' : '0');
    OUTP(" gate="); outPutc(gated ? '1' : '0');
    outNL();
  }
}

/* ============================================================================
 * CALIBRATION
 * ========================================================================= */

static void calStart(uint32_t now) {
  if (txState != TX_IDLE || !txQueue.empty()) {
    closeOpenLines();
    OUTP("ERR: BUSY detail=cal_needs_idle_transmitter\n");
    return;
  }
  calState = CAL_DARK;
  calPhaseStart = now;
  calSum = 0;
  calSamples = 0;
  setTxLed(false);
  closeOpenLines();
  OUTP("SYS: cal_start phase=dark ms=");
  outUL(CAL_PHASE_MS);
  outNL();
}

/* Fed one sample per sampler tick while a calibration phase is running. */
static void calService(uint32_t now, uint16_t level) {
  calSum += level;
  calSamples++;

  uint32_t phaseLen = (calState == CAL_BOOT) ? BOOT_SETTLE_MS : CAL_PHASE_MS;
  if (!elapsed(now, calPhaseStart, phaseLen)) return;

  uint16_t mean = (calSamples > 0) ? (uint16_t)(calSum / calSamples) : 0;

  switch (calState) {
    case CAL_BOOT:
      baselineQ4 = (int32_t)mean << 4;
      if (!contrastFromStore) contrastCounts = DEFAULT_CONTRAST_COUNTS;
      rxUpdateThresholds();
      rxArm = RX_ARMED;
      calState = CAL_NONE;
      rxRearm(now, false);
      closeOpenLines();
      OUTP("SYS: boot_cal dark="); outUL(mean);
      OUTP(" contrast="); outUL(contrastCounts);
      OUTP(" rise="); outUL(riseThreshold);
      OUTP(" fall="); outUL(fallThreshold);
      OUTP(" source="); outStrP(contrastFromStore ? PSTR("eeprom\n") : PSTR("assumed\n"));
      if (!contrastFromStore) OUTP("SYS: hint run /cal for a measured contrast\n");
      break;

    case CAL_DARK:
      calDarkMean = mean;
      calState = CAL_LIT;
      calPhaseStart = now;
      calSum = 0;
      calSamples = 0;
      setTxLed(true);
      closeOpenLines();
      OUTP("SYS: cal_phase phase=lit dark="); outUL(calDarkMean); outNL();
      break;

    case CAL_LIT: {
      setTxLed(false);
      calState = CAL_NONE;
      uint16_t lit = mean;
      closeOpenLines();
      if (lit <= calDarkMean || (uint16_t)(lit - calDarkMean) < MIN_CONTRAST_COUNTS) {
        rxArm = RX_DISARMED;
        OUTP("ERR: LOWCONTRAST dark="); outUL(calDarkMean);
        OUTP(" lit="); outUL(lit);
        OUTP(" contrast="); outI32((int32_t)lit - (int32_t)calDarkMean);
        OUTP(" need="); outUL(MIN_CONTRAST_COUNTS);
        outNL();
        /* Kept short deliberately: this whole block is emitted in one burst
         * and has to fit inside the output ring. */
        OUTP("SYS: fix close the gap, shade the sensor, retune Rfixed\n");
        OUTP("SYS: fix or check SENSOR_ACTIVE_HIGH matches the divider\n");
        OUTP("SYS: receiver=disarmed\n");
      } else {
        contrastCounts = (uint16_t)(lit - calDarkMean);
        contrastFromStore = false;
        settingsDirty = true;
        baselineQ4 = (int32_t)calDarkMean << 4;
        rxUpdateThresholds();
        rxArm = RX_ARMED;
        rxRearm(now, false);
        OUTP("SYS: cal_done dark="); outUL(calDarkMean);
        OUTP(" lit="); outUL(lit);
        OUTP(" contrast="); outUL(contrastCounts);
        OUTP(" rise="); outUL(riseThreshold);
        OUTP(" fall="); outUL(fallThreshold);
        OUTP(" receiver=armed\n");
      }
      break;
    }

    case CAL_NONE:
    default:
      break;
  }
}

static void calButtonService(uint32_t now) {
  if (!CAL_BUTTON_FITTED) return;
  bool level = (digitalRead(PIN_CAL_BUTTON) != LOW);  /* true means released */
  if (level != calBtnCandidate) {
    calBtnCandidate = level;
    calBtnCandidateMs = now;
    return;
  }
  if (calBtnCandidate == calBtnStable) return;
  if (!elapsed(now, calBtnCandidateMs, CAL_BUTTON_DEBOUNCE_MS)) return;
  calBtnStable = calBtnCandidate;
  if (calBtnStable == false && calState == CAL_NONE) calStart(now);
}

/* ============================================================================
 * PERSISTENCE
 *
 * Eleven bytes at the bottom of EEPROM hold the operator settings and the
 * measured calibration. Saving is explicit rather than automatic: an EEPROM
 * cell is good for about 100000 writes, and a sketch that saved on every
 * /wpm would burn through that faster than anyone expects. /status carries an
 * unsaved flag so nothing is lost by surprise.
 *
 * The stored dark level is deliberately NOT restored as the live baseline.
 * Ambient light is a property of the room right now, and the boot settle
 * measures it directly. What is worth restoring is the contrast, which is a
 * property of the LED, the sensor, and the spacing between them, none of
 * which change when someone opens a blind.
 * ========================================================================= */

/* The settings block is serialised by hand into a flat byte array rather than
 * being memcpy'd out of a struct. A struct of mixed width fields picks up
 * alignment padding, the padding bytes are uninitialised, and any check byte
 * computed over sizeof(struct) is then unstable between builds and between
 * architectures. Explicit offsets have none of that ambiguity.
 *
 *   [0..1]  magic     [2]  layout    [3]  wpm      [4]  mode
 *   [5]     echo      [6..7] contrast [8..9] dark
 *   [10]    farnsworth overall wpm    [11] flags, bit 0 is sidetone
 *   [12..13] sidetone pitch           [14] flags2, bit 0 is framing
 *   [15..22] callsign, NUL padded     [23..24] reserved, written as zero
 *   [25]    crc8 over bytes 0 through 24
 */
static const uint8_t SETTINGS_BYTES = 26;

/* CRC-8/ATM, polynomial 0x07, initial value 0x00, no reflection, no final
 * xor. Chosen for size, not for strength: it only has to catch a write that
 * was interrupted by a reset. */
static uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ CRC8_POLYNOMIAL) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static void settingsPack(uint8_t *b) {
  b[0] = (uint8_t)(SETTINGS_MAGIC & 0xFF);
  b[1] = (uint8_t)(SETTINGS_MAGIC >> 8);
  b[2] = SETTINGS_LAYOUT;
  b[3] = txWpm;
  b[4] = (uint8_t)duplexMode;
  b[5] = echoEnabled ? 1 : 0;
  b[6] = (uint8_t)(contrastCounts & 0xFF);
  b[7] = (uint8_t)(contrastCounts >> 8);
  uint16_t dark = (uint16_t)(baselineQ4 >> 4);
  b[8] = (uint8_t)(dark & 0xFF);
  b[9] = (uint8_t)(dark >> 8);
  b[10] = txFarnsworthWpm;
  b[11] = toneEnabled ? 0x01 : 0x00;
  b[12] = (uint8_t)(sidetonePitch & 0xFF);
  b[13] = (uint8_t)(sidetonePitch >> 8);
  b[14] = frameEnabled ? 0x01 : 0x00;
  for (uint8_t i = 0; i < CALLSIGN_MAX; i++) b[15 + i] = (uint8_t)myCall[i];
  b[23] = 0;
  b[24] = 0;
  b[SETTINGS_BYTES - 1] = crc8(b, (uint8_t)(SETTINGS_BYTES - 1));
}

static bool settingsSave() {
#if MORSE_HAS_EEPROM
  uint8_t b[SETTINGS_BYTES];
  settingsPack(b);
  for (uint8_t i = 0; i < SETTINGS_BYTES; i++) EEPROM.update(SETTINGS_ADDRESS + i, b[i]);
  settingsDirty = false;
  return true;
#else
  return false;
#endif
}

/* Returns true when a valid block was found and applied. applyCalibration is
 * false during a plain /load so a stale contrast cannot quietly arm a
 * receiver that has not been calibrated in this session. */
static bool settingsLoad(bool applyCalibration) {
#if MORSE_HAS_EEPROM
  uint8_t b[SETTINGS_BYTES];
  for (uint8_t i = 0; i < SETTINGS_BYTES; i++) b[i] = EEPROM.read(SETTINGS_ADDRESS + i);

  uint16_t magic = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
  if (magic != SETTINGS_MAGIC) {
    closeOpenLines();
    OUTP("SYS: eeprom=blank\n");
    return false;
  }
  if (b[2] != SETTINGS_LAYOUT) {
    closeOpenLines();
    OUTP("ERR: EEPROMLAYOUT found="); outUL(b[2]);
    OUTP(" want="); outUL(SETTINGS_LAYOUT); outNL();
    return false;
  }
  if (b[SETTINGS_BYTES - 1] != crc8(b, (uint8_t)(SETTINGS_BYTES - 1))) {
    closeOpenLines();
    OUTP("ERR: EEPROMCRC detail=stored_settings_ignored\n");
    return false;
  }

  if (b[3] >= WPM_MIN && b[3] <= WPM_MAX) {
    txWpm = b[3];
    txUnitMs = unitMsForWpm(txWpm);
  }
  duplexMode  = (b[4] == (uint8_t)MODE_FULL) ? MODE_FULL : MODE_HALF;
  echoEnabled = (b[5] != 0);
  if (b[10] >= WPM_MIN && b[10] < txWpm) { txFarnsworthWpm = b[10]; farnsworthExplicit = true; }
  else { txFarnsworthWpm = txWpm; farnsworthExplicit = false; }
  toneEnabled = ((b[11] & 0x01) != 0);
  uint16_t pitch = (uint16_t)b[12] | ((uint16_t)b[13] << 8);
  if (pitch >= PITCH_MIN && pitch <= PITCH_MAX) sidetonePitch = pitch;
  frameEnabled = ((b[14] & 0x01) != 0);
  {
    uint8_t n = 0;
    while (n < CALLSIGN_MAX && b[15 + n] != 0) { myCall[n] = (char)b[15 + n]; n++; }
    myCall[n] = 0;
    if (n == 0) { myCall[0] = 'N'; myCall[1] = 'O'; myCall[2] = 'C'; myCall[3] = 'A';
                  myCall[4] = 'L'; myCall[5] = 'L'; myCall[6] = 0; }
  }
  recomputeTiming();
  uint16_t storedContrast = (uint16_t)b[6] | ((uint16_t)b[7] << 8);
  if (applyCalibration && storedContrast >= MIN_CONTRAST_COUNTS && storedContrast <= ADC_MAX) {
    contrastCounts = storedContrast;
  }
  settingsDirty = false;
  closeOpenLines();
  OUTP("SYS: eeprom=loaded wpm="); outUL(txWpm);
  OUTP(" mode="); outStrP(duplexMode == MODE_HALF ? PSTR("half") : PSTR("full"));
  OUTP(" echo="); outStrP(echoEnabled ? PSTR("on") : PSTR("off"));
  OUTP(" contrast="); outUL(contrastCounts);
  OUTP(" call="); outStr(myCall);
  OUTP(" frame="); outStrP(frameEnabled ? PSTR("on") : PSTR("off"));
  OUTP(" cal="); outStrP(applyCalibration ? PSTR("applied") : PSTR("held"));
  outNL();
  return true;
#else
  (void)applyCalibration;
  closeOpenLines();
  OUTP("ERR: NOEEPROM detail=not_supported_on_this_board\n");
  return false;
#endif
}

static void settingsDefaults() {
  txWpm = WPM_DEFAULT;
  txUnitMs = unitMsForWpm(txWpm);
  duplexMode = MODE_HALF;
  echoEnabled = false;
  farnsworthExplicit = false;
  txFarnsworthWpm = txWpm;
  frameEnabled = false;
  toneEnabled = false;
  sidetonePitch = PITCH_DEFAULT;
  noTone(PIN_SIDETONE);
  sidetoneSounding = false;
  recomputeTiming();
  contrastCounts = DEFAULT_CONTRAST_COUNTS;
  rxUpdateThresholds();
  settingsDirty = true;
  closeOpenLines();
  OUTP("SYS: defaults wpm="); outUL(txWpm);
  OUTP(" mode=half echo=off contrast="); outUL(contrastCounts);
  OUTP(" note=not_saved\n");
}

/* ============================================================================
 * FRAMING, CHECK BYTES AND BACKOFF
 * ========================================================================= */

/* CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no reflection,
 * no final xor. Strong enough to make a corrupted frame vanishingly unlikely
 * to pass, and small enough to sit in an ATmega without complaint. */
static uint16_t crc16(const char *data, uint8_t len) {
  uint16_t crc = CRC16_INIT;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= (uint16_t)((uint8_t)data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ CRC16_POLYNOMIAL) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static char hexDigit(uint8_t nibble) {
  nibble &= 0x0F;
  return (char)((nibble < 10) ? ('0' + nibble) : ('A' + (nibble - 10)));
}

static bool hexValue(char c, uint8_t *out) {
  if (c >= '0' && c <= '9') { *out = (uint8_t)(c - '0'); return true; }
  if (c >= 'A' && c <= 'F') { *out = (uint8_t)(10 + c - 'A'); return true; }
  return false;
}

/* xorshift32. Not cryptographic and does not need to be: it only has to make
 * two stations pick different waits. Seeded from the low bits of micros() and
 * a floating ADC read, both of which are noisy at power on. */
static uint32_t rngNext() {
  if (rngState == 0) {
    rngState = micros() ^ ((uint32_t)analogRead(PIN_LDR) << 16) ^ 0xA5A5A5A5UL;
    if (rngState == 0) rngState = 1;
  }
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return rngState;
}

static void backoffClear() {
  txBackoffAttempt = 0;
  txBackoffArmed = false;
}

/* Doubling window, uniform draw inside it. The wait is measured from now, so a
 * station that has been waiting a long time already does not get to jump the
 * queue on the next attempt. */
static uint32_t backoffWindowMs(uint8_t attempt, uint16_t gapUnitMs) {
  uint8_t shift = attempt;
  if (shift > BACKOFF_MAX_SHIFT) shift = BACKOFF_MAX_SHIFT;
  uint32_t wordGap = (uint32_t)gapUnitMs * 7UL;
  return wordGap << shift;
}

static void backoffArm(uint32_t now) {
  uint32_t window = backoffWindowMs(txBackoffAttempt, txGapUnitMs);
  uint32_t wait = rngNext() % window;
  txBackoffStart = now;
  txBackoffWaitMs = (uint16_t)((wait > 65535UL) ? 65535UL : wait);
  txBackoffArmed = true;
  if (txBackoffAttempt < 255) txBackoffAttempt++;
  if (txBackoffAttempt == BACKOFF_REPORT_AFTER) {
    closeOpenLines();
    OUTP("SYS: backoff attempt="); outUL(txBackoffAttempt);
    OUTP(" window_ms="); outUL(window);
    OUTP(" wait_ms="); outUL(wait); outNL();
  }
}

/* Copies one space delimited token. Returns a pointer past the token, or NULL
 * if there was nothing left to read. */
static const char *takeToken(const char *p, char *out, uint8_t max) {
  while (*p == ' ') p++;
  if (*p == 0) return NULL;
  uint8_t n = 0;
  while (*p && *p != ' ') {
    if (n < max) out[n++] = *p;
    p++;
  }
  out[n] = 0;
  return p;
}

/* Builds a frame for the given text straight into the transmit queue. Returns
 * false when the text will not fit or carries a character the framing itself
 * uses. */
static bool framePush(const char *text) {
  uint8_t len = 0;
  while (text[len]) len++;
  if (len > FRAME_TEXT_MAX) return false;

  char folded[FRAME_TEXT_MAX + 1];
  for (uint8_t i = 0; i < len; i++) {
    char c = toUpperAscii(text[i]);
    if (c == '=' || c == '+') return false;
    folded[i] = c;
  }
  folded[len] = 0;

  uint16_t crc = crc16(folded, len);

  txQueue.push('=');
  txQueue.push(' ');
  for (const char *p = myCall; *p; p++) txQueue.push((uint8_t)*p);
  txQueue.push(' ');
  for (const char *p = toCall; *p; p++) txQueue.push((uint8_t)*p);
  txQueue.push(' ');
  if (len >= 10) txQueue.push((uint8_t)('0' + (len / 10)));
  txQueue.push((uint8_t)('0' + (len % 10)));
  txQueue.push(' ');
  txQueue.push('=');
  txQueue.push(' ');
  for (uint8_t i = 0; i < len; i++) txQueue.push((uint8_t)folded[i]);
  txQueue.push(' ');
  txQueue.push('=');
  txQueue.push(' ');
  txQueue.push((uint8_t)hexDigit((uint8_t)(crc >> 12)));
  txQueue.push((uint8_t)hexDigit((uint8_t)(crc >> 8)));
  txQueue.push((uint8_t)hexDigit((uint8_t)(crc >> 4)));
  txQueue.push((uint8_t)hexDigit((uint8_t)crc));
  txQueue.push(' ');
  txQueue.push('+');
  return !txQueue.full();
}

enum FrameResult : uint8_t {
  FRAME_NOT_A_FRAME,
  FRAME_MALFORMED,
  FRAME_BAD_LENGTH,
  FRAME_BAD_CRC,
  FRAME_OK
};

/* Parses a completed receive line. textOut is filled with the payload when the
 * result is FRAME_OK, FRAME_BAD_LENGTH or FRAME_BAD_CRC, so a damaged frame
 * can still be shown to the operator rather than thrown away silently. */
static FrameResult frameParse(const char *line, char *fromOut, char *toOut,
                              char *textOut, uint8_t *textLenOut) {
  const char *p = line;
  while (*p == ' ') p++;
  if (p[0] != '=' || p[1] != ' ') return FRAME_NOT_A_FRAME;
  p += 2;

  char lenTok[4];
  p = takeToken(p, fromOut, CALLSIGN_MAX);      if (!p) return FRAME_MALFORMED;
  p = takeToken(p, toOut, CALLSIGN_MAX);        if (!p) return FRAME_MALFORMED;
  p = takeToken(p, lenTok, 3);                  if (!p) return FRAME_MALFORMED;

  char sep[2];
  p = takeToken(p, sep, 1);                     if (!p) return FRAME_MALFORMED;
  if (sep[0] != '=') return FRAME_MALFORMED;

  uint16_t declared = 0;
  for (uint8_t i = 0; lenTok[i]; i++) {
    if (lenTok[i] < '0' || lenTok[i] > '9') return FRAME_MALFORMED;
    declared = (uint16_t)(declared * 10 + (lenTok[i] - '0'));
  }

  /* The trailer is a fixed shape, so find it from the end rather than trying to
   * decide where free text stops. */
  uint8_t total = 0;
  while (line[total]) total++;
  if (total < 2) return FRAME_MALFORMED;
  int16_t t = (int16_t)total - 1;
  while (t >= 0 && line[t] == ' ') t--;
  if (t < 0 || line[t] != '+') return FRAME_MALFORMED;
  t--;
  while (t >= 0 && line[t] == ' ') t--;
  if (t < 3) return FRAME_MALFORMED;

  uint16_t crcSeen = 0;
  for (int8_t i = 3; i >= 0; i--) {
    uint8_t v;
    if (!hexValue(line[t - i], &v)) return FRAME_MALFORMED;
    crcSeen = (uint16_t)((crcSeen << 4) | v);
  }
  int16_t textEnd = (int16_t)(t - 4);
  while (textEnd >= 0 && line[textEnd] == ' ') textEnd--;
  if (textEnd < 0 || line[textEnd] != '=') return FRAME_MALFORMED;
  textEnd--;
  while (textEnd >= 0 && line[textEnd] == ' ') textEnd--;

  int16_t textStart = (int16_t)(p - line);
  while (line[textStart] == ' ') textStart++;
  if (textEnd < textStart) return FRAME_MALFORMED;

  uint8_t n = 0;
  for (int16_t i = textStart; i <= textEnd && n < FRAME_TEXT_MAX; i++) textOut[n++] = line[i];
  textOut[n] = 0;
  *textLenOut = n;

  if (declared != n) return FRAME_BAD_LENGTH;
  if (crc16(textOut, n) != crcSeen) return FRAME_BAD_CRC;
  return FRAME_OK;
}

/* Called once a received line has gone quiet. Unframed traffic is reported as
 * it always was, so a v1.3 station and a v1.0 station can still hear each
 * other, just without any of the guarantees. */
static void frameHandleLine(uint32_t now) {
  if (rxLineLen == 0) return;
  rxLine[rxLineLen] = 0;

  char from[CALLSIGN_MAX + 1];
  char to[CALLSIGN_MAX + 1];
  char text[FRAME_TEXT_MAX + 1];
  uint8_t textLen = 0;
  FrameResult r = frameParse(rxLine, from, to, text, &textLen);

  closeOpenLines();
  switch (r) {
    case FRAME_OK:
      OUTP("RX: frame from="); outStr(from);
      OUTP(" to="); outStr(to);
      OUTP(" len="); outUL(textLen);
      OUTP(" crc=ok\n");
      OUTP("RX: text "); outStr(text); outNL();
      break;
    case FRAME_BAD_CRC:
      OUTP("ERR: BADCRC from="); outStr(from);
      OUTP(" len="); outUL(textLen); outNL();
      OUTP("RX+ "); outStr(text); outNL();
      /* A frame that arrived damaged is the best evidence available that
       * somebody else was transmitting at the same time. */
      if (!txQueue.empty()) backoffArm(now);
      break;
    case FRAME_BAD_LENGTH:
      OUTP("ERR: BADLEN from="); outStr(from);
      OUTP(" got="); outUL(textLen); outNL();
      if (!txQueue.empty()) backoffArm(now);
      break;
    case FRAME_MALFORMED:
      OUTP("ERR: BADFRAME detail=header_or_trailer_unreadable\n");
      if (!txQueue.empty()) backoffArm(now);
      break;
    case FRAME_NOT_A_FRAME:
    default:
      if (frameEnabled) OUTP("SYS: unframed_traffic\n");
      break;
  }
  if (rxLineTruncated) OUTP("ERR: RXLONG detail=line_buffer_full\n");
}

/* ============================================================================
 * SERIAL CONSOLE
 * ========================================================================= */

/* Case insensitive prefix match. Returns a pointer to the first argument
 * character, or NULL when the command does not match. */
static const char *matchCmd(const char *line, const char *cmdPgm) {
  uint8_t i = 0;
  for (;;) {
    char want = (char)pgm_read_byte(cmdPgm + i);
    if (want == 0) break;
    if (toUpperAscii(line[i]) != toUpperAscii(want)) return NULL;
    i++;
  }
  if (line[i] != 0 && line[i] != ' ') return NULL;
  while (line[i] == ' ') i++;
  return &line[i];
}

static bool argIsWord(const char *arg, const char *wordPgm) {
  uint8_t i = 0;
  for (;;) {
    char want = (char)pgm_read_byte(wordPgm + i);
    if (want == 0) return (arg[i] == 0);
    if (toUpperAscii(arg[i]) != toUpperAscii(want)) return false;
    i++;
  }
}

static bool parseUint(const char *s, uint16_t *out) {
  if (*s < '0' || *s > '9') return false;
  uint32_t v = 0;
  while (*s >= '0' && *s <= '9') {
    v = v * 10 + (uint32_t)(*s - '0');
    if (v > 65535UL) return false;
    s++;
  }
  if (*s != 0) return false;
  *out = (uint16_t)v;
  return true;
}

static void printVersion() {
  closeOpenLines();
  OUTP("SYS: name=");
  outStr(PROJECT_NAME);
  OUTP(" version=");
  outStr(PROJECT_VERSION);
  OUTP(" license=GPL-3.0\n");
}

/* The help text is far larger than the output ring, so it is emitted one line
 * per pass of loop() rather than in a single burst that would be truncated. */
static const uint8_t HELP_LINE_COUNT = 23;
static uint8_t helpIndex = HELP_LINE_COUNT;   /* idle when equal to the count */

static void printHelp() { helpIndex = 0; }

static void helpService() {
  if (helpIndex >= HELP_LINE_COUNT) return;
  closeOpenLines();
  switch (helpIndex) {
    case 0:  OUTP("SYS: help plain lines are transmitted, /lines are commands\n"); break;
    case 1:  OUTP("SYS: help /help            this list\n"); break;
    case 2:  OUTP("SYS: help /wpm <5..30>     set transmit speed\n"); break;
    case 3:  OUTP("SYS: help /cal             guided ambient calibration\n"); break;
    case 4:  OUTP("SYS: help /status          engine status\n"); break;
    case 5:  OUTP("SYS: help /mode <half|full> duplex mode\n"); break;
    case 6:  OUTP("SYS: help /echo <on|off>   echo sent chars as decoded text\n"); break;
    case 7:  OUTP("SYS: help /raw <on|off>    stream ADC values and edges\n"); break;
    case 8:  OUTP("SYS: help /loop            built in self test\n"); break;
    case 9:  OUTP("SYS: help /clear           flush queue, abort cleanly\n"); break;
    case 10: OUTP("SYS: help /version         name and version\n"); break;
    case 11: OUTP("SYS: help /save            store settings in EEPROM\n"); break;
    case 12: OUTP("SYS: help /load            reload stored settings\n"); break;
    case 13: OUTP("SYS: help /defaults        settings back to compiled defaults\n"); break;
    case 14: OUTP("SYS: help /fw <wpm|off>    Farnsworth overall speed\n"); break;
    case 15: OUTP("SYS: help /tone <on|off>   sidetone on the piezo\n"); break;
    case 16: OUTP("SYS: help /pitch <300..1200> sidetone frequency\n"); break;
    case 17: OUTP("SYS: help /key <on|off>    enable the straight key input\n"); break;
    case 18: OUTP("SYS: help /call <sign>     your callsign, used in frames\n"); break;
    case 19: OUTP("SYS: help /to <sign>       destination callsign, default CQ\n"); break;
    case 20: OUTP("SYS: help /frame <on|off>  send framed, checked messages\n"); break;
    case 21: OUTP("SYS: help prosigns + AR, = BT, * SK, # error\n"); break;
    default: OUTP("SYS: help prosigns + AR, = BT, * SK, # error\n"); break;
  }
  helpIndex++;
}

/* Three lines is more than the output ring can take in one burst, so status
 * pages out one line per pass of loop() the same way help does. */
static const uint8_t STATUS_LINE_COUNT = 3;
static uint8_t statusIndex = STATUS_LINE_COUNT;

static void printStatus(uint32_t now) { statusIndex = 0; (void)now; }

static void statusService(uint32_t now) {
  if (statusIndex >= STATUS_LINE_COUNT) return;
  closeOpenLines();
  switch (statusIndex) {
    case 0:
      OUTP("SYS: status wpm="); outUL(txWpm);
      OUTP(" unit_ms="); outUL(txUnitMs);
      OUTP(" fw="); outUL(txFarnsworthWpm);
      OUTP(" gap_unit_ms="); outUL(txGapUnitMs);
      OUTP(" rxwpm="); outUL(wpmForUnitMs(rxUnitMs()));
      OUTP(" rxunit_ms="); outUL(rxUnitMs());
      OUTP(" mode="); outStrP(duplexMode == MODE_HALF ? PSTR("half") : PSTR("full"));
      OUTP(" rx="); outStrP(rxArm == RX_ARMED ? PSTR("armed") : PSTR("disarmed"));
      outNL();
      break;
    case 1:
      OUTP("SYS: status baseline="); outUL((uint32_t)(baselineQ4 >> 4));
      OUTP(" adc="); outUL(rxLastLevel);
      OUTP(" contrast="); outUL(contrastCounts);
      OUTP(" rise="); outUL(riseThreshold);
      OUTP(" fall="); outUL(fallThreshold);
      outNL();
      break;
    default:
      OUTP("SYS: status queue="); outUL(txQueue.count());
      OUTP("/"); outUL(txQueue.capacityUsable());
      OUTP(" dropped="); outUL(txQueue.droppedCount);
      OUTP(" txstate="); outUL((uint32_t)txState);
      OUTP(" uptime_ms="); outUL(now);
      OUTP(" freeram="); outI32(freeRamBytes());
      OUTP(" sampleovr="); outUL(sampleOverruns);
      OUTP(" echo="); outStrP(echoEnabled ? PSTR("on") : PSTR("off"));
      OUTP(" raw="); outStrP(rawEnabled ? PSTR("on") : PSTR("off"));
      OUTP(" unsaved="); outPutc(settingsDirty ? '1' : '0');
      OUTP(" tone="); outStrP(toneEnabled ? PSTR("on") : PSTR("off"));
      OUTP(" pitch="); outUL(sidetonePitch);
      OUTP(" key="); outStrP(keyFitted ? PSTR("on") : PSTR("off"));
      OUTP(" frame="); outStrP(frameEnabled ? PSTR("on") : PSTR("off"));
      OUTP(" call="); outStr(myCall);
      OUTP(" to="); outStr(toCall);
      OUTP(" backoff="); outUL(txBackoffAttempt);
      outNL();
      break;
  }
  statusIndex++;
}

static void queueMessage(const char *text) {
  uint16_t accepted = 0;
  bool overflowed = false;

  if (frameEnabled) {
    if (!framePush(text)) {
      closeOpenLines();
      OUTP("ERR: BADTEXT max="); outUL(FRAME_TEXT_MAX);
      OUTP(" detail=too_long_or_contains_frame_characters\n");
      return;
    }
    txQueue.push(MSG_END_SENTINEL);
    closeOpenLines();
    OUTP("TX: framed to="); outStr(toCall);
    OUTP(" depth="); outUL(txQueue.count()); outNL();
    return;
  }

  for (const char *p = text; *p; p++) {
    if (!txQueue.push((uint8_t)*p)) { overflowed = true; break; }
    accepted++;
  }
  if (!txQueue.push(MSG_END_SENTINEL)) overflowed = true;

  closeOpenLines();
  if (overflowed) {
    OUTP("ERR: TXFULL accepted="); outUL(accepted);
    OUTP(" queue="); outUL(txQueue.count()); outNL();
  }
  OUTP("TX: queued chars="); outUL(accepted);
  OUTP(" depth="); outUL(txQueue.count()); outNL();
}

static void handleLine(char *line, uint32_t now) {
  if (line[0] != '/') {
    if (line[0] == 0) return;
    queueMessage(line);
    return;
  }

  const char *arg;

  if ((arg = matchCmd(line, PSTR("/help"))) != NULL) { printHelp(); return; }
  if ((arg = matchCmd(line, PSTR("/version"))) != NULL) { printVersion(); return; }
  if ((arg = matchCmd(line, PSTR("/status"))) != NULL) { printStatus(now); return; }
  if ((arg = matchCmd(line, PSTR("/cal"))) != NULL) { calStart(now); return; }

  if ((arg = matchCmd(line, PSTR("/save"))) != NULL) {
    if (settingsSave()) {
      closeOpenLines();
      OUTP("SYS: saved bytes="); outUL(SETTINGS_BYTES); outNL();
    } else {
      closeOpenLines();
      OUTP("ERR: NOEEPROM detail=not_supported_on_this_board\n");
    }
    return;
  }

  if ((arg = matchCmd(line, PSTR("/load"))) != NULL) {
    settingsLoad(false);
    rxUpdateThresholds();
    return;
  }

  if ((arg = matchCmd(line, PSTR("/defaults"))) != NULL) { settingsDefaults(); return; }

  if ((arg = matchCmd(line, PSTR("/wpm"))) != NULL) {
    uint16_t v;
    if (!parseUint(arg, &v) || v < WPM_MIN || v > WPM_MAX) {
      closeOpenLines();
      OUTP("ERR: BADARG cmd=wpm min="); outUL(WPM_MIN);
      OUTP(" max="); outUL(WPM_MAX); outNL();
      return;
    }
    txWpm = (uint8_t)v;
    recomputeTiming();
    settingsDirty = true;
    closeOpenLines();
    OUTP("SYS: wpm="); outUL(txWpm);
    OUTP(" unit_ms="); outUL(txUnitMs);
    OUTP(" fw="); outUL(txFarnsworthWpm);
    OUTP(" gap_unit_ms="); outUL(txGapUnitMs); outNL();
    return;
  }

  if ((arg = matchCmd(line, PSTR("/fw"))) != NULL) {
    uint16_t v;
    if (argIsWord(arg, PSTR("off"))) {
      farnsworthExplicit = false;
      txFarnsworthWpm = txWpm;
    } else if (parseUint(arg, &v) && v >= WPM_MIN && v <= txWpm) {
      farnsworthExplicit = (v != txWpm);
      txFarnsworthWpm = (uint8_t)v;
    } else {
      closeOpenLines();
      OUTP("ERR: BADARG cmd=fw min="); outUL(WPM_MIN);
      OUTP(" max="); outUL(txWpm);
      OUTP(" or=off\n");
      return;
    }
    recomputeTiming();
    settingsDirty = true;
    closeOpenLines();
    OUTP("SYS: fw="); outUL(txFarnsworthWpm);
    OUTP(" char_wpm="); outUL(txWpm);
    OUTP(" unit_ms="); outUL(txUnitMs);
    OUTP(" gap_unit_ms="); outUL(txGapUnitMs); outNL();
    return;
  }

  if ((arg = matchCmd(line, PSTR("/tone"))) != NULL) {
    if (argIsWord(arg, PSTR("on"))) toneEnabled = true;
    else if (argIsWord(arg, PSTR("off"))) { toneEnabled = false; noTone(PIN_SIDETONE); sidetoneSounding = false; }
    else { closeOpenLines(); OUTP("ERR: BADARG cmd=tone want=on|off\n"); return; }
    settingsDirty = true;
    closeOpenLines();
    OUTP("SYS: tone="); outStrP(toneEnabled ? PSTR("on") : PSTR("off"));
    OUTP(" pitch="); outUL(sidetonePitch); outNL();
    return;
  }

  if ((arg = matchCmd(line, PSTR("/pitch"))) != NULL) {
    uint16_t v;
    if (!parseUint(arg, &v) || v < PITCH_MIN || v > PITCH_MAX) {
      closeOpenLines();
      OUTP("ERR: BADARG cmd=pitch min="); outUL(PITCH_MIN);
      OUTP(" max="); outUL(PITCH_MAX); outNL();
      return;
    }
    sidetonePitch = v;
    settingsDirty = true;
    closeOpenLines();
    OUTP("SYS: pitch="); outUL(sidetonePitch); outNL();
    return;
  }

  if ((arg = matchCmd(line, PSTR("/call"))) != NULL) {
    uint8_t n = 0;
    while (arg[n] && arg[n] != ' ' && n < CALLSIGN_MAX) {
      char c = toUpperAscii(arg[n]);
      if (morseFindCode(c) == 0) {
        closeOpenLines();
        OUTP("ERR: BADARG cmd=call detail=character_not_sendable\n");
        return;
      }
      myCall[n] = c;
      n++;
    }
    if (n == 0) { closeOpenLines(); OUTP("ERR: BADARG cmd=call want=1_to_8_chars\n"); return; }
    myCall[n] = 0;
    settingsDirty = true;
    closeOpenLines();
    OUTP("SYS: call="); outStr(myCall); outNL();
    return;
  }

  if ((arg = matchCmd(line, PSTR("/to"))) != NULL) {
    uint8_t n = 0;
    while (arg[n] && arg[n] != ' ' && n < CALLSIGN_MAX) {
      char c = toUpperAscii(arg[n]);
      if (morseFindCode(c) == 0) {
        closeOpenLines();
        OUTP("ERR: BADARG cmd=to detail=character_not_sendable\n");
        return;
      }
      toCall[n] = c;
      n++;
    }
    if (n == 0) { closeOpenLines(); OUTP("ERR: BADARG cmd=to want=1_to_8_chars\n"); return; }
    toCall[n] = 0;
    closeOpenLines();
    OUTP("SYS: to="); outStr(toCall); outNL();
    return;
  }

  if ((arg = matchCmd(line, PSTR("/frame"))) != NULL) {
    if (argIsWord(arg, PSTR("on"))) frameEnabled = true;
    else if (argIsWord(arg, PSTR("off"))) frameEnabled = false;
    else { closeOpenLines(); OUTP("ERR: BADARG cmd=frame want=on|off\n"); return; }
    settingsDirty = true;
    closeOpenLines();
    OUTP("SYS: frame="); outStrP(frameEnabled ? PSTR("on") : PSTR("off"));
    OUTP(" call="); outStr(myCall);
    if (frameEnabled) OUTP(" warning=not_readable_by_pre_1_3_stations");
    outNL();
    return;
  }

  if ((arg = matchCmd(line, PSTR("/key"))) != NULL) {
    if (argIsWord(arg, PSTR("on"))) keyFitted = true;
    else if (argIsWord(arg, PSTR("off"))) keyFitted = false;
    else { closeOpenLines(); OUTP("ERR: BADARG cmd=key want=on|off\n"); return; }
    closeOpenLines();
    OUTP("SYS: key="); outStrP(keyFitted ? PSTR("on") : PSTR("off")); outNL();
    return;
  }

  if ((arg = matchCmd(line, PSTR("/mode"))) != NULL) {
    if (argIsWord(arg, PSTR("half"))) duplexMode = MODE_HALF;
    else if (argIsWord(arg, PSTR("full"))) duplexMode = MODE_FULL;
    else { closeOpenLines(); OUTP("ERR: BADARG cmd=mode want=half|full\n"); return; }
    settingsDirty = true;
    closeOpenLines();
    OUTP("SYS: mode="); outStrP(duplexMode == MODE_HALF ? PSTR("half") : PSTR("full"));
    if (duplexMode == MODE_FULL) OUTP(" warning=requires_optical_isolation");
    outNL();
    return;
  }

  if ((arg = matchCmd(line, PSTR("/echo"))) != NULL) {
    if (argIsWord(arg, PSTR("on"))) echoEnabled = true;
    else if (argIsWord(arg, PSTR("off"))) echoEnabled = false;
    else { closeOpenLines(); OUTP("ERR: BADARG cmd=echo want=on|off\n"); return; }
    settingsDirty = true;
    closeOpenLines();
    OUTP("SYS: echo="); outStrP(echoEnabled ? PSTR("on") : PSTR("off")); outNL();
    return;
  }

  if ((arg = matchCmd(line, PSTR("/raw"))) != NULL) {
    if (argIsWord(arg, PSTR("on"))) rawEnabled = true;
    else if (argIsWord(arg, PSTR("off"))) rawEnabled = false;
    else { closeOpenLines(); OUTP("ERR: BADARG cmd=raw want=on|off\n"); return; }
    closeOpenLines();
    OUTP("SYS: raw="); outStrP(rawEnabled ? PSTR("on") : PSTR("off")); outNL();
    return;
  }

  if ((arg = matchCmd(line, PSTR("/clear"))) != NULL) {
    txQueue.clear();
    txAbortRequested = (txState != TX_IDLE && txState != TX_DEFER);
    if (txState == TX_DEFER) txState = TX_IDLE;
    closeOpenLines();
    OUTP("SYS: cleared pending_abort=");
    outPutc(txAbortRequested ? '1' : '0');
    outNL();
    return;
  }

  if ((arg = matchCmd(line, PSTR("/loop"))) != NULL) {
    if (selfTestState != ST_IDLE) { closeOpenLines(); OUTP("ERR: BUSY detail=selftest_running\n"); return; }
    selfTestPass = 0;
    selfTestFail = 0;
    stTarget = 0;
    selfTestState = ST_ROUNDTRIP;
    closeOpenLines();
    OUTP("SYS: selftest_start\n");
    return;
  }

  closeOpenLines();
  OUTP("ERR: BADCMD line=");
  outStr(line);
  outNL();
}

static void serialService(uint32_t now) {
  /* Backpressure. A pasted block of commands can generate far more output than
   * the ring holds, and the ring drains only as fast as the UART will take it.
   * Leaving the bytes in the hardware receive buffer until the console has
   * caught up costs nothing and keeps the output stream intact. */
  if (outCount() > (OUT_BUF_SIZE / 2)) return;

  uint8_t budget = SERIAL_BYTES_PER_PASS;
  while (budget-- > 0 && Serial.available() > 0) {
    int ci = Serial.read();
    if (ci < 0) break;
    char c = (char)ci;

    if (c == '\r' || c == '\n') {
      /* Handles CR, LF and CRLF: the second terminator of a CRLF pair simply
       * closes an already empty line, which is a no-op. */
      if (lineOverflow) {
        lineOverflow = false;
        lineLen = 0;
        closeOpenLines();
        OUTP("ERR: LINEOVF max="); outUL(SERIAL_LINE_MAX - 1); outNL();
        continue;
      }
      if (lineLen == 0) continue;
      lineBuf[lineLen] = 0;
      lineLen = 0;
      handleLine(lineBuf, now);
      return;   /* one command or message per pass, see the loop budget note */
    }

    if (c < 32 || c > 126) continue;   /* ignore stray control bytes */

    if (lineLen >= (uint8_t)(SERIAL_LINE_MAX - 1)) { lineOverflow = true; continue; }
    lineBuf[lineLen++] = c;
  }
}

/* ============================================================================
 * BUILT IN SELF TEST
 *
 * Runs one group per pass of loop() so the sketch never appears hung, and
 * needs no physical light path. Nothing here touches live engine state except
 * through pure functions and a private ring buffer.
 * ========================================================================= */

/* One assertion is reported per pass of loop(), so the report can never
 * outrun the output ring. Each group is a pure function and is simply re-run
 * from the top on every pass, with stAssert reporting only the assertion whose
 * ordinal matches stTarget. Re-running the groups is cheap: the most expensive
 * of them walks the 44 entry table a few dozen times, which stays inside one
 * sample interval on an ATmega328P. */
static void stAssert(bool condition, const char *namePgm) {
  if (stOrdinal++ != stTarget) return;
  stEmitted = true;
  if (condition) selfTestPass++; else selfTestFail++;
  closeOpenLines();
  outStrP(condition ? PSTR("SYS: selftest PASS ") : PSTR("ERR: selftest FAIL "));
  outStrP(namePgm);
  outNL();
}

static void stRoundTrip() {
  bool allOk = true;
  bool lengthsOk = true;
  for (uint8_t i = 0; i < MORSE_COUNT; i++) {
    char ch = (char)pgm_read_byte(&MORSE_CHARS[i]);
    uint16_t code = morseFindCode(ch);
    if (code == 0) { allOk = false; continue; }
    uint8_t bits[MAX_ELEMENTS];
    uint8_t n = 0;
    codeToElements(code, bits, &n);
    if (n < 1 || n > MAX_ELEMENTS) lengthsOk = false;
    if (elementsToCode(bits, n) != code) allOk = false;
    if (morseFindChar(code) != ch) allOk = false;
  }
  stAssert(allOk, PSTR("encode_decode_roundtrip_all_chars"));
  stAssert(lengthsOk, PSTR("element_counts_within_1_to_MAX_ELEMENTS"));
  stAssert(morseFindCode('~') == 0, PSTR("unknown_char_rejected"));
  stAssert(morseFindChar(0xFFFF) == 0, PSTR("unknown_code_rejected"));
  stAssert(morseFindCode('e') == 0, PSTR("table_is_uppercase_only"));
  stAssert(morseFindCode(toUpperAscii('e')) == 2, PSTR("case_fold_before_lookup"));
}

static void stTiming() {
  stAssert(unitMsForWpm(WPM_MIN) == 240, PSTR("unit_at_wpm_min_is_240ms"));
  stAssert(unitMsForWpm(12) == 100, PSTR("unit_at_12wpm_is_100ms"));
  stAssert(unitMsForWpm(20) == 60, PSTR("unit_at_20wpm_is_60ms"));
  stAssert(unitMsForWpm(WPM_MAX) == 40, PSTR("unit_at_wpm_max_is_40ms"));
  stAssert(wpmForUnitMs(100) == 12, PSTR("wpm_from_unit_roundtrip"));
  /* The stated oversampling floor must actually hold at the fastest speed. */
  uint16_t samplesPerDot = (uint16_t)(((uint32_t)unitMsForWpm(WPM_MAX) * 1000UL) / SAMPLE_INTERVAL_US);
  stAssert(samplesPerDot >= MIN_OVERSAMPLE_PER_DOT, PSTR("oversampling_floor_met_at_wpm_max"));
  stAssert(EDGE_DEBOUNCE_MS < unitMsForWpm(WPM_MAX) / 2, PSTR("debounce_shorter_than_half_dot_at_wpm_max"));

  /* Farnsworth. With both speeds equal the gap unit must collapse onto the
   * element unit exactly, otherwise standard timing is silently wrong. */
  stAssert(farnsworthGapMs(12, 12) == unitMsForWpm(12), PSTR("farnsworth_off_equals_standard_timing"));
  stAssert(farnsworthGapMs(WPM_MIN, WPM_MIN) == unitMsForWpm(WPM_MIN), PSTR("farnsworth_identity_at_wpm_min"));
  stAssert(farnsworthGapMs(WPM_MAX, WPM_MAX) == unitMsForWpm(WPM_MAX), PSTR("farnsworth_identity_at_wpm_max"));
  stAssert(farnsworthGapMs(18, 5) == 522, PSTR("farnsworth_18_over_5"));
  stAssert(farnsworthGapMs(20, 10) == 217, PSTR("farnsworth_20_over_10"));
  stAssert(farnsworthGapMs(30, 5) == 566, PSTR("farnsworth_30_over_5"));
  stAssert(farnsworthGapMs(20, 8) > farnsworthGapMs(20, 12), PSTR("slower_overall_speed_widens_gaps"));
  stAssert(farnsworthGapMs(20, 30) == unitMsForWpm(20), PSTR("farnsworth_clamps_above_char_speed"));
  stAssert(farnsworthGapMs(WPM_MAX, WPM_MIN) < 1000, PSTR("farnsworth_gap_fits_uint16"));
}

static void stRollover() {
  uint32_t nearWrap = 0xFFFFFF00UL;
  uint32_t afterWrap = 0x00000100UL;
  stAssert((uint32_t)(afterWrap - nearWrap) == 0x200UL, PSTR("unsigned_subtraction_wraps"));
  stAssert(elapsed(afterWrap, nearWrap, 0x1FFUL), PSTR("elapsed_true_across_wrap"));
  stAssert(!elapsed(afterWrap, nearWrap, 0x201UL), PSTR("elapsed_false_across_wrap"));
  stAssert(!elapsed(nearWrap, nearWrap, 1UL), PSTR("elapsed_false_at_zero_span"));
  stAssert(elapsed(nearWrap, nearWrap, 0UL), PSTR("elapsed_true_for_zero_span"));
  stAssert(elapsed(0xFFFFFFFFUL, 0xFFFFFFF0UL, 15UL), PSTR("elapsed_true_at_wrap_edge"));
}

static void stRing() {
  static uint8_t testStore[8];
  CharRing r;
  r.init(testStore, 8);

  bool fillOk = true;
  for (uint8_t i = 0; i < 7; i++) if (!r.push((uint8_t)('A' + i))) fillOk = false;
  stAssert(fillOk, PSTR("ring_fills_to_usable_capacity"));
  stAssert(r.count() == 7, PSTR("ring_count_after_fill"));
  stAssert(r.full(), PSTR("ring_reports_full"));

  uint16_t droppedBefore = r.droppedCount;
  stAssert(!r.push('Z'), PSTR("ring_rejects_overrun"));
  stAssert(r.droppedCount == droppedBefore + 1, PSTR("ring_counts_overrun"));

  uint8_t peeked = 0;
  stAssert(r.peek(&peeked) && peeked == 'A', PSTR("ring_peek_does_not_consume"));

  bool drainOk = true;
  for (uint8_t i = 0; i < 7; i++) {
    uint8_t v;
    if (!r.pop(&v) || v != (uint8_t)('A' + i)) drainOk = false;
  }
  stAssert(drainOk, PSTR("ring_drains_fifo_order"));
  uint8_t sink;
  stAssert(!r.pop(&sink), PSTR("ring_pop_empty_fails"));
  stAssert(r.empty(), PSTR("ring_empty_after_drain"));

  /* Wrap the indices well past capacity to prove the masking. */
  bool wrapOk = true;
  for (uint8_t i = 0; i < 40; i++) {
    if (!r.push((uint8_t)i)) { wrapOk = false; break; }
    uint8_t v;
    if (!r.pop(&v) || v != i) { wrapOk = false; break; }
  }
  stAssert(wrapOk, PSTR("ring_wraps_correctly"));

  r.push('X');
  r.clear();
  stAssert(r.empty() && r.count() == 0, PSTR("ring_clear_empties"));
}

static void stClassify() {
  const uint16_t u = 100;
  stAssert(!markIsDash(1, u), PSTR("mark_1ms_is_dot"));
  stAssert(!markIsDash(199, u), PSTR("mark_below_2u_is_dot"));
  stAssert(markIsDash(200, u), PSTR("mark_at_2u_is_dash"));
  stAssert(markIsDash(300, u), PSTR("mark_at_3u_is_dash"));

  stAssert(classifySpace(0, u) == SPACE_ELEMENT, PSTR("space_0_is_element_gap"));
  stAssert(classifySpace(199, u) == SPACE_ELEMENT, PSTR("space_below_2u_is_element_gap"));
  stAssert(classifySpace(200, u) == SPACE_CHAR, PSTR("space_at_2u_is_char_gap"));
  stAssert(classifySpace(499, u) == SPACE_CHAR, PSTR("space_below_5u_is_char_gap"));
  stAssert(classifySpace(500, u) == SPACE_WORD, PSTR("space_at_5u_is_word_gap"));
  stAssert(classifySpace(700, u) == SPACE_WORD, PSTR("space_at_7u_is_word_gap"));

  /* Boundaries must hold at both ends of the supported speed range, where the
   * unit is 40 ms and 240 ms rather than a convenient 100. */
  const uint16_t fast = unitMsForWpm(WPM_MAX);
  stAssert(!markIsDash((uint32_t)fast, fast), PSTR("dot_at_wpm_max_is_dot"));
  stAssert(markIsDash((uint32_t)fast * 3UL, fast), PSTR("dash_at_wpm_max_is_dash"));
  const uint16_t slow = unitMsForWpm(WPM_MIN);
  stAssert(!markIsDash((uint32_t)slow, slow), PSTR("dot_at_wpm_min_is_dot"));
  stAssert(classifySpace((uint32_t)slow * 7UL, slow) == SPACE_WORD, PSTR("word_gap_at_wpm_min"));
}

/* Settings persistence. The EEPROM itself is not written by the self test,
 * only the framing and the check byte that protect it. */
static void stPersist() {
  const uint8_t vector[9] = { '1','2','3','4','5','6','7','8','9' };
  stAssert(crc8(vector, 9) == 0xF4, PSTR("crc8_matches_reference_vector"));
  const uint8_t one[1] = { 'A' };
  stAssert(crc8(one, 1) == 0xC0, PSTR("crc8_single_byte"));
  stAssert(crc8(vector, 0) == 0x00, PSTR("crc8_empty_is_zero"));

  uint8_t a[SETTINGS_BYTES];
  settingsPack(a);
  stAssert(a[SETTINGS_BYTES - 1] == crc8(a, (uint8_t)(SETTINGS_BYTES - 1)),
           PSTR("settings_crc_is_stable"));

  uint8_t b[SETTINGS_BYTES];
  for (uint8_t i = 0; i < SETTINGS_BYTES; i++) b[i] = a[i];
  b[3] = (uint8_t)(b[3] ^ 0x01);
  stAssert(b[SETTINGS_BYTES - 1] != crc8(b, (uint8_t)(SETTINGS_BYTES - 1)),
           PSTR("settings_crc_catches_a_flipped_field"));

  uint16_t magic = (uint16_t)a[0] | ((uint16_t)a[1] << 8);
  stAssert(magic == SETTINGS_MAGIC, PSTR("magic_packs_little_endian"));
  stAssert(a[3] == txWpm && a[4] == (uint8_t)duplexMode, PSTR("settings_pack_field_offsets"));
  stAssert(SETTINGS_MAGIC != 0 && SETTINGS_MAGIC != 0xFFFF,
           PSTR("magic_distinct_from_erased_eeprom"));
}

/* Framing, check bytes and backoff. No radio, no light path, no EEPROM. */
static void stFrame() {
  stAssert(crc16("123456789", 9) == 0x29B1, PSTR("crc16_matches_reference_vector"));
  stAssert(crc16("", 0) == 0xFFFF, PSTR("crc16_empty_is_init_value"));
  stAssert(crc16("HELLO", 5) == 0x49D6, PSTR("crc16_of_hello"));
  stAssert(hexDigit(0) == '0' && hexDigit(10) == 'A' && hexDigit(15) == 'F',
           PSTR("hex_digits_render"));
  uint8_t v = 0;
  stAssert(hexValue('F', &v) && v == 15, PSTR("hex_value_parses"));
  stAssert(!hexValue('G', &v), PSTR("hex_value_rejects_non_hex"));

  char from[CALLSIGN_MAX + 1];
  char to[CALLSIGN_MAX + 1];
  char text[FRAME_TEXT_MAX + 1];
  uint8_t len = 0;

  stAssert(frameParse("= N1HNP CQ 5 = HELLO = 49D6 +", from, to, text, &len) == FRAME_OK,
           PSTR("good_frame_parses"));
  stAssert(len == 5 && text[0] == 'H' && text[4] == 'O', PSTR("frame_text_extracted"));
  stAssert(from[0] == 'N' && to[0] == 'C', PSTR("frame_callsigns_extracted"));

  stAssert(frameParse("= N1HNP CQ 11 = HELLO WORLD = 5546 +", from, to, text, &len) == FRAME_OK,
           PSTR("frame_text_may_contain_spaces"));

  stAssert(frameParse("= N1HNP CQ 5 = HELLP = 49D6 +", from, to, text, &len) == FRAME_BAD_CRC,
           PSTR("corrupt_text_fails_crc"));
  stAssert(frameParse("= N1HNP CQ 6 = HELLO = 49D6 +", from, to, text, &len) == FRAME_BAD_LENGTH,
           PSTR("wrong_length_is_caught_before_crc"));
  stAssert(frameParse("HELLO WORLD", from, to, text, &len) == FRAME_NOT_A_FRAME,
           PSTR("plain_text_is_not_a_frame"));
  stAssert(frameParse("= N1HNP CQ 5 = HELLO = 49D6", from, to, text, &len) == FRAME_MALFORMED,
           PSTR("missing_terminator_is_malformed"));
  stAssert(frameParse("= N1HNP", from, to, text, &len) == FRAME_MALFORMED,
           PSTR("truncated_header_is_malformed"));

  stAssert(backoffWindowMs(0, 100) == 700, PSTR("backoff_starts_at_one_word_gap"));
  stAssert(backoffWindowMs(1, 100) == 1400, PSTR("backoff_window_doubles"));
  stAssert(backoffWindowMs(BACKOFF_MAX_SHIFT, 100) == backoffWindowMs(200, 100),
           PSTR("backoff_window_is_capped"));
  stAssert(backoffWindowMs(3, 100) > backoffWindowMs(2, 100), PSTR("backoff_window_is_monotonic"));
}

static void selfTestService(uint32_t now) {
  if (selfTestState == ST_IDLE) return;

  if (selfTestState == ST_SUMMARY) {
    closeOpenLines();
    OUTP("SYS: selftest_end pass="); outUL(selfTestPass);
    OUTP(" fail="); outUL(selfTestFail);
    OUTP(" result="); outStrP(selfTestFail == 0 ? PSTR("GREEN") : PSTR("RED"));
    outNL();
    selfTestState = ST_IDLE;
    return;
  }

  stOrdinal = 0;
  stEmitted = false;
  switch (selfTestState) {
    case ST_ROUNDTRIP: stRoundTrip(); break;
    case ST_TIMING:    stTiming();    break;
    case ST_ROLLOVER:  stRollover();  break;
    case ST_RING:      stRing();      break;
    case ST_CLASSIFY:  stClassify();  break;
    case ST_PERSIST:   stPersist();   break;
    case ST_FRAME:     stFrame();     break;
    default: break;
  }

  if (stEmitted) {
    stTarget++;
    return;
  }
  /* The group ran out of assertions, so move to the next one. */
  stTarget = 0;
  switch (selfTestState) {
    case ST_ROUNDTRIP: selfTestState = ST_TIMING;   break;
    case ST_TIMING:    selfTestState = ST_ROLLOVER; break;
    case ST_ROLLOVER:  selfTestState = ST_RING;     break;
    case ST_RING:      selfTestState = ST_CLASSIFY; break;
    case ST_CLASSIFY:  selfTestState = ST_PERSIST;  break;
    case ST_PERSIST:   selfTestState = ST_FRAME;    break;
    default:           selfTestState = ST_SUMMARY;  break;
  }
  (void)now;
}

/* ============================================================================
 * SETUP AND LOOP
 * ========================================================================= */

void setup() {
  pinMode(PIN_TX_LED, OUTPUT);
  digitalWrite(PIN_TX_LED, LOW);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
  if (CAL_BUTTON_FITTED) pinMode(PIN_CAL_BUTTON, INPUT_PULLUP);
  pinMode(PIN_SIDETONE, OUTPUT);
  if (KEY_FITTED) pinMode(PIN_KEY, INPUT_PULLUP);

  Serial.begin(SERIAL_BAUD);

  txQueue.init(txQueueStore, TX_QUEUE_SIZE);
  farnsworthExplicit = false;
  txFarnsworthWpm = txWpm;
  recomputeTiming();
  rxUnitQ4 = (int32_t)txUnitMs << 4;   /* seed the estimator with our own speed */
  lastSampleUs = micros();
  rxLastEdgeMs = millis();
  txEndedMs = millis();

  calState = CAL_BOOT;
  calPhaseStart = millis();
  calSum = 0;
  calSamples = 0;

  printVersion();
  if (settingsLoad(true)) contrastFromStore = (contrastCounts >= MIN_CONTRAST_COUNTS);
  rxUpdateThresholds();
  recomputeTiming();
  rxUnitQ4 = (int32_t)txUnitMs << 4;

  /* An unwired input with no pullup holding it reads closed. Rather than key
   * continuously against a floating pin, disable the key and say so. */
  if (KEY_FITTED && digitalRead(PIN_KEY) == LOW) {
    keyFitted = false;
    OUTP("SYS: key=off reason=pin_reads_closed_at_boot\n");
  }
  OUTP("SYS: boot settling_ms="); outUL(BOOT_SETTLE_MS);
  OUTP(" receiver=warming\n");
  OUTP("SYS: boot type /help for commands\n");
}

void loop() {
  uint32_t now = millis();

  /* Order matters. The sampler runs first so its cadence is never pushed out
   * by console work, and the output ring is drained last so anything produced
   * this pass starts moving immediately. */
  rxService(now);
  txService(now);
  serialService(now);
  calButtonService(now);
  keyService(now);
  selfTestService(now);
  helpService();
  statusService(now);
  outService();
}
