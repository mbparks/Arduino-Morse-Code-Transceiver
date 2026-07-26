# Optical Morse Terminal

Version 1.3.0 (sketch), 1.4.0 (host console)
License: GPL-3.0

A full duplex capable optical Morse code terminal on a single Arduino. One LED
is the transmitter, one photoresistor is the receiver, and the USB serial port
is the operator. Type a line and it goes out as blinked Morse. Incoming light
pulses are decoded and printed as text. Both directions run concurrently and
non-blocking, with no `delay()` anywhere in the operating path.

Two boards running this identical sketch, pointed at each other, form a working
optical link.

---

## What it is

A single self contained `.ino` file, no external libraries, standard Arduino
core only. No `String`, no `malloc`, no `new`, no dynamic allocation of any
kind. Every buffer is fixed size with a named size constant and explicit
overflow handling. Everything is a `millis()` driven state machine serviced
from `loop()`, and every elapsed time comparison uses unsigned subtraction so
the 49.7 day `millis()` rollover is a non event.

The interesting problem here is not the Morse. It is that the local LED almost
certainly illuminates the local photoresistor, so a naive build spends its life
decoding its own transmission. See Self Interference below.

---

## Hardware

Target board is an Arduino Uno or Nano (ATmega328P, 5V, 16 MHz). Every pin and
ADC assumption lives in named constants at the top of the file, so the sketch
stays portable to the Uno R4 and ESP32. On an ESP32 you will need to change
`ADC_MAX` from 1023 to 4095.

| Pin | Component | Orientation and notes |
| --- | --- | --- |
| D3 | TX LED anode (long leg) | Cathode to GND through 220R to 1k. Red and IR adjacent colors suit a CdS cell best. Blue and white are the worst match for its spectral response. |
| A0 | LDR and fixed resistor junction | See the divider below. |
| D13 | Status LED (built in) | Lights while a received mark is being detected. Optional. |
| D2 | Calibration button | Other side to GND, `INPUT_PULLUP`, active low. Optional, leave unconnected if unused. |
| D4 | Straight key | Other side to GND, active low. If the pin already reads closed at boot the key is disabled and reported, which is what an unwired input looks like. |
| D9 | Sidetone piezo | Other leg to GND. Driven by `tone()`, which uses timer 2 and does not collide with the LED on D3 because that pin is switched, never PWM'd. |
| 5V | LDR leg 1 | |
| GND | Fixed resistor leg 2 | |

```
5V ---- LDR ----+---- Rfixed ---- GND
                |
                A0
```

With the LDR on the high side, more light lowers its resistance and raises the
voltage at A0, so more light means a higher ADC count. That is what
`SENSOR_ACTIVE_HIGH = true` means. Build the divider the other way round
(fixed resistor high, LDR to GND) and set it to `false` instead. The sketch
inverts the reading for you.

### Choosing the divider resistor

The divider is most sensitive when `Rfixed` sits near the geometric mean of the
LDR resistance in your dark condition and its resistance under the LED at your
intended spacing. Measure both with a meter, multiply, take the square root,
pick the nearest standard value. A common GL5528 in a normally lit room looking
at an LED a few centimeters away lands somewhere around 10k, which is why 10k
is the usual starting point.

If `/cal` reports low contrast, the fix is almost always a different `Rfixed`,
closer spacing, or a shade tube around the sensor.

---

## What is in the repository

| Path | What it is |
| --- | --- |
| `optical_morse_terminal/optical_morse_terminal.ino` | The sketch, current release |
| `releases/` | Every tagged release kept as its own file |
| `heliograph.html` | Host console, single file, no build step |
| `tests/` | Host side simulation harness, see Testing |

## Install

The Arduino IDE requires the sketch folder name to match the file name:

```
optical_morse_terminal/
  optical_morse_terminal.ino
```

Open it, select your board, upload. No libraries to install, no build flags, no
board specific patches.

---

## Bring up

Open the serial monitor at 115200 baud.

**First boot.** You should see the version line, then
`SYS: boot settling_ms=1500 mode=half receiver=warming`, then about a second
and a half later `SYS: boot_cal dark=<n> contrast=120 rise=<n> fall=<n>
source=assumed`. A dark value near 0 or near 1023 means the divider is wrong or
`SENSOR_ACTIVE_HIGH` is backwards.

**Verify transmit.** Type `HELLO` and watch the LED. A phone camera helps if
you used an IR adjacent emitter. Turn on `/echo on` and the sketch prints
`TX+ HELLO`, which is the encoder's own output run back through the decoder. If
that line matches what you typed, the encode path and the lookup table agree
with each other.

**Verify receive.** Run `/cal`. It reports dark, lit, contrast, and the
resulting thresholds, then `receiver=armed`. Turn on `/raw on` and wave a hand
over the sensor. The ADC value should move and `DBG: EDGE` lines should appear.
Turn it off again when satisfied.

**Two boards.** Point them at each other, run `/cal` on both, type on one.

### Failure modes

| Symptom | Cause |
| --- | --- |
| `ERR: LOWCONTRAST` on `/cal` | The sensor cannot see the LED well enough. Close the gap, shade the sensor from room light, or retune `Rfixed`. |
| Nothing decodes at the working distance | Insufficient contrast at that distance. Run `/cal` at the distance you intend to use, not on the bench. |
| Text arrives full of `?` | The unit estimator is fighting you. Check `rxwpm` in `/status` against the sender's actual speed. |
| Your own text appears under `RX:` | You are in `/mode full` without real optical isolation. Go back to `/mode half`. |
| `SYS: tx_deferred` and nothing sends | Carrier sense thinks the channel is busy. Something is flickering at the sensor, or the far station is still sending. |

---

## Using it

Any line that does not start with `/` is queued for transmission. Lines that
start with `/` are commands.

| Command | Effect |
| --- | --- |
| `/help` | List all commands |
| `/wpm <5..30>` | Set transmit speed. Out of range values are rejected |
| `/cal` | Guided ambient calibration, dark then lit |
| `/status` | WPM, estimated receive WPM, thresholds, baseline, queue depth, uptime, free RAM, mode |
| `/mode <half\|full>` | Duplex mode, see Self Interference |
| `/echo <on\|off>` | Local echo of transmitted characters as decoded text |
| `/raw <on\|off>` | Stream raw ADC values and detected edge timings |
| `/loop` | Run the built in self test |
| `/clear` | Flush the transmit queue and abort the current message at the next element boundary |
| `/version` | Print name and semantic version |
| `/save` | Write settings and calibration to EEPROM |
| `/load` | Reload the stored block |
| `/defaults` | Settings back to compiled defaults, not saved |
| `/fw <wpm\|off>` | Farnsworth overall speed, never above the character speed |
| `/tone <on\|off>` | Sidetone on the piezo |
| `/pitch <300..1200>` | Sidetone frequency in Hz |
| `/key <on\|off>` | Enable or disable the straight key input |
| `/call <sign>` | Your callsign, up to 8 sendable characters |
| `/to <sign>` | Destination callsign, default CQ |
| `/frame <on\|off>` | Send framed, length checked, CRC checked messages |

### Character set

A to Z, 0 to 9, and period, comma, question mark, slash, plus, equals, plus the
AR, SK and error prosigns. Lowercase input is folded to uppercase.
Unencodable characters are dropped and reported as `ERR: TXDROP`, never
silently swallowed.

ASCII has no characters for the prosigns, so:

| Typed | Prosign | Elements |
| --- | --- | --- |
| `+` | AR, end of message | `.-.-.` |
| `=` | BT, break | `-...-` |
| `*` | SK, end of contact | `...-.-` |
| `#` | Error | `........` |

`+` and `=` are the standard assignments. `*` and `#` are local convention to
this project.

---

## Serial output grammar

Every line carries a fixed prefix so a host script can parse the stream without
guessing. Lines are terminated with a single `\n`.

```
TX: <key>=<value> ...     transmit events, one line per event
TX+ <text>                local echo of transmitted characters, decoded back
                          to text. Opened at message start, closed at message
                          end. Enabled by /echo on
RX: <text>                decoded incoming text, printed incrementally as each
                          character resolves
RX+ <text>                continuation of an RX line that had to be closed
                          early because another line was emitted
SYS: <key>=<value> ...    status and informational events
ERR: <CODE> <key>=<value> ...   errors, where CODE is a stable uppercase token
DBG: RAW t= adc= base= rise= fall= mark= gate=
DBG: EDGE t= kind=<MARK|SPACE> dur= unit=
```

Stable error codes: `LINEOVF`, `TXFULL`, `TXDROP`, `BADCMD`, `BADARG`, `BUSY`,
`LOWCONTRAST`, `NOTARMED`, `RXBREAK`, `OUTDROP`, `SAMPLEOVR`.

---

## Framing

Framing is off by default. Turned on with `/frame on`, an outgoing message is
wrapped:

```
= <FROM> <TO> <LEN> = <TEXT> = <CRC4> +
```

A frame is ordinary Morse text, so a station that knows nothing about framing
still copies something readable rather than gibberish. BT (`=`) opens the frame
and separates the header, the text and the check value. AR (`+`) closes it.
Neither can appear inside the text, because framed text rejects them. `LEN` is
the decimal character count of the text, which catches a truncation that happens
to leave a plausible check value. `CRC4` is four hex digits of CRC-16/CCITT-FALSE
over the text.

Receiving is always tolerant. A line that does not parse as a frame is printed
as plain text and noted with `SYS: unframed_traffic`, so a 1.3 station and a 1.0
station can still talk, just without any of the guarantees.

When a frame fails its length or CRC check, that is treated as evidence that
somebody else was transmitting at the same time, and a queued message backs off.

## Backoff

A deferred transmission now waits before resuming. The window starts at one word
gap and doubles per attempt up to sixteen word gaps, and the actual wait is drawn
uniformly from inside that window using a seeded xorshift, so two stations that
started deferring together do not resume together. The counter clears on any
successfully sent or received message and is visible as `backoff=` in `/status`.

## Farnsworth

`/fw <wpm>` sends the characters themselves at the `/wpm` character speed while
stretching the gaps between them to hit a slower overall speed. The standard
derivation: a PARIS word is 50 units, of which 31 are inside characters and 19
are the gaps between them, so

```
gap_unit_ms = (60000 * C - 37200 * F) / (19 * C * F)
```

With the overall speed equal to the character speed this reduces exactly to
`1200 / C`, so standard timing is the special case and needs no separate code
path. Farnsworth is opt in and never latches: raising `/wpm` carries the overall
speed with it unless `/fw` was asked for explicitly.

## Straight key

A key on D4 drives the LED and the sidetone directly and takes priority over the
transmit queue. Its own marks and spaces are run through the same classification
functions the receiver uses, so what you send comes back as text on a `KEY:`
line. The board is a practice oscillator that answers back.

---

## HELIOGRAPH, the host console

`heliograph.html` is a single file with no build step and no server. Open it
from disk in Chrome or Edge and it talks to the board over Web Serial at 115200.

- Parses the serial output grammar and colours lines by prefix, with per prefix
  filters
- Keeps station state in view, populated from `/status` and from every other
  line that carries key and value pairs
- Recomputes CRC-16 over every received frame independently, so each frame is
  checked once by the board and once by the host, by two implementations written
  from the same specification
- Command entry with history, a quick command list, log export, and log import
  for reviewing a session with no board attached
- Night, Day and High Contrast themes
- Built in self test covering the parser, the CRC and the frame grammar, which
  runs with no board and no serial port

Nothing arriving over the wire is ever parsed as markup. Every line reaches the
DOM through `textContent`.

---

## How it works

### Timing

PARIS convention throughout. One word is 50 units, so `unit_ms = 1200 / wpm`.
A dot is 1 unit on, a dash is 3. The gap between elements inside a character is
1 unit, between characters 3 units, between words 7 units.

### Transmit

Outgoing text goes into a 128 byte character ring buffer with a sentinel
marking each message boundary. A state machine walks the queue through IDLE,
MARK, INTRA\_CHARACTER\_GAP, INTER\_CHARACTER\_GAP and INTER\_WORD\_GAP,
deciding which gap follows a character by peeking at what comes next. That is
what keeps a word gap at exactly 7 units instead of 3 plus 7.

### Receive

The analog input is sampled every 1 ms. The shortest element is a dot, which at
the 30 WPM ceiling is 40 ms, so 1 ms sampling gives 40 samples per dot against
a stated floor of 20. Marks are detected against an adaptive threshold that
floats on a slow moving ambient baseline, with separate rise and fall trip
points so a drifting room cannot produce false keying. Edges are debounced with
a 6 ms minimum duration filter, which is what rejects mains flicker (a 5.0 ms
half cycle at 100 Hz, 4.2 ms at 120 Hz).

Element classification uses a running estimate of the sender's unit length, not
the locally configured WPM, so the receiver locks onto a station running at a
different speed within a few characters. The estimate is reported as `rxwpm` in
`/status`. The lookup table lives in flash as `PROGMEM` and is shared by both
the encoder and the decoder, so the two directions cannot drift apart.

Undecodable symbols print as `?` and are counted per message rather than
dropped.

### Self interference

The local LED will light the local LDR unless you have physically isolated
them. Two mitigations are provided.

**`/mode half` (default, safe).** The receiver is gated off for the whole
duration of an outgoing message, plus a 60 ms guard afterwards to let the LDR
decay back to ambient. Nothing that arrives while you are sending is heard.
This is a true half duplex link, and it cannot decode its own transmission
because it is not listening at all.

**`/mode full` (requires optical isolation).** The receiver is gated only for
12 ms on each side of every LED transition, which covers sensor rise and fall
lag. This assumes a shaded sensor or separate optical paths. If the local LED
is visible to the local LDR in this mode, the sketch will decode its own
transmission and the console will fill with echoes of what you just sent. The
tradeoff: this mode only listens through the gaps, and since the guard is 12 ms
per transition it starts to eat meaningfully into the element gaps above
roughly 15 WPM.

When gating lifts, any partially assembled symbol is discarded and reported as
`ERR: RXBREAK` rather than being silently folded into whatever arrives next.
An interrupted receive line is reopened with the `RX+` prefix.

### Carrier sense

If a message is queued while the channel is busy, transmit is deferred until
the channel has been idle for one word gap. This is advisory collision
avoidance only. It cannot detect a station that happens to be silent during an
inter character gap, it cannot hear a third station that only one of the two
can see, and two stations that begin deferring at the same moment will still
collide when the channel clears. There is no backoff.

---

## Testing

`/loop` runs the built in self test on demand. It needs no physical light path
and reports one assertion per pass of `loop()`, so the sketch never appears
hung. 80 assertions across seven groups:

- encode then decode round trip over the full supported character set
- timing math at 5, 12, 20 and 30 WPM, including the boundary values, plus a
  check that the stated oversampling floor actually holds at the speed ceiling
- `millis()` rollover arithmetic either side of the wrap point
- ring buffer fill, drain, FIFO order, wrap, overrun and clear
- element classification against synthetic durations at the decision boundaries
- the settings block: CRC-8 reference vectors, field offsets, and that a
  flipped field is actually caught
- framing: CRC-16 reference vectors, frame build and parse round trip, corrupt
  text, wrong length, malformed headers, and the backoff window math

Expected result is `SYS: selftest_end pass=80 fail=0 result=GREEN`.

Every release has additionally been verified in the host side simulation in
`tests/`, which compiles the sketch that compiles
the sketch twice under `g++`, once per namespace, so two virtual boards can be
pointed at each other through a modeled optical path with sensor lag. That
harness covers the link itself, speed lock onto a faster sender, half duplex
gating under heavy self illumination, carrier sense deferral, clean abort, low
contrast refusal, ambient drift of 300 counts with 120 Hz flicker superimposed,
persistence across a simulated cold boot, Farnsworth timing, the sidetone, the
straight key, framing end to end, an independently corrupted frame, and the
whole suite again with both clocks parked before their 32 bit wrap. It is not
required to build or run the sketch.

Build and run it with:

```
cd tests && g++ -std=c++11 -I. -Wall -Wextra -o sim main.cpp && ./sim
```

Expected result is `HARNESS SUMMARY: GREEN (0 failures)`. Pass `./sim wrap` to
run the whole suite with both clocks parked just short of their 32 bit rollover.

---

## Known Limitations

1. **Ambient light sensitivity.** The adaptive baseline handles slow drift such
   as the sun moving across a room. It does not handle a hand passing over the
   sensor, a camera flash, or someone flipping the room lights, all of which
   will inject spurious elements. Mains flicker is rejected by the debounce
   filter, not by any filtering of the signal itself.
2. **LDR response time caps usable speed.** A CdS cell takes on the order of 10
   to 50 ms to rise and considerably longer to decay. `WPM_MAX` is 30 for the
   arithmetic, but a typical CdS cell will not cleanly resolve a 40 ms dot.
   Expect 8 to 15 WPM in practice.
3. **Detection, not correction.** A framed message that fails its length or CRC
   check is rejected and reported, but nothing asks for it again. There is no
   acknowledgement, no sequence number, and no retransmission, so a lost message
   stays lost and the sender never finds out.
4. **Addressing is a label, not a filter.** Every station in the optical path
   still hears and prints every transmission. The `TO` field says who a frame is
   for, and nothing enforces it.
5. **Backoff helps two stations, not a crowd.** The window doubles and the wait
   is drawn at random inside it, which separates two stations reliably. It does
   nothing about a station that only one of the two can see, and the carrier
   sense underneath it still cannot tell a genuinely idle channel from a sender
   pausing between characters.
6. **Self interference gating assumes the transmitter is the only local light
   source that matters.** In `/mode full` the assumption of optical isolation is
   entirely on the operator to satisfy physically.
7. **The receive unit estimator tracks one sender.** If two stations at
   different speeds transmit in the same session, the estimate chases whichever
   spoke last, and the first few characters after a handover may decode wrong.
8. **Persistence is explicit, not automatic.** Change a setting, forget to
   `/save`, and the change is gone at the next reset. This is deliberate: an
   EEPROM cell tolerates roughly 100000 writes, and saving on every `/wpm` would
   spend that budget on nothing.
9. **Farnsworth sending is a one way courtesy.** A human copying by ear hears
   stretched gaps as thinking time, but a machine decoder using adaptive spacing
   thresholds reads them as word gaps, so a far station running this sketch will
   copy the right letters with the wrong word breaks. The receiver does not
   estimate character and word spacing separately.
10. **The straight key classifies against the configured character speed**, not
    an adaptive estimate. Key faster or slower than `/wpm` and the `KEY:` line
    will tell you so, which is the point, but it is not a fist reader.
11. **Console backpressure is deliberate.** Paste a long block of commands and
    the sketch stops reading input until the console has drained. This protects
    the output stream but can overrun the hardware receive buffer if the host
    keeps pushing. Send commands a line at a time.
12. **HELIOGRAPH needs Web Serial**, which means Chrome or Edge on the desktop.
    Log import and the self test work in any browser. Web Serial from a `file://`
    page depends on the browser treating local files as a secure context, and
    that behaviour has changed before.
13. **Not yet bench verified on hardware.** Releases 1.0.0 through 1.3.0 pass
    their own self tests and the host side simulation, and none of them has met
    a real photocell. The flash and RAM figures in the sketch header are
    unfilled placeholders until someone compiles it for a real board.

---

## Roadmap

Everything on the original roadmap through 1.4.0 is now built. What remains,
deliberately not built:

- Acknowledgement and retransmission, which is what would turn frame checking
  from detection into recovery
- Sequence numbers and duplicate suppression
- Photodiode plus transimpedance front end, and the higher WPM ceiling and PWM
  carrier modulation that becomes possible with it. This is the v2.0 line
  because it invalidates assumptions rather than extending them: a fast sensor
  makes `WPM_MAX` too conservative, makes the 12 ms transition guard far too
  generous, and makes full duplex genuinely practical rather than a documented
  caveat
- Separate character and word spacing estimation in the receiver, which would
  let a far station copy Farnsworth sending correctly
- Hidden node handling, which carrier sense fundamentally cannot do alone

## Changelog

| Version | Change |
| --- | --- |
| 1.4.0 | HELIOGRAPH host console. Parses the output grammar, tracks station state, and verifies every frame's CRC independently. |
| 1.3.0 | Framing with callsign, length and CRC-16. Exponential backoff with a random draw inside a doubling window. Framing is off by default and receiving stays tolerant of unframed traffic. |
| 1.2.0 | Sidetone, straight key with its own decoder on a `KEY:` line, and Farnsworth spacing. Farnsworth is opt in and never latches. |
| 1.1.0 | EEPROM persistence, protected by a magic word, a layout byte and a CRC-8. Explicit save, unsaved flag in `/status`. The stored contrast is restored, the stored ambient level is not. |
| 1.0.0 | First release. |

---

## License

GPL-3.0
