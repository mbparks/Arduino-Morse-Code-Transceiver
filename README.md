# Optical Morse Terminal

Version 1.0.0
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
hung. 44 assertions across five groups:

- encode then decode round trip over the full supported character set
- timing math at 5, 12, 20 and 30 WPM, including the boundary values, plus a
  check that the stated oversampling floor actually holds at the speed ceiling
- `millis()` rollover arithmetic either side of the wrap point
- ring buffer fill, drain, FIFO order, wrap, overrun and clear
- element classification against synthetic durations at the decision boundaries

Expected result is `SYS: selftest_end pass=44 fail=0 result=GREEN`.

Version 1.0.0 was additionally verified in a host side simulation that compiles
the sketch twice under `g++`, once per namespace, so two virtual boards can be
pointed at each other through a modeled optical path with sensor lag. That
harness covers the link itself, speed lock onto a faster sender, half duplex
gating under heavy self illumination, carrier sense deferral, clean abort, low
contrast refusal, ambient drift of 300 counts with 120 Hz flicker superimposed,
and the whole suite again with both clocks parked before their 32 bit wrap. It
is not required to build or run the sketch.

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
   Expect 8 to 15 WPM in practice. A phototransistor or photodiode front end
   would lift this ceiling substantially and is out of scope for 1.0.0.
3. **No error correction and no retransmission.** A corrupted symbol prints as
   `?` and is counted. That is the entire recovery strategy.
4. **No addressing and no framing.** Every station in the optical path hears
   every transmission. There is no callsign field, no length field, no
   checksum, and no way to tell two senders apart.
5. **Collision avoidance is advisory only**, with the limits described above.
6. **Self interference gating assumes the transmitter is the only local light
   source that matters.** In `/mode full` the assumption of optical isolation
   is entirely on the operator to satisfy physically.
7. **The receive unit estimator tracks one sender.** If two stations at
   different speeds transmit in the same session, the estimate chases whichever
   spoke last, and the first few characters after a handover may decode wrong.
8. **No persistence.** Calibration is lost at every reset.
9. **Console backpressure is deliberate.** Paste a long block of commands and
   the sketch stops reading input until the console has drained. This protects
   the output stream but can overrun the hardware receive buffer if the host
   keeps pushing. Send commands a line at a time.
10. **Not yet bench verified on hardware.** 1.0.0 passes its own self test and
    the host side simulation. The flash and RAM figures in the sketch header
    are unfilled placeholders until someone compiles it for a real board.

---

## Roadmap

Deliberately not built in 1.0.0:

- EEPROM persistence of WPM, mode, and calibration thresholds
- Framing with a callsign, length and CRC, so stations can be told apart and
  corrupt messages rejected rather than printed with `?` holes
- Exponential backoff on deferred transmit, which would make the carrier sense
  worth more than advice
- Sidetone on a piezo, and a straight key input for hand sending
- Farnsworth spacing
- Photodiode plus transimpedance front end, and the higher WPM ceiling and PWM
  carrier modulation that becomes possible with it
- A host side terminal script that consumes the output grammar

---

## License

GPL-3.0
