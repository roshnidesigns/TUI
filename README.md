# TUI — Tangible User Interface

Coursework for the Tangible User Interface class at CIID. Two related pieces of the
same object, built on an **Arduino MKR WiFi 1010**.

## Social Battery

A kinetic object whose state is its own social battery. It has three levels, and they
differ in how the thing *moves* rather than where it sits:

| level | reads as |
|---|---|
| Low | present, but holding still |
| Medium | clearly swaying, a comfortable pace |
| High | wide and lively — fully switched on |

```
SocialBattery/
├── arduino/move-motors/   three servo arms, serial protocol, web control
└── web/                   browser control page (Web Serial, no build step)
```

The web page talks to the board over USB serial and animates a live preview of the
object; it falls back to simulating the whole thing when no board is attached, so the
interaction can be shown without the hardware present.

## The motorized fader

`Fader_Mirror/` is the current sketch — a 100 mm motorized fader driven through an
HW-354 H-bridge, with an 8-pixel NeoPixel strip on D2.

**Read, capture, mimic.** Move the slider by hand and it records where you left it;
let go and it mirrors that position and ping-pongs between the two, at a speed set by
the level. Grab it at any moment and the motor releases instantly.

Where you leave the slider picks the level, measured from the **centre** of travel —
so the scale is symmetric, and a position and its mirror are always the same level:

| level | distance from centre (544) | slider | pixels | colour |
|---|---|---|---|---|
| Off | 0 – 77 | 467 – 621 | none | — |
| Low | 77 – 134 | 410–467, 621–678 | 4,5 | yellow |
| Medium | 134 – 184 | 360–410, 678–728 | 3,4,5,6 | orange |
| High | 184+ | 335–360, 728–753 | 1–8 | magenta |

Travel measured on the bench at **336 – 753**, not the full 0–1023 of the ADC.

### Other sketches

| | |
|---|---|
| `Fader_Mirror_Working/` | the mirror swing this was built from |
| `Fader_PingPong_Base/` | the plain two-point ping-pong it started as |
| `5_Fader_GoToPresetPos/` | seeking a position from a preset array |

## Things learned the hard way

- **Coast and brake are different, and both are needed.** Both driver inputs LOW lets
  the fader move freely under your fingers; both HIGH resists. To read a hand-set
  position you must coast — braking fights the user and the fader feels dead.
- **`analogWrite()` everywhere on the motor pins, never `digitalWrite()`.** On SAMD21
  `analogWrite()` re-muxes the pin to a timer, and a later `digitalWrite()` on that pin
  is silently ignored until `pinMode()` runs again. Mixing them leaves the motor on.
- **Never `while (!Serial)`** — it hangs the sketch whenever the board runs without a
  computer attached, which for a finished piece is most of the time.
- **Never `delay()` in the control loop.** A blocking delay stops the loop sampling,
  so it misses the movement it is supposed to detect.
- **Average several `analogRead()` samples.** Motor noise on the analog line reads as
  movement. Eight samples measured ±1 count; a single read swung ±15.
- **Motor power must not come from the board.** On the MKR's 3.3 V the fader crawled
  at 13 counts/sec; on an external 5 V supply, ~900. Grounds tied, and the fader's pot
  still runs on 3.3 V — the MKR's analog pins are not 5 V tolerant.
- **Detect a hand by direction, not by distance.** The slider should move the way it is
  being driven; if it moves the other way, that is a hand. Waiting for a gap to grow
  needs a big push and misses grabs early in a movement.
- **Always have an absolute per-move timeout**, or an unreachable target means pushing
  into a mechanical stop at full duty forever.

## Wiring

| | pin |
|---|---|
| HW-354 IN1 / IN2 | D4 / D5 |
| fader wiper | A1 (pot across 3.3 V and GND — never 5 V) |
| NeoPixel data | D2 |
| driver VCC/GND | external 5 V supply, ground tied to the board |

Avoid pins 8, 9 and 10 on the MKR WiFi 1010 — they are the SPI bus to the onboard NINA
WiFi module.
