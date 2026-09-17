# Prompt: add the motorized fader to the Social Battery

Open a session on `~/Desktop/Claude Projects/SocialBattery` and paste this whole
file as the first message.

---

I want to add a **motorized slider (fader)** to my existing Social Battery project.
Everything below is verified on my bench over a full working session. The wiring,
the measurements and the gotchas are all things I already hit and solved — please
don't re-derive them or "simplify" them away.

## The goal

Social Battery has three servo arms. Arm 2 (red octagon) is driven by a plain
potentiometer on A1 — a dial the software deliberately cannot move. I want to
**replace that dial with the motorized fader**, so red's level can be set by hand
*and* driven back by software. A dial that pushes back.

## Hardware

- **Arduino MKR WiFi 1010** — SAMD21, 3.3 V logic, 10-bit ADC (0–1023)
- **HW-354 motor driver board** — H-bridge, Motor A channel, IN1/IN2 inputs.
  I could not confirm the chip; treat it as "an H-bridge with no enable pin".
- **100 mm motorized fader** — DC motor plus a feedback potentiometer, six wires
- **External 5 V supply for the motor** — not optional, see the power note below

### Fader wiring

| fader wire | goes to | note |
|---|---|---|
| POT. SLIDER (green) | an analog pin | the wiper — position feedback |
| VCC SLIDER (red) | MKR **VCC** (3.3 V) | **never 5 V** — analog pins are not 5 V tolerant |
| GND SLIDER (grey) | MKR **GND** | |
| VCC MOTOR (red) | driver **Motor A / OUT1** | swap with OUT2 to reverse direction |
| GND MOTOR (black) | driver **Motor A / OUT2** | |
| TOUCH SLIDER (orange) | unconnected | capacitive strip, not used yet |

### Driver wiring

| driver pin | goes to |
|---|---|
| IN1 | a PWM-capable digital pin |
| IN2 | a PWM-capable digital pin |
| IN3 / IN4 | unused (channel B) |
| VCC / GND | **external 5 V supply**, ground tied to MKR GND |

## PIN COLLISIONS — resolve these first

The fader as currently wired **conflicts with Social Battery on two pins**:

| pin | Social Battery uses it for | fader wants it for |
|---|---|---|
| **D5** | blue arm servo (`SERVO_PIN[0]`) | `MOTOR_IN2` |
| **A1** | red arm's potentiometer (`POT_PIN`) | `SLIDER_PIN` |

Proposed resolution — tell me if you see better:

- **Move `MOTOR_IN2` from D5 to D2.** D4 and D2 are both free and PWM-capable.
  Keep `MOTOR_IN1` on D4.
- **Keep the fader's wiper on A1 and delete the standalone potentiometer.**
  The fader's pot *is* the dial now — same pin, same 0–1023 range, same
  `POT_BOUND` thresholds and hysteresis. This is the point of the integration,
  not a workaround.

Also: **pins 8, 9 and 10 are the NINA WiFi SPI bus** — nothing goes there.
And on the MKR 1010, **D9 has no PWM at all**; PWM is on 0–8, 10, A3, A4.

## Measured performance

| condition | result |
|---|---|
| Driver on MKR's 3.3 V | ~13 counts/sec, board browned out repeatedly |
| Driver on external 5 V | ~900 counts/sec, full sweep in ~0.5 s |
| Travel observed | roughly 274 → 762 raw (not the full 0–1023) |
| Position noise | ±1 count with 8-sample averaging, ±15 without |
| Overshoot at full speed | ~13 counts past target with a ±10 margin |

## The sketch

This is the current state. It self-calibrates its travel at boot, homes to the
centre, then swings between a hand-set position and its mirror. Use it as the
reference for motor control and hand detection; the swing behaviour itself is
mine and may not survive the merge.

```cpp
/*
* Tangible User Interface class - CIID
* Motorized slider - self-calibrating mirror swing
*
* On power-up the slider calibrates itself, then drives to the centre of its
* measured travel. From there it swings continuously between a detected
* position x and the mirror of x.
*
* Phases:
*   CAL_LOW   drive down until the reading stops changing - the low stop
*   CAL_HIGH  drive up until it stops - the high stop, and time the sweep
*   HOMING    drive to the centre of the measured travel
*   RESTING   motor released, waiting for your hand
*   HAND      you are moving it; waits until you are still
*   SWINGING  ping-pong between x and its mirror
*
* Grab the slider at any time and it releases the motor, then rebuilds the
* swing around wherever you let go.
*
* The mirror of x in the measured range [min, max] is (min + max - x).
*
* Board:  Arduino MKR WiFi 1010
* Driver: HW-354, Motor A on IN1/IN2
*/

// Pin Definitions
#define MOTOR_IN1 4    // HW-354 IN1 (Motor A) - direction + speed
#define MOTOR_IN2 5    // HW-354 IN2 (Motor A) - direction + speed
#define SLIDER_PIN A1  // Analog input from slider's feedback potentiometer

// ---- Control Parameters ----------------------------------------------------
#define MOTOR_SPEED 220                // Default motor speed (PWM value 0-255)
#define MAX_SLIDER 1023                // Maximum value from analog read
#define MARGIN_ERROR MAX_SLIDER / 100  // Acceptable position error (±1% of max)
#define PRINT_INTERVAL 200             // How often to print debug values (ms)
#define MOVE_TIMEOUT 8000              // Give up on an end rather than push into
                                       // a mechanical stop forever
#define SAMPLES 8                      // analogRead samples averaged per reading

// ---- Calibration -----------------------------------------------------------
#define STILL_DELTA 6       // Counts of change that still count as "moving"
#define STILL_MS 500        // No movement this long, AFTER travelling = at the stop
#define NO_MOVE_MS 2500     // Never moved at all this long = we started at the stop
#define CAL_TIMEOUT 12000   // Safety ceiling on each calibration phase
#define MIN_TRAVEL 100      // Less measured travel than this means calibration failed

// ---- Hand detection --------------------------------------------------------
// While driving, the slider should get steadily CLOSER to its target. If the
// gap instead grows by more than this, something is pushing back - you.
#define GRAB_MARGIN 45
#define SETTLE_MOVE 8      // Counts of change that still count as "hand moving"
#define SETTLE_MS 400      // Hand still this long = you let go
#define MIN_SWING 60       // Below this the two ends are too close to be useful

// Function Declarations
void motorCoast();
void motorStop();
void motorForward(int speed);
void motorBackward(int speed);

// ---- Measured travel, filled in by the calibration phases -------------------
int sliderMin = 0;
int sliderMax = MAX_SLIDER;
int centerPos = MAX_SLIDER / 2;
float sweepRate = 0;   // counts per second across the full travel

// The two ends of the swing, both derived from the detected position
int pointA = 0;     // x - the position you set
int pointB = 0;     // the mirror of x
int targetVal = 0;  // whichever end we are heading for right now

enum Mode { CAL_LOW, CAL_HIGH, HOMING, SWINGING, GRABBED, IDLE };
Mode mode = CAL_LOW;

int bestGap = 0;     // Closest we have got to the target on this leg
int settleRef = 0;   // Reading the settle timer is measured against
int idleRef = 0;     // Reading we watch for a hand while resting
int stillRef = 0;    // Reading the calibration stillness timer watches
bool movedThisPhase = false;  // Has the slider actually travelled this phase?

unsigned long lastPrint = 0;
unsigned long moveStart = 0;
unsigned long settleTime = 0;
unsigned long stillTime = 0;
unsigned long phaseStart = 0;
unsigned long sweepStart = 0;

const char* statusName() {
  if (mode == CAL_LOW)  return "CAL-LOW ";
  if (mode == CAL_HIGH) return "CAL-HIGH";
  if (mode == HOMING)   return "HOMING  ";
  if (mode == IDLE)     return "RESTING ";
  if (mode == GRABBED)  return "HAND    ";
  return "SWINGING";
}

// Average several samples. A single analogRead picks up motor noise, which
// would otherwise look like movement - or like a hand on the slider.
int readSlider() {
  long total = 0;
  for (int i = 0; i < SAMPLES; i++) {
    total += analogRead(SLIDER_PIN);
  }
  return (int)(total / SAMPLES);
}

// True once the reading has held still long enough to call it a mechanical stop.
//
// The catch: at the start of a phase the motor has not spun up yet, so the
// slider is legitimately still for a few hundred milliseconds. Treating that
// as "we have arrived" ends the phase instantly at the starting position - and
// if both phases do that, min and max come out identical. So stillness only
// counts once we have seen the slider actually travel; if it never moves at
// all, we were already sitting against that stop, which takes longer to call.
bool pressedAgainstStop(int sliderVal) {
  if (abs(sliderVal - stillRef) > STILL_DELTA) {
    stillRef = sliderVal;
    stillTime = millis();
    movedThisPhase = true;
    return false;
  }
  if (!movedThisPhase) {
    return (millis() - phaseStart >= NO_MOVE_MS);
  }
  return (millis() - stillTime >= STILL_MS);
}

void beginPhase(Mode m, int sliderVal) {
  mode = m;
  phaseStart = millis();
  moveStart = millis();
  stillRef = sliderVal;
  stillTime = millis();
  movedThisPhase = false;
}

// Build the swing around a freshly detected position
void detectFrom(int x) {
  pointA = constrain(x, sliderMin, sliderMax);
  pointB = constrain(sliderMin + sliderMax - pointA, sliderMin, sliderMax);

  Serial.print("DETECTED x = ");
  Serial.print(pointA);
  Serial.print("  ->  swinging ");
  Serial.print(pointA);
  Serial.print(" <-> ");
  Serial.println(pointB);

  if (abs(pointB - pointA) < MIN_SWING) {
    // x sat near the middle, so x and its mirror nearly coincide. Twitching
    // across a few counts reads as a fault, so rest here instead and wait for
    // a hand to give us something to work with.
    Serial.println("  (too close to the centre to swing - resting, move the slider)");
    motorCoast();
    idleRef = x;          // watch from where the slider ACTUALLY is, not the
    targetVal = x;        // clamped value - otherwise we retrigger immediately
    mode = IDLE;
    return;
  }

  targetVal = pointB;
  bestGap = abs(targetVal - pointA);
  moveStart = millis();
  mode = SWINGING;
}

// Flip to the other end of the swing and restart this leg
void swapTarget(int sliderVal) {
  targetVal = (targetVal == pointA) ? pointB : pointA;
  bestGap = abs(targetVal - sliderVal);
  moveStart = millis();
}

void setup() {
  // No `while (!Serial)` - it would stall the sketch when run without a computer
  Serial.begin(115200);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  motorCoast();

  Serial.println();
  Serial.println("=== CALIBRATING - hands off ===");
  beginPhase(CAL_LOW, readSlider());
}

void loop() {
  int sliderVal = readSlider();

  // Print elapsed time, status and position on every interval. Never use
  // `while (!Serial)` or delay() here - the loop must keep sampling.
  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();

    unsigned long ms = millis();
    Serial.print("t=");
    Serial.print(ms / 1000);
    Serial.print(".");
    unsigned long frac = ms % 1000;
    if (frac < 100) Serial.print("0");
    if (frac < 10)  Serial.print("0");
    Serial.print(frac);
    Serial.print("s  ");

    Serial.print(statusName());
    Serial.print("  slider: ");
    Serial.print(sliderVal);
    Serial.print("  target: ");
    if (mode == CAL_LOW)       Serial.println("(low stop)");
    else if (mode == CAL_HIGH) Serial.println("(high stop)");
    else if (mode == HOMING)   Serial.println(centerPos);
    else                       Serial.println(targetVal);
  }

  switch (mode) {

    // ---- Drive down until the slider stops moving: the low stop ----
    case CAL_LOW: {
      bool timedOut = (millis() - phaseStart >= CAL_TIMEOUT);

      if (pressedAgainstStop(sliderVal) || timedOut) {
        motorStop();
        sliderMin = sliderVal;
        Serial.print(timedOut ? "LOW  (timed out) = " : "LOW  stop = ");
        Serial.println(sliderMin);
        sweepStart = millis();
        beginPhase(CAL_HIGH, sliderVal);
      } else {
        motorBackward(MOTOR_SPEED);
      }
      break;
    }

    // ---- Drive up until it stops: the high stop, and time the sweep ----
    case CAL_HIGH: {
      bool timedOut = (millis() - phaseStart >= CAL_TIMEOUT);

      if (pressedAgainstStop(sliderVal) || timedOut) {
        motorStop();
        sliderMax = sliderVal;

        // Make sure min really is the smaller of the two, whichever way the
        // motor happens to be wired
        if (sliderMin > sliderMax) {
          int t = sliderMin; sliderMin = sliderMax; sliderMax = t;
        }

        // If the two stops came out on top of each other, calibration did not
        // work - fall back to the full ADC range rather than a useless window
        if (sliderMax - sliderMin < MIN_TRAVEL) {
          Serial.print("CALIBRATION FAILED - only ");
          Serial.print(sliderMax - sliderMin);
          Serial.println(" counts of travel. Falling back to 0-1023.");
          sliderMin = 0;
          sliderMax = MAX_SLIDER;
          sweepRate = 0;   // the measured rate is meaningless if nothing moved
        }

        centerPos = (sliderMin + sliderMax) / 2;

        unsigned long elapsed = millis() - sweepStart;
        if (elapsed > STILL_MS) elapsed -= STILL_MS;   // drop the settling time
        sweepRate = (elapsed > 0) ? ((sliderMax - sliderMin) * 1000.0 / elapsed) : 0;

        Serial.print(timedOut ? "HIGH (timed out) = " : "HIGH stop = ");
        Serial.println(sliderMax);
        Serial.println();
        Serial.println("=== CALIBRATION RESULTS ===");
        Serial.print("  SLIDER_MIN      ");
        Serial.println(sliderMin);
        Serial.print("  SLIDER_MAX      ");
        Serial.println(sliderMax);
        Serial.print("  CENTER          ");
        Serial.println(centerPos);
        Serial.print("  usable travel   ");
        Serial.print(sliderMax - sliderMin);
        Serial.println(" counts");
        if (sweepRate > 0) {
          Serial.print("  sweep speed     ");
          Serial.print(sweepRate, 1);
          Serial.println(" counts/sec");
          Serial.print("  full sweep      ");
          Serial.print((sliderMax - sliderMin) / sweepRate, 2);
          Serial.println(" sec");
        } else {
          Serial.println("  sweep speed     not measured (slider never moved)");
        }
        Serial.println("===========================");
        Serial.println();
        Serial.print("HOMING to ");
        Serial.println(centerPos);

        beginPhase(HOMING, sliderVal);
      } else {
        motorForward(MOTOR_SPEED);
      }
      break;
    }

    // ---- Drive to the centre of the measured travel ----
    case HOMING: {
      bool timedOut = (millis() - moveStart >= MOVE_TIMEOUT);

      if (abs(centerPos - sliderVal) <= MARGIN_ERROR || timedOut) {
        motorStop();
        if (timedOut) {
          Serial.print("HOMING gave up at ");
          Serial.println(sliderVal);
        } else {
          Serial.print("AT CENTER ");
          Serial.println(sliderVal);
        }
        detectFrom(sliderVal);
      } else if (sliderVal > centerPos) {
        motorBackward(MOTOR_SPEED);
      } else {
        motorForward(MOTOR_SPEED);
      }
      break;
    }

    // ---- Resting: motor off, waiting for a hand ----
    case IDLE: {
      motorCoast();
      if (abs(sliderVal - idleRef) > SETTLE_MOVE) {
        Serial.println("HAND DETECTED - set it where you like");
        mode = GRABBED;
        settleRef = sliderVal;
        settleTime = millis();
      }
      break;
    }

    // ---- Your hand is on it: motor off, wait for you to finish ----
    case GRABBED: {
      motorCoast();
      if (abs(sliderVal - settleRef) > SETTLE_MOVE) {
        settleRef = sliderVal;
        settleTime = millis();
      } else if (millis() - settleTime >= SETTLE_MS) {
        // Still for long enough - that is where you wanted it
        detectFrom(sliderVal);
      }
      break;
    }

    // ---- Swinging between the two ends ----
    case SWINGING: {
      int gap = abs(targetVal - sliderVal);

      // Getting closer is normal; the gap growing means something pushed back
      if (gap < bestGap) {
        bestGap = gap;
      } else if (gap > bestGap + GRAB_MARGIN) {
        Serial.println("HAND DETECTED - motor released, set it where you like");
        motorCoast();
        mode = GRABBED;
        settleRef = sliderVal;
        settleTime = millis();
        break;
      }

      // Safety net - if an end cannot be reached, swing back rather than grind
      if (millis() - moveStart >= MOVE_TIMEOUT) {
        Serial.print("GAVE UP at ");
        Serial.print(sliderVal);
        Serial.print(" chasing ");
        Serial.println(targetVal);
        motorStop();
        swapTarget(sliderVal);
        break;
      }

      // Compare current position with target, allowing the error margin
      if (sliderVal > targetVal + MARGIN_ERROR) {
        // Current position is too far forward - move backward
        motorBackward(MOTOR_SPEED);
      } else if (sliderVal < targetVal - MARGIN_ERROR) {
        // Current position is too far back - move forward
        motorForward(MOTOR_SPEED);
      } else {
        // Reached this end of the swing - turn around immediately, no dwell
        motorStop();
        swapTarget(sliderVal);
      }
      break;
    }
  }
}

// Motor Control Functions
//
// The HW-354 board has no enable pin: direction AND speed both come from
// IN1/IN2. Hold one input LOW and PWM the other to drive at that speed.
//
//   IN1   IN2   result
//   LOW   LOW   coast  - free to move by hand
//   PWM   LOW   forward at duty
//   LOW   PWM   backward at duty
//   HIGH  HIGH  brake  - resists movement, holds position
//
// Everything below uses analogWrite() rather than digitalWrite(). On the
// MKR 1010 (SAMD21) analogWrite() re-muxes the pin to a timer peripheral, and
// a later digitalWrite() on that same pin is ignored until pinMode() is called
// again. Staying on analogWrite() the whole way avoids that trap.

void motorCoast() {
  // Both inputs LOW - motor unpowered, slider moves freely by hand
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, 0);
}

void motorStop() {
  // Brake motor by setting both inputs HIGH (full duty)
  analogWrite(MOTOR_IN1, 255);
  analogWrite(MOTOR_IN2, 255);
}

void motorForward(int speed) {
  // Idle pin first so the two inputs are never briefly both driven
  analogWrite(MOTOR_IN2, 0);
  analogWrite(MOTOR_IN1, speed);
}

void motorBackward(int speed) {
  // Idle pin first so the two inputs are never briefly both driven
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, speed);
}
```

## Hard-won gotchas — do not undo these

1. **Use `analogWrite()` everywhere on the motor pins, never `digitalWrite()`.**
   On SAMD21, `analogWrite()` re-muxes the pin to a timer peripheral and a later
   `digitalWrite()` on that pin is silently ignored until `pinMode()` runs again.
   Mixing them leaves the motor stuck on. Use `analogWrite(pin, 0)` for LOW and
   `analogWrite(pin, 255)` for HIGH.

2. **Coast and brake are different, and both are needed.** Both inputs LOW =
   coast, slider moves freely under your fingers. Both HIGH = brake, it resists.
   To read a hand-set position you *must* coast — braking fights the user and the
   fader feels dead. This is the single most important thing in the file.

3. **Never `while (!Serial)`.** It hangs the sketch whenever the board runs
   without a computer attached.

4. **Never `delay()` in the control loop.** A blocking delay stops the loop
   sampling the slider, so it misses the movement it is supposed to detect and
   reads one stale value afterwards. Every timer here is `millis()`-based.

5. **Average several `analogRead()` samples.** Motor noise on the analog line
   reads as movement. A naive "has it moved?" stall check is defeated by it — I
   watched the motor stall at full duty for 14 seconds while the guard thought it
   was still travelling.

6. **Motor power must not come from the MKR.** On the 3.3 V pin the fader crawled
   at 13 counts/sec and the board repeatedly dropped off USB from brownout.
   External 5 V, grounds tied. The fader's *pot* still runs on 3.3 V.

7. **Do not conclude "we reached the end stop" from stillness alone.** At the
   start of a move the motor has not spun up, so the slider is legitimately still
   for a few hundred ms. Treating that as arrival ends calibration instantly at
   the starting position — and if both ends do it, min and max come out
   identical and every mapped value collapses. Require evidence of travel first.

8. **Always have an absolute per-move timeout.** Without one, an unreachable
   target means pushing into a mechanical stop at full duty forever.

9. **The Serial Monitor blocks uploads.** If any monitor holds the port, the
   1200-bps touch reset fails with `Resource busy` and `bossac` reports
   `No device found`. Close it before flashing. Check with `lsof | grep usbmodem`.

10. **The port number shifts** between `/dev/cu.usbmodem101` and
    `/dev/cu.usbmodem1101` across replugs. Detect it, never hardcode it.

## What I want built

1. Fold the fader into `arduino/move-motors/move-motors.ino` with the pin
   conflicts resolved.
2. The fader's pot replaces the standalone potentiometer as red's dial, keeping
   the existing `POT_BOUND` thresholds and hysteresis.
3. Software can now *drive* the fader, so setting red's state from the web page
   or over serial moves the physical fader to match. Hand-moving it still wins.
4. Extend the web page so red's control reflects and commands the fader position.

Ask me before changing the existing serial protocol or the servo motion model —
those work and I don't want them disturbed.

## Known open issues

- **Overshoot.** At ~900 counts/sec with a ±10 margin it sails past the target by
  ~13 counts. Needs a speed ramp: full duty when far, tapering near the target.
- **Occasional multi-second hangs** around the 300–430 region before breaking
  free. Either stiction or a supply current limit; not diagnosed.
- **`MIN_SPEED` was never measured.** The lowest duty that actually breaks
  friction is unknown — I have only guessed at 110 and 160. Worth measuring.
- Power delivery has been flaky all session; if the motor does nothing at all,
  check the supply before suspecting code.
