/*
 * Social Battery — full sketch
 * Tangible User Interface, CIID
 *
 * Two servo arms and a motorized fader on one board. They are completely
 * independent: neither drives the other, and they share only the board and the
 * serial port. Either half can be removed without touching the other.
 *
 * ── THE ARMS ────────────────────────────────────────────────────────────────
 *
 * Two servos that ping-pong about a resting angle. They stand still at power-up
 * and wait to be told; the web page (or the serial commands below) sets a level
 * per arm, and the level decides how wide and how fast that arm swings.
 *
 *   level    amplitude          seconds per there-and-back
 *   LOW      ±15°               35
 *   MEDIUM   ±35°               14
 *   HIGH     ±50°               10.5
 *
 * Arms ping-pong rather than sway: a sine lingers at the ends and rushes the middle,
 * where the fader crosses at a constant rate and turns sharply — matching them makes
 * the two halves read as one object.
 *
 * A stopped arm eases back to CENTER_ANGLE and then RELEASES. Holding torque at
 * rest keeps it rigidly vertical but means the servo pushes continuously, and if
 * the arm is fighting anything mechanical that is a permanent stall — full
 * current, no movement, all of it heat. Set HOLD_AT_REST 1 to go back to holding.
 *
 * ── THE FADER ───────────────────────────────────────────────────────────────
 *
 * A 100mm motorized fader that reads, captures and mimics a hand-set gesture.
 * Move it by hand and the motor releases; let go and it mirrors your position and
 * ping-pongs between the two. Where you leave it picks a level, measured as equal
 * quarters either side of the CENTRE of travel — so the scale is symmetric and a
 * position and its mirror are always the same level.
 *
 * Six pixels of a 24-pixel ring show that level: two lit at Low, four at Medium,
 * six at High, growing outward from the middle of the six.
 *
 * ── WIRING — Arduino MKR WiFi 1010 ──────────────────────────────────────────
 *
 *   D1    NeoPixel ring data          D3/D5  HW-354 driver IN1 / IN2
 *   D0    servo arm 0 (blue)          A1     fader wiper
 *   D7    servo arm 1 (orange)        D6     built-in LED (status)
 *
 *   Servo power and driver power both from an EXTERNAL 5V supply, with its
 *   ground tied to the board's. The MKR's regulator cannot source motor current,
 *   and trying makes the board brown out and drop off USB mid-movement.
 *
 *   The fader's pot runs on 3.3V, never 5V — the analog pins are not 5V tolerant.
 *
 *   Avoid D8, D9 and D10 — the SPI bus to the onboard NINA WiFi module.
 *
 * ── SERIAL, 115200 ──────────────────────────────────────────────────────────
 *
 *   H                  handshake; answers OK:HELLO ... ARMS=<n>. The web page
 *                      uses this to tell a real board from any other serial port.
 *   T:<arm>:<state>    start one arm, e.g. T:0:HIGH
 *   T:<state>          start every arm
 *   X:<arm>  /  X      stop one arm / all of them
 *   C:<arm>:<angle>    hold one arm still, for finding its resting vertical
 *   A                  print the live per-arm left/right trim table
 *   A:<arm>:<pos>:<neg>   trim one side of an arm's swing to match the other
 *   V                  print the live amplitude/speed table
 *   V:<state>:<amp>:<rate>   retune a level without re-uploading
 *   k                  calibrate the fader's travel
 *   F:<level>          paced swing at that level, timed to match the arms, e.g. F:HIGH
 *   F:<value>          flat-out to an exact raw slider position instead, e.g. F:700
 *   1 / 2 / 3 / 0      all arms to low / medium / high / stop
 *   a                  start the standalone demo cycle
 *   c                  hold every arm at CENTER_ANGLE
 *
 *   Telemetry, 10Hz:  S:<state>,<run>,<angle>;<state>,<run>,<angle>
 *
 * ── WHAT TO TUNE FIRST ──────────────────────────────────────────────────────
 *
 *   CENTER_ANGLE   each arm's resting vertical. Get this wrong and the arm is
 *                  driven past what the linkage can reach, where it stalls
 *                  against its stop and overheats. Find it with C:<arm>:<angle>.
 *   stateAmp       how far each level swings, in degrees either side of centre.
 *   SLIDER_MIN/MAX the fader's measured travel; every band edge derives from it.
 *
 * ── THINGS LEARNED THE HARD WAY — please do not undo these ──────────────────
 *
 *  1. Coast and brake are different, and both are needed on the fader. To read a
 *     hand-set position you must coast; braking fights the user.
 *  2. analogWrite() everywhere on the motor pins, never digitalWrite(). On SAMD21
 *     analogWrite() re-muxes the pin to a timer and a later digitalWrite() on it
 *     is silently ignored until pinMode() runs again.
 *  3. Never `while (!Serial)` — it hangs the sketch when no computer is attached.
 *  4. Never delay() in the loop; it stops the sampling the sketch depends on.
 *  5. Take a MEDIAN of several analogRead()s, not a mean — motor noise reaches
 *     the rails, and a mean is dragged by one bad sample where a median ignores it.
 *
 * Board: Arduino MKR WiFi 1010
 */

#include <Servo.h>               // SAMD Servo library - drives any digital pin via a timer
#include <Adafruit_NeoPixel.h>   // drives the NeoPixel ring

// ---------------------------------------------------------------- pins
// One entry per arm in SERVO_PIN below - state, telemetry and arm count all size
// themselves from it. SERVOS_ENABLED 0 stops all pulses, freeing the supply for the fader.
#define SERVOS_ENABLED 1

// Release a stopped arm once it has settled, instead of holding it there (holding
// keeps it rigidly vertical but a fighting servo stalls: full current, no movement,
// all heat). Set to 1 to hold instead.
#define HOLD_AT_REST 0
#define RELEASE_AFTER_MS 900   // settle time before letting go

const uint8_t SERVO_PIN[] = { 0, 7 };   // one entry per arm - D0 (blue) and D7 (orange)
// Avoid pins 8-10 (SPI to the onboard NINA WiFi module) - a servo there works only
// until something switches the radio on.

const uint8_t MAX_ARMS  = 3;                                     // room for a third arm, unused for now
const uint8_t ARM_COUNT = sizeof(SERVO_PIN) / sizeof(SERVO_PIN[0]);  // sizes itself from SERVO_PIN above
const uint8_t LED_PIN   = LED_BUILTIN;   // pin 6 on the MKR boards

// Servo uses a hardware timer, not analogWrite(), so D0/D7 don't collide with D3/D5
// doing analogWrite() for the motor driver below.

// ---------------------------------------------------------------- tuning

// Resting pose - the middle of each arm's swing. Both D0 and D1 confirmed vertical at
// 93 with C:<arm>:<angle>. Open-loop, so a slipped horn is invisible to the code and
// only shows up as the arm no longer sitting at 93 - re-measure the same way if so.
const int CENTER_ANGLE[MAX_ARMS] = { 93, 93, 93 };   // resting vertical, one entry per arm

// Hard clamp on every movement. Keep inside whatever the linkage can physically reach.
const int ANGLE_MIN = 10;
const int ANGLE_MAX = 210;

// Per-arm, per-side trim for a swing that reads wider on one side (usually linkage
// geometry, not the symmetric math below). 1.0 = no trim, 0.8 pulls that side in 20%.
// Tune live with A:<arm>:<posScale>:<negScale>, then paste the values back here.
float ampScalePos[MAX_ARMS] = { 1.0, 1.0, 1.0 };   // scale for the tri >= 0 half of the swing
float ampScaleNeg[MAX_ARMS] = { 1.0, 1.0, 1.0 };   // scale for the tri <  0 half of the swing

enum State { ST_LOW = 0, ST_MED = 1, ST_HIGH = 2, STATE_COUNT = 3 };   // the three swing levels
const char *STATE_NAME[STATE_COUNT] = { "LOW", "MED", "HIGH" };        // for serial printouts

// Amplitude (degrees either side of CENTER_ANGLE, symmetric) and rate (rad/s) per
// state.                                  LOW    MED    HIGH
// Live-tunable with V:<state>:<amp>:<rate> against the real object - paste back here.
float stateAmp[STATE_COUNT]  = { 10.0, 25.0, 45.0 };   // degrees either side of centre, per state
float stateRate[STATE_COUNT] = { 0.18,  0.45,  0.60 };  // radians per second

// Seconds to cross from one state's amplitude/speed to another's. A state change is a
// mood change, not a switch — it should be readable as it happens.
const float STATE_BLEND_SEC = 1.4;

// Speed limit on every movement, degrees per second. This is what makes the arm read
// as alive rather than as machinery; lower is heavier and more reluctant.
const float SLEW_DEG_PER_SEC = 140.0;

// Seconds per state in the standalone demo cycle.
const unsigned long DWELL_MS = 6000;

// ---------------------------------------------------------------- state

Servo servos[MAX_ARMS];                                     // one Servo object per possible arm
bool  servosAttached[MAX_ARMS] = { false, false, false };   // whether attach() has been called - detached = no pulses sent

// Everything below is per arm.
State armState[MAX_ARMS]   = { ST_MED, ST_MED, ST_MED };   // which level each arm is set to
bool  armRunning[MAX_ARMS] = { false, false, false };      // whether this arm is currently swinging
bool  armHeld[MAX_ARMS]    = { false, false, false };      // C:<arm>:<angle> hold - frozen, ignored by updateArms()
float armAngle[MAX_ARMS];                                  // what the servo is actually holding right now
unsigned long restSince[MAX_ARMS] = { 0, 0, 0 };           // when an arm arrived at rest (0 = not resting yet)
float armAmp[MAX_ARMS]  = { 0, 0, 0 };   // smoothed toward the state's amplitude

// One shared phase clock per STATE, not per arm, so any two arms sharing a state stay
// in lock step - it runs continuously so a newly-joining arm lines up immediately.
float statePhase[STATE_COUNT] = { 0, 0, 0 };   // radians, wraps via fmod in updateArms()

// The arms stand still at power-up and wait to be told. The standalone demo is still
// here — press 'a' on the serial line to start it — but it no longer runs uninvited,
// so the object is quiet until something asks it to move.
bool autoCycle = false;      // true while the standalone demo cycle is driving the arms
bool holdCenter = false;    // park at CENTER_ANGLE while setting the resting pose
uint8_t cycleState = 0;      // which state the demo cycle is currently on
unsigned long lastSwitch = 0;   // millis() timestamp the demo cycle last changed state

char line[32];                 // incoming serial command, built up character by character
uint8_t lineLen = 0;            // how many characters of `line` are filled so far
unsigned long lastTick = 0, lastReport = 0;   // millis() timestamps for the motion and telemetry timers
unsigned long lastSliderPrint = 0;            // millis() timestamp the slider value was last printed while grabbed

bool  lastArmRunning[MAX_ARMS] = { false, false, false };   // so the angle line below only
State lastArmState[MAX_ARMS]   = { ST_MED, ST_MED, ST_MED };  // prints when something changes

// ---------------------------------------------------------------- led

// On/off durations in ms, alternating, starting with ON. A zero ends the pattern.
// Handshake: three quick blinks, a beat, then one long one — unmistakable across a
// room, and distinct from the bootloader's own flicker at reset.
const uint16_t PATTERN_HELLO[] = { 90, 90, 90, 90, 90, 300, 700, 0 };

uint16_t ledPattern[12];             // working copy of whichever pattern is playing
uint8_t  ledSteps = 0, ledStep = 0;  // total steps in the pattern, and which one we are on
unsigned long ledStepStart = 0;      // millis() timestamp the current step began
bool ledPlaying = false;             // true while a pattern is actively running

// Copies a pattern into the working buffer and starts playing it from step 0.
void playPattern(const uint16_t *p) {
  const uint8_t cap = sizeof(ledPattern) / sizeof(ledPattern[0]);   // buffer capacity
  ledSteps = 0;
  while (p[ledSteps] != 0 && ledSteps < cap) { ledPattern[ledSteps] = p[ledSteps]; ledSteps++; }  // copy until the terminating 0
  ledStep = 0;
  ledStepStart = millis();
  ledPlaying = ledSteps > 0;
}

// True if any arm is currently swinging - used to make the LED double as an "arms
// moving" indicator once no pattern is playing.
bool anyRunning() {
  for (uint8_t i = 0; i < ARM_COUNT; i++) if (armRunning[i]) return true;
  return false;
}

// Advances whichever LED pattern is playing, or falls back to showing whether
// anything is moving once the pattern has finished.
void updateLed() {
  if (ledPlaying) {
    if (millis() - ledStepStart >= ledPattern[ledStep]) {   // this step's duration has elapsed
      ledStepStart = millis();
      if (++ledStep >= ledSteps) ledPlaying = false;        // pattern finished
    }
    if (ledPlaying) {
      digitalWrite(LED_PIN, (ledStep % 2 == 0) ? HIGH : LOW);   // even steps are ON
      return;
    }
  }
  // no pattern playing: the LED reports whether anything is moving
  digitalWrite(LED_PIN, anyRunning() ? HIGH : LOW);
}

// ---------------------------------------------------------------- helpers

// Clamps v to the [lo, hi] range.
float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Case-insensitive compare. Written out rather than using strcasecmp, which the AVR
// and SAMD cores expose from different headers.
bool eq(const char *a, const char *b) {
  while (*a && *b) { if (toupper(*a) != toupper(*b)) return false; a++; b++; }   // walk both strings together
  return *a == *b;    // both must have ended at the same time
}

// Returns STATE_COUNT if the name is not one of ours.
uint8_t parseState(const char *n) {
  for (uint8_t i = 0; i < STATE_COUNT; i++) if (eq(n, STATE_NAME[i])) return i;   // match against LOW/MED/HIGH
  if (eq(n, "MEDIUM")) return ST_MED;   // the page spells it out; accept both
  return STATE_COUNT;                   // not a recognised name
}

// Attaches or detaches one arm's servo. Detaching stops pulses entirely, which is
// what lets a resting arm go limp instead of holding torque forever.
void attachArm(uint8_t i, bool on) {
  if (!SERVOS_ENABLED) return;   // arms parked: never attach, never pulse
  if (on == servosAttached[i]) return;   // already in the requested state - nothing to do
  if (on) servos[i].attach(SERVO_PIN[i]);
  else    servos[i].detach();
  servosAttached[i] = on;
}

// Starts (or re-levels) one arm's swing. Starting from stopped snaps amplitude
// immediately (slew limiter still walks the angle there); switching states on an
// already-running arm eases amplitude across and just re-reads the new state's clock,
// which re-syncs it to that state's other arms.
void startArm(uint8_t i, State s) {
  if (!armRunning[i]) armAmp[i] = stateAmp[s];    // only snap amplitude when starting from a stop
  armState[i] = s;          // record the requested level
  armRunning[i] = true;     // this arm should now be swinging
  armHeld[i] = false;       // a state command always overrides a manual C: hold
  holdCenter = false;       // starting an arm cancels the "hold at centre" calibration mode
  attachArm(i, true);       // make sure pulses are actually reaching the servo
}

// Starts every arm at the given state.
void startAll(State s) { for (uint8_t i = 0; i < ARM_COUNT; i++) startArm(i, s); }

// Any command from outside takes the object off its standalone demo cycle.
void takeControl() { autoCycle = false; }

void stopArm(uint8_t i) { armRunning[i] = false; armHeld[i] = false; }   // eases home and holds there, powered
void stopAll() {
  for (uint8_t i = 0; i < ARM_COUNT; i++) stopArm(i);
  holdCenter = false;   // also cancel the centre-holding calibration mode
}

// ---------------------------------------------------------------- motion

// Called every motion tick (50Hz). Advances the phase clocks, eases each arm's
// amplitude toward its target, computes this instant's angle from the ping-pong
// wave, slew-limits the move and writes it to the servo.
void updateArms(float dt) {
  // Every state's clock advances regardless of use, so a joining arm lands in step.
  for (uint8_t s = 0; s < STATE_COUNT; s++) statePhase[s] += stateRate[s] * dt;   // advance each state's shared clock

  float k = clampf(dt / STATE_BLEND_SEC, 0.0, 1.0);   // how far to ease amplitude toward its target this tick
  float maxStep = SLEW_DEG_PER_SEC * dt;              // maximum angle change allowed this tick

  for (uint8_t i = 0; i < ARM_COUNT; i++) {
    // A C:<arm>:<angle> hold freezes this arm entirely, skipped here so it isn't
    // pulled back toward CENTER_ANGLE the instant it's set.
    if (armHeld[i]) continue;

    State s = armState[i];
    bool moving = armRunning[i] && !holdCenter;   // should this arm actually be swinging right now?

    // Ease amplitude toward this arm's state (a swing-width change reads as a
    // transition, not a snap); phase itself just reads the state's shared clock.
    float wantAmp = moving ? stateAmp[s] : 0.0;   // stopping fades the swing out
    armAmp[i] += (wantAmp - armAmp[i]) * k;       // exponential ease toward wantAmp

    // Triangle wave, not a sine - crosses at a constant rate like the fader does,
    // rather than lingering at the ends, so the two read as one object.
    float cycle = fmod(statePhase[s], TWO_PI);    // wrap the shared clock into one cycle
    if (cycle < 0) cycle += TWO_PI;                // fmod can return negative - correct it
    float tri = (cycle < PI) ? (cycle / PI) * 2.0 - 1.0     // -1 -> +1
                             : 1.0 - ((cycle - PI) / PI) * 2.0;  // +1 -> -1

    float dirScale = (tri >= 0) ? ampScalePos[i] : ampScaleNeg[i];   // per-side trim for this arm
    float target = moving
      ? CENTER_ANGLE[i] + tri * armAmp[i] * dirScale   // swinging - centre plus the triangle wave scaled by amplitude and side trim
      : CENTER_ANGLE[i];                    // stopped - head back to resting vertical
    target = clampf(target, ANGLE_MIN, ANGLE_MAX);   // never command past what the linkage can reach

    // slew limit so the arm never snaps
    float delta = target - armAngle[i];        // how far we would like to move this tick
    if (delta >  maxStep) delta =  maxStep;     // clamp to the speed limit, positive direction
    if (delta < -maxStep) delta = -maxStep;     // clamp to the speed limit, negative direction
    armAngle[i] += delta;                       // apply the (possibly clamped) step

    if (servosAttached[i]) servos[i].write((int)(armAngle[i] + 0.5));   // send the pulse, rounded to the nearest degree

    // Once a stopped arm has arrived, let it go — nothing to hold it against.
    if (!HOLD_AT_REST && !armRunning[i] && servosAttached[i]
        && fabs(armAngle[i] - CENTER_ANGLE[i]) <= 1.0) {   // stopped, attached, and back at rest
      if (restSince[i] == 0) restSince[i] = millis();                                   // start the settle timer
      else if (millis() - restSince[i] >= RELEASE_AFTER_MS) attachArm(i, false);        // settled long enough - detach
    } else if (armRunning[i]) {
      restSince[i] = 0;   // running again - clear the settle timer
    }

    // HOLD_AT_REST never auto-releases: a released servo goes limp and gravity pulls
    // it off CENTER_ANGLE. Detach by hand with E:0 to move the linkage freely.
  }
}


// ================================================================== FADER
//
// The motorized fader, on its own pins and its own state machine. It reads,
// captures and mimics a hand-set gesture entirely on its own — it neither drives
// the servo arms nor is driven by them. The two share only the board and the
// serial port.
// ==========================================================================

// ---- motorized fader + NeoPixel (its own pins, its own loop) ----------------
#define MOTOR_IN1 3    // HW-354 IN1 (Motor A) - direction + speed
#define MOTOR_IN2 5    // HW-354 IN2 (Motor A) - direction + speed
#define SLIDER_PIN A1  // the fader's wiper
#define STRIP_PIN 1    // NeoPixel data

// ---- Travel limits, measured on the bench ----------------------------------
// True mechanical stops (from the k command) are 11/1007, but driving flat-out to
// them stalled and caused false hand-detections, so this is pulled in to 200/800,
// shifted +7 to centre on the measured physical centre (507).
#define SLIDER_MIN 207
#define SLIDER_MAX 807

// A reading at/past these is a floated input (motor noise), not real travel - the
// slider physically cannot reach 0 or 1023. Discarded in favour of the last good one.
#define RAIL_LOW 8         // at or below this the input has floated, not moved
#define RAIL_HIGH 1015     // at or above this the input has floated, not moved
#define SLIDER_CENTER ((SLIDER_MIN + SLIDER_MAX) / 2)   // 544 - midpoint of travel, the OFF mark

// Motor noise also lands mid-range, worst right at a direction change. For
// SLIDER_BLANK_MS after swapTarget()/detectFrom() runs, a reading further than
// SLIDER_JUMP_MAX from the last good one is distrusted - otherwise a noise spike
// could register as "arrived" mid-swing or as sustained pushback (a false hand-detect).
#define SLIDER_JUMP_MAX 40    // counts further than this from lastGood = suspect, within the window
#define SLIDER_BLANK_MS 80    // how long after a direction change to distrust a big jump

// ---- The levels ------------------------------------------------------------
// faderLevel is set by distance from centre, either direction (symmetric, so a
// position and its mirror are always the same level), in equal quarters of the
// half-travel. Derived from SLIDER_MIN/MAX so recalibrating moves the bands with it.
#define HALF_TRAVEL   ((SLIDER_MAX - SLIDER_MIN) / 2)   // furthest distance from centre reachable
#define DIST_OFF_LOW  (HALF_TRAVEL / 4)                 // 1st quarter boundary - OFF ends, LOW begins
#define DIST_LOW_MED  (HALF_TRAVEL * 2 / 4)              // 2nd quarter boundary - LOW ends, MEDIUM begins
#define DIST_MED_HIGH (HALF_TRAVEL * 3 / 4)              // 3rd quarter boundary - MEDIUM ends, HIGH begins

#define BAND_HYSTERESIS 8   // a reading must clear a boundary by this much to change level

// ---- NeoPixel --------------------------------------------------------------
// 24-pixel ring, six ever lit as one arc growing outward from its middle - two at
// Low, four at Medium, six at High. Change LED_SLOT to move which six pixels light.
#define STRIP_COUNT 24        // the whole ring; only six of them are ever lit
#define STRIP_BRIGHTNESS 90
const uint8_t LED_USED = 6;                                    // how many of the 24 pixels are ever lit
const uint8_t LED_SLOT[LED_USED] = { 5, 6, 7, 8, 9, 10 };       // which physical pixel indices those six are
const uint8_t LED_LIT[4] = { 0, 2, 4, 6 };                      // pixels lit, indexed by level

// How hard each faderLevel is driven. The swing's ENDS come from the mirror - where you
// left the slider, and its reflection - so the gesture is yours; the faderLevel only says
// how energetically it comes back.
//                            Off  Low  Medium  High
const int SWING_SPEED[4] = {   0,  230,   243,   255 };            // PWM duty per level, 0-255
const char* LEVEL_NAME[4] = { "OFF", "LOW", "MEDIUM", "HIGH" };     // for serial printouts only

// Below this the two ends are too close together to be worth swinging between
#define MIN_SWING 60

// A faderLevel change needs evidence of a real hand gesture - without this, a stalled
// swing near an end could re-capture at that extreme and silently promote to HIGH.
#define MIN_GESTURE 25

// Overshoot control: ease off duty over the last stretch instead of arriving flat
// out. MIN_DUTY is the lowest duty that reliably breaks friction (conservative guess).
#define RAMP_ZONE 60         // only taper close to the target, not half the leg
#define MIN_DUTY 215        // 154 did not move it at all; 190 still crawled

// Friction varies and was never measured properly, so if the slider stalls while
// driving, wind the duty up until it moves again.
#define STALL_MS 200        // no progress for this long while driving = stuck
#define STALL_STEP 12       // how much to add each time
#define STALL_NOISE 2       // counts of change that do not count as progress
#define BOOST_MAX 60        // never wind past this - beyond it, assume a hand
#define HOLD_MS 500         // pushing at full boost this long without moving = held

// Tapers the drive duty down as the slider nears its target, so it settles instead
// of overshooting; returns `full` unchanged once outside the taper zone.
int driveSpeed(int gap, int full) {
  // Nothing to taper if the faderLevel already runs at or below the friction floor — and
  // tapering anyway inverted the ramp, so the slowest faderLevel sped UP as it approached.
  if (full <= MIN_DUTY) return full;              // this level has no headroom to taper within
  if (gap >= RAMP_ZONE) return full;              // still outside the taper zone - cruise at full speed
  int duty = MIN_DUTY + (long)(full - MIN_DUTY) * gap / RAMP_ZONE;  // linear ramp: MIN_DUTY at gap=0, full at gap=RAMP_ZONE
  if (duty < MIN_DUTY) duty = MIN_DUTY;           // never drive below the friction floor
  if (duty > full) duty = full;      // a taper must never exceed the cruise speed
  return duty;
}

// ---- Control Parameters ----------------------------------------------------
#define HOMING_SPEED 220               // Speed used to drive to the Off mark
#define FADER_MARGIN 10                // Acceptable position error, ~1% of range
#define FADER_MOVE_TIMEOUT 8000              // Give up on an end rather than push into
                                       // a mechanical stop forever
#define FADER_SAMPLES 9                      // analogRead samples averaged per reading

// ---- Hand detection --------------------------------------------------------
// While driving, the gap to target should shrink steadily; if it grows instead,
// something is pushing back - a hand. Widened from 150/12: a hard reversal at
// MEDIUM/HIGH speed was overshooting past REVERSE_MARGIN before GRACE_MS elapsed,
// real mechanical settling misread as a hand. A genuine push still clears this easily.
#define GRACE_MS 350        // ignore direction right after a turn
#define REVERSE_MARGIN 30   // counts moved against the drive = a hand
#define SETTLE_MOVE 8      // Counts of change that still count as "hand moving"
#define SETTLE_MS 400      // Hand still this long = you let go

// ---- Calibration -----------------------------------------------------------
#define CAL_STILL_DELTA 6      // change that still counts as moving
#define CAL_STILL_MS 400       // still this long AFTER travelling = at the stop
#define CAL_NO_MOVE_MS 2000    // never moved at all = we started against it
#define CAL_TIMEOUT 12000      // ceiling per direction

// Function Declarations
void motorCoast();               // both driver inputs LOW - free to move by hand
void motorStop();                // both driver inputs HIGH - brakes, holds position
void motorForward(int speed);    // drives toward SLIDER_MAX at the given PWM duty
void motorBackward(int speed);   // drives toward SLIDER_MIN at the given PWM duty

// ---- State -----------------------------------------------------------------
int faderLevel = 0;       // 0 Off, 1 Low, 2 Medium, 3 High
int pointA = 0;      // low end of the current swing
int pointB = 0;      // high end
int targetVal = 0;   // whichever end we are heading for right now

enum FaderMode { FD_HOMING, FD_SWINGING, FD_GRABBED, FD_IDLE, FD_PACED };  // FD_HOMING: driving to centre at boot
                                                                   // FD_SWINGING: motor flat-out toward pointA/pointB
                                                                   // FD_GRABBED: motor off, a hand is moving the slider
                                                                   // FD_IDLE: motor off, resting at OFF
                                                                   // FD_PACED: tracking a smoothly ping-ponging target,
                                                                   //   timed to match a chosen seconds-per-swing
FaderMode faderMode = FD_HOMING;   // start every power-up by driving to the centre mark

// ---- Paced swing (F:<level>) - speed-matched to the arms' own timing ------------
// The motor crosses its short travel in well under a second at any duty that
// overcomes friction, so matching the arms' 35/14/10.5s means pacing the TARGET
// itself - moving it smoothly on the arms' own schedule - and only nudging the motor
// to chase that slow point, rather than driving flat-out (what FD_SWINGING still does
// for a real hand-release, where matching the arms doesn't apply).
const float PACE_SECONDS[4] = { 0.0, 35.0, 14.0, 10.5 };   // OFF unused, LOW/MED/HIGH - same table as the arms
#define PACE_DEADBAND 15    // how close to the live target counts as "there" - coast inside it
float pacePhase = 0;                // radians, this swing's own clock, wraps via fmod
unsigned long paceLastTick = 0;     // millis() timestamp of the last FD_PACED update, for dt

int bestGap = 0;     // Closest we have got to the target on this leg
int legStartGap = 0; // Gap when this leg began
bool grabArmed = false;  // Grab detection only counts once the leg is making progress
int prevSlider = 0;      // Last reading, for working out which way it is moving
int against = 0;         // Counts moved against the drive on this leg
unsigned long holdTime = 0;   // when full-boost driving against something began
int stallRef = 0;        // Reading the stall timer measures progress against
unsigned long stallTime = 0;  // when the stall timer was last reset
int boost = 0;           // Extra duty added to break friction when stuck
int boostReported = 0;   // Highest boost we have mentioned, so it is logged once
int settleRef = 0;   // Reading the settle timer is measured against
int handMin = 0;     // How far the hand travelled, while it was on the fader
int handMax = 0;     // How far the hand travelled, while it was on the fader
int idleRef = 0;     // Reading we watch for a hand while resting

int lastPrintedFaderMode  = -1;   // so the fader status line only prints on a real change
int lastPrintedFaderLevel = -1;
unsigned long moveStart = 0;        // millis() timestamp this HOMING/SWINGING leg began, for timeouts
unsigned long settleTime = 0;       // millis() timestamp the hand last moved, for the let-go check
unsigned long noiseBlankUntil = 0;  // readSlider() distrusts implausible jumps until this millis() timestamp

// Human-readable name of the fader's current mode, for the status line.
const char* faderPhaseName() {
  if (faderMode == FD_HOMING)  return "FD_HOMING  ";
  if (faderMode == FD_IDLE)    return "RESTING ";
  if (faderMode == FD_GRABBED) return "HAND    ";
  if (faderMode == FD_PACED)   return "FD_PACED";
  return "FD_SWINGING";
}

// Motor noise reads as movement, and the wiper drops out intermittently - both
// handled below.
int readSlider() {
  // Median, not mean: a mean is dragged by one bad sample, a median across an odd
  // count discards it outright as long as fewer than half the samples are bad.
  int v[FADER_SAMPLES];                                          // sample buffer
  for (int i = 0; i < FADER_SAMPLES; i++) v[i] = analogRead(SLIDER_PIN);   // take the raw samples
  for (int i = 1; i < FADER_SAMPLES; i++) {        // insertion sort, tiny N
    int k = v[i], j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }   // shift larger values up
    v[j + 1] = k;                                          // drop k into its sorted slot
  }
  int median = v[FADER_SAMPLES / 2];   // middle of the sorted array

  // Reject the rails (0/1023 - impossible for a live wiper, so it means the input
  // floated) and keep the last good value instead. Not a window around the declared
  // travel, which would make it impossible to calibrate a wider range.
  static int lastGood = -1;                          // persists between calls
  if (median <= RAIL_LOW || median >= RAIL_HIGH) {   // this reading is a floated rail, not real
    if (lastGood >= 0) return lastGood;              // substitute the last trustworthy reading
  } else if (lastGood >= 0 && millis() < noiseBlankUntil
             && abs(median - lastGood) > SLIDER_JUMP_MAX) {   // implausible jump, still inside the post-reversal window
    return lastGood;                                 // distrust it - this is the motor's own transient, not real travel
  } else {
    lastGood = median;                               // remember this reading as trustworthy
  }
  return median;
}

// Which faderLevel a reading falls in, from its distance either side of centre.
// `current` is the faderLevel already showing; a reading has to clear the boundary by
// BAND_HYSTERESIS to move off it, so noise on a mark cannot flicker the faderLevel.
int levelFor(int reading, int current) {
  int dist = abs(reading - SLIDER_CENTER);              // distance from centre, direction-independent
  const int edge[3] = { DIST_OFF_LOW, DIST_LOW_MED, DIST_MED_HIGH };  // the three boundaries, indexed by level-1

  int lv = current;                                                     // start from where we already are
  while (lv < 3 && dist > edge[lv] + BAND_HYSTERESIS) lv++;            // climb a level at a time while clearly past the next edge
  while (lv > 0 && dist < edge[lv - 1] - BAND_HYSTERESIS) lv--;        // drop a level at a time while clearly short of the last edge
  return lv;
}

// ---- NeoPixel --------------------------------------------------------------------
Adafruit_NeoPixel strip(STRIP_COUNT, STRIP_PIN, NEO_GRB + NEO_KHZ800);   // the ring driver object

// Colour per faderLevel. The lit arc grows outward AND heats up as the faderLevel rises.
const uint32_t STRIP_COLOUR[4] = {
  0x000000,   // off
  0xFFC400,   // low    - yellow
  0xFF6A00,   // medium - orange
  0xFF1FA0,   // high   - magenta
};

int shownLevel = -1;   // so the strip is only rewritten when it actually changes

// Redraws the ring only when the level has actually changed, growing the lit arc
// outward from its middle so it never blinks or flickers between calls.
void showLevel(int lv) {
  if (lv == shownLevel) return;    // nothing changed - leave the strip alone
  shownLevel = lv;                 // remember what is now showing

  uint32_t c = STRIP_COLOUR[lv];   // colour for this level
  uint8_t lit = LED_LIT[lv];       // how many of the six pixels should be on
  uint8_t first = (LED_USED - lit) / 2;    // grow outward from the middle of the six

  strip.clear();                            // the other eighteen stay dark
  for (uint8_t k = 0; k < LED_USED; k++) {
    if (k >= first && k < first + lit) strip.setPixelColor(LED_SLOT[k], c);   // light this one of the six
  }
  strip.show();   // push the buffer to the physical ring
}

// Returns 4 if the name is not one of OFF/LOW/MEDIUM/HIGH.
uint8_t parseLevel(const char *n) {
  for (uint8_t i = 0; i < 4; i++) if (eq(n, LEVEL_NAME[i])) return i;   // match against OFF/LOW/MEDIUM/HIGH
  if (eq(n, "MED")) return 2;   // short form, same as the arm states accept
  return 4;                     // not a recognised name
}

// A raw slider position squarely inside a level's band, on the SLIDER_MAX side - used
// by the F:<level> command to simulate a hand leaving the slider there and letting go.
int levelPosition(uint8_t level) {
  if (level == 0) return SLIDER_CENTER;
  const int lo[4] = { 0, DIST_OFF_LOW,  DIST_LOW_MED,  DIST_MED_HIGH };
  const int hi[4] = { 0, DIST_LOW_MED,  DIST_MED_HIGH, HALF_TRAVEL   };
  return SLIDER_CENTER + (lo[level] + hi[level]) / 2;   // middle of that level's band
}

// Starts a paced swing for one level (OFF excluded, see F:): pointA/pointB are set
// like a hand-release swing, but the motor tracks a smoothly ping-ponging target on
// PACE_SECONDS[level]'s own clock instead of driving flat-out to a fixed end.
void startPaced(uint8_t level) {
  int pos = levelPosition(level);
  pointA = constrain(pos, SLIDER_MIN, SLIDER_MAX);
  pointB = constrain(SLIDER_MIN + SLIDER_MAX - pointA, SLIDER_MIN, SLIDER_MAX);
  faderLevel = level;
  pacePhase = 0;
  paceLastTick = millis();
  motorCoast();
  faderMode = FD_PACED;
  Serial.print(F("OK:paced ")); Serial.print(LEVEL_NAME[level]);
  Serial.print(F("  ")); Serial.print(pointA); Serial.print(F(" <-> ")); Serial.print(pointB);
  Serial.print(F("  over ")); Serial.print(PACE_SECONDS[level], 1); Serial.println(F("s"));
}

// Capture: the slider has been left somewhere, so mirror that position and swing
// between the two. The mirror of x in [min, max] is (min + max - x).
void detectFrom(int x) {
  // Mimic the movement, not just the destination: the hand's own two extremes become
  // the swing's ends, so a small gesture near the top gives a small swing near the
  // top. Where you left it also picks the faderLevel off the measured scale.
  int span = handMax - handMin;                                              // how far the hand actually travelled
  pointA = constrain(x, SLIDER_MIN, SLIDER_MAX);                             // where the hand left it, clamped to travel
  pointB = constrain(SLIDER_MIN + SLIDER_MAX - pointA, SLIDER_MIN, SLIDER_MAX); // its mirror about the centre

  // Only re-faderLevel on a real gesture. A span of a few counts is the swing stalling and
  // being mistaken for a hand, not you choosing something new — keep the faderLevel you set.
  if (span >= MIN_GESTURE) {
    faderLevel = levelFor(pointA, faderLevel);   // gesture was real - update the level from the resting point
  } else {
    Serial.print("  (span ");
    Serial.print(span);
    Serial.println(" — too small to be a gesture, keeping the faderLevel)");
  }

  Serial.print("DETECTED value ");
  Serial.print(pointA);
  Serial.print("  (moved ");
  Serial.print(handMin);
  Serial.print("-");
  Serial.print(handMax);
  Serial.print(", span ");
  Serial.print(span);
  Serial.print(")  ->  MODE ");
  Serial.print(LEVEL_NAME[faderLevel]);
  Serial.print("  swinging ");
  Serial.print(pointA);
  Serial.print(" <-> ");
  Serial.print(pointB);
  Serial.print("  at speed ");
  Serial.println(SWING_SPEED[faderLevel]);

  if (faderLevel == 0) {
    // Off just rests where you left it. Centring happens once, at power-up, and
    // never again - dragging it back to the middle every time would fight you.
    Serial.println("  (OFF - resting here)");
    motorCoast();            // let go of the slider completely
    idleRef = pointA;        // watch for a hand from here
    targetVal = pointA;      // nothing to drive toward, but kept in sync for the status line
    faderMode = FD_IDLE;
    return;
  }

  if (abs(pointB - pointA) < MIN_SWING) {
    // x sat near the middle, so x and its mirror nearly coincide. Twitching across
    // a few counts reads as a fault, so rest here and wait for a hand to give us
    // something to work with.
    Serial.println("  (too close to the centre to swing - resting, move the slider)");
    motorCoast();
    idleRef = pointA;
    targetVal = pointA;
    faderMode = FD_IDLE;
    return;
  }

  targetVal = (abs(x - pointA) > abs(x - pointB)) ? pointA : pointB;  // head for the FARTHER end first
  bestGap = abs(targetVal - x);        // starting distance to that end
  legStartGap = bestGap;               // remembered for reference at the start of this leg
  grabArmed = false;                   // this leg has not yet proven it is making progress
  boost = 0;                           // no extra duty yet
  stallRef = -999;                     // force the stall timer to reset on the first check
  stallTime = millis();
  holdTime = millis();
  against = 0;                         // no reverse movement counted yet
  prevSlider = x;                      // baseline for direction tracking
  moveStart = millis();                // timeout clock for this leg starts now
  noiseBlankUntil = millis() + SLIDER_BLANK_MS;   // ride out the motor's own turn-on transient
  faderMode = FD_SWINGING;
}

// Flip to the other end of the swing and restart this leg
void swapTarget(int sliderVal) {
  Serial.print(F("swap at ")); Serial.print(sliderVal);         // timing/position marker, for
  Serial.print(F("  t=")); Serial.println(millis());            // measuring the real swing period
  noiseBlankUntil = millis() + SLIDER_BLANK_MS;   // ride out the motor's own reversal transient
  targetVal = (targetVal == pointA) ? pointB : pointA;   // switch to whichever end we were not driving toward
  bestGap = abs(targetVal - sliderVal);                  // distance to the new target from here
  legStartGap = bestGap;
  grabArmed = false;      // fresh leg - progress has to be proven again
  boost = 0;               // fresh leg - no boost carried over
  stallRef = -999;          // force the stall timer to reset on the first check
  stallTime = millis();
  holdTime = millis();
  against = 0;              // fresh leg - no reverse movement counted yet
  prevSlider = sliderVal;   // baseline for direction tracking on the new leg
  moveStart = millis();     // timeout clock restarts for the new leg
}

// Every way into FD_GRABBED goes through here. Splitting it across the call sites is
// what left the idle path with a stale captureStart — so the three second window had
// already expired before the gesture began, and it acted on the first reading.
void beginCapture(int sliderVal) {
  faderMode = FD_GRABBED;                 // motor stays off until the hand lets go
  handMin = handMax = sliderVal;          // gesture span starts at zero, from here
  settleRef = sliderVal;                  // baseline the "has it stopped moving" check watches
  settleTime = millis();                  // let-go timer starts now
}

// A hand has been felt while the motor was driving: release it and start capturing.
void handDetected(int sliderVal) {
  Serial.println("HAND DETECTED - motor released, set it where you like");
  motorCoast();               // let go immediately so the hand is not fighting the motor
  beginCapture(sliderVal);    // start watching the gesture from here
}

// Drive one way until the reading stops changing, and report where it stopped.
// Stillness alone isn't enough at the start (motor hasn't spun up yet), so it only
// counts once it has actually travelled.
int findStop(int dir) {
  unsigned long start = millis(), lastMove = millis();   // overall timeout clock, and time of last movement
  int ref = readSlider();                                // baseline reading to compare progress against
  bool moved = false;                                    // has it travelled at all yet this call?

  while (millis() - start < CAL_TIMEOUT) {                          // give up after CAL_TIMEOUT regardless
    if (dir > 0) motorForward(255); else motorBackward(255);        // drive flat-out toward the requested stop
    int v = readSlider();
    if (abs(v - ref) > CAL_STILL_DELTA) { ref = v; lastMove = millis(); moved = true; }   // still travelling - reset the still-timer
    else if (moved && millis() - lastMove >= CAL_STILL_MS) break;    // was moving, now still long enough - arrived
    else if (!moved && millis() - start >= CAL_NO_MOVE_MS) break;    // never moved at all - started against the stop
  }
  motorCoast();              // release the motor once the stop is found
  delayMicroseconds(1);      // let the pin settle before the final read
  return readSlider();
}

// Drives to both mechanical ends in turn and prints the measured travel, centre and
// resulting band edges, ready to paste into the #defines above.
void calibrate() {
  Serial.println();
  Serial.println("=== CALIBRATING - hands off ===");
  int lo = findStop(-1);        // drive toward SLIDER_MIN and see where it actually stops
  int hi = findStop(+1);        // drive toward SLIDER_MAX and see where it actually stops
  if (lo > hi) { int t = lo; lo = hi; hi = t; }   // wiring can be reversed - always report lo < hi

  Serial.print("  measured travel  "); Serial.print(lo);
  Serial.print(" - "); Serial.println(hi);
  Serial.print("  span             "); Serial.println(hi - lo);
  int c = (lo + hi) / 2;      // measured centre, for reference against SLIDER_CENTER
  Serial.print("  centre           "); Serial.println(c);
  Serial.println("  paste into the sketch:");
  Serial.print("    #define SLIDER_MIN "); Serial.println(lo);
  Serial.print("    #define SLIDER_MAX "); Serial.println(hi);
  Serial.println("  band edges that follow:");
  const int e[3] = { DIST_OFF_LOW, DIST_LOW_MED, DIST_MED_HIGH };   // current boundary distances, for the printout
  const char *n[4] = { "OFF", "LOW", "MEDIUM", "HIGH" };
  for (int i = 0; i < 4; i++) {
    int dLo = (i == 0) ? 0 : e[i - 1];                     // inner edge of this level's band
    int dHi = (i == 3) ? (hi - c) : e[i];                  // outer edge of this level's band
    Serial.print("    "); Serial.print(n[i]); Serial.print("\t");
    if (i == 0) { Serial.print(c - dHi); Serial.print(" - "); Serial.println(c + dHi); }   // OFF is one band either side of centre
    else {
      Serial.print(c - dHi); Serial.print(" - "); Serial.print(c - dLo);    // this level's band on the low side
      Serial.print("   and   "); Serial.print(c + dLo);
      Serial.print(" - "); Serial.println(c + dHi);                        // and its mirror on the high side
    }
  }
  Serial.println("===============================");
  Serial.println();
  moveStart = millis();     // timeout clock for the homing leg that follows
  faderMode = FD_HOMING;    // return to the centre once calibration is done
}


// Runs once, alongside the arms' own setup() below: pin modes, motor coast, the
// NeoPixel strip and the initial FD_HOMING drive to centre.
void faderSetup() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  motorCoast();          // start with the slider free, not braked or driving

  strip.begin();                       // initialise the NeoPixel strip
  strip.setBrightness(STRIP_BRIGHTNESS);
  strip.clear();                       // all pixels off until a level is set
  strip.show();

  Serial.print(F("fader: travel ")); Serial.print(SLIDER_MIN);
  Serial.print(F(" - "));            Serial.print(SLIDER_MAX);
  Serial.print(F(", centre "));       Serial.println(SLIDER_CENTER);
  moveStart = millis();      // timeout clock for the initial homing leg
  faderMode = FD_HOMING;     // drive to the centre before doing anything else
}

// Runs every loop() pass: the fader's entire state machine, independent of
// whatever the arms are doing.
void faderUpdate() {
  int sliderVal = readSlider();     // one filtered, noise-rejected reading for this pass

  // The strip shows the faderLevel you SET and holds it - not the live reading, which
  // sweeps across every band mid-swing and would flicker the colour continuously.
  showLevel(faderLevel < 0 ? 0 : faderLevel);

  // Print status only on a real change, not on a timer (the 10Hz S: telemetry below
  // is separate and untouched - the web page depends on its steady beat).
  if (faderMode != lastPrintedFaderMode || faderLevel != lastPrintedFaderLevel) {
    lastPrintedFaderMode = faderMode;
    lastPrintedFaderLevel = faderLevel;

    unsigned long ms = millis();
    Serial.print("t=");
    Serial.print(ms / 1000);
    Serial.print(".");
    unsigned long frac = ms % 1000;
    if (frac < 100) Serial.print("0");     // pad the fractional seconds to 3 digits
    if (frac < 10)  Serial.print("0");
    Serial.print(frac);
    Serial.print("s  ");

    Serial.print(faderPhaseName());
    Serial.print("  slider: ");
    Serial.print(sliderVal);
    Serial.print("  faderLevel: ");
    Serial.print(LEVEL_NAME[faderLevel]);
    Serial.print("  target: ");
    if (faderMode == FD_HOMING) Serial.println(SLIDER_CENTER);
    else if (faderMode == FD_SWINGING) Serial.println(targetVal);
    else Serial.println("-");
  }

  switch (faderMode) {

    // ---- Drive to the Off mark on power-up ----
    case FD_HOMING: {
      bool timedOut = (millis() - moveStart >= FADER_MOVE_TIMEOUT);   // give up rather than grind forever

      if (abs(SLIDER_CENTER - sliderVal) <= FADER_MARGIN || timedOut) {   // close enough, or out of time
        motorStop();     // brake first to stop cleanly at the mark
        if (timedOut) {
          Serial.print("FD_HOMING gave up at ");
          Serial.println(sliderVal);
        } else {
          Serial.print("AT OFF MARK ");
          Serial.println(sliderVal);
        }
        // Settle here rather than calling detectFrom() again: at the centre the faderLevel
        // is OFF, and OFF sends us back to FD_HOMING, which would loop forever.
        motorCoast();                                    // release the slider now that we have arrived
        faderLevel = levelFor(sliderVal, faderLevel);    // set the level from wherever we actually landed
        idleRef = sliderVal;                             // watch for a hand from here
        targetVal = sliderVal;                           // kept in sync for the status line
        faderMode = FD_IDLE;
        Serial.print("  resting at ");
        Serial.print(sliderVal);
        Serial.print(", faderLevel ");
        Serial.println(LEVEL_NAME[faderLevel]);
      } else if (sliderVal > SLIDER_CENTER) {
        motorBackward(HOMING_SPEED);    // currently past centre - drive back toward it
      } else {
        motorForward(HOMING_SPEED);     // currently short of centre - drive up toward it
      }
      break;
    }

    // ---- Resting: motor off, waiting for a hand ----
    case FD_IDLE: {
      motorCoast();     // keep releasing the motor every pass - nothing should be driving here
      if (abs(sliderVal - idleRef) > SETTLE_MOVE) {    // it moved more than noise allows - a hand touched it
        Serial.println("HAND DETECTED - set it where you like");
        beginCapture(sliderVal);
      }
      break;
    }

    // ---- Your hand is on it: motor off, wait for you to finish ----
    case FD_GRABBED: {
      motorCoast();     // keep releasing the motor every pass while the hand is on it

      // Live reading while moving it by hand, 10Hz - read this to find SLIDER_MIN/MAX
      // manually instead of the k auto-calibration.
      if (millis() - lastSliderPrint >= 100) {
        lastSliderPrint = millis();
        Serial.print(F("slider: "));
        Serial.println(sliderVal);
      }

      // Measure the gesture, not just where it ends: the two extremes the hand
      // reached are what the fader will ping-pong between.
      if (sliderVal < handMin) handMin = sliderVal;   // track the lowest point reached
      if (sliderVal > handMax) handMax = sliderVal;   // track the highest point reached
      if (abs(sliderVal - settleRef) > SETTLE_MOVE) {   // still actively moving
        settleRef = sliderVal;
        settleTime = millis();
      } else if (millis() - settleTime >= SETTLE_MS) {   // still for long enough
        // Still for long enough - that is where you wanted it
        detectFrom(sliderVal);
      }
      break;
    }

    // ---- Swinging across the selected faderLevel's range ----
    case FD_SWINGING: {
      int gap = abs(targetVal - sliderVal);            // remaining distance to the current target
      int dir = (targetVal > sliderVal) ? 1 : -1;      // the way we are driving
      int delta = sliderVal - prevSlider;              // the way it actually went
      prevSlider = sliderVal;

      bool pastGrace = (millis() - moveStart >= GRACE_MS);   // past the settle time right after a direction change

      // Moving against the drive is a hand - nothing else pushes back.
      if (delta * dir > 0) {
        against = 0;                    // making progress, forget any wobble
      } else if (delta * dir < 0) {
        against += -delta * dir;                                 // accumulate how far it has moved backward
        if (pastGrace && against > REVERSE_MARGIN) {              // enough sustained pushback to be a hand
          handDetected(sliderVal);
          break;
        }
      }

      // Safety net - if an end cannot be reached, swing back rather than grind
      if (millis() - moveStart >= FADER_MOVE_TIMEOUT) {
        Serial.print("GAVE UP at ");
        Serial.print(sliderVal);
        Serial.print(" chasing ");
        Serial.println(targetVal);
        motorStop();              // brake before reversing direction
        swapTarget(sliderVal);    // try the other end instead
        break;
      }

      if (gap > FADER_MARGIN) {                                  // not yet close enough to call it arrived
        if (abs(sliderVal - stallRef) > STALL_NOISE) {
          // moving again
          stallRef = sliderVal;
          stallTime = millis();
          holdTime = millis();
          if (boost > boostReported) {           // log the winning boost value once, not every pass
            Serial.print("  broke friction at duty +");
            Serial.println(boost);
            boostReported = boost;
          }
          boost = 0;                              // friction broken - no boost needed for now
        } else if (millis() - stallTime >= STALL_MS) {   // no progress for STALL_MS - stuck
          stallTime = millis();
          // Wind up to break friction, but only so far - past BOOST_MAX it's a hand
          // holding it still, not friction, and shoving harder is the wrong answer.
          if (boost < BOOST_MAX) {
            boost += STALL_STEP;      // try a bit more duty next pass
          } else if (pastGrace && millis() - holdTime >= HOLD_MS) {   // at max boost and still stuck, for long enough
            // NOT treated as a hand (that's already caught above, by genuine
            // pushback). Stuck with no pushback is friction or a hard stop - give up
            // on this end and swing back rather than re-level to wherever it stalled.
            Serial.println("  (stuck at max boost, no pushback - giving up on this end)");
            motorStop();
            swapTarget(sliderVal);
            break;
          }
        }
      }

      int duty = driveSpeed(gap, SWING_SPEED[faderLevel]) + boost;   // tapered cruise speed plus any stall boost
      if (duty > 255) duty = 255;                                   // PWM ceiling

      if (sliderVal > targetVal + FADER_MARGIN) {
        motorBackward(duty);      // currently past the target - drive down toward it
      } else if (sliderVal < targetVal - FADER_MARGIN) {
        motorForward(duty);       // currently short of the target - drive up toward it
      } else {
        // Reached this end - turn around immediately, no dwell
        motorStop();
        swapTarget(sliderVal);
      }
      break;
    }

    // ---- Paced: chase a target ping-ponging smoothly on PACE_SECONDS[faderLevel] -
    // same triangle wave as the arms. The deadband makes the motor tap in short
    // bursts rather than drive continuously, which is what makes the average speed
    // slow even though the motor has no low-speed setting short of "don't move".
    case FD_PACED: {
      unsigned long now = millis();
      float dt = (now - paceLastTick) / 1000.0;
      paceLastTick = now;
      float rate = TWO_PI / PACE_SECONDS[faderLevel];
      pacePhase += rate * dt;

      float cyc = fmod(pacePhase, TWO_PI);
      if (cyc < 0) cyc += TWO_PI;
      float tri = (cyc < PI) ? (cyc / PI) * 2.0 - 1.0            // -1 -> +1
                             : 1.0 - ((cyc - PI) / PI) * 2.0;     // +1 -> -1

      int mid  = (pointA + pointB) / 2;
      int half = abs(pointB - pointA) / 2;
      int liveTarget = mid + (int)(tri * half);

      if (sliderVal < liveTarget - PACE_DEADBAND)      motorForward(SWING_SPEED[faderLevel]);
      else if (sliderVal > liveTarget + PACE_DEADBAND) motorBackward(SWING_SPEED[faderLevel]);
      else                                              motorCoast();   // close enough - let it sit

      if (millis() - lastSliderPrint >= 100) {   // 10Hz, matches the FD_GRABBED print rate
        lastSliderPrint = millis();
        Serial.print(F("paced slider: ")); Serial.print(sliderVal);
        Serial.print(F("  target: "));     Serial.println(liveTarget);
      }
      break;
    }
  }
}

// Motor Control Functions
//
// HW-354 has no enable pin: direction AND speed both come from IN1/IN2.
//   IN1   IN2   result
//   LOW   LOW   coast  - free to move by hand
//   PWM   LOW   forward at duty
//   LOW   PWM   backward at duty
//   HIGH  HIGH  brake  - resists movement, holds position
//
// Always analogWrite(), never digitalWrite() - on SAMD21 a digitalWrite() after an
// analogWrite() on the same pin is silently ignored until pinMode() runs again.

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
// ---------------------------------------------------------------- serial

// Prints one telemetry line: state, running flag and current angle for every arm,
// semicolon-separated. Read by the web page at 10Hz.
void report() {
  Serial.print(F("S:"));
  for (uint8_t i = 0; i < ARM_COUNT; i++) {
    if (i) Serial.print(';');                    // separator between arms, not before the first
    Serial.print(STATE_NAME[armState[i]]);
    Serial.print(',');
    Serial.print(armRunning[i] ? 1 : 0);
    Serial.print(',');
    Serial.print((int)(armAngle[i] + 0.5));       // rounded to the nearest degree
  }
  Serial.println();
}

// Prints one line per arm, only when it starts, stops or changes level (change-
// triggered, not timed) - meant to be read by eye, e.g. "D7 -> HIGH, 101".
void printArmChanges() {
  for (uint8_t i = 0; i < ARM_COUNT; i++) {
    if (armRunning[i] == lastArmRunning[i] && armState[i] == lastArmState[i]) continue;   // nothing changed
    Serial.print(F("D")); Serial.print(SERVO_PIN[i]);
    if (armRunning[i]) { Serial.print(F(" -> ")); Serial.print(STATE_NAME[armState[i]]); Serial.print(F(", ")); }
    else                 Serial.print(F(" stopped at "));
    Serial.print((int)(armAngle[i] + 0.5));
    Serial.println(F(" deg"));
    lastArmRunning[i] = armRunning[i];
    lastArmState[i]   = armState[i];
  }
}

// Single characters typed into the Serial Monitor. Returns true if handled.
bool handleKey(char c) {
  switch (c) {
    case '1': takeControl(); startAll(ST_LOW);                  // all arms to LOW
              Serial.println(F("OK:key all LOW")); return true;
    case '2': takeControl(); startAll(ST_MED);                  // all arms to MEDIUM
              Serial.println(F("OK:key all MED")); return true;
    case '3': takeControl(); startAll(ST_HIGH);                 // all arms to HIGH
              Serial.println(F("OK:key all HIGH")); return true;
    case '0': takeControl(); stopAll();                         // stop every arm
              Serial.println(F("OK:key stop all")); return true;
    case 'a': autoCycle = true; holdCenter = false; lastSwitch = 0;   // (re)start the standalone demo cycle
              Serial.println(F("OK:key demo cycle resumed")); return true;
    case 'c': takeControl(); holdCenter = true;                 // park every arm at CENTER_ANGLE
              for (uint8_t i = 0; i < ARM_COUNT; i++) { armRunning[i] = true; attachArm(i, true); }
              Serial.println(F("OK:key holding CENTER_ANGLE")); return true;
    case 'k': calibrate(); return true;                         // drive to both fader stops and measure travel
    default:  return false;    // not a recognised single-character command
  }
}

// Parses and executes one complete line received over serial (see the header
// comment for the full command reference).
void handleLine(char *s) {
  if (s[1] == '\0' && handleKey(s[0])) return;   // a bare single character

  char kind = toupper(s[0]);   // the command letter, case-insensitive

  if (kind == '?') { report(); return; }   // one-off telemetry line on demand

  // handshake — proof of life for the web page, and a visible blink on the board
  if (kind == 'H') {
    playPattern(PATTERN_HELLO);
    Serial.print(F("OK:HELLO social-battery ARMS="));
    Serial.println(ARM_COUNT);
    return;
  }

  // X -> stop everything,  X:<arm> -> stop one
  if (kind == 'X') {
    takeControl();
    if (s[1] == ':') {
      int idx = atoi(s + 2);                                              // arm index after "X:"
      if (idx < 0 || idx >= ARM_COUNT) { Serial.println(F("ERR:arm out of range")); return; }
      stopArm(idx);
      Serial.print(F("OK:X ")); Serial.println(idx);
    } else {
      stopAll();
      Serial.println(F("OK:X all"));
    }
    return;
  }

  // T:<state> -> every arm,  T:<arm>:<state> -> one arm
  if (kind == 'T' && s[1] == ':') {
    takeControl();
    char *rest = s + 2;                    // everything after "T:"
    char *colon = strchr(rest, ':');       // a second colon means the per-arm form

    if (colon) {                       // per-arm form
      *colon = '\0';                       // split rest into the arm index...
      int idx = atoi(rest);
      uint8_t st = parseState(colon + 1);  // ...and the state name
      if (idx < 0 || idx >= ARM_COUNT) { Serial.println(F("ERR:arm out of range")); return; }
      // The fader arm is commandable now: set the state AND drive the slider to match,
      // so the physical control never disagrees with what the software thinks red is.
      if (st >= STATE_COUNT)           { Serial.println(F("ERR:unknown state")); return; }
      startArm(idx, (State)st);
      Serial.print(F("OK:T ")); Serial.print(idx);
      Serial.print(' ');        Serial.println(STATE_NAME[st]);
      return;
    }

    uint8_t st = parseState(rest);     // all-arms form
    if (st >= STATE_COUNT) { Serial.println(F("ERR:unknown state")); return; }
    startAll((State)st);
    Serial.print(F("OK:T all ")); Serial.println(STATE_NAME[st]);
    return;
  }

  // C:<arm>:<angle> — hold one arm at an angle, so the resting vertical can be found
  // by eye rather than guessed. It stays there until you stop it or set a state.
  if (kind == 'C' && s[1] == ':') {
    char *p1 = strchr(s + 2, ':');                                        // separates arm index from angle
    if (!p1) { Serial.println(F("ERR:C needs arm and angle")); return; }
    *p1 = '\0';
    int idx = atoi(s + 2);
    int ang = atoi(p1 + 1);
    if (idx < 0 || idx >= ARM_COUNT) { Serial.println(F("ERR:arm out of range")); return; }
    ang = (int)clampf(ang, ANGLE_MIN, ANGLE_MAX);    // never command past what the linkage can reach
    takeControl();
    armRunning[idx] = false;      // this arm is being held, not swinging
    armHeld[idx] = true;          // freeze it - updateArms() now skips this arm entirely
    restSince[idx] = 0;
    attachArm(idx, true);         // make sure pulses reach the servo
    armAngle[idx] = ang;
    servos[idx].write(ang);
    Serial.print(F("OK:C ")); Serial.print(idx);
    Serial.print(F(" holding ")); Serial.println(ang);
    return;
  }

  // A:<arm>:<posScale>:<negScale> — trim one side of an arm's swing to match the
  // other, live, no re-upload. e.g. A:0:1.0:0.8 pulls arm 0's negative side in by
  // 20%. A alone prints the live table, ready to paste back into ampScalePos/Neg.
  if (kind == 'A') {
    if (s[1] == '\0') {
      for (uint8_t i = 0; i < ARM_COUNT; i++) {
        Serial.print(F("A:")); Serial.print(i);
        Serial.print(F(" pos=")); Serial.print(ampScalePos[i], 2);
        Serial.print(F(" neg=")); Serial.println(ampScaleNeg[i], 2);
      }
      return;
    }
    if (s[1] != ':') { Serial.println(F("ERR:use A or A:<arm>:<posScale>:<negScale>")); return; }

    char *p1 = strchr(s + 2, ':');                                          // separates arm from posScale
    if (!p1) { Serial.println(F("ERR:A needs arm, posScale and negScale")); return; }
    *p1 = '\0';
    char *p2 = strchr(p1 + 1, ':');                                         // separates posScale from negScale
    if (!p2) { Serial.println(F("ERR:A needs arm, posScale and negScale")); return; }
    *p2 = '\0';

    int idx = atoi(s + 2);
    if (idx < 0 || idx >= ARM_COUNT) { Serial.println(F("ERR:arm out of range")); return; }

    float pos = atof(p1 + 1);
    float neg = atof(p2 + 1);
    // Clamp rather than reject: a typo should not fling an arm into its end stop.
    if (pos < 0.2) pos = 0.2;
    if (pos > 1.5) pos = 1.5;
    if (neg < 0.2) neg = 0.2;
    if (neg > 1.5) neg = 1.5;

    ampScalePos[idx] = pos;
    ampScaleNeg[idx] = neg;

    Serial.print(F("OK:A ")); Serial.print(idx);
    Serial.print(F(" pos=")); Serial.print(pos, 2);
    Serial.print(F(" neg=")); Serial.println(neg, 2);
    return;
  }

  // V:<state>:<amp>:<rate> — retune a state live, no re-upload. e.g. V:MED:24:0.7
  // V alone prints the current table, ready to paste back into the sketch.
  if (kind == 'V') {
    if (s[1] == '\0') {
      for (uint8_t i = 0; i < STATE_COUNT; i++) {   // print the live amplitude/speed table
        Serial.print(F("V:")); Serial.print(STATE_NAME[i]);
        Serial.print(F(" amp=")); Serial.print(stateAmp[i], 1);
        Serial.print(F(" rate=")); Serial.println(stateRate[i], 2);
      }
      return;
    }
    if (s[1] != ':') { Serial.println(F("ERR:use V or V:<state>:<amp>:<rate>")); return; }

    char *p1 = strchr(s + 2, ':');                                          // separates state from amp
    if (!p1) { Serial.println(F("ERR:V needs state, amp and rate")); return; }
    *p1 = '\0';
    char *p2 = strchr(p1 + 1, ':');                                         // separates amp from rate
    if (!p2) { Serial.println(F("ERR:V needs state, amp and rate")); return; }
    *p2 = '\0';

    uint8_t st = parseState(s + 2);
    if (st >= STATE_COUNT) { Serial.println(F("ERR:unknown state")); return; }

    float amp  = atof(p1 + 1);
    float rate = atof(p2 + 1);
    // Clamp rather than reject: a typo should not fling an arm into its end stop.
    if (amp  < 0)    amp  = 0;
    if (amp  > 60)   amp  = 60;
    if (rate < 0.01) rate = 0.01;
    if (rate > 6.0)  rate = 6.0;

    stateAmp[st]  = amp;
    stateRate[st] = rate;

    // Arms already running in this state pick the change up through the usual blend,
    // so you hear and see the difference immediately rather than after a restart.
    Serial.print(F("OK:V ")); Serial.print(STATE_NAME[st]);
    Serial.print(F(" amp=")); Serial.print(amp, 1);
    Serial.print(F(" rate=")); Serial.println(rate, 2);
    return;
  }

  // F:<level> — paced swing matching the arms' timing (OFF/LOW/MEDIUM/HIGH or MED).
  // F:<raw value> (e.g. F:700) instead drives flat-out to an exact position - pacing
  // needs a named level to look up a duration for.
  if (kind == 'F' && s[1] == ':') {
    char *arg = s + 2;
    uint8_t lvl = parseLevel(arg);

    if (lvl == 0) {                          // OFF - just rest where it is, nothing to pace
      motorCoast();
      faderLevel = 0;
      idleRef = readSlider();
      targetVal = idleRef;
      faderMode = FD_IDLE;
      Serial.println(F("OK:F OFF"));
      return;
    }
    if (lvl < 4) {                           // named level - paced swing
      startPaced(lvl);
      return;
    }

    // Not a level name - treat it as a raw target position instead, same as before.
    int pos = constrain(atoi(arg), SLIDER_MIN, SLIDER_MAX);
    handMin = pos - MIN_GESTURE;
    handMax = pos + MIN_GESTURE;
    motorCoast();
    Serial.print(F("OK:F ")); Serial.println(pos);
    detectFrom(pos);
    return;
  }

  // D:<duty> — drive the fader motor directly, bypassing every bit of logic.
  // Positive drives forward, negative backward, 0 coasts. Purely a wiring/supply
  // probe: if this does nothing, the fault is not in the control code.
  if (kind == 'D' && s[1] == ':') {
    int duty = atoi(s + 2);
    faderMode = FD_IDLE;                 // stop the fader's own state machine fighting us
    if (duty == 999)   motorStop();      // brake: both inputs HIGH, windings shorted
    else if (duty > 0) motorForward(duty > 255 ? 255 : duty);     // clamp to the PWM ceiling
    else if (duty < 0) motorBackward(-duty > 255 ? 255 : -duty);  // clamp to the PWM ceiling
    else               motorCoast();
    int sv = readSlider();
    // Without this, a stale idleRef makes the next FD_IDLE pass read the current
    // position as a hand touch and restart a swing on its own - this is what made
    // D:0 look like it "didn't stick".
    idleRef = sv;
    targetVal = sv;    // keep the status line in sync too
    Serial.print(F("OK:D ")); Serial.print(duty);
    Serial.print(F(" slider=")); Serial.println(sv);
    return;
  }

  // E:<0|1> — attach (1) or detach (0) every arm's servo directly, for re-taping
  // or freeing the linkage by hand.
  if (kind == 'E' && s[1] == ':') {
    for (uint8_t i = 0; i < ARM_COUNT; i++) attachArm(i, s[2] != '0');
    Serial.print(F("OK:E ")); Serial.println(s[2] != '0' ? 1 : 0);
    return;
  }

  Serial.println(F("ERR:unknown command"));
}

// Buffers incoming serial bytes into `line` and hands off a complete line (ended
// by \n or \r) to handleLine().
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) { line[lineLen] = '\0'; handleLine(line); lineLen = 0; }   // terminate and dispatch, then reset
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;    // still room in the buffer - keep collecting
    }
  }
}


// ---------------------------------------------------------------- arduino

void setup() {
  Serial.begin(115200);
  // Deliberately no `while (!Serial)` — that would hang the sketch whenever the board
  // runs without the Serial Monitor open, which is most of the time for a piece.

  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  motorCoast();            // idle free, so the fader can be moved by hand from boot

  for (uint8_t i = 0; i < ARM_COUNT; i++) {
    armAngle[i] = CENTER_ANGLE[i];   // arms start already sitting at their resting vertical
  }

  playPattern(PATTERN_HELLO);   // says "I booted" without blocking the loop

  Serial.print(F("OK:READY social-battery ARMS="));
  Serial.println(SERVOS_ENABLED ? ARM_COUNT : 0);
  if (!SERVOS_ENABLED) Serial.println(F("servo arms are PARKED (SERVOS_ENABLED 0) - fader only"));
  Serial.println(F("keys: 1/2/3 = low/med/high, 0 = stop, a = demo cycle, c = hold centre"));
  faderSetup();   // pins, motor coast, strip init and initial FD_HOMING for the fader half

  lastTick = millis();
  lastSwitch = 0;               // start the demo cycle immediately
}

void loop() {
  readSerial();       // dispatch any complete command line received since the last pass
  updateLed();         // advance whichever LED pattern (or idle indicator) is showing
  faderUpdate();      // the fader runs its own loop, independent of the arms

  unsigned long now = millis();

  // Standalone demo: walk every arm through the states until something takes over.
  if (SERVOS_ENABLED && autoCycle && (lastSwitch == 0 || now - lastSwitch >= DWELL_MS)) {
    if (lastSwitch != 0) cycleState = (cycleState + 1) % STATE_COUNT;   // advance to the next state
    lastSwitch = now;
    startAll((State)cycleState);
    Serial.print(F("demo -> ")); Serial.println(STATE_NAME[cycleState]);
  }

  float dt = (now - lastTick) / 1000.0;   // seconds since the last motion update
  if (dt >= 0.02) {            // 50 Hz motion update
    lastTick = now;
    updateArms(dt);
  }

  // Servo telemetry commented out while reading the slider by hand, so its 10Hz S:
  // line and per-arm change lines do not clutter the Serial Monitor. Uncomment both
  // to bring arm telemetry back.
  // if (now - lastReport >= 100) {   // 10 Hz telemetry
  //   lastReport = now;
  //   report();
  //   // Fader state on its own line, so the S: format stays exactly as it was.
  // }
  //
  // printArmChanges();   // change-triggered, not on a timer - see its own comment above
}
