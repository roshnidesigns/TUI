/*
 * Social Battery — the motorized fader, plus one servo arm (D7), SYNCED demo
 * Tangible User Interface, CIID
 *
 * Based on FINAL-DIFF-SPEED. The servo and the slider now run a single hardcoded
 * sequence TOGETHER, from the moment the board boots, no hand or serial input
 * needed - 3 steps, 15 seconds each:
 *
 *   0. servo LOW,  slider OFF
 *   1. servo LOW,  slider HIGH
 *   2. servo HIGH, slider HIGH  - the instant both reach HIGH, the servo's phase
 *      clock resets so it starts that leg at the same moment the slider starts
 *      its leg - a one-time angle sync, not a continuous one. They keep their own
 *      different speeds after that (servo: exact-clock ARM_STATE_RATE; slider:
 *      duty-tuned SWING_SPEED) - matching pace continuously is a later, separate
 *      file. Then it loops back to step 0.
 *
 * A real hand on the slider, or a manual servo command (low/med/high/stop/go),
 * or 'c', all take over and cancel the sequence for good (until reboot).
 *
 * The fader drives continuously toward the real physical end of the swing (no
 * virtual paced target - like Fader_Mirror, a smooth glide). Hitting ~30/10/6s
 * per level is an approximate, duty-tuned thing (SWING_SPEED), not an exact clock
 * like PACE_SECONDS was.
 *
 * A 100mm motorized fader that reads, captures and mimics a hand-set gesture.
 *
 *   Move it by hand  →  the motor releases so it slides freely, and the position
 *                       is watched while you move.
 *   Let go           →  after 400ms of stillness the position is taken as the
 *                       value. That value picks a level, and the fader starts
 *                       ping-ponging between it and its mirror about the centre.
 *   Grab it again    →  the motor releases the instant it feels you, and the
 *                       whole thing starts over.
 *
 * WHERE THE LEVELS COME FROM
 *
 * The level is how far the slider sits FROM THE CENTRE of travel, in either
 * direction, so the scale is symmetric: push away from the middle either way and
 * the level rises. That symmetry is what makes the mirror work — a position and
 * its reflection are always the same level.
 *
 * Travel is divided into four equal quarters, derived from SLIDER_MIN/MAX, so
 * recalibrating moves every boundary with it and nothing is worked out by hand.
 *
 *   level    distance from centre     what it does
 *   OFF      nearest the middle       rests where you left it, strip dark
 *   LOW      next quarter out         narrow swing, slow
 *   MEDIUM   next quarter out         wider swing
 *   HIGH     the outer quarter        widest swing, fastest
 *
 * The level sets only the SPEED of the mimic. The two ends always come from your
 * gesture, so the movement stays yours.
 *
 * WIRING — Arduino MKR WiFi 1010
 *
 *   fader wiper (green)   →  A1          position feedback
 *   fader pot ends        →  3.3V, GND   NEVER 5V, the analog pins are not
 *                                        5V tolerant
 *   fader motor           →  HW-354 Motor A, OUT1 / OUT2
 *   driver IN1 / IN2      →  D3 / D5
 *   driver VCC / GND      →  EXTERNAL 5V supply, its ground tied to the board's.
 *                            The MKR's own regulator cannot source motor current.
 *   NeoPixel data         →  D1          8-pixel strip
 *
 *   Avoid D8, D9 and D10 — the SPI bus to the onboard NINA WiFi module.
 *
 * SERIAL, 115200 - one command per line, Enter to send
 *
 *   c            calibrate the fader: drive to each mechanical stop and report
 *                the travel, ready to paste into SLIDER_MIN / SLIDER_MAX below.
 *   low          arm swings at the LOW amplitude/rate
 *   med          arm swings at the MED amplitude/rate
 *   high         arm swings at the HIGH amplitude/rate
 *   stop         arm eases back to ARM_CENTER_ANGLE and holds there
 *   go <deg>     arm holds a fixed angle directly, e.g. "go 120" - ignores the
 *                swing entirely, useful for finding/checking ARM_CENTER_ANGLE
 *
 *   Status prints every 200ms: elapsed time, phase, slider reading, level.
 *
 * WHAT TO TUNE FIRST
 *
 *   SLIDER_MIN / SLIDER_MAX   the measured ends of travel. Everything else is
 *                             derived from these, so get them right first.
 *   SWING_SPEED               drives the swing directly now (no virtual paced
 *                             target - see the note above the array). Below
 *                             MIN_DUTY the motor will not break friction at all,
 *                             so there's only a narrow band to slow into. Time it
 *                             on the bench and nudge these to land near 30/10/6s.
 *   MIN_GESTURE               how much hand movement counts as choosing a new
 *                             level, rather than a stalled swing being mistaken
 *                             for a hand.
 *
 * SIX THINGS LEARNED THE HARD WAY — please do not undo these
 *
 *  1. Coast and brake are different, and both are needed. Both driver inputs LOW
 *     lets the fader move freely under your fingers; both HIGH resists. To read a
 *     hand-set position you MUST coast — braking fights the user.
 *  2. analogWrite() everywhere on the motor pins, never digitalWrite(). On SAMD21
 *     analogWrite() re-muxes the pin to a timer, and a later digitalWrite() on it
 *     is silently ignored until pinMode() runs again.
 *  3. Never `while (!Serial)` — it hangs the sketch whenever the board runs
 *     without a computer attached, which for a finished piece is most of the time.
 *  4. Never delay() in the loop. A blocking delay stops the sampling, so the
 *     sketch misses the movement it is meant to detect.
 *  5. Take a MEDIAN of several analogRead()s, not a mean. The motor throws enough
 *     noise onto the analog line to read the rails, and a mean is dragged by a
 *     single bad sample where a median discards it outright.
 *  6. The servo arm below is deliberately minimal next to the two-arm full
 *     sketch it was pulled from: no auto-release when it stops (it just holds
 *     ARM_CENTER_ANGLE, powered - watch it doesn't run hot for long stretches),
 *     no per-side trim, no telemetry. Just amplitude, rate, and three levels.
 *
 * Board: Arduino UNO R4 WiFi
 */

#include <Adafruit_NeoPixel.h>   // drives the NeoPixel strip
#include <Servo.h>               // drives the servo arm, any digital pin via a shared hardware timer

// ---- Pin Definitions -------------------------------------------------------
#define MOTOR_IN1 3    // HW-354 IN1 (Motor A) - direction + speed
#define MOTOR_IN2 5    // HW-354 IN2 (Motor A) - direction + speed
#define SLIDER_PIN A1  // Analog input from slider's feedback potentiometer

// ---- Servo arm (D7 only - the full sketch's second arm, D0, is not used here) ----
#define SERVO_PIN 7
#define ARM_CENTER_ANGLE 93   // resting vertical, measured by hand with C:<arm>:<angle> in the full sketch
#define ARM_ANGLE_MIN 10      // hard clamp - keep inside what the linkage can physically reach
#define ARM_ANGLE_MAX 210

enum ArmState { ARM_LOW = 0, ARM_MED = 1, ARM_HIGH = 2, ARM_STATE_COUNT = 3 };
const char* ARM_STATE_NAME[ARM_STATE_COUNT] = { "LOW", "MED", "HIGH" };

// Amplitude (degrees either side of centre) and rate (radians/second) per level.
// Rate is set from a target period (one full round trip, 2*PI/rate seconds) aimed
// at the same 30 / 10 / 6s targets the slider's SWING_SPEED is now tuned toward
// below (that one's duty-based and approximate; this one's still an exact clock).
// Amplitude is unchanged - this is a speed-only change. Peak swing speed at HIGH
// works out to ~30 deg/s, well under ARM_SLEW_DEG_PER_SEC below, so nothing here
// gets slew-clipped.
//                                    LOW    MED    HIGH
const float ARM_STATE_AMP[ARM_STATE_COUNT]  = { 10.0,   25.0,   45.0 };   // degrees either side of centre - unchanged
const float ARM_STATE_RATE[ARM_STATE_COUNT] = { 0.2094, 0.6283, 1.0472 }; // radians/second - 2*PI / (30, 10, 6)

#define ARM_STATE_BLEND_SEC 1.4      // seconds to ease amplitude across a level change, so it reads as a mood change
#define ARM_SLEW_DEG_PER_SEC 140.0   // max degrees/second on any move - keeps it reading as alive, not snapping

Servo armServo;
bool armRunning = false;               // swinging, vs eased back to centre and stopped
bool armHeld = false;                  // `go <deg>` is in effect - updateArm() skips the swing entirely
ArmState armState = ARM_MED;           // which level the arm is set to
float armAmp = 0;                      // eased toward ARM_STATE_AMP[armState]
float armAngle = ARM_CENTER_ANGLE;     // what the servo is actually holding right now
float armPhase = 0;                    // radians, this arm's own clock (wraps via fmod below)
unsigned long armLastTick = 0;         // millis() timestamp of the last motion update

// Hardcoded SYNC sequence: drives the servo AND the slider together through 3
// steps, 15s each, starting the moment the board boots - no serial/hand input
// needed:
//   0. servo LOW,  slider OFF
//   1. servo LOW,  slider HIGH
//   2. servo HIGH, slider HIGH  - the instant both are at HIGH, the servo's phase
//      clock is reset so it starts this leg exactly as the slider starts its leg.
//      They're NOT locked to the same pace after that (see SWING_SPEED vs
//      ARM_STATE_RATE - deliberately different, kept smooth on each side) - this
//      only syncs the starting angle/position, not the ongoing timing. Matching
//      pace continuously is a later, separate file.
// Then it loops back to step 0. A real hand on the slider, or a manual servo
// command (med/high/stop/go), or 'c', all take over and cancel the sequence.
#define SYNC_STEP_MS 15000   // 15 seconds per step
bool syncAutoCycle = true;
uint8_t syncStep = 0;
unsigned long lastSyncSwitch = 0;   // 0 = hasn't started its first step yet

// Starts (or re-levels) the arm's swing. Starting from stopped snaps the amplitude
// immediately (the slew limiter still walks the angle there); switching levels while
// already running eases the amplitude across instead.
void startArm(ArmState s) {
  if (!armRunning) armAmp = ARM_STATE_AMP[s];   // only snap amplitude when starting from a stop
  armState = s;
  armRunning = true;
  armHeld = false;             // a level command always overrides a `go` hold
  Serial.print("ARM -> ");
  Serial.println(ARM_STATE_NAME[s]);
}

// startSliderSwing() (below) needs the fader's own state, and readSlider() -
// declared much further down the file - so only its prototype goes here.
// Arduino auto-prototypes top-level functions, but not the variables their
// bodies use, so the body itself has to live after those are all in scope.
void startSliderSwing(bool high);

// Drives one step of the hardcoded sync sequence - see the comment above the
// sequence's state variables for what each step does.
void enterSyncStep(uint8_t step) {
  Serial.print("SYNC step ");
  Serial.print(step);
  Serial.print(": ");
  switch (step) {
    case 0:
      startArm(ARM_LOW);
      startSliderSwing(false);
      break;
    case 1:
      startArm(ARM_LOW);
      startSliderSwing(true);
      break;
    case 2:
      armPhase = 0;   // both start this leg together - see the note above
      startArm(ARM_HIGH);
      startSliderSwing(true);
      Serial.println("  (SYNCED - both starting HIGH together)");
      break;
  }
}

// Called every motion tick (50Hz, from loop() below). Advances the phase clock,
// eases the amplitude toward its target, computes this instant's angle from the
// ping-pong triangle wave, slew-limits the move and writes it to the servo.
void updateArm(float dt) {
  if (armHeld) return;   // `go <deg>` is in effect - leave the servo exactly where that put it

  armPhase += ARM_STATE_RATE[armState] * dt;      // advance this level's clock

  float k = dt / ARM_STATE_BLEND_SEC;
  if (k > 1.0) k = 1.0;
  float wantAmp = armRunning ? ARM_STATE_AMP[armState] : 0.0;   // stopping fades the swing out
  armAmp += (wantAmp - armAmp) * k;                              // exponential ease toward wantAmp

  // Triangle wave, not a sine - crosses at a constant rate rather than lingering
  // at the ends.
  float cycle = fmod(armPhase, TWO_PI);
  if (cycle < 0) cycle += TWO_PI;   // fmod can return negative - correct it
  float tri = (cycle < PI) ? (cycle / PI) * 2.0 - 1.0                  // -1 -> +1
                           : 1.0 - ((cycle - PI) / PI) * 2.0;          // +1 -> -1

  float target = armRunning ? (ARM_CENTER_ANGLE + tri * armAmp) : (float)ARM_CENTER_ANGLE;
  target = constrain(target, ARM_ANGLE_MIN, ARM_ANGLE_MAX);   // never command past what the linkage can reach

  float maxStep = ARM_SLEW_DEG_PER_SEC * dt;      // slew limit so the arm never snaps
  float delta = target - armAngle;
  if (delta > maxStep) delta = maxStep;
  if (delta < -maxStep) delta = -maxStep;
  armAngle += delta;

  armServo.write((int)(armAngle + 0.5));   // send the pulse, rounded to the nearest degree
}

// ---- Travel limits, measured on the bench ----------------------------------
#define SLIDER_MIN 300      // low stop, measured by hand with the motor idle
#define SLIDER_MAX 900      // high stop, measured by hand with the motor idle

// How far outside that travel a reading may sit before it is treated as noise. The
// motor throws a lot of electrical rubbish onto the analog line, and the readings it
// produces are not near-misses — they are 0 and 1023, the rails, which the slider
// cannot physically reach. Anything out here is discarded and the last good reading
// stands. This is what was making HIGH glitch: that is when the motor works hardest.
#define RAIL_LOW 200         // at or below this the input has floated, not moved
#define RAIL_HIGH 1000     // at or above this the input has floated, not moved
#define SLIDER_CENTER ((SLIDER_MIN + SLIDER_MAX) / 2)   // 544 - midpoint of travel, the OFF mark

// ---- The levels ------------------------------------------------------------
// The level is how far the slider sits FROM THE CENTRE, in either direction - so
// the scale is symmetric, and pushing away from the middle in either direction
// raises the intensity. That is what makes the mirror swing work: a position and
// its reflection are always the same level.
//
//     336 / 753  ->  208 from centre  ->  HIGH
//     386 / 703  ->  158              ->  MEDIUM
//     436 / 653  ->  108              ->  LOW
//     500        ->   44              ->  OFF
//
// Boundaries sit midway between the measured points.
// Equal quarters. Half the travel is the furthest the slider can sit from centre,
// and that distance divides evenly into the four levels — so each level occupies the
// same width of movement, either side of the middle.
//
// These derive from SLIDER_MIN/MAX rather than being fixed numbers, so recalibrating
// the travel moves the band edges with it and nothing has to be worked out by hand.
#define HALF_TRAVEL   ((SLIDER_MAX - SLIDER_MIN) / 2)   // furthest distance from centre reachable
#define DIST_OFF_LOW  (HALF_TRAVEL / 4)                 // 1st quarter boundary - OFF ends, LOW begins
#define DIST_LOW_MED  (HALF_TRAVEL * 2 / 4)              // 2nd quarter boundary - LOW ends, MEDIUM begins
#define DIST_MED_HIGH (HALF_TRAVEL * 3 / 4)              // 3rd quarter boundary - MEDIUM ends, HIGH begins
                                                          // (the 4th quarter, out to HALF_TRAVEL, is HIGH)

#define BAND_HYSTERESIS 8   // a reading must clear a boundary by this much to change level

// ---- NeoPixel --------------------------------------------------------------
// 8-pixel strip. The lit fill grows OUTWARD from the middle pair as the level
// rises, so it reads as the battery opening up - each level keeps everything the
// one below it lit.
//     off      nothing
//     low      4,5              the middle pair
//     medium   3,4,5,6          widening
//     high     1,2,3,4,5,6,7,8  the whole strip
// Bit 0 is pixel 1, so the masks below read right-to-left.
#define LED_PIN 1            // NeoPixel data line
#define LED_COUNT 8          // the whole strip
#define LED_BRIGHTNESS 50    // 0-255, kept well below max so the strip is not blinding
const uint8_t STRIP_MASK[4] = { 0b00000000, 0b00011000, 0b00111100, 0b11111111 };   // which pixels light, indexed by level

// No virtual paced target here - the slider just drives continuously toward the
// real physical end at this duty, same as Fader_Mirror, which is what keeps the
// motion a smooth glide instead of PACED's stop-and-catch-up rhythm. That means
// there's no formula linking duty to seconds-per-swing (unlike PACE_SECONDS) -
// these values are tuned by feel and timed on the bench, not calculated.
//
// The usable range is narrower than it looks: below MIN_DUTY the motor won't
// reliably move at all, but sitting right AT that floor isn't safe either - real
// friction varies along the track, so it constantly triggers the stall/boost
// recovery below (stall, wind up torque, break free with a jerk, repeat), which
// reads as jamming, not slow. These sit just above the floor for consistent
// motion; still aimed at roughly 30/10/6s per level, not yet confirmed on the bench.
//                            Off  Low  Medium  High
const int SWING_SPEED[4] = {   0,  222,   226,   230 };            // PWM duty per level, 0-255
const char* LEVEL_NAME[4] = { "OFF", "LOW", "MEDIUM", "HIGH" };     // for serial printouts only

// Below this the two ends are too close together to be worth swinging between
#define MIN_SWING 60

// A level change needs evidence that a hand actually moved the fader. Without this,
// any re-capture re-levels — and a stalled swing near an end triggers exactly that,
// re-capturing at whatever extreme it stalled at and promoting the level to HIGH.
// Set MEDIUM, watch it stall at the top, and it silently becomes HIGH.
#define MIN_GESTURE 25

// Overshoot control. At full duty the fader sails past the target, which then reads
// as the gap growing. Ease off over the last stretch instead of arriving flat out.
// The floor is the lowest duty that still reliably breaks friction - not measured,
// so it is set conservatively high.
#define RAMP_ZONE 60         // only taper close to the target, not half the leg
#define MIN_DUTY 215        // 154 did not move it at all; 190 still crawled

// Friction varies along the track, and lowering SWING_SPEED toward MIN_DUTY makes
// it more likely to actually stall. If the slider stops making progress while being
// driven, wind the duty up until it moves again, and report the value that worked.
#define STALL_MS 200        // no progress for this long while driving = stuck
#define STALL_STEP 12       // how much to add each time
#define STALL_NOISE 2       // counts of change that do not count as progress
#define BOOST_MAX 60        // never wind past this - beyond it, assume a hand
#define HOLD_MS 500         // pushing at full boost this long without moving = held

// Tapers the drive duty down as the slider nears its target, so it settles instead
// of overshooting; returns `full` unchanged once outside the taper zone.
int driveSpeed(int gap, int full) {
  // Nothing to taper if the level already runs at or below the friction floor — and
  // tapering anyway inverted the ramp, so the slowest level sped UP as it approached.
  if (full <= MIN_DUTY) return full;              // this level has no headroom to taper within
  if (gap >= RAMP_ZONE) return full;              // still outside the taper zone - cruise at full speed
  int duty = MIN_DUTY + (long)(full - MIN_DUTY) * gap / RAMP_ZONE;  // linear ramp: MIN_DUTY at gap=0, full at gap=RAMP_ZONE
  if (duty < MIN_DUTY) duty = MIN_DUTY;           // never drive below the friction floor
  if (duty > full) duty = full;      // a taper must never exceed the cruise speed
  return duty;
}

// ---- Control Parameters ----------------------------------------------------
#define HOMING_SPEED 220               // Speed used to drive to the Off mark
#define MARGIN_ERROR 10                // Acceptable position error, ~1% of range
#define PRINT_INTERVAL 200             // How often to print debug values (ms)
#define MOVE_TIMEOUT 8000              // Give up on an end rather than push into
                                       // a mechanical stop forever
#define FADER_SAMPLES 9                      // analogRead samples averaged per reading

// ---- Hand detection --------------------------------------------------------
// While driving, the slider should get steadily CLOSER to its target. If the gap
// instead grows by more than this, something is pushing back - you.
// Catching a hand while the motor is driving.
//
// Both widened from the original 150/12: right after a real arrival-and-turnaround
// (swapTarget()), the mechanism can still coast a little in the old direction before
// it actually reverses, which was getting misread as a hand. Still tiny next to a
// real grab (which moves the slider far more than this within the window), so real
// hand-detection should stay responsive.
#define GRACE_MS 300        // ignore direction right after a turn
#define REVERSE_MARGIN 20   // counts moved against the drive = a hand
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
int level = 0;       // 0 Off, 1 Low, 2 Medium, 3 High
int pointA = 0;      // low end of the current swing
int pointB = 0;      // high end
int targetVal = 0;   // whichever end we are heading for right now

enum Mode { HOMING, SWINGING, GRABBED, IDLE };  // HOMING: driving to centre at boot
                                                 // SWINGING: motor ping-ponging pointA<->pointB
                                                 // GRABBED: motor off, a hand is moving the slider
                                                 // IDLE: motor off, resting at OFF
Mode mode = HOMING;   // start every power-up by driving to the centre mark

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

unsigned long lastPrint = 0;    // millis() timestamp of the last status line printed
unsigned long moveStart = 0;    // millis() timestamp this HOMING/SWINGING leg began, for timeouts
unsigned long settleTime = 0;   // millis() timestamp the hand last moved, for the let-go check

// Human-readable name of the current mode, for the status line.
const char* statusName() {
  if (mode == HOMING)  return "HOMING  ";
  if (mode == IDLE)    return "RESTING ";
  if (mode == GRABBED) return "HAND    ";
  return "SWINGING";
}

// Motor noise on the analog line reads as movement, and this wiper also drops out
// intermittently. Both are handled below.
int readSlider() {
  // Median, not mean. A mean is dragged by a single bad sample, and this wiper
  // intermittently drops out — reading 46 or 1023 when the truth is 500. A median
  // across an odd number of samples discards those outright, however wild they are,
  // as long as fewer than half the samples are bad.
  int v[FADER_SAMPLES];                                         // sample buffer
  for (int i = 0; i < FADER_SAMPLES; i++) v[i] = analogRead(SLIDER_PIN);  // take the raw samples
  for (int i = 1; i < FADER_SAMPLES; i++) {        // insertion sort, tiny N
    int k = v[i], j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }   // shift larger values up
    v[j + 1] = k;                                          // drop k into its sorted slot
  }
  int median = v[FADER_SAMPLES / 2];   // middle of the sorted array

  // Reject the rails and keep the last good value instead.
  //
  // Deliberately NOT a window around the declared travel: that would prejudge the
  // answer and make it impossible to calibrate a range wider than the one already
  // written down. The rails are the only readings that are impossible on their own
  // terms — a wiper sitting on a live divider cannot reach either supply exactly, so
  // 0 and 1023 mean the input floated, which is what the motor's noise does to it.
  static int lastGood = -1;                          // persists between calls
  if (median <= RAIL_LOW || median >= RAIL_HIGH) {   // this reading is a floated rail, not real
    if (lastGood >= 0) return lastGood;              // substitute the last trustworthy reading
  } else {
    lastGood = median;                               // remember this reading as trustworthy
  }
  return median;
}

// Which level a reading falls in, from its distance either side of centre.
// `current` is the level already showing; a reading has to clear the boundary by
// BAND_HYSTERESIS to move off it, so noise on a mark cannot flicker the level.
int levelFor(int reading, int current) {
  int dist = abs(reading - SLIDER_CENTER);              // distance from centre, direction-independent
  const int edge[3] = { DIST_OFF_LOW, DIST_LOW_MED, DIST_MED_HIGH };  // the three boundaries, indexed by level-1

  int lv = current;                                                     // start from where we already are
  while (lv < 3 && dist > edge[lv] + BAND_HYSTERESIS) lv++;            // climb a level at a time while clearly past the next edge
  while (lv > 0 && dist < edge[lv - 1] - BAND_HYSTERESIS) lv--;        // drop a level at a time while clearly short of the last edge
  return lv;
}

// ---- NeoPixel --------------------------------------------------------------------
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);   // the strip driver object

// Colour per level. The lit fill grows outward AND heats up as the level rises.
const uint32_t LED_COLOUR[4] = {
  0x000000,   // off
  0xFFC400,   // low    - yellow
  0xFF6A00,   // medium - orange
  0xFF1FA0,   // high   - magenta
};

int shownLevel = -1;   // so the strip is only rewritten when it actually changes

// Redraws the strip only when the level has actually changed, so it never blinks or
// flickers between calls.
void showLevel(int lv) {
  if (lv == shownLevel) return;    // nothing changed - leave the strip alone
  shownLevel = lv;                 // remember what is now showing

  uint8_t mask = STRIP_MASK[lv];   // which of the 8 pixels light at this level
  uint32_t c = LED_COLOUR[lv];     // colour for this level
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, (mask & (1 << i)) ? c : 0);
  }
  strip.show();    // push the buffer to the physical strip
}

// Capture: the slider has been left somewhere, so mirror that position and swing
// between the two. The mirror of x in [min, max] is (min + max - x).
void detectFrom(int x) {
  // Mimic the movement, not the destination.
  //
  // The hand's own two extremes are the ends of the swing, so a small gesture near
  // the top gives a small swing near the top - the fader repeats what you did. Only
  // when the gesture was too small to be one (a tap, a nudge) does it fall back to
  // mirroring the position about the centre, which at least gives it something.
  // Where you left it is the value that counts. That position picks the level off
  // the measured scale, exactly as on the card, and the swing runs between it and
  // its mirror about the centre.
  int span = handMax - handMin;                                              // how far the hand actually travelled
  pointA = constrain(x, SLIDER_MIN, SLIDER_MAX);                             // where the hand left it, clamped to travel
  pointB = constrain(SLIDER_MIN + SLIDER_MAX - pointA, SLIDER_MIN, SLIDER_MAX); // its mirror about the centre

  // Only re-level on a real gesture. A span of a few counts is the swing stalling and
  // being mistaken for a hand, not you choosing something new — keep the level you set.
  if (span >= MIN_GESTURE) {
    level = levelFor(pointA, level);     // gesture was real - update the level from the resting point
  } else {
    Serial.print("  (span ");
    Serial.print(span);
    Serial.println(" — too small to be a gesture, keeping the level)");
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
  Serial.print(LEVEL_NAME[level]);
  Serial.print("  swinging ");
  Serial.print(pointA);
  Serial.print(" <-> ");
  Serial.print(pointB);
  Serial.print("  at speed ");
  Serial.println(SWING_SPEED[level]);

  if (level == 0) {
    // Off just rests where you left it. Centring happens once, at power-up, and
    // never again - dragging it back to the middle every time would fight you.
    Serial.println("  (OFF - resting here)");
    motorCoast();          // let go of the slider completely
    idleRef = pointA;      // watch for a hand from here
    mode = IDLE;
    return;
  }

  if (abs(pointB - pointA) < MIN_SWING) {
    // x sat near the middle, so x and its mirror nearly coincide. Twitching across
    // a few counts reads as a fault, so rest here and wait for a hand to give us
    // something to work with.
    Serial.println("  (too close to the centre to swing - resting, move the slider)");
    motorCoast();
    idleRef = pointA;
    mode = IDLE;
    return;
  }

  targetVal = (abs(x - pointA) > abs(x - pointB)) ? pointA : pointB;  // head for the FARTHER end first
  boost = 0;                           // no extra duty yet
  stallRef = -999;                     // force the stall timer to reset on the first check
  stallTime = millis();
  holdTime = millis();
  against = 0;                         // no reverse movement counted yet
  prevSlider = x;                      // baseline for direction tracking
  moveStart = millis();                // timeout clock for this leg starts now
  mode = SWINGING;
}

// Flip to the other end of the swing and restart this leg
void swapTarget(int sliderVal) {
  targetVal = (targetVal == pointA) ? pointB : pointA;   // switch to whichever end we were not driving toward
  boost = 0;               // fresh leg - no boost carried over
  stallRef = -999;          // force the stall timer to reset on the first check
  stallTime = millis();
  holdTime = millis();
  against = 0;              // fresh leg - no reverse movement counted yet
  prevSlider = sliderVal;   // baseline for direction tracking on the new leg
  moveStart = millis();     // timeout clock restarts for the new leg
}

// Hardcoded slider control for the sync sequence - sets up a swing directly,
// the same way detectFrom() would after a real hand gesture, but without needing
// one. `high` false just rests (motor released, no swing); true swings the full
// physical travel (SLIDER_MIN <-> SLIDER_MAX), which is always classified HIGH.
void startSliderSwing(bool high) {
  int sliderVal = readSlider();
  if (!high) {
    motorCoast();
    idleRef = sliderVal;
    mode = IDLE;
    Serial.println("SLIDER SYNC -> OFF");
    return;
  }
  level = 3;              // HIGH
  pointA = SLIDER_MIN;
  pointB = SLIDER_MAX;
  targetVal = (abs(sliderVal - pointA) > abs(sliderVal - pointB)) ? pointA : pointB;  // head for the farther end first
  boost = 0;
  stallRef = -999;         // force the stall timer to reset on the first check
  stallTime = millis();
  holdTime = millis();
  against = 0;
  prevSlider = sliderVal;
  moveStart = millis();
  mode = SWINGING;
  Serial.println("SLIDER SYNC -> HIGH");
}

// Every way into GRABBED goes through here. Splitting it across the call sites is
// what left the idle path with a stale captureStart — so the three second window had
// already expired before the gesture began, and it acted on the first reading.
void beginCapture(int sliderVal) {
  syncAutoCycle = false;                  // a real hand takes over from the hardcoded sequence
  mode = GRABBED;                         // motor stays off until the hand lets go
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
// Stillness alone is not enough: at the start the motor has not spun up, so the
// slider is legitimately still for a moment. Only count it once it has actually
// travelled — otherwise calibration ends instantly at the starting position.
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
  mode = HOMING;            // return to the centre once calibration is done
}

// Case-insensitive compare, so "LOW"/"low"/"Low" all match.
bool eqIgnoreCase(const char* a, const char* b) {
  while (*a && *b) { if (toupper(*a) != toupper(*b)) return false; a++; b++; }
  return *a == *b;
}

// Dispatches one complete command line. `cmd` is mutated in place (split at the
// first space) so `arg` can point straight into it rather than copying.
void handleCommand(char* cmd) {
  char* sp = strchr(cmd, ' ');
  if (sp) *sp = '\0';
  const char* arg = sp ? sp + 1 : "";

  if (eqIgnoreCase(cmd, "c")) {
    syncAutoCycle = false;   // calibrating fights the sequence driving the slider - cancel it
    calibrate();
  } else if (eqIgnoreCase(cmd, "low")) {
    syncAutoCycle = false;   // a manual arm command takes over from the hardcoded sequence
    startArm(ARM_LOW);
  } else if (eqIgnoreCase(cmd, "med") || eqIgnoreCase(cmd, "medium")) {
    syncAutoCycle = false;
    startArm(ARM_MED);
  } else if (eqIgnoreCase(cmd, "high")) {
    syncAutoCycle = false;
    startArm(ARM_HIGH);
  } else if (eqIgnoreCase(cmd, "stop")) {
    syncAutoCycle = false;
    armRunning = false;
    armHeld = false;
    Serial.println("ARM -> stopping, easing back to centre");
  } else if (eqIgnoreCase(cmd, "go") && arg[0] != '\0') {
    syncAutoCycle = false;
    armAngle = constrain((float)atof(arg), (float)ARM_ANGLE_MIN, (float)ARM_ANGLE_MAX);
    armHeld = true;
    armRunning = false;
    armServo.write((int)(armAngle + 0.5));
    Serial.print("ARM holding ");
    Serial.println(armAngle);
  } else {
    Serial.print("ERR: unknown command '");
    Serial.print(cmd);
    Serial.println("'");
  }
}

// Builds up one line at a time and dispatches it on Enter - lets `go <deg>`
// carry a number, which a single immediate character (the old 'c'-only scheme)
// could not.
char cmdLine[32];
uint8_t cmdLen = 0;

void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdLen > 0) {
        cmdLine[cmdLen] = '\0';
        handleCommand(cmdLine);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdLine) - 1) {
      cmdLine[cmdLen++] = ch;
    }
  }
}

void setup() {
  // No `while (!Serial)` - it would stall the sketch when run without a computer
  Serial.begin(115200);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  motorCoast();          // start with the slider free, not braked or driving

  strip.begin();                       // initialise the NeoPixel strip
  strip.setBrightness(LED_BRIGHTNESS);
  strip.clear();                       // all pixels off until a level is set
  strip.show();

  armServo.attach(SERVO_PIN);          // attached and held from boot - see note #6 above
  armServo.write(ARM_CENTER_ANGLE);    // start sitting at rest, not wherever it happened to be
  armAngle = ARM_CENTER_ANGLE;
  armLastTick = millis();

  Serial.println();
  Serial.println("=== Social Battery - motorized fader + hardcoded arm cycle ===");
  Serial.println("SYNC sequence running: (servo LOW/slider OFF) -> (servo LOW/slider HIGH) -> (both HIGH, synced), 15s per step, repeating");
  Serial.print("travel ");
  Serial.print(SLIDER_MIN);
  Serial.print(" - ");
  Serial.println(SLIDER_MAX);
  for (int i = 0; i < 4; i++) {                 // print the four level bands for reference at boot
    Serial.print("  ");
    Serial.print(LEVEL_NAME[i]);
    Serial.print("\t");
    const int edge[3] = { DIST_OFF_LOW, DIST_LOW_MED, DIST_MED_HIGH };
    int dLo = (i == 0) ? 0 : edge[i - 1];                                    // inner edge of this level's band
    int dHi = (i == 3) ? (SLIDER_MAX - SLIDER_CENTER) : edge[i];            // outer edge of this level's band
    if (i == 0) {
      Serial.print(SLIDER_CENTER - dHi); Serial.print(" - "); Serial.print(SLIDER_CENTER + dHi);
      Serial.println("\t\t(rests, still)");
    } else {
      Serial.print(SLIDER_CENTER - dHi); Serial.print(" - "); Serial.print(SLIDER_CENTER - dLo);
      Serial.print("  and  ");
      Serial.print(SLIDER_CENTER + dLo); Serial.print(" - "); Serial.print(SLIDER_CENTER + dHi);
      Serial.print("\tspeed "); Serial.println(SWING_SPEED[i]);
    }
  }
  Serial.println();
  Serial.print("HOMING to ");
  Serial.println(SLIDER_CENTER);

  moveStart = millis();   // timeout clock for the initial homing leg
  mode = HOMING;          // drive to the centre before doing anything else
}

void loop() {
  readSerial();                     // check for a command line (arm level, go, or c)

  unsigned long syncNow = millis();
  if (syncAutoCycle && (lastSyncSwitch == 0 || syncNow - lastSyncSwitch >= SYNC_STEP_MS)) {
    if (lastSyncSwitch != 0) syncStep = (syncStep + 1) % 3;   // advance to the next step, wrapping back to step 0
    lastSyncSwitch = syncNow;
    enterSyncStep(syncStep);
  }

  unsigned long armNow = millis();
  float armDt = (armNow - armLastTick) / 1000.0;   // seconds since the last arm motion update
  if (armDt >= 0.02) {              // 50Hz motion update, same rate the full sketch uses
    armLastTick = armNow;
    updateArm(armDt);
  }

  int sliderVal = readSlider();     // one filtered, noise-rejected reading for this pass

  // The strip shows the level you SET, and holds it.
  //
  // Not the level of wherever the slider happens to be this millisecond: during a
  // swing the fader sweeps across every band on its way between the two ends, so
  // following the live reading made the lights flicker through yellow, orange and
  // magenta continuously. The colour only changes when you set a new level.
  showLevel(level < 0 ? 0 : level);

  // Print status on an interval. Never use delay() here - the loop has to keep
  // sampling, or it misses the movement it is supposed to detect.
  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();

    unsigned long ms = millis();
    Serial.print("t=");
    Serial.print(ms / 1000);
    Serial.print(".");
    unsigned long frac = ms % 1000;
    if (frac < 100) Serial.print("0");     // pad the fractional seconds to 3 digits
    if (frac < 10)  Serial.print("0");
    Serial.print(frac);
    Serial.print("s  ");

    Serial.print(statusName());
    Serial.print("  slider: ");
    Serial.print(sliderVal);
    Serial.print("  level: ");
    Serial.print(LEVEL_NAME[level]);
    Serial.print("  target: ");
    if (mode == HOMING) Serial.println(SLIDER_CENTER);
    else if (mode == SWINGING) Serial.println(targetVal);
    else Serial.println("-");
  }

  switch (mode) {

    // ---- Drive to the Off mark on power-up ----
    case HOMING: {
      bool timedOut = (millis() - moveStart >= MOVE_TIMEOUT);   // give up rather than grind forever

      if (abs(SLIDER_CENTER - sliderVal) <= MARGIN_ERROR || timedOut) {   // close enough, or out of time
        motorStop();     // brake first to stop cleanly at the mark
        if (timedOut) {
          Serial.print("HOMING gave up at ");
          Serial.println(sliderVal);
        } else {
          Serial.print("AT OFF MARK ");
          Serial.println(sliderVal);
        }
        // Settle here rather than calling detectFrom() again: at the centre the level
        // is OFF, and OFF sends us back to HOMING, which would loop forever.
        motorCoast();                              // release the slider now that we have arrived
        level = levelFor(sliderVal, level);        // set the level from wherever we actually landed
        idleRef = sliderVal;                       // watch for a hand from here
        mode = IDLE;
        Serial.print("  resting at ");
        Serial.print(sliderVal);
        Serial.print(", level ");
        Serial.println(LEVEL_NAME[level]);
      } else if (sliderVal > SLIDER_CENTER) {
        motorBackward(HOMING_SPEED);    // currently past centre - drive back toward it
      } else {
        motorForward(HOMING_SPEED);     // currently short of centre - drive up toward it
      }
      break;
    }

    // ---- Resting: motor off, waiting for a hand ----
    case IDLE: {
      motorCoast();     // keep releasing the motor every pass - nothing should be driving here
      if (abs(sliderVal - idleRef) > SETTLE_MOVE) {    // it moved more than noise allows - a hand touched it
        Serial.println("HAND DETECTED - set it where you like");
        beginCapture(sliderVal);
      }
      break;
    }

    // ---- Your hand is on it: motor off, wait for you to finish ----
    case GRABBED: {
      motorCoast();     // keep releasing the motor every pass while the hand is on it
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

    // ---- Swinging across the selected level's range ----
    case SWINGING: {
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
      if (millis() - moveStart >= MOVE_TIMEOUT) {
        Serial.print("GAVE UP at ");
        Serial.print(sliderVal);
        Serial.print(" chasing ");
        Serial.println(targetVal);
        motorStop();              // brake before reversing direction
        swapTarget(sliderVal);    // try the other end instead
        break;
      }

      if (gap > MARGIN_ERROR) {                                  // not yet close enough to call it arrived
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
          // Wind up to break friction - but only so far. Past BOOST_MAX the thing
          // stopping it is not friction, it is a hand holding it still, and shoving
          // harder is precisely the wrong answer.
          if (boost < BOOST_MAX) {
            boost += STALL_STEP;      // try a bit more duty next pass
          } else if (pastGrace && millis() - holdTime >= HOLD_MS) {   // at max boost and still stuck, for long enough
            // This is NOT treated as a hand. A real hand is already caught reliably
            // above, by genuine pushback against the drive direction. Stuck at max
            // boost with no pushback is far more likely friction or a hard stop the
            // motor can't overcome - re-capturing from here (as a hand would) risked
            // re-levelling the fader to whatever extreme it happened to stall at.
            // Give up on this end and swing back instead, same as the timeout
            // safety net above - the level you set is left alone either way.
            Serial.println("  (stuck at max boost, no pushback - giving up on this end)");
            motorStop();
            swapTarget(sliderVal);
            break;
          }
        }
      }

      int duty = driveSpeed(gap, SWING_SPEED[level]) + boost;   // tapered cruise speed plus any stall boost
      if (duty > 255) duty = 255;                               // PWM ceiling

      if (sliderVal > targetVal + MARGIN_ERROR) {
        motorBackward(duty);      // currently past the target - drive down toward it
      } else if (sliderVal < targetVal - MARGIN_ERROR) {
        motorForward(duty);       // currently short of the target - drive up toward it
      } else {
        // Reached this end - turn around immediately, no dwell
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
// MKR 1010 (SAMD21) analogWrite() re-muxes the pin to a timer peripheral, and a
// later digitalWrite() on that same pin is ignored until pinMode() is called
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
