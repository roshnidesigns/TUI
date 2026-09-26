/*
 * Social Battery — the motorized fader
 * Tangible User Interface, CIID
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
 * SERIAL, 115200
 *
 *   c     calibrate: drive to each mechanical stop and report the travel, ready
 *         to paste into SLIDER_MIN / SLIDER_MAX below.
 *
 *   Status prints every 200ms: elapsed time, phase, slider reading, level.
 *
 * WHAT TO TUNE FIRST
 *
 *   SLIDER_MIN / SLIDER_MAX   the measured ends of travel. Everything else is
 *                             derived from these, so get them right first.
 *   SWING_SPEED               how hard each level drives. Below MIN_DUTY the
 *                             motor will not break friction at all.
 *   MIN_GESTURE               how much hand movement counts as choosing a new
 *                             level, rather than a stalled swing being mistaken
 *                             for a hand.
 *
 * FIVE THINGS LEARNED THE HARD WAY — please do not undo these
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
 *
 * Board: Arduino MKR WiFi 1010
 */

#include <Adafruit_NeoPixel.h>   // drives the NeoPixel strip

// ---- Pin Definitions -------------------------------------------------------
#define MOTOR_IN1 3    // HW-354 IN1 (Motor A) - direction + speed
#define MOTOR_IN2 5    // HW-354 IN2 (Motor A) - direction + speed
#define SLIDER_PIN A1  // Analog input from slider's feedback potentiometer

// ---- Travel limits, measured on the bench ----------------------------------
#define SLIDER_MIN 199      // low stop, measured by hand with the motor idle
#define SLIDER_MAX 877      // high stop, measured by hand with the motor idle

// How far outside that travel a reading may sit before it is treated as noise. The
// motor throws a lot of electrical rubbish onto the analog line, and the readings it
// produces are not near-misses — they are 0 and 1023, the rails, which the slider
// cannot physically reach. Anything out here is discarded and the last good reading
// stands. This is what was making HIGH glitch: that is when the motor works hardest.
#define RAIL_LOW 8         // at or below this the input has floated, not moved
#define RAIL_HIGH 1015     // at or above this the input has floated, not moved
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

// How hard each level is driven. The swing's ENDS come from the mirror - where you
// left the slider, and its reflection - so the gesture is yours; the level only says
// how energetically it comes back.
//                            Off  Low  Medium  High
const int SWING_SPEED[4] = {   0,  230,   243,   255 };            // PWM duty per level, 0-255
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

// Friction varies along the track and between faders, and the duty that actually
// breaks it has never been measured properly. So rather than trust a guess: if the
// slider stops making progress while we are driving it, wind the duty up until it
// moves again, and report the value that worked.
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
#define GRACE_MS 150        // ignore direction right after a turn
#define REVERSE_MARGIN 12   // counts moved against the drive = a hand
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
    targetVal = pointA;    // nothing to drive toward, but kept in sync for the status line
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
    targetVal = pointA;
    mode = IDLE;
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
  mode = SWINGING;
}

// Flip to the other end of the swing and restart this leg
void swapTarget(int sliderVal) {
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

// Every way into GRABBED goes through here. Splitting it across the call sites is
// what left the idle path with a stale captureStart — so the three second window had
// already expired before the gesture began, and it acted on the first reading.
void beginCapture(int sliderVal) {
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

// A single character is enough: 'c' calibrates.
void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'c' || ch == 'C') calibrate();    // any other character is ignored
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

  Serial.println();
  Serial.println("=== Social Battery - motorized fader ===");
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
  readSerial();                     // check for a 'c' calibration request

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
        targetVal = sliderVal;                     // kept in sync for the status line
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
            // re-levelling the fader to whatever extreme it happened to stall at,
            // which is exactly the "drifts to HIGH after a few seconds" bug this
            // replaced. Give up on this end and swing back instead, same as the
            // timeout safety net above - the level you set is left alone either way.
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
