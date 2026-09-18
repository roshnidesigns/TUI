/*
* Social Battery - motorized fader: read, capture, mimic
* Tangible User Interface class - CIID
*
* One 100mm motorized fader, no servos.
*
*   READ     the slider is sampled continuously, 8 reads averaged
*   CAPTURE  move it by hand and the motion is recorded, position by position
*   MIMIC    let go, and the fader replays your motion back to you, on a loop
*
* Where you leave the slider also picks a social-battery level, from the marks
* measured on the bench:
*
*     level     slider range     mimic speed
*     Off        270 - 500       (does not replay - rests, still)
*     Low        500 - 580       slow
*     Medium     580 - 660       medium
*     High       660 - 780       fast
*
* So the gesture is yours, and the level says how energetically it comes back.
* Grab the slider at any time and it releases the motor and records again.
*
* Board:  Arduino MKR WiFi 1010
* Driver: HW-354, Motor A on IN1/IN2, powered from an EXTERNAL 5V supply with its
*         ground tied to the board's. The fader's pot runs on 3.3V - never 5V,
*         the MKR's analog pins are not 5V tolerant.
*/

#include <Adafruit_NeoPixel.h>

// ---- Pin Definitions -------------------------------------------------------
#define MOTOR_IN1 4    // HW-354 IN1 (Motor A) - direction + speed
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
#define RAIL_HIGH 1015
#define SLIDER_CENTER ((SLIDER_MIN + SLIDER_MAX) / 2)   // 544

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
#define HALF_TRAVEL   ((SLIDER_MAX - SLIDER_MIN) / 2)
#define DIST_OFF_LOW  (HALF_TRAVEL / 4)
#define DIST_LOW_MED  (HALF_TRAVEL * 2 / 4)
#define DIST_MED_HIGH (HALF_TRAVEL * 3 / 4)

#define BAND_HYSTERESIS 8

// ---- NeoPixel --------------------------------------------------------------
// Eight pixels. The lit pair moves OUTWARD from the middle as the level rises,
// so the strip reads as the battery opening up.
//
//     off      nothing
//     low      pixels 4,5            the middle pair
//     medium   pixels 3,4,5,6        widening
//     high     pixels 1..8           the whole strip
//
// Numbering below is 0-based, so your 1..8 become 0..7.
#define LED_PIN 1
#define LED_COUNT 8
#define LED_BRIGHTNESS 90

// How hard each level is driven. The swing's ENDS come from the mirror - where you
// left the slider, and its reflection - so the gesture is yours; the level only says
// how energetically it comes back.
//                            Off  Low  Medium  High
const int SWING_SPEED[4] = {   0,  230,   243,   255 };
const char* LEVEL_NAME[4] = { "OFF", "LOW", "MEDIUM", "HIGH" };

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

int driveSpeed(int gap, int full) {
  // Nothing to taper if the level already runs at or below the friction floor — and
  // tapering anyway inverted the ramp, so the slowest level sped UP as it approached.
  if (full <= MIN_DUTY) return full;
  if (gap >= RAMP_ZONE) return full;
  int duty = MIN_DUTY + (long)(full - MIN_DUTY) * gap / RAMP_ZONE;
  if (duty < MIN_DUTY) duty = MIN_DUTY;
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
// The old test - "the gap to the target grew" - needed the leg to have closed 45
// counts before it would arm, so grabbing early in a leg was simply invisible. And
// it needed a big push to register. This looks at DIRECTION instead: the slider
// should be moving the way we are driving it, and if it moves the other way, that is
// a hand. Far more sensitive, and it works from the first moment of a leg.
//
// The only thing it has to forgive is the instant after a turn, when the fader is
// still carrying momentum the old way - hence a short grace period rather than a
// distance threshold.
#define GRACE_MS 150        // ignore direction right after a turn
#define REVERSE_MARGIN 12   // counts moved against the drive = a hand
#define GRAB_MARGIN 45      // legacy gap test, kept as a slower backstop
#define SETTLE_MOVE 8      // Counts of change that still count as "hand moving"
#define SETTLE_MS 400      // Hand still this long = you let go

// ---- Calibration -----------------------------------------------------------
#define CAL_STILL_DELTA 6      // change that still counts as moving
#define CAL_STILL_MS 400       // still this long AFTER travelling = at the stop
#define CAL_NO_MOVE_MS 2000    // never moved at all = we started against it
#define CAL_TIMEOUT 12000      // ceiling per direction

// Function Declarations
void motorCoast();
void motorStop();
void motorForward(int speed);
void motorBackward(int speed);

// ---- State -----------------------------------------------------------------
int level = 0;       // 0 Off, 1 Low, 2 Medium, 3 High
int pointA = 0;      // low end of the current swing
int pointB = 0;      // high end
int targetVal = 0;   // whichever end we are heading for right now

enum Mode { HOMING, SWINGING, GRABBED, IDLE };
Mode mode = HOMING;

int bestGap = 0;     // Closest we have got to the target on this leg
int legStartGap = 0; // Gap when this leg began
bool grabArmed = false;  // Grab detection only counts once the leg is making progress
int prevSlider = 0;      // Last reading, for working out which way it is moving
int against = 0;         // Counts moved against the drive on this leg
unsigned long holdTime = 0;
int stallRef = 0;        // Reading the stall timer measures progress against
unsigned long stallTime = 0;
int boost = 0;           // Extra duty added to break friction when stuck
int boostReported = 0;   // Highest boost we have mentioned, so it is logged once
int settleRef = 0;   // Reading the settle timer is measured against
int handMin = 0;     // How far the hand travelled, while it was on the fader
int handMax = 0;
int idleRef = 0;     // Reading we watch for a hand while resting

unsigned long lastPrint = 0;
unsigned long moveStart = 0;
unsigned long settleTime = 0;

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
  int v[FADER_SAMPLES];
  for (int i = 0; i < FADER_SAMPLES; i++) v[i] = analogRead(SLIDER_PIN);
  for (int i = 1; i < FADER_SAMPLES; i++) {        // insertion sort, tiny N
    int k = v[i], j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }
    v[j + 1] = k;
  }
  int median = v[FADER_SAMPLES / 2];

  // Reject the rails and keep the last good value instead.
  //
  // Deliberately NOT a window around the declared travel: that would prejudge the
  // answer and make it impossible to calibrate a range wider than the one already
  // written down. The rails are the only readings that are impossible on their own
  // terms — a wiper sitting on a live divider cannot reach either supply exactly, so
  // 0 and 1023 mean the input floated, which is what the motor's noise does to it.
  static int lastGood = -1;
  if (median <= RAIL_LOW || median >= RAIL_HIGH) {
    if (lastGood >= 0) return lastGood;
  } else {
    lastGood = median;
  }
  return median;
}

// Which level a reading falls in, from its distance either side of centre.
// `current` is the level already showing; a reading has to clear the boundary by
// BAND_HYSTERESIS to move off it, so noise on a mark cannot flicker the level.
int levelFor(int reading, int current) {
  int dist = abs(reading - SLIDER_CENTER);
  const int edge[3] = { DIST_OFF_LOW, DIST_LOW_MED, DIST_MED_HIGH };

  int lv = current;
  while (lv < 3 && dist > edge[lv] + BAND_HYSTERESIS) lv++;
  while (lv > 0 && dist < edge[lv - 1] - BAND_HYSTERESIS) lv--;
  return lv;
}

// ---- NeoPixel --------------------------------------------------------------------
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Which pixels each level lights, as a bitmask over 0..7. The fill grows OUTWARD
// from the middle pair, so the strip reads as a level rising rather than a pattern
// changing - each level keeps everything the one below it lit.
//
//     off      nothing
//     low      4,5              the middle pair
//     medium   3,4,5,6          widening
//     high     1,2,3,4,5,6,7,8  the whole strip
//
// Bit 0 is pixel 1, so the masks below read right-to-left.
const uint8_t LED_MASK[4] = { 0b00000000, 0b00011000, 0b00111100, 0b11111111 };

// Colour per level. The fill grows outward AND heats up as the level rises.
const uint32_t LED_COLOUR[4] = {
  0x000000,   // off
  0xFFC400,   // low    - yellow
  0xFF6A00,   // medium - orange
  0xFF1FA0,   // high   - magenta
};

int shownLevel = -1;   // so the strip is only rewritten when it actually changes

void showLevel(int lv) {
  if (lv == shownLevel) return;
  shownLevel = lv;

  uint8_t mask = LED_MASK[lv];
  uint32_t c = LED_COLOUR[lv];
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, (mask & (1 << i)) ? c : 0);
  }
  strip.show();
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
  int span = handMax - handMin;
  pointA = constrain(x, SLIDER_MIN, SLIDER_MAX);
  pointB = constrain(SLIDER_MIN + SLIDER_MAX - pointA, SLIDER_MIN, SLIDER_MAX);

  // Only re-level on a real gesture. A span of a few counts is the swing stalling and
  // being mistaken for a hand, not you choosing something new — keep the level you set.
  if (span >= MIN_GESTURE) {
    level = levelFor(pointA, level);
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
    motorCoast();
    idleRef = pointA;
    targetVal = pointA;
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

  targetVal = (abs(x - pointA) > abs(x - pointB)) ? pointA : pointB;
  bestGap = abs(targetVal - x);
  legStartGap = bestGap;
  grabArmed = false;
  boost = 0;
  stallRef = -999;
  stallTime = millis();
  holdTime = millis();
  against = 0;
  prevSlider = x;
  moveStart = millis();
  mode = SWINGING;
}

// Flip to the other end of the swing and restart this leg
void swapTarget(int sliderVal) {
  targetVal = (targetVal == pointA) ? pointB : pointA;
  bestGap = abs(targetVal - sliderVal);
  legStartGap = bestGap;
  grabArmed = false;
  boost = 0;
  stallRef = -999;
  stallTime = millis();
  holdTime = millis();
  against = 0;
  prevSlider = sliderVal;
  moveStart = millis();
}

// Every way into GRABBED goes through here. Splitting it across the call sites is
// what left the idle path with a stale captureStart — so the three second window had
// already expired before the gesture began, and it acted on the first reading.
void beginCapture(int sliderVal) {
  mode = GRABBED;
  handMin = handMax = sliderVal;
  settleRef = sliderVal;
  settleTime = millis();
}

void handDetected(int sliderVal) {
  Serial.println("HAND DETECTED - motor released, set it where you like");
  motorCoast();
  beginCapture(sliderVal);
}

// Drive one way until the reading stops changing, and report where it stopped.
// Stillness alone is not enough: at the start the motor has not spun up, so the
// slider is legitimately still for a moment. Only count it once it has actually
// travelled — otherwise calibration ends instantly at the starting position.
int findStop(int dir) {
  unsigned long start = millis(), lastMove = millis();
  int ref = readSlider();
  bool moved = false;

  while (millis() - start < CAL_TIMEOUT) {
    if (dir > 0) motorForward(255); else motorBackward(255);
    int v = readSlider();
    if (abs(v - ref) > CAL_STILL_DELTA) { ref = v; lastMove = millis(); moved = true; }
    else if (moved && millis() - lastMove >= CAL_STILL_MS) break;
    else if (!moved && millis() - start >= CAL_NO_MOVE_MS) break;
  }
  motorCoast();
  delayMicroseconds(1);
  return readSlider();
}

void calibrate() {
  Serial.println();
  Serial.println("=== CALIBRATING - hands off ===");
  int lo = findStop(-1);
  int hi = findStop(+1);
  if (lo > hi) { int t = lo; lo = hi; hi = t; }

  Serial.print("  measured travel  "); Serial.print(lo);
  Serial.print(" - "); Serial.println(hi);
  Serial.print("  span             "); Serial.println(hi - lo);
  int c = (lo + hi) / 2;
  Serial.print("  centre           "); Serial.println(c);
  Serial.println("  paste into the sketch:");
  Serial.print("    #define SLIDER_MIN "); Serial.println(lo);
  Serial.print("    #define SLIDER_MAX "); Serial.println(hi);
  Serial.println("  band edges that follow:");
  const int e[3] = { DIST_OFF_LOW, DIST_LOW_MED, DIST_MED_HIGH };
  const char *n[4] = { "OFF", "LOW", "MEDIUM", "HIGH" };
  for (int i = 0; i < 4; i++) {
    int dLo = (i == 0) ? 0 : e[i - 1];
    int dHi = (i == 3) ? (hi - c) : e[i];
    Serial.print("    "); Serial.print(n[i]); Serial.print("\t");
    if (i == 0) { Serial.print(c - dHi); Serial.print(" - "); Serial.println(c + dHi); }
    else {
      Serial.print(c - dHi); Serial.print(" - "); Serial.print(c - dLo);
      Serial.print("   and   "); Serial.print(c + dLo);
      Serial.print(" - "); Serial.println(c + dHi);
    }
  }
  Serial.println("===============================");
  Serial.println();
  moveStart = millis();
  mode = HOMING;
}

// A single character is enough: 'c' calibrates.
void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'c' || ch == 'C') calibrate();
  }
}

void setup() {
  // No `while (!Serial)` - it would stall the sketch when run without a computer
  Serial.begin(115200);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  motorCoast();

  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.clear();
  strip.show();

  Serial.println();
  Serial.println("=== Social Battery - motorized fader ===");
  Serial.print("travel ");
  Serial.print(SLIDER_MIN);
  Serial.print(" - ");
  Serial.println(SLIDER_MAX);
  for (int i = 0; i < 4; i++) {
    Serial.print("  ");
    Serial.print(LEVEL_NAME[i]);
    Serial.print("\t");
    const int edge[3] = { DIST_OFF_LOW, DIST_LOW_MED, DIST_MED_HIGH };
    int dLo = (i == 0) ? 0 : edge[i - 1];
    int dHi = (i == 3) ? (SLIDER_MAX - SLIDER_CENTER) : edge[i];
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

  moveStart = millis();
  mode = HOMING;
}

void loop() {
  readSerial();

  int sliderVal = readSlider();

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
    if (frac < 100) Serial.print("0");
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
      bool timedOut = (millis() - moveStart >= MOVE_TIMEOUT);

      if (abs(SLIDER_CENTER - sliderVal) <= MARGIN_ERROR || timedOut) {
        motorStop();
        if (timedOut) {
          Serial.print("HOMING gave up at ");
          Serial.println(sliderVal);
        } else {
          Serial.print("AT OFF MARK ");
          Serial.println(sliderVal);
        }
        // Settle here rather than calling detectFrom() again: at the centre the level
        // is OFF, and OFF sends us back to HOMING, which would loop forever.
        motorCoast();
        level = levelFor(sliderVal, level);
        idleRef = sliderVal;
        targetVal = sliderVal;
        mode = IDLE;
        Serial.print("  resting at ");
        Serial.print(sliderVal);
        Serial.print(", level ");
        Serial.println(LEVEL_NAME[level]);
      } else if (sliderVal > SLIDER_CENTER) {
        motorBackward(HOMING_SPEED);
      } else {
        motorForward(HOMING_SPEED);
      }
      break;
    }

    // ---- Resting: motor off, waiting for a hand ----
    case IDLE: {
      motorCoast();
      if (abs(sliderVal - idleRef) > SETTLE_MOVE) {
        Serial.println("HAND DETECTED - set it where you like");
        beginCapture(sliderVal);
      }
      break;
    }

    // ---- Your hand is on it: motor off, wait for you to finish ----
    case GRABBED: {
      motorCoast();
      // Measure the gesture, not just where it ends: the two extremes the hand
      // reached are what the fader will ping-pong between.
      if (sliderVal < handMin) handMin = sliderVal;
      if (sliderVal > handMax) handMax = sliderVal;
      if (abs(sliderVal - settleRef) > SETTLE_MOVE) {
        settleRef = sliderVal;
        settleTime = millis();
      } else if (millis() - settleTime >= SETTLE_MS) {
        // Still for long enough - that is where you wanted it
        detectFrom(sliderVal);
      }
      break;
    }

    // ---- Swinging across the selected level's range ----
    case SWINGING: {
      int gap = abs(targetVal - sliderVal);
      int dir = (targetVal > sliderVal) ? 1 : -1;      // the way we are driving
      int delta = sliderVal - prevSlider;              // the way it actually went
      prevSlider = sliderVal;

      bool pastGrace = (millis() - moveStart >= GRACE_MS);

      // Moving against the drive is a hand - nothing else pushes back.
      if (delta * dir > 0) {
        against = 0;                    // making progress, forget any wobble
      } else if (delta * dir < 0) {
        against += -delta * dir;
        if (pastGrace && against > REVERSE_MARGIN) {
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
        motorStop();
        swapTarget(sliderVal);
        break;
      }

      if (gap > MARGIN_ERROR) {
        if (abs(sliderVal - stallRef) > STALL_NOISE) {
          // moving again
          stallRef = sliderVal;
          stallTime = millis();
          holdTime = millis();
          if (boost > boostReported) {
            Serial.print("  broke friction at duty +");
            Serial.println(boost);
            boostReported = boost;
          }
          boost = 0;
        } else if (millis() - stallTime >= STALL_MS) {
          stallTime = millis();
          // Wind up to break friction - but only so far. Past BOOST_MAX the thing
          // stopping it is not friction, it is a hand holding it still, and shoving
          // harder is precisely the wrong answer.
          if (boost < BOOST_MAX) {
            boost += STALL_STEP;
          } else if (pastGrace && millis() - holdTime >= HOLD_MS) {
            Serial.println("  (held still against full drive - treating as a hand)");
            handDetected(sliderVal);
            break;
          }
        }
      }

      int duty = driveSpeed(gap, SWING_SPEED[level]) + boost;
      if (duty > 255) duty = 255;

      if (sliderVal > targetVal + MARGIN_ERROR) {
        motorBackward(duty);
      } else if (sliderVal < targetVal - MARGIN_ERROR) {
        motorForward(duty);
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
