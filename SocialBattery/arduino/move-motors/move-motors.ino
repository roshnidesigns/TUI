/*
  move-motors — Social Battery
  ----------------------------
  The whole thing in one sketch. Servo arms on a shared base; each arm carries its own
  state, and the three states step both amplitude and speed up together:

      state    amplitude   speed         reads as
      LOW      10°         slow          present, but holding still
      MEDIUM   20°         medium        clearly swaying, comfortable pace
      HIGH     35°         slightly      fully switched on — wide and lively
                           faster than
                           medium

  Every state swings the arm symmetrically about its resting angle — the same number
  of degrees left and right of centre, whichever state it's in. Only the width and
  pace of that swing change.

  It runs two ways, and you do not have to choose in advance:

    * On its own. Plug the board in and it demonstrates itself, cycling every arm
      through the three states, six seconds each. No computer needed.
    * Driven. The web page (or anything that can open a serial port) takes over the
      moment it sends a command, and the demo cycle stands down.

  The red arm is a special case: a MOTORIZED FADER is its controller. The fader's own
  feedback pot on A1 reads its position, and a motor can drive it back. So red's level
  can be set by hand AND commanded by software — a dial that pushes back.

  Hand movement always wins: the fader idles COASTING (motor free), so it can be moved
  at any time, and the reading is picked up immediately. The motor only engages while
  seeking a commanded position, then goes straight back to coasting.

  Red still never joins the standalone demo, and "every arm" commands still skip it.

  Board: Arduino MKR WiFi 1010. Serial is the native USB port, 115200 baud.

  ---------------------------------------------------------------- protocol
  One ASCII command per line, '\n' terminated:

    H                  handshake: blink the LED pattern and identify the board
    T:<arm>:<state>    start ONE arm in that state, e.g. T:0:HIGH. For the fader arm
                       this ALSO drives the fader to the matching position.
    T:<state>          start EVERY OTHER arm in that state (skips POT_ARM)
    X:<arm>            stop one arm — for the fader arm, drives it down to Off
    X                  stop every OTHER arm (skips POT_ARM)
    F:<0-1023>         drive the fader to a raw position
    V                  print the live amplitude/speed table
    V:<state>:<amp>:<rate>   retune a state live, e.g. V:MED:24:0.7
    E:<0|1>            detach / attach the servos by hand
    ?                  send one status line now

  Replies:
    OK:HELLO social-battery ARMS=<n>                     answer to H
    S:<state>,<run>,<angle>;<state>,<run>,<angle>;...    one group per arm, 10 Hz
    FD:<pos>,<band>,<seeking>                           fader telemetry, 10 Hz
    OK:<echo>  /  ERR:<reason>

  ---------------------------------------------------------------- serial monitor
  Typing the protocol by hand is tedious, so single characters work too:

    1 / 2 / 3   all arms to low / medium / high
    0           stop all arms
    a           resume the automatic demo cycle
    c           hold every arm at CENTER_ANGLE, for setting the resting pose

  Digits and lowercase letters were chosen so they cannot collide with the protocol.

  ---------------------------------------------------------------- wiring
  Three servo signals -> D5, D3, D1 (see SERVO_PIN below for which arm is which).
  Servo power from an EXTERNAL 5V supply, its ground tied to the board's ground — do not
  run servos off the MKR's own regulator.

  Motorized fader (100mm, 6 wires) + HW-354 driver:
    POT. SLIDER  (green)  -> A1            the wiper, position feedback
    VCC SLIDER   (red)    -> MKR VCC 3.3V  NEVER 5V, the analog pins are not 5V tolerant
    GND SLIDER   (grey)   -> MKR GND
    VCC MOTOR    (red)    -> driver Motor A / OUT1   (swap with OUT2 to reverse)
    GND MOTOR    (black)  -> driver Motor A / OUT2
    TOUCH SLIDER (orange) -> unconnected, capacitive strip not used yet
    driver IN1 -> D4,  IN2 -> D2   (D5 is taken by the blue arm servo)
    driver VCC/GND -> EXTERNAL 5V supply, ground tied to MKR GND. Running the driver
    off the MKR's 3.3V pin makes the fader crawl and browns the board off USB.
*/

#include <Servo.h>

// ---------------------------------------------------------------- pins
//
// One entry per arm. Add pins here and the whole sketch adapts — per-arm state,
// telemetry and the arm count reported to the web page all size themselves from it.
//
const uint8_t SERVO_PIN[] = { 5, 3, 1 };
//
// Arm order follows this array: index 0 is the blue square on the longest rod at the
// back (D5), 1 the yellow wedge in the middle (D3), 2 the red octagon at the front (D1).
//
// D1 is also the MKR's Serial1 TX pin. This sketch never uses Serial1, so it's free to
// drive a servo — but keep that in mind before adding anything that talks over Serial1.
//
// Avoid pins 8, 9 and 10 on the MKR WiFi 1010 — they are the SPI bus to the onboard
// NINA WiFi module. A servo there works only until something switches the radio on.

const uint8_t MAX_ARMS  = 3;
const uint8_t ARM_COUNT = sizeof(SERVO_PIN) / sizeof(SERVO_PIN[0]);
const uint8_t LED_PIN   = LED_BUILTIN;   // pin 6 on the MKR boards

// The SAMD Servo library drives any digital pin from a hardware timer rather than from
// analogWrite, so the plain digital pins D3, D4 and D5 are all fine here.

// ---------------------------------------------------------------- motorized fader
//
// The red arm's controller. The fader's built-in feedback pot sits on A1 exactly as a
// plain dial would, but a motor can drive the slider back, so software can set it too.
// Its pot runs on 3.3V, never 5V: the MKR's ADC reference is 3.3V and its analog pins
// are not 5V tolerant.
//
const uint8_t SLIDER_PIN = A1;   // the fader's wiper
const uint8_t POT_ARM = 2;       // red octagon — the arm the fader drives

// HW-354 driver, Motor A. IN2 is on D2 rather than the usual D5: D5 is the blue arm's
// servo signal. D9 has no PWM on the MKR at all, and 8/9/10 are the NINA WiFi SPI bus.
const uint8_t MOTOR_IN1 = 4;
const uint8_t MOTOR_IN2 = 2;

const int MOTOR_SPEED  = 220;    // PWM duty while driving
const int MARGIN_ERROR = 10;     // +/-1% of full scale — close enough, stop here

// Motor noise on the analog line reads as movement, and defeats any naive "has it
// moved?" check. Eight samples measured +/-1 count; a single read swung +/-15.
const uint8_t SLIDER_SAMPLES = 8;

// The real backstop: without it an unreachable target means pushing into a mechanical
// stop at full duty forever.
const unsigned long MOVE_TIMEOUT_MS = 8000;

// Below this much measured travel the mapping is nonsense and we pass values through.
const int MIN_TRAVEL = 100;

// ---- how the fader's travel divides into the four levels --------------------------
//
// The fader does NOT cover the full ADC range: measured travel is about 274..762, so
// raw readings never reach either end of 0..1023. Applying 0..1023 thresholds directly
// would put "Off" below anything the slider can physically reach.
//
// So every position is worked in NORMALISED units — 0..1023 stretched across the
// MEASURED travel — and the thresholds below keep their original meaning. Travel is
// split into four equal quarters:
//
//   level     normalised    raw counts     the slider, physically
//   Off         0 - 256      274 - 396     bottom quarter
//   Low       256 - 512      396 - 518     second quarter
//   Medium    512 - 768      518 - 640     third quarter
//   High      768 - 1023     640 - 762     top quarter
//
// A commanded level parks at the MIDDLE of its quarter, never on a boundary where
// noise could tip it into the neighbour:
//
//   Off 128 -> raw 335    Low 384 -> raw 457
//   Med 640 -> raw 579    High 896 -> raw 701
//
const int SLIDER_MIN = 274;   // measured; boot calibration will replace these
const int SLIDER_MAX = 762;

const int POT_BOUND[3]   = { 256, 512, 768 };   // quarter boundaries, normalised
const int POT_HYSTERESIS = 25;                  // dead zone so noise can't flicker a band
const int BAND_TARGET[4] = { 128, 384, 640, 896 };   // mid-quarter parking spots


// ---------------------------------------------------------------- tuning

// Resting pose — the middle of each arm's swing. State changes how an arm MOVES, not
// where it sits. Three entries, so the others are ready when you add them.
const int CENTER_ANGLE[MAX_ARMS] = { 60, 96, 93 };

// Hard clamp on every movement. Keep inside whatever the linkage can physically reach.
const int ANGLE_MIN = 10;
const int ANGLE_MAX = 170;

enum State { ST_LOW = 0, ST_MED = 1, ST_HIGH = 2, STATE_COUNT = 3 };
const char *STATE_NAME[STATE_COUNT] = { "LOW", "MED", "HIGH" };

// The whole design lives in these two tables. Amplitude is degrees either side of
// centre — the swing is symmetrical, so the arm travels this many degrees each way
// from CENTER_ANGLE; rate is radians per second.
//                                          LOW    MED    HIGH
// Live-tunable so a state can be dialled in against the real object without a
// re-upload — see the V: command. Whatever you settle on, paste it back here.
float stateAmp[STATE_COUNT]  = { 10.0,  20.0,  35.0 };  // degrees either side of centre
float stateRate[STATE_COUNT] = { 0.35,  0.90,  1.10 };  // radians per second

// Seconds to cross from one state's amplitude/speed to another's. A state change is a
// mood change, not a switch — it should be readable as it happens.
const float STATE_BLEND_SEC = 1.4;

// Speed limit on every movement, degrees per second. This is what makes the arm read
// as alive rather than as machinery; lower is heavier and more reluctant.
const float SLEW_DEG_PER_SEC = 140.0;

// Seconds per state in the standalone demo cycle.
const unsigned long DWELL_MS = 6000;

// ---------------------------------------------------------------- state

Servo servos[MAX_ARMS];
bool  servosAttached[MAX_ARMS] = { false, false, false };

// Everything below is per arm.
State armState[MAX_ARMS]   = { ST_MED, ST_MED, ST_MED };
bool  armRunning[MAX_ARMS] = { false, false, false };
float armAngle[MAX_ARMS];       // what the servo is actually holding
float armAmp[MAX_ARMS]  = { 0, 0, 0 };   // smoothed toward the state's amplitude

// One shared phase clock per STATE, not per arm — every arm currently in a given
// state reads the same clock, so any two arms sharing a state are always in lock
// step, however and whenever each one joined it. It runs continuously, whether or
// not any arm is using it right now, so a newly-joining arm always lines up with
// whatever's already swinging in that state instead of restarting the cycle.
float statePhase[STATE_COUNT] = { 0, 0, 0 };

bool autoCycle = true;      // stands down as soon as a command arrives
bool holdCenter = false;    // park at CENTER_ANGLE while setting the resting pose
uint8_t cycleState = 0;
unsigned long lastSwitch = 0;

char line[32];
uint8_t lineLen = 0;
unsigned long lastTick = 0, lastReport = 0;

// ---------------------------------------------------------------- led

// On/off durations in ms, alternating, starting with ON. A zero ends the pattern.
// Handshake: three quick blinks, a beat, then one long one — unmistakable across a
// room, and distinct from the bootloader's own flicker at reset.
const uint16_t PATTERN_HELLO[] = { 90, 90, 90, 90, 90, 300, 700, 0 };

uint16_t ledPattern[12];
uint8_t  ledSteps = 0, ledStep = 0;
unsigned long ledStepStart = 0;
bool ledPlaying = false;

void playPattern(const uint16_t *p) {
  const uint8_t cap = sizeof(ledPattern) / sizeof(ledPattern[0]);
  ledSteps = 0;
  while (p[ledSteps] != 0 && ledSteps < cap) { ledPattern[ledSteps] = p[ledSteps]; ledSteps++; }
  ledStep = 0;
  ledStepStart = millis();
  ledPlaying = ledSteps > 0;
}

bool anyRunning() {
  for (uint8_t i = 0; i < ARM_COUNT; i++) if (armRunning[i]) return true;
  return false;
}

void updateLed() {
  if (ledPlaying) {
    if (millis() - ledStepStart >= ledPattern[ledStep]) {
      ledStepStart = millis();
      if (++ledStep >= ledSteps) ledPlaying = false;
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

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Case-insensitive compare. Written out rather than using strcasecmp, which the AVR
// and SAMD cores expose from different headers.
bool eq(const char *a, const char *b) {
  while (*a && *b) { if (toupper(*a) != toupper(*b)) return false; a++; b++; }
  return *a == *b;
}

// Returns STATE_COUNT if the name is not one of ours.
uint8_t parseState(const char *n) {
  for (uint8_t i = 0; i < STATE_COUNT; i++) if (eq(n, STATE_NAME[i])) return i;
  if (eq(n, "MEDIUM")) return ST_MED;   // the page spells it out; accept both
  return STATE_COUNT;
}

void attachArm(uint8_t i, bool on) {
  if (on == servosAttached[i]) return;
  if (on) servos[i].attach(SERVO_PIN[i]);
  else    servos[i].detach();
  servosAttached[i] = on;
}

void startArm(uint8_t i, State s) {
  // Starting from stopped: take the state's amplitude immediately rather than fading
  // it in, so a tap moves the motor now instead of in a second (the slew limiter still
  // walks it there rather than snapping). No phase to reset any more — reading straight
  // off statePhase[s] is what puts this arm in lock step with any other arm already in
  // state s. Switching states on an arm already running leaves amplitude alone, so it
  // eases across without jumping; the phase read simply switches to the new state's
  // clock immediately, which is what re-syncs it to that state's other arms.
  if (!armRunning[i]) armAmp[i] = stateAmp[s];
  armState[i] = s;
  armRunning[i] = true;
  holdCenter = false;
  attachArm(i, true);
}

// Both skip POT_ARM: the fader is red's controller, and the standalone demo has no
// business motoring a slider the user may have their hand on.
void startAll(State s) { for (uint8_t i = 0; i < ARM_COUNT; i++) if (i != POT_ARM) startArm(i, s); }

void stopArm(uint8_t i) { armRunning[i] = false; }   // eases home and holds there, powered
void stopAll() {
  for (uint8_t i = 0; i < ARM_COUNT; i++) if (i != POT_ARM) stopArm(i);
  holdCenter = false;
}

// ---------------------------------------------------------------- fader

// Motor control. The HW-354 has no enable pin: direction AND speed both come from
// IN1/IN2.
//
//   IN1   IN2   result
//   LOW   LOW   coast — free to move by hand
//   PWM   LOW   forward at duty
//   LOW   PWM   backward at duty
//   HIGH  HIGH  brake — resists movement, holds position
//
// Everything here uses analogWrite(), never digitalWrite(). On SAMD21 analogWrite()
// re-muxes the pin to a timer peripheral, and a later digitalWrite() on that same pin
// is silently ignored until pinMode() is called again — mixing the two leaves the motor
// stuck on. analogWrite(pin, 0) is LOW and analogWrite(pin, 255) is HIGH.
void motorCoast()             { analogWrite(MOTOR_IN1, 0);   analogWrite(MOTOR_IN2, 0); }
void motorBrake()             { analogWrite(MOTOR_IN1, 255); analogWrite(MOTOR_IN2, 255); }
void motorForward(int speed)  { analogWrite(MOTOR_IN2, 0);   analogWrite(MOTOR_IN1, speed); }
void motorBackward(int speed) { analogWrite(MOTOR_IN1, 0);   analogWrite(MOTOR_IN2, speed); }

// Stretch a raw reading across the measured travel, and back again.
int sliderNorm(int raw) {
  long span = SLIDER_MAX - SLIDER_MIN;
  if (span < MIN_TRAVEL) return raw;            // travel implausible — pass it through
  long v = (long)(raw - SLIDER_MIN) * 1023 / span;
  return (int)(v < 0 ? 0 : (v > 1023 ? 1023 : v));
}

int sliderRawFor(int norm) {
  long span = SLIDER_MAX - SLIDER_MIN;
  if (span < MIN_TRAVEL) return norm;
  if (norm < 0) norm = 0;
  if (norm > 1023) norm = 1023;
  return SLIDER_MIN + (int)((long)norm * span / 1023);
}

int readSlider() {
  long sum = 0;
  for (uint8_t i = 0; i < SLIDER_SAMPLES; i++) sum += analogRead(SLIDER_PIN);
  return (int)(sum / SLIDER_SAMPLES);
}

// -1 means "not read yet" — that forces the very first call to settle on whatever band
// the fader is actually sitting at, rather than assuming it starts at Off.
int8_t potBand = -1;
int sliderPos = 0;                 // last averaged reading, for telemetry
int faderTarget = -1;              // -1 = idle and coasting, so hands always win
unsigned long seekStart = 0;

// Apply a band to the red arm. Band 0 is Off; 1/2/3 are LOW/MED/HIGH.
void applyBand(int8_t band) {
  if (band == potBand) return;
  potBand = band;
  if (band == 0) stopArm(POT_ARM);
  else           startArm(POT_ARM, (State)(band - 1));
}

int8_t bandFor(int reading, int8_t current) {
  int8_t band = (current < 0) ? 0 : current;
  while (band < 3 && reading > POT_BOUND[band] + POT_HYSTERESIS) band++;
  while (band > 0 && reading < POT_BOUND[band - 1] - POT_HYSTERESIS) band--;
  return band;
}

// Start driving the fader somewhere. Coasting resumes the moment it arrives or gives up.
void faderSeek(int target) {
  faderTarget = target < 0 ? 0 : (target > 1023 ? 1023 : target);
  seekStart = millis();
}

void faderSeekBand(int8_t band) { faderSeek(sliderRawFor(BAND_TARGET[band])); }

void updateFader() {
  sliderPos = readSlider();

  // Idle: motor coasting, so the fader is free under your fingers and whatever position
  // it is left at becomes red's level. This is the normal state.
  if (faderTarget < 0) {
    motorCoast();
    applyBand(bandFor(sliderNorm(sliderPos), potBand));
    return;
  }

  // Seeking. Give up rather than push into an end stop at full duty forever.
  if (millis() - seekStart >= MOVE_TIMEOUT_MS) {
    motorCoast();
    faderTarget = -1;
    Serial.println(F("ERR:fader seek timed out"));
    applyBand(bandFor(sliderNorm(sliderPos), potBand));
    return;
  }

  if (sliderPos > faderTarget + MARGIN_ERROR) {
    motorBackward(MOTOR_SPEED);
  } else if (sliderPos < faderTarget - MARGIN_ERROR) {
    motorForward(MOTOR_SPEED);
  } else {
    // Arrived. Coast rather than brake — a braked fader feels dead to the touch.
    motorCoast();
    faderTarget = -1;
    applyBand(bandFor(sliderNorm(sliderPos), potBand));
  }
}

// Any command from outside takes the object off its demo cycle.
void takeControl() { autoCycle = false; }

// ---------------------------------------------------------------- motion

void updateArms(float dt) {
  // Every state's clock advances every tick, whether or not an arm is currently using
  // it — that's what lets an arm joining a state mid-cycle land in step immediately
  // instead of resetting the state's cycle to zero for everyone already in it.
  for (uint8_t s = 0; s < STATE_COUNT; s++) statePhase[s] += stateRate[s] * dt;

  float k = clampf(dt / STATE_BLEND_SEC, 0.0, 1.0);
  float maxStep = SLEW_DEG_PER_SEC * dt;

  for (uint8_t i = 0; i < ARM_COUNT; i++) {
    State s = armState[i];
    bool moving = armRunning[i] && !holdCenter;

    // Ease amplitude toward this arm's state instead of jumping, so a swing-width
    // change reads as a transition rather than a snap. Phase isn't eased per arm at
    // all any more — it's read straight from that state's shared clock, which is
    // exactly what keeps every arm in a state moving as one.
    float wantAmp = moving ? stateAmp[s] : 0.0;   // stopping fades the swing out
    armAmp[i] += (wantAmp - armAmp[i]) * k;

    float target = moving
      ? CENTER_ANGLE[i] + sin(statePhase[s]) * armAmp[i]
      : CENTER_ANGLE[i];
    target = clampf(target, ANGLE_MIN, ANGLE_MAX);

    // slew limit so the arm never snaps
    float delta = target - armAngle[i];
    if (delta >  maxStep) delta =  maxStep;
    if (delta < -maxStep) delta = -maxStep;
    armAngle[i] += delta;

    if (servosAttached[i]) servos[i].write((int)(armAngle[i] + 0.5));

    // Deliberately never auto-releases at rest. A released servo goes limp, and
    // gravity pulls the arm's own weight off CENTER_ANGLE — exactly the "vertical at
    // rest" calibration this object depends on. Holding torque at idle costs a little
    // current and warmth in exchange for staying rigidly in place. Detach by hand with
    // E:0 (or the web page's controls) if you need the linkage to move freely, e.g.
    // while re-taping an arm.
  }
}

// ---------------------------------------------------------------- serial

void report() {
  Serial.print(F("S:"));
  for (uint8_t i = 0; i < ARM_COUNT; i++) {
    if (i) Serial.print(';');
    Serial.print(STATE_NAME[armState[i]]);
    Serial.print(',');
    Serial.print(armRunning[i] ? 1 : 0);
    Serial.print(',');
    Serial.print((int)(armAngle[i] + 0.5));
  }
  Serial.println();
}

// Single characters typed into the Serial Monitor. Returns true if handled.
bool handleKey(char c) {
  switch (c) {
    case '1': takeControl(); startAll(ST_LOW);
              Serial.println(F("OK:key all LOW")); return true;
    case '2': takeControl(); startAll(ST_MED);
              Serial.println(F("OK:key all MED")); return true;
    case '3': takeControl(); startAll(ST_HIGH);
              Serial.println(F("OK:key all HIGH")); return true;
    case '0': takeControl(); stopAll();
              Serial.println(F("OK:key stop all")); return true;
    case 'a': autoCycle = true; holdCenter = false; lastSwitch = 0;
              Serial.println(F("OK:key demo cycle resumed")); return true;
    case 'c': takeControl(); holdCenter = true;
              for (uint8_t i = 0; i < ARM_COUNT; i++) { armRunning[i] = true; attachArm(i, true); }
              Serial.println(F("OK:key holding CENTER_ANGLE")); return true;
    default:  return false;
  }
}

void handleLine(char *s) {
  if (s[1] == '\0' && handleKey(s[0])) return;   // a bare single character

  char kind = toupper(s[0]);

  if (kind == '?') { report(); return; }

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
      int idx = atoi(s + 2);
      if (idx < 0 || idx >= ARM_COUNT) { Serial.println(F("ERR:arm out of range")); return; }
      if (idx == POT_ARM) faderSeekBand(0);   // stop red = drive the fader down to Off
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
    char *rest = s + 2;
    char *colon = strchr(rest, ':');

    if (colon) {                       // per-arm form
      *colon = '\0';
      int idx = atoi(rest);
      uint8_t st = parseState(colon + 1);
      if (idx < 0 || idx >= ARM_COUNT) { Serial.println(F("ERR:arm out of range")); return; }
      // The fader arm is commandable now: set the state AND drive the slider to match,
      // so the physical control never disagrees with what the software thinks red is.
      if (idx == POT_ARM) faderSeekBand(st + 1);
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

  // V:<state>:<amp>:<rate> — retune a state live, no re-upload. e.g. V:MED:24:0.7
  // V alone prints the current table, ready to paste back into the sketch.
  if (kind == 'V') {
    if (s[1] == '\0') {
      for (uint8_t i = 0; i < STATE_COUNT; i++) {
        Serial.print(F("V:")); Serial.print(STATE_NAME[i]);
        Serial.print(F(" amp=")); Serial.print(stateAmp[i], 1);
        Serial.print(F(" rate=")); Serial.println(stateRate[i], 2);
      }
      return;
    }
    if (s[1] != ':') { Serial.println(F("ERR:use V or V:<state>:<amp>:<rate>")); return; }

    char *p1 = strchr(s + 2, ':');
    if (!p1) { Serial.println(F("ERR:V needs state, amp and rate")); return; }
    *p1 = '\0';
    char *p2 = strchr(p1 + 1, ':');
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

  // F:<0-1023> — drive the fader to a raw position
  if (kind == 'F' && s[1] == ':') {
    takeControl();
    int target = atoi(s + 2);
    if (target < 0 || target > 1023) { Serial.println(F("ERR:fader position 0-1023")); return; }
    faderSeek(sliderRawFor(target));   // normalised in, raw counts out
    Serial.print(F("OK:F ")); Serial.print(target);
    Serial.print(F(" raw=")); Serial.println(sliderRawFor(target));
    return;
  }

  if (kind == 'E' && s[1] == ':') {
    for (uint8_t i = 0; i < ARM_COUNT; i++) attachArm(i, s[2] != '0');
    Serial.print(F("OK:E ")); Serial.println(s[2] != '0' ? 1 : 0);
    return;
  }

  Serial.println(F("ERR:unknown command"));
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) { line[lineLen] = '\0'; handleLine(line); lineLen = 0; }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
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
    armAngle[i] = CENTER_ANGLE[i];
  }

  playPattern(PATTERN_HELLO);   // says "I booted" without blocking the loop

  Serial.print(F("OK:READY social-battery ARMS="));
  Serial.println(ARM_COUNT);
  Serial.println(F("keys: 1/2/3 = low/med/high, 0 = stop, a = demo cycle, c = hold centre"));
  Serial.print(F("fader travel ")); Serial.print(SLIDER_MIN);
  Serial.print(F("..")); Serial.print(SLIDER_MAX);
  Serial.println(F(" raw, split into four equal quarters:"));
  for (uint8_t b = 0; b < 4; b++) {
    Serial.print(F("  "));
    Serial.print(b == 0 ? "Off   " : STATE_NAME[b - 1]);
    Serial.print(F("\traw "));
    Serial.print(b == 0 ? SLIDER_MIN : sliderRawFor(POT_BOUND[b - 1]));
    Serial.print(F(" - "));
    Serial.print(b == 3 ? SLIDER_MAX : sliderRawFor(POT_BOUND[b]));
    Serial.print(F("\tparks at "));
    Serial.println(sliderRawFor(BAND_TARGET[b]));
  }

  lastTick = millis();
  lastSwitch = 0;               // start the demo cycle immediately
}

void loop() {
  readSerial();
  updateLed();
  updateFader();

  unsigned long now = millis();

  // Standalone demo: walk every arm through the states until something takes over.
  if (autoCycle && (lastSwitch == 0 || now - lastSwitch >= DWELL_MS)) {
    if (lastSwitch != 0) cycleState = (cycleState + 1) % STATE_COUNT;
    lastSwitch = now;
    startAll((State)cycleState);
    Serial.print(F("demo -> ")); Serial.println(STATE_NAME[cycleState]);
  }

  float dt = (now - lastTick) / 1000.0;
  if (dt >= 0.02) {            // 50 Hz motion update
    lastTick = now;
    updateArms(dt);
  }

  if (now - lastReport >= 100) {   // 10 Hz telemetry
    lastReport = now;
    report();
    // Fader state on its own line, so the S: format stays exactly as it was.
    // FD:<normalised>,<band>,<seeking>,<raw> — normalised leads so the page works in
    // one consistent 0-1023 space and never has to know the fader's real travel.
    Serial.print(F("FD:"));
    Serial.print(sliderNorm(sliderPos));
    Serial.print(',');
    Serial.print(potBand < 0 ? 0 : potBand);
    Serial.print(',');
    Serial.print(faderTarget >= 0 ? 1 : 0);
    Serial.print(',');
    Serial.println(sliderPos);
  }
}
