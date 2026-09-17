/*
* ============================================================
*  WORKING VERSION - verified on hardware 17 Sep 2026
*  Homing, hand detection and the mirror swing all confirmed
*  running with the driver on an external 5V supply.
*  Known rough edges: overshoots the target by ~13 counts, and
*  occasionally hangs for a few seconds around the 300-430 region.
*  Do not edit this copy - it is the snapshot to fall back to.
* ============================================================
*
* Tangible User Interface class - CIID
* Motorized slider - ping-pong around whatever position you set by hand
*
* On power-up the slider drives itself to the centre of travel, so it always
* starts from a known place. From there it swings continuously between a
* detected position x and the mirror of x across the travel.
*
* Grab the slider mid-swing and push it somewhere, and it notices - the motor
* lets go, waits for your hand to settle, then rebuilds the swing around the
* new position. Let go near the bottom and it swings low-to-high; let go near
* the middle and the swing narrows.
*
* The mirror of x in the range [a, b] is (a + b - x). With SLIDER_MIN at 0
* that is simply SLIDER_MAX - x.
*
* Board:  Arduino MKR WiFi 1010
* Driver: HW-354, Motor A on IN1/IN2
*/

// Pin Definitions
#define MOTOR_IN1 4    // HW-354 IN1 (Motor A) - direction + speed
#define MOTOR_IN2 5    // HW-354 IN2 (Motor A) - direction + speed
#define SLIDER_PIN A1  // Analog input from slider's feedback potentiometer

// ---- Travel limits ---------------------------------------------------------
#define SLIDER_MIN 0        // a - reading at one mechanical stop
#define SLIDER_MAX 1023     // b - reading at the other
#define CENTER_POSITION 500 // Where it homes to on power-up

// ---- Control Parameters ----------------------------------------------------
#define MOTOR_SPEED 220                // Default motor speed (PWM value 0-255)
#define MAX_SLIDER 1023                // Maximum value from analog read
#define MARGIN_ERROR MAX_SLIDER / 100  // Acceptable position error (±1% of max)
#define PRINT_INTERVAL 200             // How often to print debug values (ms)
#define MOVE_TIMEOUT 8000              // Give up on an end rather than push into
                                       // a mechanical stop forever
#define SAMPLES 8                      // analogRead samples averaged per reading

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

// The two ends of the swing, both derived from the detected position
int pointA = 0;     // x - the position you set
int pointB = 0;     // the mirror of x
int targetVal = 0;  // whichever end we are heading for right now

enum Mode { HOMING, SWINGING, GRABBED, IDLE };
Mode mode = HOMING;

int bestGap = 0;     // Closest we have got to the target on this leg
int settleRef = 0;   // Reading the settle timer is measured against
int idleRef = 0;     // Reading we watch for a hand while resting at centre

unsigned long lastPrint = 0;
unsigned long moveStart = 0;
unsigned long settleTime = 0;

const char* statusName() {
  if (mode == HOMING)   return "HOMING ";
  if (mode == IDLE)     return "RESTING";
  if (mode == GRABBED)  return "HAND   ";
  return "SWINGING";
}

// Average several samples. A single analogRead picks up motor noise, which
// would otherwise look like a hand on the slider.
int readSlider() {
  long total = 0;
  for (int i = 0; i < SAMPLES; i++) {
    total += analogRead(SLIDER_PIN);
  }
  return (int)(total / SAMPLES);
}

// Build the swing around a freshly detected position
void detectFrom(int x) {
  pointA = constrain(x, SLIDER_MIN, SLIDER_MAX);
  pointB = constrain(SLIDER_MIN + SLIDER_MAX - pointA, SLIDER_MIN, SLIDER_MAX);

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
    idleRef = pointA;
    targetVal = pointA;   // nothing to chase while resting; report where we sit
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
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Configure input/output pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // Ensure motor is stopped when the system starts
  motorStop();

  // Home to the centre first, so every run begins from the same place
  moveStart = millis();
  mode = HOMING;
  Serial.print("HOMING to ");
  Serial.println(CENTER_POSITION);
}

void loop() {
  int sliderVal = readSlider();

  // Print position vs target periodically so wiring can be checked on the
  // Serial Monitor. Never use `while (!Serial)` here - it would stall the
  // sketch forever whenever the board runs without a computer attached.
  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();

    // Seconds since power-up, to three decimals
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
    Serial.println(mode == HOMING ? CENTER_POSITION : targetVal);
  }

  // ---- Power-up: drive to the centre before doing anything else ----
  if (mode == HOMING) {
    bool timedOut = (millis() - moveStart >= MOVE_TIMEOUT);

    if (abs(CENTER_POSITION - sliderVal) <= MARGIN_ERROR || timedOut) {
      motorStop();
      if (timedOut) {
        Serial.print("HOMING gave up at ");
        Serial.println(sliderVal);
      } else {
        Serial.print("AT CENTER ");
        Serial.println(sliderVal);
      }
      detectFrom(sliderVal);
    } else if (sliderVal > CENTER_POSITION) {
      motorBackward(MOTOR_SPEED);
    } else {
      motorForward(MOTOR_SPEED);
    }
    return;
  }

  // ---- Resting at the centre: motor off, waiting for a hand ----
  if (mode == IDLE) {
    motorCoast();
    if (abs(sliderVal - idleRef) > SETTLE_MOVE) {
      Serial.println("HAND DETECTED - set it where you like");
      mode = GRABBED;
      settleRef = sliderVal;
      settleTime = millis();
    }
    return;
  }

  // ---- Your hand is on it: motor off, wait for you to finish ----
  if (mode == GRABBED) {
    motorCoast();

    if (abs(sliderVal - settleRef) > SETTLE_MOVE) {
      settleRef = sliderVal;
      settleTime = millis();
    } else if (millis() - settleTime >= SETTLE_MS) {
      // Still for long enough - that is where you wanted it
      detectFrom(sliderVal);
      mode = SWINGING;
    }
    return;
  }

  // ---- Swinging between the two ends ----

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
    return;
  }

  // Safety net - if an end cannot be reached, swing back rather than grind
  // against a mechanical stop indefinitely
  if (millis() - moveStart >= MOVE_TIMEOUT) {
    Serial.print("GAVE UP at ");
    Serial.print(sliderVal);
    Serial.print(" chasing ");
    Serial.println(targetVal);
    motorStop();
    swapTarget(sliderVal);
    return;
  }

  // Compare current position with target, accounting for acceptable error range
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
