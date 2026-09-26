/*
 * Social Battery — servo arm MAX SPEED test (D7 only)
 * Tangible User Interface, CIID
 *
 * A dedicated speed-ceiling test, not a level demo. Starts swinging at full
 * amplitude (45 degrees, same as HIGH) the moment it boots, governed by
 * ARM_SLEW_DEG_PER_SEC below - the hard cap on how fast the angle is allowed
 * to change per tick, in degrees/second.
 *
 * WHERE THE NUMBER COMES FROM
 *
 *   The servo's own datasheet (Miuzei MS18, 9g) rates it at ~0.09 sec/60 deg
 *   unloaded at 4.8V (~667 deg/s) - no separate 5V figure was published. Under
 *   this arm's actual load/linkage, real usable speed is almost certainly
 *   lower than that unloaded figure; a rough planning estimate is ~400-500
 *   deg/s, not a measured one.
 *
 *   ARM_SLEW_DEG_PER_SEC is set to 300 deg/s here - comfortably under that
 *   loaded estimate, and over 2x the object's normal "alive, not snapping"
 *   cap of 140 deg/s. It's a deliberate first step toward the ceiling, not
 *   the ceiling itself - raise it further only after confirming 300 is fine
 *   on the actual hardware (torque, heat, linkage stress).
 *
 *   ARM_MAX_RATE (the underlying triangle-wave rate) is set high enough that
 *   its own uncapped peak speed (~430 deg/s) already exceeds the slew cap -
 *   so ARM_SLEW_DEG_PER_SEC is what actually governs the real motion, not
 *   the rate. That makes the top speed predictable and tunable from one place.
 *
 * SERIAL, 115200 - one command per line, Enter to send
 *
 *   max          (re)start the max-speed swing - this is also the boot default
 *   center       arm moves to ARM_CENTER_ANGLE and holds
 *   go <deg>     arm holds a fixed angle directly, e.g. "go 120"
 *   stop         arm eases back to ARM_CENTER_ANGLE and holds there
 *
 *   Status prints every 500ms: current angle, running or held.
 *
 * WIRING
 *
 *   Servo signal  →  D7
 *   Servo power   →  EXTERNAL 5V supply, ground tied to the board's - the
 *                     board's own regulator cannot reliably source servo current.
 *
 * Board: Arduino UNO R4 WiFi
 */

#include <Servo.h>

// ---- Servo arm --------------------------------------------------------------
#define SERVO_PIN 7
#define ARM_CENTER_ANGLE 93   // resting vertical, measured by hand
#define ARM_ANGLE_MIN 10      // hard clamp - keep inside what the linkage can physically reach
#define ARM_ANGLE_MAX 210

#define ARM_MAX_AMP 45.0     // degrees either side of centre - same as HIGH elsewhere
#define ARM_MAX_RATE 15.0    // radians/second - deliberately far past the slew cap, see header
#define ARM_STATE_BLEND_SEC 1.4

// The actual governor. Start here; raise only after confirming this is fine
// on the real hardware. See header for how this number was chosen.
#define ARM_SLEW_DEG_PER_SEC 300.0

Servo armServo;
bool armRunning = false;
bool armHeld = false;
float armAmp = 0;
float armAngle = ARM_CENTER_ANGLE;
float armPhase = 0;
unsigned long armLastTick = 0;
unsigned long lastPrint = 0;

void startMax() {
  if (!armRunning) armAmp = ARM_MAX_AMP;
  armRunning = true;
  armHeld = false;
  Serial.println("ARM -> MAX");
}

void updateArm(float dt) {
  if (armHeld) return;

  armPhase += ARM_MAX_RATE * dt;

  float k = dt / ARM_STATE_BLEND_SEC;
  if (k > 1.0) k = 1.0;
  float wantAmp = armRunning ? ARM_MAX_AMP : 0.0;
  armAmp += (wantAmp - armAmp) * k;

  float cycle = fmod(armPhase, TWO_PI);
  if (cycle < 0) cycle += TWO_PI;
  float tri = (cycle < PI) ? (cycle / PI) * 2.0 - 1.0
                           : 1.0 - ((cycle - PI) / PI) * 2.0;

  float target = armRunning ? (ARM_CENTER_ANGLE + tri * armAmp) : (float)ARM_CENTER_ANGLE;
  target = constrain(target, ARM_ANGLE_MIN, ARM_ANGLE_MAX);

  float maxStep = ARM_SLEW_DEG_PER_SEC * dt;   // this is what actually caps real speed
  float delta = target - armAngle;
  if (delta > maxStep) delta = maxStep;
  if (delta < -maxStep) delta = -maxStep;
  armAngle += delta;

  armServo.write((int)(armAngle + 0.5));
}

bool eqIgnoreCase(const char* a, const char* b) {
  while (*a && *b) { if (toupper(*a) != toupper(*b)) return false; a++; b++; }
  return *a == *b;
}

void handleCommand(char* cmd) {
  char* sp = strchr(cmd, ' ');
  if (sp) *sp = '\0';
  const char* arg = sp ? sp + 1 : "";

  if (eqIgnoreCase(cmd, "max")) {
    startMax();
  } else if (eqIgnoreCase(cmd, "center")) {
    armHeld = true;
    armRunning = false;
    armAngle = ARM_CENTER_ANGLE;
    armServo.write(ARM_CENTER_ANGLE);
    Serial.println("ARM centered");
  } else if (eqIgnoreCase(cmd, "go") && arg[0] != '\0') {
    armAngle = constrain((float)atof(arg), (float)ARM_ANGLE_MIN, (float)ARM_ANGLE_MAX);
    armHeld = true;
    armRunning = false;
    armServo.write((int)(armAngle + 0.5));
    Serial.print("ARM holding ");
    Serial.println(armAngle);
  } else if (eqIgnoreCase(cmd, "stop")) {
    armRunning = false;
    armHeld = false;
    Serial.println("ARM -> stopping, easing back to centre");
  } else {
    Serial.print("ERR: unknown command '");
    Serial.print(cmd);
    Serial.println("'");
  }
}

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
  Serial.begin(115200);   // no `while (!Serial)` - must still run with no computer attached

  armServo.attach(SERVO_PIN);
  armServo.write(ARM_CENTER_ANGLE);
  armAngle = ARM_CENTER_ANGLE;
  armLastTick = millis();

  Serial.println();
  Serial.println("=== Social Battery - servo MAX SPEED test (D7) ===");
  Serial.print("slew cap: ");
  Serial.print(ARM_SLEW_DEG_PER_SEC);
  Serial.println(" deg/s - this is the real ceiling being tested");
  Serial.println("starting MAX swing now - commands: max | center | go <deg> | stop");

  startMax();   // boot straight into the max-speed swing, no command needed
}

void loop() {
  readSerial();

  unsigned long now = millis();
  float dt = (now - armLastTick) / 1000.0;
  if (dt >= 0.02) {
    armLastTick = now;
    updateArm(dt);
  }

  if (now - lastPrint >= 500) {
    lastPrint = now;
    Serial.print("angle: ");
    Serial.print(armAngle);
    Serial.print("  ");
    Serial.println(armHeld ? "(held)" : (armRunning ? "(swinging)" : "(stopped)"));
  }
}
