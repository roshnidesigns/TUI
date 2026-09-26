/*
 * Social Battery — servo arm bring-up test (D7 only)
 * Tangible User Interface, CIID
 *
 * Just the servo arm from FRESH-TRY-ADD-SERVO, on its own - no fader, no
 * slider, no NeoPixel strip. Use this to prove a servo plugged into D7
 * actually moves before wiring the rest of the object back up.
 *
 * SERIAL, 115200 - one command per line, Enter to send
 *
 *   center       arm moves to ARM_CENTER_ANGLE and holds
 *   go <deg>     arm holds a fixed angle directly, e.g. "go 120"
 *   low          arm swings gently (same LOW amplitude/rate as the full sketch)
 *   med          arm swings at MED
 *   high         arm swings at HIGH
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

enum ArmState { ARM_LOW = 0, ARM_MED = 1, ARM_HIGH = 2, ARM_STATE_COUNT = 3 };
const char* ARM_STATE_NAME[ARM_STATE_COUNT] = { "LOW", "MED", "HIGH" };

// Same numbers as the full sketch and FRESH-TRY-ADD-SERVO.
//                                    LOW    MED    HIGH
const float ARM_STATE_AMP[ARM_STATE_COUNT]  = { 10.0,  25.0,  45.0 };   // degrees either side of centre
const float ARM_STATE_RATE[ARM_STATE_COUNT] = { 0.18,  0.45,  0.60 };   // radians/second

#define ARM_STATE_BLEND_SEC 1.4
#define ARM_SLEW_DEG_PER_SEC 140.0

Servo armServo;
bool armRunning = false;
bool armHeld = false;
ArmState armState = ARM_MED;
float armAmp = 0;
float armAngle = ARM_CENTER_ANGLE;
float armPhase = 0;
unsigned long armLastTick = 0;
unsigned long lastPrint = 0;

// Auto-cycle: runs on its own from boot, no serial input needed. Any manual
// command (center/go/low/med/high/stop) takes over and cancels it, same as the
// full sketch's takeControl().
bool autoCycle = true;
const ArmState CYCLE_ORDER[3] = { ARM_LOW, ARM_HIGH, ARM_MED };   // the order you asked for
#define CYCLE_DWELL_MS 10000     // 10s per level
uint8_t cycleIndex = 0;
unsigned long lastSwitch = 0;    // 0 = hasn't started its first leg yet

void startArm(ArmState s) {
  if (!armRunning) armAmp = ARM_STATE_AMP[s];
  armState = s;
  armRunning = true;
  armHeld = false;
  Serial.print("ARM -> ");
  Serial.println(ARM_STATE_NAME[s]);
}

void updateArm(float dt) {
  if (armHeld) return;

  armPhase += ARM_STATE_RATE[armState] * dt;

  float k = dt / ARM_STATE_BLEND_SEC;
  if (k > 1.0) k = 1.0;
  float wantAmp = armRunning ? ARM_STATE_AMP[armState] : 0.0;
  armAmp += (wantAmp - armAmp) * k;

  float cycle = fmod(armPhase, TWO_PI);
  if (cycle < 0) cycle += TWO_PI;
  float tri = (cycle < PI) ? (cycle / PI) * 2.0 - 1.0
                           : 1.0 - ((cycle - PI) / PI) * 2.0;

  float target = armRunning ? (ARM_CENTER_ANGLE + tri * armAmp) : (float)ARM_CENTER_ANGLE;
  target = constrain(target, ARM_ANGLE_MIN, ARM_ANGLE_MAX);

  float maxStep = ARM_SLEW_DEG_PER_SEC * dt;
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

  if (eqIgnoreCase(cmd, "center")) {
    autoCycle = false;   // a recognised manual command takes over from the auto-cycle
    armHeld = true;
    armRunning = false;
    armAngle = ARM_CENTER_ANGLE;
    armServo.write(ARM_CENTER_ANGLE);
    Serial.println("ARM centered");
  } else if (eqIgnoreCase(cmd, "go") && arg[0] != '\0') {
    autoCycle = false;
    armAngle = constrain((float)atof(arg), (float)ARM_ANGLE_MIN, (float)ARM_ANGLE_MAX);
    armHeld = true;
    armRunning = false;
    armServo.write((int)(armAngle + 0.5));
    Serial.print("ARM holding ");
    Serial.println(armAngle);
  } else if (eqIgnoreCase(cmd, "low")) {
    autoCycle = false;
    startArm(ARM_LOW);
  } else if (eqIgnoreCase(cmd, "med") || eqIgnoreCase(cmd, "medium")) {
    autoCycle = false;
    startArm(ARM_MED);
  } else if (eqIgnoreCase(cmd, "high")) {
    autoCycle = false;
    startArm(ARM_HIGH);
  } else if (eqIgnoreCase(cmd, "stop")) {
    autoCycle = false;
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
  Serial.println("=== Social Battery - servo test (D7) ===");
  Serial.print("centered at ");
  Serial.println(ARM_CENTER_ANGLE);
  Serial.println("auto-cycling LOW -> HIGH -> MEDIUM, 10s each, repeating");
  Serial.println("commands: center | go <deg> | low | med | high | stop  (any of these takes over from the auto-cycle)");
}

void loop() {
  readSerial();

  unsigned long now = millis();

  if (autoCycle && (lastSwitch == 0 || now - lastSwitch >= CYCLE_DWELL_MS)) {
    if (lastSwitch != 0) cycleIndex = (cycleIndex + 1) % 3;   // advance to the next level in CYCLE_ORDER
    lastSwitch = now;
    startArm(CYCLE_ORDER[cycleIndex]);
  }

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
