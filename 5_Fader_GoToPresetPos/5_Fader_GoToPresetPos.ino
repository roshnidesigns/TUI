/*
* Tangible User Interface class - CIID
* Written by Jose Chavarria
* Teaching with Massimo Banzi and Pierluigi Dalla Rosa
* 
* This sketch introduces you to controlling a motorized slider
* The user is able to move the slider to a pre fixed position
* stored in a position array. 
* By pressing a button, the user is able to navigate the array.
*/

// Pin Definitions
#define MOTOR_IN1 7    // MX1508 IN1 (Motor A) - direction + speed
#define MOTOR_IN2 5    // MX1508 IN2 (Motor A) - direction + speed
#define SLIDER_PIN A1  // Analog input from slider's feedback potentiometer

// Control Parameters
#define MOTOR_SPEED 220                // Default motor speed (PWM value 0-255)
#define MAX_SLIDER 1023                // Maximum value from analog read
#define MARGIN_ERROR MAX_SLIDER / 100  // Acceptable position error (±1% of max range)
#define PRINT_INTERVAL 200             // How often to print debug values (ms)

// Function Declarations
void motorStop();
void motorForward(int speed);
void motorBackward(int speed);

// Predefined positions for the slider (0-1023 range)
const int PRESET_POSITIONS[] = { 400, 600, 400, 600, 400 };
const int ARRAY_SIZE = sizeof(PRESET_POSITIONS) / sizeof(PRESET_POSITIONS[0]);

// Which preset the slider seeks. No button is wired yet, so this stays fixed.
// Change the index to pick a different target: 0=0, 1=750, 2=200, 3=500, 4=1000
int currentTarget = 1;

unsigned long lastPrint = 0;  // Timestamp of the last debug print

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Configure input/output pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // Ensure motor is stopped when the system starts
  motorStop();
}

void loop() {
  // Get current target position from preset array
  int targetVal = PRESET_POSITIONS[currentTarget];

  // Read current position from slider feedback
  int sliderVal = analogRead(SLIDER_PIN);

  // Print position vs target periodically so wiring can be checked on the
  // Serial Monitor. Never use `while (!Serial)` here - it would stall the
  // sketch forever whenever the board runs without a computer attached.
  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();
    Serial.print("slider: ");
    Serial.print(sliderVal);
    Serial.print("  target: ");
    Serial.println(targetVal);
  }

  // Compare current position with target, accounting for acceptable error range
  if (sliderVal > targetVal + MARGIN_ERROR) {
    // Current position is too far forward - move backward
    motorBackward(MOTOR_SPEED);
  } else if (sliderVal < targetVal - MARGIN_ERROR) {
    // Current position is too far back - move forward
    motorForward(MOTOR_SPEED);
  } else {
    // Within acceptable range - stop motor
    motorStop();
    currentTarget++;
    if (currentTarget > 4){
      currentTarget = 0;
    }
  }
}

// Motor Control Functions
//
// The MX1508 (HW-354) has no enable pin: direction AND speed both come from
// IN1/IN2. Hold one input LOW and PWM the other to drive at that speed.
//
//   IN1   IN2   result
//   LOW   LOW   coast
//   PWM   LOW   forward at duty
//   LOW   PWM   backward at duty
//   HIGH  HIGH  brake
//
// Everything below uses analogWrite() rather than digitalWrite(). On the
// MKR 1010 (SAMD21) analogWrite() re-muxes the pin to a timer peripheral, and
// a later digitalWrite() on that same pin is ignored until pinMode() is called
// again. Staying on analogWrite() the whole way avoids that trap.

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