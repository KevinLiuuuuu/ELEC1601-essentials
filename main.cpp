#include <Servo.h>

Servo servoLeft;
Servo servoRight;

// ===================== PINS =====================
// IR LED / receiver pairs
const int irLedLeft = 10, irReceiverLeft = 11;
const int irLedMid = 6, irReceiverMid = 7;
const int irLedRight = 2, irReceiverRight = 3;

// Indicator LEDs (names match the spec: Right = A0, Mid = A1, Left = A2)
const int ledRight = A0;
const int ledMid = A1;
const int ledLeft = A2;

// ===================== SERVO PULSE WIDTHS (microseconds) =====================
// Fill these in from Part 2.2 / 2.3: each wheel's true stop value, and the pair
// of values that makes the robot drive in a straight line.
const int LEFT_STOP = 1500, RIGHT_STOP = 1500;
const int LEFT_FWD = 1600, RIGHT_FWD = 1400;
const int LEFT_BACK = 1400, RIGHT_BACK = 1600;

// ===================== TIMING / DISTANCES (TUNE IN THE LAB) =====================
// Ground speed at LEFT_FWD / RIGHT_FWD from your Part 2.2 table
// (RPM x pi x wheel diameter / 60). 6.1 assumes a 6.5 cm wheel at 18 RPM.
const float DRIVE_SPEED_CM_PER_S = 6.1;

const int ROTATE_90_MS = 700;               // spin time for 90 degrees
const int ROTATE_180_MS = 2 * ROTATE_90_MS;
const int SMALL_ROTATE_MS = 250;            // ~30 degrees / small corrections

const float STEP_CM = 5.0;        // scenario 1: move 5 cm, scenario 9: back up
const float REENTRY_CM = 8.0;     // scenarios 2/3: drive far enough to re-enter the corridor
const float DEAD_END_CM = 6.0;    // scenario 4: "at least 5 cm"
const float ADJUST_CM = 4.0;      // scenarios 5/6: sideways correction

const int DISPLAY_MS = 5000;      // LEDs show the code with the robot still for this long
const bool HALT_AFTER_SCENARIO = true;   // spec: "stop after completion". Set false for the extension maze.

// ===================== SENSOR ZONES =====================
// irDistance() returns 0-5: how many of the 38-42 kHz tones were NOT detected.
// From your calibration table: 0 = within ~5 cm, 4 = ~10 cm, 5 = nothing in range.
const int NO_WALL_ZONE = 5;
const int FAR_ZONE = 4;            // this zone or higher = far wall or no wall
const int NEAR_ZONE = 2;           // this zone or lower = wall is close
const int SIMILAR_DIFF = 1;        // left/right zones within this = "similar reading"
const int SIGNIFICANT_DIFF = 2;    // zone gap that counts as "significantly closer"

// ===================== SENSING =====================
int irDetect(int irLedPin, int irReceiverPin, long frequency) {
  tone(irLedPin, frequency);
  delay(1);
  int ir = digitalRead(irReceiverPin);   // 0 = detected, 1 = not detected
  noTone(irLedPin);
  delay(1);
  return ir;
}

int irDistance(int irLedPin, int irReceiverPin) {
  int distance = 0;
  for (long f = 38000; f <= 42000; f += 1000) {
    distance += irDetect(irLedPin, irReceiverPin, f);
  }
  return distance;
}

// ===================== MOTION =====================
void stopMotors() {
  servoLeft.writeMicroseconds(LEFT_STOP);
  servoRight.writeMicroseconds(RIGHT_STOP);
}

int msForCm(float cm) {
  return (int)(cm / DRIVE_SPEED_CM_PER_S * 1000.0);
}

void moveForward(int ms) {
  servoLeft.writeMicroseconds(LEFT_FWD);
  servoRight.writeMicroseconds(RIGHT_FWD);
  delay(ms);
  stopMotors();
}

void moveBackward(int ms) {
  servoLeft.writeMicroseconds(LEFT_BACK);
  servoRight.writeMicroseconds(RIGHT_BACK);
  delay(ms);
  stopMotors();
}

void rotateClockwise(int ms) {
  servoLeft.writeMicroseconds(LEFT_FWD);
  servoRight.writeMicroseconds(RIGHT_BACK);
  delay(ms);
  stopMotors();
}

void rotateAnticlockwise(int ms) {
  servoLeft.writeMicroseconds(LEFT_BACK);
  servoRight.writeMicroseconds(RIGHT_FWD);
  delay(ms);
  stopMotors();
}

void moveForwardCm(float cm) { moveForward(msForCm(cm)); }
void moveBackwardCm(float cm) { moveBackward(msForCm(cm)); }

void finishScenario() {
  stopMotors();
  if (HALT_AFTER_SCENARIO) {
    while (true) {
      delay(1000);
    }
  }
}

// ===================== LEDS =====================
// Argument order matches the spec: Right (A0), Mid (A1), Left (A2)
void setLEDs(int right, int mid, int left) {
  digitalWrite(ledRight, right);
  digitalWrite(ledMid, mid);
  digitalWrite(ledLeft, left);
}

// Show a steady binary code with the robot still
void showCode(int right, int mid, int left) {
  stopMotors();
  setLEDs(right, mid, left);
  delay(DISPLAY_MS);
}

// Flash one LED (1 s on, 1 s off) for at least DISPLAY_MS with the robot still
void flashCode(int ledPin) {
  stopMotors();
  setLEDs(LOW, LOW, LOW);
  for (int i = 0; i < DISPLAY_MS / 2000 + 1; i++) {
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
  }
}

// ===================== SCENARIO DETECTION =====================
// Returns the scenario number that matches the sensors, or 0 (unknown).
// Order matters: the more specific scenarios are checked first.
int detectScenario() {
  int leftZone = irDistance(irLedLeft, irReceiverLeft);
  int midZone = irDistance(irLedMid, irReceiverMid);
  int rightZone = irDistance(irLedRight, irReceiverRight);

  bool leftWall = leftZone < NO_WALL_ZONE;
  bool rightWall = rightZone < NO_WALL_ZONE;

  bool leftNear = leftZone <= NEAR_ZONE;
  bool midNear = midZone <= NEAR_ZONE;
  bool rightNear = rightZone <= NEAR_ZONE;

  // Front: a wall closer than ~10 cm counts as "wall ahead", anything else is clear
  // (spec: scenario 1 needs "no wall, or a wall at least 10 cm away").
  bool midWall = midZone < FAR_ZONE;
  bool frontClear = !midWall;

  // 7: angled toward the left wall - left and front walls close, right side far/open
  if (leftNear && midNear && rightZone >= FAR_ZONE) {
    return 7;
  }

  // 8: angled toward the right wall - right and front walls close, left side far/open
  if (rightNear && midNear && leftZone >= FAR_ZONE) {
    return 8;
  }

  // 9 (custom): dead-end shape but hugging the left wall - back up first, then re-detect
  if (leftNear && midWall && rightWall && !rightNear) {
    return 9;
  }

  // 4: dead end - walls on all three sides
  if (leftWall && midWall && rightWall) {
    return 4;
  }

  // 2: right turn - right open, left and front walls
  if (rightZone == NO_WALL_ZONE && leftWall && midWall) {
    return 2;
  }

  // 3: left turn - left open, right and front walls
  if (leftZone == NO_WALL_ZONE && rightWall && midWall) {
    return 3;
  }

  // 1: middle of a corridor - both walls at a similar distance, front clear
  if (leftWall && rightWall && frontClear &&
      abs(leftZone - rightZone) <= SIMILAR_DIFF) {
    return 1;
  }

  // 5: close to the left wall, parallel to it, front clear
  if (leftWall && rightWall && frontClear &&
      leftZone + SIGNIFICANT_DIFF <= rightZone) {
    return 5;
  }

  // 6: close to the right wall, parallel to it, front clear
  if (leftWall && rightWall && frontClear &&
      rightZone + SIGNIFICANT_DIFF <= leftZone) {
    return 6;
  }

  return 0;   // unknown
}

// ===================== MAIN =====================
void setup() {
  servoLeft.attach(13);
  servoRight.attach(12);

  pinMode(irReceiverLeft, INPUT);
  pinMode(irReceiverMid, INPUT);
  pinMode(irReceiverRight, INPUT);
  pinMode(irLedLeft, OUTPUT);
  pinMode(irLedMid, OUTPUT);
  pinMode(irLedRight, OUTPUT);

  pinMode(ledRight, OUTPUT);
  pinMode(ledMid, OUTPUT);
  pinMode(ledLeft, OUTPUT);

  setLEDs(LOW, LOW, LOW);
  stopMotors();
  delay(5000);   // 5 second delay before any movement starts
}

void loop() {
  int scenario = detectScenario();
  delay(50);
  if (scenario != detectScenario()) {
    return;   // two readings disagreed, so read again rather than act on a glitch
  }

  switch (scenario) {

    case 1:   // middle of a long corridor
      showCode(HIGH, LOW, LOW);
      moveForwardCm(STEP_CM);
      finishScenario();
      break;

    case 2:   // ideal position for a right turn
      showCode(LOW, HIGH, LOW);
      rotateClockwise(ROTATE_90_MS);
      moveForwardCm(REENTRY_CM);
      finishScenario();
      break;

    case 3:   // ideal position for a left turn
      showCode(HIGH, HIGH, LOW);
      rotateAnticlockwise(ROTATE_90_MS);
      moveForwardCm(REENTRY_CM);
      finishScenario();
      break;

    case 4:   // dead end
      showCode(LOW, LOW, HIGH);
      rotateAnticlockwise(ROTATE_180_MS);
      moveForwardCm(DEAD_END_CM);
      finishScenario();
      break;

    case 5:   // close to the left wall, parallel to it
      showCode(HIGH, LOW, HIGH);
      rotateClockwise(SMALL_ROTATE_MS);
      moveForwardCm(ADJUST_CM);
      rotateAnticlockwise(SMALL_ROTATE_MS);
      moveBackwardCm(ADJUST_CM);
      finishScenario();
      break;

    case 6:   // close to the right wall, parallel to it
      showCode(LOW, HIGH, HIGH);
      rotateAnticlockwise(SMALL_ROTATE_MS);
      moveForwardCm(ADJUST_CM);
      rotateClockwise(SMALL_ROTATE_MS);
      moveBackwardCm(ADJUST_CM);
      finishScenario();
      break;

    case 7:   // angled ~30 degrees toward the left wall
      showCode(HIGH, HIGH, HIGH);
      rotateClockwise(SMALL_ROTATE_MS);
      finishScenario();
      break;

    case 8:   // angled ~30 degrees toward the right wall
      flashCode(ledRight);
      rotateAnticlockwise(SMALL_ROTATE_MS);
      finishScenario();
      break;

    case 9:   // custom: hugging the left wall with a wall ahead - back up, then re-detect
      flashCode(ledMid);
      moveBackwardCm(STEP_CM);
      stopMotors();
      break;

    default:  // 0: unknown - LEDs off and no movement at all
      setLEDs(LOW, LOW, LOW);
      servoLeft.detach();   // stop sending pulses so the wheels cannot creep
      servoRight.detach();
      delay(DISPLAY_MS);
      while (true) {
        delay(1000);
      }
      break;
  }
}