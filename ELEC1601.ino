// ELEC1601 Shield-Bot - essentials scenarios 0-8 plus own scenario 9
#include <Servo.h>

Servo servoLeft;
Servo servoRight;

// ---------- pins ----------
const int irLedLeft = 10,  irReceiverLeft = 11;
const int irLedMid = 6,    irReceiverMid = 7;
const int irLedRight = 2,  irReceiverRight = 3;
const int ledRight = A0, ledMid = A1, ledLeft = A2;

// ---------- servo pulses (us) ----------
const int LEFT_STOP = 1500, RIGHT_STOP = 1500;
const int LEFT_FWD = 1600,  RIGHT_FWD = 1400;
const int LEFT_BACK = 1400, RIGHT_BACK = 1600;

// ---------- calibration ----------
const float DRIVE_SPEED_CM_PER_S = 12.5;
const int ROTATE_90_MS = 577;
const int ROTATE_180_MS = 2 * ROTATE_90_MS;

// ~30 degree turns, tuned separately per direction (servos are never perfectly matched).
// Used by scenarios 5-8. Tune these two in the lab (see notes) until 5 and 6 finish parallel.
const int CW_30_MS  = ROTATE_90_MS / 3;
const int CCW_30_MS = ROTATE_90_MS / 3;
const int SETTLE_MS = 200;                        // pause so the wheels fully stop between moves

const float STEP_CM = 5.0;        // scenario 1
const float REENTRY_CM = 12.0;    // scenarios 2, 3
const float DEAD_END_CM = 6.0;    // scenario 4
const float SIDE_SHIFT_CM = 5.0;  // scenarios 5, 6
const float DIAGONAL_CM = SIDE_SHIFT_CM / 0.5;        // shift / sin30
const float RETURN_CM = DIAGONAL_CM * 0.866;          // diagonal * cos30

const int DISPLAY_MS = 5500;      // LEDs on, robot still (spec: at least 5 s)

// ---------- sensor zones (0 = very close ... 5 = nothing) ----------
const int NO_WALL_ZONE = 5;
const int FAR_ZONE = 4;
const int NEAR_ZONE = 2;
const int SIMILAR_DIFF = 1;
const int SIGNIFICANT_DIFF = 2;

// ---------- sensing ----------
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

// ---------- motion ----------
void stopMotors() {
  servoLeft.writeMicroseconds(LEFT_STOP);
  servoRight.writeMicroseconds(RIGHT_STOP);
}

void drive(int left, int right, int ms) {
  servoLeft.writeMicroseconds(left);
  servoRight.writeMicroseconds(right);
  delay(ms);
  stopMotors();
}

int msForCm(float cm) { return (int)(cm / DRIVE_SPEED_CM_PER_S * 1000.0); }

void moveForwardCm(float cm)  { drive(LEFT_FWD, RIGHT_FWD, msForCm(cm)); }
void moveBackwardCm(float cm) { drive(LEFT_BACK, RIGHT_BACK, msForCm(cm)); }
void rotateClockwise(int ms)     { drive(LEFT_FWD, RIGHT_BACK, ms); }
void rotateAnticlockwise(int ms) { drive(LEFT_BACK, RIGHT_FWD, ms); }

// Scenario 5: sidestep to the right, keep original heading.
// Turn out 30, drive the diagonal, turn back 30, reverse the forward part.
void shiftRight() {
  rotateClockwise(CW_30_MS);
  delay(SETTLE_MS);
  moveForwardCm(DIAGONAL_CM);
  delay(SETTLE_MS);
  rotateAnticlockwise(CCW_30_MS);
  delay(SETTLE_MS);
  moveBackwardCm(RETURN_CM);
}

// Scenario 6: sidestep to the left, keep original heading
void shiftLeft() {
  rotateAnticlockwise(CCW_30_MS);
  delay(SETTLE_MS);
  moveForwardCm(DIAGONAL_CM);
  delay(SETTLE_MS);
  rotateClockwise(CW_30_MS);
  delay(SETTLE_MS);
  moveBackwardCm(RETURN_CM);
}

// Stop for good (spec: stop after completion)
void haltForever() {
  stopMotors();
  servoLeft.detach();
  servoRight.detach();
  while (true) {
    delay(1000);
  }
}

// ---------- LEDs (order: Right A0, Mid A1, Left A2) ----------
void setLEDs(int right, int mid, int left) {
  digitalWrite(ledRight, right);
  digitalWrite(ledMid, mid);
  digitalWrite(ledLeft, left);
}

void showCode(int right, int mid, int left) {
  stopMotors();
  setLEDs(right, mid, left);
  delay(DISPLAY_MS);
}

// Flash the code: 1 s on / 1 s off, three times (6 s) with the robot still
void flashCode(int right, int mid, int left) {
  stopMotors();
  for (int i = 0; i < 3; i++) {
    setLEDs(right, mid, left);
    delay(1000);
    setLEDs(LOW, LOW, LOW);
    delay(1000);
  }
}

// ---------- scenario detection ----------
int detectScenario() {
  int L = irDistance(irLedLeft, irReceiverLeft);
  int M = irDistance(irLedMid, irReceiverMid);
  int R = irDistance(irLedRight, irReceiverRight);

  bool leftOpen = (L == NO_WALL_ZONE);
  bool rightOpen = (R == NO_WALL_ZONE);
  bool leftWall = !leftOpen;
  bool rightWall = !rightOpen;
  bool leftNear = (L <= NEAR_ZONE);
  bool rightNear = (R <= NEAR_ZONE);
  bool midNear = (M <= NEAR_ZONE);
  bool midWall = (M < FAR_ZONE);
  bool frontClear = !midWall;

  if (rightOpen && leftWall && midWall) return 2;                    // right turn
  if (leftOpen && rightWall && midWall) return 3;                    // left turn
  if (leftWall && rightWall && midWall) return 4;                    // dead end
  if (leftWall && rightWall && frontClear && abs(L - R) <= SIMILAR_DIFF) return 1;   // corridor middle
  if (leftWall && rightWall && frontClear && L + SIGNIFICANT_DIFF <= R) return 5;    // close to left wall
  if (leftWall && rightWall && frontClear && R + SIGNIFICANT_DIFF <= L) return 6;    // close to right wall
  // Measured: angled to the left wall reads 0,5,5 and to the right wall 5,5,0
  // (only the near side sensor sees the wall; front and far side read nothing).
  if (leftNear && rightOpen && frontClear) return 7;                 // angled to left wall
  if (rightNear && leftOpen && frontClear) return 8;                 // angled to right wall
  if (leftOpen && rightOpen && midWall) return 9;                    // own: wall ahead only, both sides open
  return 0;
}

// ---------- main ----------
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
  delay(5000);   // 5 s before anything happens
}

void loop() {
  int scenario = detectScenario();
  delay(50);
  if (scenario != detectScenario()) return;   // readings disagreed - try again

  switch (scenario) {
    case 1:
      showCode(HIGH, LOW, LOW);
      moveForwardCm(STEP_CM);
      break;
    case 2:
      showCode(LOW, HIGH, LOW);
      rotateClockwise(ROTATE_90_MS);
      moveForwardCm(REENTRY_CM);
      break;
    case 3:
      showCode(HIGH, HIGH, LOW);
      rotateAnticlockwise(ROTATE_90_MS);
      moveForwardCm(REENTRY_CM);
      break;
    case 4:
      showCode(LOW, LOW, HIGH);
      rotateAnticlockwise(ROTATE_180_MS);
      moveForwardCm(DEAD_END_CM);
      break;
    case 5:
      showCode(HIGH, LOW, HIGH);
      shiftRight();
      break;
    case 6:
      showCode(LOW, HIGH, HIGH);
      shiftLeft();
      break;
    case 7:
      showCode(HIGH, HIGH, HIGH);
      rotateClockwise(CW_30_MS);
      break;
    case 8:
      flashCode(HIGH, LOW, LOW);
      rotateAnticlockwise(CCW_30_MS);
      break;
    case 9:    // own: wall ahead only (T-junction) - flash mid LED, turn right and go
      flashCode(LOW, HIGH, LOW);
      rotateClockwise(ROTATE_90_MS);
      moveForwardCm(REENTRY_CM);
      break;
    default:   // scenario 0: LEDs off, no movement
      setLEDs(LOW, LOW, LOW);
      break;
  }
  haltForever();
}
