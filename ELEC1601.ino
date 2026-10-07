/*
 * ELEC1601 maze robot - scenario detection and escape (scenarios 0-10)
 *
 *   MAZE_MODE   = false : marking mode (spec). Show the code for 5+ s, do the moves, stop.
 *   MAZE_MODE   = true  : solving mode. Short display, keeps detecting and moving.
 *   SENSOR_TEST = true  : never moves. Prints L/M/R zones and the scenario it would pick.
 */
#include <Servo.h>

Servo servoLeft;
Servo servoRight;

// ===================== MODES =====================
const bool MAZE_MODE = false;
const bool SENSOR_TEST = false;
const bool DEBUG_SERIAL = true;     // print readings to Serial Monitor (9600 baud)

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
// Each wheel's true stop value, and the pair that drives in a straight line (Part 2.2 / 2.3).
const int LEFT_STOP = 1500, RIGHT_STOP = 1500;
const int LEFT_FWD = 1600, RIGHT_FWD = 1400;
const int LEFT_BACK = 1400, RIGHT_BACK = 1600;

// ===================== MOTION CALIBRATION =====================
const float DRIVE_SPEED_CM_PER_S = 12.5;        // measured ground speed
const int ROTATE_90_MS = 577;                   // measured 90 degree spin
const int ROTATE_180_MS = 2 * ROTATE_90_MS;
const int SMALL_ROTATE_MS = ROTATE_90_MS / 3;   // ~30 degrees

// ===================== DISTANCES =====================
const float STEP_CM = 5.0;         // 1: "move 5cm forward"
const float REENTRY_CM = 12.0;     // 2/3: after turning, drive far enough to be back inside a corridor
const float DEAD_END_CM = 6.0;     // 4: "at least 5cm"
const float SIDE_SHIFT_CM = 5.0;   // 5/6/10: how far to slide sideways toward the centre
const float BACKUP_CM = 4.0;       // 9: back away from the corner
const float CREEP_CM = 2.5;        // maze mode: small step when the position is unknown
// Maze mode: with no walls anywhere (an open gap in the maze) it keeps creeping forward.
// It only stops after this many creeps in a row with no wall at all (well outside the maze).
// 12 x 2.5 cm = 30 cm. Set to 0 to never stop.
const int OPEN_CREEP_LIMIT = 12;

// Sliding sideways = turn 30 deg, drive a diagonal, turn back, reverse to the start line.
const float SIN_30 = 0.5, COS_30 = 0.866;
const float DIAGONAL_CM = SIDE_SHIFT_CM / SIN_30;   // diagonal needed for SIDE_SHIFT_CM sideways
const float RETURN_CM = DIAGONAL_CM * COS_30;       // reverse this much to end level with the start

// ===================== DISPLAY =====================
// Spec: robot does nothing for at least 5 s while the code shows. 5.5 s gives a safety margin.
const int DISPLAY_MS = MAZE_MODE ? 300 : 5500;
const bool HALT_AFTER_SCENARIO = !MAZE_MODE;        // spec: "Stop after completion"

// ===================== SENSOR ZONES (CALIBRATE WITH SENSOR_TEST) =====================
// irDistance() returns 0-5: how many of the 38-42 kHz tones were NOT detected.
// 0 = wall very close (~5 cm), 4 = wall far (~10 cm), 5 = nothing in range.
const int NO_WALL_ZONE = 5;
const int FAR_ZONE = 4;            // a wall that is seen but far away
const int NEAR_ZONE = 2;           // this zone or lower = wall is close
const int TOO_CLOSE_ZONE = 0;      // front this close = nose nearly touching (scenario 9)
const int SIMILAR_DIFF = 1;        // left/right within this = "similar reading"
const int SIGNIFICANT_DIFF = 2;    // gap that counts as "significantly closer"

// Set true only if your side sensors read 5 even when centred in a normal corridor
// (walls out of range). Scenario 1 then also accepts "no wall on either side".
const bool ACCEPT_OPEN_CORRIDOR = false;

// Last readings, kept so the scenario actions can use them
int zoneL = NO_WALL_ZONE, zoneM = NO_WALL_ZONE, zoneR = NO_WALL_ZONE;
int openCreeps = 0;   // maze mode: consecutive creeps with no wall in sight

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

void drive(int left, int right, int ms) {
  servoLeft.writeMicroseconds(left);
  servoRight.writeMicroseconds(right);
  delay(ms);
  stopMotors();
}

int msForCm(float cm) {
  return (int)(cm / DRIVE_SPEED_CM_PER_S * 1000.0);
}

void moveForwardCm(float cm)       { drive(LEFT_FWD, RIGHT_FWD, msForCm(cm)); }
void moveBackwardCm(float cm)      { drive(LEFT_BACK, RIGHT_BACK, msForCm(cm)); }
void rotateClockwise(int ms)       { drive(LEFT_FWD, RIGHT_BACK, ms); }
void rotateAnticlockwise(int ms)   { drive(LEFT_BACK, RIGHT_FWD, ms); }

// Slide toward the right while staying parallel (spec scenario 5 sequence)
void shiftRight() {
  rotateClockwise(SMALL_ROTATE_MS);
  moveForwardCm(DIAGONAL_CM);
  rotateAnticlockwise(SMALL_ROTATE_MS);
  moveBackwardCm(RETURN_CM);
}

// Slide toward the left while staying parallel (spec scenario 6 sequence)
void shiftLeft() {
  rotateAnticlockwise(SMALL_ROTATE_MS);
  moveForwardCm(DIAGONAL_CM);
  rotateClockwise(SMALL_ROTATE_MS);
  moveBackwardCm(RETURN_CM);
}

// An Arduino can't exit loop(), so "stopping" means waiting here forever.
// Detaching the servos stops all pulses so the wheels cannot creep.
void haltForever() {
  stopMotors();
  servoLeft.detach();
  servoRight.detach();
  while (true) {
    delay(1000);
  }
}

void finishScenario() {
  stopMotors();
  if (HALT_AFTER_SCENARIO) {
    haltForever();
  }
}

// ===================== LEDS =====================
// Argument order matches the spec: Right (A0), Mid (A1), Left (A2)
void setLEDs(int right, int mid, int left) {
  digitalWrite(ledRight, right);
  digitalWrite(ledMid, mid);
  digitalWrite(ledLeft, left);
}

// Steady code with the robot still
void showCode(int right, int mid, int left) {
  stopMotors();
  setLEDs(right, mid, left);
  delay(DISPLAY_MS);
}

// Flashing code (1 s on, 1 s off) with the robot still, for at least DISPLAY_MS
void flashCode(int right, int mid, int left) {
  stopMotors();
  if (MAZE_MODE) {             // no time for 1 s flashes while solving
    setLEDs(right, mid, left);
    delay(DISPLAY_MS);
    return;
  }
  for (int i = 0; i < DISPLAY_MS / 2000 + 1; i++) {
    setLEDs(right, mid, left);
    delay(1000);
    setLEDs(LOW, LOW, LOW);
    delay(1000);
  }
}

// ===================== SCENARIO DETECTION =====================
// Pure logic: turns three zone readings into a scenario number (0 = unknown).
// Order matters: more specific scenarios are checked first.
int classify(int L, int M, int R) {
  bool leftOpen = (L == NO_WALL_ZONE);
  bool rightOpen = (R == NO_WALL_ZONE);
  bool leftWall = !leftOpen;
  bool rightWall = !rightOpen;

  bool leftNear = (L <= NEAR_ZONE);
  bool midNear = (M <= NEAR_ZONE);
  bool rightNear = (R <= NEAR_ZONE);

  bool midWall = (M < FAR_ZONE);   // wall closer than ~10 cm
  bool frontClear = !midWall;      // spec 1/5/6: "no wall, or a wall at least 10cm away"

  // 9 (custom): tucked into a corner - nose almost touching, hugging the side wall,
  // other side open (idea photos 4, 5, 7). Too close to turn cleanly, so back out first.
  if (M <= TOO_CLOSE_ZONE && ((leftNear && rightOpen) || (rightNear && leftOpen))) {
    return 9;
  }

  // 2: right turn - right sees no wall, left and front see a wall
  if (rightOpen && leftWall && midWall) {
    return 2;
  }

  // 3: left turn - left sees no wall, right and front see a wall
  if (leftOpen && rightWall && midWall) {
    return 3;
  }

  // 7: ~30 deg toward the left wall - left and front close, right wall far but seen.
  // Front close is what separates this from scenario 1.
  if (leftNear && midNear && R == FAR_ZONE) {
    return 7;
  }

  // 8: ~30 deg toward the right wall - right and front close, left wall far but seen.
  // Right wall close (not open) is what separates this from scenario 2.
  if (rightNear && midNear && L == FAR_ZONE) {
    return 8;
  }

  // 10 (custom): hugging one wall with the other side open and the front clear
  // (idea photo 3, e.g. drifting at a side opening). Slide away from the wall.
  if (frontClear && ((leftNear && rightOpen) || (rightNear && leftOpen))) {
    return 10;
  }

  // 4: dead end - walls on all three sides
  if (leftWall && midWall && rightWall) {
    return 4;
  }

  // 1: middle of a corridor - similar side readings, front clear
  bool sidesSeen = (leftWall && rightWall) || (ACCEPT_OPEN_CORRIDOR && leftOpen && rightOpen);
  if (sidesSeen && frontClear && abs(L - R) <= SIMILAR_DIFF) {
    return 1;
  }

  // 5: close to the left wall, parallel, front clear
  if (leftWall && rightWall && frontClear && L + SIGNIFICANT_DIFF <= R) {
    return 5;
  }

  // 6: close to the right wall, parallel, front clear
  if (leftWall && rightWall && frontClear && R + SIGNIFICANT_DIFF <= L) {
    return 6;
  }

  return 0;
}

int detectScenario() {
  zoneL = irDistance(irLedLeft, irReceiverLeft);
  zoneM = irDistance(irLedMid, irReceiverMid);
  zoneR = irDistance(irLedRight, irReceiverRight);

  if (DEBUG_SERIAL) {
    Serial.print("L ");
    Serial.print(zoneL);
    Serial.print("  M ");
    Serial.print(zoneM);
    Serial.print("  R ");
    Serial.println(zoneR);
  }
  return classify(zoneL, zoneM, zoneR);
}

// ===================== MAIN =====================
void setup() {
  if (DEBUG_SERIAL) {
    Serial.begin(9600);
  }

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
  delay(5000);   // 5 second delay before anything starts
}

void loop() {
  int scenario = detectScenario();

  if (SENSOR_TEST) {
    Serial.print("Scenario ");
    Serial.println(scenario);
    delay(300);
    return;
  }

  // Confirm with a second reading so one glitchy sample can't trigger a move
  delay(50);
  if (scenario != detectScenario()) {
    return;
  }

  if (DEBUG_SERIAL) {
    Serial.print("Scenario ");
    Serial.println(scenario);
  }

  bool allOpen = (zoneL == NO_WALL_ZONE && zoneM == NO_WALL_ZONE && zoneR == NO_WALL_ZONE);
  if (!allOpen) {
    openCreeps = 0;   // saw a wall, so it's still in the maze
  }

  switch (scenario) {

    case 1:   // middle of a corridor: R on
      showCode(HIGH, LOW, LOW);
      moveForwardCm(STEP_CM);
      finishScenario();
      break;

    case 2:   // right turn: Mid on
      showCode(LOW, HIGH, LOW);
      rotateClockwise(ROTATE_90_MS);
      moveForwardCm(REENTRY_CM);
      finishScenario();
      break;

    case 3:   // left turn: R + Mid on
      showCode(HIGH, HIGH, LOW);
      rotateAnticlockwise(ROTATE_90_MS);
      moveForwardCm(REENTRY_CM);
      finishScenario();
      break;

    case 4:   // dead end: L on
      showCode(LOW, LOW, HIGH);
      rotateAnticlockwise(ROTATE_180_MS);
      moveForwardCm(DEAD_END_CM);
      finishScenario();
      break;

    case 5:   // close to the left wall, parallel: R + L on
      showCode(HIGH, LOW, HIGH);
      shiftRight();
      finishScenario();
      break;

    case 6:   // close to the right wall, parallel: Mid + L on
      showCode(LOW, HIGH, HIGH);
      shiftLeft();
      finishScenario();
      break;

    case 7:   // ~30 deg toward the left wall: all on
      showCode(HIGH, HIGH, HIGH);
      rotateClockwise(SMALL_ROTATE_MS);
      finishScenario();
      break;

    case 8:   // ~30 deg toward the right wall: R flashing
      flashCode(HIGH, LOW, LOW);
      rotateAnticlockwise(SMALL_ROTATE_MS);
      finishScenario();
      break;

    case 9:   // custom: tucked into a corner: Mid flashing
      flashCode(LOW, HIGH, LOW);
      moveBackwardCm(BACKUP_CM);   // now far enough to re-detect as a normal turn
      finishScenario();
      break;

    case 10:  // custom: hugging one wall, other side open: R + Mid flashing
      flashCode(HIGH, HIGH, LOW);
      if (zoneL <= zoneR) {
        shiftRight();              // left wall is the close one
      } else {
        shiftLeft();               // right wall is the close one
      }
      finishScenario();
      break;

    default:  // 0: unknown - LEDs off
      setLEDs(LOW, LOW, LOW);
      stopMotors();

      if (!MAZE_MODE) {
        haltForever();             // spec: "Not move at all"
      }

      // Maze mode: never give up on an unknown position
      if (allOpen) {
        // No walls anywhere: an open gap in the maze. Keep going straight through it,
        // unless it has been open for so long that it must have left the maze.
        if (OPEN_CREEP_LIMIT > 0 && openCreeps >= OPEN_CREEP_LIMIT) {
          haltForever();
        }
        openCreeps++;
        moveForwardCm(CREEP_CM);
      } else if (zoneM >= FAR_ZONE) {
        moveForwardCm(CREEP_CM);   // front clear: inch forward and look again
      } else if (zoneL == NO_WALL_ZONE && zoneR == NO_WALL_ZONE) {
        rotateClockwise(ROTATE_90_MS);   // facing a wall with both sides open (sideways in a corridor)
      } else {
        moveBackwardCm(CREEP_CM);  // something ahead: back off and look again
      }
      break;
  }
}
