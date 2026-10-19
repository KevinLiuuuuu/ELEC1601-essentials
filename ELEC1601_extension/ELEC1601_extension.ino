// ELEC1601 Shield-Bot - Project Extension
//
// Pick what the robot should do with TASK before uploading:
//   1 = Part 1, Blue path   : start, follow corridor, stop at the wall in front
//   2 = Part 1, Purple path : follow corridor, turn once, follow corridor, stop at the next wall
//   3 = Part 1, Yellow path : turn at every corner, stop at the dead end
//   4 = Part 1, Green path  : as 3, spin 180 at the dead end, then drive back to the start
//   5 = Part 2              : maze with T-junctions, S/U turns, short and long dead ends
#include <Servo.h>

const int TASK = 3;
const bool DEBUG_SERIAL = false;   // true: print events and the odometer (needed to calibrate Part 2)

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
const int STEER_US = 40;           // how much one wheel is slowed to steer back to the middle

// ---------- calibration (same numbers as the essentials sketch) ----------
const float DRIVE_SPEED_CM_PER_S = 12.5;
const int ROTATE_90_MS = 577;
const int ROTATE_180_MS = 2 * ROTATE_90_MS;
const int ROTATE_30_MS = ROTATE_90_MS / 3;
const int SETTLE_MS = 200;
const float REENTRY_CM = 12.0;                       // drive after a turn to get back into the corridor

// ---------- sensor zones (0 = very close ... 5 = nothing), from your measurements ----------
const int NO_WALL_ZONE = 5;
const int FAR_ZONE = 4;
const int NEAR_ZONE = 2;
const int SIGNIFICANT_DIFF = 2;
const int FRONT_WALL_ZONE = 2;     // mid sensor this close or closer = wall ahead, stop and decide
const float CORNER_CREEP_CM = 5.0; // TUNE: after stopping at a corner, creep this far so the wheels
                                   // are in the middle of the junction before turning

// ---------- Part 2 tuning ----------
const int GAP_CONFIRM_READS = 3;           // side reads in a row that must say "open" to count as a gap
const float GAP_ADVANCE_CM = 11.0;         // TUNE: drive past the end of the wall so the wheels line up with the opening
const float WALL_LENGTH_CM = 25.0;         // MEASURE: length of one wall panel (pillar to pillar)
// Odometer value when the robot stops at the end of the long dead end. It is roughly
// 2.5 wall lengths (the two walls, plus the opening it came through, minus the stopping
// distance). CALIBRATE: set DEBUG_SERIAL true, drive into the long dead end and copy the
// "front wall, odometer cm" number here.
const float LONG_DEAD_END_CM = 2.5 * WALL_LENGTH_CM;
const float LONG_TOLERANCE_CM = 0.4 * WALL_LENGTH_CM;  // a dead end one wall shorter/longer is outside this
// After turning into a side corridor the odometer starts here, so the dead end is measured
// from the same point as when the robot drives straight past an opening.
const float TURN_ENTRY_OFFSET_CM = GAP_ADVANCE_CM + 1.0;

// ---------- sensing ----------
int zoneL, zoneM, zoneR;

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

void readSensors() {
  zoneL = irDistance(irLedLeft, irReceiverLeft);
  zoneM = irDistance(irLedMid, irReceiverMid);
  zoneR = irDistance(irLedRight, irReceiverRight);
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

void moveForwardCm(float cm)     { drive(LEFT_FWD, RIGHT_FWD, msForCm(cm)); }
void rotateClockwise(int ms)     { drive(LEFT_FWD, RIGHT_BACK, ms); }
void rotateAnticlockwise(int ms) { drive(LEFT_BACK, RIGHT_FWD, ms); }

// Drive straight, but slow one wheel if we are closer to one wall than the other.
// Only steers when BOTH walls are visible, so it never veers into a side opening.
void driveForwardSteered() {
  int left = LEFT_FWD, right = RIGHT_FWD;
  bool bothWalls = (zoneL < NO_WALL_ZONE) && (zoneR < NO_WALL_ZONE);
  if (bothWalls && zoneL < zoneR) {
    right = RIGHT_FWD + STEER_US;      // closer to the left wall: veer right
  } else if (bothWalls && zoneR < zoneL) {
    left = LEFT_FWD - STEER_US;        // closer to the right wall: veer left
  }
  servoLeft.writeMicroseconds(left);
  servoRight.writeMicroseconds(right);
}

// Creep straight ahead (no steering) in 1 cm steps. Used to line up with a junction.
void creepForward(float cm) {
  for (float done = 0; done < cm; done += 1.0) {
    moveForwardCm(1.0);
  }
}

// ---------- state ----------
const int EV_NONE = 0, EV_WALL = 1, EV_GAP = 2;

float odoCm = 0;                 // distance driven since the last turn / side opening
int turnsDone = 0;
int deadEndsSeen = 0;
bool leftWallSeen = false, rightWallSeen = false;
int leftGapReads = 0, rightGapReads = 0;

void debugLine(const char* text, float value) {
  if (!DEBUG_SERIAL) return;
  Serial.print(text);
  Serial.print(" ");
  Serial.println(value);
}

// Drive forward up to cm centimetres, steering between the walls.
// Returns EV_WALL if a wall is ahead, EV_GAP if the left wall just ended (Part 2 only), else EV_NONE.
int advance(float cm, bool watchGaps) {
  float travelled = 0;
  bool moving = false;
  unsigned long last = 0;
  while (travelled < cm) {
    readSensors();
    unsigned long now = millis();
    if (moving) {
      float d = (now - last) / 1000.0 * DRIVE_SPEED_CM_PER_S;
      travelled += d;
      odoCm += d;
    }
    if (zoneM <= FRONT_WALL_ZONE) {
      stopMotors();
      return EV_WALL;
    }
    if (watchGaps) {
      if (zoneL < NO_WALL_ZONE) {
        leftWallSeen = true;
        leftGapReads = 0;
      } else if (leftWallSeen && ++leftGapReads >= GAP_CONFIRM_READS) {
        stopMotors();
        leftWallSeen = false;
        leftGapReads = 0;
        return EV_GAP;
      }
      if (zoneR < NO_WALL_ZONE) {
        rightWallSeen = true;
        rightGapReads = 0;
      } else if (rightWallSeen && ++rightGapReads >= GAP_CONFIRM_READS) {
        rightWallSeen = false;       // opening on the right: we drive past it,
        rightGapReads = 0;           // but restart the odometer so a dead end is measured from here
        odoCm = 0;
      }
    }
    if (travelled >= cm) break;
    driveForwardSteered();
    last = millis();
    moving = true;
  }
  stopMotors();
  return EV_NONE;
}

// ---------- start position fix (essentials scenarios 5-8) ----------
// Near a wall, spinning on the spot swings the back corner into it. Pivoting on one wheel
// (the other wheel stopped) moves the whole robot away from the wall instead.
// A pivot turns half as fast as a spin, so it takes twice as long for the same angle.
const int PIVOT_30_MS = 2 * ROTATE_30_MS;
const int PIVOT_45_MS = ROTATE_90_MS;

void pivotClockwise(int ms)     { drive(LEFT_FWD, RIGHT_STOP, ms); }   // pivot on the right wheel
void pivotAnticlockwise(int ms) { drive(LEFT_STOP, RIGHT_FWD, ms); }   // pivot on the left wheel

void fixStartPosition() {
  readSensors();
  bool leftWall = zoneL < NO_WALL_ZONE;
  bool rightWall = zoneR < NO_WALL_ZONE;
  bool frontClear = zoneM >= FAR_ZONE;
  if (zoneL <= NEAR_ZONE && !rightWall && frontClear) {                  // 7: angled to left wall
    pivotClockwise(PIVOT_30_MS);
  } else if (zoneR <= NEAR_ZONE && !leftWall && frontClear) {            // 8: angled to right wall
    pivotAnticlockwise(PIVOT_30_MS);
  } else if (leftWall && rightWall && frontClear && zoneL + SIGNIFICANT_DIFF <= zoneR) {   // 5: close to left wall
    pivotClockwise(PIVOT_45_MS);           // S-shape: swing right ...
    pivotAnticlockwise(PIVOT_45_MS);       // ... then straighten up, now further from the left wall
  } else if (leftWall && rightWall && frontClear && zoneR + SIGNIFICANT_DIFF <= zoneL) {   // 6: close to right wall
    pivotAnticlockwise(PIVOT_45_MS);
    pivotClockwise(PIVOT_45_MS);
  }
  delay(SETTLE_MS);
}

// ---------- events ----------
void finish() {
  stopMotors();
  digitalWrite(ledRight, HIGH);
  digitalWrite(ledMid, HIGH);
  digitalWrite(ledLeft, HIGH);
  servoLeft.detach();
  servoRight.detach();
  while (true) {
    delay(1000);
  }
}

void resetAfterTurn(float startOdo) {
  odoCm = startOdo;
  leftWallSeen = false;
  rightWallSeen = false;
  leftGapReads = 0;
  rightGapReads = 0;
  turnsDone++;
}

void turnLeft() {
  rotateAnticlockwise(ROTATE_90_MS);
  delay(SETTLE_MS);
  resetAfterTurn(TURN_ENTRY_OFFSET_CM);
  advance(REENTRY_CM, false);
}

void turnRight() {
  rotateClockwise(ROTATE_90_MS);
  delay(SETTLE_MS);
  resetAfterTurn(TURN_ENTRY_OFFSET_CM);
  advance(REENTRY_CM, false);
}

void turnAround() {
  rotateClockwise(ROTATE_180_MS);
  delay(SETTLE_MS);
  resetAfterTurn(0);
  turnsDone--;                     // a U-turn is not one of the corner turns
}

bool isLongDeadEnd() {
  return fabs(odoCm - LONG_DEAD_END_CM) <= LONG_TOLERANCE_CM;
}

// A wall is in front of the robot: decide what to do from the two side sensors.
void onFrontWall() {
  readSensors();                              // robot is stopped, take a fresh reading
  bool leftOpen = (zoneL == NO_WALL_ZONE);
  bool rightOpen = (zoneR == NO_WALL_ZONE);
  bool deadEnd = !leftOpen && !rightOpen;
  debugLine("front wall, odometer cm:", odoCm);

  if (TASK == 1) finish();                    // blue path: stop at the first wall
  if (TASK == 2 && turnsDone >= 1) finish();  // purple path: stop at the wall after the turn

  if (deadEnd) {
    if (TASK == 5) {
      if (isLongDeadEnd()) finish();          // facing forward at the end of the long dead end
      turnAround();                           // short dead end: escape
      return;
    }
    if (TASK == 4 && deadEndsSeen == 0) {     // green path: spin and drive back
      deadEndsSeen++;
      turnAround();
      return;
    }
    finish();                                 // tasks 2, 3 and the second dead end of 4
  }

  creepForward(CORNER_CREEP_CM);              // get the wheels to the middle of the junction
  if (leftOpen) {                             // left-hand rule: left first (corner or T-junction)
    turnLeft();
  } else {
    turnRight();
  }
}

// Part 2: the left wall ended while driving. Line up with the opening, then turn left.
void onLeftGap() {
  debugLine("left gap at odometer cm:", odoCm);
  int ev = advance(GAP_ADVANCE_CM, false);
  if (ev == EV_WALL) {                        // a wall came up while creeping: it is a corner
    onFrontWall();
    return;
  }
  readSensors();
  if (zoneL < NO_WALL_ZONE) return;           // the wall came back: it was a glitch, carry on
  turnLeft();
}

// ---------- main ----------
void setup() {
  if (DEBUG_SERIAL) Serial.begin(9600);
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

  digitalWrite(ledRight, LOW);
  digitalWrite(ledMid, LOW);
  digitalWrite(ledLeft, LOW);
  stopMotors();
  delay(5000);               // 5 s before anything happens

  fixStartPosition();
}

void loop() {
  int ev = advance(1000, TASK == 5);
  if (ev == EV_GAP) {
    onLeftGap();
  } else if (ev == EV_WALL) {
    onFrontWall();
  }
}
