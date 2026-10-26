// ELEC1601 Shield-Bot - Project Extension, Parts 1 and 2
//
// Set TASK before uploading:
//   1 = Blue path   : fix a bad start, follow the corridor, stop at the wall in front
//   2 = Purple path : follow the corridor, turn at the wall, follow the next corridor, stop at the next wall
//   3 = Yellow path : turn at every wall until a dead end, then stop
//   4 = Green path  : as 3, spin 180 at the dead end, then do it again to get back to the start
//   5 = Part 2      : follow the left wall until we reach the "long dead end", then stop.
//                     Short dead ends: spin 180 and keep going.
//
// Built from the essentials scenarios: 5-8 to fix the start, 1 to follow a corridor,
// 2/3 to turn, 4 for the dead end. The maze may be mirrored, so the robot never assumes
// a direction: it always turns towards whichever side is open (left first in Part 2).
#include <Servo.h>

const int TASK = 5;
const bool DEBUG = false;          // true = print what it measures to the Serial Monitor (9600 baud)

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
const int STRONG_STEER_US = 80;    // used instead when a wall is very close (zone 0)

// ---------- calibration (use the numbers from ELEC1601_calibrate) ----------
const float DRIVE_SPEED_CM_PER_S = 14.5;     // measured: 29 cm in 2 s
const int ROTATE_90_MS = 570;
const int ROTATE_180_MS = 2 * ROTATE_90_MS;
const int ROTATE_30_MS = ROTATE_90_MS / 3;
const int SETTLE_MS = 200;                 // pause so the wheels fully stop between moves
const float REENTRY_CM = 12.0;             // drive after a turn to get into the new corridor

// ---------- sensor zones (0 = very close ... 5 = nothing), from your measurements ----------
const int NO_WALL_ZONE = 5;
const int FAR_ZONE = 4;
const int NEAR_ZONE = 2;
const int SIGNIFICANT_DIFF = 2;
const int FRONT_WALL_ZONE = 2;     // mid sensor this close or closer = wall ahead
const float CORNER_CREEP_CM = 4.0; // after stopping at a corner, creep this far so the wheels
                                   // are in the middle of the corner before turning

// ---------- Part 2 ----------
const int GAP_CONFIRM_READS = 3;   // a side must read "open" this many times in a row to count,
                                   // so one bad reading cannot fool the robot
const float GAP_ADVANCE_CM = 11.0; // TUNE with test g in ELEC1601_calibrate: after the left wall ends,
                                   // drive this far so the wheels line up with the opening
const float WALL_LENGTH_CM = 25.0; // MEASURE: length of one wall panel (pillar to pillar)
// cmInCorridor when the robot stops at the end of the long dead end. It is less than two
// walls because counting starts a little after the opening and the robot stops before the
// end wall. CALIBRATE: set DEBUG = true, let the robot drive into the long dead end and
// copy the "cmInCorridor" number it prints here.
const float LONG_DEAD_END_CM = 2 * WALL_LENGTH_CM - 10;
const float LONG_TOLERANCE_CM = 0.4 * WALL_LENGTH_CM;  // one wall shorter or longer is outside this

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

// Sweep 38-42 kHz and count the frequencies that were NOT detected, scaled to 0-5.
// FAST_STEP (5 checks, about 30 ms for all three sensors) is used while driving, so the
// robot never travels far without looking. FINE_STEP (41 checks, about 250 ms) gives a
// steadier reading and is only used when the robot is stopped and about to decide something.
const long FAST_STEP = 1000;
const long FINE_STEP = 100;

int irDistance(int irLedPin, int irReceiverPin, long step) {
  int misses = 0;
  int checks = 0;
  for (long f = 38000; f <= 42000; f += step) {
    misses += irDetect(irLedPin, irReceiverPin, f);
    checks++;
  }
  return (misses * 5 + checks / 2) / checks;     // scale to 0-5, rounded
}

// Distance driven with walls on BOTH sides since the last opening. We use it to tell the
// long dead end (a straight section exactly two walls long) from short ones.
float cmInCorridor = 0;
int openReads = 0;                 // readings in a row where a side looked open

// Also resets cmInCorridor when we are not in a corridor, so we don't have to do these
// checks all over the code. It waits for GAP_CONFIRM_READS open readings in a row, so a
// single bad reading in the middle of a corridor doesn't wipe the count.
void readSensors() {             // quick reading, used while driving
  zoneL = irDistance(irLedLeft, irReceiverLeft, FAST_STEP);
  zoneM = irDistance(irLedMid, irReceiverMid, FAST_STEP);
  zoneR = irDistance(irLedRight, irReceiverRight, FAST_STEP);
  if (zoneL == NO_WALL_ZONE || zoneR == NO_WALL_ZONE) openReads++; else openReads = 0;
  if (openReads >= GAP_CONFIRM_READS) cmInCorridor = 0;
}

void readSensorsFine() {         // slow, steady reading, used only when stopped
  zoneL = irDistance(irLedLeft, irReceiverLeft, FINE_STEP);
  zoneM = irDistance(irLedMid, irReceiverMid, FINE_STEP);
  zoneR = irDistance(irLedRight, irReceiverRight, FINE_STEP);
  if (zoneL == NO_WALL_ZONE || zoneR == NO_WALL_ZONE) cmInCorridor = 0;   // steady reading: trust it
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
void rotateClockwise(int ms)     { drive(LEFT_FWD, RIGHT_BACK, ms); }   // spin on the spot
void rotateAnticlockwise(int ms) { drive(LEFT_BACK, RIGHT_FWD, ms); }
void pivotClockwise(int ms)      { drive(LEFT_FWD, RIGHT_STOP, ms); }   // turn around the right wheel
void pivotAnticlockwise(int ms)  { drive(LEFT_STOP, RIGHT_FWD, ms); }   // turn around the left wheel

// Creep straight ahead in 1 cm steps (no steering). Used to line up with a corner.
void creepForward(float cm) {
  for (float done = 0; done < cm; done += 1.0) {
    moveForwardCm(1.0);
  }
}

// Scenario 1, done continuously: drive straight, but slow one wheel if we are closer to
// one wall than the other. With only ONE wall visible (passing an opening), it keeps the
// distance to that wall it would have in the middle of a corridor (CENTRE_ZONE), so it
// doesn't drift while the other side is open. If a wall is very close (zone 0) it steers
// twice as hard.
const int CENTRE_ZONE = 1;         // side reading when the robot is in the middle of a corridor

void driveForwardSteered() {
  int left = LEFT_FWD, right = RIGHT_FWD;
  bool leftSeen = zoneL < NO_WALL_ZONE;
  bool rightSeen = zoneR < NO_WALL_ZONE;
  int steer = (zoneL == 0 || zoneR == 0) ? STRONG_STEER_US : STEER_US;
  // error > 0: too close to the left wall, steer right. error < 0: steer left.
  int error = 0;
  if (leftSeen && rightSeen) error = zoneR - zoneL;
  else if (leftSeen)         error = CENTRE_ZONE - zoneL;
  else if (rightSeen)        error = zoneR - CENTRE_ZONE;
  if (error > 0) {
    right = RIGHT_FWD + steer;         // closer to the left wall: veer right
  } else if (error < 0) {
    left = LEFT_FWD - steer;           // closer to the right wall: veer left
  }
  servoLeft.writeMicroseconds(left);
  servoRight.writeMicroseconds(right);
}

// What made driveUntil() stop.
const int FOUND_NOTHING = 0, FOUND_WALL = 1, FOUND_LEFT_GAP = 2;

// Part 2: only take a left opening after we have seen a left wall since the last turn.
// Without this, a robot that has just turned left would see "left open" again straight
// away and keep turning left in circles.
bool leftWallSeen = false;
int leftOpenReads = 0;

// Follow the corridor until a wall is in front, until maxCm has been driven, or (if
// watchLeftGap is true) until the left wall ends. Returns what it found.
int driveUntil(float maxCm, bool watchLeftGap) {
  float travelled = 0;
  bool moving = false;
  unsigned long last = 0;
  while (travelled < maxCm) {
    readSensors();
    bool bothWalls = (zoneL < NO_WALL_ZONE) && (zoneR < NO_WALL_ZONE);
    unsigned long now = millis();
    if (moving) {
      float step = (now - last) / 1000.0 * DRIVE_SPEED_CM_PER_S;   // cm since the last reading
      travelled += step;
      if (bothWalls) cmInCorridor += step;
    }
    if (zoneM <= FRONT_WALL_ZONE) {
      stopMotors();
      return FOUND_WALL;
    }
    if (watchLeftGap) {
      if (zoneL < NO_WALL_ZONE) {
        leftWallSeen = true;
        leftOpenReads = 0;
      } else if (leftWallSeen && ++leftOpenReads >= GAP_CONFIRM_READS) {
        stopMotors();
        leftWallSeen = false;
        leftOpenReads = 0;
        return FOUND_LEFT_GAP;
      }
    }
    driveForwardSteered();
    last = millis();
    moving = true;
  }
  stopMotors();
  return FOUND_NOTHING;
}

// Part 1 helper: drive until a wall is in front. Returns true if it stopped because of a wall.
bool driveUntilWall(float maxCm) {
  return driveUntil(maxCm, false) == FOUND_WALL;
}

// ---------- start position fix (essentials scenarios 5-8) ----------
// Near a wall, spinning on the spot swings the back corner into it, so this pivots on
// one wheel instead. A pivot turns half as fast as a spin, so 30 degrees takes twice as long.
const int PIVOT_30_MS = 2 * ROTATE_30_MS;

void fixStartPosition() {
  readSensorsFine();
  bool leftWall = zoneL < NO_WALL_ZONE;
  bool rightWall = zoneR < NO_WALL_ZONE;
  bool frontClear = zoneM >= FAR_ZONE;
  if (zoneL <= NEAR_ZONE && !rightWall && frontClear) {                  // 7: angled to left wall
    pivotClockwise(PIVOT_30_MS);
  } else if (zoneR <= NEAR_ZONE && !leftWall && frontClear) {            // 8: angled to right wall
    pivotAnticlockwise(PIVOT_30_MS);
  } else if (leftWall && rightWall && frontClear && zoneL + SIGNIFICANT_DIFF <= zoneR) {   // 5: close to left wall
    pivotClockwise(PIVOT_30_MS);           // S-shape: swing right ...
    pivotAnticlockwise(PIVOT_30_MS);       // ... then straighten up, now further from the left wall
  } else if (leftWall && rightWall && frontClear && zoneR + SIGNIFICANT_DIFF <= zoneL) {   // 6: close to right wall
    pivotAnticlockwise(PIVOT_30_MS);
    pivotClockwise(PIVOT_30_MS);
  }
  delay(SETTLE_MS);
}

// ---------- what to do at a wall ----------
int turnsDone = 0;
int deadEndsSeen = 0;

void finish() {
  stopMotors();
  digitalWrite(ledRight, HIGH);    // all LEDs on = task complete
  digitalWrite(ledMid, HIGH);
  digitalWrite(ledLeft, HIGH);
  servoLeft.detach();              // stop sending pulses so the wheels cannot creep
  servoRight.detach();
  while (true) {
    delay(1000);
  }
}

// After any turn: we are in a new corridor, so start counting again and wait to see a
// left wall before taking another left opening.
void afterTurn() {
  cmInCorridor = 0;
  leftWallSeen = false;
  leftOpenReads = 0;
}

void turnLeft() {                  // scenario 3
  rotateAnticlockwise(ROTATE_90_MS);
  delay(SETTLE_MS);
  turnsDone++;
  afterTurn();
  driveUntilWall(REENTRY_CM);
}

void turnRight() {                 // scenario 2
  rotateClockwise(ROTATE_90_MS);
  delay(SETTLE_MS);
  turnsDone++;
  afterTurn();
  driveUntilWall(REENTRY_CM);
}

void turnAround() {                // scenario 4
  rotateClockwise(ROTATE_180_MS);
  delay(SETTLE_MS);
  afterTurn();
}

// Part 2: a long dead end is a straight section exactly two walls long.
bool isLongDeadEnd() {
  return fabs(cmInCorridor - LONG_DEAD_END_CM) <= LONG_TOLERANCE_CM;
}

// A wall is in front of the robot: decide what to do from the two side sensors.
void onFrontWall() {
  readSensorsFine();                          // robot is stopped: take a slow, steady reading
  bool leftOpen = (zoneL == NO_WALL_ZONE);
  bool rightOpen = (zoneR == NO_WALL_ZONE);
  bool deadEnd = !leftOpen && !rightOpen;

  if (DEBUG) {
    Serial.print(F("wall ahead  L M R = "));
    Serial.print(zoneL); Serial.print(' '); Serial.print(zoneM); Serial.print(' '); Serial.print(zoneR);
    Serial.print(F("  cmInCorridor = "));
    Serial.println(cmInCorridor);
  }

  if (TASK == 1) finish();                    // blue: stop at the first wall
  if (TASK == 2 && turnsDone >= 1) finish();  // purple: stop at the wall after the turn

  if (deadEnd) {
    // green: spin 180 at the first dead end and drive back.
    // Part 2: spin 180 at every short dead end and keep exploring.
    if ((TASK == 4 && deadEndsSeen == 0) || (TASK == 5 && !isLongDeadEnd())) {
      deadEndsSeen++;
      turnAround();
      return;
    }
    finish();                                 // yellow, the end of green, and the long dead end in Part 2
  }

  creepForward(CORNER_CREEP_CM);              // wheels to the middle of the corner
  if (leftOpen) {                             // left first: in Part 2 this is the left-hand rule
    turnLeft();
  } else {
    turnRight();
  }
}

// Part 2: the left wall just ended while driving. Drive on until the wheels line up with
// the opening, then turn into it.
void onLeftGap() {
  if (DEBUG) Serial.println(F("left wall ended"));
  if (driveUntilWall(GAP_ADVANCE_CM)) {       // a wall came up first: it's a corner
    onFrontWall();
    return;
  }
  readSensorsFine();
  if (zoneL < NO_WALL_ZONE) return;           // the wall is back: it was a bad reading, carry on
  turnLeft();
}

// Part 2: keep a hand on the left wall. Take every left opening; at a wall, turn left
// if possible, otherwise right; spin out of short dead ends; stop at the long one.
void followWallLeft() {
  int found = driveUntil(1000, true);
  if (found == FOUND_LEFT_GAP) {
    onLeftGap();
  } else if (found == FOUND_WALL) {
    onFrontWall();
  }
}

// ---------- main ----------
void setup() {
  if (DEBUG) Serial.begin(9600);
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
  if (TASK == 5) {
    followWallLeft();              // Part 2
  } else if (driveUntilWall(1000)) {
    onFrontWall();                 // Part 1
  }
}
