// ELEC1601 Shield-Bot - Project Extension, Part 1 (sense-and-learn version)
//
// Set TASK before uploading:
//   0 = calibration: FORGETS everything learned, then slow spin of 4 x 90 degrees,
//       pause 3 s, then 2 s of slow driving
//   1 = Blue path   : fix a bad start, follow the corridor, stop at the wall in front
//   2 = Purple path : follow the corridor, turn at the wall, follow the next corridor, stop at the next wall
//   3 = Yellow path : turn at every wall until a dead end, then stop
//   4 = Green path  : as 3, spin 180 at the dead end, then do it again to get back to the start
//
// How it works:
//   STRAIGHTS: PD steering on the side sensors (works with one or two walls), plus a wheel
//     trim that slowly learns how much the robot naturally drifts, so the PD part only has
//     to correct real position errors. Side sensors use 9 frequencies instead of 5 for finer steps.
//   TURNS: the robot spins slowly and reads all three sensors on every step. The turn is
//     "seen" when the front looks down an open corridor AND the side sensor facing the wall
//     it is turning away from still sees that wall. From that moment it spins a little more
//     (the "extra"), then stops. The 180 works the same way.
//   LEARNING: after every turn the robot adds up how much its steering had to rotate it
//     over the next 25 cm. Rotating right after a left turn means it turned too far, so next
//     time the extra for left turns is reduced, and the other way round. Left turns, right
//     turns and the 180 each learn separately. What it learns is kept in EEPROM, so it survives
//     power-off and keeps improving from run to run. TASK 0 wipes it.
//
// The maze may be mirrored, so the robot never assumes a direction: it always turns
// towards whichever side is open.
#include <Servo.h>
#include <EEPROM.h>

const int TASK = 4;
const bool DEBUG = false;          // true = print decisions to the Serial Monitor (115200 baud)

Servo servoLeft;
Servo servoRight;

// ---------- pins ----------
const int irLedLeft = 10,  irReceiverLeft = 11;
const int irLedMid = 6,    irReceiverMid = 7;
const int irLedRight = 2,  irReceiverRight = 3;
const int ledRight = A0, ledMid = A1, ledLeft = A2;

// ---------- servo pulses (us) ----------
const int STOP_US = 1500;
const int LEFT_FWD = 1600,  RIGHT_FWD = 1400;
const int LEFT_BACK = 1400, RIGHT_BACK = 1600;
const int FAST_US = 100;           // offset from 1500 for normal driving (1600 / 1400)
const int SLOW_US = 50;            // offset from 1500 when a front wall is getting close
const int SPIN_SLOW_US = 50;       // offset from 1500 while turning

// ---------- calibration ----------
const float DRIVE_SPEED_CM_PER_S = 14.5;   // measured: 29 cm in 2 s
const float SLOW_SPEED_CM_PER_S = 8.0;     // MEASURE with TASK 0: distance driven in 2 s, divided by 2
const int ROTATE_90_MS = 570;              // fast spin, only used for the start-position pivots
const int ROTATE_30_MS = ROTATE_90_MS / 3;
const int PIVOT_30_MS = 2 * ROTATE_30_MS;  // a pivot turns half as fast as a spin
const int SLOW_ROTATE_90_MS = 1100;        // MEASURE with TASK 0: slow-spin time for 90 degrees
const int SETTLE_MS = 200;                 // pause so the wheels fully stop between moves

// ---------- sensor zones (0 = very close ... 5 = nothing) ----------
const int NO_WALL_ZONE = 5;
const int FAR_ZONE = 4;
const int NEAR_ZONE = 2;
const int SIGNIFICANT_DIFF = 2;
const int FRONT_WALL_ZONE = 2;     // front this close = wall ahead (must be seen FRONT_CONFIRM times)
const int FRONT_CONFIRM = 2;       // readings in a row needed (zone 0 or 1 stops immediately)
const int FRONT_SLOW_ZONE = 3;     // front this close = slow down for the final approach
const int CREEP_STOP_ZONE = 1;     // never creep forward if the front reads this close
const float CORNER_CREEP_CM = 4.0; // creep this far after stopping so the wheels are mid-corner

// ---------- reading resolution ----------
const long FAST_STEP = 1000;       // front while moving: 5 frequencies (~10 ms)
const long SIDE_STEP = 500;        // sides while moving: 9 frequencies (~18 ms), finer steps for steering
const long FINE_STEP = 100;        // when stopped and deciding: 41 frequencies (~82 ms per sensor)

// ---------- corridor-following controller ----------
const float CENTRE_ZONE = 3.0;     // MEASURE: side reading when the robot sits in the middle of a corridor
const float WALL_SEEN = 4.5;       // smoothed side reading below this = there is a wall on that side
const float SMOOTH = 0.5;          // 0..1, how much each new reading counts (1 = no smoothing)
const float KP_US = 25.0;          // steering per zone of error
const float KD_US = 60.0;          // steering per zone the error changed over the last D_WINDOW readings
const int D_WINDOW = 4;            // look this many readings back (~0.2 s) to see which way the robot
                                   // is heading: from one reading to the next the change is too small
const int MAX_STEER_US = 80;
const float TRIM_RATE = 0.005;     // how fast the wheel trim learns (per reading, per us of steering)
const float TRIM_MAX_US = 20.0;

// ---------- turning ----------
const int CLEAR_ZONE = 4;          // front reading this high = looking down an open corridor
const int EDGE_CONFIRM = 2;        // readings in a row that must look like the new corridor
const int IGNORE_FIRST_DEG = 30;   // never accept the new corridor in the first 30 degrees of a 90 turn
const int IGNORE_FIRST_AROUND_DEG = 100;  // ... or the first 100 of a 180: at a dead end the front can
                                          // read "clear" diagonally while it still faces the walls
const int SEARCH_PAST_DEG = 50;    // stop looking this far past the target and fall back to timing
const int START_EXTRA_DEG = 15;    // first guess at the extra spin after the edge (learning fixes it)
const int MAX_EXTRA_DEG = 45;

// ---------- learning from each turn ----------
const float LEARN_DIST_CM = 25.0;  // judge a turn by how much the steering rotates the robot over 25 cm
const float LEARN_MIN_CM = 12.0;   // corridor shorter than this = learn nothing from this turn
const float LEARN_GAIN = 0.3;      // correct this fraction of the measured error each time (kept low
                                   // so it stays stable even if SLOW_SPEED_CM_PER_S is not measured)
const int LEARN_MAX_STEP_DEG = 10; // never change a turn by more than this in one go

// Distance between the wheels, worked out from your own numbers: a full-speed spin turns
// 90 degrees (1.5708 rad) in ROTATE_90_MS with each wheel moving at DRIVE_SPEED.
const float TRACK_CM = 2.0 * DRIVE_SPEED_CM_PER_S * (ROTATE_90_MS / 1000.0) / 1.5708;
// How much one us of steering changes a wheel's speed, from the two measured speeds.
const float CM_S_PER_US = (DRIVE_SPEED_CM_PER_S - SLOW_SPEED_CM_PER_S) / (FAST_US - SLOW_US);

// =====================================================================================
// What the robot remembers (kept in EEPROM)
// =====================================================================================
const int TURN_LEFT = 0, TURN_RIGHT = 1, TURN_AROUND = 2;
const unsigned int LEARN_MAGIC = 0x1601;   // tells our saved data apart from a blank EEPROM

struct Learned {
  unsigned int magic;
  int extraMs[3];      // spin this long after the new corridor is seen (left, right, 180)
  int good90Ms[2];     // how long a good left / right 90 turn took: this is the robot's own
                       // measurement of its spin speed, so a wrong SLOW_ROTATE_90_MS matters less
  float trimUs;        // permanent steering offset for straight driving (+ = steer right)
};
Learned learned;

// ms of slow spin per degree, from the robot's own good turns (starts at SLOW_ROTATE_90_MS / 90).
float msPerDeg() { return (learned.good90Ms[TURN_LEFT] + learned.good90Ms[TURN_RIGHT]) / 180.0; }

float msPerDeg();    // spin speed, defined below once the learned data exists
int extraMax() { return (int)(MAX_EXTRA_DEG * msPerDeg()); }

void saveLearned() { EEPROM.put(0, learned); }   // put() only rewrites bytes that changed

void resetLearned() {
  learned.magic = LEARN_MAGIC;
  learned.good90Ms[TURN_LEFT] = SLOW_ROTATE_90_MS;      // set first: msPerDeg() uses these
  learned.good90Ms[TURN_RIGHT] = SLOW_ROTATE_90_MS;
  for (int k = 0; k < 3; k++) learned.extraMs[k] = (int)(START_EXTRA_DEG * msPerDeg());
  learned.trimUs = 0;
  saveLearned();
}

void loadLearned() {
  EEPROM.get(0, learned);
  bool sane = (learned.magic == LEARN_MAGIC);
  for (int k = 0; k < 2; k++) {
    if (learned.good90Ms[k] < SLOW_ROTATE_90_MS / 2 || learned.good90Ms[k] > SLOW_ROTATE_90_MS * 2) sane = false;
  }
  if (!sane) { resetLearned(); return; }
  for (int k = 0; k < 3; k++) {
    if (learned.extraMs[k] < 0 || learned.extraMs[k] > extraMax()) sane = false;
  }
  if (!(learned.trimUs >= -TRIM_MAX_US && learned.trimUs <= TRIM_MAX_US)) sane = false;
  if (!sane) resetLearned();          // blank or corrupted: start from the first guesses
}

// =====================================================================================
// Sensing
// =====================================================================================
int zoneL, zoneM, zoneR;           // whole-number zones, used for decisions
float levelL, levelM, levelR;      // same 0-5 scale with finer steps, used for steering
float fL, fM, fR;                  // smoothed levels

int irDetect(int irLedPin, int irReceiverPin, long frequency) {
  tone(irLedPin, frequency);
  delay(1);
  int ir = digitalRead(irReceiverPin);   // 0 = detected, 1 = not detected
  noTone(irLedPin);
  delay(1);
  return ir;
}

// Sweep 38-42 kHz and count the frequencies that were NOT detected, scaled to 0-5.
// More frequencies (smaller step) = finer steps but a slower reading.
float irLevel(int irLedPin, int irReceiverPin, long step) {
  int misses = 0;
  int checks = 0;
  for (long f = 38000; f <= 42000; f += step) {
    misses += irDetect(irLedPin, irReceiverPin, f);
    checks++;
  }
  return misses * 5.0 / checks;
}

int toZone(float level) { return (int)(level + 0.5); }   // round to the nearest whole zone

void readSensors() {               // while moving: about 46 ms for all three
  levelL = irLevel(irLedLeft, irReceiverLeft, SIDE_STEP);
  levelM = irLevel(irLedMid, irReceiverMid, FAST_STEP);
  levelR = irLevel(irLedRight, irReceiverRight, SIDE_STEP);
  zoneL = toZone(levelL); zoneM = toZone(levelM); zoneR = toZone(levelR);
}

void readSensorsFine() {           // when stopped: slow and steady
  levelL = irLevel(irLedLeft, irReceiverLeft, FINE_STEP);
  levelM = irLevel(irLedMid, irReceiverMid, FINE_STEP);
  levelR = irLevel(irLedRight, irReceiverRight, FINE_STEP);
  zoneL = toZone(levelL); zoneM = toZone(levelM); zoneR = toZone(levelR);
}

// Exponential smoothing: each new reading moves the smoothed value part of the way.
void smoothStart() { fL = levelL; fM = levelM; fR = levelR; }
void smoothUpdate() {
  fL += SMOOTH * (levelL - fL);
  fM += SMOOTH * (levelM - fM);
  fR += SMOOTH * (levelR - fR);
}

// =====================================================================================
// Motion
// =====================================================================================
void setWheels(int left, int right) {
  servoLeft.writeMicroseconds(left);
  servoRight.writeMicroseconds(right);
}

void stopMotors() { setWheels(STOP_US, STOP_US); }

void drive(int left, int right, long ms) {
  setWheels(left, right);
  delay(ms);
  stopMotors();
}

int msForCm(float cm) { return (int)(cm / DRIVE_SPEED_CM_PER_S * 1000.0); }

void stepForward() { drive(LEFT_FWD, RIGHT_FWD, msForCm(1.0)); }
void stepBack()    { drive(LEFT_BACK, RIGHT_BACK, msForCm(1.0)); }

// Spin on the spot. dir = +1 clockwise, -1 anticlockwise.
// Clockwise = left wheel forward (1500 + us) and right wheel backward (also 1500 + us,
// because the right servo is mounted the other way round).
void spin(int dir, int us) { setWheels(STOP_US + dir * us, STOP_US + dir * us); }
void spinFor(int dir, int us, long ms) { spin(dir, us); delay(ms); stopMotors(); }

void pivotClockwise(int ms)     { drive(LEFT_FWD, STOP_US, ms); }    // turn around the right wheel
void pivotAnticlockwise(int ms) { drive(STOP_US, RIGHT_FWD, ms); }   // turn around the left wheel

// =====================================================================================
// Corridor following (scenario 1)
// =====================================================================================
float errHist[D_WINDOW];   // the last few errors, oldest first in the ring, for the D term
int errIdx = 0;
int prevMode = -1;   // which walls were used last reading, so switching walls does not cause a jolt
int lastMode = 0;    // 3 = both walls, 1 = left only, 2 = right only, 0 = none

// error > 0 means "too close to the LEFT wall, steer right".
//   both walls : error = right - left       (0 when centred)
//   left only  : error = CENTRE - left      (hold a set distance from the left wall)
//   right only : error = right - CENTRE     (hold a set distance from the right wall)
//   no walls   : error = 0                  (drive straight)
// Steering = KP * error + KD * (change in error over the last D_WINDOW readings) + learned trim.
// The change tells us which way the robot is pointing: if it is getting closer to a wall,
// it is angled towards it, so steer harder before it gets there.
// Returns the PD part (without the trim), which is what the learning looks at.
int driveForwardSteered(int baseUs) {
  bool leftSeen = fL < WALL_SEEN;
  bool rightSeen = fR < WALL_SEEN;
  int mode;
  float error;
  if (leftSeen && rightSeen) { mode = 3; error = fR - fL; }
  else if (leftSeen)         { mode = 1; error = CENTRE_ZONE - fL; }
  else if (rightSeen)        { mode = 2; error = fR - CENTRE_ZONE; }
  else                       { mode = 0; error = 0; }
  if (mode != prevMode) {               // different walls: old errors mean nothing now
    for (int i = 0; i < D_WINDOW; i++) errHist[i] = error;
    prevMode = mode;
  }
  lastMode = mode;

  float change = error - errHist[errIdx];          // errHist[errIdx] is the oldest one
  errHist[errIdx] = error;
  errIdx = (errIdx + 1) % D_WINDOW;
  float steer = KP_US * error + KD_US * change;

  int limit = min(MAX_STEER_US, baseUs - 10);   // never slow a wheel so much that it reverses
  int pd = constrain((int)steer, -limit, limit);
  int total = constrain(pd + (int)learned.trimUs, -limit, limit);
  int left = STOP_US + baseUs;
  int right = STOP_US - baseUs;
  if (total > 0) right += total;   // slow the right wheel: veer right
  else           left += total;    // total is negative: slow the left wheel, veer left
  setWheels(left, right);
  return pd;
}

// ---------- learning: how good was the last turn? ----------
// After a turn, add up how much the steering rotates the robot over the next LEARN_DIST_CM.
// Slowing one wheel by s us makes the robot rotate at  s * CM_S_PER_US / TRACK_CM  rad/s,
// so adding that up over time gives the degrees the steering had to fix. If the turn was
// perfect, steering adds up to about zero. Positive = the steering turned the robot clockwise.
bool learnPending = false;
int learnKind, learnDir;
float learnRotCw;   // degrees the steering rotated the robot clockwise since the turn

void startLearning(int kind, int dir) {
  learnPending = true; learnKind = kind; learnDir = dir; learnRotCw = 0;
}

// Turning anticlockwise (dir -1) too far leaves the robot pointing left, so the steering has
// to rotate it clockwise (positive). Turning clockwise (dir +1) too far needs anticlockwise.
// So  overTurn = -dir * learnRotCw  is positive when the turn went too far.
void finishLearning(float travelled) {
  if (!learnPending) return;
  learnPending = false;
  if (travelled < LEARN_MIN_CM) return;                // corridor too short to judge
  float overTurnDeg = -learnDir * learnRotCw;
  int maxStep = (int)(LEARN_MAX_STEP_DEG * msPerDeg());
  float wanted = LEARN_GAIN * overTurnDeg * msPerDeg();
  int change = (int)constrain(wanted, -maxStep, maxStep);   // clamp before converting to int
  learned.extraMs[learnKind] = constrain(learned.extraMs[learnKind] - change, 0, extraMax());
  saveLearned();
  if (DEBUG) {
    Serial.print(F("learn kind=")); Serial.print(learnKind);
    Serial.print(F(" overTurnDeg=")); Serial.print(overTurnDeg);
    Serial.print(F(" extraMs now=")); Serial.println(learned.extraMs[learnKind]);
  }
}

// Follow the corridor until a wall is in front, or until maxCm has been driven.
// Returns true if it stopped because of a wall.
bool driveUntilWall(float maxCm) {
  readSensors();
  smoothStart();
  prevMode = -1;
  float travelled = 0;
  float speed = 0;               // cm/s of whatever the wheels are doing right now
  int frontHits = 0;
  int lastPd = 0;                // steering that was applied during the last interval
  bool hitWall = false;
  unsigned long last = millis();
  while (travelled < maxCm) {
    readSensors();
    smoothUpdate();
    unsigned long now = millis();
    float dt = (now - last) / 1000.0;
    travelled += dt * speed;
    last = now;
    if (learnPending) {
      learnRotCw += lastPd * CM_S_PER_US / TRACK_CM * 57.3 * dt;   // degrees
      if (travelled >= LEARN_DIST_CM) finishLearning(travelled);
    }

    if (zoneM <= FRONT_WALL_ZONE) frontHits++; else frontHits = 0;
    if (zoneM < FRONT_WALL_ZONE || frontHits >= FRONT_CONFIRM) { hitWall = true; break; }

    int base = (fM <= FRONT_SLOW_ZONE) ? SLOW_US : FAST_US;
    speed = (base == SLOW_US) ? SLOW_SPEED_CM_PER_S : DRIVE_SPEED_CM_PER_S;
    int pd = driveForwardSteered(base);
    lastPd = pd;

    if (lastMode == 3 && !learnPending) {                  // both walls, settled: learn the wheel trim
      learned.trimUs = constrain(learned.trimUs + TRIM_RATE * pd, -TRIM_MAX_US, TRIM_MAX_US);
    }
  }
  stopMotors();
  finishLearning(travelled);     // corridor ended before LEARN_DIST_CM: judge on what we have
  return hitWall;
}

// Creep forward in 1 cm steps, checking the front before every step. Returns the steps taken.
int creepGuarded(float cm) {
  int steps = 0;
  while (steps < cm) {
    if (toZone(irLevel(irLedMid, irReceiverMid, FINE_STEP)) <= CREEP_STOP_ZONE) break;
    stepForward();
    steps++;
  }
  return steps;
}

void backUp(int steps) {
  for (int i = 0; i < steps; i++) stepBack();
}

// =====================================================================================
// Turning (scenarios 2, 3 and 4)
// =====================================================================================
// Spin slowly and read all three sensors on every step. The new corridor has been reached
// when, for EDGE_CONFIRM readings in a row:
//   - the front reads clear (it is looking down an open corridor), and
//   - the "trailing" side still sees a wall. Trailing = the side facing the wall we are
//     turning away from: the right sensor when spinning anticlockwise, the left when clockwise.
//     This rejects a clear front that is really a gap or the old corridor.
// That moment (the "edge") is always the same few degrees short of straight, because the
// wall that blocks the front sensor until then is the long corridor wall. So the robot then
// spins a learned extra amount and stops. Only the edge is used, never the far side of the
// opening, so it cannot be fooled by the opening running into the corridor it came from.
//
// If nothing looks like a corridor before targetDeg + SEARCH_PAST_DEG, it spins back to the
// time the turn should take at the learned spin speed, and lights the middle LED.
// Returns true if it saw the corridor.
bool senseTurn(int dir, int kind, int targetDeg) {
  long ignoreUntil = (long)((kind == TURN_AROUND ? IGNORE_FIRST_AROUND_DEG : IGNORE_FIRST_DEG) * msPerDeg());
  long giveUp = (long)((targetDeg + SEARCH_PAST_DEG) * msPerDeg());
  bool trailingIsRight = (dir < 0);

  long edge = -1, firstSeen = 0;
  int seenCount = 0;
  unsigned long start = millis();
  spin(dir, SPIN_SLOW_US);
  while ((long)(millis() - start) < giveUp) {
    readSensors();
    long t = (long)(millis() - start);
    if (t < ignoreUntil) continue;
    float trailing = trailingIsRight ? levelR : levelL;
    bool looksLikeCorridor = (zoneM >= CLEAR_ZONE) && (trailing < WALL_SEEN);
    if (looksLikeCorridor) {
      if (seenCount == 0) firstSeen = t;
      seenCount++;
      if (seenCount >= EDGE_CONFIRM) { edge = firstSeen; break; }
    } else {
      seenCount = 0;
    }
  }

  bool found = (edge >= 0);
  long aim = found ? edge + learned.extraMs[kind] : (long)(targetDeg * msPerDeg());   // no edge: timing

  long now = (long)(millis() - start);
  if (now < aim) {
    delay(aim - now);              // keep spinning smoothly up to the aim, then stop
    stopMotors();
  } else {
    stopMotors();                  // already past the aim: stop and come back
    delay(SETTLE_MS);
    spinFor(-dir, SPIN_SLOW_US, now - aim);
  }
  delay(SETTLE_MS);

  if (found && kind != TURN_AROUND) {   // remember how long a good 90 turn takes (spin speed)
    learned.good90Ms[kind] = constrain((int)((learned.good90Ms[kind] + aim) / 2),
                                       SLOW_ROTATE_90_MS / 2, SLOW_ROTATE_90_MS * 2);
  }

  if (DEBUG) {
    Serial.print(F("turn kind=")); Serial.print(kind);
    Serial.print(F(" edge=")); Serial.print(edge);
    Serial.print(F(" extra=")); Serial.print(learned.extraMs[kind]);
    Serial.print(F(" aim=")); Serial.print(aim);
    Serial.println(found ? F(" SENSOR") : F(" TIMING"));
  }
  return found;
}

// LEDs while turning: left LED = turning left, right LED = turning right, both = 180.
// After the turn the middle LED stays on if the sensors did not see the corridor.
void doTurn(int dir, int kind, int deg) {
  digitalWrite(ledLeft,  (dir < 0 || kind == TURN_AROUND) ? HIGH : LOW);
  digitalWrite(ledRight, (dir > 0 || kind == TURN_AROUND) ? HIGH : LOW);
  bool found = senseTurn(dir, kind, deg);
  digitalWrite(ledLeft, LOW);
  digitalWrite(ledRight, LOW);
  digitalWrite(ledMid, found ? LOW : HIGH);
  if (found) startLearning(kind, dir);   // only learn from turns where the edge was seen
}

// =====================================================================================
// Start position fix (essentials scenarios 5-8), unchanged
// =====================================================================================
// Near a wall, spinning on the spot swings the back corner into it, so this pivots on one wheel.
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

// =====================================================================================
// What to do at a wall
// =====================================================================================
int turnsDone = 0;
int deadEndsSeen = 0;

void finish() {
  stopMotors();
  saveLearned();
  digitalWrite(ledRight, HIGH);    // all LEDs on = task complete
  digitalWrite(ledMid, HIGH);
  digitalWrite(ledLeft, HIGH);
  servoLeft.detach();              // stop sending pulses so the wheels cannot creep
  servoRight.detach();
  while (true) {
    delay(1000);
  }
}

// A wall is in front of the robot.
void onFrontWall() {
  saveLearned();                              // keep the trim learned on the last straight
  if (TASK == 1) finish();                    // blue: stop at the first wall
  if (TASK == 2 && turnsDone >= 1) finish();  // purple: stop at the wall after the turn

  // Look twice: once where it stopped, once after creeping to the middle of the corner.
  // Depending on how far forward the side sensors point, one of the two positions can make
  // the open side's sensor hit the front wall instead of looking down the new corridor.
  // So a side counts as open if EITHER look saw it open. A real dead end is closed in both.
  readSensorsFine();
  bool leftOpen = (zoneL == NO_WALL_ZONE);
  bool rightOpen = (zoneR == NO_WALL_ZONE);
  int crept = creepGuarded(CORNER_CREEP_CM);
  readSensorsFine();
  leftOpen = leftOpen || (zoneL == NO_WALL_ZONE);
  rightOpen = rightOpen || (zoneR == NO_WALL_ZONE);
  bool deadEnd = !leftOpen && !rightOpen;

  if (DEBUG) {
    Serial.print(F("wall L=")); Serial.print(levelL);
    Serial.print(F(" M=")); Serial.print(levelM);
    Serial.print(F(" R=")); Serial.print(levelR);
    Serial.print(F(" crept=")); Serial.print(crept);
    Serial.print(F(" trim=")); Serial.println(learned.trimUs);
  }

  if (deadEnd) {
    if (TASK == 4 && deadEndsSeen == 0) {     // green: back off, spin 180, drive back
      deadEndsSeen++;
      backUp(crept);                          // give the spin the room it had before the creep
      doTurn(+1, TURN_AROUND, 180);
      return;
    }
    finish();                                 // yellow, and the end of green
  }

  if (leftOpen) {
    doTurn(-1, TURN_LEFT, 90);                // scenario 3: left = anticlockwise
  } else {
    doTurn(+1, TURN_RIGHT, 90);               // scenario 2: right = clockwise
  }
  turnsDone++;
}

// Calibration run (TASK 0). Also wipes everything the robot has learned.
// 1. Spins slowly for 4 x SLOW_ROTATE_90_MS. If it ends facing exactly where it started,
//    SLOW_ROTATE_90_MS is right. Turned too far: lower it. Not far enough: raise it.
// 2. Waits 3 s, then drives at the slow speed for 2 s. Measure the distance and divide
//    by 2 to get SLOW_SPEED_CM_PER_S.
void calibrate() {
  resetLearned();
  spinFor(+1, SPIN_SLOW_US, 4L * SLOW_ROTATE_90_MS);
  delay(3000);
  drive(STOP_US + SLOW_US, STOP_US - SLOW_US, 2000);
  finish();
}

// =====================================================================================
// Main
// =====================================================================================
void setup() {
  if (DEBUG) Serial.begin(115200);
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

  loadLearned();
  if (DEBUG) {
    Serial.print(F("learned extra L/R/180 = "));
    Serial.print(learned.extraMs[TURN_LEFT]); Serial.print(F(" / "));
    Serial.print(learned.extraMs[TURN_RIGHT]); Serial.print(F(" / "));
    Serial.print(learned.extraMs[TURN_AROUND]);
    Serial.print(F("  trim=")); Serial.println(learned.trimUs);
  }
  delay(5000);               // 5 s before anything happens

  if (TASK == 0) calibrate();
  fixStartPosition();
}

void loop() {
  if (driveUntilWall(1000)) {
    onFrontWall();
  }
}
