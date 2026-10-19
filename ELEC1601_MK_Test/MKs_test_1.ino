// ELEC1601 Shield-Bot - Project Extension, Part 1 (closed-loop version)
//
// Set TASK before uploading:
//   0 = calibration: slow spin of 4 x 90 degrees, pause 3 s, then 2 s of slow driving
//   1 = Blue path   : fix a bad start, follow the corridor, stop at the wall in front
//   2 = Purple path : follow the corridor, turn at the wall, follow the next corridor, stop at the next wall
//   3 = Yellow path : turn at every wall until a dead end, then stop
//   4 = Green path  : as 3, spin 180 at the dead end, then do it again to get back to the start
//
// What changed from the timed version:
//   1. Corridor following is proportional + derivative (PD), and it still steers when only
//      one wall is visible (it holds a set distance from that wall).
//   2. Readings are smoothed, and a front wall must be seen twice in a row before stopping
//      (one noisy reading can no longer trigger a second, unwanted turn).
//   3. The robot slows down as a front wall gets close, so it stops in a repeatable place.
//   4. The creep into a corner checks the front sensor before every 1 cm step.
//   5. Turns are sensor-guided: the robot spins slowly, watches the front sensor, finds the
//      open corridor, and stops in the middle of it, instead of trusting a fixed time.
//
// The maze may be mirrored, so the robot never assumes a direction: it always turns
// towards whichever side is open.
#include <Servo.h>

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
const int LEFT_STOP = 1500, RIGHT_STOP = 1500;
const int LEFT_FWD = 1600,  RIGHT_FWD = 1400;
const int LEFT_BACK = 1400, RIGHT_BACK = 1600;
const int FAST_US = 100;           // offset from 1500 for normal driving (1600 / 1400)
const int SLOW_US = 50;            // offset from 1500 when a front wall is getting close
const int SPIN_SLOW_US = 50;       // offset from 1500 for sensor-guided turns

// ---------- calibration ----------
const float DRIVE_SPEED_CM_PER_S = 14.5;   // measured: 29 cm in 2 s
const float SLOW_SPEED_CM_PER_S = 8.0;     // MEASURE with TASK 0: distance driven in 2 s, divided by 2
const int ROTATE_90_MS = 577;              // fast spin, only used for the start-position pivots now
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

// ---------- corridor-following controller ----------
const float CENTRE_ZONE = 3.0;     // MEASURE: side reading when the robot sits in the middle of a corridor
const float WALL_SEEN = 4.5;       // smoothed side reading below this = there is a wall on that side
const float SMOOTH = 0.5;          // 0..1, how much each new reading counts (1 = no smoothing)
const float KP_US = 25.0;          // steering per zone of error
const float KD_US = 40.0;          // steering per zone the error changed since the last reading
const int MAX_STEER_US = 80;

// ---------- sensor-guided turn ----------
const int CLEAR_ZONE = 4;          // front reading this high = looking down an open corridor
const int WATCH_BEFORE_DEG = 60;   // start watching this many degrees before the expected opening
const int WATCH_AFTER_DEG = 60;    // give up this many degrees after it
const int MIN_GAP_DEG = 5;         // an opening narrower than this is treated as noise
const int GAP_CONFIRM = 3;         // readings in a row needed to say an opening started or ended
const int MAX_CORRECTION_PCT = 50; // ignore an opening more than this % away from the expected angle
const int EXTRA_WATCH_DEG = 40;    // if the opening has started, keep spinning up to this much longer
                                   // to see it end (covers a robot that spins slower than calibrated)

// =====================================================================================
// Sensing
// =====================================================================================
int zoneL, zoneM, zoneR;           // latest raw readings
float fL, fM, fR;                  // smoothed readings

int irDetect(int irLedPin, int irReceiverPin, long frequency) {
  tone(irLedPin, frequency);
  delay(1);
  int ir = digitalRead(irReceiverPin);   // 0 = detected, 1 = not detected
  noTone(irLedPin);
  delay(1);
  return ir;
}

// Sweep 38-42 kHz and count the frequencies that were NOT detected, scaled to 0-5.
// FAST_STEP (5 checks, ~10 ms per sensor) is used while moving.
// FINE_STEP (41 checks, ~82 ms per sensor) is used only when stopped and deciding something.
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

void readSensors() {
  zoneL = irDistance(irLedLeft, irReceiverLeft, FAST_STEP);
  zoneM = irDistance(irLedMid, irReceiverMid, FAST_STEP);
  zoneR = irDistance(irLedRight, irReceiverRight, FAST_STEP);
}

void readSensorsFine() {
  zoneL = irDistance(irLedLeft, irReceiverLeft, FINE_STEP);
  zoneM = irDistance(irLedMid, irReceiverMid, FINE_STEP);
  zoneR = irDistance(irLedRight, irReceiverRight, FINE_STEP);
}

// Exponential smoothing: each new reading moves the smoothed value part of the way.
// This stops a single bad reading from jerking the steering.
void smoothStart() { fL = zoneL; fM = zoneM; fR = zoneR; }
void smoothUpdate() {
  fL += SMOOTH * (zoneL - fL);
  fM += SMOOTH * (zoneM - fM);
  fR += SMOOTH * (zoneR - fR);
}

// =====================================================================================
// Motion
// =====================================================================================
void setWheels(int left, int right) {
  servoLeft.writeMicroseconds(left);
  servoRight.writeMicroseconds(right);
}

void stopMotors() { setWheels(LEFT_STOP, RIGHT_STOP); }

void drive(int left, int right, long ms) {
  setWheels(left, right);
  delay(ms);
  stopMotors();
}

int msForCm(float cm) { return (int)(cm / DRIVE_SPEED_CM_PER_S * 1000.0); }

void stepForward() { drive(LEFT_FWD, RIGHT_FWD, msForCm(1.0)); }
void stepBack()    { drive(LEFT_BACK, RIGHT_BACK, msForCm(1.0)); }

// Spin on the spot. dir = +1 clockwise, -1 anticlockwise.
// Clockwise means left wheel forward (1500 + us) and right wheel backward (also 1500 + us,
// because the right servo is mounted the other way round).
void spin(int dir, int us) { setWheels(STOP_US + dir * us, STOP_US + dir * us); }
void spinFor(int dir, int us, long ms) { spin(dir, us); delay(ms); stopMotors(); }

void pivotClockwise(int ms)     { drive(LEFT_FWD, RIGHT_STOP, ms); }   // turn around the right wheel
void pivotAnticlockwise(int ms) { drive(LEFT_STOP, RIGHT_FWD, ms); }   // turn around the left wheel

// =====================================================================================
// Corridor following (scenario 1, closed loop)
// =====================================================================================
float prevError = 0;
int prevMode = -1;   // which walls were used last reading, so switching walls does not cause a jolt

// error > 0 means "too close to the LEFT wall, steer right".
//   both walls : error = right - left       (0 when centred)
//   left only  : error = CENTRE - left      (hold a set distance from the left wall)
//   right only : error = right - CENTRE     (hold a set distance from the right wall)
//   no walls   : error = 0                  (drive straight)
// Steering = KP * error + KD * (change in error). The D term reacts to the robot being
// angled: if the error is growing, the robot is heading towards a wall, so it steers harder
// before it gets there.
void driveForwardSteered(int baseUs) {
  bool leftSeen = fL < WALL_SEEN;
  bool rightSeen = fR < WALL_SEEN;
  int mode;
  float error;
  if (leftSeen && rightSeen) { mode = 3; error = fR - fL; }
  else if (leftSeen)         { mode = 1; error = CENTRE_ZONE - fL; }
  else if (rightSeen)        { mode = 2; error = fR - CENTRE_ZONE; }
  else                       { mode = 0; error = 0; }
  if (mode != prevMode) { prevError = error; prevMode = mode; }

  float steer = KP_US * error + KD_US * (error - prevError);
  prevError = error;

  int limit = min(MAX_STEER_US, baseUs - 10);   // never slow a wheel so much that it reverses
  int s = constrain((int)steer, -limit, limit);
  int left = STOP_US + baseUs;
  int right = STOP_US - baseUs;
  if (s > 0) right += s;     // slow the right wheel: veer right
  else       left += s;      // s is negative: slow the left wheel, veer left
  setWheels(left, right);
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
  unsigned long last = millis();
  while (travelled < maxCm) {
    readSensors();
    smoothUpdate();
    unsigned long now = millis();
    travelled += (now - last) / 1000.0 * speed;
    last = now;

    if (zoneM <= FRONT_WALL_ZONE) frontHits++; else frontHits = 0;
    if (zoneM < FRONT_WALL_ZONE || frontHits >= FRONT_CONFIRM) {
      stopMotors();
      return true;
    }
    int base = (fM <= FRONT_SLOW_ZONE) ? SLOW_US : FAST_US;
    speed = (base == SLOW_US) ? SLOW_SPEED_CM_PER_S : DRIVE_SPEED_CM_PER_S;
    driveForwardSteered(base);
  }
  stopMotors();
  return false;
}

// Creep forward in 1 cm steps, checking the front before every step. Returns the steps taken.
int creepGuarded(float cm) {
  int steps = 0;
  while (steps < cm) {
    if (irDistance(irLedMid, irReceiverMid, FINE_STEP) <= CREEP_STOP_ZONE) break;
    stepForward();
    steps++;
  }
  return steps;
}

void backUp(int steps) {
  for (int i = 0; i < steps; i++) stepBack();
}

// =====================================================================================
// Sensor-guided turn (scenarios 2, 3 and 4, closed loop)
// =====================================================================================
// Spin slowly towards an opening expected at targetDeg (90 for a corner, 180 for a dead end).
// While spinning, watch the front sensor. The open corridor is where the front reads clear.
// Note the time the opening starts and the time it ends, then spin back to the middle of it.
// The middle of the opening is the direction of the corridor, so the final heading comes from
// what the robot sees. The timing constant only decides when to start watching and when to
// give up, so a wrong SLOW_ROTATE_90_MS no longer means a wrong turn.
//
// Spinning a little past the opening is safe: a spin on the spot sweeps a circle, so if the
// robot can spin 90 degrees at this spot without touching, it can spin 110 as well.
//
// Returns true if it found the opening, false if it had to fall back to timing.
bool sensorSpin(int dir, int targetDeg) {
  float msPerDeg = SLOW_ROTATE_90_MS / 90.0;
  long watchFrom = (long)((targetDeg - WATCH_BEFORE_DEG) * msPerDeg);
  long giveUp    = (long)((targetDeg + WATCH_AFTER_DEG) * msPerDeg);
  long expected  = (long)(targetDeg * msPerDeg);
  long hardStop  = giveUp + (long)(EXTRA_WATCH_DEG * msPerDeg);

  long gapStart = -1, gapEnd = -1;
  long firstClear = 0, firstBlocked = 0;
  int clearCount = 0, blockedCount = 0;

  unsigned long start = millis();
  spin(dir, SPIN_SLOW_US);
  while ((long)(millis() - start) < watchFrom) { }   // still facing walls: nothing to look at yet

  while (true) {
    long now = (long)(millis() - start);
    if (now >= hardStop) break;                   // absolute limit
    if (now >= giveUp && gapStart < 0) break;     // opening never appeared: give up
    int z = irDistance(irLedMid, irReceiverMid, FAST_STEP);
    long t = (long)(millis() - start);
    if (gapStart < 0) {                       // waiting for the opening to appear
      if (z >= CLEAR_ZONE) {
        if (clearCount == 0) firstClear = t;
        clearCount++;
        if (clearCount >= GAP_CONFIRM) { gapStart = firstClear; blockedCount = 0; }
      } else {
        clearCount = 0;
      }
    } else {                                  // inside the opening, waiting for it to end
      if (z < CLEAR_ZONE) {
        if (blockedCount == 0) firstBlocked = t;
        blockedCount++;
        if (blockedCount >= GAP_CONFIRM) {
          if (firstBlocked - gapStart >= (long)(MIN_GAP_DEG * msPerDeg)) {
            gapEnd = firstBlocked;            // a real opening: done
            break;
          }
          gapStart = -1;                      // too narrow, it was noise: keep looking
          clearCount = 0;
        }
      } else {
        blockedCount = 0;
      }
    }
  }
  stopMotors();
  long stoppedAt = (long)(millis() - start);
  delay(SETTLE_MS);

  long aim = expected;                        // fallback: plain timing
  bool found = false;
  if (gapStart >= 0 && gapEnd >= 0) {
    long middle = (gapStart + gapEnd) / 2;
    if (labs(middle - expected) <= expected * MAX_CORRECTION_PCT / 100) { aim = middle; found = true; }
  }

  long back = stoppedAt - aim;                // how far past the aim we are, in ms of slow spin
  if (back > 0)      spinFor(-dir, SPIN_SLOW_US, back);
  else if (back < 0) spinFor(dir, SPIN_SLOW_US, -back);
  delay(SETTLE_MS);

  if (DEBUG) {
    Serial.print(F("spin dir=")); Serial.print(dir);
    Serial.print(F(" target=")); Serial.print(targetDeg);
    Serial.print(F(" gap=")); Serial.print(gapStart); Serial.print(F("..")); Serial.print(gapEnd);
    Serial.print(F(" stop=")); Serial.print(stoppedAt);
    Serial.print(F(" aim=")); Serial.print(aim);
    Serial.println(found ? F(" SENSOR") : F(" TIMING"));
  }
  return found;
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
  digitalWrite(ledRight, HIGH);    // all LEDs on = task complete
  digitalWrite(ledMid, HIGH);
  digitalWrite(ledLeft, HIGH);
  servoLeft.detach();              // stop sending pulses so the wheels cannot creep
  servoRight.detach();
  while (true) {
    delay(1000);
  }
}

// LEDs while turning: left LED = turning left, right LED = turning right, both = 180.
// After the turn the middle LED stays on if the turn had to fall back to timing
// (the sensor did not find the opening). Watch it during testing.
void sensorTurn(int dir, int deg) {
  digitalWrite(ledLeft,  (dir < 0 || deg == 180) ? HIGH : LOW);
  digitalWrite(ledRight, (dir > 0 || deg == 180) ? HIGH : LOW);
  bool found = sensorSpin(dir, deg);
  digitalWrite(ledLeft, LOW);
  digitalWrite(ledRight, LOW);
  digitalWrite(ledMid, found ? LOW : HIGH);
}

// A wall is in front of the robot.
void onFrontWall() {
  if (TASK == 1) finish();                    // blue: stop at the first wall
  if (TASK == 2 && turnsDone >= 1) finish();  // purple: stop at the wall after the turn

  // Move the wheels to the middle of the corner, then look. Reading from the middle of the
  // corner means the open side's sensor points straight down the new corridor.
  int crept = creepGuarded(CORNER_CREEP_CM);
  readSensorsFine();
  bool leftOpen = (zoneL == NO_WALL_ZONE);
  bool rightOpen = (zoneR == NO_WALL_ZONE);
  bool deadEnd = !leftOpen && !rightOpen;

  if (DEBUG) {
    Serial.print(F("wall L=")); Serial.print(zoneL);
    Serial.print(F(" M=")); Serial.print(zoneM);
    Serial.print(F(" R=")); Serial.print(zoneR);
    Serial.print(F(" crept=")); Serial.println(crept);
  }

  if (deadEnd) {
    if (TASK == 4 && deadEndsSeen == 0) {     // green: back off, spin 180, drive back
      deadEndsSeen++;
      backUp(crept);                          // give the spin the room it had before the creep
      sensorTurn(+1, 180);
      return;
    }
    finish();                                 // yellow, and the end of green
  }

  if (leftOpen) {
    sensorTurn(-1, 90);                       // scenario 3: left = anticlockwise
  } else {
    sensorTurn(+1, 90);                       // scenario 2: right = clockwise
  }
  turnsDone++;
}

// Calibration run (TASK 0).
// 1. Spins slowly for 4 x SLOW_ROTATE_90_MS. If it ends facing exactly where it started,
//    SLOW_ROTATE_90_MS is right. Turned too far: lower it. Not far enough: raise it.
// 2. Waits 3 s, then drives at the slow speed for 2 s. Measure the distance and divide
//    by 2 to get SLOW_SPEED_CM_PER_S.
void calibrate() {
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
  delay(5000);               // 5 s before anything happens

  if (TASK == 0) calibrate();
  fixStartPosition();
}

void loop() {
  if (driveUntilWall(1000)) {
    onFrontWall();
  }
}
