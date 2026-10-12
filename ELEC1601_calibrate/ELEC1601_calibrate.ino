// ELEC1601 calibration sketch - finds the numbers for ELEC1601_extension.ino
//
// 1. Upload, open Serial Monitor at 9600 baud, set the line ending to "Newline".
// 2. Type a command and press Enter. Lowercase letters run a test, uppercase letters
//    followed by a number change a value (e.g. R1410). Type ? to see everything.
// 3. When happy, type ? and copy the printed values into ELEC1601_extension.ino.
//
// Every test waits 3 s first so you can put the robot down. Keep the USB cable slack.
#include <Servo.h>

Servo servoLeft;
Servo servoRight;

// ---------- pins (same as the main sketches) ----------
const int irLedLeft = 10,  irReceiverLeft = 11;
const int irLedMid = 6,    irReceiverMid = 7;
const int irLedRight = 2,  irReceiverRight = 3;

// ---------- values being calibrated (start from the extension sketch numbers) ----------
int leftFwd = 1600;              // L: left wheel forward pulse
int rightFwd = 1400;             // R: right wheel forward pulse
float speedCmPerS = 12.5;        // V: drive speed (or measure with d then D)
int rotate90Ms = 577;            // T: spin time for 90 degrees
float cornerCreepCm = 5.0;       // C: creep after stopping at a corner
float gapAdvanceCm = 11.0;       // G: drive past the end of a left wall before turning
int frontWallZone = 2;           // F: mid sensor zone that counts as "wall ahead"

const int STOP_US = 1500;
const int STEER_US = 40;
const int SETTLE_MS = 200;
const float REENTRY_CM = 12.0;
const int DRIVE_TEST_MS = 2000;  // the d test drives for exactly this long

int zoneL, zoneM, zoneR;

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

void readSensors() {
  zoneL = irDistance(irLedLeft, irReceiverLeft);
  zoneM = irDistance(irLedMid, irReceiverMid);
  zoneR = irDistance(irLedRight, irReceiverRight);
}

void printZones() {
  Serial.print(F("  zones L M R: "));
  Serial.print(zoneL); Serial.print(F(" "));
  Serial.print(zoneM); Serial.print(F(" "));
  Serial.println(zoneR);
}

// ---------- motion ----------
// Backward pulses mirror the forward ones around 1500, so changing L/R fixes both directions.
int leftBack()  { return 3000 - leftFwd; }
int rightBack() { return 3000 - rightFwd; }

void stopMotors() {
  servoLeft.writeMicroseconds(STOP_US);
  servoRight.writeMicroseconds(STOP_US);
}

void drive(int left, int right, int ms) {
  servoLeft.writeMicroseconds(left);
  servoRight.writeMicroseconds(right);
  delay(ms);
  stopMotors();
}

int msForCm(float cm) { return (int)(cm / speedCmPerS * 1000.0); }

void spinClockwise(int ms)     { drive(leftFwd, rightBack(), ms); }
void spinAnticlockwise(int ms) { drive(leftBack(), rightFwd, ms); }

void creepForward(float cm) {
  for (float done = 0; done < cm; done += 1.0) {
    drive(leftFwd, rightFwd, msForCm(1.0));
  }
}

void driveSteered() {
  int left = leftFwd, right = rightFwd;
  bool bothWalls = (zoneL < 5) && (zoneR < 5);
  if (bothWalls && zoneL < zoneR) right = rightFwd + STEER_US;
  else if (bothWalls && zoneR < zoneL) left = leftFwd - STEER_US;
  servoLeft.writeMicroseconds(left);
  servoRight.writeMicroseconds(right);
}

// Drive (steering) until a wall is ahead, or the left wall ends if stopAtLeftGap is true.
// Gives up after maxCm. Returns the distance driven in cm.
float driveUntil(bool stopAtLeftGap, float maxCm) {
  float travelled = 0;
  bool moving = false;
  bool leftWallSeen = false;
  int gapReads = 0;
  unsigned long last = 0;
  while (travelled < maxCm) {
    readSensors();
    unsigned long now = millis();
    if (moving) travelled += (now - last) / 1000.0 * speedCmPerS;
    if (zoneM <= frontWallZone) break;
    if (stopAtLeftGap) {
      if (zoneL < 5) { leftWallSeen = true; gapReads = 0; }
      else if (leftWallSeen && ++gapReads >= 3) break;
    }
    driveSteered();
    last = millis();
    moving = true;
  }
  stopMotors();
  return travelled;
}

void countdown() {
  Serial.println(F("  starting in 3 s..."));
  delay(3000);
}

// ---------- tests ----------
void testZones() {
  Serial.println(F("Live sensor zones for 10 s (0 = very close, 5 = nothing):"));
  for (int i = 0; i < 33; i++) {
    readSensors();
    printZones();
    delay(300);
  }
}

void testStraight() {
  Serial.println(F("Straight test: drives 3 s. Watch which way it drifts."));
  countdown();
  drive(leftFwd, rightFwd, 3000);
  Serial.println(F("  Drifts LEFT  -> right wheel is too fast: raise R by 5-10 (e.g. R1410)"));
  Serial.println(F("  Drifts RIGHT -> left wheel is too fast: lower L by 5-10 (e.g. L1590)"));
  Serial.println(F("  Repeat until it goes straight. Then re-measure speed with d."));
}

void testDistance() {
  Serial.println(F("Speed test: drives exactly 2 s. Measure how far it went in cm,"));
  Serial.println(F("then type D followed by the distance (e.g. D25)."));
  countdown();
  drive(leftFwd, rightFwd, DRIVE_TEST_MS);
}

void testSpin(bool clockwise, int quarterTurns) {
  Serial.print(F("Spin test: "));
  Serial.print(quarterTurns * 90);
  Serial.println(clockwise ? " degrees clockwise" : " degrees anticlockwise");
  countdown();
  for (int i = 0; i < quarterTurns; i++) {
    if (clockwise) spinClockwise(rotate90Ms);
    else spinAnticlockwise(rotate90Ms);
    delay(SETTLE_MS);
  }
  Serial.println(F("  Turned too far -> lower T. Not far enough -> raise T (about 6 ms per degree)."));
}

void testWall() {
  Serial.println(F("Wall test: drives down the corridor and stops at the wall ahead."));
  countdown();
  float cm = driveUntil(false, 200);
  readSensors();
  Serial.print(F("  stopped after cm: "));
  Serial.println(cm);
  printZones();
  Serial.println(F("  Stopped too close -> raise F. Too far away -> lower F."));
}

void testCorner() {
  Serial.println(F("Corner test: drive to the wall, creep C cm, turn into the open side, drive 12 cm."));
  countdown();
  driveUntil(false, 200);
  readSensors();
  printZones();
  bool leftOpen = zoneL == 5;
  bool rightOpen = zoneR == 5;
  if (!leftOpen && !rightOpen) {
    Serial.println(F("  Both sides closed: this is a dead end, not a corner. Try a corner."));
    return;
  }
  creepForward(cornerCreepCm);
  if (leftOpen) spinAnticlockwise(rotate90Ms);
  else spinClockwise(rotate90Ms);
  delay(SETTLE_MS);
  creepForward(REENTRY_CM);
  Serial.println(F("  Robot hugs the OUTSIDE wall of the new corridor -> lower C."));
  Serial.println(F("  Robot hugs the INSIDE corner wall -> raise C."));
}

void testGap() {
  Serial.println(F("Gap test: follow the corridor until the LEFT wall ends, drive G cm, turn left, drive 12 cm."));
  countdown();
  float cm = driveUntil(true, 200);
  readSensors();
  if (zoneM <= frontWallZone) {
    Serial.println(F("  Hit a wall ahead before any left opening. Start further back."));
    return;
  }
  Serial.print(F("  left gap found after cm: "));
  Serial.println(cm);
  creepForward(gapAdvanceCm);
  spinAnticlockwise(rotate90Ms);
  delay(SETTLE_MS);
  creepForward(REENTRY_CM);
  Serial.println(F("  Robot ends up near the wall it was following -> raise G."));
  Serial.println(F("  Robot ends up near the far wall of the opening -> lower G."));
}

// ---------- menu ----------
void printValues() {
  Serial.println(F("Copy these into ELEC1601_extension.ino:"));
  Serial.print(F("  const int LEFT_FWD = ")); Serial.print(leftFwd);
  Serial.print(F(",  RIGHT_FWD = ")); Serial.print(rightFwd); Serial.println(F(";"));
  Serial.print(F("  const int LEFT_BACK = ")); Serial.print(leftBack());
  Serial.print(F(", RIGHT_BACK = ")); Serial.print(rightBack()); Serial.println(F(";"));
  Serial.print(F("  const float DRIVE_SPEED_CM_PER_S = ")); Serial.print(speedCmPerS); Serial.println(F(";"));
  Serial.print(F("  const int ROTATE_90_MS = ")); Serial.print(rotate90Ms); Serial.println(F(";"));
  Serial.print(F("  const int FRONT_WALL_ZONE = ")); Serial.print(frontWallZone); Serial.println(F(";"));
  Serial.print(F("  const float CORNER_CREEP_CM = ")); Serial.print(cornerCreepCm); Serial.println(F(";"));
  Serial.print(F("  const float GAP_ADVANCE_CM = ")); Serial.print(gapAdvanceCm); Serial.println(F(";"));
}

void printMenu() {
  Serial.println();
  Serial.println(F("Tests:  z sensor zones   s straight   d speed (2 s drive)"));
  Serial.println(F("        t spin 90 cw     a spin 90 anticlockwise   f full circle (4 x 90)"));
  Serial.println(F("        w stop at wall   c corner turn   g left-gap turn"));
  Serial.println(F("Set:    L<us> R<us> V<cm/s> D<cm driven in d test> T<ms> F<zone> C<cm> G<cm>"));
  printValues();
  Serial.println();
}

void setValue(char cmd, float value) {
  switch (cmd) {
    case 'L': leftFwd = (int)value; break;
    case 'R': rightFwd = (int)value; break;
    case 'V': speedCmPerS = value; break;
    case 'D': speedCmPerS = value / (DRIVE_TEST_MS / 1000.0); break;
    case 'T': rotate90Ms = (int)value; break;
    case 'F': frontWallZone = (int)value; break;
    case 'C': cornerCreepCm = value; break;
    case 'G': gapAdvanceCm = value; break;
    default: Serial.println(F("Unknown command, type ? for the list.")); return;
  }
  Serial.println(F("Updated."));
  printValues();
}

void setup() {
  Serial.begin(9600);
  servoLeft.attach(13);
  servoRight.attach(12);
  pinMode(irReceiverLeft, INPUT);
  pinMode(irReceiverMid, INPUT);
  pinMode(irReceiverRight, INPUT);
  pinMode(irLedLeft, OUTPUT);
  pinMode(irLedMid, OUTPUT);
  pinMode(irLedRight, OUTPUT);
  stopMotors();
  Serial.println(F("ELEC1601 calibration. Type a command and press Enter."));
  printMenu();
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  char cmd = line.charAt(0);
  float value = line.substring(1).toFloat();

  switch (cmd) {
    case '?': printMenu(); break;
    case 'z': testZones(); break;
    case 's': testStraight(); break;
    case 'd': testDistance(); break;
    case 't': testSpin(true, 1); break;
    case 'a': testSpin(false, 1); break;
    case 'f': testSpin(true, 4); break;
    case 'w': testWall(); break;
    case 'c': testCorner(); break;
    case 'g': testGap(); break;
    default: setValue(cmd, value); break;
  }
  Serial.println(F("Ready."));
}
