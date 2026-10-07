#include <Servo.h>

Servo servoLeft;
Servo servoRight;

// Drive sensors
const int irLedLeft = 10, irReceiverLeft = 11;
const int irLedMid = 6, irReceiverMid = 7;
const int irLedRight = 2, irReceiverRight = 3;

// Indicator LEDs
const int ledRight = A0;
const int ledMid = A1;
const int ledLeft = A2;

// Timing constants - placeholders, tune by testing on the real robot
const int ROTATE_90_MS = 700;        // how long to spin to complete a 90 degree turn
const int FORWARD_REENTRY_MS = 1200; // how long to drive forward after turning, to re-enter the corridor
const int SMALL_ROTATE_MS = 250;
const int FORWARD_ADJUST_MS = 700;
const int BACKWARD_ADJUST_MS = 700;

// Based on your calibration data: zone 0-4 means a wall was detected somewhere
// within range, zone 5 means no detection at any frequency (no wall, or far away)
const int NO_WALL_ZONE = 5;
const int SIGNIFICANTLY_CLOSE_ZONE = 2;

int irDetect(int irLedPin, int irReceiverPin, long frequency) {
  tone(irLedPin, frequency);
  delay(1);
  int ir = digitalRead(irReceiverPin);
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

void stopRobot();

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

  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(5000);   // 5 second startup delay before any movement starts
}

void loop() {
  int scenario = detectScenario();

  switch (scenario) {

    case 2:
      // Scenario 2: Ideal position for Right Turn
      digitalWrite(ledRight, LOW);
      digitalWrite(ledMid, HIGH);
      digitalWrite(ledLeft, LOW);

      delay(5000);   // hold LED display so it's visible

      servoLeft.writeMicroseconds(1600);   // rotate 90 degrees (clockwise/right)
      servoRight.writeMicroseconds(1600);
      delay(ROTATE_90_MS);

      servoLeft.writeMicroseconds(1600);   // move forward to re-enter the corridor
      servoRight.writeMicroseconds(1400);
      delay(FORWARD_REENTRY_MS);

      servoLeft.writeMicroseconds(1500);
      servoRight.writeMicroseconds(1500);
      break;

    case 3:
      // Scenario 3: Ideal position for Left Turn
      digitalWrite(ledRight, HIGH);
      digitalWrite(ledMid, HIGH);
      digitalWrite(ledLeft, LOW);

      delay(5000);   // hold LED display so it's visible

      servoLeft.writeMicroseconds(1400);   // rotate 90 degrees (anticlockwise/left)
      servoRight.writeMicroseconds(1400);
      delay(ROTATE_90_MS);

      servoLeft.writeMicroseconds(1600);   // move forward to re-enter the corridor
      servoRight.writeMicroseconds(1400);
      delay(FORWARD_REENTRY_MS);

      servoLeft.writeMicroseconds(1500);
      servoRight.writeMicroseconds(1500);
      break;

    case 5:
      // Scenario 5: close to the left wall and parallel to it
      digitalWrite(ledRight, HIGH);
      digitalWrite(ledMid, LOW);
      digitalWrite(ledLeft, HIGH);
      delay(5000);

      // Turn clockwise, move along the corridor, then straighten up.
      servoLeft.writeMicroseconds(1600);
      servoRight.writeMicroseconds(1600);
      delay(SMALL_ROTATE_MS);
      servoLeft.writeMicroseconds(1600);
      servoRight.writeMicroseconds(1400);
      delay(FORWARD_ADJUST_MS);
      servoLeft.writeMicroseconds(1400);
      servoRight.writeMicroseconds(1400);
      delay(SMALL_ROTATE_MS);
      servoLeft.writeMicroseconds(1400);
      servoRight.writeMicroseconds(1600);
      delay(BACKWARD_ADJUST_MS);
      stopRobot();
      break;

    case 6:
      // Scenario 6: close to the right wall and parallel to it
      digitalWrite(ledRight, LOW);
      digitalWrite(ledMid, HIGH);
      digitalWrite(ledLeft, HIGH);
      delay(5000);

      // Turn anticlockwise, move along the corridor, then straighten up.
      servoLeft.writeMicroseconds(1400);
      servoRight.writeMicroseconds(1400);
      delay(SMALL_ROTATE_MS);
      servoLeft.writeMicroseconds(1600);
      servoRight.writeMicroseconds(1400);
      delay(FORWARD_ADJUST_MS);
      servoLeft.writeMicroseconds(1600);
      servoRight.writeMicroseconds(1600);
      delay(SMALL_ROTATE_MS);
      servoLeft.writeMicroseconds(1400);
      servoRight.writeMicroseconds(1600);
      delay(BACKWARD_ADJUST_MS);
      stopRobot();
      break;

    case 7:
      // Scenario 7: angled toward the left wall
      digitalWrite(ledRight, HIGH);
      digitalWrite(ledMid, HIGH);
      digitalWrite(ledLeft, HIGH);
      delay(5000);

      servoLeft.writeMicroseconds(1600);
      servoRight.writeMicroseconds(1600);
      delay(SMALL_ROTATE_MS);
      stopRobot();
      break;

    case 8:
      // Scenario 8: angled toward the right wall
      digitalWrite(ledRight, HIGH);
      digitalWrite(ledMid, LOW);
      digitalWrite(ledLeft, LOW);
      delay(1000);
      digitalWrite(ledRight, LOW);
      delay(1000);

      servoLeft.writeMicroseconds(1400);
      servoRight.writeMicroseconds(1400);
      delay(SMALL_ROTATE_MS);
      stopRobot();
      break;

    default:
      digitalWrite(ledRight, LOW);
      digitalWrite(ledMid, LOW);
      digitalWrite(ledLeft, LOW);

      servoLeft.writeMicroseconds(1500);
      servoRight.writeMicroseconds(1500);
      delay(5000);

      while (true) {
        servoLeft.writeMicroseconds(1500);
        servoRight.writeMicroseconds(1500);
      }
      break;
  }
}

void stopRobot() {
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  while (true) {
    delay(1000);
  }
}

// Returns which scenario matches the current sensor readings.
int detectScenario() {
  int leftZone = irDistance(irLedLeft, irReceiverLeft);
  int midZone = irDistance(irLedMid, irReceiverMid);
  int rightZone = irDistance(irLedRight, irReceiverRight);

  if (leftZone <= SIGNIFICANTLY_CLOSE_ZONE && midZone <= SIGNIFICANTLY_CLOSE_ZONE &&
      rightZone == NO_WALL_ZONE) {
    return 7;   // left and forward walls close, right side open
  }

  if (rightZone <= SIGNIFICANTLY_CLOSE_ZONE && midZone <= SIGNIFICANTLY_CLOSE_ZONE &&
      leftZone == NO_WALL_ZONE) {
    return 8;   // right and forward walls close, left side open
  }

  if (rightZone == NO_WALL_ZONE && leftZone < NO_WALL_ZONE && midZone < NO_WALL_ZONE) {
    return 2;   // right open, left wall in range, forward wall in range
  }

  if (leftZone == NO_WALL_ZONE && rightZone < NO_WALL_ZONE && midZone < NO_WALL_ZONE) {
    return 3;   // left open, right wall in range, forward wall in range
  }

  if (leftZone < NO_WALL_ZONE && rightZone < NO_WALL_ZONE && midZone == NO_WALL_ZONE &&
      rightZone + SIGNIFICANTLY_CLOSE_ZONE <= leftZone) {
    return 5;   // both side walls, right wall significantly closer, forward open
  }

  if (leftZone < NO_WALL_ZONE && rightZone < NO_WALL_ZONE && midZone == NO_WALL_ZONE &&
      leftZone + SIGNIFICANTLY_CLOSE_ZONE <= rightZone) {
    return 6;   // both side walls, left wall significantly closer, forward open
  }

  return -1;   // falls through to default
}