#include <Servo.h>

Servo servoLeft;
Servo servoRight;

// Drive sensors
const int irLedLeft = 10, irReceiverLeft = 11;
const int irLedMid = ___, irReceiverMid = ___;      // fill in your actual mid sensor pins
const int irLedRight = 2, irReceiverRight = 3;

// Indicator LEDs
const int ledRight = A0;
const int ledMid = A1;
const int ledLeft = A2;

// Timing constants - these are placeholders, tune them by testing on the real robot
const int ROTATE_90_MS = 700;        // how long to spin to complete a 90 degree turn
const int FORWARD_REENTRY_MS = 1200; // how long to drive forward after turning, to re-enter the corridor

int irDetect(int irLedPin, int irReceiverPin, long frequency) {
  tone(irLedPin, frequency);
  delay(1);
  int ir = digitalRead(irReceiverPin);
  noTone(irLedPin);
  delay(1);
  return ir;
}

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

      // rotate 90 degrees (clockwise/right) - both wheels same value = spin in place
      servoLeft.writeMicroseconds(1600);
      servoRight.writeMicroseconds(1600);
      delay(ROTATE_90_MS);

      // move forward to re-enter the corridor
      servoLeft.writeMicroseconds(1600);
      servoRight.writeMicroseconds(1400);
      delay(FORWARD_REENTRY_MS);

      servoLeft.writeMicroseconds(1500);
      servoRight.writeMicroseconds(1500);
      break;

    // Scenarios 3-10 will get their own case here as you build them out

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

// Returns which scenario matches the current sensor readings.
// Add real detection logic here as you build out each scenario.
int detectScenario() {
  int left = irDetect(irLedLeft, irReceiverLeft, 38000);
  int mid = irDetect(irLedMid, irReceiverMid, 38000);
  int right = irDetect(irLedRight, irReceiverRight, 38000);

  if (right == 1 && left == 0 && mid == 0) {
    return 2;   // right open, left wall, forward wall
  }

  return -1;   // falls through to default
}