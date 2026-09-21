#include <Servo.h>

Servo servoLeft;
Servo servoRight;

// Drive sensors
const int irLedLeft = 10, irReceiverLeft = 11;
const int irLedMid = 6, irReceiverMid = 7;      // fill in your actual mid sensor pins
const int irLedRight = 2, irReceiverRight = 3;

// Indicator LEDs
const int ledRight = A0;
const int ledMid = A1;
const int ledLeft = A2;

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
  int left = irDetect(irLedLeft, irReceiverLeft, 38000);
  int mid = irDetect(irLedMid, irReceiverMid, 38000);
  int right = irDetect(irLedRight, irReceiverRight, 38000);

  if (mid == 1 && left == right) {
    // Scenario 1: middle of a long corridor
    digitalWrite(ledRight, HIGH);
    digitalWrite(ledMid, LOW);
    digitalWrite(ledLeft, LOW);

    delay(5000);   // hold LED display so it's visible

    servoLeft.writeMicroseconds(1600);
    servoRight.writeMicroseconds(1400);
    delay(FORWARD_5CM_MS);

    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1500);
  }

  // add more "else if (...)" blocks here for scenarios 2-10

  else {
    // Scenario 0: Unknown Scenario
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

  // no scenarios wired up yet - falls through to default in the switch

  return -1;
}

