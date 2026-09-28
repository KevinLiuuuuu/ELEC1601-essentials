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

// Value for frequencies needed to send to tone to detect distances
// WARNING: THESE ARE PLACEHOLDER VALUES, UPDATE WHEN WE CONFIGURE SENSORS!!
const long leftClose = 24000;
const long leftMedium = 34000;
const long rightClose = 24000;
const long rightMedium = 34000;
const long midClose = 24000;
const long midMedium = 34000;


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

void setLEDs(int left, int middle, int right) {
  digitalWrite(ledLeft, left);
  digitalWrite(ledMid, middle);
  digitalWrite(ledRight, right);
}


void moveForward() {
  // Move forward
      servoLeft.writeMicroseconds(1600);
      servoRight.writeMicroseconds(1400);
      // For five seconds (adjust to time needed to move 5cm)
      delay(5000);
}

void moveBackward() {
  // Move backward
      servoLeft.writeMicroseconds(1400);
      servoRight.writeMicroseconds(1600);
      // For five seconds (adjust to time needed to move 5cm)
      delay(5000);
}

void loop() {
  int scenario = detectScenario();

  switch (scenario) {

    case 1:
      setLEDs(HIGH, LOW, LOW);

      moveForward();
      // Stop moving
      servoLeft.writeMicroseconds(1500);
      servoRight.writeMicroseconds(1500);
      break;
    case 4:
      setLEDs(LOW, LOW, HIGH);

      // Rotates counter-clockwise
      servoLeft.writeMicroseconds(1400);
      servoRight.writeMicroseconds(1400);
      // For three seconds (adjust to time needed to rotate)
      delay(3000);
      moveForward();
      // Stop moving
      servoLeft.writeMicroseconds(1500);
      servoRight.writeMicroseconds(1500);
      break;
    case 9:
      setLEDs(LOW, LOW, LOW);
      delay(1000);
      setLEDs(LOW, HIGH, LOW);
      delay(1000);
      setLEDs(LOW, LOW, LOW);

      // Move backwards, rest will be handled by case 5.
      moveBackward();
      // Stop moving
      servoLeft.writeMicroseconds(1500);
      servoRight.writeMicroseconds(1500);
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

// Returns which scenario matches the current sensor readings.
// Add real detection logic here as you build out each scenario.
int detectScenario() {
  int leftLedClose = irDetect(irLedLeft, irReceiverLeft, leftClose);
  int midLedClose = irDetect(irLedMid, irReceiverMid, midClose);
  int rightLedClose = irDetect(irLedRight, irReceiverRight, rightClose);

  int leftLedMedium = irDetect(irLedLeft, irReceiverLeft, leftMedium);
  int midLedMedium = irDetect(irLedMid, irReceiverMid, midMedium);
  int rightLedMedium = irDetect(irLedRight, irReceiverRight, rightMedium);

  if(leftLedMedium && !midLedMedium && rightLedMedium) {
    return 1;
  } else if (leftLedMedium && midLedMedium && rightLedMedium) {
    return 4;
  } else if (leftLedClose && midLedMedium && rightLedMedium) {
    // Close to (but parallel to) left wall. Wall in front.
    return 9;
  }

  return 0;
}
