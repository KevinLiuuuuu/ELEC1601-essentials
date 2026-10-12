// ELEC1601 sensor test - prints what each sensor sees. Upload this INSTEAD of the main sketch,
// open Serial Monitor at 9600 baud. Robot does not move.

const int irLedLeft = 10,  irReceiverLeft = 11;
const int irLedMid = 6,    irReceiverMid = 7;
const int irLedRight = 2,  irReceiverRight = 3;

int irDetect(int irLedPin, int irReceiverPin, long frequency) {
  tone(irLedPin, frequency);
  delay(1);
  int ir = digitalRead(irReceiverPin);   // 0 = detected, 1 = not detected
  noTone(irLedPin);
  delay(1);
  return ir;
}

// Prints e.g. "Y Y Y N N -> 2": Y = detected at 38,39,40,41,42 kHz, then the zone (count of N)
int showSensor(const char* name, int ledPin, int rxPin) {
  int zone = 0;
  Serial.print(name); Serial.print(": ");
  for (long f = 38000; f <= 42000; f += 1000) {
    int ir = irDetect(ledPin, rxPin, f);
    zone += ir;
    Serial.print(ir == 0 ? "Y " : "N ");
  }
  Serial.print("-> "); Serial.print(zone); Serial.print("    ");
  return zone;
}

void setup() {
  Serial.begin(9600);
  pinMode(irReceiverLeft, INPUT);
  pinMode(irReceiverMid, INPUT);
  pinMode(irReceiverRight, INPUT);
  pinMode(irLedLeft, OUTPUT);
  pinMode(irLedMid, OUTPUT);
  pinMode(irLedRight, OUTPUT);
}

void loop() {
  showSensor("L", irLedLeft, irReceiverLeft);
  showSensor("M", irLedMid, irReceiverMid);
  showSensor("R", irLedRight, irReceiverRight);
  Serial.println();
  delay(300);
}
