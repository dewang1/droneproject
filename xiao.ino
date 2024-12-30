// Define constants for time interval and pins
const unsigned long interval = 20;

unsigned long previousMillis = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read the incoming data for pin and value if available
  if (Serial.available() >= 2) {
    int x = Serial.read();
    int y = Serial.read();
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      analogWrite(A5, x);
      analogWrite(A4, y);
    }
  }
}
