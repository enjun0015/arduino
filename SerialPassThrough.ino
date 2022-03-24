void setup() {
   Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
   Serial1.begin(9600);
}

void loop() {
  if (Serial.available()){
    Serial1.write(Serial.read());
  }
 if (Serial1.available()){
    Serial.write(Serial1.read());
  }
}
