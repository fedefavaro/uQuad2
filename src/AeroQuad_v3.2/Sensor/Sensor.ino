void setup() {
 Serial.begin(9600);
}

void loop() {
 int sensorValue = analogRead(A0);

 Serial.print(sensorValue);
 Serial.println(" cm");

 delay(50);
}
