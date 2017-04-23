int val3 = 0;


void setup() {
  Serial.begin(9800);
}

void loop() {
  val3 = analogRead(3);
  Serial.print("moisture:");
  Serial.println(val3);
  delay(500);
}
