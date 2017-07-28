#include <String.h>
#define maxLength 64
String inString = String(maxLength);

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  inString = "";
}

void loop() {
  if (Serial.available() > 0) {
    char inChar = Serial.read();
    if (inString.length() < maxLength) {
      inString.concat(String(inChar));
      if (inChar == '\n') {
        if (inString.startsWith("ON")) {
          Serial.println("<--HI-->");
          digitalWrite(13, HIGH);
        } else if (inString.startsWith("OFF")) {
          Serial.println("<--LOW-->");
          digitalWrite(13, LOW);
        }
        inString = "";
      }
      Serial.println(inString);
    }
    else {
      Serial.println("Maximum number of characters.");
    }
  }
}
