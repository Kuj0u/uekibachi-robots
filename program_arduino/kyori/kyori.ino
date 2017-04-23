/*
  HC-SR04 Ping distance sensor:
  VCC to arduino 5v
  GND to arduino GND
  Echo to Arduino pin 7
  Trig to Arduino pin 8

  This sketch originates from Virtualmix: http://goo.gl/kJ8Gl
  Has been modified by Winkle ink here: http://winkleink.blogspot.com.au/2012/05/arduino-hc-sr04-ultrasonic-distance.html
  And modified further by ScottC here: http://arduinobasics.blogspot.com.au/2012/11/arduinobasics-hc-sr04-ultrasonic-sensor.html
  on 10 Nov 2012.
*/

//settei
int mizu_stop = 150;  // servo stop kakudo
int mizu_start = 30;  // servo start kakudo
int kyori = 30;       // start kyori(cm)
int mizu_time = 5000;  // mizu time(ms)


#define echoPin 7 // Echo Pin
#define trigPin 8 // Trigger Pin
#define LEDPin 13 // Onboard LED
#include <Servo.h>

Servo myservo;

int maximumRange = 200; // Maximum range needed
int minimumRange = 0; // Minimum range needed
long duration, distance; // Duration used to calculate distance

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LEDPin, OUTPUT); // Use LED indicator (if required)
  myservo.attach(9);
}

void loop() {
  /* The following trigPin/echoPin cycle is used to determine the
    distance of the nearest object by bouncing soundwaves off of it. */
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);

  //Calculate the distance (in cm) based on the speed of sound.
  distance = duration / 58.2;

  if (distance >= maximumRange || distance <= minimumRange) {
    /* Send a negative number to computer and Turn LED ON
      to indicate "out of range" */
    Serial.println("-1");
    digitalWrite(LEDPin, HIGH);
  }
  else {
    /* Send the distance to the computer using Serial protocol, and
      turn LED OFF to indicate successful reading. */
    Serial.println(distance);
    digitalWrite(LEDPin, LOW);
    if (distance < kyori) {  //mizu maku.
      delay(3000);
      myservo.write(mizu_start);
      delay(mizu_time);
      myservo.write(mizu_stop);
      delay(10000);
    }
    else myservo.write(mizu_stop);
  }

  //Delay 50ms before next reading.
  delay(50);
}
