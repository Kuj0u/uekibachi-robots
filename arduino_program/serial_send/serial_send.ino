#define eA 2
#define eB 3

void setup(){
  Serial.begin(9600);
  pinMode(eA, INPUT_PULLUP); //←
  pinMode(eB, INPUT_PULLUP); //→
  attachInterrupt(digitalPinToInterrupt(2), countL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), countL,CHANGE);
  Serial.print("Start");
}

void loop(){
  
}

void countL(){
  volatile byte sendval=0, A, B, temp;

  A = !digitalRead(eA);
  B = !digitalRead(eB);
  temp = (A << 1);
  sendval = temp + B;
  Serial.write(sendval);
  sendval = 0;
}
