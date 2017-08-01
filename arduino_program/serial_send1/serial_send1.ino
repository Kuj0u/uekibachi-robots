#define eA 2
#define eB 3

volatile int count;
volatile int oldLA,oldLB;

void setup(){
  Serial.begin(115200);
  pinMode(eA, INPUT_PULLUP); //←
  pinMode(eB, INPUT_PULLUP); //→
  attachInterrupt(digitalPinToInterrupt(eA), countL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(eB), countL,CHANGE);
  //Serial.print("Start");
}

void loop(){
  
}

void countL(){
  volatile int newLA, newLB, add;

  newLA = !digitalRead(eA);
  newLB = !digitalRead(eB);
  
  if(oldLA == oldLB){
    if(oldLA != newLA)
      add = 1;
    else
      add = -1;
  }
  else{
    if(oldLA == newLA)
      add = 1;
    else
      add = -1;
  }
  
  count = count + add;
  
  oldLA = newLA;
  oldLB = newLB;
  Serial.println(count);
  count = 0;
  //Serial.print(' ');
  //Serial.print(newLA);
  //Serial.print(newLB);
  //Serial.println( );
  
}
