#define encLA 2
#define encLB 3
#define encRA 18
#define encRB 19

volatile int countL, countR;
volatile int oldLA, oldLB, oldRA, oldRB;

void setup(){
  //シリアル通信設定
  Serial.begin(115200);

  //各pinの設定
  pinMode(encLA, INPUT_PULLUP);
  pinMode(encLB, INPUT_PULLUP);
  pinMode(encRA, INPUT_PULLUP);
  pinMode(encRB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encLA), count_L,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encLB), count_L,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encRB), count_R,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encRB), count_R,CHANGE);
  //Serial.print("Start");
}

void loop(){
  
}

void count_L(){
  volatile int newLA, newLB, add;

  newLA = !digitalRead(encLA);
  newLB = !digitalRead(encLB);
  
  if(oldLA == oldLB){
    if(oldLA != newLA) add = 1;
    else               add = 2;
  }
  else{
    if(oldLA == newLA) add = 1;
    else               add = 2;
  }
  countL = countL + add;
  
  oldLA = newLA;
  oldLB = newLB;
  Serial.print(countL,BIN);
  Serial.print(countR,BIN);
  Serial.println( );
  //Serial.print(countL);
  //Serial.print(' ');
  //Serial.print(countR);
  //Serial.println( );
  countL = 0;
}

void count_R(){
  volatile int newRA, newRB, add;

  newRA = !digitalRead(encRA);
  newRB = !digitalRead(encRB);
  
  if(oldRA == oldRB){
    if(oldRA != newRA) add = 1;
    else               add = 2;
  }
  else{
    if(oldRA == newRA) add = 1;
    else               add = 2;
  }
  countR = countR + add;
  
  oldRA = newRA;
  oldRB = newRB;
  Serial.print(countL,BIN);
  Serial.print(countR,BIN);
  Serial.println( );
  //Serial.print(countL);
  //Serial.print(' ');
  //Serial.print(countR);
  //Serial.println( );
  countR = 0;
}
