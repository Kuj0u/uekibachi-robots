#define encLA 2
#define encLB 3
#define encRA 18
#define encRB 19

volatile int countdata;
volatile int oldLA, oldLB, oldRA, oldRB;

void setup(){
  //シリアル通信設定
  Serial.begin(115200);

  //各pinの設定
  pinMode(encLA, INPUT_PULLUP);
  pinMode(encLB, INPUT_PULLUP);
  pinMode(encRA, INPUT_PULLUP);
  pinMode(encRB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encLA), count,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encLB), count,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encRB), count,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encRB), count,CHANGE);
  //Serial.print("Start");
}

void loop(){
  
}

void count(){
  volatile int newLA, newLB, newRA, newRB, addL, addR;

  newLA = !digitalRead(encLA);
  newLB = !digitalRead(encLB);
  newRA = !digitalRead(encRA);
  newRB = !digitalRead(encRB);
  if(oldLA == oldLB){
    if(oldLA != newLA) addL = 3;        //時計回り
    else if (oldLB == newLB) addL = 2;  //無回転
    else addL = 1;                      //反時計回り
  }
  else{
    if(oldLA != newLA) addL = 1;        //反時計
    else if(oldLB == newLB) addL = 2;   //無回転
    else addL = 3;                      //時計回り
  }
  
  if(oldRA == oldRB){
    if(oldRA != newRA) addR = 3;        //時計回り
    else if (oldRB == newRB) addR = 2;  //無回転
    else addR = 1;                      //反時計
  }
  else{
    if(oldRA != newRA) addR = 1;        //反時計
    else if(oldRB == newRB) addR = 2;   //無回転
    else addR = 3;                      //時計回り
  }
  countdata = addL + (addR << 2);
  
  oldLA = newLA;
  oldLB = newLB;
  oldRA = newRA;
  oldRB = newRB;
  Serial.println(countdata,BIN);
  //Serial.print(countL);
  //Serial.print(' ');
  //Serial.print(countR);
  //Serial.println( );
  countdata = 0;
  addL = 0;
  addR = 0;
}
