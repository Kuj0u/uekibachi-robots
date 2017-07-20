/*

このプログラムの概要！
エンコーダが読み取ったHIGH/LOWの状態を読み取って正回転/逆回転を判定するよ
参考サイト：http://robocad.blog.jp/archives/723986.html

2017/07/13：まだ組み込んでないしラズパイのセットアップも終わってない。
          　動作未確認でござる（田中）

2017/07/14:i2c通信は行けるっぽい　カウントまだ
*/

//I2C通信用のライブラリ
#include <Wire.h>

//各エンコーダのピン番号指定
#define enc_LA 2
#define enc_LB 3
#define enc_RA 18
#define enc_RB 19

//I2Cのアドレスを0x04に設定 ここは調整必要かもしれない
int SLAVE_ADDRESS = 0x04;
//uint64_tとか使うかも
volatile uint16_t countL, countR;
volatile int oldLA, oldLB;

void setup(){
  countL = 0;
  countR = 0;
  Serial.begin(250000); //デバッグ用
  //ピンのIN/OUT設定
  pinMode(enc_LA, INPUT_PULLUP);
  pinMode(enc_LB, INPUT_PULLUP);
  pinMode(enc_RA, INPUT_PULLUP);
  pinMode(enc_RB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(enc_LA), count_L,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_LB), count_L,CHANGE);
  
  //ここからI2C通信用
  //I2C接続を開始
  Wire.begin(SLAVE_ADDRESS);

  Wire.onRequest(sendCounter);
  Wire.onReceive()
  Serial.println("Start!");
}

void loop(){
  //今回は使わないよ
}

void count_L(){
  volatile int newLA, newLB, count_Ladd;

  newLA = !digitalRead(enc_LA);
  newLB = !digitalRead(enc_LB);

  if(oldLA == oldLB){
    if(oldLA != newLA)
      count_Ladd = 1;
    else
      count_Ladd = -1;
  }
  else{
    if(oldLA == newLA)
      count_Ladd = 1;
    else
      count_Ladd = -1;
  }
  
  countL = countL + count_Ladd;
  
  oldLA = newLA;
  oldLB = newLB;

  int ans[5];
  ans[0] = !digitalRead(enc_LA);
  ans[1] = !digitalRead(enc_LB);
  ans[2] = !digitalRead(enc_RA);
  ans[3] = !digitalRead(enc_RB);

  // デバッグ
  Serial.print("A:");
  Serial.print(ans[0]);
  Serial.print(" ");
  Serial.print("B:");
  Serial.print(ans[1]);
  Serial.print(" ");
  Serial.print("countL:");
  Serial.print(countL, DEC); //byte型はDECをつけるといいぞ！
  Serial.println(" ");
}

void sendCounter(){
  Wire.write((char *)&countL,4);
}

void ENC_Read(){
  int ans[5];

  ans[0] = !digitalRead(enc_LA);
  ans[1] = !digitalRead(enc_LB);
  ans[2] = !digitalRead(enc_RA);
  ans[3] = !digitalRead(enc_RB);

  /* デバッグ
  Serial.print("A:");
  Serial.print(ans[0]);
  Serial.print(" ");
  Serial.print("B:");
  Serial.print(ans[1]);
  Serial.println(" ");
  */
}

/*
void sending{
  //各エンコーダの状態を読み取る
  int e_LA = digitalRead(enc_LA);
  int e_LB = digitalRead(enc_LB);
  //e_RA = digitalRead(enc_RA);
  //e_RB = digitalRead(enc_RB);
  //各エンコーダの状態を送信
  Wire.write((char*)&count,8);
  //Wire.write(e_LB);
  //Wire.write(e_RA);
  //Wire.write(e_RB);
  Serial.println(e_LA);
  Serial.println(e_LB);
}
*/

