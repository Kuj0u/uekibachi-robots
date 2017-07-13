/*

このプログラムの概要！
エンコーダが読み取ったHIGH/LOWの状態をI2C通信を用いてラズパイに送るプログラムだよ！
参考サイト：http://robocad.blog.jp/archives/723986.html

2017/07/13：まだ組み込んでないしラズパイのセットアップも終わってない。
          　動作未確認でござる（田中）

*/

//I2C通信用のライブラリ
#include <Wire.h>

//各エンコーダのピン番号指定
#define enc_LA 2
#define enc_LB 3
#define enc_RA 18
#define enc_RB 19

//各エンコーダの読み取った値を格納しておく変数
int e_LA = 0;
int e_LB = 0;
int e_RA = 0;
int e_RB = 0;

//I2Cのアドレスを0x04に設定 ここは調整必要かもしれない
int SLAVE_ADDRESS = 0x04;

void setup(){
  //Serial.begin(9600);　シリアル通信するときなら必要
  //ピンのIN/OUT設定
  pinMode(enc_LA, INPUT);
  pinMode(enc_LB, INPUT);
  pinMode(enc_RA, INPUT);
  pinMode(enc_RB, INPUT);

  //ここからI2C通信用
  //I2C接続を開始
  Wire.begin(SLAVE_ADDRESS);

  //ラズパイから要求されたらこの関数を呼び出す
  Wire.onRequest(sendEncoderReading);
}

void loop(){
  //今回は使わないよ
}

void sendEncoderReading(){
  //各エンコーダの状態を読み取る
  e_LA = digitalRead(enc_LA);
  e_LB = digitalRead(enc_LB);
  e_RA = digitalRead(enc_RA);
  e_RB = digitalRead(enc_RB);
  //各エンコーダの状態を送信
  Wire.write(e_LA);
  Wire.write(e_LB);
  Wire.write(e_RA);
  Wire.write(e_RB);
}
