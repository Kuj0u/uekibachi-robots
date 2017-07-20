//エンコーダが正常に動くかどうかのテストプログラム

#define ENC_A 2
#define ENC_B 3

void setup() {
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(0, ENC_READ, CHANGE);
  attachInterrupt(1, ENC_READ, CHANGE);

  Serial.begin(250000);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void ENC_READ(){
  int cur[2];

  cur[0] = !digitalRead(ENC_A);
  cur[1] = !digitalRead(ENC_B);

  Serial.print("A:");
  Serial.print(cur[0]);
  Serial.print(" ");

  Serial.print("B:");
  Serial.print(cur[1]);

  Serial.println(" ");
}

