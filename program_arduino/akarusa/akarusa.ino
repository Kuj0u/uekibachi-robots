int val0 = 0; //入力される値を格納する為の変数
int val1 = 0; //入力される値を格納する為の変数
int val2 = 0; //入力される値を格納する為の変数
void setup() {
  Serial.begin(9600); //モニターに出力するための設定
}
void loop() {
  //ANALOG INの０番ピンからデータを受け付ける
  val0 = analogRead(0);
  val1 = analogRead(1);
  val2 = analogRead(2);
  //Serial.print("%d\t%d\t%d\n", val0, val1, val2); //入力された値をモニターに出力
  Serial.print(val0); //入力された値をモニターに出力
  Serial.print("\t");
  //Serial.print("Brightness_1:"); //入力された値をモニターに出力
  Serial.print(val1); //入力された値をモニターに出力
  Serial.print("\t");
  //Serial.print("Brightness_2:"); //入力された値をモニターに出力
  Serial.print(val2); //入力された値をモニターに出力
  Serial.print("\n");
}
