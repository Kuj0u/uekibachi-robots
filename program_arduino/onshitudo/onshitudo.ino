
#include <DHT.h>

#define DHTPIN 4     // what pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11 
//#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(DHTPIN, DHTTYPE);

int val0 = 0; //入力される値を格納する為の変数
int val1 = 0; //入力される値を格納する為の変数
int val2 = 0; //入力される値を格納する為の変数


void setup() {
  Serial.begin(9600); 
  Serial.print("start");
  Serial.print("\t");
 
  dht.begin();
}

void loop() {
  // Wait a few seconds between measurements.

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit
  float f = dht.readTemperature(true);
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index
  // Must send in temp in Fahrenheit!
  float hi = dht.computeHeatIndex(f, h);
  val0 = analogRead(0);
  val1 = analogRead(1);
  val2 = analogRead(2);
  Serial.print(val0); //入力された値をモニターに出力
  Serial.print("\t");
  Serial.print(val1); //入力された値をモニターに出力
  Serial.print("\t");
  Serial.print(val2); //入力された値をモニターに出力
  Serial.print("\t"); 
  Serial.print(t);
  Serial.print("\t");
  Serial.print(h);
  Serial.print("\n");
  //Serial.print(f);
  //Serial.print(" *F\t");
  //Serial.print("Heat index: ");
  //Serial.print(hi);
  //Serial.println(" *F");
}
