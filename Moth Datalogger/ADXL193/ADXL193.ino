/* 
Recoil Viewer designed by AB9VH Electronics, James Douglas III - 2012
Sparkfun ADXL193 single axis accelerometer, 5v reference

X axis analog pin 0
Ground, black wire
+5v, red wire  (Vin: voltage input 5v)

ZERO G = 508  (calibration data)
*/

int X = 0;                          // fresh data from accelerometer
int Xmax = 508;                     // maximim recoil data
int Xmin = 508;                     // minimum recoil data
int R = 0;                          // recoil reading in g force
unsigned long time = 0;             // time in millis of recoil event

void setup() {
  Serial.begin(9600);            // opens serial port for LCD shield
  Serial.println("Recoil Viewer by AB9VH Electronics");
  Serial.println();
  time = millis();               // time of recoil event
}

void loop() {
  X = analogRead(0);             // read analog input
  if (X < Xmin) Xmin = X; 
  if (X > Xmax) Xmax = X;
  if (millis()-time >= 1000) {      // display every one second
    time = millis();               // time of recoil event  
    R = (Xmin-508)*.5;              // calculate maximum recoil
    Serial.print("Min recoil = ");
    Serial.print(R);
    Serial.print("g     ");
    R = (Xmax-508)*.5;              // calculate minimum recoil
    Serial.print("Max recoil = ");
    Serial.print(R);
    Serial.println("g");
  }
}

