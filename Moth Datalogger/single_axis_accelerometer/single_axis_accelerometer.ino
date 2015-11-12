int TiltPin=8; 

void setup() {
  Serial.begin(9600);        // Turn on the Serial Port
  pinMode(TiltPin, OUTPUT);  // Tell Arduino that redLEDPin is an output pin
}

void loop() {
  Serial.println(analogRead(8));
  delay(1000);
}


