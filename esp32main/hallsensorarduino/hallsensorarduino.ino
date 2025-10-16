const int hallPin = A0;
const float Vref = 3.3;
const int ADCmax = 1023;


void setup() {
  Serial.begin(115200);
  // analogReference(DEFAULT);
  delay(100);
  // Serial.println("time_ms,adc");

}

void loop() {
  int adc = analogRead(hallPin);
  // float voltage = (adc/float(ADCmax))*Vref;
  unsigned long t = millis();
  Serial.print(t);
  Serial.print(",");
  Serial.println(adc);
  // Serial.print(",");
  // Serial.println(voltage, 4);

  delay(1);
}
