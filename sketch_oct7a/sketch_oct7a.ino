const int hallPin = A0;
const float Vref = 3.3;
const int ADCmax = 1023;


void setup() {
  Serial.begin(115200);
  analogReference(Vref);
  delay(100);
  Serial.println("time_ms,adc,voltage_V");

}

void loop() {
  int adc = analogRead(hallPin);
  float voltage = (adc/float(ADCmax))*Vref;
  unsigned long t = millis();
  Serial.println(t, adc);
  // Serial.print(adc);
  // Serial.print(voltage,4);
  delay(200);
}
