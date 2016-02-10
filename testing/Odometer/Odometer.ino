#include <Smartcar.h>

Odometer encoder(33), encoder2(33);
void setup() {
  Serial.begin(9600);
  encoder.attach(2);
  encoder2.attach(3);
  encoder.begin();
  encoder2.begin();
}
//40 pulses per meter
void loop() {
  Serial.print(encoder.getDistance());
  Serial.print("\t\t");
  Serial.println(encoder2.getDistance());
  delay(100);
}
