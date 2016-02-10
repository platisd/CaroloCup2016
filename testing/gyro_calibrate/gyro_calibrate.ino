#include <Wire.h>
#include <Servo.h>
#include <CaroloCup.h>

Gyroscope gyro;

void setup() {
  gyro.attach();
  Serial.begin(9600);
  delay(1500);
  gyro.begin();
  Serial.println("Calibrating gyroscope, this might take some seconds");
  unsigned int offset = gyro.calibrate();
  Serial.print("This gyro's offset value is: ");
  Serial.println(offset);
  Serial.print("Please change OFFSET_VALUE in Gyroscope.cpp accordingly");
}

void loop() {
}
