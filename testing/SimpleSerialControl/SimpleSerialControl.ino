#include <Smartcar.h>

const int PULSES_PER_METER = 33;
const int SERVO_PIN = 8;
const int ESC_PIN = 9;
const int ENCODER_PIN = 3;

Odometer encoderLeft(PULSES_PER_METER);
Car car(useServo(SERVO_PIN), useESC(ESC_PIN)); //use the servo motor connected at pin 8 and the ESC connected to pin 9

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);
  encoderLeft.attach(ENCODER_PIN);
  encoderLeft.begin();
  car.begin(encoderLeft);
//  car.enableCruiseControl(0.5,4,4);
}

void loop() {
  car.updateMotors();
  handleInput();
}

void handleInput() { //handle serial input if there is any
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("m")) {
      float throttle = input.substring(1).toFloat();
      car.setSpeed(throttle);
    } else if (input.startsWith("t")) {
      int deg = input.substring(1).toInt();
      car.setAngle(deg);
    }
  }
}
