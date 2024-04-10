#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();




void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);


}

int min = 80;
int max = 590;


void loop() {
  //90-550

  pwm.setPWM(0, 0, min); //100-550  (-90 till +90)
  pwm.setPWM(1, 0, min); //contin
  pwm.setPWM(2, 0, min); //80-575   (-90 till +90)
  pwm.setPWM(3, 0, min); //80-580   (-90 till +90)
  pwm.setPWM(4, 0, min); //80-580   (-90 till +90)
  pwm.setPWM(5, 0, min); //contin

  delay(2000);

  pwm.setPWM(0, 0, max);
  pwm.setPWM(1, 0, max);
  pwm.setPWM(2, 0, max);
  pwm.setPWM(3, 0, max);
  pwm.setPWM(4, 0, max);
  pwm.setPWM(5, 0, max);

  delay(2000);

}
