#include <Wire.h>

uint16_t accx, accy, accz;
uint16_t gyrox, gyroy, gyroz;
uint16_t temp;


void setup() {
  Serial.begin(9600);

  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 7*2, true);

  accx = Wire.read() << 8 | Wire.read();
  accy = Wire.read() << 8 | Wire.read();
  accz = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();
  gyrox = Wire.read() << 8 | Wire.read();
  gyroy = Wire.read() << 8 | Wire.read();
  gyroz = Wire.read() << 8 | Wire.read();

  Serial.print(accx);
  Serial.print(" ");
  Serial.print(accy);
  Serial.print(" ");
  Serial.println(accz);

}
