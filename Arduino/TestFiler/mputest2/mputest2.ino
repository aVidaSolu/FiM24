#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

uint16_t ax, ay, az, gx, gy, gz;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  Serial.println(mpu.testConnection() ? "Success" : "Fail");

}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print(ax);
  Serial.print(" ");
  Serial.print(ay);
  Serial.print(" ");
  Serial.println(az);

}
