#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int MPU_ADDR=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int minval = 265;
int maxval = 402;

int minrot = 80;
int maxrot = 575;

float curr1 = 330;
float curr2 = 330;

double x;
double y;
double z;

float K_P = 0.1;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(50);
}

void readAngles(double *x, double *y, double *z)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,14,true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  int xAng = map(AcX,minval,maxval,-90,90);
  int yAng = map(AcY,minval,maxval,-90,90);
  int zAng = map(AcZ,minval,maxval,-90,90);

  *x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI );
  *y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI );
  *z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI );
}

void setServos(int serv, int value)
{
pwm.setPWM(serv,0,value);
}

void updateVal(float *inp_val, float req_val)
{
  *inp_val += K_P*(req_val - *inp_val);
}



int ang_to_servo_val(int angle)
{
  int result = map(angle, -90, 90, 80, 580);
  return result;
}


int state = 0;


void loop() {

  readAngles(&x, &y, &z);



  switch (state)
  {
    //höften är curr1, servo 0
    //knäet är curr2, servo 1
    case 0:
      updateVal(&curr1, ang_to_servo_val(0));
      updateVal(&curr2, ang_to_servo_val(17));

      setServos(2, ang_to_servo_val(0));

      if (abs(curr1 - ang_to_servo_val(0)) < 5)
      {
        state = 1;
      }
    break;

    case 1:
      updateVal(&curr1, ang_to_servo_val(70));
      updateVal(&curr2, ang_to_servo_val(87));

      setServos(2, ang_to_servo_val(0));

      if (abs(curr1 - ang_to_servo_val(70)) < 5)
      {
        state = 2;
      }

    break;

    case 2:
      updateVal(&curr1, ang_to_servo_val(80));
      updateVal(&curr2, ang_to_servo_val(20));

      setServos(2, ang_to_servo_val(0));

      if (abs(curr2 - ang_to_servo_val(20)) < 5)
      {
        state = 3;
      }

    break;

    case 3:
      updateVal(&curr1, ang_to_servo_val(-20));
      updateVal(&curr2, ang_to_servo_val(17));

      setServos(2, ang_to_servo_val(0));

      if (abs(curr2 - ang_to_servo_val(17)) < 5)
      {
          state = 0;
      }
    break;

    case 4:
      updateVal(&curr1, ang_to_servo_val(0));
      updateVal(&curr2, ang_to_servo_val(17));
      setServos(2, map(y, 0, 360, 80, 580)+120);

      if (y < 80)
      {
        state = 0;
      }
      break;
  }

  if (y > 150)
  {
    state = 4;
  }


  setServos(0, curr1);
  setServos(1, curr2);
  

  Serial.print(80);
  Serial.print(",");
  Serial.print(580);
  Serial.print(",");
  Serial.print(curr1);
  Serial.print(",");
  Serial.println(curr2);






  //Serial.print("x: ");
  //Serial.print(minrot);
  //Serial.print(",");
  //Serial.print(maxrot);
  //Serial.print(",");
  //Serial.print(",");

  //Serial.print(x);
  //Serial.print(",");

  //Serial.print("y: ");
  //Serial.print(y);
  //Serial.print(",");

  //Serial.print("z: ");
  //Serial.print(curr_rot);
  //Serial.print(",");
  //Serial.println(y);
  //Serial.print(",");
  //Serial.println(z);

  //Serial.println("----");
}





