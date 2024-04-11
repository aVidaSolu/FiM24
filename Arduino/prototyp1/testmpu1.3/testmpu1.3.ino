#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//set up the servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//addresses to the MPU and variables to hold data from MPU
const int MPU_ADDR = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

//current values (pwm) the servo is holding, this is seperate from the values we want each servo to hold
//the function updateVal is used to update one of these current values closer towards a goal value (PID Controller)
float curr1 = 330;
float curr2 = 330;
float curr3 = 330;

//hold return rotation of the MPU
double rotx;
double roty;
double rotz;


void setup() {
  //setup MPU
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);

  //setup servo driver
  pwm.begin();
  pwm.setPWMFreq(50);

  pinMode(8, INPUT);
}



void readAngles(double *x, double *y, double *z);  //get current rotational angle from MPU
void setServos(int serv, int value);               //set servo serv to a specific PWM value
void updateVal(float *inp_val, float req_val);     //update value inp_val closer towards req_val (PID CONTROLLER)
void updateVal(float *inp_val, float req_val, float new_K);     //update value inp_val closer towards req_val (PID CONTROLLER), set K value manually
int ang_to_servo_val(int angle);                   //map angle (-90, 90) to servo PWM value (80-575)

void reset_legs();  //initialize leg positions

void main_walk_cycle();  //main switch case for walking cycle / walking case
void main_stabilize();   //main switch case for stabilization cycle




//state variables for use in the main switch case in void loop()
int state_main = 0;
int state_walk = 0;
int state_stable = 0;


void loop() {

  readAngles(&rotx, &roty, &rotz);

  int button = digitalRead(8);

  switch (state_main) {
    case 0:
      //initialization case
      reset_legs();
      state_main = 1;
      break;

    case 1:
      //walking case
      main_walk_cycle();

      if (button == 1) { state_main = 3; }
      break;

    case 2:
      //stabilization case
      main_stabilize();

      if (button == 1) { state_main = 3; }
      break;

    case 3:
      //pause intermidiary

      if (button == 0) {
        state_main = 4;
      }
      break;

    case 4:
      //pause

      if (button == 1) {
        state_main = 0;
      }
      break;
  }



  if ((roty > 120) && (state_main != 4)) {
    state_main = 2;
  }

  //set all servos to their current values
  setServos(0, curr1);
  setServos(1, curr2);
  setServos(2, curr3);


  // Serial.print(80);
  // Serial.print(",");
  // Serial.print(580);
  // Serial.print(",");
  // Serial.print(curr1);
  // Serial.print(",");
  // Serial.print(curr2);
  // Serial.print(",");
  // if (state_main == 2) {
  //   Serial.print(roty);
  //   Serial.print(",");
  //   Serial.println(curr3);
  // } else {
  //   Serial.println("");
  // }

}


void readAngles(double *x, double *y, double *z) {
  //magic accelerometer values for "integration"
  int minval = 265;
  int maxval = 402;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  int xAng = map(AcX, minval, maxval, -90, 90);
  int yAng = map(AcY, minval, maxval, -90, 90);
  int zAng = map(AcZ, minval, maxval, -90, 90);

  *x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  *y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  *z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);
}

void setServos(int serv, int value) {
  pwm.setPWM(serv, 0, value);
}

void updateVal(float *inp_val, float req_val) {
  float K_P = 0.1;
  *inp_val += K_P * (req_val - *inp_val);
}

void updateVal(float *inp_val, float req_val, float new_K) {
  *inp_val += new_K * (req_val - *inp_val);
}



int ang_to_servo_val(int angle) {
  //max and minimum pwm values for servos used
  int minrot = 80;
  int maxrot = 575;

  int result = map(angle, -90, 90, minrot, maxrot);
  return result;
}

void reset_legs() {
  curr1 = ang_to_servo_val(0);
  curr2 = ang_to_servo_val(0);
  curr3 = ang_to_servo_val(0);

  setServos(0, curr1);
  setServos(1, curr2);
  setServos(2, curr3);

  delay(1000);
}

void main_walk_cycle() {
  switch (state_walk) {
    //höften är curr1, servo 0
    //knäet är curr2, servo 1
    case 0:
      updateVal(&curr1, ang_to_servo_val(45));
      updateVal(&curr2, ang_to_servo_val(45));

      updateVal(&curr3, ang_to_servo_val(0));

      if ((abs(curr1 - ang_to_servo_val(0)) < 5) && (abs(curr2 - ang_to_servo_val(0)) < 5)) {
        //delay(1000);
        state_walk = 0;
      }
      break;

    case 1:
      updateVal(&curr1, ang_to_servo_val(40), 0.02);
      updateVal(&curr2, ang_to_servo_val(70), 0.07);

      updateVal(&curr3, ang_to_servo_val(0));

      if ((abs(curr1 - ang_to_servo_val(40)) < 5) && (abs(curr2 - ang_to_servo_val(70)) < 5)) {
        //delay(1000);
        state_walk = 2;
      }

      break;

    case 2:
      updateVal(&curr1, ang_to_servo_val(40));
      updateVal(&curr2, ang_to_servo_val(-10));

      updateVal(&curr3, ang_to_servo_val(0));

      if ((abs(curr1 - ang_to_servo_val(40)) < 5) && (abs(curr2 - ang_to_servo_val(-10)) < 5)) {
        //delay(1000);
        state_walk = 3;
      }

      break;

    case 3:
      updateVal(&curr1, ang_to_servo_val(-30), 0.01);
      updateVal(&curr2, ang_to_servo_val(-40), 0.01);

      updateVal(&curr3, ang_to_servo_val(0));

      if ((abs(curr1 - ang_to_servo_val(-30)) < 5) && (abs(curr2 - ang_to_servo_val(-40)) < 5)) {
        //delay(1000);
        state_walk = 1;
      }
      break;
  }
}


void main_stabilize() {
  switch (state_stable) {
    case 0:
      updateVal(&curr1, ang_to_servo_val(0));
      updateVal(&curr2, ang_to_servo_val(17));

      updateVal(&curr3, ang_to_servo_val(-(0.666 * roty - 60)), 0.5);


      if (roty <= 95) {
        state_walk = 0;
        state_main = 1;
      }
      break;
  }
}
