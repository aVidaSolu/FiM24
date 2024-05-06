#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//set up the servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//addresses to the MPU and variables to hold data from MPU
const int MPU_ADDR = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

//current values (pwm) the servo is holding, this is seperate from the values we want each servo to hold
//the function updateVal is used to update one of these current values closer towards a goal value (PID Controller)
float leg_currs[12];

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


  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission(true);

  Serial.begin(9600);

  //setup servo driver
  pwm.begin();
  pwm.setPWMFreq(50);

  pinMode(8, INPUT);
}



void readAngles(double *x, double *y, double *z);            //get current rotational angle from MPU
void setServos(int serv, int value);                         //set servo serv to a specific PWM value
void updateVal(float *inp_val, float req_val);               //update value inp_val closer towards req_val (PID CONTROLLER)
void updateVal(float *inp_val, float req_val, float new_K);  //update value inp_val closer towards req_val (PID CONTROLLER), set K value manually
void updateVal(float *inp_val, float req_val, float new_K, float new_I, float new_D);  //update value inp_val closer towards req_val (PID CONTROLLER), set PID manually
int ang_to_servo_val(int angle);                             //map angle (-90, 90) to servo PWM value (80-575)

void reset_legs();  //initialize leg positions

void main_walk_cycle();  //main switch case for walking cycle / walking case
void main_stabilize();   //main switch case for stabilization cycle


int const_zero_ang = ang_to_servo_val(0);



//state variables for use in the main switch case in void loop()
int state_main = 0;
int state_walk = 0;
int state_stable = 0;

//walk cycle timers
int walk_timer_base = 100;
int walk_timer = walk_timer_base;



void loop() {

  readAngles(&rotx, &roty, &rotz);

  int button = digitalRead(8);

  switch (state_main) {
    case 0:
      //initialization case
      reset_legs();

      if (button == 0) { state_main = 1; }

      break;

    case 1:
      //walking case
      //main_walk_cycle();
      reset_legs();

      if (button == 1) { state_main = 3; }
      break;

    case 2:
      //stabilization case
      main_stabilize();

      if (button == 1) { state_main = 0; }
      break;

    case 3:
      //pause intermidiary

      reset_legs();

      if (button == 0) {
        state_main = 4;
      }
      break;

    case 4:
      //pause

      state_main = 2;

      if (button == 1) {
        state_main = 0;
      }
      break;
  }






  //  set all servos to their current values
  for (int i = 0; i < 12; i++) {
    setServos(i, leg_currs[i]);
  }
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
  int const_muls[12] = { 1, -1, 1, -1, 1, -1, -1, 1, 1, -1, -1, 1 };
  int new_val = ang_to_servo_val(const_muls[serv] * value);
  int const_offs[12] = { 10, 45, 23, 15, 0, -20, -11, -12, -63, 55, 47, -15 };
  pwm.setPWM(serv, 0, new_val + (ang_to_servo_val(const_offs[serv]) - const_zero_ang));
}

void updateVal(float *inp_val, float req_val) {
  float K_P = 0.1;
  *inp_val += K_P * (req_val - *inp_val);
}

void updateVal(float *inp_val, float req_val, float new_K) {
  *inp_val += new_K * (req_val - *inp_val);
}

double int_part = 0;
double past_value = 0;

void updateVal(float *inp_val, float req_val, float new_K, float new_I, float new_D) {


	float discr = (req_val - *inp_val);
  if ( rotz >= 180 ) {  
	*inp_val += new_K*discr + new_I*int_part - new_D * (rotz - 359  - past_value);
  
  }
  else {  
   	*inp_val += new_K*discr + new_I*int_part - new_D * (rotz - past_value);
  
   }
}



int ang_to_servo_val(int angle) {
  //max and minimum pwm values for servos used
  int minrot = 80;
  int maxrot = 575;

  int result = map(angle, -90, 90, minrot, maxrot);
  return result;
}

void reset_legs() {
  for (int i = 0; i < 12; i++) {
    leg_currs[i] = 0;
  }
}



//state börjar 0
//ändra walk_timer_base i varje case för timings
void main_walk_cycle() {
  switch (state_walk) {
    case 0:
      for (int b = 4; b < 8; b++) { leg_currs[b] = 20; }
      for (int b = 8; b < 12; b++) { leg_currs[b] = 30; }

      walk_timer_base = 25;

      break;

    case 1:
      leg_currs[7] = 40;
      leg_currs[11] = 40;

      leg_currs[5] = 20;
      leg_currs[9] = 30;

      break;

    case 2:
      leg_currs[11] = 0;

      break;

    case 3:
      leg_currs[7] = 20;
      leg_currs[11] = 30;

      leg_currs[4] = -10;
      leg_currs[8] = 20;

      leg_currs[5] = 40;
      leg_currs[9] = 40;

      break;

    case 4:
      leg_currs[9] = 0;

      break;

    case 5:
      leg_currs[5] = 20;
      leg_currs[9] = 30;

      leg_currs[4] = 20;
      leg_currs[8] = 30;

      leg_currs[6] = 40;
      leg_currs[10] = 40;

      break;

    case 6:
      leg_currs[10] = 0;

      break;

    case 7:
      leg_currs[6] = 20;
      leg_currs[10] = 30;

      leg_currs[5] = -10;
      leg_currs[9] = 20;

      leg_currs[4] = 40;
      leg_currs[8] = 40;

      break;

    case 8:
      leg_currs[8] = 0;

      break;
  }

  walk_timer -= 1;
  if (walk_timer < 0) {
    walk_timer = walk_timer_base;
    state_walk = state_walk + 1;
    if (state_walk == 9) {
      state_walk = 0;
    }
  }
}


float t1, t2, v1, v2;
int itimer = 0; 

//state börjar 0
void main_stabilize() {

  switch (state_stable) {
    case 0:
      double devy = roty - 90;

    // test ok: P1: 0.01, P2: 0.017, I = 0.0001, D = -0.27

//  valid   
      // float K_ALL1 = 0.06;
      // float K_ALL2 = 0.02;
      // float K_int = 0.00016;
      // float K_d = -0.2; 

      float K_ALL1 = 0.06;
      float K_ALL2 = 0.02;
      float K_int = 0.00016;
      float K_d = 0.2; 
      
      


      updateVal(&leg_currs[0], devy, K_ALL1);
      updateVal(&leg_currs[3], devy, K_ALL1);


      updateVal(&leg_currs[1], -devy, K_ALL1);
      updateVal(&leg_currs[2], -devy, K_ALL1);      



   


      double rotz2 = (rotz*71)/4068;
     // int_part += rotz;


      if ( abs(int_part) > 3600 ) { int_part = 0; }

      if (rotz >= 180) {
        int_part -= (359-rotz);
        t1 = (0.08+0.07);
        t2 = (0.08+0.07) - 0.15*tan(-rotz2);
      }
      else
      {
          int_part += rotz;
          t1 = (0.08+0.07) - 0.15*tan(rotz2);
          t2 = (0.08+0.07);
      }

      itimer += 1;
      if (itimer > 100) { itimer = 0; int_part = 0; }

      if ( abs(int_part) > 3600 ) { int_part = 0; }


      v1 = acos(t1/(0.08+0.07));
      v2 = acos(t2/(0.08+0.07));

      if (isnan(v1))
      {
        v1 = 0;
      }

      if (isnan(v2))
      {
        v2 = 0;
      }

      v1 = (v1*4068)/71;
      v2 = (v2*4068)/71;

      // updateVal(&leg_currs[6], v2, K_ALL2, K_int);
      // updateVal(&leg_currs[7], v2, K_ALL2, K_int);
      // updateVal(&leg_currs[10], v2*2, K_ALL2, K_int);
      // updateVal(&leg_currs[11], v2*2, K_ALL2, K_int);

      // updateVal(&leg_currs[4], v1, K_ALL2, K_int);
      // updateVal(&leg_currs[5], v1, K_ALL2, K_int);
      // updateVal(&leg_currs[8], v1*2, K_ALL2, K_int);
      // updateVal(&leg_currs[9], v1*2, K_ALL2, K_int);

      // updateVal(&leg_currs[6], v2, K_ALL2);
      // updateVal(&leg_currs[7], v2, K_ALL2);
      // updateVal(&leg_currs[10], v2*2, K_ALL2);
      // updateVal(&leg_currs[11], v2*2, K_ALL2);

      // updateVal(&leg_currs[4], v1, K_ALL2);
      // updateVal(&leg_currs[5], v1, K_ALL2);
      // updateVal(&leg_currs[8], v1*2, K_ALL2);
      // updateVal(&leg_currs[9], v1*2, K_ALL2);

      
      updateVal(&leg_currs[6], v2, K_ALL2, K_int, K_d);
      updateVal(&leg_currs[7], v2, K_ALL2, K_int, K_d);
      updateVal(&leg_currs[10], v2*2, K_ALL2, K_int, K_d);
      updateVal(&leg_currs[11], v2*2, K_ALL2, K_int, K_d);
  

      updateVal(&leg_currs[4], v1, K_ALL2, K_int, K_d);
      updateVal(&leg_currs[5], v1, K_ALL2, K_int, K_d);
      updateVal(&leg_currs[8], v1*2, K_ALL2, K_int, K_d);
      updateVal(&leg_currs[9], v1*2, K_ALL2, K_int, K_d);






    if (rotz >= 180) { past_value = -359 + rotz;}
    else { past_value = rotz; }

      // Serial.print(rotz);
      // Serial.print(",");
      // Serial.print(t2);
      // Serial.print(",");
      // Serial.print((0.08+0.07) -tan(rotz2));
      // Serial.print(",");
      // Serial.println(v2);
      // Serial.print(3600);
      // Serial.print(int_part);
      // Serial.print(',');
      // Serial.println((int)rotz);

      
      




      break;
  }
}
