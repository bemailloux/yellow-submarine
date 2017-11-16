#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include "PS2X_lib.h"

#include <MPU9250.h>
#include <quaternionFilters.h>

// macros
#define PS2_DAT 4 // data - brown (white)
#define PS2_CMD 5 // command - orange
#define PS2_ATT 6 // attention - yellow
#define PS2_CLK 7 // clock - blue

#define ESC_PIN_F 8
#define ESC_PIN_B 9
#define ESC_PIN_L 10
#define ESC_PIN_R 11

#define LED_GREEN 22
#define LED_RED 23

#define MAX_REV 1064  // max reverse speed
#define MTR_OFF 1464  // stop
#define MAX_FWD 1864  // max forward speed

Servo esc_F;  // front
Servo esc_B;  // back
Servo esc_L;  // left
Servo esc_R;  // right

PS2X ps2x;

MPU9250 myIMU;

void connect_imu()
{
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    
    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    myIMU.initMPU9250();

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    Serial.println("Calibration Complete\n");

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
}

void connect_ps2()
{
  int error;
  delay(300); // time for PS2 controller to start up
    
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, false, false);
  if(error == 0){
    Serial.print("Found Controller, configured successful ");
  }  
  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  Serial.print("PS2 Setup Complete\n");
}

void readIMU()
{
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.readGyroData(myIMU.gyroCount);
    myIMU.getAres();
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes;
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes;
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes;
    myIMU.getGres();
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;
    myIMU.updateTime();
  } 
}

// AHRS Globals
float beta, q0, q1, q2, q3;

void initAHRS() {
  beta = 0.1f;
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
}

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void updateAHRS() {
  float recipNorm;
  float ax, ay, az;
  float gx, gy, gz;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Load Values from IMU
  readIMU();
  ax = myIMU.ax;
  ay = myIMU.ay;
  az = myIMU.az;
  gx = myIMU.gx;
  gy = myIMU.gy;
  gz = myIMU.gz;
    
  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * myIMU.deltat;
  q1 += qDot2 * myIMU.deltat;
  q2 += qDot3 * myIMU.deltat;
  q3 += qDot4 * myIMU.deltat;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  // Store output values in IMU angle fields
  myIMU.roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * RAD_TO_DEG;
  myIMU.pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * RAD_TO_DEG;
  myIMU.yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * RAD_TO_DEG;

  Serial.print(myIMU.roll); Serial.print(" ");
  Serial.print(myIMU.pitch); Serial.print(" ");
  Serial.print(myIMU.yaw); Serial.println("");
}

#define MAX_PITCH 20
#define MIN_PITCH -20

#define MAX_YAW 20
#define MIN_YAW -20
void autopilot(int throttle = 0, int pitch_deg = 0, int heading = 180)
{
  updateAHRS();

  // Pitch
  pitch_deg = constrain(pitch_deg, MIN_PITCH, MAX_PITCH);
  int pitch_error = myIMU.pitch - pitch_deg;
  int pitch_control = map(error, MIN_PITCH, MAX_PITCH, 100, -100);
  pitch_control = constrain(pitch_control, -100, 100);

  // Yaw
  heading = constrain(heading, 0, 360);
  int heading_error = myIMU.yaw - heading;
  int yaw_control = map(error, MIN_YAW, MAX_YAW, 100, -100);
  yaw_control = constrain(yaw_control, -100, 100);
   
  motors(throttle, yaw_control, 0, pitch_control);
}

// Centralized 'drive' funciton
//    All values normalized from - 100 to 100
//    Thrust is forward (+) and backward (-)
//    Yaw is right (+) and left (-)
//    Collective is vertically up (+) and vertically down (-)
//    Pitch is nose-up (+) and nose-down (-)
void motors(int thrust, int yaw, int collective, int pitch)
{
  int valueL = MTR_OFF;
  int valueR = MTR_OFF;
  int valueF = MTR_OFF;
  int valueB = MTR_OFF;
  // Aft-facing Motors
  // Props are counter-rotating
  valueL = map(thrust, -100, 100, MAX_REV, MAX_FWD);
  valueR = map(thrust, -100, 100, MAX_FWD, MAX_REV);

  // Yaw will NOT use opposing thrust, we will just reduce power on the inside thruster. 
  if(thrust != 0) {
    if(yaw > 0) { // RIGHT
      valueR = map(abs(yaw), 0, 100, valueR, MTR_OFF);
    } else if (yaw < 0) { // LEFT
      valueL = map(abs(yaw), 0, 100, valueL, MTR_OFF);
    }
  } else {
    // If not moving forward, we will instead run the outside thruster. 
    if(yaw > 0) { // RIGHT
      valueL = map(abs(yaw), 0, 100, MTR_OFF, MAX_FWD);
    } else if (yaw < 0) { // LEFT
      valueR = map(abs(yaw), 0, 100, MTR_OFF, MAX_REV);
    }
  }

  // Down-facing Motors
  // Props are counter-rotating
  valueB = map(collective, -100, 100, MAX_REV, MAX_FWD);
  valueF = map(collective, -100, 100, MAX_FWD, MAX_REV);

  // Yaw will NOT use opposing thrust, we will just reduce power on the inside thruster. 
  if(collective != 0) {
    if(pitch > 0) { // UP
      valueB = map(abs(pitch), 0, 100, valueB, MTR_OFF);
    } else if (pitch < 0) { // DOWN
      valueF = map(abs(pitch), 0, 100, valueF, MTR_OFF);
    }
  } else {
    // If not moving forward, we will instead run the outside thruster. 
    if(pitch > 0) { // UP
      valueF = map(abs(pitch), 0, 100, MTR_OFF, MAX_FWD);
    } else if (pitch < 0) { // DOWN
      valueB = map(abs(pitch), 0, 100, MTR_OFF, MAX_REV);
    }
  }

  valueF = constrain(valueF, MAX_REV, MAX_FWD);
  valueB = constrain(valueB, MAX_REV, MAX_FWD);
  valueL = constrain(valueL, MAX_REV, MAX_FWD);
  valueR = constrain(valueR, MAX_REV, MAX_FWD);

  //Serial.print(valueL); Serial.print(" ");
  //Serial.print(valueR); Serial.print(" ");
  //Serial.print(valueF); Serial.print(" ");
  //Serial.print(valueB); Serial.println("");
  
  esc_F.writeMicroseconds(valueF);
  esc_B.writeMicroseconds(valueB);
  esc_L.writeMicroseconds(valueL);
  esc_R.writeMicroseconds(valueR);
}

#define ANALOG_MAX 256
#define ANALOG_MIN 0

void teleoperate()
{
  int throttle = 0;
  int yaw = 0;
  int collective = 0;
  int pitch = 0;

  if(ps2x.Button(PSB_L1)) { 
    throttle = 100; 
  } else if (ps2x.Button(PSB_L2)) { 
    throttle = -100; 
  }
  
  if(ps2x.Button(PSB_R1)) { 
    collective = -100; 
  } else if (ps2x.Button(PSB_R2)) { 
    collective = 100; 
  }

  yaw = map(ps2x.Analog(PSS_LX), ANALOG_MIN, ANALOG_MAX, -100, 100);
  pitch = map(ps2x.Analog(PSS_RY), ANALOG_MIN, ANALOG_MAX, -100, 100);

  motors(throttle, yaw, collective, pitch);
}

void waitForStart()
{
  while(!ps2x.Button(PSB_START)) { ps2x.read_gamepad(false, false); };
}

///////////////////////////////////////
//////////////// SETUP ////////////////
///////////////////////////////////////
void setup() {
  Wire.begin();
  Serial.begin(115200);

  esc_F.attach(ESC_PIN_F);
  esc_B.attach(ESC_PIN_B);
  esc_L.attach(ESC_PIN_L);
  esc_R.attach(ESC_PIN_R);

  connect_imu();

  initAHRS();

  // LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  motors(0, 0, 0, 0);

  connect_ps2();

  // LEDs OFF to start
  digitalWrite(LED_RED, 0);
  digitalWrite(LED_GREEN, 0);

  waitForStart();
  digitalWrite(LED_GREEN, 1);
}

bool autonomous = false;

///////////////////////////////////////
//////////////// LOOP /////////////////
///////////////////////////////////////
void loop() {
  ps2x.read_gamepad(false, false);

  if(autonomous) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    autopilot(100, 0, 180);
  } else {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);
    teleoperate();
  }

  if(ps2x.ButtonPressed(PSB_TRIANGLE)) {
    autonomous = !autonomous;
  }
  
  // KILL SWITCH
  if(ps2x.Button(PSB_CIRCLE))
  {
    motors(0, 0, 0, 0);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);    
    while(!ps2x.Button(PSB_SQUARE)) {
      ps2x.read_gamepad(false, false);  
    }
    autonomous = false;
    digitalWrite(LED_RED, LOW);
  }
}
