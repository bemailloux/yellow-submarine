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
    myIMU.readMagData(myIMU.magCount);
    myIMU.getAres();
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes;
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes;
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes;
    myIMU.getGres();
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;
    myIMU.getMres();
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes;
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes;
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes;
    myIMU.updateTime();
    MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD, myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.mx, myIMU.my, myIMU.mz, myIMU.deltat);
    myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                  * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                  * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                  * *(getQ()+3));
    myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                  * *(getQ()+2)));
    myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                  * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                  * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                  * *(getQ()+3));
    myIMU.pitch *= RAD_TO_DEG;
    myIMU.yaw   *= RAD_TO_DEG;
    myIMU.roll  *= RAD_TO_DEG;
    Serial.print(myIMU.pitch); Serial.print(" ");
    Serial.print(myIMU.roll);  Serial.print(" ");
    Serial.print(myIMU.yaw); Serial.println(" ");
  } 
}

#define MAX_PITCH 20
#define MIN_PITCH -20
void autopilot(int throttle = 0, int pitch_deg = 0, int heading = 180)
{
  readIMU();
  pitch_deg = constrain(pitch_deg, MIN_PITCH, MAX_PITCH);
  int pitch = (int) (atan2(myIMU.ay, myIMU.az) * RAD_TO_DEG);
  //Serial.print(pitch); Serial.print(" ");

  int error = pitch - pitch_deg;
  int pitch_control = map(error, MIN_PITCH, MAX_PITCH, 100, -100);
  pitch_control = constrain(pitch_control, -100, 100);
  //Serial.println(pitch_control);
  int yaw_control = 0;
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
    autopilot();
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
