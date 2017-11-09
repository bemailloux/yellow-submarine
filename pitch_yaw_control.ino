/*************** MOTOR NOTES ******************
 * LEFT MOTOR: FWD = CCW => PWM > 1464; REV = CW => PWM < 1464
 * RIGHT MOTOR: FWD = CW => PWM < 1464; REV = CCW => PWM > 1464 
 * FRONT MOTOR: FWD = CCW => PWM > 1464; REV = CW => PWM < 1464
 * BACK MOTOR: FWD = CW => PWM > 1464; REV = CCW => PWM < 1464  
*/

// INCLUDES
#include <MPU9250.h>
#include <quaternionFilters.h>
#include <MadgwickAHRS.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <PS2X_lib.h>

// DEFINES
// debug
#define DEBUG

// madgwick
#define FREQ 25

// ps2x pins
#define PS2_DAT 4 // data - brown (white)
#define PS2_CMD 5 // command - orange
#define PS2_ATT 6 // attention - yellow
#define PS2_CLK 7 // clock - blue

// ESC pins
#define ESC_PIN_F 8
#define ESC_PIN_B 9
#define ESC_PIN_L 10
#define ESC_PIN_R 11

// LED pins
#define LED_GREEN 22
#define LED_RED 23

// motor speed constants
#define MAX_REV 1064  // max reverse speed
#define MTR_OFF 1464  // stop
#define MAX_FWD 1864  // max forward speed

// GLOBALS
MPU9250 myIMU;
Madgwick filter;
unsigned long microsNow, microsPerReading, microsPrevious;

int valueF, valueB, valueL, valueR;
int analogLX, analogLY; // left stick analog values
int analogRX, analogRY; // right stick analog values
int error;
float pitch_ref, pitch;
float yaw_ref, yaw;

Servo esc_F;  // front
Servo esc_B;  // back
Servo esc_L;  // left
Servo esc_R;  // right

PS2X ps2x;

bool autonomous_mode = false;

// FUNCTIONS
void debugPrint(const char* str) {
  #ifdef DEBUG
  Serial.print(str);
  Serial.print("\n");
  #endif
}

void teleoperation() {
  analogLX = ps2x.Analog(PSS_LX);
  analogLY = ps2x.Analog(PSS_LY);

  analogRX = ps2x.Analog(PSS_RX);
  analogRY = ps2x.Analog(PSS_RY);

  // forward / backward / left / right motion
  if (analogLX == 128 && analogLY == 128) {
    // motors off
    valueL = MTR_OFF;
    valueR = MTR_OFF;
  }
  else if (analogLX > analogLY && analogLX < 256 - analogLY) {
    // move forward; power depends on how far the stick is pushed
    // left motor CCW (> 1464), right motor CW (< 1464)    
    valueL = MTR_OFF + (MAX_FWD - MTR_OFF) / 128.0 * (128 - analogLY) ;
    valueR = MTR_OFF - (MTR_OFF - MAX_REV) / 128.0 * (128 - analogLY) ;
  }
  else if (analogLX > 256 - analogLY && analogLX < analogLY) {
    // move backward
    // left motor CW (< 1464), right motor CCW (> 1464)
    valueL = MTR_OFF - (MTR_OFF - MAX_REV) / 128.0 * (analogLY - 128);
    valueR = MTR_OFF + (MAX_FWD - MTR_OFF) / 128.0 * (analogLY - 128);
  }
  else if (analogLY < 256 - analogLX && analogLY > analogLX) {
    // move left (both motors CW)    
    valueL = MTR_OFF - (MTR_OFF - MAX_REV) / 128.0 * (128 - analogLX);
    valueR = MTR_OFF - (MTR_OFF - MAX_REV) / 128.0 * (128 - analogLX);
  }
  else if (analogLY < analogLX && analogLY > 256 - analogLX) {
    // move right (both motors CCW)
    valueL = MTR_OFF + (MAX_FWD - MTR_OFF) / 128.0 * (analogLX - 128);
    valueR = MTR_OFF + (MAX_FWD - MTR_OFF) / 128.0 * (analogLX - 128);
  }
  else {
    valueL = MTR_OFF;
    valueR = MTR_OFF;
  }

  // up / down / pitch motion
  if (analogRX == 128 && analogRY == 128) {
    // motors off
    valueF = MTR_OFF;
    valueB = MTR_OFF;
  }
  else if (analogRX > analogRY && analogRX < 256 - analogRY) {
    // move up; power depends on how far the stick is pushed
    // front CW, back CCW
    valueF = MTR_OFF - (MTR_OFF - MAX_REV) / 128.0 * (128 - analogRY) ;
    valueB = MTR_OFF + (MAX_FWD - MTR_OFF) / 128.0 * (128 - analogRY) ;
  }
  else if (analogRX > 256 - analogRY && analogRX < analogRY) {
    // move down
    // front CCW, back CW
    valueF = MTR_OFF + (MAX_FWD - MTR_OFF) / 128.0 * (analogRY - 128);
    valueB = MTR_OFF - (MTR_OFF - MAX_REV) / 128.0 * (analogRY - 128);
  }
  else if (analogRY < 256 - analogRX && analogRY > analogRX) {
    // left = pitch nose low (both CCW)
    valueF = MTR_OFF + (MAX_FWD - MTR_OFF) / 128.0 * (128 - analogRX);
    valueB = MTR_OFF + (MAX_FWD - MTR_OFF) / 128.0 * (128 - analogRX);
  }
  else if (analogRY < analogRX && analogRY > 256 - analogRX) {
    // right = pitch nose high (both CW)
    valueF = MTR_OFF - (MTR_OFF - MAX_REV) / 128.0 * (analogRX - 128);
    valueB = MTR_OFF - (MTR_OFF - MAX_REV) / 128.0 * (analogRX - 128);
  }
  else {
    valueF = MTR_OFF;
    valueB = MTR_OFF;
  }

  esc_F.writeMicroseconds(valueF);
  esc_B.writeMicroseconds(valueB);
  esc_L.writeMicroseconds(valueL);
  esc_R.writeMicroseconds(valueR);  
}

void autonomous_pitch() {
  if(ps2x.ButtonPressed(PSB_PAD_UP) && pitch_ref < 30)
  {
    pitch_ref += 5;
  }
  
  if (ps2x.ButtonPressed(PSB_PAD_DOWN) && pitch_ref > -30) 
  {
    pitch_ref -= 5;
  }
  
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  pitch = (int) (atan2(myIMU.ay, myIMU.az) * RAD_TO_DEG);
  
  myIMU.updateTime();
  
  // Serial print and/or display at 0.5 s rate independent of data rates
  myIMU.delt_t = millis() - myIMU.count;

  // update LCD once per half-second independent of read rate
  if(myIMU.delt_t > 500)
  {
    // Borrow counter for printing pitch to print yaw too
    Serial.print(yaw);
    Serial.print(" ");
    //
    Serial.println(pitch);
    myIMU.count = millis();
    myIMU.sumCount = 0;
    myIMU.sum = 0;
  }
  
  if(pitch > pitch_ref)
  {
    valueF = MTR_OFF - 10*(pitch - pitch_ref);
    valueB = MTR_OFF - 10*(pitch - pitch_ref);
  }
  else if(pitch < pitch_ref)
  {
    valueF = MTR_OFF + 10*(pitch - pitch_ref);
    valueB = MTR_OFF + 10*(pitch - pitch_ref);
  }  
  if (valueF < MAX_REV) valueF = MAX_REV;
  if (valueB < MAX_REV) valueB = MAX_REV;
  if (valueF > MAX_FWD) valueF = MAX_FWD;
  if (valueB > MAX_FWD) valueB = MAX_FWD;

  esc_F.writeMicroseconds(valueF);
  esc_B.writeMicroseconds(valueB);
}

void autonomous_yaw() {
  if(ps2x.ButtonPressed(PSB_PAD_LEFT) && yaw_ref < 210)
  {
    yaw_ref += 5;
  }
  
  if (ps2x.ButtonPressed(PSB_PAD_RIGHT) && yaw_ref > 150) 
  {
    yaw_ref -= 5;
  }
  
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    readmyIMU(&myIMU);
//  filter.update(myIMU.gx, myIMU.gy, myIMU.gz,
//                  myIMU.ax, myIMU.ay, myIMU.az,
//                  myIMU.mx, myIMU.my, myIMU.mz);
    filter.updateIMU(myIMU.gx, myIMU.gy, myIMU.gz, myIMU.ax, myIMU.ay, myIMU.az);

    yaw = filter.getYaw();

    microsPrevious = microsPrevious + microsPerReading;
  }
  
  if(yaw > yaw_ref)
  {
    valueL = MTR_OFF - 10*(yaw - yaw_ref);
    valueR = MTR_OFF - 10*(yaw - yaw_ref);
  }
  else if(yaw < yaw_ref)
  {
    valueL = MTR_OFF + 10*(yaw - yaw_ref);
    valueR = MTR_OFF + 10*(yaw - yaw_ref);
  }  
  if (valueL < MAX_REV) valueL = MAX_REV;
  if (valueR < MAX_REV) valueR = MAX_REV;
  if (valueL > MAX_FWD) valueL = MAX_FWD;
  if (valueR > MAX_FWD) valueR = MAX_FWD;

  esc_L.writeMicroseconds(valueL);
  esc_R.writeMicroseconds(valueR);
}

void setupPS2Ctrl()
{
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, false, false);
  if(error == 0){
    debugPrint("Found Controller, configured successful ");
  }  
  else if(error == 1)
    debugPrint("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
    debugPrint("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if(error == 3)
    debugPrint("Controller refusing to enter Pressures mode, may not support it. ");

  debugPrint("PS2 Setup Complete\n");
}

void setupIMU() 
{
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
//  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
//  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    debugPrint("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
//    Serial.print("x-axis self test: acceleration trim within : ");
//    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
//    Serial.print("y-axis self test: acceleration trim within : ");
//    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
//    Serial.print("z-axis self test: acceleration trim within : ");
//    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
//    Serial.print("x-axis self test: gyration trim within : ");
//    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
//    Serial.print("y-axis self test: gyration trim within : ");
//    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
//    Serial.print("z-axis self test: gyration trim within : ");
//    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    debugPrint("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
//    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
//    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    debugPrint("AK8963 initialized for active data mode....");

  } // if (c == 0x71)
  else
  {
    debugPrint("Could not connect to MPU9250: 0x");
//    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void readmyIMU(MPU9250* IMU) {
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (IMU->readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    IMU->readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    IMU->getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    IMU->ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    IMU->ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    IMU->az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    IMU->readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    IMU->getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    IMU->gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    IMU->gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    IMU->gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    IMU->readMagData(myIMU.magCount);  // Read the x/y/z adc values
    IMU->getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    IMU->magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    IMU->magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    IMU->magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    IMU->mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    IMU->my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    IMU->mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  IMU->updateTime();
}

void setup()
{
  Wire.begin();
  Serial.begin(38400);

  // LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  valueF = MTR_OFF;
  valueB = MTR_OFF;
  valueL = MTR_OFF;
  valueR = MTR_OFF;
  
  esc_F.attach(ESC_PIN_F);
  esc_B.attach(ESC_PIN_B);
  esc_L.attach(ESC_PIN_L);
  esc_R.attach(ESC_PIN_R);

  esc_F.writeMicroseconds(valueF);
  esc_B.writeMicroseconds(valueB);
  esc_L.writeMicroseconds(valueL);
  esc_R.writeMicroseconds(valueR);

  pitch = 0;
  pitch_ref = 0;
  yaw = 180;
  yaw_ref = 180;

  // PS2 controller setup
  delay(300); // time for PS2 controller to start up
  setupPS2Ctrl();  

  // IMU setup
  setupIMU();  

  filter.begin(FREQ);
  microsPerReading = 1000000 / FREQ;
  microsPrevious = micros();
}

void loop()
{
  ps2x.read_gamepad(false, false);  

  // AUTONOMOUS MODE
  if (autonomous_mode) {
    digitalWrite(LED_RED, true);
    digitalWrite(LED_GREEN, false);
    autonomous_pitch();
    autonomous_yaw();
  }
  
  // TELEOPERATION MODE
  else {
    pitch_ref = 0;
    yaw_ref = 180;
    digitalWrite(LED_RED, false);
    digitalWrite(LED_GREEN, true);
    teleoperation();
  }

  // KILL SWITCH
  if(ps2x.Button(PSB_CIRCLE))
  {
    esc_F.writeMicroseconds(MTR_OFF);
    esc_B.writeMicroseconds(MTR_OFF);
    esc_L.writeMicroseconds(MTR_OFF);
    esc_R.writeMicroseconds(MTR_OFF);

    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);    
    autonomous_mode = false;
    while(!ps2x.Button(PSB_SQUARE)) {
      ps2x.read_gamepad(false, false);  
    }
   }

  // SWITCH MODES
  if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
    autonomous_mode = !autonomous_mode;
  }
}
