#include "quaternionFilters.h"
#include "MPU9250.h"
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <PS2X_lib.h>

/*************** MOTOR NOTES ******************
 * LEFT MOTOR: FWD = CCW => PWM > 1464; REV = CW => PWM < 1464
 * RIGHT MOTOR: FWD = CW => PWM < 1464; REV = CCW => PWM > 1464 
 * FRONT MOTOR: FWD = CCW => PWM > 1464; REV = CW => PWM < 1464
 * BACK MOTOR: FWD = CW => PWM > 1464; REV = CCW => PWM < 1464  
*/

// macros
#define PS2_DAT 4 // data - brown (white)
#define PS2_CMD 5 // command - orange
#define PS2_ATT 6 // attention - yellow
#define PS2_CLK 7 // clock - blue

#define ESC_PIN_F 8
#define ESC_PIN_B 9
#define ESC_PIN_L 10
#define ESC_PIN_R 11

#define MAX_REV 1064  // max reverse speed
#define MTR_OFF 1464  // stop
#define MAX_FWD 1864  // max forward speed

// globals
MPU9250 myIMU;

int valueF, valueB, valueL, valueR;
int error;
float pitch_ref, pitch;

Servo esc_F;  // front
Servo esc_B;  // back
Servo esc_L;  // left
Servo esc_R;  // right

PS2X ps2x;

void setup()
{
  Wire.begin();
  Serial.begin(115200);

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

  pitch_ref = 0;
  
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

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    
    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    myIMU.initMPU9250();

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    // Initialize device for active mode read of magnetometer
    //myIMU.initAK8963(myIMU.factoryMagCalibration);
    
    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
   // myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    Serial.println("Calibration Complete\n");
    delay(2000); // Add delay to see results before serial spew of data

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

void loop()
{
  ps2x.read_gamepad(false, false);

  if(ps2x.Button(PSB_SQUARE))
  {
    pitch_ref = 30;
    Serial.println("SQUARE");
  }
  else
  {
    pitch_ref = 0;
  }
  
  if(ps2x.Button(PSB_CIRCLE))
  {
    esc_F.writeMicroseconds(MTR_OFF);
    esc_B.writeMicroseconds(MTR_OFF);
    esc_L.writeMicroseconds(MTR_OFF);
    esc_R.writeMicroseconds(MTR_OFF);

    while(true);
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

  esc_F.writeMicroseconds(valueF);
  esc_B.writeMicroseconds(valueB);
  esc_L.writeMicroseconds(valueL);
  esc_R.writeMicroseconds(valueR);
}
