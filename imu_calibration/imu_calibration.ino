#include "quaternionFilters.h"
#include "MPU9250.h"
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <PS2X_lib.h>

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

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
int analogLX, analogLY; // left stick analog values
int analogRX, analogRY; // right stick analog values
int error;

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

//  while(!ps2x.Button(PSB_START))
//  {
//    ps2x.read_gamepad(false, false);
//  }

  Serial.print("Start pressed, reading gamepad\n");

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
    myIMU.initAK8963(myIMU.factoryMagCalibration);
      
    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
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

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      if (SerialDebug)
      {
        // Print acceleration values in milligs!
        Serial.print("X-acceleration: "); Serial.print(1000 * myIMU.ax);
        Serial.print(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(1000 * myIMU.ay);
        Serial.print(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(1000 * myIMU.az);
        Serial.println(" mg ");

        // Print gyro values in degree/sec
        Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
        Serial.println(" degrees/sec");

        // Print mag values in degree/sec
        Serial.print("X-mag field: "); Serial.print(myIMU.mx);
        Serial.print(" mG ");
        Serial.print("Y-mag field: "); Serial.print(myIMU.my);
        Serial.print(" mG ");
        Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
        Serial.println(" mG");

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
        Serial.println(" degrees C");
      }

      myIMU.count = millis();
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 500)
    {
      if (SerialDebug)
      {
//        /*Serial.print("ax = ");*/  Serial.print((int)1000 * myIMU.ax);
//        Serial.print(" "); Serial.print((int)1000 * myIMU.ay);
//        Serial.print(" "); Serial.println((int)1000 * myIMU.az);
//        Serial.println(" mg");

//        Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
//        Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
//        Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
//        Serial.println(" deg/s");
//
//        Serial.print("mx = ");  Serial.print((int)myIMU.mx);
//        Serial.print(" my = "); Serial.print((int)myIMU.my);
//        Serial.print(" mz = "); Serial.print((int)myIMU.mz);
//        Serial.println(" mG");
//
//        Serial.print("q0 = ");  Serial.print(*getQ());
//        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
//        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
//        Serial.print(" qz = "); Serial.println(*(getQ() + 3));
      }

      // Define output variables from updated quaternion---these are Tait-Bryan
      // angles, commonly used in aircraft orientation. In this coordinate system,
      // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
      // x-axis and Earth magnetic North (or true North if corrected for local
      // declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
      // arise from the definition of the homogeneous rotation matrix constructed
      // from quaternions. Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order which for this configuration is yaw,
      // pitch, and then roll.
      // For more see
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ()
                                  * *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1)
                          * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3)
                          * *(getQ() + 3));
      myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ()
                                  * *(getQ() + 2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2)
                                  * *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1)
                          * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3)
                          * *(getQ() + 3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw  -= 8.5;
      myIMU.roll *= RAD_TO_DEG;

      if (SerialDebug)
      {
//        Serial.print("Yaw, Pitch, Roll: ");
        Serial.println(myIMU.yaw, 2);
//        Serial.print(" ");
//        Serial.print(myIMU.pitch, 2);
//        Serial.print(" ");
//        Serial.println(myIMU.roll, 2);

//        Serial.print("rate = ");
//        Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
//        Serial.println(" Hz");
      }

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)

  if(myIMU.yaw > 5)
  {
    valueL = MTR_OFF + myIMU.yaw;
    valueR = MTR_OFF;
  }
  else if(myIMU.yaw < -5)
  {
    valueL = MTR_OFF;
    valueR = MTR_OFF - myIMU.yaw;
  }
  else
  {
    valueL = MTR_OFF;
    valueR = MTR_OFF;
  }

  esc_F.writeMicroseconds(valueF);
  esc_B.writeMicroseconds(valueB);
  esc_L.writeMicroseconds(valueL);
  esc_R.writeMicroseconds(valueR);
}
