#include <MPU9250.h>
#include <MadgwickAHRS.h>

MPU9250 myIMU;
Madgwick filter;
unsigned long microsNow, microsPerReading, microsPrevious;

#define FREQ 25

void setup()
{
  Wire.begin();
  Serial.begin(38400);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

  filter.begin(FREQ);
  microsPerReading = 1000000 / FREQ;
  microsPrevious = micros();
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

void loop()
{
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    readmyIMU(&myIMU);
    filter.update(myIMU.gx, myIMU.gy, myIMU.gz,
                  myIMU.ax, myIMU.ay, myIMU.az,
                  myIMU.mx, myIMU.my, myIMU.mz);
    //filter.updateIMU(myIMU.gx, myIMU.gy, myIMU.gz, myIMU.ax, myIMU.ay, myIMU.az);
    Serial.print(filter.getPitch()); Serial.print(" ");
    Serial.print(filter.getRoll()); Serial.print(" ");
    Serial.println(filter.getYaw());
    microsPrevious = microsPrevious + microsPerReading;
  }
}
