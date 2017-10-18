#include <MPU9250.h>
#include <quaternionFilters.h>

MPU9250 myIMU;

float prev_ax, prev_ay, prev_az;
float prev_gx, prev_gy, prev_gz;
double angle_pos;
unsigned long current_time;
unsigned long prev_time;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (c != 0x71) {
    Serial.print("Communication with MPU9250 failed!");
    while(true);
  }

  myIMU.MPU9250SelfTest(myIMU.selfTest);
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  myIMU.initMPU9250();
  myIMU.getAres();
  myIMU.getGres();
//  myIMU.getMres();
//  myIMU.initAK8963(myIMU.factoryMagCalibration);
//  myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);

  // init "previous" values to 0
  prev_ax = 0;
  prev_ay = 0;
  prev_az = 0;
  prev_gx = 0;
  prev_gy = 0;
  prev_gz = 0;
  angle_pos = 0;
  current_time = micros();
  prev_time = 0;
  delay(2000);
}

void loop() {
  
  
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    prev_time = current_time;
    current_time = micros();

//    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
//    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
//    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  }

  myIMU.updateTime();

//  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
//                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
//                         myIMU.mx, myIMU.mz, myIMU.deltat);

//  myIMU.delt_t = millis() - myIMU.count;

  // integrate angular velocity to get angular position (only yaw currently)
//  double vel_diff = myIMU.gx - prev_gx;z
  double pos_diff = ((long) myIMU.gz * 10) / 10000000.0 * (double) (current_time - prev_time);

  angle_pos = angle_pos + pos_diff;

  Serial.print("angular position = ");
  Serial.print(angle_pos, 3);
  Serial.print("\n");

//  Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
//  Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
//  Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
//  Serial.println(" mg");

//  Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
//  Serial.print(" degrees/sec ");
//  Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
//  Serial.print(" degrees/sec ");
//  Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
//  Serial.println(" degrees/sec");

//    Serial.print("X-mag field: "); Serial.print(myIMU.mx);
//    Serial.print(" mG ");
//    Serial.print("Y-mag field: "); Serial.print(myIMU.my);
//    Serial.print(" mG ");
//    Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
//    Serial.println(" mG");
}
