#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include "PS2X_lib.h"

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

  Serial.print(valueL); Serial.print(" ");
  Serial.print(valueR); Serial.print(" ");
  Serial.print(valueF); Serial.print(" ");
  Serial.print(valueB); Serial.println("");
  
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

  // LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  motors(0, 0, 0, 0);

  connect_ps2();
}

///////////////////////////////////////
//////////////// LOOP /////////////////
///////////////////////////////////////
void loop() {
  ps2x.read_gamepad(false, false);

  //Serial.print(ps2x.Analog(PSS_LX)); Serial.print(" ");
  //Serial.print(ps2x.Analog(PSS_RY)); Serial.println(" ");
  
  teleoperate();
  // KILL SWITCH
  if(ps2x.Button(PSB_CIRCLE))
  {
    motors(0, 0, 0, 0);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);    
    while(!ps2x.Button(PSB_SQUARE)) {
      ps2x.read_gamepad(false, false);  
    }
  }

}
