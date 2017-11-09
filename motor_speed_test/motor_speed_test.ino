/*************** MOTOR NOTES ******************
 * LEFT MOTOR: FWD = CCW => PWM > 1464; REV = CW => PWM < 1464
 * RIGHT MOTOR: FWD = CW => PWM < 1464; REV = CCW => PWM > 1464 
 * FRONT MOTOR: FWD = CCW => PWM > 1464; REV = CW => PWM < 1464
 * BACK MOTOR: FWD = CW => PWM > 1464; REV = CCW => PWM < 1464  
*/

// INCLUDES
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <PS2X_lib.h>

// DEFINES
// debug
#define DEBUG

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

void moveForward(int motorSpeed) {
  valueF = MTR_OFF;
  valueB = MTR_OFF;
  valueL = MTR_OFF + motorSpeed;
  valueR = MTR_OFF - motorSpeed;

  esc_F.writeMicroseconds(valueF);
  esc_B.writeMicroseconds(valueB);
  esc_L.writeMicroseconds(valueL);
  esc_R.writeMicroseconds(valueR);  
}

void moveBackward(int motorSpeed) {
  valueF = MTR_OFF;
  valueB = MTR_OFF;
  valueL = MTR_OFF - motorSpeed;
  valueR = MTR_OFF + motorSpeed;

  esc_F.writeMicroseconds(valueF);
  esc_B.writeMicroseconds(valueB);
  esc_L.writeMicroseconds(valueL);
  esc_R.writeMicroseconds(valueR);
}

void moveUp(int motorSpeed) {
  valueF = MTR_OFF - motorSpeed;
  valueB = MTR_OFF + motorSpeed;
  valueL = MTR_OFF;
  valueR = MTR_OFF;

  esc_F.writeMicroseconds(valueF);
  esc_B.writeMicroseconds(valueB);
  esc_L.writeMicroseconds(valueL);
  esc_R.writeMicroseconds(valueR);  
}

void moveDown(int motorSpeed) {
  valueF = MTR_OFF + motorSpeed;
  valueB = MTR_OFF - motorSpeed;
  valueL = MTR_OFF;
  valueR = MTR_OFF;

  esc_F.writeMicroseconds(valueF);
  esc_B.writeMicroseconds(valueB);
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
}

void loop()
{ 
  int motorSpeed = 100;
  bool motorsRunning = false;

  while(!ps2x.Button(PSB_CROSS)) {
    motorsRunning = false;
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    
    esc_F.writeMicroseconds(MTR_OFF);
    esc_B.writeMicroseconds(MTR_OFF);
    esc_L.writeMicroseconds(MTR_OFF);
    esc_R.writeMicroseconds(MTR_OFF);

    ps2x.read_gamepad(false, false);
  }

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);

  while(!motorsRunning) {
    if (ps2x.Button(PSB_PAD_UP)) {
      moveForward(motorSpeed);
      motorsRunning = true;
      Serial.println("Moving forward");
    }
    else if (ps2x.Button(PSB_PAD_DOWN)) {
      moveBackward(motorSpeed);
      motorsRunning = true;
      Serial.println("Moving backward");
    }
    else if (ps2x.Button(PSB_PAD_LEFT)) {
      moveUp(motorSpeed);
      motorsRunning = true;
      Serial.println("Moving up");
    }
    else if (ps2x.Button(PSB_PAD_RIGHT)) {
      moveDown(motorSpeed);
      motorsRunning = true;
      Serial.println("Moving down");
    }  
    ps2x.read_gamepad(false, false);
  }

  while(!ps2x.Button(PSB_CIRCLE)) {
    ps2x.read_gamepad(false, false);
  }
}
