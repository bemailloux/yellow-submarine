#include <Servo.h>
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
int valueF, valueB, valueL, valueR;
int analogLX, analogLY; // left stick analog values
int analogRX, analogRY; // right stick analog values
int error;

Servo esc_F;  // front
Servo esc_B;  // back
Servo esc_L;  // left
Servo esc_R;  // right
PS2X ps2x;

void setup() {
  Serial.begin(115200);

  valueF = MTR_OFF;
  valueB = MTR_OFF;
  valueL = MTR_OFF;
  valueR = MTR_OFF;
  
  esc_F.attach(ESC_PIN_F);
  esc_B.attach(ESC_PIN_B);
  esc_L.attach(ESC_PIN_L);
  esc_R.attach(ESC_PIN_R);
    
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

  esc_F.writeMicroseconds(valueF);
  esc_B.writeMicroseconds(valueB);
  esc_L.writeMicroseconds(valueL);
  esc_R.writeMicroseconds(valueR);
  
  Serial.print("Setup Complete\n");
}

void loop() {
  ps2x.read_gamepad(false, false);

  valueF = MTR_OFF;
  valueB = MTR_OFF;

  // PS2 logic
  // consider protection from switching directions too quickly
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
  
//  Serial.print("Left Stick Values: X = ");
//  Serial.print(analogLX, DEC); 
//  Serial.print(", Y = ");
//  Serial.print(analogLY, DEC);
//
//  Serial.print("   |    Speed: L = ");
//  Serial.print(valueL, DEC); 
//  Serial.print(", R = ");
//  Serial.print(valueR, DEC);
//  Serial.print("\n");
  
  esc_F.writeMicroseconds(valueF);
  esc_B.writeMicroseconds(valueB);
  esc_L.writeMicroseconds(valueL);
  esc_R.writeMicroseconds(valueR);

//  if(Serial.available()) {
//    value = Serial.parseInt();
//    Serial.print(value);
//    Serial.print("\n");
//  }
}
