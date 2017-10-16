#include <Servo.h>

// calibration sketch for ESCs
// calibration procedure:
// 1. set PWM value for all ESCs to 1864; ESCs should play start-up beeps, then an extra beep to indicate calibration has started
// 2. set PWM value for all ESCs to 1065; ESC shoulb beep twice to indicate the value has been received
// 3. set PWM value to 1464; ESCs should play a longer beep to indicate that calibration was successful and ESCs are armed
// ESCs can now be used normally

// pin numbers for ESCs (change as desired)
#define ESC1 8
#define ESC2 9
#define ESC3 10
#define ESC4 11

int value;
Servo esc1, esc2, esc3, esc4;

void setup() {
  // put your setup code here, to run once:
  esc1.attach(ESC1);
  esc2.attach(ESC2);
  esc3.attach(ESC3);
  esc4.attach(ESC4);
  Serial.begin(115200);

  // write high value to ESCs
  value = 1864;
  delay(1000);
  esc1.writeMicroseconds(value);
  esc2.writeMicroseconds(value);
  esc3.writeMicroseconds(value);
  esc4.writeMicroseconds(value);

  // write low value to ESCs
  value = 1064;
  delay(1000);
  esc1.writeMicroseconds(value);
  esc2.writeMicroseconds(value);
  esc3.writeMicroseconds(value);
  esc4.writeMicroseconds(value);

  // write mid value to ESCs
  value = 1464;
  delay(1000);
  esc1.writeMicroseconds(value);
  esc2.writeMicroseconds(value);
  esc3.writeMicroseconds(value);
  esc4.writeMicroseconds(value);

  delay(1000);
  // ESCs should be armed
  
  Serial.print("Calibration Complete\n");
}

void loop() {
  // test different values here using the serial window
  esc1.writeMicroseconds(value);
  esc2.writeMicroseconds(value);
  esc3.writeMicroseconds(value);
  esc4.writeMicroseconds(value);

  if(Serial.available()) {
    value = Serial.parseInt();
    Serial.print(value);
    Serial.print("\n");
  }
}
