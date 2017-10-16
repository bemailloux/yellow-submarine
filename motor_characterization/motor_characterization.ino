#include <Servo.h>

int value;
Servo esc;

void setup() {
  // put your setup code here, to run once:
  value = 1864;
  esc.attach(9);
  Serial.begin(115200);
  Serial.print("Setup Complete\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  esc.writeMicroseconds(value);

  if(Serial.available()) {
    value = Serial.parseInt();
    Serial.print(value);
    Serial.print("\n");
  }
}
