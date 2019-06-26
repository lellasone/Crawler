#include <Servo.h>

#define PIN_STEERING 5

Servo servo_steering;

double steering_angle = 0;

void setup() {
  servo_steering.attach(PIN_STEERING);
  Serial.begin(57600);
}

void loop() {
  // put your main code here, to run repeatedly:
  process_serial();
  write_steering();
}

void update_steering(byte angle){
  steering_angle = map(angle, 0, 255, -90, 90);
}

void write_steering(){
  
  servo_steering.write(map(steering_angle, -90, 90, 0, 180));
}

