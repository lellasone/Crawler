#include <Servo.h>

#define PIN_STEERING 5
#define PIN_RANGE 23 // the number of highest pin

#define ERROR_WRITEPIN       1
#define ERROR_READPIN        2

Servo servo_steering;

double steering_angle = 0;

extern byte error;

void setup() {
  servo_steering.attach(PIN_STEERING);
  Serial.begin(57600);
}

void loop() {
  // put your main code here, to run repeatedly:
  process_serial();
  write_steering();
  //pinMode(8, INPUT);
  //Serial.println(digitalRead(8));
}

void update_steering(byte angle){
  steering_angle = map(angle, 0, 255, -90, 90);
}

void write_steering(){
  
  servo_steering.write(map(steering_angle, -90, 90, 0, 180));
}

/* 
 *  This function can be used to change the value of an arbitrary digital IO pin. 
 *  It's pretty slow because it re-configures the pin each time it's called, but
 *  is a good fit for controlling relays or the like over serial. 
 *  args: 
 *    pin - the arduino number of the pin in question. 
 *    value - if 0, write the pin low. Otherwise write it high. 
 */
void write_pin_digital(byte pin, byte value){
  pinMode(pin, OUTPUT);
    error = 0;
    if(value == 0x00){
      digitalWrite(pin, LOW);
    } else {
      digitalWrite(pin, HIGH);
    }
  /*if ((pin < 0) || (pin > PIN_RANGE)){
    error = ERROR_WRITEPIN;
  }
  else {
    pinMode(pin, OUTPUT);
    error = value;
    if(value == 0x00){
      digitalWrite(pin, LOW);
    } else {
      digitalWrite(pin, HIGH);
    }
  }*/
}

byte read_pin_digital(int pin){
  if ((pin < 0) || (pin > PIN_RANGE)){
    error = ERROR_READPIN;
    return(0);
  } else {
    pinMode(pin, INPUT_PULLDOWN);
    byte value = digitalRead(pin);
    error = 0;
    return value;
  }
}
