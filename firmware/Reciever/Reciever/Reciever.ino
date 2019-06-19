/* This is a simple sketch for pulling data from an
 *  RC reciever's PPM lines. The data is then 
 *  sent over Serial to some other device. 
 *  
 *  Author: Jake Ketchum (jketchum@caltech.edu)
 *  Last Updated: 6/18/2019
 */


#define NUM_PINS 4
#define PIN_INTERUPT 3

#define TIME_MIN 1000 //shortest pulse duration. 
#define TIME_MAX 2000 //longest pulse duration.


// Stores the most recent value recieved for each channel. 
// Modify this to change the default values (between -180 and 180)
int channels[] = {0,0,0,0,0,0,0,0,0};

// Contains an entry for each input channel in the form {pin, last duration, min, max}.
int pin_values[][2] = {{2,0},
                      {3,0},
                      {4,0},
                      {5,0},
                      {A7,0},
                      {A6,0},
                      {A5,0},
                      {A4,0}};

unsigned long  start = 0;
unsigned long  finish = 0;
bool flag = false;

int max_stear = 30; // default max value by which a channel can be changed to acomplish
                    // a turn. Note that this is half the total spread (plus on one 
                    // minus on the other). 

void setup() {
  
  // set up all reading pins. 
  for(int i=0; i<NUM_PINS;i++){
    pinMode(pin_values[i][0], INPUT_PULLUP);
  }
  // Set up the interrupt. 
  attachInterrupt(digitalPinToInterrupt(PIN_INTERUPT), interrupt, RISING);

  
  Serial.begin(115200);
  Serial.println("Ready to Begin");
}

void loop() {
  if(flag){
    //int start = micros();
    read_and_send();
    
    for (int i = 0; i < 7; i++){
      Serial.print(channels[i]);
      Serial.print(",");
    }
    Serial.println("");
  }

} 




void interrupt(){
  start = micros();
  finish = start + 2000 * NUM_PINS;
  flag = true;
  
}

/*
 * This function handles the data input for the arduino from the reciever. 
 * It functions as follows: After channel 1 goes high for it's pulse, this 
 * function is called. 
 * 
 * Then for each of the channels, it waits half the garenteed on time, before
 * looping until the pin goes low. The delay is then recorded, and the next
 * pin measuremnt cycle begins. 
 */
void read_and_send(){
  for(int i=0; i<NUM_PINS; i++){
  while(micros() - start < 500){
      // wait half the garenteed on time to ensure pulse has started.
  }
    if(pin_values[i][0] >=20 && pin_values[i][0] < 28){
      // handle analog pins. These will be slower.
      while(analogRead(pin_values[i][0]) > 50 && micros() < finish){
      //wait until the pin has gone low
    }
    } else {
      while(digitalRead(pin_values[i][0]) && micros() < finish){
      //wait until the pin has gone low
    }
    }
    
    
    int current = micros() - start;
    start = micros(); // begin recording for next in. 
    
    // record how long that pin was high for. 
    pin_values[i][1] = current;
    channels[i] = map(current, TIME_MAX, TIME_MIN, 0, 180);
    
  }
  flag = false;
}

