#include <Servo.h>
Servo ServoA;
Servo ServoB;
Servo ServoC;

// Switch
const int switchPin = 13;

// Distance Sensor
const int trigPin = 7;           //connects to the echo pin on the distance sensor       
const int echoPin = 8;           //connects to the trigger pin on the distance sensor     
int echoTime = 0;
unsigned long avgTime = 0;


float distance = 0;               //stores the distance measured by the distance sensor

// Motors
int motorSpeed = 250;
//const int RLDamp = 0.8;
//const int AngularDamp = 0.75;
const int RLDamp = 0.9;
const int AngularDamp = 0.85;

const int ServoAPin = 9;
const int ServoBPin = 10;
const int ServoCPin = 11;

// Shift Registers
int ioSelect = 2;     // SR Pin 15.
int clockPulse = 3;   //SR Pin 7. 
int dataOut = 4;      //SR Pin 13.

int j;               //used in a for loop to declare which bit is set to 1
int value;           //stores the digital read value of the data pin 
                     //(0 if no button is pressed, 1 if a button is pressed)

byte switchVar = 0;  //stores a byte array to show which button was pressed


void setup() {
  pinMode(ioSelect, OUTPUT);
  pinMode(clockPulse, OUTPUT);
  pinMode(dataOut, INPUT);

  pinMode(switchPin, INPUT_PULLUP);   //set this as a pullup to sense whether the switch is flipped

  pinMode(trigPin, OUTPUT);   //the trigger pin will output pulses of electricity 
  pinMode(echoPin, INPUT);    //the echo pin will measure the duration of pulses coming back from the distance sensor

  ServoA.attach(ServoAPin);
  ServoB.attach(ServoBPin);
  ServoC.attach(ServoCPin);

  ServoA.writeMicroseconds(1500);
  ServoB.writeMicroseconds(1500);
  ServoC.writeMicroseconds(1500);

  delay(5000);
  push(0b1);
  delay(5000);
}



void loop() {
  if(digitalRead(switchPin) == HIGH){
    uint16_t  dataIn = 0;       //Swap out byte for uint16_t or uint32_t
    setPins();
    uint16_t number = checkBumpers(dataIn);
    push(number);
  }
  
  else{
    avgTime = ceil(averageTime(5));
    while(avgTime < 2500 && avgTime > 0){
      push(0b11000000000);
      delay(1000);
      avgTime = ceil(averageTime(5));
    }
    motorStop();
  }
  
}

void setPins(){
  digitalWrite(ioSelect, 0);    // enables parallel inputs
  digitalWrite(clockPulse, 0);  // start clock pin low
  digitalWrite(clockPulse, 1);  // set clock pin high, data loaded into SR
  digitalWrite(ioSelect, 1);    // disable parallel inputs and enable serial output 

}

uint16_t checkBumpers(uint16_t dataIn){
  for(j = 0; j < 16; j++) {         //sets integer to values 0 through 7 for all 8 bits
    value = digitalRead(dataOut); //reads data from SR serial data out pin

    if (value) {
      int a = (1 << j);       // shifts bit to its proper place in sequence. 
      dataIn = dataIn | a;    //combines data from shifted bits to form a single 8-bit number
      }
      digitalWrite(clockPulse, LOW);  //after each bit is logged, 
      digitalWrite(clockPulse, HIGH); //pulses clock to get next bit
    }
  
     if (switchVar != dataIn)
    {
      switchVar = dataIn;
    } 
  return dataIn;
}


void motorStop(){
  ServoA.writeMicroseconds(1500);
  ServoB.writeMicroseconds(1500);
  ServoC.writeMicroseconds(1500);
}

void push(uint16_t n){
  switch(n){
    case 0b0:
      ServoA.writeMicroseconds(1500);
      ServoB.writeMicroseconds(1500);
      ServoC.writeMicroseconds(1500);
      break;
      
    case 0b1:
      ServoA.writeMicroseconds(1500 - (motorSpeed + motorSpeed));
      ServoB.writeMicroseconds(1500 + 0);
      ServoC.writeMicroseconds(1500 + (motorSpeed + motorSpeed));
      break;
      
    case 0b1000:
      ServoA.writeMicroseconds(1500 - (motorSpeed + floor(motorSpeed*RLDamp)));
      ServoB.writeMicroseconds(1500 + (motorSpeed + motorSpeed));
      ServoC.writeMicroseconds(1500 - (motorSpeed + floor(motorSpeed*RLDamp)));
      break;
      
    case 0b100000:
      ServoA.writeMicroseconds(1500 + (motorSpeed + floor(motorSpeed*RLDamp)));
      ServoB.writeMicroseconds(1500 - (motorSpeed + motorSpeed));
      ServoC.writeMicroseconds(1500 + (motorSpeed + floor(motorSpeed*RLDamp)));
      break;
      
    case 0b11000000000:
      ServoA.writeMicroseconds(1500 + (motorSpeed + motorSpeed));
      ServoB.writeMicroseconds(1500 + 0);
      ServoC.writeMicroseconds(1500 - (motorSpeed + motorSpeed));
      break;


    case 0b100:
      ServoA.writeMicroseconds(1500 - (motorSpeed + motorSpeed));
      ServoB.writeMicroseconds(1500 + (motorSpeed + motorSpeed));
      ServoC.writeMicroseconds(1500 + 0);
      break;
  
     case 0b10:
      ServoA.writeMicroseconds(1500 - (motorSpeed + floor(motorSpeed*(1-AngularDamp))));
      ServoB.writeMicroseconds(1500 + (motorSpeed + floor(motorSpeed*RLDamp)));
      ServoC.writeMicroseconds(1500 + (motorSpeed + floor(motorSpeed*AngularDamp)));
      break;

    case 0b10000000:
      ServoA.writeMicroseconds(1500 - (motorSpeed + floor(motorSpeed*RLDamp)));
      ServoB.writeMicroseconds(1500 - (motorSpeed + floor(motorSpeed*AngularDamp)));
      ServoC.writeMicroseconds(1500 + (motorSpeed + floor(motorSpeed*(1-AngularDamp))));
      break;

    case 0b1000000:
      ServoA.writeMicroseconds(1500 + 0);
      ServoB.writeMicroseconds(1500 - (motorSpeed + motorSpeed));
      ServoC.writeMicroseconds(1500 + (motorSpeed + motorSpeed));
      break;

    case 0b100000000000:
      ServoA.writeMicroseconds(1500 + (motorSpeed + motorSpeed));
      ServoB.writeMicroseconds(1500 - (motorSpeed + motorSpeed));
      ServoC.writeMicroseconds(1500 + 0);
      break;

    case 0b10000000000:
      ServoA.writeMicroseconds(1500 + (motorSpeed + floor(motorSpeed*(1-AngularDamp))));
      ServoB.writeMicroseconds(1500 - (motorSpeed + floor(motorSpeed*RLDamp)));
      ServoC.writeMicroseconds(1500 - (motorSpeed + floor(motorSpeed*AngularDamp)));
      break;

    case 0b1000000000:
      ServoA.writeMicroseconds(1500 + (motorSpeed + floor(motorSpeed*RLDamp)));
      ServoB.writeMicroseconds(1500 + (motorSpeed + floor(motorSpeed*AngularDamp)));
      ServoC.writeMicroseconds(1500 - (motorSpeed + floor(motorSpeed*(1-AngularDamp))));
      break;

    case 0b100000000:
      ServoA.writeMicroseconds(1500 + 0);
      ServoB.writeMicroseconds(1500 + (motorSpeed + motorSpeed));
      ServoC.writeMicroseconds(1500 - (motorSpeed + motorSpeed));
      break;
    
    default:
      ServoA.writeMicroseconds(1500 + 2*motorSpeed);
      ServoB.writeMicroseconds(1500 + 2*motorSpeed);
      ServoC.writeMicroseconds(1500 + 2*motorSpeed);
      break;    
  }
}


//RETURNS THE TIME MEASURED BY THE HC-SR04 DISTANCE SENSOR
unsigned long getTime()
{
  unsigned long echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  
  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH, 60000);      //use the pulsein command to see how long it takes for the
                                          //pulse to bounce back to the sensor
  return echoTime;              //send back the distance that was calculated
}


unsigned long averageTime(int n){
  unsigned long avg = 0;
  unsigned long measure = 0;
  int counter = n;
  for(int i=0; i<n; i++){
    measure = getTime();
    if(measure <= 0 || measure > 20000){
      counter--;
      measure = 0;
    }
    avg += measure;
  }
  if(counter == 0 || counter <= 2 || avg == 0) return 10000;
  return avg;
}
