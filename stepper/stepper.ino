#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

int led = 9; // LED connected to digital pin 13
int rec = 0; // byte received on the serial port
 
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2); 

void setup() {
  // initialize onboard LED (led), Powertail (pts) and serial port
  pinMode(led, OUTPUT);
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  myMotor->setSpeed(500);  // 10 rpm   
    // set the speed at 60 rpm:
  // initialize the serial port:
  Serial.begin(9600);
  
}
 
void loop()
{
  // if serial port is available, read incoming bytes
  if (Serial.available() > 0) {
    rec = Serial.read();
    
    // if 'y' (decimal 121) is received, turn LED on
    // anything other than 121 is received, turn LED off
    if (rec == 121){
    digitalWrite(led, HIGH);
    myMotor->step(200, FORWARD, DOUBLE);
   Serial.print("moving forward "); 
    }
    else
    {
     digitalWrite(led, LOW);
     myMotor->step(200, BACKWARD, DOUBLE);
      Serial.print("moving backward "); 
     }
      // step one revolution  in one direction:

/*
   Serial.println("Single coil steps");
  myMotor->step(100, FORWARD, SINGLE); 
  myMotor->step(100, BACKWARD, SINGLE); 

  Serial.println("Double coil steps");
  myMotor->step(100, FORWARD, DOUBLE); 
  myMotor->step(100, BACKWARD, DOUBLE);
  
  Serial.println("Interleave coil steps");
  myMotor->step(100, FORWARD, INTERLEAVE); 
  myMotor->step(100, BACKWARD, INTERLEAVE); 
  
  Serial.println("Microstep steps");
  myMotor->step(50, FORWARD, MICROSTEP); 
  myMotor->step(50, BACKWARD, MICROSTEP);
*/

     
    // confirm values received in serial monitor window
    Serial.print("--Arduino received: ");
    Serial.println(rec);
  }
}

