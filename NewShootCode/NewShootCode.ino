#include "classes.h"//includes the classes header file for all of the objects used

L298N launch(3,7,8);//creates the launcher motor controller, the numbers are the pins, 5:enable, 2:in1, 4:in2
L298N drive(5,2,4);//creates the drivetrain motor controller, the numbers are the pins, 5:enable, 2:in1, 4:in2

UltrasonicSensor sensor(11,10);//creates the ultrasonic 11:trigger, 10:echo

MotionMagic7 Pid(4, 0.1, 0, 0);//creates the S-curve PID object 4:kP, 0.1:kI, 0:kD, 0:kFF

void setup() {
  launch.init();//inits the launcher controller, eg the pinmode as this must be done in the setup, but I still need the global declaration
  sensor.init();//inits the ultrasonic sensor object
  drive.init();//inits the drivetrain controller, eg the pinmode as this must be done in the setup, but I still need the global declaration
  Pid.setConstraints(100, 100, 100);// sets the max vel, accel, jerk on the s curve
  Serial.begin(9600);//just serial init
  delay(1000);//a start up delay to allow for the operator to get away
  // put your setup code here, to run once:

}

//counters used in the main loop
int ShootStage = 0;//this is the counter that allows for a non blocking loop, this was mainly done when a switch was used for master safety enable
float time;//the prev time tracker units seconds
int countOut = 0;//this var is used to detect if the measurement has been within acceptable error for enough time, the units are in loops

const int shootingT = 0.1;//this is the target distance from sensor to wall on the run up to the wall, units: metres
const float shooterPower = 1.0f;//this is the const used for the power of the shooter motor [-1.0f, 1.0f]
const float shootingAcceptableError = 0.03;//this is the allowed error to move on to the next stage of the shooting stage





void loop() {
  float distance = sensor.Distance()/100.0f;// this is where the distance is tracked, the /100 for converting from cm to m.
  if(ShootStage == 0){//this is the first stage of the shooting mode, a delay section to allow the operator to get away
    delay(1000);
    ShootStage ++;//advances the stage to the next one
  }
  
  else if(ShootStage == 1){//this is the second/ initialization stage of the shooting mode
    time = millis()/1000.0f;//this is where the time is initially tracked for use in the next cycle
    Pid.setTarget(shootingT, distance);//this the the init part of the pid for calculating the s curve and setting the target.
    ShootStage++;//advance to the next stage
  }
  else if(ShootStage == 2){//this is the third/looping stage for the pid
    float output = max(-0.5f, min(0.5f,-Pid.update(distance, (millis()/1000.0f)-time)));// this is where the distance is fed in and the output is calculated using the pid object
    Serial.println(output);//output is printed to the terminal.
    time = millis()/1000.0f;//time is aquired for the next loop
    drive.setSpeed(output);//calculated output is sent to the drivetrain
    if(!(fabs(distance-shootingT)>shootingAcceptableError)){//it is determined if the error is small enough to continue
      countOut++;//count out is incremented for the next loop to see if the error has been small enough for long enough
    }
    if(countOut>10){//if we are within acceptable error for longer than 10 cycles/loops advance the shoot stage and set the drivetrain to zero output
      drive.setSpeed();
      ShootStage ++;
    }
  }
  else if(ShootStage == 3){//in the forth/ shoot stage
    launch.setSpeed(shooterPower);//set the output to the launcher
    ShootStage++;//go to the next stage
    delay(500);
  }
  else if(ShootStage == 4){//this helps prevents the launcher from continuing to drive into the hard stop for extended periods of time using uneccesary power and heating up the brushes in the motor
    launch.setSpeed();
  }

}
