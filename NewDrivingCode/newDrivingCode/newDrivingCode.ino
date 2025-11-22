/*
  This code was created and documented by Benjamin McRae (1379397)
  between September 28th, 2025 and November 20th, 2025
  
*/



#include "classes.h"//includes the classes header file for all of the objects used

L298N drive(5,2,4);//creates the drivetrain motor controller, the numbers are the pins, 5:enable, 2:in1, 4:in2

UltrasonicSensor sensor(11,10);//creates the ultrasonic 11:trigger, 10:echo

MotionMagic7 Pid(4, 0.1, 0, 0);//creates the S-curve PID object 4:kP, 0.1:kI, 0:kD, 0:kFF

void setup() {
  drive.init();//inits the drivetrain controller, eg the pinmode as this must be done in the setup, but I still need the global declaration
  sensor.init();//inits the ultrasonic sensor object
  Pid.setConstraints(100, 100, 100);// sets the max vel, accel, jerk on the s curve
  Serial.begin(9600);//just serial init
}

//These are the constants for the main loop
const float initDriveT = 0.25;//the initial target of the drive, eg distance to wall units: Metres
const float returnDriveT = 11.87;//the return distance target, :DEPRECATED
const float acceptableError = 0.05;//the acceptable error is the error(distance to target) acceptable range eg where the closed loop circuit releases unit: Metres

//These are some counters used in the main loop
float time;//this variable is used to track the time, each loop this is set to the current system time units: Second
int countOut = 0;//this var is used to detect if the measurement has been within acceptable error for enough time, the units are in loops
int DriveStage = 0; // this is the counter that allows a non blocking loop. Its just a counter to detect what part of the linear system it the drivetrain should be in


void loop() {
  float distance = sensor.Distance()/100.0f;// this is where the distance is tracked, the /100 for converting from cm to m.
  if(DriveStage ==0){// this is the first stage of the drive mode
        delay(1000);//just adds a delay at the beginning to allow the operator to get away
        DriveStage++;//increments drive stage to go to the next stage
      }
      else if(DriveStage ==1){//second stage is for pid initialization and target selection. This is also where the S-curve part is calculated
        time = millis() / 1000.0f;//time is tracked from system time in seconds, used to calc dt later
        Pid.setTarget(initDriveT, distance);// this is where the pid is told the desired target and where the pid calculates the s curve
        DriveStage++;// go to the next stage
        countOut = 0;// sets the countout to zero to show that its not in target
      }
      
      else if(DriveStage == 2){//third stage for main drive towards loop
        float output = max(-0.8f, min(-Pid.update(distance, (millis()/1000.0f)-time), 0.8f));//where the pid is given the current position and dt

        time = millis()/1000.0f;// prev time is set for the next loop
        Serial.print(output);// prints output to monitor for use
        Serial.print(" h ");// just a break
        Serial.println(distance);//disstance is printed to serial
        drive.setSpeed(output);// sets the calculated output to the drivetrain
        if(!(fabs(distance-initDriveT)>acceptableError)){//checks if inside of the acceptable error
          countOut ++;
        }
        if(countOut > 10){
          DriveStage++;//if we are in target for enough cycles go to next stage
          drive.setSpeed();//set drivetrain to zero output
        }

      }
      else if(DriveStage == 3){
        // pid2 init
        countOut = 0;
        Pid.setTarget(returnDriveT, distance);
        time = millis() / 1000.0f;
        delay(600);
        DriveStage++;
        //this is just setting the pid for the return run
      }
      else if(DriveStage == 4){
        // pid2 loop
        // float output = -Pid.update(distance, (millis()/1000.0f)-time);
        // float output = max(-0.5f, min(-Pid.update(distance, (millis()/1000.0f)-time), 0.5f));
        float output = -0.7;// this was done because the ultrasonic was found to be extremely un reliable in the atrium do to noise/interference
        time = millis()/1000.0f;// time for the pid but not used
        drive.setSpeed(output);
        // if(!(fabs(distance-returnDriveT)>acceptableError)){
        //   // DriveStage++;
        //   // drive.setSpeed();
        //   countOut++;
        // }
        if(countOut >10){
          DriveStage++;
          drive.setSpeed();
        }

      }
      else{
        Serial.println("Done driving!");
      }

}
