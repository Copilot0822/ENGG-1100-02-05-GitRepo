#include <math.h>
#include "classes.h"

Switch limit(2);
Switch mode(4);
Switch power(6);

UltrasonicSensor sensor(10, 11);

L298N drive(3, 7, 8);
L298N launch(5, 12, 13);

MotionMagic7 Pid(0.1, 0, 0, 0);

// Pid.setConstraints(100,100,100);


void setup() {
  Pid.setConstraints(100, 100, 100);
  drive.init();
  launch.init();
  sensor.init();

  limit.init();
  mode.init();
  power.init();

  Serial.begin(9600);
  // put your setup code here, to run once:

}

const float initDriveT = 12;

const float returnDriveT = 40;

const float acceptableError = 3;


const float shootingT = 12;

const float shootingAcceptableError = 2;

const float shooterPower = 0.5;


void loop() {

  if(!power.State()){
    Serial.print("distance: ");
    Serial.println(sensor.Distance());
    delay(100);
  }
  if(mode.State()){
    //drive code
    delay(1000);
    float time = millis() / 1000.0f;
    Pid.setTarget(initDriveT, sensor.Distance());

    while(fabs(sensor.Distance()-initDriveT)>acceptableError){
      float output = -Pid.update(sensor.Distance(), (millis()/1000.0f)-time);
      time = millis()/1000.0f;
      drive.setSpeed(output);
    }
    Pid.setTarget(returnDriveT, sensor.Distance());
    time = millis() / 1000.0f;
    while(fabs(sensor.Distance()-returnDriveT)>acceptableError){
      float output = -Pid.update(sensor.Distance(), (millis()/1000.0f)-time);
      time = millis()/1000.0f;
      drive.setSpeed(output);
    }
    

  }
  else{
    delay(1000);
    float time = millis()/1000.0f;
    Pid.setTarget(shootingT, sensor.Distance());

    while(fabs(sensor.Distance()-shootingT)>shootingAcceptableError){
      float output = -Pid.update(sensor.Distance(), (millis()/1000.0f)-time);
      time = millis()/1000.0f;
      drive.setSpeed(output);
    }
    launch.setSpeed(shooterPower);
    while(!limit.State()){
      continue;
    }
    launch.setSpeed();

    // Launch Code
  }
  return;
}
