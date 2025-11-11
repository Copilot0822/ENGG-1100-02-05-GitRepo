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

int DriveStage = 0;
int ShootStage = 0;
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

float time;

void loop() {

  if(!power.State()){
    Serial.print("distance: ");
    Serial.println(sensor.Distance());
    ShootStage = 0;
    DriveStage = 0; 
    delay(100);
  }
  else{
    if(mode.State()){
      //drive code
      ShootStage = 0;
      if(DriveStage ==0){
        delay(1000);
        DriveStage++;
      }
      else if(DriveStage ==1){
        //pid setup
        time = millis() / 1000.0f;
        Pid.setTarget(initDriveT, sensor.Distance());
        DriveStage++;
      }
      else if(DriveStage == 2){
        //pid loop
        float output = -Pid.update(sensor.Distance(), (millis()/1000.0f)-time);
        time = millis()/1000.0f;
        drive.setSpeed(output);
        if(!(fabs(sensor.Distance()-initDriveT)>acceptableError)){
          DriveStage++;
          drive.setSpeed();
        }
      }
      else if(DriveStage == 3){
        // pid2 init
        Pid.setTarget(returnDriveT, sensor.Distance());
        time = millis() / 1000.0f;
        DriveStage++;
      }
      else if(DriveStage == 4){
        // pid2 loop
        float output = -Pid.update(sensor.Distance(), (millis()/1000.0f)-time);
        time = millis()/1000.0f;
        drive.setSpeed(output);
        if(!(fabs(sensor.Distance()-returnDriveT)>acceptableError)){
          DriveStage++;
          drive.setSpeed();
        }
      }
      else{
        Serial.println("Done driving!");
      }
      

      
      // float time = millis() / 1000.0f;
      // Pid.setTarget(initDriveT, sensor.Distance());

      // while(fabs(sensor.Distance()-initDriveT)>acceptableError){
      //   float output = -Pid.update(sensor.Distance(), (millis()/1000.0f)-time);
      //   time = millis()/1000.0f;
      //   drive.setSpeed(output);
      // }
      // Pid.setTarget(returnDriveT, sensor.Distance());
      // time = millis() / 1000.0f;
      // while(fabs(sensor.Distance()-returnDriveT)>acceptableError){
      //   float output = -Pid.update(sensor.Distance(), (millis()/1000.0f)-time);
      //   time = millis()/1000.0f;
      //   drive.setSpeed(output);
      // }
      

    }
    else{
      DriveStage =0;
      if(ShootStage == 0){
        delay(1000);
        ShootStage ++;
      }
      
      else if(ShootStage == 1){
        time = millis()/1000.0f;
        Pid.setTarget(shootingT, sensor.Distance());
        ShootStage++;
      }
      else if(ShootStage == 2){
        float output = -Pid.update(sensor.Distance(), (millis()/1000.0f)-time);
        time = millis()/1000.0f;
        drive.setSpeed(output);
        if(!(fabs(sensor.Distance()-shootingT)>shootingAcceptableError)){
          ShootStage++;
          drive.setSpeed();
        }
      }
      else if(ShootStage == 3){
        launch.setSpeed(shooterPower);
        ShootStage++;
      }
      else if(ShootStage == 4){
        if(limit.State()){
          launch.setSpeed();
          ShootStage++;
        }
      }

      // delay(1000);
      // time = millis()/1000.0f;
      // Pid.setTarget(shootingT, sensor.Distance());

      // while(fabs(sensor.Distance()-shootingT)>shootingAcceptableError){
      //   float output = -Pid.update(sensor.Distance(), (millis()/1000.0f)-time);
      //   time = millis()/1000.0f;
      //   drive.setSpeed(output);
      // }
      // launch.setSpeed(shooterPower);
      // while(!limit.State()){
      //   continue;
      // }
      // launch.setSpeed();

      // Launch Code
    }
  }
}
