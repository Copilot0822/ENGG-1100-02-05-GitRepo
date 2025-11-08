class L298N{ //custom L298N motor driver object. 
  int ena;
  int in1;
  int in2;
  bool braking;
  public:
    L298N(int enPin, int in1Pin, int in2Pin, bool brake = true){
      ena = enPin;
      in1 = in1Pin;
      in2 = in2Pin;
      braking = brake;

    }
    void setSpeed(int speed = 0){
      if(speed == 0 && braking){
        digitalWrite(in1, HIGH);
        digitalWrite(in1, HIGH);
        digitalWrite(ena, LOW);
      }
      else if(speed == 0 && !braking){
        digitalWrite(in1, LOW);
        digitalWrite(in1, LOW);
        digitalWrite(ena, LOW);
      }
      else if(speed > 0){
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(ena, speed);
      }
      else if(speed < 0){
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(ena, speed*-1);
      }
    }
};

//pin assignment
int ena = 3;
int in1 = 4;
int in2 = 5;
L298N drive(ena, in1, in2, false);//object init, (enablePin, in1, in2, brakingMode)
void setup() {}
void loop() {
  drive.setSpeed(-255);//set speed to go forward, (speed), speed>0 for forward, speed<0 for backwards
  delay(10000);// 10s delay
  drive.setSpeed(255);//backwards command
  delay(10000);//10s delay
}