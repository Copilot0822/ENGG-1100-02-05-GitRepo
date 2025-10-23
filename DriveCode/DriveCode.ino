class L298N{
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

int ena = 3;
int in1 = 5;
int in2 = 6;


L298N drive(ena, in1, in2, false);


void setup() {
  // L298N drive(ena, in1, in2, false);
  // put your setup code here, to run once:

}

void loop() {
  // delay(1000);
  drive.setSpeed(150);
  delay(10000);
  drive.setSpeed(-150);
  delay(10000);
  // digitalWrite(13, HIGH);
  // delay(1000);
  // drive.setSpeed(-150);
  // digitalWrite(13, LOW);
  // put your main code here, to run repeatedly:

}
