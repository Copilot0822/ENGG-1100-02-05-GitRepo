class Switch{
  int pin;
  bool inverted;

  public:
    Switch(int Pin, bool Invert = false){
      pin = Pin;
      inverted = Invert;
      pinMode(pin, INPUT_PULLUP);
    }
    bool State(){
      if(inverted){
        return digitalRead(pin) == LOW;
      }
      else{
        return digitalRead(pin) == HIGH;
      }
    }
};
class UltrasonicSensor{
  int trigPin;
  int echoPin;
  long duration;
  int distance;
  
  public:
    UltrasonicSensor(int trigpin, int echopin){
      trigPin = trigpin;
      echoPin = echopin;
      pinMode(echoPin, INPUT);
      pinMode(trigPin, OUTPUT);
    }
    int Distance(){ 
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin,  HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration  = pulseIn(echoPin, HIGH);
      distance= duration*0.034/2;
      return distance;
    }

};
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
    void setSpeed(double speed = 0){
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
        analogWrite(ena, speed*255);
      }
      else if(speed < 0){
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(ena, speed*-255);
      }
    }
};

Switch limit(2);
Switch mode(4);

UltrasonicSensor sensor(10, 11);

L298N drive(3, 7, 8);
L298N launch(5, 12, 13);





void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
