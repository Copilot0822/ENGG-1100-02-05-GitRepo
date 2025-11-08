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

int trigPin = 9;
int echoPin = 10;

UltrasonicSensor sensor(trigPin, echoPin);
void setup(){
  Serial.begin(9600);
}
void loop(){
  delay(500);
  Serial.println(sensor.Distance());
}


