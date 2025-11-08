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

int pin = 2;
Switch result(pin);
void setup() {
  Serial.begin(9600);

}

void loop() {
 Serial.println(result.State());
 delay(500);

}
