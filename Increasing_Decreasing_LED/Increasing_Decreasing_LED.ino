

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(11, OUTPUT);


  // put your setup code here, to run once:

}

void loop() {
  for(int i = 0;i<255;i++){
    delay(20);
    analogWrite(11, i);
  }
  for(int i = 255; i > 0; i--){
    delay(20);
    analogWrite(11, i);
  }
  analogWrite(11, 0);
  
  
  // put your main code here, to run repeatedly:

}
