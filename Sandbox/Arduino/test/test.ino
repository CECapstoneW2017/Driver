#define BAUD 9600

void setup(){
  Serial.begin(BAUD);
}

void loop(){
  String input;
  String output;

  
  if(Serial.available() > 0){
    input = Serial.readStringUntil('\n');
    output = String("You sent the Arduino: " + input);
    Serial.println(output);
  }
}
