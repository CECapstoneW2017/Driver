//Default baud speed for communication
#define BAUD 9600
//led 
#define led 13
//macro for on/off
#define on (digitalWrite(led, HIGH))
#define off (digitalWrite(led, LOW))

void setup(){
  Serial.begin(BAUD, SERIAL_8E1);
  pinMode(led, OUTPUT);
}

void loop(){
  String input;
  //If any input is detected in arduino
  if(Serial.available() > 0){
    input = Serial.readStringUntil('\n');
    //If input == "ON" then turn on the led 
    //and send a reply
    for(char i = 0; i < 50; i++){
    if (input.equals("start")){
      //generate random angles for testing
      int thetaX = random(-31,31);
      int thetaY = random(-31,31);
      //send pos. values to computer
      Serial.println(sprintf("thetaX: %l, thetaY: %l\n", thetaX, thetaY));
      delay(200); //delay 200 ms
    }
    }
  }
}
