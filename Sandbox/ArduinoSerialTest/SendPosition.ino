//Default baud speed for communication
#define BAUD 256000
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
    //Serial.println("Write \"start\":");
    input = Serial.readStringUntil('\n');
    //If input == "start" start sending random coordinates to the computer
    if (input.equals("start")){
      //for(char i = 0; i < 50; i++){
        //generate random angles for testing
        int thetaX = 42;
        int thetaY = -43;
        //send pos. values to computer
        char out_str[255];
        sprintf(out_str, "%i,%i", thetaX, thetaY);
        Serial.println(out_str);
        //delay(200); //delay 200 ms
      //}
    }
  }
}
