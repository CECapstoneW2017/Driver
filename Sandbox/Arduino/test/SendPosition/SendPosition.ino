//Default baud speed for communication
#define BAUD 256000
//led 
#define led 13
#define xPot 0
#define yPot 1 

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
  int xPotVal = 0;
  int yPotVal = 0;
  int xAngle = 0;
  int yAngle = 0;
  //if(Serial.available() > 0){
    
    //Serial.println("Write \"start\":");
    //input = Serial.readStringUntil('\n');
    //If input == "start" start sending random coordinates to the computer
    //if (input.equals("start")){
      //for(char i = 0; i < 100; i++){
        //generate random angles for testing
        //int thetaX = 42;
        //int thetaY = -43;
        
        //read potentiometer value and send to computer 
        xPotVal = analogRead(xPot);
        yPotVal = analogRead(yPot);
        xAngle = (xPotVal - 512)*.015; //(x ticks - 512 ticks) * (15.24 cm /1024 ticks) = x cm
        yAngle = (yPotVal - 512)*.015;
        //send pos. values to computer
        char out_str[255];
        //sprintf(out_str, "%i,%i", thetaX, thetaY);
        sprintf(out_str, "%i,%i", xAngle, yAngle);
        Serial.println(out_str);
        delay(1); //delay 200 ms
      //}
    //}
  //}
}
