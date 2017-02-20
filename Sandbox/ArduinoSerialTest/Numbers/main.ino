//Default baud speed for communication
#define BAUD 9600

String message;
int count = 0;

void setup(){
  Serial.begin(BAUD);
}

void loop(){
Serial.println(count++);
}