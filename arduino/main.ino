#include <Servo.h>

Servo pinza;
Servo giraPinza

void setup() {
  Serial.begin(9600);
  pinza.attach(9);
  giraPinza.attach(10);

}

void loop() {
  while (Serial.available() > 0){
    char recived = Serial.read();
    
    
  }
}
