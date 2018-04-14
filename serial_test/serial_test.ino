#include <SoftwareSerial.h>
#define pi_tx 8;
#define pi_rx 7;

byte number = 0;

// Verbinde GPIO 14 mit Pin 7 und GPIO 15 mit Pin 8 über ein Voltage Level Converter
SoftwareSerial piS(7,8);

void setup(){
Serial.begin(115200);
delay(500);
piS.begin(115200);
delay(500);
}

void loop(){
//   if(piS.available()){
//    Serial.write(piS.read());
//  }
  if(Serial.available()){    
    piS.write(Serial.read());
  }
//  if (Serial.available()) {
//    number = Serial.read();
//    Serial.write(number);
//  }
}
