#include <Arduino.h>

uint32_t data[3];


union u_tag{
  uint32_t number;
  uint8_t array[4];
}m_data;

void setup() {
  Serial.begin(9600);
  SerialUSB.begin(9600); 
  delay(3000);
  SerialUSB.println("Ready");
}

void loop() {
  // while(!Serial.available()){} // If there is no data do nothing 
  // We do something when we recieve somthing
  if (Serial.available()){
    if(Serial.read()=='\n'){
      for(uint8_t j=0;j<2;j++){
        for(uint8_t i=0;i<3;i++){
          m_data.array[i] = Serial.read();
        }
        data[j]=m_data.number;
      }   
    }
    SerialUSB.println("Recieved M1 = "+String(data[0])+" M2 = "+String(data[1])+" M3 = "+String(data[2]));
  }
}
