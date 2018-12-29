#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Serial.println("init....");
  Wire.begin();
  delay(1000);
}

uint8_t adr = 0x10; //Current slave address
uint8_t adrNew = 0x11; //New slave address

void loop() {
  Wire.beginTransmission(adr);
  Wire.write(0x09);
  Wire.write(adrNew);
  Wire.write(0xFF);
  Wire.endTransmission();
  delay(500);
  
  Wire.beginTransmission(adr);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
  
  Wire.requestFrom(adr, 9);
  while (Wire.available()) {
    uint8_t c = Wire.read();
    Serial.print("0x");
    Serial.print(c,HEX);
    Serial.print("\n");
  }
  Serial.print("New address is 0x");
  Serial.println(adrNew,HEX);
  while(1){
  }
}



