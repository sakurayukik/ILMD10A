#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Serial.println("init....");
  Wire.begin();
  delay(1000);
}

uint8_t temp = 0;
uint8_t adr = 0x10;
uint8_t adr2 = 0x11;

void loop() {
  init(adr,0x02);
  init(adr2,0x01);
  delay(500);

  while(1){
    drive(adr,0x00FF);
    drive(adr2,0x00BF);
    delay(800);
    
    drive(adr,0x017E);
    drive(adr2,0x00FF);
    delay(800);
    
    drive(adr,0x00FF);
    drive(adr2,0x013F);
    delay(800);
    
    drive(adr,0x007F);
    drive(adr2,0x00FF);
    delay(800);
    
  }
}

void drive(uint8_t slvAdr,uint16_t duty){
    uint8_t dutyH = ((duty&0xFF00)>>8);
    uint8_t dutyL = duty&0x00FF;

    Wire.beginTransmission(slvAdr);
    Wire.write(0x05);
    Wire.write(dutyH);
    Wire.write(dutyL);
    Wire.endTransmission();
}

void init(uint8_t slvAdr,uint16_t SR){
  uint8_t SRH = ((SR&0xFF00)>>8);
  uint8_t SRL = SR&0x00FF;

  Wire.beginTransmission(slvAdr);
  Wire.write(0x02);
  Wire.write(0x27);
  Wire.write(SRH);
  Wire.write(SRL);
  Wire.write(0x00);
  Wire.write(0xFF);
  Wire.write(0x01);
  Wire.write(0xFF);

  Wire.endTransmission();
}

