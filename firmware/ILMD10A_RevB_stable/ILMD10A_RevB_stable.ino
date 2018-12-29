#include <EEPROM.h>
#include <Wire.h>
#include <MsTimer2.h>

#define STBY 8
#define LED1 A0
#define LED2 A1
#define LED3 A2

#define ALERT1 7
#define ALERT2 6

#define WaI    ctrlReg[0x00]
#define STATUS ctrlReg[0x01]
#define MODE   ctrlReg[0x02]
#define SR_H   ctrlReg[0x03]
#define SR_L   ctrlReg[0x04]
#define DUTY_H ctrlReg[0x05]
#define DUTY_L ctrlReg[0x06]
#define TOP_H  ctrlReg[0x07]
#define TOP_L  ctrlReg[0x08]
#define ADR    ctrlReg[0x09]
#define WRITE  ctrlReg[0x0A]

uint8_t i2cBuf[16];
uint8_t i2cBufLen;

uint8_t ctrlRegAdr = 0x00;
uint8_t ctrlReg[16] = {0x1A,0x00,0x00,0x00 , 0x00,0x01,0xFF,0x03,
                       0xFF,0x10,0x00,0x00 , 0x01,0x00,0x00,0x00};

uint8_t abInv;
uint8_t srEn;
uint8_t driveMode;
uint16_t slewRate;
uint16_t pwmDuty;
uint16_t pwmDutyRef;
uint16_t pwmPeriod;

uint8_t wdtEn;
uint16_t wdtCnt = 0;


void setup() {
//LED_OUT
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

//ALERT_IN
  pinMode(ALERT1,INPUT_PULLUP);
  pinMode(ALERT2,INPUT_PULLUP);

//MD_OUTPUT
  pinMode(STBY, OUTPUT);
  analogWrite(9,1);
  analogWrite(10,1);
  TCCR1A = 0x02;//fast PWM
  TCCR1B = 0x19;//fast pwm max ICR1
  
//flash
  digitalWrite(LED1, HIGH);
  delay(80);
  digitalWrite(LED1, LOW);
  delay(80);
  digitalWrite(LED2, HIGH);
  delay(80);
  digitalWrite(LED2, LOW);
  delay(80);
  digitalWrite(LED3, HIGH);
  delay(80);
  digitalWrite(LED3, LOW);
  delay(200);

  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
  delay(200);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  delay(200);

//Serial
  Serial.begin(9600);
  Serial.println("init....");
//analog VR detect
  uint16_t adres4 = 511;
  uint16_t adcnt = 0;
  while((400 < adres4) && (adres4 < 600)){
    adres4 = analogRead(4);
    Serial.println(adres4,DEC);
    delay(5);
    adcnt++;
    if(adcnt > 10){
      Serial.println("Analog Mode!");
      analogMode();
    }
  }

//EEPROM_READ
  if(EEPROM.read(0x0A) != 0xFF){
    for(uint8_t forcnt=0x02;forcnt<0x0A;forcnt++){
      ctrlReg[forcnt] = EEPROM.read(forcnt);
    }
    WRITE = 0x00;
  }
  regUpdate();

//I2C
  Wire.begin(ADR & 0x7F);
  Wire.onRequest(i2cRequest);
  Wire.onReceive(i2cReceive);
  pinMode(SDA,INPUT);
  pinMode(SCL,INPUT);

//timer
  MsTimer2::set(10,tim10ms);
  MsTimer2::start();

//debug
  if(1){
    for(int forcnt=0;forcnt<16;forcnt++){
      Serial.print(ctrlReg[forcnt],HEX);
      Serial.print("\t");
    }
    Serial.print("\n");
  }
}

void alertDetect(void){
//alert
  switch(PIND&0xC0){
    case 0xC0://ALERT 1 & 2
      digitalWrite(LED2,LOW);
      digitalWrite(LED3,LOW);
      STATUS = (STATUS & 0xFC);
    break;
    case 0x80://ALERT1
      digitalWrite(LED2,LOW);
      digitalWrite(LED3,HIGH);
      STATUS = (STATUS & 0xFD);
      STATUS = (STATUS | 0x02);
    break;
    case 0x40://ALERT2
      digitalWrite(LED2,HIGH);
      digitalWrite(LED3,LOW);
      STATUS = (STATUS & 0xFE);
      STATUS = (STATUS | 0x01);
    break;
    default :
      digitalWrite(LED2,HIGH);
      digitalWrite(LED3,HIGH);
      
      STATUS = (STATUS | 0x03);
    break;
  }
}

void romWrite(void){
  for(uint8_t forcnt = 0x02;forcnt < 0x0A;forcnt++){
    EEPROM.write(forcnt,ctrlReg[forcnt]);
  }
  EEPROM.write(0x0A,0x55);
  WRITE = 0x00;
  MsTimer2::stop();
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,HIGH);
  digitalWrite(LED3,HIGH);
  while(1){}
}

void i2cRequest(void){
  for(ctrlRegAdr;ctrlRegAdr<= 0x0A;ctrlRegAdr++){
    Wire.write(ctrlReg[ctrlRegAdr]);
  }
}

void i2cReceive(int len){
  i2cBufLen = 0;
  for(uint8_t forcnt=0;forcnt<len;forcnt++){
    if(forcnt == 0){
      ctrlRegAdr = Wire.read();
    }
    else if(forcnt < 16){
      i2cBuf[forcnt-1] = Wire.read();
    }
    else{
      i2cBuf[15] = Wire.read();
    }
  }
  i2cBufLen = len - 1;
}

void regUpdate(){
//WDT
  wdtCnt = 0;
  
//SR
  slewRate = SR_H*256 + SR_L;
  
//DUTY
  pwmDutyRef = DUTY_H*256 + DUTY_L;

//PERIOD
  pwmPeriod = TOP_H*256 + TOP_L;
  if(ICR1 != pwmPeriod){
    ICR1 = pwmPeriod;//f = 8M / ICR1 [Hz](prescale = 1)
    pwmDuty = pwmDutyRef;
  }

//MODE
  if(MODE & 0x01){
    digitalWrite(STBY,HIGH);
  }
  else{
    digitalWrite(STBY ,LOW);
  }

  if(MODE & 0x02){
    //WDT EN
    wdtEn = 1;
  }
  else{
    //WDT DIS
    wdtEn = 0;
  }

  if(MODE & 0x04){
    //SR EN
    srEn = 1;
  }
  else{
    //SR DIS
    srEn = 0;
  }

  if(MODE & 0x08){
    //A INV. B nINV.
    abInv = 1;
  }
  else{
    //A nINV. B INV.
    abInv = 0;
  }

  switch(MODE & 0x30){
    case 0x10 :
      //On Break
      driveMode = 1;
    break;

    case 0x20 :
      //Lock Anti Phase
      driveMode = 2;
    break;

    case 0x30 :
      //Single Magnitude
      driveMode = 3;
    break;

    default :
      //port off
      driveMode = 0;
    break;
  }

  switch(MODE & 0xC0){
    case 0x40 :
    //prescale x8
      TCCR1B &= 0xF8;
      TCCR1B |= 0x02;
    break;
    
    case 0x80 :
    //prescale x64
      TCCR1B &= 0xF8;
      TCCR1B |= 0x03;
    break;

    case 0xC0:
    //prescale x256
    TCCR1B = 0x1A;
      TCCR1B &= 0xF8;
      TCCR1B |= 0x04;
    break;

    default:
    //prescale x1
      TCCR1B &= 0xF8;
      TCCR1B |= 0x01;
    break;
  }
}


void tim10ms(){
//ctrlReg reflesh
  if(i2cBufLen > 0){
    for(uint8_t forcnt=0;forcnt<i2cBufLen;forcnt++){
      if((forcnt + ctrlRegAdr) > 1){
        ctrlReg[forcnt+ctrlRegAdr] = i2cBuf[forcnt];
      }
    }
    i2cBufLen = 0;
    regUpdate();
  }

//WDT
  if((wdtEn == 1) && (wdtCnt > 100)){
    TCCR1A &= 0x0F;
    pwmDutyRef = ICR1 / 2;
    digitalWrite(STBY,LOW);
    digitalWrite(9,LOW);
    digitalWrite(10,LOW);
  }
  else{
    wdtCnt++;
  }


//SR
  if(srEn == 1){
    if(pwmDutyRef > pwmDuty){
      if(slewRate > pwmDutyRef - pwmDuty){
        pwmDuty = pwmDutyRef;
      }
      else{
        pwmDuty += slewRate;
      }
    }
    else if(pwmDutyRef < pwmDuty){
      if(slewRate > pwmDuty - pwmDutyRef){
        pwmDuty = pwmDutyRef;
      }
      else{
        pwmDuty -= slewRate;
      }
    }
  }
  else{
    pwmDuty = pwmDutyRef;
  }

//pwm OUT
  uint16_t center = ICR1 / 2;
  uint16_t abDuty;

  switch(driveMode){
    case 1:
    //On Break
      //A INV. B INV.
      TCCR1A &= 0x0F;
      TCCR1A |= 0xF0;
      if(pwmDuty > center){
        //foward
        abDuty = pwmDuty - center;
        abDuty *= 2;
        if(abInv == 1){
          OCR1A = abDuty;
          OCR1B = 0;
        }
        else{
          OCR1A = 0;
          OCR1B = abDuty;
        }
      }
      else if(pwmDuty < center){
        //rev
        abDuty = center - pwmDuty;
        abDuty *= 2;
        if(abInv == 1){
          OCR1A = 0;
          OCR1B = abDuty;
        }
        else{
          OCR1A = abDuty;
          OCR1B = 0;
        }
      }
      else{
        OCR1A = 0;
        OCR1B = 0;
      }
    break;

    case 2:
    //LockAntiPhase
      if(abInv == 1){
        //A INV. B nINV.
        TCCR1A &= 0x0F;
        TCCR1A |= 0xE0;
      }
      else{
        //A nINV. B INV.
        TCCR1A &= 0x0F;
        TCCR1A |= 0xB0;
      }
      OCR1A = pwmDuty;
      OCR1B = pwmDuty;
    break;

    case 3:
    //Single Magnitude(On-Free)
      //A nINV. B nINV.
      TCCR1A &= 0x0F;
      TCCR1A |= 0xA0;
      if(pwmDuty > center){
        //foward
        abDuty = pwmDuty - center;
        abDuty *= 2;
        if(abInv == 1){
          OCR1A = 0;
          OCR1B = abDuty;
        }
        else{
          OCR1A = abDuty;
          OCR1B = 0;
        }
      }
      else if(pwmDuty < center){
        //rev
        abDuty = center - pwmDuty;
        abDuty *= 2;
        if(abInv == 1){
          OCR1A = abDuty;
          OCR1B = 0;
        }
        else{
          OCR1A = 0;
          OCR1B = abDuty;
        }
      }
      else{
        OCR1A = 0;
        OCR1B = 0;
      }
    break;
    
    default:
      //A nINV. B nINV.
      TCCR1A &= 0x0F;
      TCCR1A |= 0xA0;
      OCR1A = 0;
      OCR1B = 0;
    break;
  }

//blink
  if(digitalRead(LED1)==HIGH){
    digitalWrite(LED1,LOW);
  }
  else{
    digitalWrite(LED1,HIGH);
  }

//EEPROM write
  if(WRITE == 0xFF){
    romWrite();
  }
}

void analogMode(void){
  uint16_t adres4;
  TCCR1A = 0xB2;//fast PWM
  ICR1 = 512;
  digitalWrite(STBY,HIGH);
  while(1){
    adres4 = analogRead(4);
    adres4 = adres4*0.5;
    OCR1A = adres4;
    OCR1B = adres4;
    Serial.println(adres4,DEC);
    if(digitalRead(LED2)==HIGH){
      digitalWrite(LED2,LOW);
      digitalWrite(LED3,HIGH);
    }
    else{
      digitalWrite(LED2,HIGH);
      digitalWrite(LED3,LOW);
    }
    delay(40);
  }
}


void loop() {
//alert
  alertDetect();
}

