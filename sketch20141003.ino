// K型熱電対温度センサモジュールキット(SPI接続)MAX31855使用(5V版/3.3V版)サンプルスケッチ
// Example sketch for MAX31855 Type-K thermocouple sensor module (5V/3.3V) 
// Switch-Science 2013.5.14(Tue)
#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>

#define SLAVE 10
#define RelayPin 5
//LiquidCrystal(rs, enable, d4, d5, d6, d7) 
LiquidCrystal lcd(14, 15, 16, 17, 18, 19);
//PID
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);
int WindowSize = 500;
unsigned long windowStartTime, inter_time;

void setup() {
  //7ピン　5V設定
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH); 
  //6ピン　GND 設定
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW); 
  //4ピン　GND 設定
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW); 

  //2ピン　input pullup
  pinMode(2, INPUT_PULLUP);
  //3ピン　input pullup
  pinMode(3, INPUT_PULLUP);
  //タクトスイッチ　割り込み処理
  //L push 3ピン
  attachInterrupt(1, L_Pushed, FALLING);
  //R push 2ピン　
  attachInterrupt(0, R_Pushed, FALLING);

  Serial.begin(9600);
  //SPIのSS
  pinMode(SLAVE, OUTPUT);
  digitalWrite(SLAVE, HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setDataMode(SPI_MODE0);

  //ヒーターへPWM出力
  pinMode(RelayPin, OUTPUT);        
  digitalWrite(RelayPin, LOW);
  Setpoint = 200;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  lcd.begin(16, 2);
  lcd.clear();

}

void loop() {
  unsigned int thermocouple; // 14-Bit Thermocouple Temperature Data + 2-Bit
  unsigned int internal; // 12-Bit Internal Temperature Data + 4-Bit
  double disp; // display value 
  char buf[16];
  delay(1000);

  digitalWrite(SLAVE, LOW);                             //  Enable the chip
  thermocouple = (unsigned int)SPI.transfer(0x00) << 8;   //  Read high byte thermocouple
  thermocouple |= (unsigned int)SPI.transfer(0x00);       //  Read low byte thermocouple 
  internal = (unsigned int)SPI.transfer(0x00) << 8;       //  Read high byte internal
  internal |= (unsigned int)SPI.transfer(0x00);           //  Read low byte internal 
  digitalWrite(SLAVE, HIGH);                            //  Disable the chip

  if((thermocouple & 0x0001) != 0) {

    lcd.setCursor(0,0);
    if ((internal & 0x0004) !=0) {
      lcd.print("Short to Vcc");
    }
    if ((internal & 0x0002) !=0) {
      lcd.print("Short to GND");
    }
    if ((internal & 0x0001) !=0) {
      lcd.print("Open Circuit");
    }    
  } 
  else {
    if((thermocouple & 0x8000) == 0){ // 0℃以上   above 0 Degrees Celsius 
      disp = (thermocouple >> 2) * 0.25;
    } 
    else {   // 0℃未満   below zero
      disp = (0x3fff - (thermocouple >> 2) + 1)  * -0.25;
    }

    Serial.print(disp);
    Serial.println();
    //K温度をLCDに表示
    dtostrf(disp, 3, 0, buf);
    lcd.setCursor(0,0);
    lcd.print("P:");
    lcd.print(buf);
    lcd.print((char)0xdf);
    lcd.print('C');
    //設定温度をLCDに表示
    dtostrf(Setpoint, 3, 0, buf);
    lcd.print(" S:");
    lcd.print(buf);
    lcd.print((char)0xdf);
    lcd.print('C');
    //温度センサーの出力をPIDライブラリに渡す
    Input = disp; 
    myPID.Compute();            //PID計算の実行

    /************************************************
     * turn the output pin on/off based on pid output
     ************************************************/
    unsigned long now = millis();
    if ( now - windowStartTime > WindowSize ) { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    if ( Output > now - windowStartTime ) {
      digitalWrite( RelayPin, HIGH);
    } 
    else {
      digitalWrite( RelayPin, LOW);
    }

    //PWM
    //analogWrite(RelayPin, Output);      //analogWriteはPWM出力。計算された数値はOutputに入っている

  }

  if((internal & 0x8000) == 0){ // 0℃以上   above 0 Degrees Celsius
    disp = (internal >> 4) * 0.0625;
  } 
  else {                          // 0℃未満   below zero
    disp = (((0xffff - internal) >> 4) + 1)  * -0.0625;
  }

  // 室温温度をLCDに表示
  dtostrf(disp, 3, 0, buf);
  lcd.setCursor(0,1);
  lcd.print("R:");
  lcd.print(buf);
  lcd.print((char)0xdf);
  lcd.print('c');

}

void L_Pushed() {
  if ((millis() - inter_time) > 200 ) {
    Setpoint += 100;
    if (Setpoint > 400) {
      Setpoint -= 400;
    } 
    inter_time = millis();
  }
}

void R_Pushed() {
  if ((millis() - inter_time) > 200 ) {
    Setpoint += 10;
    if ((int)Setpoint % 100 == 0) {
      Setpoint -= 100;
    }
    inter_time = millis();
  }
}




