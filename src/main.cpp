#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define ONE_WIRE_BUS 14
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ตั้งค่าจอ Object OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void updateDisplay(double Input); // prototype ฟังก์ชันแสดงผล

#define SW1_PIN  34
#define SW2_PIN  35
#define SW3_PIN  32

#define RL1_PIN  17
#define RL2_PIN  16
#define RL3_PIN  4
 
#define ISOIN1_PIN 33
#define ISOIN2_PIN 27


void setup() {
  pinMode(RL1_PIN, OUTPUT);
  pinMode(RL2_PIN, OUTPUT);
  pinMode(RL3_PIN, OUTPUT);

  pinMode(SW1_PIN, INPUT);
  Serial.begin(9600);
  sensors.begin();

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // ถ้าจอเสียหรือถอดจอ ให้หยุดทำงานตรงนี้
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(10, 20);
  display.println(F("SYSTEM STARTING"));
  display.setCursor(10, 40);
  display.println(F("PID CONTROL"));
  display.display();
  delay(2000);

}

void loop() {
  sensors.requestTemperatures();
  double currentTemp = sensors.getTempCByIndex(0);
  Serial.print("Temp C: ");
  Serial.println(currentTemp);

  updateDisplay(currentTemp);

  int value = digitalRead(SW1_PIN);
  Serial.print("SW1=");
  Serial.println(value);
  
  if(value == LOW){
    digitalWrite(RL1_PIN,LOW);
    digitalWrite(RL2_PIN,LOW);
    digitalWrite(RL3_PIN,LOW);

  }else{
    digitalWrite(RL1_PIN,HIGH); 
    digitalWrite(RL2_PIN,HIGH);
    digitalWrite(RL3_PIN,HIGH); 
  }
  
  delay(500);
  
}


void updateDisplay(double Input) {
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0,0);
  display.print(F("TEMP.CONTROL"));

  display.setTextSize(2); // แสดง PV (ตัวใหญ่)
  display.setCursor(0, 14);
  display.print(F("PV:"));
  display.print(Input, 1); 

  // กราฟแท่งแสดงTemp(เต็มหน้าจอ ชิดขวา)
  int barHeight = map(Input, 0, 100, 0, 64);
  display.drawRect(118, 0, 10, 64, WHITE); // กรอบเต็มความสูง
  display.fillRect(118, 64 - barHeight, 10, barHeight, WHITE); //แท่งใน
  display.display();
}

