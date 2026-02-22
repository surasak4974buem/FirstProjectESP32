/**
 * ESP32 PID Temperature Controller (Professional Edition)
 * ---------------------------------------------------
 * รายละเอียด Hardware:
 * - MCU: ESP32 DevKit V2 Board (THAITECHZONE)
 * - Sensor: DS18B20 (ต่อที่ GPIO 14)
 * - Heater: MOSFET หรือ SSR (ต่อที่ GPIO 13) -> ควบคุมด้วย PWM
 * - Display: OLED 0.96" (I2C: SDA=21, SCL=22)
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>

// =========================================
// 1. การกำหนดขาพอร์ต (Pin Definitions)
// =========================================
#define HEATER_PIN 13    // ขาจ่ายสัญญาณ PWM ไปยัง Heater Driver
#define ONE_WIRE_BUS 14  // ขา Data ของ Sensor DS18B20

// Manual Control IO (สำรองไว้ใช้ในอนาคต)
#define SW1_PIN 34       // สวิตช์ 1 (Input Only - ต้องต่อ R Pull-up 10k)
#define SW2_PIN 35       // สวิตช์ 2 (Input Only - ต้องต่อ R Pull-up 10k)
#define SW3_PIN 32       // สวิตช์ 3 (มี Internal Pull-up)
#define RL1_PIN 17       // รีเลย์ 1
#define RL2_PIN 16       // รีเลย์ 2
#define RL3_PIN 4        // รีเลย์ 3

// Isolated Inputs (สำรองไว้ใช้ในอนาคต)
#define ISOIN1_PIN 33
#define ISOIN2_PIN 27

// =========================================
// 2. การตั้งค่าระบบ (System Settings)
// =========================================
// ขอบเขตอุณหภูมิและความปลอดภัย
#define TEMP_MIN 25.0    // ค่าต่ำสุดที่ยอมให้ตั้ง
#define TEMP_MAX 100.0    // ค่าสูงสุด และจุดตัด Safety Cutoff

// การตั้งค่า PWM (สำหรับ ESP32)
const int PWM_FREQ = 1000;     // ความถี่ 1kHz (เหมาะกับ MOSFET)
const int PWM_CHANNEL = 0;     // ช่องสัญญาณ PWM 0
const int PWM_RESOLUTION = 8;  // ความละเอียด 8-bit (ค่า 0-255)

// =========================================
// 3. ประกาศตัวแปรและ Object
// =========================================
// ตั้งค่าจอ OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ตั้งค่า Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ตัวแปร PID
double Setpoint, Input, Output;
// ค่า Tuning (ปรับจูนตามความเหมาะสมของระบบจริง)
double Kp = 10.0, Ki = 0.5, Kd = 1.0; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ตัวแปรจับเวลา
unsigned long lastDisplayTime = 0;

// ประกาศชื่อฟังก์ชันล่วงหน้า (Function Prototypes)
void updateDisplay();
void displayError(String title, String msg);
void debugSerial();

void setup() {
  Serial.begin(115200);
  
  // --- A. ตั้งค่า PWM สำหรับ Heater ---
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(HEATER_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0); // เริ่มต้นปิด Heater

  // --- B. เริ่มต้น Sensor ---
  sensors.begin();

  // --- C. เริ่มต้นจอ OLED ---
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // ถ้าจอเสีย ให้หยุดทำงานตรงนี้
  }
  
  // แสดง Logo เริ่มต้น
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(10, 20);
  display.println(F("SYSTEM STARTING..."));
  display.setCursor(10, 40);
  display.println(F("PWM PID CONTROL"));
  display.display();
  delay(1500);

  // --- D. เริ่มต้น PID ---
  Setpoint = 40.0; // ตั้งค่าเริ่มต้นที่ 50 องศา
  
  // กำหนดขอบเขต Output ให้ตรงกับ PWM (0-255)
  myPID.SetOutputLimits(0, 255);
  myPID.SetMode(AUTOMATIC);

  Serial.println(F("--- ESP32 PID Ready ---"));
  Serial.println(F("Type temp in Serial (e.g., 60.5) to change Setpoint"));
}

void loop() {
  // 1. รับคำสั่งเปลี่ยนอุณหภูมิผ่าน Serial
  if (Serial.available() > 0) {
    float newSp = Serial.parseFloat();
    while(Serial.available()) Serial.read(); // เคลียร์ Buffer
    
    if (newSp >= TEMP_MIN && newSp <= TEMP_MAX) {
      Setpoint = newSp;
      Serial.print(F("New Setpoint: ")); Serial.println(Setpoint);
    } else if (newSp > 0) {
      Serial.println(F("Error: Temp out of range!"));
    }
  }

  // 2. อ่านค่าอุณหภูมิ
  sensors.requestTemperatures(); 
  double currentTemp = sensors.getTempCByIndex(0);

  // 3. ตรวจสอบความปลอดภัย (Safety Checks)
  
  // กรณี 3.1: Sensor มีปัญหา (ค่า -127 หรือ 85)
  if (currentTemp == -127.00 || currentTemp == 85.00) {
    ledcWrite(PWM_CHANNEL, 0); // ตัด Heater ทันที
    displayError("SENSOR", "ERROR");
    return; 
  }
  
  Input = currentTemp;

  // กรณี 3.2: อุณหภูมิเกินกำหนด (Overheat)
  if (Input > TEMP_MAX) {
    ledcWrite(PWM_CHANNEL, 0); // ตัด Heater ทันที
    Output = 0;
    displayError("OVERHEAT", "> 100C");
    Serial.println(F("ALARM: Overheat detected!"));
    return;
  }

  // 4. คำนวณ PID
  myPID.Compute();

  // 5. ส่งค่าไปยัง Hardware (PWM Output)
  // ส่งค่า 0-255 ไปควบคุมความกว้างพัลส์
  ledcWrite(PWM_CHANNEL, (int)Output);

  // 6. แสดงผล (ทุกๆ 200ms)
  if (millis() - lastDisplayTime > 200) {
    updateDisplay();
    debugSerial();
    lastDisplayTime = millis();
  }
}

void updateDisplay() {
  display.clearDisplay();

  // ส่วนหัว
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print(F("TEMP. PID CONTROL"));

  // แสดง PV (ตัวใหญ่)
  display.setTextSize(2);
  display.setCursor(0, 14);
  display.print(F("PV:"));
  display.print(Input, 1); 

  // แสดง Setpoint และ % กำลังไฟ
  display.setTextSize(1);
  display.setCursor(0, 36);
  display.print(F("Set:")); display.print(Setpoint, 0);
  display.print(F(" Pwr:")); 
  display.print(map(Output, 0, 255, 0, 100)); // แปลงเป็น %
  display.print(F("%"));

  // แสดงค่า PID Tuning (Kp, Ki, Kd)
  display.setCursor(0, 48);
  display.print(F("P:")); display.print(Kp, 1);
  display.print(F(" I:")); display.print(Ki, 1);
  display.print(F(" D:")); display.print(Kd, 1);

  // กราฟแท่งแสดงกำลังไฟ Heater (เต็มหน้าจอ ชิดขวา)
  int barHeight = map(Output, 0, 255, 0, 64);
  display.drawRect(118, 0, 10, 64, WHITE); // กรอบเต็มความสูง
  display.fillRect(118, 64 - barHeight, 10, barHeight, WHITE); // ไส้ใน

  display.display();
}

void displayError(String title, String msg) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(5, 10);
  display.println(title);
  
  display.setTextSize(2);
  display.setCursor(5, 35);
  display.print(F("!! "));
  display.print(msg);
  
  display.display();
}

void debugSerial() {
  // รูปแบบข้อมูลสำหรับ Serial Plotter: SP, PV, Output
  Serial.print("Set:"); Serial.print(Setpoint); Serial.print(",");
  Serial.print("PV:"); Serial.print(Input); Serial.print(",");
  Serial.print("Out:"); Serial.println(Output);
}