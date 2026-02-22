/**
 * ESP32 Kalman Filter Temperature Monitoring System
 * ---------------------------------------------------
 * รายละเอียด Hardware:
 * - MCU: ESP32 DevKit V2 Board (THAITECHZONE)
 * - Sensor: DS18B20 (ต่อที่ GPIO 14)
 * - Heater: MOSFET หรือ SSR (ต่อที่ GPIO 13) -> PWM คงที่ 40%
 * - Display: OLED 0.96" (I2C: SDA=21, SCL=22)
 * - Filter: Kalman Filter สำหรับกรองสัญญาณเซนเซอร์
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "SimpleKalmanFilter.h"  // Kalman Filter สำหรับกรองสัญญาณเซนเซอร์


// =========================================
// 1. การกำหนดขาพอร์ต (Pin Definitions)
// =========================================
#define HEATER_PIN 13    // ขาจ่ายสัญญาณ PWM ไปยัง Heater Driver
#define ONE_WIRE_BUS 14  // ขา Data ของ Sensor DS18B20

// Manual Control IO / Isolated Inputs ถูกตัดออก (ไม่ใช้ในโหมด Fuzzy Logic ล้วน)

// =========================================
// 2. การตั้งค่าระบบ (System Settings)
// =========================================
// ขอบเคตอุณหภูมิและความปลอดภัย
#define TEMP_MIN 0.0     // ค่าต่ำสุดที่ยอมให้ตั้ง
#define TEMP_MAX 100.0   // ค่าสูงสุด และจุดตัด Safety Cutoff

// การตั้งค่า PWM (สำหรับ ESP32)
const int PWM_FREQ = 1000;     // ความถี่ 1kHz (เหมาะกับ MOSFET)
const int PWM_CHANNEL = 0;     // ช่องสัญญาณ PWM 0
const int PWM_RESOLUTION = 8;  // ความละเอียด 8-bit (ค่า 0-255)
const int PWM_FIXED_VALUE = 64;  // ค่า PWM คงที่ 20%

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

// =========================================
// พารามิเตอร์ Kalman Filter
// =========================================
const double KALMAN_Q = 0.01;              // Process Noise (ความผันผวนของกระบวนการ)
                                            // ค่าต่ำ = อุณหภูมิเปลี่ยนแปลงช้า
                                            // ค่าสูง = อุณหภูมิเปลี่ยนแปลงเร็ว
                                            
const double KALMAN_R = 0.5;               // Sensor Noise (ความผันผวนของเซนเซอร์)
                                            // ค่าต่ำ = เชื่อค่าเซนเซอร์มาก (กรองน้อย)
                                            // ค่าสูง = เชื่อค่าเซนเซอร์น้อย (กรองมาก)
                                            
const double KALMAN_P = 1.0;               // Estimation Error Covariance (ค่าเริ่มต้น)
const double KALMAN_INITIAL_VALUE = 25.0;  // ค่าอุณหภูมิเริ่มต้น (°C)

// สร้าง Kalman Filter Object
SimpleKalmanFilter tempKalmanFilter(KALMAN_Q, KALMAN_R, KALMAN_P, KALMAN_INITIAL_VALUE);

// =========================================
// ตัวแปรสำหรับการทดสอบ Kalman Filter
// =========================================
double rawTemp = 0.0;      // ค่าอุณหภูมิดิบจากเซนเซอร์
double filteredTemp = 0.0; // ค่าอุณหภูมิหลังผ่าน Kalman Filter

// ตัวแปรจับเวลา
unsigned long lastDisplayTime = 0;
unsigned long lastSensorReadTime = 0;
unsigned long lastSerialDebugTime = 0;
unsigned long startupTime = 0;
bool startupComplete = false;

// Timing Constants
#define DISPLAY_UPDATE_INTERVAL 200    // ms - อัปเดตหน้าจอ
#define SENSOR_READ_INTERVAL 250       // ms - อ่านค่า sensor
#define SERIAL_DEBUG_INTERVAL 500      // ms - แสดงผล Serial
#define STARTUP_DELAY 1500             // ms - delay เริ่มต้น

// =========================================
// 4. ประกาศฟังก์ชันล่วงหน้า (Function Prototypes)
// =========================================
void updateDisplay();
void displayError(String title, String msg);
void debugSerial();
void drawMainScreen();

void setup() {
  Serial.begin(115200);
  
  // --- A. ตั้งค่า PWM สำหรับ Heater (คงที่ 40%) ---
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(HEATER_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, PWM_FIXED_VALUE); // ตั้งค่า PWM คงที่ 40%

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
  display.println(F("KALMAN FILTER TEST"));
  display.display();
  
  // บันทึกเวลาเริ่มต้น (จะรอ 1.5 วินาทีใน loop)
  startupTime = millis();

  Serial.println(F("--- ESP32 Kalman Filter Test ---"));
  Serial.println(F("PWM Fixed: 40%"));
  Serial.print(F("Kalman Q:")); Serial.print(KALMAN_Q);
  Serial.print(F(" R:")); Serial.print(KALMAN_R);
  Serial.print(F(" P:")); Serial.println(KALMAN_P);
}

void loop() {
  unsigned long currentTime = millis();
  
  // รอจนกว่าจะผ่านเวลา startup
  if (!startupComplete) {
    if (currentTime - startupTime >= STARTUP_DELAY) {
      startupComplete = true;
      Serial.println(F("Startup complete!"));
    } else {
      return;  // ยังไม่ถึงเวลา ให้รอต่อ
    }
  }
  
  // 1. อ่านค่าอุณหภูมิ (ทุกๆ SENSOR_READ_INTERVAL)
  if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    sensors.requestTemperatures(); 
    rawTemp = sensors.getTempCByIndex(0);             // อ่านค่าดิบจากเซนเซอร์
    filteredTemp = tempKalmanFilter.update(rawTemp);  // กรองด้วย Kalman Filter

    lastSensorReadTime = currentTime;
  }

  // 2. ตรวจสอบความปลอดภัย (Safety Checks)
  // กรณี 2.1: Sensor มีปัญหา (ค่า -127 หรือ 85)
  if (rawTemp == -127.00 || rawTemp == 85.00) {
    ledcWrite(PWM_CHANNEL, 0); // ปิด Heater ทันที
    displayError("SENSOR", "ERROR");
    return; 
  }

  // กรณี 2.2: อุณหภูมิเกินกำหนด (Overheat)
  if (filteredTemp > TEMP_MAX) {
    ledcWrite(PWM_CHANNEL, 0); // ปิด Heater ทันที
    displayError("OVERHEAT", ">100C");
    Serial.println(F("ALARM: Overheat detected!"));
    return;
  }

  // 3. แสดงผลหน้าจอ (ทุกๆ DISPLAY_UPDATE_INTERVAL)
  if (currentTime - lastDisplayTime >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    lastDisplayTime = currentTime;
  }
  
  // 4. แสดงผล Serial Debug (ทุกๆ SERIAL_DEBUG_INTERVAL)
  if (currentTime - lastSerialDebugTime >= SERIAL_DEBUG_INTERVAL) {
    debugSerial();
    lastSerialDebugTime = currentTime;
  }
}

void updateDisplay() {
  drawMainScreen();
}

void drawMainScreen() {
  display.clearDisplay();

  // ส่วนหัว
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print(F("KALMAN FILTER"));

  // แสดง Filtered Temperature (ตัวใหญ่)
  display.setTextSize(2);
  display.setCursor(0, 12);
  display.print(filteredTemp, 1); 
  display.print(F("C"));

  // แสดง Raw Temperature
  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print(F("Raw: "));
  display.print(rawTemp, 1);
  display.print(F("C"));

  // แสดง PWM คงที่
  display.setCursor(0, 40);
  display.print(F("PWM: 20% (Fixed)"));

  // แสดง Kalman Gain
  display.setCursor(0, 50);
  display.print(F("K-Gain: "));
  display.print(tempKalmanFilter.getKalmanGain(), 3);

  // กราฟแท่งแสดงกำลังไฟ Heater (คงที่ที่ 40%)
  int barHeight = map(PWM_FIXED_VALUE, 0, 255, 0, 64);
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
  // รูปแบบข้อมูลสำหรับ Serial Plotter: Raw, Filtered, PWM, Kalman Gain
  Serial.print("Raw:"); Serial.print(rawTemp); Serial.print(",");
  Serial.print("Filt:"); Serial.print(filteredTemp); Serial.print(",");
  Serial.print("PWM:"); Serial.print(PWM_FIXED_VALUE); Serial.print(",");
  Serial.print("KGain:"); Serial.println(tempKalmanFilter.getKalmanGain(), 4);
}
