#include <Wire.h>
#include <Adafruit_LIS3DH.h>
//#include <Adafruit_Sensor.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
//#include <Arduino.h>

#define I2C_SCL_PIN 2
#define I2C_SDA_PIN 1
#define OLED_ADDR (0x3C << 1)
void i2c_delay() { delayMicroseconds(2); }
void i2c_sda_out() { pinMode(I2C_SDA_PIN, OUTPUT); }
void i2c_sda_in() { pinMode(I2C_SDA_PIN, INPUT_PULLUP); }

int previousHeartRate = -1;

TinyGPSPlus gps;
// TwoWire buses for sensors
TwoWire WireMAX = TwoWire(0);     // MAX30105 on pins 5 (SDA), 6 (SCL)
TwoWire WireACC = TwoWire(1);     // LIS3DH on pins 3 (SDA), 4 (SCL)

// Create a serial object for GPS on UART2
HardwareSerial GPSserial(2);  // UART2
SoftwareSerial mySerial(8, 9);
int timeout;
// Pin configuration
#define GPS_RX_PIN 44  // GPS TX to ESP32 RX
#define GPS_TX_PIN 43  // GPS RX to ESP32 TX


double lastLat = 0.0;
double lastLng = 0.0;

// Function prototypes
void printGPSData();
void CallNumber();
void SendMessage(String message);
String readSerial();
// Sensors
MAX30105 particleSensor;
Adafruit_LIS3DH lis(&WireACC);

// Heart rate / SPO2 buffers and variables
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t redBuffer[100];
uint16_t irBuffer;
#else
uint32_t redBuffer [100];
uint32_t irBuffer [100];
#endif

const int bufferLength = 100;
int32_t heartRate;
int8_t validHeartRate;
int32_t spo2;
int8_t validSPO2;

// Rolling buffer index
int sampleIndex = 0;

// Fall threshold: g-force value considered a fall (adjust after field testing)
const float FALL_THRESHOLD_G = 2.0;
void i2c_start() {
  i2c_sda_out();
  digitalWrite(I2C_SDA_PIN, HIGH);
  digitalWrite(I2C_SCL_PIN, HIGH);
  i2c_delay();
  digitalWrite(I2C_SDA_PIN, LOW);
  i2c_delay();
  digitalWrite(I2C_SCL_PIN, LOW);
  i2c_delay();
}

void i2c_stop() {
  i2c_sda_out();
  digitalWrite(I2C_SDA_PIN, LOW);
  i2c_delay();
  digitalWrite(I2C_SCL_PIN, HIGH);
  i2c_delay();
  digitalWrite(I2C_SDA_PIN, HIGH);
  i2c_delay();
}

bool i2c_write_byte(uint8_t data) {
  i2c_sda_out();
  for (int i = 0; i < 8; i++) {
    digitalWrite(I2C_SDA_PIN, (data & 0x80) ? HIGH : LOW);
    digitalWrite(I2C_SCL_PIN, HIGH);
    i2c_delay();
    digitalWrite(I2C_SCL_PIN, LOW);
    i2c_delay();
    data <<= 1;
  }
  i2c_sda_in();
  digitalWrite(I2C_SCL_PIN, HIGH);
  i2c_delay();
  bool ack = !digitalRead(I2C_SDA_PIN);
  digitalWrite(I2C_SCL_PIN, LOW);
  i2c_delay();
  i2c_sda_out();
  return ack;
}

void oled_command(uint8_t cmd) {
  i2c_start();
  i2c_write_byte(OLED_ADDR);
  i2c_write_byte(0x00);
  i2c_write_byte(cmd);
  i2c_stop();
}

void oled_data(uint8_t dat) {
  i2c_start();
  i2c_write_byte(OLED_ADDR);
  i2c_write_byte(0x40);
  i2c_write_byte(dat);
  i2c_stop();
}

void oled_init() {
  pinMode(I2C_SCL_PIN, OUTPUT);
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SCL_PIN, HIGH);
  digitalWrite(I2C_SDA_PIN, HIGH);
  delay(100);

  oled_command(0xAE); oled_command(0xD5); oled_command(0x80); oled_command(0xA8); oled_command(0x3F);
  oled_command(0xD3); oled_command(0x00); oled_command(0x40); oled_command(0x8D); oled_command(0x14);
  oled_command(0x20); oled_command(0x00); oled_command(0xA1); oled_command(0xC8); oled_command(0xDA);
  oled_command(0x12); oled_command(0x81); oled_command(0x7F); oled_command(0xD9); oled_command(0xF1);
  oled_command(0xDB); oled_command(0x40); oled_command(0xA4); oled_command(0xA6); oled_command(0xAF);
}

void oled_clear() {
  for (uint8_t page = 0; page < 8; page++) {
    oled_command(0xB0 | page);
    oled_command(0x00);
    oled_command(0x10);
    for (uint8_t col = 0; col < 128; col++) oled_data(0x00);
  }
}

const uint8_t font5x7[][5] = {
  {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, 
  {0x14,0x7F,0x14,0x7F,0x14}, {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
  {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00}, {0x00,0x1C,0x22,0x41,0x00}, 
  {0x00,0x41,0x22,0x1C,0x00}, {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08}, 
  {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00},
  {0x20,0x10,0x08,0x04,0x02}, {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, 
  {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31}, {0x18,0x14,0x12,0x7F,0x10}, 
  {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03}, 
  {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, 
  {0x00,0x56,0x36,0x00,0x00}, {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14},
  {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06}, {0x32,0x49,0x79,0x41,0x3E},
  {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
  {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
  {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40}, 
  {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E}, 
  {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, 
  {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, 
  {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x30,0x40,0x3C}, {0x44,0x28,0x10,0x28,0x44}, 
  {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00}, 
  {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, {0x04,0x02,0x01,0x02,0x04}, 
  {0x40,0x40,0x40,0x40,0x40}, {0x00,0x03,0x05,0x00,0x00}, {0x20,0x54,0x54,0x54,0x78},
  {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20}, {0x38,0x44,0x44,0x48,0x7F},
  {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E},
  {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x44,0x3D,0x00},
  {0x7F,0x10,0x28,0x44,0x00}, {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x7C},
  {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38}, {0x7C,0x14,0x14,0x14,0x08},
  {0x08,0x14,0x14,0x18,0x7C}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
  {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C},
  {0x3C,0x40,0x30,0x40,0x3C}, {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C},
  {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00}, {0x00,0x00,0x7F,0x00,0x00},
  {0x00,0x41,0x36,0x08,0x00}, {0x10,0x08,0x08,0x10,0x08}, {0x00,0x00,0x00,0x00,0x00}
};

void oled_set_cursor(uint8_t page, uint8_t col) {
  oled_command(0xB0 | (page & 0x07));
  oled_command(0x00 | (col & 0x0F));
  oled_command(0x10 | ((col >> 4) & 0x0F));
}

void oled_write_char(char c) {
  uint8_t idx = (c < 32 || c > 127) ? 0 : (c - 32);
  for (int i = 0; i < 5; i++) {
    oled_data(font5x7[idx][i]);
  }
}

void oled_write_str(uint8_t startPage, uint8_t startCol, const char *str) {
  uint8_t page = startPage;
  uint8_t col = startCol;

  oled_set_cursor(page, col);

  while (*str) {
    // If current column exceeds display width, move to the next line
    if (col > 123) {  // Max 128 cols - 5 columns per char
      col = 0;
      page++;
      // If we reach the last line, stop
      if (page > 7) {
        break;
      }
      oled_set_cursor(page, col);
    }

    if (*str == ' ') {
      // Print 5 columns of blank space for a space character
      for (int i = 0; i < 5; i++) {
        oled_data(0x00);
      }
      col += 5;
    } else {
      // Print character normally
      oled_write_char(*str);
      col += 5;
    }
    str++;
  }
}


const char* statusMsgs[] = {
  "patient vitals are normal pumping heart animation",
  "Emergency! Patient has fallen. Immediate medical attention required",
  "Urgent medical attention required abnormal heart rate and SpO2 detected"
};
const int numMsgs = sizeof(statusMsgs)/sizeof(statusMsgs[0]);
int msgIdx = 0;
unsigned long lastChange = 0;

// Function to display first status message
void displayStatus1() {
  oled_clear();
  displayVitals(heartRate, spo2);
  oled_write_str(2, 2, "Patient Vitals are NORMAL ");
}

// Function to display second status message
void displayStatus2() {
  oled_clear();
  oled_write_str(3, 3, "Emergency! Patient has fallen. Immediate medical attention required");
}

// Function to display third status message
void displayStatus3() {
  oled_clear();
  oled_write_str(1, 1, "Urgent medical attention required abnormal heart rate and SpO2 detected");
}

void displayStatus4() {
  oled_clear();
  //oled_write_str(43, 4, "WelcomeW");
  oled_write_str(2, 45, "Welcome");
  oled_write_str(5, 50, "SAGAR");
}
void displayStatus5() {
  oled_clear();
  oled_write_str(2, 30, "Patient Heath");
  oled_write_str(4, 25, "Tracking System");
}

void displaystatus6()
{
  displayStatus4();
  delay(5000);
  displayStatus5();
}

void displayVitals(int32_t hr, int32_t spo2_val) {

 char buffer[20];
 // Print Heart Rate
 sprintf(buffer, "HR: %ld bpm", hr);
 oled_write_str(4, 0, buffer);
 // Print SpO2
 sprintf(buffer, "SpO2: %ld%%", spo2_val);
 oled_write_str(6, 0, buffer);
}


void setup() {
  Serial.begin(9600);
  oled_init();
  oled_clear();
  mySerial.begin(9600);
  GPSserial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);  // GPS module baud rate
 // Serial.println("GPS module initialized on Serial2 (GPIO 43/44)");
  //Serial.println("Type\n s) to send an SMS\n r) to receive an SMS\n c) to make a call");
  Serial.println("Initializing..."); 
  delay(1000);
  mySerial.println("AT");
  // Initialize MAX30105
  WireMAX.begin(5, 6, 400000);
  if (!particleSensor.begin(WireMAX, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found");
    while (1);
  }
  particleSensor.setup(0x1F, 8, 3, 100, 411, 4096);

  // Initialize LIS3DH
  WireACC.begin(3, 4);
  if (!lis.begin(0x18)) {
    Serial.println("LIS3DH not found");
    while (1);
  }
  lis.setRange(LIS3DH_RANGE_2_G);

  // Fill buffers initially
  Serial.println("Warming up sensors...");
  displaystatus6();
  for (int i = 0; i < bufferLength; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer,
    &spo2, &validSPO2, &heartRate, &validHeartRate);
}


void loop() {
  // Read new sample from MAX30105 in rolling buffer
  if (particleSensor.available()) {
    redBuffer[sampleIndex] = particleSensor.getRed();
    irBuffer[sampleIndex] = particleSensor.getIR();
    particleSensor.nextSample();

    sampleIndex++;
    if (sampleIndex >= bufferLength) {
      sampleIndex = 0;
      // Calculate heart rate and SpO2 once buffer is full
      maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer,
        &spo2, &validSPO2, &heartRate, &validHeartRate);
    }
  } else {
    particleSensor.check(); // Keep sensor up to date
  }

  // Read accelerometer and calculate total g
  lis.read();
  // LIS3DH: ±2g mode, raw value 1g ≈ 16384 (LSB/g)
  float ax = lis.x / 16384.0;
  float ay = lis.y / 16384.0;
  float az = lis.z / 16384.0;
  float totalG = sqrt(ax * ax + ay * ay + az * az);

  // Fall detection logic -- triggers when g-force is higher than threshold
  bool fallDetected = (totalG > FALL_THRESHOLD_G);

  // Output
  Serial.print("HR: ");
  Serial.print(heartRate);
  Serial.print(validHeartRate ? " (valid)" : " (invalid)");
  Serial.print("  SPO2: ");
  Serial.print(spo2);
  Serial.print(validSPO2 ? " (valid)" : " (invalid)");
  Serial.print("  TotalG: ");
  Serial.print(totalG, 2);
  Serial.print(" Fall: ");
  Serial.println(fallDetected ? "YES" : "NO");

// Global/static variable to remember old heart rate



  // Example: assume heartRate, validHeartRate, validSPO2, fallDetected are already updated

  if (validHeartRate && validSPO2 && !fallDetected) {
    if (heartRate != previousHeartRate) {
      displayStatus1();   // Call only when heartRate changes
      previousHeartRate = heartRate; // Update last value
    }
  }

  // ... rest of loop



  // Only trigger alert if SPO2 or HR is invalid, or a real fall detected
 // --- Part 1: Check for heart rate or SpO2 invalidity ---
if ((!validHeartRate) || (!validSPO2)) {
    Serial.println("ALERT: HR/SpO2 anomaly detected!");
      displayStatus3();
    bool locationSent = false;
    while (GPSserial.available() > 0) {
        if (gps.encode(GPSserial.read())) {
            if (gps.location.isValid() && !locationSent) {
                String message = createSMSMessage();
                SendMessage(message);
                Serial.println("SMS Sent (anomaly by HR/SpO2)");
                delay(2000);
                CallNumber();
                Serial.println("Call Attempted (anomaly by HR/SpO2)");
                delay(5000);
                locationSent = true; // To avoid multiple sends in a single loop
            } else if (!gps.location.isValid()) {
                Serial.println("Location not available (HR/SpO2 anomaly)");
            }
        }
    }
}

// --- Part 2: Check for fall detection ---
if (fallDetected) {
    Serial.println("ALERT: Fall detected!");
     displayStatus2();
    bool locationSent = false;
    while (GPSserial.available() > 0) {
        if (gps.encode(GPSserial.read())) {
            if (gps.location.isValid() && !locationSent) {
                String message = createSMSMessage1();
                SendMessage(message);
                Serial.println("SMS Sent (fall detected)");
                delay(2000);
                CallNumber();
                Serial.println("Call Attempted (fall detected)");
                delay(5000);
                locationSent = true; // To avoid multiple sends in a single loop
            } else if (!gps.location.isValid()) {
                Serial.println("Location not available (fall detected)");
            }
        }
    }
}


  delay(100); // Adjust sampling interval as needed
}

String createSMSMessage1() {
  String message = "Emergency! Patient has fallen. Immediate medical attention required \n";
  
  if (gps.location.isValid()) {
    message += "Latitude: " + String(gps.location.lat(), 6) + "\n";
    message += "Longitude: " + String(gps.location.lng(), 6) + "\n";
    
    // Create Google Maps link
    String googleMapsLink = "http://maps.google.com/maps?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    message += "Link: " + googleMapsLink + "\n";
  }

  if (gps.date.isValid()) {
    char dateBuffer[12];
    sprintf(dateBuffer, "%02d/%02d/%02d", gps.date.day(),gps.date.month(), gps.date.year());
    message += "Date: " + String(dateBuffer) + "\n";
  }

 if (gps.time.isValid()) {
    char timeBuffer[12];
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();

    // Add 5:30 for IST timezone
    int addedHours = 5;
    int addedMinutes = 30;

    int totalSeconds = hour * 3600 + minute * 60 + second + addedHours * 3600 + addedMinutes * 60;
    int newHour = (totalSeconds / 3600) % 24;
    int newMinute = (totalSeconds % 3600) / 60;
    int newSecond = totalSeconds % 60;

    sprintf(timeBuffer, "%02d:%02d:%02d", newHour, newMinute, newSecond);
    message += "Time: " + String(timeBuffer) + "\n";
  }
  return message;
}

String createSMSMessage() {
  String message = "Urgent medical attention required abnormal heart rate and SpO2 detected.\n";
  
  if (gps.location.isValid()) {
    message += "Latitude: " + String(gps.location.lat(), 6) + "\n";
    message += "Longitude: " + String(gps.location.lng(), 6) + "\n";
    
    // Create Google Maps link
    String googleMapsLink = "http://maps.google.com/maps?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    message += "Link: " + googleMapsLink + "\n";
  }

  if (gps.date.isValid()) {
    char dateBuffer[12];
    sprintf(dateBuffer, "%02d/%02d/%02d", gps.date.day(),gps.date.month(), gps.date.year());
    message += "Date: " + String(dateBuffer) + "\n";
  }

 if (gps.time.isValid()) {
    char timeBuffer[12];
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();

    // Add 5:30 for IST timezone
    int addedHours = 5;
    int addedMinutes = 30;

    int totalSeconds = hour * 3600 + minute * 60 + second + addedHours * 3600 + addedMinutes * 60;
    int newHour = (totalSeconds / 3600) % 24;
    int newMinute = (totalSeconds % 3600) / 60;
    int newSecond = totalSeconds % 60;

    sprintf(timeBuffer, "%02d:%02d:%02d", newHour, newMinute, newSecond);
    message += "Time: " + String(timeBuffer) + "\n";
  }
  return message;
}

void printGPSData() {
  // This function is no longer needed for the SMS logic but can be kept for serial debugging
  Serial.print("Latitude: ");
  Serial.println(gps.location.lat(), 6);
  Serial.print("Longitude: ");
  Serial.println(gps.location.lng(), 6);

  if (gps.satellites.isValid()) {
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
  } else {
    Serial.println("Satellite data not available");
  }

  if (gps.date.isValid()) {
    char dateBuffer[12];
    sprintf(dateBuffer, "%02d/%02d/%02d", gps.date.day(),gps.date.month(), gps.date.year());
    Serial.print("Date: ");
    Serial.println(dateBuffer);
  } else {
    Serial.println("Date not available");
  }

  if (gps.time.isValid()) {
    char timeBuffer[12];
    sprintf(timeBuffer, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    Serial.print("Time: ");
    Serial.println(timeBuffer);
  } else {
    Serial.println("Time not available");
  }
}

void CallNumber() {
  mySerial.println("ATD+ +918296520196;");
  Serial.println(readSerial());
  delay(20000); 
  mySerial.println("ATH"); 
  delay(200);
  Serial.println(readSerial());
}

void SendMessage(String message) {
  mySerial.println("AT+CMGF=1"); 
  Serial.println(readSerial());
  mySerial.println("AT+CMGS=\"+918296520196\"");
  Serial.println(readSerial());
  mySerial.println(message);
  mySerial.println((char)26);
  Serial.println(readSerial());
}

String readSerial() {
  delay(100);
  if (mySerial.available()) {
    return mySerial.readString();
  }
  return "";
}