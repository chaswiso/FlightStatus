// Arduino ESP32 Nano + NEO-7M GPS - UART2 on GPIO4/5 with Raw Data Debug
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// OLED Display settings
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Use UART2 on GPIO4/5
HardwareSerial GPSSerial(2);
TinyGPSPlus gps;
#define GPS_RX_PIN 4
#define GPS_TX_PIN 5
#define GPS_BAUD   9600

// I2C on GPIO8 (SDA) and GPIO9 (SCL)
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// WiFi credentials
const char* ssid     = "Dick Johnson";
const char* password = "sub$onic Unicorn thre@d";

const int ledPin = 25;

// Timing globals
unsigned long previousMillis    = 0;
const long interval             = 1000;
unsigned long lastDisplayUpdate = 0;
const long displayInterval      = 2000;
unsigned long lastPageChange    = 0;
const long pageChangeInterval   = 5000;
int currentPage                 = 0;
const int totalPages            = 5;

// GPS state
float gps_lat = 0, gps_lng = 0, gps_alt = 0, gps_speed = 0;
int   gps_satellites = 0;
bool  gps_fix = false;

// MPU6050 state
Adafruit_MPU6050 mpu;
float mpu_accel_x = 0, mpu_accel_y = 0, mpu_accel_z = 0;
float mpu_gyro_x = 0, mpu_gyro_y = 0, mpu_gyro_z = 0;
bool mpuShakeDetected = false;

void displayPage(int page) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  switch (page) {
    case 0:
      display.setCursor(0,0);
      display.println("Flight Status");
      if (WiFi.status() == WL_CONNECTED) {
        display.setCursor(0,20);
        display.println("WiFi: Connected");
        display.setCursor(0,32);
        display.println(WiFi.localIP().toString());
      } else {
        display.setCursor(0,20);
        display.println("WiFi: Disconnected");
      }
      display.setCursor(0,56);
      if (mpuShakeDetected) display.print("MOTION DETECTED");
      break;

    case 1:
      display.setCursor(0,0);
      display.println("GPS Status");
      display.setCursor(0,20);
      display.print("Fix: "); display.println(gps_fix ? "YES" : "NO");
      display.setCursor(0,32);
      display.print("Sats: "); display.print(gps_satellites);
      break;

    case 2:
      display.setCursor(0,0);
      display.println("GPS Details");
      display.setCursor(0,16);
      display.print("Lat:  "); display.println(gps_lat, 6);
      display.setCursor(0,28);
      display.print("Lng:  "); display.println(gps_lng, 6);
      display.setCursor(0,40);
      display.print("Alt:  "); display.print(gps_alt * 3.28084, 1); display.println(" ft");
      display.setCursor(0,52);
      display.print("Spd:  "); display.print(gps_speed, 1); display.println(" mph");
      break;

    case 3:
      display.setCursor(0,0);
      display.println("MPU6050 Accel");
      display.setCursor(0,16);
      display.print("AX: "); display.print(mpu_accel_x, 4);
      display.setCursor(0,28);
      display.print("AY: "); display.print(mpu_accel_y, 4);
      display.setCursor(0,40);
      display.print("AZ: "); display.print(mpu_accel_z, 4);
      display.setCursor(0,52);
      if (fabs(mpu_accel_x) < 0.1 && fabs(mpu_accel_y) < 0.1 && fabs(mpu_accel_z - 9.8) < 0.1) {
        display.print("STATIONARY");
      }
      break;

    case 4:
      display.setCursor(0,0);
      display.println("MPU6050 Gyro");
      display.setCursor(0,16);
      display.print("GX: "); display.print(mpu_gyro_x, 4);
      display.setCursor(0,28);
      display.print("GY: "); display.print(mpu_gyro_y, 4);
      display.setCursor(0,40);
      display.print("GZ: "); display.print(mpu_gyro_z, 4);
      break;
  }

  display.setCursor(110,56);
  display.print(page+1);
  display.print("/");
  display.print(totalPages);
  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize I2C on GPIO8 (SDA) and GPIO9 (SCL)
  Wire.begin(8, 9);
  Wire.setClock(100000); // Lowered to 100kHz for compatibility

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
  }
  display.clearDisplay();

  // WiFi connect
  WiFi.begin(ssid, password);
  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && ++attempt < 20) {
    delay(500);
  }

  // GPS Serial init
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("GPS UART2 on RX=%d, TX=%d @%d\n", GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);

  // Pin check for MPU6050
  bool imuFound = mpu.begin(0x68, &Wire);
  if (!imuFound) {
    Serial.println("MPU6050 not found on I2C pins 8/9!");
  } else {
    Serial.println("MPU6050 found on I2C pins 8/9.");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // Pin check for GPS (basic check: see if GPSSerial is available)
  if (!GPSSerial) {
    Serial.println("GPS Serial not available on pins 4/5!");
  } else {
    Serial.println("GPS Serial available on pins 4/5.");
  }
}

void loop() {
  unsigned long now = millis();

  // Always clear IMU values at the start of the loop
  mpu_accel_x = mpu_accel_y = mpu_accel_z = 0;
  mpu_gyro_x = mpu_gyro_y = mpu_gyro_z = 0;
  mpuShakeDetected = false;

  // Read and print raw GPS data (no serial output)
  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    // Serial.write(c); // Removed
    if (gps.encode(c)) {
      gps_fix        = gps.location.isValid();
      gps_lat        = gps.location.lat();
      gps_lng        = gps.location.lng();
      gps_alt        = gps.altitude.meters();
      gps_speed      = gps.speed.mph();
      gps_satellites = gps.satellites.value();
    }
  }

  // MPU6050 polling for shake/bump (no serial output)
  bool imuFound = mpu.begin(0x68, &Wire); // Check if IMU is still present
  if (imuFound) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    mpu_accel_x = a.acceleration.x;
    mpu_accel_y = a.acceleration.y;
    mpu_accel_z = a.acceleration.z;
    mpu_gyro_x = g.gyro.x;
    mpu_gyro_y = g.gyro.y;
    mpu_gyro_z = g.gyro.z;
    float shake_threshold = 15.0; // m/s^2, adjust as needed (approx 1.5g)
    mpuShakeDetected = (fabs(mpu_accel_x) > shake_threshold ||
                        fabs(mpu_accel_y) > shake_threshold ||
                        fabs(mpu_accel_z) > shake_threshold);
  }

  // Restore carousel
  if (now - lastPageChange >= pageChangeInterval) {
    lastPageChange = now;
    currentPage = (currentPage + 1) % totalPages;
  }

  if (now - lastDisplayUpdate >= displayInterval) {
    lastDisplayUpdate = now;
    // If IMU not found, show debug message on IMU pages
    if (!imuFound && (currentPage == 3 || currentPage == 4)) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("MPU6050 NOT FOUND");
      display.display();
    } else {
      displayPage(currentPage);
    }
  }

  delay(10);
}
