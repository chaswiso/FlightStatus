// Arduino ESP32 Nano + NEO-7M GPS - UART2 on GPIO4/5 with Raw Data Debug
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

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
const int totalPages            = 3;

// GPS state
float gps_lat = 0, gps_lng = 0, gps_alt = 0, gps_speed = 0;
int   gps_satellites = 0;
bool  gps_fix = false;

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
        display.setCursor(0,44);
        display.print("RSSI: "); display.print(WiFi.RSSI()); display.println(" dBm");
      } else {
        display.setCursor(0,20);
        display.println("WiFi: Disconnected");
      }
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
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

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
}

void loop() {
  unsigned long now = millis();

  // Read and print raw GPS data
  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    Serial.write(c);
    if (gps.encode(c)) {
      gps_fix        = gps.location.isValid();
      gps_lat        = gps.location.lat();
      gps_lng        = gps.location.lng();
      gps_alt        = gps.altitude.meters();
      gps_speed      = gps.speed.mph();
      gps_satellites = gps.satellites.value();
    }
  }

  // Auto-page rotate
  if (now - lastPageChange >= pageChangeInterval) {
    lastPageChange = now;
    currentPage = (currentPage + 1) % totalPages;
  }

  // Refresh display
  if (now - lastDisplayUpdate >= displayInterval) {
    lastDisplayUpdate = now;
    displayPage(currentPage);
  }

  delay(10);
}
