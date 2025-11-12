#include <WiFi.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Firebase_ESP_Client.h>
#include "addons/RTDBHelper.h"

// -------------------- WiFi & Firebase --------------------
#define WIFI_SSID "vivoY02t"
#define WIFI_PASSWORD "sia@1234"

#define DATABASE_URL "https://yenapoya-19435-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define DATABASE_SECRET "5xoDbTguptYcGp0w95ROzNxyf5qDhURqqY4gMGIz"

// -------------------- Objects --------------------
FirebaseData fbdo;
FirebaseConfig config;
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

unsigned long lastFirebaseUpdate = 0;
unsigned long outsideZoneStart = 0;
unsigned long sosActivatedTime = 0; 
bool sosActive = false;

// -------------------- Temperature --------------------
#define TEMP_ADDR 0x48

// -------------------- Geofence --------------------
#define SAFE_CENTER_LAT 12.9716
#define SAFE_CENTER_LON 77.5946
#define SAFE_RADIUS_METERS 100
#define OUTSIDE_TIME_THRESHOLD 5000

// -------------------- LED for heartbeat --------------------
#define LED_PIN 2

// -------------------- SOS button --------------------
#define SOS_PIN 26
#define SOS_AUTO_RESET 30000  // 30 seconds
#define DEBOUNCE_DELAY 50     // 50 ms

unsigned long lastButtonTime = 0;

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(LED_PIN, OUTPUT);
  pinMode(SOS_PIN, INPUT_PULLUP);  // SOS button
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

  // WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n✅ WiFi connected");

  // Firebase with token
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = DATABASE_SECRET;
  Firebase.begin(&config, nullptr);
  Firebase.reconnectWiFi(true);
  Serial.println("🔥 Firebase initialized with token");
}

// -------------------- Temperature Read --------------------
float readTemperature() {
  Wire.beginTransmission(TEMP_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(TEMP_ADDR, 2);
  if (Wire.available() == 2) {
    byte msb = Wire.read();
    byte lsb = Wire.read();
    int16_t tempData = ((msb << 8) | lsb) >> 7;
    return tempData * 0.5;
  }
  return 36.5 + random(-5, 6) * 0.1; // fallback simulate 36-37°C
}

// -------------------- GPS --------------------
bool getGPS(float &lat, float &lon) {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
    return true;
  }
  return false;
}

// -------------------- Distance --------------------
float distanceMeters(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371000;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

// -------------------- Main Loop --------------------
void loop() {
  // ---- Simulated heart rate & SpO2 ----
  float heartRate = random(65, 90);      
  float spo2 = random(95, 99);            

  // Blink LED like heartbeat
  digitalWrite(LED_PIN, HIGH);
  delay(60);
  digitalWrite(LED_PIN, LOW);
  delay(60000 / heartRate - 60);

  float tempC = readTemperature();
  float latitude = 0, longitude = 0;
  bool gpsValid = getGPS(latitude, longitude);

  // Geofence check
  bool outsideZone = false;
  if (gpsValid) {
    float dist = distanceMeters(latitude, longitude, SAFE_CENTER_LAT, SAFE_CENTER_LON);
    if (dist > SAFE_RADIUS_METERS) {
      outsideZone = true;
      if (outsideZoneStart == 0) outsideZoneStart = millis();
    } else {
      outsideZone = false;
      outsideZoneStart = 0;
    }
  }
  bool alertGeofence = outsideZone && (millis() - outsideZoneStart > OUTSIDE_TIME_THRESHOLD);

  // -------------------- SOS button check --------------------
  bool buttonPressed = digitalRead(SOS_PIN) == LOW;
  if (buttonPressed && !sosActive && millis() - lastButtonTime > DEBOUNCE_DELAY) {
    sosActive = true;
    sosActivatedTime = millis();
    lastButtonTime = millis();
    Serial.println("🚨 SOS Button Pressed!");
  }

  // Auto-reset SOS after 30 sec
  if (sosActive && millis() - sosActivatedTime > SOS_AUTO_RESET) {
    sosActive = false;
    sosActivatedTime = 0;
    Serial.println("✅ SOS Reset");
  }

  // Friendly status
  String status = alertGeofence ? "Outside Safe Zone" : "Safe";

  // Serial Output
  Serial.print(gpsValid ? "📍 GPS OK  " : "📡 Waiting GPS...  ");
  Serial.print("| 💓 HR: "); Serial.print(heartRate);
  Serial.print(" bpm | 🩸 SpO₂: "); Serial.print(spo2);
  Serial.print("% | 🌡️ Temp: "); Serial.print(tempC);
  Serial.print("°C | Status: "); Serial.print(status);
  Serial.print(" | SOS: "); Serial.println(sosActive ? "YES" : "NO");

  // -------------------- Firebase Upload --------------------
  if (Firebase.ready() && (millis() - lastFirebaseUpdate > 1000 || lastFirebaseUpdate == 0)) {
    lastFirebaseUpdate = millis();
    String path = "/TouristSafe/device01";

    Firebase.RTDB.setFloat(&fbdo, path + "/heartRate", heartRate);
    Firebase.RTDB.setFloat(&fbdo, path + "/spo2", spo2);
    Firebase.RTDB.setFloat(&fbdo, path + "/temperature_c", tempC);

    if (gpsValid) {
      Firebase.RTDB.setFloat(&fbdo, path + "/latitude", latitude);
      Firebase.RTDB.setFloat(&fbdo, path + "/longitude", longitude);
    } else {
      Firebase.RTDB.setString(&fbdo, path + "/gps_status", "Waiting for fix");
    }

    Firebase.RTDB.setBool(&fbdo, path + "/geofence_alert", alertGeofence);
    Firebase.RTDB.setString(&fbdo, path + "/status", status);
    Firebase.RTDB.setBool(&fbdo, path + "/sos_alert", sosActive);
    Firebase.RTDB.setString(&fbdo, path + "/timestamp", String(millis()));
  }
}
