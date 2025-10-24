// ------------------------- Libraries -------------------------
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoJson.h>



// ------------------------- WiFi & Firebase Config -------------------------
#define WIFI_SSID "Erasmus lab"
#define WIFI_PASSWORD "Erasmus@lab#"

#define API_KEY "AIzaSyBhDwUY9ZU7KrgQSzREW8Dl8QyombvcIqc" // Replace with your Firebase Web API key
#define DATABASE_URL "https://shapeos-smarthome-default-rtdb.firebaseio.com/"

// ------------------------- Firebase Objects -------------------------
FirebaseData fbData;
FirebaseAuth auth;
FirebaseConfig config;

// ------------------------- RTC -------------------------
RTC_DS1307 rtc;
bool rtcAvailable = false;

// ------------------------- Pins -------------------------
#define BULB_RELAY  16
#define RELAY_BELL  17
#define RELAY_FAN   18
#define PUMP_RELAY  19

#define SMOKE_PIN   34
#define FLAME_PIN   27
#define PIR_PIN     26
#define ACS_PIN     35
#define ZMPT_PIN    32

// ------------------------- Thresholds -------------------------
int smokeThresholdHigh = 450;
int smokeThresholdSafe = 350;

bool lastFanState  = false;
bool lastBellState = false;
bool lastBulbState = false;
bool lastPumpState = false;

bool smokeAlertActive  = false;
bool flameAlertActive  = false;
bool motionAlertActive = false;

unsigned long bellOnTime = 0;
bool bellTimerActive = false;

float zeroCurrentVoltage = 0;
float sensitivityACS = 0.185;
float calibrationFactorVoltage = 67.0;

unsigned long lastEnergyMillis = 0;
float cumulativeEnergy = 0.0;

unsigned long lastFirebaseRead = 0;
const unsigned long firebaseReadInterval = 2000UL;
unsigned long lastEnergyUpload = 0;
const unsigned long energyUploadInterval = 5000UL;

// ------------------------- Appliance IDs -------------------------
String fanId  = "1";
String bulbId = "2";
String pumpId = "3";
String bellId = "4";

// ------------------------- Helper Functions -------------------------
String getTimeStamp() {
  if (rtcAvailable) {
    DateTime t = rtc.now();
    char buf[20];
    sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
            t.year(), t.month(), t.day(),
            t.hour(), t.minute(), t.second());
    return String(buf);
  } else {
    unsigned long s = millis() / 1000;
    unsigned long h = s / 3600;
    unsigned long m = (s % 3600) / 60;
    unsigned long sec = s % 60;
    char buf[20];
    sprintf(buf, "uptime %02lu:%02lu:%02lu", h, m, sec);
    return String(buf);
  }
}

void pushHistory(String event) {
  FirebaseJson json;
  json.add("time", getTimeStamp());
  json.add("event", event);
  if (!Firebase.RTDB.pushJSON(&fbData, "/alerts_history", &json)) {
    Serial.print("Push history failed: ");
    Serial.println(fbData.errorReason());
  }
}

void updateAlert(String type, String msg, bool &lastState) {
  if (msg == "Safe" && lastState) {
    Firebase.RTDB.setString(&fbData, "/alerts/" + type, "Safe");
    pushHistory(type + " safe");
    lastState = false;
  } else if (msg != "Safe" && !lastState) {
    Firebase.RTDB.setString(&fbData, "/alerts/" + type, msg);
    pushHistory(msg);
    lastState = true;
  }
}

bool setAppliance(const String &id, const String &name, int pin, bool state, bool &lastState) {
  if (lastState != state) {
    digitalWrite(pin, state ? LOW : HIGH);
    Firebase.RTDB.setBool(&fbData, "/appliances/" + id + "/isOn", state);
    pushHistory(name + (state ? " turned ON" : " turned OFF"));
    lastState = state;
    return true;
  }
  return false;
}

bool readBoolSafe(const String &path) {
  if (!WiFi.isConnected()) {
    Serial.println("WiFi disconnected, returning false");
    return false;
  }
  if (Firebase.RTDB.getBool(&fbData, path)) {
    if (fbData.dataType() == "boolean") return fbData.boolData();
  } else {
    Serial.print("Firebase read failed: ");
    Serial.println(fbData.errorReason());
  }
  return false;
}

// ------------------------- Calibration -------------------------
void calibrateCurrentSensor() {
  const int samples = 1000;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(ACS_PIN);
    delay(1);
  }
  float avgADC = sum / (float)samples;
  zeroCurrentVoltage = (avgADC / 4095.0) * 3.3;
}

// ------------------------- Read Sensors -------------------------
float readCurrent() {
  const int samples = 300;
  long sum = 0;
  for (int i = 0; i < samples; i++) sum += analogRead(ACS_PIN);
  float avgADC = sum / (float)samples;
  float voltage = (avgADC / 4095.0) * 3.3;
  float current = (voltage - zeroCurrentVoltage) / sensitivityACS;
  if (abs(current) < 0.02) return 0.0;
  return abs(current);
}

float readVoltage() {
  const int samples = 300;
  long sum = 0;
  for (int i = 0; i < samples; i++) sum += analogRead(ZMPT_PIN);
  float avgADC = sum / (float)samples;
  float voltage = (avgADC / 4095.0) * 3.3;
  return voltage * calibrationFactorVoltage;
}

// ------------------------- Setup -------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22, 100000);

  if (rtc.begin()) {
    rtcAvailable = true;
    if (!rtc.isrunning()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("RTC initialized");
  } else {
    Serial.println("RTC not found");
  }

  pinMode(BULB_RELAY, OUTPUT);
  pinMode(RELAY_BELL, OUTPUT);
  pinMode(RELAY_FAN, OUTPUT);
  pinMode(PUMP_RELAY, OUTPUT);

  digitalWrite(BULB_RELAY, HIGH);
  digitalWrite(RELAY_BELL, HIGH);
  digitalWrite(RELAY_FAN, HIGH);
  digitalWrite(PUMP_RELAY, HIGH);

  pinMode(PIR_PIN, INPUT);
  pinMode(FLAME_PIN, INPUT);

  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWi-Fi connected!");

    config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  // Sign up anonymously
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase signup successful!");
  } else {
    Serial.printf("Firebase signup failed: %s\n", config.signer.signupError.message.c_str());
  }

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println("Firebase initialized successfully!");

  calibrateCurrentSensor();
  Serial.println("Current sensor calibrated");
}

// ------------------------- Loop -------------------------
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    delay(2000);
    return;
  }

  int smoke = analogRead(SMOKE_PIN);
  bool flameDetected = (digitalRead(FLAME_PIN) == LOW);
  bool motionDetected = (digitalRead(PIR_PIN) == HIGH);

  bool smokeDetected = smoke >= smokeThresholdHigh;
  updateAlert("smoke", smokeDetected ? "Smoke detected" : "Safe", smokeAlertActive);

  if (smokeDetected) setAppliance(fanId, "Fan", RELAY_FAN, true, lastFanState);
  else setAppliance(fanId, "Fan", RELAY_FAN, false, lastFanState);

  if (flameDetected && !bellTimerActive) {
    updateAlert("flame", "Flame detected", flameAlertActive);
    setAppliance(bellId, "Bell", RELAY_BELL, true, lastBellState);
    bellOnTime = millis();
    bellTimerActive = true;
  }

  if (bellTimerActive && millis() - bellOnTime >= 3000) {
    setAppliance(bellId, "Bell", RELAY_BELL, false, lastBellState);
    updateAlert("flame", "Safe", flameAlertActive);
    bellTimerActive = false;
  }

  if (motionDetected)
    Firebase.RTDB.setString(&fbData, "/alerts/motion", "Motion detected");
  else
    Firebase.RTDB.setString(&fbData, "/alerts/motion", "Safe");

  if (millis() - lastFirebaseRead >= firebaseReadInterval) {
    lastFirebaseRead = millis();
    bool bellToggle = readBoolSafe("/appliances/" + bellId + "/isOn");
    setAppliance(bellId, "Bell", RELAY_BELL, bellToggle, lastBellState);

    bool pumpToggle = readBoolSafe("/appliances/" + pumpId + "/isOn");
    setAppliance(pumpId, "Pump", PUMP_RELAY, pumpToggle, lastPumpState);

    bool bulbToggle = readBoolSafe("/appliances/" + bulbId + "/isOn");
    setAppliance(bulbId, "Bulb", BULB_RELAY, bulbToggle, lastBulbState);
  }

  float voltage = readVoltage();
  float current = lastBulbState ? readCurrent() : 0.0;
  float power = voltage * current;

  unsigned long now = millis();
  if (lastEnergyMillis > 0) {
    float hours = (now - lastEnergyMillis) / 3600000.0;
    cumulativeEnergy += power * hours / 1000.0;
  }
  lastEnergyMillis = now;

  if (millis() - lastEnergyUpload >= energyUploadInterval) {
    lastEnergyUpload = millis();
    Firebase.RTDB.setFloat(&fbData, "/appliances/" + bulbId + "/voltage", voltage);
    Firebase.RTDB.setFloat(&fbData, "/appliances/" + bulbId + "/current", current);
    Firebase.RTDB.setFloat(&fbData, "/appliances/" + bulbId + "/power", power);
    Firebase.RTDB.setFloat(&fbData, "/appliances/" + bulbId + "/energy", cumulativeEnergy);
  }

  Serial.printf("Bulb -> V: %.2f V | I: %.2f A | P: %.2f W | E: %.3f kWh\n",
                voltage, current, power, cumulativeEnergy);

  delay(200);
}
