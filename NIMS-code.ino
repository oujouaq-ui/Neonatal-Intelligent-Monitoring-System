#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* ===== CONFIG ===== */
#define DEVICE_NAME "NeonatalMonitor_ESP32"
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

#define DHT_PIN 4
#define KY013_PIN 34
#define HW827_PIN 15
#define BUZZER_PIN 25
#define DHT_TYPE DHT11

/* ===== OBJETS ===== */
DHT dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
MAX30105 particleSensor;
BLECharacteristic *pTxCharacteristic;

/* ===== VARIABLES ===== */
bool deviceConnected = false;
unsigned long previousBleMillis = 0, lcdUpdateMillis = 0, spo2UpdateMillis = 0, previousMillis = 0;
unsigned long lastAlarmMillis = 0;

// SpO2
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE], rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvgMAX = 0, spo2Value = 0;
bool fingerDetected = false;

#define BUFFER_SIZE 50
long redBuffer[BUFFER_SIZE], irBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

// HW827
volatile unsigned long lastBeatTimeHW827 = 0, beatIntervalHW827 = 0;
volatile int bpmHW827 = 0;
const int BPM_SAMPLES = 5;
int bpmReadings[BPM_SAMPLES], bpmIndex = 0, bpmAvgHW827 = 0;

// Capteurs
float globalHumidity = NAN, globalTempDHT = NAN, globalTempKY013 = NAN;

// Alarmes
enum AlarmType { NONE, BPM_LOW, BPM_HIGH, SPO2_LOW, TEMP_LOW, TEMP_HIGH };
AlarmType currentAlarm = NONE;
int displayMode = 0;

/* ===== BUZZER AMÉLIORÉ ===== */
void handleAlarm() {
  unsigned long currentMillis = millis();
  
  if (currentAlarm == NONE) {
    noTone(BUZZER_PIN);
    return;
  }
  
  // Alarme intermittente pour cas critiques
  if (currentMillis - lastAlarmMillis >= 1000) { // Toutes les secondes
    lastAlarmMillis = currentMillis;
    
    switch (currentAlarm) {
      case SPO2_LOW: // SpO2 < 90% - CRITIQUE - 3 bips rapides
        tone(BUZZER_PIN, 3500, 150); delay(200);
        tone(BUZZER_PIN, 3500, 150); delay(200);
        tone(BUZZER_PIN, 3500, 150);
        break;
        
      case BPM_LOW: // Bradycardie - 2 bips graves
        tone(BUZZER_PIN, 2000, 200); delay(300);
        tone(BUZZER_PIN, 2000, 200);
        break;
        
      case BPM_HIGH: // Tachycardie - 2 bips aigus
        tone(BUZZER_PIN, 3000, 200); delay(300);
        tone(BUZZER_PIN, 3000, 200);
        break;
        
      case TEMP_LOW: // Hypothermie - 1 bip long grave
        tone(BUZZER_PIN, 1500, 500);
        break;
        
      case TEMP_HIGH: // Hyperthermie - 1 bip long aigu
        tone(BUZZER_PIN, 3500, 500);
        break;
        
      default:
        noTone(BUZZER_PIN);
        break;
    }
  }
}

void beepHeartbeat() {
  tone(BUZZER_PIN, 2000, 30); // Bip court pour battement normal
}

void beepBleConnect() {
  tone(BUZZER_PIN, 2500, 100); delay(150); tone(BUZZER_PIN, 2500, 100);
}

/* ===== INTERRUPTION HW827 ===== */
void IRAM_ATTR heartBeatDetectedHW827() {
  unsigned long currentTime = millis();
  beatIntervalHW827 = currentTime - lastBeatTimeHW827;

  if (beatIntervalHW827 > 300 && beatIntervalHW827 < 2000) {
    bpmHW827 = 60000 / beatIntervalHW827;
    if (bpmHW827 >= 80 && bpmHW827 <= 180) {
      bpmReadings[bpmIndex] = bpmHW827;
      bpmIndex = (bpmIndex + 1) % BPM_SAMPLES;

      int sum = 0, validCount = 0;
      for (int i = 0; i < BPM_SAMPLES; i++) {
        if (bpmReadings[i] >= 80 && bpmReadings[i] <= 180) {
          sum += bpmReadings[i];
          validCount++;
        }
      }
      bpmAvgHW827 = validCount > 0 ? sum / validCount : 0;
      beepHeartbeat();
    }
  }
  lastBeatTimeHW827 = currentTime;
}

/* ===== BLE CALLBACKS ===== */
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("🔵 BLE connecté");
    beepBleConnect();
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("🔴 BLE déconnecté");
    tone(BUZZER_PIN, 2000, 200);
    BLEDevice::startAdvertising();
  }
};

/* ===== SPO2 ===== */
void updateSpO2() {
  redBuffer[bufferIndex] = particleSensor.getRed();
  irBuffer[bufferIndex] = particleSensor.getIR();
  
  fingerDetected = (irBuffer[bufferIndex] > 5000);
  
  if (fingerDetected) {
    if (checkForBeat(irBuffer[bufferIndex])) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute >= 80 && beatsPerMinute <= 180) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;

        beatAvgMAX = 0;
        byte validCount = 0;
        for (byte x = 0; x < RATE_SIZE; x++) {
          if (rates[x] >= 80 && rates[x] <= 180) {
            beatAvgMAX += rates[x];
            validCount++;
          }
        }
        beatAvgMAX = validCount > 0 ? beatAvgMAX / validCount : 0;
      }
    }
    
    if (bufferFull) {
      long irMin = irBuffer[0], irMax = irBuffer[0];
      long redMin = redBuffer[0], redMax = redBuffer[0];
      
      for (int i = 1; i < BUFFER_SIZE; i++) {
        if (irBuffer[i] < irMin) irMin = irBuffer[i];
        if (irBuffer[i] > irMax) irMax = irBuffer[i];
        if (redBuffer[i] < redMin) redMin = redBuffer[i];
        if (redBuffer[i] > redMax) redMax = redBuffer[i];
      }
      
      long irAC = irMax - irMin, irDC = (irMax + irMin) / 2;
      long redAC = redMax - redMin, redDC = (redMax + redMin) / 2;
      
      if (irDC > 0 && redDC > 0 && irAC > 0) {
        float ratio = (redAC * irDC) / (float)(redDC * irAC);
        spo2Value = constrain(110 - (ratio * 25), 80, 100);
      }
    } else {
      spo2Value = constrain(map(irBuffer[bufferIndex], 10000, 80000, 85, 100), 85, 100);
    }
  } else {
    fingerDetected = false;
    spo2Value = 0;
    beatAvgMAX = 0;
  }
  
  bufferIndex++;
  if (bufferIndex >= BUFFER_SIZE) {
    bufferIndex = 0;
    bufferFull = true;
  }
}

/* ===== KY-013 ===== */
float readKY013Temperature() {
  int analogValue = analogRead(KY013_PIN);
  float R2 = 10000.0 * analogValue / (4095.0 - analogValue);
  if (R2 <= 0) return NAN;
  
  float logR2 = log(R2);
  float tempK = 1.0 / (0.001129148 + 0.000234125 * logR2 + 0.0000000876741 * logR2 * logR2 * logR2);
  return tempK - 273.15;
}

/* ===== ALARMES ===== */
void checkAlarms() {
  currentAlarm = NONE;
  
  int currentBPM = (fingerDetected && beatAvgMAX > 0) ? beatAvgMAX : bpmAvgHW827;
  
  // Priorité aux alarmes critiques
  if (spo2Value > 0 && spo2Value < 90) {
    currentAlarm = SPO2_LOW; // CRITIQUE
  } else if (currentBPM > 0) {
    if (currentBPM < 100) currentAlarm = BPM_LOW;
    else if (currentBPM > 160) currentAlarm = BPM_HIGH;
  } else if (!isnan(globalTempKY013)) {
    if (globalTempKY013 < 36.0) currentAlarm = TEMP_LOW;
    else if (globalTempKY013 > 37.5) currentAlarm = TEMP_HIGH;
  }
  
  handleAlarm();
}

/* ===== SETUP ===== */
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(HW827_PIN, INPUT_PULLUP);
  pinMode(KY013_PIN, INPUT);

  dht.begin();
  lcd.init();
  lcd.backlight();
  lcd.print("Init...");

  Wire.begin(21, 22);
  if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x1F);
    particleSensor.setPulseAmplitudeIR(0x1F);
    Serial.println("✅ MAX30102 OK");
  }

  attachInterrupt(digitalPinToInterrupt(HW827_PIN), heartBeatDetectedHW827, RISING);
  for (int i = 0; i < BPM_SAMPLES; i++) bpmReadings[i] = 0;

  BLEDevice::init(DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();

  lcd.clear();
  lcd.print("Ready");
  beepBleConnect();
  Serial.println("✅ Système prêt");
}

/* ===== LOOP ===== */
void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - spo2UpdateMillis >= 50) {
    spo2UpdateMillis = currentMillis;
    updateSpO2();
  }

  if (currentMillis - previousMillis >= 2000) {
    previousMillis = currentMillis;
    globalHumidity = dht.readHumidity();
    globalTempDHT = dht.readTemperature();
    globalTempKY013 = readKY013Temperature();
    
    if (millis() - lastBeatTimeHW827 > 5000) bpmAvgHW827 = 0;
    
    checkAlarms();
    
    Serial.printf("❤️ BPM:%d SpO2:%d%% 🌡️ Corps:%.1f°C Inc:%.1f°C Hum:%.0f%%\n", 
                  (fingerDetected ? beatAvgMAX : bpmAvgHW827), spo2Value, 
                  globalTempKY013, globalTempDHT, globalHumidity);
  }

  if (currentMillis - lcdUpdateMillis >= 500) {
    lcdUpdateMillis = currentMillis;
    lcd.clear();
    displayMode = (millis() / 3000) % 5;

    switch (displayMode) {
      case 0:
        lcd.print("Incubateur");
        lcd.setCursor(0, 1);
        lcd.printf("T:%.1fC H:%.0f%%", globalTempDHT, globalHumidity);
        break;
      case 1:
        lcd.print("HR Capteur");
        lcd.setCursor(0, 1);
        lcd.printf("BPM:%d%s", bpmAvgHW827, (currentAlarm == BPM_LOW || currentAlarm == BPM_HIGH) ? " ALM" : "");
        break;
      case 2:
        lcd.print("SpO2 Monitor");
        lcd.setCursor(0, 1);
        if (fingerDetected) lcd.printf("BPM:%d O2:%d%%%s", beatAvgMAX, spo2Value, currentAlarm == SPO2_LOW ? "!" : "");
        else lcd.print("Placez doigt");
        break;
      case 3:
        lcd.print("Temp Corps");
        lcd.setCursor(0, 1);
        lcd.printf("%.1fC%s", globalTempKY013, (currentAlarm == TEMP_LOW || currentAlarm == TEMP_HIGH) ? " ALM" : "");
        break;
      case 4:
        lcd.print("Status BLE");
        lcd.setCursor(0, 1);
        lcd.printf("%s %s", deviceConnected ? "Conn" : "Wait", currentAlarm != NONE ? "ALARM" : "OK");
        break;
    }
  }

  if (deviceConnected && millis() - previousBleMillis >= 1000) {
    previousBleMillis = millis();
    StaticJsonDocument<256> doc;
    
    int hr = (fingerDetected && beatAvgMAX > 0) ? beatAvgMAX : bpmAvgHW827;
    doc["heartRate"] = hr;
    doc["spo2"] = spo2Value;
    doc["fingerDetected"] = fingerDetected;
    doc["bodyTemp"] = !isnan(globalTempKY013) ? globalTempKY013 : 0.0;
    doc["incubatorTemp"] = !isnan(globalTempDHT) ? globalTempDHT : 0.0;
    doc["humidity"] = !isnan(globalHumidity) ? globalHumidity : 0.0;
    doc["alarmType"] = currentAlarm;

    char buffer[256];
    serializeJson(doc, buffer);
    pTxCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
    pTxCharacteristic->notify();
  }

  delay(10);
}