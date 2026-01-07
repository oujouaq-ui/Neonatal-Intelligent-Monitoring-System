# Neonatal-Intelligent-Monitoring-System
# NIMS – ESP32 Firmware

This repository contains the ESP32 firmware used in the
Neonatal Intelligent Monitoring System (NIMS).

## Description
The ESP32 acts as a BLE peripheral and simulates or reads neonatal vital signs:
- Incubator Temperature
- Body Temperature
- Heart Rate
- SpO2

The data is transmitted via Bluetooth Low Energy (BLE)
to a desktop C# application.

## BLE Configuration
- Device Name: NeonatalMonitor_ESP32
- Service UUID: 6e400001-b5a3-f393-e0a9-e50e24dcca9e
- Notify Characteristic UUID: 6e400003-b5a3-f393-e0a9-e50e24dcca9e

## Data Format
```json
{
  "incubatorTemp": 36.5,
  "bodyTemp": 37.1,
  "heartRate": 140,
  "spo2": 96
}
